#include <vector>
#include <memory>
#include <chrono>
#include "pose_graph_node.h"


#define SKIP_FIRST_CNT 10
using namespace std;
using namespace std::chrono_literals;
using std::placeholders::_1;

PoseGraphNode::PoseGraphNode()
        : Node("pose_graph"), cameraposevisual(1,0,0,1),last_t(-100, -100, -100)
{
    readParameters();

    posegraph = new PoseGraph(VISUALIZATION_SHIFT_X,VISUALIZATION_SHIFT_Y,VINS_RESULT_PATH,DEBUG_IMAGE,
                              POSE_GRAPH_SAVE_PATH,FAST_RELOCALIZATION,BRIEF_PATTERN_FILE,m_camera,ROW,COL,
                              tic,qic,pub_match_img,pub_match_points,this->get_logger());

    posegraph->registerPub(*this);

    std::string vocabulary_file = vins_path_ + "/support_files/brief_k10L6.bin";
    cout << "vocabulary_file" << vocabulary_file << endl;
    posegraph->loadVocabulary(vocabulary_file);

    if (LOAD_PREVIOUS_POSE_GRAPH)
    {
        printf("load pose graph\n");
        m_process.lock();
        posegraph->loadPoseGraph();
        m_process.unlock();
        printf("load pose graph finish\n");
        load_flag = 1;
    }
    else
    {
        printf("no previous pose graph\n");
        load_flag = 1;
    }

    rclcpp::QoS qos(100);
    auto rmw_qos_profile = qos.get_rmw_qos_profile();

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    if (LOOP_CLOSURE){
        // create subscribers to the topics of interest
        image_sub_.subscribe(this, "/pose_graph_raspberry/image", rmw_qos_profile);
        pointcloud_sub_.subscribe(this, "/pose_graph_raspberry/pointcloud", rmw_qos_profile);
        pose_sub_.subscribe(this, "/pose_graph_raspberry/pose", rmw_qos_profile);

        temp_sync_ = std::make_shared<message_filters::TimeSynchronizer<sensor_msgs::msg::Image, sensor_msgs::msg::PointCloud,nav_msgs::msg::Odometry>>(image_sub_, pointcloud_sub_,pose_sub_, 100);
        temp_sync_->registerCallback(std::bind(&PoseGraphNode::process, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
    }




    // Subscribers
    sub_imu_forward = this->create_subscription<nav_msgs::msg::Odometry>("/vins_estimator/imu_propagate", 2000, std::bind(&PoseGraphNode::imu_forward_callback, this, _1));                    // visuallize
    sub_vio = this->create_subscription<nav_msgs::msg::Odometry>("/vins_estimator/odometry", 2000, std::bind(&PoseGraphNode::vio_callback, this, _1));                                         // visualize
//    sub_image = this->create_subscription<sensor_msgs::msg::Image>("/cam0/image_raw", 2000, std::bind(&PoseGraphNode::image_callback, this, _1));                                    // important
//    sub_pose = this->create_subscription<nav_msgs::msg::Odometry>("/vins_estimator/keyframe_pose", 2000, std::bind(&PoseGraphNode::pose_callback, this, _1));                                  // important
    sub_extrinsic = this->create_subscription<nav_msgs::msg::Odometry>("/vins_estimator/extrinsic", 2000, std::bind(&PoseGraphNode::extrinsic_callback, this, _1));                            // important
//    sub_point = this->create_subscription<sensor_msgs::msg::PointCloud>("/vins_estimator/keyframe_point", 2000, std::bind(&PoseGraphNode::point_callback, this, _1));                          // important
    sub_relo_relative_pose = this->create_subscription<nav_msgs::msg::Odometry>("/vins_estimator/relo_relative_pose", 2000, std::bind(&PoseGraphNode::relo_relative_pose_callback, this, _1)); // important

    // Publisher
    pub_key_odometrys = this->create_publisher<visualization_msgs::msg::Marker>("pose_graph/key_odometrys", 1000);
    pub_camera_pose_visual = this->create_publisher<visualization_msgs::msg::MarkerArray>("pose_graph/camera_pose_visual", 1000);
    pub_match_img = this->create_publisher<sensor_msgs::msg::Image>("pose_graph/match_image", 1000);
    pub_vio_path = this->create_publisher<nav_msgs::msg::Path>("pose_graph/no_loop_path", 1000);
    pub_match_points = this->create_publisher<sensor_msgs::msg::PointCloud>("pose_graph/match_points", 100);
    pub_odom = this->create_publisher<nav_msgs::msg::Odometry>("pose_graph/odometry", 100);

//    measurement_process = std::thread(&PoseGraphNode::process, this);
    keyboard_command_process = std::thread(&PoseGraphNode::command, this);

}

PoseGraphNode::~PoseGraphNode()
{
    keyboard_command_process.join();
//    measurement_process.join();
}

void PoseGraphNode::readParameters()
{

    this->declare_parameter<std::string>("config_file", "");
    this->declare_parameter<std::string>("vins_path", "");
//    this->declare_parameter<std::string>("visualization_shift_x","0");
//    this->declare_parameter<std::string>("visualization_shift_y","0");
//    this->declare_parameter<std::string>("skip_cnt","0");
//    this->declare_parameter<std::string>("skip_dis","0");
    RCLCPP_INFO(this->get_logger(), "2");
    this->get_parameter("config_file", camera_config_file_);
    this->get_parameter("vins_path", vins_path_);
//    this->get_parameter("visualization_shift_x", VISUALIZATION_SHIFT_X);
//    this->get_parameter("visualization_shift_y", VISUALIZATION_SHIFT_Y);
//    this->get_parameter("skip_cnt", SKIP_CNT);
//    this->get_parameter("skip_dis", SKIP_DIS);
     VISUALIZATION_SHIFT_X = 0;
     VISUALIZATION_SHIFT_Y = 0;
     SKIP_CNT = 0;
     SKIP_DIS = 0;

    RCLCPP_INFO(this->get_logger(), "CONFIG FILE: %s", camera_config_file_.c_str());

    cv::FileStorage fsSettings(camera_config_file_, cv::FileStorage::READ);
    if(!fsSettings.isOpened())
    {
        std::cerr << "ERROR: Wrong path to settings" << std::endl;
        assert(!fsSettings.isOpened());
    }

    double camera_visual_size = fsSettings["visualize_camera_size"];
    cameraposevisual.setScale(camera_visual_size);
    cameraposevisual.setLineWidth(camera_visual_size / 10.0);

    LOOP_CLOSURE = fsSettings["loop_closure"];
    std::string IMAGE_TOPIC;
    if (LOOP_CLOSURE)
    {
        DOWN_SCALE = fsSettings["down_scale_pose"];
        DOWN_SCALE_RASPBERRY = fsSettings["down_scale_raspberry"];

        ROW = fsSettings["image_height"];
        COL = fsSettings["image_width"];

        ROW = ROW / DOWN_SCALE;
        COL = COL / DOWN_SCALE;

        BRIEF_PATTERN_FILE = vins_path_ + "/support_files/brief_pattern.yml";
        cout << "BRIEF_PATTERN_FILE" << BRIEF_PATTERN_FILE << endl;
        m_camera = camodocal::CameraFactory::instance()->generateCameraFromYamlFile(camera_config_file_,DOWN_SCALE);

        fsSettings["image_topic"] >> IMAGE_TOPIC;
        fsSettings["pose_graph_save_path"] >> POSE_GRAPH_SAVE_PATH;
        fsSettings["output_path"] >> VINS_RESULT_PATH;
        fsSettings["save_image"] >> DEBUG_IMAGE;

        // create folder if not exists
        FileSystemHelper::createDirectoryIfNotExists(POSE_GRAPH_SAVE_PATH.c_str());
        FileSystemHelper::createDirectoryIfNotExists(VINS_RESULT_PATH.c_str());


        VISUALIZE_IMU_FORWARD = fsSettings["visualize_imu_forward"];
        LOAD_PREVIOUS_POSE_GRAPH = fsSettings["load_previous_pose_graph"];
        FAST_RELOCALIZATION = fsSettings["fast_relocalization"];
        VINS_RESULT_PATH = VINS_RESULT_PATH + "/vins_result_loop.csv";
        std::ofstream fout(VINS_RESULT_PATH, std::ios::out);
        fout.close();
    }
    fsSettings.release();
}

void PoseGraphNode::new_sequence()
{
    printf("new sequence\n");
    sequence++;
    printf("sequence cnt %d \n", sequence);
    if (sequence > 5)
    {
        RCLCPP_WARN(this->get_logger(), "only support 5 sequences since it's boring to copy code for more sequences.");
        rclcpp::shutdown();
    }
    posegraph->posegraph_visualization->reset();
    posegraph->publish();
    m_buf.lock();
    while (!image_buf.empty())
        image_buf.pop();
    while (!point_buf.empty())
        point_buf.pop();
    while (!pose_buf.empty())
        pose_buf.pop();
    while (!odometry_buf.empty())
        odometry_buf.pop();
    m_buf.unlock();
}


void PoseGraphNode::imu_forward_callback(const nav_msgs::msg::Odometry::ConstSharedPtr forward_msg)
{
    if (VISUALIZE_IMU_FORWARD)
    {
        Vector3d vio_t(forward_msg->pose.pose.position.x, forward_msg->pose.pose.position.y, forward_msg->pose.pose.position.z);
        Quaterniond vio_q;
        vio_q.w() = forward_msg->pose.pose.orientation.w;
        vio_q.x() = forward_msg->pose.pose.orientation.x;
        vio_q.y() = forward_msg->pose.pose.orientation.y;
        vio_q.z() = forward_msg->pose.pose.orientation.z;

        vio_t = posegraph->w_r_vio * vio_t + posegraph->w_t_vio;
        vio_q = posegraph->w_r_vio * vio_q;

        vio_t = posegraph->r_drift * vio_t + posegraph->t_drift;
        vio_q = posegraph->r_drift * vio_q;

        Vector3d vio_t_cam;
        Quaterniond vio_q_cam;
        vio_t_cam = vio_t + vio_q * tic;
        vio_q_cam = vio_q * qic;

        cameraposevisual.reset();
        cameraposevisual.add_pose(vio_t_cam, vio_q_cam);
        cameraposevisual.publish_by(pub_camera_pose_visual, forward_msg->header);
        UpdateOdometryPose(vio_t_cam, vio_q_cam,forward_msg->header);
    }
}
void PoseGraphNode::relo_relative_pose_callback(const nav_msgs::msg::Odometry::ConstSharedPtr pose_msg)
{
    Vector3d relative_t = Vector3d(pose_msg->pose.pose.position.x,
                                   pose_msg->pose.pose.position.y,
                                   pose_msg->pose.pose.position.z);
    Quaterniond relative_q;
    relative_q.w() = pose_msg->pose.pose.orientation.w;
    relative_q.x() = pose_msg->pose.pose.orientation.x;
    relative_q.y() = pose_msg->pose.pose.orientation.y;
    relative_q.z() = pose_msg->pose.pose.orientation.z;
    double relative_yaw = pose_msg->twist.twist.linear.x;
    int index = pose_msg->twist.twist.linear.y;
    // printf("receive index %d \n", index );
    Eigen::Matrix<double, 8, 1> loop_info;
    loop_info << relative_t.x(), relative_t.y(), relative_t.z(),
            relative_q.w(), relative_q.x(), relative_q.y(), relative_q.z(),
            relative_yaw;
    posegraph->updateKeyFrameLoop(index, loop_info);
}

void PoseGraphNode::vio_callback(const nav_msgs::msg::Odometry::ConstSharedPtr pose_msg)
{
    // RCLCPP_INFO(this->get_logger(),"vio_callback!");
    Vector3d vio_t(pose_msg->pose.pose.position.x, pose_msg->pose.pose.position.y, pose_msg->pose.pose.position.z);
    Quaterniond vio_q;
    vio_q.w() = pose_msg->pose.pose.orientation.w;
    vio_q.x() = pose_msg->pose.pose.orientation.x;
    vio_q.y() = pose_msg->pose.pose.orientation.y;
    vio_q.z() = pose_msg->pose.pose.orientation.z;

    vio_t = posegraph->w_r_vio * vio_t + posegraph->w_t_vio;
    vio_q = posegraph->w_r_vio * vio_q;

    vio_t = posegraph->r_drift * vio_t + posegraph->t_drift;
    vio_q = posegraph->r_drift * vio_q;

    Vector3d vio_t_cam;
    Quaterniond vio_q_cam;
    vio_t_cam = vio_t + vio_q * tic;
    vio_q_cam = vio_q * qic;

    if (!VISUALIZE_IMU_FORWARD)
    {
        cameraposevisual.reset();
        cameraposevisual.add_pose(vio_t_cam, vio_q_cam);
        cameraposevisual.publish_by(pub_camera_pose_visual, pose_msg->header);
        UpdateOdometryPose(vio_t_cam, vio_q_cam,pose_msg->header);
    }

    odometry_buf.push(vio_t_cam);
    if (odometry_buf.size() > 10)
    {
        odometry_buf.pop();
    }

    visualization_msgs::msg::Marker key_odometrys;
    key_odometrys.header = pose_msg->header;
    key_odometrys.header.frame_id = "world";
    key_odometrys.ns = "key_odometrys";
    key_odometrys.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    key_odometrys.action = visualization_msgs::msg::Marker::ADD;
    key_odometrys.pose.orientation.w = 1.0;
    // key_odometrys.lifetime = this->get_clock()

    // static int key_odometrys_id = 0;
    key_odometrys.id = 0; // key_odometrys_id++;
    key_odometrys.scale.x = 0.1;
    key_odometrys.scale.y = 0.1;
    key_odometrys.scale.z = 0.1;
    key_odometrys.color.r = 1.0;
    key_odometrys.color.a = 1.0;

    for (unsigned int i = 0; i < odometry_buf.size(); i++)
    {
        geometry_msgs::msg::Point pose_marker;
        Vector3d vio_t;
        vio_t = odometry_buf.front();
        odometry_buf.pop();
        pose_marker.x = vio_t.x();
        pose_marker.y = vio_t.y();
        pose_marker.z = vio_t.z();
        key_odometrys.points.push_back(pose_marker);
        odometry_buf.push(vio_t);
    }
    pub_key_odometrys->publish(key_odometrys);

    if (!LOOP_CLOSURE)
    {
        geometry_msgs::msg::PoseStamped pose_stamped;
        pose_stamped.header = pose_msg->header;
        pose_stamped.header.frame_id = "world";
        pose_stamped.pose.position.x = vio_t.x();
        pose_stamped.pose.position.y = vio_t.y();
        pose_stamped.pose.position.z = vio_t.z();
        no_loop_path.header = pose_msg->header;
        no_loop_path.header.frame_id = "world";
        no_loop_path.poses.push_back(pose_stamped);
        pub_vio_path->publish(no_loop_path);
    }
}

void PoseGraphNode::extrinsic_callback(const nav_msgs::msg::Odometry::ConstSharedPtr pose_msg)
{
    m_process.lock();
    tic = Vector3d(pose_msg->pose.pose.position.x,
                   pose_msg->pose.pose.position.y,
                   pose_msg->pose.pose.position.z);
    qic = Quaterniond(pose_msg->pose.pose.orientation.w,
                      pose_msg->pose.pose.orientation.x,
                      pose_msg->pose.pose.orientation.y,
                      pose_msg->pose.pose.orientation.z)
            .toRotationMatrix();
    m_process.unlock();
}


void PoseGraphNode::UpdateOdometryPose(const Eigen::Vector3d& p, const Eigen::Quaterniond& q,const std_msgs::msg::Header &header){
    nav_msgs::msg::Odometry odometry;
    odometry.header = header;
    odometry.header.frame_id = "world";
    odometry.pose.pose.position.x = p.x();
    odometry.pose.pose.position.y = p.y();
    odometry.pose.pose.position.z = p.z();
    odometry.pose.pose.orientation.x = q.x();
    odometry.pose.pose.orientation.y = q.y();
    odometry.pose.pose.orientation.z = q.z();
    odometry.pose.pose.orientation.w = q.w();
    pub_odom->publish(odometry);




    tf2::Transform tmp_tf;
    tmp_tf.setOrigin(tf2::Vector3(p.x(),
                                  p.y(), p.z()));


   /* tf2::Stamped<tf2::Transform> base_to_map(
            tf2::Transform(quaternion, tf2::Vector3(p.x(),
                                                    p.y(), p.z())), tf2_ros::fromMsg(header.stamp), "camera");
    tf2::Transform transform1 = base_to_map.inverse();*/
    try{
        tf2::Quaternion quaternion;
        quaternion.setW(q.w());
        quaternion.setX(q.x());
        quaternion.setY(q.y());
        quaternion.setZ(q.z());
        tmp_tf.setRotation(quaternion);

        tf2::Stamped<tf2::Transform> map_to_base(tmp_tf.inverse(),tf2_ros::fromMsg(header.stamp), "camera");


        geometry_msgs::msg::TransformStamped map_to_base_msg, latest_tf_msg;

        map_to_base_msg.header.stamp = tf2_ros::toMsg(map_to_base.stamp_);
        map_to_base_msg.header.frame_id = map_to_base.frame_id_;
        map_to_base_msg.transform.translation.x = map_to_base.getOrigin().getX();
        map_to_base_msg.transform.translation.y = map_to_base.getOrigin().getY();
        map_to_base_msg.transform.translation.z = map_to_base.getOrigin().getZ();
        map_to_base_msg.transform.rotation = tf2::toMsg(map_to_base.getRotation());


        /*geometry_msgs::msg::TransformStamped base_to_map_msg, odom_to_map_msg;

        base_to_map_msg.header.stamp = tf2_ros::toMsg(base_to_map.stamp_);
        base_to_map_msg.header.frame_id = base_to_map.frame_id_;
        base_to_map_msg.transform.translation.x = transform1.getOrigin().getX();
        base_to_map_msg.transform.translation.y = transform1.getOrigin().getY();
        base_to_map_msg.transform.translation.z = transform1.getOrigin().getZ();
        base_to_map_msg.transform.rotation = tf2::toMsg(transform1.getRotation());*/

        //tf_buffer_->waitForTransform("camera","odom",tf2_ros::fromMsg(header.stamp),tf2::durationFromSec(10.0),);
        tf2::Stamped<tf2::Transform> latest_tf_;

        tf_buffer_->lookupTransform("camera","odom",tf2_ros::fromMsg(header.stamp),10s);
        latest_tf_msg = tf_buffer_->transform(map_to_base_msg, "odom");
        tf2::fromMsg(latest_tf_msg, latest_tf_);

        tf2::Transform map_to_odom_;
        map_to_odom_ = tf2::Transform(tf2::Quaternion(latest_tf_.getRotation()),
                                      tf2::Vector3(latest_tf_.getOrigin())).inverse();

        geometry_msgs::msg::TransformStamped msg;
        msg.transform = tf2::toMsg(map_to_odom_);
        msg.child_frame_id = "odom";
        msg.header.frame_id = "world";
        msg.header.stamp = header.stamp;

        tf_broadcaster_->sendTransform(msg);

    }
    catch(tf2::TransformException &e)
    {
        RCLCPP_INFO(this->get_logger(),"Error occurred: %s ", e.what());
        return;
    }



}





// void process(const sensor_msgs::msg::Image::ConstSharedPtr &image_msg, const sensor_msgs::msg::PointCloud::ConstSharedPtr &point_msg, const nav_msgs::msg::Odometry::ConstSharedPtr &pose_msg)
void PoseGraphNode::process(sensor_msgs::msg::Image::ConstSharedPtr image_msg,sensor_msgs::msg::PointCloud::ConstSharedPtr point_msg, nav_msgs::msg::Odometry::ConstSharedPtr pose_msg)
{
    if (skip_first_cnt < SKIP_FIRST_CNT)
    {
        skip_first_cnt++;
        return;
    }

    if (skip_cnt < SKIP_CNT)
    {
        skip_cnt++;
        return;
    }
    else
    {
        skip_cnt = 0;
    }

    cv_bridge::CvImageConstPtr ptr;

    if (image_msg->encoding == "8UC1")
    {
        sensor_msgs::msg::Image img;
        img.header = image_msg->header;
        img.height = image_msg->height;
        img.width = image_msg->width;
        img.is_bigendian = image_msg->is_bigendian;
        img.step = image_msg->step;
        img.data = image_msg->data;
        img.encoding = "mono8";
        ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::MONO8);
    }
    else
        ptr = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::MONO8);

    cv::Mat image = ptr->image;
    if(DOWN_SCALE > 1){
        resize(image, image, cv::Size(COL, ROW), cv::INTER_LINEAR);
    }
    // build keyframe
    Vector3d T = Vector3d(pose_msg->pose.pose.position.x,
                          pose_msg->pose.pose.position.y,
                          pose_msg->pose.pose.position.z);
    Matrix3d R = Quaterniond(pose_msg->pose.pose.orientation.w,
                             pose_msg->pose.pose.orientation.x,
                             pose_msg->pose.pose.orientation.y,
                             pose_msg->pose.pose.orientation.z)
            .toRotationMatrix();
    if ((T - last_t).norm() > SKIP_DIS)
    {
        vector<cv::Point3f> point_3d;
        vector<cv::Point2f> point_2d_uv;
        vector<cv::Point2f> point_2d_normal;
        vector<double> point_id;

        for (unsigned int i = 0; i < point_msg->points.size(); i++)
        {
            cv::Point3f p_3d;
            p_3d.x = point_msg->points[i].x;
            p_3d.y = point_msg->points[i].y;
            p_3d.z = point_msg->points[i].z;
            point_3d.push_back(p_3d);

            cv::Point2f p_2d_uv, p_2d_normal;
            double p_id;
            p_2d_normal.x = point_msg->channels[i].values[0] * DOWN_SCALE_RASPBERRY;
            p_2d_normal.y = point_msg->channels[i].values[1] * DOWN_SCALE_RASPBERRY;
            p_2d_uv.x = point_msg->channels[i].values[2] * DOWN_SCALE_RASPBERRY;
            p_2d_uv.y = point_msg->channels[i].values[3] * DOWN_SCALE_RASPBERRY;
            p_id = point_msg->channels[i].values[4];
            point_2d_normal.push_back(p_2d_normal);
            point_2d_uv.push_back(p_2d_uv);
            point_id.push_back(p_id);

            // printf("u %f, v %f \n", p_2d_uv.x, p_2d_uv.y);
        }

        KeyFrame *keyframe = new KeyFrame(rclcpp::Time(pose_msg->header.stamp.sec, pose_msg->header.stamp.nanosec).seconds(), frame_index, T, R, image,
                                          point_3d, point_2d_uv, point_2d_normal, point_id, sequence,DEBUG_IMAGE,BRIEF_PATTERN_FILE,m_camera,
                                          ROW,COL,tic,qic,FAST_RELOCALIZATION,pub_match_img,pub_match_points,this->get_logger());

        m_process.lock();
        start_flag = true;
        posegraph->addKeyFrame(keyframe, true);
        m_process.unlock();
        frame_index++;
        last_t = T;
    }
}
void PoseGraphNode::command()
{
    if (!LOOP_CLOSURE)
        return;
    while (1)
    {
        char c = getchar();
        if (c == 's')
        {
            m_process.lock();
            posegraph->savePoseGraph();
            m_process.unlock();
            printf("save pose graph finish\nyou can set 'load_previous_pose_graph' to 1 in the config file to reuse it next time\n");
            // printf("program shutting down...\n");
            // ros::shutdown();
        }
        if (c == 'n')
            new_sequence();

        std::chrono::milliseconds dura(5);
        std::this_thread::sleep_for(dura);
    }
}




int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    rclcpp::spin(std::make_shared<PoseGraphNode>());
    rclcpp::shutdown();
    return 0;

}
