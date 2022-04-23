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

    // Subscribers
    sub_imu_forward = this->create_subscription<nav_msgs::msg::Odometry>("/vins_estimator/imu_propagate", 2000, std::bind(&PoseGraphNode::imu_forward_callback, this, _1));                    // visuallize
    sub_vio = this->create_subscription<nav_msgs::msg::Odometry>("/vins_estimator/odometry", 2000, std::bind(&PoseGraphNode::vio_callback, this, _1));                                         // visualize
    sub_image = this->create_subscription<sensor_msgs::msg::Image>("/cam0/image_raw", 2000, std::bind(&PoseGraphNode::image_callback, this, _1));                                    // important
    sub_pose = this->create_subscription<nav_msgs::msg::Odometry>("/vins_estimator/keyframe_pose", 2000, std::bind(&PoseGraphNode::pose_callback, this, _1));                                  // important
    sub_extrinsic = this->create_subscription<nav_msgs::msg::Odometry>("/vins_estimator/extrinsic", 2000, std::bind(&PoseGraphNode::extrinsic_callback, this, _1));                            // important
    sub_point = this->create_subscription<sensor_msgs::msg::PointCloud>("/vins_estimator/keyframe_point", 2000, std::bind(&PoseGraphNode::point_callback, this, _1));                          // important
    sub_relo_relative_pose = this->create_subscription<nav_msgs::msg::Odometry>("/vins_estimator/relo_relative_pose", 2000, std::bind(&PoseGraphNode::relo_relative_pose_callback, this, _1)); // important

    // Publisher
    pub_key_odometrys = this->create_publisher<visualization_msgs::msg::Marker>("key_odometrys", 1000);
    pub_camera_pose_visual = this->create_publisher<visualization_msgs::msg::MarkerArray>("camera_pose_visual", 1000);
    pub_match_img = this->create_publisher<sensor_msgs::msg::Image>("match_image", 1000);
    pub_vio_path = this->create_publisher<nav_msgs::msg::Path>("no_loop_path", 1000);
    pub_match_points = this->create_publisher<sensor_msgs::msg::PointCloud>("match_points", 100);

    measurement_process = std::thread(&PoseGraphNode::process, this);
    keyboard_command_process = std::thread(&PoseGraphNode::command, this);

}

PoseGraphNode::~PoseGraphNode()
{
    keyboard_command_process.join();
    measurement_process.join();
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
        //DOWN_SCALE = fsSettings["down_scale_pose"];
        //DOWN_SCALE_RASPBERRY = fsSettings["down_scale_raspberry"];

        ROW = fsSettings["image_height"];
        COL = fsSettings["image_width"];

        //ROW = ROW / DOWN_SCALE;
        //COL = COL / DOWN_SCALE;

        BRIEF_PATTERN_FILE = vins_path_ + "/support_files/brief_pattern.yml";
        cout << "BRIEF_PATTERN_FILE" << BRIEF_PATTERN_FILE << endl;
        m_camera = camodocal::CameraFactory::instance()->generateCameraFromYamlFile(camera_config_file_.c_str(),1);

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

void PoseGraphNode::image_callback(const sensor_msgs::msg::Image::ConstSharedPtr image_msg)
{
    // RCLCPP_INFO("image_callback!");
    if (!LOOP_CLOSURE)
        return;
    m_buf.lock();
    image_buf.push(image_msg);
    m_buf.unlock();
    // printf(" image time %f \n", image_msg->header.stamp.toSec());

    // detect unstable camera stream
    if (last_image_time == -1)
        last_image_time = rclcpp::Time(image_msg->header.stamp.sec, image_msg->header.stamp.nanosec).seconds();
    else if (rclcpp::Time(image_msg->header.stamp.sec, image_msg->header.stamp.nanosec).seconds() - last_image_time > 1.0 || rclcpp::Time(image_msg->header.stamp.sec, image_msg->header.stamp.nanosec).seconds() < last_image_time)
    {
        RCLCPP_WARN(this->get_logger(), "image discontinue! detect a new sequence!");
        new_sequence();
    }
    last_image_time = rclcpp::Time(image_msg->header.stamp.sec, image_msg->header.stamp.nanosec).seconds();
}

void PoseGraphNode::point_callback(const sensor_msgs::msg::PointCloud::ConstSharedPtr point_msg)
{
    // RCLCPP_INFO(this->get_logger(),"point_callback!");
    if (!LOOP_CLOSURE)
        return;
    m_buf.lock();
    point_buf.push(point_msg);
    m_buf.unlock();
    /*
    for (unsigned int i = 0; i < point_msg->points.size(); i++)
    {
        printf("%d, 3D point: %f, %f, %f 2D point %f, %f \n",i , point_msg->points[i].x,
                                                    point_msg->points[i].y,
                                                    point_msg->points[i].z,
                                                    point_msg->channels[i].values[0],
                                                    point_msg->channels[i].values[1]);
    }
    */
}
void PoseGraphNode::pose_callback(const nav_msgs::msg::Odometry::ConstSharedPtr pose_msg)
{
    // RCLCPP_INFO(this->get_logger(),"pose_callback!");
    if (!LOOP_CLOSURE)
        return;
    m_buf.lock();
    pose_buf.push(pose_msg);
    m_buf.unlock();
    /*
    printf("pose t: %f, %f, %f   q: %f, %f, %f %f \n", pose_msg->pose.pose.position.x,
                                                    pose_msg->pose.pose.position.y,
                                                    pose_msg->pose.pose.position.z,
                                                    pose_msg->pose.pose.orientation.w,
                                                    pose_msg->pose.pose.orientation.x,
                                                    pose_msg->pose.pose.orientation.y,
                                                    pose_msg->pose.pose.orientation.z);
    */
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

// void process(const sensor_msgs::msg::Image::ConstSharedPtr &image_msg, const sensor_msgs::msg::PointCloud::ConstSharedPtr &point_msg, const nav_msgs::msg::Odometry::ConstSharedPtr &pose_msg)
void PoseGraphNode::process()
{

    if (!LOOP_CLOSURE){
        return;
    }
    while (true)
    {
        sensor_msgs::msg::Image::ConstPtr image_msg = NULL;
        sensor_msgs::msg::PointCloud::ConstPtr point_msg = NULL;
        nav_msgs::msg::Odometry::ConstPtr pose_msg = NULL;
        // find out the messages with same time stamp
        m_buf.lock();
        if (!image_buf.empty() && !point_buf.empty() && !pose_buf.empty())
        {
            if (rclcpp::Time(image_buf.front()->header.stamp.sec, image_buf.front()->header.stamp.nanosec).seconds() > rclcpp::Time(pose_buf.front()->header.stamp.sec, pose_buf.front()->header.stamp.nanosec).seconds())
            {
                pose_buf.pop();
                printf("throw pose at beginning\n");
            }
            else if (rclcpp::Time(image_buf.front()->header.stamp.sec, image_buf.front()->header.stamp.nanosec).seconds() > rclcpp::Time(point_buf.front()->header.stamp.sec, point_buf.front()->header.stamp.nanosec).seconds())
            {
                point_buf.pop();
                printf("throw point at beginning\n");
            }
            else if (rclcpp::Time(image_buf.back()->header.stamp.sec, image_buf.back()->header.stamp.nanosec).seconds() >= rclcpp::Time(point_buf.front()->header.stamp.sec, point_buf.front()->header.stamp.nanosec).seconds() && rclcpp::Time(point_buf.back()->header.stamp.sec, point_buf.back()->header.stamp.nanosec).seconds() >= rclcpp::Time(point_buf.front()->header.stamp.sec, point_buf.front()->header.stamp.nanosec).seconds())
            {
                pose_msg = pose_buf.front();
                pose_buf.pop();
                while (!pose_buf.empty())
                    pose_buf.pop();
                while (rclcpp::Time(image_buf.front()->header.stamp.sec, point_buf.front()->header.stamp.nanosec).seconds() < rclcpp::Time(pose_msg->header.stamp.sec, pose_msg->header.stamp.nanosec).seconds())
                    image_buf.pop();
                image_msg = image_buf.front();
                image_buf.pop();

                while (rclcpp::Time(point_buf.front()->header.stamp.sec, point_buf.front()->header.stamp.nanosec).seconds() < rclcpp::Time(pose_msg->header.stamp.sec, pose_msg->header.stamp.nanosec).seconds())
                    point_buf.pop();
                point_msg = point_buf.front();
                point_buf.pop();
            }
        }
        m_buf.unlock();

        if (pose_msg != NULL)
        {
            // printf(" pose time %f \n", pose_msg->header.stamp.toSec());
            // printf(" point time %f \n", point_msg->header.stamp.toSec());
            // printf(" image time %f \n", image_msg->header.stamp.toSec());
            //  skip fisrt few
            if (skip_first_cnt < SKIP_FIRST_CNT)
            {
                skip_first_cnt++;
                continue;
            }

            if (skip_cnt < SKIP_CNT)
            {
                skip_cnt++;
                continue;
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
                    p_2d_normal.x = point_msg->channels[i].values[0];
                    p_2d_normal.y = point_msg->channels[i].values[1];
                    p_2d_uv.x = point_msg->channels[i].values[2];
                    p_2d_uv.y = point_msg->channels[i].values[3];
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
                start_flag = 1;
                posegraph->addKeyFrame(keyframe, 1);
                m_process.unlock();
                frame_index++;
                last_t = T;
            }
        }

        std::chrono::milliseconds dura(5);
        std::this_thread::sleep_for(dura);
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
