#include <vector>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <std_msgs/msg/bool.hpp>
#include <cv_bridge/cv_bridge.h>
#include <iostream>
// #include <ros/package.h>
#include <mutex>
#include <queue>
#include <thread>
#include <eigen3/Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
#include "keyframe.h"
#include "utility/tic_toc.h"
#include "pose_graph.h"
#include "utility/CameraPoseVisualization.h"
#include "parameters.h"
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#define SKIP_FIRST_CNT 10
using namespace std;
CameraPoseVisualization cameraposevisual(1, 0, 0, 1);
Eigen::Vector3d last_t(-100, -100, -100);
class PoseGraphNode : public rclcpp::Node
{
public:
    PoseGraphNode()
        : Node("pose_graph")
    {
        this->declare_parameter<std::string>("config_file", "");
        this->get_parameter("config_file", camera_config_file_);
        RCLCPP_INFO(this->get_logger(), "CONFIG FILE: %s", camera_config_file_.c_str());
        readParameters(camera_config_file_, this->get_logger());
        std::string vocabulary_file = camera_config_file_ + "/../../../support_files/brief_k10L6.bin";
        std::cout << "vocabulary_file" << vocabulary_file << std::endl;
        posegraph.loadVocabulary(vocabulary_file);

        RCLCPP_INFO(this->get_logger(), "LOAD_PREVIOUS_POSE_GRAPH: %i", LOAD_PREVIOUS_POSE_GRAPH);
        RCLCPP_INFO(this->get_logger(), "DOWN_SCALE_RASPBERRY: %i", DOWN_SCALE_RASPBERRY);

        if (LOAD_PREVIOUS_POSE_GRAPH)
        {
            printf("load pose graph\n");
            m_process.lock();
            posegraph.loadPoseGraph();
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
        sub_imu_forward = this->create_subscription<nav_msgs::msg::Odometry>("/vins_estimator/imu_propagate", 2000, std::bind(&PoseGraphNode::imu_forward_callback, this));                    // visuallize
        sub_vio = this->create_subscription<nav_msgs::msg::Odometry>("/vins_estimator/odometry", 2000, std::bind(&PoseGraphNode::vio_callback, this));                                         // visualize
        sub_image = this->create_subscription<sensor_msgs::msg::Image>("/vins_estimator/raw_image", 2000, std::bind(&PoseGraphNode::image_callback, this));                                    // important
        sub_pose = this->create_subscription<nav_msgs::msg::Odometry>("/vins_estimator/keyframe_pose", 2000, std::bind(&PoseGraphNode::pose_callback, this));                                  // important
        sub_extrinsic = this->create_subscription<nav_msgs::msg::Odometry>("/vins_estimator/extrinsic", 2000, std::bind(&PoseGraphNode::extrinsic_callback, this));                            // important
        sub_point = this->create_subscription<sensor_msgs::msg::PointCloud>("/vins_estimator/keyframe_point", 2000, std::bind(&PoseGraphNode::point_callback, this));                          // important
        sub_relo_relative_pose = this->create_subscription<nav_msgs::msg::Odometry>("/vins_estimator/relo_relative_pose", 2000, std::bind(&PoseGraphNode::relo_relative_pose_callback, this)); // important
        // Publisher
        pub_key_odometrys = this->create_publisher<visualization_msgs::msg::Marker>("restart", 1000);
        pub_camera_pose_visual = this->create_publisher<visualization_msgs::msg::MarkerArray>("feature", 1000);
        pub_match_img = this->create_publisher<sensor_msgs::msg::Image>("feature_img", 1000);
        pub_vio_path = this->create_publisher<nav_msgs::msg::Path>("feature_img", 1000);
        pub_match_points = this->create_publisher<sensor_msgs::msg::PointCloud>("feature_img", 100);

        raw_image_sub_.subscribe(this, "/pose_graph_raspberry/raw_image");
        pointcloud_sub_.subscribe(this, "/pose_graph_raspberry/pointcloud");
        pose_sub_.subscribe(this, "/pose_graph_raspberry/pose");
        message_filters::TimeSynchronizer<sensor_msgs::msg::Image, sensor_msgs::msg::PointCloud, nav_msgs::msg::Odometry> sync(raw_image_sub_, pointcloud_sub_, pose_sub_, 2000);
        sync.registerCallback(process);

        //     //std::thread measurement_process;
        std::thread keyboard_command_process;

        //      //measurement_process  CameraPoseVisualization cameraposevisual(1, 0, 0, 1);= std::thread(process);
        keyboard_command_process = std::thread(command);
    }

private:
    queue<sensor_msgs::msg::Image::ConstPtr> image_buf;
    queue<sensor_msgs::msg::PointCloud::ConstPtr> point_buf;
    queue<nav_msgs::msg::Odometry::ConstPtr> pose_buf;
    queue<Eigen::Vector3d> odometry_buf;
    std::mutex m_buf;
    std::mutex m_process;
    int frame_index = 0;
    int sequence = 1;
    PoseGraph posegraph;
    int skip_first_cnt = 0;
    int SKIP_CNT;
    int skip_cnt = 0;
    bool load_flag = 0;
    bool start_flag = 0;
    double SKIP_DIS = 0;
    std::string camera_config_file_;

    int VISUALIZATION_SHIFT_X;
    int VISUALIZATION_SHIFT_Y;
    int ROW;
    int COL;
    int DEBUG_IMAGE;
    int VISUALIZE_IMU_FORWARD;
    int LOOP_CLOSURE;
    int FAST_RELOCALIZATION;
    double DOWN_SCALE;
    double DOWN_SCALE_RASPBERRY;

    Eigen::Matrix3d qic;
    // Subs:
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_imu_forward;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_vio;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_image;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_pose;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_extrinsic;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud>::SharedPtr sub_point;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_relo_relative_pose;
    // Pubs
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_match_img;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud>::SharedPtr pub_match_points;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_camera_pose_visual;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_key_odometrys;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_vio_path;
    nav_msgs::msg::Path no_loop_path;

    message_filters::Subscriber<sensor_msgs::msg::Image> raw_image_sub_;
    message_filters::Subscriber<sensor_msgs::msg::PointCloud> pointcloud_sub_;
    message_filters::Subscriber<nav_msgs::msg::Odometry> pose_sub_;

    std::string BRIEF_PATTERN_FILE;
    std::string POSE_GRAPH_SAVE_PATH;
    std::string VINS_RESULT_PATH;

    double last_image_time = -1;

    void new_sequence()
    {
        printf("new sequence\n");
        sequence++;
        printf("sequence cnt %d \n", sequence);
        if (sequence > 5)
        {
            RCLCPP_WARN(this->get_logger(), "only support 5 sequences since it's boring to copy code for more sequences.");
            rclcpp::shutdown();
        }
        posegraph.posegraph_visualization->reset();
        posegraph.publish();
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

    void image_callback(const sensor_msgs::msg::Image::ConstSharedPtr image_msg)
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

    void point_callback(const sensor_msgs::msg::PointCloud::ConstSharedPtr point_msg)
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

    void pose_callback(const nav_msgs::msg::Odometry::ConstSharedPtr pose_msg)
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

    void imu_forward_callback(const nav_msgs::msg::Odometry::ConstSharedPtr forward_msg)
    {
        if (VISUALIZE_IMU_FORWARD)
        {
            Vector3d vio_t(forward_msg->pose.pose.position.x, forward_msg->pose.pose.position.y, forward_msg->pose.pose.position.z);
            Quaterniond vio_q;
            vio_q.w() = forward_msg->pose.pose.orientation.w;
            vio_q.x() = forward_msg->pose.pose.orientation.x;
            vio_q.y() = forward_msg->pose.pose.orientation.y;
            vio_q.z() = forward_msg->pose.pose.orientation.z;

            vio_t = posegraph.w_r_vio * vio_t + posegraph.w_t_vio;
            vio_q = posegraph.w_r_vio * vio_q;

            vio_t = posegraph.r_drift * vio_t + posegraph.t_drift;
            vio_q = posegraph.r_drift * vio_q;

            Vector3d vio_t_cam;
            Quaterniond vio_q_cam;
            vio_t_cam = vio_t + vio_q * tic;
            vio_q_cam = vio_q * qic;

            cameraposevisual.reset();
            cameraposevisual.add_pose(vio_t_cam, vio_q_cam);
            cameraposevisual.publish_by(pub_camera_pose_visual, forward_msg->header);
        }
    }
    void relo_relative_pose_callback(const nav_msgs::msg::Odometry::ConstSharedPtr pose_msg)
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
        posegraph.updateKeyFrameLoop(index, loop_info);
    }

    void vio_callback(const nav_msgs::msg::Odometry::ConstSharedPtr pose_msg)
    {
        // RCLCPP_INFO(this->get_logger(),"vio_callback!");
        Vector3d vio_t(pose_msg->pose.pose.position.x, pose_msg->pose.pose.position.y, pose_msg->pose.pose.position.z);
        Quaterniond vio_q;
        vio_q.w() = pose_msg->pose.pose.orientation.w;
        vio_q.x() = pose_msg->pose.pose.orientation.x;
        vio_q.y() = pose_msg->pose.pose.orientation.y;
        vio_q.z() = pose_msg->pose.pose.orientation.z;

        vio_t = posegraph.w_r_vio * vio_t + posegraph.w_t_vio;
        vio_q = posegraph.w_r_vio * vio_q;

        vio_t = posegraph.r_drift * vio_t + posegraph.t_drift;
        vio_q = posegraph.r_drift * vio_q;

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

    void extrinsic_callback(const nav_msgs::msg::Odometry::ConstSharedPtr pose_msg)
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

    void process(const sensor_msgs::msg::Image::ConstSharedPtr image_msg, const sensor_msgs::msg::PointCloud::ConstSharedPtr point_msg, const nav_msgs::msg::Odometry::ConstSharedPtr pose_msg)
    {

        // printf(" pose time %f \n", pose_msg->header.stamp.toSec());
        // printf(" point time %f \n", point_msg->header.stamp.toSec());
        // printf(" image time %f \n", image_msg->header.stamp.toSec());
        //  skip fisrt few
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
        if (DOWN_SCALE > 1)
        {
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

                // printf("u %f, v %f \n", p_2d_uv.x, p_2d_uv.y);
            }

            KeyFrame *keyframe = new KeyFrame(rclcpp::Time(pose_msg->header.stamp.sec, pose_msg->header.stamp.nanosec).seconds(), frame_index, T, R, image,
                                              point_3d, point_2d_uv, point_2d_normal, point_id, sequence);
            m_process.lock();
            start_flag = 1;
            posegraph.addKeyFrame(keyframe, 1);
            m_process.unlock();
            frame_index++;
            last_t = T;
        }
    }

    void command()
    {
        if (!LOOP_CLOSURE)
            return;
        while (1)
        {
            char c = getchar();
            if (c == 's')
            {
                m_process.lock();
                posegraph.savePoseGraph();
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
};
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    rclcpp::spin(std::make_shared<PoseGraphNode>());
    rclcpp::shutdown();
    return 0;

    // posegraph.registerPub(n);
}
