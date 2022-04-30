//
// Created by ubuntu on 4/23/22.
//

#ifndef DEV_WS_POSE_GRAPH_NODE_H
#define DEV_WS_POSE_GRAPH_NODE_H
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
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include "message_filters/sync_policies/approximate_time.h"
#include "message_filters/sync_policies/exact_time.h"
#include "message_filters/synchronizer.h"
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include <tf2_ros/buffer.h>

class PoseGraphNode : public rclcpp::Node {
public:
    PoseGraphNode();
    ~PoseGraphNode();


    //     //std::thread measurement_process;
    // std::shared_ptr<message_filters::TimeSynchronizer<sensor_msgs::msg::Image, sensor_msgs::msg::PointCloud, nav_msgs::msg::Odometry>> temp_sync_;
    std::thread keyboard_command_process;
    std::thread measurement_process;

    queue<sensor_msgs::msg::Image::ConstSharedPtr> image_buf;
    queue<sensor_msgs::msg::PointCloud::ConstSharedPtr> point_buf;
    queue<nav_msgs::msg::Odometry::ConstSharedPtr> pose_buf;
    queue<Eigen::Vector3d> odometry_buf;
    std::mutex m_buf;
    std::mutex m_process;
    int frame_index = 0;
    int sequence = 1;
    PoseGraph* posegraph = NULL;
    int skip_first_cnt = 0;
    int SKIP_CNT;
    int skip_cnt = 0;
    bool load_flag = 0;
    bool start_flag = 0;
    double SKIP_DIS = 0;

    std::string camera_config_file_;
    std::string vins_path_;


    double DOWN_SCALE;
    double DOWN_SCALE_RASPBERRY;

    Eigen::Vector3d tic;
    Eigen::Matrix3d qic;
    int VISUALIZATION_SHIFT_X;
    int VISUALIZATION_SHIFT_Y;
    int ROW;
    int COL;
    int DEBUG_IMAGE;
    int VISUALIZE_IMU_FORWARD;
    int LOOP_CLOSURE;
    int FAST_RELOCALIZATION;
    std::string BRIEF_PATTERN_FILE;
    std::string POSE_GRAPH_SAVE_PATH;
    std::string VINS_RESULT_PATH;
    CameraPoseVisualization cameraposevisual;
    Eigen::Vector3d last_t;
    double last_image_time = -1;
    camodocal::CameraPtr m_camera;

    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    // Pubs
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_match_img;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud>::SharedPtr pub_match_points;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_camera_pose_visual;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_key_odometrys;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_vio_path;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom;

    nav_msgs::msg::Path no_loop_path;


    // Subs:
    rclcpp::Subscription<nav_msgs::msg::Odometry>::ConstSharedPtr sub_imu_forward;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::ConstSharedPtr sub_vio;
//    message_filters::Subscriber<sensor_msgs::msg::Image::ConstSharedPtr> sub_image;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::ConstSharedPtr sub_pose;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::ConstSharedPtr sub_extrinsic;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud>::ConstSharedPtr sub_point;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::ConstSharedPtr sub_relo_relative_pose;

    std::shared_ptr<message_filters::TimeSynchronizer
    <sensor_msgs::msg::Image, sensor_msgs::msg::PointCloud,nav_msgs::msg::Odometry>> temp_sync_;

    message_filters::Subscriber<sensor_msgs::msg::Image> image_sub_;
    message_filters::Subscriber<sensor_msgs::msg::PointCloud> pointcloud_sub_;
    message_filters::Subscriber<nav_msgs::msg::Odometry> pose_sub_;


    int LOAD_PREVIOUS_POSE_GRAPH;

private:



    void readParameters();
    void new_sequence();
    void imu_forward_callback(const nav_msgs::msg::Odometry::ConstSharedPtr forward_msg);
    void UpdateOdometryPose(const Eigen::Vector3d& p, const Eigen::Quaterniond& q,const std_msgs::msg::Header &header);
    void relo_relative_pose_callback(const nav_msgs::msg::Odometry::ConstSharedPtr pose_msg);
    void vio_callback(const nav_msgs::msg::Odometry::ConstSharedPtr pose_msg);
    void extrinsic_callback(const nav_msgs::msg::Odometry::ConstSharedPtr pose_msg);
    void process(sensor_msgs::msg::Image::ConstSharedPtr image_msg,sensor_msgs::msg::PointCloud::ConstSharedPtr point_msg, nav_msgs::msg::Odometry::ConstSharedPtr pose_msg);
    void command();
};

#endif //DEV_WS_POSE_GRAPH_NODE_H
