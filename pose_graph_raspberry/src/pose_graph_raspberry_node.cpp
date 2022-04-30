#include <vector>
#include "rclcpp/rclcpp.hpp"
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <std_msgs/msg/bool.hpp>
#include <cv_bridge/cv_bridge.h>
#include <iostream>
#include <mutex>
#include <queue>
#include <thread>
#include <eigen3/Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
#define SKIP_FIRST_CNT 10
using namespace std;

using std::placeholders::_1;




class PoseGraphRaspberryNode : public rclcpp::Node
{
public:
    PoseGraphRaspberryNode()
            : Node("pose_graph_raspberry")
    {
        this->declare_parameter<std::string>("config_file", "");
        this->get_parameter("config_file", config_file);
        cv::FileStorage fsSettings(config_file, cv::FileStorage::READ);
        if(!fsSettings.isOpened())
        {
            std::cerr << "ERROR: Wrong path to settings" << std::endl;
        }
        fsSettings["image_topic"] >> IMAGE_TOPIC;
        fsSettings.release();
        sub_image = this->create_subscription<sensor_msgs::msg::Image>(IMAGE_TOPIC, 2000, std::bind(&PoseGraphRaspberryNode::image_callback, this, _1));
        sub_point = this->create_subscription<sensor_msgs::msg::PointCloud>("/vins_estimator/keyframe_point", 2000, std::bind(&PoseGraphRaspberryNode::point_callback, this, _1));
        sub_pose = this->create_subscription<nav_msgs::msg::Odometry>("/vins_estimator/keyframe_pose", 2000, std::bind(&PoseGraphRaspberryNode::pose_callback, this, _1));

        pub_image = this->create_publisher<sensor_msgs::msg::Image>("/pose_graph_raspberry/image", 1000);
        pub_points = this->create_publisher<sensor_msgs::msg::PointCloud>("/pose_graph_raspberry/pointcloud", 1000);
        pub_pose = this->create_publisher<nav_msgs::msg::Odometry>("/pose_graph_raspberry/pose", 1000);


        processThread = std::thread(&PoseGraphRaspberryNode::process, this);

    }

    ~PoseGraphRaspberryNode() { processThread.join();}

private:

    std::thread processThread;

    double last_image_time = -1;
    queue<sensor_msgs::msg::Image::ConstSharedPtr> image_buf;
    queue<sensor_msgs::msg::PointCloud::ConstSharedPtr> point_buf;
    queue<nav_msgs::msg::Odometry::ConstSharedPtr> pose_buf;
    queue<Eigen::Vector3d> odometry_buf;
    std::mutex m_buf;
    std::mutex m_process;
    int sequence = 1;
    std::string config_file;
    std::string IMAGE_TOPIC;


    rclcpp::Subscription<sensor_msgs::msg::Image>::ConstSharedPtr sub_image;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud>::ConstSharedPtr sub_point;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::ConstSharedPtr sub_pose;


    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_image;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud>::SharedPtr pub_points;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_pose;

    void new_sequence()
    {
        printf("new sequence\n");
        sequence++;
        printf("sequence cnt %d \n", sequence);
        if (sequence > 5)
        {
            RCLCPP_WARN(this->get_logger(), "only support 5 sequences since it's boring to copy code for more sequences.");
            assert(true);
        }
        m_buf.lock();
        while(!image_buf.empty())
            image_buf.pop();
        while(!point_buf.empty())
            point_buf.pop();
        while(!pose_buf.empty())
            pose_buf.pop();
        while(!odometry_buf.empty())
            odometry_buf.pop();
        m_buf.unlock();
    }

    void image_callback(sensor_msgs::msg::Image::ConstSharedPtr image_msg)
    {
        //ROS_INFO("image_callback!");
        m_buf.lock();
        image_buf.push(image_msg);
        m_buf.unlock();
        //printf(" image time %f \n", image_msg->header.stamp.toSec());

        double imageTime = rclcpp::Time(image_msg->header.stamp.sec,image_msg->header.stamp.nanosec).seconds();
        // detect unstable camera stream
        if (last_image_time != -1 && (imageTime - last_image_time > 1.0 || imageTime < last_image_time))
        {
            RCLCPP_WARN(this->get_logger(),"image discontinue! detect a new sequence!");
            new_sequence();
        }
        last_image_time = imageTime;
    }

    void point_callback(sensor_msgs::msg::PointCloud::ConstSharedPtr point_msg)
    {
        //ROS_INFO("point_callback!");
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

    void pose_callback(nav_msgs::msg::Odometry::ConstSharedPtr pose_msg)
    {
        //ROS_INFO("pose_callback!");
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



    void process()
    {
        while (true)
        {
            sensor_msgs::msg::Image::ConstSharedPtr image_msg = NULL;
            sensor_msgs::msg::PointCloud::ConstSharedPtr point_msg = NULL;
            nav_msgs::msg::Odometry::ConstSharedPtr pose_msg = NULL;

            // find out the messages with same time stamp
            m_buf.lock();
            if(!image_buf.empty() && !point_buf.empty() && !pose_buf.empty())
            {
                if (rclcpp::Time(image_buf.front()->header.stamp.sec,
                                 image_buf.front()->header.stamp.nanosec).seconds() >
                                 rclcpp::Time(pose_buf.front()->header.stamp.sec,
                                              pose_buf.front()->header.stamp.nanosec).seconds())
                {
                    pose_buf.pop();
                    printf("throw pose at beginning\n");
                }
                else if (rclcpp::Time(image_buf.front()->header.stamp.sec,
                                      image_buf.front()->header.stamp.nanosec).seconds()
                                      > rclcpp::Time(point_buf.front()->header.stamp.sec,
                                                     point_buf.front()->header.stamp.nanosec).seconds())
                {
                    point_buf.pop();
                    printf("throw point at beginning\n");
                }
                else if (rclcpp::Time(image_buf.back()->header.stamp.sec,
                                      image_buf.back()->header.stamp.nanosec).seconds()
                                      >= rclcpp::Time(pose_buf.front()->header.stamp.sec,
                                                      pose_buf.front()->header.stamp.nanosec).seconds()
                                                      && rclcpp::Time(point_buf.back()->header.stamp.sec,
                                                                      point_buf.back()->header.stamp.nanosec).seconds()
                                                      >= rclcpp::Time(pose_buf.front()->header.stamp.sec,
                                                                      pose_buf.front()->header.stamp.nanosec).seconds()){
                    pose_msg = pose_buf.front();
                    pose_buf.pop();
                    while (!pose_buf.empty())
                        pose_buf.pop();
                    while (rclcpp::Time(image_buf.front()->header.stamp.sec,image_buf.front()->header.stamp.nanosec).seconds()
                    < rclcpp::Time(pose_msg->header.stamp.sec,pose_msg->header.stamp.nanosec).seconds())
                        image_buf.pop();
                    image_msg = image_buf.front();
                    image_buf.pop();

                    while (rclcpp::Time(point_buf.front()->header.stamp.sec,point_buf.front()->header.stamp.sec).seconds()
                    < rclcpp::Time(pose_msg->header.stamp.sec,pose_msg->header.stamp.nanosec).seconds())
                        point_buf.pop();
                    point_msg = point_buf.front();
                    point_buf.pop();
                }
            }
            m_buf.unlock();

            if (pose_msg != NULL)
            {
                pub_image->publish(*image_msg);
                pub_points->publish(*point_msg);
                pub_pose->publish(*pose_msg);
            }

            std::chrono::milliseconds dura(100);
            std::this_thread::sleep_for(dura);
        }
    }
};







int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PoseGraphRaspberryNode>());
    rclcpp::shutdown();
    return 0;

    /*
    ros::init(argc, argv, "pose_graph_raspberry");
    ros::NodeHandle n("~");
    //posegraph.registerPub(n);

    // read param
   // n.getParam("visualization_shift_x", VISUALIZATION_SHIFT_X);
   // n.getParam("visualization_shift_y", VISUALIZATION_SHIFT_Y);
   // n.getParam("skip_cnt", SKIP_CNT);
    //n.getParam("skip_dis", SKIP_DIS);
    std::string config_file;
    n.getParam("config_file", config_file);
    cv::FileStorage fsSettings(config_file, cv::FileStorage::READ);
    if(!fsSettings.isOpened())
    {
        std::cerr << "ERROR: Wrong path to settings" << std::endl;
    }

    //double camera_visual_size = fsSettings["visualize_camera_size"];
    //cameraposevisual.setScale(camera_visual_size);
    //cameraposevisual.setLineWidth(camera_visual_size / 10.0);


    //LOOP_CLOSURE = fsSettings["loop_closure"];
    std::string IMAGE_TOPIC;
    //int LOAD_PREVIOUS_POSE_GRAPH;
    fsSettings["image_topic"] >> IMAGE_TOPIC;        
    fsSettings.release();


    ros::Subscriber sub_image = n.subscribe(IMAGE_TOPIC, 2000, image_callback); //important
    ros::Subscriber sub_pose = n.subscribe("/vins_estimator/keyframe_pose", 2000, pose_callback); // important
    ros::Subscriber sub_point = n.subscribe("/vins_estimator/keyframe_point", 2000, point_callback); // important
    

    pub_raw_img = n.advertise<sensor_msgs::Image>("raw_image", 1000);
    pub_points = n.advertise<sensor_msgs::PointCloud>("pointcloud", 100);
    pub_pose = n.advertise<nav_msgs::Odometry>("pose", 1000);

    std::thread measurement_process;

    measurement_process = std::thread(process);
    

    ros::spin();
*/
}
