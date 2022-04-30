#include <cstdio>
#include <queue>
#include <thread>
#include <mutex>
#include <condition_variable>
#include "rclcpp/rclcpp.hpp"
#include <cv_bridge/cv_bridge.h>
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/point_cloud.hpp"
#include "std_msgs/msg/bool.hpp"
#include "estimator.h"
#include "parameters.h"
#include "utility/visualization.h"
//#include <message_filters/subscriber.h>
//#include <message_filters/time_synchronizer.h>

using std::placeholders::_1;

class VinsEstimatorNode : public rclcpp::Node
{
public:
    VinsEstimatorNode()
        : Node("vins_estimator")
    {

        this->declare_parameter<std::string>("config_file", "");
        this->get_parameter("config_file", camera_config_file_);
        RCLCPP_INFO(this->get_logger(), "CONFIG FILE: %s", camera_config_file_.c_str());
        readParameters(camera_config_file_, this->get_logger());
        estimator.setParameter();
#ifdef EIGEN_DONT_PARALLELIZE
        RCLCPP_DEBUG(this->get_logger(), "EIGEN_DONT_PARALLELIZE");
#endif
        RCLCPP_WARN(this->get_logger(), "waiting for image and imu...");
        registerPub(*this);

        RCLCPP_INFO(this->get_logger(), "IMU_TOPIC : %s", IMU_TOPIC.c_str());



        RCLCPP_INFO(this->get_logger(), "####START SUBS: ");
        subscription_imu_ = this->create_subscription<sensor_msgs::msg::Imu>(IMU_TOPIC, rclcpp::SensorDataQoS(), std::bind(&VinsEstimatorNode::imu_callback, this, _1));
        subscription_feature_ = this->create_subscription<sensor_msgs::msg::PointCloud>("feature_tracker/feature", 2000, std::bind(&VinsEstimatorNode::feature_callback, this, _1));
        subscription_restart_ = this->create_subscription<std_msgs::msg::Bool>("feature_tracker/restart", 2000, std::bind(&VinsEstimatorNode::restart_callback, this, _1));
//        this->create_subscription<sensor_msgs::msg::PointCloud>("/match_points", 2000, std::bind(&VinsEstimatorNode::relocalization_callback, this, _1));
        measurement_process_ = std::thread(&VinsEstimatorNode::process, this);
        RCLCPP_INFO(this->get_logger(), "####FINSIH INIT: ");
    }

    ~VinsEstimatorNode() { measurement_process_.join(); }

private:
    std::thread measurement_process_;
    std::string camera_config_file_;
    Estimator estimator = Estimator(this->get_logger());
    rclcpp::Subscription<sensor_msgs::msg::Imu>::ConstSharedPtr subscription_imu_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud>::ConstSharedPtr subscription_feature_;
    rclcpp::Subscription<std_msgs::msg::Bool>::ConstSharedPtr subscription_restart_;
//    rclcpp::Subscription<sensor_msgs::msg::PointCloud>::SharedPtr subscription_relocalization;



    std::condition_variable con;
    double current_time = -1;
    queue<sensor_msgs::msg::Imu::ConstSharedPtr> imu_buf;
    queue<sensor_msgs::msg::PointCloud::ConstSharedPtr> feature_buf;
    queue<sensor_msgs::msg::PointCloud::SharedPtr> relo_buf;
    int sum_of_wait = 0;

    std::mutex m_buf;
    std::mutex m_state;
    std::mutex i_buf;
    std::mutex m_estimator;

    double latest_time;
    Eigen::Vector3d tmp_P;
    Eigen::Quaterniond tmp_Q;
    Eigen::Vector3d tmp_V;
    Eigen::Vector3d tmp_Ba;
    Eigen::Vector3d tmp_Bg;
    Eigen::Vector3d acc_0;
    Eigen::Vector3d gyr_0;
    bool init_feature = 0;
    bool init_imu = 1;
    double last_imu_t = 0;

    void imu_callback(const sensor_msgs::msg::Imu::ConstSharedPtr imu_msg)
    {
//        RCLCPP_WARN(this->get_logger(), "got imu message ");

        if (rclcpp::Time(imu_msg->header.stamp.sec,imu_msg->header.stamp.nanosec).seconds() <= last_imu_t)
        {
            RCLCPP_WARN(this->get_logger(), "imu message in disorder!");
            return;
        }

        m_buf.lock();
        imu_buf.push(imu_msg);
        m_buf.unlock();
        con.notify_one();

        last_imu_t = rclcpp::Time(imu_msg->header.stamp.sec,imu_msg->header.stamp.nanosec).seconds();

        {
            std::lock_guard<std::mutex> lg(m_state);
            predict(imu_msg);
            std_msgs::msg::Header header = imu_msg->header;
            header.frame_id = "odom";
            if (estimator.solver_flag == Estimator::SolverFlag::NON_LINEAR){
               // pubLatestOdometry(tmp_P, tmp_Q, tmp_V, header);
            }
        }
    }

    void feature_callback(const sensor_msgs::msg::PointCloud::ConstSharedPtr feature_msg)
    {
//        RCLCPP_INFO(this->get_logger(), "####start FEATURE CB ");
        if (!init_feature)
        {
            // skip the first detected feature, which doesn't contain optical flow speed
            init_feature = true;
            return;
        }
        m_buf.lock();
        feature_buf.push(feature_msg);
//        RCLCPP_INFO(this->get_logger(), "####FINSIH FEATURE CB ");
        m_buf.unlock();
        con.notify_one();
    }

    void restart_callback(const std_msgs::msg::Bool::ConstSharedPtr restart_msg)
    {
        if (restart_msg->data)
        {
            RCLCPP_WARN(this->get_logger(), "restart the estimator!");
            m_buf.lock();
            while (!feature_buf.empty())
            {
                feature_buf.pop();
            }
            while (!imu_buf.empty()){
                imu_buf.pop();
            }
            m_buf.unlock();
            m_estimator.lock();
            estimator.clearState();
            estimator.setParameter();
            m_estimator.unlock();
            current_time = -1;
            last_imu_t = 0;
        }
    }

    void relocalization_callback(const sensor_msgs::msg::PointCloud::SharedPtr& points_msg)
    {
        // printf("relocalization callback! \n");
        m_buf.lock();
        relo_buf.push(points_msg);
        m_buf.unlock();
    }

    void predict(const sensor_msgs::msg::Imu::ConstSharedPtr &imu_msg)
    {
        double t = rclcpp::Time(imu_msg->header.stamp.sec,imu_msg->header.stamp.nanosec).seconds();
        if (init_imu)
        {
            latest_time = t;
            init_imu = false;
            return;
        }
        double dt = t - latest_time;
        latest_time = t;

        double dx = imu_msg->linear_acceleration.x;
        double dy = imu_msg->linear_acceleration.y;
        double dz = imu_msg->linear_acceleration.z;
        Eigen::Vector3d linear_acceleration{dx, dy, dz};

        double rx = imu_msg->angular_velocity.x;
        double ry = imu_msg->angular_velocity.y;
        double rz = imu_msg->angular_velocity.z;
        Eigen::Vector3d angular_velocity{rx, ry, rz};

        Eigen::Vector3d un_acc_0 = tmp_Q * (acc_0 - tmp_Ba) - estimator.g;

        Eigen::Vector3d un_gyr = 0.5 * (gyr_0 + angular_velocity) - tmp_Bg;
        tmp_Q = tmp_Q * Utility::deltaQ(un_gyr * dt);

        Eigen::Vector3d un_acc_1 = tmp_Q * (linear_acceleration - tmp_Ba) - estimator.g;

        Eigen::Vector3d un_acc = 0.5 * (un_acc_0 + un_acc_1);

        tmp_P = tmp_P + dt * tmp_V + 0.5 * dt * dt * un_acc;
        tmp_V = tmp_V + dt * un_acc;

        acc_0 = linear_acceleration;
        gyr_0 = angular_velocity;
    }

    void update()
    {
        TicToc t_predict;
        latest_time = current_time;
        tmp_P = estimator.Ps[WINDOW_SIZE];
        tmp_Q = estimator.Rs[WINDOW_SIZE];
        tmp_V = estimator.Vs[WINDOW_SIZE];
        tmp_Ba = estimator.Bas[WINDOW_SIZE];
        tmp_Bg = estimator.Bgs[WINDOW_SIZE];
        acc_0 = estimator.acc_0;
        gyr_0 = estimator.gyr_0;

        queue<sensor_msgs::msg::Imu::ConstSharedPtr> tmp_imu_buf = imu_buf;
        for (sensor_msgs::msg::Imu::SharedPtr tmp_imu_msg; !tmp_imu_buf.empty(); tmp_imu_buf.pop())
            predict(tmp_imu_buf.front());
    }

    std::vector<std::pair<std::vector<sensor_msgs::msg::Imu::ConstSharedPtr>, sensor_msgs::msg::PointCloud::ConstSharedPtr>>
    getMeasurements()
    {
        std::vector<std::pair<std::vector<sensor_msgs::msg::Imu::ConstSharedPtr>, sensor_msgs::msg::PointCloud::ConstSharedPtr>> measurements;

        while (true)
        {
            if (imu_buf.empty() || feature_buf.empty())
                return measurements;

            double imu_buf_back_time = rclcpp::Time(imu_buf.back()->header.stamp.sec, imu_buf.back()->header.stamp.nanosec).seconds();
            double imu_buf_front_time = rclcpp::Time(imu_buf.front()->header.stamp.sec, imu_buf.front()->header.stamp.nanosec).seconds();
            double feature_buf_front_time = rclcpp::Time(feature_buf.front()->header.stamp.sec,feature_buf.front()->header.stamp.nanosec).seconds();

            if (imu_buf_back_time <= feature_buf_front_time + estimator.td)
            {
                RCLCPP_WARN(this->get_logger(), "wait for imu, only should happen at the beginning");
                // ROS_WARN("wait for imu, only should happen at the beginning");
                sum_of_wait++;
                return measurements;
            }
            if (imu_buf_front_time >= feature_buf_front_time + estimator.td)
            {
                RCLCPP_WARN(this->get_logger(), "throw img, only should happen at the beginning");
                feature_buf.pop();
                continue;
            }
            sensor_msgs::msg::PointCloud::ConstSharedPtr img_msg = feature_buf.front();
            feature_buf.pop();
            std::vector<sensor_msgs::msg::Imu::ConstSharedPtr> IMUs;
            while (imu_buf_front_time
            < feature_buf_front_time + estimator.td)
            {
                IMUs.emplace_back(imu_buf.front());
                imu_buf.pop();
                imu_buf_front_time = rclcpp::Time(imu_buf.front()->header.stamp.sec, imu_buf.front()->header.stamp.nanosec).seconds();
            }
            IMUs.emplace_back(imu_buf.front());
            if (IMUs.empty())
                RCLCPP_INFO(this->get_logger(), "no imu between two image");
            measurements.emplace_back(IMUs, img_msg);
        }
    }

    // thread: visual-inertial odometry
    void process()
    {
        while (true)
        {
            std::vector<std::pair<std::vector<sensor_msgs::msg::Imu::ConstSharedPtr>, sensor_msgs::msg::PointCloud::ConstSharedPtr>>
                measurements;
            std::unique_lock<std::mutex> lk(m_buf);
            con.wait(lk, [&]
            {
                return !(measurements = getMeasurements()).empty();
            });
            lk.unlock();
            m_estimator.lock();

            for (auto &measurement : measurements)
            {
                auto img_msg = measurement.second;
                double dx = 0, dy = 0, dz = 0, rx = 0, ry = 0, rz = 0;
                for (auto &imu_msg : measurement.first)
                {
                    double t = rclcpp::Time(imu_msg->header.stamp.sec,imu_msg->header.stamp.nanosec).seconds();
                    double img_t = rclcpp::Time(img_msg->header.stamp.sec,img_msg->header.stamp.nanosec).seconds() + estimator.td;
                    if (t <= img_t)
                    {
                        if (current_time < 0)
                            current_time = t;
                        double dt = t - current_time;
                        assert(dt >= 0);
                        current_time = t;
                        dx = imu_msg->linear_acceleration.x;
                        dy = imu_msg->linear_acceleration.y;
                        dz = imu_msg->linear_acceleration.z;
                        rx = imu_msg->angular_velocity.x;
                        ry = imu_msg->angular_velocity.y;
                        rz = imu_msg->angular_velocity.z;
                        estimator.processIMU(dt, Vector3d(dx, dy, dz), Vector3d(rx, ry, rz));
                        // printf("imu: dt:%f a: %f %f %f w: %f %f %f\n",dt, dx, dy, dz, rx, ry, rz);
                    }
                    else
                    {
                        double dt_1 = img_t - current_time;
                        double dt_2 = t - img_t;
                        current_time = img_t;
                        assert(dt_1 >= 0);
                        assert(dt_2 >= 0);
                        assert(dt_1 + dt_2 > 0);
                        double w1 = dt_2 / (dt_1 + dt_2);
                        double w2 = dt_1 / (dt_1 + dt_2);
                        dx = w1 * dx + w2 * imu_msg->linear_acceleration.x;
                        dy = w1 * dy + w2 * imu_msg->linear_acceleration.y;
                        dz = w1 * dz + w2 * imu_msg->linear_acceleration.z;
                        rx = w1 * rx + w2 * imu_msg->angular_velocity.x;
                        ry = w1 * ry + w2 * imu_msg->angular_velocity.y;
                        rz = w1 * rz + w2 * imu_msg->angular_velocity.z;
                        estimator.processIMU(dt_1, Vector3d(dx, dy, dz), Vector3d(rx, ry, rz));
                        // printf("dimu: dt:%f a: %f %f %f w: %f %f %f\n",dt_1, dx, dy, dz, rx, ry, rz);
                    }
                }
                // set re-localization frame
                sensor_msgs::msg::PointCloud::SharedPtr relo_msg = nullptr;
                while (!relo_buf.empty())
                {
                    relo_msg = relo_buf.front();
                    relo_buf.pop();
                }
                if (relo_msg != nullptr)
                {
                    vector<Vector3d> match_points;
                    double frame_stamp = rclcpp::Time(relo_msg->header.stamp.sec,relo_msg->header.stamp.nanosec).seconds();
                    for (auto & point : relo_msg->points)
                    {
                        Vector3d u_v_id;
                        u_v_id.x() = point.x;
                        u_v_id.y() = point.y;
                        u_v_id.z() = point.z;
                        match_points.push_back(u_v_id);
                    }
                    Vector3d relo_t(relo_msg->channels[0].values[0], relo_msg->channels[0].values[1], relo_msg->channels[0].values[2]);
                    Quaterniond relo_q(relo_msg->channels[0].values[3], relo_msg->channels[0].values[4], relo_msg->channels[0].values[5], relo_msg->channels[0].values[6]);
                    Matrix3d relo_r = relo_q.toRotationMatrix();
                    int frame_index;
                    frame_index = relo_msg->channels[0].values[7];
                    estimator.setReloFrame(frame_stamp, frame_index, match_points, relo_t, relo_r);
                }

                RCLCPP_DEBUG(this->get_logger(), "processing vision data with stamp %f \n", rclcpp::Time(img_msg->header.stamp.sec,img_msg->header.stamp.nanosec).seconds());

                TicToc t_s;
                map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> image;
                for (unsigned int i = 0; i < img_msg->points.size(); i++)
                {
                    int v = img_msg->channels[0].values[i] + 0.5;
                    int feature_id = v / NUM_OF_CAM;
                    int camera_id = v % NUM_OF_CAM;
                    double x = img_msg->points[i].x;
                    double y = img_msg->points[i].y;
                    double z = img_msg->points[i].z;
                    double p_u = img_msg->channels[1].values[i];
                    double p_v = img_msg->channels[2].values[i];
                    double velocity_x = img_msg->channels[3].values[i];
                    double velocity_y = img_msg->channels[4].values[i];
                    assert(z == 1);
                    Eigen::Matrix<double, 7, 1> xyz_uv_velocity;
                    xyz_uv_velocity << x, y, z, p_u, p_v, velocity_x, velocity_y;
                    image[feature_id].emplace_back(camera_id, xyz_uv_velocity);
                }

                estimator.processImage(image, img_msg->header);

                double whole_t = t_s.toc();
                printStatistics(estimator, whole_t, this->get_logger());
                std_msgs::msg::Header header = img_msg->header;
                header.frame_id = "odom";

                pubTF(estimator, header);
                pubOdometry(estimator, header);
                pubKeyframe(estimator, header);
                pubCameraPose(estimator, header);
                if (relo_msg != nullptr){
                    pubRelocalization(estimator);
                }

            }

            m_estimator.unlock();
            m_buf.lock();
            m_state.lock();
            if (estimator.solver_flag == Estimator::SolverFlag::NON_LINEAR) {
                update();
            }
            m_state.unlock();
            m_buf.unlock();
        }
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VinsEstimatorNode>());
    rclcpp::shutdown();
    return 0;
}
