#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/bool.hpp"
#include <cv_bridge/cv_bridge.h>
#include "sensor_msgs/image_encodings.hpp"

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "feature_tracker.h"

using std::placeholders::_1;

class MinimalSubscriber : public rclcpp::Node {
public:

    MinimalSubscriber()
            : Node("feature_tracker") {
        //subscription_ = this->create_subscription<sensor_msgs::msg::Image>(IMAGE_TOPIC, 10, std::bind(&FeatureTrackerNode::img_callback, this, _1));
        subscription_ = this->create_subscription<sensor_msgs::msg::Image>("topic", 10, std::bind(&MinimalSubscriber::img_callback, this, _1));
        pub_restart = this->create_publisher<std_msgs::msg::Bool>("restart", 1000);
    }


private:
    std::string IMAGE_TOPIC;

    bool first_image_flag = true;
    double first_image_time;
    double last_image_time = 0;
    int pub_count = 1;
    bool PUB_THIS_FRAME = true;
    const int NUM_OF_CAM = 1;
    int FREQ;
    int STEREO_TRACK = false;
    int EQUALIZE = false;
    FeatureTracker trackerData[1];
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;

    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_restart;

    void img_callback(const sensor_msgs::msg::Image::SharedPtr img_msg) {
        if (first_image_flag) {
            first_image_flag = false;
            first_image_time = img_msg->header.stamp.sec;
            last_image_time = img_msg->header.stamp.sec;
            return;
        }

        if (img_msg->header.stamp.sec - last_image_time > 1.0 || img_msg->header.stamp.sec < last_image_time) {
            RCLCPP_WARN(this->get_logger(), "image discontinue! reset the feature tracker!");
            first_image_flag = true;
            last_image_time = 0;
            pub_count = 1;
            std_msgs::msg::Bool restart_flag;
            restart_flag.data = true;
            pub_restart->publish(restart_flag);
            return;
        }
        last_image_time = img_msg->header.stamp.sec;

        if (round(1.0 * pub_count / (img_msg->header.stamp.sec - first_image_time)) <= FREQ) {
            PUB_THIS_FRAME = true;
            // reset the frequency control
            if (abs(1.0 * pub_count / (img_msg->header.stamp.sec - first_image_time) - FREQ) < 0.01 * FREQ) {
                first_image_time = img_msg->header.stamp.sec;
                pub_count = 0;
            }
        } else {
            return;
        }
        cv_bridge::CvImageConstPtr ptr;
        if (img_msg->encoding == "8UC1") {
            sensor_msgs::msg::Image img;
            img.header = img_msg->header;
            img.height = img_msg->height;
            img.width = img_msg->width;
            img.is_bigendian = img_msg->is_bigendian;
            img.step = img_msg->step;
            img.data = img_msg->data;
            img.encoding = "mono8";             
            ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::MONO8);
        } else {
            ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::MONO8);
        }
        cv::Mat show_img = ptr->image;
        TicToc t_r;
        for (int i = 0; i < NUM_OF_CAM; i++) {
            if (i != 1 || !STEREO_TRACK) {
                trackerData[i].readImage(ptr->image.rowRange(ROW * i, ROW * (i + 1)), img_msg->header.stamp.sec);
            } else {
                if (EQUALIZE) {
                    cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE();
                    clahe->apply(ptr->image.rowRange(ROW * i, ROW * (i + 1)), trackerData[i].cur_img);
                } else
                    trackerData[i].cur_img = ptr->image.rowRange(ROW * i, ROW * (i + 1));
            }

        }

    };
};



int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MinimalSubscriber>());
    rclcpp::shutdown();
    return 0;
}