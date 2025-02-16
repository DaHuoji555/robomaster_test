#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

class ResultDisplay : public rclcpp::Node {
public:
    ResultDisplay() : Node("result_display") {
        subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            "image_processed", 10, std::bind(&ResultDisplay::show_frame, this, std::placeholders::_1));

        cv::namedWindow("Result Display", cv::WINDOW_NORMAL);
        cv::resizeWindow("Result Display", 640, 480);  // 将窗口初始化为 640x480
    }

private:
    void show_frame(const sensor_msgs::msg::Image::SharedPtr msg) {
        cv::Mat frame = cv_bridge::toCvCopy(msg, "bgr8")->image;
        // 你也可以在这里再调整尺寸，如： cv::resize(frame, frame, cv::Size(640, 480));
        cv::imshow("Result Display", frame);
        cv::waitKey(1);
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ResultDisplay>());
    rclcpp::shutdown();
    return 0;
}
