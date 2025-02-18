#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

class VideoPublisher : public rclcpp::Node {
public:
    VideoPublisher() : Node("video_publisher") {
        publisher_ = this->create_publisher<sensor_msgs::msg::Image>("image_raw", 10);
        timer_ = this->create_wall_timer(std::chrono::milliseconds(30),
                                         std::bind(&VideoPublisher::publish_frame, this));

        // 打开视频文件
        cap_.open("/home/lyl/robomaster/video/output.avi", cv::CAP_FFMPEG);

        if (!cap_.isOpened()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open video file.");
        } else {
            original_width_ = static_cast<int>(cap_.get(cv::CAP_PROP_FRAME_WIDTH));
            original_height_ = static_cast<int>(cap_.get(cv::CAP_PROP_FRAME_HEIGHT));
            RCLCPP_INFO(this->get_logger(), "Video opened successfully. Original resolution: %d x %d",
                        original_width_, original_height_);
        }

        // 设定目标尺寸
        target_width_ = 1440;  // 目标宽度
        target_height_ = 1080; // 目标高度
    }

private:
    void publish_frame() {
        cv::Mat frame, resized_frame;
        cap_ >> frame;
        if (frame.empty()) return;

        // 调整图像大小到指定的 target_width_ x target_height_
        cv::resize(frame, resized_frame, cv::Size(target_width_, target_height_));

        // 发布缩放后的图像
        auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", resized_frame).toImageMsg();
        publisher_->publish(*msg);
    }

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    cv::VideoCapture cap_;

    int original_width_, original_height_; // 原始分辨率
    int target_width_, target_height_;     // 目标分辨率
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VideoPublisher>());
    rclcpp::shutdown();
    return 0;
}
