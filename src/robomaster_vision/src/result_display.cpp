#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "geometry_msgs/msg/point.hpp"
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

class ResultDisplay : public rclcpp::Node {
public:
    ResultDisplay() : Node("result_display"), has_predicted_point_(false) {
        // 订阅处理后的图像数据
        subscription_image_ = this->create_subscription<sensor_msgs::msg::Image>(
            "image_processed", 10, std::bind(&ResultDisplay::show_frame, this, std::placeholders::_1));

        // 订阅预测点数据
        subscription_point_ = this->create_subscription<geometry_msgs::msg::Point>(
            "predicted_point", 10, std::bind(&ResultDisplay::update_predicted_point, this, std::placeholders::_1));

        // 初始化显示窗口
        cv::namedWindow("Result Display", cv::WINDOW_NORMAL);
        cv::resizeWindow("Result Display", 1440, 1080);  // 设置窗口大小

        RCLCPP_INFO(this->get_logger(), "ResultDisplay node has started.");
    }

private:
    // 处理订阅的图像数据并显示
    void show_frame(const sensor_msgs::msg::Image::SharedPtr msg) {
        cv::Mat frame = cv_bridge::toCvCopy(msg, "bgr8")->image;

        // 仅在当前帧内收到预测点时才绘制
        if (has_predicted_point_) {
            cv::circle(frame, cv::Point(predicted_point_.x, predicted_point_.y), 
                       5, cv::Scalar(0, 165, 255), -1);

            std::string point_text = "(" + std::to_string((int)predicted_point_.x) + 
                                     ", " + std::to_string((int)predicted_point_.y) + ")";
            cv::putText(frame, point_text, cv::Point(predicted_point_.x + 10, predicted_point_.y - 10),
                        cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1);
        }

        // **关键修改：每帧刷新后重置 `has_predicted_point_`，等待新消息**
        has_predicted_point_ = false;

        // 显示处理后的画面
        cv::imshow("Result Display", frame);
        cv::waitKey(1);
    }

    // 处理订阅的预测点数据
    void update_predicted_point(const geometry_msgs::msg::Point::SharedPtr msg) {
        predicted_point_.x = msg->x;
        predicted_point_.y = msg->y;
        has_predicted_point_ = true;  // 仅当本帧收到预测点时才绘制

        RCLCPP_INFO(this->get_logger(), "Updated predicted point: (%.2f, %.2f)", msg->x, msg->y);
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_image_; // 订阅图像数据
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr subscription_point_; // 订阅预测点数据

    cv::Point predicted_point_;  // 存储最新的预测点坐标
    bool has_predicted_point_;   // **每帧刷新后重置，防止绘制过时的预测点**
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ResultDisplay>());
    rclcpp::shutdown();
    return 0;
}
