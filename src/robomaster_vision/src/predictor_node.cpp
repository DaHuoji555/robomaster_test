#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.hpp"
#include <cmath>

class PredictorNode : public rclcpp::Node {
public:
    PredictorNode() : Node("predictor_node"), has_last_point_(false) {
        // 订阅装甲板中心点
        subscription_ = this->create_subscription<geometry_msgs::msg::Point>(
            "armor_center", 10,
            std::bind(&PredictorNode::predict_position, this, std::placeholders::_1));

        // 发布预测点
        publisher_ = this->create_publisher<geometry_msgs::msg::Point>("predicted_point", 10);

        RCLCPP_INFO(this->get_logger(), "PredictorNode has started.");
    }

private:
    void predict_position(const geometry_msgs::msg::Point::SharedPtr msg) {
        // 如果这是第一帧，直接保存并跳过预测
        if (!has_last_point_) {
            last_center_ = *msg;
            has_last_point_ = true;
            RCLCPP_INFO(this->get_logger(), "First frame received: (%.2f, %.2f)", msg->x, msg->y);
            return;
        }

        // 计算两帧之间的欧几里得距离
        double distance = std::sqrt(std::pow(msg->x - last_center_.x, 2) + 
                                    std::pow(msg->y - last_center_.y, 2));

        // 如果变化过大（大于 50 像素），认为异常，跳过预测，并不发布预测点
        if (distance > 50.0) {
            RCLCPP_WARN(this->get_logger(), "Too large movement detected (%.2f), skipping prediction.", distance);
            last_center_ = *msg;
            return;
        }

        // 计算速度（假设时间间隔恒定，每帧间隔 1 个单位时间）
        double velocity_x = msg->x - last_center_.x;
        double velocity_y = msg->y - last_center_.y;

        // 计算运动方向（角度）
        double angle = std::atan2(velocity_y, velocity_x) * 180.0 / M_PI;

        // 预测下一帧的位置（假设匀速运动）
        geometry_msgs::msg::Point predicted_msg;
        predicted_msg.x = msg->x + velocity_x;
        predicted_msg.y = msg->y + velocity_y;
        predicted_msg.z = msg->z;

        RCLCPP_INFO(this->get_logger(), "Received center: (%.2f, %.2f), Velocity: (%.2f, %.2f), Angle: %.2f°, Predicted: (%.2f, %.2f)",
                    msg->x, msg->y, velocity_x, velocity_y, angle, predicted_msg.x, predicted_msg.y);

        // 仅在正常情况下发布预测点
        publisher_->publish(predicted_msg);

        // 更新上一帧的中心点
        last_center_ = *msg;
    }

    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr subscription_;
    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr publisher_;

    geometry_msgs::msg::Point last_center_; // 记录上一帧的中心点
    bool has_last_point_;  // 是否接收过第一帧数据
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PredictorNode>());
    rclcpp::shutdown();
    return 0;
}
