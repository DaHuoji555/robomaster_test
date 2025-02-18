#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.hpp"
#include <opencv2/opencv.hpp>
#include <cmath>

using namespace cv;
using namespace std;

class PredictorNode : public rclcpp::Node {
public:
    PredictorNode() : Node("predictor_node"), has_last_point_(false), has_last_prediction_(false) {
        // 订阅装甲板中心点
        subscription_ = this->create_subscription<geometry_msgs::msg::Point>(
            "armor_center", 10,
            std::bind(&PredictorNode::predict_position, this, std::placeholders::_1));

        // 发布预测点
        publisher_ = this->create_publisher<geometry_msgs::msg::Point>("predicted_point", 10);

        // **相机内参矩阵**
        camera_matrix_ = (Mat_<double>(3, 3) << 
            1795.48075, 0.0, 719.15967,
            0.0, 1788.97397, 554.12545,
            0.0, 0.0, 1.0);

        // **畸变系数**
        dist_coeffs_ = (Mat_<double>(1, 5) << 
            -0.073464, 0.128799, 0.001334, 0.001541, 0.0);

        // **装甲板世界坐标 (单位: 米)**
        object_points_ = {
            Point3f(-0.115, -0.062, 0),  // 左上
            Point3f( 0.115, -0.062, 0),  // 右上
            Point3f( 0.115,  0.062, 0),  // 右下
            Point3f(-0.115,  0.062, 0)   // 左下
        };

        RCLCPP_INFO(this->get_logger(), "PredictorNode initialized with PnP.");
    }

private:
    void predict_position(const geometry_msgs::msg::Point::SharedPtr msg) {
        // **第一帧，不预测**
        if (!has_last_point_) {
            last_center_ = *msg;
            has_last_point_ = true;
            RCLCPP_INFO(this->get_logger(), "First frame received: (%.2f, %.2f)", msg->x, msg->y);
            return;
        }

        // **像素坐标 -> 相机坐标转换**
        vector<Point2f> image_points = {
            Point2f(msg->x - 50, msg->y - 30), // 左上
            Point2f(msg->x + 50, msg->y - 30), // 右上
            Point2f(msg->x + 50, msg->y + 30), // 右下
            Point2f(msg->x - 50, msg->y + 30)  // 左下
        };

        Mat rvec, tvec;
        bool success = solvePnP(object_points_, image_points, camera_matrix_, dist_coeffs_, rvec, tvec, false, SOLVEPNP_ITERATIVE);

        if (!success) {
            RCLCPP_WARN(this->get_logger(), "PnP failed, skipping prediction.");
            last_center_ = *msg;
            return;
        }

        // **计算目标 3D 位置**
        double x = tvec.at<double>(0);
        double y = tvec.at<double>(1);
        double z = tvec.at<double>(2);

        // **调试信息：输出检测到的 3D 坐标**
        RCLCPP_INFO(this->get_logger(), "Armor 2D: (%.2f, %.2f) | 3D Pos: (%.3f, %.3f, %.3f)", 
                    msg->x, msg->y, x, y, z);

        // **计算速度**
        double vx = (x - last_position_.x);
        double vy = (y - last_position_.y);
        double vz = (z - last_position_.z);

        // **防止帧率不稳定导致跳动**
        double dt = 1.0 / 100.0;  // 10ms 预测窗口

        double v_magnitude = sqrt(vx * vx + vy * vy + vz * vz);
        if (v_magnitude > 10.0) {  // 过滤过快的速度
            RCLCPP_WARN(this->get_logger(), "Unrealistic velocity (%.2f m/s), skipping.", v_magnitude);
            last_position_ = Point3f(x, y, z);
            last_center_ = *msg;
            return;
        }

        // **预测 3D 位置**
        Point3f predicted_3D(x + vx * dt, y + vy * dt, z + vz * dt);

        // **调试信息：预测的 3D 位置**
        RCLCPP_INFO(this->get_logger(), "Predicted 3D: (%.3f, %.3f, %.3f)", 
                    predicted_3D.x, predicted_3D.y, predicted_3D.z);

        // **使用新的 tvec 进行投影**
        Mat tvec_pred = (Mat_<double>(3, 1) << predicted_3D.x, predicted_3D.y, predicted_3D.z);

        vector<Point3f> predicted_3D_points = {Point3f(0, 0, 0)};  // 预测点的相机坐标（假设预测点中心）
        vector<Point2f> projected_2D_points;

        cv::projectPoints(predicted_3D_points, rvec, tvec_pred, camera_matrix_, dist_coeffs_, projected_2D_points);
        
        cv::Point2f predicted_pixel = projected_2D_points[0];

        // **计算投影误差**
        double proj_error = sqrt(pow(predicted_pixel.x - msg->x, 2) + pow(predicted_pixel.y - msg->y, 2));

        // **如果投影误差过大，跳过**
        if (proj_error > 200.0) {
            RCLCPP_WARN(this->get_logger(), "Projection Error Too Large (%.2f), Skipping.", proj_error);
            return;
        }

        // **发布预测的 2D 位置**
        geometry_msgs::msg::Point predicted_msg_2D;
        predicted_msg_2D.x = predicted_pixel.x;
        predicted_msg_2D.y = predicted_pixel.y;
        predicted_msg_2D.z = 0.0;

        publisher_->publish(predicted_msg_2D);

        // **调试信息：投影回来的 2D 坐标**
        RCLCPP_INFO(this->get_logger(), "Projected 2D: (%.2f, %.2f)", predicted_pixel.x, predicted_pixel.y);

        // **更新上一帧状态**
        last_position_ = Point3f(x, y, z);
        last_center_ = *msg;
        has_last_prediction_ = true;
    }

    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr subscription_;
    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr publisher_;

    Mat camera_matrix_;
    Mat dist_coeffs_;
    vector<Point3f> object_points_;

    geometry_msgs::msg::Point last_center_;
    Point3f last_position_;
    bool has_last_point_;
    bool has_last_prediction_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PredictorNode>());
    rclcpp::shutdown();
    return 0;
}
