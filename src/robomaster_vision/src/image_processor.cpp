#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "geometry_msgs/msg/point.hpp"  // 用于发布装甲板中心点
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

// 你原先的头文件
#include "Armor.h"
#include "Light.h"
#include "KNN.h"
#include "Judge_Light.h"
#include <unistd.h>  // 需要这个头文件
#include <iostream>


using namespace cv;
using namespace std;
using namespace cv::ml;

// 声明外部函数，如 trainKNNFromDataset(), filterAndAddLights(), predictLabel() 等
// 确保它们在对应的头文件里有正确声明。

class ImageProcessor : public rclcpp::Node {
public:
    ImageProcessor()
    : Node("image_processor") 
    {

        char cwd[1024];
        getcwd(cwd, sizeof(cwd));
        std::cout << "当前工作目录: " << cwd << std::endl;

        // 1. 加载或训练 KNN
        knn_ = trainKNNFromDataset("dataset");
        if (!knn_) {
            RCLCPP_ERROR(this->get_logger(), "Failed to load or train KNN!");
        }

        // 2. 订阅 原始图像
        subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            "image_raw", 10,
            std::bind(&ImageProcessor::process_frame, this, std::placeholders::_1));

        // 3. 发布 处理后的图像
        publisher_ = this->create_publisher<sensor_msgs::msg::Image>(
            "image_processed", 10);

        // 4. **新增** 发布最大装甲板中心点
        publisher_center_ = this->create_publisher<geometry_msgs::msg::Point>(
            "armor_center", 10);

        RCLCPP_INFO(this->get_logger(), "ImageProcessor node has been started.");
    }

private:
    void process_frame(const sensor_msgs::msg::Image::SharedPtr msg) 
    {
        // === (A) 将 ROS 图像转换成 OpenCV Mat ===
        cv::Mat frame = cv_bridge::toCvCopy(msg, "bgr8")->image;
        if (frame.empty()) {
            RCLCPP_WARN(this->get_logger(), "Received an empty frame.");
            return;
        }

        // === (B) 转为灰度图并二值化 ===
        cv::Mat gray, binary;
        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
        cv::threshold(gray, binary, 220, 255, cv::THRESH_BINARY);

        // === (C) 形态学操作 (腐蚀 + 膨胀) ===
        cv::Mat erosionKernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
        cv::Mat dilationKernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));

        cv::Mat temp, morph;
        cv::erode(binary, temp, erosionKernel, cv::Point(-1, -1), 2);
        cv::dilate(temp, morph, dilationKernel, cv::Point(-1, -1), 3);

        // === (D) 寻找轮廓并过滤成灯条 ===
        vector<vector<Point>> contours;
        vector<Vec4i> hierarchy;
        cv::findContours(morph, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        vector<Light> lights;
        filterAndAddLights(contours, lights);

        // === (E) 匹配灯条成装甲板 ===
        vector<vector<Point2f>> boards = Light::light_match(lights, frame);
        vector<Armor> armors;
        for (auto & board : boards) {
            armors.emplace_back(board);
        }

        // === (F) 用 KNN 预测装甲板数字，并剔除标签为 10 的装甲板 ===
        for (auto it = armors.begin(); it != armors.end();) {
            it->number = predictLabel(knn_, it->transformToMatrix(frame));
            if (it->number == 10) {
                it = armors.erase(it);
            } else {
                ++it;
            }
        }

        // === (G) 绘制灯条信息 ===
        for (const auto& light : lights) {
            Point2f vertices[4];
            light.rect.points(vertices);
            for (int j = 0; j < 4; j++) {
                cv::line(frame, vertices[j], vertices[(j + 1) % 4], Scalar(0, 255, 0), 2);
            }
        }

       // === (H) 绘制装甲板信息 ===
        double max_area = 0.0;  // 记录最大装甲板的面积
        Point2f max_center;      // 记录最大装甲板的中心点
        bool has_max_armor = false; // 记录是否找到最大装甲板

        for (const auto& armor : armors) {
            // 计算四边形面积
            double area = contourArea(std::vector<cv::Point2f>(std::begin(armor.originalPoints), std::end(armor.originalPoints)));
            
            // 画四边形
            for (int k = 0; k < 4; k++) {
                cv::line(frame, armor.originalPoints[k],
                        armor.originalPoints[(k + 1) % 4],
                        Scalar(0, 255, 0), 2);
            }

            // 计算中心点
            Point2f center = (armor.originalPoints[0] + armor.originalPoints[2]) * 0.5f;
            cv::circle(frame, center, 5, Scalar(0, 0, 255), -1);

            // 显示坐标
            string centerText = format("(%d, %d)", (int)center.x, (int)center.y);
            cv::putText(frame, centerText, center + Point2f(10, 10),
                        cv::FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255, 255, 255), 1);

            // 显示 KNN 预测数字
            cv::putText(frame, format("(%d)", armor.number),
                        center + Point2f(20, 20),
                        cv::FONT_HERSHEY_SIMPLEX,
                        1.0,
                        Scalar(255, 0, 255),
                        2);

            // 记录最大装甲板的信息
            if (area > max_area) {
                max_area = area;
                max_center = center;
                has_max_armor = true;
            }
        }

        // === 仅发布最大装甲板的中心点 ===
        if (has_max_armor) {
            geometry_msgs::msg::Point center_msg;
            center_msg.x = max_center.x;
            center_msg.y = max_center.y;
            center_msg.z = 0.0;  // 2D 图像，Z 轴设为 0

            publisher_center_->publish(center_msg);
        }

        // === (I) 发布处理后的图像到 /image_processed ===
        auto out_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg();
        publisher_->publish(*out_msg);
    }

    // 维护 KNN 对象
    Ptr<KNearest> knn_;

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr publisher_center_;  // 新增的发布器

};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImageProcessor>());
    rclcpp::shutdown();
    return 0;
}