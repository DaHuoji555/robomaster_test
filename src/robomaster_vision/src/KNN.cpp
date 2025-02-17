#include "../include/robomaster_vision/KNN.h"
#include "rclcpp/rclcpp.hpp" 


// Function to load images and labels
void loadDataset(const string& datasetPath, Mat& features, Mat& labels) {
    vector<Mat> images;
    vector<int> labelVec;

    RCLCPP_INFO(rclcpp::get_logger("KNN"), "Checking dataset path: %s", datasetPath.c_str());

    for (const auto& entry : fs::directory_iterator(datasetPath)) {
        string filename = entry.path().filename().string();

        // Check for valid image file and parse label
        if (filename.find(".png") != string::npos) {
            Mat img = imread(entry.path().string(), IMREAD_GRAYSCALE);

            if (img.empty()) {
                RCLCPP_WARN(rclcpp::get_logger("KNN"), "Failed to load image: %s", entry.path().string().c_str());
                continue;
            }

            if (img.size() != Size(200, 200)) {
                RCLCPP_WARN(rclcpp::get_logger("KNN"), "Invalid image size: %s, expected 200x200, got %d x %d",
                            entry.path().string().c_str(), img.cols, img.rows);
                continue;
            }

            images.push_back(img);

            // Parse label from filename
            if (filename.find("neg") != string::npos) {
                labelVec.push_back(10);  // Assign label 10 for negative samples
            } else {
                int label = stoi(filename.substr(0, filename.find('_')));
                labelVec.push_back(label);
            }
        }
    }

    RCLCPP_INFO(rclcpp::get_logger("KNN"), "Loaded %ld valid images.", images.size());

    // Convert images to feature matrix
    features = Mat(images.size(), 200 * 200, CV_32F);
    for (size_t i = 0; i < images.size(); i++) {
        Mat flattened = images[i].reshape(1, 1);
        flattened.convertTo(features.row(static_cast<int>(i)), CV_32F);
    }

    // Convert labels to Mat
    labels = Mat(labelVec.size(), 1, CV_32S);
    for (size_t i = 0; i < labelVec.size(); i++) {
        labels.at<int>(i, 0) = labelVec[i];
    }

    if (features.empty() || labels.empty()) {
        RCLCPP_ERROR(rclcpp::get_logger("KNN"), "❌ Error: No valid images found in %s", datasetPath.c_str());
    }
}


// Train KNN model
Ptr<KNearest> trainKNNFromDataset(const string& datasetPath) {
    Mat features, labels;

    // Load dataset
    loadDataset(datasetPath, features, labels);

    // Train KNN
    Ptr<KNearest> knn = KNearest::create();
    knn->setDefaultK(3);
    knn->setIsClassifier(true);
    knn->train(features, ROW_SAMPLE, labels);

    return knn;
}

// Predict label using KNN
int predictLabel(const Ptr<KNearest>& knn, const Mat& inputBlock) {
    Mat inputFlattened = inputBlock.reshape(1, 1);
    inputFlattened.convertTo(inputFlattened, CV_32F);

    Mat results;
    float response = knn->findNearest(inputFlattened, knn->getDefaultK(), results);
    return static_cast<int>(response);
}