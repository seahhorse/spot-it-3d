/**
 * @file yolo.cpp
 * Modified from yolov5-opencv-cpp-python by doleron
 * https://github.com/doleron/yolov5-opencv-cpp-python/blob/main/cpp/yolo.cpp
 * 
 * Code to run Yolov5 detection algorithm on C++
 * Used for detection of large objects, where blob detection is not effective
*/

#include "yolo.hpp"

#include <string>
#include <vector>

#include <fstream>

#include <opencv2/opencv.hpp>

namespace mcmt {

YoloDetector::YoloDetector(){
    input_width = 640.0;
    input_height = 640.0;
    score_thres = 0.2;
    nms_thres = 0.4;
    confidence_thres = 0.4;
    class_list = load_class_list();
    load_net(0);
}

/**
 * Load Yolo v5 default classification list
 */
std::vector<std::string> YoloDetector::load_class_list() {
    std::vector<std::string> class_list;
    std::ifstream ifs("yolo_config/classes.txt");
    std::string line;
    while (getline(ifs, line)) {
        class_list.push_back(line);
    }
    return class_list;
}

/**
 * Load Yolov5 network
 */
void YoloDetector::load_net(bool is_cuda) {
    auto result = cv::dnn::readNet("yolo_config/yolov5s.onnx");
    if (is_cuda)
    {
        result.setPreferableBackend(cv::dnn::DNN_BACKEND_CUDA);
        result.setPreferableTarget(cv::dnn::DNN_TARGET_CUDA_FP16);
    }
    else
    {
        result.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
        result.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);
    }
    net = result;
}

/**
 * Format image to meet Yolov5 requirements
 * Yolov5 requires an input image in RGB format, pixel values in [0,1], and size 640x640
 */
cv::Mat YoloDetector::format_yolov5(cv::Mat &source) {
    int col = source.cols;
    int row = source.rows;
    int _max = MAX(col, row);
    cv::Mat result = cv::Mat::zeros(_max, _max, CV_8UC3);
    source.copyTo(result(cv::Rect(0, 0, col, row)));
    return result;
}

/**
 * Performn object detection using the "net" and "classname"
 * Detections are stored in "output"
 */
std::vector<yolo_object> YoloDetector::detect(cv::Mat &image) {
    cv::Mat blob;

    auto input_image = format_yolov5(image);
    
    cv::dnn::blobFromImage(input_image, blob, 1./255., cv::Size(input_width, input_height), cv::Scalar(), true, false);
    net.setInput(blob);
    std::vector<cv::Mat> outputs;
    net.forward(outputs, net.getUnconnectedOutLayersNames());

    float x_factor = input_image.cols / input_width;
    float y_factor = input_image.rows / input_height;
    
    float *data = (float *)outputs[0].data;

    int dimensions = 85;
    int rows = 25200;
    
    std::vector<int> class_ids;
    std::vector<float> confidences;
    std::vector<cv::Rect> boxes;

    for (int i = 0; i < rows; ++i) {

        float confidence = data[4];
        if (confidence >= confidence_thres) {

            float * classes_scores = data + 5;
            cv::Mat scores(1, class_list.size(), CV_32FC1, classes_scores);
            cv::Point class_id;
            double max_class_score;
            minMaxLoc(scores, 0, &max_class_score, 0, &class_id);
            if (max_class_score > score_thres) {

                confidences.push_back(confidence);

                class_ids.push_back(class_id.x);

                float x = data[0];
                float y = data[1];
                float w = data[2];
                float h = data[3];
                int left = int((x - 0.5 * w) * x_factor);
                int top = int((y - 0.5 * h) * y_factor);
                int width = int(w * x_factor);
                int height = int(h * y_factor);
                boxes.push_back(cv::Rect(left, top, width, height));
            }

        }

        data += 85;

    }

    std::vector<int> nms_result;
    cv::dnn::NMSBoxes(boxes, confidences, score_thres, nms_thres, nms_result);

    std::vector<yolo_object> detections;
    for (int i = 0; i < nms_result.size(); i++) {
        int idx = nms_result[i];
        yolo_object result;
        result.class_id = class_ids[idx];
        result.confidence = confidences[idx];
        result.box = boxes[idx];
        detections.push_back(result);
    }

    return detections;
}

}