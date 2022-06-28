/**
 * @file yolo.hpp
 * Modified from yolov5-opencv-cpp-python by doleron
 * https://github.com/doleron/yolov5-opencv-cpp-python/blob/main/cpp/yolo.cpp
 * 
 * Code to run Yolov5 detection algorithm on C++
 * Used for detection of large objects, where blob detection is not effective
*/

#ifndef YOLO_HPP_
#define YOLO_HPP_

#include <string>
#include <vector>

#include <opencv2/opencv.hpp>

namespace mcmt {

// Yolov5 detected objects
struct yolo_object
{
    int class_id;
    float confidence;
    cv::Rect box;
};

class YoloDetector{
    private:
        float input_width, input_height, score_thres, nms_thres, confidence_thres;
        std::vector<std::string> class_list;
        cv::dnn::Net net;

    public:
        YoloDetector();
        virtual ~YoloDetector() {}

        std::vector<std::string> load_class_list();
        void load_net(bool is_cuda);
        cv::Mat format_yolov5(cv::Mat &source);
        std::vector<yolo_object> detect(cv::Mat &image);

};

}

#endif          // YOLO_HPP_