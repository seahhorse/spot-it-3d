/**
 * @file WSrtInterface.cpp
 * Copyright (c) 2021 Vilota Pte Ltd
 * @author Huimin C.
 * Modifications by:
 * @author Dr Sutthiphong Srigrarom (Spot), spot.srigrarom@nus.edu.sg
 * @author Mr Niven Sie, sieniven@gmail.com
 * @author Mr Seah Shao Xuan, seahshaoxuan@gmail.com
 * @author Mr Lau Yan Han, sps08.lauyanhan@gmail.com
 * 
 * This code is conceptualised, created and published by the SPOT-IT 3D team
 * from the Department of Mechanical Engineering, Faculty of Engineering 
 * at the National University of Singapore. SPOT-IT 3D refers to the 
 * Simultaneous Positioning, Observing, Tracking, Identifying Targets in 3D.
 * This software utilizes a multi-camera surveillance system for real-time 
 * multiple target tracking capabilities. This software capability is highly
 * applicable for monitoring specific areas, and some use cases include monitoring 
 * airspaces, traffic junctions, etc.
 * 
 * This file is part of the SPOT-IT 3D repository and can be downloaded at:
 * https://github.com/sieniven/spot-it-3d
 * 
 * This file contains modified interface with Vilota Websocket and SRT (WSrt)
 * to receive images and detected blobs from Vilota Edge Cameras in real time.
 */

#include "WSrtInterface.hpp"

#include <thread>
#include <chrono>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <spdlog/spdlog.h>

#include <csignal>

#include <gst/gst.h>

namespace mcmt {

WSrt::WSrt(int cam_index, std::string cam_ip, std::string srtPort, std::string websocketPort, std::string decoder) {
    cam_index_ = cam_index;
    srtAddress_ = "srt://" + cam_ip + ":" + srtPort;
    websocketAddress_ = "http://" + cam_ip + ":" + websocketPort;
    decoder_ = decoder;

    // NOTE: this seems needed, and contains glib main loop logics
    gst_init(nullptr, nullptr);

    // srt
    srtReceiver_ = srtFactory_.getReceiver(srtAddress_, decoder_);
    srtReceiver_->start();

    // websocket
    // websocket does not need start manually, it is tagged to the glib main loop
    websocketClient_ = websocketFactory_.getClient(websocketAddress_);
}

wsrt_output WSrt::extract_data() {

    wsrt_output output;
    
    auto queueSrt = srtReceiver_->receiverMessageQueue;
    auto queueWebsocket = websocketClient_->clientMessageQueue;
    reset_msgs();
    
    // cv::namedWindow("Client Visualisation", cv::WINDOW_AUTOSIZE);

    queueSrt.pop(msgSrt_);
    // pop until the latest message
    while (queueSrt.try_pop(msgSrt_))
    ;
    // nullptr indicating end of stream
    if (!msgSrt_)
        return output;
    spdlog::info("got srt message!");

    if (queueWebsocket.try_pop(msgWebsocket_)) {
        spdlog::info("got websocket message!");
        while (queueWebsocket.try_pop(msgWebsocket_))
        ;
    }

    if (msgSrt_->hasImage) {
        unsigned char *rawData;
        int rawDataSize;
        msgSrt_->getData(rawData, rawDataSize);

        spdlog::debug("image timestamp {}, size {}", msgSrt_->imageTimestamp, rawDataSize);

        cv::Mat cvImage(cv::Size(msgSrt_->imageWidth, msgSrt_->imageHeight), CV_8UC3, rawData, cv::Mat::AUTO_STEP);

        output.image = cvImage;
        output.imageWidth = msgSrt_->imageWidth;
        output.imageHeight = msgSrt_->imageHeight;
        output.imageTimestamp = msgSrt_->imageTimestamp;
        output.imagePixelFormat = msgSrt_->imagePixelFormat;

        // TODO: no time sync features implemented yet
        if (msgWebsocket_ && msgWebsocket_->hasJson) {

            auto &json = msgWebsocket_->json["objects"];
            spdlog::debug("json: {}", json.dump());
            spdlog::debug(json.size());
            for (auto &object : json) {
                wsrt_detections detections;
                detections.x = object["x"];
                detections.y = object["y"];
                detections.width = object["w"];
                detections.height = object["h"];
                output.detections.push_back(detections);
                
                // Draw rectangles for debugging
                // spdlog::info("object: {}", object.dump());
                // int x = object["x"];
                // int y = object["y"];
                // int w = object["w"];
                // int h = object["h"];
                // cv::rectangle(cvImage, {x, y}, {x + w, y + h}, 150, 3);
            }
        }
        // cv::imshow("Client Visualisation", cvImage);
    }
    return output;
}

void WSrt::reset_msgs() {
    msgSrt_.reset();
    msgWebsocket_.reset();
}

void WSrt::reset_receivers() {
    srtReceiver_.reset();
    websocketClient_.reset();
}

}