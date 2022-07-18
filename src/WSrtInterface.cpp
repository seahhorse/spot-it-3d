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

// this macro is enabled directly in CMakelists.txt
#ifdef WSRT_ENABLED

#include <thread>
#include <chrono>
#include <queue>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <spdlog/spdlog.h>

#include <csignal>

#include <gst/gst.h>

namespace mcmt {

/**
 * WSrt client constructor. Intializes the srt and websocket clients
 */
WSrt::WSrt(int cam_index, std::string cam_ip, std::string srtPort, std::string websocketPort, std::string decoder) {
    cam_index_ = cam_index;
    srtAddress_ = "srt://" + cam_ip + ":" + srtPort;
    websocketAddress_ = "http://" + cam_ip + ":" + websocketPort;
    decoder_ = decoder;
    detection_delays_ = 0;

    // NOTE: this seems needed, and contains glib main loop logics
    gst_init(nullptr, nullptr);

    // srt
    srtReceiver_ = srtFactory_.getReceiver(srtAddress_, decoder_);
    srtReceiver_->start();

    // websocket
    // websocket does not need start manually, it is tagged to the glib main loop
    websocketClient_ = websocketFactory_.getClient(websocketAddress_);
}

/**
 * Calls the main handler function and passes in the srt and websocket concurrent queues
 * For some reason, the queues must be passed as std::ref intead of being declared
 * directly within the main handler function, otherwise it will not receive data
 */
wsrt_output WSrt::extract_data() {
    wsrt_output output = extract_data_callback(std::ref(srtReceiver_->receiverMessageQueue),
                 std::ref(websocketClient_->clientMessageQueue));
    return output;
}

/**
 * Main function to obtain image and detections from edge cam
 * and return them in a wsrt_output structure
 */
wsrt_output WSrt::extract_data_callback(vilota::SrtReceiverInterface::MessageQueue &queueSrt,
                  vilota::WebsocketClientInterface::MessageQueue &queueWebsocket) {

    wsrt_output output;
    reset_msgs();

    queueSrt.pop(msgSrt_);
    
    // pop until the latest message
    while (queueSrt.try_pop(msgSrt_))
    ;
    // nullptr indicating end of stream
    if (!msgSrt_)
        return output;
    spdlog::info("got srt message!");

    while (queueWebsocket.try_pop(msgWebsocket_)) {
        spdlog::info("got websocket message!");
        // Save the detections and their timestamps in the waiting queue
        auto &json = msgWebsocket_->json["objects"];
        saved_detection saved;
        saved.timestamp = msgWebsocket_->json["timestamp-buffer-pts"];
        spdlog::info("detection timestamp inserted to queue: {}", saved.timestamp);
        for (auto &object : json) {
            wsrt_detections detections;
            detections.x = object["x"];
            detections.y = object["y"];
            detections.width = object["w"];
            detections.height = object["h"];
            saved.detections.push_back(detections);
        }
        detection_queue_.push(saved);
        // while (queueWebsocket.try_pop(msgWebsocket_))
        // ;
    }

    if (msgSrt_->hasImage) {
        unsigned char *rawData;
        int rawDataSize;
        msgSrt_->getData(rawData, rawDataSize);

        spdlog::info("image timestamp {}, size {}", msgSrt_->imageTimestamp, rawDataSize);

        cv::Mat cvImage(cv::Size(msgSrt_->imageWidth, msgSrt_->imageHeight), CV_8UC3, rawData, cv::Mat::AUTO_STEP);

        output.image = cvImage;
        output.imageWidth = msgSrt_->imageWidth;
        output.imageHeight = msgSrt_->imageHeight;
        output.imageTimestamp = msgSrt_->imageTimestamp;
        output.imagePixelFormat = msgSrt_->imagePixelFormat;
        output.delay_required = false;

        // compare SRT image and websocket detection timestamps to make sure they are time-synced
        // detection_queue_ stores pending detections from websocket that are not synced with an image
        while (!detection_queue_.empty()) {
            auto earliest_detection = detection_queue_.front();
            uint64_t detection_timestamp = earliest_detection.timestamp;

            // if detection is later than image, release image without detections
            // let the main loop know that delay is required
            if (detection_timestamp > msgSrt_->imageTimestamp) {
                spdlog::warn("Detection is later than image, delaying release of detection");
                // spdlog::warn("detection timestamp {}", detection_timestamp);
                output.delay_required = true;
                break;
            }
            else if (detection_timestamp == msgSrt_->imageTimestamp) {
                spdlog::info("Detection and Image timestamps in sync");
                for (auto &object : earliest_detection.detections) {
                    wsrt_detections detections;
                    detections.x = object.x;
                    detections.y = object.y;
                    detections.width = object.width;
                    detections.height = object.height;
                    output.detections.push_back(detections);
                }
                detection_queue_.pop();
                output.delay_required = false;
                break;
            }
            else {
                spdlog::warn("Detection is earlier than image, dropping detection");
                // spdlog::warn("detection timestamp {}", detection_timestamp);
                detection_queue_.pop();
                output.delay_required = true;
            }
        }
    }
    return output;
}

/**
 * Reset Srt and Websocket msgs for next iteration
 */
void WSrt::reset_msgs() {
    msgSrt_.reset();
    msgWebsocket_.reset();
}

/**
 * Reset Srt and Websocket receivers before terminating
 */
void WSrt::reset_receivers() {
    srtReceiver_.reset();
    websocketClient_.reset();
}

}

#endif // WSRT_ENABLED