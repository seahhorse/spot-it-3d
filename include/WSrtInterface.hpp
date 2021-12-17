// Copyright (c) 2021 Vilota Pte Ltd
// Author: Huimin C.

#ifndef WSRTINTERFACE_HPP_
#define WSRTINTERFACE_HPP_

#include <queue>
#include <string>
#include <vector>

#include <opencv2/core.hpp>

#include "srt_receiver/SrtReceiverFactory.hpp"
#include "srt_receiver/SrtReceiverInterface.hpp"

#include "websocket_client/WebsocketClientFactory.hpp"
#include "websocket_client/WebsocketClientInterface.hpp"

namespace mcmt {

// Detections from edge cam
struct wsrt_detections {
    int x, y, width, height;
};

// Output structure containing image data and detections
struct wsrt_output {
    std::vector<wsrt_detections> detections;
    int imageWidth, imageHeight;
    std::string imagePixelFormat;
    uint64_t imageTimestamp;
    cv::Mat image;
    bool delay_required;
};

// Structure to temporarily store detections to be placed in a waiting queue
struct saved_detection {
    std::vector<wsrt_detections> detections;
    uint64_t timestamp;
};

/**
 * The Wsrt client class to obtain data from edge cam and pass it to the main code
 * An instance of this class is spawned for each edge cam
 */
class WSrt {
    private:
        std::queue<saved_detection> detection_queue_; // detection waiting queue, used for time syncing
    public:
        WSrt(
            int cam_index,
            std::string cam_ip,
            std::string srtPort,
            std::string websocketPort,
            std::string decoder);

    virtual ~WSrt() {}

        int cam_index_;
        std::string srtAddress_, websocketAddress_, decoder_;

        vilota::SrtReceiverFactory srtFactory_;
        vilota::SrtReceiverInterface::Ptr srtReceiver_;
        vilota::WebsocketClientFactory websocketFactory_;
        vilota::WebsocketClientInterface::Ptr websocketClient_;
        vilota::SrtMessage::Ptr msgSrt_;
        vilota::WebsocketMessage::Ptr msgWebsocket_;

        wsrt_output extract_data();
        wsrt_output extract_data_callback(vilota::SrtReceiverInterface::MessageQueue &queueSrt,
                  vilota::WebsocketClientInterface::MessageQueue &queueWebsocket);
        void reset_msgs();
        void reset_receivers();

};

}

#endif          // WSRTINTERFACE_HPP_