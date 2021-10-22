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

void wsrt_signal_handler(int signal) {
    spdlog::info("quiting signal received");

    if (srtReceiver_)
    {
        spdlog::info("shutdown srt receiver");
        srtReceiver_->stop();
    }
}

WSrt::WSrt(int cam_index, std::string srtAddress, std::string websocketAddress, std::string decoder) {
    cam_index_ = cam_index;
    srtAddress_ = srtAddress;
    websocketAddress_ = websocketAddress;
    decoder_ = decoder;
}

void WSrt::extract_data(vilota::SrtReceiverInterface::MessageQueue &queueSrt,
                  vilota::WebsocketClientInterface::MessageQueue &queueWebsocket) {
    vilota::SrtMessage::Ptr msgSrt;
    vilota::WebsocketMessage::Ptr msgWebsocket;

    cv::namedWindow("Client Visualisation", cv::WINDOW_AUTOSIZE);

    while (true) {
        queueSrt.pop(msgSrt);
        // pop until the latest message
        while (queueSrt.try_pop(msgSrt))
            ;
        // nullptr indicating end of stream
        if (!msgSrt)
            return;
        spdlog::info("got srt message!");

        if (queueWebsocket.try_pop(msgWebsocket)) {
            spdlog::info("got websocket message!");
            while (queueWebsocket.try_pop(msgWebsocket))
            ;
        }

        if (msgSrt->hasImage) {
            unsigned char *rawData;
            int rawDataSize;
            msgSrt->getData(rawData, rawDataSize);

            spdlog::debug("image timestamp {}, size {}", msgSrt->imageTimestamp, rawDataSize);

            cv::Mat cvImage(cv::Size(msgSrt->imageWidth, msgSrt->imageHeight), CV_8UC3, rawData, cv::Mat::AUTO_STEP);

            // TODO: no time sync features implemented yet
            if (msgWebsocket && msgWebsocket->hasJson) {

                auto &json = msgWebsocket->json["objects"];
                spdlog::debug("json: {}", json.dump());
                spdlog::debug(json.size());
                for (auto &object : json) {
                    spdlog::info("object: {}", object.dump());
                    int x = object["x"];
                    int y = object["y"];
                    int w = object["w"];
                    int h = object["h"];

                    cv::rectangle(cvImage, {x, y}, {x + w, y + h}, 150, 3);
                }
            }

            cv::imshow("Client Visualisation", cvImage);
            cv::waitKey(1);
        }
    }
}

void WSrt::WSrt_handler() {
    // NOTE: this seems needed, and contains glib main loop logics
    gst_init(nullptr, nullptr);

    //// srt
    auto srtFactory = vilota::SrtReceiverFactory();
    srtReceiver_ = srtFactory.getReceiver(srtAddress_, decoder_);
    //srtReceiver_ = srtFactory.getReceiver("srt://192.168.1.146:8888", "avdec_h264");

    // start the gst pipeline
    srtReceiver_->start();

    //// websocket
    auto websocketFactory = vilota::WebsocketClientFactory();
    websocketClient_ = websocketFactory.getClient(websocketAddress_);
    // websocketClient_ = websocketFactory.getClient("http://192.168.1.146:49999");
    // websocket does not need start manually, it is tagged to the glib main loop

    // Facilitate graceful Ctrl+C exit
    // Todo: Signals cannot call member functions, should we still include this?
    // std::signal(SIGINT, signal_handler);

    extract_data(std::ref(srtReceiver_->receiverMessageQueue),
                 std::ref(websocketClient_->clientMessageQueue));

    srtReceiver_.reset();
    websocketClient_.reset();

    spdlog::info("main exit");
}