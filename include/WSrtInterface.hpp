// Copyright (c) 2021 Vilota Pte Ltd
// Author: Huimin C.

#ifndef WSRTINTERFACE_HPP_
#define WSRTINTERFACE_HPP_

#include <string>
#include <vector>

#include "srt_receiver/SrtReceiverFactory.hpp"
#include "srt_receiver/SrtReceiverInterface.hpp"

#include "websocket_client/WebsocketClientFactory.hpp"
#include "websocket_client/WebsocketClientInterface.hpp"

class WSrt {
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

        void extract_data();
        void reset_msgs();
        void reset_receivers();

};

#endif          // WSRTINTERFACE_HPP_