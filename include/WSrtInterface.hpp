// Copyright (c) 2021 Vilota Pte Ltd
// Author: Huimin C.

#ifndef WSRTINTERFACE_HPP_
#define WSRTINTERFACE_HPP_

#include <string>

#include "srt_receiver/SrtReceiverFactory.hpp"
#include "srt_receiver/SrtReceiverInterface.hpp"

#include "websocket_client/WebsocketClientFactory.hpp"
#include "websocket_client/WebsocketClientInterface.hpp"

class WSrt {
    public:
        WSrt(
            int cam_index,
            std::string srtAddress,
            std::string websocketAddress,
            std::string decoder);

    virtual ~WSrt() {}

        int cam_index_;
        std::string srtAddress_, websocketAddress_, decoder_;
        vilota::SrtReceiverInterface::Ptr srtReceiver_;
        vilota::WebsocketClientInterface::Ptr websocketClient_;

        void extract_data(vilota::SrtReceiverInterface::MessageQueue &queueSrt,
                  vilota::WebsocketClientInterface::MessageQueue &queueWebsocket);
        void WSrt_handler();

};

#endif          // WSRTINTERFACE_HPP_