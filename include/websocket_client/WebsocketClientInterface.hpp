// Copyright (c) 2021 Vilota Pte Ltd
// Author: Huimin C.
#pragma once

#include <memory>
#include <tbb/concurrent_queue.h>

#include <nlohmann/json.hpp>

namespace vilota
{
    struct WebsocketMessage
    {
        typedef std::shared_ptr<WebsocketMessage> Ptr;
        bool hasJson = false;
        nlohmann::json json;
    };

    class WebsocketClientInterface
    {

    public:
        typedef tbb::concurrent_bounded_queue<WebsocketMessage::Ptr> MessageQueue;
        typedef std::shared_ptr<WebsocketClientInterface> Ptr;

        MessageQueue clientMessageQueue;

        // push one last message with nullptr to indicate end of queue and closing
        void endQueue()
        {
            clientMessageQueue.push(nullptr);
        }
    };
} //namespace