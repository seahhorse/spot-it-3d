// Copyright (c) 2021 Vilota Pte Ltd
// Author: Huimin C.
#pragma once

#include <memory>
#include <tbb/concurrent_queue.h>
// #include <opencv2/core.hpp>

namespace vilota
{
    struct SrtMessage
    {
        typedef std::shared_ptr<SrtMessage> Ptr;

        // image info
        bool hasImage = false;

        int imageWidth, imageHeight;
        std::string imagePixelFormat;
        uint64_t imageTimestamp;

        void reset(unsigned char *data, int size);
        void getData(unsigned char *&data, int &size);

        // deallocation called by share pointer
        ~SrtMessage();

    private:
        unsigned char *rawData = nullptr;
        int rawDataSize = -1; // in bytes
    };

    class SrtReceiverInterface
    {

    public:
        typedef tbb::concurrent_bounded_queue<SrtMessage::Ptr> MessageQueue;

        typedef std::shared_ptr<SrtReceiverInterface> Ptr;

        MessageQueue receiverMessageQueue;

        virtual void enablePreviewBin(bool enabled) = 0;

        virtual void start() = 0;

        virtual void stop() = 0;

        virtual ~SrtReceiverInterface() = default;

    protected:
        // push one last message with nullptr to indicate end of queue and closing
        void endQueue()
        {
            receiverMessageQueue.push(nullptr);
        }
    };

} // namespace vilota
