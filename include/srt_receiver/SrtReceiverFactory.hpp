// Copyright (c) 2021 Vilota Pte Ltd
// Author: Huimin C.
#pragma once

#include "SrtReceiverInterface.hpp"

namespace vilota
{
    class SrtReceiverFactory
    {
    public:
        // the factory is responsible to return an instance that is accessed using the defined interface
        // should start with `srt:://`
        SrtReceiverInterface::Ptr getReceiver(std::string uri, std::string h264Decoder = "vaapih264dec");
    };

} //namespace