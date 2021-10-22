// Copyright (c) 2021 Vilota Pte Ltd
// Author: Huimin C.
#pragma once

#include "WebsocketClientInterface.hpp"

namespace vilota
{
    class WebsocketClientFactory
    {
    public:
        // the factory is responsible to return an instance that is accessed using the defined interface
        WebsocketClientInterface::Ptr getClient(std::string uri);
    };

} //namespace