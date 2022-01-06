# Vilota Wsrt Edge Camera Guide

This guide shows how to set up and operate the Vilota Edge Camera and its corresponding WSrt client code, along with the Spot-it-3d code

The Wsrt code consists of two sockets: (1) SRT socket that transmits images, and (2) Websocket that transmits detections in a JSON format. These sockets transmit information from the edge cameras to the client. Each edge camera consists of a camera with an attached single board computer (e.g. Raspberry Pi) that shall henceforth be called the edge computer. The edge computer contains the server code. The client laptop running the Spotit-it-3d code will contain the client code (`Wsrtinterface.cpp`)

## Requirements

The client laptop should be on Ubuntu 20.04 LTS and later

## Dependencies Installation

The Wsrt client package comes with several dependencies

### Gstreamer
```bash
sudo apt install gstreamer1.0-tools gstreamer1.0-plugins-good gstreamer1.0-plugins-ugly gstreamer1.0-plugins-bad gstreamer1.0-plugins-base-apps

sudo apt install libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev libgstreamer-plugins-good1.0-dev

sudo apt install gstreamer1.0-vaapi

sudo apt install gstreamer1.0-libav

# optional
sudo apt install  libgstreamer-plugins-bad1.0-dev 
```

### Build System
```bash
sudo apt install cmake pkg-config
```

### Logging

```bash
sudo apt install libspdlog-dev
```

### Soup

```bash
sudo apt install libsoup2.4-dev
```

### Multi-threading

```bash
sudo apt install libtbb-dev
```

### OpenCV

```bash
sudo apt install libopencv-dev
```

## Running the Code

### Setup

The client laptop and the edge computers should be connected within the same network, either over Ethernet or Wifi

### Initialise server (edge camera)

SSH into the edge computers' containers containing the server code. Ensure that the client laptop's public key is added to the edge computers so that it can SSH without requiring a password

ssh command:

```bash
ssh ubuntu@vilota-<last 4 digits of edge computer ip address>.local

# Example if the edge computer's MAC address ends in 1234:
ssh ubuntu@vilota-1234.local
```

All startup scripts for the edge camera are contained in the "examples" folder

- `camera_setup.sh` - use this to tune camera exposure time and other settings. Change the script command before running it to change the params
- Run the `v4l2-480/720p` bash scripts to initialise the server side

### Initialise client

Go to the parameters file of the Spot-it-3d code and change the following parameters

- **IS_REALTIME_**: Set to 2
- **VIDEO_INPUT_**: Set to the IP Addresses of the edge cams

Then compile with cmake and run the spot-it-3d executable as usual

### Troubleshooting

There is an existing issue where, if the client side is disconnected, both the server and client code must be restarted in order to reconnect

We can add params directly into the edge computer's 480/720p bash scripts to change the behaviour of the plugins (bg subtractor, blob detection). To see what params can be tuned, use `gst-inspect1.0 command + plugin name`

For other forms of debugging, we can ssh directly into the edge computers' hosts with the provided password: 

```bash
ssh vilota@vilota-desktop-<last 4 digits of edge computer ip address>.local

# Example if the edge computer's MAC address ends in 1234:
ssh vilota@vilota-desktop-1234.local
```

Before disconnecting power to the edge computers, it is good practice to do a proper shutdown by sending `sudo shutdown` commands to each edge computer (through ssh)

## Old Build Instructions (deprecated)

These are the build instructions for the original Wsrt code, which was compiled with Meson/Ninja. The current system is built with Cmake.

### Installing Build System

```bash
sudo apt install python3-pip python-is-python3
pip install meson
sudo apt install cmake ninja-build pkg-config
```

### Building Project

```bash
cd wsrt-client-shipping

# create build folder and configure the build, check for dependencies
meson build
# run the actual building process
ninja -C build
```