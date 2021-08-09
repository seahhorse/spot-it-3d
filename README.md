# Simultaneous Positioning, Observing, Tracking, Identifying Targets in 3D (SPOT-IT 3D)

## 1. Introduction

**SPOT-IT 3D** (Simultaneous Positioning, Observing, Tracking, Identifying Targets in 3D) is a project by Mechanical Engineering @ National University of Singapore. This project aims to develop a software that utilizes a multi-camera surveillance system for real-time multiple target tracking capabilities. This software capability is highly applicable for monitoring specific areas, and some use cases include monitoring airspaces, traffic junctions, etc.


## 2. Table of Contents

- [3. Publications](#3-publications)
- [4. Aim](#4-aim)
- [5. Capabilities](#5-capabilities)
- [6. Installation Guide](#6-installation-guide)
  * [6.1 Linux-based Operating Systems](#61-linux-based-operating-systems)
  * [6.2 Windows Operating Systems](#62-windows-operating-systems)
- [7. Acknowledgements](#9-acknowledgements)


## 3. Publications

![Software Demo](./docs/software_demo_1.gif)

1. Paper on trajectory-based target matching and re-identification between cameras:
	* Niven Sie Jun Liang and Sutthiphong Srigrarom. "Multi-camera multi-target tracking systems with trajectory-based target matching and re-identification." In *2021 IEEE International Conference on Unmanned Aerial Systems (ICUAS)*, IEEE, Athens, Greece, 2021.
	* Link to paper: (To be added into IEEE Xplore soon)

2. Paper on field test validations for using trajectory-based tracking with a multiple camera system for target tracking and 3-dimensional localization:
	* Niven Sie Jun Liang, Sutthiphong Srigrarom and Sunan Huang. "Field test validations of vision-based multi-camera multi-drone tracking and 3D localizing, using concurrent camera pose estimation." In *2021 IEEE 6th International Conference on Control and Robotics Engineering (ICCRE)*, IEEE, Beijing, China, 2021.
	* Link to paper: https://ieeexplore.ieee.org/abstract/document/9358454

3. Paper on state estimation filters and proposed use of implementing multiple state estimation filters in parrallel (Integrated Multiple Model):
	* Sutthiphong Srigrarom, Niven Sie Jun Liang, Jiahe Yi, Kim Hoe Chew, Floorian Holzapfel, Henrik Hesse, Teng Hooi Chan and Jalvin Jiaxiang Chen. "Vision-based drones tracking using correlation filters and Linear Integrated Multiple Model." In *2021 IEEE International Conference on Electrical Engineering/Electronics, Computer, Telecommunications and Information Technology (ECTI-CON)*, IEEE, Chiang Mai, Thailand, 2021.
	* Link to paper: (To be added into IEEE Xplore soon)

4. Paper on integrated kinematic-based detection tracking estimation system for dynamic localization of small aerial vehicles:
	* Sutthiphong Srigrarom, Shawndy Michael Lee, Mengda Lee, Foong Shaohui and Photchara Ratsamee. "An integrated vision-based detection-tracking-estimation system for dynamic localization of small aerial vehicles." In *2020 IEEE 5th International Conference on Control and Robotics Engineering (ICCRE)*, IEEE, Osaka, Japan, 2020.
	* Link to paper: https://ieeexplore.ieee.org/abstract/document/9096259

5. Paper on binocular and stereo cameras for multiple drone detection and 3-dimensional localization:
	* Yi, Jiahe, and Sutthiphong Srigrarom. "Near-Parallel Binocular-Like Camera Pair for Multi-Drone Detection and 3D Localization." In *2020 16th International Conference on Control, Automation, Robotics and Vision (ICARCV)*, pp. 204-210. IEEE, Shenzhen, China, 2020.
	* Link to paper: https://ieeexplore.ieee.org/abstract/document/9305485


## 4. Aim

This project aims to develop a methodology of identification, localization, and tracking of **small and fast moving targets**, such as flying drones, using an integrated multiple camera monitoring system. Our study focuses on using **motion-based** features to track targets, instead of traditional tracking algorithms that use appearance-based features by incorporating deep convolutional neural networks for target tracking. 

As we focus on small and fast moving targets, such as drones, using appearance-based features in this specific case may be difficult, as these targets often appear as small black "dots/blobs" in video frames. This would specifically mean that we use targets’ **trajectory features**, such as the derivatives of displacement, heading and turning angle, to identify and track targets instead.

Thus, our software incorporates the use of a multiple camera surveillance system to track and localize these moving targets. We aim to be able to continuously track drones in monitored airspaces, re-identify these targets between the cameras, and montor their 3-dimensional coordinates when they are inside the monitored space. 


## 5. Capabilities

SPOT-IT 3D can be thought of as being broadly comprising two main processes. They are the **detection and tracking process**, and the **re-identification and trackplot process**. The software is built entirely in C++ for high runtime performance capabilities.

The following is a general high-level overview of the main code pipeline:

1. Open and read from single / double camera sensors, and obtain live camera frames.

2. Apply image processing techniques to remove background noise, by distinguisihing between foreground and background. We apply background subtraction in our image processing pipeline, and subsequently identify contours that do not conform to a minimum level of circularity. We deem these contours as background, and subtract them out of our image frames.

3. Apply morphological operations such as dilation and erosion to remove noise and enhance our detection.

4. Apply thresholding and binarization of our frames to obtained masked frames to detect targets using blob detection.

5. Use of Kalman filtering (KF) and Discriminative Correlation Filter (DCF) as our state estimation techniques to predict our targets' next known location in the following frames. Both filters are implemented in our tracking pipeline, as these targets move at fast and erratic speeds, and using these filters allow us to better predict their positions for continuous tracking.

6. Implements the use of Hungarian / Munkre's algorithm to match our detections to tracks. The matching algorithm is based on a 2 dimensional cost matrix of tracks and detections, where the cost is computed by comparing every detection's euclidean distance away from each tracked target predicted location from our DCF and EKF filters. 

7. The software has re-identification capabilities of targets between cameras. This would mean that every camera will be able to know that they are tracking the same target, purely based on their kinematic features. We implement cross-correlation matching of tracks' trajectory features, and obtain a 2-dimensional correlation score between the cameras' tracks.

8. Apply graph matching algorithm for geomentry-based identification using relative coordinates. The software initializes a complete bipartite graph, that calculates the maximum sum of the weights of the edges that span across the two disjoint groups of the complete bipartite graph. Three distinct methods are used to calculate a similarity score and serve as graph edge weights in our re-identification process.

9. Estimation of targets' location in 3-dimensional space using triangulation and stereo camera methodology. We use disparities between the two camera frames to obtain depth estimations of the tracked targets. *In improving the accuracy of 3D coordinates, development is underway in using a simplified method adapted from Takagi, 2018.*


## 6. Installation Guide

The following step-by-step processing will guide you on the installation process. Our software runs on a Linux environment. It is assumed that the Linux environment is pre-installed with GCC (GNU Compiler Collection). GCC comes with native installations of Ubuntu. If GCC is not yet installed, please follow the instructions at https://www.ubuntupit.com/how-to-install-and-use-gcc-compiler-on-linux-system/ or any GCC installation tutorial.

### 6.1 Linux-based Operating Systems

1. Pull spot-it-3d repository from GitHub.

	``` bash
	get clone https://github.com/sieniven/spot-it-3d.git
	```

2. Navigate to the spot-it-3d parameters file and open it with a text editor of choice.

	``` bash
	nano spot-it-3d/include/multi_cam_params.hpp
	```

3. 	Replace the parameters as shown below to your specific setup.

	``` bash
	// declare filepaths
	const bool IS_REALTIME_ = // true if realtime processing, false otherwise ;
    const int NUM_OF_CAMERAS_ = // 1 for single camera processing, 2 for double camera processing;
    const std::string VIDEO_INPUT_1_ = // "#" camera number for realtime processing, "data/input/filename.extension" for non-realtime processing ;
    const std::string VIDEO_INPUT_2_ = // "#" camera number for realtime processing, "data/input/filename.extension" for non-realtime processing ;
	const std::string VIDEO_OUTPUT_1_ = // "data/output/filename.extension" (single camera output, only saves if realtime processing);
    const std::string VIDEO_OUTPUT_2_ = // "data/output/filename.extension" (single camera output, only saves if realtime processing);
	const std::string VIDEO_OUTPUT_ANNOTATED_ = // "data/output/filename.extension" (combined output file, annotated with tracks);
	const std::string TARGETS_2D_OUTPUT_ = "data/output/json_filename.json";
    const std::string TARGETS_3D_OUTPUT_ = "data/output/json_filename.json";
    const std::string FRAME_TIME_ = "data/output/csv_filename.csv";
    
	// declare video parameters
	const int FRAME_WIDTH_ = // specify camera\'s target width;
    const int FRAME_HEIGHT_ = // specify camera\'s target height;
    const int VIDEO_FPS_ = // specify camera\'s target FPS;
	```

	Take special note that each camera is pre-built with a set of (resolution, fps) settings. To check all possible combinations, run:

	```bash
	v4l2-ctl -d /dev/video# --list-formats-ext
	```
	with # replaced with your camera device number.


4. Run the bash script. You will be prompted to install OpenCV 4 if the system does not have the required version.

	``` bash
	bash compile.sh
	```
### 6.2 Windows Operating Systems

1. Navigate to the Windows Store and install WSL2 (Windows Subsystem for Linux 2). Detailed instructions can be found at https://docs.microsoft.com/en-us/windows/wsl/install-win10.

2. Download and install XMing X Server for Windows. Detailed instructions can be found at http://www.straightrunning.com/XmingNotes/.

3. Launch XLaunch and select the following settings:

	Multiple Windows
	Display Number: -1
	Start no client
	
	Select all checkbox options for "Extra Settings", ensuring that "Disable access control" is checked.

	*For ease of starting up in subsequent runs, you may wish to save these settings by clicking on the Save Configuration button.*

	If the client was launched successfully, you should see a mini-icon in the sidebar at the bottom right-hand corner of your screen.

4. Open up a WSL2 terminal and follow the instructions in **6.1 Linux-based Operating Systems**.

## 7. Acknowledgements

We would like to thank the lead researcher in this project, Dr. Sutthiphong Srigrarom, for his continuous guidance and supervision with the development of this project. We would also like to acknowledge the hard work the team in playing a part in developing this software. Our research team comprises:

1. Dr. Sutthiphong Srigrarom (email: spot.srigrarom@nus.edu.sg, GitHub profile: spotkrub)
2. Niven Sie Jun Liang (email: sieniven@gmail.com, GitHub profile: https://github.com/sieniven)
3. Seah Shao Xuan (email: shaoxuan.seah@gmail.com, GitHub profile: https://github.com/seahhorse)
4. Lau Yan Han (email: sps08.lauyanhan@gmail.com, GitHub profile: https://github.com/disgruntled-patzer)