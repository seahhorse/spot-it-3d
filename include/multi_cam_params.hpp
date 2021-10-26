/**
 * @file multi_cam_params.hpp
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
 * This file contains the common user-tuned parameters which are used for the
 * detection, tracking, re-identification process, alongside the user interface
 * settings for the display window. 
 */

#ifndef MULTI_CAM_PARAMS_HPP_
#define MULTI_CAM_PARAMS_HPP_

// opencv header files
#include "opencv2/opencv.hpp"

// standard package imports
#include <string>
#include <vector>
#include <set>
#include <fstream>

namespace mcmt {

	// declare filepaths
	const bool IS_REALTIME_ = false;
    const int NUM_OF_CAMERAS_ = 1;
    const std::string VIDEO_INPUT_1_ = "192.168.1.141";
    const std::string VIDEO_INPUT_2_ = "192.168.1.146";
	const std::string VIDEO_OUTPUT_1_ = "data/output/A_out.avi";
    const std::string VIDEO_OUTPUT_2_ = "data/output/B_out.avi";
	const std::string VIDEO_OUTPUT_ANNOTATED_ = "data/output/test.avi";
	const std::string TARGETS_2D_OUTPUT_ = "data/output/targets_2d_out.json";
    const std::string TARGETS_3D_OUTPUT_ = "data/output/targets_3d_out.json";
    const std::string FRAME_TIME_ = "data/output/frame_time.csv";
    const std::string VILOTA_SRT_PORT_ = "8888";
    const std::string VILOTA_WEBSOCK_PORT_ = "49999";
    
	// declare video parameters
	const int FRAME_WIDTH_ = 1920;
    const int FRAME_HEIGHT_ = 1080;
    const int VIDEO_FPS_ = 30;
    const int MAX_TOLERATED_CONSECUTIVE_DROPPED_FRAMES_ = 5;

	// declare filter parameters
    const float VISIBILITY_RATIO_ = 0.8;
    const float VISIBILITY_THRESH_ = 1.0;
    const float CONSECUTIVE_THRESH_ = 1.0;
    const float AGE_THRESH_ = 1.0;
    const int SECONDARY_FILTER_ = 2;
    const float SEC_FILTER_DELAY_ = 1.0;

    // declare environment compensation parameters
    const bool USE_HIST_EQUALISE_ = false;

    // declare image subtraction parameters
    const float DELTA_FRAME_PROPORTION_ = 0.85;

	// declare background subtractor parameters
    const bool USE_BG_SUBTRACTOR_ = false;
    const int FGBG_HISTORY_ = 5;
    const float BACKGROUND_RATIO_ = 0.05;
    const int NMIXTURES_ = 5;
    const int BRIGHTNESS_GAIN_ = 15;
    const float FGBG_LEARNING_RATE_ = 0.2;
    const int DILATION_ITER_ = 2;
    const float REMOVE_GROUND_ITER_ = 5.75;
    const float BACKGROUND_CONTOUR_CIRCULARITY_ = 0.5;

	// declare plotting parameters
	const int PLOT_HISTORY_ = 200;
	const double FONT_SCALE_ = 0.5;
    std::vector<cv::Scalar> COLORS_ = {
        cv::Scalar(124, 104, 66), // 1
        cv::Scalar(20, 60, 96), // 2
        cv::Scalar(46, 188, 243), // 3
        cv::Scalar(143, 89, 255), // 4
        cv::Scalar(6, 39, 156), // 5
        cv::Scalar(92, 215, 206), // 6
        cv::Scalar(105, 139, 246), // 7
        cv::Scalar(84, 43, 0), // 8
        cv::Scalar(137, 171, 197), // 9
        cv::Scalar(147, 226, 255) // 10
    };

    // declare starting frame
    int frame_count_;
}

#endif    // MULTI_CAM_PARAMS_HPP_