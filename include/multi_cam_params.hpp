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

    // realtime parameters
    const std::vector<std::string> VIDEO_INPUT_ = {     "data/output/17-12_OHR_OldWebcam1_1.avi",
                                                        "6"
                                                    };
    const std::vector<std::string> VIDEO_OUTPUT_ = {    "data/output/20-12_WCP_1a.avi",
                                                        "data/output/20-12_WCP_1b.avi"
                                                    };
	const std::string VIDEO_OUTPUT_ANNOTATED_ = "data/output/17-12_OHR_OldWebcam1_1_out.avi";
	const std::string TARGETS_2D_OUTPUT_ = "data/output/20-12_WCP_1_targets_2d_out.json";
    const std::string TARGETS_3D_OUTPUT_ = "data/output/20-12_WCP_1_targets_3d_out.json";
    const std::string FRAME_TIME_ = "data/output/17-12_OHR_OldWebcam1_1_frame_time_.csv";
    
    // declare display settings
    const bool GRAPHIC_UI_ = true;
    const bool SHOW_UNMATCHED_TARGETS_ = true;
    const bool SHOW_CAM_NUM_ = false;
    const bool SHOW_ID_ = true;
    const bool SHOW_3D_COORDINATES_ = true;

	// declare video parameters
	const int FRAME_WIDTH_ = 640;
    const int FRAME_HEIGHT_ = 480;
    const int VIDEO_FPS_ = 30;
    const int MAX_TOLERATED_CONSECUTIVE_DROPPED_FRAMES_ = 5;

	// declare filter parameters
    const float VISIBILITY_RATIO_ = 0.5;
    const float VISIBILITY_THRESH_ = 1.0;
    const float CONSECUTIVE_THRESH_ = 5.0;
    const float AGE_THRESH_ = 1.0;
    const int SECONDARY_FILTER_ = 2;
    const float SEC_FILTER_DELAY_ = 1.0;

    // declare environment compensation parameters
    const bool USE_HIST_EQUALISE_ = false;

    // declare image subtraction parameters
    const float DELTA_FRAME_PROPORTION_ = 0.00;

	// declare background subtractor parameters
    const bool USE_BG_SUBTRACTOR_ = true;
    const int FGBG_HISTORY_ = 30;
    const float BACKGROUND_RATIO_ = 0.4;
    const int NMIXTURES_ = 5;
    const int BRIGHTNESS_GAIN_ = 15;
    const float FGBG_LEARNING_RATE_ = 0.5;
    const int DILATION_ITER_ = 1;
    const float REMOVE_GROUND_ITER_ = 3;
    const float BACKGROUND_CONTOUR_CIRCULARITY_ = 0.4;

    // declare tracking parameters
    const int PRUNE_HIST_ = 30;

    // declare re-id parameters
    const bool USE_3D_REID_ = true;
    const double W1_ = 0.3;
    const double W2_ = 0.4;
    const double W3_ = 0.3;
    const int TRACK_XJ_MAX_ = 120;
    const int HDG_SCORE_TRACK_HIST_ = 30;

	// declare plotting parameters
	const int PLOT_HISTORY_ = 200;
	const double FONT_SCALE_ = 0.5;
    const bool DISPLAY_MATCHED_ONLY_ = false;
    const bool DISPLAY_ID_ = true;
    const bool DISPLAY_3D_ = true;
    const bool DISPLAY_STATUS_ = true;
    const std::vector<cv::Scalar> COLORS_{
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

    extern int frame_count_;

}

#endif    // MULTI_CAM_PARAMS_HPP_