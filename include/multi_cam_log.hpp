/**
 * @file multi_cam_log.hpp
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
 * This file contains the declarations of the functions primarily used in the logging
 * of the image and data pipeline. It also contains the functions used in the user
 * interface.
 */

/**
 * @file mcmt_multi_tracker_node.hpp
 * @author Niven Sie, sieniven@gmail.com
 * @author Seah Shao Xuan
 * 
 * This code contains the McmtMultiTrackerNode class that runs our tracking and
 * re-identification process
 */

#define _USE_MATH_DEFINES

#ifndef MULTI_CAM_LOG_HPP_
#define MULTI_CAM_LOG_HPP_

// opencv header files
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/ximgproc.hpp>

// local header files
#include "multi_cam_track_utils.hpp"
#include "multi_cam_params.hpp"

// standard package imports
#include <string>
#include <map>
#include <memory>
#include <chrono>
#include <vector>
#include <list>
#include <array>
#include <numeric>
#include <fstream>
#include "json.h"

using namespace mcmt;

namespace mcmt {

	// declare video parameters
	extern cv::VideoWriter annotated_;

	extern std::ofstream frame_time_file, targets_2d_file, targets_3d_file;
	extern Json::StreamWriterBuilder builder;
	extern std::unique_ptr<Json::StreamWriter> writer;

	extern cv::Mat ui_;

	// define debugging tools
	extern std::vector<std::vector<double>> lines;
	extern std::vector<std::string> debug_messages;

	// declare logging variables
	extern Json::Value detections_2d_;
	extern Json::Value detections_3d_;

	// declare recording functions
	void initialize_recording(int frame_width, int frame_height);

	// declare UI functions
	void print_frame_summary();
	void annotate_frames(std::array<std::shared_ptr<cv::Mat>, NUM_OF_CAMERAS_> frames_, std::array<std::shared_ptr<CameraTracks>, NUM_OF_CAMERAS_> cumulative_tracks_);
	void graphical_UI(cv::Mat combined_frame, std::array<std::shared_ptr<CameraTracks>, NUM_OF_CAMERAS_> cumulative_tracks_, double actual_fps);
	
	// declare logging functions
	void initialize_logs();
	void log_2D();
	void log_3D(std::shared_ptr<TrackPlot> track_plot_a, std::shared_ptr<TrackPlot> track_plot_b);

}

#endif			// MULTI_CAM_LOG_HPP_