/**
 * @file multi_cam_detect.hpp
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
 * This file contains the definitions of the functions primarily used in the 
 * detection pipeline. These functions interact with the key classes Camera 
 * and Track which are essential to the detection process. These classes may 
 * be found at multi_cam_detect_utils.cpp.
 */

#ifndef MULTI_CAM_DETECT_HPP_
#define MULTI_CAM_DETECT_HPP_

// opencv header files
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/ximgproc.hpp>

// local header files
#include "multi_cam_detect_utils.hpp"
#include "multi_cam_params.hpp"
#include "Hungarian.h"

// standard package imports
#include <string>
#include <memory>
#include <chrono>
#include <vector>
#include <list>
#include <array>

// other package imports
#include "json.h"

using namespace mcmt;

namespace mcmt {
	
	// declare Camera variables
	extern std::vector<std::shared_ptr<Camera>> cameras_;
	extern cv::Mat element_;

	// declare detection and tracking functions
	void initialize_cameras						();
	void apply_env_compensation					(std::shared_ptr<Camera> & camera);
	void apply_bg_subtractions					(std::shared_ptr<Camera> & camera, int frame_id);
	void detect_objects							(std::shared_ptr<Camera> & camera);
	void remove_ground							(std::shared_ptr<Camera> & camera, int masked_id);
	void predict_new_locations_of_tracks		(std::shared_ptr<Camera> & camera);
	void clear_track_variables					(std::shared_ptr<Camera> & camera);
	void detection_to_track_assignment_KF		(std::shared_ptr<Camera> & camera);
	void detection_to_track_assignment_DCF		(std::shared_ptr<Camera> & camera);
	void compare_cost_matrices					(std::shared_ptr<Camera> & camera);
	void update_assigned_tracks					(std::shared_ptr<Camera> & camera);
	void update_unassigned_tracks				(std::shared_ptr<Camera> & camera);
	void create_new_tracks						(std::shared_ptr<Camera> & camera);
	void delete_lost_tracks						(std::shared_ptr<Camera> & camera);
	void filter_tracks							(std::shared_ptr<Camera> & camera);
	void close_cameras							();

	// declare utility functions
	float euclideanDist(cv::Point2f & p, cv::Point2f & q);
	std::vector<int> apply_hungarian_algo(std::vector<std::vector<double>> & cost_matrix);
	int average_brightness(std::shared_ptr<Camera> & camera);
	string mat_type2encoding(int mat_type);
	int encoding2mat_type(const string & encoding);

}

#endif				// MULTI_CAM_DETECT_HPP_