/**
 * @file multi_cam_track.hpp
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
 * This file contains the declarations of the functions primarily used in the tracking 
 * and re-identification pipeline. These functions interact with the key classes
 * CameraTracks and TrackPlots which are essential to the tracking and 
 * re-identification process. These classes may be found at multi_cam_detect_utils.cpp.
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

#ifndef MULTI_CAM_TRACK_HPP_
#define MULTI_CAM_TRACK_HPP_

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
#include <eigen3/Eigen/Dense>

using namespace mcmt;

namespace mcmt {

	// define good tracks
	typedef struct GoodTrack {
		int id;
		int x;
		int y;
		int size;
	} GoodTrack;

	// declare tracking variables
	extern int next_id_;
	extern std::array<std::shared_ptr<CameraTracks>, NUM_OF_CAMERAS_> cumulative_tracks_;
	extern std::array<std::map<int, int>, NUM_OF_CAMERAS_> matching_dict_;
	extern std::map<int, std::array<int, NUM_OF_CAMERAS_ + 1>> matched_tracks_;

	// declare tracking arrays
	extern std::array<std::shared_ptr<cv::Mat>, NUM_OF_CAMERAS_> frames_;
	extern std::array<std::vector<std::shared_ptr<GoodTrack>>, NUM_OF_CAMERAS_> good_tracks_;

	// declare tracking functions	
	void initialize_tracks(cv::Mat sample_frame);
	template <typename T1, typename T2>
	bool exists(std::map<T1, T2> arr, T1 item);
	void update_cumulative_tracks(int index, std::vector<std::shared_ptr<GoodTrack>> & good_tracks);
	void prune_tracks(int index);
	void verify_existing_tracks(int idx_a, int idx_b);
	void process_new_tracks(int idx, int alt, std::vector<std::shared_ptr<GoodTrack>> & good_tracks);
	void get_total_number_of_tracks();
	std::vector<double> normalise_track_plot(std::shared_ptr<TrackPlot> track_plot);
	double compute_matching_score(std::shared_ptr<TrackPlot> track_plot_a, std::shared_ptr<TrackPlot> track_plot_b, int idx_a, int idx_b);
	double crossCorrelation(std::vector<double> X, std::vector<double> Y);
	double geometric_similarity(std::vector<std::shared_ptr<TrackPlot::OtherTrack>> & other_tracks_a,
		std::vector<std::shared_ptr<TrackPlot::OtherTrack>> & other_tracks_b);
	double heading_score(std::shared_ptr<TrackPlot> track_plot_a, std::shared_ptr<TrackPlot> track_plot_b);
	double crossCorrelation_3D(std::vector<std::array<double, 3>> X, std::vector<std::array<double, 3>> Y);
	void generate_matched_ids(int idx_a, int idx_b);
	void promote_to_matched(int index, std::shared_ptr<TrackPlot> track_plot, int matched_id, int old_id);
	void calculate_3D();
	void imshow_resized_dual(std::string & window_name, cv::Mat & img);

}

#endif			// MULTI_CAM_TRACK_HPP_