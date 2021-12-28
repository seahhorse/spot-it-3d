/**
 * @file multi_cam_track_utils.hpp
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
 * This file contains the declaration of the classes (CameraTracks and TrackPlots)
 * and their associated methods, which is primarily used in the re-identification
 * pipeline. The full definition of the classes and their methods can be found 
 * in the file multi_cam_track_utils.cpp. 
 */

#ifndef MCMT_TRACK_UTILS_HPP_
#define MCMT_TRACK_UTILS_HPP_

// TODO : FIND OUT WHY CANNOT IMPORT
// local header files
// #include "multi_cam_params.hpp"

// opencv header files
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/ximgproc.hpp>

// standard package imports
#include <string>
#include <map>
#include <memory>
#include <chrono>
#include <vector>
#include <list>
#include <array>
#include <curl/curl.h>

namespace mcmt {

	struct MemoryStruct {
		char *memory;
		size_t size;
	};
 
	static size_t
	WriteMemoryCallback(void *contents, size_t size, size_t nmemb, void *userp)	{
		size_t realsize = size * nmemb;
		struct MemoryStruct *mem = (struct MemoryStruct *)userp;
		
		char *ptr = (char*)realloc(mem->memory, mem->size + realsize + 1);
		if(!ptr) return 0;		
		mem->memory = ptr;
		memcpy(&(mem->memory[mem->size]), contents, realsize);
		mem->size += realsize;
		mem->memory[mem->size] = 0;
		
		return realsize;
	}

	
	/**
	 * This class is for storing our tracks' information, calculating track feature
	 * variable, and caluclating geometrical angles and distances of the tracks.
	 */
	class TrackPlot {
		public:
			TrackPlot(int track_id);
			virtual ~TrackPlot() {}
			
			// define other_tracks
			typedef struct OtherTrack {
				float angle;
				float dist;
				int dx;
				int dy;
			} OtherTrack;

			// declare track information
			bool is_drone_ = true;
			int id_, oid_, lastSeen_, mismatch_count_;
			double classification_confidence_ = -1;
			std::vector<int> xs_, ys_, size_, frameNos_, area_;
			std::vector<double> xyz_, turning_angle_, curvature_, track_feature_variable_;
			std::vector<std::array<double, 3>> vel_orient_;
			std::vector<std::shared_ptr<OtherTrack>> other_tracks_;

			// declare methods
			void update(std::vector<int> & location, int & size, int & frame_no, cv::Mat frame);
			void update_track(std::vector<int> & location, int & size, int & frame_no);
			void update_track_feature_variable(int & frame_no);
			void update_area(cv::Mat frame);
			void update_3D_velocity_orientation(int & frame_no);
			void construct_look_ahead_polygon();
			void update_classification(int & frame_no);
			bool check_stationary();
	};

	/**
	 * This class is for tracking the camera's track plots.
	 */
	class CameraTracks {
		public:
			CameraTracks(int index);
			virtual ~CameraTracks() {}

			// declare camera parameters
			int index_;

			// declare track plot variables
			std::map<int, std::shared_ptr<TrackPlot>> track_plots_, track_new_plots_;
	};

	void update_other_tracks(std::shared_ptr<TrackPlot> trackplot, 
		std::shared_ptr<CameraTracks> & cumulative_tracks);

	void combine_track_plots(
		int & index,
		std::shared_ptr<CameraTracks> camera_tracks,
		std::shared_ptr<TrackPlot> track_plot,
		int & frame_count);

}

#endif			// MCMT_TRACK_UTILS_HPP_