/**
 * @file multi_cam_track_utils.cpp
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
 * This file contains the definition of the classes (CameraTracks and TrackPlots)
 * and their associated methods, which is primarily used in the re-identification
 * pipeline.
 */

// mathematical constants
#define _USE_MATH_DEFINES

// local header files
#include "multi_cam_track_utils.hpp"
#include "multi_cam_params.hpp"

// opencv header files
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/ximgproc.hpp>

// standard package imports
#include <stdlib.h>
#include <math.h>
#include <memory>
#include <algorithm>
#include <functional>
#include <numeric>
#include <iostream>
#include <curl/curl.h>

using namespace mcmt;

namespace mcmt {

	/**
	 * This class is for tracking the camera's track plots
	 */
	CameraTracks::CameraTracks(int index) {
		// set camera index
		index_ = index;
	}

	/**
	 * This class is for storing our tracks' information, calculating track feature
	 * variable, and caluclating geometrical angles and distances of the tracks.
	 */
	TrackPlot::TrackPlot(int track_id) {
		// set the trackid and the mismatch count to initial value of zero
		id_ = track_id;
		oid_ = track_id;
		mismatch_count_ = 0;
		lastSeen_ = 0;

	}

	/**
	 * This function updates the track with the latest track information
	 */
	void TrackPlot::update(std::vector<int> & location, int & size, int & frame_no, cv::Mat frame) {
		update_track(location, size, frame_no);
		update_track_feature_variable(frame_no);
		update_area(frame);
		update_3D_velocity_orientation(frame_no);
		try {
			update_classification(frame_no);
		} catch(...) {
			is_drone_ = true;
			classification_confidence_ = -1;
		}
	}

	/**
	 * This function updates the track with the latest track information
	 */
	void TrackPlot::update_track(std::vector<int> & location, int & size, int & frame_no) {
		xs_.push_back(location[0]);
		ys_.push_back(location[1]);
		size_.push_back(size);
		frameNos_.push_back(frame_no);
		lastSeen_ = frame_no;
	}

	/**
	 * This function calculates the track feature variable of the track in the 
	 * current frame, and stores the value of the track feature variable inside
	 * the std::vector track_feature_variable_
	 */
	void TrackPlot::update_track_feature_variable(int & frame_no) {
		// checks if there is enough data to calculate the turning angle (at least
		// 3 points) and that the data is from the current frame
		if (frameNos_.size() >= 3 && frameNos_.back() == frame_no) {
			// check if the last 3 frames are consecutive
			if ((frameNos_.end()[-2] == (frameNos_.end()[-1] - 1)) &&
					(frameNos_.end()[-3] == (frameNos_.end()[-2] - 1))) {
				// retrieve the x and y values of the last 3 points
				int t = 2;
				std::vector<int> x(xs_.end() - 3, xs_.end());
				std::vector<int> y(ys_.end() - 3, ys_.end());

				// Turning angle and curvature
				
				/** 
				 * This code block has not been ported yet as the old method of calculation
				 * is not in the current pipeline. to see the old methodology, refer to the
				 * python version of mcmt_track_utils.py.
				 */

				// Pace
				float d = hypot((x[t] - x[t - 1]), (y[t] - y[t - 1]));				
				// float pace = d * fps;
				// pace_.push_back(pace);

				// append track feature variable. the current pipeline uses pace as our
				// track feature variable value
				track_feature_variable_.push_back(d);

				// truncates after a certain length
				if (track_feature_variable_.size() > TRACK_XJ_MAX_) {
					track_feature_variable_.erase(track_feature_variable_.begin());
				}
			}
		}
	}

	/**
	 * This function updates the area calculated by thresholding black pixels 
	 * within the size of detection.
	 */
	void TrackPlot::update_area(cv::Mat frame) {
		cv::Mat local_region;
		int left = std::min(FRAME_WIDTH_, std::max(0, xs_.back() - size_.back()));
		int top = std::min(FRAME_HEIGHT_, std::max(0, ys_.back() - size_.back()));
		int width = std::min(2 * size_.back(), FRAME_WIDTH_ - left);
		int height = std::min(2 * size_.back(), FRAME_HEIGHT_ - top);
		frame(cv::Rect(left, top, width, height)).copyTo(local_region);
		cv::cvtColor(local_region, local_region, cv::COLOR_BGR2GRAY);
		cv::threshold(local_region, local_region, cv::mean(local_region)[0] - 10, 255, 0);
		int area = (4 * size_.back() * size_.back()) - cv::countNonZero(local_region);	
		area_.push_back(area);
	}

	/**
	 * This function calculates the track feature variable of the track in the 
	 * current frame, and stores the value of the track feature variable inside
	 * the std::vector track_feature_variable_
	 */
	void TrackPlot::update_3D_velocity_orientation(int & frame_no) {
		// checks if the data is from the current frame
		if (frameNos_.size() > 15 && frameNos_.back() == frame_no) {

			int history = 15;

			// declare camera lens parameters
			
			// Specs for the Creative Camera
			const int FOV_X_ = 69.5;
			const int FOV_Y_ = 42.6;

			// Specs for the GOPRO HERO 9 Camera
			// const int FOV_X_ = 87;
			// const int FOV_Y_ = 56;
			
			double dx = xs_.end()[-1] - xs_.end()[-1-history];
			double dy = ys_.end()[-1] - ys_.end()[-1-history];

			int area = area_.end()[-1];
			int last_area = area_.end()[-1-history];
			
			double r;
			if (!area || !last_area) {
				r = 1;
			} else {
				r = sqrt((double) area / (double) last_area);
			}

			std::array<double, 3> velocity;

			double velocity_x = (dx / FRAME_WIDTH_) * 2 * tan(M_PI / 180.0 * FOV_X_ / 2.0);
			velocity_x += ((1.0 / r) - 1.0) * tan((((double) xs_.end()[-1] / FRAME_WIDTH_) - 0.5) * FOV_X_ * M_PI / 180.0);

			double velocity_y = - (dy / FRAME_HEIGHT_) * 2 * tan(M_PI / 180.0 * FOV_Y_ / 2.0);
			velocity_y -= ((1.0 / r) - 1.0) * tan((((double) ys_.end()[-1] / FRAME_HEIGHT_) - 0.5) * FOV_Y_ * M_PI / 180.0);

			double velocity_z = (1.0 / r) - 1.0;

			double velocity_length = sqrt(pow(velocity_x, 2) + pow(velocity_y, 2) + pow(velocity_z, 2));

			if (velocity_length != 0 && velocity_length >= 0.05) {
				velocity[0] = velocity_x / velocity_length; 
				velocity[1] = velocity_y / velocity_length; 
				velocity[2] = velocity_z / velocity_length; 
			} else {
				velocity[0] = 0.0;
				velocity[1] = 0.0;
				velocity[2] = 0.0;
			}

			vel_orient_.push_back(velocity);
		}
	}

	void TrackPlot::update_classification(int & frame_no) {
		
		if (frameNos_.size() == 8 && frameNos_.back() == frame_no) {			
			std::string payload = "{ \"flight_path\":[ ";
			for (int i = -8; i < -1; i++) {
				payload += std::to_string((double) xs_.end()[i] / FRAME_WIDTH_) + ", ";
				payload += std::to_string((double) ys_.end()[i] / FRAME_HEIGHT_) + ", ";
			}
			payload += std::to_string(xs_.back()) + ", ";
			payload += std::to_string(ys_.back()) + "]}";

			CURL *curl;
			CURLcode res;
			struct MemoryStruct chunk;
			chunk.memory = (char*)malloc(1);
			chunk.size = 0;

			curl_global_init(CURL_GLOBAL_ALL);
			curl = curl_easy_init();
			if(curl) {
				curl_easy_setopt(curl, CURLOPT_URL, "localhost:4000/api/classify");

				struct curl_slist *list = NULL;
				list = curl_slist_append(list, "Content-Type: application/json");
				list = curl_slist_append(list, "Accept: application/json");
				curl_easy_setopt(curl, CURLOPT_HTTPHEADER, list);
				curl_easy_setopt(curl, CURLOPT_POSTFIELDS, payload.c_str());
				curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, WriteMemoryCallback);
				curl_easy_setopt(curl, CURLOPT_WRITEDATA, (void *)&chunk);
				res = curl_easy_perform(curl);

				if(res != CURLE_OK)
					fprintf(stderr, "curl_easy_perform() failed: %s\n",
							curl_easy_strerror(res));

				is_drone_ = chunk.memory[14] == 'd';

				std::string confidence = "";
				for(int i=40;i<45;i++)  
					confidence += chunk.memory[i];

				classification_confidence_ = std::stod(confidence);
				free(chunk.memory);
				curl_easy_cleanup(curl);
			}
			curl_global_cleanup();
		}
	}



	/**
	 * This function checks if the current TrackPlot has been stationary for too long,
	 * by calculating the distance the track moved for the last 9 frames. it returns
	 * a bool True if its average distance moved per frame is less than 3.0, and returns
	 * False if otherwise
	 */
	bool TrackPlot::check_stationary() {

		// declare function variable to store total Euclidean distance traveled
		float distance = 0.0;

		for (int i = -1; i >= -9; i--) {
			int dx = xs_[xs_.size() + i] - xs_[xs_.size() + i - 1];
			int dy = ys_[ys_.size() + i] - ys_[ys_.size() + i - 1];

			// get euclidean distance, and add to total distance
			distance += hypot(dx, dy);
		}

		distance /= 9;

		if (distance < 3.0) {
			return true;
		} else {
			return false;
		}
	}

	/**
	 * this function updates the current frame's other_tracks list
	 */
	void update_other_tracks(std::shared_ptr<TrackPlot> trackplot,
		std::shared_ptr<CameraTracks> & cumulative_tracks) {
		
		// clear other_tracks vector
		trackplot->other_tracks_.clear();

		std::map<int, std::shared_ptr<TrackPlot>> all_tracks(cumulative_tracks->track_new_plots_);
		all_tracks.insert(cumulative_tracks->track_plots_.begin(), cumulative_tracks->track_plots_.end());

		for (auto & other_track : all_tracks) {

			if (other_track.second->xs_.size() != 0 && other_track.second->ys_.size() != 0) {
				int dx = other_track.second->xs_.end()[-1] - trackplot->xs_.end()[-1];
				int dy = other_track.second->ys_.end()[-1] - trackplot->ys_.end()[-1];

				if (dx != 0 && dy != 0) {
					auto new_other_track = std::shared_ptr<TrackPlot::OtherTrack>(new TrackPlot::OtherTrack());
					new_other_track->angle = atan2(dy, dx);
					new_other_track->dist = hypot(dx, dy);
					new_other_track->dx = dx;
					new_other_track->dy = dy;
					trackplot->other_tracks_.push_back(new_other_track);
				}
			}
		}
	}
}