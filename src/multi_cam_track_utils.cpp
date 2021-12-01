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

// standard package imports
#include <stdlib.h>
#include <math.h>
#include <memory>
#include <algorithm>
#include <functional>
#include <iostream>

using namespace mcmt;
using namespace cv;

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
		vel_angle_leeway = 0.75; // Set to 0.75 radians, ~ 45 degrees
		vel_threshold = 16; // Set the threshold for velocity to be half the bounding box size
		circle_step = 16; // Resolution for circle polygon in low velocity search
	}

	/**
	 * This function updates the track with the latest track information
	 */
	void TrackPlot::update(std::vector<int> & location, int & size, int & frame_no) {
		update_track(location, size, frame_no);
		update_track_feature_variable(frame_no);
		update_3D_velocity_orientation(frame_no);
	}

	/**
	 * This function updates the track with the latest track information
	 */
	void TrackPlot::update_track(std::vector<int> & location, int & size, int & frame_no) {
		xs_.push_back(location[0]);
		ys_.push_back(location[1]);

		// Calculate diffrentials to determine drone velocity and magnitude using the
		// current and previous points
		diff_x = xs_[xs_.size() - 1] - xs_[xs_.size() - 2];
		diff_y = ys_[xs_.size() - 1] - ys_[xs_.size() - 2];
		last_vel = sqrt(diff_x, 2) + pow(diff_y, 2));
		last_vel_dir = atan(diff_y, diff_x);

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
	 * This function calculates the track feature variable of the track in the 
	 * current frame, and stores the value of the track feature variable inside
	 * the std::vector track_feature_variable_
	 */
	void TrackPlot::update_3D_velocity_orientation(int & frame_no) {
		// checks if there is enough data to calculate the turning angle (at least
		// 3 points) and that the data is from the current frame
		if (frameNos_.size() >= 3 && frameNos_.back() == frame_no) {
			// check if the last 3 frames are consecutive
			if ((frameNos_.end()[-2] == (frameNos_.end()[-1] - 1)) &&
					(frameNos_.end()[-3] == (frameNos_.end()[-2] - 1))) {

				// declare camera lens parameters
    			const int FOV_X_ = 69.5;
    			const int FOV_Y_ = 42.6;

				const int FRAME_WIDTH_ = 1920;
    			const int FRAME_HEIGHT_ = 1080;
				
				double dx = xs_.end()[-1] - xs_.end()[-2];
				double dy = ys_.end()[-1] - ys_.end()[-2];
				double r = (double) size_.end()[-1] / size_.end()[-2];

				std::array<double, 3> velocity;

				double velocity_x = (dx / FRAME_WIDTH_) * 2 * tan(M_PI / 180.0 * FOV_X_ / 2.0);
				velocity_x += ((1.0 / r) - 1.0) * tan((((double) xs_.end()[-1] / FRAME_WIDTH_) - 0.5) * FOV_X_ * M_PI / 180.0);

				double velocity_y = - (dy / FRAME_HEIGHT_) * 2 * tan(M_PI / 180.0 * FOV_Y_ / 2.0);
				velocity_y -= ((1.0 / r) - 1.0) * tan((((double) ys_.end()[-1] / FRAME_HEIGHT_) - 0.5) * FOV_Y_ * M_PI / 180.0);

				double velocity_z = (1.0 / r) - 1.0;

				double velocity_length = sqrt(pow(velocity_x, 2) + pow(velocity_y, 2) + pow(velocity_z, 2));

				if (velocity_length != 0) {
					velocity[0] = velocity_x / velocity_length; 
					velocity[1] = velocity_y / velocity_length; 
					velocity[2] = velocity_z / velocity_length; 
				} else {
					velocity[0] = 0.0;
					velocity[1] = 0.0;
					velocity[2] = 0.0;
				}

				// std::cout << "Velocity: " << velocity[0] << " " << velocity[1] << " " << velocity[2] << std::endl;

				vel_orient_.push_back(velocity);
			}
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
	 * This functions calculates the future search polygon for the specific track plot, done only when track is lost
	 * 
	 * return structure is a vector with element <int, int> tuple, convert to matrix only on search
	 */
	void TrackPlot::construct_look_ahead_polygon() {
		// Return Structure for polygon
		std::vector<std::tuple<int, int>> search_polygon;

		// If the drone is travelling fast, use cone to track drone
		if (last_vel_mag > vel_threshold) { 
			last_x = xs_[xs_.size() - 1];
			last_y = ys_[ys_.size() - 1];
			std::tuple<int, int> starting_point = (last_x, last_y);

			//Calculate the upper bound, center bound and lower bound locations, using polar coordinates representation
			std::tuple<int, int> upper_bound = (last_x + last_vel_mag * cos(last_vel_dir + vel_angle_leeway), last_y + last_vel_mag * sin(last_vel_dir + vel_angle_leeway));
			std::tuple<int, int> lower_bound = (last_x + last_vel_mag * cos(last_vel_dir - vel_angle_leeway), last_y + last_vel_mag * sin(last_vel_dir - vel_angle_leeway));
			std::tuple<int, int> center_bound = (last_x + last_vel_mag * cos(last_vel_dir), last_y + last_vel_mag * sin(last_vel_dir));

			search_polygon.push_back(starting_point);
			search_polygon.push_back(upper_bound);
			search_polygon.push_back(center_bound);
			search_polygon.push_back(lower_bound);

			return search_polygon
		}

		else { // else use Circular polygon  to find the drone 
			for (i = 0, i < 2*M_PI, i += M_PI/8) {
				global_x = xs_[xs_.size() - 1] + circle_step*cos(i);
				global_y = ys_[ys_.size() - 1] + circle_step*sin(i);
				std::tuple<int, int> circle_point = (global_x, global_y);
				search_polygon.push_back(circle _point);
			}

			return search_polygon
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