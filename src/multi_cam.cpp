/**
 * @file multi_cam.cpp
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
 * This file forms the main pipeline of our software and currently runs the 
 * detection and re-identification capabilities for multiple targets.
 * Any modifications that do not fundamentally amend the code logic should be
 * edited in the parameters file multi_cam_params.hpp. 
 * 
 */

// standard package imports
#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include <csignal>
#include <chrono>
#include <math.h>
#include <memory>
#include <algorithm>
#include <functional>
#include <fstream>
#include <map>
#include <set>
#include <eigen3/Eigen/Dense>

// local header files
#include "multi_cam_detect.hpp"
#include "multi_cam_detect_utils.hpp"
#include "multi_cam_track.hpp"
#include "multi_cam_track_utils.hpp"
#include "multi_cam_log.hpp"
#include "multi_cam_params.hpp"
#include "WSrtInterface.hpp"

// opencv header files
#include "opencv2/opencv.hpp"

// other package imports
#include "Hungarian.h"
#include "json.h"

using namespace mcmt;

int mcmt::frame_count_;

void imshow_resized(std::string window_name, cv::Mat img);

int main(int argc, char * argv[]) {

	frame_count_ = 1;
	bool is_disconnected_ = false;

	std::vector<cv::Mat> sample_frames = initialize_cameras();
	initialize_tracks(sample_frames[0]);
	initialize_recording(sample_frames[0]);
	initialize_logs();

	if (IS_REALTIME_ != 2) { // To remove when interface shifts
	for (int cam_idx = 0; cam_idx < NUM_OF_CAMERAS_; cam_idx++) {
		cameras_[cam_idx]->cap_ >> cameras_[cam_idx]->frame_store_;
	}
	}

	while (true) {
		
		auto frame_start = std::chrono::system_clock::now();
		auto detect_start = std::chrono::system_clock::now();

		for (auto & camera : cameras_) {

			// get camera frame
			if (IS_REALTIME_ == 2) { // To remove when interface shifts
				
				// here is where data from edge cam is passed to the main detection/tracking pipeline
				wsrt_output edgecam_data;
				edgecam_data = camera->edgecam_cap_->extract_data();

				// get image from edge cam - keep this code when interface shifts!
				camera->frame_ = edgecam_data.image.clone();

				// copy frame to masked to avoid downstream errors - to remove when interface shifts
				for (auto & it : camera->masked_) {
					it = camera->frame_.clone();
				}

				// clear detection variable vectors
				camera->sizes_.clear();
				camera->centroids_.clear();

				// get detections from edge cam - keep this code when interface shifts!
				for (auto & detection : edgecam_data.detections) {
					float size = std::max(float(detection.width), float(detection.height)); // "size" is a diameter!
					cv::Point2f coords(float(detection.x), float(detection.y));
					camera->centroids_.push_back(coords);
					camera->sizes_.push_back(size);
				}
			}
			else {
				camera->cap_ >> camera->frame_;
			}
			camera->frame_original_ = camera->frame_.clone();

			// check if getting frame was successful
			if (camera->frame_.empty()) {
				std::cout << "Error: Video camera is disconnected!" << std::endl;
				is_disconnected_ = true;
				break;
			}

			if (IS_REALTIME_) {
				recordings_[camera->cam_index_]->write(camera->frame_);
			}

			if (IS_REALTIME_ != 2) { // To remove when interface shifts

			// apply frame by frame subtraction for feature enhancement
			frame_to_frame_subtraction(camera);

			// correct for environmental effects
			apply_env_compensation(camera);

			// apply background subtraction
			for (int i = 0; i < camera->masked_.size(); i++){
				camera->masked_[i] = apply_bg_subtractions(camera, i);
			}

			// clear detection variable vectors
			camera->sizes_.clear();
			camera->centroids_.clear();
			for (int i = 0; i < camera->sizes_temp_.size(); i++){
				camera->sizes_temp_[i].clear();
				camera->centroids_temp_[i].clear();
			}
			
			// get detections
			detect_objects(camera);
			// cv::imshow("Remove Ground Original", camera->removebg_[0]);
			// cv::imshow("Remove Ground EC", camera->removebg_[1]);

			} // Edge Cam Interface is currently before KF/DCF. To be shifted
			
			// apply state estimation filters
			predict_new_locations_of_tracks(camera);

			clear_track_variables(camera);

			// get KF cost matrix and match detections and track targets
			detection_to_track_assignment_KF(camera);

			// get DCF cost matrix and match detections and track targets
			detection_to_track_assignment_DCF(camera);

			// compare DCF and KF cost matrix
			compare_cost_matrices(camera);

			// updated assigned tracks
			update_assigned_tracks(camera);

			// update unassigned tracks, and delete lost tracks
			update_unassigned_tracks(camera);
			delete_lost_tracks(camera);

			// create new tracks
			create_new_tracks(camera);

			// convert masked to BGR
			if (IS_REALTIME_ != 2) { // To remove when interface shifts
			for (auto & it : camera->masked_) {
				cv::cvtColor(it, it, cv::COLOR_GRAY2BGR);
			}
			}

			// filter the tracks
			camera->good_tracks_ = filter_tracks(camera);
			
			// update the stored t-1 frame 
			if (IS_REALTIME_ != 2) { // To remove when interface shifts
			camera->frame_store_ = camera->frame_original_.clone();
			}

		}

		if (is_disconnected_) {
			break;
		}

		auto detect_end = std::chrono::system_clock::now();
		
		for (int i = 0; i < NUM_OF_CAMERAS_; i++) {
			frames_[i] = std::make_shared<cv::Mat>(cameras_[i]->frame_original_);
			good_tracks_[i].clear();
			for (auto & track : cameras_[i]->good_tracks_) {
				auto good_track = std::shared_ptr<GoodTrack>(new GoodTrack());
				good_track->id = track->id_;
				good_track->x = track->centroid_.x;
				good_track->y = track->centroid_.y;
				good_track->size = track->size_;
				good_tracks_[i].push_back(good_track);
			}
		}
		
		auto track_start = std::chrono::system_clock::now();
		
		log_2D();

		for (int cam_idx = 0; cam_idx < NUM_OF_CAMERAS_; cam_idx++) {
			update_cumulative_tracks(cam_idx, good_tracks_[cam_idx]);
			prune_tracks(cam_idx);
		}

		if (NUM_OF_CAMERAS_ > 1) {
			process_new_tracks(0, 1, good_tracks_[0]);
			process_new_tracks(1, 0, good_tracks_[1]);

			verify_existing_tracks(0, 1);
			calculate_3D();
		}

		print_frame_summary();

		annotate_frames(frames_, cumulative_tracks_);

		auto track_end = std::chrono::system_clock::now();
		
		// show and save video combined tracking frame
		cv::Mat combined_frame = *frames_[0].get();
		for (int i = 1; i < NUM_OF_CAMERAS_; i++) {
			cv::hconcat(combined_frame, *frames_[i].get(), combined_frame);
		}
		
		// for (auto line : lines) {
		// 	cv::line(combined_frame, cv::Point((int) line[0], (int)line[1]), cv::Point((int) line[2], (int) line[3]), cv::Scalar(0, (int) (line[4] * 255), (int) ((1 - line[4]) * 255)), 1);
		// 	std::string scores;
		// 	scores = std::to_string(line[5]).substr(0,4) + ", " + std::to_string(line[6]).substr(0,4);
		// 	cv::putText(combined_frame, scores, cv::Point((int) ((line[0] + line[2]) / 2), (int) ((line[1] + line[3]) / 2)),  
		// 					cv::FONT_HERSHEY_SIMPLEX, FONT_SCALE_ * 1.5, cv::Scalar(0, (int) (line[4] * 255), (int) ((1 - line[4]) * 255)), 3, cv::LINE_AA);
		// }
		// lines.clear();

		auto frame_end = std::chrono::system_clock::now();

		std::chrono::duration<float> detect_elapsed_seconds = detect_end - detect_start;
		std::cout << "Detection took: " << detect_elapsed_seconds.count() << "s\n";	
		std::chrono::duration<float> track_elapsed_seconds = track_end - track_start;
		std::cout << "Tracking took: " << track_elapsed_seconds.count() << "s\n";
		std::chrono::duration<float> elapsed_seconds = frame_end - frame_start;
		std::cout << "Total frame took: " << elapsed_seconds.count() << "s\n";

		graphical_UI(combined_frame, cumulative_tracks_, sample_frames[0].size(), 1.0 / elapsed_seconds.count());
	
		frame_time_file << detect_elapsed_seconds.count() << ", " << track_elapsed_seconds.count() << ", " << elapsed_seconds.count() << "\n"; 

		cv::vconcat(combined_frame, ui_, combined_frame);
		
		recording_.write(combined_frame);

		// show cv window
		imshow_resized("Annotated", combined_frame);
		
		frame_count_ += 1;

		cv::waitKey(1);
	}

	std::cout << "Writing logs..." << std::endl;
	writer -> write(detections_2d_, &targets_2d_file);
	if (NUM_OF_CAMERAS_ > 1) {
		Json::Value output_3d;
		output_3d["Detections"] = detections_3d_;
		writer -> write(output_3d, &targets_3d_file);
	}

	recording_.release();
	if (IS_REALTIME_) {
		for (int cam_idx = 0; cam_idx < NUM_OF_CAMERAS_; cam_idx++) {
			recordings_[cam_idx]->release();
		}
	}
	close_cameras();

	frame_time_file.close();
	targets_2d_file.close();
	targets_3d_file.close();

	std::raise(SIGINT);

}

void imshow_resized(std::string window_name, cv::Mat img) {
	
	cv::Size img_size = img.size();

	double aspect_ratio = (double) img_size.width / (double) img_size.height;

	cv::Size window_size;
	window_size.width = 1280;
	window_size.height = (int) (1280.0 / aspect_ratio);
	
	cv::Mat img_;	
	cv::resize(img, img_, window_size, 0, 0, cv::INTER_CUBIC);
	cv::imshow(window_name, img_);
}