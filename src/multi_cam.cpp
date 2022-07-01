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

	initialize_cameras();
	initialize_tracks();
	if (RUN_DETECT_TRACK_) initialize_recording(cameras_[0]->frame_w_, cameras_[0]->frame_h_);
	if (RUN_DETECT_TRACK_) initialize_logs();

	auto frame_end = std::chrono::system_clock::now();
	std::chrono::duration<float> elapsed_seconds = std::chrono::duration<float>::zero();

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

				// clear detection variable vectors for next batch of detections
				// if the edgecam detections and images are not in sync, the clearing of detections is delayed
				// if, after delaying for a max number of frames specified by MAX_DETECTION_DELAY_,
				// the edgecam is still out of sync, the detections are cleared anyway to avoid clogging them up
				if (!edgecam_data.delay_required || camera->edgecam_cap_->detection_delays_ >= MAX_DETECTION_DELAY_){
					camera->sizes_.clear();
					camera->centroids_.clear();
					camera->edgecam_cap_->detection_delays_ = 0;
				}
				else{
					camera->edgecam_cap_->detection_delays_ += 1;
				}

				// get detections from edge cam - keep this code when interface shifts!
				for (auto & detection : edgecam_data.detections) {
					float size = std::max(float(detection.width), float(detection.height)); // "size" is a diameter!
					cv::Point2f coords(float(detection.x), float(detection.y));
					camera->centroids_.push_back(coords);
					camera->sizes_.push_back(size);
				}

				// display raw detections for debug purpose
				// cv::Mat srt_frame = camera->frame_.clone();
				// for (int i = 0; i < camera->centroids_.size(); i++) {
				// 	cv::rectangle(srt_frame, {camera->centroids_[i].x, camera->centroids_[i].y}, 
				// 	{camera->centroids_[i].x + camera->sizes_[i], camera->centroids_[i].y + camera->sizes_[i]}, 150, 3);
				// }
				// cv::imshow("SRT raw detections", srt_frame);
			}
			else {
				camera->cap_ >> camera->frame_;
			}

			// check if getting frame was successful
			if (camera->frame_.empty()) {
				std::cout << "Error: Video camera is disconnected!" << std::endl;
				is_disconnected_ = true;
			 	break;
			}
			else {
				// make a copy of the frame to be run through env compensation pipeline
				camera->frame_ec_ = camera->frame_.clone();
				if (IS_REALTIME_) {
					camera->recording_.write(camera->frame_);
				}
			}

			if (RUN_DETECT_TRACK_) {

				if (IS_REALTIME_ != 2) { // To remove when interface shifts

					// clear detection variable vectors
					camera->clear_detection_variables();

					if (USE_YOLO_DETECTION_) {
						yolo_detection(camera);
					}
					else{

					// correct for environmental effects
					apply_env_compensation(camera);

					// apply background subtractor
					// remove_ground(camera, 0);
					// remove_ground(camera, 1);

					simple_background_subtraction(camera);

					// get detections
					if (USE_BLOB_DETECTION) {
						detect_objects(camera);
					}
					else {
						contour_detection(camera);
					}

					}
				}// Edge Cam Interface is currently before KF/DCF. To be shifted

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

				// filter the tracks
				filter_tracks(camera);
			}
		}

		if (is_disconnected_) break;
		if (RUN_DETECT_TRACK_) log_2D();

		auto detect_end = std::chrono::system_clock::now();
		
		for (int i = 0; i < NUM_OF_CAMERAS_; i++) {
			frames_[i] = cameras_[i]->frame_;
			if (RUN_DETECT_TRACK_) {
				good_tracks_[i].clear();
				for (auto & track : cameras_[i]->good_tracks_) {
					auto good_track = std::shared_ptr<GoodTrack>(new GoodTrack());
					good_track->id = track->id_;
					good_track->x = track->centroid_.x;
					good_track->y = track->centroid_.y;
					good_track->size = track->size_;
					if (USE_YOLO_DETECTION_) {
						good_track->class_id = track->yolo_class_id_;
						good_track->confidence = track->yolo_confidence_;
					}
					good_tracks_[i].push_back(good_track);
				}
			}
		}

		auto track_start = std::chrono::system_clock::now();

		if (RUN_DETECT_TRACK_) {
		
			for (int cam_idx = 0; cam_idx < NUM_OF_CAMERAS_; cam_idx++) {

				// update all tracks with incoming information
				update_cumulative_tracks(cam_idx);

				// remove trackplots that are not relevant anymore
				prune_tracks(cam_idx);
			}

			if (NUM_OF_CAMERAS_ > 1) {
				for (int cam_idx_a = 0; cam_idx_a < NUM_OF_CAMERAS_; cam_idx_a++) {
					for (int cam_idx_b = 0; cam_idx_b < NUM_OF_CAMERAS_; cam_idx_b++) {
						if (cam_idx_a != cam_idx_b) process_new_tracks(cam_idx_a, cam_idx_b);
					}
				}
				join_matched_tracks();
				
				for (int cam_idx = 0; cam_idx < NUM_OF_CAMERAS_; cam_idx++)	verify_existing_tracks(cam_idx);
				calculate_3D();
			}

			print_frame_summary();
			annotate_frames();
		}

		auto track_end = std::chrono::system_clock::now();
		
		// show and save video combined tracking frame
		cv::hconcat(frames_, combined_frame_);

		// post image and data to the server hosted on the specified URL
		// @TODO: Middleman throws segmentation fault for some videos where the bounding box
		// exceeds the frame. Set POST_TO_SERVER_ to false until the problem is fixed
		if (POST_TO_SERVER_) {
			cv::imwrite("middleman/image.jpg", frames_[0]); 
			post_to_server("localhost:5000");
		}

		combined_frame_ = draw_lines(combined_frame_);

		std::cout << "Total frame took: " << elapsed_seconds.count() << "s\n";

		if (RUN_DETECT_TRACK_) {
			std::chrono::duration<float> detect_elapsed_seconds = detect_end - detect_start;
			std::cout << "Detection took: " << detect_elapsed_seconds.count() << "s\n";	
			std::chrono::duration<float> track_elapsed_seconds = track_end - track_start;
			std::cout << "Tracking took: " << track_elapsed_seconds.count() << "s\n";
			frame_time_file << detect_elapsed_seconds.count() << ", " << track_elapsed_seconds.count() << ", " << elapsed_seconds.count() << "\n"; 
		}

		if (GRAPHIC_UI_) {
			graphical_UI(1.0 / elapsed_seconds.count());
			cv::vconcat(combined_frame_, ui_, combined_frame_);
		}
	
		if (RUN_DETECT_TRACK_) annotated_.write(combined_frame_);

		// show cv window
		imshow_resized("Annotated", combined_frame_);
		
		frame_count_ += 1;

		if (cv::waitKey(1) == 'q') break;

		frame_end = std::chrono::system_clock::now();
		elapsed_seconds = frame_end - frame_start;
	}

	if (RUN_DETECT_TRACK_) {
		write_logs();
	 	annotated_.release();
	}
	close_cameras();
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