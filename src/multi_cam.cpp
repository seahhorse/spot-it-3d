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
#include "multi_cam_params.hpp"

// opencv header files
#include "opencv2/opencv.hpp"

// other package imports
#include "Hungarian.h"
#include "json.h"

using namespace mcmt;

std::ofstream frame_time_file, targets_2d_file, targets_3d_file;
Json::StreamWriterBuilder builder;
std::unique_ptr<Json::StreamWriter> writer(builder.newStreamWriter());

cv::Mat ui_;

void initialize_logs();
void annotate_frames(std::array<std::shared_ptr<cv::Mat>, NUM_OF_CAMERAS_> frames_, std::array<std::shared_ptr<CameraTracks>, NUM_OF_CAMERAS_> cumulative_tracks_);
void graphical_UI(cv::Mat combined_frame, std::array<std::shared_ptr<CameraTracks>, NUM_OF_CAMERAS_> cumulative_tracks_, cv::Size frame_size, double actual_fps);
void imshow_resized(std::string window_name, cv::Mat img);

int main(int argc, char * argv[]) {

	frame_count_ = 1;
	bool is_disconnected_ = false;

	cv::Mat sample_frame = initialize_cameras();
	initialize_tracks(sample_frame);
	initialize_logs();

	while (frame_count_ <= 90) {
		
		auto frame_start = std::chrono::system_clock::now();
		auto detect_start = std::chrono::system_clock::now();

		for (auto & camera : cameras_) {

			// get camera frame
			camera->cap_ >> camera->frame_;

			// check if getting frame was successful
			if (camera->frame_.empty()) {
				std::cout << "Error: Video camera is disconnected!" << std::endl;
				is_disconnected_ = true;
				break;
			}

			if (IS_REALTIME_) {
				recordings_[camera->cam_index_]->write(camera->frame_);
			}

			// Correct for environmental effects
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
			for (auto & it : camera->masked_) {
				cv::cvtColor(it, it, cv::COLOR_GRAY2BGR);
			}

			// filter the tracks
			camera->good_tracks_ = filter_tracks(camera);

			std::cout << "Total number of tracks in camera " << camera->cam_index_ << ": " << camera->tracks_.size() << std::endl;

			log_2D();
		}

		if (is_disconnected_) {
			break;
		}

		auto detect_end = std::chrono::system_clock::now();
		
		for (int i = 0; i < NUM_OF_CAMERAS_; i++) {
			frames_[i] = std::make_shared<cv::Mat>(cameras_[i]->frame_);
			good_tracks_[i].clear();
			for (auto & track : cameras_[i]->good_tracks_) {
				auto good_track = std::shared_ptr<GoodTrack>(new GoodTrack());
				good_track->id = track->id_;
				good_track->x = track->centroid_.x;
				good_track->y = track->centroid_.y;
				good_track->size = track->size_;
				good_tracks_[i].push_back(good_track);
			}
			// create filter copy of good_tracks list
			dead_tracks_[i] = cameras_[i]->dead_tracks_;
			filter_good_tracks_[i] = good_tracks_[i];
		}
		
		auto track_start = std::chrono::system_clock::now();

		for (int i = 0; i < NUM_OF_CAMERAS_; i++) {
			update_cumulative_tracks(i, good_tracks_);
		}

		if (NUM_OF_CAMERAS_ > 1) {

			process_new_tracks(0, 1, good_tracks_, filter_good_tracks_, dead_tracks_);
			process_new_tracks(1, 0, good_tracks_, filter_good_tracks_, dead_tracks_);

			generate_matched_ids();

			verify_existing_tracks();

			calculate_3D();
			
			log_3D();

			prune_tracks(0);
			prune_tracks(1);

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

		graphical_UI(combined_frame, cumulative_tracks_, sample_frame.size(), 1.0 / elapsed_seconds.count());
	
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

void initialize_logs() {

	// initialize logs
	frame_time_file.open(FRAME_TIME_);
	frame_time_file << "detect_time, track_time, total_time" << "\n"; 
	
	targets_2d_file.open(TARGETS_2D_OUTPUT_);
	targets_3d_file.open(TARGETS_3D_OUTPUT_);
}

void annotate_frames(std::array<std::shared_ptr<cv::Mat>, NUM_OF_CAMERAS_> frames_, std::array<std::shared_ptr<CameraTracks>, NUM_OF_CAMERAS_> cumulative_tracks_) {
	
	// draw tracks on opencv GUI to monitor the detected tracks
	// lopp through each camera frame
	for (int i = 0; i < NUM_OF_CAMERAS_; i++) {

		cv::putText(*frames_[i].get(), "CAM " + std::to_string(i), cv::Point(20, 30),
			cv::FONT_HERSHEY_SIMPLEX, FONT_SCALE_ * 1, cv::Scalar(255, 0, 0), 2, cv::LINE_AA);

		std::map<int, std::shared_ptr<TrackPlot>> display_tracks;

		if (NUM_OF_CAMERAS_ > 1) {
			display_tracks = cumulative_tracks_[i]->track_plots_;
		} else {
			display_tracks = cumulative_tracks_[i]->track_new_plots_;
		}
		
		// loop through every track plot
		if (display_tracks.empty() == false) {
			for (auto track = display_tracks.begin(); track != display_tracks.end(); track++) {
				if ((frame_count_ - track->second->lastSeen_) <= VIDEO_FPS_) {
					
					cv::Point2i rect_top_left((track->second->xs_.back() - (track->second->size_.back())), 
															(track->second->ys_.back() - (track->second->size_.back())));
		
					cv::Point2i rect_bottom_right((track->second->xs_.back() + (track->second->size_.back())), 
																	(track->second->ys_.back() + (track->second->size_.back())));
		
					
					cv::rectangle(*frames_[i].get(), rect_top_left, rect_bottom_right, COLORS_[track->second->id_ % 10], 2);

					cv::Scalar status_color;
					if (track->second->lastSeen_ == frame_count_) {
						status_color = cv::Scalar(0, 255, 0);
					} else {
						status_color = cv::Scalar(0, 0, 255);
					}
					
					// get last frames up till plot history (200)
					shown_indexes_.clear();

					for(int j = track->second->frameNos_.size() - 1; j >= 0; j--) {
						if (track->second->frameNos_[j] > (frame_count_ - PLOT_HISTORY_)) {
							shown_indexes_.push_back(j);
						} else {
							break;
						}
					}

					for (auto & idx : shown_indexes_) {

						int color_idx = track->second->frameNos_[idx] - frame_count_ + PLOT_HISTORY_ - 1;
						double alpha = 0.5 + (double) color_idx / 400;
						double beta = 1 - alpha;

						cv::Vec3b pixelColor = (*frames_[i].get()).at<cv::Vec3b>(track->second->ys_[idx], track->second->xs_[idx]);

						cv::circle(*frames_[i].get(), cv::Point(track->second->xs_[idx], track->second->ys_[idx]), 3,
							cv::Scalar((int) (pixelColor[0] * beta + (COLORS_[track->second->id_ % 10][0] * alpha)),
										(int) (pixelColor[1] * beta + (COLORS_[track->second->id_ % 10][1] * alpha)),
										(int) (pixelColor[2] * beta + (COLORS_[track->second->id_ % 10][2] * alpha))), -1);
					}

					// put ID and XYZ coordinates on opencv GUI
					if (shown_indexes_.empty() == false) {
						cv::putText(*frames_[i].get(), "ID: " + std::to_string(track->second->id_).substr(0,4), 
							cv::Point(rect_top_left.x + 20, rect_top_left.y - 5), cv::FONT_HERSHEY_SIMPLEX,
							FONT_SCALE_, COLORS_[track->second->id_ % 10], 1, cv::LINE_AA);
						
						if (track->second->xyz_.empty() == false) {
							cv::putText(*frames_[i].get(), "X: " + std::to_string(track->second->xyz_[0]).substr(0,4),
								cv::Point(rect_bottom_right.x + 10, rect_top_left.y + 10), cv::FONT_HERSHEY_SIMPLEX,
								FONT_SCALE_, COLORS_[track->second->id_ % 10], 1, cv::LINE_AA);

							cv::putText(*frames_[i].get(), "Y: " + std::to_string(track->second->xyz_[1]).substr(0,4),
								cv::Point(rect_bottom_right.x + 10, rect_top_left.y + 25), cv::FONT_HERSHEY_SIMPLEX,
								FONT_SCALE_, COLORS_[track->second->id_ % 10], 1, cv::LINE_AA);

							cv::putText(*frames_[i].get(), "Z: " + std::to_string(track->second->xyz_[2]).substr(0,4),
								cv::Point(rect_bottom_right.x + 10, rect_top_left.y + 40), cv::FONT_HERSHEY_SIMPLEX,
								FONT_SCALE_, COLORS_[track->second->id_ % 10], 1, cv::LINE_AA);

						}
					}
					
					cv::circle(*frames_[i].get(), cv::Point(rect_top_left.x + 5, rect_top_left.y - 10), 5, status_color, -1);	

				}

				// if (track->second->check_stationary()) {
				// 	cv::putText(*frames_[i].get(), "S", cv::Point(track->second->xs_.back(), track->second->ys_.back() - 40),
				// 			cv::FONT_HERSHEY_SIMPLEX, FONT_SCALE_ * 2, cv::Scalar(255, 0, 0), 2, cv::LINE_AA);
				// }

			}
		}

	}
}

void graphical_UI(cv::Mat combined_frame, std::array<std::shared_ptr<CameraTracks>, NUM_OF_CAMERAS_> cumulative_tracks_, cv::Size frame_size, double actual_fps) {

	int ui_width = 3840;
	int ui_height = 360;
	cv::Mat ui(ui_height, ui_width, CV_8UC3, cv::Scalar(0,0,0));

	// Summary box
	cv::rectangle(ui, cv::Point(20, 20), cv::Point(1910, 80), cv::Scalar(220,220,220), -1);
	cv::rectangle(ui, cv::Point(20, 20), cv::Point(1910, 80), cv::Scalar(110,110,110), 4);
	int spacing = 0;
	int drones_on_screen = 0;
	std::map<int, std::shared_ptr<TrackPlot>> display_tracks;
	if (NUM_OF_CAMERAS_ > 1) {
		display_tracks = cumulative_tracks_[0]->track_plots_;
	} else {
		display_tracks = cumulative_tracks_[0]->track_new_plots_;
	}
	if (display_tracks.empty() == false) {
		for (auto track = display_tracks.begin(); track != display_tracks.end(); track++) {
			if ((frame_count_ - track->second->lastSeen_) <= VIDEO_FPS_) {
				drones_on_screen++;
				cv::putText(ui, "ID: " + std::to_string(track->second->id_).substr(0,4), cv::Point(60 + spacing, 65), cv::FONT_HERSHEY_SIMPLEX,
						FONT_SCALE_ * 2.5, COLORS_[track->second->id_ % 10], 3, cv::LINE_AA);
				spacing += 200;	
			}
		}
	}

	// Notification box
	cv::rectangle(ui, cv::Point(280, 100), cv::Point(1910, 340), cv::Scalar(200,200,200), -1);
	int num_of_messages = 4;
	spacing = 0;
	for (int i = 0; i < num_of_messages && i < debug_messages.size(); i++, spacing -= 50) {
		cv::putText(ui, debug_messages[debug_messages.size() - 1 - i], cv::Point(320, 300 + spacing), 
						cv::FONT_HERSHEY_SIMPLEX, FONT_SCALE_ * 2.5, cv::Scalar(0,0,0), 3, cv::LINE_AA);
	}

	// Targets box
	cv::rectangle(ui, cv::Point(20, 100), cv::Point(260, 340), cv::Scalar(220,220,220), -1);
	cv::rectangle(ui, cv::Point(20, 100), cv::Point(260, 340), cv::Scalar(110,110,110), 4);
	cv::putText(ui, "TARGETS", cv::Point(40, 150), cv::FONT_HERSHEY_SIMPLEX,
				FONT_SCALE_ * 3, cv::Scalar(0,0,0), 3, cv::LINE_AA);
	cv::putText(ui, std::to_string(drones_on_screen), cv::Point(80, 305), cv::FONT_HERSHEY_SIMPLEX,
				FONT_SCALE_ * 12, cv::Scalar(0,0,0), 15, cv::LINE_AA);

	// Diagnostics
	cv::rectangle(ui, cv::Point(1930, 20), cv::Point(3000, 340), cv::Scalar(200,200,200), -1);
	cv::putText(ui, "Resolution: " + std::to_string(frame_size.width) + " x " + std::to_string(frame_size.height), cv::Point(1950, 80), cv::FONT_HERSHEY_SIMPLEX,
				FONT_SCALE_ * 2.5, cv::Scalar(0,0,0), 3, cv::LINE_AA);
	cv::putText(ui, "Actual FPS: " + std::to_string(actual_fps).substr(0,4), cv::Point(1950, 140), cv::FONT_HERSHEY_SIMPLEX,
				FONT_SCALE_ * 2.5, cv::Scalar(0,0,0), 3, cv::LINE_AA);
	cv::putText(ui, "Frame Count: " + std::to_string(frame_count_), cv::Point(1950, 200), cv::FONT_HERSHEY_SIMPLEX,
				FONT_SCALE_ * 2.5, cv::Scalar(0,0,0), 3, cv::LINE_AA);

	time_t now = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
	std::string s(21, '\0');
	std::strftime(&s[0], s.size(), "%Y/%m/%d %H:%M:%S", std::localtime(&now));

	cv::putText(ui, "Current Time: " + s.substr(0,19), cv::Point(1950, 260), cv::FONT_HERSHEY_SIMPLEX,
				FONT_SCALE_ * 2.5, cv::Scalar(0,0,0), 3, cv::LINE_AA);

	cv::resize(ui, ui_, cv::Size(combined_frame.cols, ui_height * combined_frame.cols / ui_width), 0, 0, cv::INTER_CUBIC);


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