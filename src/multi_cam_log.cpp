/**
 * @file multi_cam_log.cpp
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
 * This file contains the definitions of the functions primarily used in the logging
 * of the image and data pipeline. It also contains the functions used in the user
 * interface.
 */

// mathematical constants
#define _USE_MATH_DEFINES

// opencv header files
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/ximgproc.hpp>

// local header files
#include "multi_cam_track.hpp"
#include "multi_cam_log.hpp"
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

	cv::VideoWriter recording_;

	std::ofstream frame_time_file, targets_2d_file, targets_3d_file;
	Json::StreamWriterBuilder builder;
	std::unique_ptr<Json::StreamWriter> writer(builder.newStreamWriter());

	cv::Mat ui_;

	// define debugging tools
	std::vector<std::vector<double>> lines;
	std::vector<std::string> debug_messages;

	// declare logging variables
	Json::Value detections_2d_(Json::arrayValue);
	Json::Value detections_3d_(Json::arrayValue);

	void initialize_recording(cv::Mat sample_frame, std::string filename) {
		
		// intialize video writer
		recording_ = cv::VideoWriter(filename, cv::VideoWriter::fourcc('M','P','4','V'), VIDEO_FPS_, 
			cv::Size(NUM_OF_CAMERAS_ * sample_frame.cols, sample_frame.rows + (360 * NUM_OF_CAMERAS_ * sample_frame.cols / 3840)));
	}

	void annotate_frames(std::array<std::shared_ptr<cv::Mat>, NUM_OF_CAMERAS_> frames_, std::array<std::shared_ptr<CameraTracks>, NUM_OF_CAMERAS_> cumulative_tracks_) {
	
		// draw tracks on opencv GUI to monitor the detected tracks
		// lopp through each camera frame
		for (int i = 0; i < NUM_OF_CAMERAS_; i++) {

			cv::putText(*frames_[i].get(), "CAM " + std::to_string(i), cv::Point(20, 30),
				cv::FONT_HERSHEY_SIMPLEX, FONT_SCALE_ * 1, cv::Scalar(255, 0, 0), 2, cv::LINE_AA);

			std::map<int, std::shared_ptr<TrackPlot>> display_tracks;

			if (NUM_OF_CAMERAS_ == 1) {
				display_tracks = cumulative_tracks_[i]->track_new_plots_;
			} else if (DISPLAY_MATCHED_ONLY_) {
				display_tracks = cumulative_tracks_[i]->track_plots_;
			} else {
				display_tracks.insert(cumulative_tracks_[i]->track_new_plots_.begin(), cumulative_tracks_[i]->track_new_plots_.end());
				display_tracks.insert(cumulative_tracks_[i]->track_plots_.begin(), cumulative_tracks_[i]->track_plots_.end());
			}
			
			// loop through every track plot
			for (auto & track : display_tracks) {
				
				// Display up to one second after lost
				if ((frame_count_ - track.second->lastSeen_) <= VIDEO_FPS_) {

					int id = track.second->id_;
					std::vector<int> xs = track.second->xs_;
					std::vector<int> ys = track.second->ys_;
					std::vector<int> size = track.second->size_;
					std::vector<double> xyz = track.second->xyz_;
					cv::Scalar color = (NUM_OF_CAMERAS_ == 1 || id != track.second->oid_) ? COLORS_[id % 10] : cv::Scalar(50, 50, 50);
					bool status = track.second->lastSeen_ == frame_count_;
					cv::Scalar status_color = status ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 0, 255);
					
					cv::Point2i rect_top_left(xs.back() - size.back(), ys.back() - size.back());
					cv::Point2i rect_bottom_right(xs.back() + size.back(), ys.back() + size.back());		
					
					cv::rectangle(*frames_[i].get(), rect_top_left, rect_bottom_right, color, 2);	
					
					bool annotate_flag = false;
					
					// get last frames up till plot history
					for(int idx = track.second->frameNos_.size() - 1; idx >= 0 
							&& track.second->frameNos_[idx] > (frame_count_ - PLOT_HISTORY_); idx--) {

						annotate_flag = true;

						int color_idx = track.second->frameNos_[idx] - frame_count_ + PLOT_HISTORY_ - 1;
						double alpha = 0.5 + (double) color_idx / 400;
						double beta = 1 - alpha;

						cv::Vec3b pixelColor = (*frames_[i].get()).at<cv::Vec3b>(ys[idx], xs[idx]);

						cv::circle(*frames_[i].get(), cv::Point(xs[idx], ys[idx]), 3,
							cv::Scalar((int) (pixelColor[0] * beta + (color[0] * alpha)),
										(int) (pixelColor[1] * beta + (color[1] * alpha)),
										(int) (pixelColor[2] * beta + (color[2] * alpha))), -1);
					}

					// put ID, status and XYZ coordinates on opencv GUI
					if (annotate_flag) {
						if (DISPLAY_ID_) {
							cv::putText(*frames_[i].get(), "ID: " + std::to_string(id), 
								cv::Point(rect_top_left.x + 20, rect_top_left.y - 5), cv::FONT_HERSHEY_SIMPLEX,
								FONT_SCALE_, color, 1, cv::LINE_AA);
						}
						if (!xyz.empty() && DISPLAY_3D_) {
							cv::putText(*frames_[i].get(), "X: " + std::to_string(xyz[0]).substr(0,4),
								cv::Point(rect_bottom_right.x + 10, rect_top_left.y + 10), cv::FONT_HERSHEY_SIMPLEX,
								FONT_SCALE_, color, 1, cv::LINE_AA);

							cv::putText(*frames_[i].get(), "Y: " + std::to_string(xyz[1]).substr(0,4),
								cv::Point(rect_bottom_right.x + 10, rect_top_left.y + 25), cv::FONT_HERSHEY_SIMPLEX,
								FONT_SCALE_, color, 1, cv::LINE_AA);

							cv::putText(*frames_[i].get(), "Z: " + std::to_string(xyz[2]).substr(0,4),
								cv::Point(rect_bottom_right.x + 10, rect_top_left.y + 40), cv::FONT_HERSHEY_SIMPLEX,
								FONT_SCALE_, color, 1, cv::LINE_AA);
						}
						if (DISPLAY_STATUS_) {
							cv::circle(*frames_[i].get(), cv::Point(rect_top_left.x + 5, rect_top_left.y - 10), 5, status_color, -1);
						}
					}
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

		for (auto & matched_track : matched_tracks_) {
			cv::putText(ui, "ID: " + std::to_string(matched_track.first), cv::Point(60 + spacing, 65), cv::FONT_HERSHEY_SIMPLEX,
				FONT_SCALE_ * 2.5, COLORS_[matched_track.first % 10], 3, cv::LINE_AA);
				spacing += 200;	
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
		cv::putText(ui, std::to_string(matched_tracks_.size()), cv::Point(80, 305), cv::FONT_HERSHEY_SIMPLEX,
					FONT_SCALE_ * 12, cv::Scalar(0,0,0), 15, cv::LINE_AA);

		// Get current time
		time_t now = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
		std::string s(21, '\0');
		std::strftime(&s[0], s.size(), "%Y/%m/%d %H:%M:%S", std::localtime(&now));
		// Diagnostics
		cv::rectangle(ui, cv::Point(1930, 20), cv::Point(3000, 340), cv::Scalar(200,200,200), -1);
		cv::putText(ui, "Resolution: " + std::to_string(frame_size.width) + " x " + std::to_string(frame_size.height), cv::Point(1950, 80), cv::FONT_HERSHEY_SIMPLEX,
					FONT_SCALE_ * 2.5, cv::Scalar(0,0,0), 3, cv::LINE_AA);
		cv::putText(ui, "Actual FPS: " + std::to_string(actual_fps).substr(0,4), cv::Point(1950, 140), cv::FONT_HERSHEY_SIMPLEX,
					FONT_SCALE_ * 2.5, cv::Scalar(0,0,0), 3, cv::LINE_AA);
		cv::putText(ui, "Frame Count: " + std::to_string(frame_count_), cv::Point(1950, 200), cv::FONT_HERSHEY_SIMPLEX,
					FONT_SCALE_ * 2.5, cv::Scalar(0,0,0), 3, cv::LINE_AA);
		cv::putText(ui, "Current Time: " + s.substr(0,19), cv::Point(1950, 260), cv::FONT_HERSHEY_SIMPLEX,
					FONT_SCALE_ * 2.5, cv::Scalar(0,0,0), 3, cv::LINE_AA);

		cv::resize(ui, ui_, cv::Size(combined_frame.cols, ui_height * combined_frame.cols / ui_width), 0, 0, cv::INTER_CUBIC);

	}

	void initialize_logs(std::string filename) {

		// initialize logs
		frame_time_file.open(FRAME_TIME_);
		frame_time_file << "detect_time, track_time, total_time" << "\n"; 
		
		targets_2d_file.open(filename);
		// targets_3d_file.open(TARGETS_3D_OUTPUT_);
	}

	/**
	 * This function logs the 2D coordinates for each of the cameras.
	 */
	void log_2D() {
		Json::Value frame_detections;
		frame_detections["Frame Number"] = frame_count_;
		for (int i = 0; i < NUM_OF_CAMERAS_; i++) {
			Json::Value detections(Json::arrayValue);

			for (auto & track : good_tracks_[i]) {
				Json::Value detection;
				detection["ID"] = track->id;
				int top_left_x = track->x - (track->size / 2);
				int top_left_y = track->y - (track->size / 2);
				int bottom_right_x = track->x + (track->size / 2);
				int bottom_right_y = track->y + (track->size / 2);
				std::string top_left = "(" + std::to_string(top_left_x) + ", " + std::to_string(top_left_y) + ")";
				std::string bottom_right = "(" + std::to_string(bottom_right_x) + ", " + std::to_string(bottom_right_y) + ")";
				detection["Top-Left"] = top_left;
				detection["Bottom-Right"] = bottom_right;
				detections.append(detection);
			}
			frame_detections["Cam " + std::to_string(i)] = detections;
		}
		detections_2d_.append(frame_detections);
	}

	/**
	 * This function logs the 3D coordinates for each of the cameras.
	 */
	void log_3D(std::shared_ptr<TrackPlot> track_plot_a, std::shared_ptr<TrackPlot> track_plot_b) {
			Json::Value detection;
			detection["ID"] = track_plot_a->id_;
			Json::Value xyz(Json::arrayValue);
			xyz.append(track_plot_a->xyz_[0]);
			xyz.append(track_plot_a->xyz_[1]);
			xyz.append(track_plot_a->xyz_[2]);
			detection["XYZ-Coordinates"] = xyz;
			detection["Geolocation"] = Json::Value("Placeholder Geolocation");
			time_t now = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
			std::string s(14, '\0');
			std::strftime(&s[0], s.size(), "%Y-%m-%d %H:%M:%S", std::localtime(&now));
			detection["Timestamp"] = s;
			detections_3d_.append(detection);
	}

	void print_frame_summary() {

		std::cout << "----------------------------------------------------------" << std::endl;
		std::cout << "SUMMARY OF FRAME " << frame_count_ << std::endl;
		std::cout << "----------------------------------------------------------" << std::endl;

		for (int i = 0; i < NUM_OF_CAMERAS_; i++) {
			std::cout << "Camera " << i << " New Tracks: ";
			for (auto it = cumulative_tracks_[i]->track_new_plots_.begin(); it != cumulative_tracks_[i]->track_new_plots_.end(); it++) {
				std::cout << "(" << it->first << ": " << it->second->id_ << ") | ";
			}
			std::cout << std::endl;
			std::cout << "Camera " << i << " Tracks: ";
			for (auto it = cumulative_tracks_[i]->track_plots_.begin(); it != cumulative_tracks_[i]->track_plots_.end(); it++) {
				std::cout << "(" << it->first << ": " << it->second->id_ << ") | ";
			}
			std::cout << std::endl;
			std::cout << "Camera " << i << " Matching: ";
			for (auto it = matching_dict_[i].begin(); it != matching_dict_[i].end(); it++) {
				std::cout << "(" << it->first << ": " << it->second << ") | ";
			}
			std::cout << std::endl;
		}

		std::cout << "Matched Tracks: ";
		for (auto it = matched_tracks_.begin(); it != matched_tracks_.end(); it++) {
			std::cout << "(" << it->first << ": [" << it->second[0] << ", " << it->second[1] << "]<" << it->second[2] << ">) | ";
		}
		std::cout << std::endl;

    }

}