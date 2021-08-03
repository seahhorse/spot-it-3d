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
 * This file contains the declarations and definitions of the functions 
 * primarily used in the tracking and re-identification pipeline. These functions 
 * interact with the key classes CameraTracks and TrackPlots which are essential 
 * to the tracking and re-identification process. These classes may be found 
 * at multi_cam_detect_utils.cpp.
 */

/**
 * @file mcmt_multi_tracker_node.hpp
 * @author Niven Sie, sieniven@gmail.com
 * @author Seah Shao Xuan
 * 
 * This code contains the McmtMultiTrackerNode class that runs our tracking and
 * re-identification process
 */

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

	// declare video parameters
	cv::VideoWriter recording_;

	// declare tracking variables
	int next_id_;
	std::array<std::shared_ptr<CameraTracks>, NUM_OF_CAMERAS_> cumulative_tracks_;
	std::array<int, NUM_OF_CAMERAS_> total_tracks_;
	std::array<std::map<int, int>, NUM_OF_CAMERAS_> matching_dict_;

	// declare tracking arrays
	std::array<std::shared_ptr<cv::Mat>, NUM_OF_CAMERAS_> frames_;
	std::array<std::vector<std::shared_ptr<GoodTrack>>, NUM_OF_CAMERAS_> good_tracks_, filter_good_tracks_;
	std::array<std::vector<int>, NUM_OF_CAMERAS_> dead_tracks_;
	
	// declare matched ids
	std::set<int> matched_ids_;

	// declare plotting parameters
	std::vector<int> shown_indexes_;

	// define debugging tools
	std::vector<std::vector<double>> lines;
	std::vector<std::string> debug_messages;

	// declare logging variables
	Json::Value detections_3d_(Json::arrayValue);

	// declare tracking functions	
	void initialize_tracks(cv::Mat sample_frame);
	void update_cumulative_tracks(int index, std::array<std::vector<std::shared_ptr<GoodTrack>>, NUM_OF_CAMERAS_> & good_tracks);
	void prune_tracks(int index);
	void verify_existing_tracks();
	void process_new_tracks(int index, int alt,
		std::array<std::vector<std::shared_ptr<GoodTrack>>, NUM_OF_CAMERAS_> & good_tracks,
		std::array<std::vector<std::shared_ptr<GoodTrack>>, NUM_OF_CAMERAS_> & filter_good_tracks,
		std::array<std::vector<int>, NUM_OF_CAMERAS_> & dead_tracks);
	void get_total_number_of_tracks();
	std::vector<double> normalise_track_plot(std::shared_ptr<TrackPlot> track_plot);
	double crossCorrelation(std::vector<double> X, std::vector<double> Y);
	double compute_matching_score(std::shared_ptr<TrackPlot> track_plot,
		std::shared_ptr<TrackPlot> alt_track_plot, int index, int alt);
	double geometric_similarity(std::vector<std::shared_ptr<TrackPlot::OtherTrack>> & other_tracks_0,
		std::vector<std::shared_ptr<TrackPlot::OtherTrack>> & other_tracks_1);
	double geometric_similarity_cartesian(std::vector<std::shared_ptr<TrackPlot::OtherTrack>> & other_tracks_0,
		std::vector<std::shared_ptr<TrackPlot::OtherTrack>> & other_tracks_1);
	double heading_error(std::shared_ptr<TrackPlot> track_plot, 
		std::shared_ptr<TrackPlot> alt_track_plot, int history);
	void generate_matched_ids();
	void calculate_3D();
	void log_3D();
	void print_frame_summary();
	void imshow_resized_dual(std::string & window_name, cv::Mat & img);
	int encoding2mat_type(const std::string & encoding);

	void initialize_tracks(cv::Mat sample_frame) {

		// intialize video writer;
		recording_ = cv::VideoWriter(VIDEO_OUTPUT_ANNOTATED_, cv::VideoWriter::fourcc('M','P','4','V'), VIDEO_FPS_, 
			cv::Size(NUM_OF_CAMERAS_ * sample_frame.cols, sample_frame.rows));

		// initialize matched track ids
		next_id_ = 0;

		// initialize cumulative camera tracks
		cumulative_tracks_[0] = std::shared_ptr<CameraTracks>(new CameraTracks(0));
		cumulative_tracks_[1] = std::shared_ptr<CameraTracks>(new CameraTracks(1));

	}

	/**
	 * this function creates new tracks and the addition to the cumulative tracks log for each frame
	 */
	void update_cumulative_tracks(int index,
		std::array<std::vector<std::shared_ptr<GoodTrack>>, NUM_OF_CAMERAS_> & good_tracks) {
			
		int track_id, centroid_x, centroid_y, size;

		for (auto & track : good_tracks[index]) {
			
			// Extract details from the track
			track_id = track->id;
			centroid_x = track->x;
			centroid_y = track->y;
			size = track->size;

			// occurance of a new track
			if (matching_dict_[index].find(track_id) == matching_dict_[index].end()) {
				cumulative_tracks_[index]->track_new_plots_[track_id] = std::shared_ptr<TrackPlot>(
					new TrackPlot(track_id));
				matching_dict_[index][track_id] = track_id;
				
			}

			std::vector<int> location;
			location.push_back(centroid_x);
			location.push_back(centroid_y);

			std::shared_ptr<TrackPlot> track_plot;
			if (cumulative_tracks_[index]->track_plots_.find(matching_dict_[index][track_id]) 
				== cumulative_tracks_[index]->track_plots_.end()) {
				track_plot = cumulative_tracks_[index]->track_new_plots_[track_id];			
			} else {
				track_plot = cumulative_tracks_[index]->track_plots_[matching_dict_[index][track_id]];
			}
			// Update track_new_plots with centroid and feature variable of every new frame
			track_plot->update(location, size, frame_count_);
			track_plot->calculate_track_feature_variable(frame_count_, VIDEO_FPS_);
		}			

	}

	/**
	 * this function removes dead tracks if they have not appeared for more than 300 frames
	 */
	void prune_tracks(int index) {
		
		std::vector<int> prune;
		// prune dead tracks if they have not appeared for more than 300 frames
		for (auto & track : cumulative_tracks_[index]->track_new_plots_) {
			if ((frame_count_ - track.second->lastSeen_) > 60) {
				prune.push_back(track.second->id_);
			}
		}

		for (auto & track_id : prune) {
			cumulative_tracks_[index]->track_new_plots_.erase(track_id);
			matching_dict_[index].erase(track_id);
		}
	}

	/**
	 * checks matched tracks to see if they are still valid. also checks if multiple tracks
	 * within each camera are tracking the same target
	 */
	void verify_existing_tracks() {

		int original_track_id_0, original_track_id_1;

		for (auto & matched_id : matched_ids_) {

			std::shared_ptr<TrackPlot> track_plot_0 = cumulative_tracks_[0]->track_plots_[matched_id];
			std::shared_ptr<TrackPlot> track_plot_1 = cumulative_tracks_[1]->track_plots_[matched_id];

			// normalization of cross correlation values
			std::vector<double> track_plot_normalize_xj = normalise_track_plot(track_plot_0);
			std::vector<double> alt_track_plot_normalize_xj = normalise_track_plot(track_plot_1);

			if (track_plot_normalize_xj.size() > 120) {
				std::vector<double> track_plot_normalize_xj_trunc(track_plot_normalize_xj.end() - 120, track_plot_normalize_xj.end());
				track_plot_normalize_xj = track_plot_normalize_xj_trunc;
			}
			if (alt_track_plot_normalize_xj.size() > 120) {
				std::vector<double> alt_track_plot_normalize_xj_trunc(alt_track_plot_normalize_xj.end() - 120, alt_track_plot_normalize_xj.end());
				alt_track_plot_normalize_xj = alt_track_plot_normalize_xj_trunc;
			}

			// track feature variable correlation strength
			auto r_value = crossCorrelation(track_plot_normalize_xj, alt_track_plot_normalize_xj);

			// heading deviation error score
			double heading_err = heading_error(track_plot_0, track_plot_1, 30);

			// std::vector<double> line{track_plot_0->xs_.back(), track_plot_0->ys_.back(), track_plot_1->xs_.back() + 1920, track_plot_1->ys_.back(), r_value, r_value, r_value};
			// lines.push_back(line);
			

			if (r_value < 0.4 && heading_err > 0.2 && track_plot_0->frameNos_.size() > 180 && track_plot_1->frameNos_.size() > 180) {
					if (track_plot_0->check_stationary() != track_plot_1->check_stationary()) {
						track_plot_0->mismatch_count_ += 4;
						track_plot_1->mismatch_count_ += 4;
					} else {
						track_plot_0->mismatch_count_ += 1;
						track_plot_1->mismatch_count_ += 1;
					}
				
			} else {
				track_plot_0->mismatch_count_ = 0;
				track_plot_1->mismatch_count_ = 0;
			}

			if (track_plot_0->mismatch_count_ >= 120 && track_plot_1->mismatch_count_ >= 120) {
				track_plot_0->mismatch_count_ = 0;
				track_plot_1->mismatch_count_ = 0;

				debug_messages.push_back("Target ID " +  std::to_string(track_plot_0->id_)  + " is dropped due to mismatch ");

				for (auto it = matching_dict_[0].begin(); it != matching_dict_[0].end(); it++) {
					if (it->second == track_plot_0->id_) {
						original_track_id_0 = it->first;
						break;
					}
				}

				for (auto it = matching_dict_[1].begin(); it != matching_dict_[1].end(); it++) {
					if (it->second == track_plot_1->id_) {
						original_track_id_1 = it->first;
						break;
					}
				}

				track_plot_0->id_ = original_track_id_0;
				track_plot_1->id_ = original_track_id_1;

				cumulative_tracks_[0]->track_new_plots_[original_track_id_0] = track_plot_0;
				cumulative_tracks_[0]->track_plots_.erase(matched_id);

				cumulative_tracks_[1]->track_new_plots_[original_track_id_1] = track_plot_1;
				cumulative_tracks_[1]->track_plots_.erase(matched_id);

				matching_dict_[0][original_track_id_0] = original_track_id_0;
				matching_dict_[1][original_track_id_1] = original_track_id_1;
			}
			
		}

	}

	void process_new_tracks(
		int index, int alt,
		std::array<std::vector<std::shared_ptr<GoodTrack>>, NUM_OF_CAMERAS_> & good_tracks,
		std::array<std::vector<std::shared_ptr<GoodTrack>>, NUM_OF_CAMERAS_> & filter_good_tracks,
		std::array<std::vector<int>, NUM_OF_CAMERAS_> & dead_tracks) {
		
		get_total_number_of_tracks();
		std::map<int, std::map<int, double>> corrValues;
		std::set<int> removeSet;
		int track_id, centroid_x, centroid_y, size;
		int row = 0;

		for (auto & track : good_tracks[index]) {

			// Extract details from the track
			track_id = track->id;

			if (cumulative_tracks_[index]->track_plots_.find(matching_dict_[index][track_id]) 
				== cumulative_tracks_[index]->track_plots_.end()) {

				auto track_plot = cumulative_tracks_[index]->track_new_plots_[track_id];			

				// Check if track feature variable has non-zero elements
				double sum = 0;
				for (int i = 0; i < track_plot->track_feature_variable_.size(); i++) {
					sum += track_plot->track_feature_variable_[i];
				}

				// if track is not a new track, we use 90 frames as the minimum requirement before matching occurs
				if (track_plot->frameNos_.size() >= 30 && track_plot->track_feature_variable_.size() >= 30 && sum != 0) {

					// look into 2nd camera's new tracks (new tracks first)
					std::map<int, std::shared_ptr<TrackPlot>>::iterator alt_track_plot;
					for (alt_track_plot = cumulative_tracks_[alt]->track_new_plots_.begin();
						alt_track_plot != cumulative_tracks_[alt]->track_new_plots_.end(); alt_track_plot++) {
						sum = 0;
						for (int i = 0; i < static_cast<int>(alt_track_plot->second->track_feature_variable_.size()); i++) {
							sum += alt_track_plot->second->track_feature_variable_[i];
						}
						// track in 2nd camera must fulfills requirements to have at least 90 frames to match
						if (alt_track_plot->second->frameNos_.size() >= 30 && alt_track_plot->second->track_feature_variable_.size() >= 30 
							&& sum != 0) {
							double score = compute_matching_score(track_plot, alt_track_plot->second, index, alt);
							if (score != 0)	{
								corrValues[track_id][alt_track_plot->second->id_] = score;
							}
						}
					}

					// look into other camera's matched tracks list (old tracks last)
					for (alt_track_plot = cumulative_tracks_[alt]->track_plots_.begin();
						alt_track_plot != cumulative_tracks_[alt]->track_plots_.end(); alt_track_plot++) {

						bool eligibility_flag = true;
						
						// do not consider dead tracks from the other camera
						for (auto & dead_track : dead_tracks[alt]) {
							if (matching_dict_[alt].find(dead_track) != matching_dict_[alt].end() && matching_dict_[alt][dead_track] == alt_track_plot->second->id_) {
								eligibility_flag = false; // 2nd camera's track has already been lost. skip the process of matching for this track
							}
						}
												
						// test to see if alternate camera's track is currently being matched with current camera                        
						for (auto & alt_track : good_tracks[index])	{
							if (alt_track_plot->second->id_ == matching_dict_[index][alt_track->id]) {
								eligibility_flag = false; // 2nd camera's track has already been matched. skip the process of matching for this track
							}
						}

						sum = 0;
						for (int i = 0; i < static_cast<int>(alt_track_plot->second->track_feature_variable_.size()); i++)	{
							sum += alt_track_plot->second->track_feature_variable_[i];
						}

						if (eligibility_flag && sum != 0) {
							double score = compute_matching_score(track_plot, alt_track_plot->second, index, alt);
							if (score != 0)
							{
								corrValues[track_id][alt_track_plot->second->id_] = score;
							}
						}
					}
				}

				row += 1;

			} else {
				filter_good_tracks[index].erase(filter_good_tracks[index].begin() + row);
			}

		}


		for (auto & track : filter_good_tracks[index]) {
			std::map<int, double> maxValues = corrValues[track->id];
			int maxID = -1;
			double maxValue = -1;
			int global_max_flag = 0;

			// for the selected max track in the 2nd camera, we check to see if the track has a higher
			// cross correlation value with another track in current camera

			while (global_max_flag == 0 && maxValues.size() != 0) {
				for (auto it = maxValues.begin(); it != maxValues.end(); it++) {
					if (maxValue < it->second) {
						maxID = it->first;
					}
				}
				maxValue = maxValues[maxID];

				// search through current camera's tracks again, for the selected track that we wish to re-id with.
				// we can note that if there is a track in the current camera that has a higher cross correlation value
				// than the track we wish to match with, then the matching will not occur.
				for (auto & track_1 : filter_good_tracks[index]) {
					if (corrValues[track_1->id].find(maxID) != corrValues[track_1->id].end()) {
						if (corrValues[track_1->id][maxID] > maxValue) {
							maxValues.erase(maxID);
							global_max_flag = 1;
							break;
						}
					}
				}

				if (global_max_flag == 1) {
					// there existed a value larger than the current maxValue. thus, re-id cannot occur
					global_max_flag = 0;
					continue;
				} else {
					// went through the whole loop without breaking, thus it is the maximum value. re-id can occur
					global_max_flag = 2;
				}
			}

			// re-id process
			if (global_max_flag == 2) {

				// if track is in 2nd camera's new track list
				if (maxID != 1 && 
					(cumulative_tracks_[alt]->track_new_plots_.find(maxID) != cumulative_tracks_[alt]->track_new_plots_.end())) {
					// add notification message
					debug_messages.push_back("New target ID " +  std::to_string(next_id_)  + " acquired with a score of " + std::to_string(maxValue));

					// remove track plot in new tracks' list and add into matched tracks' list for alternate camera
					cumulative_tracks_[alt]->track_new_plots_[maxID]->id_ = next_id_;
					cumulative_tracks_[alt]->track_plots_.insert(
						std::pair<int, std::shared_ptr<TrackPlot>>(next_id_, cumulative_tracks_[alt]->track_new_plots_[maxID]));
					// update dictionary matching
					matching_dict_[alt][maxID] = next_id_;
					removeSet.insert(maxID);

					// remove track plot in new tracks' list and add into matched tracks' list for current camera
					int track_id = track->id;
					auto track_plot = cumulative_tracks_[index]->track_new_plots_[track_id];
					track_plot->id_ = next_id_;

					cumulative_tracks_[index]->track_plots_.insert({next_id_, track_plot});
					cumulative_tracks_[index]->track_new_plots_.erase(track_id);

					// update dictionary matching list
					matching_dict_[index][track_id] = next_id_;
					next_id_ += 1;

				// if track is in 2nd camera's matched track list
				} else {
					int track_id = track->id;
					auto track_plot = cumulative_tracks_[index]->track_new_plots_[track_id];
					track_plot->id_ = cumulative_tracks_[alt]->track_plots_[maxID]->id_;

					// add notification message
					debug_messages.push_back("New target ID " +  std::to_string(track_plot->id_)  + " acquired with a score of " + std::to_string(maxValue));

					// update track plot in the original track ID
					combine_track_plots(track_plot->id_, cumulative_tracks_[index], track_plot, frame_count_);

					// update dictionary matching list
					for (std::map<int, int>::iterator old_id = matching_dict_[index].begin(); old_id != matching_dict_[index].end(); old_id++)
					{
						if (old_id->second == track_plot->id_)
						{
							old_id->second = old_id->first;
							cumulative_tracks_[index]->track_new_plots_[old_id->first] = cumulative_tracks_[index]->track_plots_[track_plot->id_];
							cumulative_tracks_[index]->track_new_plots_[old_id->first]->id_ = old_id->first;
							break;
						}
					}

					// remove track plot in new tracks' list
					cumulative_tracks_[index]->track_plots_[track_plot->id_] = track_plot;
					cumulative_tracks_[index]->track_new_plots_.erase(track_id);
					matching_dict_[index][track_id] = track_plot->id_;

				}
			}
		}

		for (auto & remove_id : removeSet) {
			cumulative_tracks_[alt]->track_new_plots_.erase(remove_id);
		}
	}

	/**
	 * Check for IDs that belong to both cumulative tracks 0 and 1
	 */ 
	void generate_matched_ids() {

		matched_ids_.clear();
		for (auto i = cumulative_tracks_[0]->track_plots_.begin(); i != cumulative_tracks_[0]->track_plots_.end(); i++){
			for (auto j = cumulative_tracks_[1]->track_plots_.begin(); j != cumulative_tracks_[1]->track_plots_.end(); j++){
				if ((i->second->lastSeen_ == frame_count_) && (j->second->lastSeen_ == frame_count_) && (i->first == j->first)) {
					matched_ids_.insert(i->first);
					break;
				}
			}
		}

	}

	void get_total_number_of_tracks() {
		total_tracks_[0] = cumulative_tracks_[0]->track_new_plots_.size() + 
			cumulative_tracks_[0]->track_plots_.size();
		total_tracks_[1] = cumulative_tracks_[1]->track_new_plots_.size() + 
			cumulative_tracks_[1]->track_plots_.size();
	}

	/**
	 * Normalises the existing track plot based on mean and sd
	 */
	std::vector<double> normalise_track_plot(std::shared_ptr<TrackPlot> track_plot) {

		int total_track_feature = track_plot->track_feature_variable_.size();
		double mean = 0, variance = 0, std;
		std::vector<double> result;

		// Mean
		for (int i = 0; i < total_track_feature; i++){
			mean += track_plot->track_feature_variable_[i];
		}
		mean = mean / total_track_feature;

		// Variance and stdev
		for (int i = 0; i < total_track_feature; i++){
			variance += pow(track_plot->track_feature_variable_[i] - mean, 2);
		}
		variance = variance / total_track_feature;
		std = sqrt(variance);

		// Normalise
		for (int i = 0; i < total_track_feature; i++){
			double res = (track_plot->track_feature_variable_[i] - mean) / (std * sqrt(total_track_feature));
			result.push_back(res);
		}

		return result;
	}

	double compute_matching_score(std::shared_ptr<TrackPlot> track_plot,
			std::shared_ptr<TrackPlot> alt_track_plot, int index, int alt) {
				
		// Normalization of cross correlation values
		auto track_plot_normalize_xj = normalise_track_plot(track_plot);
		auto alt_track_plot_normalize_xj = normalise_track_plot(alt_track_plot);

		// Updating of tracks in the local neighbourhood
		update_other_tracks(track_plot, cumulative_tracks_[index]);
		update_other_tracks(alt_track_plot, cumulative_tracks_[alt]);

		// Track feature variable correlation strength
		auto r_value = crossCorrelation(track_plot_normalize_xj, alt_track_plot_normalize_xj);

		// Geometric track matching strength value
		double geometric_strength = geometric_similarity(track_plot->other_tracks_, alt_track_plot->other_tracks_);
		double geometric_strength_c = geometric_similarity_cartesian(track_plot->other_tracks_, alt_track_plot->other_tracks_);

		// Heading deviation error value
		int track_history = 30;
		double heading_err = heading_error(track_plot, alt_track_plot, track_history);

		double w1 = 0.3;
		double w2 = 0.4;
		double w3 = 0.3;
		double score = (w1 * r_value) + (w2 * geometric_strength_c) + (w3 * (1 - heading_err));

		// if (index == 0) {
		// 	std::vector<double> line{track_plot->xs_.back(), track_plot->ys_.back(), alt_track_plot->xs_.back() + 1920, alt_track_plot->ys_.back(), score, r_value, r_value_2};
		// 	lines.push_back(line);
		// }

		if (r_value > 0.4 && (geometric_strength_c == 0 || geometric_strength_c >= 0.5) && heading_err < 0.1 && score >= 0.75){
			return score;
		}
		else {
			return 0;
		}
	}

	/**
	 * Find cross correlation of two 1D arrays with size n
	 * Involves the convolution of array X with array Y by sliding std::vector Y from left to right
	 */
	double crossCorrelation(std::vector<double> X, std::vector<double> Y)	{

			double max = 0;
			std::vector<double> A;
			std::vector<double> K;

			if (X.size() >= Y.size()) {
				A = X;
				K = Y;
			} else {
				A = Y;
				K = X;
			}
			
			for (int i = 1; i <= A.size() + K.size() - 1; i++) {
				double sum = 0;

				// Kernel is outside (to the left of) the array
				if (i <= K.size() - 1) {
					for (int k = K.size() - i; k < K.size(); k++) {
						sum += K[k] * A[i + k - K.size()];
					}
				// Kernel is outside (to the left of) the array
				} else if (i >= A.size() + 1) {
					for (int k = 0; k < A.size() + K.size() - i; k++) {
						sum += K[k] * A[i + k - K.size()];
					}
				// Kernel is fully within the array
				} else {
					for (int k = 0; k < K.size(); k++) {
						sum += K[k] * A[i + k - K.size()];
					}
				}
				// Only keep the peak cross-correlation
				if (sum > max) {
					max = sum;
				}			
			}

			return max;
	}

	double geometric_similarity(
		std::vector<std::shared_ptr<TrackPlot::OtherTrack>> & other_tracks_0, 
		std::vector<std::shared_ptr<TrackPlot::OtherTrack>> & other_tracks_1)	{

		std::vector<double> relative_distances, shortest_distances;

		int total_num_other_tracks_0 = other_tracks_0.size();
		int total_num_other_tracks_1 = other_tracks_1.size();

		for (int i = 0; i < total_num_other_tracks_0; i++){
			double a_angle = other_tracks_0[i]->angle;
			double a_dist = other_tracks_0[i]->dist;

			relative_distances.clear();
			for (int j = 0; j < total_num_other_tracks_1; j++){
				double b_angle = other_tracks_1[j]->angle;
				double b_dist = other_tracks_1[j]->dist;

				relative_distances.push_back((std::min<double>(std::abs(a_angle - b_angle),
					(2 * M_PI) - std::abs(a_angle - b_angle))) / M_PI * 
					std::min<double>(a_dist / b_dist, b_dist / a_dist));
			}

			int total_num_relative_distances = relative_distances.size();
			if (total_num_relative_distances > 0){
				double minimum = relative_distances.front();
				for (int i = 0; i < total_num_relative_distances; i++){
					if (relative_distances[i] < minimum){
						minimum = relative_distances[i];
					}
				}
				shortest_distances.push_back(minimum);
			}
		}

		int total_num_shortest_distances = shortest_distances.size();
		if (total_num_shortest_distances > 0) {
			// Get average value of shortest_distances
			double avg_shortest_distances = 0;
			for (int i = 0; i < total_num_shortest_distances; i++){
				avg_shortest_distances += shortest_distances[i];
			}
			avg_shortest_distances = avg_shortest_distances / total_num_shortest_distances;
			return std::max<double>(0.001, 0.2 - avg_shortest_distances) * 5;
		} else {
			return 0;
		}

	}

	double geometric_similarity_cartesian(
		std::vector<std::shared_ptr<TrackPlot::OtherTrack>> & other_tracks_0, 
		std::vector<std::shared_ptr<TrackPlot::OtherTrack>> & other_tracks_1)	{

		std::vector<double> relative_distances, shortest_distances;

		int total_num_other_tracks_0 = other_tracks_0.size();
		int total_num_other_tracks_1 = other_tracks_1.size();

		for (int i = 0; i < total_num_other_tracks_0; i++){
			int a_dx = other_tracks_0[i]->dx;
			int a_dy = other_tracks_0[i]->dy;

			relative_distances.clear();
			for (int j = 0; j < total_num_other_tracks_1; j++){
				int b_dx = other_tracks_1[j]->dx;
				int b_dy = other_tracks_1[j]->dy;

				double relative_distance = hypot(std::abs(a_dx - b_dx), std::abs(a_dy - b_dy)) / 1920;
				relative_distances.push_back(relative_distance);
			}

			int total_num_relative_distances = relative_distances.size();
			if (total_num_relative_distances > 0){
				double minimum = relative_distances.front();
				for (int i = 0; i < total_num_relative_distances; i++){
					if (relative_distances[i] < minimum){
						minimum = relative_distances[i];
					}
				}
				shortest_distances.push_back(minimum);
			}
		}

		int total_num_shortest_distances = shortest_distances.size();
		if (total_num_shortest_distances > 0) {
			// Get average value of shortest_distances
			double avg_shortest_distances = 0;
			for (int i = 0; i < total_num_shortest_distances; i++){
				avg_shortest_distances += shortest_distances[i];
			}
			avg_shortest_distances /= total_num_shortest_distances;
			return std::max<double>(0.001, 0.2 - avg_shortest_distances) * 5;
		} else {
			return 0;
		}

	}

	double heading_error(std::shared_ptr<TrackPlot> track_plot, 
		std::shared_ptr<TrackPlot> alt_track_plot, int history) {
		
		double deviation = 0;
		int dx_0 = track_plot->xs_.back() - track_plot->xs_[track_plot->xs_.size() - 2];
		int dy_0 = track_plot->ys_.back() - track_plot->ys_[track_plot->ys_.size() - 2];
		double rotation_0 = (atan2((double) dy_0, (double) dx_0) + M_PI) / (2 * M_PI);

		int dx_1 = alt_track_plot->xs_.back() - alt_track_plot->xs_[alt_track_plot->xs_.size() - 2];
		int dy_1 = alt_track_plot->ys_.back() - alt_track_plot->ys_[alt_track_plot->ys_.size() - 2];
		double rotation_1 = (atan2((double) dy_1, (double) dx_1) + M_PI) / (2 * M_PI);

		for (int i = -2; i > 1-history; i--) {
			dx_0 = track_plot->xs_[track_plot->xs_.size() - 1 + i] - track_plot->xs_[track_plot->xs_.size() - 2 + i];
			dy_0 = track_plot->ys_[track_plot->ys_.size() - 1 + i] - track_plot->ys_[track_plot->ys_.size() - 2 + i];
			double angle_0 = (atan2((double) dy_0, (double) dx_0) + M_PI) / (2 * M_PI);

			dx_1 = alt_track_plot->xs_[alt_track_plot->xs_.size() - 1 + i] - alt_track_plot->xs_[alt_track_plot->xs_.size() - 2 + i];
			dy_1 = alt_track_plot->ys_[alt_track_plot->ys_.size() - 1 + i] - alt_track_plot->ys_[alt_track_plot->ys_.size() - 2 + i];
			double angle_1 = (atan2((double) dy_1, (double) dx_1) + M_PI) / (2 * M_PI);

			double relative_0 = angle_0 - rotation_0;
			double relative_1 = angle_1 - rotation_1;

			if (relative_0 < 0) {
				relative_0 += 1;
			}

			if (relative_1 < 0) {
				relative_1 += 1;
			}

			deviation += std::min(std::abs(relative_0 - relative_1), 1 - std::abs(relative_0 - relative_1));

		}
			return (deviation / (history - 1));
	}

	/**
	 * Computes the 3D position of a matched drone through triangulation methods.
	 */
	void calculate_3D()	{

		double fx = 1454.6;
		double cx = 960.9;
		double fy = 1450.3;
		double cy = 543.7;
		double B = 1.5;
		int epsilon = 7;

		for (std::set<int>::iterator it = matched_ids_.begin(); it != matched_ids_.end(); it++) {
			auto track_plot_0 = cumulative_tracks_[0]->track_plots_[*it];
			auto track_plot_1 = cumulative_tracks_[1]->track_plots_[*it];

			if ((track_plot_0->lastSeen_ == frame_count_) && (track_plot_1->lastSeen_ == frame_count_)){
				int x_L = track_plot_0->xs_.back();
				int y_L = track_plot_0->ys_.back();
				int x_R = track_plot_1->xs_.back();
				int y_R = track_plot_1->ys_.back();

				double alpha_L = atan2(x_L - cx, fx) / M_PI * 180;
				double alpha_R = atan2(x_R - cx, fx) / M_PI * 180;

				double Z = B / (tan((alpha_L + epsilon / 2) * (M_PI / 180)) - tan((alpha_L - epsilon / 2) * (M_PI / 180)));
				double X = (Z * tan((alpha_L + epsilon / 2) * (M_PI / 180)) - B / 2
							+ Z * tan((alpha_R - epsilon / 2) * (M_PI / 180)) + B / 2) / 2;
				double Y = (Z * - (y_L - cy) / fy + Z * - (y_R - cy) / fy) / 2;

				double tilt = 10 * M_PI / 180;
				Eigen::Matrix3d R;
				R << 1, 0, 0,
					0, cos(tilt), sin(tilt),
					0, -sin(tilt), cos(tilt);
				Eigen::Vector3d XYZ_original;
				XYZ_original << X, Y, Z;
				auto XYZ = R * XYZ_original;
				X = XYZ(0);
				Y = XYZ(1);
				Z = XYZ(2);

				Y += 1;

				X = (std::round(X*100))/100;
				Y = (std::round(Y*100))/100;
				Z = (std::round(Z*100))/100;

				track_plot_0->xyz_.clear();
				track_plot_0->xyz_.push_back(X);
				track_plot_0->xyz_.push_back(Y);
				track_plot_0->xyz_.push_back(Z);
				track_plot_1->xyz_.clear();
				track_plot_1->xyz_.push_back(X);
				track_plot_1->xyz_.push_back(Y);
				track_plot_1->xyz_.push_back(Z);

			} else {
				track_plot_0->xyz_.clear();
				track_plot_1->xyz_.clear();
			}
		}
	}

	void log_3D() {
		Json::Value detection(Json::arrayValue);

		for (std::set<int>::iterator it = matched_ids_.begin(); it != matched_ids_.end(); it++) {
				
			auto track_plot_0 = cumulative_tracks_[0]->track_plots_[*it];
			auto track_plot_1 = cumulative_tracks_[1]->track_plots_[*it];

			Json::Value detection;
			detection["ID"] = *it;
			Json::Value xyz(Json::arrayValue);
			xyz.append(track_plot_0->xyz_[0]);
			xyz.append(track_plot_0->xyz_[1]);
			xyz.append(track_plot_0->xyz_[2]);
			detection["XYZ-Coordinates"] = xyz;
			detection["Geolocation"] = Json::Value("Placeholder Geolocation");
			time_t now = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
			std::string s(19, '\0');
			std::strftime(&s[0], s.size(), "%Y-%m-%d %H:%M:%S", std::localtime(&now));
			detection["Timestamp"] = s;
			detections_3d_.append(detection);
		}
	}

	void print_frame_summary() {

		std::cout << "SUMMARY OF FRAME " << frame_count_ << std::endl;

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
    }

	void imshow_resized_dual(std::string & window_name, cv::Mat & img) {

		cv::Size img_size = img.size();

		double aspect_ratio = img_size.width / img_size.height;

		cv::Size window_size;
		window_size.width = 1920;
		window_size.height = 1920 / aspect_ratio;
		
		cv::resize(img, img, window_size, 0, 0, cv::INTER_CUBIC);
		cv::imshow(window_name, img);
	}

}

#endif			// MULTI_CAM_TRACK_HPP_