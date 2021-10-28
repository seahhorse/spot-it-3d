/**
 * @file multi_cam_track.cpp
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
 * This file contains the definitions of the functions primarily used in the tracking 
 * and re-identification pipeline. These functions interact with the key classes
 * CameraTracks and TrackPlots which are essential to the tracking and 
 * re-identification process. These classes may be found at multi_cam_detect_utils.cpp.
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

	// declare tracking variables
	int next_id_ = 0;
	std::array<std::shared_ptr<CameraTracks>, NUM_OF_CAMERAS_> cumulative_tracks_;
	std::array<std::map<int, int>, NUM_OF_CAMERAS_> matching_dict_;
	std::map<int, std::array<int, NUM_OF_CAMERAS_ + 1>> matched_tracks_;

	// declare tracking arrays
	std::array<std::shared_ptr<cv::Mat>, NUM_OF_CAMERAS_> frames_;
	std::array<std::vector<std::shared_ptr<GoodTrack>>, NUM_OF_CAMERAS_> good_tracks_;

	/**
	 * This function initializes the new tracks for subsequent addition
	 */
	void initialize_tracks(cv::Mat sample_frame) {
		for (int cam_idx = 0; cam_idx < NUM_OF_CAMERAS_; cam_idx++) {
			cumulative_tracks_[cam_idx] = std::shared_ptr<CameraTracks>(new CameraTracks(cam_idx));
		}
	}

	/**
	 * This function creates new tracks and the addition to the cumulative tracks log for each frame
	 */
	void update_cumulative_tracks(int index, std::vector<std::shared_ptr<GoodTrack>> & good_tracks) {
			
		for (auto & track : good_tracks) {
			
			// Extract details from the track
			int track_id = track->id;
			int size = track->size;
			std::vector<int> location{track->x, track->y};
			std::shared_ptr<TrackPlot> track_plot;

			// Check for location of track (either in track_new_plots_ or track_plots_ or neither)
			if (exists(cumulative_tracks_[index]->track_new_plots_, track_id)) {
				// Track is in track_new_plots_, so we extract it using the track_id
				track_plot = cumulative_tracks_[index]->track_new_plots_[track_id];
			} else if (exists(matching_dict_[index], track_id)) {
				// Track is in track_plots_, so we find the matched_id to extract it
				track_plot = cumulative_tracks_[index]->track_plots_[matching_dict_[index][track_id]];
			} else {
				// Track is new, so we create a new TrackPlot and add it to track_new_plots
				cumulative_tracks_[index]->track_new_plots_[track_id] = std::shared_ptr<TrackPlot>(
					new TrackPlot(track_id));
				matching_dict_[index][track_id] = track_id;
				track_plot = cumulative_tracks_[index]->track_new_plots_[track_id];
			}

			// Update the TrackPlot
			track_plot->update(location, size, frame_count_);
		}			
	}

	/**
	 * This function removes dead tracks if they have not appeared for more than 30 frames
	 */
	void prune_tracks(int index) {
		
		std::vector<int> prune;
		
		// Clean the new tracks first
		prune.clear();
		for (auto & track : cumulative_tracks_[index]->track_new_plots_) {
			if ((frame_count_ - track.second->lastSeen_) > PRUNE_HIST_) {
				prune.push_back(track.second->id_);
			}
		}

		// Remove new tracks if they are older than PRUNE_HIST_
		for (auto & track_id : prune) {
			cumulative_tracks_[index]->track_new_plots_.erase(track_id);
			matching_dict_[index].erase(track_id);
		}

		// Clean the matched tracks last
		prune.clear();
		for (auto & track : cumulative_tracks_[index]->track_plots_) {
			if ((frame_count_ - track.second->lastSeen_) > PRUNE_HIST_) {
				prune.push_back(track.second->oid_);
			}
		}
		
		// Remove new tracks if they are older than PRUNE_HIST_
		for (auto & track_id : prune) {
			cumulative_tracks_[index]->track_plots_.erase(matching_dict_[index][track_id]);
			matched_tracks_[matching_dict_[index][track_id]][index] = 0;
			matched_tracks_[matching_dict_[index][track_id]][NUM_OF_CAMERAS_]--;

			// Update the matched_tracks_ map and remove if all tracks are gone
			if (!matched_tracks_[matching_dict_[index][track_id]][NUM_OF_CAMERAS_]) {
				matched_tracks_.erase(matching_dict_[index][track_id]);
			}
			matching_dict_[index].erase(track_id);
		}
	}

	/**
	 * checks matched tracks to see if they are still valid. also checks if multiple tracks
	 * within each camera are tracking the same target
	 */
	void verify_existing_tracks(int idx_a, int idx_b) {

		for (auto & matched_track : matched_tracks_) {		
			if (matched_track.second[NUM_OF_CAMERAS_] < 2 )	continue;

			int matched_id = matched_track.first;

			auto track_plot_a = cumulative_tracks_[idx_a]->track_plots_[matched_id];
			auto track_plot_b = cumulative_tracks_[idx_b]->track_plots_[matched_id];

			if (track_plot_a->lastSeen_ != frame_count_ || track_plot_b->lastSeen_ != frame_count_) continue;

			// x1 : track feature variable correlation strength based on normalised vector
			double x1 = crossCorrelation(normalise_track_plot(track_plot_a), normalise_track_plot(track_plot_b));
			// x3 : heading deviation error score
			double x3 = heading_score(track_plot_a, track_plot_b);

			double score = compute_matching_score(track_plot_a, track_plot_b, idx_a, idx_b);

			// FOR TESTING
			auto convolution = crossCorrelation_3D(track_plot_a->vel_orient_, track_plot_b->vel_orient_);

			// std::vector<double> line{track_plot_a->xs_.back(), track_plot_a->ys_.back(), track_plot_b->xs_.back() + FRAME_WIDTH_, track_plot_b->ys_.back(), score, convolution, 0};
			// lines.push_back(line);
			
			if (score < 0.5 && track_plot_a->frameNos_.size() > 180 && track_plot_b->frameNos_.size() > 180) {
			// if (x1 < 0.4 && x3 < 0.8 && track_plot_a->frameNos_.size() > 180 && track_plot_b->frameNos_.size() > 180) {
				
				if (track_plot_a->check_stationary() != track_plot_b->check_stationary()) {
					track_plot_a->mismatch_count_ += 3;
					track_plot_b->mismatch_count_ += 3;
				} else {
					track_plot_a->mismatch_count_ += 1;
					track_plot_b->mismatch_count_ += 1;
				}
			
			} else {
				track_plot_a->mismatch_count_ = 0;
				track_plot_b->mismatch_count_ = 0;
			}

			if (track_plot_a->mismatch_count_ >= 90 && track_plot_b->mismatch_count_ >= 90) {
				track_plot_a->mismatch_count_ = 0;
				track_plot_b->mismatch_count_ = 0;

				debug_messages.push_back("Target ID " +  std::to_string(matched_id)  + " is dropped due to mismatch");

				track_plot_a->id_ = track_plot_a->oid_;
				track_plot_b->id_ = track_plot_b->oid_;

				cumulative_tracks_[0]->track_new_plots_[track_plot_a->oid_] = track_plot_a;
				cumulative_tracks_[0]->track_plots_.erase(matched_id);

				cumulative_tracks_[1]->track_new_plots_[track_plot_b->oid_] = track_plot_b;
				cumulative_tracks_[1]->track_plots_.erase(matched_id);

				matching_dict_[0][track_plot_a->oid_] = track_plot_a->oid_;
				matching_dict_[1][track_plot_b->oid_] = track_plot_b->oid_;

				matched_tracks_.erase(matched_id);
			}
			
		}

	}

	void process_new_tracks(int idx, int alt, std::vector<std::shared_ptr<GoodTrack>> & good_tracks) {
		
		std::vector<std::shared_ptr<GoodTrack>> filter_good_tracks(good_tracks);

		std::map<int, std::map<int, double>> corrValues;
		std::shared_ptr<TrackPlot> track_plot, alt_track_plot;
		int track_id, alt_track_id;
		double score = 0;
		int row = 0;

		for (auto & track : good_tracks) {

			// Extract details from the track
			track_id = track->id;

			if (exists(cumulative_tracks_[idx]->track_new_plots_, track_id)) {

				auto track_plot = cumulative_tracks_[idx]->track_new_plots_[track_id];			

				// if track is a new track, we set a criteria of a min number of frames and non-zero track feature variables
				if (track_plot->frameNos_.size() >= 30 && track_plot->track_feature_variable_.size() >= 30 
						&& std::accumulate(track_plot->track_feature_variable_.begin(), track_plot->track_feature_variable_.end(), 0)) {
					// Look into other camera's new tracks list (new tracks first)
					for (auto & alt_track_item : cumulative_tracks_[alt]->track_new_plots_) {

						alt_track_plot = alt_track_item.second;
						alt_track_id = alt_track_plot->id_;
						
						// if track is a new track, we set a criteria of a min number of frames and non-zero track feature variables
						if (alt_track_plot->frameNos_.size() >= 30 && alt_track_plot->track_feature_variable_.size() >= 30 
								&& std::accumulate(alt_track_plot->track_feature_variable_.begin(), alt_track_plot->track_feature_variable_.end(), 0)) {

							// Compute a matching score for both tracks and append to corrValues only if non-zero
							score = compute_matching_score(track_plot, alt_track_plot, idx, alt);
							if (score != 0)	{
								// track is on dim 0, alt_track is on dim 1
								corrValues[track_id][alt_track_id] = score;
							}
						}
					}
					// Look into other camera's matched tracks list (old tracks last)
					for (auto & alt_track_item : cumulative_tracks_[alt]->track_plots_) {

						alt_track_plot = alt_track_item.second;
						alt_track_id = alt_track_plot->id_;

						bool eligibility_flag = true;
						
						// test to see if alternate camera's track is currently being matched with current camera
						for (auto & oth_idx_track : cumulative_tracks_[idx]->track_plots_) {
							if (alt_track_id == oth_idx_track.second->id_) {
								eligibility_flag = false; // 2nd camera's track has already been matched. skip the process of matching for this track
								break;
							}
						}

						if (eligibility_flag && std::accumulate(alt_track_plot->track_feature_variable_.begin(), alt_track_plot->track_feature_variable_.end(), 0)) {
							
							// Compute a matching score for both tracks and append to corrValues only if non-zero
							score = compute_matching_score(track_plot, alt_track_plot, idx, alt);
							if (score != 0)	{
								// track is on dim 0, alt_track is on dim 1
								corrValues[track_id][alt_track_id] = score;
							}
						}
					}
				}
				row += 1;
			} else {
				filter_good_tracks.erase(filter_good_tracks.begin() + row);
			}

		}
		for (auto & track : filter_good_tracks) {

			// Extract details from the track
			track_id = track->id;
			int maxID = -1;
			double maxValue = -1;

			// for the selected max track in the 2nd camera, we check to see if the track has a higher
			// cross correlation value with another track in current camera

			while (corrValues[track_id].size() != 0) {
				for (auto it : corrValues[track_id]) {
					if (maxValue < it.second) {
						maxID = it.first;
						maxValue = it.second;
					}
				}
				
				// search through current camera's tracks again, for the selected track that we wish to re-id with.
				// we can note that if there is a track in the current camera that has a higher cross correlation value
				// than the track we wish to match with, then the matching will not occur.
				bool global_max_flag = false;
				for (auto & oth_idx_track : filter_good_tracks) {
					if (exists(corrValues[oth_idx_track->id], maxID) && (corrValues[oth_idx_track->id][maxID] > maxValue)) {
						corrValues[track_id].erase(maxID);
						global_max_flag = true;
						break;
					}
				}

				if (global_max_flag == true) {
					// there existed a value larger than the current maxValue. thus, re-id cannot occur
					maxID = -1;
					maxValue = -1;
					continue;
				} else {
					// went through the whole loop without finding any other max, thus it is the maximum value. re-id can occur
					break;
				}
			}
			// re-id process
			if (maxID != -1) {

				track_plot = cumulative_tracks_[idx]->track_new_plots_[track_id];

				// if track is in alt camera's track_new_plots
				if (maxID != 1 && exists(cumulative_tracks_[alt]->track_new_plots_, maxID)) {

					alt_track_id = maxID;
					alt_track_plot = cumulative_tracks_[alt]->track_new_plots_[alt_track_id];
					
					// add notification message
					debug_messages.push_back("New target ID " +  std::to_string(next_id_)  + " acquired with a score of " + std::to_string(maxValue));
					
					promote_to_matched(idx, track_plot, next_id_, track_id);
					promote_to_matched(alt, alt_track_plot, next_id_, alt_track_id);
				
					// increment next id assignment value
					next_id_ += 1;

				// if track is in alt camera's track_plots
				} else {

					alt_track_id = maxID;
					alt_track_plot = cumulative_tracks_[alt]->track_plots_[alt_track_id];
					// add notification message
					debug_messages.push_back("New target ID " +  std::to_string(track_plot->id_)  + " acquired with a score of " + std::to_string(maxValue));
					
					promote_to_matched(idx, track_plot, alt_track_plot->id_, track_id);
				}
			}
		}
	}

	/**
	 * Moves a track from track_new_plots to track_plots and updates all necessary variables
	 */ 
	void promote_to_matched(int index, std::shared_ptr<TrackPlot> track_plot, int matched_id, int old_id) {
		track_plot->id_ = matched_id;
		cumulative_tracks_[index]->track_plots_.insert({matched_id, track_plot});
		cumulative_tracks_[index]->track_new_plots_.erase(old_id);
		matching_dict_[index][old_id] = matched_id;
		matched_tracks_[matched_id][index] = old_id;
		matched_tracks_[matched_id][NUM_OF_CAMERAS_]++;	
	}

	/**
	 * Normalises the existing track plot based on mean and sd
	 */
	std::vector<double> normalise_track_plot(std::shared_ptr<TrackPlot> track_plot) {

		int total_track_feature = track_plot->track_feature_variable_.size();
		double mean = 0, variance = 0, std;
		std::vector<double> result;

		// Mean
		mean = accumulate(track_plot->track_feature_variable_.begin(), track_plot->track_feature_variable_.end(), 0.0) / total_track_feature;

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

	double compute_matching_score(std::shared_ptr<TrackPlot> track_plot_a,
			std::shared_ptr<TrackPlot> track_plot_b, int idx_a, int idx_b) {
				
		// Updating of tracks in the local neighbourhood
		update_other_tracks(track_plot_a, cumulative_tracks_[idx_a]);
		update_other_tracks(track_plot_b, cumulative_tracks_[idx_b]);

		// x1: Track feature variable correlation strength based on normalised values
		double x1 = crossCorrelation(normalise_track_plot(track_plot_a), normalise_track_plot(track_plot_b));
		// x2: Geometric track matching strength value
		double x2 = geometric_similarity(track_plot_a->other_tracks_, track_plot_b->other_tracks_);
		// x3: Heading deviation error value
		double x3 = heading_score(track_plot_a, track_plot_b);

		// TESTING FOR 3D VELOCITY ORIENTATION
		auto convolution = crossCorrelation_3D(track_plot_a->vel_orient_, track_plot_b->vel_orient_);

		double score = (W1_ * x1) + (W2_ * x2) + (W3_ * x3);

		// if (idx_a == 0) {
		// 	std::vector<double> line{track_plot_a->xs_.back(), track_plot_b->ys_.back(), track_plot_b->xs_.back() + FRAME_WIDTH_, track_plot_b->ys_.back(), score, convolution, 0};
		// 	lines.push_back(line);
		// }
		// std::cout << "R_value: " << r_value << ", Score: " << score << ", 3D conv: " << convolution << std::endl;

		return (score > 0.5) ? score : 0;
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

	/**
	 * Find cross correlation of two 3D arrays with size n
	 * Involves the convolution of array X with array Y by sliding std::vector Y from left to right
	 */
	double crossCorrelation_3D(std::vector<std::array<double, 3>> X, std::vector<std::array<double, 3>> Y)	{

			int length = std::min(X.size(), Y.size());
			length = std::min(length, 60);
			
			double sum = 0;
			double theta = 40;
			std::vector<std::array<double, 3>> A(X.end() - length, X.end());
			std::vector<std::array<double, 3>> B(Y.end() - length, Y.end());

			for (int i = 0; i < length; i++) {
				// std::cout << "Old: " << B[i][0] << " " << B[i][2] << std::endl;
				double x_prime = (B[i][0] * cos(theta / 180.0 * M_PI)) - (B[i][2] * sin(theta / 180.0 * M_PI));
				double z_prime = (B[i][0] * sin(theta / 180.0 * M_PI)) + (B[i][2] * cos(theta / 180.0 * M_PI));
				B[i][0] = x_prime;
				B[i][2] = z_prime;
				// std::cout << "New: " << B[i][0] << " " << B[i][2] << std::endl;
			}

			for (int i = 0; i < length; i++) {
				for (int axis = 0; axis < 3; axis++) {
					double l = A[i][axis] * B[i][axis];
					sum += l / length;
					// std::cout << "Result: " << l << std::endl;
				}
				// std::cout << "Matching: " << A[i][0] << " " << B[i][0] << ", " << A[i][1] << " " << B[i][1] << ", " << A[i][2] << " " << B[i][2] <<  std::endl;
			}
			
			return sum;
	}

	/**
	 * Find geometirc similarity by comparing the relative coordinates of other tracks
	 */
	double geometric_similarity(
		std::vector<std::shared_ptr<TrackPlot::OtherTrack>> & other_tracks_a, 
		std::vector<std::shared_ptr<TrackPlot::OtherTrack>> & other_tracks_b)	{

		std::vector<double> relative_distances, shortest_distances;

		for (auto & other_track_a : other_tracks_a) {
			int a_dx = other_track_a->dx;
			int a_dy = other_track_a->dy;

			relative_distances.clear();

			for (auto & other_track_b : other_tracks_b) {
				int b_dx = other_track_b->dx;
				int b_dy = other_track_b->dy;

				double relative_distance = hypot(std::abs(a_dx - b_dx), std::abs(a_dy - b_dy)) / 1920;
				relative_distances.push_back(relative_distance);
			}

			if (relative_distances.size() > 0) {
				double minimum = relative_distances.front();
				for (auto & relative_distance : relative_distances) {
					if (relative_distance < minimum){
						minimum = relative_distance;
					}
				}
				shortest_distances.push_back(minimum);
			}
		}

		if (shortest_distances.size() > 0) {
			// Get average value of shortest_distances
			double avg_shortest_distances = std::accumulate(shortest_distances.begin(), shortest_distances.end(), 0) / shortest_distances.size();
			return std::max<double>(0.001, 0.2 - avg_shortest_distances) * 5;
		} else {
			return 0;
		}

	}
	
	/**
	 * Find similarity by comparing the direction signature of the plot history
	 */
	double heading_score(std::shared_ptr<TrackPlot> track_plot_a, std::shared_ptr<TrackPlot> track_plot_b) {
		
		double deviation = 0;
		int dx_0 = track_plot_a->xs_.back() - track_plot_a->xs_[track_plot_a->xs_.size() - 2];
		int dy_0 = track_plot_a->ys_.back() - track_plot_a->ys_[track_plot_a->ys_.size() - 2];
		double rotation_0 = (atan2((double) dy_0, (double) dx_0) + M_PI) / (2 * M_PI);

		int dx_1 = track_plot_b->xs_.back() - track_plot_b->xs_[track_plot_b->xs_.size() - 2];
		int dy_1 = track_plot_b->ys_.back() - track_plot_b->ys_[track_plot_b->ys_.size() - 2];
		double rotation_1 = (atan2((double) dy_1, (double) dx_1) + M_PI) / (2 * M_PI);

		for (int i = -2; i > 1 - HDG_SCORE_TRACK_HIST_; i--) {
			dx_0 = track_plot_a->xs_[track_plot_a->xs_.size() - 1 + i] - track_plot_a->xs_[track_plot_a->xs_.size() - 2 + i];
			dy_0 = track_plot_a->ys_[track_plot_a->ys_.size() - 1 + i] - track_plot_a->ys_[track_plot_a->ys_.size() - 2 + i];
			double angle_0 = (atan2((double) dy_0, (double) dx_0) + M_PI) / (2 * M_PI);

			dx_1 = track_plot_b->xs_[track_plot_b->xs_.size() - 1 + i] - track_plot_b->xs_[track_plot_b->xs_.size() - 2 + i];
			dy_1 = track_plot_b->ys_[track_plot_b->ys_.size() - 1 + i] - track_plot_b->ys_[track_plot_b->ys_.size() - 2 + i];
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
			return 1 - (deviation / (HDG_SCORE_TRACK_HIST_ - 1));
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

		for (auto & matched_track : matched_tracks_) {

			if (matched_track.second[NUM_OF_CAMERAS_] < 2) continue;

			int matched_id = matched_track.first;

			auto track_plot_a = cumulative_tracks_[0]->track_plots_[matched_id];
			auto track_plot_b = cumulative_tracks_[1]->track_plots_[matched_id];

			if (track_plot_a->lastSeen_ != frame_count_ || track_plot_b->lastSeen_ != frame_count_) continue;

			int x_L = track_plot_a->xs_.back();
			int y_L = track_plot_a->ys_.back();
			int x_R = track_plot_b->xs_.back();
			int y_R = track_plot_b->ys_.back();

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

			track_plot_a->xyz_ = {X, Y, Z};
			track_plot_b->xyz_ = {X, Y, Z};

			log_3D(track_plot_a, track_plot_b);
		}
	}

	/**
	 * Helper function to check if an element is present in an array.
	 */
	template <typename T1, typename T2>
	bool exists(std::map<T1, T2> arr, T1 item) {
		return arr.find(item) != arr.end();
	}
}