/**
 * @file multi_cam_detect.cpp
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
 * This file contains the declarations of the functions primarily used in the 
 * detection pipeline. These functions interact with the key classes Camera 
 * and Track which are essential to the detection process. These classes may 
 * be found at multi_cam_detect_utils.cpp.
 */

// mathematical constants
#define _USE_MATH_DEFINES

// opencv header files
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/ximgproc.hpp>

// local header files
#include "multi_cam_detect_utils.hpp"
#include "multi_cam_params.hpp"
#include "Hungarian.h"
#include "WSrtInterface.hpp"

#include "multi_cam_detect.hpp"
#include "multi_cam_params.hpp"

// standard package imports
#include <string>
#include <memory>
#include <chrono>
#include <vector>
#include <list>
#include <array>

using namespace mcmt;

namespace mcmt {

	// declare Camera variables
	std::vector<std::shared_ptr<Camera>> cameras_;
	cv::Mat element_;
	std::vector<std::shared_ptr<cv::VideoWriter>> recordings_;

	/**
	 * Constants, variable and functions definition
	 */
	std::vector<cv::Mat> initialize_cameras() {

		std::string vid_input, vid_output;

		for (int cam_idx = 0; cam_idx < NUM_OF_CAMERAS_; cam_idx++) {
			
			vid_input = VIDEO_INPUT_[cam_idx];
			vid_output = VIDEO_OUTPUT_[cam_idx];

			cameras_.push_back(std::shared_ptr<Camera>(
				new Camera(cam_idx, IS_REALTIME_, vid_input, VIDEO_FPS_, FRAME_WIDTH_, 
				FRAME_HEIGHT_, FGBG_HISTORY_, BACKGROUND_RATIO_, NMIXTURES_)));

			if (IS_REALTIME_) {
				recordings_.push_back(std::shared_ptr<cv::VideoWriter>(new cv::VideoWriter(
				vid_output, cv::VideoWriter::fourcc('M','J','P','G'), VIDEO_FPS_, 
				cv::Size(FRAME_WIDTH_, FRAME_HEIGHT_))));
			}
			
		}

		std::vector<cv::Mat> sample_frames;
		cv::Mat sample_frame;
		for (int cam_idx = 0; cam_idx < NUM_OF_CAMERAS_; cam_idx++) {
			if (IS_REALTIME_ == 2) { // To remove when interface shifts
				wsrt_output edgecam_data = cameras_[cam_idx]->edgecam_cap_->extract_data();
				sample_frame = edgecam_data.image.clone();
			}
			else {
				cameras_[cam_idx]->cap_ >> sample_frame;
			}
			sample_frames.push_back(sample_frame);
		}
		
		// initialize kernel used for morphological transformations
		element_ = cv::getStructuringElement(0, cv::Size(5, 5));

		return sample_frames;
	}

	/**
	 * Takes the difference between frames at t and t-1 to extract out moving objects.
	 * Especially useful for when the drone passes behind noisy background features.
	 * The moving objects are added back to the original frame as feature enhancements 
	 */
	void frame_to_frame_subtraction(std::shared_ptr<Camera> & camera) {
		
			cv::Mat frame_delta_, frame_delta_grayscale_;
			cv::absdiff(camera->frame_, camera->frame_store_, frame_delta_);

			cv::cvtColor(frame_delta_, frame_delta_grayscale_, cv::COLOR_BGR2GRAY);
			cv::bitwise_not(frame_delta_grayscale_, frame_delta_grayscale_);
			cv::cvtColor(frame_delta_grayscale_, frame_delta_, cv::COLOR_GRAY2BGR);

   			cv::addWeighted(frame_delta_, DELTA_FRAME_PROPORTION_, camera->frame_, 1.0 - DELTA_FRAME_PROPORTION_, 0.0, camera->frame_);
	}

	/**
	 * Apply environmental compensation on frame. This is needed when environmental conditions prevent
 	 * the target from standing out. Localised contrast and saturation changes are applied to
 	 * regions of the frame identified as sky depending on brightness conditions in each region
	 */
	void apply_env_compensation(std::shared_ptr<Camera> & camera) {
		cv::Mat hsv, sky, non_sky, mask;
		camera->frame_ec_ = camera->frame_.clone();
		
		// Get HSV version of the frame
		cv::cvtColor(camera->frame_ec_, hsv, cv::COLOR_BGR2HSV);

		// Threshold the HSV image to extract the sky and put it in sky frame
		// The threshold V value for sky is determined using Otsu thresholding
		// Keep the sky frame in hsv for subsequent operations
		std::vector<cv::Mat> channels;
		cv::split(hsv, channels);
		cv::threshold(channels[2], mask, -1, 255, cv::THRESH_OTSU);
		cv::bitwise_and(hsv, hsv, sky, mask);
		// cv::imshow("sky", sky);

		// Extract the treeline and put it in non_sky frame
		// The mask for the treeline is the inversion of the sky mask
		// Convert treeline back to RGB using bitwise_and
		cv::bitwise_not(mask, mask);
		cv::bitwise_and(camera->frame_ec_, camera->frame_ec_, non_sky, mask);
		// cv::imshow("non sky", non_sky);

		// Again, split the sky into 3 channels (HSV)
		channels.clear();
		cv::split(sky, channels);

		// Only apply following transformations to pixels with V > 0
		cv::Mat sky_mask(FRAME_WIDTH_, FRAME_HEIGHT_, CV_8UC1);
		cv::Mat sky_mask_f(FRAME_WIDTH_, FRAME_HEIGHT_, CV_32FC1);
		cv::threshold(channels[2], sky_mask, 1, 0, 2);
		sky_mask.convertTo(sky_mask_f, CV_32FC1);

		// Convert value channel to float matrix	
		cv::Mat value_f(FRAME_WIDTH_, FRAME_HEIGHT_, CV_32FC1);
		channels[2].convertTo(value_f, CV_32FC1);
	
		// Decrease saturation based on how bright the pixel is
		// The brighter the pixel, the greater the decrease
		// The formula used is our own model that assumes linear relationship
		// between saturation scale factor (sat) and pixel brightness
		// Formula: pixel[sat] *= 1 - 0.7 * pixel[val] / 255
		value_f = sky_mask_f - (value_f * (0.7/255));
		cv::multiply(channels[1], value_f, channels[1], 1, CV_8UC1);

		// If the pixel is too dark, max its value to provide contrast
		cv::Mat dark;
		cv::inRange(channels[2], 1, 149, dark);
		cv::add(channels[2], dark, channels[2]);
		// Merge to combine back to sky
		cv::merge(channels, sky);

		cv::cvtColor(sky, sky, cv::COLOR_HSV2BGR);

		// Treeline (non-sky) enhancements using histogram equalisation on intensity channel
		if (USE_HIST_EQUALISE_) {
			channels.clear();
			cv::Mat ycrcb;
			cv::cvtColor(non_sky, ycrcb, cv::COLOR_BGR2YCrCb);
			cv::split(ycrcb, channels);
			cv::equalizeHist(channels[0], channels[0]);
			cv::merge(channels, ycrcb);
			cv::cvtColor(ycrcb, non_sky, cv::COLOR_YCrCb2BGR);
		}

		// Recombine the sky and treeline
		cv::add(sky, non_sky, camera->frame_ec_);
		cv::imshow("After sun compensation", camera->frame_ec_);
	}

	/** 
	 * This function uses the background subtractor to subtract the history from the current frame.
	 */
	void remove_ground(std::shared_ptr<Camera> & camera, int masked_id)	{

		cv::Mat masked;
		
		// Apply contrast and brightness gains
		// To-do: Explain how the formula for calculating brightness in the 2nd line works
		cv::convertScaleAbs((masked_id ? camera->frame_ec_ : camera->frame_), masked, 1, (256 - average_brightness(camera) + BRIGHTNESS_GAIN_));
		
		// subtract background
		camera->fgbg_[masked_id]->apply(masked, masked, FGBG_LEARNING_RATE_);
		masked.convertTo(camera->masked_[masked_id], CV_8UC1);

		if (USE_BG_SUBTRACTOR_){
			// declare variables
			std::vector<std::vector<cv::Point>> contours;
			std::vector<std::vector<cv::Point>> background_contours;

			// number of iterations determines how close objects need to be to be considered background
			cv::Mat dilated;
			cv::dilate(camera->masked_[masked_id], dilated, element_, cv::Point(), int(REMOVE_GROUND_ITER_ * camera->scale_factor_));
			findContours(dilated, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

			for (auto & it : contours) {
				float circularity = 4 * M_PI * cv::contourArea(it) / (pow(cv::arcLength(it, true), 2));
				if (circularity <= BACKGROUND_CONTOUR_CIRCULARITY_) {
					background_contours.push_back(it);
				}
			}
			
			// show removed background on raw image frames
			// cv::Mat bg_removed = masked_id ? camera->frame_ec_.clone() : camera->frame_.clone();
			// cv::drawContours(bg_removed, background_contours, -1, cv::Scalar(0, 255, 0), 3);
			// cv::imshow("bg_removed", bg_removed);

			// draw contours on masked frame using solid shape to remove background
			cv::drawContours(camera->masked_[masked_id], background_contours, -1, cv::Scalar(0, 0, 0), -1);
		}
	}

	/**
	 * This function applies masking operations before performing blob detection
	 */
	void detect_objects(std::shared_ptr<Camera> & camera) {
		
		// Loop through both original and env compensated frames
		for (int i = 0; i < camera->masked_.size(); i++) {
		
			// apply morphological transformation
			cv::dilate(camera->masked_[i], camera->masked_[i], element_, cv::Point(), DILATION_ITER_);

			// invert frame such that black pixels are foreground
			cv::bitwise_not(camera->masked_[i], camera->masked_[i]);

			// apply blob detection
			std::vector<cv::KeyPoint> keypoints;
			camera->detector_->detect(camera->masked_[i], keypoints);

			// clear vectors to store sizes and centroids of current frame's detected targets
			for (auto & it : keypoints) {
				camera->centroids_temp_[i].push_back(it.pt);
				camera->sizes_temp_[i].push_back(it.size);
			}
		}

		remove_overlapped_detections(camera);

		// add final sizes and centroids after filtering out overlaps
		for (int i = 0; i < camera->centroids_temp_.size(); i++) {
			for (auto & it : camera->centroids_temp_[i]) {
				if (it.x >= 0 && it.y >= 0) {
					camera->centroids_.push_back(it);
				}
			}
			for (auto & it : camera->sizes_temp_[i]) {
				if (it >= 0) {
					camera->sizes_.push_back(it);
				}
			}
		}
	}

	/**
	 * This function detects and removes overlaps in the detections made
	 * in the original and the env compensated frames
	*/
	void remove_overlapped_detections(std::shared_ptr<Camera> & camera) {	
		// delta = distance between two detected blobs across different frames
		// TODO: What delta threshold to set?
		float delta, delta_thres = 5.0;
	
		// for each detection in frame 0 (original), check with detections in frame 1 (env compensated).
		// detections that have separations below a specified threshold are considered overlaps
		// marked the overlapped detections in frame 1 with -1.0 to signify that they should be ignored
		for (int i = 0; i < camera->centroids_temp_[0].size(); i++) {
			for (int j = 0; j < camera->centroids_temp_[1].size(); j++) {
				delta = sqrt(pow(camera->centroids_temp_[0][i].x - camera->centroids_temp_[1][j].x, 2)
						+ pow(camera->centroids_temp_[0][i].y - camera->centroids_temp_[1][j].y, 2));
				if (delta < delta_thres) {
					camera->centroids_temp_[1][j].x = -1.0;
					camera->centroids_temp_[1][j].y = -1.0;
					camera->sizes_temp_[1][j] = -1.0;
				}
			}
		}
	}

	/**
	 * This function uses the kalman filter and DCF to predict new location of current tracks
	 * in the next frame.
	 */
	void predict_new_locations_of_tracks(std::shared_ptr<Camera> & camera)
	{
		for (auto & it : camera->tracks_) {
			// predict next location using KF and DCF
			it->predictKF();
			it->predictDCF(camera->frame_);
		}
	}

	/**
	 * This function clears the track variables at each new frame.
	 */
	void clear_track_variables(std::shared_ptr<Camera> & camera) {
		camera->assignments_.clear();
		camera->unassigned_tracks_.clear();
		camera->unassigned_detections_.clear();
		camera->tracks_to_be_removed_.clear();
		
		camera->assignments_kf_.clear();
		camera->unassigned_tracks_kf_.clear();
		camera->unassigned_detections_kf_.clear();

		camera->assignments_dcf_.clear();
		camera->unassigned_tracks_dcf_.clear();
		camera->unassigned_detections_dcf_.clear();
	}

	/**
	 * This function assigns detections to tracks using Munkre's algorithm. The algorithm is 
	 * based on cost calculated euclidean distance between detections and tracks' prediction location
	 * based on KF. Detections being located too far away from existing tracks will be designated as
	 * unassigned detections. Tracks without any nearby detections are also designated as unassigned tracks
	 */
	void detection_to_track_assignment_KF(std::shared_ptr<Camera> & camera)	{
		// declare non assignment cost
		float cost_of_non_assignment = 10 * camera->scale_factor_;

		// num of tracks and centroids, and get min and max sizes
		int num_of_tracks = camera->tracks_.size();
		int num_of_centroids = camera->centroids_.size();
		int total_size = num_of_tracks + num_of_centroids;

		// declare 2-D cost matrix and required variables
		std::vector<std::vector<double>> cost(total_size, std::vector<double>(total_size, 0.0));
		int row_index = 0;
		int col_index = 0;
		std::vector<int> assignments_all;

		// get euclidean distance of every detected centroid with each track's predicted location
		for (auto & track : camera->tracks_) {
			for (auto & centroid : camera->centroids_) {
				cost[row_index][col_index] = euclideanDist(track->predicted_, centroid);
				col_index++;
			}
			// padding cost matrix with dummy columns to account for unassigned tracks, used to fill
			// the top right corner of the cost matrix
			for (int i = 0; i < num_of_tracks; i++) {
				cost[row_index][col_index] = cost_of_non_assignment;
				col_index++;
			}
			row_index++;
			col_index = 0;
		}

		// padding for cost matrix to account for unassigned detections, used to fill the bottom
		// left corner of the cost matrix
		std::vector<float> append_row;
		for (int i = num_of_tracks; i < total_size; i++) {
			for (int j = 0; j < num_of_centroids; j++) {
				cost[i][j] = cost_of_non_assignment;
			}
		}

		// the bottom right corner of the cost matrix, corresponding to dummy detections being 
		// matched to dummy tracks, is left with 0 cost to ensure that the excess dummies are
		// always matched to each other

		// apply Hungarian algorithm to get assignment of detections to tracked targets
		if (total_size > 0) {
			assignments_all = apply_hungarian_algo(cost);
			int track_index = 0;
		
			// get assignments, unassigned tracks, and unassigned detections
			for (auto & assignment : assignments_all) {
				if (track_index < num_of_tracks) {
					// track index is less than no. of tracks, thus the current assignment is a valid track
					if (assignment < num_of_centroids) {
						// if in the top left of the cost matrix (no. of tracks x no. of detections), these 
						// assignments are successfully matched detections and tracks. For this, the assigned 
						// detection index must be less than number of centroids. if so, we will store the 
						// track indexes and detection indexes in the assignments vector
						std::vector<int> indexes(2, 0);
						indexes[0] = track_index;
						indexes[1] = assignment;
						camera->assignments_kf_.push_back(indexes);
					} else {
						// at the top right of the cost matrix. if detection index is equal to or more than 
						// the no. of detections, then the track is assigned to a dummy detection. As such, 
						// it is an unassigned track
						camera->unassigned_tracks_kf_.push_back(track_index);
					}
				} else {
					// at the lower half of cost matrix. Thus, if detection index is less than no. of 
					// detections, then the detection is assigned to a dummy track. As such it is an 
					// unassigned detections
					if (assignment < num_of_centroids) {
						camera->unassigned_detections_kf_.push_back(assignment);
					}
					// if not, then the last case is when excess dummies are matching with each other. 
					// we will ignore these cases
				}
				track_index++;
			}
		}
	}

	/**
	 * This function assigns detections to tracks using Munkre's algorithm. The algorithm is 
	 * based on cost calculated euclidean distance between detections and tracks' prediction location
	 * based on DCF. Detections being located too far away from existing tracks will be designated as
	 * unassigned detections. Tracks without any nearby detections are also designated as unassigned tracks
	 */
	void detection_to_track_assignment_DCF(std::shared_ptr<Camera> & camera)
	{
		// declare non assignment cost
		float cost_of_non_assignment = 10 * camera->scale_factor_;

		// num of tracks and centroids, and get min and max sizes
		int num_of_tracks = camera->tracks_.size();
		int num_of_centroids = camera->centroids_.size();
		int total_size = num_of_tracks + num_of_centroids;

		// declare 2-D cost matrix and required variables
		std::vector<std::vector<double>> cost(total_size, std::vector<double>(total_size, 0.0));
		int row_index = 0;
		int col_index = 0;
		std::vector<int> assignments_all;

		// get euclidean distance of every detected centroid with each track's predicted location
		for (auto & track : camera->tracks_) {
			for (auto & centroid : camera->centroids_) {
				cv::Point2f point;
				point.x = track->box_.x + (0.5 * track->box_.width);
				point.y = track->box_.y + (0.5 * track->box_.height);
				cost[row_index][col_index] = euclideanDist(point, centroid);
				col_index++;
			}
			// padding cost matrix with dummy columns to account for unassigned tracks, used to fill
			// the top right corner of the cost matrix
			for (int i = 0; i < num_of_tracks; i++) {
				cost[row_index][col_index] = cost_of_non_assignment;
				col_index++;
			}
			row_index++;
			col_index = 0;
		}

		// padding for cost matrix to account for unassigned detections, used to fill the bottom
		// left corner of the cost matrix
		std::vector<float> append_row;
		for (int i = num_of_tracks; i < total_size; i++) {
			for (int j = 0; j < num_of_centroids; j++) {
				cost[i][j] = cost_of_non_assignment;
			}
		}

		// the bottom right corner of the cost matrix, corresponding to dummy detections being 
		// matched to dummy tracks, is left with 0 cost to ensure that the excess dummies are
		// always matched to each other

		// apply Hungarian algorithm to get assignment of detections to tracked targets
		if (total_size > 0) {
			assignments_all = apply_hungarian_algo(cost);
			int track_index = 0;
		
			// get assignments, unassigned tracks, and unassigned detections
			for (auto & assignment : assignments_all) {
				if (track_index < num_of_tracks) {
					// track index is less than no. of tracks, thus the current assignment is a valid track
					if (assignment < num_of_centroids) {
						// if in the top left of the cost matrix (no. of tracks x no. of detections), these 
						// assignments are successfully matched detections and tracks. For this, the assigned 
						// detection index must be less than number of centroids. if so, we will store the 
						// track indexes and detection indexes in the assignments vector
						std::vector<int> indexes(2, 0);
						indexes[0] = track_index;
						indexes[1] = assignment;
						camera->assignments_dcf_.push_back(indexes);
					} else {
						// at the top right of the cost matrix. if detection index is equal to or more than 
						// the no. of detections, then the track is assigned to a dummy detection. As such, 
						// it is an unassigned track
						camera->unassigned_tracks_dcf_.push_back(track_index);
					}
				} else {
					// at the lower half of cost matrix. Thus, if detection index is less than no. of 
					// detections, then the detection is assigned to a dummy track. As such it is an 
					// unassigned detections
					if (assignment < num_of_centroids) {
						camera->unassigned_detections_dcf_.push_back(assignment);
					}
					// if not, then the last case is when excess dummies are matching with each other. 
					// we will ignore these cases
				}
				track_index++;
			}
		}
	}

	/**
	 * This function processes the two cost matrices from the DCF and KF predictions for the matching
	 * of detections to tracks. We obtain the final assignments, unassigned_tracks, and unassigned
	 * detections vectors from this function.
	 */
	void compare_cost_matrices(std::shared_ptr<Camera> & camera)
	{
		// check to see if it is the case where there are no assignments in the current frame
		if (camera->assignments_kf_.size() == 0 && camera->assignments_dcf_.size() == 0) {
				camera->assignments_ = camera->assignments_kf_;
				camera->unassigned_tracks_ = camera->unassigned_tracks_kf_;
				camera->unassigned_detections_ = camera->unassigned_detections_kf_;
			return;
		}

		// check to see if assignments by kf and dcf are equal
		else if (camera->assignments_kf_.size() == camera->assignments_dcf_.size()) {
			// get bool if the kf and dcf assignments are equal
			bool is_equal = std::equal(
				camera->assignments_kf_.begin(), camera->assignments_kf_.end(), camera->assignments_dcf_.begin());
			
			if (is_equal == true) {
				// assignments are the same. get the final tracking assignment vectors
				camera->assignments_ = camera->assignments_kf_;
				camera->unassigned_tracks_ = camera->unassigned_tracks_kf_;
				camera->unassigned_detections_ = camera->unassigned_detections_kf_;
				return;
			} 
			else {
				// we will always choose to use detection-to-track assignments using the dcf
				camera->assignments_ = camera->assignments_dcf_;
				camera->unassigned_tracks_ = camera->unassigned_tracks_dcf_;
				camera->unassigned_detections_ = camera->unassigned_detections_dcf_;
				return;
			}
		}

		// when kf and dcf assignments are not zero, and they are not equal as well. in this code
		// block, we know that the dcf and kf differ in terms of assigning the tracks and detections.
		// this means that either the dcf or the kf is able to assign a detection to a track, while the
		// other filter was not able to. in this case, we will loop through their assignments, and 
		// implement all successful assignments to the final tracking assignment vectors
		else {
			// declare flags
			bool different_flag;
			bool different_assignment_flag;
			bool already_assigned_flag;
			std::vector<int> assigned_tracks;
			std::vector<int> assigned_detections;

			// iterate through every dcf assignment
			for (auto & dcf_assignment : camera->assignments_dcf_) {
				different_flag = true;
				different_assignment_flag = false;
				
				// interate through kf assignments
				for (auto & kf_assignment : camera->assignments_kf_) {
					// check if dcf assignment is the same as the kf assignment. if it is, break immediately
					// condition: different_flag = false, different_assignment_flag = false
					if (dcf_assignment == kf_assignment) {
						different_flag = false;
						break;
					}
					// check if dcf assignment track index equal to the kf assignment track index
					// if it is, then the kf and dcf assigned the same track to a different detection
					// condition: different_flag = true, different_assignment_flag = true
					else {
						if (dcf_assignment[0] == kf_assignment[0]) {
							different_assignment_flag = true;
							break;
						}
					}
				}

				// both kf and dcf assigned the same detection to track
				// condition: different_flag = false
				if (different_flag == false) {
					camera->assignments_.push_back(dcf_assignment);
					assigned_tracks.push_back(dcf_assignment[0]);
					assigned_detections.push_back(dcf_assignment[1]);
				}

				// kf and dcf did not assign the same detection to track
				// condition: different_flag = true
				else {
					// see if kf and dcf assigned the track to a different detections
					// condition: different_flag = false, different_assignment_flag = true
					// for this case, we will always go with dcf predictions
					if (different_assignment_flag == true) {
						camera->assignments_.push_back(dcf_assignment);
						assigned_tracks.push_back(dcf_assignment[0]);
						assigned_detections.push_back(dcf_assignment[1]);
					}
					// dcf assigned the track to a detection, but the kf did not. 
					// condition: different_flag = false, different_assignment_flag = false
					// for this case, we will assign the track to the detection that the dcf assigned it to.
					else {
						camera->assignments_.push_back(dcf_assignment);
						assigned_tracks.push_back(dcf_assignment[0]);
						assigned_detections.push_back(dcf_assignment[1]);
					}
				}
			}

			// iterate through every kf assignment. in this iteration, we will find for any tracks that
			// the kf assigned, but the dcf did not
			for (auto & kf_assignment : camera->assignments_kf_) {
				already_assigned_flag = false;
				different_assignment_flag = false;

				// interate through dcf assignments
				for (auto & dcf_assignment : camera->assignments_dcf_) {
					// check if kf assignment is the same as the dcf assignment. if it is, immediately break
					// and move on to the next dcf track
					if (dcf_assignment == kf_assignment) {
						break;
					}
					// check if kf assignment track index equal to the dcf assignment track index
					// if it is, then the kf and dcf assigned the same track to a different detection
					// condition: different_flag = true
					else {
						if (dcf_assignment[0] == kf_assignment[0]) {
							different_assignment_flag = true;
							break;
						}
					}
				}

				// code block are for cases where dcf_assignment and kf_assignment are different, and
				// that the kf assigned the track to a detection, but the dcf did not
				if (already_assigned_flag == false || different_assignment_flag == false) {
					// check first to see if the detection is already part of an assignment
					// if so, do not add it as it potentially results in multiple tracks assigned to a single detection
					if (std::find(assigned_detections.begin(), assigned_detections.end(), kf_assignment[1]) != assigned_detections.end()) {
						// existing assignment to this detection exists. since this is likely a crossover event, we prioritise KF
						// look for confliction DCF assignment
						for (auto & dcf_assignment : camera->assignments_dcf_) {
							if (kf_assignment[1] == dcf_assignment[1]) {
								// once conflicting DCF assignment found, delete it from assignments
								camera->assignments_.erase(std::remove(camera->assignments_.begin(), camera->assignments_.end(), dcf_assignment), camera->assignments_.end());
								assigned_tracks.erase(std::remove(assigned_tracks.begin(), assigned_tracks.end(), dcf_assignment[0]), assigned_tracks.end());
								assigned_detections.erase(std::remove(assigned_detections.begin(), assigned_detections.end(), dcf_assignment[1]), assigned_detections.end());
								// attempt to assign conflicting track with prior KF assignment, if it exists
								// otherwise, don't bother
								for (auto & prior_kf_assignment : camera->assignments_kf_) {
									// check to ensure that the new assignment detection is unassigned
									if (prior_kf_assignment[0] == dcf_assignment[0] && std::find(assigned_detections.begin(), assigned_detections.end(), prior_kf_assignment[1]) != assigned_detections.end()) {
										camera->assignments_.push_back(prior_kf_assignment);
										assigned_tracks.push_back(prior_kf_assignment[0]);
										assigned_detections.push_back(prior_kf_assignment[1]);
										break;
									}
								}
								break;
							}
						}
					}
					// update the KF assignment
					camera->assignments_.push_back(kf_assignment);
					assigned_tracks.push_back(kf_assignment[0]);
					assigned_detections.push_back(kf_assignment[1]);
				}
				// for the case where kf and dcf assigned the same track different detections (condition
				// when different_assignment_flag = true), we will take the dcf assignments
			}

			// get unassigned tracks
			for (auto & unassigned_track : camera->unassigned_tracks_dcf_) {
				if (std::find(assigned_tracks.begin(), assigned_tracks.end(), unassigned_track) != assigned_tracks.end()) {
					continue;
				}
				camera->unassigned_tracks_.push_back(unassigned_track);
			}

			// get unassigned detections
			for (auto & unassigned_detection : camera->unassigned_detections_dcf_) {
				if (std::find(assigned_detections.begin(), assigned_detections.end(), unassigned_detection) != assigned_detections.end()) {
					continue;
				}
				camera->unassigned_detections_.push_back(unassigned_detection);
			}
		}
	}

	/**
	 * This function processes the valid assignments of tracks and detections using the detection
	 * and track indices, and updates the tracks with the matched detections
	 */
	void update_assigned_tracks(std::shared_ptr<Camera> & camera)
	{
		for (auto & assignment : camera->assignments_) {
			int track_index = assignment[0];
			int detection_index = assignment[1];

			cv::Point2f cen = camera->centroids_[detection_index];
			float size = camera->sizes_[detection_index];
			std::shared_ptr<Track> track = camera->tracks_[track_index];

			// update kalman filter
			track->updateKF(cen);

			// update DCF
			track->checkDCF(cen, camera->frame_);
			
			// update track info
			track->size_ = size;
			track->age_++;
			track->totalVisibleCount_++;
			track->consecutiveInvisibleCount_ = 0;
		}
	}

	/**
	 * This function updates the unassigned tracks obtained from our detection_to_track_assignments.
	 * we process the tracks do not have an existing matched detection in the current frame by 
	 * increasing their consecutive invisible count. It also gets any track that has been invisible
	 * for too long, and stores them in the vector tracks_to_be_removed_
	 */
	void update_unassigned_tracks(std::shared_ptr<Camera> & camera)
	{
		int invisible_for_too_long = int(CONSECUTIVE_THRESH_ * VIDEO_FPS_);
		int age_threshold = int(AGE_THRESH_ * VIDEO_FPS_);

		for (auto & track_index : camera->unassigned_tracks_) {
			std::shared_ptr<Track> track = camera->tracks_[track_index];
			track->age_++;
			track->consecutiveInvisibleCount_++;
			
			float visibility = float(track->totalVisibleCount_) / float(track->age_);

			// if invisible for too long, append track to be removed
			if ((track->age_ < age_threshold && visibility < VISIBILITY_RATIO_) ||
					(track->consecutiveInvisibleCount_ >= invisible_for_too_long) ||
					(track->outOfSync_ == true)) {
				camera->tracks_to_be_removed_.push_back(track_index);
			}
		}
	}

	/**
	 * This function creates new tracks for detections that are not assigned to any existing
	 * track. We will initialize a new Track with the location of the detection
	 */
	void create_new_tracks(std::shared_ptr<Camera> & camera)
	{
		for (auto & unassigned_detection : camera->unassigned_detections_) {
			cv::Point2f cen = camera->centroids_[unassigned_detection];
			float size = camera->sizes_[unassigned_detection];
			// initialize new track
			auto new_track = std::shared_ptr<Track>(
				new Track(camera->next_id_, size, cen, VIDEO_FPS_, SEC_FILTER_DELAY_));
			camera->tracks_.push_back(new_track);
			camera->next_id_++;
		}
	}

	/**
	 * This function removes the tracks to be removed, in the vector tracks_to_be_removed_
	 */
	void delete_lost_tracks(std::shared_ptr<Camera> & camera)
	{
		for (auto & track_index : camera->tracks_to_be_removed_) {
			camera->dead_tracks_.push_back(track_index);
			camera->tracks_.erase(camera->tracks_.begin() + track_index);
		}
	}

	/**
	 * This function filters out good tracks to be considered for re-identification. it also
	 * filters out noise and only shows tracks that are considered "good" in our output processed
	 * frames. it draws bounding boxes into these tracks into our camera frame to continuously 
	 * identify and track the detected good tracks
	 */
	std::vector<std::shared_ptr<Track>> filter_tracks(std::shared_ptr<Camera> & camera)
	{
		std::vector<std::shared_ptr<Track>> good_tracks;
		int min_track_age = int(std::max((AGE_THRESH_ * VIDEO_FPS_), float(30.0)));
		int min_visible_count = int(std::max((VISIBILITY_THRESH_ * VIDEO_FPS_), float(30.0)));

		for (auto & track : camera->tracks_) {
			// requirement for track to be considered in re-identification
			// note that min no. of frames being too small may lead to loss of continuous tracking
			if (track->age_ > min_track_age && track->totalVisibleCount_ > min_visible_count
			&& track->consecutiveInvisibleCount_ <= 5 && track->centroid_.x >= 0 && track->centroid_.x <= FRAME_WIDTH_ 
			&& track->centroid_.y >= 0 && track->centroid_.y <= FRAME_HEIGHT_) {
				
				track->is_goodtrack_ = true;
				good_tracks.push_back(track);			
			}
		}
		return good_tracks;
	}

	/**
	 * This function calculates the euclidean distance between two points
	 */
	float euclideanDist(cv::Point2f & p, cv::Point2f & q) {
		cv::Point2f diff = p - q;
		return sqrt(diff.x*diff.x + diff.y*diff.y);
	}

	/**
	 * This function applies hungarian algorithm to obtain the optimal assignment for 
	 * our cost matrix of tracks and detections
	 */
	std::vector<int> apply_hungarian_algo(
		std::vector<std::vector<double>> & cost_matrix) {
		// declare function variables
		HungarianAlgorithm hungAlgo;
		std::vector<int> assignment;

		hungAlgo.Solve(cost_matrix, assignment);
		return assignment;
	}

	/**
	 * This function calculates the average brightness value of the frame
	 */
	int average_brightness(std::shared_ptr<Camera> & camera) {
		// declare and initialize required variables
		cv::Mat hist;
		int bins = 16;
		float hrange[] = {0, 256};
		const float* range = {hrange};
		float weighted_sum = 0;

		// get grayscale frame and calculate histogram
		cv::cvtColor(camera->frame_, camera->gray_, cv::COLOR_BGR2GRAY);
		cv::calcHist(&camera->gray_, 1, 0, cv::Mat(), hist, 1, &bins, &range, true, false);
		cv::Scalar total_sum = cv::sum(hist);

		// iterate through each bin
		for (int i=0; i < 16; i++) {
			weighted_sum += (i * (hist.at<float>(i)));
		}
		return int((weighted_sum/total_sum.val[0]) * (256/16));
	}


	/**
	 * This function closes the cameras after the program ends.
	 */
	void close_cameras() {
		for (int cam_idx = 0; cam_idx < NUM_OF_CAMERAS_; cam_idx++) {
			if (IS_REALTIME_ == 2) { // To remove when interface shifts
				cameras_[cam_idx]->edgecam_cap_->reset_receivers();
			}
			else {
				cameras_[cam_idx].get()->cap_.release();
			}
		}
	}

}