/**
 * @file multi_cam_detect_utils.hpp
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
 * This file contains the declaration of the classes (Camera and Track) and their
 * associated methods, which is primarily used in the detection pipeline.
 * The full definition of the classes and their methods can be found in the
 * file multi_cam_detect_utils.cpp. 
 */

#ifndef MULTI_CAM_DETECT_UTILS_HPP_
#define MULTI_CAM_DETECT_UTILS_HPP_

// opencv header files
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/ximgproc.hpp>
#include <opencv2/tracking.hpp>
#include <WSrtInterface.hpp>
#include <yolo.hpp>

// standard package imports
#include <string>
#include <memory>
#include <chrono>
#include <vector>
#include <list>
#include <array>

namespace mcmt {
	
	class Track {
		public:
			Track(
				int track_id, 
				float size,
				cv::Point2f centroid,
				int video_fps,
				float sec_filter_delay);

			
		virtual ~Track() {}
			
			// video parameters
			int vid_fps_;
			float sec_filter_delay_;

			// variable to store predicted and actual locations from kf
			cv::Point2f centroid_, predicted_;

			cv::Point2f previous; // Last known point
			int vel_mag, vel_angle; // Velocity Variables

			int vel_angle_leeway; // Angle for high velocity search zone
			int circle_step; // Radius of search circle
			int frame_step; // How long to keep looking for high velocity
			int search_frame_counter; // Counter for search zone timings
			int vel_threshold; // velocity threshold between high and low velocity

			// declare DCF bool variable
			bool is_dcf_init_, outOfSync_;

			// size of detected blob
			float size_;

			// yolo detection class id and confidence
			int yolo_class_id_;
			float yolo_confidence_;

			// declare tracking variables
			int id_, age_, totalVisibleCount_, consecutiveInvisibleCount_;
			bool is_goodtrack_;

			// declare class functions
			void predictKF();
			void updateKF(cv::Point2f & measurement);
			std::vector<cv::Point2f> search_polygon();
			void predictDCF(cv::Mat & frame);
			void checkDCF(cv::Point2f & measurement, cv::Mat & frame);

			// declare kf variables
			std::shared_ptr<cv::KalmanFilter> kf_;

			// declare dcf variables
			cv::Ptr<cv::Tracker> tracker_;
			#if CV_VERSION_MAJOR >= 4 && CV_VERSION_MINOR > 2 // OpenCV 4.5.2 uses cv::Rect
				cv::Rect box_;
			#else
				cv::Rect2d box_;
			#endif
	};

	class Camera {
		public:
			Camera(
				int cam_index,
				std::string video_input
			);

		virtual ~Camera() {}

			// declare video parameters
			cv::VideoCapture cap_;
			#if WSRT_ENABLED
				std::shared_ptr<WSrt> edgecam_cap_; // To remove when interface shifts
			#endif
			cv::VideoWriter recording_;
			cv::Mat frame_, frame_ec_, gray_;
			std::array<cv::Mat, 2> masked_;
			std::string video_input_;
			int cam_index_, frame_w_, frame_h_, fps_, next_id_;
			float scale_factor_, aspect_ratio_;
			bool downsample_;

			// declare tracking variables
			std::vector<std::shared_ptr<Track>> tracks_, good_tracks_;
			std::vector<int> dead_tracks_;

			// declare detection variables
			std::vector<float> sizes_;
			std::vector<cv::Point2f> centroids_;

			// declare yolo detection variables
			std::vector<int> yolo_class_ids_;
			std::vector<float> yolo_confidences_;

			// declare tracking variables
			std::vector<int> unassigned_tracks_, unassigned_detections_;
			std::vector<int> unassigned_tracks_kf_, unassigned_detections_kf_;
			std::vector<int> unassigned_tracks_dcf_, unassigned_detections_dcf_;
			
			// we store the matched track index and detection index in the assigments vector
			std::vector<std::vector<int>> assignments_;
			std::vector<std::vector<int>> assignments_kf_;
			std::vector<std::vector<int>> assignments_dcf_;
			std::vector<int> tracks_to_be_removed_;

			// declare blob detector and background subtractor
			cv::Ptr<cv::SimpleBlobDetector> detector_;
			std::array<cv::Ptr<cv::BackgroundSubtractorMOG2>, 2> fgbg_;

			// Simple background Subtractor and other variables
			cv::Ptr<cv::BackgroundSubtractorMOG2> simple_MOG2;
			cv::Ptr<cv::BackgroundSubtractorMOG2> simple_MOG2_ec;
			cv::Mat foreground_mask;
			cv::Mat foreground_mask_ec;

			// Contour detection parameters
			std::vector<std::vector<cv::Point>> current_frame_contours;
			std::vector<cv::Vec4i> hierarchy;
			cv::Point2f contour_center;
			float contour_radius;

			// Yolo detection parameters
			std::shared_ptr<YoloDetector> yolo_detector;
			std::vector<yolo_object> yolo_objects;

			// declare class functions
			// bool get_frame();
			void clear_detection_variables();
	};
}

#endif			// MULTI_CAM_DETECT_UTILS_HPP_