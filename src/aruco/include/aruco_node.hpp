#pragma once
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <std_msgs/msg/bool.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/core/quaternion.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <custom_msgs/msg/tag_corners.hpp>

class ArucoNode : public rclcpp::Node
{
public:
	ArucoNode();

private:
	void image_callback(const sensor_msgs::msg::Image::SharedPtr msg);
	void camera_info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg);

	void update_corners(const std::vector<std::vector<cv::Point2f>> corners, std::array<geometry_msgs::msg::Point,4>& msg_field);
	void estimate_pose(const std::vector<std::vector<cv::Point2f>> corners, const std_msgs::msg::Header header, rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr _pub, float tagSize);

	rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr _image_sub;
	rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr _camera_info_sub;

	rclcpp::Publisher<custom_msgs::msg::TagCorners>::SharedPtr _target_marker_pub;
	rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr _otag_pose_pub;
	rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr _itag_pose_pub;
	
	std::unique_ptr<cv::aruco::ArucoDetector> _detector_6x6;
	std::unique_ptr<cv::aruco::ArucoDetector> _detector_4x4;
	cv::Mat _camera_matrix;
	cv::Mat _dist_coeffs;

	double _otag_size;
	double _itag_size;
};
