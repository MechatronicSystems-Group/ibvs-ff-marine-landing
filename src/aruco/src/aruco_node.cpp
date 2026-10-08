#include "aruco_node.hpp"
#include <sstream>

// Aruco identification node, publishing image plane coordinates of points on the Aruco landing pad
ArucoNode::ArucoNode()
	: Node("aruco_node")
{

	// Declare parameters with default values
	this->declare_parameter<std::string>("image_topic", "/x500/camera/image");
	this->declare_parameter<std::string>("camera_info_topic", "/x500/camera/camera_info");

	this->declare_parameter<double>("otag_size", 1.50f);
	this->declare_parameter<double>("itag_size", 0.30f);

	// Retrieve the parameter values
	std::string image_topic = this->get_parameter("image_topic").as_string();
	std::string camera_info_topic = this->get_parameter("camera_info_topic").as_string();

	_otag_size = this->get_parameter("otag_size").as_double();
	_itag_size = this->get_parameter("itag_size").as_double();

	RCLCPP_INFO(this->get_logger(), "Image topic: %s", image_topic.c_str());
	RCLCPP_INFO(this->get_logger(), "Camera info topic: %s", camera_info_topic.c_str());
	RCLCPP_INFO(this->get_logger(), "Otag size: %f", _otag_size);
	RCLCPP_INFO(this->get_logger(), "Itag size: %f", _itag_size);

	auto detectorParams = cv::aruco::DetectorParameters();

	auto dictionary_6x6 = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
	_detector_6x6 = std::make_unique<cv::aruco::ArucoDetector>(dictionary_6x6, detectorParams);

	auto dictionary_4x4 = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_250);
	_detector_4x4 = std::make_unique<cv::aruco::ArucoDetector>(dictionary_4x4, detectorParams);

	auto qos = rclcpp::QoS(1).best_effort();

	// Node subscribers
	_image_sub = create_subscription<sensor_msgs::msg::Image>(
		image_topic, qos, std::bind(&ArucoNode::image_callback, this, std::placeholders::_1)
	);

	_camera_info_sub = create_subscription<sensor_msgs::msg::CameraInfo>(
		camera_info_topic, qos, std::bind(&ArucoNode::camera_info_callback, this, std::placeholders::_1)
	);

	// Node publishers
	_target_marker_pub = create_publisher<custom_msgs::msg::TagCorners>(
		"/tag/corners", qos
	);

	_otag_pose_pub = create_publisher<geometry_msgs::msg::PoseStamped>(
		"/tag/otag/camera_aruco_pose_estimate", qos
	);

	_itag_pose_pub = create_publisher<geometry_msgs::msg::PoseStamped>(
		"/tag/itag/camera_aruco_pose_estimate", qos
	);
}

void ArucoNode::camera_info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
{
	// Always update the camera matrix and distortion coefficients from the new message
	_camera_matrix = cv::Mat(3, 3, CV_64F, const_cast<double*>(msg->k.data())).clone();   // Use clone to ensure a deep copy
	_dist_coeffs = cv::Mat(msg->d.size(), 1, CV_64F, const_cast<double*>(msg->d.data())).clone();   // Use clone to ensure a deep copy

	// Log the expected image size
	RCLCPP_INFO(
		get_logger(),
		"CameraInfo image size: %u x %u",
		msg->width,
		msg->height
	);


	// Log the first row of the camera matrix to verify correct values
	RCLCPP_INFO(get_logger(), "Camera matrix updated:\n[%f, %f, %f]\n[%f, %f, %f]\n[%f, %f, %f]",
		    _camera_matrix.at<double>(0, 0), _camera_matrix.at<double>(0, 1), _camera_matrix.at<double>(0, 2),
		    _camera_matrix.at<double>(1, 0), _camera_matrix.at<double>(1, 1), _camera_matrix.at<double>(1, 2),
		    _camera_matrix.at<double>(2, 0), _camera_matrix.at<double>(2, 1), _camera_matrix.at<double>(2, 2));
	RCLCPP_INFO(get_logger(), "Camera Matrix: fx=%f, fy=%f, cx=%f, cy=%f",
		    _camera_matrix.at<double>(0, 0), // fx
		    _camera_matrix.at<double>(1, 1), // fy
		    _camera_matrix.at<double>(0, 2), // cx
		    _camera_matrix.at<double>(1, 2)  // cy
		   );

	// Check if camera focal length is 0 after update
	if (_camera_matrix.at<double>(0,0) == 0) 
	{
		RCLCPP_ERROR(get_logger(), "Focal length is zero after updating camera info");
	}
	else
	{
		RCLCPP_INFO(get_logger(), "Updated camera intrinsics from camer_info topic");
		RCLCPP_INFO(get_logger(), "Unsubscribing from camera info topic");
		_camera_info_sub.reset();
	}
}

void ArucoNode::image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
{

	// auto t_start = std::chrono::steady_clock::now();

	// Convert ROS2 message to OpenCV image mat
	cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

	// Detect markers
	std::vector<int> ids_6x6;
	std::vector<std::vector<cv::Point2f>> corners_6x6;
	_detector_6x6->detectMarkers(cv_ptr->image, corners_6x6, ids_6x6);
	
	std::vector<int> ids_4x4;
	std::vector<std::vector<cv::Point2f>> corners_4x4;
	_detector_4x4->detectMarkers(cv_ptr->image, corners_4x4, ids_4x4);
	
	// Populate pad_im_coord message with detected marker corners and publish to ROS2 topic
	custom_msgs::msg::TagCorners tag_corners;

	// Update tag corners header with timestamp
	tag_corners.header.stamp = msg->header.stamp;
	
	if (!_dist_coeffs.empty() && !_camera_matrix.empty()) {
		if (!ids_6x6.empty() && ids_6x6[0] == 19) { // Outer tag detected
			// RCLCPP_INFO(this->get_logger(), "19");
			tag_corners.otag = true;
			update_corners(corners_6x6, tag_corners.otag_corners);
			estimate_pose(corners_6x6, msg->header, _otag_pose_pub, _otag_size);
		}

		if (!ids_4x4.empty() && ids_4x4[0] == 0) { // Inner tag detected
			// RCLCPP_INFO(this->get_logger(), "0");
			tag_corners.itag = true;
			update_corners(corners_4x4, tag_corners.itag_corners);
			estimate_pose(corners_4x4, msg->header, _itag_pose_pub, _itag_size);
		}
	}
	_target_marker_pub->publish(tag_corners);

	// auto t_end = std::chrono::steady_clock::now();
	// double elapsed_ms = std::chrono::duration<double, std::milli>(t_end - t_start).count();
	// RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "image_callback took %.2f ms", elapsed_ms);
}

void ArucoNode::update_corners(const std::vector<std::vector<cv::Point2f>> corners, std::array<geometry_msgs::msg::Point,4>& msg_field) {
	for (int i=0; i<4; i++) {
		msg_field[i].x = corners[0][i].x;
		msg_field[i].y = corners[0][i].y;
	}
}

void ArucoNode::estimate_pose(
	const std::vector<std::vector<cv::Point2f>> corners,
	const std_msgs::msg::Header header,
	rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr _pub,
	float tag_size
	) 
	{

	std::vector<std::vector<cv::Point2f>> undistortedCorners;

	for (const auto& corner : corners) {
		std::vector<cv::Point2f> undistortedCorner;
		cv::undistortPoints(corner, undistortedCorner, _camera_matrix, _dist_coeffs, cv::noArray(), _camera_matrix);
		undistortedCorners.push_back(undistortedCorner);
	}

	// Calculate marker size from camera intrinsics
	float half_size = tag_size / 2.0f;
	std::vector<cv::Point3f> objectPoints = {
		cv::Point3f(-half_size,  half_size, 0),  // top left
		cv::Point3f(half_size,  half_size, 0),   // top right
		cv::Point3f(half_size, -half_size, 0),   // bottom right
		cv::Point3f(-half_size, -half_size, 0)   // bottom left
	};

	// Use PnP solver to estimate pose
	cv::Vec3d rvec, tvec;
	cv::solvePnP(objectPoints, undistortedCorners[0], _camera_matrix, cv::noArray(), rvec, tvec);

	// Quaternion from rotation matrix
	cv::Mat rot_mat;
	cv::Rodrigues(rvec, rot_mat);
	cv::Quatd quat = cv::Quatd::createFromRotMat(rot_mat).normalize();

	// Publish target pose
	geometry_msgs::msg::PoseStamped pose_msg;
	pose_msg.header.stamp = header.stamp;
	pose_msg.header.frame_id = "x500_custom_0/camera_frame";
	pose_msg.pose.position.x = tvec[0];
	pose_msg.pose.position.y = tvec[1];
	pose_msg.pose.position.z = tvec[2];
	pose_msg.pose.orientation.x = quat.x;
	pose_msg.pose.orientation.y = quat.y;
	pose_msg.pose.orientation.z = quat.z;
	pose_msg.pose.orientation.w = quat.w;

	_pub->publish(pose_msg);

}

int main(int argc, char** argv)
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<ArucoNode>());
	rclcpp::shutdown();
	return 0;
}