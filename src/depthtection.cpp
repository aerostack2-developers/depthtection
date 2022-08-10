#include "depthtection.hpp"

#include <opencv2/core/matx.hpp>

using std::placeholders::_1;

static cv::Mat cropImageWithDetection(const cv::Mat &img,
                                      const vision_msgs::msg::Detection2D &detection);
static cv::Vec3f get_point_from_depth(const cv::Mat &depth_img, const cv::Point &point,
                                      const cv::Mat &K, const cv::Mat &D);

Depthtection::Depthtection() : Node("depthtection") {
  // Declare node parameters
  this->declare_parameter<std::string>("camera_topic", "camera");
  this->declare_parameter<std::string>("detection_topic", "detection");
  this->declare_parameter<std::string>("computed_pose_topic", "pose_computed");
  this->declare_parameter<std::string>("base_frame", "base_link");
  this->declare_parameter<bool>("show_detection", false);

  // Read parameters
  std::string camera_topic, detection_topic, computed_pose_topic;

  this->get_parameter("camera_topic", camera_topic);
  this->get_parameter("detection_topic", detection_topic);
  this->get_parameter("base_frame", base_frame_);
  this->get_parameter("show_detection", show_detection_);
  this->get_parameter("computed_pose_topic", computed_pose_topic);

  // Check topic name format
  if (camera_topic.back() == '/') camera_topic.pop_back();

  // Topic subscription
  if (show_detection_) {
    RCLCPP_INFO(this->get_logger(), "Show_detection enabled: Subscribing to %s",
                camera_topic.c_str());
    rgb_img_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        camera_topic + "/image_raw", 10, std::bind(&Depthtection::rgbImageCallback, this, _1));
  }
  depth_img_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      camera_topic + "/depth", 10, std::bind(&Depthtection::depthImageCallback, this, _1));
  camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
      camera_topic + "/camera_info", rclcpp::SensorDataQoS(),
      std::bind(&Depthtection::cameraInfoCallback, this, _1));
  detection_sub_ = this->create_subscription<vision_msgs::msg::Detection2DArray>(
      detection_topic, rclcpp::SensorDataQoS(),
      std::bind(&Depthtection::detectionCallback, this, _1));

  // Topic publication
  pose_pub_ = this->create_publisher<as2_msgs::msg::PoseStampedWithID>(computed_pose_topic,
                                                                       rclcpp::QoS(10));

  // TF listening
  tfCamCatched_ = false;
  tfImuCatched_ = false;
  tfBuffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tfListener_ = std::make_shared<tf2_ros::TransformListener>(*tfBuffer_);
}

Depthtection::~Depthtection(void) { cv::destroyAllWindows(); }

void Depthtection::rgbImageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
  // convert to cv::Mat
  cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  rgb_img_ = cv_ptr->image;
}

void Depthtection::depthImageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
  // convert to cv::Mat
  cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);
  depth_img_ = cv_ptr->image;
}

void Depthtection::cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
  if (!haveCalibration_) {
    K_ = cv::Mat(3, 3, CV_64FC1, (void *)msg->k.data()).clone();
    D_ = cv::Mat(msg->d.size(), 1, CV_64FC1, (void *)msg->d.data()).clone();
    imgSize_.width = msg->width;
    imgSize_.height = msg->height;
    haveCalibration_ = true;
  }
}

void Depthtection::detectionCallback(const vision_msgs::msg::Detection2DArray::SharedPtr msg) {
  if (show_detection_) {
    // check if image is available
    if (rgb_img_.empty()) {
      RCLCPP_WARN(this->get_logger(), "No RGB image available");
      return;
    }
    for (auto &detection : msg->detections) {
      auto center = detection.bbox.center;
      auto width = detection.bbox.size_x;
      auto height = detection.bbox.size_y;
      cv::rectangle(rgb_img_, cv::Point(center.x - width / 2, center.y - height / 2),
                    cv::Point(center.x + width / 2, center.y + height / 2), cv::Scalar(0, 255, 0),
                    2);
      // add text with detection id
      cv::putText(rgb_img_, std::string(detection.id),
                  cv::Point(center.x - width / 2, center.y - height / 2), cv::FONT_HERSHEY_SIMPLEX,
                  0.5, cv::Scalar(0, 255, 0), 1);
    }
    if (! depth_img_.empty()) {
      if (!haveCalibration_){
        RCLCPP_WARN(this->get_logger(), "No camera calibration available");
        return;
      }

      auto poses = extractEstimatedPose(depth_img_, *(msg.get()));
      for (auto &pose : poses) {
        cv::Vec3f point(pose.pose.position.x,
                        pose.pose.position.y,
                        pose.pose.position.z);
        RCLCPP_INFO(this->get_logger(), "Point: %f %f %f", point[0], point[1], point[2]);
        // // project 3d point into 2d image and draw a circle at the projected points
        // cv::Vec2i projected_points;
        // cv::projectPoints(cv::Mat(1, 1, CV_32FC3, point.val), cv::Mat(), cv::Mat(), K_, D_,
        //                   projected_points);
        // cv::circle(rgb_img_, projected_points, 5, cv::Scalar(0, 0, 255), -1);
      } 

    }
    // window resizable
    // TODO: IMPROVE THIS
    static bool first_time_ = true;
    if (first_time_) {
      cv::namedWindow("rgb_img", cv::WINDOW_NORMAL);
      cv::resizeWindow("rgb_img", imgSize_.width, imgSize_.height);
      first_time_ = false;
    }

    cv::imshow("rgb_img", rgb_img_);
    cv::waitKey(1);
  }
}

std::vector<as2_msgs::msg::PoseStampedWithID> Depthtection::extractEstimatedPose(
    const cv::Mat &depth_img, const vision_msgs::msg::Detection2DArray &detections) {
  std::vector<as2_msgs::msg::PoseStampedWithID> poses;
  as2_msgs::msg::PoseStampedWithID pose;
  poses.reserve(detections.detections.size());

  for (auto &detection : detections.detections) {
    auto center = detection.bbox.center;
    // auto width = detection.bbox.size_x;
    // auto height = detection.bbox.size_y;
    auto point = get_point_from_depth(depth_img, cv::Point(center.x, center.y), K_, D_);
    if (point[2] == 0) continue;
    pose.pose.position.x = point[0];
    pose.pose.position.y = point[1];
    pose.pose.position.z = point[2];
    pose.pose.orientation.w = 1;
    pose.pose.orientation.x = 0;
    pose.pose.orientation.y = 0;
    pose.pose.orientation.z = 0;
    pose.id = detection.id;
    poses.emplace_back(pose);
  }
  return poses;
}

// Auxiliary function to crop image with detection

cv::Mat cropImageWithDetection(const cv::Mat &img, const vision_msgs::msg::Detection2D &detection) {
  auto center = detection.bbox.center;
  auto width = detection.bbox.size_x;
  auto height = detection.bbox.size_y;
  return img(cv::Rect(center.x - width / 2, center.y - height / 2, width, height));
}

// obtain 3D point from depth image
cv::Vec3f get_point_from_depth(const cv::Mat &depth_img, const cv::Point &point, const cv::Mat &K,
                               const cv::Mat &D) {
  double depth = depth_img.at<float>(point.y, point.x);
  if (depth == 0 || depth == std::numeric_limits<float>::infinity()) {
    return cv::Vec3f(0, 0, 0);
  }
  // undistort point using camera calibration
  
  // TODO: check distortion model
  cv::Vec2f point_undistorted;

  auto cx = K.at<double>(0, 2);
  auto cy = K.at<double>(1, 2);
  auto fx = K.at<double>(0, 0);
  auto fy = K.at<double>(1, 1);

  auto x = (point.x - cx) * depth / fx;
  auto y = (point.y - cy) * depth / fy;
  auto z = depth;

  return cv::Vec3f(x, y, z);

  // RCLCPP_WARN(rclcpp::get_logger("a"),"depth: %f", depth);

  // // undistort point
  // cv::Vec2d point_undistorted;
  // cv::undistortPoints(cv::Mat(1, 1, CV_32FC2, cv::Scalar(point.x, point.y)), point_undistorted, K, D);

  // // project point into 3d space
  // cv::Vec3f point_3d;
  // point_3d[0] = point_undistorted[0] * depth;
  // point_3d[1] = point_undistorted[1] * depth;
  // point_3d[2] = depth;
  // return point_3d;

  // cv::Mat point_3d =
  //     (cv::Mat_<double>(4, 1) << (point.x - K.at<double>(0, 2)) * depth / K.at<double>(0, 0),
  //      (point.y - K.at<double>(1, 2)) * depth / K.at<double>(1, 1), depth, 1);

  // cv::Mat point_3d_undistorted = K.inv() * point_3d;
  // point_3d_undistorted /= point_3d_undistorted.at<double>(3, 0);
  // cv::Mat point_3d_undistorted_normalized = D.inv() * point_3d_undistorted;
  // point_3d_undistorted_normalized /= point_3d_undistorted_normalized.at<double>(3, 0);
  // return cv::Vec3f(point_3d_undistorted_normalized.at<double>(0, 0),
  //                  point_3d_undistorted_normalized.at<double>(1, 0),
  //                  point_3d_undistorted_normalized.at<double>(2, 0));
}

