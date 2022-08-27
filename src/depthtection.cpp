#include "depthtection.hpp"

#include <geometry_msgs/msg/detail/pose_stamped__struct.hpp>
#include <opencv2/core/matx.hpp>

static void showImage(const std::string &title, const cv::Mat &img) {
  static std::unordered_map<std::string, bool> window_created;
  if (window_created.find(title) == window_created.end()) {
    cv::namedWindow(title, cv::WINDOW_NORMAL);
    cv::resizeWindow(title, img.cols, img.rows);
    window_created[title] = true;
  }
  cv::imshow(title, img);
  cv::waitKey(1);
}

static cv::Mat cropImageWithDetection(const cv::Mat &img,
                                      const vision_msgs::msg::Detection2D &detection);
static cv::Vec3f get_point_from_depth(const cv::Mat &depth_img, const cv::Point &point,
                                      const cv::Mat &K, const cv::Mat &D);

Depthtection::Depthtection() : Node("depthtection") {
  // Declare node parameters
  this->declare_parameter<std::string>("camera_topic", "camera");
  this->declare_parameter<std::string>("detection_topic", "detection");
  this->declare_parameter<std::string>("computed_pose_topic", "pose_computed");
  this->declare_parameter<std::string>("barometer_topic", "air_pressure");
  this->declare_parameter<std::string>("base_frame", "base_link");
  this->declare_parameter<bool>("show_detection", false);

  // Read parameters
  std::string camera_topic, detection_topic, computed_pose_topic, barometer_topic;

  this->get_parameter("camera_topic", camera_topic);
  this->get_parameter("detection_topic", detection_topic);
  this->get_parameter("base_frame", base_frame_);
  this->get_parameter("show_detection", show_detection_);

  this->get_parameter("computed_pose_topic", computed_pose_topic);
  this->get_parameter("barometer_topic", barometer_topic);

  // Check topic name format
  if (camera_topic.back() == '/') camera_topic.pop_back();

  // Topic subscription
  if (show_detection_) {
    RCLCPP_INFO(this->get_logger(), "Show_detection enabled: Subscribing to %s",
                camera_topic.c_str());
    rgb_img_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        camera_topic + "/image_raw", 10,
        std::bind(&Depthtection::rgbImageCallback, this, std::placeholders::_1));
  }
  depth_img_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      camera_topic + "/depth", 10,
      std::bind(&Depthtection::depthImageCallback, this, std::placeholders::_1));
  camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
      camera_topic + "/camera_info", rclcpp::SensorDataQoS(),
      std::bind(&Depthtection::cameraInfoCallback, this, std::placeholders::_1));
  detection_sub_ = this->create_subscription<vision_msgs::msg::Detection2DArray>(
      detection_topic, rclcpp::SensorDataQoS(),
      std::bind(&Depthtection::detectionCallback, this, std::placeholders::_1));
  /* pressure_sub_ = this->create_subscription<sensor_msgs::msg::FluidPressure>(
      barometer_topic, rclcpp::SensorDataQoS(),
      std::bind(&Depthtection::fluidPressureCallback, this, std::placeholders::_1)); */

  // Topic publication
  pose_pub_ =
      this->create_publisher<geometry_msgs::msg::PoseStamped>(computed_pose_topic, rclcpp::QoS(10));

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
    if (!depth_img_.empty()) {
      if (!haveCalibration_) {
        RCLCPP_WARN(this->get_logger(), "No camera calibration available");
        return;
      }

      auto poses = extractEstimatedPose(depth_img_, *(msg.get()));
      for (auto &pose : poses) {
        cv::Vec3f point(pose.pose.position.x, pose.pose.position.y, pose.pose.position.z);
        RCLCPP_INFO(this->get_logger(), "Point: %f %f %f", point[0], point[1], point[2]);
      }
    }
    showImage("RGB Image", rgb_img_);
  }
}

std::vector<geometry_msgs::msg::PoseStamped> Depthtection::extractEstimatedPose(
    const cv::Mat &depth_img, const vision_msgs::msg::Detection2DArray &detections) {
  std::vector<geometry_msgs::msg::PoseStamped> poses;
  geometry_msgs::msg::PoseStamped pose;
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
    // pose.id = detection.id;
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
}

void Depthtection::pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
  // convert to pcl::PointCloud

  auto cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*msg, *cloud);

  // filter cloud when z > 0 in earth frame
  tf2::Stamped<tf2::Transform> earthTf;
  tf2::Stamped<tf2::Transform> base_frame_Tf;
  try {
    geometry_msgs::msg::TransformStamped tf;
    tf = tfBuffer_->lookupTransform("earth", msg->header.frame_id, tf2::TimePointZero);
    tf2::fromMsg(tf, earthTf);
    tf = tfBuffer_->lookupTransform(base_frame_, msg->header.frame_id, tf2::TimePointZero);
    tf2::fromMsg(tf, base_frame_Tf);
  } catch (tf2::TransformException &ex) {
    RCLCPP_ERROR_ONCE(this->get_logger(), "Could not transform %s to %s: %s", "earth",
                      msg->header.frame_id.c_str(), ex.what());
    return;
  }
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
  cloud_filtered->points.reserve(cloud->points.size());

  Eigen::Vector3d centroid(0, 0, 0);

  /* if (!have_visual_pose_estimation_) {
    return;
  } */

  for (auto &point : cloud->points) {
    tf2::Vector3 pointLidar(point.x, point.y, point.z);
    const tf2::Vector3 pointEarth = earthTf * pointLidar;
    const tf2::Vector3 pointBase = base_frame_Tf * pointLidar;

    // check NaN values
    if (!std::isfinite(pointEarth.x()) || !std::isfinite(pointEarth.y()) ||
        !std::isfinite(pointEarth.z())) {
      continue;
    }

    // point must be above the ground and near visual_pose estimation
    /* const double range_diff = 10.0;
    if (pointEarth.z() < 0.0 || std::abs(pointEarth.x() - visual_pose_estimation_.pose.position.x)
    > range_diff || std::abs(pointEarth.y() - visual_pose_estimation_.pose.position.y) >
    range_diff) { continue;
    } */

    cloud_filtered->points.emplace_back(pointEarth.x(), pointEarth.y(), pointEarth.z());

    centroid.x() = centroid.x() + pointEarth.x();
    centroid.y() = centroid.y() + pointEarth.y();
    centroid.z() = centroid.z() + pointEarth.z();
  }
  if (cloud_filtered->points.size() < 20) {
    return;
  }

  centroid /= cloud_filtered->points.size();

  if (!std::isfinite(centroid.x()) || !std::isfinite(centroid.y()) ||
      !std::isfinite(centroid.z())) {
    return;
  }

  // create msg PointCloud2 with the cloud_filtered points
  sensor_msgs::msg::PointCloud2 cloud_filtered_msg;
  pcl::toROSMsg(*cloud_filtered, cloud_filtered_msg);
  cloud_filtered_msg.header = msg->header;
  cloud_filtered_msg.header.frame_id = "earth";

  // publish filtered cloud
  static auto pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("cloud_filtered", 10);
  pub->publish(cloud_filtered_msg);

  /* have_lidar_pose_estimation_ = true;
  lidar_pose_estimation_.header = cloud_filtered_msg.header;
  lidar_pose_estimation_.pose.position.x = centroid.x();
  lidar_pose_estimation_.pose.position.y = centroid.y();
  lidar_pose_estimation_.pose.position.z = centroid.z(); */
  // RCLCPP_INFO(this->get_logger(), "Have lidar pose estimation [%f, %f, %f]", centroid.x(),
  // centroid.y(), centroid.z());
}

