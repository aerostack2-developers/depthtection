/**
 * @file depthtection.hpp
 * @brief Header file for the depthtection class.
 * @author Miguel Fernandez-Cortizas miguel.fernandez.cortizas@upm.es
 * @date June 2022
 */

#ifndef __DEPTHTECION_HPP__
#define __DEPTHTECION_HPP__

#include <algorithm>
#include <candidate.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/qos.hpp>
#include <vector>

#include "as2_msgs/msg/pose_stamped_with_id.hpp"
#include "cv_bridge/cv_bridge.h"
#include "nav_msgs/msg/odometry.hpp"
#include "pcl/common/common.h"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl_ros/transforms.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/fluid_pressure.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "tf2/LinearMath/Transform.h"
#include "tf2/exceptions.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2_msgs/msg/tf_message.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "vision_msgs/msg/detection2_d_array.hpp"

class Depthtection : public rclcpp::Node {
  enum Phase {
    NO_DETECTION,
    VISUAL_DETECTION_WITHOUT_DEPTH,
    VISUAL_DETECTION_WITH_DEPTH,
    ONLY_DEPTH_DETECTION,
    TOO_NEAR_TO_DETECT,
  } current_phase_;

  // Camera calibration information
  cv::Size imgSize_;
  cv::Mat K_, D_;
  bool haveCalibration_;

  // Sensor TFs
  std::string base_frame_;
  bool tfCamCatched_, tfImuCatched_;
  tf2::Stamped<tf2::Transform> camBaseTf, imuBaseTf;
  std::shared_ptr<tf2_ros::TransformListener> tfListener_{nullptr};
  std::unique_ptr<tf2_ros::Buffer> tfBuffer_;

  // flags
  bool show_detection_;
  double height_estimation_;
  cv::Mat rgb_img_, depth_img_;

  std::vector<Candidate::Ptr> candidates_;
  Candidate::Ptr best_candidate_;

  int n_images_without_detection_ = 0;
  bool new_detection_ = false;

  std::string target_detection_;
  // Messages

  vision_msgs::msg::Detection2DArray detection_msg_;
  geometry_msgs::msg::PoseStamped best_pose_msg_;
  geometry_msgs::msg::PoseStamped only_depth_pose_msg_;
  geometry_msgs::msg::PoseStamped visual_detection_pose_msg_;
  geometry_msgs::msg::PoseStamped visual_depth_detection_pose_msg_;

  // Data subscribers
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr rgb_img_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_img_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_sub_;
  rclcpp::Subscription<vision_msgs::msg::Detection2DArray>::SharedPtr detection_sub_;
  // rclcpp::Subscription<sensor_msgs::msg::FluidPressure>::SharedPtr pressure_sub_;

  // Data publishers
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;

  // Methods

  public:
  Depthtection();
  ~Depthtection(void);

  private:
  bool getBestPose(geometry_msgs::msg::PoseStamped& best_pose) {
    switch (current_phase_) {
      case Phase::NO_DETECTION:
        return false;
      case Phase::VISUAL_DETECTION_WITHOUT_DEPTH: {
        best_pose = visual_detection_pose_msg_;
        return true;
      }
      case Phase::VISUAL_DETECTION_WITH_DEPTH: {
        best_pose = visual_depth_detection_pose_msg_;
        return true;
      }
      case Phase::ONLY_DEPTH_DETECTION: {
        best_pose = only_depth_pose_msg_;
        return true;
      }
      case Phase::TOO_NEAR_TO_DETECT: {
        return false;
      }
    }
    return false;
  }

  void pubPose() {
    if (getBestPose(best_pose_msg_)) {
      pose_pub_->publish(best_pose_msg_);
    }
  }

  geometry_msgs::msg::PointStamped extractEstimatedPoint(const cv::Mat& depth_img,
                                                         const vision_msgs::msg::Detection2D& msg);
  bool updateCandidateFromPointCloud(const Candidate::Ptr& candidate,
                                     const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);
  // Subscribers callbacks
  void rgbImageCallback(const sensor_msgs::msg::Image::SharedPtr msg);
  void depthImageCallback(const sensor_msgs::msg::Image::SharedPtr msg);
  void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg);
  void detectionCallback(const vision_msgs::msg::Detection2DArray::SharedPtr msg);
  void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
};

#endif
