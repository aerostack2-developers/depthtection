/**
 * @file depthtection.hpp
 * @brief Header file for the depthtection class.
 * @author Miguel Fernandez-Cortizas miguel.fernandez.cortizas@upm.es
 * @date June 2022
 */

#ifndef __DEPTHTECION_HPP__
#define __DEPTHTECION_HPP__

#include <algorithm>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/qos.hpp>
#include <vector>

#include "cv_bridge/cv_bridge.h"
#include "nav_msgs/msg/odometry.hpp"
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
#include "as2_msgs/msg/pose_stamped_with_id.hpp"

class Depthtection : public rclcpp::Node {
  public:
  /**
   * @brief Constructor
   */
  Depthtection();

  /** @brief Destructor */
  ~Depthtection(void);
  

  private:
  /**
   * @brief RGB image callback
   * @param msg RGB image message
   */
  void rgbImageCallback(const sensor_msgs::msg::Image::SharedPtr msg);

  /**
   * @brief Depth image callback
   * @param msg Depth image message
   */
  void depthImageCallback(const sensor_msgs::msg::Image::SharedPtr msg);
  

  /**
   * @brief Camera calibration info callback
   * @param msg Camera calibration message
   */
  void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg);

  /**
   * @brief detection callback
   * @param msg Detection message
   */
  void detectionCallback(const vision_msgs::msg::Detection2DArray::SharedPtr msg);

  std::vector<as2_msgs::msg::PoseStampedWithID> extractEstimatedPose(const cv::Mat& depth_img, const vision_msgs::msg::Detection2DArray& msg);
  
  
  // Data subscribers
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr rgb_img_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_img_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
  rclcpp::Subscription<vision_msgs::msg::Detection2DArray>::SharedPtr detection_sub_;

  // Camera calibration information
  cv::Size imgSize_;
  cv::Mat K_, D_;
  bool haveCalibration_;

  // Current odometry computation
  rclcpp::Publisher<as2_msgs::msg::PoseStampedWithID>::SharedPtr pose_pub_;

  // Sensor TFs
  std::string base_frame_;
  bool tfCamCatched_, tfImuCatched_;
  tf2::Stamped<tf2::Transform> camBaseTf, imuBaseTf;
  std::shared_ptr<tf2_ros::TransformListener> tfListener_{nullptr};
  std::unique_ptr<tf2_ros::Buffer> tfBuffer_;

  // flags
  bool show_detection_;
  cv::Mat rgb_img_, depth_img_;
  vision_msgs::msg::Detection2DArray detection_msg_;

};

#endif
