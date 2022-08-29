#ifndef __CANDIDATE_HPP__
#define __CANDIDATE_HPP__

#include <Eigen/Dense>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <iostream>
#include <memory>
#include <rclcpp/clock.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>

#define N_STEPS_PREDICTION 10

struct Candidate {
  typedef std::shared_ptr<Candidate> Ptr;
  typedef std::shared_ptr<const Candidate> ConstPtr;
  typedef std::vector<std::shared_ptr<Candidate>> Vec;

  int id;
  float confidence;
  std::string class_name;
  geometry_msgs::msg::PointStamped point;

  std::shared_ptr<rclcpp::Clock> clock;

  Candidate(int id, float confidence, std::string_view class_name, geometry_msgs::msg::PointStamped point,
            std::shared_ptr<rclcpp::Clock> clock)
      : id(id), confidence(confidence), class_name(class_name), point(point), clock(clock) {
    speed = Eigen::Vector3d::Zero();
  }

  Eigen::Vector3d getEigen() const { return Eigen::Vector3d(point.point.x, point.point.y, point.point.z); }
  Eigen::Vector3d getRawEigen() const {
    return Eigen::Vector3d(raw_point.point.x, raw_point.point.y, raw_point.point.z);
  }
  Eigen::Vector3d getFilteredEigen() const {
    return Eigen::Vector3d(filtered_point.point.x, filtered_point.point.y, filtered_point.point.z);
  }

  Eigen::Vector3d speed;
  int n_measures = 0;
  geometry_msgs::msg::PointStamped raw_point;
  geometry_msgs::msg::PointStamped filtered_point;

  void updatePoint(geometry_msgs::msg::PointStamped point, bool filter = true) {
    n_measures += 1;
    if (filter) {
      static const float alpha = 0.01;
      this->point.point.x = alpha * point.point.x + (1 - alpha) * this->point.point.x;
      this->point.point.y = alpha * point.point.y + (1 - alpha) * this->point.point.y;
      this->point.point.z = alpha * point.point.z + (1 - alpha) * this->point.point.z;
      this->point.header = point.header;
      this->filtered_point = this->point;
      const auto dt = getDt();
      estimateSpeed(dt);
      if (n_measures > 10) {
        std::cout << "Filtering dt: " << dt << std::endl;
        this->filtered_point.point.x = this->point.point.x + speed(0) * N_STEPS_PREDICTION * dt;
        this->filtered_point.point.y = this->point.point.y + speed(1) * N_STEPS_PREDICTION * dt;
        this->filtered_point.point.z = this->point.point.z + speed(2) * N_STEPS_PREDICTION * dt;
      }

    } else {
      this->point = point;
    }
    this->raw_point = point;
  }

  double getDt() const {
    static auto last_time = clock->now();
    auto now = clock->now();
    auto dt = (now - last_time).seconds();
    last_time = now;
    return dt;
  }

  void estimateSpeed(double dt) {
    if (dt <= 0) {
      return;
    }
    static const float alpha = 0.01;
    static auto last_position = getFilteredEigen();
    speed = alpha * (getFilteredEigen() - last_position) / dt + (1 - alpha) * speed;
    last_position = getFilteredEigen();
    std::cout << "speed: " << speed.transpose() << std::endl;
  }
  operator geometry_msgs::msg::PointStamped() & { return point; }
  double& x() { return point.point.x; }
  double& y() { return point.point.y; }
  double& z() { return point.point.z; }

  operator std::string() const {
    static auto str = std::string("id: ") + std::to_string(id) + ", confidence: " + std::to_string(confidence) +
                      ", class_name: " + std::string(class_name) + ", point: " + std::to_string(point.point.x) + ", " +
                      std::to_string(point.point.y) + ", " + std::to_string(point.point.z);
    return str;
  }

  std::string to_string() const { return std::string(*this); }

  friend std::ostream& operator<<(std::ostream& os, const Candidate& candidate) {
    os << candidate.to_string();
    return os;
  }

  operator geometry_msgs::msg::PoseStamped() & {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = point.header;
    pose.pose.position = point.point;
    pose.pose.orientation = geometry_msgs::msg::Quaternion();
    return pose;
  }
};

Candidate::Ptr match_candidate(const Candidate::Vec& candidate_list, std::string_view class_name,
                               geometry_msgs::msg::PointStamped point,
                               double max_distance = std::numeric_limits<double>::max());

#endif  // __CANDIDATE_HPP__
