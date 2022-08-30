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
#include <rclcpp/time.hpp>

#define N_STEPS_PREDICTION 10
#define N_MEASURES_BEFORE_PREDICTION 2

struct Candidate {
  typedef std::shared_ptr<Candidate> Ptr;
  typedef std::shared_ptr<const Candidate> ConstPtr;
  typedef std::vector<std::shared_ptr<Candidate>> Vec;

  int id;
  float confidence;
  std::string class_name;
  geometry_msgs::msg::PointStamped point;
  geometry_msgs::msg::PointStamped raw_point;

  std::shared_ptr<rclcpp::Clock> clock;

  Candidate(int id, float confidence, std::string_view class_name, geometry_msgs::msg::PointStamped point,
            std::shared_ptr<rclcpp::Clock> clock)
      : id(id), confidence(confidence), class_name(class_name), point(point), clock(clock) {
    speed = Eigen::Vector3d::Zero();
    filtered_point = point;
    last_time = clock->now();
  }

  Eigen::Vector3d getEigen() const { return Eigen::Vector3d(point.point.x, point.point.y, point.point.z); }
  Eigen::Vector3d getFilteredEigen() const {
    return Eigen::Vector3d(filtered_point.point.x, filtered_point.point.y, filtered_point.point.z);
  }

  Eigen::Vector3d speed;
  geometry_msgs::msg::PointStamped filtered_point;
  geometry_msgs::msg::PointStamped compensated_point;
  double dt_cummulative = 0.0;
  int n_speed_measures = 0;

  rclcpp::Time last_time;

  void updatePoint(geometry_msgs::msg::PointStamped point, bool filter = true) {
    raw_point = point;
    if (filter) {
      const float alpha = 0.1;
      std::cout << "point " << point.point.x << " " << point.point.y << " " << point.point.z << std::endl;
      this->filtered_point.point.x = alpha * point.point.x + (1 - alpha) * this->filtered_point.point.x;
      this->filtered_point.point.y = alpha * point.point.y + (1 - alpha) * this->filtered_point.point.y;
      this->filtered_point.point.z = alpha * point.point.z + (1 - alpha) * this->filtered_point.point.z;
      this->filtered_point.header = point.header;
      std::cout << "filtered_point" << filtered_point.point.x << " " << filtered_point.point.y << " "
                << filtered_point.point.z << std::endl;
      const auto dt = getDt();
      this->point = filtered_point;

      if (dt_cummulative < 1.0) {
        dt_cummulative += dt;
        std::cout << "dt_cummulative " << dt_cummulative << std::endl;
      } else {
        estimateSpeed(dt_cummulative, this->getFilteredEigen());
        dt_cummulative = 0.0;
      }

      if (n_speed_measures > N_MEASURES_BEFORE_PREDICTION) {
        // if (false) {
        std::cout << "Filtering dt: " << dt << std::endl;
        this->compensated_point.point.x = this->filtered_point.point.x + speed.x() * N_STEPS_PREDICTION * 0.008;
        this->compensated_point.point.y = this->filtered_point.point.y + speed.y() * N_STEPS_PREDICTION * 0.008;
        this->compensated_point.point.z = this->filtered_point.point.z + speed.z() * N_STEPS_PREDICTION * 0.008;
        this->compensated_point.header = this->filtered_point.header;
      } 

    } else {
      this->point = point;
    }
    this->point.header = point.header;
  }


  double getDt() {
    rclcpp::Time now = clock->now();
    auto dt = (now - last_time).seconds();
    this->last_time = now;
    return dt;
  }

  Eigen::Vector3d last_position;
  void estimateSpeed(double dt, Eigen::Vector3d position) {
    if (dt <= 0) {
      std::cerr << "dt <= 0" << std::endl;
      return;
    }
    const float alpha = 0.1;
    if (n_speed_measures== 0) {
      last_position = position;
      n_speed_measures++;
      return;
    }
    if (n_speed_measures == 1) {
      speed.x() = (position.x() - last_position.x()) / dt;
      speed.y() = (position.y() - last_position.y()) / dt;
      speed.z() = (position.z() - last_position.z()) / dt;
    } else {
      speed.x() = alpha * (position.x() - last_position.x()) / dt + (1 - alpha) * speed.x();
      speed.y() = alpha * (position.y() - last_position.y()) / dt + (1 - alpha) * speed.y();
      speed.z() = alpha * (position.z() - last_position.z()) / dt + (1 - alpha) * speed.z();
      // speed = alpha * (position - last_position) / dt + (1 - alpha) * speed;
    }
    n_speed_measures++;
    last_position = position;
    std::cout << "speed: " << speed.transpose() << std::endl;
  }
  operator geometry_msgs::msg::PointStamped() & { return point; }
  double& x() { return point.point.x; }
  double& y() { return point.point.y; }
  double& z() { return point.point.z; }

  operator std::string() const {
    auto str = std::string("id: ") + std::to_string(id) + ", confidence: " + std::to_string(confidence) +
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
