#ifndef __CANDIDATE_HPP__
#define __CANDIDATE_HPP__

#include <Eigen/Dense>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <memory>

struct Candidate {
  typedef std::shared_ptr<Candidate> Ptr;
  typedef std::shared_ptr<const Candidate> ConstPtr;
  typedef std::vector<std::shared_ptr<Candidate>> Vec;

  int id;
  float confidence;
  std::string class_name;
  geometry_msgs::msg::PointStamped point;

  Candidate(int id, float confidence, std::string_view class_name, geometry_msgs::msg::PointStamped point)
      : id(id), confidence(confidence), class_name(class_name), point(point) {}

  Eigen::Vector3d getEigen() const { return Eigen::Vector3d(point.point.x, point.point.y, point.point.z); }

  void updatePoint(geometry_msgs::msg::PointStamped point, bool filter = true) {
    if (filter) {
      static const float alpha = 0.1;
      this->point.point.x = alpha * this->point.point.x + (1 - alpha) * point.point.x;
      this->point.point.y = alpha * this->point.point.y + (1 - alpha) * point.point.y;
      this->point.point.z = alpha * this->point.point.z + (1 - alpha) * point.point.z;
      this->point.header = point.header;
    } else {
      this->point = point;
    }
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
