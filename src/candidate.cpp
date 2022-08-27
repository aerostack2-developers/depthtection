#include <candidate.hpp>

Candidate::Ptr match_candidate(const Candidate::Vec &candidate_list, std::string_view class_name,
                               geometry_msgs::msg::PointStamped point, double max_distance) {
  double min_distance = std::numeric_limits<double>::max();
  for (auto &candidate : candidate_list) {
    if (candidate->class_name == class_name) {
      double distance = std::sqrt(std::pow(point.point.x - candidate->point.point.x, 2) +
                                  std::pow(point.point.y - candidate->point.point.y, 2) +
                                  std::pow(point.point.z - candidate->point.point.z, 2));
      if (distance < min_distance && distance < max_distance) {
        min_distance = distance;
        return candidate;
      }
    }
  }
  return nullptr;
};

