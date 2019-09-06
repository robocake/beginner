#include "algorithm.h"

int search_direction{1};

geometry_msgs::Twist control(
                     std::unordered_map<std::string, double> proximity,
                     std::unordered_map<std::string, double> reflectance,
                     double threshold) {
  geometry_msgs::Twist msg;
  double distance = -1;
  if (proximity["front"] < 0.8) {
    distance = proximity["front"];
    msg.angular.z = 0;
  } else if (proximity["left"] < 0.8) {
    distance = proximity["left"];
    search_direction = 1;
    msg.angular.z = 1;
  } else if (proximity["right"] < 0.8) {
    distance = proximity["right"];
    search_direction = -1;
    msg.angular.z = -1;
  }
  if (distance > 0) {
    auto v = std::min(std::abs(distance - threshold) / 0.05, 1.0) * 0.2;
    msg.linear.x = distance < threshold ? -v : v;
  } else {
    msg.angular.z = search_direction * 0.5;
  }
  return msg;
}
