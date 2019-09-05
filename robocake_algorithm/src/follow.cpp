#include "algorithm.h"

geometry_msgs::Twist control(
                     std::unordered_map<std::string, double> proximity,
                     std::unordered_map<std::string, double> reflectance,
                     double threshold) {
  geometry_msgs::Twist msg;
  msg.angular.z = 1;
  return msg;
}
