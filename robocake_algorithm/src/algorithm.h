#include <geometry_msgs/Twist.h>

#include <unordered_map>

geometry_msgs::Twist control(
                     std::unordered_map<std::string, double> proximity,
                     std::unordered_map<std::string, double> reflectance,
                     double threshold);
