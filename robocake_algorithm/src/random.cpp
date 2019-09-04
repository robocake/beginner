#include <ros/ros.h>
#include <sensor_msgs/Range.h>
#include <std_msgs/Float64.h>

#include <unordered_map>

std::unordered_map<std::string, double> proximity, reflectance;

void update_proximity(std::string direction, double value) {
  proximity[direction] = value;
}

void update_reflectance(std::string direction, double value) {
  reflectance[direction] = value;
}


void update_proximity_left(const sensor_msgs::Range& msg) {
  update_proximity("left", msg.range);
}

void update_proximity_right(const sensor_msgs::Range& msg) {
  update_proximity("right", msg.range);
}

void update_proximity_front(const sensor_msgs::Range& msg) {
  update_proximity("front", msg.range);
}


void update_reflectance_left(const std_msgs::Float64& msg) {
  update_reflectance("left", msg.data);
}

void update_reflectance_right(const std_msgs::Float64& msg) {
  update_reflectance("right", msg.data);
}

void update_reflectance_center(const std_msgs::Float64& msg) {
  update_reflectance("center", msg.data);
}


void control(const ros::TimerEvent&) {
  ROS_INFO("(%lf, %lf, %lf) (%lf, %lf, %lf)",
           reflectance["left"], reflectance["center"], reflectance["right"],
           proximity["left"], proximity["front"], proximity["right"]);
}


int main(int argc, char **argv) {
  ros::init(argc, argv, "robocake_random");
  ros::NodeHandle node;

  auto proximity_left_subscriber = node.subscribe("proximity_left", 1,
                                                  update_proximity_left);
  auto proximity_right_subscriber = node.subscribe("proximity_right", 1,
                                                   update_proximity_right);
  auto proximity_front_subscriber = node.subscribe("proximity_front", 1,
                                                   update_proximity_front);

  auto reflectance_left_subscriber = node.subscribe("reflectance_left", 1,
                                                    update_reflectance_left);
  auto reflectance_right_subscriber = node.subscribe("reflectance_right", 1,
                                                     update_reflectance_right);
  auto reflectance_center_subscriber = node.subscribe("reflectance_center", 1,
                                                     update_reflectance_center);

  auto timer = node.createTimer(ros::Duration{1}, control);

  ros::spin();
}
