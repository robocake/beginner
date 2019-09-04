#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Range.h>
#include <std_msgs/Float64.h>

#include <cmath>
#include <unordered_map>

std::unordered_map<std::string, double> proximity, reflectance;
ros::Publisher publisher;

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

int direction{0};

void control(const ros::TimerEvent&) {
  std::uniform_int_distribution<> d{0, 1};
  std::random_device r;
  std::default_random_engine g(r());
  geometry_msgs::Twist msg;
  if (proximity["front"] > 0.1 && proximity["left"] > 0.1 &&
      proximity["right"] > 0.1 &&
      reflectance["left"] > 0.2 && reflectance["right"] > 0.2) {
    direction = 0;
    msg.linear.x = 0.2;
    msg.angular.z = 0;
  } else {
    if (!direction) {
      if (proximity["right"] < 0.1 || reflectance["right"] < 0.2) {
        direction = 1;
      } else if (proximity["left"] < 0.1 || reflectance["left"] < 0.2) {
        direction = -1;
      } else {
        direction = d(g) * 2 - 1;
      }
    }
    msg.linear.x = 0;
    msg.angular.z = direction * M_PI / 2;
  }
  publisher.publish(msg);
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

  publisher = node.advertise<geometry_msgs::Twist>(
      "diff_drive_controller/cmd_vel", 1);

  auto timer = node.createTimer(ros::Duration{0.5}, control);

  ros::spin();
}
