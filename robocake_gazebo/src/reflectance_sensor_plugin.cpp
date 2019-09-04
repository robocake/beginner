#include <gazebo/plugins/CameraPlugin.hh>
#include <gazebo_plugins/gazebo_ros_utils.h>
#include <ros/ros.h>
#include <std_msgs/Float64.h>

namespace gazebo {
  class ReflectanceSensorPlugin : public CameraPlugin {
  public:
    void Load(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf) override {
      this->CameraPlugin::Load(_sensor, _sdf);
      ros::NodeHandle node(GetRobotNamespace(_sensor, _sdf));
      auto topic = _sdf->Get<std::string>("topicName");
      this->publisher = node.advertise<std_msgs::Float64>(topic, 1);
    }
  protected:
    void OnNewFrame(const unsigned char *_image,
                    unsigned int _width, unsigned int _height,
                    unsigned int _depth, const std::string &_format) override {
      std_msgs::Float64 msg;
      auto length = _width * _height * _depth;
      msg.data = std::accumulate(_image, _image + length, 0.0) / length / 255;
      this->publisher.publish(msg);
    }
  private:
    ros::Publisher publisher;
  };
  GZ_REGISTER_SENSOR_PLUGIN(ReflectanceSensorPlugin)
}
