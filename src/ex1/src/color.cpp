#include "ros/ros.h"
#include "std_msgs/ColorRGBA.h"
#include <iostream>
#include <tuple>
#include <utility>

std::istream &operator>>(std::istream &is, std_msgs::ColorRGBA &c) {
  is >> c.r >> c.g >> c.b >> c.a;
  return is;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "color");
  ros::NodeHandle n;

  auto color_publisher = n.advertise<std_msgs::ColorRGBA>("color", 1000);

  auto msg = []() {
    std_msgs::ColorRGBA m;
    std::tie(m.r, m.g, m.b, m.a) = std::make_tuple(1, 2, 3, 4);
    return m;
  }();

  while (ros::ok()) {
    const auto msg_opt = [] {
      std_msgs::ColorRGBA m;
      std::cout << "Insert a color 'r g b a':" << '\n';
      if (std::cin >> m) {
        return std::make_pair(true, m);
      } else {
        return std::make_pair(false, m);
      }
    }();

    if (!msg_opt.first) {
      return 0;
    }

    const auto msg = msg_opt.second;

    ROS_INFO("publish: (%f %f %f %f)", msg.r, msg.g, msg.b, msg.a);
    color_publisher.publish(msg);
    ros::spinOnce();
  }
  return 0;
}
