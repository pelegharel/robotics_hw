#include "ros/ros.h"
#include "std_msgs/String.h"
#include <iostream>
#include <string>
#include <tuple>
#include <utility>

int main(int argc, char **argv) {
  ros::init(argc, argv, "color");
  ros::NodeHandle n;

  auto color_publisher = n.advertise<std_msgs::String>("color", 1000);

  while (ros::ok()) {
    std::string s;

    std::cout << "Insert a color name, q to quit" << '\n';

    std::cin >> s;

    if (s == "q") {
      return 0;
    }

    std_msgs::String msg;
    msg.data = s;
    ROS_INFO("publish: %s", s.c_str());

    color_publisher.publish(msg);
    ros::spinOnce();
  }
  return 0;
}
