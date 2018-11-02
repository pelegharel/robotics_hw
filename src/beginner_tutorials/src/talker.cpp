#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>

int main(int argc, char **argv) {
  using std_msgs::String;
  using std::stringstream;

  ros::init(argc, argv, "talker");
  ros::NodeHandle n;
  auto chatter_pub = n.advertise<String>("chatter", 1000);
  ros::Rate loop_rate(10);

  int count = 0;
  while (ros::ok()) {
    const String msg = [count] {
      stringstream ss;
      ss << "hello world " << count;
      String msg;
      msg.data = ss.str();
      return msg;
    }();

    ROS_INFO("%s", msg.data.c_str());

    chatter_pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();

    ++count;
  }
  return 0;
}
