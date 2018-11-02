#include "beginner_tutorials/AddTwoInts.h"
#include "ros/ros.h"
#include <string>

int main(int argc, char **argv) {
  using beginner_tutorials::AddTwoInts;

  ros::init(argc, argv, "add_two_ints_client");

  if (argc != 3) {
    ROS_INFO("usege add_two_ints_cline X Y");
    return 1;
  }

  ros::NodeHandle n;
  auto client = n.serviceClient<AddTwoInts>("add_two_ints");
  AddTwoInts srv;

  srv.request.a = std::stol(argv[1]);
  srv.request.b = std::stol(argv[2]);

  if (client.call(srv)) {
    ROS_INFO("Sum: %ld", (long int)srv.response.sum);
  } else {
    ROS_ERROR("Failed to call service add_two_ints");
    return 1;
  }
}
