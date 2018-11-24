#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>
#include <iostream>
#include <stdexcept>
#include <string>
#include <tuple>
#include <utility>

using namespace std;

struct Publishers {
  ros::Publisher velocity;
};

void move_forward(ros::Publisher &velocity) {
  cout << "publishing speed!!" << '\n';
  const auto msg = [x = 100] {
    geometry_msgs::Twist msg;
    msg.linear.x = x;
    return msg;
  }
  ();
  velocity.publish(msg);
}

string input_text() {
  return R"(
Plesae enter number of one of the following commands:
    1. Move forward
    2. Turn around
    3. Distance to object with color X
    4. Find object with color X 
    5. quit
)";
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "instruction");
  ros::NodeHandle n;

  Publishers publishers{n.advertise<geometry_msgs::Twist>("cmd_vel", 1000)};

  while (ros::ok()) {
    std::cout << input_text() << '\n';

    std::string s;
    std::cin >> s;

    try {
      const int command = stoi(s);
      cout << "cmd" << command << '\n';
      if (command == 5) {
        return 0;
      }

      [command, &publishers] {
        switch (command) {
        case 1:
          return move_forward(publishers.velocity);
        }
      }();

    } catch (const invalid_argument) {
      cerr << "Wrong command inserted";
    }
  };
  ros::spinOnce();
  return 0;
}
