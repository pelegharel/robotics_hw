#include "boost/optional.hpp"
#include "ros/ros.h"
#include <boost/math/constants/constants.hpp>
#include <functional>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>
#include <iostream>
#include <sensor_msgs/LaserScan.h>
#include <stdexcept>
#include <string>
#include <tuple>
#include <utility>

using namespace std;

struct Publishers {
  ros::Publisher velocity;
};

class TurtleBotSensors {
private:
  ros::Subscriber laser_sub;
public:
  TurtleBotSensors(ros::NodeHandle &n)
      : laser_sub(n.subscribe("scan", 1000, &TurtleBotSensors::recieved_scan,
                              this)) {}

  boost::optional<sensor_msgs::LaserScan> last_scan;

  void recieved_scan(const sensor_msgs::LaserScan::ConstPtr &msg) {
    last_scan.emplace(*msg);
  }

};

auto fwd_speed_msg(double m_p_sec) {
  geometry_msgs::Twist msg;
  msg.linear.x = m_p_sec;
  return msg;
}

auto clk_speed_msg(double alpha) {
  geometry_msgs::Twist msg;
  msg.angular.z = -alpha;
  return msg;
}

void speed_for_time(ros::Publisher &velocity, const geometry_msgs::Twist &msg,
                    double move_time_sec) {
  velocity.publish(msg);

  ros::spinOnce();
  ros::Duration(move_time_sec).sleep();

  velocity.publish(fwd_speed_msg(0));
  ros::spinOnce();
}

void move_forward(const sensor_msgs::LaserScan &last_scan,
                  ros::Publisher &velocity, double m_to_move) {
  double min_obs_m = last_scan.range_min;
  if (min_obs_m < m_to_move) {
    return;
  }

  speed_for_time(velocity, fwd_speed_msg(m_to_move / 4), 4);
}

void rotate_clockwise(ros::Publisher &velocity, double alpha) {
  speed_for_time(velocity, clk_speed_msg(alpha / 4), 4);
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
  TurtleBotSensors sensors(n);

  Publishers publishers{n.advertise<geometry_msgs::Twist>("cmd_vel", 1000)};

  while (ros::ok()) {
    ros::spinOnce();
    std::cout << input_text() << '\n';

    std::string s;
    std::cin >> s;
    try {
      const int command = stoi(s);

      if (command == 5) {
        return 0;
      }

      [command, &publishers, &sensors] {
        if (!sensors.last_scan) {
          cout << "No connection to laser scan yet\n";
          return;
        }
        switch (command) {
        case 1:
          ros::spinOnce();
          return move_forward(*sensors.last_scan, publishers.velocity, 0.05);
        case 2:
          cout << "Enter alpha:\n";
          const double alpha = [] {
            string s;
            cin >> s;
            return stod(s);
          }();
          return rotate_clockwise(
              publishers.velocity,
              alpha * (boost::math::constants::pi<double>() / 180.0));
      }
      }();

    } catch (const invalid_argument) {
      cerr << "Wrong command inserted";
    }
  };
  return 0;
}
