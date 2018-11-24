#include "boost/optional.hpp"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "ros/ros.h"
#include <boost/math/constants/constants.hpp>
#include <functional>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>
#include <image_geometry/pinhole_camera_model.h>
#include <image_transport/image_transport.h>`
#include <iostream>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/LaserScan.h>
#include <stdexcept>
#include <string>
#include <tuple>
#include <utility>

using namespace std;
using namespace cv;

struct Publishers {
  ros::Publisher velocity;
};

class TurtleBotSensors {
private:
  ros::Subscriber laser_sub;
  void recieved_scan(const sensor_msgs::LaserScan::ConstPtr &msg) {
    last_scan.emplace(*msg);
  }

public:
  TurtleBotSensors(ros::NodeHandle &n)
      : laser_sub(n.subscribe("scan", 1000, &TurtleBotSensors::recieved_scan,
                              this)) {}

  boost::optional<sensor_msgs::LaserScan> last_scan;
};

class Camera {
private:
  image_transport::CameraSubscriber camera_sub;
  void recieved_image(const sensor_msgs::ImageConstPtr &image_msg,
                      const sensor_msgs::CameraInfoConstPtr &info_msg) {
    auto cv_ptr =
        cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::RGB8);
    last_image.emplace(cv_ptr->image);
    last_model = image_geometry::PinholeCameraModel();
    last_model->fromCameraInfo(info_msg);

    ROS_INFO("CAMERA update");
  }

public:
  boost::optional<Mat> last_image;
  boost::optional<image_geometry::PinholeCameraModel> last_model;

  Camera(image_transport::ImageTransport &it)
      : camera_sub(it.subscribeCamera("camera/image_raw", 1,
                                      &Camera::recieved_image, this)) {}
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

void dist_color() {}

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
  image_transport::ImageTransport it(n);

  TurtleBotSensors sensors(n);
  Camera camera(it);

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
        case 1: {
          ros::spinOnce();
          return move_forward(*sensors.last_scan, publishers.velocity, 0.05);
        }
        case 2: {
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
        case 3: {
          return dist_color();
        }
      }
      }();

    } catch (const invalid_argument) {
      cerr << "Wrong command inserted";
    }
  };
  return 0;
}
