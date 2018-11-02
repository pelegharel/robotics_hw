#include "boost/optional.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "ros/ros.h"
#include "std_msgs/ColorRGBA.h"

using namespace std;
using namespace cv;

namespace {
class Detector {
private:
  boost::optional<Scalar> color;

public:
  void detect() {
    if (!color) {
      return;
    }
    const auto c = *color;
    ROS_INFO("%f %f %f %f", c[0], c[1], c[2], c[3]);
  }

  void received_color(const std_msgs::ColorRGBA::ConstPtr &c) {
    color.emplace(c->b, c->g, c->r, c->a);
    detect();
  }
};
}

int main(int argc, char **argv) {

  ros::init(argc, argv, "circle");
  ros::NodeHandle n;

  Detector detector;
  auto color_suc =
      n.subscribe("color", 1000, &Detector::received_color, &detector);

  ros::spin();
}
