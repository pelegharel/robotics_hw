#include "boost/optional.hpp"
#include "cv_bridge/cv_bridge.h"
#include "image_transport/image_transport.h"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "ros/ros.h"
#include "sensor_msgs/image_encodings.h"
#include "std_msgs/ColorRGBA.h"
using namespace std;
using namespace cv;

namespace {
class Detector {
private:
  boost::optional<Scalar> color;
  boost::optional<Mat> image;

public:
  void detect() {
    if (!color || !image) {
      return;
    }
    const auto c = *color;
    ROS_INFO("%f %f %f %f", c[0], c[1], c[2], c[3]);
    cv::imshow("img", *image);
    cv::waitKey(0);
  }

  void received_color(const std_msgs::ColorRGBA::ConstPtr &c) {
    color.emplace(c->b, c->g, c->r, c->a);
    detect();
  }

  void received_image(const sensor_msgs::ImageConstPtr &msg) {
    auto cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8);
    image.emplace(cv_ptr->image);
    detect();
  }
};
}

int main(int argc, char **argv) {

  ros::init(argc, argv, "circle");
  ros::NodeHandle n;
  image_transport::ImageTransport it(n);

  Detector detector;
  auto color_sub =
      n.subscribe("color", 1000, &Detector::received_color, &detector);

  auto img_sub =
      it.subscribe("colors_image", 1, &Detector::received_image, &detector);
  ros::spin();
}
