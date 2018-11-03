#include "boost/optional.hpp"
#include "cv_bridge/cv_bridge.h"
#include "image_transport/image_transport.h"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "ros/ros.h"
#include "sensor_msgs/image_encodings.h"
#include "std_msgs/String.h"
#include <string>
#include <utility>

using namespace std;
using namespace cv;

namespace {

struct Circle {
  Point2d center;
  double radius;
};

pair<Scalar, Scalar> color2range(string name) {
  return make_pair(Scalar(100, 240, 230), Scalar(110, 260, 245));
}

Circle detect(const Mat &image, const Scalar low, const Scalar high) {
  Mat hsv_image;
  cvtColor(image, hsv_image, cv::COLOR_RGB2HSV);

  auto filtered = [hsv_image, low, high] {
    Mat filtered;
    inRange(hsv_image, low, high, filtered);
    return filtered;
  }();

  Mat bgr_image;
  cvtColor(image, bgr_image, cv::COLOR_RGB2BGR);
  imshow("bgr image", hsv_image);
  waitKey(0);
  imshow("filtered", filtered);
  waitKey(0);
}

class Detector {
private:
  boost::optional<std::string> color;
  boost::optional<Mat> image;

public:
  void refresh() {
    if (!color || !image) {
      return;
    }
    ROS_INFO("detecting %s", color->c_str());
    const auto range = color2range(*color);

    detect(*image, range.first, range.second);
  }

  void received_color(const std_msgs::String::ConstPtr &c) {
    color.emplace(c->data);
    refresh();
  }

  void received_image(const sensor_msgs::ImageConstPtr &msg) {
    auto cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8);
    image.emplace(cv_ptr->image);
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
