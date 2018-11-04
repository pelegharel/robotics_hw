#include "boost/optional.hpp"
#include "cv_bridge/cv_bridge.h"
#include "geometry_msgs/Vector3.h"
#include "image_transport/image_transport.h"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "ros/ros.h"
#include "sensor_msgs/image_encodings.h"
#include "std_msgs/String.h"
#include <algorithm>
#include <array>
#include <string>
#include <tuple>
#include <utility>
using namespace std;
using namespace cv;
namespace {

struct Circle {
  Point2d center;
  int radius;
};

boost::optional<pair<Scalar, Scalar>> color2range(string name) {

  array<tuple<string, Scalar, Scalar>, 5> mapping = {
      {make_tuple("blue", Scalar(100, 240, 230), Scalar(110, 260, 245)),
       make_tuple("red", Scalar(0, 190, 225), Scalar(5, 210, 250)),
       make_tuple("yellow", Scalar(25, 210, 240), Scalar(35, 230, 260)),
       make_tuple("green", Scalar(70, 130, 95), Scalar(80, 150, 110)),
       make_tuple("purple", Scalar(155, 130, 130), Scalar(165, 160, 140))}};

  const auto it = find_if(begin(mapping), end(mapping),
                          [name](auto x) { return get<0>(x) == name; });
  if (it == end(mapping)) {
    return boost::optional<pair<Scalar, Scalar>>();
  } else {
    return make_pair(get<1>(*it), get<2>(*it));
  }
}

Circle detect(const Mat &image, const Scalar low, const Scalar high) {
  Mat hsv_image;
  cvtColor(image, hsv_image, cv::COLOR_RGB2HSV);

  auto filtered = [hsv_image, low, high] {
    Mat filtered;
    inRange(hsv_image, low, high, filtered);
    return filtered;
  }();

  const Mat circle_points = [filtered] {
    Mat points;
    findNonZero(filtered, points);
    return points;
  }();

  const Circle res = [circle_points] {
    Point2f center;
    float radius;

    minEnclosingCircle(circle_points, center, radius);

    return Circle{center, (int)radius};
  }();

  return res;
}

class Detector {
private:
  boost::optional<std::string> color;
  boost::optional<Mat> image;

public:
  ros::Publisher pub;
  Detector(const ros::Publisher &pub) : pub(pub) {}
  void refresh() {
    if (!color || !image) {
      return;
    }
    ROS_INFO("detecting %s", color->c_str());
    const auto range = color2range(*color);
    if (range) {
      const auto circ = detect(*image, range->first, range->second);

      ROS_INFO("detected circle x:%f y:%f r:%d", circ.center.x, circ.center.y,
               circ.radius);

      geometry_msgs::Vector3 msg;

      msg.x = circ.center.x;
      msg.y = circ.center.y;
      msg.z = circ.radius;

      pub.publish(msg);
    }
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

  Detector detector(n.advertise<geometry_msgs::Vector3>("circle", 5));
  auto color_sub =
      n.subscribe("color", 1000, &Detector::received_color, &detector);

  auto img_sub =
      it.subscribe("colors_image", 1, &Detector::received_image, &detector);
  ros::spin();
}
