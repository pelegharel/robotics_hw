#include "boost/optional.hpp"
#include "cv_bridge/cv_bridge.h"
#include "geometry_msgs/Vector3.h"
#include "image_transport/image_transport.h"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "ros/package.h"
#include "ros/ros.h"
#include "sensor_msgs/image_encodings.h"
#include <array>
#include <string>

using namespace cv;
using namespace std;
namespace {
void mark_circle(Mat &image, array<double, 3> circle) {
  cvtColor(image, image, cv::COLOR_RGB2BGR);
  cv::circle(image, Point(get<0>(circle), get<1>(circle)), get<2>(circle),
             Scalar(0, 0, 0), 5);
}

class Writer {
private:
  boost::optional<Mat> image;

public:
  void received_circle(const geometry_msgs::Vector3ConstPtr &msg) {
    if (image) {
      mark_circle(*image, {msg->x, msg->y, msg->z});
      const auto img_path =
          ros::package::getPath("ex1") + "/src/colors_marked.jpg";
      imwrite(img_path, *image);
      ROS_INFO("writen image to %s", img_path.c_str());
    }
  }

  void received_image(const sensor_msgs::ImageConstPtr &msg) {
    auto cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8);
    image.emplace(cv_ptr->image);
  }
};
}
int main(int argc, char **argv) {
  ros::init(argc, argv, "image_writer");
  ros::NodeHandle n;
  image_transport::ImageTransport it(n);

  Writer writer;

  auto circle_sub = n.subscribe("circle", 5, &Writer::received_circle, &writer);

  auto img_sub =
      it.subscribe("colors_image", 1, &Writer::received_image, &writer);
  ros::spin();
  return 0;
}
