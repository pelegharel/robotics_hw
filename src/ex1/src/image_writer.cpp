#include "boost/optional.hpp"
#include "cv_bridge/cv_bridge.h"
#include "geometry_msgs/Vector3.h"
#include "image_transport/image_transport.h"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "ros/ros.h"
#include "sensor_msgs/image_encodings.h"

using namespace cv;
using namespace std;
class Writer {
private:
  boost::optional<Mat> image;

public:
  void received_circle(const geometry_msgs::Vector3ConstPtr &msg) {
    if (image) {
      cvtColor(*image, *image, cv::COLOR_RGB2BGR);
      circle(*image, Point(msg->x, msg->y), msg->z, Scalar(0,0,0),5);
      imshow("circ", *image);
      waitKey(0);  
      ROS_INFO("got circle");
    }
  }

  void received_image(const sensor_msgs::ImageConstPtr &msg) {
    auto cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8);
    image.emplace(cv_ptr->image);
  }
};
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
