#include "cv_bridge/cv_bridge.h"
#include "image_transport/image_transport.h"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "ros/package.h"
#include "ros/ros.h"
#include "sensor_msgs/image_encodings.h"
#include <string>

int main(int argc, char **argv) {
  ros::init(argc, argv, "img");
  ros::NodeHandle n;
  image_transport::ImageTransport it(n);

  const auto img_path = ros::package::getPath("ex1") + "/src/colors.jpg";
  const auto img = cv::imread(img_path);

  const auto img_pub = it.advertise("colors_image", 1);
  cv::waitKey(30);

  const auto msg =
      cv_bridge::CvImage(std_msgs::Header(), "bgr8", img).toImageMsg();

  ros::Rate loop_rate(5);
  while (n.ok()) {
    img_pub.publish(msg);
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
