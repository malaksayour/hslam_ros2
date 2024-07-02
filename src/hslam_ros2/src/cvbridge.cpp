//#include <sensor_msgs/msg/header.hpp>
#include <boost/thread.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/header.hpp>

namespace cv_bridge {

class CvImage
{
public:
  std_msgs::msg::Header header;
  std::string encoding;
  cv::Mat image;
};

typedef boost::shared_ptr<CvImage> CvImagePtr;
typedef boost::shared_ptr<CvImage const> CvImageConstPtr;

}