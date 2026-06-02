#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>

namespace masterpi_bringup
{

class ArucoDetectorComponent : public rclcpp::Node
{
public:
  explicit ArucoDetectorComponent(const rclcpp::NodeOptions & options)
  : Node("aruco_detector_component", options)
  {
    this->declare_parameter<std::string>("image_topic", "/camera/image_raw");
    this->declare_parameter<std::string>("marker_id_topic", "/aruco/ids");
    this->declare_parameter<int>("dictionary_id", cv::aruco::DICT_4X4_50);

    this->get_parameter("image_topic", image_topic_);
    this->get_parameter("marker_id_topic", marker_id_topic_);
    int dictionary_id;
    this->get_parameter("dictionary_id", dictionary_id);
    dictionary_ = cv::aruco::getPredefinedDictionary(dictionary_id);

    auto qos = rclcpp::SensorDataQoS();
    image_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
      image_topic_,
      qos,
      std::bind(&ArucoDetectorComponent::image_callback, this, std::placeholders::_1)
    );

    marker_publisher_ = this->create_publisher<std_msgs::msg::Int32MultiArray>(marker_id_topic_, 10);

    RCLCPP_INFO(this->get_logger(), "ArucoDetectorComponent listo. Subscrito a: %s", image_topic_.c_str());
  }

private:
  // En la declaración privada y en la implementación:
    void image_callback(const sensor_msgs::msg::Image::ConstSharedPtr msg)
  {
    try {
      std::string encoding = msg->encoding;
      if (encoding != sensor_msgs::image_encodings::BGR8 &&
          encoding != sensor_msgs::image_encodings::RGB8 &&
          encoding != sensor_msgs::image_encodings::MONO8) {
        encoding = sensor_msgs::image_encodings::BGR8;
      }

      cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(msg, encoding);
      const cv::Mat & frame = cv_ptr->image;
      if (frame.empty()) {
        return;
      }

      cv::Mat gray;
      if (frame.channels() == 3) {
        if (encoding == sensor_msgs::image_encodings::RGB8) {
          cv::cvtColor(frame, gray, cv::COLOR_RGB2GRAY);
        } else {
          cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
        }
      } else {
        gray = frame;
      }

      std::vector<int> ids;
      std::vector<std::vector<cv::Point2f>> corners;
      cv::aruco::detectMarkers(gray, dictionary_, corners, ids);

      if (!ids.empty()) {
        std_msgs::msg::Int32MultiArray marker_msg;
        marker_msg.data = ids;
        marker_publisher_->publish(marker_msg);

        std::string ids_string;
        for (size_t i = 0; i < ids.size(); ++i) {
          ids_string += std::to_string(ids[i]);
          if (i + 1 < ids.size()) {
            ids_string += ", ";
          }
        }
        RCLCPP_INFO(this->get_logger(), "Aruco markers detectados: %s", ids_string.c_str());
      }
    } catch (const cv_bridge::Exception & e) {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge error: %s", e.what());
    } catch (const std::exception & e) {
      RCLCPP_ERROR(this->get_logger(), "Aruco detector error: %s", e.what());
    }
  }

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscription_;
  rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr marker_publisher_;
  std::string image_topic_;
  std::string marker_id_topic_;
  cv::Ptr<cv::aruco::Dictionary> dictionary_;
};

}  // namespace masterpi_bringup

RCLCPP_COMPONENTS_REGISTER_NODE(masterpi_bringup::ArucoDetectorComponent)
