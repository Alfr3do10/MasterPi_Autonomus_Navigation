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
#include <opencv2/calib3d.hpp>

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
    this->declare_parameter<double>("marker_size", 0.06);  // Tamaño del marcador en metros (5cm)

    this->get_parameter("image_topic", image_topic_);
    this->get_parameter("marker_id_topic", marker_id_topic_);
    this->get_parameter("marker_size", marker_size_);
    
    int dictionary_id;
    this->get_parameter("dictionary_id", dictionary_id);
    dictionary_ = cv::aruco::getPredefinedDictionary(dictionary_id);

    // La cámara ya está calibrada en camera_component.cpp
    // Usamos matriz identidad ya que la imagen viene desortorsionada
    camera_matrix_ = cv::Mat::eye(3, 3, CV_64F);
    dist_coeffs_ = cv::Mat::zeros(1, 5, CV_64F);

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

  void calculate_marker_pose(const std::vector<cv::Point2f> & marker_corners,
                            double & distance, double & yaw)
  {
    // Puntos 3D del marcador en su sistema de coordenadas local
    std::vector<cv::Point3f> object_points = {
      cv::Point3f(-marker_size_ / 2, marker_size_ / 2, 0),
      cv::Point3f(marker_size_ / 2, marker_size_ / 2, 0),
      cv::Point3f(marker_size_ / 2, -marker_size_ / 2, 0),
      cv::Point3f(-marker_size_ / 2, -marker_size_ / 2, 0)
    };

    cv::Mat rvec, tvec;
    bool success = cv::solvePnP(
      object_points,
      marker_corners,
      camera_matrix_,
      dist_coeffs_,
      rvec,
      tvec,
      false,
      cv::SOLVEPNP_IPPE_SQUARE
    );

    if (success) {
      // Distancia: norma del vector de traslación (tvec)
      distance = cv::norm(tvec);

      // Convertir vector de rotación a matriz de rotación
      cv::Mat rotation_matrix;
      cv::Rodrigues(rvec, rotation_matrix);

      // Cálculo directo del verdadero YAW (Rotación sobre el eje Y de la cámara)
      yaw = std::atan2(-rotation_matrix.at<double>(0, 2), -rotation_matrix.at<double>(2, 2));

      // Convertir de radianes a grados
      yaw = yaw * 180.0 / M_PI;

      // Normalizar el ángulo estrictamente entre -180° y 180°
      if (yaw > 180.0) yaw -= 360.0;
      if (yaw < -180.0) yaw += 360.0;

    } else {
      distance = -1.0;
      yaw = 0.0;
    }
  }
  

  void image_callback(const sensor_msgs::msg::Image::ConstSharedPtr msg)
  {
    try {
      // 1. Apuntamos DIRECTAMENTE a la memoria compartida en formato MONO8 (Gris)
      // Al coincidir con lo que publica la cámara, cv_bridge NO COPIA NADA.
      cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::MONO8);
      const cv::Mat & gray = cv_ptr->image; 

      if (gray.empty()) {
        return;
      }

      // 2. Directo a la detección (Ya no necesitas cv::cvtColor)
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

        // Calcular distancia y orientación para cada marcador
        for (size_t i = 0; i < ids.size(); ++i) {
          double distance, yaw;
          calculate_marker_pose(corners[i], distance, yaw);
          
          if (distance > 0) {
            RCLCPP_INFO(this->get_logger(), 
              "Marcador ID: %d | Distancia: %.3f m| Yaw: %.1f°",
              ids[i], distance, yaw);
          }
        }
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
  cv::Mat camera_matrix_;
  cv::Mat dist_coeffs_;
  double marker_size_;
};

}  // namespace masterpi_bringup

RCLCPP_COMPONENTS_REGISTER_NODE(masterpi_bringup::ArucoDetectorComponent)