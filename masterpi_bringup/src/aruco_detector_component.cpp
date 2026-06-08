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
#include <ament_index_cpp/get_package_share_directory.hpp>
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
    this->declare_parameter<double>("marker_size", 0.06);  // Tamaño del marcador en metros (6cm)
    this->declare_parameter<std::string>("calibration_file", "");

    this->get_parameter("image_topic", image_topic_);
    this->get_parameter("marker_id_topic", marker_id_topic_);
    this->get_parameter("marker_size", marker_size_);
    

    std::string calibration_file;
    this->get_parameter("calibration_file", calibration_file);
    int dictionary_id;
    this->get_parameter("dictionary_id", dictionary_id);
    dictionary_ = cv::aruco::getPredefinedDictionary(dictionary_id);

    if (calibration_file.empty()) {
      try {
        std::string pkg_share = ament_index_cpp::get_package_share_directory("masterpi_bringup");
        calibration_file = pkg_share + "/config/calibration/calibration_param.yaml";
      } catch (const std::exception & e) {
        RCLCPP_ERROR(this->get_logger(), "No se encontró el share del paquete: %s", e.what());
      }
    }
    // 2. Cargar los parámetros desde el archivo YAML
    camera_matrix_ = cv::Mat::eye(3, 3, CV_64F); // Valor por defecto por si falla    
    dist_coeffs_ = cv::Mat::zeros(1, 5, CV_64F);

    if (!calibration_file.empty()) {
    try {
      cv::FileStorage fs(calibration_file, cv::FileStorage::READ);
      if (fs.isOpened()) {
        fs["camera_matrix"] >> camera_matrix_; 
        fs.release();
        
        // --- AJUSTE POR CAMBIO DE RESOLUCIÓN (De 640x480 a 320x240) ---
        double scale_factor = 0.5; // Cambia esto si usas otra escala
        
        camera_matrix_.at<double>(0, 0) *= scale_factor; // fx
        camera_matrix_.at<double>(1, 1) *= scale_factor; // fy
        camera_matrix_.at<double>(0, 2) *= scale_factor; // cx
        camera_matrix_.at<double>(1, 2) *= scale_factor; // cy
        // --------------------------------------------------------------

        RCLCPP_INFO(this->get_logger(), 
          "Matriz de cámara cargada y escalada (x%.2f) con éxito desde: %s", 
          scale_factor, calibration_file.c_str());
      } else {
        RCLCPP_ERROR(this->get_logger(), "No se pudo abrir el archivo de calibración: %s", calibration_file.c_str());
      }
    } catch (const std::exception & e) {
      RCLCPP_ERROR(this->get_logger(), "Error al parsear el archivo YAML: %s", e.what());
    }
  }
  
  RCLCPP_INFO(this->get_logger(), "Focal Real fx: %.2f, cx: %.2f", camera_matrix_.at<double>(0,0), camera_matrix_.at<double>(0,2));

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
    // Puntos 3D del marcador en su sistema de coordenadas local (cuadrado de marker_size x marker_size)
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

      // Calculo de Yaw
      yaw = std::atan2(-rotation_matrix.at<double>(0, 2), -rotation_matrix.at<double>(2, 2));
      yaw = yaw * 180.0 / M_PI;

      // Normalizar entre -180° y 180°
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

        // Calcular distancia y orientación para cada marcador
        for (size_t i = 0; i < ids.size(); ++i) {
          double distance, yaw;
          calculate_marker_pose(corners[i], distance, yaw);
          
          if (distance > 0) {
            RCLCPP_INFO(this->get_logger(), 
              "Marcador ID: %d | Distancia: %.3f m | Yaw: %.1f°",
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