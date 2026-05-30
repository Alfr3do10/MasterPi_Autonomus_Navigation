#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>
#include <opencv2/opencv.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <memory>
#include <string>

class CameraNodeCpp : public rclcpp::Node
{
public:
    CameraNodeCpp() : Node("camera_node_cpp")
    {
        // Configuración de los nuevos parámetros solicitados
        this->declare_parameter<std::string>("frame_id", "camera_link");
        this->declare_parameter<double>("publish_rate", 15.0); // Modificado a 15 Hz
        
        this->get_parameter("frame_id", frame_id_);
        this->get_parameter("publish_rate", publish_rate_);

        // Ancho y alto a la mitad de la resolución estándar (640x480 -> 320x240)
        width_ = 640;
        height_ = 480;

        // Cargar y ajustar la calibración dinámicamente para 320x240
        if (!cargar_y_escalar_calibracion()) {
            RCLCPP_ERROR(this->get_logger(), "Fallo crítico: No se pudo configurar la rectificación.");
            return;
        }

        // Inicializar hardware mediante V4L2 nativo
        cap_.open(-1, cv::CAP_V4L2); 
        if (!cap_.isOpened()) {
            RCLCPP_ERROR(this->get_logger(), "No se pudo abrir la cámara de la Raspberry Pi.");
            return;
        }

        // LE PEDIMOS AL HARDWARE LA MITAD DE RESOLUCIÓN Y 15 FPS
        cap_.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('Y', 'U', 'Y', 'V'));
        cap_.set(cv::CAP_PROP_FRAME_WIDTH, width_);
        cap_.set(cv::CAP_PROP_FRAME_HEIGHT, height_);
        cap_.set(cv::CAP_PROP_FPS, publish_rate_);
    }

    void init_transport()
    {
        auto shared_node = std::shared_ptr<rclcpp::Node>(this, [](rclcpp::Node*){});
        image_transport_ = std::make_unique<image_transport::ImageTransport>(shared_node);
        image_pub_ = image_transport_->advertise("/camera/image_raw", 10);

        auto interval = std::chrono::duration<double>(1.0 / publish_rate_);
        timer_ = this->create_wall_timer(interval, std::bind(&CameraNodeCpp::publish_image, this));

        RCLCPP_INFO(this->get_logger(), "Nodo Optimizado iniciado a %d x %d @ %.1f Hz.", width_, height_, publish_rate_);
    }

    ~CameraNodeCpp() override
    {
        if (cap_.isOpened()) cap_.release();
    }

private:
    bool cargar_y_escalar_calibracion()
    {
        try {
            std::string pkg_path = ament_index_cpp::get_package_share_directory("masterpi_bringup");
            std::string yaml_path = pkg_path + "/config/calibration/calibration_param.yaml";

            cv::FileStorage fs(yaml_path, cv::FileStorage::READ);
            if (!fs.isOpened()) return false;

            cv::Mat mtx, dist;
            fs["camera_matrix"] >> mtx;
            fs["distortion_coefficients"] >> dist;
            fs.release();

            // AJUSTE MATEMÁTICO: Escalamos la matriz intrínseca a la mitad
            // fx = mtx.at<double>(0,0), fy = mtx.at<double>(1,1)
            // cx = mtx.at<double>(0,2), cy = mtx.at<double>(1,2)
            // mtx.at<double>(0, 0) /= 2.0; // fx
            // mtx.at<double>(1, 1) /= 2.0; // fy
            // mtx.at<double>(0, 2) /= 2.0; // cx
            // mtx.at<double>(1, 2) /= 2.0; // cy

            cv::Size new_image_size(width_, height_);

            cv::Mat new_camera_mtx = cv::getOptimalNewCameraMatrix(
                mtx, dist, new_image_size, 0, new_image_size
            );

            cv::initUndistortRectifyMap(
                mtx, dist, cv::Mat(), new_camera_mtx,
                new_image_size, CV_32FC1, mapx_, mapy_
            );
            
            return true;
        }
        catch (const std::exception &e) {
            RCLCPP_ERROR(this->get_logger(), "Error adaptando calibración: %s", e.what());
            return false;
        }
    }

    void publish_image()
    {
        cv::Mat frame_raw, frame_rectificado;

        cap_ >> frame_raw;
        if (frame_raw.empty()) return;

        // ¡YA NO NECESITAMOS CV::RESIZE! Ahorramos muchísima CPU.
        // El frame viene directo de la cámara a 320x240.

        // Remap directo ultrarrápido
        cv::remap(frame_raw, frame_rectificado, mapx_, mapy_, cv::INTER_LINEAR);

        std_msgs::msg::Header header;
        header.stamp = this->get_clock()->now();
        header.frame_id = frame_id_;

        auto msg = cv_bridge::CvImage(header, "bgr8", frame_rectificado).toImageMsg();
        image_pub_.publish(*msg);
    }

    cv::VideoCapture cap_;
    cv::Mat mapx_, mapy_;
    std::string frame_id_;
    double publish_rate_;
    int width_;
    int height_;
    
    std::unique_ptr<image_transport::ImageTransport> image_transport_;
    image_transport::Publisher image_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CameraNodeCpp>();
    node->init_transport();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
