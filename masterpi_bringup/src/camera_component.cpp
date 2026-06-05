#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp> // <- NUEVO
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>
#include <opencv2/opencv.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <memory>
#include <string>

namespace masterpi_bringup
{

class CameraComponent : public rclcpp::Node // <- Cambiado el nombre para que coincida con el Launch
{
public:
    // CORRECCIÓN: El constructor ahora acepta NodeOptions indispensable para componentes
    explicit CameraComponent(const rclcpp::NodeOptions & options) 
    : Node("camera_node_cpp", options)
    {
        // 1. Declarar y obtener parámetros de ROS 2
        this->declare_parameter<std::string>("frame_id", "camera_link");
        this->declare_parameter<double>("publish_rate", 15.0);
        this->declare_parameter<bool>("publish_color", false);  
        this->declare_parameter<bool>("publish_gray", true);
        this->declare_parameter<std::string>("image_topic", "/camera/image_raw");
        this->declare_parameter<std::string>("gray_topic", "/camera/image_gray");
        
        this->get_parameter("frame_id", frame_id_);
        this->get_parameter("publish_rate", publish_rate_);
        this->get_parameter("publish_color", publish_color_);

        // 2. Cargar la calibración geométrica
        if (!cargar_calibracion()) {
            RCLCPP_ERROR(this->get_logger(), "Fallo crítico: No se pudo configurar la rectificación de la lente.");
            return;
        }

        // 3. Inicializar la cámara física mediante OpenCV nativo
        cap_.open(-1, cv::CAP_V4L2);
        if (!cap_.isOpened()) {
            RCLCPP_ERROR(this->get_logger(), "No se pudo abrir la cámara de la Raspberry Pi (-1).");
            return;
        }

        cap_.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('Y', 'U', 'Y', 'V'));
        cap_.set(cv::CAP_PROP_FRAME_WIDTH, 640);
        cap_.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
        cap_.set(cv::CAP_PROP_FPS, 30);

        // 4. Configurar el publicador de imágenes
        // Pasamos una referencia al nodo base usando rclcpp::Node::shared_ptr de forma segura
        image_transport_ = std::make_unique<image_transport::ImageTransport>(
            rclcpp::Node::SharedPtr(this, [](rclcpp::Node*){} )
        );
        image_pub_ = image_transport_->advertise("/camera/image_raw", 10);

        // 5. Crear el temporizador
        auto interval = std::chrono::duration<double>(1.0 / publish_rate_);
        timer_ = this->create_wall_timer(interval, std::bind(&CameraComponent::publish_image, this));

        RCLCPP_INFO(this->get_logger(), "Nodo de Cámara en Componente iniciado a %.1f FPS.", publish_rate_);
    }

    ~CameraComponent() override
    {
        if (cap_.isOpened()) {
            cap_.release();
        }
        RCLCPP_INFO(this->get_logger(), "Cámara liberada.");
    }

private:
    bool cargar_calibracion()
    {
        try {
            std::string pkg_path = ament_index_cpp::get_package_share_directory("masterpi_bringup");
            std::string yaml_path = pkg_path + "/config/calibration/calibration_param.yaml";

            cv::FileStorage fs(yaml_path, cv::FileStorage::READ);
            if (!fs.isOpened()) {
                return false;
            }

            cv::Mat mtx, dist;
            fs["camera_matrix"] >> mtx;
            fs["distortion_coefficients"] >> dist;
            fs.release();

            // =====================================================================
            // ¡EL TRUCO MATEMÁTICO! 
            // Escalar los parámetros intrínsecos de píxeles al 50% (de 640 a 320)
            // =====================================================================
            mtx.at<double>(0, 0) *= 0.5; // fx (Focal x)
            mtx.at<double>(1, 1) *= 0.5; // fy (Focal y)
            mtx.at<double>(0, 2) *= 0.5; // cx (Centro óptico x)
            mtx.at<double>(1, 2) *= 0.5; // cy (Centro óptico y)

            // Ahora sí, creamos el mapa optimizado para 320x240 de forma perfecta
            cv::Size image_size(320, 240);
            cv::Mat new_camera_mtx = cv::getOptimalNewCameraMatrix(mtx, dist, image_size, 0, image_size);
            cv::initUndistortRectifyMap(mtx, dist, cv::Mat(), new_camera_mtx, image_size, CV_32FC1, mapx_, mapy_);
            
            return true;
        }
        catch (const std::exception &e) {
            RCLCPP_ERROR(this->get_logger(), "Error leyendo el archivo YAML: %s", e.what());
            return false;
        }
    }

    void publish_image()
    {
        cv::Mat frame_raw, frame_gray, frame_rectificado;
        cap_ >> frame_raw;
        if (frame_raw.empty()) {
            return;
        }

        // 1. Redimensionar primero (Color) para procesar menos píxeles desde el inicio
        cv::resize(frame_raw, frame_raw, cv::Size(320, 240), 0, 0, cv::INTER_NEAREST);

        std::string image_format;

        if (!publish_color_) {
            // 2. CONVERTIR A GRISES AQUÍ (Cuando la imagen es de 320x240 y antes del remap)
            cv::cvtColor(frame_raw, frame_gray, cv::COLOR_BGR2GRAY);
            
            // 3. El remap ahora vuela porque solo procesa UN canal monocromático
            cv::remap(frame_gray, frame_rectificado, mapx_, mapy_, cv::INTER_LINEAR);
            image_format = "mono8";
        } else {
            // Si por alguna razón activas el color, hace el remap normal de 3 canales
            cv::remap(frame_raw, frame_rectificado, mapx_, mapy_, cv::INTER_LINEAR);
            image_format = "bgr8";
        }

        std_msgs::msg::Header header;
        header.stamp = this->get_clock()->now();
        header.frame_id = frame_id_;

        auto msg = cv_bridge::CvImage(header, image_format, frame_rectificado).toImageMsg();
        image_pub_.publish(*msg);
    }

    cv::VideoCapture cap_;
    cv::Mat mapx_, mapy_;
    std::string frame_id_;
    bool publish_color_;
    double publish_rate_;
    std::unique_ptr<image_transport::ImageTransport> image_transport_;
    image_transport::Publisher image_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

} // namespace masterpi_bringup

// Registramos el componente de la cámara de manera oficial
RCLCPP_COMPONENTS_REGISTER_NODE(masterpi_bringup::CameraComponent)