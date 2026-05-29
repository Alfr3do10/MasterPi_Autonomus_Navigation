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
        // 1. Declarar y obtener parámetros de ROS 2
        this->declare_parameter<std::string>("frame_id", "camera_link");
        this->declare_parameter<double>("publish_rate", 15.0); // ¡Ahora podemos subir a 30 FPS nativos!
        
        this->get_parameter("frame_id", frame_id_);
        double publish_rate;
        this->get_parameter("publish_rate", publish_rate);

        // 2. Cargar la calibración geométrica desde tu nuevo archivo YAML
        if (!cargar_calibracion()) {
            RCLCPP_ERROR(this->get_logger(), "Fallo crítico: No se pudo configurar la rectificación de la lente.");
            return;
        }

        // 3. Inicializar la cámara física mediante OpenCV nativo
        cap_.open(-1, cv::CAP_V4L2); // Usamos la API nativa de Linux V4L2 para máxima velocidad
        if (!cap_.isOpened()) {
            RCLCPP_ERROR(this->get_logger(), "No se pudo abrir la cámara de la Raspberry Pi (-1).");
            return;
        }

        // Configurar el hardware en formato YUYV a 640x480 como lo hacía el SDK
        cap_.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('Y', 'U', 'Y', 'V'));
        cap_.set(cv::CAP_PROP_FRAME_WIDTH, 640);
        cap_.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
        cap_.set(cv::CAP_PROP_FPS, 30);

        // 4. Configurar el publicador optimizado de imágenes de ROS 2
        // image_transport es mucho más rápido para flujos de video que un publicador estándar
        image_transport_ = std::make_unique<image_transport::ImageTransport>(shared_from_this());
        image_pub_ = image_transport_->advertise("/camera/image_raw", 10);

        // 5. Crear el temporizador para capturar y publicar de manera estable
        auto interval = std::chrono::duration<double>(1.0 / publish_rate);
        timer_ = this->create_wall_timer(interval, std::bind(&CameraNodeCpp::publish_image, this));

        RCLCPP_INFO(this->get_logger(), "Nodo de Cámara en C++ iniciado a %.1f FPS. ¡Rectificación activa!", publish_rate);
    }

    ~CameraNodeCpp() override
    {
        if (cap_.isOpened()) {
            cap_.release();
        }
        RCLCPP_INFO(this->get_logger(), "Cámara liberada y nodo C++ cerrado.");
    }

private:
    bool cargar_calibracion()
    {
        try {
            // Buscamos dinámicamente la ruta del paquete híbrido
            std::string pkg_path = ament_index_cpp::get_package_share_directory("masterpi_bringup");
            std::string yaml_path = pkg_path + "/config/calibration/calibration_param.yaml";

            RCLCPP_INFO(this->get_logger(), "Cargando calibración desde: %s", yaml_path.c_str());

            cv::FileStorage fs(yaml_path, cv::FileStorage::READ);
            if (!fs.isOpened()) {
                return false;
            }

            cv::Mat mtx, dist;
            fs["camera_matrix"] >> mtx;
            fs["distortion_coefficients"] >> dist;
            fs.release();

            cv::Size image_size(320, 240);

            // Pre-calculamos los mapas matemáticos de pixeles UNA sola vez para ahorrar CPU
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
        cv::Mat frame_raw, frame_rectificado;

        // Captura directa desde el bus de hardware
        cap_ >> frame_raw;
        if (frame_raw.empty()) {
            return;
        }

        // Forzar tamaño por si el driver entrega otra resolución
        cv::resize(frame_raw, frame_raw, cv::Size(320, 240), 0, 0, cv::INTER_NEAREST);

        // ¡Aquí ocurre el milagro! El remap en C++ corre directo sobre la RAM optimizada
        cv::remap(frame_raw, frame_rectificado, mapx_, mapy_, cv::INTER_LINEAR);

        // CÓDIGO NUEVO CORREGIDO
        std_msgs::msg::Header header;
        header.stamp = this->get_clock()->now();
        header.frame_id = frame_id_;

        // Pasamos el objeto header directamente (por valor/referencia)
        auto msg = cv_bridge::CvImage(
            header,
            "bgr8",
            frame_rectificado
        ).toImageMsg();

        // Publicar la imagen a la red de ROS 2
        image_pub_.publish(*msg);

        // Estampar el tiempo y el ID del frame transformado
        msg->header.stamp = this->get_clock()->now();
        msg->header.frame_id = frame_id_;

        // Publicar la imagen a la red de ROS 2
        image_pub_.publish(*msg);
    }

    // Variables miembros del nodo
    cv::VideoCapture cap_;
    cv::Mat mapx_, mapy_;
    std::string frame_id_;
    
    std::unique_ptr<image_transport::ImageTransport> image_transport_;
    image_transport::Publisher image_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    // Usamos un nodo capaz de compartir punteros para optimizar el uso de memoria
    rclcpp::spin(std::make_shared<CameraNodeCpp>());
    rclcpp::shutdown();
    return 0;
}