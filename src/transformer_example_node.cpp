#include "promobot_tf_library/coordinate_transformer.hpp" // Подключаем нашу библиотеку
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <chrono> // Для std::chrono::seconds

using namespace std::chrono_literals; // Для удобного задания времени (e.g., 500ms)

// Функция для печати статуса трансформации
std::string statusToString(promobot_tf_library::TransformStatus status) {
    using promobot_tf_library::TransformStatus;
    switch (status) {
        case TransformStatus::SUCCESS:         return "SUCCESS";
        case TransformStatus::TRANSFORM_FAILED:return "TRANSFORM_FAILED";
        case TransformStatus::FRAME_NOT_FOUND: return "FRAME_NOT_FOUND";
        case TransformStatus::OUT_OF_BOUNDS:   return "OUT_OF_BOUNDS";
        default:                               return "UNKNOWN_STATUS";
    }
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    // Создаем узел библиотеки. Он сам себе rclcpp::Node.
    auto transformer_node = std::make_shared<promobot_tf_library::CoordinateTransformer>();

    RCLCPP_INFO(transformer_node->get_logger(), "*** Coordinate Transformer Example Node Started ***");

    // --- Демонстрация использования библиотеки ---

    // 1. Загрузка трансформаций из YAML
    // Укажи путь к файлу YAML. Если запускаешь `ros2 run`, он должен быть
    // либо в папке install/<pkg_name>/share/<pkg_name>, либо указан абсолютный путь,
    // либо путь относительно директории, из которой запущен `ros2 run`.
    // Проще всего положить его рядом и запускать узел из папки пакета (после сборки).
    // Или передать путь как параметр ROS.
    std::string yaml_file_name = "transforms.yaml"; // Имя файла
    // Попробуем найти файл в директории установки пакета
    std::string package_share_directory = ament_index_cpp::get_package_share_directory("promobot_tf_library");
    std::string yaml_path = package_share_directory + "/" + yaml_file_name;

    RCLCPP_INFO(transformer_node->get_logger(), "Attempting to load transforms from: %s", yaml_path.c_str());
    if (!transformer_node->loadTransformsFromYaml(yaml_path)) {
        RCLCPP_WARN(transformer_node->get_logger(), "Could not load transforms from %s. Check file existence and format. Continuing...", yaml_path.c_str());
    }

    // 2. Добавление трансформации вручную
    // Добавим связь base_link -> camera_link (камера смещена вперед и вверх, повернута вниз)
    RCLCPP_INFO(transformer_node->get_logger(), "Adding manual transform: base_link -> camera_link");
    transformer_node->addManualTransform(
        "base_link",         // Родительский фрейм
        "camera_link",       // Дочерний фрейм
        0.1, 0.0, 0.2,       // Смещение (tx, ty, tz)
        0.0, M_PI / 6.0, 0.0 // Вращение RPY (roll=0, pitch=30_deg, yaw=0) в радианах!
    );

    // Важно: Дадим время tf2 и static_transform_broadcaster распространить статические трансформации
    // В реальных системах это обычно происходит до основного цикла работы.
    RCLCPP_INFO(transformer_node->get_logger(), "Waiting for transforms to propagate...");
    rclcpp::sleep_for(1s); // Подождем 1 секунду

    // 3. Демонстрация установки границ через параметры ROS2
    // Параметры для фреймов из YAML и добавленных вручную УЖЕ ОБЪЯВЛЕНЫ библиотекой.
    // Их можно (и нужно) устанавливать ИЗВНЕ, например, через командную строку или launch-файл.
    RCLCPP_INFO(transformer_node->get_logger(), "-----------------------------------------------------");
    RCLCPP_INFO(transformer_node->get_logger(), "Boundary parameters are declared for known frames.");
    RCLCPP_INFO(transformer_node->get_logger(), "Use ANOTHER TERMINAL to set parameters, e.g.:");
    RCLCPP_INFO(transformer_node->get_logger(), "ros2 param set /coordinate_transformer boundaries.map.min_x -10.0");
    RCLCPP_INFO(transformer_node->get_logger(), "ros2 param set /coordinate_transformer boundaries.map.max_x 10.0");
    RCLCPP_INFO(transformer_node->get_logger(), "ros2 param set /coordinate_transformer boundaries.base_link.max_z 0.5");
    RCLCPP_INFO(transformer_node->get_logger(), "...");
    RCLCPP_INFO(transformer_node->get_logger(), "Set parameters now and observe logs in this terminal.");
    RCLCPP_INFO(transformer_node->get_logger(), "-----------------------------------------------------");
    // Здесь можно поставить паузу, чтобы пользователь успел установить параметры
    rclcpp::sleep_for(10s); // Ждем 10 секунд для установки параметров


    // 4. Пример преобразования точки
    geometry_msgs::msg::PointStamped point_in_camera;
    point_in_camera.header.frame_id = "camera_link"; // Точка задана в системе координат камеры
    // Используем tf2::TimePointZero (эквивалентно rclcpp::Time(0)) для получения самой последней доступной трансформации
    point_in_camera.header.stamp = tf2_ros::toMsg(tf2::TimePointZero);
    point_in_camera.point.x = 1.0; // Объект в 1 метре перед камерой
    point_in_camera.point.y = 0.5; // Справа от центра камеры
    point_in_camera.point.z = 0.1; // Чуть ниже оптической оси

    std::string target_frame = "map"; // Хотим узнать координаты этой точки в системе 'map'

    RCLCPP_INFO(transformer_node->get_logger(), "-----------------------------------------------------");
    RCLCPP_INFO(transformer_node->get_logger(), "Attempting to transform point [x: %.2f, y: %.2f, z: %.2f] from '%s' to '%s'",
                point_in_camera.point.x, point_in_camera.point.y, point_in_camera.point.z,
                point_in_camera.header.frame_id.c_str(), target_frame.c_str());

    auto [transformed_point_opt, status] = transformer_node->transformPoint(point_in_camera, target_frame);

    // Обработка результата
    RCLCPP_INFO(transformer_node->get_logger(), "Transformation Status: %s", statusToString(status).c_str());

    if (transformed_point_opt) { // Если точка была возвращена (даже при OUT_OF_BOUNDS)
        const auto& transformed_point = *transformed_point_opt;
        RCLCPP_INFO(transformer_node->get_logger(), "Transformed Point in '%s': [x: %.3f, y: %.3f, z: %.3f]",
                    transformed_point.header.frame_id.c_str(), // Должен быть target_frame
                    transformed_point.point.x,
                    transformed_point.point.y,
                    transformed_point.point.z);
        if(status == promobot_tf_library::TransformStatus::OUT_OF_BOUNDS) {
            RCLCPP_WARN(transformer_node->get_logger(), ">>> Point is OUTSIDE the boundaries defined for frame '%s'!", target_frame.c_str());
        } else if (status == promobot_tf_library::TransformStatus::SUCCESS) {
             RCLCPP_INFO(transformer_node->get_logger(), ">>> Point is INSIDE the boundaries defined for frame '%s'.", target_frame.c_str());
        }
    } else { // Если std::nullopt (при ошибках TRANSFORM_FAILED или FRAME_NOT_FOUND)
        RCLCPP_ERROR(transformer_node->get_logger(), "Transformation failed, no point returned. Check TF tree and logs.");
    }
    RCLCPP_INFO(transformer_node->get_logger(), "-----------------------------------------------------");


    // Запускаем цикл обработки событий узла (включая колбеки параметров)
    // Узел будет работать, пока его не остановят (Ctrl+C)
    RCLCPP_INFO(transformer_node->get_logger(), "Example node finished demonstration. Spinning to handle parameter updates.");
    RCLCPP_INFO(transformer_node->get_logger(), "Press Ctrl+C to exit.");
    rclcpp::spin(transformer_node);

    RCLCPP_INFO(transformer_node->get_logger(), "*** Coordinate Transformer Example Node Shutting Down ***");
    rclcpp::shutdown();
    return 0;
}

