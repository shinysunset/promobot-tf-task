#ifndef PROMOBOT_TF_LIBRARY_COORDINATE_TRANSFORMER_HPP_
#define PROMOBOT_TF_LIBRARY_COORDINATE_TRANSFORMER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/parameter_event_handler.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <yaml-cpp/yaml.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp> // Для удобных преобразований tf2::Quaternion <-> geometry_msgs::msg::Quaternion

#include <string>
#include <vector>
#include <map>
#include <optional>
#include <fstream>
#include <memory>
#include <limits>
#include <set>
#include <shared_mutex> // Для защиты границ при чтении/записи из разных потоков

namespace promobot_tf_library
{

/**
 * @brief Структура для хранения прямоугольных границ системы координат.
 */
struct BoundingBox {
    double min_x = -std::numeric_limits<double>::infinity();
    double max_x = std::numeric_limits<double>::infinity();
    double min_y = -std::numeric_limits<double>::infinity();
    double max_y = std::numeric_limits<double>::infinity();
    double min_z = -std::numeric_limits<double>::infinity();
    double max_z = std::numeric_limits<double>::infinity();
};

/**
 * @brief Статусы выполнения операции трансформации.
 */
enum class TransformStatus {
    SUCCESS,          ///< Трансформация успешна, точка в границах.
    TRANSFORM_FAILED, ///< Общая ошибка tf2 при попытке трансформации.
    FRAME_NOT_FOUND,  ///< Не найдена исходная или целевая система координат в дереве tf.
    OUT_OF_BOUNDS     ///< Трансформация успешна, но точка вышла за пределы границ целевой СК.
};

/**
 * @brief Основной класс библиотеки для работы с трансформациями координат.
 *
 * Предоставляет методы для загрузки статических трансформаций,
 * преобразования точек между системами координат и проверки границ.
 * Использует tf2 для получения трансформаций и параметры ROS2 для
 * динамического задания границ.
 */
class CoordinateTransformer : public rclcpp::Node
{
public:
    /**
     * @brief Конструктор.
     * @param options Опции для инициализации rclcpp::Node.
     */
    explicit CoordinateTransformer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

    /**
     * @brief Загружает статические трансформации из указанного YAML файла.
     *
     * Ожидаемый формат YAML:
     * static_transforms:
     *   - parent_frame: "frame1"
     *     child_frame: "frame2"
     *     translation: {x: 1.0, y: 0.0, z: 0.0}
     *     # Либо rotation_rpy (углы в радианах), либо rotation_quat
     *     rotation_rpy: {roll: 0.0, pitch: 0.0, yaw: 1.57}
     *     # rotation_quat: {x: 0.0, y: 0.0, z: 0.707, w: 0.707}
     *
     * Объявляет параметры ROS2 для границ (`boundaries.<frame_id>.*`) для всех
     * упомянутых в файле систем координат.
     *
     * @param yaml_path Путь к YAML файлу с трансформациями.
     * @return true если загрузка и публикация прошли успешно, иначе false.
     */
    bool loadTransformsFromYaml(const std::string& yaml_path);

    /**
     * @brief Добавляет статическую трансформацию вручную.
     *
     * Объявляет параметры ROS2 для границ (`boundaries.<frame_id>.*`) для
     * родительской и дочерней систем координат, если они еще не объявлены.
     *
     * @param parent_frame Имя родительской системы координат.
     * @param child_frame Имя дочерней системы координат.
     * @param tx Смещение по оси X.
     * @param ty Смещение по оси Y.
     * @param tz Смещение по оси Z.
     * @param roll Угол рыскания (Roll) в радианах.
     * @param pitch Угол тангажа (Pitch) в радианах.
     * @param yaw Угол поворота (Yaw) в радианах.
     * @return true если трансформация успешно добавлена и опубликована, иначе false.
     */
    bool addManualTransform(
        const std::string& parent_frame,
        const std::string& child_frame,
        double tx, double ty, double tz,
        double roll, double pitch, double yaw
    );

    /**
     * @brief Преобразует точку из одной системы координат в другую.
     *
     * Использует tf2 для получения необходимой трансформации. После успешной
     * трансформации проверяет, попадает ли результат в границы, заданные
     * для целевой системы координат через параметры ROS2.
     *
     * @param input_point Точка с указанием ее текущей системы координат (header.frame_id).
     *                      header.stamp определяет время, на которое запрашивается трансформация
     *                      (используйте rclcpp::Time(0) или tf2::TimePointZero для последней доступной).
     * @param target_frame Имя целевой системы координат.
     * @return Пара: опционально преобразованная точка (std::nullopt при ошибках TRANSFORM_FAILED/FRAME_NOT_FOUND)
     *         и статус операции (TransformStatus). При OUT_OF_BOUNDS точка возвращается.
     */
    std::pair<std::optional<geometry_msgs::msg::PointStamped>, TransformStatus> transformPoint(
        const geometry_msgs::msg::PointStamped& input_point,
        const std::string& target_frame);

private:
    /**
     * @brief Проверяет, находится ли точка в границах указанной системы координат.
     *
     * Использует текущие значения границ, хранящиеся в `boundaries_`.
     * Потокобезопасен для чтения (использует shared_lock).
     *
     * @param point Точка для проверки (ожидается в координатах frame_id).
     * @param frame_id Система координат, для которой проверяются границы.
     * @return true если точка внутри границ или границы не заданы, иначе false.
     */
    bool checkBounds(const geometry_msgs::msg::Point& point, const std::string& frame_id);

    /**
     * @brief Объявляет параметры ROS2 для границ указанной системы координат, если они еще не были объявлены.
     *
     * Создает параметры: boundaries.<frame_id>.min_x, ...max_z с дефолтными значениями +/- infinity.
     * Считывает начальные значения параметров в `boundaries_`.
     * Потокобезопасен для записи (использует unique_lock).
     *
     * @param frame_id Имя системы координат.
     */
    void declareBoundaryParameters(const std::string& frame_id);

    /**
     * @brief Callback-функция для обработки изменений параметров ROS2.
     *
     * Реагирует на изменения параметров с префиксом "boundaries.", обновляя
     * соответствующие значения в `boundaries_`.
     * Потокобезопасен для записи (использует unique_lock).
     *
     * @param parameters Список измененных параметров.
     * @return Результат установки параметров (успех/неуспех с причиной).
     */
    rcl_interfaces::msg::SetParametersResult onParameterEvent(
        const std::vector<rclcpp::Parameter> & parameters);

    // --- Члены класса ---

    /// TF2 буфер для хранения трансформаций.
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    /// TF2 слушатель для получения трансформаций.
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    /// TF2 публикатор для статических трансформаций.
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;

    /// Хранилище текущих границ для каждой системы координат (frame_id -> BoundingBox).
    std::map<std::string, BoundingBox> boundaries_;
    /// Мьютекс для потокобезопасного доступа к `boundaries_`.
    mutable std::shared_mutex boundaries_mutex_;

    /// Обработчик для подписки на изменения параметров ROS2.
    std::shared_ptr<rclcpp::ParameterEventHandler> param_subscriber_;
    /// Хендлер для зарегистрированного callback'а параметров.
    rclcpp::ParameterEventHandler::ParameterCallbackHandle::SharedPtr param_callback_handle_;

    /// Множество для отслеживания систем координат, для которых уже объявлены параметры границ.
    std::set<std::string> declared_boundary_frames_;
};

} // namespace promobot_tf_library

#endif // PROMOBOT_TF_LIBRARY_COORDINATE_TRANSFORMER_HPP_

