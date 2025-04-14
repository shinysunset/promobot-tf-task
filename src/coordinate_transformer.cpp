#include "promobot_tf_library/coordinate_transformer.hpp"
#include <rclcpp_components/register_node_macro.hpp> // Для возможной композиции узла

#include <tf2/exceptions.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp> // Для toMsg
#include <filesystem> // Для проверки существования файла

namespace promobot_tf_library
{

// --- Конструктор и инициализация ---

CoordinateTransformer::CoordinateTransformer(const rclcpp::NodeOptions & options)
    : Node("coordinate_transformer", options.automatically_declare_parameters_from_overrides(true)) // Имя узла, разрешаем параметры из launch/YAML
{
    RCLCPP_INFO(this->get_logger(), "Initializing CoordinateTransformer node...");

    // Инициализация TF2
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    // tf_buffer_->setUsingDedicatedThread(true); // Раскомментировать, если будут проблемы с производительностью TF
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, this, false); // false - не создаем отдельный поток для листенера здесь
    static_tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

    // Инициализация обработчика параметров ROS2
    param_subscriber_ = std::make_shared<rclcpp::ParameterEventHandler>(this);

    // Устанавливаем callback на изменение ЛЮБЫХ параметров с префиксом "boundaries."
    // Это более эффективно, чем отслеживать все параметры узла.
    auto param_callback = [this](const std::vector<rclcpp::Parameter> & parameters) {
            return this->onParameterEvent(parameters);
        };
    param_callback_handle_ = param_subscriber_->add_parameters_callback("boundaries", param_callback);


    RCLCPP_INFO(this->get_logger(), "CoordinateTransformer node initialized successfully.");
}

// --- Загрузка и добавление трансформаций ---

bool CoordinateTransformer::loadTransformsFromYaml(const std::string& yaml_path)
{
    RCLCPP_INFO(this->get_logger(), "Attempting to load transforms from YAML: %s", yaml_path.c_str());

    if (!std::filesystem::exists(yaml_path)) {
         RCLCPP_ERROR(this->get_logger(), "YAML file not found: %s", yaml_path.c_str());
         return false;
    }

    try {
        YAML::Node config = YAML::LoadFile(yaml_path);
        if (!config["static_transforms"]) {
             RCLCPP_WARN(this->get_logger(), "No 'static_transforms' section found in YAML file: %s", yaml_path.c_str());
             return true; // Не ошибка, просто нет трансформаций для загрузки
        }

        std::vector<geometry_msgs::msg::TransformStamped> transforms_to_send;
        for (const auto& entry : config["static_transforms"]) {
            if (!entry["parent_frame"] || !entry["child_frame"] || !entry["translation"]) {
                 RCLCPP_ERROR(this->get_logger(), "Invalid transform entry in YAML: missing parent_frame, child_frame, or translation.");
                 continue; // Пропускаем некорректную запись
            }

            geometry_msgs::msg::TransformStamped t;
            t.header.stamp = this->get_clock()->now(); // Ставим текущее время для статической трансформации
            t.header.frame_id = entry["parent_frame"].as<std::string>();
            t.child_frame_id = entry["child_frame"].as<std::string>();

            try {
                t.transform.translation.x = entry["translation"]["x"].as<double>();
                t.transform.translation.y = entry["translation"]["y"].as<double>();
                t.transform.translation.z = entry["translation"]["z"].as<double>();
            } catch (const YAML::Exception& e) {
                 RCLCPP_ERROR(this->get_logger(), "Invalid 'translation' format for %s -> %s: %s",
                              t.header.frame_id.c_str(), t.child_frame_id.c_str(), e.what());
                 continue;
            }


            // Обработка вращения: сначала RPY, потом кватернион
            tf2::Quaternion q;
            bool rotation_set = false;
            if (entry["rotation_rpy"]) {
                 try {
                     q.setRPY(
                         entry["rotation_rpy"]["roll"].as<double>(0.0), // Значения по умолчанию 0
                         entry["rotation_rpy"]["pitch"].as<double>(0.0),
                         entry["rotation_rpy"]["yaw"].as<double>(0.0)
                     );
                     rotation_set = true;
                     RCLCPP_DEBUG(this->get_logger(), "Using RPY rotation for %s -> %s", t.header.frame_id.c_str(), t.child_frame_id.c_str());
                 } catch (const YAML::Exception& e) {
                     RCLCPP_ERROR(this->get_logger(), "Invalid 'rotation_rpy' format for %s -> %s: %s",
                                  t.header.frame_id.c_str(), t.child_frame_id.c_str(), e.what());
                     continue;
                 }
            } else if (entry["rotation_quat"]) {
                 try {
                     q.setX(entry["rotation_quat"]["x"].as<double>(0.0)); // Значения по умолчанию для identity
                     q.setY(entry["rotation_quat"]["y"].as<double>(0.0));
                     q.setZ(entry["rotation_quat"]["z"].as<double>(0.0));
                     q.setW(entry["rotation_quat"]["w"].as<double>(1.0));
                     q.normalize(); // Нормализуем на всякий случай
                     rotation_set = true;
                     RCLCPP_DEBUG(this->get_logger(), "Using Quaternion rotation for %s -> %s", t.header.frame_id.c_str(), t.child_frame_id.c_str());
                 } catch (const YAML::Exception& e) {
                      RCLCPP_ERROR(this->get_logger(), "Invalid 'rotation_quat' format for %s -> %s: %s",
                                   t.header.frame_id.c_str(), t.child_frame_id.c_str(), e.what());
                      continue;
                 }
            }

            if (!rotation_set) {
                RCLCPP_WARN(this->get_logger(), "No rotation specified for transform %s -> %s. Assuming identity rotation (0,0,0,1).",
                            t.header.frame_id.c_str(), t.child_frame_id.c_str());
                q.setRPY(0, 0, 0); // Identity quaternion
            }

            t.transform.rotation = tf2::toMsg(q);
            transforms_to_send.push_back(t);

            // Объявляем параметры для границ этих фреймов, если еще не сделали
            declareBoundaryParameters(t.header.frame_id);
            declareBoundaryParameters(t.child_frame_id);

            RCLCPP_INFO(this->get_logger(), "Loaded static transform from YAML: %s -> %s",
                        t.header.frame_id.c_str(), t.child_frame_id.c_str());
        }

        if (!transforms_to_send.empty()) {
             static_tf_broadcaster_->sendTransform(transforms_to_send);
             RCLCPP_INFO(this->get_logger(), "Published %zu static transforms from YAML.", transforms_to_send.size());
        } else {
            RCLCPP_INFO(this->get_logger(), "No valid static transforms found to publish from YAML.");
        }

    } catch (const YAML::Exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Failed to parse YAML file %s: %s", yaml_path.c_str(), e.what());
        return false;
    } catch (const std::exception& e) { // Ловим другие возможные исключения (например, filesystem)
        RCLCPP_ERROR(this->get_logger(), "Error loading transforms from YAML: %s", e.what());
        return false;
    }
    return true;
}

bool CoordinateTransformer::addManualTransform(
    const std::string& parent_frame,
    const std::string& child_frame,
    double tx, double ty, double tz,
    double roll, double pitch, double yaw)
{
    if (parent_frame.empty() || child_frame.empty()) {
        RCLCPP_ERROR(this->get_logger(), "Parent and child frame IDs cannot be empty for manual transform.");
        return false;
    }

    RCLCPP_INFO(this->get_logger(), "Adding manual static transform: %s -> %s", parent_frame.c_str(), child_frame.c_str());

    try {
        geometry_msgs::msg::TransformStamped t;
        t.header.stamp = this->get_clock()->now();
        t.header.frame_id = parent_frame;
        t.child_frame_id = child_frame;

        t.transform.translation.x = tx;
        t.transform.translation.y = ty;
        t.transform.translation.z = tz;

        tf2::Quaternion q;
        q.setRPY(roll, pitch, yaw);
        t.transform.rotation = tf2::toMsg(q);

        static_tf_broadcaster_->sendTransform(t);

        // Объявляем параметры для границ, если еще не сделали
        declareBoundaryParameters(t.header.frame_id);
        declareBoundaryParameters(t.child_frame_id);

        RCLCPP_INFO(this->get_logger(), "Successfully added and published manual static transform.");
        return true;
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Error adding manual transform %s -> %s: %s",
                     parent_frame.c_str(), child_frame.c_str(), e.what());
        return false;
    }
}

// --- Преобразование точки и проверка границ ---

std::pair<std::optional<geometry_msgs::msg::PointStamped>, TransformStatus> CoordinateTransformer::transformPoint(
    const geometry_msgs::msg::PointStamped& input_point,
    const std::string& target_frame)
{
    if (input_point.header.frame_id.empty()) {
         RCLCPP_ERROR(this->get_logger(), "Input point is missing frame_id.");
         return {{}, TransformStatus::FRAME_NOT_FOUND};
    }
    if (target_frame.empty()) {
        RCLCPP_ERROR(this->get_logger(), "Target frame cannot be empty.");
         return {{}, TransformStatus::FRAME_NOT_FOUND};
    }
    if (input_point.header.frame_id == target_frame) {
        RCLCPP_WARN(this->get_logger(), "Input and target frames are the same ('%s'). No transformation needed.", target_frame.c_str());
        // Проверяем границы даже если трансформация не нужна
        if (!checkBounds(input_point.point, target_frame)) {
            return {input_point, TransformStatus::OUT_OF_BOUNDS};
        }
        return {input_point, TransformStatus::SUCCESS};
    }

    RCLCPP_DEBUG(this->get_logger(), "Attempting to transform point from '%s' to '%s' at time %f",
                 input_point.header.frame_id.c_str(), target_frame.c_str(), rclcpp::Time(input_point.header.stamp).seconds());

    geometry_msgs::msg::PointStamped transformed_point_msg;
    try {
        // Устанавливаем таймаут ожидания трансформации
        // tf2::durationFromSec(0) может не подождать, лучше небольшой таймаут
        rclcpp::Duration timeout = rclcpp::Duration::from_seconds(0.1); // 100 мс

        // Используем canTransform с таймаутом, чтобы не блокировать надолго, если трансформация недоступна
        std::string tf_error_msg;
        if (!tf_buffer_->canTransform(target_frame, input_point.header.frame_id, input_point.header.stamp, timeout, &tf_error_msg))
        {
            RCLCPP_ERROR(this->get_logger(), "Cannot transform point: %s", tf_error_msg.c_str());
            // Попробуем определить причину более точно (хотя canTransform часто возвращает общую ошибку)
            if (tf_error_msg.find("target_frame") != std::string::npos || tf_error_msg.find("source_frame") != std::string::npos) {
                 return {{}, TransformStatus::FRAME_NOT_FOUND};
            }
            return {{}, TransformStatus::TRANSFORM_FAILED};
        }

        // Если canTransform прошел, сама трансформация должна выполниться быстро
        transformed_point_msg = tf_buffer_->transform(input_point, target_frame, tf2::durationFromSec(0)); // Используем 0, т.к. уже проверили доступность

        RCLCPP_DEBUG(this->get_logger(), "Transform successful. Checking bounds for frame '%s'.", target_frame.c_str());

        // Проверяем границы в целевой системе координат
        if (!checkBounds(transformed_point_msg.point, target_frame)) {
            RCLCPP_WARN(this->get_logger(), "Transformed point [x: %.3f, y: %.3f, z: %.3f] in frame '%s' is OUT OF BOUNDS.",
                        transformed_point_msg.point.x, transformed_point_msg.point.y, transformed_point_msg.point.z,
                        target_frame.c_str());
            // Возвращаем точку, но со статусом OUT_OF_BOUNDS
            return {transformed_point_msg, TransformStatus::OUT_OF_BOUNDS};
        }

        // Все успешно
        RCLCPP_DEBUG(this->get_logger(), "Transformed point is within bounds.");
        return {transformed_point_msg, TransformStatus::SUCCESS};

    } catch (const tf2::LookupException& ex) {
        RCLCPP_ERROR(this->get_logger(), "Transform lookup failed from '%s' to '%s': %s",
            input_point.header.frame_id.c_str(), target_frame.c_str(), ex.what());
        return {{}, TransformStatus::FRAME_NOT_FOUND};
    } catch (const tf2::ConnectivityException& ex) {
        RCLCPP_ERROR(this->get_logger(), "Transform connectivity failed from '%s' to '%s': %s",
            input_point.header.frame_id.c_str(), target_frame.c_str(), ex.what());
         return {{}, TransformStatus::FRAME_NOT_FOUND}; // Часто означает, что нет пути между фреймами
    } catch (const tf2::ExtrapolationException& ex) {
         RCLCPP_ERROR(this->get_logger(), "Transform extrapolation failed from '%s' to '%s' (time mismatch?): %s",
            input_point.header.frame_id.c_str(), target_frame.c_str(), ex.what());
         return {{}, TransformStatus::TRANSFORM_FAILED}; // Ошибка времени
    } catch (const tf2::TransformException& ex) {
        // Ловим остальные ошибки tf2
        RCLCPP_ERROR(this->get_logger(), "Generic TF transform error from '%s' to '%s': %s",
             input_point.header.frame_id.c_str(), target_frame.c_str(), ex.what());
        return {{}, TransformStatus::TRANSFORM_FAILED};
    } catch (const std::exception& e) {
         // Ловим другие возможные исключения C++
         RCLCPP_ERROR(this->get_logger(), "Unexpected error during transform: %s", e.what());
         return {{}, TransformStatus::TRANSFORM_FAILED};
    }
}

bool CoordinateTransformer::checkBounds(const geometry_msgs::msg::Point& point, const std::string& frame_id) {
    // Используем shared_lock для чтения, чтобы разрешить одновременное чтение нескольким потокам
    std::shared_lock lock(boundaries_mutex_);
    auto it = boundaries_.find(frame_id);
    if (it != boundaries_.end()) {
        const auto& box = it->second;
        bool in_bounds = point.x >= box.min_x && point.x <= box.max_x &&
                         point.y >= box.min_y && point.y <= box.max_y &&
                         point.z >= box.min_z && point.z <= box.max_z;
        RCLCPP_DEBUG(this->get_logger(), "Checking bounds for %s: Point[%.2f, %.2f, %.2f] vs Box[X(%.2f,%.2f) Y(%.2f,%.2f) Z(%.2f,%.2f)] -> %s",
                    frame_id.c_str(), point.x, point.y, point.z,
                    box.min_x, box.max_x, box.min_y, box.max_y, box.min_z, box.max_z,
                    in_bounds ? "IN" : "OUT");
        return in_bounds;
    }
    RCLCPP_DEBUG(this->get_logger(), "No bounds defined for frame '%s'. Assuming point is within bounds.", frame_id.c_str());
    return true; // Если границы не заданы, считаем, что точка внутри
}


// --- Обработка параметров ROS2 ---

void CoordinateTransformer::declareBoundaryParameters(const std::string& frame_id) {
    if (frame_id.empty()) return;

    // Блокируем мьютекс для записи, так как будем модифицировать declared_boundary_frames_ и boundaries_
    std::unique_lock lock(boundaries_mutex_);

    // Проверяем, объявляли ли уже параметры для этого фрейма
    if (declared_boundary_frames_.count(frame_id)) {
        RCLCPP_DEBUG(this->get_logger(), "Boundary parameters for frame '%s' already declared.", frame_id.c_str());
        return;
    }

    // Формируем префикс для параметров этого фрейма
    std::string prefix = "boundaries." + frame_id + ".";
    RCLCPP_INFO(this->get_logger(), "Declaring boundary parameters for frame: %s (prefix: %s)", frame_id.c_str(), prefix.c_str());

    try {
        // Используем rcl_interfaces::msg::ParameterDescriptor для описания
        auto desc_min_x = rcl_interfaces::build<rcl_interfaces::msg::ParameterDescriptor>()
                            .name(prefix + "min_x")
                            .description("Minimum X boundary for frame " + frame_id)
                            .read_only(false) // Параметр можно изменять
                            .dynamic_typing(false) // Тип фиксирован (double)
                            .type(rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE);
        auto desc_max_x = rcl_interfaces::build<rcl_interfaces::msg::ParameterDescriptor>()... // Аналогично для max_x
        auto desc_min_y = rcl_interfaces::build<rcl_interfaces::msg::ParameterDescriptor>()... // min_y
        auto desc_max_y = rcl_interfaces::build<rcl_interfaces::msg::ParameterDescriptor>()... // max_y
        auto desc_min_z = rcl_interfaces::build<rcl_interfaces::msg::ParameterDescriptor>()... // min_z
        auto desc_max_z = rcl_interfaces::build<rcl_interfaces::msg::ParameterDescriptor>()... // max_z


        // Объявляем параметры с дефолтными значениями +/- infinity
        double default_min = -std::numeric_limits<double>::infinity();
        double default_max = std::numeric_limits<double>::infinity();

        this->declare_parameter<double>(prefix + "min_x", default_min, desc_min_x);
        this->declare_parameter<double>(prefix + "max_x", default_max, desc_max_x);
        this->declare_parameter<double>(prefix + "min_y", default_min, desc_min_y);
        this->declare_parameter<double>(prefix + "max_y", default_max, desc_max_y);
        this->declare_parameter<double>(prefix + "min_z", default_min, desc_min_z);
        this->declare_parameter<double>(prefix + "max_z", default_max, desc_max_z);

        // Сразу считываем начальные значения (дефолтные или из launch/YAML файла) в нашу структуру
        boundaries_[frame_id] = BoundingBox{
            .min_x = this->get_parameter(prefix + "min_x").as_double(),
            .max_x = this->get_parameter(prefix + "max_x").as_double(),
            .min_y = this->get_parameter(prefix + "min_y").as_double(),
            .max_y = this->get_parameter(prefix + "max_y").as_double(),
            .min_z = this->get_parameter(prefix + "min_z").as_double(),
            .max_z = this->get_parameter(prefix + "max_z").as_double()
        };

        // Отмечаем, что параметры для этого фрейма объявлены
        declared_boundary_frames_.insert(frame_id);
        RCLCPP_INFO(this->get_logger(), "Successfully declared and initialized boundary parameters for frame '%s'.", frame_id.c_str());

    } catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException &e) {
        // Это может произойти, если два потока одновременно пытаются объявить параметры для одного фрейма.
        // Или если параметры были заданы через launch файл до вызова declare_parameter.
        RCLCPP_WARN(this->get_logger(), "Parameters for frame '%s' already declared (possibly from launch file?): %s", frame_id.c_str(), e.what());
        // Попробуем считать значения, раз они уже объявлены
        try {
             boundaries_[frame_id] = BoundingBox{
                .min_x = this->get_parameter(prefix + "min_x").as_double(),
                .max_x = this->get_parameter(prefix + "max_x").as_double(),
                .min_y = this->get_parameter(prefix + "min_y").as_double(),
                .max_y = this->get_parameter(prefix + "max_y").as_double(),
                .min_z = this->get_parameter(prefix + "min_z").as_double(),
                .max_z = this->get_parameter(prefix + "max_z").as_double()
             };
             declared_boundary_frames_.insert(frame_id); // Все равно отмечаем
        } catch (const rclcpp::exceptions::ParameterNotDeclaredException &e_inner) {
             RCLCPP_ERROR(this->get_logger(), "Failed to get already declared parameters for frame '%s': %s", frame_id.c_str(), e_inner.what());
        }
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Failed to declare parameters for frame '%s': %s", frame_id.c_str(), e.what());
    }
}

rcl_interfaces::msg::SetParametersResult CoordinateTransformer::onParameterEvent(
    const std::vector<rclcpp::Parameter> & parameters)
{
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true; // По умолчанию считаем успешным

    // Блокируем мьютекс для записи, так как будем обновлять boundaries_
    std::unique_lock lock(boundaries_mutex_);

    for (const auto & param : parameters) {
        const std::string& name = param.get_name();
        RCLCPP_INFO(this->get_logger(), "Parameter change detected: %s (type: %s)", name.c_str(), param.get_type_name().c_str());

        // Проверяем, относится ли параметр к границам (по префиксу "boundaries.")
        if (name.rfind("boundaries.", 0) != 0) {
            RCLCPP_DEBUG(this->get_logger(), "Ignoring parameter '%s' (does not start with 'boundaries.')", name.c_str());
            continue; // Игнорируем параметры, не связанные с границами
        }

        // Разбираем имя параметра: boundaries.<frame_id>.<coord_key>
        std::string frame_and_coord = name.substr(strlen("boundaries."));
        size_t last_dot = frame_and_coord.rfind('.');
        if (last_dot == std::string::npos || last_dot == 0 || last_dot == frame_and_coord.length() - 1) {
            RCLCPP_WARN(this->get_logger(), "Malformed boundary parameter name: %s. Expected 'boundaries.<frame_id>.<coord_key>'", name.c_str());
            result.successful = false;
            result.reason = "Malformed boundary parameter name: " + name;
            continue; // Пропускаем некорректный параметр
        }

        std::string frame_id = frame_and_coord.substr(0, last_dot);
        std::string coord_key = frame_and_coord.substr(last_dot + 1);

        // Проверяем тип параметра
        if (param.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE) {
             RCLCPP_WARN(this->get_logger(), "Boundary parameter '%s' must be a double, but received type '%s'.",
                        name.c_str(), param.get_type_name().c_str());
             result.successful = false;
             result.reason = "Boundary parameter " + name + " must be a double.";
             continue;
        }

        double value = param.as_double();

        // Находим или создаем запись для frame_id в boundaries_
        // (Создание нужно на случай, если параметр устанавливается до того, как фрейм был добавлен через YAML/addManual)
        auto it = boundaries_.find(frame_id);
        if (it == boundaries_.end()) {
            RCLCPP_INFO(this->get_logger(), "Initializing boundaries for frame '%s' due to parameter set.", frame_id.c_str());
            // Создаем запись с дефолтными значениями (infinity), затем обновим конкретное поле
            boundaries_[frame_id] = BoundingBox{};
            it = boundaries_.find(frame_id); // Получаем итератор на созданный элемент
            // Отметим, что параметры теперь существуют (хотя мы их не объявляли явно здесь,
            // но они были установлены извне, и мы их обрабатываем)
            if (declared_boundary_frames_.find(frame_id) == declared_boundary_frames_.end()) {
                declared_boundary_frames_.insert(frame_id);
                 RCLCPP_INFO(this->get_logger(), "Implicitly marked frame '%s' as having boundary parameters.", frame_id.c_str());
            }
        }

        // Обновляем соответствующее поле в структуре границ
        bool updated = true;
        if (coord_key == "min_x") it->second.min_x = value;
        else if (coord_key == "max_x") it->second.max_x = value;
        else if (coord_key == "min_y") it->second.min_y = value;
        else if (coord_key == "max_y") it->second.max_y = value;
        else if (coord_key == "min_z") it->second.min_z = value;
        else if (coord_key == "max_z") it->second.max_z = value;
        else {
            RCLCPP_WARN(this->get_logger(), "Unknown coordinate key '%s' in parameter name: %s",
                        coord_key.c_str(), name.c_str());
            result.successful = false;
            result.reason = "Unknown coordinate key '" + coord_key + "' in parameter name " + name;
            updated = false;
        }

        if (updated) {
            RCLCPP_INFO(this->get_logger(), "Updated boundary %s.%s to %.3f",
                       frame_id.c_str(), coord_key.c_str(), value);
        }
    } // end for loop parameters

    // Если хотя бы один параметр не удалось установить, результат будет неуспешным
    return result;
}


} // namespace promobot_tf_library

// Регистрация компонента для возможного использования в launch-файлах через composition
// Это позволяет запускать несколько узлов в одном процессе для эффективности.
RCLCPP_COMPONENTS_REGISTER_NODE(promobot_tf_library::CoordinateTransformer)

