#include "mono-inertial-node.hpp"

#include <chrono>
#include <cmath>     // ИЗМЕНЕНО: проверка корректности числовых значений.
#include <cstdint>  // ИЗМЕНЕНО: счётчики пропущенных сообщений.
#include <thread>   // ИЗМЕНЕНО: ожидание без удержания mutex.
#include <utility>  // ИЗМЕНЕНО: std::move.
#include "sensor_msgs/image_encodings.hpp"  // ИЗМЕНЕНО [1]: MONO8.
#include <iostream>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>

MonocularInertialNode::MonocularInertialNode(
    ORB_SLAM3::System *SLAM)
    :
    Node("ORB_SLAM3_ROS2_MONO_INERTIAL"),
    SLAM_(SLAM)
{
    // ИЗМЕНЕНО: профиль QoS для датчиков совместим с публикациями Best Effort
    // и Reliable (при обычных настройках ROS 2). Глубину очереди ограничиваем.
    image_sub_ =
        this->create_subscription<ImageMsg>(
            "camera",
            rclcpp::SensorDataQoS().keep_last(20),
            std::bind(
                &MonocularInertialNode::GrabImage,
                this,
                std::placeholders::_1));

    imu_sub_ =
        this->create_subscription<ImuMsg>(
            "imu",
            rclcpp::SensorDataQoS().keep_last(500),
            std::bind(
                &MonocularInertialNode::GrabImu,
                this,
                std::placeholders::_1));

    odom_pub_ =
        this->create_publisher<nav_msgs::msg::Odometry>(
            "odom", 10);

    tf_broadcaster_ =
        std::make_unique<tf2_ros::TransformBroadcaster>(
            this);

    sync_thread_ =
        std::thread(
            &MonocularInertialNode::SyncWithImu,
            this);
}

MonocularInertialNode::~MonocularInertialNode()
{
    running_ = false;

    if (sync_thread_.joinable())
        sync_thread_.join();

    SLAM_->Shutdown();

    SLAM_->SaveKeyFrameTrajectoryTUM(
        "KeyFrameTrajectory.txt");
}

void MonocularInertialNode::GrabImage(
    const ImageMsg::SharedPtr msg)
{
    // ИЗМЕНЕНО: размер очереди увеличен с 2 до 10; выбранный кадр
    // удаляет только SyncWithImu() внутри того же mutex.
    std::lock_guard<std::mutex> lock(image_mutex_);

    image_buf_.push(msg);

    static std::uint64_t dropped_images = 0;
    constexpr std::size_t kMaxImages = 10;

    bool dropped_now = false;
    while (image_buf_.size() > kMaxImages)
    {
        image_buf_.pop();
        ++dropped_images;
        dropped_now = true;
    }

    if (dropped_now && (dropped_images == 1 || dropped_images % 100 == 0))
    {
        RCLCPP_WARN(
            this->get_logger(),
            "Image buffer overflow: total dropped=%llu",
            static_cast<unsigned long long>(dropped_images));
    }
}

void MonocularInertialNode::GrabImu(
    const ImuMsg::SharedPtr msg)
{
    // ИЗМЕНЕНО: отбрасываем нечисловые и повторяющиеся/неупорядоченные
    // IMU-измерения до их попадания в ORB-SLAM3.
    const double t = Utility::StampToSec(msg->header.stamp);

    if (!std::isfinite(t) ||
        !std::isfinite(msg->linear_acceleration.x) ||
        !std::isfinite(msg->linear_acceleration.y) ||
        !std::isfinite(msg->linear_acceleration.z) ||
        !std::isfinite(msg->angular_velocity.x) ||
        !std::isfinite(msg->angular_velocity.y) ||
        !std::isfinite(msg->angular_velocity.z))
    {
        RCLCPP_WARN(this->get_logger(), "Discarding non-finite IMU measurement");
        return;
    }

    std::lock_guard<std::mutex> lock(imu_mutex_);

    // ИЗМЕНЕНО: сохраняем предыдущий timestamp, даже когда очередь
    // полностью опустела. В данном исполняемом файле используется одна нода.
    static double last_imu_timestamp = -1.0;
    if (last_imu_timestamp >= 0.0 && t <= last_imu_timestamp)
    {
        RCLCPP_WARN(
            this->get_logger(),
            "Discarding non-increasing IMU timestamp: %.9f",
            t);
        return;
    }

    last_imu_timestamp = t;
    imu_buf_.push(msg);

    // ИЗМЕНЕНО: ограничение памяти при остановленном SLAM или сбое входных данных.
    // При переполнении требуется проверить нагрузку/частоту обработки.
    constexpr std::size_t kMaxImu = 500;
    static std::uint64_t dropped_imu = 0;
    bool dropped_now = false;
    while (imu_buf_.size() > kMaxImu)
    {
        imu_buf_.pop();
        ++dropped_imu;
        dropped_now = true;
    }
    if (dropped_now && (dropped_imu == 1 || dropped_imu % 500 == 0))
    {
        RCLCPP_WARN(
            this->get_logger(),
            "IMU buffer overflow: total dropped=%llu",
            static_cast<unsigned long long>(dropped_imu));
    }
}

void MonocularInertialNode::SyncWithImu()
{
    // ИЗМЕНЕНО: диагностика потери кадров и тайм-аут ожидания IMU.
    // Время в секундах относится к header.stamp (время симуляции).
    double previous_image_time = -1.0;
    std::uint64_t processed_frames = 0;
    constexpr double kImageGapWarning = 0.10; // Для камеры 20 Гц: ожидается ~0.05 с.
    constexpr double kImuGapWarning = 0.025;  // Для IMU 200 Гц: ожидается ~0.005 с.
    const auto kMaxImuWait = std::chrono::seconds(5);

    while (running_ && rclcpp::ok())
    {
        ImageMsg::SharedPtr image_msg;

        // ИЗМЕНЕНО: ВАЖНО - читаем И удаляем один и тот же кадр атомарно.
        {
            std::lock_guard<std::mutex> lock(image_mutex_);
            if (!image_buf_.empty())
            {
                image_msg = image_buf_.front();
                image_buf_.pop();
            }
        }

        if (!image_msg)
        {
            // ИЗМЕНЕНО: ожидание вне области блокировки mutex.
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            continue;
        }

        const double tImage = Utility::StampToSec(image_msg->header.stamp);
        if (!std::isfinite(tImage))
        {
            RCLCPP_WARN(this->get_logger(), "Discarding image with invalid timestamp");
            continue;
        }

        // ИЗМЕНЕНО: не передаём в ORB-SLAM3 кадры с обратным временем.
        if (previous_image_time >= 0.0)
        {
            const double dt = tImage - previous_image_time;
            if (dt <= 0.0)
            {
                RCLCPP_ERROR(
                    this->get_logger(),
                    "Non-increasing image time: previous=%.6f current=%.6f. "
                    "If Gazebo simulation time was reset, restart this node.",
                    previous_image_time, tImage);
                continue;
            }
            if (dt > kImageGapWarning)
            {
                RCLCPP_WARN(
                    this->get_logger(),
                    "Large gap between processed frames: %.6f s (%.6f -> %.6f)",
                    dt, previous_image_time, tImage);
            }
        }

        // ИЗМЕНЕНО: проверяем изображение ДО удаления IMU из очереди.
        // В противном случае ошибка cv_bridge приводила бы к потере IMU-данных.
        cv_bridge::CvImagePtr cv_ptr;
        try
        {
            cv_ptr = cv_bridge::toCvCopy(
                image_msg, sensor_msgs::image_encodings::MONO8);
        }
        catch (const cv_bridge::Exception &e)
        {
            RCLCPP_ERROR(this->get_logger(),
                         "cv_bridge exception: %s", e.what());
            continue;
        }

        // ИЗМЕНЕНО: пакет IMU формируется атомарно и не расходуется,
        // пока недостаточно измерений. В пакет включаем ПЕРВОЕ измерение
        // с tImu >= tImage, которое требуется ORB-SLAM3 для правой границы
        // предынтегрирования (Tracking::PreintegrateIMU).
        std::vector<ORB_SLAM3::IMU::Point> imu_measurements;
        bool imu_ready = false;
        bool no_older_imu = false;
        const auto wait_started = std::chrono::steady_clock::now();

        while (running_ && rclcpp::ok())
        {
            {
                std::lock_guard<std::mutex> lock(imu_mutex_);

                if (!imu_buf_.empty())
                {
                    const double latest_time =
                        Utility::StampToSec(imu_buf_.back()->header.stamp);
                    const double earliest_time =
                        Utility::StampToSec(imu_buf_.front()->header.stamp);

                    // Все оставшиеся измерения IMU уже новее данного кадра.
                    // Обработать такой кадр корректно нельзя.
                    if (earliest_time > tImage)
                    {
                        no_older_imu = true;
                    }
                    else if (latest_time >= tImage)
                    {
                        // Снимок очереди позволяет проверить пакет, ничего не удаляя.
                        auto pending = imu_buf_;
                        std::size_t to_consume = 0;
                        bool has_left_boundary = false;
                        bool has_right_boundary = false;
                        std::vector<ORB_SLAM3::IMU::Point> candidate;

                        while (!pending.empty())
                        {
                            const auto &imu = pending.front();
                            const double tImu =
                                Utility::StampToSec(imu->header.stamp);

                            const cv::Point3f acc(
                                imu->linear_acceleration.x,
                                imu->linear_acceleration.y,
                                imu->linear_acceleration.z);
                            const cv::Point3f gyro(
                                imu->angular_velocity.x,
                                imu->angular_velocity.y,
                                imu->angular_velocity.z);

                            candidate.emplace_back(acc, gyro, tImu);
                            ++to_consume;
                            has_left_boundary = has_left_boundary || (tImu <= tImage);
                            has_right_boundary = has_right_boundary || (tImu >= tImage);

                            if (tImu >= tImage)
                                break;

                            pending.pop();
                        }

                        // Не удаляем IMU, если пакет неполный: он понадобится
                        // для следующего кадра при пропуске текущего.
                        if (has_left_boundary && has_right_boundary &&
                            candidate.size() >= 2)
                        {
                            imu_measurements = std::move(candidate);
                            for (std::size_t i = 0; i < to_consume; ++i)
                                imu_buf_.pop();
                            imu_ready = true;
                        }
                    }
                }
            }

            if (imu_ready || no_older_imu)
                break;

            if (std::chrono::steady_clock::now() - wait_started > kMaxImuWait)
                break;

            // ИЗМЕНЕНО: IMU callback может работать во время ожидания.
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }

        if (!imu_ready)
        {
            // ИЗМЕНЕНО: пропускаем ТОЛЬКО выбранное изображение;
            // не делаем повторный pop() и не удаляем частичный пакет IMU.
            RCLCPP_WARN(
                this->get_logger(),
                "Skipping image %.6f: %s",
                tImage,
                no_older_imu ? "IMU data begin after this image" :
                               "incomplete IMU interval or timeout");
            continue;
        }

        // ИЗМЕНЕНО: диагностика слишком больших внутренних пробелов IMU.
        for (std::size_t i = 1; i < imu_measurements.size(); ++i)
        {
            const double dt = imu_measurements[i].t - imu_measurements[i - 1].t;
            if (dt > kImuGapWarning)
            {
                RCLCPP_WARN(
                    this->get_logger(),
                    "IMU gap %.6f s before image %.6f",
                    dt, tImage);
            }
        }

        // ИЗМЕНЕНО: CLAHE временно отключён для сопоставления с Stereo
        // при doEqual=false.
        // Не включайте его без явной настройки и изменения заголовочного файла.
        // clahe_->apply(cv_ptr->image, cv_ptr->image);

        // ИЗМЕНЕНО: передаём строго возрастающие кадры и IMU-пакет
        // с граничным измерением в ORB-SLAM3.
        const Sophus::SE3f Tcw = SLAM_->TrackMonocular(
            cv_ptr->image, tImage, imu_measurements);
        previous_image_time = tImage;
        ++processed_frames;

        // ИЗМЕНЕНО: проверяем реальное состояние трекинга,
        // а не только нулевую матрицу. При потере трекинга запрещаем
        // связывать относительные позы из разных карт.
        const int tracking_state = SLAM_->GetTrackingState();
        if (tracking_state != ORB_SLAM3::Tracking::OK ||
            !Tcw.matrix().allFinite() ||
            Tcw.matrix().isZero(0))
        {
            if (prev_tracking_ok_)
            {
                RCLCPP_WARN(
                    this->get_logger(),
                    "Tracking lost/invalid, state=%d, image=%.6f",
                    tracking_state, tImage);
            }
            prev_tracking_ok_ = false;
            continue;
        }

        // Camera -> World
        const Sophus::SE3f Twc = Tcw.inverse();

        if (!has_prev_pose_)
        {
            prev_pose_ = Twc;
            accumulated_pose_ = Sophus::SE3f();
            has_prev_pose_ = true;
        }
        else if (prev_tracking_ok_)
        {
            const Sophus::SE3f delta = prev_pose_.inverse() * Twc;
            accumulated_pose_ = accumulated_pose_ * delta;
        }
        else
        {
            // ИЗМЕНЕНО: восстановление трекинга: начинаем новый
            // сегмент без скачка между несвязанными системами координат.
            RCLCPP_INFO(
                this->get_logger(),
                "Tracking recovered at image %.6f; starting new odom segment",
                tImage);
        }

        prev_pose_ = Twc;
        prev_tracking_ok_ = true;

        Eigen::Vector3f t_slam = accumulated_pose_.translation();
        Eigen::Matrix3f R_slam = accumulated_pose_.rotationMatrix();

        // ИЗМЕНЕНО: оставлено исходное преобразование осей.
        Eigen::Matrix3f R_slam_to_ros;
        R_slam_to_ros <<
             0,  0, 1,
            -1,  0, 0,
             0, -1, 0;

        Eigen::Vector3f t_ros = R_slam_to_ros * t_slam;
        Eigen::Matrix3f R_ros =
            R_slam_to_ros * R_slam * R_slam_to_ros.transpose();

        Eigen::Quaternionf q_ros(R_ros);
        q_ros.normalize();

        nav_msgs::msg::Odometry odom_msg;
        odom_msg.header.stamp = image_msg->header.stamp;
        odom_msg.header.frame_id = "odom";
        odom_msg.child_frame_id = "base_link";

        odom_msg.pose.pose.position.x = t_ros.x();
        odom_msg.pose.pose.position.y = t_ros.y();
        odom_msg.pose.pose.position.z = t_ros.z();

        odom_msg.pose.pose.orientation.x = q_ros.x();
        odom_msg.pose.pose.orientation.y = q_ros.y();
        odom_msg.pose.pose.orientation.z = q_ros.z();
        odom_msg.pose.pose.orientation.w = q_ros.w();

        odom_pub_->publish(odom_msg);

        geometry_msgs::msg::TransformStamped tf_msg;
        tf_msg.header = odom_msg.header;
        tf_msg.child_frame_id = odom_msg.child_frame_id;

        tf_msg.transform.translation.x = t_ros.x();
        tf_msg.transform.translation.y = t_ros.y();
        tf_msg.transform.translation.z = t_ros.z();
        tf_msg.transform.rotation = odom_msg.pose.pose.orientation;

        tf_broadcaster_->sendTransform(tf_msg);

        // ИЗМЕНЕНО: периодическая диагностика фактической обработки.
        if (processed_frames % 100 == 0)
        {
            RCLCPP_INFO(
                this->get_logger(),
                "Processed %llu frames; most recent image %.6f, IMU samples=%zu",
                static_cast<unsigned long long>(processed_frames),
                tImage, imu_measurements.size());
        }
    }
}
