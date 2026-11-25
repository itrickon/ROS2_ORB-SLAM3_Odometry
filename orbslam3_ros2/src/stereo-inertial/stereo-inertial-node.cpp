#include "stereo-inertial-node.hpp"

#include <opencv2/core/core.hpp>

#include <sophus/se3.hpp>

#include <iostream>

#include <Eigen/Dense>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <chrono>
#include <thread>

#include <cmath>
#include <vector>

using std::placeholders::_1;

StereoInertialNode::StereoInertialNode(ORB_SLAM3::System *SLAM, const string &strSettingsFile, const string &strDoRectify, const string &strDoEqual) :
    Node("ORB_SLAM3_ROS2"),
    SLAM_(SLAM)
{
    stringstream ss_rec(strDoRectify);
    ss_rec >> boolalpha >> doRectify_;

    stringstream ss_eq(strDoEqual);
    ss_eq >> boolalpha >> doEqual_;

    bClahe_ = doEqual_;
    std::cout << "Rectify: " << doRectify_ << std::endl;
    std::cout << "Equal: " << doEqual_ << std::endl;

    if (doRectify_)
    {
        // Load settings related to stereo calibration
        cv::FileStorage fsSettings(strSettingsFile, cv::FileStorage::READ);
        if (!fsSettings.isOpened())
        {
            cerr << "ERROR: Wrong path to settings" << endl;
            assert(0);
        }

        cv::Mat K_l, K_r, P_l, P_r, R_l, R_r, D_l, D_r;
        fsSettings["LEFT.K"] >> K_l;
        fsSettings["RIGHT.K"] >> K_r;

        fsSettings["LEFT.P"] >> P_l;
        fsSettings["RIGHT.P"] >> P_r;

        fsSettings["LEFT.R"] >> R_l;
        fsSettings["RIGHT.R"] >> R_r;

        fsSettings["LEFT.D"] >> D_l;
        fsSettings["RIGHT.D"] >> D_r;

        int rows_l = fsSettings["LEFT.height"];
        int cols_l = fsSettings["LEFT.width"];
        int rows_r = fsSettings["RIGHT.height"];
        int cols_r = fsSettings["RIGHT.width"];

        if (K_l.empty() || K_r.empty() || P_l.empty() || P_r.empty() || R_l.empty() || R_r.empty() || D_l.empty() || D_r.empty() ||
            rows_l == 0 || rows_r == 0 || cols_l == 0 || cols_r == 0)
        {
            cerr << "ERROR: Calibration parameters to rectify stereo are missing!" << endl;
            assert(0);
        }

        cv::initUndistortRectifyMap(K_l, D_l, R_l, P_l.rowRange(0, 3).colRange(0, 3), cv::Size(cols_l, rows_l), CV_32F, M1l_, M2l_);
        cv::initUndistortRectifyMap(K_r, D_r, R_r, P_r.rowRange(0, 3).colRange(0, 3), cv::Size(cols_r, rows_r), CV_32F, M1r_, M2r_);
    }
    /*
    subImu_ = this->create_subscription<ImuMsg>("imu", 1000, std::bind(&StereoInertialNode::GrabImu, this, _1));
    subImgLeft_ = this->create_subscription<ImageMsg>("camera/left", 100, std::bind(&StereoInertialNode::GrabImageLeft, this, _1));
    subImgRight_ = this->create_subscription<ImageMsg>("camera/right", 100, std::bind(&StereoInertialNode::GrabImageRight, this, _1));
    */
    
    subImu_ = this->create_subscription<ImuMsg>("imu", rclcpp::SensorDataQoS().keep_last(500), std::bind(&StereoInertialNode::GrabImu, this, _1)); // Вставка
    subImgLeft_ = this->create_subscription<ImageMsg>("camera/left", rclcpp::SensorDataQoS().keep_last(20), std::bind(&StereoInertialNode::GrabImageLeft, this, _1));
    subImgRight_ = this->create_subscription<ImageMsg>("camera/right", rclcpp::SensorDataQoS().keep_last(20), std::bind(&StereoInertialNode::GrabImageRight, this, _1));
    //
    
    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);
    syncThread_ = new std::thread(&StereoInertialNode::SyncWithImu, this);
}

StereoInertialNode::~StereoInertialNode()
{
    running_ = false; // Вставка
    // Delete sync thread
    syncThread_->join();
    delete syncThread_;

    // Stop all threads
    SLAM_->Shutdown();

    // Save camera trajectory
    SLAM_->SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
}

/*void StereoInertialNode::GrabImu(const ImuMsg::SharedPtr msg)
{
    bufMutex_.lock();
    imuBuf_.push(msg);
    bufMutex_.unlock();
}*/ // Замена

void StereoInertialNode::GrabImu(const ImuMsg::SharedPtr msg) // Вставка
{
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

    std::lock_guard<std::mutex> lock(bufMutex_);

    static double last_imu_timestamp = -1.0;
    if (last_imu_timestamp >= 0.0 && t <= last_imu_timestamp)
    {
        RCLCPP_WARN(this->get_logger(), "Discarding non-increasing IMU timestamp: %.9f", t);
        return;
    }
    last_imu_timestamp = t;

    imuBuf_.push(msg);

    constexpr std::size_t kMaxImu = 500;
    while (imuBuf_.size() > kMaxImu)
    {
        imuBuf_.pop();
        ++droppedCount_;
    }
} //

void StereoInertialNode::GrabImageLeft(const ImageMsg::SharedPtr msgLeft)
{
    bufMutexLeft_.lock();

    /*if (!imgLeftBuf_.empty())
        imgLeftBuf_.pop();
    imgLeftBuf_.push(msgLeft);*/ // Замена
    
    
    imgLeftBuf_.push(msgLeft);          // сначала push: очередь никогда не бывает пустой
    while (imgLeftBuf_.size() > 2)      // держим максимум 2 кадра: старый + новый
        imgLeftBuf_.pop();  // Вставка

    bufMutexLeft_.unlock();
}

void StereoInertialNode::GrabImageRight(const ImageMsg::SharedPtr msgRight)
{
    bufMutexRight_.lock();

    /*if (!imgRightBuf_.empty())
        imgRightBuf_.pop();
    imgRightBuf_.push(msgRight);*/ // Замена
    
    
    imgRightBuf_.push(msgRight);
    while (imgRightBuf_.size() > 2)
        imgRightBuf_.pop(); // Вставка

    bufMutexRight_.unlock();
}

cv::Mat StereoInertialNode::GetImage(const ImageMsg::SharedPtr msg)
{
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptr;

    try
    {
        cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::MONO8);
    }
    catch (cv_bridge::Exception &e)
    {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    }
    // return cv::Mat(); // Вставка
    if (cv_ptr->image.type() == 0)
    {
        return cv_ptr->image.clone();
    }
    else
    {
        std::cerr << "Error image type" << std::endl;
        return cv_ptr->image.clone();
    }
}

/*void StereoInertialNode::SyncWithImu()
{
    // const double maxTimeDiff = 0.01; // Замена
    const double maxTimeDiff = 0.025; // Вставка
    static unsigned long long mismatchCount = 0; // Вставка
    static unsigned long long droppedCount = 0;  
    static unsigned long long processedCount = 0; 
    static auto lastLogT = std::chrono::steady_clock::now(); //
    while (1)
    {
        cv::Mat imLeft, imRight;
        double tImLeft = 0, tImRight = 0;
        /* if (!imgLeftBuf_.empty() && !imgRightBuf_.empty() && !imuBuf_.empty())
        {
            tImLeft = Utility::StampToSec(imgLeftBuf_.front()->header.stamp);
            tImRight = Utility::StampToSec(imgRightBuf_.front()->header.stamp);

            bufMutexRight_.lock();
            while ((tImLeft - tImRight) > maxTimeDiff && imgRightBuf_.size() > 1)
            {
                imgRightBuf_.pop();
                tImRight = Utility::StampToSec(imgRightBuf_.front()->header.stamp);
            }
            bufMutexRight_.unlock();

            bufMutexLeft_.lock();
            while ((tImRight - tImLeft) > maxTimeDiff && imgLeftBuf_.size() > 1)
            {
                imgLeftBuf_.pop();
                tImLeft = Utility::StampToSec(imgLeftBuf_.front()->header.stamp);
            }
            bufMutexLeft_.unlock();

            if ((tImLeft - tImRight) > maxTimeDiff || (tImRight - tImLeft) > maxTimeDiff)
            {
                std::cout << "big time difference" << std::endl;
                continue;
            }
            if (tImLeft > Utility::StampToSec(imuBuf_.back()->header.stamp))
                continue;
            */ // Замена
              
        /* bool bReady = false; //
          {
              std::lock_guard<std::mutex> lockLeft(bufMutexLeft_);
              std::lock_guard<std::mutex> lockRight(bufMutexRight_);
              std::lock_guard<std::mutex> lockImu(bufMutex_);

              if (!imgLeftBuf_.empty() && !imgRightBuf_.empty() && !imuBuf_.empty())
              {
                  tImLeft = Utility::StampToSec(imgLeftBuf_.front()->header.stamp);
                  tImRight = Utility::StampToSec(imgRightBuf_.front()->header.stamp);
                  /*
                  while ((tImLeft - tImRight) > maxTimeDiff && imgRightBuf_.size() > 1)
                  {
                      imgRightBuf_.pop();
                      tImRight = Utility::StampToSec(imgRightBuf_.front()->header.stamp);
                  }
                  while ((tImRight - tImLeft) > maxTimeDiff && imgLeftBuf_.size() > 1)
                  {
                      imgLeftBuf_.pop();
                      tImLeft = Utility::StampToSec(imgLeftBuf_.front()->header.stamp);
                  }
                  */ // Замена
                  
                  /* while ((tImLeft - tImRight) > maxTimeDiff && imgRightBuf_.size() > 1) // Вставка
                  {
                      imgRightBuf_.pop();
                      ++droppedCount;
                      tImRight = Utility::StampToSec(imgRightBuf_.front()->header.stamp);
                  }
                  while ((tImRight - tImLeft) > maxTimeDiff && imgLeftBuf_.size() > 1)
                  {
                      imgLeftBuf_.pop();
                      ++droppedCount;
                      tImLeft = Utility::StampToSec(imgLeftBuf_.front()->header.stamp);
                  }

                  //
                  if ((tImLeft - tImRight) > maxTimeDiff || (tImRight - tImLeft) > maxTimeDiff)
                  {
                      ++mismatchCount;
                      if (mismatchCount % 5000 == 1) // Замена 1000 на 5000
                          std::cout << "big time difference: dt=" << (tImLeft - tImRight)
                                    << " s, count=" << mismatchCount << std::endl;
                  }
                  else if (tImLeft <= Utility::StampToSec(imuBuf_.back()->header.stamp))
                  {
                      bReady = true;
                  }
              }
          }

          if (!bReady) // Вставка
          {
              std::this_thread::sleep_for(std::chrono::milliseconds(1)); // не крутим CPU вхолостую
          } //

          if (bReady)
          { // Вставка
            /*
            bufMutexLeft_.lock();
            imLeft = GetImage(imgLeftBuf_.front());
            imgLeftBuf_.pop();
            bufMutexLeft_.unlock(); */ // Замена
            
            /* bufMutexLeft_.lock(); // Вставка
            const builtin_interfaces::msg::Time stampImg = imgLeftBuf_.front()->header.stamp;
            imLeft = GetImage(imgLeftBuf_.front());
            imgLeftBuf_.pop();
            bufMutexLeft_.unlock(); //

            bufMutexRight_.lock();
            imRight = GetImage(imgRightBuf_.front());
            imgRightBuf_.pop();
            bufMutexRight_.unlock();

            vector<ORB_SLAM3::IMU::Point> vImuMeas;
            bufMutex_.lock();
            if (!imuBuf_.empty())
            {
                // Load imu measurements from buffer
                vImuMeas.clear();
                while (!imuBuf_.empty() && Utility::StampToSec(imuBuf_.front()->header.stamp) <= tImLeft)
                {
                    double t = Utility::StampToSec(imuBuf_.front()->header.stamp);
                    cv::Point3f acc(imuBuf_.front()->linear_acceleration.x, imuBuf_.front()->linear_acceleration.y, imuBuf_.front()->linear_acceleration.z);
                    cv::Point3f gyr(imuBuf_.front()->angular_velocity.x, imuBuf_.front()->angular_velocity.y, imuBuf_.front()->angular_velocity.z);
                    vImuMeas.push_back(ORB_SLAM3::IMU::Point(acc, gyr, t));
                    imuBuf_.pop();
                }
            }
            bufMutex_.unlock();

            if (bClahe_)
            {
                clahe_->apply(imLeft, imLeft);
                clahe_->apply(imRight, imRight);
            }

            if (doRectify_)
            {
                cv::remap(imLeft, imLeft, M1l_, M2l_, cv::INTER_LINEAR);
                cv::remap(imRight, imRight, M1r_, M2r_, cv::INTER_LINEAR);
            }

            Sophus::SE3f Tcw = SLAM_->TrackStereo(imLeft, imRight, tImLeft, vImuMeas);

            // std::cout << "Tcw = \n" << Tcw.matrix() << std::endl;

            // Print translation
            // Eigen::Vector3f t = Tcw.translation();
            // std::cout << "Pose -> x: " << t.x()
            //         << " y: " << t.y()
            //         << " z: " << t.z() << std::endl;

            // // Print quaternion
            // Eigen::Quaternionf q(Tcw.rotationMatrix());
            // std::cout << "Quat -> x: " << q.x()
            //         << " y: " << q.y()
            //         << " z: " << q.z()
            //         << " w: " << q.w() << std::endl;

            if (Tcw.matrix().isZero(0)) {
                RCLCPP_WARN(this->get_logger(), "Invalid pose from ORB-SLAM3");
                // return; // Замена
                continue; // Вставка
            }

            // 2. Invert to get camera-in-world
            Sophus::SE3f Twc = Tcw.inverse();

            // --- Compute delta and accumulate ---
            if (!has_prev_pose_) {
                prev_pose_ = Twc;
                has_prev_pose_ = true;
            }

            // Delta between frames
            Sophus::SE3f delta = prev_pose_.inverse() * Twc;
            accumulated_pose_ = accumulated_pose_ * delta;
            prev_pose_ = Twc;

            // 3. Extract translation & rotation
            Eigen::Vector3f t_slam = accumulated_pose_.translation();
            Eigen::Matrix3f R_slam = accumulated_pose_.rotationMatrix();

            // 4. Axis conversion SLAM → ROS
            // ORB-SLAM3: X=right, Y=down, Z=forward
            // ROS REP-103: X=forward, Y=left, Z=up
            Eigen::Matrix3f R_slam_to_ros;
            R_slam_to_ros << 0,  0, 1,
                        -1,  0, 0,
                        0,  -1, 0;

            Eigen::Vector3f t_ros = R_slam_to_ros * t_slam;
            Eigen::Matrix3f R_ros = R_slam_to_ros * R_slam * R_slam_to_ros.transpose();
            

            // 5. Convert to quaternion
            Eigen::Quaternionf q_ros(R_ros);
            q_ros.normalize();

            // 6. Publish Odometry
            nav_msgs::msg::Odometry odom_msg;
            // odom_msg.header.stamp = this->get_clock()->now(); // Замена
            odom_msg.header.stamp = stampImg; // Вставка
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

            // 7. Publish TF
            geometry_msgs::msg::TransformStamped tf_msg;
            tf_msg.header = odom_msg.header;
            tf_msg.child_frame_id = odom_msg.child_frame_id;

            tf_msg.transform.translation.x = t_ros.x();
            tf_msg.transform.translation.y = t_ros.y();
            tf_msg.transform.translation.z = t_ros.z();

            tf_msg.transform.rotation.x = q_ros.x();
            tf_msg.transform.rotation.y = q_ros.y();
            tf_msg.transform.rotation.z = q_ros.z();
            tf_msg.transform.rotation.w = q_ros.w();

            tf_broadcaster_->sendTransform(tf_msg);

            ++processedCount;                          // Вставка
            {
                const auto nowT = std::chrono::steady_clock::now();
                if (std::chrono::duration_cast<std::chrono::seconds>(nowT - lastLogT).count() >= 5)
                {
                    lastLogT = nowT;
                    std::cout << "[sync] processed=" << processedCount
                              << " dropped=" << droppedCount
                              << " mismatch=" << mismatchCount << std::endl;
                }
            } //

            std::chrono::milliseconds tSleep(1);
            std::this_thread::sleep_for(tSleep);
        }
    }
}*/

void StereoInertialNode::SyncWithImu()
{
    const double maxTimeDiff = 0.01; // допуск согласования L/R (было 0.025)

    while (running_ && rclcpp::ok())
    {
        ImageMsg::SharedPtr leftMsg, rightMsg;
        double tImLeft = 0, tImRight = 0;
        std::vector<ORB_SLAM3::IMU::Point> vImuMeas;
        bool bReady = false;
        bool skipPair = false;

        // Фаза А: проверка И снятие сообщений атомарно (все три лока)
        {
            std::lock_guard<std::mutex> lockLeft(bufMutexLeft_);
            std::lock_guard<std::mutex> lockRight(bufMutexRight_);
            std::lock_guard<std::mutex> lockImu(bufMutex_);

            if (!imgLeftBuf_.empty() && !imgRightBuf_.empty() && !imuBuf_.empty())
            {
                tImLeft = Utility::StampToSec(imgLeftBuf_.front()->header.stamp);
                tImRight = Utility::StampToSec(imgRightBuf_.front()->header.stamp);

                // выравнивание пары по меткам времени
                while ((tImLeft - tImRight) > maxTimeDiff && imgRightBuf_.size() > 1)
                {
                    imgRightBuf_.pop();
                    ++droppedCount_;
                    tImRight = Utility::StampToSec(imgRightBuf_.front()->header.stamp);
                }
                while ((tImRight - tImLeft) > maxTimeDiff && imgLeftBuf_.size() > 1)
                {
                    imgLeftBuf_.pop();
                    ++droppedCount_;
                    tImLeft = Utility::StampToSec(imgLeftBuf_.front()->header.stamp);
                }

                if (std::fabs(tImLeft - tImRight) > maxTimeDiff)
                {
                    // не согласовалось: выбрасываем более старый кадр, чтобы не блокировать конвейер
                    ++mismatchCount_;
                    if (tImLeft < tImRight) { imgLeftBuf_.pop(); }
                    else                    { imgRightBuf_.pop(); }
                    ++droppedCount_;
                }
                else if (tImLeft > Utility::StampToSec(imuBuf_.back()->header.stamp))
                {
                    // IMU ещё не дотянул до кадра — ждём (bReady остаётся false)
                }
                else if (Utility::StampToSec(imuBuf_.front()->header.stamp) > tImLeft)
                {
                    // IMU начинается позже кадра — кадр нерабочий, выбрасываем согласованную пару
                    imgLeftBuf_.pop();
                    imgRightBuf_.pop();
                    ++droppedCount_;
                    skipPair = true;
                }
                else
                {
                    // IMU-пакет: все отсчёты <= tImLeft + ПЕРВЫЙ отсчёт >= tImLeft (правая граница)
                    auto pending = imuBuf_;
                    std::vector<ORB_SLAM3::IMU::Point> candidate;
                    std::size_t toConsume = 0;
                    bool hasLeft = false, hasRight = false;

                    while (!pending.empty())
                    {
                        const auto &m = pending.front();
                        const double t = Utility::StampToSec(m->header.stamp);
                        candidate.emplace_back(
                            cv::Point3f(m->linear_acceleration.x, m->linear_acceleration.y, m->linear_acceleration.z),
                            cv::Point3f(m->angular_velocity.x, m->angular_velocity.y, m->angular_velocity.z),
                            t);
                        ++toConsume;
                        hasLeft = hasLeft || (t <= tImLeft);
                        hasRight = hasRight || (t >= tImLeft);
                        if (t >= tImLeft)
                            break;
                        pending.pop();
                    }

                    if (hasLeft && hasRight && candidate.size() >= 2)
                    {
                        vImuMeas = std::move(candidate);
                        for (std::size_t i = 0; i < toConsume; ++i)
                            imuBuf_.pop();

                        // коммит: снимаем именно ту пару кадров, по которой всё проверили
                        leftMsg = imgLeftBuf_.front();   imgLeftBuf_.pop();
                        rightMsg = imgRightBuf_.front(); imgRightBuf_.pop();
                        bReady = true;
                    }
                    // пакет неполный — ничего не трогаем, ждём новые отсчёты IMU
                }
            }
        }

        if (skipPair)
        {
            RCLCPP_WARN(this->get_logger(), "Skipping image %.6f: IMU begins after this image", tImLeft);
            continue;
        }

        if (!bReady)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            continue;
        }

        // --- работа с изображениями вне локов ---
        cv::Mat imLeft = GetImage(leftMsg);
        cv::Mat imRight = GetImage(rightMsg);
        if (imLeft.empty() || imRight.empty())
        {
            ++droppedCount_;
            continue;
        }

        if (bClahe_)
        {
            clahe_->apply(imLeft, imLeft);
            clahe_->apply(imRight, imRight);
        }

        if (doRectify_)
        {
            cv::remap(imLeft, imLeft, M1l_, M2l_, cv::INTER_LINEAR);
            cv::remap(imRight, imRight, M1r_, M2r_, cv::INTER_LINEAR);
        }

        Sophus::SE3f Tcw = SLAM_->TrackStereo(imLeft, imRight, tImLeft, vImuMeas);

        // ворота состояния трекинга (как в mono) — не склеиваем позы из другого карт-атласа
        const int tracking_state = SLAM_->GetTrackingState();
        if (tracking_state != ORB_SLAM3::Tracking::OK ||
            !Tcw.matrix().allFinite() ||
            Tcw.matrix().isZero(0))
        {
            if (prev_tracking_ok_)
                RCLCPP_WARN(this->get_logger(), "Tracking lost/invalid, state=%d, image=%.6f",
                            tracking_state, tImLeft);
            prev_tracking_ok_ = false;
            continue;
        }

        const Sophus::SE3f Twc = Tcw.inverse();

        if (!has_prev_pose_)
        {
            prev_pose_ = Twc;
            accumulated_pose_ = Sophus::SE3f(); // сброс накопления на первом кадре
            has_prev_pose_ = true;
        }
        else if (prev_tracking_ok_)
        {
            accumulated_pose_ = accumulated_pose_ * (prev_pose_.inverse() * Twc);
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "Tracking recovered at image %.6f; new odom segment", tImLeft);
        }

        prev_pose_ = Twc;
        prev_tracking_ok_ = true;

        // накопленная поза -> ROS
        const Eigen::Vector3f t_slam = accumulated_pose_.translation();
        const Eigen::Matrix3f R_slam = accumulated_pose_.rotationMatrix();

        Eigen::Matrix3f R_slam_to_ros;
        R_slam_to_ros << 0, 0, 1,
                        -1, 0, 0,
                         0, -1, 0;

        const Eigen::Vector3f t_ros = R_slam_to_ros * t_slam;
        const Eigen::Matrix3f R_ros = R_slam_to_ros * R_slam * R_slam_to_ros.transpose();
        Eigen::Quaternionf q_ros(R_ros);
        q_ros.normalize();

        nav_msgs::msg::Odometry odom_msg;
        odom_msg.header.stamp = leftMsg->header.stamp;
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

        ++processedCount_;
        {
            const auto nowT = std::chrono::steady_clock::now();
            if (std::chrono::duration_cast<std::chrono::seconds>(nowT - lastLogT_).count() >= 5)
            {
                lastLogT_ = nowT;
                std::cout << "[sync] processed=" << processedCount_
                          << " dropped=" << droppedCount_
                          << " mismatch=" << mismatchCount_ << std::endl;
            }
        }
    }
}
