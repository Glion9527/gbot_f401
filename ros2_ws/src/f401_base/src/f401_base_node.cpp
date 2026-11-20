#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <functional>
#include <iomanip>
#include <memory>
#include <numbers>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "f401/drivers/serial_base.hpp"
#include "robot_geometry.h"

namespace {

using namespace std::chrono_literals;
using f401::core::ChassisSpeeds;
using f401::core::WheelSpeeds;
using f401::drivers::SerialBase;

geometry_msgs::msg::Quaternion quaternionFromYaw(double yaw)
{
    geometry_msgs::msg::Quaternion q;
    q.x = 0.0;
    q.y = 0.0;
    q.z = std::sin(yaw * 0.5);
    q.w = std::cos(yaw * 0.5);
    return q;
}

std::string jsonEscape(const std::string& text)
{
    std::string out;
    out.reserve(text.size() + 8U);
    for (const char ch : text) {
        switch (ch) {
            case '"': out += "\\\""; break;
            case '\\': out += "\\\\"; break;
            case '\n': out += "\\n"; break;
            case '\r': out += "\\r"; break;
            case '\t': out += "\\t"; break;
            default: out += ch; break;
        }
    }
    return out;
}

class F401BaseNode final : public rclcpp::Node {
public:
    F401BaseNode()
        : Node("f401_base"), tf_broadcaster_(*this), static_tf_broadcaster_(*this)
    {
        port_ = declare_parameter<std::string>("port", "/dev/f401");
        baud_ = declare_parameter<int>("baud", 115200);
        cmd_timeout_s_ = declare_parameter<double>("cmd_timeout", 0.30);
        cpr_ = declare_parameter<std::vector<double>>(
            "cpr", std::vector<double>{3960.0, 3960.0, 3960.0, 3960.0});
        odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
        base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
        imu_frame_ = declare_parameter<std::string>("imu_frame", "imu_link");
        (void)declare_parameter<double>("imu_x", 0.0);
        (void)declare_parameter<double>("imu_y", 0.0);
        (void)declare_parameter<double>("imu_z", 0.0);
        (void)declare_parameter<double>("imu_roll", 0.0);
        (void)declare_parameter<double>("imu_pitch", 0.0);
        (void)declare_parameter<double>("imu_yaw", 0.0);

        if (cpr_.size() != LINK_V2_WHEEL_COUNT) {
            RCLCPP_ERROR(get_logger(), "cpr 需要 4 个值，使用默认 3960");
            cpr_ = {3960.0, 3960.0, 3960.0, 3960.0};
        }
        for (const double value : cpr_) {
            if (!std::isfinite(value) || value <= 0.0) {
                RCLCPP_ERROR(get_logger(), "cpr 必须为正有限数，使用默认 3960");
                cpr_ = {3960.0, 3960.0, 3960.0, 3960.0};
                break;
            }
        }
        cmd_timeout_s_ = std::max(0.05, cmd_timeout_s_);

        cmd_sub_ = create_subscription<geometry_msgs::msg::TwistStamped>(
            "/cmd_vel", rclcpp::QoS(1),
            std::bind(&F401BaseNode::onCmd, this, std::placeholders::_1));
        odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
        joint_pub_ = create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);
        imu_pub_ = create_publisher<sensor_msgs::msg::Imu>("/imu/data_raw", 10);
        mag_pub_ = create_publisher<sensor_msgs::msg::MagneticField>("/imu/mag", 10);
        state_pub_ = create_publisher<std_msgs::msg::String>("/robot/state", 10);

        arm_srv_ = create_service<std_srvs::srv::Trigger>(
            "/robot/arm",
            std::bind(&F401BaseNode::armService, this, std::placeholders::_1,
                      std::placeholders::_2));
        disarm_srv_ = create_service<std_srvs::srv::Trigger>(
            "/robot/disarm",
            std::bind(&F401BaseNode::disarmService, this, std::placeholders::_1,
                      std::placeholders::_2));
        connect_srv_ = create_service<std_srvs::srv::Trigger>(
            "/robot/connect",
            std::bind(&F401BaseNode::connectService, this, std::placeholders::_1,
                      std::placeholders::_2));

        timer_ = create_wall_timer(20ms, std::bind(&F401BaseNode::tick, this));
        openSerial();
        publishStaticImuTransform();
        /* 每秒按当前参数重发一次静态 TF，便于用 ros2 param set 现场调安装角。 */
        imu_tf_timer_ = create_wall_timer(
            1s, std::bind(&F401BaseNode::publishStaticImuTransform, this));
    }

    ~F401BaseNode() override
    {
        if (base_) {
            base_->setArmed(false);
            base_->setVelocity({});
        }
    }

private:
    bool openSerial()
    {
        try {
            base_ = std::make_unique<SerialBase>(port_, baud_);
        } catch (const std::exception& error) {
            base_.reset();
            last_serial_error_ = error.what();
            RCLCPP_ERROR(get_logger(), "打开 %s 失败: %s", port_.c_str(), error.what());
            return false;
        }

        last_serial_error_.clear();
        last_logged_error_.clear();
        target_ = ChassisSpeeds{};
        have_cmd_ = false;
        if (!base_->syncSequence(1000ms)) {
            RCLCPP_WARN(get_logger(), "暂未收到 TELEM，seq 同步失败；继续运行等待遥测");
        } else {
            RCLCPP_INFO(get_logger(), "串口 %s 已打开，seq 已同步", port_.c_str());
        }
        return true;
    }

    void onCmd(const geometry_msgs::msg::TwistStamped::SharedPtr msg)
    {
        if (!msg) {
            return;
        }
        const auto& twist = msg->twist;
        if (!std::isfinite(twist.linear.x) || !std::isfinite(twist.linear.y) ||
            !std::isfinite(twist.angular.z)) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "忽略非有限速度");
            return;
        }
        if (!msg->header.frame_id.empty() && msg->header.frame_id != base_frame_) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                                 "忽略 frame_id=%s 的速度", msg->header.frame_id.c_str());
            return;
        }

        const rclcpp::Time stamp(msg->header.stamp, get_clock()->get_clock_type());
        const double age = (now() - stamp).seconds();
        if (!std::isfinite(age) || age < -0.1 || age > 0.5) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                                 "忽略过期/未来速度，age=%.3f", age);
            return;
        }

        target_.vx_mps = twist.linear.x;
        target_.vy_mps = twist.linear.y;
        target_.wz_radps = twist.angular.z;
        last_cmd_ = std::chrono::steady_clock::now();
        have_cmd_ = true;
    }

    void tick()
    {
        if (!base_) {
            publishState();
            return;
        }

        const auto now_steady = std::chrono::steady_clock::now();
        if (!have_cmd_ ||
            now_steady - last_cmd_ > std::chrono::duration<double>(cmd_timeout_s_)) {
            target_ = ChassisSpeeds{};
        }

        if (!base_->setVelocity(target_)) {
            handleLinkError();
            return;
        }
        base_->poll();
        if (!base_->lastError().empty()) {
            handleLinkError();
            return;
        }

        publishOdometry();
        publishJoints();
        publishImu();
        publishState();
    }

    void handleLinkError()
    {
        const std::string error = base_ ? base_->lastError() : last_serial_error_;
        if (error != last_logged_error_) {
            RCLCPP_ERROR(get_logger(), "链路错误: %s", error.c_str());
            last_logged_error_ = error;
        }
        if (base_) {
            base_->setArmed(false);
        }
        target_ = ChassisSpeeds{};
        have_cmd_ = false;
        publishState();
    }

    void publishOdometry()
    {
        const auto odometry = base_ ? base_->getOdometry() : std::nullopt;
        if (!odometry) {
            return;
        }

        nav_msgs::msg::Odometry msg;
        msg.header.stamp = now();
        msg.header.frame_id = odom_frame_;
        msg.child_frame_id = base_frame_;
        msg.pose.pose.position.x = odometry->pose.x_m;
        msg.pose.pose.position.y = odometry->pose.y_m;
        msg.pose.pose.position.z = 0.0;
        msg.pose.pose.orientation = quaternionFromYaw(odometry->pose.yaw_rad);
        msg.pose.covariance.fill(0.0);
        msg.pose.covariance[0] = 0.01;
        msg.pose.covariance[7] = 0.01;
        msg.pose.covariance[35] = 0.05;

        if (const auto wheel = base_->getWheelState()) {
            const double radius = F401_WHEEL_DIAMETER_MM / 2000.0;
            const double rpm_to_mps = 2.0 * std::numbers::pi * radius / 60.0 *
                                      SerialBase::kOdomWheelScale;
            const WheelSpeeds speeds{
                wheel->rpm[0] * rpm_to_mps, wheel->rpm[1] * rpm_to_mps,
                wheel->rpm[2] * rpm_to_mps, wheel->rpm[3] * rpm_to_mps};
            const ChassisSpeeds chassis = f401::core::toChassisSpeeds(speeds);
            msg.twist.twist.linear.x = chassis.vx_mps;
            msg.twist.twist.linear.y = chassis.vy_mps;
            msg.twist.twist.angular.z = chassis.wz_radps;
        }
        msg.twist.covariance.fill(0.0);
        msg.twist.covariance[0] = 0.01;
        msg.twist.covariance[7] = 0.01;
        msg.twist.covariance[35] = 0.05;

        odom_pub_->publish(msg);

        geometry_msgs::msg::TransformStamped tf;
        tf.header = msg.header;
        tf.child_frame_id = base_frame_;
        tf.transform.translation.x = msg.pose.pose.position.x;
        tf.transform.translation.y = msg.pose.pose.position.y;
        tf.transform.translation.z = 0.0;
        tf.transform.rotation = msg.pose.pose.orientation;
        tf_broadcaster_.sendTransform(tf);
    }

    void publishJoints()
    {
        if (!base_) {
            return;
        }
        const auto wheel = base_->getWheelState();
        if (!wheel) {
            return;
        }

        sensor_msgs::msg::JointState msg;
        msg.header.stamp = now();
        msg.name = {"front_left_wheel_joint", "front_right_wheel_joint",
                    "rear_left_wheel_joint", "rear_right_wheel_joint"};
        msg.position.resize(LINK_V2_WHEEL_COUNT);
        msg.velocity.resize(LINK_V2_WHEEL_COUNT);
        for (std::size_t index = 0; index < LINK_V2_WHEEL_COUNT; ++index) {
            msg.position[index] = static_cast<double>(wheel->counts[index]) / cpr_[index] *
                                  2.0 * std::numbers::pi;
            msg.velocity[index] =
                wheel->rpm[index] * 2.0 * std::numbers::pi / 60.0;
        }
        joint_pub_->publish(msg);
    }

    void publishStaticImuTransform()
    {
        const double x = get_parameter("imu_x").as_double();
        const double y = get_parameter("imu_y").as_double();
        const double z = get_parameter("imu_z").as_double();
        const double roll = get_parameter("imu_roll").as_double();
        const double pitch = get_parameter("imu_pitch").as_double();
        const double yaw = get_parameter("imu_yaw").as_double();

        const double cr = std::cos(roll * 0.5);
        const double sr = std::sin(roll * 0.5);
        const double cp = std::cos(pitch * 0.5);
        const double sp = std::sin(pitch * 0.5);
        const double cy = std::cos(yaw * 0.5);
        const double sy = std::sin(yaw * 0.5);

        geometry_msgs::msg::TransformStamped tf;
        /* 静态 TF 用时间 0；用 now() 会让 tf2_monitor 把启动时刻当前
         * 的时间差算成 imu_link 的巨大延迟。 */
        tf.header.stamp = rclcpp::Time(0, 0, get_clock()->get_clock_type());
        tf.header.frame_id = base_frame_;
        tf.child_frame_id = imu_frame_;
        tf.transform.translation.x = x;
        tf.transform.translation.y = y;
        tf.transform.translation.z = z;
        tf.transform.rotation.x = sr * cp * cy - cr * sp * sy;
        tf.transform.rotation.y = cr * sp * cy + sr * cp * sy;
        tf.transform.rotation.z = cr * cp * sy - sr * sp * cy;
        tf.transform.rotation.w = cr * cp * cy + sr * sp * sy;
        static_tf_broadcaster_.sendTransform(tf);
    }

    void publishImu()
    {
        if (!base_) {
            return;
        }
        const auto imu = base_->getImu();
        if (!imu || (imu->flags & LINK_V2_IMU_VALID) == 0U) {
            return;
        }

        const rclcpp::Time stamp = now();
        constexpr double gravity = 9.80665;
        constexpr double deg_to_rad = std::numbers::pi / 180.0;

        sensor_msgs::msg::Imu msg;
        msg.header.stamp = stamp;
        msg.header.frame_id = imu_frame_;
        msg.linear_acceleration.x = static_cast<double>(imu->acc_mg[0]) * 0.001 * gravity;
        msg.linear_acceleration.y = static_cast<double>(imu->acc_mg[1]) * 0.001 * gravity;
        msg.linear_acceleration.z = static_cast<double>(imu->acc_mg[2]) * 0.001 * gravity;
        msg.angular_velocity.x = static_cast<double>(imu->gyro_dps10[0]) * 0.1 * deg_to_rad;
        msg.angular_velocity.y = static_cast<double>(imu->gyro_dps10[1]) * 0.1 * deg_to_rad;
        msg.angular_velocity.z = static_cast<double>(imu->gyro_dps10[2]) * 0.1 * deg_to_rad;

        /* 当前只发布原始数据，不在 MCU/host 提供姿态。 */
        msg.orientation.w = 1.0;
        msg.orientation_covariance.fill(0.0);
        msg.orientation_covariance[0] = -1.0;

        msg.linear_acceleration_covariance.fill(0.0);
        msg.linear_acceleration_covariance[0] = 0.01;
        msg.linear_acceleration_covariance[4] = 0.01;
        msg.linear_acceleration_covariance[8] = 0.01;
        msg.angular_velocity_covariance.fill(0.0);
        msg.angular_velocity_covariance[0] = 1e-4;
        msg.angular_velocity_covariance[4] = 1e-4;
        msg.angular_velocity_covariance[8] = 1e-4;
        imu_pub_->publish(msg);

        if ((imu->flags & LINK_V2_IMU_MAG_OK) == 0U) {
            return;
        }
        sensor_msgs::msg::MagneticField mag;
        mag.header.stamp = stamp;
        mag.header.frame_id = imu_frame_;
        mag.magnetic_field.x = static_cast<double>(imu->mag_ut10[0]) * 0.1 * 1e-6;
        mag.magnetic_field.y = static_cast<double>(imu->mag_ut10[1]) * 0.1 * 1e-6;
        mag.magnetic_field.z = static_cast<double>(imu->mag_ut10[2]) * 0.1 * 1e-6;
        mag.magnetic_field_covariance.fill(0.0);
        mag.magnetic_field_covariance[0] = 1e-8;
        mag.magnetic_field_covariance[4] = 1e-8;
        mag.magnetic_field_covariance[8] = 1e-8;
        mag_pub_->publish(mag);
    }

    void publishState()
    {
        const auto odometry = base_ ? base_->getOdometry() : std::nullopt;
        const auto imu = base_ ? base_->getImu() : std::nullopt;
        const bool connected = base_ && base_->lastError().empty();
        const std::uint16_t status = odometry ? odometry->status : 0U;
        const std::uint16_t fault_mask = odometry ? odometry->fault_mask : 0U;
        const std::string reason = base_ ? base_->lastError() : last_serial_error_;

        std::ostringstream out;
        out << std::fixed << std::setprecision(3);
        out << "{\"connected\":" << (connected ? "true" : "false")
            << ",\"armed\":" << ((status & LINK_V2_STATUS_ARMED) != 0U ? "true" : "false")
            << ",\"status\":" << status
            << ",\"fault_mask\":" << fault_mask
            << ",\"seq\":" << (odometry ? odometry->seq : 0U)
            << ",\"x\":" << (odometry ? odometry->pose.x_m : 0.0)
            << ",\"y\":" << (odometry ? odometry->pose.y_m : 0.0)
            << ",\"yaw\":" << (odometry ? odometry->pose.yaw_rad : 0.0)
            << ",\"imu_flags\":" << (imu ? imu->flags : 0U)
            << ",\"imu_temp_c\":" << (imu ? static_cast<double>(imu->temp_c10) / 10.0 : 0.0)
            << ",\"reason\":\"" << jsonEscape(reason) << "\"}";

        std_msgs::msg::String msg;
        msg.data = out.str();
        state_pub_->publish(msg);
    }

    void armService(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        (void)request;
        response->success = false;
        if (!base_) {
            response->message = "串口未打开";
            return;
        }
        if (!base_->lastError().empty()) {
            response->message = "链路错误: " + base_->lastError();
            return;
        }

        if (!base_->syncSequence(500ms)) {
            response->message = "未收到 TELEM，无法同步 seq";
            return;
        }

        base_->setArmed(true);
        const auto deadline = std::chrono::steady_clock::now() + 800ms;
        bool armed = false;
        while (rclcpp::ok() && std::chrono::steady_clock::now() < deadline) {
            if (!base_->setVelocity({})) {
                break;
            }
            base_->poll();
            if (const auto odometry = base_->getOdometry()) {
                if ((odometry->status & LINK_V2_STATUS_ARMED) != 0U) {
                    armed = true;
                    break;
                }
            }
            std::this_thread::sleep_for(20ms);
        }

        if (armed) {
            response->success = true;
            response->message = "已使能";
            return;
        }

        base_->setArmed(false);
        (void)base_->setVelocity({});
        response->message = "MCU 拒绝 ARM（检查四轮静止/故障/USER 键）";
    }

    void disarmService(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                       std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        (void)request;
        response->success = false;
        if (!base_) {
            response->message = "串口未打开";
            return;
        }
        base_->setArmed(false);
        const bool sent = base_->setVelocity({});
        response->success = sent;
        response->message = sent ? "已解除使能" : "发送停止失败";
        target_ = ChassisSpeeds{};
        have_cmd_ = false;
    }

    void connectService(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        (void)request;
        if (base_) {
            base_->setArmed(false);
            (void)base_->setVelocity({});
        }
        base_.reset();
        const bool opened = openSerial();
        response->success = opened;
        response->message = opened ? "串口已打开" : "打开串口失败";
    }

    std::string port_;
    int baud_ = 115200;
    double cmd_timeout_s_ = 0.30;
    std::vector<double> cpr_;
    std::string odom_frame_;
    std::string base_frame_;
    std::string imu_frame_;

    std::unique_ptr<SerialBase> base_;
    ChassisSpeeds target_{};
    std::chrono::steady_clock::time_point last_cmd_{};
    bool have_cmd_ = false;
    std::string last_serial_error_;
    std::string last_logged_error_;

    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr mag_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr arm_srv_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr disarm_srv_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr connect_srv_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr imu_tf_timer_;
    tf2_ros::TransformBroadcaster tf_broadcaster_;
    tf2_ros::StaticTransformBroadcaster static_tf_broadcaster_;
};

}  // namespace

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<F401BaseNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
