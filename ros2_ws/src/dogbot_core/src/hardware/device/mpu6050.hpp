#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>

#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <linux/i2c.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <functional>
#include <numbers>
#include <stdexcept>
#include <string>
#include <tuple>

namespace dogbot_core::hardware::device {

// MPU6050 六轴 IMU：通过树莓派 5 内核 I2C 驱动（/dev/i2c-N）读取原始加速度
// 与角速度，经陀螺积分四元数 + 逐轴一维卡尔曼滤波得到欧拉角（弧度制），
// 在 update() 中发布到 <prefix>/pitch、<prefix>/yaw、<prefix>/roll。
class Mpu6050 {
public:
    Mpu6050()                          = default;
    Mpu6050(const Mpu6050&)            = delete;
    Mpu6050& operator=(const Mpu6050&) = delete;

    ~Mpu6050() {
        if (fd_ >= 0) {
            ::close(fd_);
        }
    }

    // 初始化：打开 /dev/i2c-{bus} 并设从机地址，校验 WHO_AM_I，退出休眠；
    // 按前缀创建 pitch/yaw/roll 发布者（Float64，弧度制）
    void init(
        rclcpp::Node* node, const std::string& prefix, int i2c_bus = 1,
        uint8_t i2c_addr = kAddrDefault) {
        node_      = node;
        roll_pub_  = node_->create_publisher<std_msgs::msg::Float64>(prefix + "/roll", 10);
        pitch_pub_ = node_->create_publisher<std_msgs::msg::Float64>(prefix + "/pitch", 10);
        yaw_pub_   = node_->create_publisher<std_msgs::msg::Float64>(prefix + "/yaw", 10);

        const std::string device = "/dev/i2c-" + std::to_string(i2c_bus);
        fd_                      = ::open(device.c_str(), O_RDWR);
        i2c_addr_                = i2c_addr;
        if (fd_ < 0) {
            throw std::runtime_error("open " + device + " failed: " + std::strerror(errno));
        }
        if (ioctl(fd_, I2C_SLAVE, i2c_addr) < 0) {
            const std::string err = std::strerror(errno);
            ::close(fd_);
            fd_ = -1;
            throw std::runtime_error("ioctl I2C_SLAVE on " + device + " failed: " + err);
        }

        uint8_t who = 0;
        read_regs(kRegWhoAmI, &who, 1);
        if (who != kAddrDefault && who != kAddrMpu6500) {
            ::close(fd_);
            fd_ = -1;
            throw std::runtime_error(
                "MPU6050/MPU6500 WHO_AM_I mismatch: " + std::to_string(who) + ", expected "
                + std::to_string(kAddrDefault) + " or " + std::to_string(kAddrMpu6500));
        }
        write_reg(kRegPwrMgmt1, 0x00); // 退出休眠
        last_time_ = std::chrono::steady_clock::now();
    }

    // 设定 IMU 数据到机体坐标系的向量映射：对 (x,y,z) 逐轴变换，
    // 同一映射同时作用于加速度与角速度向量。未调用时按单位映射处理。
    // 例：[](double x, double y, double z) { return std::make_tuple(-x, -y, +z); }
    // 即绕 z 轴旋转 180°。
    void set_coordinate_mapping(
        std::function<std::tuple<double, double, double>(double, double, double)> mapper) {
        coordinate_mapping_ = std::move(mapper);
    }

    // 读取原始数据 → 四元数 → 卡尔曼滤波 → 发布弧度制欧拉角
    void update() {
        RawData raw = get_raw_data();
        if (coordinate_mapping_) {
            apply_coordinate_mapping(raw);
        }
        const Quaternion q = store(raw);
        const Euler e      = kalman_filter(q, raw);
        publish(e);
    }

    // 获取当前欧拉角（弧度制，update() 后刷新）
    double get_roll() const { return roll_kf_.angle; }
    double get_pitch() const { return pitch_kf_.angle; }
    double get_yaw() const { return yaw_kf_.angle; }

private:
    // 传感器寄存器与量程（MPU6050 固有参数）
    static constexpr uint8_t kRegWhoAmI     = 0x75;
    static constexpr uint8_t kRegPwrMgmt1   = 0x6B;
    static constexpr uint8_t kRegAccelXOutH = 0x3B;
    static constexpr uint8_t kAddrDefault   = 0x68;
    // MPU6500 与 MPU6050 寄存器地图兼容，WHO_AM_I 为 0x70
    static constexpr uint8_t kAddrMpu6500 = 0x70;
    static constexpr double kAccelLsbPerG   = 16384.0; // ±2g
    static constexpr double kGyroLsbPerDps  = 131.0;   // ±250dps
    static constexpr double kDeg2Rad        = std::numbers::pi_v<double> / 180.0;

    struct RawData {
        double ax, ay, az;       // 加速度（g）
        double gx, gy, gz;       // 角速度（rad/s）
        double dt;               // 距上次采样时间（s）
    };

    struct Quaternion {
        double w = 1.0, x = 0.0, y = 0.0, z = 0.0;
    };

    struct Euler {
        double roll, pitch, yaw; // 弧度制
    };

    // 一维卡尔曼滤波器状态（角度 + 陀螺零偏）
    struct KalmanState {
        double angle = 0.0;
        double bias  = 0.0;
        double p00 = 1.0, p01 = 0.0, p10 = 0.0, p11 = 1.0;

        void predict(double rate, double dt) {
            angle += dt * (rate - bias);
            p00 += dt * (dt * p11 - p01 - p10 + kQAngle);
            p01 -= dt * p10;
            p10 -= dt * p01;
            p11 += kQBias * dt;
        }

        void update(double measurement) {
            const double s  = p00 + kRMeasure;
            const double k0 = p00 / s;
            const double k1 = p01 / s;
            const double y  = measurement - angle;
            angle += k0 * y;
            bias += k1 * y;
            p00 -= k0 * p00;
            p01 -= k0 * p01;
            p10 -= k1 * p00;
            p11 -= k1 * p01;
        }
    };

    // 合并事务：write+read 在一次 I2C 事务内完成（repeated start，中间无 STOP），
    // 避免两段式事务（写地址 STOP 后再读）导致从机失同步
    void read_regs(uint8_t reg, uint8_t* buf, size_t n) const {
        uint8_t reg_buf = reg;
        struct i2c_msg msgs[2];
        msgs[0].addr  = i2c_addr_;
        msgs[0].flags = 0;
        msgs[0].len   = sizeof(reg_buf);
        msgs[0].buf   = &reg_buf;
        msgs[1].addr  = i2c_addr_;
        msgs[1].flags = I2C_M_RD;
        msgs[1].len   = static_cast<__u16>(n);
        msgs[1].buf   = buf;

        struct i2c_rdwr_ioctl_data data;
        data.msgs  = msgs;
        data.nmsgs = 2;
        if (ioctl(fd_, I2C_RDWR, &data) < 0) {
            throw std::runtime_error("I2C_RDWR read failed: " + std::string(std::strerror(errno)));
        }
    }

    void write_reg(uint8_t reg, uint8_t value) const {
        uint8_t buf[2] = {reg, value};
        struct i2c_msg msg;
        msg.addr  = i2c_addr_;
        msg.flags = 0;
        msg.len   = sizeof(buf);
        msg.buf   = buf;

        struct i2c_rdwr_ioctl_data data;
        data.msgs  = &msg;
        data.nmsgs = 1;
        if (ioctl(fd_, I2C_RDWR, &data) < 0) {
            throw std::runtime_error("I2C_RDWR write failed: " + std::string(std::strerror(errno)));
        }
    }

    static int16_t read_int16(const uint8_t* buf, size_t off) {
        return static_cast<int16_t>((buf[off] << 8) | buf[off + 1]);
    }

    static void normalize(Quaternion& q) {
        const double n = std::sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
        if (n > 1e-12) {
            q.w /= n;
            q.x /= n;
            q.y /= n;
            q.z /= n;
        }
    }

    // 从 6050 获取原始数据：连续读 ACCEL_XOUT_H(0x3B) 起 14 字节
    // （加速度×3 + 温度 + 角速度×3），换算为 g / rad/s，并计算采样间隔
    RawData get_raw_data() {
        uint8_t buf[14];
        read_regs(kRegAccelXOutH, buf, sizeof(buf));

        const auto now  = std::chrono::steady_clock::now();
        const double dt = std::chrono::duration<double>(now - last_time_).count();
        last_time_      = now;

        RawData raw;
        raw.ax = read_int16(buf, 0) / kAccelLsbPerG;
        raw.ay = read_int16(buf, 2) / kAccelLsbPerG;
        raw.az = read_int16(buf, 4) / kAccelLsbPerG;
        raw.gx = read_int16(buf, 8) / kGyroLsbPerDps * kDeg2Rad;
        raw.gy = read_int16(buf, 10) / kGyroLsbPerDps * kDeg2Rad;
        raw.gz = read_int16(buf, 12) / kGyroLsbPerDps * kDeg2Rad;
        raw.dt = std::clamp(dt, 0.0, 0.1); // 首次调用/长时间阻塞时限制积分步长
        return raw;
    }

    // 将原始数据转化为四元数：前向欧拉积分陀螺角速度（dq/dt = ½ q ⊗ ω）后归一化
    Quaternion store(const RawData& raw) {
        const double dq0 = -0.5 * (q_.x * raw.gx + q_.y * raw.gy + q_.z * raw.gz);
        const double dq1 = 0.5 * (q_.w * raw.gx + q_.y * raw.gz - q_.z * raw.gy);
        const double dq2 = 0.5 * (q_.w * raw.gy - q_.x * raw.gz + q_.z * raw.gx);
        const double dq3 = 0.5 * (q_.w * raw.gz + q_.x * raw.gy - q_.y * raw.gx);

        Quaternion q = q_;
        q.w += raw.dt * dq0;
        q.x += raw.dt * dq1;
        q.y += raw.dt * dq2;
        q.z += raw.dt * dq3;
        normalize(q);
        q_ = q;
        return q;
    }

    // 四元数 → 欧拉角（弧度），再逐轴一维卡尔曼滤波：
    // roll/pitch 以加速度计（重力方向参考）为观测，yaw 无绝对参考仅陀螺积分预测
    Euler kalman_filter(const Quaternion& q, const RawData& raw) {
        const double euler_roll =
            std::atan2(2.0 * (q.w * q.x + q.y * q.z), 1.0 - 2.0 * (q.x * q.x + q.y * q.y));
        const double euler_pitch = std::asin(std::clamp(2.0 * (q.w * q.y - q.z * q.x), -1.0, 1.0));
        const double euler_yaw =
            std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));

        const double accel_roll  = std::atan2(raw.ay, raw.az);
        const double accel_pitch = std::atan2(-raw.ax, std::hypot(raw.ay, raw.az));

        if (!filter_started_) {
            // 首个样本以四元数/观测角初始化滤波状态
            roll_kf_.angle  = euler_roll;
            pitch_kf_.angle = euler_pitch;
            yaw_kf_.angle   = euler_yaw;
            filter_started_ = true;
        }

        roll_kf_.predict(raw.gx, raw.dt);
        roll_kf_.update(accel_roll);
        pitch_kf_.predict(raw.gy, raw.dt);
        pitch_kf_.update(accel_pitch);
        yaw_kf_.predict(raw.gz, raw.dt);

        Euler e;
        e.roll  = roll_kf_.angle;
        e.pitch = pitch_kf_.angle;
        e.yaw   = yaw_kf_.angle;
        return e;
    }

    void publish(const Euler& e) {
        std_msgs::msg::Float64 msg;
        msg.data = e.roll;
        roll_pub_->publish(msg);
        msg.data = e.pitch;
        pitch_pub_->publish(msg);
        msg.data = e.yaw;
        yaw_pub_->publish(msg);
    }

    // 将坐标系映射分别应用到加速度与角速度向量
    void apply_coordinate_mapping(RawData& raw) {
        const auto [ax, ay, az] = coordinate_mapping_(raw.ax, raw.ay, raw.az);
        const auto [gx, gy, gz] = coordinate_mapping_(raw.gx, raw.gy, raw.gz);
        raw.ax                  = ax;
        raw.ay                  = ay;
        raw.az                  = az;
        raw.gx                  = gx;
        raw.gy                  = gy;
        raw.gz                  = gz;
    }

    rclcpp::Node* node_ = nullptr;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr roll_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pitch_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr yaw_pub_;

    int fd_                                 = -1;
    uint8_t i2c_addr_                       = kAddrDefault;
    std::chrono::steady_clock::time_point last_time_{};
    Quaternion q_;
    KalmanState roll_kf_, pitch_kf_, yaw_kf_;
    bool filter_started_ = false;
    std::function<std::tuple<double, double, double>(double, double, double)> coordinate_mapping_;

    // 卡尔曼噪声参数（经典 MPU6050 逐轴 Kalman 调参）
    static constexpr double kQAngle   = 0.001;
    static constexpr double kQBias    = 0.003;
    static constexpr double kRMeasure = 0.03;
};
} // namespace dogbot_core::hardware::device
