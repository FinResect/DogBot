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

// IMU660RB（逐飞科技 IMU660RX 系列，传感器芯片 ST LSM6DSRTR）六轴 IMU：
// 通过树莓派 5 内核 I2C 驱动（/dev/i2c-N）读取原始加速度与角速度。
// LSM6DSR 为裸传感器、无片上姿态融合，本驱动经陀螺积分四元数 + 逐轴
// 一维卡尔曼滤波得到欧拉角（弧度制），在 update() 中发布到
// <prefix>/pitch、<prefix>/yaw、<prefix>/roll（Float64）。
// init() 不抛异常：I2C 失败仅记日志，update() 内部按 ≥1s 间隔自动重试
// 打开/配置直至成功，传感器恢复后自动继续发布。对外接口与 Mpu6050 保持一致。
class IMU660RB {
public:
    IMU660RB()                           = default;
    IMU660RB(const IMU660RB&)            = delete;
    IMU660RB& operator=(const IMU660RB&) = delete;

    ~IMU660RB() {
        if (fd_ >= 0) {
            ::close(fd_);
        }
    }

    // 初始化：按前缀创建 pitch/yaw/roll 发布者（弧度制），并尝试打开
    // /dev/i2c-{bus} 配置传感器（软复位 → boot 等待 → WHO_AM_I 轮询 →
    // 配置写入读回校验）。I2C 失败不抛异常：仅记录错误并保持未就绪，
    // 由 update() 内部自动重试（可重复调用，不会重复创建发布者）
    void init(
        rclcpp::Node* node, const std::string& prefix, int i2c_bus = 1,
        uint8_t i2c_addr = kAddrDefault) {
        if (node_ == nullptr) {
            node_      = node;
            roll_pub_  = node_->create_publisher<std_msgs::msg::Float64>(prefix + "/roll", 10);
            pitch_pub_ = node_->create_publisher<std_msgs::msg::Float64>(prefix + "/pitch", 10);
            yaw_pub_   = node_->create_publisher<std_msgs::msg::Float64>(prefix + "/yaw", 10);
        }
        i2c_bus_  = i2c_bus;
        i2c_addr_ = i2c_addr;
        if (!configure_sensor()) {
            RCLCPP_ERROR(
                node_->get_logger(), "IMU660RB init failed, will retry in update(): %s",
                last_error_.c_str());
        }
    }

    // 设定 IMU 数据到机体坐标系的向量映射：对 (x,y,z) 逐轴变换，
    // 同一映射同时作用于加速度与角速度向量。未调用时按单位映射处理。
    // 例：[](double x, double y, double z) { return std::make_tuple(-x, -y, +z); }
    // 即绕 z 轴旋转 180°。
    void set_coordinate_mapping(
        std::function<std::tuple<double, double, double>(double, double, double)> mapper) {
        coordinate_mapping_ = std::move(mapper);
    }

    // 读取原始数据 → 四元数 → 卡尔曼滤波 → 发布弧度制欧拉角。
    // 传感器未就绪时按 ≥1s 间隔自动重试打开/配置，失败仅记日志不抛异常；
    // 就绪后读取失败仍抛出（由调用方处理）
    void update() {
        if (node_ == nullptr) {
            return;
        }
        if (fd_ < 0) {
            const auto now = std::chrono::steady_clock::now();
            if (std::chrono::duration<double>(now - last_init_attempt_).count() < kReinitIntervalS) {
                return;
            }
            last_init_attempt_ = now;
            if (!configure_sensor()) {
                RCLCPP_ERROR_THROTTLE(
                    node_->get_logger(), *node_->get_clock(), 1000,
                    "IMU660RB not ready, retry failed: %s", last_error_.c_str());
                return;
            }
        }
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
    // LSM6DSR 寄存器与量程（逐飞 IMU660RB 模块，SA0 默认接高）
    static constexpr uint8_t kRegFuncCfgAccess = 0x01;
    static constexpr uint8_t kRegInt1Ctrl      = 0x0D;
    static constexpr uint8_t kRegWhoAmI        = 0x0F;
    static constexpr uint8_t kRegCtrl1Xl       = 0x10;
    static constexpr uint8_t kRegCtrl2G        = 0x11;
    static constexpr uint8_t kRegCtrl3C        = 0x12;
    static constexpr uint8_t kRegCtrl4C        = 0x13;
    static constexpr uint8_t kRegCtrl5C        = 0x14;
    static constexpr uint8_t kRegCtrl6C        = 0x15;
    static constexpr uint8_t kRegCtrl7G        = 0x16;
    static constexpr uint8_t kRegCtrl9Xl       = 0x18;
    static constexpr uint8_t kRegOutXLG        = 0x22; // 陀螺数据起点（6 字节）
    static constexpr uint8_t kRegOutXLA        = 0x28; // 加速度数据起点（6 字节）
    static constexpr uint8_t kAddrDefault      = 0x6B; // SA0 接高；SA0 接地为 0x6A
    static constexpr uint8_t kWhoAmIValue      = 0x6B; // LSM6DSR 芯片 ID
    static constexpr int kBootWaitUs           = 50'000; // 软复位后 boot 等待（ST 内核驱动同款 msleep(50)）
    static constexpr int kBootWaitTries        = 20; // WHO_AM_I 轮询次数（5ms 间隔，最长约 100ms）
    static constexpr int kBootPollUs           = 5'000; // WHO_AM_I 轮询间隔
    static constexpr int kConfigTries          = 3; // 配置写入 + 读回校验重试次数
    static constexpr int kConfigRetryUs        = 20'000; // 配置写入重试间隔
    static constexpr double kReinitIntervalS   = 1.0; // update() 未就绪时重试的最小间隔
    // 配置值（写入后读回比对校验用）
    static constexpr uint8_t kAccelConfig      = 0x3C; // CTRL1_XL：±8g，52Hz
    static constexpr uint8_t kGyroConfig       = 0x5C; // CTRL2_G：±2000dps，208Hz
    static constexpr uint8_t kCtrl3Config      = 0x44; // CTRL3_C：BDU + IF_INC
    static constexpr double kAccLsbPerG    = 4098.0; // ±8g（0.244 mg/LSB）
    static constexpr double kGyroLsbPerDps = 14.3;   // ±2000dps（70 mdps/LSB）
    static constexpr double kDeg2Rad       = std::numbers::pi_v<double> / 180.0;

    struct RawData {
        double ax, ay, az;                           // 加速度（g）
        double gx, gy, gz;                           // 角速度（rad/s）
        double dt;                                   // 距上次采样时间（s）
    };

    struct Quaternion {
        double w = 1.0, x = 0.0, y = 0.0, z = 0.0;
    };

    struct Euler {
        double roll, pitch, yaw;                     // 弧度制
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

    // 打开 /dev/i2c-{bus} 并执行完整配置序列：软复位 → 等待 boot →
    // WHO_AM_I 轮询（NACK 容错）→ 配置写入 + 读回校验重试。
    // 成功返回 true；失败关闭 fd 并在 last_error_ 记录原因，可反复调用
    bool configure_sensor() {
        if (fd_ >= 0) {
            ::close(fd_);
            fd_ = -1;
        }
        last_error_.clear();

        const std::string device = "/dev/i2c-" + std::to_string(i2c_bus_);
        fd_                      = ::open(device.c_str(), O_RDWR);
        if (fd_ < 0) {
            last_error_ = "open " + device + " failed: " + std::strerror(errno);
            return false;
        }
        if (ioctl(fd_, I2C_SLAVE, i2c_addr_) < 0) {
            last_error_ = "ioctl I2C_SLAVE on " + device + " failed: " + std::strerror(errno);
            ::close(fd_);
            fd_ = -1;
            return false;
        }

        try {
            write_reg(kRegFuncCfgAccess, 0x00); // 关闭传感器 HUB 寄存器访问
            write_reg(kRegCtrl3C, 0x01);        // 设备软复位
            ::usleep(kBootWaitUs);              // boot 最长约 50ms，期间写入会被 NACK

            // boot 期间 I2C 会 NACK 或 WHO_AM_I 返回错误值：轮询直到就绪
            uint8_t who = 0;
            for (int i = 0; i < kBootWaitTries; ++i) {
                try {
                    read_regs(kRegWhoAmI, &who, 1);
                } catch (const std::exception&) {
                    who = 0; // boot 窗口内 NACK，视为未就绪
                }
                if (who == kWhoAmIValue) {
                    break;
                }
                ::usleep(kBootPollUs);
            }
            if (who != kWhoAmIValue) {
                last_error_ = "WHO_AM_I=" + std::to_string(who) + ", expected "
                    + std::to_string(kWhoAmIValue) + ", check I2C address "
                    + std::to_string(i2c_addr_) + " (0x6B for SA0 high, 0x6A for SA0 low)";
                ::close(fd_);
                fd_ = -1;
                return false;
            }

            // 配置写入 + 读回校验：boot 尾巴上的写入可能被丢弃，整体重写兜底
            bool configured = false;
            for (int attempt = 0; attempt < kConfigTries; ++attempt) {
                try {
                    write_config();
                    configured = verify_config();
                } catch (const std::exception&) {
                    configured = false;
                }
                if (configured) {
                    break;
                }
                ::usleep(kConfigRetryUs);
            }
            if (!configured) {
                last_error_ = "config verify failed, check I2C link to sensor";
                ::close(fd_);
                fd_ = -1;
                return false;
            }
        } catch (const std::exception& e) {
            last_error_ = e.what();
            ::close(fd_);
            fd_ = -1;
            return false;
        }
        last_time_ = std::chrono::steady_clock::now();
        return true;
    }

    // 写入全部配置寄存器（量程/ODR/BDU/IF_INC 等，与逐飞官方例程一致）
    void write_config() {
        write_reg(kRegInt1Ctrl, 0x03); // 陀螺/加速度数据就绪中断（与官方例程一致）
        write_reg(kRegCtrl1Xl, kAccelConfig);
        write_reg(kRegCtrl2G, kGyroConfig);
        write_reg(kRegCtrl3C, kCtrl3Config);
        write_reg(kRegCtrl4C, 0x02);
        write_reg(kRegCtrl5C, 0x00);
        write_reg(kRegCtrl6C, 0x00);
        write_reg(kRegCtrl7G, 0x00);
        write_reg(kRegCtrl9Xl, 0x01); // 关闭 I3C 接口
    }

    // 读回关键配置寄存器，校验写入是否生效
    bool verify_config() {
        uint8_t v = 0;
        read_regs(kRegCtrl1Xl, &v, 1);
        if (v != kAccelConfig) {
            return false;
        }
        read_regs(kRegCtrl2G, &v, 1);
        if (v != kGyroConfig) {
            return false;
        }
        read_regs(kRegCtrl3C, &v, 1);
        return v == kCtrl3Config;
    }

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
        return static_cast<int16_t>((buf[off + 1] << 8) | buf[off]); // 小端
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

    // 从 LSM6DSR 获取原始数据：连续读加速度（0x28 起 6 字节）与陀螺
    // （0x22 起 6 字节），int16 小端，换算为 g / rad/s，并计算采样间隔
    RawData get_raw_data() {
        uint8_t buf[12];
        read_regs(kRegOutXLA, buf, 6);
        read_regs(kRegOutXLG, buf + 6, 6);

        const auto now  = std::chrono::steady_clock::now();
        const double dt = std::chrono::duration<double>(now - last_time_).count();
        last_time_      = now;

        RawData raw;
        raw.ax = read_int16(buf, 0) / kAccLsbPerG;
        raw.ay = read_int16(buf, 2) / kAccLsbPerG;
        raw.az = read_int16(buf, 4) / kAccLsbPerG;
        raw.gx = read_int16(buf, 6) / kGyroLsbPerDps * kDeg2Rad;
        raw.gy = read_int16(buf, 8) / kGyroLsbPerDps * kDeg2Rad;
        raw.gz = read_int16(buf, 10) / kGyroLsbPerDps * kDeg2Rad;
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

    int fd_           = -1;
    int i2c_bus_      = 1;
    uint8_t i2c_addr_ = kAddrDefault;
    std::string last_error_;
    std::chrono::steady_clock::time_point last_init_attempt_{};
    std::chrono::steady_clock::time_point last_time_{};
    Quaternion q_;
    KalmanState roll_kf_, pitch_kf_, yaw_kf_;
    bool filter_started_ = false;
    std::function<std::tuple<double, double, double>(double, double, double)> coordinate_mapping_;

    // 卡尔曼噪声参数（与 Mpu6050 驱动一致的经典调参）
    static constexpr double kQAngle   = 0.001;
    static constexpr double kQBias    = 0.003;
    static constexpr double kRMeasure = 0.03;
};
} // namespace dogbot_core::hardware::device
