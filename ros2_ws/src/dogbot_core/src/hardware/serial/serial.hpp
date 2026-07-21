#pragma once

#include <array>
#include <cerrno>
#include <cstdint>
#include <cstring>
#include <mutex>
#include <stdexcept>
#include <string>

#include <fcntl.h>
#include <sys/select.h>
#include <termios.h>
#include <unistd.h>

class SerialPort {
public:
    SerialPort(const std::string &device, int baud_rate)
        : device_(device), baud_rate_(baud_rate), fd_(-1) {
        open();
        configure();
    }

    ~SerialPort() {
        close();
    }

    SerialPort(const SerialPort &) = delete;
    SerialPort &operator=(const SerialPort &) = delete;
    SerialPort(SerialPort &&other) noexcept
        : device_(std::move(other.device_)),
          baud_rate_(other.baud_rate_),
          fd_(other.fd_) {
        other.fd_ = -1;
    }
    SerialPort &operator=(SerialPort &&other) noexcept {
        if (this != &other) {
            close();
            device_ = std::move(other.device_);
            baud_rate_ = other.baud_rate_;
            fd_ = other.fd_;
            other.fd_ = -1;
        }
        return *this;
    }

    bool isOpen() const {
        return fd_ >= 0;
    }

    const std::string &getDevice() const {
        return device_;
    }

    void sendPacket(const std::array<uint8_t, 16> &data) {
        std::lock_guard<std::mutex> lock(mutex_);
        size_t written = 0;
        while (written < data.size()) {
            ssize_t ret = ::write(fd_, data.data() + written,
                                  data.size() - written);
            if (ret < 0) {
                throw std::runtime_error("Serial write failed: " +
                                         std::string(std::strerror(errno)));
            }
            written += static_cast<size_t>(ret);
        }
    }

    void writeString(const std::string &data) {
        std::lock_guard<std::mutex> lock(mutex_);
        size_t written = 0;
        while (written < data.size()) {
            ssize_t ret = ::write(fd_, data.data() + written,
                                  data.size() - written);
            if (ret < 0) {
                throw std::runtime_error("Serial write failed: " +
                                         std::string(std::strerror(errno)));
            }
            written += static_cast<size_t>(ret);
        }
    }

    ssize_t readAvailable(uint8_t *buf, size_t len, int timeout_ms = 100) {
        std::lock_guard<std::mutex> lock(mutex_);
        struct timeval tv {};
        tv.tv_usec = (timeout_ms % 1000) * 1000;
        tv.tv_sec = timeout_ms / 1000;
        fd_set fds;
        FD_ZERO(&fds);
        FD_SET(fd_, &fds);
        if (::select(fd_ + 1, &fds, nullptr, nullptr, &tv) <= 0) {
            return 0;
        }
        return ::read(fd_, buf, len);
    }

private:
    void open() {
        fd_ = ::open(device_.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
        if (fd_ < 0) {
            throw std::runtime_error("Failed to open " + device_ + ": " +
                                     std::string(std::strerror(errno)));
        }
    }

    void configure() {
        struct termios tty {};
        if (::tcgetattr(fd_, &tty) != 0) {
            close();
            throw std::runtime_error("tcgetattr failed: " +
                                     std::string(std::strerror(errno)));
        }

        speed_t speed = getBaudConstant(baud_rate_);
        ::cfsetospeed(&tty, speed);
        ::cfsetispeed(&tty, speed);

        tty.c_cflag |= (CLOCAL | CREAD);
        tty.c_cflag &= ~CSIZE;
        tty.c_cflag |= CS8;
        tty.c_cflag &= ~PARENB;
        tty.c_cflag &= ~CSTOPB;
        tty.c_cflag &= ~CRTSCTS;

        tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ECHOK | ECHONL | ISIG);
        tty.c_iflag &= ~(IXON | IXOFF | IXANY | ICRNL | INLCR);
        tty.c_oflag &= ~OPOST;

        tty.c_cc[VMIN] = 0;
        tty.c_cc[VTIME] = 0;

        if (::tcsetattr(fd_, TCSANOW, &tty) != 0) {
            close();
            throw std::runtime_error("tcsetattr failed: " +
                                     std::string(std::strerror(errno)));
        }
    }

    void close() {
        if (fd_ >= 0) {
            ::close(fd_);
            fd_ = -1;
        }
    }

    static speed_t getBaudConstant(int baud_rate) {
        switch (baud_rate) {
            case 0:       return B0;
            case 50:      return B50;
            case 75:      return B75;
            case 110:     return B110;
            case 134:     return B134;
            case 150:     return B150;
            case 200:     return B200;
            case 300:     return B300;
            case 600:     return B600;
            case 1200:    return B1200;
            case 1800:    return B1800;
            case 2400:    return B2400;
            case 4800:    return B4800;
            case 9600:    return B9600;
            case 19200:   return B19200;
            case 38400:   return B38400;
            case 57600:   return B57600;
            case 115200:  return B115200;
            case 230400:  return B230400;
            case 460800:  return B460800;
            case 500000:  return B500000;
            case 576000:  return B576000;
            case 921600:  return B921600;
            case 1000000: return B1000000;
            case 1152000: return B1152000;
            case 1500000: return B1500000;
            case 2000000: return B2000000;
            case 2500000: return B2500000;
            case 3000000: return B3000000;
            case 3500000: return B3500000;
            case 4000000: return B4000000;
            default:
                throw std::invalid_argument(
                    "Unsupported baud rate: " + std::to_string(baud_rate));
        }
    }

    std::string device_;
    int baud_rate_;
    int fd_;
    mutable std::mutex mutex_;
};
