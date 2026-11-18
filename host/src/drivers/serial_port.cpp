#include "f401/drivers/serial_port.hpp"

#include <fcntl.h>
#include <poll.h>
#include <termios.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <stdexcept>
#include <system_error>

namespace f401::drivers {
namespace {

/* 只支持底盘实际会用到的波特率；遇到不认识的配置直接报错，
 * 而不是悄悄按默认速率打开导致“能打开但全是乱码”。 */
speed_t baudConstant(int baud_rate)
{
    switch (baud_rate) {
        case 9600: return B9600;
        case 19200: return B19200;
        case 38400: return B38400;
        case 57600: return B57600;
        case 115200: return B115200;
        case 230400: return B230400;
        default: throw std::invalid_argument("unsupported baud rate");
    }
}

/* 把 steady_clock 的截止时间换算成 poll() 需要的毫秒数。 */
int millisUntil(std::chrono::steady_clock::time_point deadline)
{
    const auto now = std::chrono::steady_clock::now();
    if (now >= deadline) {
        return 0;
    }
    const auto remaining =
        std::chrono::duration_cast<std::chrono::milliseconds>(deadline - now).count();
    return remaining > 0 ? static_cast<int>(remaining) : 0;
}

}  // namespace

UniqueFd::UniqueFd(int fd) noexcept : fd_(fd) {}

UniqueFd::~UniqueFd()
{
    reset();
}

UniqueFd::UniqueFd(UniqueFd&& other) noexcept : fd_(other.release()) {}

UniqueFd& UniqueFd::operator=(UniqueFd&& other) noexcept
{
    if (this != &other) {
        reset(other.release());
    }
    return *this;
}

int UniqueFd::release() noexcept
{
    const int fd = fd_;
    fd_ = -1;
    return fd;
}

void UniqueFd::reset(int fd) noexcept
{
    if (fd_ >= 0) {
        ::close(fd_);
    }
    fd_ = fd;
}

SerialPort::SerialPort(const std::string& device, int baud_rate)
    : fd_(openDevice(device))
{
    /* 打开成功但配置失败时，UniqueFd 的析构会负责关闭，
     * 再把异常抛给调用方；不会出现“fd 泄漏在半配置状态”。 */
    try {
        configure(fd_.get(), baud_rate);
    } catch (...) {
        fd_.reset();
        throw;
    }
}

UniqueFd SerialPort::openDevice(const std::string& device)
{
    const int fd = ::open(device.c_str(), O_RDWR | O_NOCTTY | O_CLOEXEC);
    if (fd < 0) {
        throw std::system_error(errno, std::generic_category(), "open " + device);
    }
    return UniqueFd(fd);
}

void SerialPort::configure(int fd, int baud_rate)
{
    termios settings{};
    if (::tcgetattr(fd, &settings) != 0) {
        throw std::system_error(errno, std::generic_category(), "tcgetattr");
    }

    /* raw 模式：不做行缓冲、不回显、不把 0x0D 映射成换行；
     * 我们要按字节流自己组帧。 */
    ::cfmakeraw(&settings);

    /* 8N1、无流控、本地连接；和 STM32 USART2 的配置一致。 */
    settings.c_cflag |= (CLOCAL | CREAD);
    settings.c_cflag &= ~CSTOPB;
    settings.c_cflag &= ~PARENB;
    settings.c_cflag &= ~CSIZE;
    settings.c_cflag |= CS8;
#ifdef CRTSCTS
    settings.c_cflag &= ~CRTSCTS;
#endif

    /* VMIN=0/VTIME=0 表示 read 不自行阻塞；等待由 poll() 的超时控制。 */
    settings.c_cc[VMIN] = 0;
    settings.c_cc[VTIME] = 0;

    if (::cfsetspeed(&settings, baudConstant(baud_rate)) != 0) {
        throw std::system_error(errno, std::generic_category(), "cfsetspeed");
    }
    if (::tcsetattr(fd, TCSANOW, &settings) != 0) {
        throw std::system_error(errno, std::generic_category(), "tcsetattr");
    }
    if (::tcflush(fd, TCIOFLUSH) != 0) {
        throw std::system_error(errno, std::generic_category(), "tcflush");
    }
}

std::size_t SerialPort::readSome(std::span<std::uint8_t> buffer,
                                 std::chrono::milliseconds timeout)
{
    if (!fd_.valid()) {
        throw std::system_error(EBADF, std::generic_category(), "serial port is closed");
    }
    if (buffer.empty()) {
        return 0;
    }

    const auto deadline = std::chrono::steady_clock::now() + timeout;
    for (;;) {
        pollfd descriptor{};
        descriptor.fd = fd_.get();
        descriptor.events = POLLIN;

        const int ready = ::poll(&descriptor, 1, millisUntil(deadline));
        if (ready < 0) {
            if (errno == EINTR) {
                continue; /* 被信号打断：按剩余时间继续等 */
            }
            throw std::system_error(errno, std::generic_category(), "poll read");
        }
        if (ready == 0) {
            return 0; /* 超时，没有数据 */
        }
        /* 先处理可读数据：对端关闭时 POLLIN 和 POLLHUP 可能同时置位，
         * 如果把 HUP 判在前面会丢掉缓冲区里最后几帧。 */
        if ((descriptor.revents & POLLIN) == 0) {
            if ((descriptor.revents & (POLLERR | POLLNVAL | POLLHUP)) != 0) {
                throw std::system_error(EIO, std::generic_category(), "serial link lost");
            }
            continue;
        }

        const ssize_t count = ::read(fd_.get(), buffer.data(), buffer.size());
        if (count < 0) {
            if (errno == EINTR) {
                continue;
            }
            if (errno == EAGAIN || errno == EWOULDBLOCK) {
                return 0;
            }
            throw std::system_error(errno, std::generic_category(), "read");
        }
        return static_cast<std::size_t>(count);
    }
}

void SerialPort::writeAll(std::span<const std::uint8_t> data,
                          std::chrono::milliseconds timeout)
{
    if (!fd_.valid()) {
        throw std::system_error(EBADF, std::generic_category(), "serial port is closed");
    }

    const auto deadline = std::chrono::steady_clock::now() + timeout;
    std::size_t written = 0;
    while (written < data.size()) {
        pollfd descriptor{};
        descriptor.fd = fd_.get();
        descriptor.events = POLLOUT;

        const int ready = ::poll(&descriptor, 1, millisUntil(deadline));
        if (ready < 0) {
            if (errno == EINTR) {
                continue;
            }
            throw std::system_error(errno, std::generic_category(), "poll write");
        }
        if (ready == 0) {
            throw std::system_error(ETIMEDOUT, std::generic_category(), "write timeout");
        }
        if ((descriptor.revents & (POLLERR | POLLNVAL | POLLHUP)) != 0) {
            throw std::system_error(EIO, std::generic_category(), "serial link lost");
        }
        if ((descriptor.revents & POLLOUT) == 0) {
            continue;
        }

        const ssize_t count =
            ::write(fd_.get(), data.data() + written, data.size() - written);
        if (count < 0) {
            if (errno == EINTR || errno == EAGAIN || errno == EWOULDBLOCK) {
                continue;
            }
            throw std::system_error(errno, std::generic_category(), "write");
        }
        written += static_cast<std::size_t>(count);
    }
}

void SerialPort::discardInput()
{
    if (!fd_.valid()) {
        return;
    }
    if (::tcflush(fd_.get(), TCIFLUSH) != 0) {
        throw std::system_error(errno, std::generic_category(), "tcflush input");
    }
}

}  // namespace f401::drivers
