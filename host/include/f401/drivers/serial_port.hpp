#pragma once

/*
 * POSIX 串口 RAII 封装（平台/驱动层）。
 *
 * 设计目的：
 *   1. 文件描述符在构造时打开、析构时关闭，即使 read/write 抛异常也不会泄漏；
 *   2. 禁止拷贝、允许移动，保证同一 fd 只有一个所有者；
 *   3. 读写都带超时，上层不会被阻塞的 read/write 卡死；
 *   4. 拔掉 USB / STM32 复位时，poll/read 返回 EIO，上层能感知“链路断开”。
 *
 * 这里不用 std::unique_ptr<int, decltype(&::close)>，因为 fd 的 -1 空值语义
 * 用自定义 UniqueFd 更直观；两者体现的是同一个 RAII 思想。
 */

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <span>
#include <string>

namespace f401::drivers {

/* 独占所有权的最小 RAII fd 包装。析构自动 close，移动后源对象变无效。 */
class UniqueFd {
public:
    UniqueFd() noexcept = default;
    explicit UniqueFd(int fd) noexcept;
    ~UniqueFd();

    UniqueFd(UniqueFd&& other) noexcept;
    UniqueFd& operator=(UniqueFd&& other) noexcept;

    UniqueFd(const UniqueFd&) = delete;
    UniqueFd& operator=(const UniqueFd&) = delete;

    int get() const noexcept { return fd_; }
    bool valid() const noexcept { return fd_ >= 0; }
    int release() noexcept;
    void reset(int fd = -1) noexcept;

private:
    int fd_ = -1;
};

class SerialPort {
public:
    /* 构造即打开并配置串口；失败抛 std::system_error。 */
    explicit SerialPort(const std::string& device, int baud_rate = 115200);
    ~SerialPort() = default;

    SerialPort(SerialPort&&) noexcept = default;
    SerialPort& operator=(SerialPort&&) noexcept = default;
    SerialPort(const SerialPort&) = delete;
    SerialPort& operator=(const SerialPort&) = delete;

    /* 读最多 buffer.size() 字节；超时返回 0。
     * 链路断开（USB 拔出 / 对端异常）抛 std::system_error。 */
    std::size_t readSome(std::span<std::uint8_t> buffer, std::chrono::milliseconds timeout);

    /* 写完整块数据；超时或链路断开抛 std::system_error。 */
    void writeAll(std::span<const std::uint8_t> data, std::chrono::milliseconds timeout);

    /* 丢弃内核输入缓冲，用于重连后清掉旧半帧。 */
    void discardInput();

    bool isOpen() const noexcept { return fd_.valid(); }
    int nativeHandle() const noexcept { return fd_.get(); }

private:
    static UniqueFd openDevice(const std::string& device);
    static void configure(int fd, int baud_rate);

    UniqueFd fd_;
};

}  // namespace f401::drivers
