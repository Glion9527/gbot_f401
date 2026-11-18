/* 串口 RAII 测试：用 PTY 伪装串口，不接真实硬件。
 * 覆盖：写读回环、超时、移动语义、析构关闭。
 *
 * 注意：部分沙箱环境（如 macOS 的受限终端）禁止 posix_openpt，
 * 此时测试打印 SKIP 并返回 0；在树莓派/Linux 或普通终端上会完整执行。 */

#define _XOPEN_SOURCE 600

#include <fcntl.h>
#include <poll.h>
#include <stdlib.h>
#include <unistd.h>

#include <array>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <optional>
#include <string>

#include "f401/drivers/serial_port.hpp"
#include "test_support.hpp"

namespace {

using namespace std::chrono_literals;

struct PtyPair {
    int master = -1;
    std::string slave;
};

/* posix_openpt + grantpt + unlockpt 是 POSIX 提供的伪终端 API；
 * 打开从端后它表现得就像一个普通串口设备，正好用来测 SerialPort。 */
std::optional<PtyPair> tryOpenPty()
{
    const int master = ::posix_openpt(O_RDWR | O_NOCTTY);
    if (master < 0) {
        return std::nullopt;
    }
    if (::grantpt(master) != 0 || ::unlockpt(master) != 0) {
        ::close(master);
        return std::nullopt;
    }
    const char* name = ::ptsname(master);
    if (name == nullptr) {
        ::close(master);
        return std::nullopt;
    }
    return PtyPair{master, std::string(name)};
}

int readWithTimeout(int fd, void* buffer, std::size_t size, int timeout_ms)
{
    pollfd descriptor{};
    descriptor.fd = fd;
    descriptor.events = POLLIN;
    const int ready = ::poll(&descriptor, 1, timeout_ms);
    if (ready <= 0) {
        return 0;
    }
    return static_cast<int>(::read(fd, buffer, size));
}

/* 不依赖 PTY 的 RAII 测试：用 pipe 验证 UniqueFd 的移动/释放语义。 */
void testUniqueFdMoveAndRelease()
{
    int descriptors[2] = {-1, -1};
    CHECK_EQ(::pipe(descriptors), 0);

    {
        f401::drivers::UniqueFd owner(descriptors[0]);
        CHECK(owner.valid());
        CHECK_EQ(owner.get(), descriptors[0]);

        f401::drivers::UniqueFd moved(std::move(owner));
        CHECK(!owner.valid());
        CHECK(moved.valid());
        CHECK_EQ(moved.get(), descriptors[0]);
        CHECK_EQ(moved.release(), descriptors[0]);
        CHECK(!moved.valid());
    }

    /* release 之后所有权已交还，测试自己关闭。 */
    ::close(descriptors[0]);
    ::close(descriptors[1]);
}

void testLoopbackAndTimeout(PtyPair& pair)
{
    f401::drivers::SerialPort port(pair.slave);
    CHECK(port.isOpen());

    /* 主端写 -> SerialPort 读。 */
    const std::array<std::uint8_t, 3> outgoing = {0x11, 0x22, 0x33};
    CHECK_EQ(::write(pair.master, outgoing.data(), outgoing.size()),
             static_cast<ssize_t>(outgoing.size()));

    std::array<std::uint8_t, 8> incoming{};
    const std::size_t count = port.readSome(incoming, 100ms);
    CHECK_EQ(count, std::size_t{3});
    CHECK_EQ(incoming[0], std::uint8_t{0x11});
    CHECK_EQ(incoming[2], std::uint8_t{0x33});

    /* SerialPort 写 -> 主端读。 */
    const std::array<std::uint8_t, 2> back = {0xAA, 0xBB};
    port.writeAll(back, 100ms);
    std::array<std::uint8_t, 2> received{};
    CHECK_EQ(readWithTimeout(pair.master, received.data(), received.size(), 100), 2);
    CHECK_EQ(received[0], std::uint8_t{0xAA});
    CHECK_EQ(received[1], std::uint8_t{0xBB});

    /* 没有数据时按超时返回 0，而不是永久阻塞。 */
    std::array<std::uint8_t, 1> nothing{};
    CHECK_EQ(port.readSome(nothing, 20ms), std::size_t{0});

    /* 移动后旧对象失效、新对象可用；旧对象析构不能误关新 fd。 */
    f401::drivers::SerialPort moved(std::move(port));
    CHECK(moved.isOpen());
    CHECK(!port.isOpen());
}

void testDestructorCloses()
{
    const auto pair = tryOpenPty();
    CHECK(pair.has_value());
    if (!pair) {
        return;
    }
    {
        f401::drivers::SerialPort port(pair->slave);
        CHECK(port.isOpen());
    }
    /* 析构后同一路径可以再次打开；若 fd 泄漏，这里会因设备忙而失败。 */
    f401::drivers::SerialPort reopened(pair->slave);
    CHECK(reopened.isOpen());
    ::close(pair->master);
}

}  // namespace

int main()
{
    testUniqueFdMoveAndRelease();

    auto pair = tryOpenPty();
    if (!pair) {
        std::printf("SKIP serial_port (PTY not permitted in this sandbox; "
                    "run on Linux/Raspberry Pi for full coverage)\n");
        return 0;
    }

    testLoopbackAndTimeout(*pair);
    ::close(pair->master);
    testDestructorCloses();
    return test::finish("serial_port RAII");
}
