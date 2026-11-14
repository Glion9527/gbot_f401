/*
 * 宿主骨架的验收程序。
 *
 * 用法：
 *   f401_host --dry-run                 打印一帧 CMD 的十六进制，不碰硬件
 *   f401_host --sim                     用 SimBase 跑 2 秒仿真并打印位姿
 *   f401_host --port /dev/ttyACM0 --arm 打开串口，发 ARM + 零速，读 TELEM
 *
 * 注意：当前固件还没有实现 Link v2 接收端，所以 --port 模式在固件升级前
 * 只会得到“能打开串口、能发帧、没有遥测”的结果；这是阶段 0 的预期状态。
 */

#include <chrono>
#include <csignal>
#include <cstdlib>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <span>
#include <string>
#include <thread>

#include "f401/app/sim_base.hpp"
#include "f401/drivers/link_v2.hpp"
#include "f401/drivers/serial_base.hpp"

namespace {

volatile std::sig_atomic_t g_stop = 0;

void onSignal(int)
{
    g_stop = 1;
}

struct Options {
    std::string port;
    int baud = 115200;
    bool sim = false;
    bool dry_run = false;
    bool arm = false;
    bool help = false;
};

Options parseOptions(int argc, char** argv)
{
    Options options;
    for (int index = 1; index < argc; ++index) {
        const std::string argument = argv[index];
        if (argument == "--help" || argument == "-h") {
            options.help = true;
        } else if (argument == "--sim") {
            options.sim = true;
        } else if (argument == "--dry-run") {
            options.dry_run = true;
        } else if (argument == "--arm") {
            options.arm = true;
        } else if (argument == "--port" && index + 1 < argc) {
            options.port = argv[++index];
        } else if (argument == "--baud" && index + 1 < argc) {
            options.baud = std::atoi(argv[++index]);
        } else {
            std::cerr << "未知参数: " << argument << "\n";
            options.help = true;
        }
    }
    return options;
}

void printUsage()
{
    std::cout << "用法:\n"
                 "  f401_host --dry-run                 打印示例 CMD 帧\n"
                 "  f401_host --sim                     仿真运行 2 秒\n"
                 "  f401_host --port <设备> [--baud N] [--arm]  打开串口\n";
}

void printHex(std::span<const std::uint8_t> bytes)
{
    for (const std::uint8_t byte : bytes) {
        std::cout << std::hex << std::uppercase << std::setw(2) << std::setfill('0')
                  << static_cast<int>(byte) << ' ';
    }
    std::cout << std::dec << "\n";
}

int runDryRun()
{
    f401::drivers::WheelCommand command{};
    command.seq = 2;
    command.flags = LINK_V2_FLAG_ARM;
    command.wheel_mm_s = {300, 300, 300, 300};
    const auto frame = f401::drivers::encodeCommand(command);
    std::cout << "CMD 帧（18 字节）: ";
    printHex(frame);
    return 0;
}

int runSim()
{
    f401::app::SimBase base;
    base.setVelocity({0.1, 0.0, 0.0});
    for (int step = 0; step < 20; ++step) {
        base.poll();
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    if (const auto odometry = base.getOdometry()) {
        std::cout << "仿真位姿: x=" << odometry->pose.x_m << " m, y=" << odometry->pose.y_m
                  << " m, yaw=" << odometry->pose.yaw_rad << " rad\n";
        return 0;
    }
    std::cerr << "仿真没有产生里程计\n";
    return 1;
}

int runSerial(const Options& options)
{
    try {
        f401::drivers::SerialBase base(options.port, options.baud);
        base.setArmed(options.arm);
        std::signal(SIGINT, onSignal);

        /* 立即发一帧零速：既验证写链路，也把 ARM/DISARM 状态同步给 MCU。 */
        if (!base.setVelocity({})) {
            std::cerr << "发送失败: " << base.lastError() << "\n";
            return 1;
        }
        std::cout << "已打开 " << options.port << "，使能=" << (options.arm ? "是" : "否")
                  << "，Ctrl-C 退出\n";

        auto next_print = std::chrono::steady_clock::now();
        while (!g_stop) {
            base.poll();
            if (!base.lastError().empty()) {
                std::cerr << "链路错误: " << base.lastError() << "\n";
                return 1;
            }

            const auto now = std::chrono::steady_clock::now();
            if (now >= next_print) {
                next_print = now + std::chrono::seconds(1);
                if (const auto odometry = base.getOdometry()) {
                    std::cout << "seq=" << odometry->seq << " status=0x" << std::hex
                              << odometry->status << std::dec
                              << " x=" << odometry->pose.x_m
                              << " y=" << odometry->pose.y_m
                              << " yaw=" << odometry->pose.yaw_rad << "\n";
                } else {
                    std::cout << "等待 TELEM（当前固件尚未实现 v2 接收端）\n";
                }
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }

        base.setVelocity({});
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "串口失败: " << error.what() << "\n";
        return 1;
    }
}

}  // namespace

int main(int argc, char** argv)
{
    const Options options = parseOptions(argc, argv);

    if (options.help) {
        printUsage();
        return options.port.empty() && !options.sim && !options.dry_run ? 0 : 2;
    }
    if (options.dry_run) {
        return runDryRun();
    }
    if (options.sim) {
        return runSim();
    }
    if (options.port.empty()) {
        printUsage();
        return 2;
    }
    return runSerial(options);
}
