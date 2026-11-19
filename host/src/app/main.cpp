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
    bool imu = false;
    bool help = false;
    double vx = 0.0;
    double vy = 0.0;
    double wz = 0.0;
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
        } else if (argument == "--imu") {
            options.imu = true;
        } else if (argument == "--port" && index + 1 < argc) {
            options.port = argv[++index];
        } else if (argument == "--baud" && index + 1 < argc) {
            options.baud = std::atoi(argv[++index]);
        } else if (argument == "--vx" && index + 1 < argc) {
            options.vx = std::atof(argv[++index]);
        } else if (argument == "--vy" && index + 1 < argc) {
            options.vy = std::atof(argv[++index]);
        } else if (argument == "--wz" && index + 1 < argc) {
            options.wz = std::atof(argv[++index]);
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
                 "  f401_host --port <设备> [--baud N] [--arm] [--imu] [--vx M/S] [--vy M/S] [--wz RAD/S]\n";
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

        /* v2 没有会话握手，STM32 会跨 host 进程保留 s_last_seq。
         * 新进程必须先把本地 seq 对齐到 TELEM 回显的 last_seq，
         * 否则第一帧 seq=1 会被当成旧序号丢弃，ARM 永远不成功。 */
        if (!base.syncSequence(std::chrono::milliseconds(1000))) {
            std::cerr << "等待 TELEM 失败: " << base.lastError() << "\n";
            return 1;
        }

        /* ARM 必须持续发零速帧重试：固件要求四轮停稳 250ms，
         * 只发一帧就等，会在静默窗口未满足时直接失败。 */
        if (options.arm) {
            bool armed = false;
            const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(800);
            while (!g_stop && std::chrono::steady_clock::now() < deadline) {
                if (!base.setVelocity({})) {
                    std::cerr << "发送失败: " << base.lastError() << "\n";
                    return 1;
                }
                base.poll();
                const auto odometry = base.getOdometry();
                if (odometry && (odometry->status & LINK_V2_STATUS_ARMED) != 0U) {
                    armed = true;
                    break;
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(20));
            }
            if (!armed) {
                std::uint16_t status = 0U;
                if (const auto odometry = base.getOdometry()) {
                    status = odometry->status;
                }
                std::cerr << "MCU 未接受 ARM（TELEM status=0x" << std::hex << status
                          << std::dec << "）；检查四轮是否静止、USER 键是否被按下\n";
                return 1;
            }
        } else {
            /* 未要求使能时也发一帧零速，把 DISARM 意图明确送达。 */
            if (!base.setVelocity({})) {
                std::cerr << "发送失败: " << base.lastError() << "\n";
                return 1;
            }
        }
        std::cout << "已打开 " << options.port << "，使能=" << (options.arm ? "是" : "否")
                  << "，目标 vx=" << options.vx << " vy=" << options.vy
                  << " wz=" << options.wz << "，Ctrl-C 退出\n";

        const f401::core::ChassisSpeeds target{options.vx, options.vy, options.wz};
        auto next_send = std::chrono::steady_clock::now();
        auto next_print = std::chrono::steady_clock::now();
        std::uint32_t last_count = base.telemetryCount();

        while (!g_stop) {
            /* 50Hz 持续发送：v2 固件靠新帧刷新 300ms 许可，并喂电机看门狗。 */
            if (!base.setVelocity(target)) {
                std::cerr << "发送失败: " << base.lastError() << "\n";
                return 1;
            }
            base.poll();
            if (!base.lastError().empty()) {
                std::cerr << "链路错误: " << base.lastError() << "\n";
                return 1;
            }

            const auto now = std::chrono::steady_clock::now();
            if (now >= next_print) {
                next_print = now + std::chrono::seconds(1);
                const std::uint32_t count = base.telemetryCount();
                const std::uint32_t delta = count - last_count;
                last_count = count;
                if (const auto odometry = base.getOdometry()) {
                    std::cout << "telem=" << delta << "Hz seq=" << odometry->seq
                              << " status=0x" << std::hex << odometry->status
                              << std::dec << " x=" << odometry->pose.x_m
                              << " y=" << odometry->pose.y_m
                              << " yaw=" << odometry->pose.yaw_rad << "\n";
                } else {
                    std::cout << "telem=" << delta << "Hz 等待 TELEM\n";
                }
                if (options.imu) {
                    if (const auto imu = base.getImu()) {
                        std::cout << "  imu flags=0x" << std::hex << imu->flags << std::dec
                                  << " acc_mg=[" << imu->acc_mg[0] << ','
                                  << imu->acc_mg[1] << ',' << imu->acc_mg[2] << "]"
                                  << " gyro_dps10=[" << imu->gyro_dps10[0] << ','
                                  << imu->gyro_dps10[1] << ',' << imu->gyro_dps10[2] << "]"
                                  << " mag_ut10=[" << imu->mag_ut10[0] << ','
                                  << imu->mag_ut10[1] << ',' << imu->mag_ut10[2] << "]"
                                  << " temp_c10=" << imu->temp_c10 << "\n";
                    } else {
                        std::cout << "  imu 等待有效 TELEM\n";
                    }
                }
            }

            next_send += std::chrono::milliseconds(20);
            std::this_thread::sleep_until(next_send);
        }

        base.setArmed(false);
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
