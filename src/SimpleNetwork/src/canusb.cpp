#include <iostream>
#include <vector>
#include <stdexcept>
#include<thread>
#include <csignal>
#include <chrono>
#include <random>
#include <algorithm>
#include <sstream>
#include <iomanip>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <errno.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <asm/termbits.h> /* struct termios2 */
#include <time.h>
#include <ctype.h>
#include <signal.h>
#include <sys/time.h>
#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/JointState.h>
#include <liancheng_socket/MotorOrder.h>
#include <std_msgs/UInt8MultiArray.h>

// Constants
constexpr int CANUSB_INJECT_SLEEP_GAP_DEFAULT = 200; // ms
constexpr int CANUSB_TTY_BAUD_RATE_DEFAULT = 2000000;

// Enumerations
enum class CANUSB_SPEED {
  SPEED_1000000 = 0x01,
  SPEED_800000  = 0x02,
  SPEED_500000  = 0x03,
  SPEED_400000  = 0x04,
  SPEED_250000  = 0x05,
  SPEED_200000  = 0x06,
  SPEED_125000  = 0x07,
  SPEED_100000  = 0x08,
  SPEED_50000   = 0x09,
  SPEED_20000   = 0x0a,
  SPEED_10000   = 0x0b,
  SPEED_5000    = 0x0c
};

enum class CANUSB_MODE {
  NORMAL          = 0x00,
  LOOPBACK        = 0x01,
  SILENT          = 0x02,
  LOOPBACK_SILENT = 0x03
};

enum class CANUSB_FRAME {
  STANDARD = 0x01,
  EXTENDED = 0x02
};

enum class CANUSB_PAYLOAD_MODE {
  RANDOM      = 0,
  INCREMENTAL = 1,
  FIXED       = 2
};

// Global variables
static int terminate_after = 1;
static bool program_running = true;
static CANUSB_PAYLOAD_MODE inject_payload_mode = CANUSB_PAYLOAD_MODE::FIXED;
static float inject_sleep_gap = CANUSB_INJECT_SLEEP_GAP_DEFAULT;
static int print_traffic = 1;

// Function declarations will go here

// C++ 版本的辅助函数
class CanUsbHelper {
public:
    static CANUSB_SPEED int_to_speed(int speed) {
        switch (speed) {
            case 1000000: return CANUSB_SPEED::SPEED_1000000;
            case 800000:  return CANUSB_SPEED::SPEED_800000;
            case 500000:  return CANUSB_SPEED::SPEED_500000;
            case 400000:  return CANUSB_SPEED::SPEED_400000;
            case 250000:  return CANUSB_SPEED::SPEED_250000;
            case 200000:  return CANUSB_SPEED::SPEED_200000;
            case 125000:  return CANUSB_SPEED::SPEED_125000;
            case 100000:  return CANUSB_SPEED::SPEED_100000;
            case 50000:   return CANUSB_SPEED::SPEED_50000;
            case 20000:   return CANUSB_SPEED::SPEED_20000;
            case 10000:   return CANUSB_SPEED::SPEED_10000;
            case 5000:    return CANUSB_SPEED::SPEED_5000;
            default:      return static_cast<CANUSB_SPEED>(0);
        }
    }

    static int generate_checksum(const std::vector<unsigned char>& data) {
        int checksum = 0;
        for (auto byte : data) {
            checksum += byte;
        }
        return checksum & 0xff;
    }

    static bool frame_is_complete(const std::vector<unsigned char>& frame) {
        size_t frame_len = frame.size();

        if (frame_len > 0 && frame[0] != 0xaa) {
            // Need to sync on 0xaa at start of frames, so just skip.
            return true;
        }

        if (frame_len < 2) {
            return false;
        }

        if (frame[1] == 0x55) { // Command frame
            return frame_len >= 20; // Command frames are always 20 bytes.
        } else if ((frame[1] >> 4) == 0xc) { // Data frame
            return frame_len >= (frame[1] & 0xf) + 5; // payload and 5 bytes.
        }

        // Unhandled frame type.
        return true;
    }

    // 其他辅助函数将在此添加
};

class CanUsbInterface {
public:
    CanUsbInterface(const std::string& device, int baudrate,ros::NodeHandle & nh) 
        : tty_device(device), tty_fd(-1), baudrate(baudrate),nh_(nh) {
            sub=nh_.subscribe("/CANController_motor_order",5,&CanUsbInterface::callback,this);
            speed=CanUsbHelper::int_to_speed(1000000);
        }

    ~CanUsbInterface() {
        if (tty_fd != -1) {
            close(tty_fd);
        }
    }

    bool initialize() {
        tty_fd = open(tty_device.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
        if (tty_fd == -1) {
            std::cerr << "open(" << tty_device << ") failed: " << strerror(errno) << "\n";
            return false;
        }

        struct termios2 tio;
        if (ioctl(tty_fd, TCGETS2, &tio) == -1) {
            std::cerr << "ioctl() failed: " << strerror(errno) << "\n";
            close(tty_fd);
            return false;
        }

        tio.c_cflag &= ~CBAUD;
        tio.c_cflag = BOTHER | CS8 | CSTOPB;
        tio.c_iflag = IGNPAR;
        tio.c_oflag = 0;
        tio.c_lflag = 0;
        tio.c_ispeed = baudrate;
        tio.c_ospeed = baudrate;

        if (ioctl(tty_fd, TCSETS2, &tio) == -1) {
            std::cerr << "ioctl() failed: " << strerror(errno) << "\n";
            close(tty_fd);
            return false;
        }

        return true;
    }

    bool sendFrame(const std::vector<unsigned char>& frame) {
        if (print_traffic) {
            std::cout << ">>> ";
            for (auto byte : frame) {
                std::cout << std::hex << static_cast<int>(byte) << " ";
            }
            std::cout << "\n";
        }


        ssize_t result = write(tty_fd, frame.data(), frame.size());
	std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<long>(inject_sleep_gap)));
        if (result == -1) {
            std::cerr << "write() failed: " << strerror(errno) << "\n";
            return false;
        }

        return true;
    }

    std::vector<unsigned char> receiveFrame(int frame_len_max) {
        std::vector<unsigned char> frame;
        unsigned char byte;
        ssize_t result;

        while (program_running && frame.size() < frame_len_max) {
            result = read(tty_fd, &byte, 1);
            if (result == -1) {
                if (errno != EAGAIN && errno != EWOULDBLOCK) {
                    std::cerr << "read() failed: " << strerror(errno) << "\n";
                    break;
                }
            } else if (result > 0) {
                frame.push_back(byte);
                if (CanUsbHelper::frame_is_complete(frame)) {
                    break;
                }
            }
            std::this_thread::sleep_for(std::chrono::microseconds(10));
        }

        return frame;
    }

    bool commandSettings(CANUSB_SPEED speed, CANUSB_MODE mode, CANUSB_FRAME frameType) {
        std::vector<unsigned char> cmdFrame(20, 0);

        cmdFrame[0] = 0xaa;
        cmdFrame[1] = 0x55;
        cmdFrame[2] = 0x12;
        cmdFrame[3] = static_cast<unsigned char>(speed);
        cmdFrame[4] = static_cast<unsigned char>(frameType);
        // Bytes 5-12 are reserved for filter ID and mask ID, not handled here
        cmdFrame[13] = static_cast<unsigned char>(mode);
        cmdFrame[14] = 0x01; // reserved
        // Bytes 15-18 are reserved
        cmdFrame[19] = CanUsbHelper::generate_checksum({cmdFrame.begin() + 2, cmdFrame.end() - 1});

        return sendFrame(cmdFrame);
    }

    bool sendDataFrame(CANUSB_FRAME frameType, unsigned char idLsb, unsigned char idMsb, const std::vector<unsigned char>& data) {
        if (data.size() > 8) {
            std::cerr << "Data length code (DLC) must be between 0 and 8!\n";
            return false;
        }

        std::vector<unsigned char> dataFrame;
        dataFrame.reserve(13); // Max frame size
        dataFrame.push_back(0xaa); // Byte 0: Packet Start

        // Byte 1: CAN Bus Data Frame Information
        unsigned char frameInfo = 0xC0; // Bits 7 and 6 always 1
        frameInfo |= (frameType == CANUSB_FRAME::STANDARD) ? 0 : 0x20; // STD or EXT frame
        frameInfo |= static_cast<unsigned char>(data.size()); // DLC=data_len
        dataFrame.push_back(frameInfo);

        // Bytes 2-3: ID
        dataFrame.push_back(idLsb); // LSB
        dataFrame.push_back(idMsb); // MSB

        // Bytes 4 to (4+data_len): Data
        dataFrame.insert(dataFrame.end(), data.begin(), data.end());

        // Last byte: End of frame
        dataFrame.push_back(0x55);

        return sendFrame(dataFrame);
    }

    void setProgramOptions(int argc, char* argv[]) {
        for (int i = 1; i < argc; ++i) {
            std::string arg = argv[i];
            if (arg == "-h") {
                displayHelp();
                std::exit(EXIT_SUCCESS);
            } else if (arg == "-t") {
                print_traffic++;
            } else if (arg == "-d" && i + 1 < argc) {
                tty_device = argv[++i];
            } else if (arg == "-s" && i + 1 < argc) {
                speed = CanUsbHelper::int_to_speed(std::stoi(argv[++i]));
            } else if (arg == "-b" && i + 1 < argc) {
                baudrate = std::stoi(argv[++i]);
            } else if (arg == "-i" && i + 1 < argc) {
                // 设置注入 ID
                inject_id=argv[++i];
            } else if (arg == "-j" && i + 1 < argc) {
                // 设置注入数据
                inject_data=argv[++i];
            } else if (arg == "-n" && i + 1 < argc) {
                terminate_after = std::stoi(argv[++i]);
            } else if (arg == "-g" && i + 1 < argc) {
                inject_sleep_gap = std::stof(argv[++i]);
            } else if (arg == "-m" && i + 1 < argc) {
                inject_payload_mode = static_cast<CANUSB_PAYLOAD_MODE>(std::stoi(argv[++i]));
            }
        }
    }

    // 显示帮助信息的函数
    void displayHelp() {
        std::cout << "Usage: [options]\n"
                  << "Options:\n"
                  << "  -h          Display this help and exit.\n"
                  << "  -t          Print TTY/serial traffic debugging info on stderr.\n"
                  << "  -d DEVICE   Use TTY DEVICE.\n"
                  << "  -s SPEED    Set CAN SPEED in bps.\n"
                  << "  -b BAUDRATE Set TTY/serial BAUDRATE (default: " << CANUSB_TTY_BAUD_RATE_DEFAULT << ").\n"
                  << "  -i ID       Inject using ID (specified as hex string).\n"
                  << "  -j DATA     CAN DATA to inject (specified as hex string).\n"
                  << "  -n COUNT    Terminate after COUNT frames (default: infinite).\n"
                  << "  -g MS       Inject sleep gap in MS milliseconds (default: " << CANUSB_INJECT_SLEEP_GAP_DEFAULT << " ms).\n"
                  << "  -m MODE     Inject payload MODE (0 = random, 1 = incremental, 2 = fixed).\n"
                  << std::endl;
    }

    void run() {
        // setProgramOptions(argc, argv);
        setupSignalHandlers();

        if (tty_device.empty()) {
            std::cerr << "Please specify a TTY device.\n";
            displayHelp();
            std::exit(EXIT_FAILURE);
        }

        if (!initialize()) {
            std::cerr << "Failed to initialize the adapter.\n";
            std::exit(EXIT_FAILURE);
        }

        commandSettings(speed, CANUSB_MODE::NORMAL, CANUSB_FRAME::STANDARD);

        // if (inject_data.empty()) {
        //     // 调用数据帧接收和打印函数
        //     dumpDataFrames();
        // } else {
            // 调用数据帧发送函数
            if (inject_data_frame() == -1) {
                std::exit(EXIT_FAILURE);
            }
        // }
    }

    void dumpDataFrames() {
        while (program_running) {
            auto frame = receiveFrame(32); // 假设最大帧长度为32

            if (!program_running) {
                break;
            }

            if (frame.empty()) {
                continue; // 处理接收错误或空帧
            }

            printFrame(frame);
        }
    }

    // 发送数据帧的方法
    int inject_data_frame() {
        // 将注入数据和ID从十六进制字符串转换为二进制
        // auto binary_data = convertFromHex(inject_data);
        // auto binary_id = convertFromHex(inject_id);

        // if (binary_data.empty() || binary_id.empty()) {
        //     std::cerr << "Invalid data or ID format.\n";
        //     return -1;
        // }

        // 根据注入模式发送数据帧
        while (program_running) {
            modifyDataForInjectionMode(data); // 根据注入模式修改数据
            if (!sendDataFrame(CANUSB_FRAME::STANDARD, station_num_low, station_num_high, data)) {
                std::cerr << "Failed to send data frame.\n";
                return -1;
            }

            //std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<long>(inject_sleep_gap)));

            if (terminate_after > 0 && (--terminate_after == 0)) {
                break;
            }
        }

        return 0;
    }

    void setupSignalHandlers() {
        std::signal(SIGTERM, CanUsbInterface::signalHandler);
        std::signal(SIGHUP, CanUsbInterface::signalHandler);
        std::signal(SIGINT, CanUsbInterface::signalHandler);
    }

    // 其他函数将在此添加

private:
    std::string tty_device;
    int tty_fd;
    int baudrate;
    CANUSB_SPEED speed;
    std::string inject_data;  // 注入数据的字符串
    std::string inject_id;    // 注入ID的字符串
    ros::NodeHandle nh_;
    ros::Subscriber sub;
    uint8_t station_num_high;
    uint8_t station_num_low;
    std::vector<unsigned char> data;

    std::vector<unsigned char> convertFromHex(const std::string& hex) {
        std::vector<unsigned char> binary;
        for (size_t i = 0; i < hex.length(); i += 2) {
            std::string byteString = hex.substr(i, 2);
            unsigned char byte = static_cast<unsigned char>(std::stoi(byteString, nullptr, 16));
            binary.push_back(byte);
        }
        return binary;
    }

    // 根据注入模式修改数据的辅助函数
    void modifyDataForInjectionMode(std::vector<unsigned char>& data) {
        switch (inject_payload_mode) {
            case CANUSB_PAYLOAD_MODE::RANDOM: {
                std::generate(data.begin(), data.end(), []() { return static_cast<unsigned char>(rand() % 256); });
                break;
            }
            case CANUSB_PAYLOAD_MODE::INCREMENTAL: {
                std::transform(data.begin(), data.end(), data.begin(), [](unsigned char byte) { return byte + 1; });
                break;
            }
            case CANUSB_PAYLOAD_MODE::FIXED:
            default:
                break; // 不改变数据
        }
    }

    void printFrame(const std::vector<unsigned char>& frame) {
        // 示例：简单的打印逻辑
        std::cout << "Received Frame: ";
        for (auto byte : frame) {
            std::cout << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(byte) << " ";
        }
        std::cout << std::endl;
    }

    static void signalHandler(int signum) {
        std::cerr << "Received signal: " << signum << std::endl;
        program_running = false;
    }

    void callback(const liancheng_socket::MotorOrder & order)
    {
        int order_num=order.station_num.size()/2;
        for (int i=0;i<order_num;i++)
        {
            station_num_low=order.station_num[2*i+1];
            station_num_high=order.station_num[2*i];
            terminate_after = 1;
            data.clear();
            data.push_back(0xa4);
            data.push_back(0x00);
            data.push_back(uint8_t(order.vel[i]&0x00ff));
            data.push_back(uint8_t(order.vel[i]>>8));
            int16_t pos_high=int16_t(order.pos[i]/(256*256));
            int16_t pos_low=int16_t(order.pos[i]%(256*256));
            data.push_back(uint8_t(pos_low&0x00ff));
            data.push_back(uint8_t(pos_low>>8));
            data.push_back(uint8_t(pos_high&0x00ff));
            data.push_back(uint8_t(pos_high>>8));
            run();
        }
    }
};



int main(int argc, char* argv[]) {
    try {
        ROS_INFO("starting");
        ros::init( argc, argv, "can" );
        ros::NodeHandle nh;
        CanUsbInterface canInterface("/dev/ttyUSB0",CANUSB_TTY_BAUD_RATE_DEFAULT,nh);
        
        ros::Rate rate(50.0);
        ros::spin();
        
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}
