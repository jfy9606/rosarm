#include <servo/vacuum_node.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <string>
#include <vector>
#include <dirent.h>
#include <cstring>

namespace servo {

VacuumNode::VacuumNode(const std::string& port, int baudrate) : serial_port_(-1) {
    // 获取参数
    nh_.param("max_power", max_power_, 100);
    nh_.param("default_power", default_power_, 80);
    port_name_ = port;
    
    // 初始化串口
    if (!initializeSerialPort(port, baudrate)) {
        ROS_ERROR("Initialization failed, will try to reconnect periodically");
        // 设置定期重试连接的定时器
        reconnect_timer_ = nh_.createTimer(ros::Duration(5.0), 
                                          [this, port, baudrate](const ros::TimerEvent&) {
                                              if (serial_port_ < 0) {
                                                  ROS_INFO("Attempting to reconnect serial port...");
                                                  initializeSerialPort(port, baudrate);
                                              }
                                          });
    }
    
    // 设置订阅者
    vacuum_cmd_sub_ = nh_.subscribe("vacuum/command", 10, &VacuumNode::vacuumCommandCallback, this);
    vacuum_power_sub_ = nh_.subscribe("vacuum/power", 10, &VacuumNode::vacuumPowerCallback, this);
    
    // 设置服务
    vacuum_service_ = nh_.advertiseService("vacuum_control", &VacuumNode::vacuumServiceCallback, this);
    
    ROS_INFO("Vacuum suction controller initialized, port: %s, baudrate: %d", port.c_str(), baudrate);
    
    // 默认关闭真空吸盘
    setVacuum(false, 0);
}

VacuumNode::~VacuumNode() {
    if (serial_port_ >= 0) {
        // 确保关闭真空吸盘
        setVacuum(false, 0);
        close(serial_port_);
    }
}

std::string VacuumNode::findSerialDevice(const std::string& preferred_port) {
    // 首先检查指定的端口
    if (access(preferred_port.c_str(), F_OK) != -1) {
        return preferred_port;
    }
    
    // 如果指定端口不可用，尝试查找其他可能的USB串口设备
    std::vector<std::string> possible_devices;
    
    // 搜索/dev目录
    DIR* dir = opendir("/dev");
    if (dir) {
        struct dirent* entry;
        while ((entry = readdir(dir)) != nullptr) {
            std::string name = entry->d_name;
            // 查找可能的串口设备
            if (name.find("ttyUSB") == 0 || name.find("ttyACM") == 0) {
                possible_devices.push_back("/dev/" + name);
            }
        }
        closedir(dir);
    }
    
    // 如果找到多个设备，优先选择备选端口
    for (const auto& device : possible_devices) {
        if (device == "/dev/ttyUSB1" || device == "/dev/ttyUSB0") {
            ROS_WARN("Using alternative serial device: %s", device.c_str());
            return device;
        }
    }
    
    // 如果找到任何设备，返回第一个
    if (!possible_devices.empty()) {
        ROS_WARN("Using available serial device: %s", possible_devices[0].c_str());
        return possible_devices[0];
    }
    
    // 如果没有找到设备，返回原始端口
    ROS_ERROR("No available serial device found, will try using original port: %s", preferred_port.c_str());
    return preferred_port;
}

bool VacuumNode::initializeSerialPort(const std::string& port, int baudrate) {
    // 检查并关闭已存在的连接
    if (serial_port_ >= 0) {
        close(serial_port_);
        serial_port_ = -1;
    }
    
    // 尝试查找可用串口设备
    std::string actual_port = findSerialDevice(port);
    port_name_ = actual_port;
    
    // 打开串口
    serial_port_ = open(actual_port.c_str(), O_RDWR | O_NOCTTY);
    if (serial_port_ < 0) {
        ROS_ERROR("Unable to open serial port %s", actual_port.c_str());
        return false;
    }
    
    // 配置串口
    struct termios tty;
    memset(&tty, 0, sizeof(tty));
    if (tcgetattr(serial_port_, &tty) != 0) {
        ROS_ERROR("tcgetattr error");
        close(serial_port_);
        serial_port_ = -1;
        return false;
    }
    
    // 设置波特率
    speed_t baud;
    switch (baudrate) {
        case 9600: baud = B9600; break;
        case 19200: baud = B19200; break;
        case 38400: baud = B38400; break;
        case 57600: baud = B57600; break;
        case 115200: baud = B115200; break;
        case 1000000: 
            #ifdef B1000000
            baud = B1000000; 
            #else
            ROS_WARN("Baudrate 1000000 not supported by this system, using 115200");
            baud = B115200;
            #endif
            break;
        default:
            ROS_WARN("Unsupported baudrate %d, using default 115200", baudrate);
            baud = B115200;
            break;
    }
    
    cfsetispeed(&tty, baud);
    cfsetospeed(&tty, baud);
    
    tty.c_cflag |= (CLOCAL | CREAD);    // 忽略调制解调器控制线
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;         // 8位数据位
    tty.c_cflag &= ~PARENB;     // 无奇偶校验
    tty.c_cflag &= ~CSTOPB;     // 1个停止位
    tty.c_cflag &= ~CRTSCTS;    // 无硬件流控制
    
    tty.c_iflag &= ~(IXON | IXOFF | IXANY); // 禁用软件流控制
    tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);
    
    tty.c_oflag &= ~OPOST; // 原始输出
    tty.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
    
    tty.c_cc[VMIN] = 0;  // 非阻塞读取
    tty.c_cc[VTIME] = 10; // 100ms超时
    
    if (tcsetattr(serial_port_, TCSANOW, &tty) != 0) {
        ROS_ERROR("tcsetattr error");
        close(serial_port_);
        serial_port_ = -1;
        return false;
    }
    
    ROS_INFO("Successfully connected to serial port: %s, baudrate: %d", actual_port.c_str(), baudrate);
    return true;
}

void VacuumNode::vacuumCommandCallback(const std_msgs::Bool::ConstPtr& msg) {
    setVacuum(msg->data, msg->data ? default_power_ : 0);
}

void VacuumNode::vacuumPowerCallback(const std_msgs::Int32::ConstPtr& msg) {
    int power = msg->data;
    if (power < 0) power = 0;
    if (power > max_power_) power = max_power_;
    
    if (power > 0) {
        setVacuum(true, power);
    } else {
        setVacuum(false, 0);
    }
}

bool VacuumNode::vacuumServiceCallback(servo::VacuumCmd::Request& req, 
                                     servo::VacuumCmd::Response& res) {
    int power = req.power;
    if (power < 0) power = 0;
    if (power > max_power_) power = max_power_;
    
    bool success = setVacuum(req.enable, power);
    res.success = success;
    res.message = success ? "Success" : "Failure";
    return true;
}

bool VacuumNode::setVacuum(bool enable, int power) {
    // 如果端口未打开，尝试重新连接
    if (serial_port_ < 0) {
        ROS_ERROR("Serial port not open, attempting to reconnect");
        if (!initializeSerialPort(port_name_, 115200)) {
            return false;
        }
    }
    
    // 使用VacuumControl更新状态
    if (enable) {
        vacuum_control_.activate();
        vacuum_control_.setSuctionPercentage(power);
    } else {
        vacuum_control_.deactivate();
    }
    
    // 限制功率范围
    if (power < 0) power = 0;
    if (power > max_power_) power = max_power_;
    
    // 构建命令
    uint8_t cmd[5];
    cmd[0] = 0xA5;  // 起始字节
    cmd[1] = 0x01;  // 命令类型：真空控制
    cmd[2] = enable ? 0x01 : 0x00;  // 开/关
    cmd[3] = static_cast<uint8_t>(power);  // 功率
    cmd[4] = cmd[0] ^ cmd[1] ^ cmd[2] ^ cmd[3];  // 校验和
    
    // 发送命令，尝试多次
    bool success = false;
    for (int i = 0; i < 3; i++) { // 最多尝试3次
        ssize_t bytes_written = write(serial_port_, cmd, sizeof(cmd));
        if (bytes_written == sizeof(cmd)) {
            success = true;
            break;
        }
        
        ROS_WARN("Write to serial port failed, retrying (%d/3)", i+1);
        // 短暂延迟后重试
        ros::Duration(0.1).sleep();
    }
    
    if (!success) {
        ROS_ERROR("Multiple attempts to write to serial port failed, possibly hardware connection issue");
        // 可能是串口断开，将端口标记为无效以便重新连接
        if (serial_port_ >= 0) {
            close(serial_port_);
            serial_port_ = -1;
        }
        return false;
    }
    
    ROS_INFO("Vacuum status: %s, power: %d%%", enable ? "ON" : "OFF", power);
    return true;
}

} // namespace servo

// 主函数
int main(int argc, char** argv) {
    ros::init(argc, argv, "vacuum_node");
    ros::NodeHandle nh("~");
    
    // 获取参数
    std::string port = "/dev/ttyUSB1";
    int baudrate = 115200;
    
    // 检查命令行参数
    if (argc > 1) {
        port = argv[1];
    }
    
    // 从参数服务器获取波特率
    nh.param("baudrate", baudrate, 115200);
    
    servo::VacuumNode node(port, baudrate);
    ros::spin();
    
    return 0;
} 