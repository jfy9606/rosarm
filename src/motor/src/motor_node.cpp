#include <motor/motor_node.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <string>
#include <vector>
#include <dirent.h>
#include <cstring>

namespace motor {

MotorNode::MotorNode(const std::string& port, int baudrate) 
    : serial_port_(-1), port_name_(port), baudrate_(baudrate), max_velocity_(1000) {
    
    // 获取参数
    nh_.param("max_velocity", max_velocity_, 1000);
    
    // 初始化串口
    if (!initializeSerialPort(port, baudrate)) {
        ROS_ERROR("初始化串口失败，将尝试定期重新连接");
        // 设置定期重试连接的定时器
        reconnect_timer_ = nh_.createTimer(ros::Duration(5.0), 
                                          [this, port, baudrate](const ros::TimerEvent&) {
                                              if (serial_port_ < 0) {
                                                  ROS_INFO("尝试重新连接串口...");
                                                  initializeSerialPort(port, baudrate);
                                              }
                                          });
    }
    
    // 设置订阅者
    motor_order_sub_ = nh_.subscribe("/Controller_motor_order", 10, &MotorNode::motorOrderCallback, this);
    
    ROS_INFO("电机控制器已初始化，串口: %s，波特率: %d", port.c_str(), baudrate);
}

MotorNode::~MotorNode() {
    if (serial_port_ >= 0) {
        close(serial_port_);
    }
}

std::string MotorNode::findSerialDevice(const std::string& preferred_port) {
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
        if (device == "/dev/ttyUSB0" || device == "/dev/ttyUSB2") {
            ROS_WARN("使用备选串口设备: %s", device.c_str());
            return device;
        }
    }
    
    // 如果找到任何设备，返回第一个
    if (!possible_devices.empty()) {
        ROS_WARN("使用可用的串口设备: %s", possible_devices[0].c_str());
        return possible_devices[0];
    }
    
    // 如果没有找到设备，返回原始端口
    ROS_ERROR("未找到可用的串口设备，将尝试使用原始端口: %s", preferred_port.c_str());
    return preferred_port;
}

bool MotorNode::initializeSerialPort(const std::string& port, int baudrate) {
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
        ROS_ERROR("无法打开串口 %s", actual_port.c_str());
        return false;
    }
    
    // 配置串口
    struct termios tty;
    memset(&tty, 0, sizeof(tty));
    if (tcgetattr(serial_port_, &tty) != 0) {
        ROS_ERROR("tcgetattr 错误");
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
        default:
            ROS_WARN("不支持的波特率 %d，使用默认值 115200", baudrate);
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
        ROS_ERROR("tcsetattr 错误");
        close(serial_port_);
        serial_port_ = -1;
        return false;
    }
    
    ROS_INFO("成功连接到串口: %s，波特率: %d", actual_port.c_str(), baudrate);
    return true;
}

void MotorNode::motorOrderCallback(const motor::MotorOrder::ConstPtr& msg) {
    // 处理兼容liancheng_socket风格的电机命令
    size_t num_commands = std::min({
        msg->station_num.size(),
        msg->form.size(),
        msg->vel.size(),
        msg->vel_ac.size(),
        msg->vel_de.size(),
        msg->pos_mode.size(),
        msg->pos.size(),
        msg->pos_thr.size()
    });
    
    for (size_t i = 0; i < num_commands; ++i) {
        uint8_t station_num = msg->station_num[i];
        uint8_t form = msg->form[i];
        int16_t velocity = msg->vel[i];
        uint16_t acceleration = msg->vel_ac[i];
        uint16_t deceleration = msg->vel_de[i];
        bool pos_mode = msg->pos_mode[i];
        int32_t position = msg->pos[i];
        uint16_t pos_thr = msg->pos_thr[i];
        
        sendMotorCommand(station_num, form, velocity, acceleration, deceleration, 
                        pos_mode, position, pos_thr);
    }
}

bool MotorNode::sendMotorCommand(uint8_t motor_id, uint8_t form, int16_t velocity,
                               uint16_t acceleration, uint16_t deceleration,
                               bool is_position_mode, int32_t position,
                               uint16_t position_threshold) {
    // 检查串口连接
    if (serial_port_ < 0) {
        ROS_ERROR("串口未打开，尝试重新连接");
        if (!initializeSerialPort(port_name_, baudrate_)) {
            return false;
        }
    }
    
    // 使用MotorControl发送命令到硬件
    if (is_position_mode) {
        return motor_control_.setMotorPosition(motor_id, position, velocity, 
                                           acceleration, deceleration, 
                                           is_position_mode, position_threshold);
    } else {
        return motor_control_.setMotorVelocity(motor_id, velocity, 
                                            acceleration, deceleration);
    }
}

} // namespace motor 