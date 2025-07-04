#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int32.h>
#include <servo_wrist/VacuumControl.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <string>

class VacuumController {
private:
    ros::NodeHandle nh_;
    ros::Subscriber vacuum_cmd_sub_;
    ros::Subscriber vacuum_power_sub_;
    ros::ServiceServer vacuum_service_;
    
    int serial_port_;
    int max_power_;
    int default_power_;
    int current_power_;
    bool is_on_;
    std::string port_name_;
    
public:
    VacuumController(const std::string& port, int baudrate) : is_on_(false), current_power_(0) {
        // 获取参数
        nh_.param("max_power", max_power_, 100);
        nh_.param("default_power", default_power_, 80);
        port_name_ = port;
        
        // 打开串口
        serial_port_ = open(port.c_str(), O_RDWR | O_NOCTTY);
        if (serial_port_ < 0) {
            ROS_ERROR("无法打开串口 %s", port.c_str());
            return;
        }
        
        // 配置串口
        struct termios tty;
        memset(&tty, 0, sizeof(tty));
        if (tcgetattr(serial_port_, &tty) != 0) {
            ROS_ERROR("tcgetattr 错误");
            close(serial_port_);
            serial_port_ = -1;
            return;
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
            return;
        }
        
        // 设置订阅者
        vacuum_cmd_sub_ = nh_.subscribe("vacuum/command", 10, &VacuumController::vacuumCommandCallback, this);
        vacuum_power_sub_ = nh_.subscribe("vacuum/power", 10, &VacuumController::vacuumPowerCallback, this);
        
        // 设置服务
        vacuum_service_ = nh_.advertiseService("vacuum_control", &VacuumController::vacuumServiceCallback, this);
        
        ROS_INFO("真空吸盘控制器已初始化，串口: %s，波特率: %d", port.c_str(), baudrate);
        
        // 默认关闭真空吸盘
        setVacuum(false, 0);
    }
    
    ~VacuumController() {
        if (serial_port_ >= 0) {
            // 确保关闭真空吸盘
            setVacuum(false, 0);
            close(serial_port_);
        }
    }
    
    void vacuumCommandCallback(const std_msgs::Bool::ConstPtr& msg) {
        setVacuum(msg->data, msg->data ? default_power_ : 0);
    }
    
    void vacuumPowerCallback(const std_msgs::Int32::ConstPtr& msg) {
        int power = msg->data;
        if (power < 0) power = 0;
        if (power > max_power_) power = max_power_;
        
        if (power > 0) {
            setVacuum(true, power);
        } else {
            setVacuum(false, 0);
        }
    }
    
    bool vacuumServiceCallback(servo_wrist::VacuumControl::Request& req, 
                              servo_wrist::VacuumControl::Response& res) {
        int power = req.power;
        if (power < 0) power = 0;
        if (power > max_power_) power = max_power_;
        
        bool success = setVacuum(req.enable, power);
        res.success = success;
        res.message = success ? "成功" : "失败";
        return true;
    }
    
    bool setVacuum(bool enable, int power) {
        if (serial_port_ < 0) {
            ROS_ERROR("串口未打开");
            return false;
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
        
        // 发送命令
        ssize_t bytes_written = write(serial_port_, cmd, sizeof(cmd));
        if (bytes_written != sizeof(cmd)) {
            ROS_ERROR("写入串口失败");
            return false;
        }
        
        // 更新状态
        is_on_ = enable;
        current_power_ = enable ? power : 0;
        
        ROS_INFO("真空吸盘: %s, 功率: %d%%", enable ? "开启" : "关闭", power);
        return true;
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "vacuum_controller");
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
    
    VacuumController controller(port, baudrate);
    ros::spin();
    
    return 0;
} 