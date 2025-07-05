#!/bin/bash
# 机械臂控制系统启动脚本

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[0;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
NC='\033[0m' # No Color

# 修复串口和摄像头权限
fix_permissions() {
    echo -e "${BLUE}=====================================${NC}"
    echo -e "${GREEN}正在修复设备权限...${NC}"
    
    # 检查是否为root用户
    if [ "$EUID" -eq 0 ]; then
        # 修复串口权限
        echo -e "${CYAN}修复串口权限...${NC}"
        for port in /dev/ttyUSB* /dev/ttyACM*; do
            if [ -e "$port" ]; then
                chmod 666 "$port"
                echo -e "${GREEN}已设置权限: $port${NC}"
            fi
        done
        
        # 修复摄像头权限
        echo -e "${CYAN}修复摄像头权限...${NC}"
        for camera in /dev/video*; do
            if [ -e "$camera" ]; then
                chmod 666 "$camera"
                echo -e "${GREEN}已设置权限: $camera${NC}"
            fi
        done
        
        # 添加当前用户到相关组
        if [ -n "$SUDO_USER" ]; then
            echo -e "${CYAN}将用户 $SUDO_USER 添加到dialout和video组...${NC}"
            usermod -a -G dialout "$SUDO_USER"
            usermod -a -G video "$SUDO_USER"
            echo -e "${GREEN}用户组设置完成${NC}"
        fi
    else
        # 非root用户，使用sudo尝试修复权限
        echo -e "${YELLOW}需要管理员权限来修复设备权限${NC}"
        echo -e "${CYAN}请输入密码以修复权限:${NC}"
        
        # 使用sudo修复串口权限
        sudo -p "" sh -c '
            for port in /dev/ttyUSB* /dev/ttyACM*; do
                if [ -e "$port" ]; then
                    chmod 666 "$port"
                    echo -e "\033[0;32m已设置权限: $port\033[0m"
                fi
            done
            
            # 修复摄像头权限
            for camera in /dev/video*; do
                if [ -e "$camera" ]; then
                    chmod 666 "$camera"
                    echo -e "\033[0;32m已设置权限: $camera\033[0m"
                fi
            done
            
            # 添加当前用户到相关组
            usermod -a -G dialout "'$USER'"
            usermod -a -G video "'$USER'"
            echo -e "\033[0;32m用户组设置完成\033[0m"
        ' || {
            echo -e "${RED}权限修复失败。请手动运行以下命令:${NC}"
            echo -e "${YELLOW}sudo chmod 666 /dev/ttyUSB* /dev/ttyACM* /dev/video*${NC}"
            echo -e "${YELLOW}sudo usermod -a -G dialout,video $USER${NC}"
        }
    fi
    
    echo -e "${BLUE}=====================================${NC}"
    echo -e "${GREEN}权限修复完成，按任意键继续...${NC}"
    read -r -n 1
}

# 自动检测串口设备
detect_ports() {
    echo -e "${BLUE}=====================================${NC}"
    echo -e "${GREEN}智能串口设备检测${NC}"
    echo -e "${BLUE}=====================================${NC}"
    
    # 检查是否有串口设备
    USB_PORTS=$(ls /dev/ttyUSB* 2>/dev/null || echo "")
    ACM_PORTS=$(ls /dev/ttyACM* 2>/dev/null || echo "")
    
    # 所有可用串口
    ALL_PORTS="${USB_PORTS} ${ACM_PORTS}"
    
    # 计算串口数量
    PORT_COUNT=$(echo "${ALL_PORTS}" | wc -w)
    
    if [ "${PORT_COUNT}" -eq 0 ]; then
        echo -e "${RED}未检测到任何串口设备${NC}"
        echo -e "${YELLOW}请检查USB连接或设备驱动${NC}"
        # 添加按键继续
        echo -e "${YELLOW}按任意键继续...${NC}"
        read -r -n 1
        return 1
    fi
    
    # 显示检测到的串口
    echo -e "${GREEN}检测到 ${PORT_COUNT} 个串口设备:${NC}"
    for port in ${ALL_PORTS}; do
        echo -e "${CYAN}- ${port}${NC}"
    done
    
    # 智能检测舵机串口 - 通过串口特性识别
    echo -e "\n${GREEN}开始智能检测舵机控制串口...${NC}"
    
    # 1. 创建临时目录
    TEMP_DIR=$(mktemp -d)
    SERVO_DETECTION_LOG="$TEMP_DIR/servo_detection.log"
    
    # 2. 定义一个函数来测试串口是否是舵机控制串口
    test_servo_port() {
        local port="$1"
        local device_type=""
        echo -e "${CYAN}测试串口 ${port}...${NC}"
        
        # 保存原始串口权限
        local original_perm=$(stat -c %a "$port" 2>/dev/null)
        
        # 设置串口权限
        sudo chmod 666 "$port" 2>/dev/null
        
        # 测试两种可能的波特率
        local baud_rates=("1000000" "115200")
        local device_types=("舵机控制器(1M波特率)" "电机控制器(115200波特率)")
        
        for i in "${!baud_rates[@]}"; do
            local baud=${baud_rates[$i]}
            local device=${device_types[$i]}
            
            echo -e "${CYAN}尝试以 ${baud} 波特率测试 ${port}...${NC}"
            
            # 尝试与设备通信
            {
                # 设置串口参数
                stty -F "$port" $baud -echo raw 2>/dev/null
                # 短暂延迟
                sleep 0.2
                
                # 创建响应日志文件
                local log_file="$TEMP_DIR/device_${baud}_detection.log"
                
                # 尝试发送无害的命令并读取响应
                # 这里使用timeout命令限制等待时间
                timeout 0.5 cat "$port" > "$log_file" 2>/dev/null &
                local cat_pid=$!
                
                # 根据波特率发送对应的查询命令
                if [ "$baud" == "1000000" ]; then
                    # SCS舵机的查询命令 (0xFF 0xFD ID 0x01 0x00 CHKSUM)
                    echo -en '\xFF\xFD\x01\x01\x00\xFF' > "$port" 2>/dev/null
                else
                    # 电机控制器的查询命令 (使用简单的读取状态命令)
                    # 这里使用的是基本查询命令，需要根据实际协议调整
                    echo -en '\x03\x0b\x00\x00\x01' > "$port" 2>/dev/null
                fi
                
                sleep 0.3
                
                # 尝试停止cat进程
                kill -9 $cat_pid 2>/dev/null
                
                # 检查是否有响应数据
                local response_size=$(wc -c < "$log_file" 2>/dev/null || echo "0")
                
                if [ "$response_size" -gt 0 ]; then
                    echo -e "${GREEN}串口 ${port} 在 ${baud} 波特率下有响应 (${response_size} 字节)${NC}"
                    # 记录设备类型
                    device_type="$device"
                    # 根据波特率设置不同的返回值
                    if [ "$baud" == "1000000" ]; then
                        DETECTED_DEVICE_TYPE="servo"
                    else
                        DETECTED_DEVICE_TYPE="motor"
                    fi
                    break
                fi
            } 2>/dev/null || true
        done
        
        # 恢复原始权限
        if [ -n "$original_perm" ]; then
            sudo chmod "$original_perm" "$port" 2>/dev/null
        fi
        
        if [ -n "$device_type" ]; then
            echo -e "${GREEN}在 ${port} 检测到 ${device_type}！${NC}"
            return 0
        else
            echo -e "${YELLOW}串口 ${port} 无响应${NC}"
            return 1
        fi
    }
    
    # 3. 遍历所有串口进行测试
    DETECTED_SERVO_PORT=""
    DETECTED_MOTOR_PORT=""
    DETECTED_DEVICE_TYPE=""
    
    # 首先测试USB设备 (通常设备控制器连接到USB)
    for port in ${USB_PORTS}; do
        if test_servo_port "$port"; then
            if [ "$DETECTED_DEVICE_TYPE" == "servo" ]; then
                DETECTED_SERVO_PORT="$port"
                echo -e "${GREEN}已将 ${port} 标识为舵机控制器${NC}"
            elif [ "$DETECTED_DEVICE_TYPE" == "motor" ]; then
                DETECTED_MOTOR_PORT="$port"
                echo -e "${GREEN}已将 ${port} 标识为电机控制器${NC}"
            fi
            
            # 如果已经找到舵机端口，继续检查剩余端口寻找电机端口
            if [ -n "$DETECTED_SERVO_PORT" ] && [ -z "$DETECTED_MOTOR_PORT" ]; then
                continue
            # 如果已经找到电机端口，继续检查剩余端口寻找舵机端口
            elif [ -z "$DETECTED_SERVO_PORT" ] && [ -n "$DETECTED_MOTOR_PORT" ]; then
                continue
            # 如果两种端口都已找到，可以停止搜索
            elif [ -n "$DETECTED_SERVO_PORT" ] && [ -n "$DETECTED_MOTOR_PORT" ]; then
                break
            fi
        fi
    done
    
    # 如果在USB设备中未找全部端口，测试ACM设备
    if [ -z "$DETECTED_SERVO_PORT" ] || [ -z "$DETECTED_MOTOR_PORT" ] && [ -n "${ACM_PORTS}" ]; then
        for port in ${ACM_PORTS}; do
            if test_servo_port "$port"; then
                if [ "$DETECTED_DEVICE_TYPE" == "servo" ] && [ -z "$DETECTED_SERVO_PORT" ]; then
                    DETECTED_SERVO_PORT="$port"
                    echo -e "${GREEN}已将 ${port} 标识为舵机控制器${NC}"
                elif [ "$DETECTED_DEVICE_TYPE" == "motor" ] && [ -z "$DETECTED_MOTOR_PORT" ]; then
                    DETECTED_MOTOR_PORT="$port"
                    echo -e "${GREEN}已将 ${port} 标识为电机控制器${NC}"
                fi
                
                # 如果两种端口都已找到，可以停止搜索
                if [ -n "$DETECTED_SERVO_PORT" ] && [ -n "$DETECTED_MOTOR_PORT" ]; then
                    break
                fi
            fi
        done
    fi
    
    # 4. 如果自动检测失败，提供手动选择
    if [ -z "$DETECTED_SERVO_PORT" ] || [ -z "$DETECTED_MOTOR_PORT" ]; then
        echo -e "${YELLOW}未自动检测到舵机或电机控制器${NC}"
        echo -e "${YELLOW}请从以下串口中手动选择:${NC}"
        
        # 显示编号的串口列表
        local i=1
        local ports_array=()
        for port in ${ALL_PORTS}; do
            ports_array[$i]=$port
            echo -e "${CYAN}$i) ${port}${NC}"
            i=$((i+1))
        done
        
        # 舵机端口选择
        if [ -z "$DETECTED_SERVO_PORT" ]; then
            echo -ne "${GREEN}请选择舵机控制串口 (1-$((i-1)), 或按Enter跳过): ${NC}"
            read -r port_choice
            
            # 验证输入
            if [[ "$port_choice" =~ ^[0-9]+$ ]] && [ "$port_choice" -ge 1 ] && [ "$port_choice" -lt "$i" ]; then
                DETECTED_SERVO_PORT="${ports_array[$port_choice]}"
                echo -e "${GREEN}已选择 ${DETECTED_SERVO_PORT} 作为舵机控制串口${NC}"
            else
                # 默认使用第一个串口
                DETECTED_SERVO_PORT=$(echo ${ALL_PORTS} | awk '{print $1}')
                echo -e "${YELLOW}使用默认串口: ${DETECTED_SERVO_PORT} 作为舵机控制串口${NC}"
            fi
        fi
        
        # 电机端口选择
        if [ -z "$DETECTED_MOTOR_PORT" ]; then
            echo -ne "${GREEN}请选择电机控制串口 (1-$((i-1)), 或按Enter跳过): ${NC}"
            read -r port_choice
            
            # 验证输入
            if [[ "$port_choice" =~ ^[0-9]+$ ]] && [ "$port_choice" -ge 1 ] && [ "$port_choice" -lt "$i" ]; then
                DETECTED_MOTOR_PORT="${ports_array[$port_choice]}"
                echo -e "${GREEN}已选择 ${DETECTED_MOTOR_PORT} 作为电机控制串口${NC}"
            else
                # 默认使用第一个串口
                DETECTED_MOTOR_PORT=$(echo ${ALL_PORTS} | awk '{print $1}')
                echo -e "${YELLOW}使用默认串口: ${DETECTED_MOTOR_PORT} 作为电机控制串口${NC}"
            fi
        fi
    fi
    
    # 5. 清理临时文件
    rm -rf "$TEMP_DIR" 2>/dev/null
    
    # 6. 更新launch文件配置
    if [ -n "$DETECTED_SERVO_PORT" ] && [ -n "$DETECTED_MOTOR_PORT" ]; then
        echo -e "${GREEN}将使用 ${DETECTED_SERVO_PORT} 作为舵机控制串口，${DETECTED_MOTOR_PORT} 作为电机控制串口${NC}"
        
        # 更新launch文件中的串口配置
        LAUNCH_FILE="$WORKSPACE_DIR/src/stereo_vision/launch/stereo_vision.launch"
        if [ -f "${LAUNCH_FILE}" ]; then
            # 备份原始文件
            cp "${LAUNCH_FILE}" "${LAUNCH_FILE}.bak" 2>/dev/null
            
            # 更新串口配置
            sed -i "s|<arg name=\"servo_port\" default=\"/dev/ttyUSB[0-9]*\"|<arg name=\"servo_port\" default=\"${DETECTED_SERVO_PORT}\"|g" "${LAUNCH_FILE}" 2>/dev/null
            sed -i "s|<arg name=\"servo_port\" default=\"/dev/ttyACM[0-9]*\"|<arg name=\"servo_port\" default=\"${DETECTED_SERVO_PORT}\"|g" "${LAUNCH_FILE}" 2>/dev/null
            
            # 添加或更新电机端口配置
            if grep -q "motor_port" "${LAUNCH_FILE}"; then
                # 如果已存在motor_port参数，则更新
            sed -i "s|<arg name=\"motor_port\" default=\"/dev/ttyUSB[0-9]*\"|<arg name=\"motor_port\" default=\"${DETECTED_MOTOR_PORT}\"|g" "${LAUNCH_FILE}" 2>/dev/null
            sed -i "s|<arg name=\"motor_port\" default=\"/dev/ttyACM[0-9]*\"|<arg name=\"motor_port\" default=\"${DETECTED_MOTOR_PORT}\"|g" "${LAUNCH_FILE}" 2>/dev/null
            else
                # 如果不存在motor_port参数，则添加
                sed -i "/<arg name=\"servo_port\"/a \  <arg name=\"motor_port\" default=\"${DETECTED_MOTOR_PORT}\" doc=\"电机控制串口（波特率115200）\" />" "${LAUNCH_FILE}" 2>/dev/null
            fi
            
            echo -e "${GREEN}已更新 ${LAUNCH_FILE} 中的串口配置${NC}"
            echo -e "${YELLOW}按任意键继续...${NC}"
            read -r -n 1
            return 0
        else
            echo -e "${YELLOW}未找到launch文件: ${LAUNCH_FILE}${NC}"
            echo -e "${YELLOW}按任意键继续...${NC}"
            read -r -n 1
            return 1
        fi
    else
        echo -e "${RED}未找到可用的串口设备${NC}"
        echo -e "${YELLOW}按任意键继续...${NC}"
        read -r -n 1
        return 1
    fi
}

# 创建ROS日志配置文件以减少不必要的输出
setup_log_config() {
    echo -e "${BLUE}=====================================${NC}"
    echo -e "${GREEN}配置ROS日志级别${NC}"
    echo -e "${BLUE}=====================================${NC}"
    
    # 创建日志配置目录
    LOG_CONFIG_DIR="$WORKSPACE_DIR/src/arm_trajectory/config"
    mkdir -p "$LOG_CONFIG_DIR"
    
    # 创建rosconsole.conf文件
    LOG_CONFIG_FILE="$LOG_CONFIG_DIR/rosconsole.conf"
    cat > "$LOG_CONFIG_FILE" << EOF
# ROS日志配置文件
# 设置默认日志级别为WARN，减少INFO级别的输出
log4j.logger.ros=WARN
log4j.logger.ros.roscpp=WARN
log4j.logger.ros.roscpp.superdebug=WARN

# 特定节点的日志级别
log4j.logger.ros.camera_node=WARN
log4j.logger.ros.arm_gui_node=WARN

# 允许ERROR级别的日志
log4j.logger.ros.servo=ERROR
log4j.logger.ros.visual_servo_controller=ERROR
log4j.logger.ros.trajectory_bridge=ERROR
log4j.logger.ros.path_planner=ERROR
log4j.logger.ros.vacuum_controller=ERROR
EOF
    
    echo -e "${GREEN}已创建日志配置文件: ${LOG_CONFIG_FILE}${NC}"
    echo -e "${BLUE}=====================================${NC}"
}

# 检查ROS环境
if [ -z "$ROS_DISTRO" ]; then
    echo -e "${RED}未检测到ROS环境，正在尝试加载...${NC}"
    if [ -f "/opt/ros/noetic/setup.bash" ]; then
        source /opt/ros/noetic/setup.bash
    else
        echo -e "${RED}无法找到ROS环境，请确保ROS已安装并手动加载环境${NC}"
        echo -e "${YELLOW}示例: source /opt/ros/noetic/setup.bash${NC}"
        exit 1
    fi
fi

# 检查工作空间
WORKSPACE_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
if [ -f "$WORKSPACE_DIR/devel/setup.bash" ]; then
    source "$WORKSPACE_DIR/devel/setup.bash"
    echo -e "${GREEN}已加载工作空间环境${NC}"
else
    echo -e "${YELLOW}警告: 未找到工作空间的setup.bash文件，将使用系统ROS环境${NC}"
fi

# 设置日志配置
setup_log_config

# 默认参数
YOLO_ENABLED=true
YOLO_MODEL=""
YOLO_THRESHOLD=0.5
STEREO_CAM_DEVICE="/dev/video0"
INITIAL_VIEW_MODE=0
VACUUM_ENABLED=true

# 命令行参数模式
COMMAND_MODE=false

# 帮助信息
show_help() {
    echo "用法: $0 [选项]"
    echo "选项:"
    echo "  -y, --yolo             启用YOLO目标检测"
    echo "  -m, --model PATH       指定YOLOv8模型路径"
    echo "  -t, --threshold VALUE  设置检测置信度阈值 (0.0-1.0)"
    echo "  -c, --camera DEVICE    指定摄像头设备 (默认: /dev/video0)"
    echo "  -v, --view-mode MODE   设置初始视图模式 (0=左, 1=右, 2=深度)"
    echo "  -n, --no-vacuum        禁用真空吸盘功能"
    echo "  -h, --help             显示此帮助信息"
    exit 0
}

# 解析命令行参数
while [[ $# -gt 0 ]]; do
    case $1 in
        -y|--yolo)
            YOLO_ENABLED=true
            COMMAND_MODE=true
            shift
            ;;
        -m|--model)
            YOLO_MODEL="$2"
            COMMAND_MODE=true
            shift 2
            ;;
        -t|--threshold)
            YOLO_THRESHOLD="$2"
            COMMAND_MODE=true
            shift 2
            ;;
        -c|--camera)
            STEREO_CAM_DEVICE="$2"
            COMMAND_MODE=true
            shift 2
            ;;
        -v|--view-mode)
            INITIAL_VIEW_MODE="$2"
            COMMAND_MODE=true
            shift 2
            ;;
        -n|--no-vacuum)
            VACUUM_ENABLED=false
            COMMAND_MODE=true
            shift
            ;;
        -h|--help)
            show_help
            ;;
        *)
            echo -e "${RED}未知选项: $1${NC}"
            show_help
            ;;
    esac
done

# 启动完整系统
launch_full_system() {
    echo -e "${GREEN}启动机械臂视觉控制系统...${NC}"
    
    if [ -n "$YOLO_MODEL" ]; then
        echo -e "${CYAN}使用自定义YOLOv8模型: $YOLO_MODEL${NC}"
    else
        echo -e "${CYAN}使用默认YOLOv8n模型${NC}"
    fi
    echo -e "${CYAN}检测置信度阈值: $YOLO_THRESHOLD${NC}"
    echo -e "${CYAN}摄像头设备: $STEREO_CAM_DEVICE${NC}"
    echo -e "${CYAN}初始视图模式: $INITIAL_VIEW_MODE${NC}"
    echo -e "${CYAN}真空吸盘功能: $([ "$VACUUM_ENABLED" = true ] && echo "启用" || echo "禁用")${NC}"
    
    # 构建启动命令
    LAUNCH_CMD="roslaunch stereo_vision stereo_vision.launch enable_detection:=$YOLO_ENABLED"
    
    # 添加额外参数
    if [ -n "$YOLO_MODEL" ]; then
        LAUNCH_CMD="$LAUNCH_CMD yolo_model_path:=$YOLO_MODEL"
    fi
    
    LAUNCH_CMD="$LAUNCH_CMD stereo_cam_device:=$STEREO_CAM_DEVICE initial_view_mode:=$INITIAL_VIEW_MODE vacuum_enabled:=$VACUUM_ENABLED"
    
    # 执行启动命令
    $LAUNCH_CMD
}

# 根据命令行参数启动
launch_system() {
    launch_full_system
}

# 显示简化菜单（如果没有命令行参数）
show_menu() {
    while true; do
        clear
        echo -e "${BLUE}=====================================${NC}"
        echo -e "${GREEN}机械臂视觉控制系统启动菜单${NC}"
        echo -e "${BLUE}=====================================${NC}"
        echo -e "${YELLOW}1. 启动视觉控制系统${NC}"
        echo -e "${YELLOW}2. 检测串口设备${NC}"
        echo -e "${YELLOW}3. 配置参数${NC}"
        echo -e "${YELLOW}4. 修复系统权限${NC}"
        echo -e "${YELLOW}0. 退出${NC}"
        echo -e "${BLUE}=====================================${NC}"
        echo -ne "${GREEN}请选择: ${NC}"
        read -r choice
        
        case $choice in
            1)
                launch_full_system
                ;;
            2)
                detect_ports
                ;;
            3)
                configure_params
                ;;
            4)
                fix_permissions
                ;;
            0)
                echo -e "${GREEN}退出程序${NC}"
                exit 0
                ;;
            *)
                echo -e "${RED}无效选择，请重试${NC}"
                sleep 1
                ;;
        esac
    done
}

# 配置参数
configure_params() {
    echo -e "${BLUE}=====================================${NC}"
    echo -e "${GREEN}配置参数${NC}"
    echo -e "${BLUE}=====================================${NC}"
    
    echo -e "${YELLOW}当前YOLOv8模型: $([ -n "$YOLO_MODEL" ] && echo "$YOLO_MODEL" || echo "默认")${NC}"
    echo -ne "${GREEN}输入YOLOv8模型路径 (留空使用默认模型): ${NC}"
    read -r new_model
    if [ -n "$new_model" ]; then
        YOLO_MODEL="$new_model"
    fi
    
    echo -e "${YELLOW}当前检测置信度阈值: $YOLO_THRESHOLD${NC}"
    echo -ne "${GREEN}输入新的置信度阈值 (0.0-1.0，留空保持不变): ${NC}"
    read -r new_threshold
    if [ -n "$new_threshold" ]; then
        YOLO_THRESHOLD="$new_threshold"
    fi
    
    echo -e "${YELLOW}当前摄像头设备: $STEREO_CAM_DEVICE${NC}"
    echo -ne "${GREEN}输入新的摄像头设备 (留空保持不变): ${NC}"
    read -r new_camera
    if [ -n "$new_camera" ]; then
        STEREO_CAM_DEVICE="$new_camera"
    fi
    
    echo -e "${YELLOW}当前视图模式: $INITIAL_VIEW_MODE${NC}"
    echo -ne "${GREEN}输入新的视图模式 (0=左, 1=右, 2=深度，留空保持不变): ${NC}"
    read -r new_view_mode
    if [ -n "$new_view_mode" ]; then
        INITIAL_VIEW_MODE="$new_view_mode"
    fi
    
    echo -e "${YELLOW}真空吸盘功能: $([ "$VACUUM_ENABLED" = true ] && echo "启用" || echo "禁用")${NC}"
    echo -ne "${GREEN}启用真空吸盘功能? (y/n，留空保持不变): ${NC}"
    read -r vacuum_choice
    if [ "$vacuum_choice" = "y" ] || [ "$vacuum_choice" = "Y" ]; then
        VACUUM_ENABLED=true
    elif [ "$vacuum_choice" = "n" ] || [ "$vacuum_choice" = "N" ]; then
        VACUUM_ENABLED=false
    fi
    
    echo -e "${BLUE}=====================================${NC}"
    echo -e "${GREEN}配置已更新${NC}"
    echo -e "${BLUE}=====================================${NC}"
    
    echo -e "${YELLOW}按任意键返回菜单...${NC}"
    read -r -n 1
}

# 主程序入口点
if [ "$COMMAND_MODE" = true ]; then
    # 使用命令行参数直接启动
    launch_system
else
    # 显示交互式菜单
    show_menu
fi