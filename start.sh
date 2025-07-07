#!/bin/bash
# 机械臂控制系统启动脚本

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[0;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
NC='\033[0m' # No Color

# 获取工作空间目录
WORKSPACE_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

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
        
        # 使用sudo修复权限
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
        echo -e "${YELLOW}按任意键继续...${NC}"
        read -r -n 1
        return 1
    fi
    
    # 显示检测到的串口
    echo -e "${GREEN}检测到 ${PORT_COUNT} 个串口设备:${NC}"
    for port in ${ALL_PORTS}; do
        echo -e "${CYAN}- ${port}${NC}"
    done
    
    # 智能检测舵机串口和电机串口
    echo -e "\n${GREEN}开始智能检测串口设备类型...${NC}"
    
    # 默认设置
    DETECTED_SERVO_PORT=""
    DETECTED_MOTOR_PORT=""
    
    # 如果只有一个串口设备，将其同时用作舵机和电机控制
    if [ "${PORT_COUNT}" -eq 1 ]; then
        DETECTED_SERVO_PORT=$(echo ${ALL_PORTS} | awk '{print $1}')
        DETECTED_MOTOR_PORT=${DETECTED_SERVO_PORT}
        echo -e "${YELLOW}只检测到一个串口设备，将用于舵机和电机控制: ${DETECTED_SERVO_PORT}${NC}"
    else
        # 如果有多个串口设备，假设第一个是电机控制，第二个是舵机控制
        DETECTED_MOTOR_PORT=$(echo ${ALL_PORTS} | awk '{print $1}')
        DETECTED_SERVO_PORT=$(echo ${ALL_PORTS} | awk '{print $2}')
        
        if [ -z "${DETECTED_SERVO_PORT}" ]; then
            DETECTED_SERVO_PORT=${DETECTED_MOTOR_PORT}
        fi
        
        echo -e "${GREEN}检测到多个串口设备:${NC}"
        echo -e "${CYAN}电机控制串口: ${DETECTED_MOTOR_PORT}${NC}"
        echo -e "${CYAN}舵机控制串口: ${DETECTED_SERVO_PORT}${NC}"
        
        # 提供手动选择选项
        echo -e "\n${YELLOW}是否要手动选择串口? (y/n)${NC}"
        read -r choice
        
        if [ "$choice" = "y" ] || [ "$choice" = "Y" ]; then
            # 显示编号的串口列表
            local i=1
            local ports_array=()
            for port in ${ALL_PORTS}; do
                ports_array[$i]=$port
                echo -e "${CYAN}$i) ${port}${NC}"
                i=$((i+1))
            done
            
            # 电机端口选择
            echo -ne "${GREEN}请选择电机控制串口 (1-$((i-1))): ${NC}"
            read -r port_choice
            
            # 验证输入
            if [[ "$port_choice" =~ ^[0-9]+$ ]] && [ "$port_choice" -ge 1 ] && [ "$port_choice" -lt "$i" ]; then
                DETECTED_MOTOR_PORT="${ports_array[$port_choice]}"
                echo -e "${GREEN}已选择 ${DETECTED_MOTOR_PORT} 作为电机控制串口${NC}"
            fi
            
            # 舵机端口选择
            echo -ne "${GREEN}请选择舵机控制串口 (1-$((i-1))): ${NC}"
            read -r port_choice
            
            # 验证输入
            if [[ "$port_choice" =~ ^[0-9]+$ ]] && [ "$port_choice" -ge 1 ] && [ "$port_choice" -lt "$i" ]; then
                DETECTED_SERVO_PORT="${ports_array[$port_choice]}"
                echo -e "${GREEN}已选择 ${DETECTED_SERVO_PORT} 作为舵机控制串口${NC}"
            fi
        fi
    fi
    
    # 更新launch文件配置
    update_launch_files
    
    echo -e "${BLUE}=====================================${NC}"
    echo -e "${GREEN}串口配置完成，按任意键继续...${NC}"
    read -r -n 1
}

# 更新launch文件配置
update_launch_files() {
    if [ -n "$DETECTED_SERVO_PORT" ] && [ -n "$DETECTED_MOTOR_PORT" ]; then
        echo -e "${GREEN}更新launch文件配置...${NC}"
        
        # 更新motor_control.launch
        MOTOR_LAUNCH="$WORKSPACE_DIR/src/motor/launch/motor_control.launch"
        if [ -f "${MOTOR_LAUNCH}" ]; then
            # 备份原始文件
            cp "${MOTOR_LAUNCH}" "${MOTOR_LAUNCH}.bak" 2>/dev/null
            
            # 更新串口配置
            sed -i "s|<arg name=\"port\" default=\"/dev/ttyUSB[0-9]*\"|<arg name=\"port\" default=\"${DETECTED_MOTOR_PORT}\"|g" "${MOTOR_LAUNCH}" 2>/dev/null
            sed -i "s|<arg name=\"port\" default=\"/dev/ttyACM[0-9]*\"|<arg name=\"port\" default=\"${DETECTED_MOTOR_PORT}\"|g" "${MOTOR_LAUNCH}" 2>/dev/null
            
            echo -e "${GREEN}已更新 ${MOTOR_LAUNCH} 中的串口配置${NC}"
        fi
        
        # 更新servo_control.launch
        SERVO_LAUNCH="$WORKSPACE_DIR/src/servo/launch/servo_control.launch"
        if [ -f "${SERVO_LAUNCH}" ]; then
            # 备份原始文件
            cp "${SERVO_LAUNCH}" "${SERVO_LAUNCH}.bak" 2>/dev/null
            
            # 更新串口配置
            sed -i "s|<arg name=\"port\" default=\"/dev/ttyUSB[0-9]*\"|<arg name=\"port\" default=\"${DETECTED_SERVO_PORT}\"|g" "${SERVO_LAUNCH}" 2>/dev/null
            sed -i "s|<arg name=\"port\" default=\"/dev/ttyACM[0-9]*\"|<arg name=\"port\" default=\"${DETECTED_SERVO_PORT}\"|g" "${SERVO_LAUNCH}" 2>/dev/null
            
            echo -e "${GREEN}已更新 ${SERVO_LAUNCH} 中的串口配置${NC}"
        fi
        
        # 更新vacuum_control.launch
        VACUUM_LAUNCH="$WORKSPACE_DIR/src/servo/launch/vacuum_control.launch"
        if [ -f "${VACUUM_LAUNCH}" ]; then
            # 备份原始文件
            cp "${VACUUM_LAUNCH}" "${VACUUM_LAUNCH}.bak" 2>/dev/null
            
            # 更新串口配置
            sed -i "s|<arg name=\"port\" default=\"/dev/ttyUSB[0-9]*\"|<arg name=\"port\" default=\"${DETECTED_SERVO_PORT}\"|g" "${VACUUM_LAUNCH}" 2>/dev/null
            sed -i "s|<arg name=\"port\" default=\"/dev/ttyACM[0-9]*\"|<arg name=\"port\" default=\"${DETECTED_SERVO_PORT}\"|g" "${VACUUM_LAUNCH}" 2>/dev/null
            
            echo -e "${GREEN}已更新 ${VACUUM_LAUNCH} 中的串口配置${NC}"
        fi
        
        # 更新主launch文件
        MAIN_LAUNCH="$WORKSPACE_DIR/launch/arm_control.launch"
        if [ -f "${MAIN_LAUNCH}" ]; then
            # 备份原始文件
            cp "${MAIN_LAUNCH}" "${MAIN_LAUNCH}.bak" 2>/dev/null
            
            # 更新串口配置
            sed -i "s|<arg name=\"motor_port\" default=\"/dev/ttyUSB[0-9]*\"|<arg name=\"motor_port\" default=\"${DETECTED_MOTOR_PORT}\"|g" "${MAIN_LAUNCH}" 2>/dev/null
            sed -i "s|<arg name=\"motor_port\" default=\"/dev/ttyACM[0-9]*\"|<arg name=\"motor_port\" default=\"${DETECTED_MOTOR_PORT}\"|g" "${MAIN_LAUNCH}" 2>/dev/null
            sed -i "s|<arg name=\"servo_port\" default=\"/dev/ttyUSB[0-9]*\"|<arg name=\"servo_port\" default=\"${DETECTED_SERVO_PORT}\"|g" "${MAIN_LAUNCH}" 2>/dev/null
            sed -i "s|<arg name=\"servo_port\" default=\"/dev/ttyACM[0-9]*\"|<arg name=\"servo_port\" default=\"${DETECTED_SERVO_PORT}\"|g" "${MAIN_LAUNCH}" 2>/dev/null
            
            echo -e "${GREEN}已更新 ${MAIN_LAUNCH} 中的串口配置${NC}"
        fi
    else
        echo -e "${RED}未找到可用的串口设备，无法更新配置${NC}"
    fi
}

# 检查ROS环境
check_ros_env() {
    echo -e "${BLUE}=====================================${NC}"
    echo -e "${GREEN}检查ROS环境${NC}"
    echo -e "${BLUE}=====================================${NC}"
    
    if [ -z "$ROS_DISTRO" ]; then
        echo -e "${RED}未检测到ROS环境，正在尝试加载...${NC}"
        if [ -f "/opt/ros/noetic/setup.bash" ]; then
            source /opt/ros/noetic/setup.bash
            echo -e "${GREEN}已加载ROS环境${NC}"
        else
            echo -e "${RED}无法找到ROS环境，请确保ROS已安装并手动加载环境${NC}"
            echo -e "${YELLOW}示例: source /opt/ros/noetic/setup.bash${NC}"
            return 1
        fi
    else
        echo -e "${GREEN}已检测到ROS环境: ${ROS_DISTRO}${NC}"
    fi
    
    # 检查工作空间
    if [ -f "$WORKSPACE_DIR/devel/setup.bash" ]; then
        source "$WORKSPACE_DIR/devel/setup.bash"
        echo -e "${GREEN}已加载工作空间环境${NC}"
    else
        echo -e "${YELLOW}警告: 未找到工作空间的setup.bash文件，将使用系统ROS环境${NC}"
    fi
    
    return 0
}

# 启动完整系统
launch_full_system() {
    echo -e "${BLUE}=====================================${NC}"
    echo -e "${GREEN}启动机械臂控制系统${NC}"
    echo -e "${BLUE}=====================================${NC}"
    
    # 检查主launch文件是否存在
    MAIN_LAUNCH="$WORKSPACE_DIR/launch/arm_control.launch"
    if [ ! -f "${MAIN_LAUNCH}" ]; then
        echo -e "${RED}未找到主launch文件: ${MAIN_LAUNCH}${NC}"
        echo -e "${YELLOW}请确保文件存在并且路径正确${NC}"
        return 1
    fi
    
    # 启动系统
    echo -e "${GREEN}正在启动系统，请稍候...${NC}"
    roslaunch $MAIN_LAUNCH
}

# 显示菜单
show_menu() {
    while true; do
        clear
        echo -e "${BLUE}=====================================${NC}"
        echo -e "${GREEN}机械臂控制系统启动菜单${NC}"
        echo -e "${BLUE}=====================================${NC}"
        echo -e "${YELLOW}1. 启动完整系统${NC}"
        echo -e "${YELLOW}2. 检测串口设备${NC}"
        echo -e "${YELLOW}3. 修复设备权限${NC}"
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

# 主程序入口点
check_ros_env || exit 1
show_menu