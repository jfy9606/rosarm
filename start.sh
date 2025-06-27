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
}

# 执行权限修复
fix_permissions

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

# 默认参数
YOLO_ENABLED=true
YOLO_MODEL=""
YOLO_THRESHOLD=0.5

# 命令行参数模式
COMMAND_MODE=false

# 帮助信息
show_help() {
    echo "用法: $0 [选项]"
    echo "选项:"
    echo "  -y, --yolo             启用YOLO目标检测"
    echo "  -m, --model PATH       指定YOLOv8模型路径"
    echo "  -t, --threshold VALUE  设置检测置信度阈值 (0.0-1.0)"
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
    echo -e "${CYAN}YOLO目标检测: 在GUI中控制开关${NC}"
    
    if [ -n "$YOLO_MODEL" ]; then
        echo -e "${CYAN}使用自定义YOLOv8模型: $YOLO_MODEL${NC}"
    else
        echo -e "${CYAN}使用默认YOLOv8n模型${NC}"
    fi
    echo -e "${CYAN}检测置信度阈值: $YOLO_THRESHOLD${NC}"
    
    # 构建启动命令
    LAUNCH_CMD="roslaunch arm_trajectory visual_servo.launch yolo_enabled:=$YOLO_ENABLED"
    
    # 添加额外参数
    if [ -n "$YOLO_MODEL" ]; then
        LAUNCH_CMD="$LAUNCH_CMD yolo_model_path:=$YOLO_MODEL"
    fi
    
    LAUNCH_CMD="$LAUNCH_CMD yolo_confidence:=$YOLO_THRESHOLD"
    
    # 执行启动命令
    $LAUNCH_CMD
}

# 根据命令行参数启动
launch_system() {
    launch_full_system
}

# 显示简化菜单（如果没有命令行参数）
show_menu() {
    echo -e "${BLUE}=====================================${NC}"
    echo -e "${GREEN}机械臂视觉控制系统启动菜单${NC}"
    echo -e "${BLUE}=====================================${NC}"
    echo -e "${YELLOW}1. 启动视觉控制系统${NC}"
    echo -e "${YELLOW}2. 配置参数${NC}"
    echo -e "${YELLOW}0. 退出${NC}"
    echo -e "${BLUE}=====================================${NC}"
    echo -e "${CYAN}功能说明:${NC}"
    echo -e "${CYAN}- 系统使用机械臂末端双目摄像头${NC}"
    echo -e "${CYAN}- 支持直接输入三维坐标控制机械臂末端位置${NC}"
    echo -e "${CYAN}- 支持使用YOLO检测物体并移动到物体位置${NC}"
    echo -e "${BLUE}=====================================${NC}"
    echo -ne "${GREEN}请选择: ${NC}"
    read -r choice
    
    case $choice in
        1)
            launch_full_system
            ;;
        2)
            configure_params
            ;;
        0)
            echo -e "${GREEN}退出程序${NC}"
            exit 0
            ;;
        *)
            echo -e "${RED}无效选择，请重试${NC}"
            ;;
    esac
    
    echo -e "${YELLOW}按任意键返回菜单...${NC}"
    read -r -n 1
    show_menu
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
    
    echo -e "${BLUE}=====================================${NC}"
    echo -e "${GREEN}配置已更新${NC}"
    echo -e "${BLUE}=====================================${NC}"
}

# 主程序入口点
if [ "$COMMAND_MODE" = true ]; then
    # 使用命令行参数直接启动
    launch_system
else
    # 显示交互式菜单
    show_menu
fi