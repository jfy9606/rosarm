#!/bin/bash
# 摄像头启动脚本

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[0;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
NC='\033[0m' # No Color

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

# 检测摄像头
echo -e "${BLUE}正在检测摄像头设备...${NC}"
VIDEO_DEVICES=$(ls /dev/video* 2>/dev/null | sort)
VIDEO_COUNT=$(echo "$VIDEO_DEVICES" | wc -l)

if [ "$VIDEO_COUNT" -eq 0 ]; then
    echo -e "${RED}未检测到任何摄像头设备${NC}"
    echo -e "${YELLOW}请确保摄像头已连接并有正确权限${NC}"
    HAS_CAMERA=false
else
    echo -e "${GREEN}检测到以下摄像头设备:${NC}"
    echo "$VIDEO_DEVICES"
    CAMERA=$(echo "$VIDEO_DEVICES" | head -n 1)
    echo -e "${GREEN}将使用设备: ${CAMERA}${NC}"
    HAS_CAMERA=true
fi

# 默认参数
CAMERA=${CAMERA:-"/dev/video0"}
RESOLUTION="1280x480"
FPS=30
YOLO_ENABLED=false

# 命令行参数模式
COMMAND_MODE=false
DIRECT_MODE=false
TEST_MODE=false

# 帮助信息
show_help() {
    echo "用法: $0 [选项]"
    echo "选项:"
    echo "  -c, --camera PATH      摄像头设备路径 (默认: 自动检测)"
    echo "  -r, --resolution WxH   分辨率 (默认: 1280x480)"
    echo "  -f, --fps NUMBER       帧率 (默认: 30)"
    echo "  -y, --yolo             启用YOLO目标检测"
    echo "  -d, --direct           直接模式，不使用ROS"
    echo "  -t, --test             仅测试摄像头"
    echo "  -h, --help             显示此帮助信息"
    exit 0
}

# 解析命令行参数
while [[ $# -gt 0 ]]; do
    case $1 in
        -c|--camera)
            CAMERA="$2"
            COMMAND_MODE=true
            shift 2
            ;;
        -r|--resolution)
            RESOLUTION="$2"
            COMMAND_MODE=true
            shift 2
            ;;
        -f|--fps)
            FPS="$2"
            COMMAND_MODE=true
            shift 2
            ;;
        -y|--yolo)
            YOLO_ENABLED=true
            COMMAND_MODE=true
            shift
            ;;
        -d|--direct)
            DIRECT_MODE=true
            COMMAND_MODE=true
            shift
            ;;
        -t|--test)
            TEST_MODE=true
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

# 解析分辨率
WIDTH=$(echo $RESOLUTION | cut -d'x' -f1)
HEIGHT=$(echo $RESOLUTION | cut -d'x' -f2)

# 根据命令行参数启动
launch_system() {
    if [ "$TEST_MODE" = true ]; then
        echo -e "${GREEN}测试摄像头模式${NC}"
        # 使用Python直接测试摄像头
        python3 src/stereo_vision/scripts/camera_direct.py --device 0 --width $WIDTH --height $HEIGHT --fps $FPS
    elif [ "$DIRECT_MODE" = true ]; then
        echo -e "${GREEN}直接模式，不使用ROS${NC}"
        # 启动直接摄像头模式
        roslaunch launch/direct_camera.launch
    else
        echo -e "${GREEN}启动ROS系统${NC}"
        echo -e "${BLUE}摄像头: $CAMERA, 分辨率: ${WIDTH}x${HEIGHT}, FPS: $FPS, YOLO: $YOLO_ENABLED${NC}"
        if [ "$YOLO_ENABLED" = true ]; then
            echo -e "${CYAN}YOLO目标检测初始状态: 启用${NC}"
        else
            echo -e "${CYAN}YOLO目标检测初始状态: 禁用${NC}"
        fi
        echo -e "${CYAN}注意: 启动后，可以在GUI中随时开关YOLO检测功能${NC}"
        
        # 启动ROS节点
        roslaunch launch/demo.launch camera_device:=$CAMERA yolo_enabled:=$YOLO_ENABLED
    fi
}

# 显示菜单（如果没有命令行参数）
show_menu() {
    echo -e "${BLUE}=====================================${NC}"
    echo -e "${GREEN}机械臂视觉控制系统启动菜单${NC}"
    echo -e "${BLUE}=====================================${NC}"
    echo -e "${YELLOW}1. 启动完整系统${NC}"
    echo -e "${YELLOW}2. 启动系统 (YOLO初始状态: 启用)${NC}"
    echo -e "${YELLOW}3. 测试摄像头${NC}"
    echo -e "${YELLOW}4. 直接模式 (不使用ROS)${NC}"
    echo -e "${YELLOW}5. 配置参数${NC}"
    echo -e "${YELLOW}0. 退出${NC}"
    echo -e "${BLUE}=====================================${NC}"
    echo -e "${CYAN}注意: 在系统运行后，可以通过GUI控制面板随时开关YOLO检测功能${NC}"
    echo -e "${BLUE}=====================================${NC}"
    echo -ne "${GREEN}请选择: ${NC}"
    read -r choice
    
    case $choice in
        1)
            echo -e "${GREEN}启动完整系统...${NC}"
            YOLO_ENABLED=false
            launch_system
            ;;
        2)
            echo -e "${GREEN}启动系统 (YOLO初始状态: 启用)...${NC}"
            YOLO_ENABLED=true
            launch_system
            ;;
        3)
            echo -e "${GREEN}测试摄像头...${NC}"
            TEST_MODE=true
            launch_system
            ;;
        4)
            echo -e "${GREEN}直接模式...${NC}"
            DIRECT_MODE=true
            launch_system
            ;;
        5)
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
    
    echo -e "${YELLOW}当前摄像头: $CAMERA${NC}"
    echo -ne "${GREEN}输入新的摄像头设备 (留空保持不变): ${NC}"
    read -r new_camera
    if [ -n "$new_camera" ]; then
        CAMERA=$new_camera
    fi
    
    echo -e "${YELLOW}当前分辨率: ${WIDTH}x${HEIGHT}${NC}"
    echo -ne "${GREEN}输入新的分辨率 (格式: 宽x高，留空保持不变): ${NC}"
    read -r new_resolution
    if [ -n "$new_resolution" ]; then
        RESOLUTION=$new_resolution
        WIDTH=$(echo $RESOLUTION | cut -d'x' -f1)
        HEIGHT=$(echo $RESOLUTION | cut -d'x' -f2)
    fi
    
    echo -e "${YELLOW}当前帧率: $FPS${NC}"
    echo -ne "${GREEN}输入新的帧率 (留空保持不变): ${NC}"
    read -r new_fps
    if [ -n "$new_fps" ]; then
        FPS=$new_fps
    fi
    
    echo -e "${YELLOW}YOLO目标检测: $([ "$YOLO_ENABLED" = true ] && echo "启用" || echo "禁用")${NC}"
    echo -ne "${GREEN}是否启用YOLO目标检测 (y/n，留空保持不变): ${NC}"
    read -r yolo_choice
    if [ "$yolo_choice" = "y" ] || [ "$yolo_choice" = "Y" ]; then
        YOLO_ENABLED=true
    elif [ "$yolo_choice" = "n" ] || [ "$yolo_choice" = "N" ]; then
        YOLO_ENABLED=false
    fi
    
    echo -e "${GREEN}参数已更新${NC}"
}

# 主程序入口点
if [ "$COMMAND_MODE" = true ]; then
    # 使用命令行参数直接启动
    launch_system
else
    # 显示交互式菜单
    show_menu
fi