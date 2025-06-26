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

# 默认摄像头设备
DEFAULT_CAMERA="/dev/video0"

if [ "$VIDEO_COUNT" -eq 0 ]; then
    echo -e "${YELLOW}未检测到任何摄像头设备，将使用默认设备: ${DEFAULT_CAMERA}${NC}"
    echo -e "${YELLOW}请确保摄像头已连接并有正确权限${NC}"
    CAMERA=$DEFAULT_CAMERA
    HAS_CAMERA=false
else
    echo -e "${GREEN}检测到以下摄像头设备:${NC}"
    echo "$VIDEO_DEVICES"
    CAMERA=$(echo "$VIDEO_DEVICES" | head -n 1)
    echo -e "${GREEN}将使用设备: ${CAMERA}${NC}"
    HAS_CAMERA=true
fi

# 默认参数
RESOLUTION="1280x480"
FPS=30
YOLO_ENABLED=true
YOLO_MODEL=""
YOLO_THRESHOLD=0.5

# 命令行参数模式
COMMAND_MODE=false
TEST_MODE=false

# 帮助信息
show_help() {
    echo "用法: $0 [选项]"
    echo "选项:"
    echo "  -c, --camera PATH      摄像头设备路径 (默认: 自动检测)"
    echo "  -r, --resolution WxH   分辨率 (默认: 1280x480)"
    echo "  -f, --fps NUMBER       帧率 (默认: 30)"
    echo "  -y, --yolo             启用YOLO目标检测"
    echo "  -m, --model PATH       指定YOLOv8模型路径"
    echo "  -t, --threshold VALUE  设置检测置信度阈值 (0.0-1.0)"
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

# 启动完整系统
launch_full_system() {
    echo -e "${GREEN}启动完整视觉控制系统...${NC}"
    echo -e "${BLUE}摄像头: $CAMERA, 分辨率: ${WIDTH}x${HEIGHT}, FPS: $FPS${NC}"
    echo -e "${CYAN}YOLO目标检测: 在GUI中控制开关${NC}"
    
    if [ -n "$YOLO_MODEL" ]; then
        echo -e "${CYAN}使用自定义YOLOv8模型: $YOLO_MODEL${NC}"
    else
        echo -e "${CYAN}使用默认YOLOv8s模型${NC}"
    fi
    echo -e "${CYAN}检测置信度阈值: $YOLO_THRESHOLD${NC}"
    
    # 构建启动命令
    LAUNCH_CMD="roslaunch launch/demo.launch camera_device:=$CAMERA yolo_enabled:=$YOLO_ENABLED"
    
    # 添加额外参数
    if [ -n "$YOLO_MODEL" ]; then
        LAUNCH_CMD="$LAUNCH_CMD yolo_model_path:=$YOLO_MODEL"
    fi
    
    LAUNCH_CMD="$LAUNCH_CMD yolo_confidence:=$YOLO_THRESHOLD"
    
    # 执行启动命令
    $LAUNCH_CMD
}

# 测试摄像头
test_camera() {
    echo -e "${GREEN}测试摄像头...${NC}"
    # 使用Python直接测试摄像头
    python3 src/stereo_vision/scripts/camera_direct.py --device 0 --width $WIDTH --height $HEIGHT --fps $FPS
}

# 根据命令行参数启动
launch_system() {
    if [ "$TEST_MODE" = true ]; then
        test_camera
    else
        launch_full_system
    fi
}

# 显示简化菜单（如果没有命令行参数）
show_menu() {
    echo -e "${BLUE}=====================================${NC}"
    echo -e "${GREEN}机械臂视觉控制系统启动菜单${NC}"
    echo -e "${BLUE}=====================================${NC}"
    echo -e "${YELLOW}1. 启动视觉控制系统${NC}"
    echo -e "${YELLOW}2. 测试摄像头${NC}"
    echo -e "${YELLOW}3. 配置参数${NC}"
    echo -e "${YELLOW}0. 退出${NC}"
    echo -e "${BLUE}=====================================${NC}"
    echo -e "${CYAN}注意: 系统包含路径规划和3D视图功能${NC}"
    echo -e "${CYAN}      可以通过GUI控制面板随时开关YOLO检测${NC}"
    echo -e "${BLUE}=====================================${NC}"
    echo -ne "${GREEN}请选择: ${NC}"
    read -r choice
    
    case $choice in
        1)
            launch_full_system
            ;;
        2)
            test_camera
            ;;
        3)
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
        CAMERA="$new_camera"
    fi
    
    echo -e "${YELLOW}当前分辨率: $RESOLUTION${NC}"
    echo -ne "${GREEN}输入新的分辨率，格式为 宽x高 (留空保持不变): ${NC}"
    read -r new_resolution
    if [ -n "$new_resolution" ]; then
        RESOLUTION="$new_resolution"
        WIDTH=$(echo $RESOLUTION | cut -d'x' -f1)
        HEIGHT=$(echo $RESOLUTION | cut -d'x' -f2)
    fi
    
    echo -e "${YELLOW}当前帧率: $FPS${NC}"
    echo -ne "${GREEN}输入新的帧率 (留空保持不变): ${NC}"
    read -r new_fps
    if [ -n "$new_fps" ]; then
        FPS="$new_fps"
    fi
    
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