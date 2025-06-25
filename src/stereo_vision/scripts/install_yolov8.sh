#!/bin/bash

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[0;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

echo -e "${BLUE}=============================${NC}"
echo -e "${GREEN}YOLOv8依赖安装脚本${NC}"
echo -e "${BLUE}=============================${NC}"

# 检查Python版本
PYTHON_VERSION=$(python3 --version 2>&1)
echo -e "${BLUE}当前Python版本: ${PYTHON_VERSION}${NC}"

# 创建虚拟环境目录
VENV_DIR="$HOME/yolov8_venv"
echo -e "${YELLOW}将在以下位置创建虚拟环境: ${VENV_DIR}${NC}"

# 检查是否已存在虚拟环境
if [ -d "$VENV_DIR" ]; then
    echo -e "${YELLOW}虚拟环境已存在，是否重新创建? [y/N]${NC}"
    read -r answer
    if [[ "$answer" =~ ^[Yy]$ ]]; then
        echo -e "${YELLOW}移除现有虚拟环境...${NC}"
        rm -rf "$VENV_DIR"
    else
        echo -e "${GREEN}将使用现有虚拟环境${NC}"
    fi
fi

# 创建虚拟环境
if [ ! -d "$VENV_DIR" ]; then
    echo -e "${BLUE}创建Python虚拟环境...${NC}"
    python3 -m venv "$VENV_DIR"
    if [ $? -ne 0 ]; then
        echo -e "${RED}创建虚拟环境失败，尝试安装python3-venv${NC}"
        sudo apt-get update
        sudo apt-get install -y python3-venv
        python3 -m venv "$VENV_DIR"
        if [ $? -ne 0 ]; then
            echo -e "${RED}无法创建虚拟环境，退出${NC}"
            exit 1
        fi
    fi
fi

# 激活虚拟环境
echo -e "${BLUE}激活虚拟环境...${NC}"
source "$VENV_DIR/bin/activate"

# 检查是否成功激活
if [[ "$VIRTUAL_ENV" != "$VENV_DIR" ]]; then
    echo -e "${RED}虚拟环境激活失败，退出${NC}"
    exit 1
fi

# 升级pip
echo -e "${BLUE}升级pip...${NC}"
pip install --upgrade pip

# 安装YOLOv8依赖
echo -e "${BLUE}安装YOLOv8及其依赖...${NC}"
pip install ultralytics

# 安装其他ROS相关依赖
echo -e "${BLUE}安装其他ROS相关Python包...${NC}"
pip install rospkg catkin_pkg empy

# 验证安装
echo -e "${BLUE}验证YOLOv8安装...${NC}"
python -c "from ultralytics import YOLO; print('YOLOv8已成功安装')"
if [ $? -ne 0 ]; then
    echo -e "${RED}YOLOv8安装验证失败${NC}"
else
    echo -e "${GREEN}YOLOv8已成功安装${NC}"
fi

# 添加激活脚本到workspace
WORKSPACE_DIR=$(dirname $(dirname $(dirname $(readlink -f "$0"))))
ACTIVATE_SCRIPT="$WORKSPACE_DIR/activate_yolov8.sh"

echo -e "${BLUE}创建激活脚本: ${ACTIVATE_SCRIPT}${NC}"

cat > "$ACTIVATE_SCRIPT" << EOF
#!/bin/bash
# 激活YOLOv8虚拟环境
source "$VENV_DIR/bin/activate"
echo "YOLOv8虚拟环境已激活"
EOF

chmod +x "$ACTIVATE_SCRIPT"

# 修改start.sh脚本，添加自动激活虚拟环境
START_SCRIPT="$WORKSPACE_DIR/start.sh"
if [ -f "$START_SCRIPT" ]; then
    echo -e "${BLUE}更新启动脚本...${NC}"
    
    # 检查是否已包含激活代码
    if grep -q "activate_yolov8.sh" "$START_SCRIPT"; then
        echo -e "${YELLOW}启动脚本已包含虚拟环境激活代码${NC}"
    else
        # 在ROS环境加载后添加虚拟环境激活
        TEMP_FILE=$(mktemp)
        awk -v venv="$ACTIVATE_SCRIPT" '
            /^fi$/ && !done {
                print $0
                print ""
                print "# 激活YOLOv8虚拟环境"
                print "if [ -f \"" venv "\" ]; then"
                print "    source \"" venv "\""
                print "    echo -e \"${GREEN}已激活YOLOv8虚拟环境${NC}\""
                print "else"
                print "    echo -e \"${YELLOW}警告: YOLOv8虚拟环境激活脚本未找到${NC}\""
                print "fi"
                print ""
                done=1
                next
            }
            {print}
        ' "$START_SCRIPT" > "$TEMP_FILE"
        
        mv "$TEMP_FILE" "$START_SCRIPT"
        chmod +x "$START_SCRIPT"
        echo -e "${GREEN}启动脚本已更新${NC}"
    fi
fi

echo -e "${BLUE}=============================${NC}"
echo -e "${GREEN}安装完成${NC}"
echo -e "${YELLOW}使用方法:${NC}"
echo -e "1. 使用 ${GREEN}source $ACTIVATE_SCRIPT${NC} 激活YOLOv8环境"
echo -e "2. 或直接运行 ${GREEN}./start.sh${NC} 启动系统"
echo -e "${BLUE}=============================${NC}"

# 提示退出虚拟环境
echo -e "${YELLOW}输入 'deactivate' 可退出虚拟环境${NC}" 