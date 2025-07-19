#!/usr/bin/env python3

"""
ROS 2机械臂关节状态发布器主程序。
"""

import sys
import os
from pathlib import Path

# 查找可能的Python包位置
def find_package_paths():
    # 获取当前脚本的绝对路径
    script_path = Path(__file__).resolve()
    # 如果是在安装目录中
    if 'install' in str(script_path):
        install_dir = script_path.parent.parent.parent
        paths = [
            # 1. 安装目录中的site-packages - 支持Python 3.11
            install_dir / "lib" / "python3.11" / "site-packages",
            # 2. 安装目录中的site-packages - 兼容Python 3.10
            install_dir / "lib" / "python3.10" / "site-packages",
            # 3. 相对于安装目录的源码目录
            install_dir.parent / "src" / "gui",
        ]
    # 如果是在源码目录中
    else:
        src_dir = script_path.parent.parent
        paths = [
            # 1. 源码目录
            src_dir,
            # 2. 源码目录的src子目录
            src_dir / "src",
            # 3. 安装目录 (如果已经构建)
            src_dir.parent.parent / "install" / "gui" / "lib" / "python3.11" / "site-packages",
        ]
    
    # 添加环境变量中的路径
    env_paths = os.environ.get('PYTHONPATH', '').split(os.pathsep)
    paths.extend([Path(p) for p in env_paths if p])
    
    # 过滤掉不存在的路径
    return [str(p) for p in paths if p.exists()]

# 添加找到的所有有效路径
for path in find_package_paths():
    if path not in sys.path:
        sys.path.insert(0, path)

# 导入ROS和必要的模块
try:
    import rclpy
    from gui_python.joint_state_publisher import JointStatePublisherNode
except ImportError as e:
    try:
        import rclpy
        from src.gui_python.joint_state_publisher import JointStatePublisherNode
    except ImportError:
        print(f"错误: 无法导入必要的模块: {e}")
        print(f"Python版本: {sys.version}")
        print(f"Python路径: {sys.path}")
        sys.exit(1)


def main(args=None):
    rclpy.init(args=args)
    node = JointStatePublisherNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main() 