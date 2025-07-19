#!/usr/bin/env python3

"""
ROS 2机械臂控制GUI主程序。
使用Tkinter创建轻量级GUI界面。
"""

import sys
import os
from pathlib import Path

# 添加Python包路径
package_path = str(Path(__file__).parent.parent)
if package_path not in sys.path:
    sys.path.insert(0, package_path)

try:
    from src.gui_python.gui_app import create_and_run
except ImportError:
    try:
        from gui_python.gui_app import create_and_run
    except ImportError:
        print("Error: Could not import gui_app module")
        sys.exit(1)


def main(args=None):
    create_and_run(args)


if __name__ == "__main__":
    main() 