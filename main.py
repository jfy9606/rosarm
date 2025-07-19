#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
机械臂控制程序入口
直接启动Tkinter GUI
"""

import sys
import os
from robot_arm_gui import main

if __name__ == "__main__":
    # 确保当前目录在sys.path中
    script_dir = os.path.dirname(os.path.abspath(__file__))
    if script_dir not in sys.path:
        sys.path.insert(0, script_dir)
        
    # 启动GUI
    main() 