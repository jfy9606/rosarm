#!/usr/bin/env python3
# 从父目录的whales_optimizer.py导入类和函数

import os
import sys

# 获取当前文件所在的目录
current_dir = os.path.dirname(os.path.abspath(__file__))

# 获取父目录路径
parent_dir = os.path.dirname(current_dir)

# 导入父目录中的whales_optimizer.py文件
sys.path.append(parent_dir)

# 从whales_optimizer.py导入所需类和函数
try:
    from whales_optimizer import WhaleOptimizer, forward_kinematics_dh
    # 导出这些类和函数，使它们可以通过模块直接访问
    __all__ = ['WhaleOptimizer', 'forward_kinematics_dh']
except ImportError as e:
    print(f"从whales_optimizer.py导入失败: {e}")
