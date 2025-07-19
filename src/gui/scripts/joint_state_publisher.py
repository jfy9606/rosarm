#!/usr/bin/env python3

"""
ROS 2机械臂关节状态发布器主程序。
"""

import sys
import os
from pathlib import Path

# 添加Python包路径
package_path = str(Path(__file__).parent.parent)
if package_path not in sys.path:
    sys.path.insert(0, package_path)

try:
    import rclpy
    from src.gui_python.joint_state_publisher import JointStatePublisherNode
except ImportError:
    try:
        import rclpy
        from gui_python.joint_state_publisher import JointStatePublisherNode
    except ImportError:
        print("Error: Could not import required modules")
        sys.exit(1)


def main(args=None):
    rclpy.init(args=args)
    node = JointStatePublisherNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main() 