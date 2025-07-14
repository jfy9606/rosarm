#!/usr/bin/env python3

import rclpy
from trajectory.path_planner import PathPlannerNode

def main():
    rclpy.init()
    node = PathPlannerNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main() 