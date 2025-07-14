#!/usr/bin/env python3

import rclpy
from trajectory.trajectory_planner import TrajectoryPlannerNode

def main():
    rclpy.init()
    node = TrajectoryPlannerNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main() 