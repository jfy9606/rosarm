# arm_trajectory scripts包初始化
import rospy

# 导入鲸鱼优化器模块
try:
    from .whales_optimizer import WhaleOptimizer, forward_kinematics_dh
except ImportError as e:
    rospy.logerr(f"导入WhaleOptimizer失败: {e}")
    WhaleOptimizer = None
    forward_kinematics_dh = None 