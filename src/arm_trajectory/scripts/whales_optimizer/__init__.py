# 从父目录的whales_optimizer.py导入类和函数
import os
import sys
import rospy

# 获取当前文件所在目录
current_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(current_dir)

# 从whales_optimizer.py导入所需类和函数
try:
    from whales_optimizer import WhaleOptimizer, forward_kinematics_dh
    rospy.loginfo("成功从whales_optimizer.py导入WhaleOptimizer和forward_kinematics_dh")
except ImportError as e:
    rospy.logerr(f"从whales_optimizer.py导入失败: {e}")
    WhaleOptimizer = None
    forward_kinematics_dh = None 