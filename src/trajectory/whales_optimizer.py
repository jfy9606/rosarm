import numpy as np
import rospy

class WhaleOptimizer:
    def __init__(self, lower_bounds, upper_bounds, max_iter, pop_size):
        self.lb = np.array(lower_bounds)
        self.ub = np.array(upper_bounds)
        self.dim = len(lower_bounds)
        self.max_iter = max_iter
        self.pop_size = pop_size
        
        rospy.loginfo("WhaleOptimizer initialization complete") 