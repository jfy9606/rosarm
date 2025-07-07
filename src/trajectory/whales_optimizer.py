#!/usr/bin/env python3
import numpy as np
import rospy
import random
import math
from geometry_msgs.msg import Pose, Point, Quaternion

class WhaleOptimizer:
    """
    鲸鱼优化算法实现，用于机械臂逆运动学求解
    
    参数:
    - lower_bounds: 下界约束
    - upper_bounds: 上界约束
    - max_iter: 最大迭代次数
    - pop_size: 种群大小
    """
    def __init__(self, lower_bounds, upper_bounds, max_iter=50, pop_size=30):
        self.lb = np.array(lower_bounds)
        self.ub = np.array(upper_bounds)
        self.dim = len(lower_bounds)
        self.max_iter = max_iter
        self.pop_size = pop_size
        
        # 检查参数
        if len(lower_bounds) != len(upper_bounds):
            raise ValueError("下界和上界维度不匹配")
        
        # 确保下界小于上界
        if not all(l < u for l, u in zip(lower_bounds, upper_bounds)):
            raise ValueError("下界必须小于上界")
        
        # 优化器参数
        self.a_max = 2  # 优化参数a的最大值
        self.a_min = 0  # 优化参数a的最小值
        self.spiral_param = 1  # 螺旋参数
        
        # 初始化种群
        self._init_population()
        
        rospy.loginfo("WhaleOptimizer初始化完成")
    
    def _init_population(self):
        """初始化种群"""
        # 随机初始化在约束范围内
        self.population = np.zeros((self.pop_size, self.dim))
        for i in range(self.pop_size):
            self.population[i] = np.random.uniform(self.lb, self.ub)
        
        # 初始化最佳位置
        self.best_position = None
        self.best_fitness = float('inf')
    
    def optimize(self, fitness_func, max_time=None, callback=None):
        """
        运行优化算法
        
        参数:
        - fitness_func: 适应度函数，接受一个numpy数组，返回一个标量
        - max_time: 最大运行时间（秒），如果为None则不限制
        - callback: 每次迭代后调用的回调函数，接受当前迭代、最佳适应度和最佳位置
        
        返回:
        - best_position: 找到的最佳位置
        - best_fitness: 最佳位置的适应度
        """
        start_time = rospy.get_time() if max_time else None
        
        # 评估初始种群
        fitness = np.array([fitness_func(ind) for ind in self.population])
        best_idx = np.argmin(fitness)
        self.best_position = self.population[best_idx].copy()
        self.best_fitness = fitness[best_idx]
        
        # 迭代优化
        for t in range(self.max_iter):
            # 更新a参数（线性减小）
            a = self.a_max - t * ((self.a_max - self.a_min) / self.max_iter)
            
            # 更新每个鲸鱼位置
            for i in range(self.pop_size):
                # 生成随机参数
                r = np.random.random()
                A = 2 * a * np.random.random(self.dim) - a  # [-a, a]
                C = 2 * np.random.random(self.dim)  # [0, 2]
                l = np.random.random() * 2 - 1  # [-1, 1]
                p = np.random.random()  # [0, 1]
                
                # 根据概率选择更新策略
                if p < 0.5:
                    # 计算|A|
                    A_abs = np.abs(A)
                    
                    if np.all(A_abs < 1):
                        # 围猎策略
                        D = np.abs(C * self.best_position - self.population[i])
                        new_position = self.best_position - A * D
                    else:
                        # 搜索策略（选择随机鲸鱼）
                        random_idx = np.random.randint(0, self.pop_size)
                        random_whale = self.population[random_idx]
                        D = np.abs(C * random_whale - self.population[i])
                        new_position = random_whale - A * D
                else:
                    # 螺旋更新策略
                    D = np.abs(self.best_position - self.population[i])
                    spiral = D * np.exp(self.spiral_param * l) * np.cos(2 * np.pi * l)
                    new_position = self.best_position + spiral
                
                # 边界检查并更新位置
                new_position = np.clip(new_position, self.lb, self.ub)
                self.population[i] = new_position
            
            # 重新评估种群
            fitness = np.array([fitness_func(ind) for ind in self.population])
            current_best_idx = np.argmin(fitness)
            
            # 更新全局最优
            if fitness[current_best_idx] < self.best_fitness:
                self.best_position = self.population[current_best_idx].copy()
                self.best_fitness = fitness[current_best_idx]
            
            # 调用回调函数
            if callback:
                callback(t, self.best_fitness, self.best_position)
            
            # 检查时间限制
            if max_time and (rospy.get_time() - start_time > max_time):
                rospy.loginfo(f"优化达到时间限制 {max_time} 秒，提前结束")
                break
        
        return self.best_position, self.best_fitness

# DH参数正向运动学函数
def forward_kinematics_dh(joint_values, dh_params):
    """
    使用DH参数计算正向运动学
    
    参数:
    - joint_values: 关节角度值的列表 [theta1, theta2, ...]
    - dh_params: DH参数的列表，每个元素为 [theta_offset, d, a, alpha]
    
    返回:
    - T: 变换矩阵(4x4)，表示末端执行器的位姿
    """
    T = np.eye(4)
    
    for i, theta in enumerate(joint_values):
        theta_i = theta + dh_params[i][0]
        d_i = dh_params[i][1]
        a_i = dh_params[i][2]
        alpha_i = dh_params[i][3]
        
        ct = np.cos(theta_i)
        st = np.sin(theta_i)
        ca = np.cos(alpha_i)
        sa = np.sin(alpha_i)
        
        # DH变换矩阵
        T_i = np.array([
            [ct, -st*ca, st*sa, a_i*ct],
            [st, ct*ca, -ct*sa, a_i*st],
            [0, sa, ca, d_i],
            [0, 0, 0, 1]
        ])
        
        # 累积变换
        T = T @ T_i
    
    return T

# 简单测试代码
if __name__ == "__main__":
    rospy.loginfo("成功导入WhaleOptimizer模块")
    
    # 测试WhaleOptimizer
    try:
        lb = [-5, -5]
        ub = [5, 5]
        
        # 定义测试函数(二维Rosenbrock函数)
        def test_func(x):
            return (1-x[0])**2 + 100*(x[1]-x[0]**2)**2
        
        optimizer = WhaleOptimizer(lb, ub, max_iter=50, pop_size=30)
        best_pos, best_fitness = optimizer.optimize(test_func)
        
        print(f"最佳位置: {best_pos}")
        print(f"最佳适应度: {best_fitness}")
    except Exception as e:
        print(f"测试出错: {e}")
