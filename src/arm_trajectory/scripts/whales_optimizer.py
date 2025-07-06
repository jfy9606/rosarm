#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import numpy as np
import random
import math
import rospy
import locale

# 设置正确的字符编码环境
try:
    locale.setlocale(locale.LC_ALL, '')
    encoding = locale.getpreferredencoding()
    rospy.loginfo(f"当前编码: {encoding}")
except Exception as e:
    rospy.logwarn(f"无法设置本地化环境: {str(e)}")
    encoding = 'utf-8'

def compute_dh_transform(theta, d, a, alpha):
    """计算单个关节的DH变换矩阵"""
    ct = np.cos(theta)
    st = np.sin(theta)
    ca = np.cos(alpha)
    sa = np.sin(alpha)
    
    return np.array([
        [ct, -st*ca, st*sa, a*ct],
        [st, ct*ca, -ct*sa, a*st],
        [0, sa, ca, d],
        [0, 0, 0, 1]
    ])

def forward_kinematics_dh(joint_values, dh_params=None):
    """
    使用DH参数计算正向运动学
    
    Args:
        joint_values: 关节值列表 [θ1, d2, θ3, θ4, θ5, d6]
        dh_params: DH参数列表，如果为None则使用默认参数
    
    Returns:
        T: 末端执行器变换矩阵 (4x4)
    """
    if dh_params is None:
        # 默认DH参数 [type, d, theta, a, alpha]
        # type: 0=revolute, 1=prismatic
        a1, a2, a3 = 13, 13, 25  # 杆长
        dh_params = [
            # type, d, theta, a, alpha
            [0, 0, joint_values[0], a1, math.pi/2],  # 第一关节 (旋转)
            [1, joint_values[1], math.pi/4, 0, 0],   # 第二关节 (伸缩)
            [0, a2, joint_values[2], 0, math.pi/2],  # 第三关节 (旋转)
            [0, 0, joint_values[3], a3, math.pi/2],  # 第四关节 (旋转)
            [0, 0, joint_values[4], 0, math.pi/2],   # 第五关节 (固定)
            [1, joint_values[5], math.pi/2, 0, 0]    # 第六关节 (伸缩)
        ]
    
    T = np.eye(4)
    
    for joint_type, d, theta, a, alpha in dh_params:
        # 计算此关节的变换矩阵
        transform = compute_dh_transform(theta, d, a, alpha)
        # 累积变换
        T = T @ transform
    
    return T

class WhalesOptimizer:
    """
    鲸鱼优化算法 (Whale Optimization Algorithm, WOA)
    用于机械臂轨迹规划的校准实现
    """
    def __init__(self, num_whales=30, dim=6, max_iter=100, lb=None, ub=None, **kwargs):
        """
        初始化优化器
        
        Args:
            num_whales: 鲸鱼数量
            dim: 维度 (关节数量)
            max_iter: 最大迭代次数
            lb: 下界 [θ1_min, d2_min, θ3_min, θ4_min, θ5_min, d6_min]
            ub: 上界 [θ1_max, d2_max, θ3_max, θ4_max, θ5_max, d6_max]
            **kwargs: 额外参数，兼容不同调用方式
        """
        self.num_whales = num_whales
        self.dim = dim
        self.max_iter = max_iter
        
        # 如果未提供边界，使用默认边界
        if lb is None:
            self.lb = np.array([-math.pi, 0, -math.pi/2, 0, math.pi/2, 5])
        else:
            self.lb = np.array(lb)
            
        if ub is None:
            self.ub = np.array([math.pi, 43, math.pi/2, math.pi, math.pi/2, 15])
        else:
            self.ub = np.array(ub)
        
        self.target_T = None
        
        # 初始化最优解
        self.best_position = None
        self.best_fitness = float('inf')
        self.fitness_history = []
        
        rospy.loginfo("WhaleOptimizer初始化完成")
    
    def calculate_fitness(self, position):
        """
        计算适应度函数
        基于机械臂末端位置与目标位置的误差
        
        Args:
            position: 关节值列表
            
        Returns:
            fitness: 适应度值 (越小越好)
        """
        if self.target_T is None:
            return float('inf')
            
        # 计算当前关节配置的末端位置
        T = forward_kinematics_dh(position)
        
        # 计算位置误差
        position_error = np.linalg.norm(T[:3, 3] - self.target_T[:3, 3])
        
        # 计算方向误差
        rotation_error = np.linalg.norm(T[:3, :3] - self.target_T[:3, :3], 'fro')
        
        # 总误差为位置误差和方向误差的加权和
        fitness = 0.7 * position_error + 0.3 * rotation_error
        
        return fitness
    
    def init_population(self):
        """初始化种群"""
        population = np.zeros((self.num_whales, self.dim))
        
        for i in range(self.num_whales):
            # 在每个维度上随机生成一个在下界和上界之间的值
            for j in range(self.dim):
                population[i, j] = random.uniform(self.lb[j], self.ub[j])
        
        return population
    
    def run(self):
        """运行优化算法"""
        # 初始化种群
        population = self.init_population()
        
        # 计算每个个体的适应度
        fitness = np.zeros(self.num_whales)
        for i in range(self.num_whales):
            fitness[i] = self.calculate_fitness(population[i])
        
        # 找到最佳个体
        best_idx = np.argmin(fitness)
        self.best_position = population[best_idx].copy()
        self.best_fitness = fitness[best_idx]
        
        # 主循环
        for t in range(self.max_iter):
            # 更新a从2到0线性递减
            a = 2 - t * (2 / self.max_iter)
            
            # 更新每个鲸鱼的位置
            for i in range(self.num_whales):
                # 随机生成r1, r2
                r1 = random.random()
                r2 = random.random()
                
                # 更新位置的因子
                A = 2 * a * r1 - a
                C = 2 * r2
                
                # 探索行为的参数
                l = random.uniform(-1, 1)
                p = 0.5  # 概率因子
                
                # 选择更新策略
                if random.random() < p:
                    # 螺旋更新位置
                    if abs(A) < 1:
                        # 收缩包围
                        D = abs(C * self.best_position - population[i])
                        population[i] = self.best_position - A * D
                    else:
                        # 随机选择一个鲸鱼
                        random_idx = random.randint(0, self.num_whales - 1)
                        random_whale = population[random_idx]
                        
                        # 探索新区域
                        D = abs(C * random_whale - population[i])
                        population[i] = random_whale - A * D
                else:
                    # 螺旋更新
                    D = abs(self.best_position - population[i])
                    b = 1  # 螺旋常数
                    spiral_factor = math.exp(b * l) * math.cos(2 * math.pi * l)
                    population[i] = D * spiral_factor + self.best_position
                
                # 边界处理
                for j in range(self.dim):
                    population[i, j] = max(self.lb[j], min(self.ub[j], population[i, j]))
            
            # 更新适应度
            for i in range(self.num_whales):
                fitness[i] = self.calculate_fitness(population[i])
            
            # 更新最优解
            curr_best_idx = np.argmin(fitness)
            if fitness[curr_best_idx] < self.best_fitness:
                self.best_position = population[curr_best_idx].copy()
                self.best_fitness = fitness[curr_best_idx]
            
            # 记录历史最佳适应度
            self.fitness_history.append(self.best_fitness)
            
            # 每10次迭代打印一次状态
            if (t + 1) % 10 == 0 or t == 0 or t == self.max_iter - 1:
                rospy.loginfo(f"迭代 {t+1}/{self.max_iter}: 最佳适应度 = {self.best_fitness:.6f}")
        
        return self.best_position, self.best_fitness
    
    def optimize(self, target_T, initial_joints=None):
        """兼容SimpleOptimizer接口的方法"""
        self.target_T = target_T
        best_position, best_fitness = self.run()
        return best_position, best_fitness, self.fitness_history

# 在导入时打印一条消息，帮助调试
if __name__ != "__main__":
    rospy.loginfo("成功导入WhaleOptimizer模块") 