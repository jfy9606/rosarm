#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
机械臂 Qt5 图形界面控制程序
"""

import sys
import os
import time
from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, 
                            QHBoxLayout, QPushButton, QLabel, QComboBox, 
                            QSlider, QGroupBox, QTabWidget, QGridLayout, 
                            QSpinBox, QDoubleSpinBox, QMessageBox, QStatusBar)
from PyQt5.QtCore import Qt, QTimer, pyqtSlot, QThread, pyqtSignal
from PyQt5.QtGui import QFont, QIcon
import serial.tools.list_ports

# 导入机械臂控制类
from src.robot_arm import RobotArm


class PortScanThread(QThread):
    """串口扫描线程"""
    ports_found = pyqtSignal(list)
    
    def run(self):
        ports = list(serial.tools.list_ports.comports())
        available_ports = [p.device for p in ports]
        self.ports_found.emit(available_ports)


class ServoScanThread(QThread):
    """舵机扫描线程"""
    servos_found = pyqtSignal(list)
    error_occurred = pyqtSignal(str)
    
    def __init__(self, robot, start_id, end_id):
        super().__init__()
        self.robot = robot
        self.start_id = start_id
        self.end_id = end_id
        
    def run(self):
        try:
            found_servos = self.robot.scan_servos(self.start_id, self.end_id)
            self.servos_found.emit(found_servos)
        except Exception as e:
            self.error_occurred.emit(str(e))


class RobotArmGUI(QMainWindow):
    """机械臂控制图形界面"""
    
    def __init__(self):
        super().__init__()
        
        # 初始化机械臂控制器（未连接）
        self.robot = RobotArm()
        
        # 初始化界面
        self.init_ui()
        
        # 连接状态
        self.connected = False
        
        # 舵机信息
        self.servos = []
        
        # 刷新串口列表
        self.refresh_ports()
        
        # 启动定时器以更新状态
        self.status_timer = QTimer(self)
        self.status_timer.timeout.connect(self.update_status)
        
    def init_ui(self):
        """初始化用户界面"""
        self.setWindowTitle("机械臂控制系统")
        self.setMinimumSize(800, 600)
        
        # 创建中心部件和布局
        central_widget = QWidget()
        main_layout = QVBoxLayout(central_widget)
        
        # 连接面板
        connection_group = QGroupBox("连接设置")
        connection_layout = QGridLayout(connection_group)
        
        # 舵机端口选择
        connection_layout.addWidget(QLabel("舵机串口:"), 0, 0)
        self.servo_port_combo = QComboBox()
        connection_layout.addWidget(self.servo_port_combo, 0, 1)
        
        # 舵机波特率选择
        connection_layout.addWidget(QLabel("舵机波特率:"), 0, 2)
        self.servo_baudrate_combo = QComboBox()
        self.servo_baudrate_combo.addItems(["1000000", "500000", "115200"])
        self.servo_baudrate_combo.setCurrentText("1000000")
        connection_layout.addWidget(self.servo_baudrate_combo, 0, 3)
        
        # 电机端口选择
        connection_layout.addWidget(QLabel("电机串口:"), 1, 0)
        self.motor_port_combo = QComboBox()
        connection_layout.addWidget(self.motor_port_combo, 1, 1)
        
        # 电机波特率选择
        connection_layout.addWidget(QLabel("电机波特率:"), 1, 2)
        self.motor_baudrate_combo = QComboBox()
        self.motor_baudrate_combo.addItems(["115200", "9600"])
        self.motor_baudrate_combo.setCurrentText("115200")
        connection_layout.addWidget(self.motor_baudrate_combo, 1, 3)
        
        # 刷新和连接按钮
        self.refresh_button = QPushButton("刷新端口")
        self.refresh_button.clicked.connect(self.refresh_ports)
        connection_layout.addWidget(self.refresh_button, 0, 4)
        
        self.connect_button = QPushButton("连接")
        self.connect_button.clicked.connect(self.connect_robot)
        connection_layout.addWidget(self.connect_button, 1, 4)
        
        main_layout.addWidget(connection_group)
        
        # 创建标签页
        tab_widget = QTabWidget()
        
        # 基本控制标签页
        basic_tab = QWidget()
        basic_layout = QVBoxLayout(basic_tab)
        
        # 舵机控制组
        servo_group = QGroupBox("舵机控制")
        servo_layout = QGridLayout(servo_group)
        
        # 舵机扫描按钮
        self.scan_button = QPushButton("扫描舵机")
        self.scan_button.clicked.connect(self.scan_servos)
        servo_layout.addWidget(self.scan_button, 0, 0, 1, 2)
        
        # 舵机使能控制
        self.enable_button = QPushButton("使能舵机")
        self.enable_button.clicked.connect(self.toggle_torque)
        servo_layout.addWidget(self.enable_button, 0, 2, 1, 2)
        
        # 舵机速度控制
        servo_layout.addWidget(QLabel("舵机速度:"), 1, 0)
        self.servo_speed_slider = QSlider(Qt.Horizontal)
        self.servo_speed_slider.setRange(0, 1000)
        self.servo_speed_slider.setValue(500)
        self.servo_speed_slider.valueChanged.connect(self.update_speeds)
        servo_layout.addWidget(self.servo_speed_slider, 1, 1)
        self.servo_speed_label = QLabel("500")
        servo_layout.addWidget(self.servo_speed_label, 1, 2)
        
        # 加速度控制
        servo_layout.addWidget(QLabel("加速度:"), 2, 0)
        self.acc_slider = QSlider(Qt.Horizontal)
        self.acc_slider.setRange(1, 50)
        self.acc_slider.setValue(5)
        self.acc_slider.valueChanged.connect(self.update_acceleration)
        servo_layout.addWidget(self.acc_slider, 2, 1)
        self.acc_label = QLabel("5")
        servo_layout.addWidget(self.acc_label, 2, 2)
        
        # 添加舵机控制滑块
        joint_names = ["底座旋转 (1)", "肩部 (2)", "肘部 (3)", "腕部 (4)"]
        self.joint_sliders = {}
        self.joint_value_labels = {}
        
        for i, joint_name in enumerate(joint_names):
            row = i + 3
            joint_key = f"joint{i+1}"
            
            # 添加标签
            servo_layout.addWidget(QLabel(joint_name), row, 0)
            
            # 添加滑块
            slider = QSlider(Qt.Horizontal)
            if i == 0:  # 底座可以360度旋转
                slider.setRange(0, 4095)
                slider.setValue(2048)
            else:
                slider.setRange(500, 3500)
                slider.setValue(2048)
            
            slider.setProperty("joint", joint_key)
            slider.valueChanged.connect(self.move_joint)
            servo_layout.addWidget(slider, row, 1)
            self.joint_sliders[joint_key] = slider
            
            # 添加值标签
            value_label = QLabel("2048")
            servo_layout.addWidget(value_label, row, 2)
            self.joint_value_labels[joint_key] = value_label
        
        basic_layout.addWidget(servo_group)
        
        # DC电机控制组
        dc_motor_group = QGroupBox("DC电机控制")
        dc_layout = QGridLayout(dc_motor_group)
        
        # 俯仰电机控制
        dc_layout.addWidget(QLabel("俯仰速度:"), 0, 0)
        self.pitch_speed_slider = QSlider(Qt.Horizontal)
        self.pitch_speed_slider.setRange(0, 255)
        self.pitch_speed_slider.setValue(100)
        self.pitch_speed_slider.valueChanged.connect(self.update_speeds)
        dc_layout.addWidget(self.pitch_speed_slider, 0, 1)
        self.pitch_speed_label = QLabel("100")
        dc_layout.addWidget(self.pitch_speed_label, 0, 2)
        
        dc_layout.addWidget(QLabel("俯仰位置:"), 1, 0)
        self.pitch_slider = QSlider(Qt.Horizontal)
        self.pitch_slider.setRange(-10000, 10000)
        self.pitch_slider.setValue(0)
        self.pitch_slider.valueChanged.connect(self.move_pitch)
        dc_layout.addWidget(self.pitch_slider, 1, 1)
        self.pitch_value_label = QLabel("0")
        dc_layout.addWidget(self.pitch_value_label, 1, 2)
        
        # 线性电机控制
        dc_layout.addWidget(QLabel("线性速度:"), 2, 0)
        self.linear_speed_slider = QSlider(Qt.Horizontal)
        self.linear_speed_slider.setRange(0, 255)
        self.linear_speed_slider.setValue(100)
        self.linear_speed_slider.valueChanged.connect(self.update_speeds)
        dc_layout.addWidget(self.linear_speed_slider, 2, 1)
        self.linear_speed_label = QLabel("100")
        dc_layout.addWidget(self.linear_speed_label, 2, 2)
        
        dc_layout.addWidget(QLabel("线性位置:"), 3, 0)
        self.linear_slider = QSlider(Qt.Horizontal)
        self.linear_slider.setRange(0, 20000)
        self.linear_slider.setValue(0)
        self.linear_slider.valueChanged.connect(self.move_linear)
        dc_layout.addWidget(self.linear_slider, 3, 1)
        self.linear_value_label = QLabel("0")
        dc_layout.addWidget(self.linear_value_label, 3, 2)
        
        basic_layout.addWidget(dc_motor_group)
        
        # 紧急停止和复位按钮
        control_layout = QHBoxLayout()
        
        self.stop_button = QPushButton("紧急停止")
        self.stop_button.setStyleSheet("background-color: red; color: white; font-weight: bold;")
        self.stop_button.clicked.connect(self.emergency_stop)
        control_layout.addWidget(self.stop_button)
        
        self.home_button = QPushButton("归位")
        self.home_button.clicked.connect(self.home_position)
        control_layout.addWidget(self.home_button)
        
        self.calibrate_button = QPushButton("校准")
        self.calibrate_button.clicked.connect(self.calibrate_robot)
        control_layout.addWidget(self.calibrate_button)
        
        basic_layout.addLayout(control_layout)
        
        # 添加标签页
        tab_widget.addTab(basic_tab, "基本控制")
        
        # 笛卡尔坐标控制标签页
        cartesian_tab = QWidget()
        cartesian_layout = QVBoxLayout(cartesian_tab)
        
        cartesian_group = QGroupBox("笛卡尔坐标控制")
        cart_grid = QGridLayout(cartesian_group)
        
        # X坐标控制
        cart_grid.addWidget(QLabel("X 坐标:"), 0, 0)
        self.x_spinbox = QSpinBox()
        self.x_spinbox.setRange(-50, 50)
        self.x_spinbox.setValue(0)
        cart_grid.addWidget(self.x_spinbox, 0, 1)
        
        # Y坐标控制
        cart_grid.addWidget(QLabel("Y 坐标:"), 1, 0)
        self.y_spinbox = QSpinBox()
        self.y_spinbox.setRange(-50, 50)
        self.y_spinbox.setValue(15)
        cart_grid.addWidget(self.y_spinbox, 1, 1)
        
        # Z坐标控制
        cart_grid.addWidget(QLabel("Z 坐标:"), 2, 0)
        self.z_spinbox = QSpinBox()
        self.z_spinbox.setRange(-50, 50)
        self.z_spinbox.setValue(0)
        cart_grid.addWidget(self.z_spinbox, 2, 1)
        
        # 移动按钮
        self.move_cart_button = QPushButton("移动到位置")
        self.move_cart_button.clicked.connect(self.move_cartesian)
        cart_grid.addWidget(self.move_cart_button, 3, 0, 1, 2)
        
        cartesian_layout.addWidget(cartesian_group)
        
        # 预设位置
        presets_group = QGroupBox("预设动作")
        presets_layout = QVBoxLayout(presets_group)
        
        self.pick_place_button = QPushButton("执行抓取-放置示例")
        self.pick_place_button.clicked.connect(self.run_pick_place_demo)
        presets_layout.addWidget(self.pick_place_button)
        
        self.square_demo_button = QPushButton("执行方形轨迹示例")
        self.square_demo_button.clicked.connect(self.run_square_demo)
        presets_layout.addWidget(self.square_demo_button)
        
        cartesian_layout.addWidget(presets_group)
        
        # 添加笛卡尔标签页
        tab_widget.addTab(cartesian_tab, "笛卡尔控制")
        
        # 将标签页添加到主布局
        main_layout.addWidget(tab_widget)
        
        # 状态栏
        self.statusBar = QStatusBar()
        self.setStatusBar(self.statusBar)
        self.statusBar.showMessage("未连接")
        
        # 设置中央部件
        self.setCentralWidget(central_widget)
        
        # 初始禁用控制
        self.enable_controls(False)
        
    def enable_controls(self, enabled=True):
        """启用或禁用控制组件"""
        # 舵机控制
        self.scan_button.setEnabled(enabled)
        self.enable_button.setEnabled(enabled)
        self.servo_speed_slider.setEnabled(enabled)
        self.acc_slider.setEnabled(enabled)
        
        # 关节滑块
        for slider in self.joint_sliders.values():
            slider.setEnabled(enabled)
        
        # DC电机控制
        self.pitch_speed_slider.setEnabled(enabled)
        self.pitch_slider.setEnabled(enabled)
        self.linear_speed_slider.setEnabled(enabled)
        self.linear_slider.setEnabled(enabled)
        
        # 按钮
        self.stop_button.setEnabled(enabled)
        self.home_button.setEnabled(enabled)
        self.calibrate_button.setEnabled(enabled)
        
        # 笛卡尔控制
        self.x_spinbox.setEnabled(enabled)
        self.y_spinbox.setEnabled(enabled)
        self.z_spinbox.setEnabled(enabled)
        self.move_cart_button.setEnabled(enabled)
        
        # 预设动作
        self.pick_place_button.setEnabled(enabled)
        self.square_demo_button.setEnabled(enabled)
    
    def refresh_ports(self):
        """刷新可用串口列表"""
        self.statusBar.showMessage("正在扫描可用端口...")
        
        # 启动扫描线程
        self.port_thread = PortScanThread()
        self.port_thread.ports_found.connect(self.update_port_list)
        self.port_thread.start()
    
    def update_port_list(self, ports):
        """更新端口列表"""
        current_servo = self.servo_port_combo.currentText()
        current_motor = self.motor_port_combo.currentText()
        
        self.servo_port_combo.clear()
        self.motor_port_combo.clear()
        
        if not ports:
            self.statusBar.showMessage("未检测到可用串口")
            return
        
        self.servo_port_combo.addItems(ports)
        self.motor_port_combo.addItems(ports)
        
        # 尝试保持之前选择的端口
        if current_servo in ports:
            self.servo_port_combo.setCurrentText(current_servo)
        
        if current_motor in ports:
            self.motor_port_combo.setCurrentText(current_motor)
        
        self.statusBar.showMessage(f"找到 {len(ports)} 个可用串口")
    
    def connect_robot(self):
        """连接或断开机械臂"""
        if not self.connected:
            servo_port = self.servo_port_combo.currentText()
            motor_port = self.motor_port_combo.currentText()
            
            if not servo_port or not motor_port:
                QMessageBox.warning(self, "错误", "请选择舵机和电机端口")
                return
            
            servo_baudrate = int(self.servo_baudrate_combo.currentText())
            motor_baudrate = int(self.motor_baudrate_combo.currentText())
            
            # 创建新的机械臂实例
            self.robot = RobotArm()
            
            self.statusBar.showMessage("正在连接...")
            
            # 连接到控制器
            if self.robot.connect(servo_port, motor_port, 
                                servo_baudrate=servo_baudrate,
                                motor_baudrate=motor_baudrate):
                self.connected = True
                self.connect_button.setText("断开连接")
                self.enable_controls(True)
                self.statusBar.showMessage("已连接")
                
                # 启动状态更新定时器
                self.status_timer.start(1000)  # 每秒更新一次
            else:
                QMessageBox.critical(self, "连接失败", "无法连接到机械臂控制器，请检查端口设置")
                self.statusBar.showMessage("连接失败")
        else:
            # 断开连接
            self.status_timer.stop()
            
            if self.robot:
                self.robot.stop_all()
                self.robot.enable_torque(False)
                self.robot.disconnect()
            
            self.connected = False
            self.connect_button.setText("连接")
            self.enable_controls(False)
            self.statusBar.showMessage("已断开连接")
    
    def scan_servos(self):
        """扫描舵机"""
        if not self.connected or not self.robot:
            return
        
        self.statusBar.showMessage("正在扫描舵机...")
        self.scan_thread = ServoScanThread(self.robot, 1, 10)
        self.scan_thread.servos_found.connect(self.update_servo_list)
        self.scan_thread.error_occurred.connect(self.show_error)
        self.scan_thread.start()
    
    def update_servo_list(self, servos):
        """更新找到的舵机列表"""
        self.servos = servos
        
        if not servos:
            QMessageBox.warning(self, "扫描结果", "未找到任何舵机")
            self.statusBar.showMessage("未找到舵机")
        else:
            servo_info = "\n".join([f"ID: {servo_id}, 型号: {model}" for servo_id, model in servos])
            QMessageBox.information(self, "扫描结果", f"找到 {len(servos)} 个舵机:\n{servo_info}")
            self.statusBar.showMessage(f"找到 {len(servos)} 个舵机")
    
    def toggle_torque(self):
        """切换舵机力矩状态"""
        if not self.connected or not self.robot:
            return
        
        if self.enable_button.text() == "使能舵机":
            if self.robot.enable_torque(True):
                self.enable_button.setText("禁用舵机")
                self.statusBar.showMessage("舵机已使能")
        else:
            if self.robot.enable_torque(False):
                self.enable_button.setText("使能舵机")
                self.statusBar.showMessage("舵机已禁用")
    
    def update_speeds(self):
        """更新各种电机的速度"""
        if not self.connected or not self.robot:
            return
        
        servo_speed = self.servo_speed_slider.value()
        pitch_speed = self.pitch_speed_slider.value()
        linear_speed = self.linear_speed_slider.value()
        
        self.servo_speed_label.setText(str(servo_speed))
        self.pitch_speed_label.setText(str(pitch_speed))
        self.linear_speed_label.setText(str(linear_speed))
        
        self.robot.set_speeds(servo_speed=servo_speed, pitch_speed=pitch_speed, linear_speed=linear_speed)
    
    def update_acceleration(self):
        """更新舵机加速度"""
        if not self.connected or not self.robot:
            return
        
        acc = self.acc_slider.value()
        self.acc_label.setText(str(acc))
        self.robot.set_all_servo_acc(acc)
    
    def move_joint(self):
        """移动关节"""
        if not self.connected or not self.robot:
            return
        
        # 确定哪个滑块触发了这个事件
        sender = self.sender()
        if not sender:
            return
            
        joint = sender.property("joint")
        position = sender.value()
        
        # 更新标签
        self.joint_value_labels[joint].setText(str(position))
        
        # 移动关节
        self.robot.set_joint_position(joint, position, blocking=False)
    
    def move_pitch(self):
        """移动俯仰电机"""
        if not self.connected or not self.robot:
            return
        
        position = self.pitch_slider.value()
        self.pitch_value_label.setText(str(position))
        self.robot.set_pitch_position(position, blocking=False)
    
    def move_linear(self):
        """移动线性电机"""
        if not self.connected or not self.robot:
            return
        
        position = self.linear_slider.value()
        self.linear_value_label.setText(str(position))
        self.robot.set_linear_position(position, blocking=False)
    
    def move_cartesian(self):
        """基于笛卡尔坐标移动"""
        if not self.connected or not self.robot:
            return
        
        x = self.x_spinbox.value()
        y = self.y_spinbox.value()
        z = self.z_spinbox.value()
        
        self.statusBar.showMessage(f"移动到笛卡尔坐标: X={x}, Y={y}, Z={z}")
        self.robot.move_cartesian(x, y, z)
    
    def emergency_stop(self):
        """紧急停止"""
        if not self.connected or not self.robot:
            return
        
        self.statusBar.showMessage("紧急停止！")
        self.robot.stop_all()
        
        # 禁用力矩
        self.robot.enable_torque(False)
        self.enable_button.setText("使能舵机")
    
    def home_position(self):
        """移动到初始位置"""
        if not self.connected or not self.robot:
            return
        
        self.statusBar.showMessage("返回原位...")
        self.robot.home()
        
        # 更新滑块位置
        self.update_sliders_to_home()
    
    def update_sliders_to_home(self):
        """更新所有滑块到原位位置"""
        # 关节滑块
        for joint in self.joint_sliders:
            self.joint_sliders[joint].setValue(2048)
            self.joint_value_labels[joint].setText("2048")
        
        # DC电机滑块
        self.pitch_slider.setValue(0)
        self.pitch_value_label.setText("0")
        self.linear_slider.setValue(0)
        self.linear_value_label.setText("0")
    
    def calibrate_robot(self):
        """校准机械臂"""
        if not self.connected or not self.robot:
            return
        
        self.statusBar.showMessage("正在校准...")
        self.robot.calibrate()
        self.statusBar.showMessage("校准完成")
        
        # 更新滑块位置
        self.update_sliders_to_home()
    
    def run_pick_place_demo(self):
        """运行抓取和放置演示"""
        if not self.connected or not self.robot:
            return
        
        self.statusBar.showMessage("执行抓取-放置演示...")
        
        # 移动到抓取位置
        self.robot.set_speeds(servo_speed=500, pitch_speed=100, linear_speed=100)
        self.robot.set_joint_position('joint1', 1500, blocking=True)  # 底座旋转
        self.robot.set_joint_position('joint2', 1700, blocking=True)  # 肩部
        self.robot.set_joint_position('joint3', 2300, blocking=True)  # 肘部
        self.robot.set_joint_position('joint4', 2048, blocking=True)  # 手腕
        
        # 降低以抓取
        self.robot.set_pitch_position(-1000, blocking=True)
        
        # 模拟抓取
        time.sleep(1)
        
        # 抓取后提升
        self.robot.set_pitch_position(0, blocking=True)
        
        # 移动到放置位置
        self.robot.set_joint_position('joint1', 2500, blocking=True)  # 旋转底座
        
        # 降低以放置
        self.robot.set_pitch_position(-800, blocking=True)
        
        # 模拟释放
        time.sleep(1)
        
        # 放置后提升
        self.robot.set_pitch_position(0, blocking=True)
        
        # 返回原位
        self.robot.home(blocking=True)
        
        self.statusBar.showMessage("抓取-放置演示完成")
    
    def run_square_demo(self):
        """运行方形轨迹演示"""
        if not self.connected or not self.robot:
            return
        
        self.statusBar.showMessage("执行方形轨迹演示...")
        
        self.robot.set_speeds(servo_speed=500, pitch_speed=100, linear_speed=100)
        
        # 在正方形轨迹上移动
        self.robot.move_cartesian(10, 10, 10)
        time.sleep(1)
        
        self.robot.move_cartesian(10, 20, 10)
        time.sleep(1)
        
        self.robot.move_cartesian(-10, 20, 10)
        time.sleep(1)
        
        self.robot.move_cartesian(-10, 10, 10)
        time.sleep(1)
        
        self.robot.move_cartesian(10, 10, 10)
        time.sleep(1)
        
        # 在垂直轨迹上移动
        self.robot.move_cartesian(0, 15, -10)
        time.sleep(1)
        
        self.robot.move_cartesian(0, 15, 10)
        time.sleep(1)
        
        self.robot.move_cartesian(0, 15, 0)
        time.sleep(1)
        
        # 返回原位
        self.robot.home(blocking=True)
        
        self.statusBar.showMessage("方形轨迹演示完成")
    
    def update_status(self):
        """更新机械臂状态信息"""
        if not self.connected or not self.robot:
            return
        
        try:
            # 读取关节位置
            positions = self.robot.read_joint_positions()
            if positions:
                for joint, position in positions.items():
                    if joint in self.joint_value_labels:
                        self.joint_value_labels[joint].setText(str(position))
                        if self.joint_sliders[joint].value() != position:
                            self.joint_sliders[joint].blockSignals(True)
                            self.joint_sliders[joint].setValue(position)
                            self.joint_sliders[joint].blockSignals(False)
            
            # 读取电机状态
            status = self.robot.read_motor_status()
            if status:
                pitch_pos = status['pitch_position']
                linear_pos = status['linear_position']
                
                self.pitch_value_label.setText(str(pitch_pos))
                self.linear_value_label.setText(str(linear_pos))
                
                if self.pitch_slider.value() != pitch_pos:
                    self.pitch_slider.blockSignals(True)
                    self.pitch_slider.setValue(pitch_pos)
                    self.pitch_slider.blockSignals(False)
                
                if self.linear_slider.value() != linear_pos:
                    self.linear_slider.blockSignals(True)
                    self.linear_slider.setValue(linear_pos)
                    self.linear_slider.blockSignals(False)
        except Exception as e:
            self.statusBar.showMessage(f"状态更新错误: {e}")
    
    def show_error(self, error_msg):
        """显示错误信息"""
        QMessageBox.critical(self, "错误", error_msg)
    
    def closeEvent(self, event):
        """程序关闭时的清理操作"""
        if self.connected and self.robot:
            try:
                self.status_timer.stop()
                self.robot.stop_all()
                self.robot.enable_torque(False)
                self.robot.disconnect()
            except Exception:
                pass
            
        event.accept()
        
    def preselect_ports(self, servo_port, motor_port, servo_baudrate=1000000, motor_baudrate=115200, protocol_end=0):
        """
        预选串口和设置参数，用于命令行参数传递
        
        Args:
            servo_port: 舵机串口
            motor_port: 电机串口
            servo_baudrate: 舵机波特率
            motor_baudrate: 电机波特率
            protocol_end: 协议结束位
        """
        # 检查端口是否在列表中，如果不在，就刷新端口列表
        if servo_port not in [self.servo_port_combo.itemText(i) for i in range(self.servo_port_combo.count())]:
            self.refresh_ports()
            
        # 设置串口
        if servo_port in [self.servo_port_combo.itemText(i) for i in range(self.servo_port_combo.count())]:
            self.servo_port_combo.setCurrentText(servo_port)
            
        if motor_port in [self.motor_port_combo.itemText(i) for i in range(self.motor_port_combo.count())]:
            self.motor_port_combo.setCurrentText(motor_port)
        
        # 设置波特率
        if str(servo_baudrate) in [self.servo_baudrate_combo.itemText(i) for i in range(self.servo_baudrate_combo.count())]:
            self.servo_baudrate_combo.setCurrentText(str(servo_baudrate))
            
        if str(motor_baudrate) in [self.motor_baudrate_combo.itemText(i) for i in range(self.motor_baudrate_combo.count())]:
            self.motor_baudrate_combo.setCurrentText(str(motor_baudrate))
            
        # 自动连接
        QTimer.singleShot(500, self.connect_robot)


if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = RobotArmGUI()
    window.show()
    sys.exit(app.exec_()) 