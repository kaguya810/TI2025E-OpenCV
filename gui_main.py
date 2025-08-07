#!/usr/bin/env python3
"""
PyQt5 GUI for TI2025E-OpenCV Servo/Laser Control System
基于PyQt5的OpenCV舵机激光控制系统图形界面
"""

import sys
import threading
import time
from collections import deque

import cv2
import numpy as np
from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, 
                             QHBoxLayout, QGridLayout, QLabel, QPushButton, 
                             QSlider, QSpinBox, QDoubleSpinBox, QGroupBox,
                             QComboBox, QCheckBox, QTextEdit, QSplitter,
                             QFrame, QSizePolicy)
from PyQt5.QtCore import Qt, QTimer, pyqtSignal, QThread, pyqtSlot
from PyQt5.QtGui import QImage, QPixmap, QFont

# 导入项目模块
from include.SerialCtrl import SerialComm
from include.dect import RectangleDetector, find_black_rectangle_center
from include.PWM import ServoController
from include.camera_reader import CameraReader
from include.pid import PID, PIDParams, PIDController
from include.display import DebugDisplay


# 检测参数类
class DetectionParams:
    def __init__(self):
        self.min_area = 5000
        self.min_rectangularity = 0.8
        self.max_aspect_ratio = 2.0
        self.distance_weight = 0.3
        self.adaptive_block_size = 11
        self.adaptive_c = 2
        self.morph_kernel_size = 3
        self.gaussian_blur_size = 5
        self.show_params = False


class CameraThread(QThread):
    """相机采集线程"""
    frameReady = pyqtSignal(np.ndarray)
    
    def __init__(self, cam_id=0):
        super().__init__()
        self.cam_id = cam_id
        self.camera_reader = None
        self.running = False
    
    def run(self):
        try:
            self.camera_reader = CameraReader(self.cam_id)
            self.camera_reader.start()
            self.running = True
            
            while self.running:
                if self.camera_reader.ret:
                    frame = self.camera_reader.get_frame()
                    if frame is not None:
                        self.frameReady.emit(frame)
                self.msleep(33)  # ~30fps
                
        except Exception as e:
            print(f"Camera thread error: {e}")
    
    def stop(self):
        self.running = False
        if self.camera_reader:
            self.camera_reader.stop()
        self.wait()


class DetectionParamsWidget(QGroupBox):
    """检测参数控制面板"""
    
    def __init__(self):
        super().__init__("检测参数 Detection Parameters")
        self.init_ui()
        
    def init_ui(self):
        layout = QGridLayout()
        
        # 最小面积
        layout.addWidget(QLabel("最小面积:"), 0, 0)
        self.min_area_spinbox = QSpinBox()
        self.min_area_spinbox.setRange(100, 50000)
        self.min_area_spinbox.setValue(5000)
        layout.addWidget(self.min_area_spinbox, 0, 1)
        
        # 矩形度
        layout.addWidget(QLabel("矩形度:"), 1, 0)
        self.rectangularity_spinbox = QDoubleSpinBox()
        self.rectangularity_spinbox.setRange(0.1, 1.0)
        self.rectangularity_spinbox.setSingleStep(0.1)
        self.rectangularity_spinbox.setValue(0.8)
        layout.addWidget(self.rectangularity_spinbox, 1, 1)
        
        # 最大宽高比
        layout.addWidget(QLabel("最大宽高比:"), 2, 0)
        self.aspect_ratio_spinbox = QDoubleSpinBox()
        self.aspect_ratio_spinbox.setRange(1.0, 10.0)
        self.aspect_ratio_spinbox.setSingleStep(0.1)
        self.aspect_ratio_spinbox.setValue(2.0)
        layout.addWidget(self.aspect_ratio_spinbox, 2, 1)
        
        # 高斯模糊尺寸
        layout.addWidget(QLabel("高斯模糊:"), 3, 0)
        self.gaussian_blur_spinbox = QSpinBox()
        self.gaussian_blur_spinbox.setRange(1, 15)
        self.gaussian_blur_spinbox.setSingleStep(2)
        self.gaussian_blur_spinbox.setValue(5)
        layout.addWidget(self.gaussian_blur_spinbox, 3, 1)
        
        self.setLayout(layout)
    
    def get_params(self):
        """获取检测参数"""
        params = DetectionParams()
        params.min_area = self.min_area_spinbox.value()
        params.min_rectangularity = self.rectangularity_spinbox.value()
        params.max_aspect_ratio = self.aspect_ratio_spinbox.value()
        params.gaussian_blur_size = self.gaussian_blur_spinbox.value()
        return params


class PIDParamsWidget(QGroupBox):
    """PID参数控制面板"""
    
    def __init__(self):
        super().__init__("PID参数 PID Parameters")
        self.init_ui()
        
    def init_ui(self):
        layout = QGridLayout()
        
        # Pan PID
        layout.addWidget(QLabel("Pan Kp:"), 0, 0)
        self.pan_kp_spinbox = QDoubleSpinBox()
        self.pan_kp_spinbox.setRange(0.0, 10.0)
        self.pan_kp_spinbox.setSingleStep(0.01)
        self.pan_kp_spinbox.setValue(2.8)
        layout.addWidget(self.pan_kp_spinbox, 0, 1)
        
        layout.addWidget(QLabel("Pan Ki:"), 1, 0)
        self.pan_ki_spinbox = QDoubleSpinBox()
        self.pan_ki_spinbox.setRange(0.0, 1.0)
        self.pan_ki_spinbox.setSingleStep(0.001)
        self.pan_ki_spinbox.setValue(0.01)
        layout.addWidget(self.pan_ki_spinbox, 1, 1)
        
        layout.addWidget(QLabel("Pan Kd:"), 2, 0)
        self.pan_kd_spinbox = QDoubleSpinBox()
        self.pan_kd_spinbox.setRange(0.0, 1.0)
        self.pan_kd_spinbox.setSingleStep(0.001)
        self.pan_kd_spinbox.setValue(0.02)
        layout.addWidget(self.pan_kd_spinbox, 2, 1)
        
        # Tilt PID
        layout.addWidget(QLabel("Tilt Kp:"), 0, 2)
        self.tilt_kp_spinbox = QDoubleSpinBox()
        self.tilt_kp_spinbox.setRange(0.0, 10.0)
        self.tilt_kp_spinbox.setSingleStep(0.01)
        self.tilt_kp_spinbox.setValue(2.5)
        layout.addWidget(self.tilt_kp_spinbox, 0, 3)
        
        layout.addWidget(QLabel("Tilt Ki:"), 1, 2)
        self.tilt_ki_spinbox = QDoubleSpinBox()
        self.tilt_ki_spinbox.setRange(0.0, 1.0)
        self.tilt_ki_spinbox.setSingleStep(0.001)
        self.tilt_ki_spinbox.setValue(0.008)
        layout.addWidget(self.tilt_ki_spinbox, 1, 3)
        
        layout.addWidget(QLabel("Tilt Kd:"), 2, 2)
        self.tilt_kd_spinbox = QDoubleSpinBox()
        self.tilt_kd_spinbox.setRange(0.0, 1.0)
        self.tilt_kd_spinbox.setSingleStep(0.001)
        self.tilt_kd_spinbox.setValue(0.015)
        layout.addWidget(self.tilt_kd_spinbox, 2, 3)
        
        self.setLayout(layout)
    
    def get_params(self):
        """获取PID参数"""
        params = PIDParams()
        params.pan_kp = self.pan_kp_spinbox.value()
        params.pan_ki = self.pan_ki_spinbox.value()
        params.pan_kd = self.pan_kd_spinbox.value()
        params.tilt_kp = self.tilt_kp_spinbox.value()
        params.tilt_ki = self.tilt_ki_spinbox.value()
        params.tilt_kd = self.tilt_kd_spinbox.value()
        return params


class ControlWidget(QGroupBox):
    """控制面板"""
    
    def __init__(self):
        super().__init__("控制面板 Control Panel")
        self.init_ui()
        
    def init_ui(self):
        layout = QVBoxLayout()
        
        # 主控制按钮
        self.start_button = QPushButton("开始控制 Start Control")
        self.start_button.setStyleSheet("QPushButton { background-color: #4CAF50; color: white; font-weight: bold; }")
        self.stop_button = QPushButton("停止控制 Stop Control")
        self.stop_button.setStyleSheet("QPushButton { background-color: #f44336; color: white; font-weight: bold; }")
        self.stop_button.setEnabled(False)
        
        layout.addWidget(self.start_button)
        layout.addWidget(self.stop_button)
        
        # 激光控制
        laser_layout = QHBoxLayout()
        self.laser_on_button = QPushButton("激光开 Laser ON")
        self.laser_off_button = QPushButton("激光关 Laser OFF")
        laser_layout.addWidget(self.laser_on_button)
        laser_layout.addWidget(self.laser_off_button)
        layout.addLayout(laser_layout)
        
        # 模式选择
        layout.addWidget(QLabel("控制模式:"))
        self.mode_combo = QComboBox()
        self.mode_combo.addItems(["自动跟踪", "手动控制", "激光连续"])
        layout.addWidget(self.mode_combo)
        
        # 串口设置
        layout.addWidget(QLabel("串口端口:"))
        self.serial_combo = QComboBox()
        self.serial_combo.addItems(["/dev/ttyUSB0", "/dev/ttyUSB1", "/dev/ttyACM0"])
        self.serial_combo.setEditable(True)
        layout.addWidget(self.serial_combo)
        
        self.setLayout(layout)


class StatusWidget(QGroupBox):
    """状态显示面板"""
    
    def __init__(self):
        super().__init__("状态信息 Status Information")
        self.init_ui()
        
    def init_ui(self):
        layout = QGridLayout()
        
        # FPS显示
        layout.addWidget(QLabel("FPS:"), 0, 0)
        self.fps_label = QLabel("0.0")
        layout.addWidget(self.fps_label, 0, 1)
        
        # 处理时间
        layout.addWidget(QLabel("处理时间:"), 1, 0)
        self.process_time_label = QLabel("0.0 ms")
        layout.addWidget(self.process_time_label, 1, 1)
        
        # 控制状态
        layout.addWidget(QLabel("控制状态:"), 2, 0)
        self.control_status_label = QLabel("停止")
        layout.addWidget(self.control_status_label, 2, 1)
        
        # 激光状态
        layout.addWidget(QLabel("激光状态:"), 3, 0)
        self.laser_status_label = QLabel("关闭")
        layout.addWidget(self.laser_status_label, 3, 1)
        
        # 串口状态
        layout.addWidget(QLabel("串口状态:"), 4, 0)
        self.serial_status_label = QLabel("未连接")
        layout.addWidget(self.serial_status_label, 4, 1)
        
        # 目标状态
        layout.addWidget(QLabel("目标检测:"), 5, 0)
        self.target_status_label = QLabel("无目标")
        layout.addWidget(self.target_status_label, 5, 1)
        
        self.setLayout(layout)
    
    def update_status(self, fps=None, process_time=None, control_active=None, 
                     laser_active=None, serial_connected=None, target_detected=None):
        """更新状态显示"""
        if fps is not None:
            self.fps_label.setText(f"{fps:.1f}")
        if process_time is not None:
            self.process_time_label.setText(f"{process_time:.1f} ms")
        if control_active is not None:
            self.control_status_label.setText("运行" if control_active else "停止")
        if laser_active is not None:
            self.laser_status_label.setText("开启" if laser_active else "关闭")
        if serial_connected is not None:
            self.serial_status_label.setText("已连接" if serial_connected else "未连接")
        if target_detected is not None:
            self.target_status_label.setText("检测到目标" if target_detected else "无目标")


class MainWindow(QMainWindow):
    """主窗口"""
    
    def __init__(self):
        super().__init__()
        self.setWindowTitle("TI2025E-OpenCV 舵机激光控制系统")
        self.setGeometry(100, 100, 1400, 900)
        
        # 系统组件
        self.camera_thread = None
        self.detector_params = DetectionParams()
        self.pid_params = PIDParams()
        self.pid_controller = None
        self.serial_comm = None
        self.servo_controller = None
        
        # 状态变量
        self.control_active = False
        self.laser_active = False
        self.serial_connected = False
        self.prev_center = None
        
        # 初始化UI
        self.init_ui()
        
        # 性能计算
        self.fps_counter = 0
        self.fps_timer = time.time()
        self.current_fps = 0.0
        
        # 定时器用于更新状态
        self.status_timer = QTimer()
        self.status_timer.timeout.connect(self.update_status_display)
        self.status_timer.start(100)  # 100ms更新一次
        
    def init_ui(self):
        """初始化用户界面"""
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        
        # 主布局
        main_layout = QHBoxLayout()
        
        # 左侧：视频显示
        left_splitter = QSplitter(Qt.Vertical)
        
        # 视频显示区域
        self.video_label = QLabel("等待摄像头启动...")
        self.video_label.setMinimumSize(640, 480)
        self.video_label.setStyleSheet("border: 2px solid gray;")
        self.video_label.setAlignment(Qt.AlignCenter)
        self.video_label.setScaledContents(True)
        left_splitter.addWidget(self.video_label)
        
        # 日志显示区域
        self.log_text = QTextEdit()
        self.log_text.setMaximumHeight(150)
        self.log_text.setPlaceholderText("系统日志将在这里显示...")
        left_splitter.addWidget(self.log_text)
        
        # 右侧：控制面板
        right_widget = QWidget()
        right_layout = QVBoxLayout()
        
        # 检测参数
        self.detection_params = DetectionParamsWidget()
        right_layout.addWidget(self.detection_params)
        
        # PID参数
        self.pid_params = PIDParamsWidget()
        right_layout.addWidget(self.pid_params)
        
        # 控制面板
        self.control_panel = ControlWidget()
        right_layout.addWidget(self.control_panel)
        
        # 状态显示
        self.status_panel = StatusWidget()
        right_layout.addWidget(self.status_panel)
        
        # 添加伸缩空间
        right_layout.addStretch()
        
        right_widget.setLayout(right_layout)
        right_widget.setMaximumWidth(350)
        
        # 添加到主布局
        main_layout.addWidget(left_splitter, 2)
        main_layout.addWidget(right_widget, 1)
        
        central_widget.setLayout(main_layout)
        
        # 连接信号
        self.connect_signals()
        
    def connect_signals(self):
        """连接信号和槽"""
        self.control_panel.start_button.clicked.connect(self.start_control)
        self.control_panel.stop_button.clicked.connect(self.stop_control)
        self.control_panel.laser_on_button.clicked.connect(self.laser_on)
        self.control_panel.laser_off_button.clicked.connect(self.laser_off)
        
    def start_control(self):
        """开始控制"""
        self.log_message("正在启动控制系统...")
        
        try:
            # 启动摄像头
            if self.camera_thread is None:
                self.camera_thread = CameraThread()
                self.camera_thread.frameReady.connect(self.update_video_frame)
                self.camera_thread.start()
                
            # 初始化串口通信
            serial_port = self.control_panel.serial_combo.currentText()
            if not self.serial_connected:
                try:
                    self.serial_comm = SerialComm(serial_port, 9600)
                    self.serial_connected = True
                    self.log_message(f"串口 {serial_port} 连接成功")
                except Exception as e:
                    self.log_message(f"串口连接失败: {e}")
                    
            # 初始化PID控制器
            pid_params = self.pid_params.get_params()
            self.pid_controller = PIDController(pid_params)
            
            self.control_active = True
            self.control_panel.start_button.setEnabled(False)
            self.control_panel.stop_button.setEnabled(True)
            
            self.log_message("控制系统启动成功")
            
        except Exception as e:
            self.log_message(f"启动失败: {e}")
            
    def stop_control(self):
        """停止控制"""
        self.log_message("正在停止控制系统...")
        
        self.control_active = False
        
        # 停止摄像头
        if self.camera_thread:
            self.camera_thread.stop()
            self.camera_thread = None
            
        # 断开串口
        if self.serial_comm:
            self.serial_comm.close()
            self.serial_comm = None
            self.serial_connected = False
            
        self.control_panel.start_button.setEnabled(True)
        self.control_panel.stop_button.setEnabled(False)
        
        self.log_message("控制系统已停止")
        
    def laser_on(self):
        """开启激光"""
        if self.serial_comm and self.serial_connected:
            try:
                self.serial_comm.send_data(b'1;')
                self.laser_active = True
                self.log_message("激光已开启")
            except Exception as e:
                self.log_message(f"激光控制失败: {e}")
        else:
            self.log_message("串口未连接，无法控制激光")
            
    def laser_off(self):
        """关闭激光"""
        if self.serial_comm and self.serial_connected:
            try:
                self.serial_comm.send_data(b'0;')
                self.laser_active = False
                self.log_message("激光已关闭")
            except Exception as e:
                self.log_message(f"激光控制失败: {e}")
        else:
            self.log_message("串口未连接，无法控制激光")
            
    @pyqtSlot(np.ndarray)
    def update_video_frame(self, frame):
        """更新视频帧显示"""
        try:
            # 计算FPS
            self.fps_counter += 1
            current_time = time.time()
            if current_time - self.fps_timer >= 1.0:
                self.current_fps = self.fps_counter / (current_time - self.fps_timer)
                self.fps_counter = 0
                self.fps_timer = current_time
            
            # 如果控制激活，进行目标检测
            if self.control_active:
                detection_params = self.detection_params.get_params()
                start_time = time.time()
                
                # 目标检测 - 使用原有的函数接口
                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                gray = cv2.GaussianBlur(gray, (detection_params.gaussian_blur_size, detection_params.gaussian_blur_size), 0)
                filtered_point, contour = find_black_rectangle_center(gray, self.prev_center, detection_params)
                
                process_time = (time.time() - start_time) * 1000
                
                # 绘制检测结果
                if filtered_point is not None:
                    cv2.circle(frame, tuple(map(int, filtered_point)), 10, (0, 255, 0), -1)
                    self.status_panel.update_status(target_detected=True)
                    self.prev_center = filtered_point
                    
                    # 如果有PID控制器，计算控制输出
                    if self.pid_controller:
                        img_center = (frame.shape[1] // 2, frame.shape[0] // 2)
                        offset_x = filtered_point[0] - img_center[0]
                        offset_y = filtered_point[1] - img_center[1]
                        self.pid_controller.update_offset((offset_x, offset_y))
                        pan_output, tilt_output = self.pid_controller.get_output()
                        
                        # 在图像上显示偏移信息
                        cv2.putText(frame, f"Offset: ({offset_x:.1f}, {offset_y:.1f})", 
                                   (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
                        cv2.putText(frame, f"PID: ({pan_output:.1f}, {tilt_output:.1f})", 
                                   (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
                else:
                    self.status_panel.update_status(target_detected=False)
                    
                # 绘制中心点十字线
                img_center = (frame.shape[1] // 2, frame.shape[0] // 2)
                cv2.line(frame, (img_center[0] - 20, img_center[1]), (img_center[0] + 20, img_center[1]), (0, 0, 255), 2)
                cv2.line(frame, (img_center[0], img_center[1] - 20), (img_center[0], img_center[1] + 20), (0, 0, 255), 2)
                    
                # 更新处理时间
                self.status_panel.update_status(process_time=process_time)
            
            # 转换为Qt格式并显示
            rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            h, w, ch = rgb_frame.shape
            bytes_per_line = ch * w
            qt_image = QImage(rgb_frame.data, w, h, bytes_per_line, QImage.Format_RGB888)
            
            # 缩放以适应显示区域
            pixmap = QPixmap.fromImage(qt_image)
            scaled_pixmap = pixmap.scaled(self.video_label.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation)
            self.video_label.setPixmap(scaled_pixmap)
            
        except Exception as e:
            self.log_message(f"视频帧更新错误: {e}")
            
    def update_status_display(self):
        """更新状态显示"""
        self.status_panel.update_status(
            fps=self.current_fps,
            control_active=self.control_active,
            laser_active=self.laser_active,
            serial_connected=self.serial_connected
        )
        
    def log_message(self, message):
        """添加日志消息"""
        timestamp = time.strftime("%H:%M:%S")
        formatted_message = f"[{timestamp}] {message}"
        self.log_text.append(formatted_message)
        
        # 自动滚动到底部
        scrollbar = self.log_text.verticalScrollBar()
        scrollbar.setValue(scrollbar.maximum())
        
    def closeEvent(self, event):
        """窗口关闭事件"""
        if self.control_active:
            self.stop_control()
        event.accept()


def main():
    """主函数"""
    app = QApplication(sys.argv)
    app.setStyle('Fusion')  # 设置现代化样式
    
    # 设置应用程序字体
    font = QFont("Arial", 10)
    app.setFont(font)
    
    window = MainWindow()
    window.show()
    
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()