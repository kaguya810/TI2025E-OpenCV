import cv2
import numpy as np
import time
import serial
import struct
import threading
import sys
import select
from collections import deque
from math import pi, isnan

# PID控制器类
class PID:
    def __init__(self, p=0.0, i=0.0, d=0.0, imax=0.0):
        """
        PID控制器初始化
        :param p: 比例系数
        :param i: 积分系数
        :param d: 微分系数
        :param imax: 积分限幅值(绝对值)
        """
        self._kp = float(p)
        self._ki = float(i)
        self._kd = float(d)
        self._imax = abs(imax)
        self.reset()
    
    def reset(self):
        """重置控制器内部状态"""
        self._integrator = 0.0
        self._last_error = 0.0
        self._last_derivative = float('nan')
        self._last_t = 0
    
    def get_pid(self, error, scaler=1.0):
        """
        计算PID输出
        :param error: 当前误差值
        :param scaler: 输出缩放因子
        :return: PID控制量
        """
        tnow = time.time() * 1000  # 转换为毫秒时间戳
        dt = tnow - self._last_t
        
        # 异常时间间隔处理
        if self._last_t == 0 or dt > 1000:
            dt = 0
            self.reset()
        
        self._last_t = tnow
        delta_time = dt / 1000.0  # 转换为秒
        
        # 比例项
        output = error * self._kp
        
        # 微分项(带低通滤波)
        if abs(self._kd) > 0 and dt > 0:
            if isnan(self._last_derivative):
                # 首次计算时初始化
                derivative = 0.0
                self._last_derivative = 0.0
            else:
                # 计算原始微分值
                derivative = (error - self._last_error) / delta_time
            
            # 应用一阶低通滤波(RC=1/(2π*20))
            RC = 1 / (2 * pi * 20)
            derivative = self._last_derivative + (
                (delta_time / (RC + delta_time)) * 
                (derivative - self._last_derivative)
            )
            
            self._last_error = error
            self._last_derivative = derivative
            output += self._kd * derivative
        
        # 应用输出缩放
        output *= scaler
        
        # 积分项(带限幅)
        if abs(self._ki) > 0 and dt > 0:
            # 积分项单独缩放
            self._integrator += (error * self._ki) * scaler * delta_time
            
            # 积分限幅
            if self._integrator < -self._imax:
                self._integrator = -self._imax
            elif self._integrator > self._imax:
                self._integrator = self._imax
                
            output += self._integrator
        
        return output

# 可调检测参数（全局变量）
class DetectionParams:
    def __init__(self):
        # 基本参数
        self.min_area = 3320         # 最小轮廓面积
        self.min_rectangularity = 0.7  # 最小矩形度（轮廓面积/最小外接矩形面积）
        self.max_aspect_ratio = 5.0   # 最大长宽比（长边/短边）
        self.distance_weight = 0.3    # 位置连续性权重
        
        # 高级参数
        self.adaptive_block_size = 11  # 自适应阈值块大小
        self.adaptive_c = 2            # 自适应阈值常数
        self.morph_kernel_size = 3     # 形态学操作核大小
        self.gaussian_blur_size = 5    # 高斯模糊核大小
        
        # 状态标志
        self.show_params = False       # 是否显示参数值

# 创建参数实例
detection_params = DetectionParams()

# PID控制器参数
class PIDParams:
    def __init__(self):
        # 水平方向PID参数
        self.pan_kp = 0.1
        self.pan_ki = 0.01
        self.pan_kd = 0.05
        self.pan_imax = 100
        
        # 垂直方向PID参数
        self.tilt_kp = 0.1
        self.tilt_ki = 0.01
        self.tilt_kd = 0.05
        self.tilt_imax = 100
        
        # 输出缩放因子
        self.output_scaler = 1.0

# 创建PID参数实例
pid_params = PIDParams()

# 卡尔曼滤波器类
class KalmanFilter:
    def __init__(self, process_noise=1e-5, measurement_noise=1e-1):
        self.kf = cv2.KalmanFilter(4, 2)
        self.kf.measurementMatrix = np.array([[1,0,0,0],[0,1,0,0]], np.float32)
        self.kf.transitionMatrix = np.array([[1,0,1,0],[0,1,0,1],[0,0,1,0],[0,0,0,1]], np.float32)
        self.kf.processNoiseCov = np.eye(4, dtype=np.float32) * process_noise
        self.kf.measurementNoiseCov = np.eye(2, dtype=np.float32) * measurement_noise
        self.kf.errorCovPost = np.eye(4, dtype=np.float32)
        self.predicted = None
        
    def predict(self, pt):
        if pt is None:
            return None
            
        if self.predicted is None:
            self.kf.statePost = np.array([[pt[0]], [pt[1]], [0], [0]], dtype=np.float32)
            
        self.kf.predict()
        measurement = np.array([[np.float32(pt[0])], [np.float32(pt[1])]])
        self.kf.correct(measurement)
        prediction = self.kf.statePost
        self.predicted = (int(prediction[0]), int(prediction[1]))
        return self.predicted

def find_black_rectangle_center(thresh, prev_center=None):
    """优化后的矩形检测函数，支持参数调整"""
    # 使用自适应阈值
    adaptive_thresh = cv2.adaptiveThreshold(
        thresh, 255, 
        cv2.ADAPTIVE_THRESH_GAUSSIAN_C, 
        cv2.THRESH_BINARY_INV, 
        detection_params.adaptive_block_size, 
        detection_params.adaptive_c
    )
    
    # 形态学操作
    kernel = np.ones((detection_params.morph_kernel_size, detection_params.morph_kernel_size), np.uint8)
    morphed = cv2.morphologyEx(adaptive_thresh, cv2.MORPH_CLOSE, kernel)
    morphed = cv2.dilate(morphed, kernel, iterations=1)
    
    # 查找轮廓
    contours, _ = cv2.findContours(morphed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    if not contours:
        return None, None
    
    # 寻找最佳候选轮廓
    best_contour = None
    max_score = 0
    
    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area < detection_params.min_area:  # 使用可调最小面积
            continue
            
        # 计算矩形属性
        rect = cv2.minAreaRect(cnt)
        width, height = rect[1]
        if min(width, height) == 0:
            continue
            
        aspect_ratio = max(width, height) / min(width, height)
        rectangularity = area / (width * height)
        
        # 应用参数阈值
        if aspect_ratio > detection_params.max_aspect_ratio:
            continue
        if rectangularity < detection_params.min_rectangularity:
            continue
        
        # 综合评分
        score = area + rectangularity * 100 + (1 / aspect_ratio) * 50
        
        # 位置连续性评分
        if prev_center:
            M = cv2.moments(cnt)
            if M["m00"] != 0:
                cX = int(M["m10"] / M["m00"])
                cY = int(M["m01"] / M["m00"])
                distance = np.sqrt((cX - prev_center[0])**2 + (cY - prev_center[1])**2)
                distance_score = max(0, 100 - distance) * detection_params.distance_weight
                score += distance_score
        
        if score > max_score:
            max_score = score
            best_contour = cnt
    
    if best_contour is None:
        return None, None
    
    # 使用最小外接矩形中心
    rect = cv2.minAreaRect(best_contour)
    center = (int(rect[0][0]), int(rect[0][1]))
    
    return center, best_contour

# 全局变量
stop_sending = False
serial_port = None
kalman_filter = KalmanFilter(process_noise=1e-4, measurement_noise=1e-2)
prev_center = None
frame_queue = deque(maxlen=3)
trail_image = None

# 初始化PID控制器
pan_pid = PID(
    p=pid_params.pan_kp, 
    i=pid_params.pan_ki, 
    d=pid_params.pan_kd, 
    imax=pid_params.pan_imax
)

tilt_pid = PID(
    p=pid_params.tilt_kp, 
    i=pid_params.tilt_ki, 
    d=pid_params.tilt_kd, 
    imax=pid_params.tilt_imax
)

def input_listener():
    """监听键盘输入"""
    global stop_sending
    while not stop_sending:
        try:
            if sys.stdin in select.select([sys.stdin], [], [], 0)[0]:
                user_input = sys.stdin.read(1)
                if user_input.lower() == 'q':
                    stop_sending = True
                    print("\n停止发送...")
        except:
            break

def send_serial_data(pan_output, tilt_output, detected):
    """通过串口发送PID输出数据"""
    global serial_port
    if serial_port and serial_port.is_open:
        try:
            # 将PID输出转换为整数
            pan_int = int(pan_output)
            tilt_int = int(tilt_output)
            
            # 帧结构: 帧头(0x2C, 0x12), 水平输出(2字节), 垂直输出(2字节), 检测标志(1字节), 帧尾(0x5B)
            data = struct.pack('<hhbb', 
                               pan_int,    # 水平PID输出
                               tilt_int,   # 垂直PID输出
                               1 if detected else 0,  # 检测标志
                               0x5B)       # 帧尾
            serial_port.write(data)
        except Exception as e:
            print(f"串口发送错误: {e}")

# 初始化串口
try:
    serial_port = serial.Serial(
        port='/dev/ttyUSB0',
        baudrate=115200,
        bytesize=serial.EIGHTBITS,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        timeout=0.05
    )
    time.sleep(1)
    print(f"串口 {serial_port.port} 已打开，波特率 {serial_port.baudrate}")
except Exception as e:
    print(f"无法打开串口: {e}")
    serial_port = None

# 启动键盘监听线程
try:
    input_thread = threading.Thread(target=input_listener)
    input_thread.daemon = True
    input_thread.start()
    print("按 'q' 停止程序")
except:
    print("无法启动输入监听线程")

# 初始化摄像头
capture = cv2.VideoCapture(0)
if not capture.isOpened():
    print("无法访问摄像头！")
    exit()

# 设置摄像头参数
capture.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
capture.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
capture.set(cv2.CAP_PROP_FPS, 60)
capture.set(cv2.CAP_PROP_AUTO_WB, 0)
capture.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.25)
capture.set(cv2.CAP_PROP_EXPOSURE, -4)
capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)

# 性能监控
frame_count = 0
start_time = time.time()
fps = 0
processing_times = deque(maxlen=30)
send_counter = 0
send_interval = 2

# 参数调整步长
PARAM_STEP = {
    'min_area': 10,
    'min_rectangularity': 0.05,
    'max_aspect_ratio': 0.5,
    'distance_weight': 0.1,
    'adaptive_block_size': 2,
    'adaptive_c': 1,
    'morph_kernel_size': 1,
    'gaussian_blur_size': 2,
    # PID参数步长
    'pan_kp': 0.01,
    'pan_ki': 0.001,
    'pan_kd': 0.005,
    'tilt_kp': 0.01,
    'tilt_ki': 0.001,
    'tilt_kd': 0.005,
    'output_scaler': 0.1
}

try:
    while capture.isOpened() and not stop_sending:
        retval, frame = capture.read()
        if not retval:
            continue
            
        frame_queue.append(frame)
        if len(frame_queue) < frame_queue.maxlen:
            continue
            
        process_frame = frame_queue.popleft()
        process_start = time.time()
        
        # 预处理
        gray = cv2.cvtColor(process_frame, cv2.COLOR_BGR2GRAY)
        gray = cv2.GaussianBlur(gray, 
                               (detection_params.gaussian_blur_size, detection_params.gaussian_blur_size), 
                               0)
        
        # 检测矩形
        center_point, contour = find_black_rectangle_center(gray, prev_center)
        
        # 应用卡尔曼滤波
        filtered_point = None
        if center_point:
            filtered_point = kalman_filter.predict(center_point)
            prev_center = filtered_point
        else:
            if kalman_filter.predicted:
                filtered_point = kalman_filter.predicted
        
        # 计算性能指标
        process_time = (time.time() - process_start) * 1000
        processing_times.append(process_time)
        avg_process_time = sum(processing_times) / len(processing_times) if processing_times else 0
        
        frame_count += 1
        elapsed_time = time.time() - start_time
        if elapsed_time > 1:
            fps = frame_count / elapsed_time
            frame_count = 0
            start_time = time.time()
        
        # 创建显示图像
        display_img = process_frame.copy()
        height, width = display_img.shape[:2]
        img_center = (width // 2, height // 2)
        
        # 显示性能信息
        cv2.putText(display_img, f"FPS: {fps:.1f}", (10, 30), 
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv2.putText(display_img, f"Proc: {avg_process_time:.1f}ms", (10, 60), 
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        
        # 显示当前参数值（如果开启）
        if detection_params.show_params:
            y_offset = 90
            cv2.putText(display_img, f"Min Area: {detection_params.min_area}", (10, y_offset), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 200, 255), 1)
            cv2.putText(display_img, f"Min Rect: {detection_params.min_rectangularity:.2f}", (10, y_offset+25), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 200, 255), 1)
            cv2.putText(display_img, f"Max Aspect: {detection_params.max_aspect_ratio:.1f}", (10, y_offset+50), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 200, 255), 1)
            cv2.putText(display_img, f"Dist Weight: {detection_params.distance_weight:.1f}", (10, y_offset+75), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 200, 255), 1)
            
            # 显示PID参数
            cv2.putText(display_img, f"Pan Kp: {pid_params.pan_kp:.3f}", (10, y_offset+100), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 0, 200), 1)
            cv2.putText(display_img, f"Pan Ki: {pid_params.pan_ki:.3f}", (10, y_offset+125), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 0, 200), 1)
            cv2.putText(display_img, f"Pan Kd: {pid_params.pan_kd:.3f}", (10, y_offset+150), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 0, 200), 1)
            cv2.putText(display_img, f"Tilt Kp: {pid_params.tilt_kp:.3f}", (10, y_offset+175), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 0, 200), 1)
            cv2.putText(display_img, f"Tilt Ki: {pid_params.tilt_ki:.3f}", (10, y_offset+200), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 0, 200), 1)
            cv2.putText(display_img, f"Tilt Kd: {pid_params.tilt_kd:.3f}", (10, y_offset+225), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 0, 200), 1)
        
        # 绘制中心点
        cv2.circle(display_img, img_center, 5, (0, 0, 255), -1)
        
        # 如果找到矩形
        if filtered_point is not None:
            if trail_image is None:
                trail_image = np.zeros_like(display_img)
            
            cv2.circle(trail_image, filtered_point, 2, (0, 255, 255), -1)
            display_img = cv2.add(display_img, trail_image)
            cv2.circle(display_img, filtered_point, 8, (255, 0, 0), -1)
            
            text = f"({filtered_point[0]}, {filtered_point[1]})"
            cv2.putText(display_img, text, (filtered_point[0] + 10, filtered_point[1] - 10), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
            
            cv2.line(display_img, img_center, filtered_point, (0, 255, 0), 1)
            
            if contour is not None:
                cv2.drawContours(display_img, [contour], -1, (0, 255, 0), 2)
            
            # 计算偏移量
            offset_x = filtered_point[0] - img_center[0]
            offset_y = filtered_point[1] - img_center[1]
            
            # PID计算
            pan_output = pan_pid.get_pid(offset_x, pid_params.output_scaler)
            tilt_output = tilt_pid.get_pid(offset_y, pid_params.output_scaler)

            print("\033c", end="")
            print(f"=== 实时检测结果 (FPS: {fps:.1f}) ===")
            print(f"矩形中心坐标: ({filtered_point[0]}, {filtered_point[1]})")
            print(f"中心偏移量: X: {offset_x}, Y: {offset_y}")
            print(f"PID输出: Pan: {pan_output:.1f}, Tilt: {tilt_output:.1f}")
            print(f"处理延迟: {avg_process_time:.1f}ms")
            print("=" * 40)
            
            send_counter += 1
            if send_counter >= send_interval:
                # 发送PID输出值
                send_serial_data(pan_output, tilt_output, True)
                send_counter = 0
        else:
            # 未检测到矩形时重置PID控制器
            pan_pid.reset()
            tilt_pid.reset()
            
            print("\033c", end="")
            print(f"=== 实时检测结果 (FPS: {fps:.1f}) ===")
            print("未检测到黑色矩形")
            print(f"处理延迟: {avg_process_time:.1f}ms")
            print("=" * 40)
            
            trail_image = None
            send_counter += 1
            if send_counter >= send_interval:
                # 发送零值
                send_serial_data(0, 0, False)
                send_counter = 0
        
        # 显示帧
        cv2.imshow("Rectangle Detection (Press 'h' for help)", display_img)
        
        # 键盘控制
        key = cv2.waitKey(1)
        if key == 32:  # 空格键 - 退出
            stop_sending = True
        elif key == ord('c'):  # 清除轨迹
            trail_image = None
        elif key == ord('r'):  # 重置卡尔曼滤波器
            kalman_filter = KalmanFilter()
            prev_center = None
            print("卡尔曼滤波器已重置")
        elif key == ord('h'):  # 显示帮助
            print("\n===== 键盘控制帮助 =====")
            print("空格键: 退出程序")
            print("c: 清除轨迹")
            print("r: 重置卡尔曼滤波器")
            print("p: 显示/隐藏检测参数")
            print("1/2: 增加/减少最小面积")
            print("3/4: 增加/减少最小矩形度")
            print("5/6: 增加/减少最大长宽比")
            print("7/8: 增加/减少位置权重")
            print("9/0: 增加/减少形态学核大小")
            print("a/s: 增加/减少水平比例系数(Kp)")
            print("d/f: 增加/减少水平积分系数(Ki)")
            print("g/h: 增加/减少水平微分系数(Kd)")
            print("z/x: 增加/减少垂直比例系数(Kp)")
            print("c/v: 增加/减少垂直积分系数(Ki)")
            print("b/n: 增加/减少垂直微分系数(Kd)")
            print("m/,: 增加/减少输出缩放因子")
            print("h: 显示此帮助")
            print("=======================")
        elif key == ord('p'):  # 显示/隐藏参数
            detection_params.show_params = not detection_params.show_params
            print(f"参数显示: {'开启' if detection_params.show_params else '关闭'}")
        
        # 检测参数调整快捷键
        elif key == ord('1'):  # 增加最小面积
            detection_params.min_area += PARAM_STEP['min_area']
            print(f"最小面积: {detection_params.min_area}")
        elif key == ord('2'):  # 减少最小面积
            detection_params.min_area = max(10, detection_params.min_area - PARAM_STEP['min_area'])
            print(f"最小面积: {detection_params.min_area}")
        elif key == ord('3'):  # 增加最小矩形度
            detection_params.min_rectangularity = min(0.95, detection_params.min_rectangularity + PARAM_STEP['min_rectangularity'])
            print(f"最小矩形度: {detection_params.min_rectangularity:.2f}")
        elif key == ord('4'):  # 减少最小矩形度
            detection_params.min_rectangularity = max(0.1, detection_params.min_rectangularity - PARAM_STEP['min_rectangularity'])
            print(f"最小矩形度: {detection_params.min_rectangularity:.2f}")
        elif key == ord('5'):  # 增加最大长宽比
            detection_params.max_aspect_ratio += PARAM_STEP['max_aspect_ratio']
            print(f"最大长宽比: {detection_params.max_aspect_ratio:.1f}")
        elif key == ord('6'):  # 减少最大长宽比
            detection_params.max_aspect_ratio = max(1.0, detection_params.max_aspect_ratio - PARAM_STEP['max_aspect_ratio'])
            print(f"最大长宽比: {detection_params.max_aspect_ratio:.1f}")
        elif key == ord('7'):  # 增加位置权重
            detection_params.distance_weight = min(1.0, detection_params.distance_weight + PARAM_STEP['distance_weight'])
            print(f"位置权重: {detection_params.distance_weight:.1f}")
        elif key == ord('8'):  # 减少位置权重
            detection_params.distance_weight = max(0.0, detection_params.distance_weight - PARAM_STEP['distance_weight'])
            print(f"位置权重: {detection_params.distance_weight:.1f}")
        elif key == ord('9'):  # 增加形态学核大小
            detection_params.morph_kernel_size = min(15, detection_params.morph_kernel_size + PARAM_STEP['morph_kernel_size'])
            print(f"形态学核大小: {detection_params.morph_kernel_size}")
        elif key == ord('0'):  # 减少形态学核大小
            detection_params.morph_kernel_size = max(1, detection_params.morph_kernel_size - PARAM_STEP['morph_kernel_size'])
            print(f"形态学核大小: {detection_params.morph_kernel_size}")
            
        # PID参数调整快捷键
        elif key == ord('a'):  # 增加水平比例系数
            pid_params.pan_kp += PARAM_STEP['pan_kp']
            pan_pid = PID(pid_params.pan_kp, pid_params.pan_ki, pid_params.pan_kd, pid_params.pan_imax)
            print(f"水平比例系数(Kp): {pid_params.pan_kp:.3f}")
        elif key == ord('s'):  # 减少水平比例系数
            pid_params.pan_kp = max(0, pid_params.pan_kp - PARAM_STEP['pan_kp'])
            pan_pid = PID(pid_params.pan_kp, pid_params.pan_ki, pid_params.pan_kd, pid_params.pan_imax)
            print(f"水平比例系数(Kp): {pid_params.pan_kp:.3f}")
        elif key == ord('d'):  # 增加水平积分系数
            pid_params.pan_ki += PARAM_STEP['pan_ki']
            pan_pid = PID(pid_params.pan_kp, pid_params.pan_ki, pid_params.pan_kd, pid_params.pan_imax)
            print(f"水平积分系数(Ki): {pid_params.pan_ki:.3f}")
        elif key == ord('f'):  # 减少水平积分系数
            pid_params.pan_ki = max(0, pid_params.pan_ki - PARAM_STEP['pan_ki'])
            pan_pid = PID(pid_params.pan_kp, pid_params.pan_ki, pid_params.pan_kd, pid_params.pan_imax)
            print(f"水平积分系数(Ki): {pid_params.pan_ki:.3f}")
        elif key == ord('g'):  # 增加水平微分系数
            pid_params.pan_kd += PARAM_STEP['pan_kd']
            pan_pid = PID(pid_params.pan_kp, pid_params.pan_ki, pid_params.pan_kd, pid_params.pan_imax)
            print(f"水平微分系数(Kd): {pid_params.pan_kd:.3f}")
        elif key == ord('h'):  # 减少水平微分系数
            pid_params.pan_kd = max(0, pid_params.pan_kd - PARAM_STEP['pan_kd'])
            pan_pid = PID(pid_params.pan_kp, pid_params.pan_ki, pid_params.pan_kd, pid_params.pan_imax)
            print(f"水平微分系数(Kd): {pid_params.pan_kd:.3f}")
        elif key == ord('z'):  # 增加垂直比例系数
            pid_params.tilt_kp += PARAM_STEP['tilt_kp']
            tilt_pid = PID(pid_params.tilt_kp, pid_params.tilt_ki, pid_params.tilt_kd, pid_params.tilt_imax)
            print(f"垂直比例系数(Kp): {pid_params.tilt_kp:.3f}")
        elif key == ord('x'):  # 减少垂直比例系数
            pid_params.tilt_kp = max(0, pid_params.tilt_kp - PARAM_STEP['tilt_kp'])
            tilt_pid = PID(pid_params.tilt_kp, pid_params.tilt_ki, pid_params.tilt_kd, pid_params.tilt_imax)
            print(f"垂直比例系数(Kp): {pid_params.tilt_kp:.3f}")
        elif key == ord('c'):  # 增加垂直积分系数
            pid_params.tilt_ki += PARAM_STEP['tilt_ki']
            tilt_pid = PID(pid_params.tilt_kp, pid_params.tilt_ki, pid_params.tilt_kd, pid_params.tilt_imax)
            print(f"垂直积分系数(Ki): {pid_params.tilt_ki:.3f}")
        elif key == ord('v'):  # 减少垂直积分系数
            pid_params.tilt_ki = max(0, pid_params.tilt_ki - PARAM_STEP['tilt_ki'])
            tilt_pid = PID(pid_params.tilt_kp, pid_params.tilt_ki, pid_params.tilt_kd, pid_params.tilt_imax)
            print(f"垂直积分系数(Ki): {pid_params.tilt_ki:.3f}")
        elif key == ord('b'):  # 增加垂直微分系数
            pid_params.tilt_kd += PARAM_STEP['tilt_kd']
            tilt_pid = PID(pid_params.tilt_kp, pid_params.tilt_ki, pid_params.tilt_kd, pid_params.tilt_imax)
            print(f"垂直微分系数(Kd): {pid_params.tilt_kd:.3f}")
        elif key == ord('n'):  # 减少垂直微分系数
            pid_params.tilt_kd = max(0, pid_params.tilt_kd - PARAM_STEP['tilt_kd'])
            tilt_pid = PID(pid_params.tilt_kp, pid_params.tilt_ki, pid_params.tilt_kd, pid_params.tilt_imax)
            print(f"垂直微分系数(Kd): {pid_params.tilt_kd:.3f}")
        elif key == ord('m'):  # 增加输出缩放因子
            pid_params.output_scaler += PARAM_STEP['output_scaler']
            print(f"输出缩放因子: {pid_params.output_scaler:.1f}")
        elif key == ord(','):  # 减少输出缩放因子
            pid_params.output_scaler = max(0.1, pid_params.output_scaler - PARAM_STEP['output_scaler'])
            print(f"输出缩放因子: {pid_params.output_scaler:.1f}")

finally:
    capture.release()
    cv2.destroyAllWindows()
    if serial_port and serial_port.is_open:
        serial_port.close()
        print("串口已关闭")
    print("程序已退出")
