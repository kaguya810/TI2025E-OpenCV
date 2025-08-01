import cv2
import numpy as np
import time
import threading
import sys
import select
import serial  # 新增串口通信模块
from collections import deque
from math import pi, isnan
from PWM import ServoController
from camera_reader import CameraReader
from pid import PID, PIDParams

# 串口配置参数
SERIAL_PORT = '/dev/ttyUSB0'  # 根据实际设备修改
BAUD_RATE = 9600
START_SIGNAL = b's'  # 开始控制信号
LASER_SIGNAL = b'f'  # 激光发射信号

tilt_value = 700  # 垂直舵机初始值

# 可调检测参数
class DetectionParams:
    def __init__(self):
        self.min_area = 3320
        self.min_rectangularity = 0.7
        self.max_aspect_ratio = 5.0
        self.distance_weight = 0.3
        self.adaptive_block_size = 11
        self.adaptive_c = 2
        self.morph_kernel_size = 3
        self.gaussian_blur_size = 5
        self.show_params = False

detection_params = DetectionParams()
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
    """优化后的矩形检测函数"""
    adaptive_thresh = cv2.adaptiveThreshold(
        thresh, 255, 
        cv2.ADAPTIVE_THRESH_GAUSSIAN_C, 
        cv2.THRESH_BINARY_INV, 
        detection_params.adaptive_block_size, 
        detection_params.adaptive_c
    )
    
    kernel = np.ones((detection_params.morph_kernel_size, detection_params.morph_kernel_size), np.uint8)
    morphed = cv2.morphologyEx(adaptive_thresh, cv2.MORPH_CLOSE, kernel)
    morphed = cv2.dilate(morphed, kernel, iterations=1)
    
    contours, _ = cv2.findContours(morphed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    if not contours:
        return None, None
    
    best_contour = None
    max_score = 0
    
    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area < detection_params.min_area:
            continue
            
        rect = cv2.minAreaRect(cnt)
        width, height = rect[1]
        if min(width, height) == 0:
            continue
            
        aspect_ratio = max(width, height) / min(width, height)
        rectangularity = area / (width * height)
        
        if aspect_ratio > detection_params.max_aspect_ratio:
            continue
        if rectangularity < detection_params.min_rectangularity:
            continue
        
        score = area + rectangularity * 100 + (1 / aspect_ratio) * 50
        
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
    
    rect = cv2.minAreaRect(best_contour)
    center = (int(rect[0][0]), int(rect[0][1]))
    
    return center, best_contour

# 全局变量
stop_sending = False
kalman_filter = KalmanFilter(process_noise=1e-4, measurement_noise=1e-2)
prev_center = None
frame_queue = deque(maxlen=3)
trail_image = None
control_enabled = True  # 控制状态标志
laser_sent = False       # 激光发射标志

# 初始化舵机控制器
controller = ServoController()
# 设置舵机初始位置
controller.servoset(servonum=3, angle=480)  # 水平舵机
controller.servoset(servonum=4, angle=768)  # 垂直舵机

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

# 初始化串口
try:
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1)
    print(f"串口初始化成功: {SERIAL_PORT}")
except Exception as e:
    print(f"串口初始化失败: {e}")
    ser = None

def input_listener():
    """监听键盘输入"""
    global stop_sending
    while not stop_sending:
        try:
            if sys.stdin in select.select([sys.stdin], [], [], 0)[0]:
                user_input = sys.stdin.read(1)
                if user_input.lower() == 'q':
                    stop_sending = True
                    print("\n停止程序...")
        except:
            break

def control_servos(pan_output, tilt_output, detected):
    """控制舵机运动"""
    try:
        global tilt_value
        pan_value = int(480 - pan_output * 0.8)
        pan_value = max(256, min(768, pan_value))
        
        tilt_value = int(tilt_value - tilt_output * 0.2)
        tilt_value = max(256, min(1023, tilt_value))
        
        controller.servoset(servonum=3, angle=pan_value)
        controller.servoset(servonum=4, angle=tilt_value)
        
        print(f"舵机控制: 水平: {pan_value}, 垂直: {tilt_value}, 检测: {detected}")
    except Exception as e:
        print(f"舵机控制错误: {e}")

# 启动键盘监听线程
try:
    input_thread = threading.Thread(target=input_listener)
    input_thread.daemon = True
    input_thread.start()
    print("按 'q' 停止程序")
except:
    print("无法启动输入监听线程")

# 使用CameraReader初始化摄像头
try:
    camera_reader = CameraReader(
        cam_id=0, 
        width=640, 
        height=480, 
        max_fps=60
    )
    print("摄像头初始化成功")
except Exception as e:
    print(f"摄像头初始化失败: {e}")
    exit()

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
    'pan_kp': 0.01,
    'pan_ki': 0.001,
    'pan_kd': 0.005,
    'tilt_kp': 0.01,
    'tilt_ki': 0.001,
    'tilt_kd': 0.005,
    'output_scaler': 0.1
}

try:
    while not stop_sending:
        # 检查串口信号
        if ser and ser.in_waiting > 0:
            data = ser.read(1)
            if data == START_SIGNAL:
                control_enabled = True
                laser_sent = False  # 重置激光发射标志
                print("接收到开始控制信号")
        
        # 从CameraReader获取帧
        retval, frame = camera_reader.read()
        if not retval:
            print("无法从摄像头读取帧，等待下一帧...")
            time.sleep(0.01)
            continue
            
        process_start = time.time()
        
        # 预处理
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
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
        display_img = frame.copy()
        height, width = display_img.shape[:2]
        img_center = (width // 2, height // 2)
        
        # 显示性能信息
        cv2.putText(display_img, f"FPS: {fps:.1f}", (10, 30), 
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv2.putText(display_img, f"Proc: {avg_process_time:.1f}ms", (10, 60), 
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        # 显示控制状态
        control_status = "控制状态: " + ("已启用" if control_enabled else "已禁用")
        cv2.putText(display_img, control_status, (10, 90), 
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0) if control_enabled else (0, 0, 255), 2)
    
        # 显示当前参数值
        if detection_params.show_params:
            y_offset = 120
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
            
            # 仅在控制启用时进行PID计算
            if control_enabled:
                pan_output = pan_pid.get_pid(offset_x, pid_params.output_scaler)
                tilt_output = tilt_pid.get_pid(offset_y, pid_params.output_scaler)
            else:
                pan_output = 0
                tilt_output = 0

            print("\033c", end="")
            print(f"=== 实时检测结果 (FPS: {fps:.1f}) ===")
            print(f"矩形中心坐标: ({filtered_point[0]}, {filtered_point[1]})")
            print(f"中心偏移量: X: {offset_x}, Y: {offset_y}")
            print(f"控制状态: {'已启用' if control_enabled else '已禁用'}")
            if control_enabled:
                print(f"PID输出: Pan: {pan_output:.1f}, Tilt: {tilt_output:.1f}")
            print(f"处理延迟: {avg_process_time:.1f}ms")
            print("=" * 40)
            
            send_counter += 1
            if send_counter >= send_interval:
                # 仅在控制启用时控制舵机
                if control_enabled:
                    control_servos(pan_output, tilt_output, True)
                    
                    # 检查是否满足激光发射条件
                    if abs(offset_x) < 16 and abs(offset_y) < 16 and not laser_sent:
                        if ser:
                            ser.write(LASER_SIGNAL)
                            print("发送激光发射指令")
                        laser_sent = True
                send_counter = 0
        else:
            # 未检测到矩形时重置PID控制器
            pan_pid.reset()
            tilt_pid.reset()
            laser_sent = False  # 重置激光发射标志
            
            print("\033c", end="")
            print(f"=== 实时检测结果 (FPS: {fps:.1f}) ===")
            print("未检测到黑色矩形")
            print(f"控制状态: {'已启用' if control_enabled else '已禁用'}")
            print(f"处理延迟: {avg_process_time:.1f}ms")
            print("=" * 40)
            
            trail_image = None
            send_counter += 1
            if send_counter >= send_interval:
                # 仅在控制启用时控制舵机
                if control_enabled:
                    control_servos(0, 0, False)
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
    # 停止摄像头
    if 'camera_reader' in locals():
        camera_reader.stop()
    
    cv2.destroyAllWindows()           
    # 释放舵机
    controller.servo_release(servonum=3)
    controller.servo_release(servonum=4)
    # 关闭串口
    if ser and ser.is_open:
        ser.close()
    print("舵机已释放")
    print("串口已关闭")
    print("程序已退出")
