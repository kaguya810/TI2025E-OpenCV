import cv2
import numpy as np
import time
import threading
import sys
import select
import serial
from collections import deque
from PWM import ServoController
from camera_reader import CameraReader
from include.pid import PID, PIDParams,PIDController

# 串口配置参数
SERIAL_PORT = '/dev/ttyUSB0'  # 根据实际设备修改
BAUD_RATE = 9600
START1_SIGNAL = b'start1'  # 开始控制信号1
START2_SIGNAL = b'start2'  # 开始控制信号2 (舵机方向转换)
START3_SIGNAL = b'start3'  # 开始控制信号3（激光连续模式）
LASER_ON_SIGNAL = b'1;'    # 激光开启信号
LASER_OFF_SIGNAL = b'0;'   # 激光关闭信号

tilt_value = 700  # 垂直舵机初始值

# 可调检测参数
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

detection_params = DetectionParams()
pid_params = PIDParams()

class DebugDisplay:
    def __init__(self, detection_params, pid_params):
        self.detection_params = detection_params
        self.pid_params = pid_params
        self.show_params = False
        self.trail_image = None
        self.fps = 0
        self.avg_process_time = 0
        self.in_center_zone = False
        self.center_stay_timer = 0

    def update(self, fps, avg_process_time, in_center_zone, center_stay_timer):
        self.fps = fps
        self.avg_process_time = avg_process_time
        self.in_center_zone = in_center_zone
        self.center_stay_timer = center_stay_timer

    def draw(self, display_img, img_center, filtered_point, contour, control_enabled, laser_active, current_mode):
        # 性能信息
        cv2.putText(display_img, f"FPS: {self.fps:.1f}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv2.putText(display_img, f"Proc: {self.avg_process_time:.1f}ms", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        control_status = "CTRL: " + ("ON" if control_enabled else "OFF")
        cv2.putText(display_img, control_status, (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0) if control_enabled else (0, 0, 255), 2)
        laser_status = "LASER: " + ("ON" if laser_active else "OFF")
        cv2.putText(display_img, laser_status, (10, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0) if laser_active else (0, 0, 255), 2)
        mode_status = f"MODE: {current_mode}"
        cv2.putText(display_img, mode_status, (10, 150), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 200, 255), 2)
        if self.in_center_zone:
            stay_time = time.time() - self.center_stay_timer
            cv2.putText(display_img, f"Stay: {stay_time:.1f}s", (10, 180), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        if self.show_params:
            y_offset = 210
            cv2.putText(display_img, f"Min Area: {self.detection_params.min_area}", (10, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 200, 255), 1)
            cv2.putText(display_img, f"Min Rect: {self.detection_params.min_rectangularity:.2f}", (10, y_offset+25), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 200, 255), 1)
            cv2.putText(display_img, f"Max Aspect: {self.detection_params.max_aspect_ratio:.1f}", (10, y_offset+50), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 200, 255), 1)
            cv2.putText(display_img, f"Dist Weight: {self.detection_params.distance_weight:.1f}", (10, y_offset+75), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 200, 255), 1)
            cv2.putText(display_img, f"Pan Kp: {self.pid_params.pan_kp:.3f}", (10, y_offset+100), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 0, 200), 1)
            cv2.putText(display_img, f"Pan Ki: {self.pid_params.pan_ki:.3f}", (10, y_offset+125), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 0, 200), 1)
            cv2.putText(display_img, f"Pan Kd: {self.pid_params.pan_kd:.3f}", (10, y_offset+150), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 0, 200), 1)
            cv2.putText(display_img, f"Tilt Kp: {self.pid_params.tilt_kp:.3f}", (10, y_offset+175), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 0, 200), 1)
            cv2.putText(display_img, f"Tilt Ki: {self.pid_params.tilt_ki:.3f}", (10, y_offset+200), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 0, 200), 1)
            cv2.putText(display_img, f"Tilt Kd: {self.pid_params.tilt_kd:.3f}", (10, y_offset+225), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 0, 200), 1)
        # 绘制中心点和区域
        cv2.circle(display_img, img_center, 5, (0, 0, 255), -1)
        center_zone_size = 48
        cv2.rectangle(display_img, (img_center[0] - center_zone_size, img_center[1] - center_zone_size), (img_center[0] + center_zone_size, img_center[1] + center_zone_size), (0, 255, 255), 1)
        # 轨迹与目标
        if filtered_point is not None:
            if self.trail_image is None or self.trail_image.shape != display_img.shape:
                self.trail_image = np.zeros_like(display_img)
            cv2.circle(self.trail_image, filtered_point, 2, (0, 255, 255), -1)
            display_img = cv2.add(display_img, self.trail_image)
            cv2.circle(display_img, filtered_point, 8, (255, 0, 0), -1)
            text = f"({filtered_point[0]}, {filtered_point[1]})"
            cv2.putText(display_img, text, (filtered_point[0] + 10, filtered_point[1] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
            cv2.line(display_img, img_center, filtered_point, (0, 255, 0), 1)
            if contour is not None:
                cv2.drawContours(display_img, [contour], -1, (0, 255, 0), 2)
        return display_img

class SerialComm:
    def __init__(self, port, baudrate, timeout=0.1):
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.ser = None
        self.buffer = bytearray()
        self._init_serial()

    def _init_serial(self):
        try:
            self.ser = serial.Serial(self.port, self.baudrate, timeout=self.timeout)
            print(f"串口初��化成功: {self.port}")
        except Exception as e:
            print(f"串口初始化失败: {e}")
            self.ser = None

    def is_open(self):
        return self.ser is not None and self.ser.is_open

    def write(self, cmd):
        try:
            if self.is_open():
                self.ser.write(cmd)
        except Exception as e:
            print(f"串口写入异常: {e}")

    def read_all(self):
        if self.is_open() and self.ser.in_waiting > 0:
            data = self.ser.read(self.ser.in_waiting)
            self.buffer.extend(data)
            return data
        return b''

    def check_signal(self, signal):
        if signal in self.buffer:
            index = self.buffer.find(signal)
            self.buffer = self.buffer[index + len(signal):]
            return True
        return False

    def clear_buffer(self):
        self.buffer.clear()

    def close(self):
        if self.is_open():
            self.ser.close()
            print("串口已关闭")

class RectangleDetector:
    def __init__(self, detection_params):
        self.detection_params = detection_params
        self.frame = None
        self.result = (None, None)
        self.running = False
        self.lock = threading.Lock()
        self.thread = threading.Thread(target=self._run, daemon=True)

    def start(self):
        self.running = True
        self.thread.start()

    def stop(self):
        self.running = False
        self.thread.join()

    def update_frame(self, frame):
        with self.lock:
            self.frame = frame.copy()

    def get_result(self):
        with self.lock:
            return self.result

    def _run(self):
        prev_center = None
        while self.running:
            with self.lock:
                frame = self.frame.copy() if self.frame is not None else None
            if frame is None:
                time.sleep(0.01)
                continue
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            gray = cv2.GaussianBlur(gray, (self.detection_params.gaussian_blur_size, self.detection_params.gaussian_blur_size), 0)
            center, contour = find_black_rectangle_center(gray, prev_center, self.detection_params)
            with self.lock:
                self.result = (center, contour)
            prev_center = center
            time.sleep(0.01)

def find_black_rectangle_center(thresh, prev_center, detection_params):
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
# kalman_filter = KalmanFilter(process_noise=1e-4, measurement_noise=1e-2)
prev_center = None
frame_queue = deque(maxlen=3)
trail_image = None
control_enabled = False  # 控制状态标志
laser_sent = False       # 激光发射标志
serial_buffer = bytearray()  # 串口接收缓冲区
laser_timer = 0          # 激光计时器
laser_active = False     # 激光激活状态
current_mode = "idle"    # 当前模式: idle/start1/start2
center_stay_timer = 0    # 中心区域停留计时器
in_center_zone = False   # 是否在中心区域
fanzhuan = False  # 是否反转舵机方向

# 初始化舵机控制器
controller = ServoController()
# 设置舵机初始位置
controller.servoset(servonum=3, angle=480)  # 水平舵机
controller.servoset(servonum=4, angle=700)  # 垂直舵机

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
serial_comm = SerialComm(SERIAL_PORT, BAUD_RATE, timeout=0.1)

def safe_serial_write(cmd):
    """安全写串口，防止阻塞"""
    try:
        if serial_comm.is_open():
            serial_comm.write(cmd)
    except Exception as e:
        print(f"串口写入异常: {e}")

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
    """控制舵机运动，优化反转逻辑和边界处理"""
    try:
        global tilt_value
        global fanzhuan
        global controller
        # 舵机边界
        PAN_MIN, PAN_MAX = 128, 892
        TILT_MIN, TILT_MAX = 256, 700
        PAN_CENTER = 480
        PAN_STEP = 30
        # 未检测到目标时，自动扫描，方向可反转
        if not detected and control_enabled:
            pan_value = PAN_CENTER + (PAN_STEP if not fanzhuan else -PAN_STEP)
            # 垂直舵机保持当前值
        else:
            # 反转逻辑：pan_output正负方向取反
            pan_dir = -1 if fanzhuan else 1
            pan_value = int(PAN_CENTER - pan_output * 0.5 * pan_dir)
            pan_value = max(PAN_MIN, min(PAN_MAX, pan_value))
            tilt_value = int(tilt_value - tilt_output * 0.2)
            tilt_value = max(TILT_MIN, min(TILT_MAX, tilt_value))
        controller.servoset(servonum=3, angle=pan_value)
        controller.servoset(servonum=4, angle=tilt_value)
        print(f"舵机控制: 水平: {pan_value}, 垂直: {tilt_value}, 检测: {detected}, 反转: {fanzhuan}")
    except Exception as e:
        print(f"舵机控制错误: {e}")

# 启动键盘监听线程
try:
    input_thread = threading.Thread(target=input_listener)
    input_thread.daemon = True
    input_thread.start()
    print("按 'q' 停��程序")
except:
    print("无法启动输入监听线程")

# 使用CameraReader初始化摄像头
try:
    camera_reader = CameraReader(
        cam_id=0, 
        width=800, 
        height=600, 
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
    'pan_kp': 0.002,
    'pan_ki': 0.001,
    'pan_kd': 0.005,
    'tilt_kp': 0.01,
    'tilt_ki': 0.001,
    'tilt_kd': 0.005,
    'output_scaler': 0.1
}

# 指示预备
if serial_comm.is_open():
    serial_comm.write(b'2;')  # 指示预备

# 初始化多线程检测与PID控制类
rect_detector = RectangleDetector(detection_params)
rect_detector.start()
pid_controller = PIDController(pid_params)
pid_controller.start()
debug_display = DebugDisplay(detection_params, pid_params)

try:
    while not stop_sending:
        # 检查串口信号（非阻塞方式）
        serial_comm.read_all()
        # 检查start1信号
        if serial_comm.check_signal(START1_SIGNAL):
            control_enabled = True
            laser_sent = False
            laser_active = False
            current_mode = "start1"
            in_center_zone = False
            center_stay_timer = 0
            print("接收到开始控制信号 (模式1)")
        # 检查start2信号
        elif serial_comm.check_signal(START2_SIGNAL):
            control_enabled = True
            current_mode = "start2"
            in_center_zone = False
            center_stay_timer = 0
            fanzhuan = not fanzhuan
        # 检查start3信号
        if serial_comm.check_signal(START3_SIGNAL):
            control_enabled = True
            current_mode = "start3"
            if serial_comm.is_open():
                serial_comm.write(LASER_ON_SIGNAL)
                print("发送激光开启指令 (连续模式)")
            laser_active = True
            laser_sent = True
            print("接收到开始控制信号 (模式3)")
        # 清空串口缓冲区，防止信号残留
        serial_comm.clear_buffer()

        # 处理激光控制
        if control_enabled:
            if current_mode == "start3":
                # 连续模式：确保激光保持开启
                if not laser_sent:
                    if serial_comm.is_open():
                        serial_comm.write(LASER_ON_SIGNAL)
                        print("发送激光开启指令 (连续模式)")
                    laser_sent = True
                    laser_active = True
            elif current_mode in ("start1", "start2"):
                # 点射模式：严格依赖检测和中心区域
                current_time = time.time()
                # 只有激光已激活时才计时关闭
                if laser_active and (current_time - laser_timer >= 2.0):
                    if serial_comm.is_open():
                        serial_comm.write(LASER_OFF_SIGNAL)
                        print("发送激光关闭指令")
                    laser_active = False
                    current_mode = "start1"
                    laser_sent = False
                    serial_comm.clear_buffer()

        # 从CameraReader获取帧
        retval, frame = camera_reader.read()
        if not retval:
            print("无法从摄像头读取帧，等待下一帧...")
            time.sleep(0.01)
            continue
        process_start = time.time()
        # 多线���检测
        rect_detector.update_frame(frame)
        center_point, contour = rect_detector.get_result()
        filtered_point = center_point if center_point else None
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
        display_img = frame.copy()
        height, width = display_img.shape[:2]
        img_center = (width // 2, (height // 2) - 4)
        # 计算偏移量
        if filtered_point is not None:
            offset_x = filtered_point[0] - img_center[0]
            offset_y = filtered_point[1] - img_center[1]
        else:
            offset_x = 0
            offset_y = 0
        # 更新PID参数（热更新，确保PID对象参数同步）
        pid_controller.update_pid_params(pid_params)
        pid_controller.update_offset((offset_x, offset_y))
        pan_output, tilt_output = pid_controller.get_output()
        # 检查中心区域
        center_zone_size = 48
        in_center = filtered_point is not None and abs(offset_x) < center_zone_size and abs(offset_y) < center_zone_size
        if in_center:
            if not in_center_zone:
                in_center_zone = True
                center_stay_timer = time.time()
        else:
            in_center_zone = False
        debug_display.update(fps, avg_process_time, in_center_zone, center_stay_timer)
        display_img = debug_display.draw(display_img, img_center, filtered_point, contour, control_enabled, laser_active, current_mode)

        # 控制舵机运动
        if control_enabled:
            control_servos(pan_output, tilt_output, filtered_point is not None)
            # 激光发射条件：start1/start2模式+中心区域+未激活+停留时间满足
            if current_mode in ("start1", "start2") and in_center_zone and not laser_active:
                if time.time() - center_stay_timer >= 0.6:
                    if serial_comm.is_open():
                        serial_comm.write(LASER_ON_SIGNAL)
                        print("发送激光开启指令 (满足停留时间)")
                    laser_active = True
                    laser_sent = True
                    laser_timer = time.time()
        else:
            control_servos(0, 0, False)

        # 打印调试信息
        print("\033c", end="")
        print(f"=== 实时检测结果 (FPS: {fps:.1f}) ===")
        if filtered_point is not None:
            print(f"矩形中心坐标: ({filtered_point[0]}, {filtered_point[1]})")
            print(f"中心偏移量: X: {offset_x}, Y: {offset_y}")
            if in_center_zone:
                stay_time = time.time() - center_stay_timer
                print(f"中心区域停留: {stay_time:.2f}s")
        print(f"控制状态: {'已启用' if control_enabled else '已禁用'}")
        print(f"激光状态: {'开启' if laser_active else '关闭'}")
        print(f"当前模式: {current_mode}")
        print(f"处理延迟: {avg_process_time:.1f}ms")
        print("=" * 40)

        send_counter += 1
        if send_counter >= send_interval:
            send_counter = 0
except Exception as e:
    print(f"运行时错误: {e}")
finally:
    rect_detector.stop()
    pid_controller.stop()
    # 停止摄像头
    if 'camera_reader' in locals():
        camera_reader.stop()
    
    cv2.destroyAllWindows()           
    # 释放舵机
    controller.servo_release(servonum=3)
    controller.servo_release(servonum=4)
    
    # 确保激光关闭
    if laser_active and serial_comm.is_open():
        serial_comm.write(LASER_OFF_SIGNAL)
        print("发送激光关闭指令")
    
    # 关闭串口
    serial_comm.close()
    print("舵机已释放")
    print("程序已退出")
