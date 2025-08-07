import cv2
import time
import threading
import sys
import select
import serial
from collections import deque
""" 以下是自建包，存放于include目录下 """
from include.SerialCtrl import SerialComm
from include.dect import RectangleDetector
from include.PWM import ServoController
from include.camera_reader import CameraReader
from include.pid import PID, PIDParams,PIDController
from include.display import DebugDisplay

# 串口配置参数
SERIAL_PORT = '/dev/ttyUSB0'  # 根据实际设备修改
BAUD_RATE = 9600
START1_SIGNAL = b'start1'  # 开始控制信号1
START2_SIGNAL = b'start2'  # 开始控制信号2 (舵机方向转换)
START3_SIGNAL = b'start3'  # 开始控制信号3（激光连续模式）
LASER_ON_SIGNAL = b'1;'    # 激光开启信号
LASER_OFF_SIGNAL = b'0;'   # 激光关闭信号

tilt_value = 600  # 垂直舵机初始值

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
