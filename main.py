import cv2
import numpy as np
import time
import serial
import struct
import threading
import sys
import select

def find_black_rectangle_center(thresh):
    """在帧中查找黑色矩形并返回其中心点坐标"""
    # 查找轮廓
    contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    # 寻找最大矩形轮廓
    max_rect = None
    max_area = 0
    for cnt in contours:
        # 计算轮廓面积
        area = cv2.contourArea(cnt)
        if area < 100:  # 忽略小面积噪声
            continue
            
        # 多边形逼近
        peri = cv2.arcLength(cnt, True)
        approx = cv2.approxPolyDP(cnt, 0.02 * peri, True)
        
        # 检查是否为四边形
        if len(approx) == 4:
            # 检查凸性
            if cv2.isContourConvex(approx):
                if area > max_area:
                    max_area = area
                    max_rect = approx
    
    if max_rect is None:
        return None
    
    # 计算矩形的中心点
    M = cv2.moments(max_rect)
    if M["m00"] != 0:
        cX = int(M["m10"] / M["m00"])
        cY = int(M["m01"] / M["m00"])
        return (cX, cY)
    else:
        # 如果无法计算矩心，使用四个点的平均值
        points = max_rect.reshape(4, 2)
        cX = int(np.mean(points[:, 0]))
        cY = int(np.mean(points[:, 1]))
        return (cX, cY)

# 全局变量，控制发送循环
stop_sending = False
serial_port = None

def input_listener():
    """监听键盘输入，如果输入 'q' 则停止发送"""
    global stop_sending
    while not stop_sending:
        try:
            # 非阻塞方式检查输入
            if sys.stdin in select.select([sys.stdin], [], [], 0)[0]:
                user_input = sys.stdin.read(1)
                if user_input.lower() == 'q':
                    stop_sending = True
                    print("\n停止发送...")
        except:
            break

def send_serial_data(offset_x, offset_y, detected):
    """通过串口发送偏移量数据"""
    global serial_port
    if serial_port and serial_port.is_open:
        try:
            # 帧结构: 帧头(0x2C, 0x12), 偏移量X(2字节), 偏移量Y(2字节), 检测标志(1字节), 保留位(1字节), 帧尾(0x5B)
            # 使用小端字节序打包数据
            data = struct.pack('<hhbb', 
                               int(offset_x),  # X偏移量
                               int(offset_y),  # Y偏移量
                               1 if detected else 0,  # 检测标志
                               0x5B)  # 帧尾
            serial_port.write(data)
            # print(f"发送数据: X={offset_x}, Y={offset_y}, 检测={detected}")
        except Exception as e:
            print(f"串口发送错误: {e}")

# 初始化串口
try:
    # 请根据实际情况修改串口号
    # Windows: 'COM3', Linux: '/dev/ttyUSB0'
    serial_port = serial.Serial(
        port='/dev/ttyUSB0',  # 修改为你的串口号
        baudrate=9600,
        bytesize=serial.EIGHTBITS,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        timeout=0.1
    )
    time.sleep(1)  # 等待串口初始化
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

# 初始化摄像头捕获
capture = cv2.VideoCapture(0)

# 检查摄像头是否成功打开
if not capture.isOpened():
    print("无法访问摄像头！")
    exit()

# 关闭自动设置
capture.set(cv2.CAP_PROP_AUTO_WB, 0)       # 关闭自动白平衡
capture.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.25)  # 手动曝光模式
capture.set(cv2.CAP_PROP_EXPOSURE, -4)     # 曝光值（根据实际环境调整）

# 帧率计算变量
frame_count = 0
start_time = time.time()
fps = 0

# 创建用于绘制轨迹的空白图像
trail_image = None

# 发送计数器（控制发送频率）
send_counter = 0
send_interval = 2  # 每2帧发送一次数据

try:
    while capture.isOpened() and not stop_sending:
        # 读取帧
        retval, frame = capture.read()

        if not retval:
            print("无法获取帧！")
            break
            
        # 转换为灰度图
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        # 使用Canny边缘检测
        thresh = cv2.Canny(gray, 75, 200)
        
        # 查找黑色矩形中心
        center_point = find_black_rectangle_center(thresh)
        
        # 计算帧率
        frame_count += 1
        elapsed_time = time.time() - start_time
        if elapsed_time > 1:  # 每秒更新一次帧率
            fps = frame_count / elapsed_time
            frame_count = 0
            start_time = time.time()
        
        # 创建用于显示的彩色图像
        display_img = cv2.cvtColor(thresh, cv2.COLOR_GRAY2BGR)
        
        # 获取图像中心
        height, width = display_img.shape[:2]
        img_center = (width // 2, height // 2)
        
        # 在图像上显示帧率
        cv2.putText(display_img, f"FPS: {fps:.1f}", (10, 30), 
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        
        # 绘制图像中心点
        cv2.circle(display_img, img_center, 5, (0, 0, 255), -1)
        cv2.putText(display_img, "Center", (img_center[0] + 10, img_center[1] - 10), 
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
        
        # 如果找到矩形中心
        if center_point is not None:
            # 初始化轨迹图像
            if trail_image is None:
                trail_image = np.zeros_like(display_img)
            
            # 在轨迹图像上添加当前点
            cv2.circle(trail_image, center_point, 2, (0, 255, 255), -1)
            
            # 将轨迹叠加到显示图像上
            display_img = cv2.add(display_img, trail_image)
            
            # 绘制当前中心点
            cv2.circle(display_img, center_point, 8, (255, 0, 0), -1)
            
            # 显示坐标
            text = f"({center_point[0]}, {center_point[1]})"
            cv2.putText(display_img, text, (center_point[0] + 10, center_point[1] - 10), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
            
            # 绘制到图像中心的连线
            cv2.line(display_img, img_center, center_point, (0, 255, 0), 1)
            
            # 计算偏移量
            offset_x = center_point[0] - img_center[0]
            offset_y = center_point[1] - img_center[1]
            
            # 在终端打印中心点坐标和偏移量
            print("\033c", end="")  # 清空终端
            print(f"=== 实时检测结果 (FPS: {fps:.1f}) ===")
            print(f"矩形中心坐标: ({center_point[0]}, {center_point[1]})")
            print(f"中心偏移量: X: {offset_x}, Y: {offset_y}")
            print("=" * 40)
            
            # 发送串口数据（控制发送频率）
            send_counter += 1
            if send_counter >= send_interval:
                send_serial_data(offset_x, offset_y, True)
                send_counter = 0
        else:
            # 未检测到矩形时清屏并显示提示
            print("\033c", end="")  # 清空终端
            print(f"=== 实时检测结果 (FPS: {fps:.1f}) ===")
            print("未检测到黑色矩形")
            print("=" * 40)
            
            # 重置轨迹图像
            trail_image = None
            
            # 发送未检测到矩形的信号
            send_counter += 1
            if send_counter >= send_interval:
                send_serial_data(0, 0, False)
                send_counter = 0
        
        # 显示帧
        cv2.imshow("Black Rectangle Center Detection", display_img)
        
        # 按空格键退出（ASCII码32是空格键）
        key = cv2.waitKey(1)
        if key == 32:  # 空格键
            stop_sending = True
        elif key == ord('c'):  # 按'c'键清除轨迹
            trail_image = None

finally:
    # 确保资源被释放
    capture.release()
    cv2.destroyAllWindows()
    if serial_port and serial_port.is_open:
        serial_port.close()
        print("串口已关闭")
    print("程序已退出")
