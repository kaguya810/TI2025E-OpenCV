# serial_txd.py
import time
import serial
import threading
import sys

# 全局变量，控制发送循环
stop_sending = False

def input_listener():
    """监听键盘输入，如果输入 'q' 则停止发送"""
    global stop_sending
    while not stop_sending:
        try:
            # 在 Linux 下，`input()` 会阻塞，所以我们需要非阻塞方式
            # 这里使用 `sys.stdin` 检查是否有输入
            if sys.stdin in select.select([sys.stdin], [], [], 0)[0]:
                user_input = sys.stdin.read(1)
                if user_input.lower() == 'q':
                    stop_sending = True
                    print("\nStopping the sender...")
        except:
            break

# 初始化串口
com = serial.Serial(
    port='/dev/ttyUSB0',
    baudrate=115200,
    bytesize=serial.EIGHTBITS,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
)

time.sleep(1)  # 等待串口初始化

# 启动键盘监听线程（非阻塞）
try:
    import select  # 用于非阻塞输入检测
    input_thread = threading.Thread(target=input_listener)
    input_thread.daemon = True  # 设置为守护线程，主线程退出时自动结束
    input_thread.start()
except ImportError:
    print("Warning: `select` module not available, keyboard input may not work.")

print("Starting sender. Press 'q' to stop.")

# 主循环：发送数据
try:
    while not stop_sending:
        hex_str = 'AA55'
        print("send")
        com.write(bytes.fromhex(hex_str))
        time.sleep(1)
except KeyboardInterrupt:
    print("\nReceived keyboard interrupt, stopping...")
finally:
    com.close()
    print("Serial port closed.")
