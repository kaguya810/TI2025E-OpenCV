#!/usr/bin/env python3
"""
GUI启动脚本 - TI2025E-OpenCV 舵机激光控制系统
GUI Launcher Script for TI2025E-OpenCV Servo/Laser Control System
"""

import sys
import os

def check_environment():
    """检查运行环境"""
    print("正在检查运行环境...")
    
    # 检查Python版本
    if sys.version_info < (3, 6):
        print("❌ 需要Python 3.6或更高版本")
        return False
    
    print(f"✓ Python版本: {sys.version}")
    
    # 检查关键依赖
    required_modules = [
        'cv2', 'numpy', 'PyQt5', 'serial', 'time', 'threading'
    ]
    
    missing_modules = []
    for module in required_modules:
        try:
            __import__(module)
            print(f"✓ {module}")
        except ImportError:
            missing_modules.append(module)
            print(f"❌ {module}")
    
    if missing_modules:
        print(f"\n缺少依赖模块: {', '.join(missing_modules)}")
        print("请运行: pip install opencv-python PyQt5 pyserial numpy python-periphery")
        return False
    
    # 检查项目文件
    required_files = [
        'gui_main.py',
        'include/camera_reader.py',
        'include/dect.py',
        'include/pid.py',
        'include/SerialCtrl.py',
        'include/PWM.py',
        'include/display.py'
    ]
    
    missing_files = []
    for file in required_files:
        if not os.path.exists(file):
            missing_files.append(file)
            print(f"❌ {file}")
        else:
            print(f"✓ {file}")
    
    if missing_files:
        print(f"\n缺少项目文件: {', '.join(missing_files)}")
        return False
    
    return True

def check_hardware():
    """检查硬件设备"""
    print("\n正在检查硬件设备...")
    
    # 检查摄像头
    import cv2
    camera_available = False
    for i in range(3):  # 检查前3个摄像头
        try:
            cap = cv2.VideoCapture(i)
            if cap.isOpened():
                ret, frame = cap.read()
                if ret:
                    camera_available = True
                    print(f"✓ 摄像头 {i} 可用")
                    cap.release()
                    break
                cap.release()
        except:
            pass
    
    if not camera_available:
        print("⚠️  未检测到摄像头，将使用演示模式")
    
    # 检查串口设备
    serial_ports = ['/dev/ttyUSB0', '/dev/ttyUSB1', '/dev/ttyACM0']
    serial_available = False
    
    for port in serial_ports:
        if os.path.exists(port):
            try:
                import serial
                ser = serial.Serial(port, 9600, timeout=1)
                ser.close()
                print(f"✓ 串口 {port} 可用")
                serial_available = True
                break
            except:
                pass
    
    if not serial_available:
        print("⚠️  未检测到串口设备，激光控制将不可用")
    
    return camera_available, serial_available

def main():
    """主函数"""
    print("TI2025E-OpenCV GUI启动检查")
    print("=" * 50)
    
    # 环境检查
    if not check_environment():
        print("\n❌ 环境检查失败，无法启动GUI")
        return 1
    
    # 硬件检查
    camera_ok, serial_ok = check_hardware()
    
    print("\n" + "=" * 50)
    print("检查完成，准备启动GUI...")
    
    if not camera_ok:
        print("⚠️  注意：摄像头不可用，某些功能可能无法正常工作")
    
    if not serial_ok:
        print("⚠️  注意：串口不可用，激光控制功能将不可用")
    
    print("\n正在启动GUI界面...")
    
    try:
        # 导入并启动GUI
        from gui_main import main as gui_main
        return gui_main()
        
    except KeyboardInterrupt:
        print("\n用户中断，退出程序")
        return 0
        
    except Exception as e:
        print(f"\n❌ GUI启动失败: {e}")
        import traceback
        traceback.print_exc()
        return 1

if __name__ == "__main__":
    sys.exit(main())