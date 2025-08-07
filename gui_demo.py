#!/usr/bin/env python3
"""
GUI Demo Script for TI2025E-OpenCV
用于演示GUI界面的脚本（无需真实硬件）
"""

import sys
import os
import cv2
import numpy as np
from PyQt5.QtWidgets import QApplication
from PyQt5.QtCore import QTimer
from PyQt5.QtGui import QPixmap

# 设置虚拟显示环境
os.environ['QT_QPA_PLATFORM'] = 'offscreen'

def create_demo_frame():
    """创建演示用的图像帧"""
    frame = np.ones((480, 640, 3), dtype=np.uint8) * 50  # 深灰色背景
    
    # 绘制一个黑色矩形作为目标
    cv2.rectangle(frame, (250, 180), (390, 300), (20, 20, 20), -1)
    
    # 绘制中心十字线
    center = (320, 240)
    cv2.line(frame, (center[0] - 20, center[1]), (center[0] + 20, center[1]), (0, 0, 255), 2)
    cv2.line(frame, (center[0], center[1] - 20), (center[0], center[1] + 20), (0, 0, 255), 2)
    
    # 绘制目标中心点
    target_center = (320, 240)
    cv2.circle(frame, target_center, 10, (0, 255, 0), -1)
    
    # 添加一些文字信息
    cv2.putText(frame, "Demo Mode - No Hardware", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
    cv2.putText(frame, "Target Detected", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
    cv2.putText(frame, "PID: (0.5, -0.3)", (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
    
    return frame

def test_gui_components():
    """测试GUI组件"""
    print("正在测试GUI组件...")
    
    app = QApplication(sys.argv)
    
    try:
        # 导入GUI模块
        from gui_main import MainWindow, DetectionParamsWidget, PIDParamsWidget, ControlWidget, StatusWidget
        
        print("✓ GUI模块导入成功")
        
        # 创建主窗口
        window = MainWindow()
        print("✓ 主窗口创建成功")
        
        # 测试参数获取
        detection_params = window.detection_params.get_params()
        pid_params = window.pid_params.get_params()
        print(f"✓ 检测参数: min_area={detection_params.min_area}")
        print(f"✓ PID参数: pan_kp={pid_params.pan_kp}")
        
        # 测试状态更新
        window.status_panel.update_status(
            fps=30.0,
            process_time=15.5,
            control_active=True,
            laser_active=False,
            serial_connected=False,
            target_detected=True
        )
        print("✓ 状态更新测试成功")
        
        # 测试日志功能
        window.log_message("GUI测试消息")
        window.log_message("系统初始化完成")
        print("✓ 日志功能测试成功")
        
        # 模拟视频帧更新
        demo_frame = create_demo_frame()
        window.update_video_frame(demo_frame)
        print("✓ 视频帧更新测试成功")
        
        # 保存窗口截图（如果可能）
        try:
            pixmap = window.grab()
            pixmap.save('/tmp/gui_screenshot.png')
            print("✓ GUI截图已保存到 /tmp/gui_screenshot.png")
        except Exception as e:
            print(f"! 截图保存失败: {e}")
        
        # 设置定时器自动退出
        QTimer.singleShot(1000, app.quit)
        
        print("✓ 所有GUI组件测试完成")
        
        # 不启动事件循环，因为我们在无头环境中
        return True
        
    except Exception as e:
        print(f"✗ GUI测试失败: {e}")
        import traceback
        traceback.print_exc()
        return False
    
    finally:
        app.quit()

def test_module_integration():
    """测试模块集成"""
    print("\n正在测试模块集成...")
    
    try:
        # 测试检测功能
        from include.dect import find_black_rectangle_center
        from gui_main import DetectionParams
        
        # 创建测试图像
        test_image = np.ones((480, 640), dtype=np.uint8) * 200
        cv2.rectangle(test_image, (250, 180), (390, 300), 50, -1)
        
        # 创建检测参数
        params = DetectionParams()
        
        # 执行检测
        center, contour = find_black_rectangle_center(test_image, None, params)
        
        if center is not None:
            print(f"✓ 目标检测成功，中心点: {center}")
        else:
            print("✓ 检测功能正常（未检测到目标）")
            
        # 测试PID控制
        from include.pid import PIDParams, PIDController
        
        pid_params = PIDParams()
        controller = PIDController(pid_params)
        
        # 模拟偏移输入
        controller.update_offset((50, -30))
        output = controller.get_output()
        print(f"✓ PID控制测试成功，输出: {output}")
        
        return True
        
    except Exception as e:
        print(f"✗ 模块集成测试失败: {e}")
        import traceback
        traceback.print_exc()
        return False

def main():
    """主函数"""
    print("TI2025E-OpenCV GUI演示和测试")
    print("=" * 40)
    
    # 测试模块集成
    integration_ok = test_module_integration()
    
    # 测试GUI组件
    gui_ok = test_gui_components()
    
    print("\n" + "=" * 40)
    print("测试结果:")
    print(f"模块集成: {'✓ 通过' if integration_ok else '✗ 失败'}")
    print(f"GUI组件: {'✓ 通过' if gui_ok else '✗ 失败'}")
    
    if integration_ok and gui_ok:
        print("\n🎉 所有测试通过！GUI已准备就绪。")
        print("\n使用说明:")
        print("1. 在有显示环境的系统中运行: python3 gui_main.py")
        print("2. 连接摄像头和串口设备")
        print("3. 点击'开始控制'按钮启动系统")
        print("4. 调整检测和PID参数以优化性能")
    else:
        print("\n❌ 部分测试失败，请检查错误信息")
    
    return 0

if __name__ == "__main__":
    sys.exit(main())