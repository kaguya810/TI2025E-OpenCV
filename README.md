# TI2025E-OpenCV

本项目为基于 OpenCV 的舵机与激光控制实验平台，集成了摄像头采集、图像处理、PID 调节、舵机控制、激光控制与串口通信等功能，适用于目标检测与自动跟踪实验。

## 主要功能
- 实时摄像头采集与图像预处理
- 黑色矩形目标检测与中心点提取
- 多线程 PID 控制算法，支持参数热更新
- 舵机自动控制与方向反转
- 激光点射/连续控制逻辑，串口通信协议
- 详细调试信息与可视化界面
- Arduino 端激光与按键互锁程序

## 目录结构
- `main.py`：基础目标检测与串口输出 PID 控制量
- `main_servo.py`：完整流程，集成摄像头、舵机、激光控制
- `include/`
  - `camera_reader.py`：摄像头多线程采集
  - `dect.py`：黑色矩形检测与中心点提取
  - `display.py`：调试信息与可视化
  - `pid.py`：PID 控制算法及参数
  - `PWM.py`：PWM 舵机控制
  - `SerialCtrl.py`：串口通信封装
- `Arduino_Laseron/`：Arduino 激光/按键互锁程序

## 依赖环境
- Python 3.x
- OpenCV
- pyserial
- numpy
- collections
- python-periphery（如需 GPIO/PWM 支持）

安装依赖：
```bash
pip install opencv-python pyserial numpy python-periphery collections
```

## 运行方式
- 目标检测与串口输出：
  ```bash
  python main.py
  ```
- 舵机+激光一体控制：
  ```bash
  python main_servo.py
  ```
- 摄像头性能测试：
  ```bash
  python dxc_test.py
  ```
- 舵机测试：
  ```bash
  python pwmdemo.py
  ```

## 注意事项
- 需在 Linux 下运行，访问 GPIO/PWM 需 sudo 权限
- 串口端口、舵机参数等请根据实际硬件环境修改
- 建议使用 PlatformIO 上传 Arduino 端代码

## 推荐改进
1. GPIO/PWM 权限与 sudo 逻辑可抽象为共用工具
2. 主脚本建议进一步模块化，便于维护与测试
3. 敏感信息建议通过环境变量或配置文件管理
4. 建议用 logging 替换 print，统一异常处理
5. camera_reader.py 可优化线程休眠逻辑，降低 CPU 占用

---
如有问题或建议，欢迎 issue 反馈。
