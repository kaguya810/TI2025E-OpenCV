# TI2025E-OpenCV

基于 OpenCV 的舵机与激光控制实验项目，包含摄像头采集、图像处理、PID 调节以及串口通信等功能，同时提供 Arduino 端的激光与按键互锁程序。

## 目录结构

- `main.py`：使用 OpenCV 检测黑色矩形并通过串口输出 PID 控制量。
- `main_servo.py`：结合摄像头、舵机和激光模块的完整流程控制脚本。
- `camera_reader.py`：独立线程读取摄像头帧，避免主循环阻塞。
- `GPIO.py`：基于 python-periphery 的 GPIO 封装，自动处理权限。
- `PWM.py`：PWM 舵机控制器，支持 sudo 自动提权。
- `pid.py`：PID 控制算法及默认参数。
- `dxc_test.py`：摄像头性能测试脚本。
- `pwmdemo.py`：舵机运动示例。
- `Arduino_Laseron/`：基于 PlatformIO 的 Arduino 程序，用于激光和按键互锁控制。

## 依赖

需要 Python 3 环境，并安装以下依赖：

```bash
pip install opencv-python pyserial python-periphery numpy
```

访问 GPIO / PWM 设备需要运行在具备 sudo 权限的 Linux 系统。

## 运行示例

- 摄像头检测与串口输出：`python main.py`
- 舵机+激光一体控制：`python main_servo.py`
- 摄像头 FPS 测试：`python dxc_test.py`
- 舵机测试：`python pwmdemo.py`

Arduino 端代码位于 `Arduino_Laseron/src/main.cpp`，使用 PlatformIO 编译并上传至开发板。

## 计划修改

1. `GPIO.py` 与 `PWM.py` 中的 sudo 调用和权限配置逻辑相似，可抽取为共用工具以减少重复。
2. 多个主脚本体积较大，建议拆分为模块并补充单元测试，提升可维护性。
3. 用户名和密码以明文写入源码，推荐改为环境变量或配置文件形式。
4. `camera_reader.py` 中线程持续循环，可加入适当的睡眠或条件变量以降低 CPU 占用。
5. 使用 `logging` 模块替代大量 `print`，统一串口、舵机和摄像头异常的处理和记录。
