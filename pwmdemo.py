from include.PWM import ServoController
import time

# 创建 ServoController 实例
controller = ServoController()
# 设置舵机0到中位
controller.servoset(servonum=3, angle=480)
# 设置舵机1到最大角度
controller.servoset(servonum=4, angle=0)
# 保持2秒
time.sleep(8)
# 释放舵机
controller.servo_release(servonum=3)
controller.servo_release(servonum=4)