"""
from servo_control import ServoController

# 创建舵机控制器实例
servo = ServoController(password="temppwd")

# 设置舵机编号0到中位角度512
servo.servoset(servonum=0, angle=512)

# 设置舵机编号1到最大角度1023
servo.servoset(servonum=1, angle=1023)

# 释放舵机0的PWM控制
servo.servo_release(servonum=0)
"""
import subprocess

# 常量：舵机PWM参数
SERVO_PERIOD_NS = 20_000_000  # 50Hz = 20ms = 20,000,000ns
MIN_PULSE_NS = 1_000_000      # 0度 = 1ms
MAX_PULSE_NS = 2_000_000      # 180度 = 2ms


def run_root_command(cmd: str, password: str):
    """
    通过sudo su获取root权限后执行命令
    """
    proc = subprocess.run(f"echo {password} | sudo -S su -c \"{cmd}\"",
                          shell=True,
                          text=True,
                          capture_output=True)
    if proc.returncode != 0:
        print(f"[ERROR] 命令执行失败:\n{proc.stderr}")


def angle_to_duty_cycle(angle: int) -> int:
    """
    将角度(0-1023)映射到对应的舵机占空比(ns)
    """
    duty = MIN_PULSE_NS + (MAX_PULSE_NS - MIN_PULSE_NS) * (angle / 1023)
    return int(duty)


class ServoController:
    def __init__(self, password: str = "temppwd"):
        self.password = password

    def servoset(self, servonum: int, angle: int):
        pwm_channel = "0"  # 固定为通道0
        duty_cycle = angle_to_duty_cycle(angle)
        polarity = "normal"

        pwmchip_path = f"pwmchip{servonum}"
        pwm_path = f"pwm{pwm_channel}"

        print(f"\n=== 配置舵机 {servonum} 角度 {angle} ===")

        # 1. 导出PWM
        run_root_command(f"echo {pwm_channel} > /sys/class/pwm/{pwmchip_path}/export", self.password)

        # 2. 设置周期 (50Hz)
        run_root_command(f"echo {SERVO_PERIOD_NS} > /sys/class/pwm/{pwmchip_path}/{pwm_path}/period", self.password)

        # 3. 设置占空比
        run_root_command(f"echo {duty_cycle} > /sys/class/pwm/{pwmchip_path}/{pwm_path}/duty_cycle", self.password)

        # 4. 设置极性
        run_root_command(f"echo {polarity} > /sys/class/pwm/{pwmchip_path}/{pwm_path}/polarity", self.password)

        # 5. 使能PWM
        run_root_command(f"echo 1 > /sys/class/pwm/{pwmchip_path}/{pwm_path}/enable", self.password)

        print(f"舵机 {servonum} 已设置到角度 {angle}\n")

    def servo_release(self, servonum: int):
        pwm_channel = "0"
        pwmchip_path = f"pwmchip{servonum}"
        run_root_command(f"echo {pwm_channel} > /sys/class/pwm/{pwmchip_path}/unexport", self.password)
        print(f"舵机 {servonum} PWM已释放\n")


if __name__ == "__main__":
    controller = ServoController()
    controller.servoset(servonum=0, angle=512)  # 示例：舵机0，角度中位
    input("\n按回车键释放舵机...")
    controller.servo_release(servonum=0)
