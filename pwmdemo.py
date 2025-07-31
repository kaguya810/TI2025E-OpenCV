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
    # 线性映射: 0 -> MIN_PULSE_NS, 1023 -> MAX_PULSE_NS
    duty = MIN_PULSE_NS + (MAX_PULSE_NS - MIN_PULSE_NS) * (angle / 1023)
    return int(duty)


def pwm_control():
    sudo_password = "temppwd"

    # 用户输入
    pwmchip = input("请输入舵机编号 (对应 pwmchipX): ").strip() or "0"
    pwm_channel = "0"  # 默认通道固定为0
    angle = int(input("请输入舵机角度 (0-1023): ").strip() or "512")
    duty_cycle = angle_to_duty_cycle(angle)
    polarity = "normal"  # 固定为 normal

    pwmchip_path = f"pwmchip{pwmchip}"
    pwm_path = f"pwm{pwm_channel}"

    print("\n=== 开始配置舵机PWM ===")

    # 1. 导出PWM
    run_root_command(f"echo {pwm_channel} > /sys/class/pwm/{pwmchip_path}/export", sudo_password)

    # 2. 设置周期 (50Hz)
    run_root_command(f"echo {SERVO_PERIOD_NS} > /sys/class/pwm/{pwmchip_path}/{pwm_path}/period", sudo_password)

    # 3. 设置占空比 (根据角度计算)
    run_root_command(f"echo {duty_cycle} > /sys/class/pwm/{pwmchip_path}/{pwm_path}/duty_cycle", sudo_password)

    # 4. 设置极性
    run_root_command(f"echo {polarity} > /sys/class/pwm/{pwmchip_path}/{pwm_path}/polarity", sudo_password)

    # 5. 使能PWM
    run_root_command(f"echo 1 > /sys/class/pwm/{pwmchip_path}/{pwm_path}/enable", sudo_password)

    input("\n按回车键取消导出PWM...")

    # 6. 取消导出
    run_root_command(f"echo {pwm_channel} > /sys/class/pwm/{pwmchip_path}/unexport", sudo_password)
    print("\n=== 舵机PWM配置完成 ===")


if __name__ == "__main__":
    pwm_control()
