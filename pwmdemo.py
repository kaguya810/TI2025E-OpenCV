import subprocess

def run_sudo_command(cmd: str, password: str):
    """
    以sudo权限执行命令并自动传递密码
    """
    proc = subprocess.run(f"sudo -S {cmd}", 
                          input=password + "\n", 
                          shell=True, 
                          text=True, 
                          capture_output=True)
    if proc.returncode != 0:
        print(f"[ERROR] 命令执行失败: {cmd}\n{proc.stderr}")
    else:
        print(f"[OK] {cmd}")

def pwm_control():
    sudo_password = "temppwd"

    # 用户输入
    pwmchip = input("请输入PWM Chip编号 (例如 3 表示 pwmchip3): ").strip() or "3"
    pwm_channel = input("请输入PWM通道编号 (例如 0 表示 pwm0): ").strip() or "0"
    period = input("请输入PWM周期 (ns, 默认1000000): ").strip() or "1000000"
    duty_cycle = input("请输入PWM占空比 (ns, 默认500000): ").strip() or "500000"
    polarity = "normal"  # 固定为 normal

    pwmchip_path = f"pwmchip{pwmchip}"
    pwm_path = f"pwm{pwm_channel}"

    print("\n=== 开始配置PWM ===")

    # 1. 导出PWM
    run_sudo_command(f"echo {pwm_channel} > /sys/class/pwm/{pwmchip_path}/export", sudo_password)

    # 2. 设置周期
    run_sudo_command(f"echo {period} > /sys/class/pwm/{pwmchip_path}/{pwm_path}/period", sudo_password)

    # 3. 设置占空比
    run_sudo_command(f"echo {duty_cycle} > /sys/class/pwm/{pwmchip_path}/{pwm_path}/duty_cycle", sudo_password)

    # 4. 设置极性
    run_sudo_command(f"echo {polarity} > /sys/class/pwm/{pwmchip_path}/{pwm_path}/polarity", sudo_password)

    # 5. 使能PWM
    run_sudo_command(f"echo 1 > /sys/class/pwm/{pwmchip_path}/{pwm_path}/enable", sudo_password)

    input("\n按回车键取消导出PWM...")

    # 6. 取消导出
    run_sudo_command(f"echo {pwm_channel} > /sys/class/pwm/{pwmchip_path}/unexport", sudo_password)
    print("\n=== PWM配置完成 ===")

if __name__ == "__main__":
    pwm_control()

