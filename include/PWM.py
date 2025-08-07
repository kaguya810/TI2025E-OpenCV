# from periphery import PWM
import os
import subprocess
import sys
import time

# 常量：舵机PWM参数&权限配置
SERVO_PERIOD_NS = 20_000_000  # 50Hz = 20ms
MIN_PULSE_NS = 1_000_000      # 0度 = 1ms
MAX_PULSE_NS = 2_000_000      # 180度 = 2ms
USERNAME = "cat"          # 替换为您的用户名
PASSWORD = "temppwd"      # 替换为您的密码

def run_sudo_command(cmd: str, password: str):
    """使用 sudo 执行命令"""
    try:
        # 使用 -S 选项让 sudo 从标准输入读取密码
        process = subprocess.Popen(
            ['sudo', '-S'] + cmd.split(),
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True
        )
        
        # 发送密码 + 换行符
        stdout, stderr = process.communicate(input=password + '\n')
        
        if process.returncode != 0:
            print(f"[ERROR] 命令执行失败 (code {process.returncode}): {stderr.strip()}")
            return False
        return True
    except Exception as e:
        print(f"[ERROR] 执行命令时出错: {str(e)}")
        return False

def configure_pwm_permissions(servonum: int, password: str):
    """配置 PWM 设备权限"""
    # 1. 导出 PWM 通道
    run_sudo_command(f"echo 0 > /sys/class/pwm/pwmchip{servonum}/export", password)
    
    # 2. 设置 PWM 设备权限
    pwm_path = f"/sys/class/pwm/pwmchip{servonum}/pwm0"
    if os.path.exists(pwm_path):
        # 修改设备文件权限
        run_sudo_command(f"chmod 666 {pwm_path}/period", password)
        run_sudo_command(f"chmod 666 {pwm_path}/duty_cycle", password)
        run_sudo_command(f"chmod 666 {pwm_path}/enable", password)
        run_sudo_command(f"chmod 666 {pwm_path}/polarity", password)
        print(f"已配置舵机 {servonum} 的 PWM 权限")
    else:
        print(f"[WARNING] PWM 设备路径不存在: {pwm_path}")

def angle_to_duty_cycle(angle: int) -> float:
    """将角度(0-1023)映射到占空比(0.0-1.0)"""
    pulse_ns = MIN_PULSE_NS + (MAX_PULSE_NS - MIN_PULSE_NS) * (angle / 1023)
    return pulse_ns / SERVO_PERIOD_NS

class ServoController:
    def __init__(self, username: str = USERNAME, password: str = PASSWORD):
        self.username = username
        self.password = password
        self.pwm_instances = {}
        self.configured_chips = set()
        
        # 检查当前用户权限
        self.check_user_permissions()
    
    def check_user_permissions(self):
        """检查当前用户是否有权限访问 PWM 设备"""
        if os.geteuid() == 0:
            print("当前以 root 权限运行，无需额外权限")
            return True
        
        print(f"当前以用户 {os.getlogin()} 运行，需要配置 PWM 权限")
        return False
    
    def ensure_pwm_permissions(self, servonum: int):
        """确保 PWM 设备有适当权限"""
        if servonum in self.configured_chips:
            return True
        
        # 如果当前不是 root 用户，配置权限
        if os.geteuid() != 0:
            print(f"为舵机 {servonum} 配置 PWM 权限...")
            configure_pwm_permissions(servonum, self.password)
            self.configured_chips.add(servonum)
        return True
    
    def servoset(self, servonum: int, angle: int):
        """设置舵机角度"""
        # 确保有权限访问 PWM 设备
        self.ensure_pwm_permissions(servonum)
        
        # 计算占空比
        duty_cycle = angle_to_duty_cycle(angle)
        
        #print(f"\n=== 配置舵机 {servonum} 角度 {angle} ===")
        
        try:
            # 如果尚未打开，创建新的 PWM 实例
            if servonum not in self.pwm_instances:
                # 使用 sudo 权限运行当前脚本
                if os.geteuid() != 0:
                    print("尝试使用 sudo 权限重新运行...")
                    self.rerun_with_sudo()
                    return

                #pwm = PWM(chip=servonum, channel=0)
                pwm.frequency = 1 / (SERVO_PERIOD_NS * 1e-9)  # 50Hz
                pwm.duty_cycle = duty_cycle
                pwm.polarity = "normal"
                pwm.enable()
                self.pwm_instances[servonum] = pwm
            else:
                # 更新已存在的 PWM 实例
                self.pwm_instances[servonum].duty_cycle = duty_cycle
            
            #print(f"舵机 {servonum} 已设置到角度 {angle} (占空比: {duty_cycle:.2%})")
        
        except Exception as e:
            print(f"设置舵机 {servonum} 失败: {str(e)}")
            if "Permission denied" in str(e):
                print("检测到权限问题，尝试使用 sudo 重新运行...")
                self.rerun_with_sudo()
    
    def rerun_with_sudo(self):
        """使用 sudo 重新运行当前脚本"""
        try:
            # 获取当前脚本路径
            script_path = os.path.abspath(sys.argv[0])
            
            # 构建 sudo 命令
            sudo_cmd = f"echo '{self.password}' | sudo -S python3 {script_path}"
            
            # 执行命令
            subprocess.run(sudo_cmd, shell=True, check=True)
            
            # 退出当前进程
            sys.exit(0)
        except Exception as e:
            print(f"使用 sudo 重新运行时出错: {str(e)}")
            sys.exit(1)
    
    def servo_release(self, servonum: int):
        """释放舵机 PWM 资源并使其失去动力自由旋转"""
        if servonum in self.pwm_instances:
            try:
                # 关键修改：先禁用PWM输出使舵机失去动力
                self.pwm_instances[servonum].disable()
                # 然后关闭PWM设备
                self.pwm_instances[servonum].close()
                del self.pwm_instances[servonum]
                print(f"舵机 {servonum} 已释放并失去动力")
            except Exception as e:
                print(f"释放舵机 {servonum} 失败: {str(e)}")
        else:
            print(f"舵机 {servonum} 未初始化")


if __name__ == "__main__":
    # 创建 ServoController 实例
    controller = ServoController()
    # 设置舵机0到中位
    controller.servoset(servonum=3, angle=480)
    # 设置舵机1到最大角度
    controller.servoset(servonum=4, angle=512)
    # 保持8秒
    time.sleep(2)
    # 释放舵机
    controller.servo_release(servonum=3)
    controller.servo_release(servonum=4)
    
    print("所有舵机已释放")
