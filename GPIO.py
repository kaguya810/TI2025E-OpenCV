from periphery import GPIO
import re
import os
import subprocess
import sys

# 默认用户凭证（根据实际环境修改）
USERNAME = "cat"
PASSWORD = "temppwd"

def run_sudo_command(cmd: str, password: str):
    """使用sudo执行系统命令"""
    try:
        process = subprocess.Popen(
            ['sudo', '-S'] + cmd.split(),
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True
        )
        stdout, stderr = process.communicate(input=password + '\n')
        return process.returncode == 0
    except Exception as e:
        print(f"[ERROR] Command execution failed: {str(e)}")
        return False

class GPIO_ctrl:
    # 类变量：记录已配置权限的GPIO芯片
    configured_chips = set()
    
    def __init__(self, gpio_name: str, direction: str, 
                 username: str = USERNAME, password: str = PASSWORD):
        """
        初始化GPIO控制器
        :param gpio_name: GPIO名称 (e.g., "GPIO3_A6")
        :param direction: 方向模式 ("in" 或 "out")
        :param username: 系统用户名
        :param password: 用户密码
        """
        self.gpio_name = gpio_name
        self.direction = direction.lower()
        self.username = username
        self.password = password
        self.gpio_num = self._parse_gpio_name()
        self.gpio_handle = None
        
        # 验证方向参数
        if self.direction not in ["in", "out"]:
            raise ValueError(f"Invalid direction: {direction}. Must be 'in' or 'out'")
        
        # 计算GPIO芯片和偏移量
        self.chip_index = self.gpio_num // 32
        self.offset = self.gpio_num % 32
        self.device_path = f"/dev/gpiochip{self.chip_index}"
        
        # 权限检查和初始化
        self._ensure_permissions()
        self._initialize_gpio()
    
    def _parse_gpio_name(self) -> int:
        """解析GPIO名称为全局编号"""
        match = re.match(r"GPIO(\d+)_([A-D])(\d+)", self.gpio_name)
        if not match:
            raise ValueError(f"Invalid GPIO name format: {self.gpio_name}")
        
        bank = int(match.group(1))
        port_letter = match.group(2)
        pin = int(match.group(3))
        
        # 端口字母转数字 (A=0, B=1, C=2, D=3)
        port_map = {'A': 0, 'B': 1, 'C': 2, 'D': 3}
        if port_letter not in port_map:
            raise ValueError(f"Invalid port letter: {port_letter}")
        
        return bank * 32 + port_map[port_letter] * 8 + pin
    
    def _ensure_permissions(self):
        """确保GPIO设备访问权限"""
        # Root用户无需配置
        if os.geteuid() == 0:
            return
        
        # 已配置过的芯片跳过
        if self.chip_index in GPIO_ctrl.configured_chips:
            return
        
        # 配置设备权限
        if run_sudo_command(f"chmod 666 {self.device_path}", self.password):
            print(f"Configured permissions for {self.device_path}")
            GPIO_ctrl.configured_chips.add(self.chip_index)
        else:
            print(f"[WARNING] Failed to configure {self.device_path}")
    
    def _initialize_gpio(self):
        """初始化GPIO设备"""
        try:
            # 使用字符串方向参数
            self.gpio_handle = GPIO(self.device_path, self.offset, self.direction)
        except PermissionError:
            self._rerun_with_sudo()
        except Exception as e:
            raise RuntimeError(f"GPIO initialization failed: {str(e)}")
    
    def _rerun_with_sudo(self):
        """使用sudo权限重新运行程序"""
        script_path = os.path.abspath(sys.argv[0])
        sudo_cmd = f"echo '{self.password}' | sudo -S python3 {script_path}"
        subprocess.run(sudo_cmd, shell=True, check=True)
        sys.exit(0)
    
    def read(self) -> bool:
        """读取GPIO当前值（返回布尔值）"""
        if not self.gpio_handle:
            raise RuntimeError("GPIO not initialized")
        return self.gpio_handle.read()
    
    def read_int(self) -> int:
        """读取GPIO当前值（返回0/1）"""
        return 1 if self.read() else 0
    
    def write(self, value: bool):
        """写入GPIO值 (仅输出模式，使用布尔值)"""
        if not self.gpio_handle:
            raise RuntimeError("GPIO not initialized")
        if self.direction != "out":
            raise RuntimeError("GPIO not in output mode")
        self.gpio_handle.write(value)
    
    def write_int(self, value: int):
        """写入GPIO值 (仅输出模式，使用0/1)"""
        self.write(value != 0)
    
    def close(self):
        """释放GPIO资源"""
        if self.gpio_handle:
            self.gpio_handle.close()
            self.gpio_handle = None

if __name__ == "__main__":
    # 使用示例
    try:
        # 创建输入模式GPIO (GPIO3_A6)
        input_gpio = GPIO_ctrl("GPIO3_A6", "in")
        print(f"GPIO3_A6 value (bool): {input_gpio.read()}")
        print(f"GPIO3_A6 value (int): {input_gpio.read_int()}")
        
        # 创建输出模式GPIO (GPIO3_B7)
        output_gpio = GPIO_ctrl("GPIO3_B7", "out")
        
        # 使用布尔值写入
        output_gpio.write(True)  # 设置高电平
        print("GPIO3_B7 set to HIGH (bool)")
        
        # 使用整数值写入
        output_gpio.write_int(0)  # 设置低电平
        print("GPIO3_B7 set to LOW (int)")
        
        # 清理资源
        input_gpio.close()
        output_gpio.close()
    except Exception as e:
        print(f"GPIO operation failed: {str(e)}")
