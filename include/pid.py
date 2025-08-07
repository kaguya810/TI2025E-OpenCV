import time
from math import pi, isnan

# PID控制器参数
class PIDParams:
    def __init__(self):
        # 水平方向PID参数
        self.pan_kp = 0.325
        self.pan_ki = 0.020
        self.pan_kd = 0.001
        self.pan_imax = 100
        
        # 垂直方向PID参数
        self.tilt_kp = 0.360
        self.tilt_ki = 0.0
        self.tilt_kd = 0.01
        self.tilt_imax = 100
        
        # 输出缩放因子
        self.output_scaler = 1.0

# PID控制器类
class PID:
    def __init__(self, p=0.0, i=0.0, d=0.0, imax=0.0):
        """
        PID控制器初始化
        :param p: 比例系数
        :param i: 积分系数
        :param d: 微分系数
        :param imax: 积分限幅值(绝对值)
        """
        self._kp = float(p)
        self._ki = float(i)
        self._kd = float(d)
        self._imax = abs(imax)
        self.reset()
    
    def reset(self):
        """重置控制器内部状态"""
        self._integrator = 0.0
        self._last_error = 0.0
        self._last_derivative = float('nan')
        self._last_t = 0
    
    def set_params(self, p, i, d, imax):
        """设置PID参数"""
        self._kp = float(p)
        self._ki = float(i)
        self._kd = float(d)
        self._imax = abs(imax)
    
    def get_pid(self, error, scaler=1.0):
        """
        计算PID输出
        :param error: 当前误差值
        :param scaler: 输出缩放因子
        :return: PID控制量
        """
        tnow = time.time() * 1000  # 转换为毫秒时间戳
        dt = tnow - self._last_t
        
        # 异常时间间隔处理
        if self._last_t == 0 or dt > 1000:
            dt = 0
            self.reset()
        
        self._last_t = tnow
        delta_time = dt / 1000.0  # 转换为秒
        
        # 比例项
        output = error * self._kp
        
        # 微分项(带低通滤波)
        if abs(self._kd) > 0 and dt > 0:
            if isnan(self._last_derivative):
                # 首次计算时初始化
                derivative = 0.0
                self._last_derivative = 0.0
            else:
                # 计算原始微分值
                derivative = (error - self._last_error) / delta_time
            
            # 应用一阶低通滤波(RC=1/(2π*20))
            RC = 1 / (2 * pi * 20)
            derivative = self._last_derivative + (
                (delta_time / (RC + delta_time)) * 
                (derivative - self._last_derivative)
            )
            
            self._last_error = error
            self._last_derivative = derivative
            output += self._kd * derivative
        
        # 应用输出缩放
        output *= scaler
        
        # 积分项(带限幅)
        if abs(self._ki) > 0 and dt > 0:
            # 积分项单独缩放
            self._integrator += (error * self._ki) * scaler * delta_time
            
            # 积分限幅
            if self._integrator < -self._imax:
                self._integrator = -self._imax
            elif self._integrator > self._imax:
                self._integrator = self._imax
                
            output += self._integrator
        
        return output

