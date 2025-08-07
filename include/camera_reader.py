import cv2
import threading

class CameraReader:
    def __init__(self, cam_id=0, width=1280, height=720, max_fps=30, settings=None):
        # 在摄像头启动时显示测试图片（如果运行在独立模式）
        import os
        test_image_path = "camera_startup_test.jpg"
        if os.path.exists(test_image_path):
            print("CameraReader: 加载摄像头启动测试图片...")
            startup_image = cv2.imread(test_image_path)
            if startup_image is not None:
                print("CameraReader: 测试图片加载成功")
            else:
                print("CameraReader: 警告 - 无法加载测试图片")
        
        print("CameraReader: 正在初始化摄像头...")
        self.cap = cv2.VideoCapture(cam_id, cv2.CAP_V4L2)
        if not self.cap.isOpened():
            raise RuntimeError("无法打开摄像头")
        
        # 设置基本参数
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        self.cap.set(cv2.CAP_PROP_FPS, max_fps)
        self.cap.set(cv2.CAP_PROP_AUTO_WB,0)
        self.cap.set(cv2.CAP_PROP_AUTO_EXPOSURE,0.25 )# 手动曝光
        self.cap.set(cv2.CAP_PROP_EXPOSURE,-4        ) # 曝光值
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE,1)
        
        # 应用额外设置
        if settings:
            for prop, value in settings.items():
                self.cap.set(prop, value)
        
        print("CameraReader: 摄像头参数设置完成，等待设备稳定...")
        
        self.ret = False
        self.frame = None
        self.running = True
        self.lock = threading.Lock()

        self.thread = threading.Thread(target=self._update, daemon=True)
        self.thread.start()
        
        print("CameraReader: 摄像头初始化完成")

    def _update(self):
        while self.running:
            ret, frame = self.cap.read()
            if not ret:
                continue
            with self.lock:
                self.ret = ret
                self.frame = frame

    def read(self):
        with self.lock:
            return self.ret, self.frame.copy() if self.frame is not None else None

    def stop(self):
        self.running = False
        self.thread.join()
        self.cap.release()
        print("摄像头资源已释放")
