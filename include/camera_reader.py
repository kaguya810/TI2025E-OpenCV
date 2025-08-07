import cv2
import threading

class CameraReader:
    def __init__(self, cam_id=0, width=1280, height=720, max_fps=30, settings=None):
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
        
        self.ret = False
        self.frame = None
        self.running = True
        self.lock = threading.Lock()

        self.thread = threading.Thread(target=self._update, daemon=True)
        self.thread.start()

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
