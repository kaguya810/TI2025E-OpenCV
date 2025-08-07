import threading
import time
import cv2
import numpy as np

class RectangleDetector:
    def __init__(self, detection_params):
        self.detection_params = detection_params
        self.frame = None
        self.result = (None, None)
        self.running = False
        self.lock = threading.Lock()
        self.thread = threading.Thread(target=self._run, daemon=True)

    def start(self):
        self.running = True
        self.thread.start()

    def stop(self):
        self.running = False
        self.thread.join()

    def update_frame(self, frame):
        with self.lock:
            self.frame = frame.copy()

    def get_result(self):
        with self.lock:
            return self.result

    def _run(self):
        prev_center = None
        while self.running:
            with self.lock:
                frame = self.frame.copy() if self.frame is not None else None
            if frame is None:
                time.sleep(0.01)
                continue
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            gray = cv2.GaussianBlur(gray, (self.detection_params.gaussian_blur_size, self.detection_params.gaussian_blur_size), 0)
            center, contour = find_black_rectangle_center(gray, prev_center, self.detection_params)
            with self.lock:
                self.result = (center, contour)
            prev_center = center
            time.sleep(0.01)

def find_black_rectangle_center(thresh, prev_center, detection_params):
    adaptive_thresh = cv2.adaptiveThreshold(
        thresh, 255,
        cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
        cv2.THRESH_BINARY_INV,
        detection_params.adaptive_block_size,
        detection_params.adaptive_c
    )
    kernel = np.ones((detection_params.morph_kernel_size, detection_params.morph_kernel_size), np.uint8)
    morphed = cv2.morphologyEx(adaptive_thresh, cv2.MORPH_CLOSE, kernel)
    morphed = cv2.dilate(morphed, kernel, iterations=1)
    contours, _ = cv2.findContours(morphed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None, None
    best_contour = None
    max_score = 0
    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area < detection_params.min_area:
            continue
        rect = cv2.minAreaRect(cnt)
        width, height = rect[1]
        if min(width, height) == 0:
            continue
        aspect_ratio = max(width, height) / min(width, height)
        rectangularity = area / (width * height)
        if aspect_ratio > detection_params.max_aspect_ratio:
            continue
        if rectangularity < detection_params.min_rectangularity:
            continue
        score = area + rectangularity * 100 + (1 / aspect_ratio) * 50
        if prev_center:
            M = cv2.moments(cnt)
            if M["m00"] != 0:
                cX = int(M["m10"] / M["m00"])
                cY = int(M["m01"] / M["m00"])
                distance = np.sqrt((cX - prev_center[0])**2 + (cY - prev_center[1])**2)
                distance_score = max(0, 100 - distance) * detection_params.distance_weight
                score += distance_score
        if score > max_score:
            max_score = score
            best_contour = cnt
    if best_contour is None:
        return None, None
    rect = cv2.minAreaRect(best_contour)
    center = (int(rect[0][0]), int(rect[0][1]))
    return center, best_contour