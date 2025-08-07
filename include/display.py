import time
import cv2
import numpy as np


class DebugDisplay:
    def __init__(self, detection_params, pid_params):
        self.detection_params = detection_params
        self.pid_params = pid_params
        self.show_params = False
        self.trail_image = None
        self.fps = 0
        self.avg_process_time = 0
        self.in_center_zone = False
        self.center_stay_timer = 0

    def update(self, fps, avg_process_time, in_center_zone, center_stay_timer):
        self.fps = fps
        self.avg_process_time = avg_process_time
        self.in_center_zone = in_center_zone
        self.center_stay_timer = center_stay_timer

    def draw(self, display_img, img_center, filtered_point, contour, control_enabled, laser_active, current_mode):
        # 性能信息
        cv2.putText(display_img, f"FPS: {self.fps:.1f}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv2.putText(display_img, f"Proc: {self.avg_process_time:.1f}ms", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        control_status = "CTRL: " + ("ON" if control_enabled else "OFF")
        cv2.putText(display_img, control_status, (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0) if control_enabled else (0, 0, 255), 2)
        laser_status = "LASER: " + ("ON" if laser_active else "OFF")
        cv2.putText(display_img, laser_status, (10, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0) if laser_active else (0, 0, 255), 2)
        mode_status = f"MODE: {current_mode}"
        cv2.putText(display_img, mode_status, (10, 150), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 200, 255), 2)
        if self.in_center_zone:
            stay_time = time.time() - self.center_stay_timer
            cv2.putText(display_img, f"Stay: {stay_time:.1f}s", (10, 180), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        if self.show_params:
            y_offset = 210
            cv2.putText(display_img, f"Min Area: {self.detection_params.min_area}", (10, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 200, 255), 1)
            cv2.putText(display_img, f"Min Rect: {self.detection_params.min_rectangularity:.2f}", (10, y_offset+25), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 200, 255), 1)
            cv2.putText(display_img, f"Max Aspect: {self.detection_params.max_aspect_ratio:.1f}", (10, y_offset+50), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 200, 255), 1)
            cv2.putText(display_img, f"Dist Weight: {self.detection_params.distance_weight:.1f}", (10, y_offset+75), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 200, 255), 1)
            cv2.putText(display_img, f"Pan Kp: {self.pid_params.pan_kp:.3f}", (10, y_offset+100), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 0, 200), 1)
            cv2.putText(display_img, f"Pan Ki: {self.pid_params.pan_ki:.3f}", (10, y_offset+125), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 0, 200), 1)
            cv2.putText(display_img, f"Pan Kd: {self.pid_params.pan_kd:.3f}", (10, y_offset+150), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 0, 200), 1)
            cv2.putText(display_img, f"Tilt Kp: {self.pid_params.tilt_kp:.3f}", (10, y_offset+175), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 0, 200), 1)
            cv2.putText(display_img, f"Tilt Ki: {self.pid_params.tilt_ki:.3f}", (10, y_offset+200), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 0, 200), 1)
            cv2.putText(display_img, f"Tilt Kd: {self.pid_params.tilt_kd:.3f}", (10, y_offset+225), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 0, 200), 1)
        # 绘制中心点和区域
        cv2.circle(display_img, img_center, 5, (0, 0, 255), -1)
        center_zone_size = 48
        cv2.rectangle(display_img, (img_center[0] - center_zone_size, img_center[1] - center_zone_size), (img_center[0] + center_zone_size, img_center[1] + center_zone_size), (0, 255, 255), 1)
        # 轨迹与目标
        if filtered_point is not None:
            if self.trail_image is None or self.trail_image.shape != display_img.shape:
                self.trail_image = np.zeros_like(display_img)
            cv2.circle(self.trail_image, filtered_point, 2, (0, 255, 255), -1)
            display_img = cv2.add(display_img, self.trail_image)
            cv2.circle(display_img, filtered_point, 8, (255, 0, 0), -1)
            text = f"({filtered_point[0]}, {filtered_point[1]})"
            cv2.putText(display_img, text, (filtered_point[0] + 10, filtered_point[1] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
            cv2.line(display_img, img_center, filtered_point, (0, 255, 0), 1)
            if contour is not None:
                cv2.drawContours(display_img, [contour], -1, (0, 255, 0), 2)
        return display_img
