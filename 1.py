import time
import cv2
from camera_reader import CameraReader
max_fps=40
def main():
    cam = CameraReader(cam_id=0, width=1280, height=720, max_fps=max_fps)

    prev_time = time.time()
    fps = 0
    frame_count = 0
    frame_interval = 1.0 / max_fps 

    while True:
        start_loop = time.time()
        ret, frame = cam.read()
        if not ret or frame is None:
            continue

        frame_count += 1
        current_time = time.time()
        if current_time - prev_time >= 1.0:
            fps = frame_count / (current_time - prev_time)
            prev_time = current_time
            frame_count = 0

        cv2.putText(frame, f'FPS: {fps:.2f}', (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

        cv2.imshow("Camera", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

        elapsed = time.time() - start_loop
        sleep_time = frame_interval - elapsed
        if sleep_time > 0:
            time.sleep(sleep_time)

    cam.stop()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
