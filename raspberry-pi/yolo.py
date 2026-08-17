import cv2
import time
import numpy as np
from ultralytics import YOLO
from picamera2 import Picamera2
from picamera2.encoders import H264Encoder
from picamera2.outputs import FileOutput

# 初始化模型和相机
model = YOLO('710.pt')  # 确保使用最优模型
picam2 = Picamera2()

# 优化相机配置
config = picam2.create_video_configuration(
    main={"size": (640, 640), "format": "BGR888"},
    controls={"FrameRate": 30}
)
picam2.configure(config)
picam2.start()

# 创建OpenCV窗口
window_name = "YOLOv8"
cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
cv2.resizeWindow(window_name, 640, 660)

# 创建二值化阈值滑块
cv2.createTrackbar('2', window_name, 120, 255, lambda x: None)

# 性能优化变量
frame_count = 0
start_time = time.monotonic()
last_fps_time = start_time
fps = 0.0
use_binary = True
threshold_value = 120

# 预分配内存
gray = np.zeros((640, 640), dtype=np.uint8)
binary = np.zeros((640, 640), dtype=np.uint8)
binary_rgb = np.zeros((640, 640, 3), dtype=np.uint8)

try:
    while True:
        # 捕获图像 (直接获取BGR格式)
        frame = picam2.capture_array()
        
        # 性能监控
        frame_count += 1
        current_time = time.monotonic()
        
        # 每秒更新FPS
        if current_time - last_fps_time >= 1.0:
            fps = frame_count / (current_time - last_fps_time)
            frame_count = 0
            last_fps_time = current_time
        
        # 二值化处理
        processed_frame = frame
        if use_binary:
            threshold_value = cv2.getTrackbarPos('2', window_name)
            
            # 优化: 使用灰度转换和阈值处理的快速版本
            cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY, dst=gray)
            cv2.threshold(gray, threshold_value, 255, cv2.THRESH_BINARY, dst=binary)
            
            # 复用内存创建三通道图像
            binary_rgb[:, :, 0] = binary
            binary_rgb[:, :, 1] = binary
            binary_rgb[:, :, 2] = binary
            processed_frame = binary_rgb
        
        # 模型推理 - 使用较低分辨率提升速度
        results = model(processed_frame, 
                       imgsz=320,  # 降低输入分辨率
                       conf=0.8, 
                       verbose=False,
                       device='cpu',  # 明确指定设备
                       half=False)   # 禁用半精度(树莓派上可能更慢)
        
        # 绘制结果
        annotated_frame = results[0].plot()
        
        # 显示性能信息
        mode_text = "2" if use_binary else "1"
        cv2.putText(annotated_frame, f"FPS: {fps:.1f}", (10, 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv2.putText(annotated_frame, f"{mode_text}", (10, 60),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv2.putText(annotated_frame, f"{threshold_value}", (10, 90),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        
        # 显示结果
        cv2.imshow(window_name, annotated_frame)
        
        # 按键处理
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('b'):
            use_binary = not use_binary
            print(f"切换到{'2' if use_binary else '1'}模式")

finally:
    picam2.stop()
    cv2.destroyAllWindows()
