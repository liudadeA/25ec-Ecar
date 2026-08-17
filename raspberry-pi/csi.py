import cv2
import zmq
import base64
import numpy as np
import os
import time

# 禁用PyKMS预览（避免摄像头预览冲突）
os.environ["PYKMS_NO_PREVIEW"] = "1"
from picamera2 import Picamera2

# 配置参数
IP = '192.168.68.53'  # 目标IP
PORT = 5500           # 目标端口
WINDOW_NAME = "Local Camera View"  # 本地显示窗口名称

# 初始化ZMQ
context = zmq.Context()
footage_socket = context.socket(zmq.PUSH)
footage_socket.connect(f'tcp://{IP}:{PORT}')

# 初始化摄像头
picam2 = Picamera2()
# 配置摄像头（640x480分辨率）
picam2.configure(picam2.create_video_configuration(main={"size": (1280, 720)}))
picam2.start()

# 初始化OpenCV窗口（用于本地显示）
cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)

# 帧率计算变量
frame_count = 0
start_time = time.time()
fps = 0.0

try:
    while True:
        # 捕获摄像头帧（RGB格式）
        frame = picam2.capture_array()
        # 转换为OpenCV支持的BGR格式（用于显示和编码）
        frame_bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        
        # 计算帧率（每1秒更新一次）
        frame_count += 1
        elapsed = time.time() - start_time
        if elapsed >= 1.0:
            fps = frame_count / elapsed
            frame_count = 0
            start_time = time.time()
        
        # 编码为JPG并base64处理（用于网络传输）
        ret, buffer = cv2.imencode('.jpg', frame_bgr, [cv2.IMWRITE_JPEG_QUALITY, 80])  # 降低质量减少传输量
        jpg_as_text = base64.b64encode(buffer).decode('utf-8')
        
        # 发送数据（格式：帧率|base64编码的图像）
        msg = f"{fps:.2f}|{jpg_as_text}"
        footage_socket.send_string(msg)
        
        # 本地显示（在画面上叠加帧率信息）
        cv2.putText(
            frame_bgr, 
            f"FPS: {fps:.2f}", 
            (10, 30), 
            cv2.FONT_HERSHEY_SIMPLEX, 
            1, 
            (0, 255, 0), 
            2
        )
        cv2.imshow(WINDOW_NAME, frame_bgr)
        
        # 按'q'键退出循环（等待1ms处理窗口事件）
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

finally:
    # 释放资源
    picam2.stop()  # 停止摄像头
    cv2.destroyAllWindows()  # 关闭所有OpenCV窗口
    footage_socket.close()  # 关闭ZMQ连接
    context.term()  # 终止ZMQ上下文
