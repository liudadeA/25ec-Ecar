import cv2
import time
import numpy as np
from ultralytics import YOLO
from picamera2 import Picamera2

# ====================== 初始化部分 ======================
# 逻辑部分：初始化模型和相机
model = YOLO('710.pt')  # 加载YOLO数字识别模型
picam2 = Picamera2()    # 创建树莓派相机对象

# 逻辑部分：相机配置优化
config = picam2.create_video_configuration(
    main={"size": (640, 640), "format": "BGR888"},  # 设置分辨率640x640，BGR格式
    controls={"FrameRate": 30}  # 帧率设置为30fps
)
picam2.configure(config)  # 应用配置
picam2.start()            # 启动相机

# 绘图部分：创建显示窗口
window_name = "Digit Navigation (Binary Mode)"
cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)  # 创建可调整大小的窗口
cv2.resizeWindow(window_name, 640, 660)          # 设置窗口尺寸

# 绘图部分：创建阈值滑动条
cv2.createTrackbar('Threshold', window_name, 120, 255, lambda x: None)  # 阈值范围0-255，初始值120

# ====================== 状态变量 ======================
# 逻辑部分：系统状态管理
target_digit = None       # 当前目标数字
frame_count = 0           # 帧计数器
start_time = time.monotonic()  # 起始时间
last_fps_time = start_time    # 上次计算FPS的时间
fps = 0.0                 # 当前帧率
setup_mode = True         # 是否处于目标设置模式

# 逻辑部分：延迟确认机制
CONFIRMATION_FRAMES = 2   # 连续检测到相同数字的帧数阈值
digit_counters = {str(i): 0 for i in range(1, 9)}  # 数字1-8的计数器
last_detected_digit = None  # 上次检测到的数字

# 逻辑部分：检测区域设置
FRAME_WIDTH = 640         # 帧宽度
FRAME_HEIGHT = 640        # 帧高度
CENTER_X = FRAME_WIDTH // 2  # 屏幕中心X坐标
CENTER_Y = FRAME_HEIGHT // 2 # 屏幕中心Y坐标

# 绘图部分：颜色定义（BGR格式）
COLOR_RED = (0, 0, 255)       # 红色
COLOR_GREEN = (0, 255, 0)     # 绿色
COLOR_BLUE = (255, 0, 0)      # 蓝色
COLOR_YELLOW = (0, 255, 255)  # 黄色
COLOR_ORANGE = (0, 165, 255)  # 橙色
COLOR_PURPLE = (128, 0, 128)  # 紫色
COLOR_CYAN = (255, 255, 0)    # 青色
COLOR_WHITE = (255, 255, 255) # 白色

# 逻辑部分：动作稳定性控制
action_counter = 0       # 相同动作的连续帧数
last_action = "Forward"  # 上一个动作
current_action = "Forward"  # 当前稳定动作

# 逻辑部分：内存预分配（优化性能）
gray_frame = np.zeros((640, 640), dtype=np.uint8)    # 灰度图像
binary_frame = np.zeros((640, 640), dtype=np.uint8)  # 二值图像
binary_rgb = np.zeros((640, 640, 3), dtype=np.uint8) # 三通道二值图像

print("Starting... Waiting for target digit (1-8)")

# ====================== 主循环 ======================
try:
    while True:
        # 逻辑部分：图像采集
        original_frame = picam2.capture_array()  # 从相机获取BGR图像
        
        # 逻辑部分：获取阈值
        threshold_value = cv2.getTrackbarPos('Threshold', window_name)
        
        # 逻辑部分：图像预处理
        cv2.cvtColor(original_frame, cv2.COLOR_BGR2GRAY, dst=gray_frame)  # 转灰度
        _, binary_frame = cv2.threshold(gray_frame, threshold_value, 255, cv2.THRESH_BINARY)  # 二值化
        
        # 绘图部分：创建三通道二值图像（用于显示）
        display_frame = np.zeros((640, 640, 3), dtype=np.uint8)
        display_frame[:, :, 0] = binary_frame  # 蓝色通道
        display_frame[:, :, 1] = binary_frame  # 绿色通道
        display_frame[:, :, 2] = binary_frame  # 红色通道
        
        # 逻辑部分：准备推理帧
        inference_frame = display_frame.copy()
        
        # 逻辑部分：性能监控（FPS计算）
        frame_count += 1
        current_time = time.monotonic()
        if current_time - last_fps_time >= 1.0:  # 每秒更新FPS
            fps = frame_count / (current_time - last_fps_time)
            frame_count = 0
            last_fps_time = current_time
        
        # ====================== 模型推理 ======================
        # 逻辑部分：YOLO模型推理
        results = model(inference_frame, 
                       imgsz=320,    # 降低输入分辨率提高速度
                       conf=0.8,     # 高置信度阈值
                       verbose=False,
                       device='cpu',
                       half=False)    # 禁用半精度

        # 绘图部分：使用YOLO内置方法绘制检测结果
        annotated_frame = results[0].plot(img=inference_frame.copy())
        
        # ====================== 导航逻辑 ======================
        # 绘图部分：绘制检测区域
        cv2.rectangle(annotated_frame, (0, 0), (CENTER_X, FRAME_HEIGHT), COLOR_BLUE, 2)  # 左区域
        cv2.rectangle(annotated_frame, (CENTER_X, 0), (FRAME_WIDTH, FRAME_HEIGHT), COLOR_BLUE, 2)  # 右区域
        
        # 逻辑部分：初始化检测变量
        detected_digits = []  # 检测到的数字列表
        action = "Forward"    # 当前动作
        target_detected = False  # 是否检测到目标
        current_digit = None     # 当前帧检测到的数字
        
        # 逻辑部分：处理检测结果
        max_conf = -1  # 最高置信度
        max_box = None  # 最高置信度的检测框
        max_digit = None  # 最高置信度的数字
        max_center_x = None  # 检测框中心X坐标
        
        if results[0].boxes is not None:
            # 遍历所有检测框
            for box in results[0].boxes:
                x1, y1, x2, y2 = map(int, box.xyxy[0])  # 框坐标
                conf = float(box.conf[0])                # 置信度
                cls = int(box.cls[0])                    # 类别索引
                digit = str(model.names[cls])                     # 数字标签(1-8)
                
                # 绘图部分：设置模式下绘制所有高置信度框
                if setup_mode and conf >= 0.7:
                    cv2.rectangle(annotated_frame, (x1, y1), (x2, y2), COLOR_GREEN, 2)
                    cv2.putText(annotated_frame, f"{digit}:{conf:.2f}", (x1, y1-10), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.7, COLOR_GREEN, 2)
                
                # 逻辑部分：记录最高置信度的检测
                if conf > max_conf:
                    max_conf = conf
                    max_box = box
                    max_digit = digit
                    max_center_x = (x1 + x2) // 2  # 框中心X坐标
        
        # 逻辑部分：处理最高置信度检测
        if max_digit:
            detected_digits.append(max_digit)
            current_digit = max_digit
            
            # 导航模式下判断目标数字
            if not setup_mode and max_digit == target_digit:
                target_detected = True
                
                # 逻辑部分：根据目标位置决定动作
                if max_center_x < CENTER_X:
                    action = "-1"
                elif max_center_x > CENTER_X:
                    action = "1"
                else:
                    action = "0"
        
        # 逻辑部分：目标确认机制（设置模式）
        if setup_mode:
            if current_digit and current_digit == last_detected_digit:
                digit_counters[current_digit] += 1  # 增加计数器
                # 达到确认阈值，设置目标数字
                if digit_counters[current_digit] >= CONFIRMATION_FRAMES:
                    target_digit = current_digit
                    setup_mode = False
                    print(f"Target digit confirmed: {target_digit}")
            else:
                # 数字变化时重置计数器
                if last_detected_digit:
                    digit_counters[last_detected_digit] = 0
            last_detected_digit = current_digit
        
        # 逻辑部分：动作稳定性处理
        if action == last_action:
            action_counter = min(action_counter + 1, 10)  # 增加稳定性计数
        else:
            action_counter = max(action_counter - 2, 0)    # 减少稳定性计数

        # 当连续2帧动作相同时更新稳定动作
        if action_counter >= 2:
            current_action = action
            last_action = action
        
        # ====================== 状态显示 ======================
        # 绘图部分：显示系统状态信息
        if setup_mode:
            # 设置模式状态
            status_text = "Status: Waiting for target digit (1-8)"
            cv2.putText(annotated_frame, status_text, (10, 30), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, COLOR_PURPLE, 2)
            
            # 显示数字计数器
            counter_y = 60
            for digit, count in digit_counters.items():
                if count > 0:
                    counter_text = f"Digit {digit}: {count}/{CONFIRMATION_FRAMES}"
                    cv2.putText(annotated_frame, counter_text, (10, counter_y), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.6, COLOR_CYAN, 2)
                    counter_y += 30
        else:
            # 导航模式状态
            status_text = f"Target: {target_digit}"
            cv2.putText(annotated_frame, status_text, (10, 30), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, COLOR_GREEN, 2)
            
            action_text = f"Action: {current_action}"
            cv2.putText(annotated_frame, action_text, (10, 60), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, COLOR_GREEN, 2)
        
        # 绘图部分：显示FPS
        cv2.putText(annotated_frame, f"FPS: {fps:.1f}", (10, 90), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, COLOR_GREEN, 2)
        
        # 绘图部分：显示检测到的数字
        digits_text = f"Detected: {', '.join(detected_digits)}" if detected_digits else "No digits detected"
        cv2.putText(annotated_frame, digits_text, (10, 120), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, COLOR_GREEN, 2)
        
        # 绘图部分：显示当前阈值
        cv2.putText(annotated_frame, f"Threshold: {threshold_value}", (10, 150), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, COLOR_GREEN, 2)
        
        # 绘图部分：显示模式标识
        cv2.putText(annotated_frame, "BINARY MODE", (FRAME_WIDTH - 150, 30), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, COLOR_RED, 2)
        
        # 绘图部分：设置模式说明
        if setup_mode:
            cv2.putText(annotated_frame, "Place target digit in view", 
                        (CENTER_X - 180, CENTER_Y - 20), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.9, COLOR_PURPLE, 2)
            cv2.putText(annotated_frame, "System will auto-set target", 
                        (CENTER_X - 180, CENTER_Y + 20), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.9, COLOR_PURPLE, 2)
            cv2.putText(annotated_frame, f"Requires {CONFIRMATION_FRAMES} consecutive detections", 
                        (CENTER_X - 180, CENTER_Y + 60), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, COLOR_CYAN, 2)
        
        # 绘图部分：显示最终图像
        cv2.imshow(window_name, annotated_frame)
        
        # ====================== 键盘控制 ======================
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):  # 退出程序
            break
        elif key == ord('r'):  # 重置目标
            # 逻辑部分：重置所有状态
            target_digit = None
            setup_mode = True
            action = "Forward"
            last_action = "Forward"
            current_action = "Forward"
            digit_counters = {str(i): 0 for i in range(1, 9)}
            last_detected_digit = None
            print("Target digit reset, waiting for new target")

finally:
    # 清理资源
    picam2.stop()  # 停止相机
    cv2.destroyAllWindows()  # 关闭所有窗口
    print("Program exited")
