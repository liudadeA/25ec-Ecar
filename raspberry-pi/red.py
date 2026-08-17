import cv2
import numpy as np
import os
import time
import json
from picamera2 import Picamera2
from collections import deque

# ====================== 初始化部分 ======================
# 逻辑部分：禁用 PyKMS 预览（优化树莓派性能）
os.environ["PYKMS_NO_PREVIEW"] = "1"

# 逻辑部分：初始化树莓派相机
picam2 = Picamera2()
# 配置相机参数：640x640分辨率
picam2.configure(picam2.create_video_configuration(main={"size": (640, 640)}))
picam2.start()  # 启动相机

# 逻辑部分：定义分辨率常量
MAX_WIDTH = 640
MAX_HEIGHT = 640

# 绘图部分：创建显示窗口
WINDOW_NAME = 'Red Line Tracking'
cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)
# 设置窗口大小（增加高度用于显示控制面板）
cv2.resizeWindow(WINDOW_NAME, MAX_WIDTH, MAX_HEIGHT + 200)

# 逻辑部分：定义默认参数
DEFAULT_PARAMS = {
    'h_low1': 0,      # 红色范围1的低H值
    'h_high1': 10,    # 红色范围1的高H值
    'h_low2': 170,    # 红色范围2的低H值
    'h_high2': 180,   # 红色范围2的高H值
    's_low': 120,     # 饱和度低阈值
    's_high': 255,    # 饱和度高阈值
    'v_low': 120,     # 明度低阈值
    'v_high': 255,    # 明度高阈值
    'min_area': 500,  # 最小区域面积
    'max_area': MAX_WIDTH * MAX_HEIGHT,  # 最大区域面积
    'hyster_high': 40,  # 滞回高阈值 (0-100)
    'hyster_low': 30    # 滞回低阈值 (0-100)
}

# 逻辑部分：配置文件路径
CONFIG_FILE = "red_detection_config.json"
params = DEFAULT_PARAMS.copy()  # 使用默认参数初始化

# 逻辑部分：尝试从配置文件加载参数
if os.path.exists(CONFIG_FILE):
    try:
        with open(CONFIG_FILE, 'r') as f:
            loaded = json.load(f)
            # 只加载存在的参数，防止版本不兼容
            for k in DEFAULT_PARAMS:
                if k in loaded:
                    params[k] = loaded[k]
        print("Parameters loaded from config file")
    except Exception as e:
        print(f"Config load failed: {e}")
else:
    print("No config file found, using default parameters")

# 逻辑部分：空回调函数（用于滑块）
def nothing(x): pass

# 绘图部分：创建HSV参数滑块
cv2.createTrackbar('H1 Low', WINDOW_NAME, params['h_low1'], 180, nothing)
cv2.createTrackbar('H1 High', WINDOW_NAME, params['h_high1'], 180, nothing)
cv2.createTrackbar('H2 Low', WINDOW_NAME, params['h_low2'], 180, nothing)
cv2.createTrackbar('H2 High', WINDOW_NAME, params['h_high2'], 180, nothing)

# 绘图部分：创建区域面积滑块
cv2.createTrackbar('Min Area', WINDOW_NAME, params['min_area'], MAX_WIDTH*MAX_HEIGHT, nothing)
cv2.createTrackbar('Max Area', WINDOW_NAME, params['max_area'], MAX_WIDTH*MAX_HEIGHT, nothing)

# 绘图部分：创建滞回阈值滑块
cv2.createTrackbar('Hyster High', WINDOW_NAME, params['hyster_high'], 100, nothing)
cv2.createTrackbar('Hyster Low', WINDOW_NAME, params['hyster_low'], 100, nothing)

# 逻辑部分：帧率统计
frame_count = 0
start_time = time.time()
fps = 0.0

# ====================== 循迹参数 ======================
# 逻辑部分：传感器配置
NUM_SENSORS = 8  # 模拟8路传感器
SENSOR_HEIGHT = 40  # 传感器区域高度
SENSOR_WIDTH = MAX_WIDTH // NUM_SENSORS  # 每个传感器的宽度
BOTTOM_SENSOR_Y = MAX_HEIGHT - 80  # 底部传感器区域Y坐标（靠近车头）
TOP_SENSOR_Y = 80  # 顶部传感器区域Y坐标（远离车头）

# 逻辑部分：传感器权重（用于计算偏差方向）
# 权重设计：左侧为负值，右侧为正值，中间为零点
SENSOR_WEIGHTS = [-4, -3, -2, -1, 1, 2, 3, 4]

# 逻辑部分：传感器状态记忆
last_valid_bottom_sensors = [0] * NUM_SENSORS  # 上一次有效的底部传感器状态
last_valid_top_sensors = [0] * NUM_SENSORS     # 上一次有效的顶部传感器状态
use_top_sensor = False  # 顶部传感器开关（默认关闭）
last_detection_time = time.time()  # 上一次检测到红线的时间
valid_detection_timeout = 0.5      # 有效状态超时时间(秒)

# ====================== 功能函数 ======================
def save_params():
    """逻辑部分：保存当前参数到配置文件"""
    try:
        with open(CONFIG_FILE, 'w') as f:
            json.dump(params, f, indent=4)
        print("Parameters saved successfully")
    except Exception as e:
        print(f"Save failed: {e}")

def get_sensor_values(mask, y_pos, last_values):
    """
    逻辑部分：获取传感器值（0或1），使用滞回比较
    mask: 二值化图像
    y_pos: 传感器区域的Y坐标
    last_values: 上一次的传感器值（用于滞回比较）
    返回值: 当前传感器值列表
    """
    sensor_values = []
    # 获取当前滞回阈值（转换为0-1范围）
    hyster_high = params['hyster_high'] / 100.0
    hyster_low = params['hyster_low'] / 100.0
    
    for i in range(NUM_SENSORS):
        # 计算当前传感器的ROI区域
        x_start = i * SENSOR_WIDTH
        x_end = (i + 1) * SENSOR_WIDTH
        roi = mask[y_pos:y_pos+SENSOR_HEIGHT, x_start:x_end]
        
        # 计算ROI中红色像素的比例
        red_pixels = cv2.countNonZero(roi)  # 非零像素计数（白色代表红色）
        total_pixels = roi.size
        ratio = red_pixels / total_pixels if total_pixels > 0 else 0
        
        # 滞回比较逻辑（防止传感器值抖动）
        if ratio > hyster_high:  # 高于高阈值，判定为有红线
            sensor_value = 1
        elif ratio < hyster_low:  # 低于低阈值，判定为无红线
            sensor_value = 0
        else:  # 在滞回区间内，保持上一次的值
            sensor_value = last_values[i]
        
        sensor_values.append(sensor_value)
    
    return sensor_values

def calculate_direction(sensors):
    """
    逻辑部分：根据传感器值计算方向和偏差值
    sensors: 当前传感器的状态列表（0或1）
    返回值: 偏差值（负数表示左偏，正数表示右偏）
    """
    # 找到最左侧和最右侧激活的传感器
    left_index = None  # 最左侧激活的传感器索引
    right_index = None  # 最右侧激活的传感器索引
    
    # 从左向右扫描找到最左侧激活的传感器
    for i in range(NUM_SENSORS):
        if sensors[i] == 1:
            left_index = i
            break
    
    # 从右向左扫描找到最右侧激活的传感器
    for i in range(NUM_SENSORS-1, -1, -1):
        if sensors[i] == 1:
            right_index = i
            break
    
    # 如果没有检测到红线，返回0（停止）
    if left_index is None or right_index is None:
        return 0
    
    # 计算左右权重和
    left_sum = 0
    right_sum = 0
    
    # 计算最左侧激活传感器的权重和（左侧所有激活传感器的权重）
    for i in range(left_index+1):
        if sensors[i] == 1:
            left_sum += SENSOR_WEIGHTS[i]
    
    # 计算最右侧激活传感器的权重和（右侧所有激活传感器的权重）
    for i in range(right_index, NUM_SENSORS):
        if sensors[i] == 1:
            right_sum += SENSOR_WEIGHTS[i]
    
    # 根据左右权重和确定偏差值
    if left_sum + right_sum < 0:  # 左侧权重更大
        bias = SENSOR_WEIGHTS[left_index]  # 使用最左侧激活传感器的权重
    elif left_sum + right_sum == 0:  # 左右平衡
        bias = 0  # 直行
    else:  # 右侧权重更大
        bias = SENSOR_WEIGHTS[right_index]  # 使用最右侧激活传感器的权重
    
    return bias

def determine_direction(bottom_sensors, top_sensors):
    """
    逻辑部分：确定行驶方向，优先使用底部传感器，支持状态保持
    bottom_sensors: 底部传感器状态
    top_sensors: 顶部传感器状态
    返回值: 偏差值
    """
    global last_valid_bottom_sensors, last_valid_top_sensors, last_detection_time
    
    # 从滑块获取滞回阈值（实时更新）
    params['hyster_high'] = cv2.getTrackbarPos('Hyster High', WINDOW_NAME)
    params['hyster_low'] = cv2.getTrackbarPos('Hyster Low', WINDOW_NAME)
    
    # 确保滞回阈值合理（低阈值 < 高阈值）
    if params['hyster_low'] > params['hyster_high']:
        params['hyster_low'], params['hyster_high'] = params['hyster_high'], params['hyster_low']
        cv2.setTrackbarPos('Hyster Low', WINDOW_NAME, params['hyster_low'])
        cv2.setTrackbarPos('Hyster High', WINDOW_NAME, params['hyster_high'])
    
    # 检测底部传感器是否有红线
    bottom_has_line = any(bottom_sensors)
    
    # 更新有效状态（传感器记忆逻辑）
    if bottom_has_line:  # 底部有红线
        last_valid_bottom_sensors = bottom_sensors.copy()
        last_detection_time = time.time()
        sensors = bottom_sensors
    # 如果底部没有红线，但启用了顶部传感器且顶部有红线
    elif use_top_sensor and any(top_sensors):
        last_valid_top_sensors = top_sensors.copy()
        last_detection_time = time.time()
        sensors = top_sensors
    # 如果都没有红线，但在超时时间内，使用上一次有效状态
    elif time.time() - last_detection_time < valid_detection_timeout:
        sensors = last_valid_bottom_sensors
    # 完全丢失红线
    else:
        return 0  # 停止
    
    # 计算偏差值
    bias = calculate_direction(sensors)
    
    return bias

# ====================== 主循环 ======================
try:
    while True:
        # 逻辑部分：从相机捕获帧
        frame = picam2.capture_array()
        # 将RGB格式转换为BGR格式（OpenCV标准）
        frame_bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

        # 逻辑部分：从滑块更新参数
        for key, track_name in [
            ('h_low1', 'H1 Low'), ('h_high1', 'H1 High'),
            ('h_low2', 'H2 Low'), ('h_high2', 'H2 High'),
            ('min_area', 'Min Area'), ('max_area', 'Max Area')]:
            params[key] = cv2.getTrackbarPos(track_name, WINDOW_NAME)

        # 逻辑部分：面积边界修正（确保最小面积 < 最大面积）
        if params['min_area'] > params['max_area']:
            params['min_area'], params['max_area'] = params['max_area'], params['min_area']
            cv2.setTrackbarPos('Min Area', WINDOW_NAME, params['min_area'])
            cv2.setTrackbarPos('Max Area', WINDOW_NAME, params['max_area'])

        # 逻辑部分：计算帧率
        frame_count += 1
        elapsed = time.time() - start_time
        if elapsed >= 1.0:  # 每秒更新一次FPS
            fps = frame_count / elapsed
            frame_count = 0
            start_time = time.time()
        
        # ====================== 图像处理 ======================
        # 逻辑部分：转换为HSV颜色空间（更适合颜色检测）
        hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)
        # 应用高斯模糊减少噪声
        blurred = cv2.GaussianBlur(hsv, (5, 5), 0)

        # 逻辑部分：构建双范围红色掩码（覆盖0-10和170-180的H值）
        lower1 = np.array([params['h_low1'], params['s_low'], params['v_low']])
        upper1 = np.array([params['h_high1'], params['s_high'], params['v_high']])
        lower2 = np.array([params['h_low2'], params['s_low'], params['v_low']])
        upper2 = np.array([params['h_high2'], params['s_high'], params['v_high']])
        # 合并两个红色范围的掩码
        mask = cv2.inRange(blurred, lower1, upper1) | cv2.inRange(blurred, lower2, upper2)

        # 逻辑部分：自适应形态学处理
        # 根据图像尺寸动态计算核大小
        ksize = max(3, int(min(frame_bgr.shape[:2]) / 100) * 2 + 1)
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (ksize, ksize))
        # 开运算：去除小噪点
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        # 闭运算：填充小孔洞
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        # 逻辑部分：获取传感器值（使用滞回比较）
        bottom_sensors = get_sensor_values(mask, BOTTOM_SENSOR_Y, last_valid_bottom_sensors)
        top_sensors = get_sensor_values(mask, TOP_SENSOR_Y, last_valid_top_sensors)
        
        # 逻辑部分：确定偏差值
        bias = determine_direction(bottom_sensors, top_sensors)
        
        # ====================== 绘图部分 ======================
        # 在图像上显示底部传感器区域
        for i, value in enumerate(bottom_sensors):
            x_start = i * SENSOR_WIDTH
            x_end = (i + 1) * SENSOR_WIDTH
            # 绿色表示检测到红线，红色表示未检测到
            color = (0, 255, 0) if value == 1 else (0, 0, 255)
            cv2.rectangle(frame_bgr, (x_start, BOTTOM_SENSOR_Y), 
                          (x_end, BOTTOM_SENSOR_Y + SENSOR_HEIGHT), color, 2)
        
        # 只有当顶部传感器启用时才显示顶部传感器区域
        if use_top_sensor:
            for i, value in enumerate(top_sensors):
                x_start = i * SENSOR_WIDTH
                x_end = (i + 1) * SENSOR_WIDTH
                color = (0, 255, 0) if value == 1 else (0, 0, 255)
                cv2.rectangle(frame_bgr, (x_start, TOP_SENSOR_Y), 
                              (x_end, TOP_SENSOR_Y + SENSOR_HEIGHT), color, 2)
        
        # 显示帧率
        cv2.putText(frame_bgr, f"FPS: {fps:.1f}", (10, 30), 
                   cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2)
        # 显示偏差值
        cv2.putText(frame_bgr, f"Bias: {bias}", (10, 70), 
                   cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2)
        
        # 显示当前模式（顶部传感器是否启用）
        top_status = "ON" if use_top_sensor else "OFF"
        cv2.putText(frame_bgr, f"Top Sensor: {top_status}", (10, 110), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        
        # 显示控制提示
        cv2.putText(frame_bgr, "Controls: [Q]uit [S]ave [R]eset [T]oggle Top Sensor", 
                   (10, MAX_HEIGHT + 150), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200, 200, 200), 1)
        
        # 显示图像
        cv2.imshow(WINDOW_NAME, frame_bgr)
        
        # ====================== 键盘控制 ======================
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):  # 退出程序
            save_params()
            break
        elif key == ord('s'):  # 保存参数
            save_params()
        elif key == ord('r'):  # 重置参数
            params = DEFAULT_PARAMS.copy()
            # 更新所有滑块位置
            cv2.setTrackbarPos('H1 Low', WINDOW_NAME, params['h_low1'])
            cv2.setTrackbarPos('H1 High', WINDOW_NAME, params['h_high1'])
            cv2.setTrackbarPos('H2 Low', WINDOW_NAME, params['h_low2'])
            cv2.setTrackbarPos('H2 High', WINDOW_NAME, params['h_high2'])
            cv2.setTrackbarPos('Min Area', WINDOW_NAME, params['min_area'])
            cv2.setTrackbarPos('Max Area', WINDOW_NAME, params['max_area'])
            cv2.setTrackbarPos('Hyster High', WINDOW_NAME, params['hyster_high'])
            cv2.setTrackbarPos('Hyster Low', WINDOW_NAME, params['hyster_low'])
            print("Parameters reset to defaults")
        elif key == ord('t'):  # 切换顶部传感器
            use_top_sensor = not use_top_sensor
            print(f"Top sensor {'enabled' if use_top_sensor else 'disabled'}")

finally:
    # 清理资源
    save_params()
    picam2.stop()  # 停止相机
    cv2.destroyAllWindows()  # 关闭所有窗口
    print("Program exited cleanly")
