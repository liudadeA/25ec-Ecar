import cv2
import numpy as np
import os
import time
from picamera2 import Picamera2
import serial
import fcntl

# ====================== 初始化部分 ======================
# 禁用 PyKMS 预览（优化树莓派性能）
os.environ["PYKMS_NO_PREVIEW"] = "1"

# 初始化树莓派相机
picam2 = Picamera2()
picam2.configure(picam2.create_video_configuration(main={"size": (640, 640)}))
picam2.start()

# 定义分辨率常量
MAX_WIDTH = 640
MAX_HEIGHT = 640

# 创建显示窗口
WINDOW_NAME = 'Red Line Sensors'
cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)
cv2.resizeWindow(WINDOW_NAME, MAX_WIDTH, MAX_HEIGHT)

# 定义固定参数
PARAMS = {
    'h_low1': 0,      # 红色范围1的低H值
    'h_high1': 10,    # 红色范围1的高H值
    'h_low2': 170,    # 红色范围2的低H值
    'h_high2': 180,   # 红色范围2的高H值
    's_low': 120,     # 饱和度低阈值
    's_high': 255,    # 饱和度高阈值
    'v_low': 120,     # 明度低阈值
    'v_high': 255,    # 明度高阈值
    'hyster_high': 40,  # 滞回高阈值 (0-100)
    'hyster_low': 30    # 滞回低阈值 (0-100)
}

# ====================== 串口配置 ======================
def send_data(frames, port='/dev/serial0', baudrate=115200, timeout=0.1):
    """通过串口发送带帧头帧尾的数据，非阻塞模式"""
    HEADER = b'\xaa'  # 包头
    FOOTER = b'\x55\xAA\x33'  # 包尾
    
    try:
        with serial.Serial(port, baudrate, timeout=timeout) as ser:
            # 非阻塞获取文件锁
            fcntl.flock(ser.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
            
            for data in frames:
                if isinstance(data, str):
                    data = data.encode('utf-8')
                
                frame = HEADER + data + FOOTER
                print(f"发送帧: {frame.hex()}")
                ser.write(frame)
                ser.flush()  # 确保数据发送完成
            
            return True      
    except BlockingIOError:
        print("串口被占用，等待下次发送...")
        return False
    except Exception as e:
        print(f"发送异常: {str(e)}")
        return False

# ====================== 传感器配置 ======================
NUM_SENSORS = 8  # 8路传感器
SENSOR_HEIGHT = 40  # 传感器区域高度
SENSOR_WIDTH = MAX_WIDTH // NUM_SENSORS  # 每个传感器的宽度
BOTTOM_SENSOR_Y = MAX_HEIGHT - 80  # 底部传感器区域Y坐标
TOP_SENSOR_Y = 80  # 顶部传感器区域Y坐标

# 传感器状态记忆
last_valid_bottom_sensors = [0] * NUM_SENSORS
last_valid_top_sensors = [0] * NUM_SENSORS

# ====================== 功能函数 ======================
def get_sensor_values(mask, y_pos, last_values):
    """获取传感器值（0或1），使用滞回比较"""
    sensor_values = []
    hyster_high = PARAMS['hyster_high'] / 100.0
    hyster_low = PARAMS['hyster_low'] / 100.0
    
    for i in range(NUM_SENSORS):
        x_start = i * SENSOR_WIDTH
        x_end = (i + 1) * SENSOR_WIDTH
        roi = mask[y_pos:y_pos+SENSOR_HEIGHT, x_start:x_end]
        
        red_pixels = cv2.countNonZero(roi)
        total_pixels = roi.size
        ratio = red_pixels / total_pixels if total_pixels > 0 else 0
        
        # 滞回比较逻辑
        if ratio > hyster_high:
            sensor_value = 1
        elif ratio < hyster_low:
            sensor_value = 0
        else:
            sensor_value = last_values[i]
        
        sensor_values.append(sensor_value)
    
    return sensor_values

def sensor_list_to_byte(sensor_list):
    """将传感器状态列表转换为一个字节"""
    state_byte = 0
    for i in range(8):
        if sensor_list[i] == 1:
            state_byte |= (1 << (7 - i))  # sensors[0]对应最高位
    return state_byte

# ====================== 主循环 ======================
try:
    # 初始化发送计时器
    last_send_time = time.time()
    send_interval = 1  # 100ms发送一次
    
    while True:
        # 捕获并处理图像
        frame = picam2.capture_array()
        frame_bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

        # 图像处理
        hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)
        blurred = cv2.GaussianBlur(hsv, (5, 5), 0)

        # 双范围红色掩码
        lower1 = np.array([PARAMS['h_low1'], PARAMS['s_low'], PARAMS['v_low']])
        upper1 = np.array([PARAMS['h_high1'], PARAMS['s_high'], PARAMS['v_high']])
        lower2 = np.array([PARAMS['h_low2'], PARAMS['s_low'], PARAMS['v_low']])
        upper2 = np.array([PARAMS['h_high2'], PARAMS['s_high'], PARAMS['v_high']])
        mask = cv2.inRange(blurred, lower1, upper1) | cv2.inRange(blurred, lower2, upper2)

        # 形态学处理
        ksize = max(3, int(min(frame_bgr.shape[:2]) / 100) * 2 + 1)
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (ksize, ksize))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        # 获取传感器值
        bottom_sensors = get_sensor_values(mask, BOTTOM_SENSOR_Y, last_valid_bottom_sensors)
        top_sensors = get_sensor_values(mask, TOP_SENSOR_Y, last_valid_top_sensors)
        
        # 更新传感器状态记忆
        last_valid_bottom_sensors = bottom_sensors.copy()
        last_valid_top_sensors = top_sensors.copy()
        
        # 转换为字节数据
        bottom_byte = sensor_list_to_byte(bottom_sensors)
        top_byte = sensor_list_to_byte(top_sensors)
        data_to_send = [bytes([bottom_byte])]

        # 定时发送（100ms一次）
        current_time = time.time()
        if current_time - last_send_time >= send_interval:
            success = send_data(data_to_send)
            if not success:
                print("发送数据失败，将在下次重试")
            
            # 打印传感器状态（仅在发送时更新）
            bottom_bin = bin(bottom_byte)[2:].zfill(8)
            top_bin = bin(top_byte)[2:].zfill(8)
            print(f"Bottom Sensors: {bottom_bin} | Top Sensors: {top_bin}")
            
            # 更新上次发送时间
            last_send_time = current_time
        
        # 绘图显示
        # 底部传感器区域
        for i, value in enumerate(bottom_sensors):
            x_start = i * SENSOR_WIDTH
            x_end = (i + 1) * SENSOR_WIDTH
            color = (0, 255, 0) if value == 1 else (0, 0, 255)
            cv2.rectangle(frame_bgr, (x_start, BOTTOM_SENSOR_Y), 
                          (x_end, BOTTOM_SENSOR_Y + SENSOR_HEIGHT), color, 2)
            cv2.putText(frame_bgr, str(i), (x_start + 5, BOTTOM_SENSOR_Y + 25), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
        
        # 顶部传感器区域
        for i, value in enumerate(top_sensors):
            x_start = i * SENSOR_WIDTH
            x_end = (i + 1) * SENSOR_WIDTH
            color = (0, 255, 0) if value == 1 else (0, 0, 255)
            cv2.rectangle(frame_bgr, (x_start, TOP_SENSOR_Y), 
                          (x_end, TOP_SENSOR_Y + SENSOR_HEIGHT), color, 2)
            cv2.putText(frame_bgr, str(i), (x_start + 5, TOP_SENSOR_Y + 25), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)

        # 显示图像
        cv2.imshow(WINDOW_NAME, frame_bgr)
        
        # 按键退出（q键）
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

finally:
    # 资源清理
    picam2.stop()
    cv2.destroyAllWindows()
    print("程序已退出，资源已释放")
