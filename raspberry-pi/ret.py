import cv2
import numpy as np
import os
import time
from picamera2 import Picamera2
import serial
import fcntl
from ultralytics import YOLO
import threading
import json
from functools import partial

# ====================== 初始化部分 ======================
# 禁用 PyKMS 预览(优化树莓派性能)
os.environ["PYKMS_NO_PREVIEW"] = '1'

# 定义分辨率常量
MAX_WIDTH = 640
MAX_HEIGHT = 640

# 初始化树莓派相机
picam2 = Picamera2()
config = picam2.create_video_configuration(
    main={"size": (MAX_WIDTH, MAX_HEIGHT), "format": "BGR888"},
    controls={"FrameRate": 30}
)
picam2.configure(config)
picam2.start()

# 初始化YOLO模型
model = YOLO('710.pt', verbose=False)

# 创建显示窗口
WINDOW_NAME = 'Combined Sensor & Digit Detection'
cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)
cv2.resizeWindow(WINDOW_NAME, MAX_WIDTH, MAX_HEIGHT)

# 二值化阈值
Threshold = 120

# 初始化发送计时器
last_send_time = time.time()
send_interval = 0.5  # 500ms发送一次

# 全局变量定义
turn_record = []  # 转向记录列表，动态增长
return_path = []  # 返程路径指令列表
current_return_index = 0  # 当前返程动作索引
in_return_mode = False  # 是否处于返回模式

# 串口配置
PORT = '/dev/serial0'
BAUDRATE = 115200
TIMEOUT = 0.1
HEADER = b'\xAA'       # 包头
FOOTER = b'\x55\x44\x33'  # 包尾
FOOTER_LEN = len(FOOTER)

# ====================== STM32消息定义 ======================
# STM32发送的消息类型定义
STM32_MESSAGES = {
    b'b': 'RETURN_MODE',      # 开始返回模式
    b'r': 'RESET_SYSTEM',     # 重置系统
    b's': 'START_MISSION',    # 开始任务
}

# ====================== 传感器配置 ======================
# 定义传感器参数
SENSOR_PARAMS = {
    'h_low1': 0,      # 红色范围1的低H值
    'h_high1':30,    # 红色范围1的高H值
    'h_low2': 160,    # 红色范围2的低H值
    'h_high2': 180,   # 红色范围2的高H值
    's_low': 10,     # 饱和度低阈值
    's_high': 255,    # 饱和度高阈值
    'v_low': 10,     # 明度低阈值
    'v_high': 255,    # 明度高阈值
    'hyster_high': 40,  # 滞回高阈值 (0-100)
    'hyster_low': 30    # 滞回低阈值 (0-100)
}

NUM_SENSORS = 8  # 8路传感器
SENSOR_HEIGHT = 40  # 传感器区域高度
SENSOR_WIDTH = MAX_WIDTH // NUM_SENSORS  # 每个传感器的宽度
BOTTOM_SENSOR_Y = MAX_HEIGHT - 80  # 底部传感器区域Y坐标
TOP_SENSOR_Y = 80  # 顶部传感器区域Y坐标

# 传感器状态记忆
last_valid_bottom_sensors = [0,0,0,0,0,0,0,0] 
last_valid_top_sensors = [0,0,0,0,0,0,0,0] 

# ====================== 数字检测配置 ======================
CENTER_X = MAX_WIDTH // 2
CENTER_Y = MAX_HEIGHT // 2
CONFIRMATION_FRAMES = 5  # 连续检测到相同数字的帧数阈值(起始)
CONFIRMATION_FRAMES_2 = 2  # 连续检测到相同区域的帧数阈值(中间)
# 颜色定义(BGR格式)
COLOR_RED = (0, 0, 255)
COLOR_GREEN = (0, 255, 0)
COLOR_BLUE = (255, 0, 0)
COLOR_YELLOW = (0, 255, 255)

# ====================== 系统状态变量 ======================
# 系统模式：0=数字设置模式, 1=传感器模式, 2=数字检测模式, 3=返回模式
system_mode = 0
target_digit = None  # 目标数字
system_paused = False  # 系统暂停标志
emergency_stop = False  # 紧急停止标志

# 数字检测状态
digit_counters = {str(i): 0 for i in range(1, 9)}
last_detected_digit = None
action_counter = 0
last_action = 0
current_action = 0

last_detected_region = None  # 上一次检测到的区域(1为左，2为右)
region_counter = 0           # 连续检测到同一区域的帧数
no_target_counter = 0        # 连续未检测到目标的帧数

# 性能监控
frame_count = 0
start_time = time.monotonic()
last_fps_time = start_time
fps = 0.0

# 内存预分配
gray_frame = np.zeros((640, 640), dtype=np.uint8)
binary_frame = np.zeros((640, 640), dtype=np.uint8)

def reset():
    """系统重置函数"""
    global system_mode, target_digit, digit_counters, last_detected_digit
    global action_counter, last_action, current_action
    global turn_record, return_path, current_return_index, in_return_mode
    global last_detected_region, region_counter, no_target_counter
    global system_paused, emergency_stop, last_valid_bottom_sensors, last_valid_top_sensors
    
    system_mode = 0
    target_digit = None
    digit_counters = {str(i): 0 for i in range(1, 9)}
    last_detected_digit = None
    action_counter = 0
    last_action = 0
    current_action = 0
    system_paused = False
    emergency_stop = False
    
    last_valid_bottom_sensors = [0,0,0,0,0,0,0,0] 
    last_valid_top_sensors = [0,0,0,0,0,0,0,0] 
    
    # 重置返程相关变量
    turn_record = []
    return_path = []
    current_return_index = 0
    in_return_mode = False
    
    # 重置数字检测状态
    last_detected_region = None
    region_counter = 0
    no_target_counter = 0
    
    print("系统已重置")

class STM32SerialReceiver:
    """增强版STM32串口接收器"""
    
    def __init__(self, port=PORT, baudrate=BAUDRATE):
        self.send_lock = threading.Lock() # 新增：为发送操作添加一个锁
        self.serial_port = None
        self.port = port
        self.baudrate = baudrate
        self.running = False
        self.receive_thread = None
        self.buffer = bytearray()  # 接收缓冲区
        self.interrupt_handlers = {}  # 中断处理函数字典
        
        # 消息历史记录
        self.received_messages = []  # 存储格式：[{'type': 消息类型, 'data': 原始数据, 'timestamp': 时间戳}, ...]
        self.data_lock = threading.Lock()  # 线程安全锁
        self.max_history = 1000  # 最大历史消息量
        
        # 统计信息
        self.stats = {
            'total_received': 0,
            'total_sent': 0,
            'errors': 0,
            'last_error': None
        }
    def send(self, data):
        """
        通过持久打开的串口发送数据。
        这个方法是线程安全的。
        """
        if not self.is_connected():
            print("发送失败：串口未连接。")
            return False
            
        try:
            # 使用锁确保发送操作的原子性，防止与接收线程冲突
            with self.send_lock:
                if isinstance(data, str):
                    data = data.encode('utf-8')
                
                frame = HEADER + data + FOOTER
                self.serial_port.write(frame)
                self.serial_port.flush() # 建议在关键指令后刷新缓冲区
                self.stats['total_sent'] += 1
                
                # print(f"已发送帧: {frame.hex()}") # 可以取消注释用于调试
                return True
        except Exception as e:
            print(f"发送异常: {str(e)}")
            self.stats['errors'] += 1
            self.stats['last_error'] = f"发送错误: {str(e)}"
            return False
        
    def register_interrupt(self, trigger_data, handler):
        """注册中断：当接收到trigger_data时，调用handler函数"""
        self.interrupt_handlers[trigger_data] = handler
        print(f"注册中断处理器: {trigger_data} -> {handler.__name__}")

    def start(self):
        """启动接收线程"""
        try:
            self.serial_port = serial.Serial(
                self.port,
                self.baudrate,
                timeout=TIMEOUT,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                bytesize=serial.EIGHTBITS
            )

            if self.serial_port.is_open:
                print(f"成功打开串口: {self.port}")
                self.running = True
                self.receive_thread = threading.Thread(target=self._receive_loop, daemon=True)
                self.receive_thread.start()
                
                print("串口接收器已启动")               
                return True
            return False

        except Exception as e:
            print(f"打开串口失败: {str(e)}")
            self.stats['errors'] += 1
            self.stats['last_error'] = str(e)
            return False

    def _receive_loop(self):
        """接收数据主循环(持续运行在子线程)"""
        while self.running and self.serial_port.is_open:
            try:
                # 读取可用数据
                data = self.serial_port.read(self.serial_port.in_waiting or 1)
                if data:
                    self.buffer.extend(data)
                    self._process_buffer()

            except serial.SerialException as e:
                print(f"串口错误: {str(e)}")
                self.stats['errors'] += 1
                self.stats['last_error'] = str(e)
                time.sleep(1)
            except Exception as e:
                print(f"接收错误: {str(e)}")
                self.stats['errors'] += 1
                self.stats['last_error'] = str(e)
                time.sleep(1)

    def _process_buffer(self):
        while len(self.buffer) > 0:
            # 查找可能的包头位置（0xAA 或 0xBB）
            header_aa_idx = self.buffer.find(b'\xAA')
            header_bb_idx = self.buffer.find(b'\xBB')
            
            # 确定最先出现的有效包头位置
            header_idx = -1
            header_type = None
            
            if header_aa_idx != -1 and header_bb_idx != -1:
                if header_aa_idx < header_bb_idx:
                    header_idx = header_aa_idx
                    header_type = b'\xAA'
                else:
                    header_idx = header_bb_idx
                    header_type = b'\xBB'
            elif header_aa_idx != -1:
                header_idx = header_aa_idx
                header_type = b'\xAA'
            elif header_bb_idx != -1:
                header_idx = header_bb_idx
                header_type = b'\xBB'
            
            # 没有找到有效包头
            if header_idx == -1:
                break
                
            # 移除包头前的无效数据
            if header_idx > 0:
                del self.buffer[:header_idx]
                continue
                
            # 处理0xAA包头的消息（原逻辑）
            if header_type == b'\xAA':
                # 查找包尾（在整个缓冲区中查找）
                footer_idx = self.buffer.find(FOOTER, len(HEADER))
                if footer_idx == -1:
                    break  # 包尾未找到，保留数据等待后续接收
                    
                # 提取完整数据包
                packet_end = footer_idx + FOOTER_LEN
                packet = self.buffer[:packet_end]
                del self.buffer[:packet_end]
                
                # 提取有效载荷（去头去尾）
                payload = bytes(packet[len(HEADER):-FOOTER_LEN])
                
                if payload:
                    self._save_message(payload)
                    self._check_interrupt(payload)
                    self.stats['total_received'] += 1
                    
            # 处理0xBB包头的消息（新逻辑）
            elif header_type == b'\xBB':
                # 查找包尾（跳过包头位置）
                footer_idx = self.buffer.find(FOOTER, 1)  # 从索引1开始查找
                if footer_idx == -1:
                    break  # 包尾未找到，保留数据等待后续接收
                    
                # 提取完整数据包（包括包头和包尾）
                packet_end = footer_idx + FOOTER_LEN
                packet = self.buffer[:packet_end]
                del self.buffer[:packet_end]
                
                # 提取有效载荷（去头去尾，使用与AA包一致的方式）
                payload = bytes(packet[1:-FOOTER_LEN])  # BB包头长度为1字节
                
                if payload:
                    try:
                        # 尝试将载荷作为UTF-8字符串解码
                        message = payload.decode('utf-8')
                        print(f"Received message: {message}")
                    except UnicodeDecodeError:
                        # 若解码失败，打印原始字节的十六进制表示
                        hex_str = ' '.join([f'{b:02X}' for b in payload])
                        print(f"Received payload (hex): {hex_str}")
                
                self.stats['total_received'] += 1


    def _save_message(self, payload):
        """保存接收到的消息(线程安全)"""
        with self.data_lock:
            timestamp = time.time()
            
            # 解析消息类型
            message_type = 'UNKNOWN'
            if payload in STM32_MESSAGES:
                message_type = STM32_MESSAGES[payload]
            
            # 尝试解码为字符串
            try:
                char_data = payload.decode('utf-8')
            except UnicodeDecodeError:
                char_data = f"[无法解码: {payload.hex()}]"
            
            message_info = {
                'type': message_type,
                'raw_bytes': payload,
                'char_data': char_data,
                'timestamp': timestamp,
                'formatted_time': time.strftime('%H:%M:%S', time.localtime(timestamp))
            }
            
            self.received_messages.append(message_info)
            
            # 限制历史长度
            if len(self.received_messages) > self.max_history:
                self.received_messages.pop(0)

    def _check_interrupt(self, data):
        """检查并触发中断"""
        
        # 检查注册的中断
        for trigger, handler in self.interrupt_handlers.items():
            if data == trigger:
                print(f"\n触发中断！检测到: {STM32_MESSAGES.get(trigger, trigger)}")
                try:
                    # 在新线程中执行中断处理
                    threading.Thread(target=handler, daemon=True).start()
                except Exception as e:
                    print(f"中断处理器执行错误: {str(e)}")
                    self.stats['errors'] += 1
                    self.stats['last_error'] = f"中断处理错误: {str(e)}"
                return
        
        # 处理其他消息
        if data not in self.interrupt_handlers.keys():
            message_type = STM32_MESSAGES.get(data, "UNKNOWN")
            print(f"\n收到STM32消息: {message_type}")
            print(f"   原始数据: {data.hex()}")
            print(f"   字符数据: {data.decode('utf-8', errors='replace')}")

    def get_recent_messages(self, count=10):
        """获取最近的消息"""
        with self.data_lock:
            return self.received_messages[-count:].copy()

    def get_latest_message(self):
        """获取最新一条消息"""
        with self.data_lock:
            if self.received_messages:
                return self.received_messages[-1].copy()
            return None

    def clear_message_history(self):
        """清空消息历史"""
        with self.data_lock:
            self.received_messages.clear()
        print("已清空消息历史")

    def get_stats(self):
        """获取通信统计信息"""
        return self.stats.copy()

    def is_connected(self):
        """检查连接状态"""
        return (self.running and 
                self.serial_port and 
                self.serial_port.is_open)

    def stop(self):
        """停止接收并释放资源"""
        self.running = False
        if self.receive_thread and self.receive_thread.is_alive():
            self.receive_thread.join(timeout=2)
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.close()
            print("串口已关闭")

# ====================== STM32中断处理函数 ======================
def handle_return_mode_interrupt():
    """处理返回模式中断(收到'b')"""
    global system_mode, in_return_mode
    system_mode = 3  # 切换到返回模式
    in_return_mode = True
    print("执行返回模式中断 - 开始返程")

def handle_reset_interrupt():
    """处理系统重置中断(收到'r')"""
    print("执行系统重置中断")
    reset()

# ====================== 传感器与数字检测功能函数 ======================
def get_sensor_values(mask, y_pos, last_values):
    """获取传感器值(0或1)，使用滞回比较"""
    sensor_values = []
    hyster_high = SENSOR_PARAMS['hyster_high'] / 100.0
    hyster_low = SENSOR_PARAMS['hyster_low'] / 100.0
    
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
            state_byte |= (1 << (7 - i))
    return state_byte

def process_sensor_mode1(frame_bgr, serial_comm):
    """处理传感器模式1(正常模式)"""
    global last_valid_bottom_sensors, last_valid_top_sensors, system_mode,last_send_time, in_return_mode 

    # 图像处理
    hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)
    blurred = cv2.GaussianBlur(hsv, (5, 5), 0)

    # 双范围红色掩码
    lower1 = np.array([SENSOR_PARAMS['h_low1'], SENSOR_PARAMS['s_low'], SENSOR_PARAMS['v_low']])
    upper1 = np.array([SENSOR_PARAMS['h_high1'], SENSOR_PARAMS['s_high'], SENSOR_PARAMS['v_high']])
    lower2 = np.array([SENSOR_PARAMS['h_low2'], SENSOR_PARAMS['s_low'], SENSOR_PARAMS['v_low']])
    upper2 = np.array([SENSOR_PARAMS['h_high2'], SENSOR_PARAMS['s_high'], SENSOR_PARAMS['v_high']])
    mask = cv2.inRange(blurred, lower1, upper1) | cv2.inRange(blurred, lower2, upper2)

    # 形态学处理
    ksize = max(3, int(min(frame_bgr.shape[:2]) / 100) * 2 + 1)
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (ksize, ksize))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    # 获取传感器值
    bottom_sensors = get_sensor_values(mask, BOTTOM_SENSOR_Y, last_valid_bottom_sensors)
    top_sensors = get_sensor_values(mask, TOP_SENSOR_Y, last_valid_top_sensors)

    bottom_byte = sensor_list_to_byte(bottom_sensors)  # 保证后续使用不报错

    # 状态更新判断
    updated = False
    if last_valid_bottom_sensors != bottom_sensors:

        last_valid_bottom_sensors = bottom_sensors.copy()
        updated = True
        # 发送传感器数据(受计时器控制)
        current_time = time.time()
        if current_time - last_send_time >= send_interval:
            serial_comm.send(bytes([bottom_byte]))
            last_send_time = current_time  # 更新发送时间
        # 打印传感器状态
        bottom_bin = bin(bottom_byte)[2:].zfill(8)
        top_bin = bin(sensor_list_to_byte(top_sensors))[2:].zfill(8)
        print(f"Bottom Sensors: {bottom_bin} | Top Sensors: {top_bin}")

    # 检查是否所有底部传感器都为1
    if updated and all(sensor == 1 for sensor in bottom_sensors):
        serial_comm.send(b'c')
        serial_comm.send(b'c')
        serial_comm.send(b'c')
        print("所有底部传感器检测到红色，切换到数字检测模式")
        system_mode = 2  # 切换到数字检测模式

    if updated and all(sensor == 0 for sensor in bottom_sensors) :
        serial_comm.send(b'f')
        serial_comm.send(b'f')
        serial_comm.send(b'f')
        print("所有底部传感器未检测到红色,发送00000000")

    return frame_bgr, bottom_sensors, top_sensors  

def process_sensor_mode3(frame_bgr, serial_comm):
    """处理传感器模式3(返回模式)"""
    global last_valid_bottom_sensors, last_valid_top_sensors, last_send_time
    global in_return_mode, current_return_index, system_mode 
    
    # 图像处理
    hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)
    blurred = cv2.GaussianBlur(hsv, (5, 5), 0)

    # 双范围红色掩码
    lower1 = np.array([SENSOR_PARAMS['h_low1'], SENSOR_PARAMS['s_low'], SENSOR_PARAMS['v_low']])
    upper1 = np.array([SENSOR_PARAMS['h_high1'], SENSOR_PARAMS['s_high'], SENSOR_PARAMS['v_high']])
    lower2 = np.array([SENSOR_PARAMS['h_low2'], SENSOR_PARAMS['s_low'], SENSOR_PARAMS['v_low']])
    upper2 = np.array([SENSOR_PARAMS['h_high2'], SENSOR_PARAMS['s_high'], SENSOR_PARAMS['v_high']])
    mask = cv2.inRange(blurred, lower1, upper1) | cv2.inRange(blurred, lower2, upper2)

    # 形态学处理
    ksize = max(3, int(min(frame_bgr.shape[:2]) / 100) * 2 + 1)
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (ksize, ksize))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    # 获取传感器值
    bottom_sensors = get_sensor_values(mask, BOTTOM_SENSOR_Y, last_valid_bottom_sensors)
    top_sensors = get_sensor_values(mask, TOP_SENSOR_Y, last_valid_top_sensors)
    
    bottom_byte = sensor_list_to_byte(bottom_sensors)  # 保证后续使用不报错

    updated = False
    if last_valid_bottom_sensors != bottom_sensors:
        last_valid_bottom_sensors = bottom_sensors.copy()
        last_valid_top_sensors = top_sensors.copy()
        updated = True
        # 发送传感器数据(受计时器控制)
        current_time = time.time()
        if current_time - last_send_time >= send_interval:
            serial_comm.send(bytes([bottom_byte]))
            last_send_time = current_time  # 更新发送时间
        # 打印传感器状态
        bottom_bin = bin(bottom_byte)[2:].zfill(8)
        top_bin = bin(sensor_list_to_byte(top_sensors))[2:].zfill(8)
        #print(f"Bottom Sensors: {bottom_bin} | Top Sensors: {top_bin}")

    # 检查是否所有底部传感器都为1
    if updated and all(sensor == 1 for sensor in bottom_sensors):
        serial_comm.send(b'c')
        serial_comm.send(b'c')
        serial_comm.send(b'c')
        print("所有底部传感器检测到红色,按顺序返回")
        # 确保索引有效
        if 0 <= current_return_index < len(return_path):
            action = return_path[current_return_index]
            if serial_comm.send(bytes([action])):
                print(f"已发送返程动作: {action}")
            current_return_index += 1

    if not in_return_mode:
        # 准备返程路径
        prepare_return_path()
        in_return_mode = True
        print(f"准备返程路径: {return_path}")

    if updated and all(sensor == 0 for sensor in bottom_sensors) and current_return_index == len(return_path):
        serial_comm.send(b'e')
        serial_comm.send(b'e')
        serial_comm.send(b'e')
        print("完成返程")
        reset()
    
    return frame_bgr, bottom_sensors, top_sensors

def draw_sensor_overlay(frame_bgr, bottom_sensors, top_sensors):
    """绘制传感器覆盖层"""
    # 底部传感器区域
    for i, value in enumerate(bottom_sensors):
        x_start = i * SENSOR_WIDTH
        x_end = (i + 1) * SENSOR_WIDTH
        color = COLOR_GREEN if value == 1 else COLOR_RED
        cv2.rectangle(frame_bgr, (x_start, BOTTOM_SENSOR_Y), 
                      (x_end, BOTTOM_SENSOR_Y + SENSOR_HEIGHT), color, 2)
        cv2.putText(frame_bgr, str(i), (x_start + 5, BOTTOM_SENSOR_Y + 25), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, COLOR_RED, 1)
    
    # 顶部传感器区域
    for i, value in enumerate(top_sensors):
        x_start = i * SENSOR_WIDTH
        x_end = (i + 1) * SENSOR_WIDTH
        color = COLOR_GREEN if value == 1 else COLOR_RED
        cv2.rectangle(frame_bgr, (x_start, TOP_SENSOR_Y), 
                      (x_end, TOP_SENSOR_Y + SENSOR_HEIGHT), color, 2)
        cv2.putText(frame_bgr, str(i), (x_start + 5, TOP_SENSOR_Y + 25), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, COLOR_RED, 1)

def process_digit_setup_mode(frame_bgr, serial_comm):
    """处理数字设置模式"""
    global target_digit, system_mode, digit_counters, last_detected_digit 

    # 使用单次操作完成灰度化和二值化
    gray_frame = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2GRAY)
    _, binary_frame = cv2.threshold(gray_frame, Threshold, 255, cv2.THRESH_BINARY)
    
    # 高效创建三通道图像
    display_frame = cv2.cvtColor(binary_frame, cv2.COLOR_GRAY2BGR)
    
    # YOLO推理
    results = model.predict(
        display_frame, 
        imgsz=320,
        conf=0.8,
        device='cpu',
        half=False,
        augment=False,
        max_det=1,
        verbose=False
    )

    # 只绘制最大置信度的目标数字框
    annotated_frame = display_frame.copy()
    current_digit = None
    max_conf = -1
    max_box = None
    max_digit = None
    if results[0].boxes is not None:
        for box in results[0].boxes:
            conf = float(box.conf[0])
            cls = int(box.cls[0])
            digit = str(model.names[cls])
            if conf > max_conf:
                max_conf = conf
                max_box = box
                max_digit = digit
        if max_box is not None:
            x1, y1, x2, y2 = map(int, max_box.xyxy[0])
            cv2.rectangle(annotated_frame, (x1, y1), (x2, y2), COLOR_RED, 2)
            cv2.putText(annotated_frame, f"{max_digit}:{max_conf:.2f}", (x1, y1-10), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, COLOR_RED, 2)
            current_digit = max_digit

    # 目标确认机制
    if current_digit and current_digit == last_detected_digit:
        digit_counters[current_digit] += 1
        if digit_counters[current_digit] >= CONFIRMATION_FRAMES:
            target_digit = current_digit
            # 发送直行动作
            if serial_comm.send(b'w'):
                print("已发送直行动作(w)")

            system_mode = 1  # 切换到传感器模式
            print(f"目标数字确认: {target_digit}, 切换到传感器模式")
            # 重置检测状态
            digit_counters = {str(i): 0 for i in range(1, 9)}
            last_detected_digit = None
    else:
        if last_detected_digit:
            digit_counters[last_detected_digit] = 0

    last_detected_digit = current_digit

    return annotated_frame, current_digit

def process_digit_detection_mode(frame_bgr, serial_comm):
    """处理数字检测模式"""
    global system_mode, action_counter, last_action, current_action, turn_record, current_return_index
    global last_detected_region, region_counter, no_target_counter 

    # 图像预处理
    gray_frame = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2GRAY)
    _, binary_frame = cv2.threshold(gray_frame, Threshold, 255, cv2.THRESH_BINARY)
    display_frame = cv2.cvtColor(binary_frame, cv2.COLOR_GRAY2BGR)
    
    # 优化推理参数
    results = model.predict(
        display_frame,
        imgsz=320,
        conf=0.8,
        device='cpu',
        half=False,
        augment=False,
        max_det=1,
        verbose=False
    )

    # 绘制检测区域
    annotated_frame = display_frame.copy()
    cv2.rectangle(annotated_frame, (0, 0), (CENTER_X, MAX_HEIGHT), COLOR_RED, 2)
    cv2.rectangle(annotated_frame, (CENTER_X, 0), (MAX_WIDTH, MAX_HEIGHT), COLOR_RED, 2)

    # 处理检测结果
    action = 'w'
    target_detected = False
    max_conf = -1
    max_center_x = None
    max_box = None
    max_digit = None

    if results[0].boxes is not None:
        for box in results[0].boxes:
            conf = float(box.conf[0])
            cls = int(box.cls[0])
            digit = str(model.names[cls])
            if conf > max_conf:
                max_conf = conf
                max_box = box
                max_digit = digit
        if max_box is not None:
            x1, y1, x2, y2 = map(int, max_box.xyxy[0])
            max_center_x = (x1 + x2) // 2
            # 判断是否为目标数字且置信度足够
            if max_digit == target_digit and max_conf >= 0.8:
                target_detected = True
                cv2.rectangle(annotated_frame, (x1, y1), (x2, y2), COLOR_GREEN, 2)
                cv2.putText(annotated_frame, f"{max_digit}:{max_conf:.2f}", (x1, y1-10), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, COLOR_GREEN, 2)

    # 动作决策
    if target_detected and max_center_x is not None:
        no_target_counter = 0
        if max_center_x < CENTER_X:
            current_region = 1  # 左侧区域
        else:
            current_region = 2  # 右侧区域

        if current_region == last_detected_region:
            region_counter += 1
        else:
            region_counter = 1
            last_detected_region = current_region

        if region_counter >= CONFIRMATION_FRAMES_2:
            if current_region == 1:
                action = 'a'
                turn_record.append(action)
            else:
                action = 'd'
                turn_record.append(action)
    else:
        # 未检测到目标数字
        no_target_counter += 0
        region_counter = 0
        last_detected_region = None
        #print(f"未检测到目标数字，no_target_counter = {no_target_counter}")

        if no_target_counter >= CONFIRMATION_FRAMES_2:
            action = 'w'
            turn_record.append(action)
            print(f"no_target_counter 达到阈值，action='w'")

    # 动作稳定性处理
    if action == last_action:
        action_counter = min(action_counter + 1, 10)
    else:
        action_counter = max(action_counter - 2, 0)

    if action_counter >= 2:
        current_action = action
        last_action = action

    # 完成检测后切换回传感器模式
    if region_counter >= CONFIRMATION_FRAMES_2 or no_target_counter >= CONFIRMATION_FRAMES_2:
        # 发送动作数据
        if action in ['a', 'd', 'w']: 
            if serial_comm.send(action.encode('utf-8')):
                serial_comm.send(action.encode('utf-8'))
                print(f"已发送动作: {action}")

        # 更新返程索引
        current_return_index = len(turn_record) - 1

        # 重置检测状态
        region_counter = 0
        no_target_counter = 0
        system_mode = 1  # 切换回传感器模式
        print(f"完成数字检测，切换回传感器模式。当前动作: {action}")

    return annotated_frame, target_detected, action

def prepare_return_path():
    """准备返程路径"""
    global return_path, current_return_index
    
    # 反转并转换方向
    return_path = []
    for action in reversed(turn_record):
        if action == 'a':
            return_path.append(2)  # 左转变右转
        elif action == 'd':
            return_path.append(1)  # 右转变左转
        else:
            return_path.append(0)  # 直行不变
    
    current_return_index = 0
    print(f"准备返程路径: {return_path}")

# ====================== 主循环 ======================
def main():
    # 创建串口中断接收器实例
    serial_comm = STM32SerialReceiver()
    
    # 注册串口中断处理函数
    serial_comm.register_interrupt(b'b', handle_return_mode_interrupt)
    serial_comm.register_interrupt(b'r', handle_reset_interrupt)
    
    # 启动串口通信
    if not serial_comm.start():
        print("串口初始化失败，程序退出")
        return
    
    print("Starting Combined System...")
    print("Phase 1: Digit Setup Mode - Place target digit (1-8) in view")

    while True:
        # 捕获图像
        frame = picam2.capture_array()
        frame_bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
            
        # 更新FPS
        global frame_count, start_time, last_fps_time, fps
        frame_count += 1
        current_time = time.monotonic()
        if current_time - last_fps_time >= 1.0:
            fps = frame_count / (current_time - last_fps_time)
            frame_count = 0
            last_fps_time = current_time
            
        # 根据系统模式处理
        if system_mode == 0:  # 数字设置模式
            display_frame, current_digit = process_digit_setup_mode(frame_bgr, serial_comm)
            
            # 显示状态信息
            cv2.putText(display_frame, "Mode: Digit Setup", (10, 30), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, COLOR_RED, 2)
            cv2.putText(display_frame, "Place target digit (1-8) in view", (10, 60), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, COLOR_RED, 2)
            cv2.putText(display_frame, f"FPS: {fps:.1f}", (10, MAX_HEIGHT-10), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, COLOR_GREEN, 1)
                
        elif system_mode == 1:  # 传感器模式
            display_frame, bottom_sensors, top_sensors = process_sensor_mode1(frame_bgr, serial_comm)
            draw_sensor_overlay(display_frame, bottom_sensors, top_sensors)
                
            # 显示状态信息
            cv2.putText(display_frame, f"Mode: Sensor (Target: {target_digit})", (10, 30), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, COLOR_GREEN, 2)
            cv2.putText(display_frame, "Waiting for all bottom sensors = 1", (10, 60), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, COLOR_GREEN, 2)
            cv2.putText(display_frame, f"FPS: {fps:.1f}", (10, MAX_HEIGHT-10), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, COLOR_GREEN, 1)
                
        elif system_mode == 2:  # 数字检测模式
            display_frame, target_detected, action = process_digit_detection_mode(frame_bgr, serial_comm)
                
            # 显示状态信息
            cv2.putText(display_frame, f"Mode: Digit Detection (Target: {target_digit})", (10, 30), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, COLOR_RED, 2)
            cv2.putText(display_frame, f"Action: {current_action}", (10, 60), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, COLOR_RED, 2)
            cv2.putText(display_frame, f"FPS: {fps:.1f}", (10, MAX_HEIGHT-10), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, COLOR_GREEN, 1)
                       
        elif system_mode == 3:  # 返回模式
            display_frame, bottom_sensors, top_sensors = process_sensor_mode3(frame_bgr, serial_comm)
            draw_sensor_overlay(display_frame, bottom_sensors, top_sensors)

            # 显示状态信息
            cv2.putText(display_frame, "Mode: Return Path", (10, 30), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, COLOR_GREEN, 2)
            cv2.putText(display_frame, f"Step: {current_return_index}/{len(return_path)}", (10, 60), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, COLOR_GREEN, 2)
            cv2.putText(display_frame, f"FPS: {fps:.1f}", (10, MAX_HEIGHT-10), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, COLOR_GREEN, 1)

        # 显示图像
        cv2.imshow(WINDOW_NAME, display_frame)
            
        # 键盘控制
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('r'):
            # 重置系统
            print("重置系统")
            reset()

if __name__ == "__main__":
    main()
