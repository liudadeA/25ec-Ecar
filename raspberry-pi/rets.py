import cv2
import numpy as np
import time
import serial
import math
import os
from picamera2 import Picamera2

# ============== 配置参数 ==============
# 图像分辨率设置
PICTURE_WIDTH = 640
PICTURE_HEIGHT = 480

# 显示器配置（左右并排）
DISPLAY_WIDTH = 2 * PICTURE_WIDTH + 80  # 左侧原始图 + 中间处理图 + 右侧控制区
DISPLAY_HEIGHT = PICTURE_HEIGHT         # 高度与单张图像一致

# 计算相机中心点坐标
CAMERA_CENTER = (PICTURE_WIDTH // 2, PICTURE_HEIGHT // 2)

# 滑块配置（右侧）
SLIDER_WIDTH = 40
SLIDER_HEIGHT = 200
SLIDER_X = 2 * PICTURE_WIDTH + 20  # 右侧控制区X坐标（两张图宽度+间距）
SLIDER_Y = (DISPLAY_HEIGHT - SLIDER_HEIGHT) // 2  # 垂直居中
THRESHOLD_MIN = 0
THRESHOLD_MAX = 255
current_threshold = 120
auto_threshold = True  # 自动阈值开关

# 中心点误差阈值（横纵方向都需小于此值）
CENTER_ERROR_THRESHOLD = 50

# 显示窗口设置
WINDOW_NAME = 'Target Tracker'

# 显示控制开关
show_display = True  # 默认开启本地显示

# ============== 初始化硬件 ==============
def init_camera():
    """初始化树莓派相机"""
    try:
        picam2 = Picamera2()
        config = picam2.create_video_configuration(
            main={"size": (PICTURE_WIDTH, PICTURE_HEIGHT), "format": "BGR888"},
            controls={"FrameRate": 60}
        )
        picam2.configure(config)
        picam2.start()
        print("摄像头初始化成功")
        return picam2
    except Exception as e:
        print(f"摄像头初始化失败: {e}")
        return None

def init_uart():
    """初始化UART串口"""
    try:
        u1 = serial.Serial('/dev/serial0', baudrate=115200, timeout=0.1)
        print("串口初始化成功")
        return u1
    except Exception as e:
        print(f"串口初始化失败: {e}")
        return None

def init_display():
    """初始化显示窗口"""
    if show_display:
        cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(WINDOW_NAME, DISPLAY_WIDTH, DISPLAY_HEIGHT)
        print("显示窗口初始化成功")

# ============== 鼠标事件处理 ==============
class MouseHandler:
    def __init__(self):
        self.x = 0
        self.y = 0
        self.pressed = False
    
    def callback(self, event, x, y, flags, param):
        # 只有显示开启时才处理鼠标事件
        if not show_display:
            return
            
        self.x, self.y = x, y
        if event == cv2.EVENT_LBUTTONDOWN:
            self.pressed = True
        elif event == cv2.EVENT_LBUTTONUP:
            self.pressed = False

# ============== 矩形检测类 ==============
class RectangleDetector:
    def __init__(self):
        self.min_area = 1000
        self.aspect_ratio_min = 0.5
        self.aspect_ratio_max = 2
    
    def find_rectangles(self, gray_img, threshold, use_auto_threshold=False):
        """在灰度图像中查找矩形，支持自动阈值"""
        # 二值化图像
        if use_auto_threshold:
            # 使用Otsu自动阈值
            _, binary = cv2.threshold(gray_img, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)
            auto_thresh_value = cv2.threshold(gray_img, 0, 255, cv2.THRESH_OTSU)[0]
        else:
            # 使用手动阈值
            _, binary = cv2.threshold(gray_img, threshold, 255, cv2.THRESH_BINARY_INV)
            auto_thresh_value = threshold
        
        # 查找轮廓
        contours, _ = cv2.findContours(binary, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
        
        rectangles = []
        for contour in contours:
            # 计算轮廓面积
            area = cv2.contourArea(contour)
            if area < self.min_area:
                continue
            
            # 计算最小外接矩形
            rect = cv2.minAreaRect(contour)
            box = cv2.boxPoints(rect)
            box = np.intp(box)
            
            # 获取矩形参数
            x, y, w, h = cv2.boundingRect(box)
            
            # 计算宽高比
            aspect_ratio = max(w, h) / min(w, h)
            if self.aspect_ratio_min < aspect_ratio < self.aspect_ratio_max:
                # 创建矩形对象
                rectangle = Rectangle(x, y, w, h, box)
                rectangles.append(rectangle)
        
        return rectangles, binary, auto_thresh_value

class Rectangle:
    """矩形对象，兼容K230版本的接口"""
    def __init__(self, x, y, w, h, corners_array=None):
        self._x = x
        self._y = y
        self._w = w
        self._h = h
        self._corners = corners_array if corners_array is not None else []
    
    def x(self):
        return self._x
    
    def y(self):
        return self._y
    
    def w(self):
        return self._w
    
    def h(self):
        return self._h
    
    def area(self):
        return self._w * self._h
    
    def corners(self):
        """返回矩形的四个角点"""
        if len(self._corners) >= 4:
            return [(int(p[0]), int(p[1])) for p in self._corners]
        else:
            # 如果没有角点信息，根据矩形生成
            return [
                (self._x, self._y + self._h),  # 左下
                (self._x + self._w, self._y + self._h),  # 右下
                (self._x + self._w, self._y),  # 右上
                (self._x, self._y)  # 左上
            ]
    
    def rect(self):
        return (self._x, self._y, self._w, self._h)

# ============== 目标跟踪状态 ==============
class TargetTracker:
    def __init__(self):
        self.locked = False
        self.position = (0, 0)
        self.size = (0, 0)
        self.lost_count = 0
        self.max_lost_frames = 15
        self.nested_rects = []

        # 增强的跟踪参数
        self.confidence = 0.0
        self.stable_count = 0
        self.min_stable_frames = 5
        self.position_history = []
        self.size_history = []
        self.max_history = 8

        # 卡尔曼滤波器参数
        self.kalman_gain = 0.8
        self.position_estimate = (0, 0)
        self.velocity = (0, 0)
        self.velocity_gain = 0.8

        # 搜索区域参数
        self.search_radius_factor = 0.8
        self.size_tolerance = 0.4

    def update(self, rects):
        """更新目标位置和状态"""
        # 计算所有矩形的实际中心
        centers = []
        for r in rects:
            centers.append(self.calculate_actual_center(r))

        # 处理嵌套矩形逻辑 - 新增中心点误差判断
        if len(rects) == 2:
            center1, center2 = centers
            # 计算x和y方向的误差
            x_diff = abs(center1[0] - center2[0])
            y_diff = abs(center1[1] - center2[1])
            
            # 只有横纵误差都小于阈值时才视为有效目标
            if x_diff < CENTER_ERROR_THRESHOLD and y_diff < CENTER_ERROR_THRESHOLD:
                distance = math.sqrt((center1[0]-center2[0])**2 + (center1[1]-center2[1])** 2)
                avg_size = (rects[0].w() + rects[1].w()) / 4

                if distance < avg_size:
                    # 取两个中心的平均值
                    new_pos = ((center1[0] + center2[0]) // 2, (center1[1] + center2[1]) // 2)
                    # 取两个矩形尺寸的平均值
                    new_size = ((rects[0].w() + rects[1].w()) // 2,
                                (rects[0].h() + rects[1].h()) // 2)
                    self.nested_rects = rects
                    if not show_display:  # 不显示时也打印关键信息
                        print(f"检测到符合条件的双矩形，x误差: {x_diff}, y误差: {y_diff}")
                else:
                    # 不满足距离条件，不锁定
                    self.reset()
                    return
            else:
                # 横纵误差超过阈值，不锁定
                if not show_display:  # 不显示时也打印关键信息
                    print(f"矩形中心误差过大，x误差: {x_diff}, y误差: {y_diff}，不锁定")
                self.reset()
                return
        else:
            # 不是两个矩形，不锁定目标
            self.reset()
            return

        # 首次锁定目标
        if not self.locked:
            self.position = new_pos
            self.size = new_size
            self.position_estimate = new_pos
            self.velocity = (0, 0)
            self.locked = True
            self.lost_count = 0
            self.stable_count = 1
            self.confidence = 0.5

            # 初始化历史记录
            self.position_history = [new_pos]
            self.size_history = [new_size]
            return

        # 更新历史记录
        self.position_history.append(new_pos)
        self.size_history.append(new_size)
        if len(self.position_history) > self.max_history:
            self.position_history.pop(0)
            self.size_history.pop(0)

        # 计算速度
        if len(self.position_history) >= 2:
            prev_pos = self.position_history[-2]
            curr_pos = self.position_history[-1]
            new_velocity = (
                (curr_pos[0] - prev_pos[0]) * self.velocity_gain + self.velocity[0] * (1 - self.velocity_gain),
                (curr_pos[1] - prev_pos[1]) * self.velocity_gain + self.velocity[1] * (1 - self.velocity_gain)
            )
            self.velocity = new_velocity

        # 卡尔曼滤波更新位置估计
        predicted_x = self.position_estimate[0] + self.velocity[0]
        predicted_y = self.position_estimate[1] + self.velocity[1]

        x = predicted_x + self.kalman_gain * (new_pos[0] - predicted_x)
        y = predicted_y + self.kalman_gain * (new_pos[1] - predicted_y)
        self.position_estimate = (x, y)

        # 更新实际位置和尺寸（使用历史平滑）
        if len(self.position_history) >= 3:
            weights = [0.5, 0.3, 0.2]
            avg_x = sum(pos[0] * w for pos, w in zip(self.position_history[-3:], weights))
            avg_y = sum(pos[1] * w for pos, w in zip(self.position_history[-3:], weights))
            self.position = (int(round(avg_x)), int(round(avg_y)))

            avg_w = sum(size[0] * w for size, w in zip(self.size_history[-3:], weights))
            avg_h = sum(size[1] * w for size, w in zip(self.size_history[-3:], weights))
            self.size = (int(round(avg_w)), int(round(avg_h)))
        else:
            self.position = (int(round(new_pos[0])), int(round(new_pos[1])))
            self.size = (int(round(new_size[0])), int(round(new_size[1])))

        # 更新跟踪状态
        self.lost_count = 0
        self.stable_count += 1

        # 动态调整置信度
        distance_to_predicted = math.sqrt(
            (new_pos[0] - self.position_estimate[0])**2 +
            (new_pos[1] - self.position_estimate[1])** 2
        )
        confidence_factor = max(0.1, 1.0 - distance_to_predicted / 100.0)
        self.confidence = min(1.0, self.confidence * 0.9 + confidence_factor * 0.1)

    def calculate_actual_center(self, r):
        """计算矩形的实际中心（对角线交点）"""
        corners = r.corners()
        if len(corners) < 4:
            rect = r.rect()
            return (rect[0] + rect[2]//2, rect[1] + rect[3]//2)

        # 使用对角线交点计算中心
        p0, p1, p2, p3 = corners[:4]
        x0, y0 = p0
        x1, y1 = p1
        x2, y2 = p2
        x3, y3 = p3

        # 计算对角线交点
        A1 = y2 - y0
        B1 = x0 - x2
        C1 = x2 * y0 - x0 * y2

        A2 = y3 - y1
        B2 = x1 - x3
        C2 = x3 * y1 - x1 * y3

        denominator = A1 * B2 - A2 * B1

        if abs(denominator) < 1e-5:
            avg_x = (x0 + x1 + x2 + x3) / 4
            avg_y = (y0 + y1 + y2 + y3) / 4
            return (int(avg_x), int(avg_y))
        else:
            x_center = (B1 * C2 - B2 * C1) / denominator
            y_center = (A2 * C1 - A1 * C2) / denominator
            return (int(round(x_center)), int(round(y_center)))

    def predict(self):
        """预测下一帧位置"""
        predicted_x = self.position_estimate[0] + self.velocity[0]
        predicted_y = self.position_estimate[1] + self.velocity[1]
        return (predicted_x, predicted_y)

    def get_search_region(self):
        """获取搜索区域"""
        predicted_pos = self.predict()
        search_radius = max(self.size[0], self.size[1]) * self.search_radius_factor
        return {
            'center': predicted_pos,
            'radius': search_radius
        }

    def is_in_search_region(self, center):
        """检查点是否在搜索区域内"""
        search_region = self.get_search_region()
        distance = math.sqrt(
            (center[0] - search_region['center'][0])**2 +
            (center[1] - search_region['center'][1])** 2
        )
        return distance <= search_region['radius']

    def is_truly_locked(self):
        """检查是否真正稳定锁定"""
        return self.locked and self.stable_count >= self.min_stable_frames and self.confidence > 0.3

    def reset(self):
        """重置跟踪器"""
        self.locked = False
        self.position = (0, 0)
        self.size = (0, 0)
        self.lost_count = 0
        self.position_estimate = (0, 0)
        self.nested_rects = []
        self.confidence = 0.0
        self.stable_count = 0
        self.position_history = []
        self.size_history = []
        self.velocity = (0, 0)

# ============== 匹配算法 ==============
def find_best_matches(detected_rects, tracker):
    """改进的匹配算法，返回最佳匹配的矩形"""
    if not detected_rects:
        return []

    if not tracker.locked:
        return detected_rects

    # 获取搜索区域
    search_region = tracker.get_search_region()
    predicted_pos = search_region['center']

    # 计算所有矩形的匹配分数
    matches = []
    for r in detected_rects:
        center = tracker.calculate_actual_center(r)
        size = (r.w(), r.h())

        # 位置距离分数
        distance = math.sqrt(
            (center[0] - predicted_pos[0])**2 +
            (center[1] - predicted_pos[1])** 2
        )
        distance_score = max(0, 100 - distance)

        # 尺寸相似度分数
        w_ratio = min(size[0], tracker.size[0]) / max(size[0], tracker.size[0])
        h_ratio = min(size[1], tracker.size[1]) / max(size[1], tracker.size[1])
        size_score = (w_ratio + h_ratio) * 50

        # 搜索区域内的加分
        region_bonus = 50 if tracker.is_in_search_region(center) else 0

        # 综合分数
        total_score = distance_score + size_score + region_bonus

        matches.append({
            'rect': r,
            'center': center,
            'size': size,
            'score': total_score,
            'distance': distance
        })

    # 按分数排序
    matches.sort(key=lambda x: x['score'], reverse=True)

    # 返回最佳匹配 - 只考虑两个矩形的情况
    if len(matches) >= 2:
        best_match = matches[0]
        second_match = matches[1]

        # 检查中心点横纵误差
        cx1, cy1 = best_match['center']
        cx2, cy2 = second_match['center']
        x_diff = abs(cx1 - cx2)
        y_diff = abs(cy1 - cy2)

        # 检查嵌套关系
        rect1 = best_match['rect'].rect()
        rect2 = second_match['rect'].rect()

        if ((rect1[0] < rect2[0] and rect1[1] < rect2[1] and
             rect1[0]+rect1[2] > rect2[0]+rect2[2] and
             rect1[1]+rect1[3] > rect2[1]+rect2[3]) or
            (rect2[0] < rect1[0] and rect2[1] < rect1[1] and
             rect2[0]+rect2[2] > rect1[0]+rect1[2] and
             rect2[1]+rect2[3] > rect1[1]+rect1[3])):

            # 检查中心距离和误差阈值
            center_distance = math.sqrt(
                (best_match['center'][0] - second_match['center'][0])**2 +
                (best_match['center'][1] - second_match['center'][1])** 2
            )

            max_size = max(rect1[2], rect1[3], rect2[2], rect2[3])
            if center_distance < max_size / 4 and x_diff < CENTER_ERROR_THRESHOLD and y_diff < CENTER_ERROR_THRESHOLD:
                return [best_match['rect'], second_match['rect']]

    return []

# ============== 绘制函数 ==============
def draw_to_canvas(canvas, show_img, processed_img, tracker, detected_rects, threshold, auto_thresh, fps):
    """统一绘制所有元素到画布（左右并排布局）"""
    # 如果关闭显示，不执行任何绘制操作
    if not show_display:
        return
        
    # 1. 绘制原始图像到左侧
    canvas[0:PICTURE_HEIGHT, 0:PICTURE_WIDTH] = show_img
    
    # 2. 绘制处理后的图像到中间（左侧图像右侧）
    if len(processed_img.shape) == 2:
        processed_img = cv2.cvtColor(processed_img, cv2.COLOR_GRAY2BGR)
    canvas[0:PICTURE_HEIGHT, PICTURE_WIDTH:2*PICTURE_WIDTH] = processed_img
    
    # 3. 绘制右侧控制面板背景
    canvas[0:DISPLAY_HEIGHT, 2*PICTURE_WIDTH:DISPLAY_WIDTH] = (0, 0, 0)
    
    # 4. 绘制滑块
    draw_slider(canvas, threshold, auto_thresh)
    
    # 5. 绘制状态信息
    draw_status_info(canvas, tracker, fps, auto_thresh)
    
    # 6. 绘制搜索区域（在两张图上都显示）
    if tracker.locked:
        search_region = tracker.get_search_region()
        # 在左侧原始图上绘制
        cv2.circle(canvas, 
                  (int(search_region['center'][0]), int(search_region['center'][1])),
                  int(search_region['radius']), 
                  (128, 128, 128), 1)
        # 在中间处理图上绘制（坐标偏移PICTURE_WIDTH）
        cv2.circle(canvas, 
                  (int(search_region['center'][0]) + PICTURE_WIDTH, int(search_region['center'][1])),
                  int(search_region['radius']), 
                  (128, 128, 128), 1)

    # 7. 绘制检测到的矩形（在两张图上都显示）
    draw_detected_rectangles(canvas, detected_rects, tracker)
    
    # 8. 绘制跟踪状态（在两张图上都显示）
    draw_tracking_status(canvas, tracker)
    
    # 9. 显示显示状态
    display_status = "显示: 开启" if show_display else "显示: 关闭"
    cv2.putText(canvas, display_status, (SLIDER_X-75, SLIDER_Y + SLIDER_HEIGHT + 30), 
               cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)

def draw_slider(canvas, threshold, auto_thresh):
    """绘制滑块（右侧控制区）"""
    # 滑块背景条
    if auto_thresh:
        slider_color = (0, 165, 255)  # 橙色表示自动模式
    else:
        slider_color = (0, 255, 255)  # 黄色表示手动模式
        
    cv2.rectangle(canvas, 
                 (SLIDER_X, SLIDER_Y), 
                 (SLIDER_X + SLIDER_WIDTH, SLIDER_Y + SLIDER_HEIGHT), 
                 slider_color, 1)
    
    # 滑块按钮
    ratio = (threshold - THRESHOLD_MIN) / (THRESHOLD_MAX - THRESHOLD_MIN)
    slider_pos_y = SLIDER_Y + int((SLIDER_HEIGHT - 10) * (1 - ratio))
    cv2.rectangle(canvas, 
                 (SLIDER_X - 5, slider_pos_y), 
                 (SLIDER_X + SLIDER_WIDTH + 5, slider_pos_y + 10), 
                 slider_color, -1)
    
    # 阈值文本
    text = f"TH:{threshold}"
    cv2.putText(canvas, text, (SLIDER_X-75, SLIDER_Y - 45), 
               cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
    
    # 自动阈值模式指示
    mode_text = "AUTO" if auto_thresh else "MANUAL"
    cv2.putText(canvas, mode_text, (SLIDER_X-45, SLIDER_Y - 65), 
               cv2.FONT_HERSHEY_SIMPLEX, 0.5, slider_color, 1)

def draw_status_info(canvas, tracker, fps, auto_thresh):
    """绘制状态信息（右侧控制区）"""
    # FPS显示（左侧图像顶部）
    cv2.putText(canvas, f"FPS: {fps:.1f}", (10, 20), 
               cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
    
    # 跟踪状态（右侧控制区）
    if tracker.locked:
        status_text = f"LOCK:{tracker.confidence:.2f}"
        if tracker.is_truly_locked():
            status_color = (0, 255, 0)  # 绿色表示稳定锁定
        else:
            status_color = (0, 255, 255)  # 黄色表示初始锁定
        cv2.putText(canvas, status_text, (SLIDER_X-75, SLIDER_Y - 25), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, status_color, 1)

def draw_detected_rectangles(canvas, detected_rects, tracker):
    """绘制检测到的矩形（在两张图上都显示）"""
    for r in detected_rects:
        center = tracker.calculate_actual_center(r)
        corners = r.corners()

        # 根据是否在搜索区域内决定颜色
        if tracker.locked and tracker.is_in_search_region(center):
            rect_color = (0, 255, 0)  # 绿色表示在搜索区域内
        else:
            rect_color = (255, 0, 0)  # 蓝色表示普通检测

        # 在左侧原始图上绘制
        # 绘制角点
        for corner in corners:
            cv2.circle(canvas, corner, 2, rect_color, -1)
        # 绘制中心点
        cv2.circle(canvas, center, 2, rect_color, -1)

        # 在中间处理图上绘制（坐标偏移PICTURE_WIDTH）
        # 绘制角点
        for corner in corners:
            cv2.circle(canvas, (corner[0] + PICTURE_WIDTH, corner[1]), 2, rect_color, -1)
        # 绘制中心点
        cv2.circle(canvas, (center[0] + PICTURE_WIDTH, center[1]), 2, rect_color, -1)

def draw_tracking_status(canvas, tracker):
    """绘制跟踪状态标记（在两张图上都显示）"""
    if not tracker.locked:
        return
        
    # 计算锁定框坐标
    rect_x = int(tracker.position[0] - tracker.size[0]//2)
    rect_y = int(tracker.position[1] - tracker.size[1]//2)

    # 根据锁定稳定性选择颜色
    if tracker.is_truly_locked():
        lock_color = (0, 255, 0)  # 绿色表示稳定锁定
        thickness = 3
    else:
        lock_color = (0, 255, 255)  # 黄色表示初始锁定
        thickness = 2

    # 在左侧原始图上绘制锁定框
    cv2.rectangle(canvas, 
                 (rect_x, rect_y), 
                 (rect_x + tracker.size[0], rect_y + tracker.size[1]), 
                 lock_color, thickness)
    # 绘制目标中心点
    cv2.circle(canvas, (int(tracker.position[0]), int(tracker.position[1])), 
              2, lock_color, -1)

    # 在中间处理图上绘制锁定框（坐标偏移PICTURE_WIDTH）
    cv2.rectangle(canvas, 
                 (rect_x + PICTURE_WIDTH, rect_y), 
                 (rect_x + tracker.size[0] + PICTURE_WIDTH, rect_y + tracker.size[1]), 
                 lock_color, thickness)
    # 绘制目标中心点
    cv2.circle(canvas, (int(tracker.position[0]) + PICTURE_WIDTH, int(tracker.position[1])), 
              2, lock_color, -1)

    # 绘制预测位置（两张图都显示）
    predicted_pos = tracker.predict()
    # 左侧原始图
    cv2.circle(canvas, (int(predicted_pos[0]), int(predicted_pos[1])), 
              2, (255, 0, 0), -1)
    # 中间处理图
    cv2.circle(canvas, (int(predicted_pos[0]) + PICTURE_WIDTH, int(predicted_pos[1])), 
              2, (255, 0, 0), -1)

    # 如果是嵌套矩形，绘制连接线（两张图都显示）
    if tracker.nested_rects and len(tracker.nested_rects) >= 2:
        center1 = tracker.calculate_actual_center(tracker.nested_rects[0])
        center2 = tracker.calculate_actual_center(tracker.nested_rects[1])

        # 左侧原始图绘制连接线
        cv2.line(canvas, center1, center2, (255, 0, 255), 1)
        # 绘制平均中心点
        avg_center = (
            (center1[0] + center2[0]) // 2,
            (center1[1] + center2[1]) // 2
        )
        cv2.circle(canvas, avg_center, 4, (255, 0, 255), -1)

        # 中间处理图绘制连接线（坐标偏移）
        cv2.line(canvas, 
                (center1[0] + PICTURE_WIDTH, center1[1]), 
                (center2[0] + PICTURE_WIDTH, center2[1]), 
                (255, 0, 255), 1)
        # 绘制平均中心点
        cv2.circle(canvas, (avg_center[0] + PICTURE_WIDTH, avg_center[1]), 4, (255, 0, 255), -1)

# ============== 触摸处理函数 ==============
def check_slider_touch(mouse_handler):
    """检查鼠标是否在滑块区域并更新阈值"""
    # 只有显示开启时才处理滑块触摸
    if not show_display:
        return False
        
    global current_threshold
    if not mouse_handler.pressed:
        return False

    touch_x, touch_y = mouse_handler.x, mouse_handler.y
    if (SLIDER_X - 10 <= touch_x <= SLIDER_X + SLIDER_WIDTH + 10 and
        SLIDER_Y <= touch_y <= SLIDER_Y + SLIDER_HEIGHT):
        ratio = 1 - ((touch_y - SLIDER_Y) / SLIDER_HEIGHT)
        new_threshold = int(THRESHOLD_MIN + ratio * (THRESHOLD_MAX - THRESHOLD_MIN))
        current_threshold = max(THRESHOLD_MIN, min(THRESHOLD_MAX, new_threshold))
        print(f"阈值更新为: {current_threshold}")
        return True
    return False

def send_uart_data(u1, tracker, fps):
    """发送UART数据"""
    if u1 is None or not tracker.locked:
        return
        
    tx, ty = tracker.position[0], tracker.position[1]
    aaa = f"{tx:03d}"  # 确保三位数
    bbb = f"{ty:03d}"  # 确保三位数
    send_data = f"\xAA{aaa},{bbb}\x55\x44\x33"
    try:
        u1.write(send_data.encode())
        print(f"发送数据: {aaa},{bbb}, FPS: {fps:.1f}")
    except Exception as e:
        print(f"串口发送失败: {e}")

def correct_color_channels(frame):
    """纠正红蓝通道顺序"""
    # 交换红和蓝通道
    b, g, r = cv2.split(frame)
    return cv2.merge([r, g, b])

# ============== 主程序 ==============
def main():
    global current_threshold, auto_threshold, show_display
    
    # 初始化硬件
    picam2 = init_camera()
    u1 = init_uart()
    init_display()
    
    if picam2 is None:
        print("摄像头初始化失败，程序退出")
        return
    
    # 初始化组件
    mouse_handler = MouseHandler()
    if show_display:  # 只有显示开启时才设置鼠标回调
        cv2.setMouseCallback(WINDOW_NAME, mouse_handler.callback)
    
    detector = RectangleDetector()
    tracker = TargetTracker()
    
    # 创建画布（左右并排布局）
    canvas = np.zeros((DISPLAY_HEIGHT, DISPLAY_WIDTH, 3), dtype=np.uint8)
    
    # FPS计算变量
    frame_count = 0
    start_time = time.time()
    fps = 0.0
    
    print("开始主循环...")
    print("按 'd' 键切换显示开启/关闭")
    
    try:
        while True:
            # 捕获图像
            frame = picam2.capture_array()
            frame = cv2.resize(frame, (PICTURE_WIDTH, PICTURE_HEIGHT))
            
            # 纠正红蓝通道
            frame = correct_color_channels(frame)
            
            # 转换为灰度图
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            
            # 检查滑块触摸（仅在手动模式和显示开启时有效）
            if not auto_threshold and show_display:
                check_slider_touch(mouse_handler)
            else:
                # 在自动模式下，点击滑块区域切换到手动模式（仅显示开启时）
                if show_display and mouse_handler.pressed:
                    touch_x, touch_y = mouse_handler.x, mouse_handler.y
                    if (SLIDER_X - 10 <= touch_x <= SLIDER_X + SLIDER_WIDTH + 10 and
                        SLIDER_Y - 80 <= touch_y <= SLIDER_Y + SLIDER_HEIGHT):
                        auto_threshold = False
                        print("切换到手动阈值模式")

            # 图像预处理
            gray_blur = cv2.GaussianBlur(gray, (7, 7), 0)
            
            # 检测矩形（使用自动或手动阈值）
            detected_rects, binary_img, used_threshold = detector.find_rectangles(
                gray_blur, current_threshold, auto_threshold)
            
            # 更新当前阈值（如果使用自动模式）
            if auto_threshold:
                current_threshold = used_threshold

            # 目标跟踪逻辑
            if tracker.locked:
                matched_rects = find_best_matches(detected_rects, tracker)

                if matched_rects and len(matched_rects) == 2:  # 只处理两个矩形的情况
                    tracker.update(matched_rects)
                    # 发送UART数据（无论显示是否开启都发送）
                    send_uart_data(u1, tracker, fps)
                else:
                    tracker.lost_count += 1
                    if tracker.lost_count > tracker.max_lost_frames:
                        tracker.reset()
                        print("目标丢失超过阈值，重置跟踪器")
            else:
                # 未锁定时，尝试锁定目标（只考虑两个矩形的情况）
                if len(detected_rects) >= 2:
                    # 选择最接近图像中心且面积较大的两个矩形
                    best_candidates = []
                    for r in detected_rects:
                        center = tracker.calculate_actual_center(r)
                        dist_to_center = math.sqrt(
                            (center[0] - CAMERA_CENTER[0])**2 +
                            (center[1] - CAMERA_CENTER[1])** 2
                        )
                        area_score = r.area()
                        combined_score = area_score / (1 + dist_to_center * 0.01)
                        best_candidates.append((r, combined_score))

                    # 按综合分数排序
                    best_candidates.sort(key=lambda x: x[1], reverse=True)

                    if len(best_candidates) >= 2:
                        best_rect = best_candidates[0][0]
                        second_rect = best_candidates[1][0]

                        # 检查中心点横纵误差
                        center1 = tracker.calculate_actual_center(best_rect)
                        center2 = tracker.calculate_actual_center(second_rect)
                        x_diff = abs(center1[0] - center2[0])
                        y_diff = abs(center1[1] - center2[1])

                        # 检查嵌套关系
                        r1 = best_rect.rect()
                        r2 = second_rect.rect()
                        is_nested = ((r1[0] < r2[0] and r1[1] < r2[1] and
                                     r1[0]+r1[2] > r2[0]+r2[2] and
                                     r1[1]+r1[3] > r2[1]+r2[3]) or
                                    (r2[0] < r1[0] and r2[1] < r1[1] and
                                     r2[0]+r2[2] > r1[0]+r1[2] and
                                     r2[1]+r2[3] > r1[1]+r1[3]))

                        # 只有满足所有条件才锁定
                        if is_nested and x_diff < CENTER_ERROR_THRESHOLD and y_diff < CENTER_ERROR_THRESHOLD:
                            tracker.update([best_rect, second_rect])
                            print(f"锁定符合条件的目标: 位置={tracker.position}, 尺寸={tracker.size}")

            # 计算FPS
            frame_count += 1
            if frame_count >= 10:
                end_time = time.time()
                fps = frame_count / (end_time - start_time)
                frame_count = 0
                start_time = time.time()
            
            # 绘制到画布（只有显示开启时才绘制）
            if show_display:
                canvas.fill(0)
                draw_to_canvas(canvas, frame, binary_img, tracker, detected_rects, 
                              used_threshold, auto_threshold, fps)

                # 显示画布
                cv2.imshow(WINDOW_NAME, canvas)
            
            # 按键处理
            # 当显示关闭时，使用cv2.waitKey(1)而不检查按键，避免程序卡住
            key = cv2.waitKey(1) & 0xFF if show_display else cv2.waitKey(1)
            
            if key == ord('q'):
                break
            elif key == ord('r'):
                tracker.reset()
                print("手动重置跟踪器")
            elif key == ord('t'):
                # 切换自动/手动阈值模式
                auto_threshold = not auto_threshold
                mode = "自动" if auto_threshold else "手动"
                print(f"切换到{mode}阈值模式，当前阈值: {current_threshold}")
            elif key == ord('s'):
                if show_display:  # 只有显示开启时才保存截图
                    cv2.imwrite("screenshot.jpg", canvas)
                    print("截图已保存为 screenshot.jpg")
                else:
                    print("显示已关闭，无法保存截图")
            elif key == ord('h'):
                print_help()
            elif key == ord('d'):
                # 切换显示开启/关闭
                show_display = not show_display
                status = "开启" if show_display else "关闭"
                print(f"本地显示已{status}")
                
                # 根据显示状态更新窗口和鼠标回调
                if show_display:
                    init_display()
                    cv2.setMouseCallback(WINDOW_NAME, mouse_handler.callback)
                else:
                    cv2.destroyWindow(WINDOW_NAME)

    except KeyboardInterrupt:
        print("用户终止程序")
    except Exception as e:
        print(f"程序异常: {e}")
        import traceback
        traceback.print_exc()
    finally:
        # 清理资源
        if picam2:
            picam2.stop()
        cv2.destroyAllWindows()
        if u1:
            u1.close()
        print("程序已退出")

def print_help():
    """打印帮助信息"""
    print("=== 控制说明 ===")
    print("q: 退出程序")
    print("r: 重置跟踪器")
    print("t: 切换自动/手动阈值模式")
    print("s: 保存截图")
    print("h: 显示帮助")
    print("d: 切换本地显示开启/关闭")
    print("鼠标: 点击右侧滑块调整阈值（手动模式下，显示开启时）")

if __name__ == "__main__":
    main()
