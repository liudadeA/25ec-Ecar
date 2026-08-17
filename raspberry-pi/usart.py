import serial
import time
import fcntl
import os

def send_data(frames, port='/dev/serial0', baudrate=115200, timeout=1.0):
    """
    通过串口发送带帧头帧尾的数据
    
    参数:
        frames: 待发送的数据列表,每个元素为bytes类型
        port: 串口设备路径
        baudrate: 波特率
        timeout: 单次发送的超时时间(秒)
    """
    HEADER = b'\xaa'  # 包头
    FOOTER = b'\x55\xAA\x33'  # 包尾
    
    try:
        # 打开串口
        with serial.Serial(port, baudrate, timeout=timeout) as ser:
            # 获取文件锁
            fcntl.flock(ser.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
            
            # 发送所有数据帧
            for data in frames:
                # 确保数据是bytes类型
                if isinstance(data, str):
                    data = data.encode('utf-8')
                
                # 构建带包头包尾的数据帧
                frame = HEADER + data + FOOTER
                
                # 发送数据并记录发送日志
                print(f"发送帧: {frame.hex()}")
                ser.write(frame)
                
                # 等待发送完成
                ser.flush()
                time.sleep(0.01)  # 帧间隔时间
                
            return True      
    except Exception as e:
        print(f"发送异常: {str(e)}")
        return False

# 使用示例
if __name__ == "__main__":
    # 准备要发送的数据
    data_to_send = [
        b"CMD:START",
        b"PARAM:123",
        b"END"
    ]
    
    # 发送数据
    success = send_data(data_to_send)
    
    if success:
        print("数据发送成功")
    else:
        print("数据发送失败")
