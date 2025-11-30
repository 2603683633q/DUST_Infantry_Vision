#!/usr/bin/env python3
"""
简单的UDP数据接收器，用于接收sp_vision的plotter数据
"""
import socket
import json
import time
from datetime import datetime

def main():
    # 创建UDP socket
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(('127.0.0.1', 9870))

    print('UDP数据接收器已启动')
    print('监听端口: 127.0.0.1:9870')
    print('等待数据... (按Ctrl+C退出)')

    try:
        while True:
            data, addr = sock.recvfrom(4096)
            timestamp = datetime.now().strftime('%H:%M:%S.%f')[:-3]

            try:
                json_data = json.loads(data.decode('utf-8'))
                print(f'[{timestamp}] 收到JSON数据:')
                for key, value in json_data.items():
                    print(f'  {key}: {value}')
                print()
            except json.JSONDecodeError:
                print(f'[{timestamp}] 收到原始数据: {data}')
                print()

    except KeyboardInterrupt:
        print('\n停止接收数据')
    finally:
        sock.close()

if __name__ == '__main__':
    main()