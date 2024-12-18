import serial
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# Bluetooth 포트 설정 (COM5 또는 COM6에 연결된 경우)
bluetooth_port = 'COM6'  # 또는 'COM5'
baud_rate = 115200
ser = serial.Serial(bluetooth_port, baud_rate)

# 앵커 0, 1, 2의 좌표 설정 (예시 좌표)
anchor_coords = {
    0: (0, 0),  # 앵커 0
    1: (0, 4.23),   # 앵커 1
    2: (7.04, 4.23)     # 앵커 2
}

# 실시간 플롯 설정
fig, ax = plt.subplots()
ax.set_xlim(-8, 8)  # x축 범위
ax.set_ylim(-8, 8)  # y축 범위

# 앵커 좌표를 빨간 점으로 표시
for anchor, (x, y) in anchor_coords.items():
    ax.plot(x, y, 'ro')  # 빨간색 점 (앵커)

# 움직이는 태그의 좌표를 표시할 점 (파란색 점)
point, = ax.plot([], [], 'bo')

# 데이터 업데이트 함수
def update(frame):
    # Bluetooth로부터 데이터 읽기
    if ser.in_waiting > 0:
        data = ser.readline().decode('utf-8').strip()
        print(f"Received data: {data}")  # 받은 데이터 확인
        
        if data.startswith("(x,y) = ("):
            # 좌표 데이터에서 불필요한 문자열 제거
            data = data[9:-1]  # "(x,y) = ("와 ")" 제거
            x, y = map(float, data.split(','))
            
            # 좌표 업데이트 (x, y는 리스트로 전달)
            point.set_data([x], [y])
    
    return point,


# 애니메이션 설정
ani = FuncAnimation(fig, update, frames=range(100), interval=100, blit=True)

plt.show()
