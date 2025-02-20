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
    1: (1.00, 0.00),   # 앵커 1
    2: (0.00, 1.00)     # 앵커 2
}

# 실시간 플롯 설정
fig, ax = plt.subplots()
ax.set_xlim(-3, 3)  # x축 범위
ax.set_ylim(-3, 3)  # y축 범위

# 앵커 좌표를 빨간 점으로 표시
for anchor, (x, y) in anchor_coords.items():
    ax.plot(x, y, 'ro', label=f"Anchor {anchor}")  # 빨간색 점 (앵커)

# 움직이는 태그의 좌표를 표시할 점 (파란색 점)
point, = ax.plot([], [], 'bo')

# 원을 저장할 딕셔너리
circles = {}
for anchor in anchor_coords:
    circle = plt.Circle(anchor_coords[anchor], 0, color='b', fill=False, linestyle='--')
    ax.add_artist(circle)
    circles[anchor] = circle

# 데이터 업데이트 함수
def update(frame):
    # Bluetooth로부터 데이터 읽기
    if ser.in_waiting > 0:
        data = ser.readline().decode('utf-8').strip()
        print(f"Received data: {data}")  # 받은 데이터 확인

        if data.startswith("a0"):
            # 거리 데이터 추출
            distances = {}
            for anchor in range(3):
                prefix = f"a{anchor} = "
                start = data.find(prefix)
                if start != -1:
                    end = data.find("\n", start)
                    distances[anchor] = float(data[start+len(prefix):end if end != -1 else None])
            
            # 거리 데이터를 기준으로 원 업데이트
            for anchor, radius in distances.items():
                circles[anchor].set_radius(radius)

        elif data.startswith("(x,y) = ("):
            # 태그 좌표 데이터 추출
            data = data[9:-1]  # "(x,y) = ("와 ")" 제거
            x, y = map(float, data.split(','))

            # 태그 위치 업데이트
            point.set_data([x], [y])

    return [point] + list(circles.values())

# 애니메이션 설정
ani = FuncAnimation(fig, update, frames=range(100), interval=100, blit=True)

plt.legend()
plt.show()
