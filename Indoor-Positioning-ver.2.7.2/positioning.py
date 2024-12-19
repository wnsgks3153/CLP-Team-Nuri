import serial
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.lines import Line2D
from matplotlib.widgets import Button

# Bluetooth 포트 설정 (COM5 또는 COM6에 연결된 경우)
bluetooth_port = 'COM6'  # 또는 'COM5'
baud_rate = 115200
ser = serial.Serial(bluetooth_port, baud_rate)

# 앵커 0, 1, 2의 좌표 설정 (예시 좌표)
anchor_coords = {
    0: (0, 0),  # 앵커 0
    1: (1.0, 0),  # 앵커 1
    2: (0.0, 1.0)  # 앵커 2
}

# 각 앵커의 색상 설정
anchor_colors = {
    0: 'red',    # 앵커 0: 빨간색
    1: 'green',  # 앵커 1: 초록색
    2: 'blue'    # 앵커 2: 파란색
}

# 실시간 플롯 설정
fig, ax = plt.subplots()
ax.set_xlim(-6, 6)  # x축 범위
ax.set_ylim(-3, 3)  # y축 범위

# x축, y축 격자 간격을 1로 설정
ax.set_xticks(range(-12, 13, 1))  # x축 눈금 간격 1
ax.set_yticks(range(-6, 7, 1))    # y축 눈금 간격 1

# 배경에 격자 레이어 추가
ax.grid(color='gray', linestyle='--', alpha=0.7)

# 앵커 좌표를 해당 색상으로 표시하면서 설명 텍스트 추가
for anchor, (x, y) in anchor_coords.items():
    ax.plot(x, y, 'o', color=anchor_colors[anchor], label=f"Anchor {anchor}")
    ax.text(x, y, f'Anchor {anchor}', color=anchor_colors[anchor], fontsize=12, ha='right')

# 태그 좌표에 대한 설명을 추가
point, = ax.plot([], [], 'ko', label='Tag')  # 'ko'로 검은색 원형 점 설정, 태그 설명 추가
# 태그에 대한 텍스트 라벨 추가 (초기 위치에서 텍스트는 보이지 않음)
tag_label = ax.text(0, 0, 'Tag', color='black', fontsize=12, ha='right')

# 앵커와 태그를 연결하는 선을 저장할 딕셔너리
lines = {}
for anchor, (x, y) in anchor_coords.items():
    line = Line2D([x, x], [y, y], color=anchor_colors[anchor], linestyle='--')  # 초기 선 생성
    ax.add_line(line)
    lines[anchor] = line

# 원을 저장할 딕셔너리
circles = {}
for anchor in anchor_coords:
    circle = plt.Circle(anchor_coords[anchor], 0, color=anchor_colors[anchor], fill=False, linestyle='--')
    ax.add_artist(circle)
    circles[anchor] = circle

# 기능 상태 (직선과 원이 보이는지 여부)
show_lines = True
show_circles = True

# 직선 토글 함수
def toggle_lines(event):
    global show_lines
    show_lines = not show_lines
    for line in lines.values():
        line.set_visible(show_lines)  # 직선의 가시성 변경
    fig.canvas.draw()

# 원 토글 함수
def toggle_circles(event):
    global show_circles
    show_circles = not show_circles
    for circle in circles.values():
        circle.set_visible(show_circles)  # 원의 가시성 변경
    fig.canvas.draw()

# 버튼 추가 (오른쪽 하단으로 위치 변경하고, 크기를 줄임)
ax_line_button = plt.axes([0.75, 0.05, 0.15, 0.075])  # 직선 토글 버튼 (y값을 더 아래로 이동)
button_line = Button(ax_line_button, 'Toggle Lines')
button_line.on_clicked(toggle_lines)

ax_circle_button = plt.axes([0.75, 0.15, 0.15, 0.075])  # 원 토글 버튼 (y값을 더 아래로 이동)
button_circle = Button(ax_circle_button, 'Toggle Circles')
button_circle.on_clicked(toggle_circles)

# 데이터 업데이트 함수
def update(frame):
    # Bluetooth로부터 데이터 읽기
    if ser.in_waiting > 0:
        data = ser.readline().decode('utf-8').strip()
        print(f"Received data: {data}")  # 받은 데이터 확인

        if data.startswith("a0") or data.startswith("a1") or data.startswith("a2"):
            # 거리 데이터 추출
            distances = {}
            for anchor in range(3):  # a0, a1, a2
                prefix = f"a{anchor} = "
                start = data.find(prefix)
                if start != -1:
                    end = data.find("\n", start)
                    try:
                        distances[anchor] = float(data[start + len(prefix):end if end != -1 else None])
                    except ValueError:
                        pass

            # 거리 데이터를 기준으로 원 업데이트
            for anchor, radius in distances.items():
                if radius > 0:  # 유효한 거리 데이터만 반영
                    circles[anchor].set_radius(radius)
                else:
                    circles[anchor].set_radius(0)  # 반지름 0으로 설정

        elif data.startswith("(x,y) = ("):
            # 태그 좌표 데이터 추출
            data = data[9:-1]  # "(x,y) = ("와 ")" 제거
            x, y = map(float, data.split(','))

            # 태그 위치 업데이트
            point.set_data([x], [y])

            # 태그에 대한 텍스트 라벨 업데이트
            tag_label.set_position((x, y))
            tag_label.set_text(f'Tag ({x:.2f}, {y:.2f})')  # 태그 좌표 텍스트 추가

            # 앵커와 태그를 연결하는 선 업데이트
            for anchor, (anchor_x, anchor_y) in anchor_coords.items():
                lines[anchor].set_data([anchor_x, x], [anchor_y, y])

    return [point, tag_label] + list(circles.values()) + list(lines.values())

# 애니메이션 설정
ani = FuncAnimation(fig, update, frames=range(100), interval=100, blit=True)

# 범례 표시
plt.legend()
plt.show()
