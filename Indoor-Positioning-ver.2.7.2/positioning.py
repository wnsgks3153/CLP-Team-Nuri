import serial
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# 설정: 블루투스 포트와 통신 속도
PORT = 'COM5'  # 또는 'COM6'
BAUD_RATE = 115200

def parse_data(data):
    """
    수신 데이터를 파싱하여 거리와 좌표 정보를 반환합니다.
    데이터 형식 예제:
    "a0 = 1.03\na1 = 0.86\na2 = 1.03\na3 = 1.03\n(x,y) = (0.52, 0.43)"
    """
    try:
        lines = data.strip().split('\n')
        parsed = {}
        for line in lines:
            if line.startswith('a'):
                key, val = line.split('=')
                parsed[key.strip()] = float(val.strip())
            elif line.startswith('(x,y)'):
                coords = line.split('=')[1].strip().strip('()').split(',')
                parsed['X'] = float(coords[0])
                parsed['Y'] = float(coords[1])
        return parsed
    except Exception as e:
        print(f"데이터 파싱 오류: {e}")
        return None

def update(frame):
    """실시간 데이터로 그래프 업데이트"""
    global serial_connection
    try:
        if serial_connection.in_waiting:
            raw_data = serial_connection.read_until(b'\n(x,y)').decode('utf-8')
            parsed_data = parse_data(raw_data)

            if parsed_data:
                # 앵커와 현재 좌표 업데이트
                current_position.set_data(parsed_data['X'], parsed_data['Y'])

                # 그래프 제목 업데이트
                ax.set_title(f"X: {parsed_data['X']:.2f}, Y: {parsed_data['Y']:.2f}")
                fig.canvas.draw()
    except Exception as e:
        print(f"업데이트 오류: {e}")

# 시리얼 포트 연결
serial_connection = serial.Serial(PORT, BAUD_RATE, timeout=1)

# 앵커 위치 설정
anchor_positions = [(0, 0), (1, 0), (1, 1), (0, 1)]  # 예시 좌표 (a0, a1, a2, a3)

# Matplotlib 초기화
fig, ax = plt.subplots()
ax.set_xlim(-0.5, 1.5)  # X축 범위
ax.set_ylim(-0.5, 1.5)  # Y축 범위
ax.set_xlabel("X")
ax.set_ylabel("Y")
ax.grid(True)

# 앵커를 그래프에 표시
for idx, (x, y) in enumerate(anchor_positions):
    ax.scatter(x, y, c='red', label=f"Anchor a{idx}")

# 현재 위치를 표시할 객체 생성
current_position, = ax.plot([], [], 'bo', label="Current Position")
ax.legend()

# 실시간 업데이트 설정
ani = FuncAnimation(fig, update, interval=100)

# 그래프 출력
plt.show()

# 종료 시 연결 닫기
serial_connection.close()