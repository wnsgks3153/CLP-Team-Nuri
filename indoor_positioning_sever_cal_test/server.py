import turtle
import bluetooth
import json

# Bluetooth 서버 설정
server_sock = bluetooth.BluetoothSocket(bluetooth.RFCOMM)  # 블루투스 소켓 생성
server_sock.bind(("", bluetooth.PORT_ANY))                # 모든 포트에서 수신
server_sock.listen(1)                                     # 연결 대기

print("Waiting for connection...")
client_sock, client_info = server_sock.accept()           # 클라이언트 연결 수락
print(f"Connected to {client_info}")

# Turtle 그래픽 초기화
screen = turtle.Screen()
screen.title("Real-Time Tag Location")                   # 창 제목
screen.setup(width=600, height=600)                      # 창 크기
screen.tracer(0)                                         # 실시간 업데이트 비활성화

# 태그와 앵커 표시
tag = turtle.Turtle()                                    # 태그 생성
tag.shape("circle")
tag.color("blue")
tag.penup()

anchors = [
    {"x": 0, "y": 0},
    {"x": 0, "y": 300},
    {"x": 400, "y": 300},
    {"x": 400, "y": 0},
]

# 앵커 위치 시각화
for anchor in anchors:
    anchor_t = turtle.Turtle()
    anchor_t.penup()
    anchor_t.goto(anchor["x"] - 200, anchor["y"] - 200)  # 창 중심 조정
    anchor_t.shape("square")
    anchor_t.color("red")

# 삼각 측량으로 태그 위치 계산
def calculate_position(distances):
    x1, y1, r1 = anchors[0]["x"], anchors[0]["y"], distances[0]
    x2, y2, r2 = anchors[1]["x"], anchors[1]["y"], distances[1]
    x3, y3, r3 = anchors[2]["x"], anchors[2]["y"], distances[2]

    A = 2 * (x2 - x1)
    B = 2 * (y2 - y1)
    C = r1**2 - r2**2 - x1**2 + x2**2 - y1**2 + y2**2
    D = 2 * (x3 - x2)
    E = 2 * (y3 - y2)
    F = r2**2 - r3**2 - x2**2 + x3**2 - y2**2 + y3**2

    denominator = E * A - B * D
    if denominator == 0:  # 분모가 0이면 계산 불가능
        return None, None

    x = (C * E - F * B) / denominator
    y = (C * D - A * F) / denominator
    return x, y

# 데이터 수신 및 처리
try:
    while True:
        data = client_sock.recv(1024).decode().strip()  # Bluetooth 데이터 수신
        try:
            distances = json.loads(data)  # JSON 파싱
            distances = [distances[f"Anchor{i}"] for i in range(3)]  # 거리 추출
        except json.JSONDecodeError:
            print("Invalid JSON data:", data)
            continue

        if len(distances) >= 3:  # 데이터가 충분할 때만 처리
            x, y = calculate_position(distances)
            if x is not None and y is not None:
                tag.goto(x - 200, y - 200)  # 창 중심 좌표로 조정
                screen.update()

except KeyboardInterrupt:
    print("Closing connection...")
    client_sock.close()
    server_sock.close()