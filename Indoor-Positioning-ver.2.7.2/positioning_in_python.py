import serial
import turtle
import math

# Bluetooth 포트 설정 (COM6에 연결된 경우)
bluetooth_port = 'COM6'  # 또는 사용 중인 포트
baud_rate = 115200
ser = serial.Serial(bluetooth_port, baud_rate)

# 앵커 0, 1, 2의 좌표 설정 (예시 좌표)
anchor_coords = {
    0: (0, 0),      # 앵커 0
    1: (0, 423),    # 앵커 1 (단위 변환: m -> cm)
    2: (704, 423)   # 앵커 2 (단위 변환: m -> cm)
}

# Turtle 초기화
turtle_screen = turtle.Screen()
turtle_screen.setup(width=800, height=800)
turtle_screen.setworldcoordinates(-1000, -1000, 1000, 1000)
turtle_screen.title("Real-Time Tag Position")

# 앵커 그리기
turtle.tracer(0)  # 애니메이션 업데이트 비활성화
anchor_turtles = []
for anchor, (x, y) in anchor_coords.items():
    anchor_turtle = turtle.Turtle()
    anchor_turtle.penup()
    anchor_turtle.goto(x, y)
    anchor_turtle.dot(10, "red")
    anchor_turtle.hideturtle()
    anchor_turtles.append(anchor_turtle)

# 태그 그리기
tag_turtle = turtle.Turtle()
tag_turtle.penup()
tag_turtle.shape("circle")
tag_turtle.color("blue")
tag_turtle.shapesize(0.5, 0.5)

# 거리 데이터를 저장할 딕셔너리
distances = {}

# 유효 범위 필터링 함수
def is_valid_coordinate(x, y):
    return -1000 <= x <= 1000 and -1000 <= y <= 1000

# 삼변측량을 이용한 좌표 계산 함수
def calculate_coordinates(a0, a1, a2):
    x0, y0 = anchor_coords[0]
    x1, y1 = anchor_coords[1]
    x2, y2 = anchor_coords[2]

    A = 2 * (x1 - x0)
    B = 2 * (y1 - y0)
    C = a0**2 - a1**2 - x0**2 + x1**2 - y0**2 + y1**2
    D = 2 * (x2 - x1)
    E = 2 * (y2 - y1)
    F = a1**2 - a2**2 - x1**2 + x2**2 - y1**2 + y2**2

    x = (C * E - F * B) / (E * A - B * D)
    y = (C * D - A * F) / (B * D - A * E)
    return x, y

# 실시간 업데이트 함수
try:
    while True:
        if ser.in_waiting > 0:
            try:
                # Bluetooth로부터 데이터 읽기
                data = ser.readline().decode('utf-8').strip()
                print(f"Received data: {data}")

                # (x, y) 좌표 데이터 처리
                if data.startswith("(x,y) = ("):
                    data = data[9:-1]  # "(x,y) = ("와 ")" 제거
                    x, y = map(float, data.split(','))

                    if is_valid_coordinate(x, y):
                        print(f"Parsed coordinates: x={x}, y={y}")
                        tag_turtle.goto(x, y)
                    else:
                        print(f"Invalid coordinates: x={x}, y={y}")

                # a0, a1, a2 거리 데이터 처리
                elif data.startswith("a"):
                    key, value = data.split(' = ')
                    distances[key] = float(value)

                    # 모든 거리 데이터(a0, a1, a2)가 수신되었을 때 좌표 계산
                    if len(distances) == 3:
                        a0 = distances['a0']
                        a1 = distances['a1']
                        a2 = distances['a2']

                        x, y = calculate_coordinates(a0, a1, a2)
                        if is_valid_coordinate(x, y):
                            print(f"Calculated coordinates: x={x}, y={y}")
                            tag_turtle.goto(x, y)
                        else:
                            print(f"Invalid calculated coordinates: x={x}, y={y}")

            except Exception as e:
                print(f"Error while processing data: {e}")

        turtle_screen.update()  # 화면 업데이트

except KeyboardInterrupt:
    print("Program terminated.")
    ser.close()
    turtle.bye()
