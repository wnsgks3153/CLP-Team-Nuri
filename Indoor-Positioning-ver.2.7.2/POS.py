import serial
import math

# 시리얼 포트와 통신 속도 설정
PORT = 'COM5'  # COM6으로 변경 가능
BAUD_RATE = 9600

def parse_data(line):
    """
    수신한 데이터에서 a0, a1, a2, a3 값을 파싱하여 반환합니다.
    """
    data = {}
    try:
        # 데이터 형식 예: "a0 = 1.00\na1 = 1.55\na2 = 1.44\na3 = 0.00"
        for part in line.strip().split("\n"):
            key, value = part.split("=")
            data[key.strip()] = float(value.strip())
    except ValueError:
        print("데이터 파싱 오류:", line)
    return data

def calculate_coordinates(ax1, ay1, ar1, ax2, ay2, ar2, ax3, ay3, ar3):
    """
    주어진 변수들로 (tx, ty) 값을 계산합니다.
    """
    tA = 2 * ax2 - 2 * ax1
    tB = 2 * ay2 - 2 * ay1
    tC = (ar1**2) - (ar2**2) - (ax1**2) + (ax2**2) - (ay1**2) + (ay2**2)
    tD = 2 * ax3 - 2 * ax2
    tE = 2 * ay3 - 2 * ay2
    tF = (ar2**2) - (ar3**2) - (ax2**2) + (ax3**2) - (ay2**2) + (ay3**2)

    denominator_x = tE * tA - tB * tD
    denominator_y = tB * tD - tA * tE

    if abs(denominator_x) < 1e-6 or abs(denominator_y) < 1e-6:
        print("분모가 0이거나 너무 작습니다. 계산을 중단합니다.")
        return None

    tx = (tC * tE - tF * tB) / denominator_x
    ty = (tC * tD - tA * tF) / denominator_y

    return tx, ty

def main():
    # 초기 설정된 좌표와 거리
    ax1, ay1, ar1 = 0.0, 0.0, 1.0  # 첫 번째 앵커
    ax2, ay2, ar2 = 1.0, 1.0, 1.5  # 두 번째 앵커
    ax3, ay3, ar3 = 2.0, 0.0, 1.2  # 세 번째 앵커

    try:
        with serial.Serial(PORT, BAUD_RATE, timeout=1) as ser:
            print(f"포트 {PORT}에서 데이터 수신 대기 중...")
            while True:
                if ser.in_waiting > 0:
                    # 데이터 수신
                    line = ser.read_until(b'\n').decode('utf-8')
                    data = parse_data(line)

                    # 새로운 거리 데이터 업데이트
                    if data:
                        ar1 = data.get('a0', ar1)
                        ar2 = data.get('a1', ar2)
                        ar3 = data.get('a2', ar3)

                        # 계산 함수 호출
                        result = calculate_coordinates(ax1, ay1, ar1, ax2, ay2, ar2, ax3, ay3, ar3)
                        if result:
                            tx, ty = result
                            print(f"계산된 좌표: tx = {tx}, ty = {ty}")
    except serial.SerialException as e:
        print(f"시리얼 포트 오류: {e}")

if __name__ == "__main__":
    main()
