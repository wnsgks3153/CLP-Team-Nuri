#include "dw3000.h"              // DW3000 UWB 모듈 제어 라이브러리
#include "BluetoothSerial.h"     // ESP32 Bluetooth 통신 라이브러리

// 핀 설정
#define PIN_RST 27               // Reset 핀
#define PIN_IRQ 34               // Interrupt 핀
#define PIN_SS 4                 // SPI Chip Select 핀

// UWB 설정
#define RNG_DELAY_MS 500          // 거리 계산 지연(ms)
#define TX_ANT_DLY 16385         // 송신 안테나 지연
#define RX_ANT_DLY 16385         // 수신 안테나 지연
#define POLL_TX_TO_RESP_RX_DLY_UUS 240  // 송신-수신 지연
#define RESP_RX_TIMEOUT_UUS 400  // 응답 수신 타임아웃

BluetoothSerial SerialBT;        // Bluetooth 객체 생성

// 송신 및 수신 메시지 정의
static uint8_t tx_poll_msg_t0[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'T', 'A', 'G', '0', 0xE0, 0, 0};
static uint8_t rx_resp_msg_a0[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'A', 'N', 'C', '0', 0xE1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

// 거리 계산 결과 저장 변수
float distances[4] = {0.0, 0.0, 0.0, 0.0};

void setup() {
  SerialBT.begin("ESP32_BT_Module");  // 블루투스 모듈 초기화
  Serial.begin(115200);              // 디버깅용 시리얼 통신

  spiBegin(PIN_IRQ, PIN_RST);        // SPI 초기화
  spiSelect(PIN_SS);                 // SPI 디바이스 선택

  // DW3000 초기화
  if (dwt_initialise(DWT_DW_INIT) == DWT_ERROR) {
    while (1);  // 초기화 실패 시 멈춤
  }

  // 안테나 및 수신 타임아웃 설정
  dwt_setrxantennadelay(RX_ANT_DLY);
  dwt_settxantennadelay(TX_ANT_DLY);
  dwt_setrxaftertxdelay(POLL_TX_TO_RESP_RX_DLY_UUS);
  dwt_setrxtimeout(RESP_RX_TIMEOUT_UUS);
}

void loop() {
  // 각 앵커와 거리 측정
  for (int i = 0; i < 4; i++) {
    distances[i] = measure_distance(tx_poll_msg_t0, rx_resp_msg_a0 + i * sizeof(rx_resp_msg_a0));
    delay(RNG_DELAY_MS);
  }

  // 거리 데이터 JSON 형식으로 전송
  send_json(distances, 4);
  delay(1000);  // 1초 간격으로 전송
}

// 거리 계산 함수
float measure_distance(uint8_t *tx_msg, uint8_t *rx_msg) {
  tx_msg[2] = frame_seq_nb++;  // 메시지 시퀀스 번호 증가
  dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS_BIT_MASK);  // 상태 레지스터 초기화
  dwt_writetxdata(sizeof(tx_poll_msg_t0), tx_msg, 0);          // 송신 데이터 설정
  dwt_writetxfctrl(sizeof(tx_poll_msg_t0), 0, 1);             // 송신 제어 설정
  dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED); // 송신 시작

  // 수신 완료 또는 오류 대기
  while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG_BIT_MASK | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR))) {
  }

  // 수신 성공
  if (status_reg & SYS_STATUS_RXFCG_BIT_MASK) {
    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG_BIT_MASK); // 상태 초기화
    dwt_readrxdata(rx_buffer, sizeof(rx_buffer), 0);             // 수신 데이터 읽기

    // 수신 메시지 검증
    if (memcmp(rx_buffer, rx_msg, sizeof(rx_resp_msg_a0)) == 0) {
      uint32_t poll_tx_ts = dwt_readtxtimestamplo32();  // 송신 타임스탬프
      uint32_t resp_rx_ts = dwt_readrxtimestamplo32();  // 수신 타임스탬프
      tof = (resp_rx_ts - poll_tx_ts) * DWT_TIME_UNITS; // 왕복 시간 계산
      return tof * SPEED_OF_LIGHT;                     // 거리 반환
    }
  }
  return -1.0;  // 실패 시 -1 반환
}

// JSON 데이터 전송 함수
void send_json(float *distances, int num_anchors) {
  SerialBT.print("{");
  for (int i = 0; i < num_anchors; i++) {
    SerialBT.print("\"Anchor");
    SerialBT.print(i);
    SerialBT.print("\": ");
    SerialBT.print(distances[i]);
    if (i < num_anchors - 1) SerialBT.print(", ");
  }
  SerialBT.println("}");
}