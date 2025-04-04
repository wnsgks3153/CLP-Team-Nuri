#include <vector>
#include <algorithm>
#include <cmath>
#include "dw3000.h"

#define APP_NAME "Multi-Floor UWB Tag v1.0"

// 연결 핀 정의
const uint8_t PIN_RST = 27; // 리셋 핀
const uint8_t PIN_IRQ = 34; // IRQ 핀
const uint8_t PIN_SS = 4;   // SPI 선택 핀

/* 기본 통신 설정. 기본 non-STS DW 모드를 사용합니다. */
static dwt_config_t config = {
        5,               /* 채널 번호. */
        DWT_PLEN_128,    /* 프리앰블 길이. TX에서만 사용됨. */
        DWT_PAC8,        /* 프리앰블 수집 청크 크기. RX에서만 사용됨. */
        9,               /* TX 프리앰블 코드. TX에서만 사용됨. */
        9,               /* RX 프리앰블 코드. RX에서만 사용됨. */
        1,               /* 0: 표준 8 심볼 SFD 사용, 1: 비표준 8 심볼 사용, 2: 비표준 16 심볼 SFD 사용, 3: 4z 8 심볼 SDF 타입 사용 */
        DWT_BR_6M8,      /* 데이터 속도. */
        DWT_PHRMODE_STD, /* PHY 헤더 모드. */
        DWT_PHRRATE_STD, /* PHY 헤더 속도. */
        (129 + 8 - 8),   /* SFD 타임아웃 (프리앰블 길이 + 1 + SFD 길이 - PAC 크기). RX에서만 사용됨. */
        DWT_STS_MODE_OFF, /* STS 비활성화 */
        DWT_STS_LEN_64,/* STS 길이 (Enum dwt_sts_lengths_e에서 허용된 값 참조) */
        DWT_PDOA_M0      /* PDOA 모드 비활성화 */
};

/* 거리 측정 사이클 간 지연 시간 (밀리초). */
#define CYCLE_DELAY_MS 500 // 모든 앵커 측정 후 지연
#define INTRA_RANGING_DELAY_MS 50 // 다른 앵커 측정 사이의 짧은 지연

/* 64 MHz PRF에 대한 기본 안테나 지연 값. */
#define TX_ANT_DLY 16385
#define RX_ANT_DLY 16385

/* 소스 주소 (바이트 7-8) - 이 Initiator의 고유 ID 설정 */
#define INITIATOR_ID_BYTE_7 0xDE 
#define INITIATOR_ID_BYTE_8 0x11 

/* 거리 측정 프로세스에서 사용되는 프레임. */
static uint8_t tx_poll_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 0x00, 0x00, INITIATOR_ID_BYTE_7, INITIATOR_ID_BYTE_8, 0xE0, 0, 0}; // 목적지 주소는 각 앵커마다 업데이트됨
static uint8_t rx_resp_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, INITIATOR_ID_BYTE_7, INITIATOR_ID_BYTE_8, 0x00, 0x00, 0xE1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}; 

/* 메시지의 공통 부분 길이 (기능 코드까지 포함). */
#define ALL_MSG_COMMON_LEN 10
/* 위에서 정의된 프레임의 일부 필드에 접근하기 위한 인덱스. */
#define ALL_MSG_SN_IDX 2 // 시퀀스 번호 인덱스
#define ALL_MSG_DEST_ADDR_IDX 5 // 목적지 주소 인덱스
#define ALL_MSG_SRC_ADDR_IDX 7 // 소스 주소 인덱스
#define RESP_MSG_POLL_RX_TS_IDX 10 // 응답 메시지 내 Poll 수신 타임스탬프 인덱스
#define RESP_MSG_RESP_TX_TS_IDX 14 // 응답 메시지 내 응답 송신 타임스탬프 인덱스
#define RESP_MSG_TS_LEN 4 // 타임스탬프 길이 (바이트)
/* 프레임 시퀀스 번호, 각 전송 후 증가됨. */
static uint8_t frame_seq_nb = 0;

/* 수신된 응답 메시지를 저장할 버퍼. */
#define RX_BUF_LEN 20
static uint8_t rx_buffer[RX_BUF_LEN];

/* 디버그 중단점에서 검사할 수 있도록 상태 레지스터 상태의 복사본을 여기에 보관. */
static uint32_t status_reg = 0;

/* 프레임 간 지연 시간 (UWB 마이크로초). */
#define POLL_TX_TO_RESP_RX_DLY_UUS 240
/* 응답 수신 타임아웃. */
#define RESP_RX_TIMEOUT_UUS 400

/* --- 다층 삼각측량 구조체 --- */
// 3차원 좌표를 나타내는 구조체
struct Point {
    float x;
    float y;
    float z;
};

// 앵커 데이터를 저장하는 구조체
struct AnchorData {
  uint16_t id;       // 16비트 앵커 ID
  Point pos;         // 앵커 위치 (x, y, z)
  double distance;   // 마지막으로 측정된 거리
  bool distance_valid; // 마지막 측정이 성공적이었는지 여부
  uint64_t last_heard_ts; // 마지막 성공적인 거리 측정 타임스탬프
  uint8_t floor;     // 앵커가 설치된 층 (0=1층, 1=2층, 2=3층...)
};

// 층별 앵커 ID 범위
// 1층 앵커: 0xA001~0xA999 (예: 0xA001, 0xA002, ...)
// 2층 앵커: 0xB001~0xB999 (예: 0xB001, 0xB002, ...)
// 3층 앵커: 0xC001~0xC999 (예: 0xC001, 0xC002, ...)

// 앵커 정의
#define NUM_ANCHORS 10 // 앵커 총 개수
AnchorData anchors[NUM_ANCHORS] = {
  // 1층 앵커 (ID는 0xA로 시작)
  {0xA001, {0.0, 0.0, 1.5}, -1.0, false, 0, 0},     // 앵커 1 (1층)
  {0xA002, {5.0, 0.0, 1.5}, -1.0, false, 0, 0},     // 앵커 2 (1층)
  {0xA003, {0.0, 5.0, 1.5}, -1.0, false, 0, 0},     // 앵커 3 (1층)
  {0xA004, {5.0, 5.0, 1.5}, -1.0, false, 0, 0},     // 앵커 4 (1층)

  // 2층 앵커 (ID는 0xB로 시작)
  {0xB001, {0.0, 0.0, 4.5}, -1.0, false, 0, 1},     // 앵커 5 (2층)
  {0xB002, {5.0, 0.0, 4.5}, -1.0, false, 0, 1},     // 앵커 6 (2층)
  {0xB003, {0.0, 5.0, 4.5}, -1.0, false, 0, 1},     // 앵커 7 (2층)

  // 3층 앵커 (ID는 0xC로 시작)
  {0xC001, {0.0, 0.0, 7.5}, -1.0, false, 0, 2},     // 앵커 8 (3층)
  {0xC002, {5.0, 0.0, 7.5}, -1.0, false, 0, 2},     // 앵커 9 (3층)
  {0xC003, {0.0, 5.0, 7.5}, -1.0, false, 0, 2}      // 앵커 10 (3층)
};

// 계산된 위치 결과를 저장하는 구조체
Point current_position = {0.0, 0.0, 0.0};
bool position_valid = false; // 위치 계산 성공 여부

// 층 결정 변수
uint8_t current_floor = 0; // 현재 결정된 층 (0=1층, 1=2층, 2=3층...)
uint8_t floor_change_count = 0; // 층 변경 안정화 카운터
uint8_t floor_vote_threshold = 3; // 층 변경을 위한 카운터 임계값

/* PG_DELAY 및 TX_POWER 레지스터 값 */
extern dwt_txconfig_t txconfig_options;

// --- 함수 프로토타입 ---
void findNearestThreeAnchors(AnchorData* nearest[3]); // 가장 가까운 앵커 3개 찾기
bool calculatePosition(AnchorData* a1, AnchorData* a2, AnchorData* a3, Point& result_pos); // 위치 계산 (삼각측량)
void determineFloor(); // 현재 층 결정
void serialPrintPosition(); // 위치 정보 출력 함수

// 라이브러리에서 제공하는 함수
extern void resp_msg_get_ts(uint8_t *ts_field, uint32_t *ts); // 응답 메시지에서 타임스탬프 추출

// 플랫폼 특정 함수
extern void test_run_info(unsigned char *s); // 정보 출력 함수
extern void spiBegin(uint8_t pin_irq, uint8_t pin_rst);
extern void spiSelect(uint8_t pin_ss);


void setup() {
  Serial.begin(115200);
  test_run_info((unsigned char *)APP_NAME); // 앱 이름 출력

  /* SPI 속도 설정, DW3000은 최대 38 MHz 지원 */
  /* DW IC 리셋 */
  spiBegin(PIN_IRQ, PIN_RST);
  spiSelect(PIN_SS);

  delay(2); // DW3000 시작에 필요한 시간

  while (!dwt_checkidlerc()) // 진행하기 전에 DW IC가 IDLE_RC 상태인지 확인 필요
  {
    test_run_info((unsigned char *)"IDLE 실패 - 중단됨");
    while (1) ; // 중단
  }

  if (dwt_initialise(DWT_DW_INIT) == DWT_ERROR) // DW IC 초기화
  {
    test_run_info((unsigned char *)"INIT 실패 - 중단됨");
    while (1) ; // 중단
  }

  // 디버깅을 위해 LED 활성화
  dwt_setleds(DWT_LEDS_ENABLE | DWT_LEDS_INIT_BLINK);

  /* DW IC 설정 */
  if(dwt_configure(&config)) // dwt_configure가 DWT_ERROR를 반환하면 PLL 또는 RX 보정 실패
  {
    test_run_info((unsigned char *)"CONFIG 실패 - 중단됨");
    while (1) ; // 중단
  }

  /* TX 스펙트럼 파라미터 설정 (전력, PG 지연, PG 카운트) */
  dwt_configuretxrf(&txconfig_options);

  /* 기본 안테나 지연 값 적용 */
  dwt_setrxantennadelay(RX_ANT_DLY);
  dwt_settxantennadelay(TX_ANT_DLY);

  /* 예상 응답의 지연 및 타임아웃 설정 */
  dwt_setrxaftertxdelay(POLL_TX_TO_RESP_RX_DLY_UUS);
  dwt_setrxtimeout(RESP_RX_TIMEOUT_UUS);

  /* TX/RX LED 활성화 */
  dwt_setlnapamode(DWT_LNA_ENABLE | DWT_PA_ENABLE); // LNA/PA 활성화

  test_run_info((unsigned char *)"설정 완료. 거리 측정 루프 시작.");
}

void loop() {
  // --- 거리 측정 사이클 ---
  test_run_info((unsigned char *)"거리 측정 사이클 시작...");
  int valid_ranges_count = 0; // 유효한 거리 측정 횟수 카운트

  for (int i = 0; i < NUM_ANCHORS; i++) { // 모든 앵커에 대해 반복
    // 거리 측정 전 현재 앵커의 유효성 초기화
    anchors[i].distance_valid = false;
    anchors[i].distance = -1.0;

    // Poll 메시지의 목적지 주소 설정
    tx_poll_msg[ALL_MSG_DEST_ADDR_IDX] = anchors[i].id & 0xFF; // 하위 바이트
    tx_poll_msg[ALL_MSG_DEST_ADDR_IDX + 1] = (anchors[i].id >> 8) & 0xFF; // 상위 바이트

    /* 프레임 데이터를 DW IC에 쓰고 전송 준비 */
    tx_poll_msg[ALL_MSG_SN_IDX] = frame_seq_nb; // 시퀀스 번호 설정
    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS_BIT_MASK); // TX 상태 비트 클리어
    dwt_writetxdata(sizeof(tx_poll_msg), tx_poll_msg, 0); /* TX 버퍼 오프셋 0. */
    dwt_writetxfctrl(sizeof(tx_poll_msg), 0, 1); /* TX 버퍼 오프셋 0, 거리 측정 비트 설정. */

    /* 전송 시작 */
    dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED);

    /* 전송이 올바르게 완료되었다고 가정하고, 프레임 수신 또는 오류/타임아웃 폴링 */
    while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG_BIT_MASK | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR)))
    { }; // 상태 비트가 설정될 때까지 대기

    /* 전송 시도 후 프레임 시퀀스 번호 증가 (성공 여부와 관계없이). */
    frame_seq_nb++;

    if (status_reg & SYS_STATUS_RXFCG_BIT_MASK) // 좋은 프레임 수신 성공
    {
      uint32_t frame_len; // 수신된 프레임 길이

      /* DW IC 상태 레지스터에서 좋은 RX 프레임 이벤트 클리어. */
      dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG_BIT_MASK);

      /* 프레임이 수신되었으므로 로컬 버퍼로 읽어들임. */
      frame_len = dwt_read32bitreg(RX_FINFO_ID) & RXFLEN_MASK; // 프레임 정보 레지스터에서 길이 읽기
      if (frame_len <= sizeof(rx_buffer)) // 버퍼 크기 확인
      {
        dwt_readrxdata(rx_buffer, frame_len, 0); // RX 데이터 읽기

        // --- 응답 유효성 검사 ---
        // 1. 기능 코드 확인 (응답은 0xE1이어야 함)
        // 2. 목적지 주소 확인 (Initiator ID와 일치해야 함)
        // 3. 소스 주소 확인 (폴링한 앵커 ID와 일치해야 함)
        bool correct_response = (rx_buffer[9] == 0xE1) &&
                                (rx_buffer[ALL_MSG_DEST_ADDR_IDX] == INITIATOR_ID_BYTE_7) &&
                                (rx_buffer[ALL_MSG_DEST_ADDR_IDX + 1] == INITIATOR_ID_BYTE_8) &&
                                (rx_buffer[ALL_MSG_SRC_ADDR_IDX] == (anchors[i].id & 0xFF)) &&
                                (rx_buffer[ALL_MSG_SRC_ADDR_IDX + 1] == ((anchors[i].id >> 8) & 0xFF));


        if (correct_response) // 응답이 유효하면 거리 계산 수행
        {
          uint32_t poll_tx_ts, resp_rx_ts, poll_rx_ts, resp_tx_ts; // 타임스탬프 변수
          int32_t rtd_init, rtd_resp; // Round-trip delay 변수
          float clockOffsetRatio; // 클럭 오프셋 비율
          double tof_calc; // 계산된 Time-of-flight
          double distance_calc; // 계산된 거리

          /* Poll 전송 및 응답 수신 타임스탬프 검색 */
          poll_tx_ts = dwt_readtxtimestamplo32(); // Poll TX 타임스탬프 (하위 32비트)
          resp_rx_ts = dwt_readrxtimestamplo32(); // 응답 RX 타임스탬프 (하위 32비트)

          /* 반송파 적분기 값 읽고 클럭 오프셋 비율 계산 */
          clockOffsetRatio = ((float)dwt_readclockoffset()) / (uint32_t)(1<<26);

          /* 응답 메시지에 포함된 타임스탬프 가져오기. */
          resp_msg_get_ts(&rx_buffer[RESP_MSG_POLL_RX_TS_IDX], &poll_rx_ts); // Responder의 Poll RX 타임스탬프
          resp_msg_get_ts(&rx_buffer[RESP_MSG_RESP_TX_TS_IDX], &resp_tx_ts); // Responder의 응답 TX 타임스탬프

          /* 거리 계산 */
          rtd_init = resp_rx_ts - poll_tx_ts; // Initiator에서의 Round-trip 시간
          rtd_resp = resp_tx_ts - poll_rx_ts; // Responder에서의 Round-trip 시간

          tof_calc = ((rtd_init - rtd_resp * (1.0 - clockOffsetRatio)) / 2.0) * DWT_TIME_UNITS; // ToF 계산
          distance_calc = tof_calc * SPEED_OF_LIGHT; // 거리 계산 (ToF * 빛의 속도)

          // 현재 앵커에 대한 결과 저장
          anchors[i].distance = distance_calc;
          anchors[i].distance_valid = true;
          anchors[i].last_heard_ts = dwt_readsystimestamphi32(); // 시스템 타임스탬프 상위 비트 저장 (참고용)
          valid_ranges_count++; // 유효 측정 카운트 증가

          // 계산된 거리 출력
          char dist_str[50];
          snprintf(dist_str, sizeof(dist_str), "앵커 0x%04X (층:%d): %.2f m", 
                  anchors[i].id, anchors[i].floor + 1, anchors[i].distance);
          test_run_info((unsigned char *)dist_str);

        } else {
          // 프레임은 수신했지만 올바른 응답이 아님 (잘못된 ID, 타입 등)
          char err_str[40];
          snprintf(err_str, sizeof(err_str), "앵커 0x%04X: 잘못된 응답 RX", anchors[i].id);
          test_run_info((unsigned char *)err_str);
          // distance_valid는 false로 유지
        }
      } else {
        // 프레임은 수신했지만 버퍼에 비해 너무 김
        char err_str[40];
        snprintf(err_str, sizeof(err_str), "앵커 0x%04X: 프레임 길이 오류 (%lu)", 
                anchors[i].id, (unsigned long)frame_len);
        test_run_info((unsigned char *)err_str);
      }
    } else { // 프레임 수신 실패 (타임아웃 또는 오류)
      /* DW IC 상태 레지스터에서 RX 오류/타임아웃 이벤트 클리어. */
      dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);
      
      char err_str[40];
      snprintf(err_str, sizeof(err_str), "앵커 0x%04X: RX 실패/타임아웃", anchors[i].id);
      test_run_info((unsigned char *)err_str);
      // distance_valid는 false로 유지
    }

    /* 다른 앵커와의 거리 측정 시도 사이에 짧은 지연 */
    if (i < NUM_ANCHORS - 1) {
       delay(INTRA_RANGING_DELAY_MS);
    }
  } // 모든 앵커에 대한 거리 측정 루프 종료

  // --- 위치 계산 ---
  if (valid_ranges_count >= 3) { // 최소 3개의 유효한 거리가 있어야 삼각측량 가능
    AnchorData* nearest[3] = {nullptr, nullptr, nullptr}; // 가장 가까운 앵커 3개를 저장할 포인터 배열
    findNearestThreeAnchors(nearest); // 가장 가까운 3개 앵커 찾기

    if (nearest[0] && nearest[1] && nearest[2]) { // 3개의 앵커를 성공적으로 찾았다면
      test_run_info((unsigned char *)"가장 가까운 앵커 3개 발견. 위치 계산 중...");
      position_valid = calculatePosition(nearest[0], nearest[1], nearest[2], current_position); // 위치 계산

      if (position_valid) { // 위치 계산 성공
        // 층 결정 알고리즘 실행
        determineFloor();
        
        // 위치 정보 출력 (시리얼)
        serialPrintPosition();
      } else { // 위치 계산 실패 (기하학적/수학적 오류)
        test_run_info((unsigned char *)"삼각측량 실패 (기하학적/수학적 오류).");
      }
    } else {
      // 이 경우는 valid_ranges_count >= 3 이면 발생하지 않아야 하지만, 안전을 위해 확인
      test_run_info((unsigned char *)"오류: 유효 범위 >=3개지만 가장 가까운 3개 선택 불가.");
      position_valid = false;
    }
  } else { // 유효한 거리가 3개 미만인 경우
    char err_str[60];
    snprintf(err_str, sizeof(err_str), "삼각측량 위해 유효 범위 >=3개 필요, %d개 발견.", valid_ranges_count);
    test_run_info((unsigned char *)err_str);
    position_valid = false;
  }

  /* 다음 전체 거리 측정 사이클 전에 지연 실행. */
  delay(CYCLE_DELAY_MS);
}


/*
 * @brief 가장 적합한 앵커 3개를 찾습니다.
 *        현재 층의 앵커를 우선 선택하고, 부족한 경우 다른 층의 앵커를 사용합니다.
 *
 * @param nearest - 선택된 앵커를 가리키는 포인터 3개를 담을 배열.
 *                  유효 거리가 3개 미만이면 포인터는 null이 됩니다.
 */
void findNearestThreeAnchors(AnchorData* nearest[3]) {
  // 포인터 초기화
  nearest[0] = nearest[1] = nearest[2] = nullptr;
  
  // 층별 앵커 목록 만들기
  AnchorData* current_floor_anchors[NUM_ANCHORS]; // 현재 층 앵커
  AnchorData* other_floor_anchors[NUM_ANCHORS];   // 다른 층 앵커
  double current_floor_dists[NUM_ANCHORS];        // 현재 층 앵커 거리
  double other_floor_dists[NUM_ANCHORS];          // 다른 층 앵커 거리
  
  int current_floor_count = 0; // 현재 층의 유효한 앵커 수
  int other_floor_count = 0;   // 다른 층의 유효한 앵커 수
  
  // 앵커를 층별로 분류
  for (int i = 0; i < NUM_ANCHORS; ++i) {
    if (anchors[i].distance_valid && anchors[i].distance >= 0) {
      if (anchors[i].floor == current_floor) {
        // 현재 층의 앵커
        current_floor_anchors[current_floor_count] = &anchors[i];
        current_floor_dists[current_floor_count] = anchors[i].distance;
        current_floor_count++;
      } else {
        // 다른 층의 앵커
        other_floor_anchors[other_floor_count] = &anchors[i];
        other_floor_dists[other_floor_count] = anchors[i].distance;
        other_floor_count++;
      }
    }
  }
  
  // 현재 층 앵커 정렬 (삽입 정렬)
  for (int i = 1; i < current_floor_count; i++) {
    AnchorData* key_anchor = current_floor_anchors[i];
    double key_dist = current_floor_dists[i];
    int j = i - 1;
    
    while (j >= 0 && current_floor_dists[j] > key_dist) {
      current_floor_anchors[j + 1] = current_floor_anchors[j];
      current_floor_dists[j + 1] = current_floor_dists[j];
      j--;
    }
    current_floor_anchors[j + 1] = key_anchor;
    current_floor_dists[j + 1] = key_dist;
  }
  
  // 다른 층 앵커 정렬 (삽입 정렬)
  for (int i = 1; i < other_floor_count; i++) {
    AnchorData* key_anchor = other_floor_anchors[i];
    double key_dist = other_floor_dists[i];
    int j = i - 1;
    
    while (j >= 0 && other_floor_dists[j] > key_dist) {
      other_floor_anchors[j + 1] = other_floor_anchors[j];
      other_floor_dists[j + 1] = other_floor_dists[j];
      j--;
    }
    other_floor_anchors[j + 1] = key_anchor;
    other_floor_dists[j + 1] = key_dist;
  }
  
  // 현재 층의 앵커를 우선적으로 사용
  int nearest_count = 0;
  
  // 1. 현재 층의 앵커 추가
  for (int i = 0; i < current_floor_count && nearest_count < 3; i++) {
    nearest[nearest_count++] = current_floor_anchors[i];
  }
  
  // 2. 부족한 경우 다른 층의 앵커로 채움
  for (int i = 0; i < other_floor_count && nearest_count < 3; i++) {
    nearest[nearest_count++] = other_floor_anchors[i];
  }
  
  // 선택된 앵커 디버그 출력
  char dbg_str[50];
  for (int i = 0; i < 3; i++) {
    if (nearest[i]) {
      snprintf(dbg_str, sizeof(dbg_str), "선택된 앵커 %d: %04X 층:%d D:%.2f", 
              i+1, nearest[i]->id, nearest[i]->floor+1, nearest[i]->distance);
      test_run_info((unsigned char*)dbg_str);
    } else {
      snprintf(dbg_str, sizeof(dbg_str), "선택된 앵커 %d: 없음", i+1);
      test_run_info((unsigned char*)dbg_str);
    }
  }
}


/*
 * @brief 2D 삼각측량을 사용하여 위치를 계산합니다.
 *        층은 별도의 알고리즘으로 결정합니다.
 *
 * @param a1, a2, a3 - 유효한 거리를 가진 가장 가까운 앵커 3개에 대한 포인터. 호출 컨텍스트에 따라 null이 아니라고 가정합니다.
 * @param result_pos - 계산된 위치를 저장할 Point 구조체에 대한 참조.
 * @return 계산이 성공하면 true, 그렇지 않으면 false (예: 동일 선상 앵커, 수학적 오류).
 */
bool calculatePosition(AnchorData* a1, AnchorData* a2, AnchorData* a3, Point& result_pos) {
  // 가독성을 위해 데이터 추출 (2D 좌표만 사용)
  double x1 = a1->pos.x, y1 = a1->pos.y, r1 = a1->distance;
  double x2 = a2->pos.x, y2 = a2->pos.y, r2 = a2->distance;
  double x3 = a3->pos.x, y3 = a3->pos.y, r3 = a3->distance;
  
  // 거리가 비정상적으로 크거나 작은 경우 처리
  const double MAX_VALID_DISTANCE = 30.0; // 최대 30m 가정
  if (r1 > MAX_VALID_DISTANCE || r2 > MAX_VALID_DISTANCE || r3 > MAX_VALID_DISTANCE) {
    test_run_info((unsigned char *)"삼각측량 오류: 비정상적으로 큰 거리 측정값");
    return false;
  }
  
  // 2D 삼각측량 (교차점 계산 방법 사용)
  
  // 동일 지점 앵커 확인
  if ((abs(x1 - x2) < 1e-6 && abs(y1 - y2) < 1e-6) ||
      (abs(x1 - x3) < 1e-6 && abs(y1 - y3) < 1e-6) ||
      (abs(x2 - x3) < 1e-6 && abs(y2 - y3) < 1e-6)) {
    test_run_info((unsigned char *)"삼각측량 오류: 앵커가 동일 지점에 있습니다.");
    return false;
  }
  
  // 앵커가 동일 선상에 있는지 확인
  double slope1 = (x2 == x1) ? 1e9 : (y2 - y1) / (x2 - x1); // A1->A2 직선의 기울기
  double slope2 = (x3 == x1) ? 1e9 : (y3 - y1) / (x3 - x1); // A1->A3 직선의 기울기
  
  if (abs(slope1 - slope2) < 1e-6) {
    test_run_info((unsigned char *)"삼각측량 오류: 앵커 3개가 동일 선상에 있습니다.");
    return false;
  }
  
  // 삼변측량 (trilateration) 계산
  // 원의 방정식을 연립하여 교점 계산
  
  // 원의 방정식: (x-xi)^2 + (y-yi)^2 = ri^2, i = 1,2,3
  // 연립 방정식을 선형 방정식으로 변환
  
  // A1과 A2의 방정식을 빼서 선형화
  double A = 2 * (x2 - x1);
  double B = 2 * (y2 - y1);
  double C = r1*r1 - r2*r2 - x1*x1 + x2*x2 - y1*y1 + y2*y2;
  
  // A1과 A3의 방정식을 빼서 선형화
  double D = 2 * (x3 - x1);
  double E = 2 * (y3 - y1);
  double F = r1*r1 - r3*r3 - x1*x1 + x3*x3 - y1*y1 + y3*y3;
  
  // 두 선형 방정식의 교점 계산
  double det = A*E - B*D;
  if (abs(det) < 1e-6) {
    test_run_info((unsigned char *)"삼각측량 오류: 수학적 계산 실패 (특이행렬)");
    return false;
  }
  
  // 태그의 X, Y 좌표 계산
  result_pos.x = (C*E - B*F) / det;
  result_pos.y = (A*F - C*D) / det;
  
  // Z 좌표는 현재 층 높이로 설정 (각 층은 3m 높이 가정)
  const float FLOOR_HEIGHT = 3.0; // 층 높이 (미터)
  result_pos.z = current_floor * FLOOR_HEIGHT + 1.5; // 바닥 + 1.5m 높이에 태그 위치
  
  // 계산 결과 검증
  // 계산된 위치가 앵커들로부터 측정된 거리와 일치하는지 대략 확인
  double calc_dist1 = sqrt(pow(result_pos.x - x1, 2) + pow(result_pos.y - y1, 2));
  double calc_dist2 = sqrt(pow(result_pos.x - x2, 2) + pow(result_pos.y - y2, 2));
  double calc_dist3 = sqrt(pow(result_pos.x - x3, 2) + pow(result_pos.y - y3, 2));
  
  // 계산된 거리와 측정 거리 사이의 오차가 너무 큰 경우 경고
  const double MAX_ERROR = 2.0; // 최대 허용 오차 2m
  if (abs(calc_dist1 - r1) > MAX_ERROR ||
      abs(calc_dist2 - r2) > MAX_ERROR ||
      abs(calc_dist3 - r3) > MAX_ERROR) {
    char err_str[80];
    snprintf(err_str, sizeof(err_str), "삼각측량 경고: 오차 큼 (%.1f, %.1f, %.1f)",
             abs(calc_dist1 - r1), abs(calc_dist2 - r2), abs(calc_dist3 - r3));
    test_run_info((unsigned char *)err_str);
    // 경고만 출력하고 계산은 계속 진행
  }
  
  return true; // 계산 성공
}

/**
 * @brief 현재 층을 결정하는 알고리즘
 * 층마다 존재하는 앵커들로부터의 거리를 기반으로 사용자의 층을 결정합니다.
 * 층 변경은 안정성을 위해 히스테리시스 매커니즘을 사용합니다.
 */
void determineFloor() {
  // 층별 가중치 계산
  double floor_weights[3] = {0, 0, 0}; // 최대 3개 층 지원
  int floor_anchor_counts[3] = {0, 0, 0}; // 층별 유효 앵커 수 저장

  // 각 층마다 앵커까지의 거리의 역수를 가중치로 사용
  // 더 가까울수록 더 높은 가중치를 가짐
  for (int i = 0; i < NUM_ANCHORS; i++) {
    if (anchors[i].distance_valid && anchors[i].distance > 0) {
      // 앵커 층 인덱스 (0부터 시작)
      uint8_t anchor_floor = anchors[i].floor;
      
      // 거리에 대한 가중치 계산 (거리의 역수)
      double weight = 1.0 / anchors[i].distance;
      
      // 해당 층 가중치에 더함
      floor_weights[anchor_floor] += weight;
      
      // 해당 층 유효 앵커 수 증가
      floor_anchor_counts[anchor_floor]++;
    }
  }

  // 층별 평균 가중치 계산 (앵커 수로 나눔)
  for (int f = 0; f < 3; f++) {
    if (floor_anchor_counts[f] > 0) {
      floor_weights[f] /= floor_anchor_counts[f];
    }
  }

  // 가장 높은 가중치를 가진 층 찾기
  uint8_t highest_weight_floor = 0;
  double highest_weight = floor_weights[0];
  
  for (int f = 1; f < 3; f++) {
    if (floor_weights[f] > highest_weight) {
      highest_weight = floor_weights[f];
      highest_weight_floor = f;
    }
  }

  // 층 변경 히스테리시스 로직
  if (highest_weight_floor != current_floor) {
    // 층 변경 카운터 증가
    floor_change_count++;
    
    // 일정 횟수 이상 동일 층으로 계산된 경우 층 변경
    if (floor_change_count >= floor_vote_threshold) {
      // 층 변경
      char floor_str[50];
      snprintf(floor_str, sizeof(floor_str), "층 변경: %d층 -> %d층", 
              current_floor+1, highest_weight_floor+1);
      test_run_info((unsigned char *)floor_str);
      
      current_floor = highest_weight_floor;
      floor_change_count = 0; // 카운터 리셋
    }
  } else {
    // 동일한 층으로 계속 측정되면 카운터 리셋
    floor_change_count = 0;
  }
  
  // 층별 가중치 디버그 출력
  char weight_str[60];
  snprintf(weight_str, sizeof(weight_str), "층 가중치: 1층=%.2f, 2층=%.2f, 3층=%.2f", 
          floor_weights[0], floor_weights[1], floor_weights[2]);
  test_run_info((unsigned char *)weight_str);
}


// resp_msg_get_ts 함수는 DW3000 라이브러리의 dw3000_shared_functions.cpp에서 제공하므로 여기서 정의하지 않습니다.

/**
 * @brief 현재 계산된 위치 정보를 시리얼로 출력하는 함수
 * 안드로이드 앱에서 파싱할 수 있는 정형화된 포맷으로 출력
 */
void serialPrintPosition() {
  // JSON 형태의 위치 정보 출력 (안드로이드 파싱용)
  char json_str[100];
  snprintf(json_str, sizeof(json_str), 
        "{\"x\":%.2f,\"y\":%.2f,\"z\":%.2f,\"floor\":%d,\"valid\":%s}", 
        current_position.x, current_position.y, current_position.z, 
        current_floor + 1, position_valid ? "true" : "false");
  
  // 웹페이지나 앱에서 파싱하기 쉽게 태그를 붙임
  Serial.print("<POS>");
  Serial.print(json_str);
  Serial.println("</POS>");
  
  // 가독성을 위한 일반 텍스트 정보 (디버깅용)
  char pos_str[80];
  snprintf(pos_str, sizeof(pos_str), "위치: X=%.2f m, Y=%.2f m, Z=%.2f m, 층=%d", 
           current_position.x, current_position.y, current_position.z, current_floor + 1);
  test_run_info((unsigned char *)pos_str);
}