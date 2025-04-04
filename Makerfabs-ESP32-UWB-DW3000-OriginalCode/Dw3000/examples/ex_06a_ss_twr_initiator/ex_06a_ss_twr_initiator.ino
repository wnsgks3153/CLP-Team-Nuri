#include <vector> // std::vector 사용을 위해 포함
#include <algorithm> // std::sort 또는 std::partial_sort 사용을 위해 포함
#include <cmath> // 삼각측량에서 sqrt, pow 사용을 위해 포함

#include "dw3000.h"

#define APP_NAME "SS TWR INIT 다층 측위 v1.0" // 애플리케이션 이름 수정

// 연결 핀 정의
const uint8_t PIN_RST = 27; // 리셋 핀
const uint8_t PIN_IRQ = 34; // IRQ 핀
const uint8_t PIN_SS = 4; // SPI 선택 핀

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

/* 64 MHz PRF에 대한 기본 안테나 지연 값. 아래 NOTE 2 참조. */
#define TX_ANT_DLY 16385
#define RX_ANT_DLY 16385

/* 거리 측정 프로세스에서 사용되는 프레임. 아래 NOTE 3 참조. */
// 소스 주소 (바이트 7-8) - 이 Initiator의 고유 ID 설정
#define INITIATOR_ID_BYTE_7 0xDE // 예제 ID - 실제 값으로 교체 필요
#define INITIATOR_ID_BYTE_8 0x11 // 예제 ID - 실제 값으로 교체 필요
static uint8_t tx_poll_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 0x00, 0x00, INITIATOR_ID_BYTE_7, INITIATOR_ID_BYTE_8, 0xE0, 0, 0}; // 목적지 주소 (바이트 5-6)는 각 앵커마다 업데이트됨
static uint8_t rx_resp_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, INITIATOR_ID_BYTE_7, INITIATOR_ID_BYTE_8, 0x00, 0x00, 0xE1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}; // 예상 응답 템플릿 (소스 주소 바이트 7-8 확인됨)
/* 메시지의 공통 부분 길이 (기능 코드까지 포함). 아래 NOTE 3 참조. */
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

/* 수신된 응답 메시지를 저장할 버퍼.
 * 이 예제 코드가 처리해야 하는 가장 긴 프레임에 맞게 크기가 조정됨. */
#define RX_BUF_LEN 20
static uint8_t rx_buffer[RX_BUF_LEN];

/* 디버그 중단점에서 검사할 수 있도록 상태 레지스터 상태의 복사본을 여기에 보관. */
static uint32_t status_reg = 0;

/* 프레임 간 지연 시간 (UWB 마이크로초). 아래 NOTE 1 참조. */
#define POLL_TX_TO_RESP_RX_DLY_UUS 240
/* 응답 수신 타임아웃. 아래 NOTE 5 참조. */
#define RESP_RX_TIMEOUT_UUS 400

/* --- 다층 측위 구조체 및 상수 --- */
// 3차원 좌표를 나타내는 구조체
struct Point {
    float x;
    float y;
    float z;
};

// 앵커 데이터를 저장하는 구조체
struct AnchorData {
  uint16_t id;      // 16비트 앵커 ID
  Point pos;        // 앵커 위치 (x, y, z)
  int floor;        // 앵커가 위치한 층 번호 (0부터 시작)
  double distance;  // 마지막으로 측정된 거리
  bool distance_valid; // 마지막 측정이 성공적이었는지 여부
  uint64_t last_heard_ts; // 마지막 성공적인 거리 측정 타임스탬프
};

// 층간 높이 상수 (미터)
#define FLOOR_HEIGHT 3.0

// 층 전환시 사용할 히스테리시스 임계값 (층간 전환을 방지하기 위한 가중치)
#define FLOOR_VOTE_THRESHOLD 0.7

// !!! 중요: 실제 앵커 ID와 위치로 교체해야 합니다 !!!
#define NUM_ANCHORS 20 // 총 앵커 개수
#define MAX_ANCHORS_PER_FLOOR 10 // 층당 최대 앵커 수

AnchorData anchors[NUM_ANCHORS] = {
  // 0층 앵커들 (예시 - 실제 값으로 교체 필요)
  {0xA001, {0.0, 0.0, 1.5}, 0, -1.0, false, 0},
  {0xA002, {5.0, 0.0, 1.5}, 0, -1.0, false, 0},
  {0xA003, {10.0, 0.0, 1.5}, 0, -1.0, false, 0},
  {0xA004, {0.0, 5.0, 1.5}, 0, -1.0, false, 0},
  {0xA005, {5.0, 5.0, 1.5}, 0, -1.0, false, 0},
  {0xA006, {10.0, 5.0, 1.5}, 0, -1.0, false, 0},
  {0xA007, {0.0, 10.0, 1.5}, 0, -1.0, false, 0},
  {0xA008, {5.0, 10.0, 1.5}, 0, -1.0, false, 0},
  
  // 1층 앵커들 (예시 - 실제 값으로 교체 필요)
  {0xA101, {0.0, 0.0, 1.5}, 1, -1.0, false, 0},
  {0xA102, {5.0, 0.0, 1.5}, 1, -1.0, false, 0},
  {0xA103, {10.0, 0.0, 1.5}, 1, -1.0, false, 0},
  {0xA104, {0.0, 5.0, 1.5}, 1, -1.0, false, 0},
  {0xA105, {5.0, 5.0, 1.5}, 1, -1.0, false, 0},
  {0xA106, {10.0, 5.0, 1.5}, 1, -1.0, false, 0},
  
  // 2층 앵커들 (예시 - 실제 값으로 교체 필요)
  {0xA201, {0.0, 0.0, 1.5}, 2, -1.0, false, 0},
  {0xA202, {5.0, 0.0, 1.5}, 2, -1.0, false, 0},
  {0xA203, {10.0, 0.0, 1.5}, 2, -1.0, false, 0},
  {0xA204, {0.0, 5.0, 1.5}, 2, -1.0, false, 0},
  {0xA205, {5.0, 5.0, 1.5}, 2, -1.0, false, 0},
  {0xA206, {10.0, 5.0, 1.5}, 2, -1.0, false, 0}
};

// 계산된 위치와 층 결과를 저장하는 변수
Point current_position = {0.0, 0.0, 0.0};
int current_floor = 0; // 현재 층 (0부터 시작)
int prev_floor = 0;    // 이전 층 (히스테리시스를 위해 필요)
bool position_valid = false; // 위치 계산 성공 여부

// 직렬 출력을 위한 JSON 버퍼
char json_buffer[256];

/* PG_DELAY 및 TX_POWER 레지스터 값은 현재 온도의 스펙트럼 대역폭과 전력을 반영합니다.
 * 이 값들은 참조 측정 전에 보정될 수 있습니다. 아래 NOTE 2 참조. */
extern dwt_txconfig_t txconfig_options;

// --- 함수 프로토타입 ---
void findNearestAnchorsOnFloor(int floor, AnchorData* result[3], int* found_count); // 특정 층에서 가장 가까운 앵커들 찾기
bool calculatePosition(AnchorData* a1, AnchorData* a2, AnchorData* a3, Point& result_pos); // 위치 계산 (삼각측량)
int determineFloor(void); // 층 결정 알고리즘
void createPositionJson(char* buffer, int buffer_size); // 위치 정보 JSON 생성
void resp_msg_get_ts(uint8_t *ts_field, uint32_t *ts); // 응답 메시지에서 타임스탬프 추출

// 플랫폼 특정 함수 (외부에서 정의되거나 스텁 제공 필요)
extern void UART_init();
extern void UART_puts(const char *s);
extern void test_run_info(unsigned char *s); // 정보 출력 함수
extern void spiBegin(uint8_t pin_irq, uint8_t pin_rst);
extern void spiSelect(uint8_t pin_ss);
extern void Sleep(uint32_t ms); // 지연 함수 (또는 Arduino의 delay())
extern void delay(uint32_t ms); // Arduino 호환성을 위한 delay 함수


void setup() {
  Serial.begin(115200); // 시리얼 통신 초기화
  test_run_info((unsigned char *)APP_NAME); // 앱 이름 출력

  /* SPI 속도 설정, DW3000은 최대 38 MHz 지원 */
  /* DW IC 리셋 */
  spiBegin(PIN_IRQ, PIN_RST);
  spiSelect(PIN_SS);

  delay(2); // DW3000 시작에 필요한 시간 (INIT_RC -> IDLE_RC 전환 또는 SPIRDY 이벤트 대기)

  while (!dwt_checkidlerc()) // 진행하기 전에 DW IC가 IDLE_RC 상태인지 확인 필요
  {
    Serial.println("IDLE FAILED"); // Serial 출력 사용
    test_run_info((unsigned char *)"IDLE 실패 - 중단됨"); // test_run_info를 사용하여 출력
    while (1) ; // 중단
  }

  if (dwt_initialise(DWT_DW_INIT) == DWT_ERROR) // DW IC 초기화
  {
    Serial.println("INIT FAILED"); // Serial 출력 사용
    test_run_info((unsigned char *)"INIT 실패 - 중단됨"); // test_run_info를 사용하여 출력
    while (1) ; // 중단
  }

  // 디버깅을 위해 LED 활성화. 각 TX마다 DW3000 빨간색 평가 쉴드 보드의 D1 LED가 깜박임.
  dwt_setleds(DWT_LEDS_ENABLE | DWT_LEDS_INIT_BLINK);

  /* DW IC 설정. 아래 NOTE 6 참조. */
  if(dwt_configure(&config)) // dwt_configure가 DWT_ERROR를 반환하면 PLL 또는 RX 보정 실패. 호스트는 장치를 리셋해야 함.
  {
    Serial.println("CONFIG FAILED"); // Serial 출력 사용
    test_run_info((unsigned char *)"CONFIG 실패 - 중단됨"); // test_run_info를 사용하여 출력
    while (1) ; // 중단
  }

    /* TX 스펙트럼 파라미터 설정 (전력, PG 지연, PG 카운트) */
    dwt_configuretxrf(&txconfig_options);

    /* 기본 안테나 지연 값 적용. 아래 NOTE 2 참조. */
    dwt_setrxantennadelay(RX_ANT_DLY);
    dwt_settxantennadelay(TX_ANT_DLY);

    /* 예상 응답의 지연 및 타임아웃 설정. 아래 NOTE 1 및 5 참조. */
    // 이 값들은 앵커별 응답 시간이 다른 경우 조정이 필요할 수 있음.
    dwt_setrxaftertxdelay(POLL_TX_TO_RESP_RX_DLY_UUS);
    dwt_setrxtimeout(RESP_RX_TIMEOUT_UUS);

    /* 디버깅을 돕기 위해 GPIO 5, 6에서 TX/RX 상태 출력 활성화 가능, TX/RX LED도 활성화 가능.
     * 참고: 실제 저전력 애플리케이션에서는 LED를 사용하지 않아야 함. */
    dwt_setlnapamode(DWT_LNA_ENABLE | DWT_PA_ENABLE); // LNA/PA 활성화

    test_run_info((unsigned char *)"설정 완료. 다층 측위 시작.");
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

        /* 프레임 데이터를 DW IC에 쓰고 전송 준비. 아래 NOTE 7 참조. */
        tx_poll_msg[ALL_MSG_SN_IDX] = frame_seq_nb; // 시퀀스 번호 설정
        dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS_BIT_MASK); // TX 상태 비트 클리어
        dwt_writetxdata(sizeof(tx_poll_msg), tx_poll_msg, 0); /* TX 버퍼 오프셋 0. */
        dwt_writetxfctrl(sizeof(tx_poll_msg), 0, 1); /* TX 버퍼 오프셋 0, 거리 측정 비트 설정. */

        /* 전송 시작, 응답이 예상됨을 나타내어 프레임 전송 후 dwt_setrxaftertxdelay()로 설정된 지연 시간이 지나면 자동으로 수신 활성화. */
        dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED);

        /* 전송이 올바르게 완료되었다고 가정하고, 프레임 수신 또는 오류/타임아웃 폴링. 아래 NOTE 8 참조. */
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

                    /* Poll 전송 및 응답 수신 타임스탬프 검색. 아래 NOTE 9 참조. */
                    poll_tx_ts = dwt_readtxtimestamplo32(); // Poll TX 타임스탬프 (하위 32비트)
                    resp_rx_ts = dwt_readrxtimestamplo32(); // 응답 RX 타임스탬프 (하위 32비트)

                    /* 반송파 적분기 값 읽고 클럭 오프셋 비율 계산. 아래 NOTE 11 참조. */
                    clockOffsetRatio = ((float)dwt_readclockoffset()) / (uint32_t)(1<<26);

                    /* 응답 메시지에 포함된 타임스탬프 가져오기. */
                    resp_msg_get_ts(&rx_buffer[RESP_MSG_POLL_RX_TS_IDX], &poll_rx_ts); // Responder의 Poll RX 타임스탬프
                    resp_msg_get_ts(&rx_buffer[RESP_MSG_RESP_TX_TS_IDX], &resp_tx_ts); // Responder의 응답 TX 타임스탬프

                    /* 로컬 및 원격 클럭 속도 차이를 보정하기 위해 클럭 오프셋 비율을 사용하여 비행 시간 및 거리 계산 */
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
                              anchors[i].id, anchors[i].floor, anchors[i].distance);
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
                 snprintf(err_str, sizeof(err_str), "앵커 0x%04X: 프레임 길이 오류 (%lu)", anchors[i].id, (unsigned long)frame_len);
                 test_run_info((unsigned char *)err_str);
            }
        }
        else // 프레임 수신 실패 (타임아웃 또는 오류)
        {
            /* DW IC 상태 레지스터에서 RX 오류/타임아웃 이벤트 클리어. */
            dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);
            char err_str[40];
            snprintf(err_str, sizeof(err_str), "앵커 0x%04X: RX 실패/타임아웃", anchors[i].id);
            test_run_info((unsigned char *)err_str);
            // distance_valid는 false로 유지
        }

        /* 다른 앵커와의 거리 측정 시도 사이에 짧은 지연 */
        if (i < NUM_ANCHORS - 1) {
             delay(INTRA_RANGING_DELAY_MS); // 표준 Arduino delay 또는 Sleep() 사용
        }
    } // 모든 앵커에 대한 거리 측정 루프 종료

    // --- 층 결정 및 위치 계산 로직 ---
    if (valid_ranges_count >= 3) { // 최소 3개의 유효한 거리가 있어야 삼각측량 가능
        // 층 결정 알고리즘 실행
        int determined_floor = determineFloor();
        test_run_info((unsigned char *)"층 결정 완료");
        char floor_str[40];
        snprintf(floor_str, sizeof(floor_str), "결정된 층: %d (이전 층: %d)", determined_floor, current_floor);
        test_run_info((unsigned char *)floor_str);

        // 현재 층에서 가장 가까운 앵커 3개 찾기
        AnchorData* closest_anchors[3] = {nullptr, nullptr, nullptr};
        int found_anchors_count = 0;
        findNearestAnchorsOnFloor(determined_floor, closest_anchors, &found_anchors_count);

        // 충분한 앵커가 없으면 인접 층에서도 찾아봄
        if (found_anchors_count < 3) {
            test_run_info((unsigned char *)"현재 층에 앵커가 부족합니다. 인접 층 탐색 중...");
            
            // 위층 먼저 탐색
            if (determined_floor < 2) { // 최상층이 아니면
                AnchorData* upper_anchors[3] = {nullptr, nullptr, nullptr};
                int upper_count = 0;
                findNearestAnchorsOnFloor(determined_floor + 1, upper_anchors, &upper_count);
                
                // 찾은 앵커 추가
                for (int i = 0; i < upper_count && found_anchors_count < 3; i++) {
                    closest_anchors[found_anchors_count++] = upper_anchors[i];
                }
            }
            
            // 여전히 부족하면 아래층 탐색
            if (found_anchors_count < 3 && determined_floor > 0) { // 최하층이 아니면
                AnchorData* lower_anchors[3] = {nullptr, nullptr, nullptr};
                int lower_count = 0;
                findNearestAnchorsOnFloor(determined_floor - 1, lower_anchors, &lower_count);
                
                // 찾은 앵커 추가
                for (int i = 0; i < lower_count && found_anchors_count < 3; i++) {
                    closest_anchors[found_anchors_count++] = lower_anchors[i];
                }
            }
        }

        // 위치 계산 수행
        if (found_anchors_count >= 3) {
            test_run_info((unsigned char *)"삼각측량을 위한 가장 가까운 앵커 3개 발견. 위치 계산 중...");
            
            // 선택된 앵커 정보 출력
            for (int i = 0; i < 3; i++) {
                char anchor_str[70];
                snprintf(anchor_str, sizeof(anchor_str), "선택된 앵커 %d: ID=0x%04X, 층=%d, 거리=%.2f m", 
                         i+1, closest_anchors[i]->id, closest_anchors[i]->floor, closest_anchors[i]->distance);
                test_run_info((unsigned char *)anchor_str);
            }
            
            // 삼각측량 계산 수행
            position_valid = calculatePosition(closest_anchors[0], closest_anchors[1], closest_anchors[2], current_position);
            
            // 계산된 위치에 층 높이 적용
            current_position.z = determined_floor * FLOOR_HEIGHT + 1.5; // 1.5m는 대략적인 태그 높이 가정
            
            // 층 갱신
            prev_floor = current_floor;
            current_floor = determined_floor;

            if (position_valid) { // 위치 계산 성공
                // 위치 정보 출력
                char pos_str[60];
                snprintf(pos_str, sizeof(pos_str), "위치 (X,Y,Z): %.2f, %.2f, %.2f, 층: %d", 
                         current_position.x, current_position.y, current_position.z, current_floor);
                test_run_info((unsigned char *)pos_str);
                
                // JSON 형식으로 변환하여 Serial로 출력 (외부 시스템 연동용)
                createPositionJson(json_buffer, sizeof(json_buffer));
                Serial.println(json_buffer); // JSON 데이터 출력
            } else { // 위치 계산 실패 (기하학적/수학적 오류)
                test_run_info((unsigned char *)"삼각측량 실패 (기하학적/수학적 오류). 이전 위치 유지.");
            }
        } else {
            test_run_info((unsigned char *)"위치 계산 실패: 유효한 앵커 3개를 찾을 수 없습니다.");
            position_valid = false;
        }
    } else { // 유효한 거리가 3개 미만인 경우
        char err_str[60];
        snprintf(err_str, sizeof(err_str), "삼각측량 위해 유효 범위 >=3개 필요, %d개 발견.", valid_ranges_count);
        test_run_info((unsigned char *)err_str);
        position_valid = false;
    }

    /* 다음 전체 거리 측정 사이클 전에 지연 실행. */
    delay(CYCLE_DELAY_MS); // 표준 Arduino delay 또는 Sleep() 사용
}


/*
 * @brief 특정 층에서 가장 가까운 앵커 3개를 찾습니다.
 *        특정 층에 속한 앵커들 중에서 가장 거리가 짧은 순서대로 최대 3개까지 반환합니다.
 *
 * @param floor - 찾을 앵커들이 위치한 층
 * @param result - 가장 가까운 앵커를 가리키는 포인터 배열
 * @param found_count - 실제로 찾은 앵커 개수를 저장할 포인터
 */
void findNearestAnchorsOnFloor(int floor, AnchorData* result[3], int* found_count) {
    *found_count = 0; // 결과 카운터 초기화
    
    // 각 층의 앵커들 중 유효한 거리 측정값을 가진 것들을 찾음
    std::vector<AnchorData*> valid_anchors;
    
    for (int i = 0; i < NUM_ANCHORS; i++) {
        if (anchors[i].floor == floor && anchors[i].distance_valid && anchors[i].distance >= 0) {
            valid_anchors.push_back(&anchors[i]);
        }
    }
    
    // 발견된 유효한 앵커가 없으면 함수 종료
    if (valid_anchors.empty()) {
        char msg[40];
        snprintf(msg, sizeof(msg), "층 %d에 유효한 앵커가 없습니다.", floor);
        test_run_info((unsigned char *)msg);
        return;
    }
    
    // 거리 기준으로 정렬
    std::sort(valid_anchors.begin(), valid_anchors.end(), 
              [](AnchorData* a, AnchorData* b) { return a->distance < b->distance; });
    
    // 가장 가까운 앵커 최대 3개를 결과 배열에 복사
    int copy_count = std::min(3, (int)valid_anchors.size());
    for (int i = 0; i < copy_count; i++) {
        result[i] = valid_anchors[i];
    }
    
    *found_count = copy_count; // 찾은 앵커 수 반환
    
    char msg[40];
    snprintf(msg, sizeof(msg), "층 %d에서 %d개의 유효한 앵커 발견", floor, copy_count);
    test_run_info((unsigned char *)msg);
}


/*
 * @brief 가중치 기반 투표 시스템을 사용하여 태그의 현재 층을 결정합니다.
 *        1) 각 측정은 거리에 반비례하는 가중치를 가집니다 (가까울수록 가중치 높음)
 *        2) 히스테리시스를 적용하여 층간 바운싱을 방지합니다
 *        
 * @return 태그가 있는 것으로 판단된 층 번호
 */
int determineFloor() {
    // 층별 점수 합계 계산을 위한 배열 (0층, 1층, 2층)
    double floor_scores[3] = {0, 0, 0};
    int valid_anchors_count[3] = {0, 0, 0};
    double total_weight = 0;
    
    // 각 앵커의 가중치 계산 및 층별 점수 합산
    for (int i = 0; i < NUM_ANCHORS; i++) {
        if (anchors[i].distance_valid && anchors[i].distance > 0) {
            int anchor_floor = anchors[i].floor;
            if (anchor_floor >= 0 && anchor_floor < 3) { // 유효한 층 범위 확인
                // 거리에 반비례하는 가중치 계산 (더 가까울수록 가중치 높음)
                // 1/거리^2 사용하여 가중치 계산
                double weight = 1.0 / (anchors[i].distance * anchors[i].distance);
                
                // 층별 점수 및 카운트 증가
                floor_scores[anchor_floor] += weight;
                valid_anchors_count[anchor_floor]++;
                total_weight += weight;
            }
        }
    }
    
    // 총 가중치가 0이면 기본값 반환
    if (total_weight <= 0) {
        return current_floor; // 이전 층 유지
    }
    
    // 각 층별 정규화된 점수 계산
    double normalized_scores[3];
    for (int i = 0; i < 3; i++) {
        normalized_scores[i] = floor_scores[i] / total_weight;
    }
    
    // 결과 출력 (디버깅)
    char score_str[80];
    snprintf(score_str, sizeof(score_str), "층별 점수 - 0층: %.2f (%d개), 1층: %.2f (%d개), 2층: %.2f (%d개)", 
             normalized_scores[0], valid_anchors_count[0], 
             normalized_scores[1], valid_anchors_count[1], 
             normalized_scores[2], valid_anchors_count[2]);
    test_run_info((unsigned char *)score_str);
    
    // 가장 높은 점수를 가진 층 찾기
    int max_floor = 0;
    double max_score = normalized_scores[0];
    
    for (int i = 1; i < 3; i++) {
        if (normalized_scores[i] > max_score) {
            max_score = normalized_scores[i];
            max_floor = i;
        }
    }
    
    // 히스테리시스 적용 - 현재 층과 다른 층으로 변경하려면 임계값을 초과해야 함
    if (max_floor != current_floor) {
        // 현재 층의 점수가 특정 임계값 이상이면 층 변경을 지연
        if (normalized_scores[current_floor] > FLOOR_VOTE_THRESHOLD * max_score) {
            // 충분히 강한 증거가 없으면 현재 층 유지
            return current_floor;
        }
    }
    
    return max_floor;
}


/*
 * @brief 2D 삼각측량을 사용하여 위치를 계산합니다.
 *        Wikipedia의 Trilateration 문서를 기반으로 하며, 적당한 오차에 강건합니다.
 *        2D 평면에서 계산을 단순화하기 위해 z 좌표는 무시합니다.
 *
 * @param a1, a2, a3 - 유효한 거리를 가진 가장 가까운 앵커 3개에 대한 포인터. 호출 컨텍스트에 따라 null이 아니라고 가정합니다.
 * @param result_pos - 계산된 위치를 저장할 Point 구조체에 대한 참조.
 * @return 계산이 성공하면 true, 그렇지 않으면 false (예: 동일 선상 앵커, 수학적 오류).
 */
bool calculatePosition(AnchorData* a1, AnchorData* a2, AnchorData* a3, Point& result_pos) {
    // 가독성을 위해 데이터 추출 (z 좌표는 무시하고 2D 평면에서 계산)
    double x1 = a1->pos.x, y1 = a1->pos.y, r1 = a1->distance;
    double x2 = a2->pos.x, y2 = a2->pos.y, r2 = a2->distance;
    double x3 = a3->pos.x, y3 = a3->pos.y, r3 = a3->distance;

    // 2D 평면에서 삼각측량 수행
    // 선형 방정식 접근법 사용
    double A = 2 * (x2 - x1);
    double B = 2 * (y2 - y1);
    double C = r1*r1 - r2*r2 - x1*x1 + x2*x2 - y1*y1 + y2*y2;
    double D = 2 * (x3 - x2);
    double E = 2 * (y3 - y2);
    double F = r2*r2 - r3*r3 - x2*x2 + x3*x3 - y2*y2 + y3*y3;

    // 선형 방정식의 행렬식 계산
    double det = A*E - B*D;
    
    // 행렬식이 0에 가까우면 앵커들이 동일 선상에 있다는 의미
    if (fabs(det) < 1e-6) {
        test_run_info((unsigned char *)"삼각측량 오류: 앵커가 동일 선상에 있습니다.");
        return false;
    }

    // 선형 방정식 풀이
    result_pos.x = (C*E - B*F) / det;
    result_pos.y = (A*F - C*D) / det;
    // z 좌표는 호출 컨텍스트에서 설정됨 (층 기반)

    return true; // 계산 성공
}


/*
 * @brief 위치 데이터를 JSON 문자열로 변환합니다.
 *        외부 시스템(안드로이드 앱 등)과 통신하기 위한 형식입니다.
 *
 * @param buffer - JSON 문자열을 저장할 버퍼
 * @param buffer_size - 버퍼 크기
 */
void createPositionJson(char* buffer, int buffer_size) {
    // JSON 형식: {"x":123.45,"y":67.89,"z":1.5,"floor":1,"valid":true,"timestamp":1234567890}
    long timestamp = millis(); // 현재 시간 (밀리초)
    
    snprintf(buffer, buffer_size, 
             "{\"x\":%.2f,\"y\":%.2f,\"z\":%.2f,\"floor\":%d,\"valid\":%s,\"timestamp\":%ld}",
             current_position.x, current_position.y, current_position.z, 
             current_floor, position_valid ? "true" : "false", timestamp);
}


/*
 * @brief 응답 메시지 바이트에서 타임스탬프를 추출하는 헬퍼 함수.
 *        리틀 엔디안 바이트 순서를 가정합니다.
 *
 * @param ts_field - 버퍼 내 타임스탬프의 첫 번째 바이트에 대한 포인터.
 * @param ts - 추출된 타임스탬프를 저장할 uint32_t 변수에 대한 포인터.
 */
void resp_msg_get_ts(uint8_t *ts_field, uint32_t *ts)
{
    *ts = 0;
    // RESP_MSG_TS_LEN이 올바르게 정의되었는지 확인 (보통 32비트 타임스탬프의 경우 4)
    for (int i = 0; i < RESP_MSG_TS_LEN; i++)
    {
        *ts |= ((uint32_t)ts_field[i]) << (i * 8); // 각 바이트를 해당 위치에 OR 연산
    }
}


/*****************************************************************************************************************************************************
 * 노트:
 * (SS-TWR 원리, 안테나 지연, 프레임 등에 관한 기존 노트 적용됨)
 *
 * 14. 다층 측위 시스템 관련 노트:
 *   a) 이 시스템은 각 층에 여러 앵커(3개 이상 권장)를 배치하여 층별 위치를 계산합니다.
 *   b) 층 결정은 앵커들의 거리 측정치에 기반한 가중치 투표 메커니즘을 사용합니다.
 *   c) 삼각측량은 2D 기하학을 사용하여 X, Y 좌표만 계산하고, Z는 층 번호로부터 도출됩니다.
 *   d) 히스테리시스 로직이 층간 바운싱을 방지하기 위해 구현되어 있습니다.
 *
 * 15. JSON 데이터 출력 형식:
 *   {"x":<x좌표>,"y":<y좌표>,"z":<z좌표>,"floor":<층번호>,"valid":<유효성>,"timestamp":<타임스탬프>}
 *   이 데이터는 시리얼 포트를 통해 외부 시스템(예: 안드로이드 앱)으로 전송될 수 있습니다.
 *
 * 16. 실제 사용을 위한 설정:
 *   a) anchors 배열에서 앵커 ID와 좌표를 실제 설치된 값으로 교체해야 합니다.
 *   b) INITIATOR_ID_BYTE_7 및 INITIATOR_ID_BYTE_8을 고유한 태그 ID로 설정해야 합니다.
 *   c) FLOOR_HEIGHT 상수를 실제 층간 높이로 조정해야 합니다.
 *   d) 필요에 따라 FLOOR_VOTE_THRESHOLD를 조정하여 층 판별 민감도를 제어할 수 있습니다.
 *****************************************************************************************************************************************************/

// --- 더미/스텁 구현 (테스트/컴파일용, 필요시 사용) ---
#ifndef DW3000_API_PROVIDED // 더미 포함을 제어하는 플래그 사용
#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

// 플랫폼 함수 스텁
void test_run_info(unsigned char *s) { printf("정보: %s\n", (char*)s); }
void delay(uint32_t ms) { usleep(ms * 1000); } // delay를 usleep에 매핑

// 하드웨어 시간 함수 스텁
unsigned long millis() { return (unsigned long)(time(NULL) * 1000); }

#endif // DW3000_API_PROVIDED