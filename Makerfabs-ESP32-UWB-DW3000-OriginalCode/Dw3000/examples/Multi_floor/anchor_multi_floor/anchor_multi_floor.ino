#include "dw3000.h"

#define APP_NAME "Multi-Floor UWB Anchor v1.0"

// 연결 핀 정의
const uint8_t PIN_RST = 27; // 리셋 핀
const uint8_t PIN_IRQ = 34; // IRQ 핀
const uint8_t PIN_SS = 4;   // SPI 선택 핀

// !!! 중요: 각 앵커마다 올바른 ID로 변경 필요 !!!
// 층별 앵커 ID 범위
// 1층 앵커: 0xA001~0xA999 (예: 0xA001, 0xA002, ...)
// 2층 앵커: 0xB001~0xB999 (예: 0xB001, 0xB002, ...)
// 3층 앵커: 0xC001~0xC999 (예: 0xC001, 0xC002, ...)
#define ANCHOR_ID 0xA001  // !!! 각 앵커마다 변경 필요 !!! (기본값: 1층 첫번째 앵커)

// 앵커 위치 설정 (미터 단위)
// !!! 중요: 각 앵커마다 올바른 좌표로 변경 필요 !!!
#define ANCHOR_X 0.0  // X 좌표
#define ANCHOR_Y 0.0  // Y 좌표
#define ANCHOR_Z 1.5  // Z 좌표 (높이)

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

/* 64 MHz PRF에 대한 기본 안테나 지연 값. */
#define TX_ANT_DLY 16385
#define RX_ANT_DLY 16385

/* 거리 측정 프로세스에서 사용되는 프레임. */
/* Poll 메시지 템플릿 (Tag -> Anchor) */
static uint8_t rx_poll_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 0, 0, 0, 0, 0xE0, 0, 0}; 
/* Response 메시지 (Anchor -> Tag) */
static uint8_t tx_resp_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 0, 0, 0, 0, 0xE1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

/* 메시지의 공통 부분 길이 (기능 코드까지 포함). */
#define ALL_MSG_COMMON_LEN 10
/* 위에서 정의된 프레임의 일부 필드에 접근하기 위한 인덱스. */
#define ALL_MSG_SN_IDX 2 // 시퀀스 번호 인덱스
#define ALL_MSG_DEST_ADDR_IDX 5 // 목적지 주소 인덱스
#define ALL_MSG_SRC_ADDR_IDX 7 // 소스 주소 인덱스
#define RESP_MSG_POLL_RX_TS_IDX 10 // 응답 메시지 내 Poll 수신 타임스탬프 인덱스
#define RESP_MSG_RESP_TX_TS_IDX 14 // 응답 메시지 내 응답 송신 타임스탬프 인덱스
#define RESP_MSG_TS_LEN 4 // 타임스탬프 길이 (바이트)

/* 수신된 프레임을 저장할 버퍼. */
#define RX_BUF_LEN 12
static uint8_t rx_buffer[RX_BUF_LEN];

/* 디버그 중단점에서 검사할 수 있도록 상태 레지스터 상태의 복사본을 여기에 보관. */
static uint32_t status_reg = 0;

/* RX 타임아웃 값 */
#define RX_TIMEOUT_LONG_MS 3000 // 초기 RX 타임아웃 (3초)
#define RX_TIMEOUT_SHORT_US 400 // 수신 타임아웃 (마이크로초) - 짧은 버전

/* PG_DELAY 및 TX_POWER 레지스터 값 */
extern dwt_txconfig_t txconfig_options;

/* Anchor의 ID 바이트를 응답에 포함시키기 위한 설정 */
#define ANCHOR_ID_BYTE_7 (ANCHOR_ID & 0xFF) // 하위 바이트
#define ANCHOR_ID_BYTE_8 ((ANCHOR_ID >> 8) & 0xFF) // 상위 바이트

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

  /* TX/RX LED 활성화 */
  dwt_setlnapamode(DWT_LNA_ENABLE | DWT_PA_ENABLE);

  // 응답 메시지에 앵커 ID 설정
  tx_resp_msg[ALL_MSG_SRC_ADDR_IDX] = ANCHOR_ID_BYTE_7;
  tx_resp_msg[ALL_MSG_SRC_ADDR_IDX + 1] = ANCHOR_ID_BYTE_8;

  // 앵커 정보 출력
  char info_str[80];
  snprintf(info_str, sizeof(info_str), "앵커 ID: 0x%04X, 위치: (%.2f, %.2f, %.2f)",
          ANCHOR_ID, ANCHOR_X, ANCHOR_Y, ANCHOR_Z);
  test_run_info((unsigned char *)info_str);

  // 백그라운드에서 수신 가능한 프레임 계속 대기 시작
  dwt_setpreambledetecttimeout(0); // 없음 - 계속 대기
  dwt_setrxtimeout(0); // 무제한 수신 대기
  dwt_rxenable(DWT_START_RX_IMMEDIATE);

  test_run_info((unsigned char *)"설정 완료. 대기 상태로 진입...");
}

void loop() {
  // Poll 메시지 수신 대기
  if (dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_RXFCG_BIT_MASK) {
    uint32_t frame_len;

    /* DW IC 상태 레지스터에서 좋은 RX 프레임 이벤트 클리어 */
    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG_BIT_MASK);

    /* 수신된 프레임 길이 읽기 */
    frame_len = dwt_read32bitreg(RX_FINFO_ID) & RXFLEN_MASK;
    if (frame_len <= RX_BUF_LEN) {
      /* 프레임 데이터를 로컬 버퍼로 읽기 */
      dwt_readrxdata(rx_buffer, frame_len, 0);

      /* 프레임 검증: Poll 메시지이고 이 앵커를 대상으로 하는지 확인 */
      if ((rx_buffer[9] == 0xE0) && // Poll 메시지 타입
          (rx_buffer[ALL_MSG_DEST_ADDR_IDX] == ANCHOR_ID_BYTE_7) && // 목적지 주소 확인
          (rx_buffer[ALL_MSG_DEST_ADDR_IDX + 1] == ANCHOR_ID_BYTE_8)) {
        
        uint8_t initiator_addr_byte_7 = rx_buffer[ALL_MSG_SRC_ADDR_IDX];
        uint8_t initiator_addr_byte_8 = rx_buffer[ALL_MSG_SRC_ADDR_IDX + 1];
        uint32_t poll_rx_ts = dwt_readrxtimestamplo32();

        // Poll 수신에 대한 디버그 메시지
        char poll_str[50];
        snprintf(poll_str, sizeof(poll_str), "Poll 수신: 0x%02X%02X -> 0x%04X",
                initiator_addr_byte_8, initiator_addr_byte_7, ANCHOR_ID);
        test_run_info((unsigned char *)poll_str);

        // 응답 메시지 준비
        tx_resp_msg[ALL_MSG_SN_IDX] = rx_buffer[ALL_MSG_SN_IDX]; // 시퀀스 번호 유지
        
        // 목적지 주소를 태그(Initiator) 주소로 설정
        tx_resp_msg[ALL_MSG_DEST_ADDR_IDX] = initiator_addr_byte_7;
        tx_resp_msg[ALL_MSG_DEST_ADDR_IDX + 1] = initiator_addr_byte_8;

        // 타임스탬프 삽입
        uint32_t resp_tx_time = (poll_rx_ts + (RX_TIMEOUT_SHORT_US * UUS_TO_DWT_TIME)) >> 8;
        uint32_t resp_tx_ts = (resp_tx_time & 0xFFFFFFFE) << 8;

        resp_tx_time = resp_tx_time + TX_ANT_DLY;

        // 응답 메시지에 Poll 수신 타임스탬프 삽입
        uint8_t *poll_rx_ts_ptr = (uint8_t *)&poll_rx_ts;
        for (int i = 0; i < RESP_MSG_TS_LEN; i++) {
          tx_resp_msg[RESP_MSG_POLL_RX_TS_IDX + i] = poll_rx_ts_ptr[i];
        }

        // 응답 메시지에 응답 전송 타임스탬프 삽입
        uint8_t *resp_tx_ts_ptr = (uint8_t *)&resp_tx_ts;
        for (int i = 0; i < RESP_MSG_TS_LEN; i++) {
          tx_resp_msg[RESP_MSG_RESP_TX_TS_IDX + i] = resp_tx_ts_ptr[i];
        }

        // 응답 메시지 전송 준비
        dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS_BIT_MASK);
        dwt_writetxdata(sizeof(tx_resp_msg), tx_resp_msg, 0);
        dwt_writetxfctrl(sizeof(tx_resp_msg), 0, 1);

        // 응답 메시지 전송
        dwt_setrxaftertxdelay(0);
        dwt_setrxtimeout(0);

        // 응답 전송
        dwt_starttx(DWT_START_TX_DELAYED | DWT_RESPONSE_EXPECTED);

        // 응답 전송 대기
        while (!(dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS_BIT_MASK)) {
          yield(); // CPU 양보 (다른 작업 처리 가능)
        }

        // 응답 전송 완료 알림
        test_run_info((unsigned char *)"응답 전송 완료");

        // 다시 수신 모드로 전환
        dwt_rxenable(DWT_START_RX_IMMEDIATE);
      } else {
        // 이 앵커를 대상으로 하지 않는 메시지 무시
        dwt_rxenable(DWT_START_RX_IMMEDIATE);
      }
    } else {
      // 프레임이 너무 긴 경우 처리
      test_run_info((unsigned char *)"프레임 너무 긺 - 무시");
      dwt_rxenable(DWT_START_RX_IMMEDIATE);
    }
  } else if (dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_ALL_RX_ERR) {
    // 수신 오류 처리
    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_ERR);
    dwt_rxenable(DWT_START_RX_IMMEDIATE);
  }

  // LED 상태 업데이트 또는 다른 유지 관리 작업 수행 가능
  yield(); // CPU 양보 (다른 작업 처리 가능)
}