#include "dw3000.h"
#include "dw3000_mac_802_15_4.h"

// === 추가된 코드 시작 ===

// 앵커 정보를 저장하기 위한 구조체
typedef struct {
  uint64_t address; // 앵커의 64비트 주소
  double x;         // 앵커의 X 좌표 (미터 단위)
  double y;         // 앵커의 Y 좌표 (미터 단위)
  double distance;  // 태그로부터 측정된 거리 (미터 단위)
  bool valid;       // 거리 측정 성공 여부
} AnchorData;

// 알려진 앵커 목록 (예시 값)
// 실제 환경에 맞게 앵커 주소와 좌표를 수정해야 합니다.
#define NUM_ANCHORS 4
AnchorData knownAnchors[NUM_ANCHORS] = {
  {0xA1A1A1A1A1A1A1A1, 0.0, 0.0, 0.0, false}, // 앵커 1 (원점)
  {0xA2A2A2A2A2A2A2A2, 5.0, 0.0, 0.0, false}, // 앵커 2 (x축 위)
  {0xA3A3A3A3A3A3A3A3, 0.0, 5.0, 0.0, false}, // 앵커 3 (y축 위)
  {0xA4A4A4A4A4A4A4A4, 5.0, 5.0, 0.0, false}  // 앵커 4
};

// 측정된 거리 데이터를 저장할 배열
AnchorData measuredAnchors[NUM_ANCHORS];

// 태그의 계산된 위치
double tagX = 0.0;
double tagY = 0.0;

// === 추가된 코드 끝 ===


#define APP_NAME "SS TWR AES INIT v1.0 - MultiAnchor" // 앱 이름 변경

// connection pins
const uint8_t PIN_RST = 27; // reset pin
const uint8_t PIN_IRQ = 34; // irq pin
const uint8_t PIN_SS = 4; // spi select pin

/* Sample of 802_15_4 frame*/
#if 0
mac_frame_802_15_4_format_t     mac_frame=
{
    /*
    * Frame control[0] = 0x09 = Data frame, security enabled, PEND not set, no ACK required, PANID compression set to zero (no PANID for source)
    * Frame control[1] = 0xEC = With seq num, no IEs, using extended address, frame ver 2 (IEEE Std 802.15.4)
    */
    .mhr_802_15_4.frame_ctrl[0]=0x09,
    .mhr_802_15_4.frame_ctrl[1]=0xEC,

    /* Sequence number initialize value*/
    .mhr_802_15_4.sequence_num=0x00,

    .mhr_802_15_4.dest_pan_id[0]=0x21,
    .mhr_802_15_4.dest_pan_id[1]=0x43,


    /* Set the Security Control field in the Auxiliary Security Header
    *  Security Control = 0xF:
    *                         Security level: 0x7 = MIC 16 (data confidentiality OFF, data authenticity Yes),
    *                         Key Identifier Mode: 0x1 = key determined from key index field,
    *                         Frame Counter Suppression: 0x0 = has the frame counter and the frame counter generates the nonce.
    *                         ASN in Nonce: 0x0 = frame counter is used to generate the nonce (CCM* nonce = SRC ADDR (8), Frame Counter (4) and Nonce Security Level (1) - set to 0x7 above)
    *  This means that format of the AUX header is Security Control (1 octet) + Fame Counter (4 octets) + Key Identifier (1 octet) = 6 octets
    */
    .mhr_802_15_4.aux_security.security_ctrl=0x0F,

};
#endif
mac_frame_802_15_4_format_t     mac_frame= {
  {
    {0x09, 0xEC},
    0x00,
    {0x21, 0x43},
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, // DEST ADDR - 루프에서 설정됨
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, // SRC ADDR - 아래에서 설정됨
    { 0x0F, {0x00, 0x00, 0x00, 0x00}, 0x00 }
  },
  0x00
};

#if 0
static dwt_aes_config_t aes_config=
{
    .key_load           = AES_KEY_Load,         // load the key into AES engine see Note 15 below
    .key_size           = AES_KEY_128bit,       // use 128bit key
    .key_src            = AES_KEY_Src_Register, // the key source is IC registers
    .aes_core_type      = AES_core_type_CCM,    // Use CCM core
    .aes_key_otp_type   = AES_key_RAM,
    .key_addr           = 0
};
#endif

static dwt_aes_config_t aes_config = {
  AES_key_RAM,
  AES_core_type_CCM,
  MIC_0, // MIC 크기는 루프에서 설정됨
  AES_KEY_Src_Register,
  AES_KEY_Load,
  0,
  AES_KEY_128bit,
  AES_Encrypt // 모드는 루프에서 설정됨
};

/* Initiator data */
// #define DEST_ADDR       0x1122334455667788 /* 더 이상 사용되지 않음 */
#define SRC_ADDR        0x8877665544332211 /* 이 태그(이니시에이터)의 주소 */
#define DEST_PAN_ID     0x4321             /* 이 예제에서 사용되는 PAN ID */

/* Default communication configuration. We use default non-STS DW mode. */
static dwt_config_t config =
{
    5,               /* Channel number. */
    DWT_PLEN_128,    /* Preamble length. Used in TX only. */
    DWT_PAC8,        /* Preamble acquisition chunk size. Used in RX only. */
    9,               /* TX preamble code. Used in TX only. */
    9,               /* RX preamble code. Used in RX only. */
    1,               /* 0 to use standard 8 symbol SFD, 1 to use non-standard 8 symbol, 2 for non-standard 16 symbol SFD and 3 for 4z 8 symbol SDF type */
    DWT_BR_6M8,      /* Data rate. */
    DWT_PHRMODE_STD, /* PHY header mode. */
    DWT_PHRRATE_STD, /* PHY header rate. */
    (129 + 8 - 8),   /* SFD timeout (preamble length + 1 + SFD length - PAC size). Used in RX only. */
    DWT_STS_MODE_OFF, /* STS disabled */
    DWT_STS_LEN_64,/* STS length see allowed values in Enum dwt_sts_lengths_e */
    DWT_PDOA_M0      /* PDOA mode off */
};


/* Optional keys according to the key index - In AUX security header*/
static dwt_aes_key_t    keys_options[NUM_OF_KEY_OPTIONS]=
{
    {0x00010203, 0x04050607, 0x08090A0B, 0x0C0D0E0F, 0x00000000, 0x00000000, 0x00000000, 0x00000000},
    {0x11223344, 0x55667788, 0x99AABBCC, 0xDDEEFF00, 0x00000000, 0x00000000, 0x00000000, 0x00000000},
    {0xFFEEDDCC, 0xBBAA9988, 0x77665544, 0x33221100, 0x00000000, 0x00000000, 0x00000000, 0x00000000}
};

/* Inter-ranging delay period, in milliseconds. */
#define RNG_DELAY_MS 1000 // 각 앵커 세트 측정 사이의 지연 시간

/* Default antenna delay values for 64 MHz PRF. See NOTE 2 below. */
#define TX_ANT_DLY 16385
#define RX_ANT_DLY 16385

/* MAC payload data of the frames used in the ranging process. See NOTE 3 below. */
/* Poll message from the initiator to the responder */
static uint8_t tx_poll_msg[] = {'P','o','l','l',' ','m','e','s','s','a','g','e'};
/* Response message to the initiator. The first 8 bytes are used for Poll RX time and Response TX time.*/
static uint8_t rx_resp_msg[] = {0,0,0,0,0,0,0,0,'R','e','s','p','o','n','s','e'};

#define START_RECEIVE_DATA_LOCATION     8   //MAC payload user data starts at index 8 (e.g. 'R' - in above response message)

/* Indexes to access some of the fields in the frames defined above. */
#define ALL_MSG_SN_IDX 2            //sequence number byte index in MHR
#define RESP_MSG_POLL_RX_TS_IDX 0   //index in the MAC payload for Poll RX time
#define RESP_MSG_RESP_TX_TS_IDX 4   //index in the MAC payload for Response TX time
#define RESP_MSG_TS_LEN 4

/* Note, the key index of 0 is forbidden to send as key index. Thus index 1 is the first.
 * This example uses this index for the key table for the encryption of initiator's data */
#define INITIATOR_KEY_INDEX     1

/* Buffer to store received response message.
 * Its size is adjusted to longest frame that this example code can handle. */
#define RX_BUF_LEN 127 /* The received frame cannot be bigger than 127 if STD PHR mode is used */
static uint8_t rx_buffer[RX_BUF_LEN];

/* Delay between frames, in UWB microseconds. See NOTE 1 below. */
#define POLL_TX_TO_RESP_RX_DLY_UUS 1720
/* Receive response timeout. See NOTE 5 below. */
#define RESP_RX_TIMEOUT_UUS 250 // 응답 타임아웃 시간 줄이기 (필요시 조정)

/* Hold copies of computed time of flight and distance here for reference so that it can be examined at a debug breakpoint. */
static double tof;      // 개별 측정용 임시 변수
static double distance; // 개별 측정용 임시 변수

/* Values for the PG_DELAY and TX_POWER registers reflect the bandwidth and power of the spectrum at the current
 * temperature. These values can be calibrated prior to taking reference measurements. See NOTE 2 below. */
extern dwt_txconfig_t txconfig_options;

static uint32_t   frame_cnt=0;  /* See Note 13 */
static uint8_t    seq_cnt=0x0A; /* Frame sequence number, incremented after each transmission. */
uint32_t          status_reg;
uint8_t           nonce[13];    /* 13-byte nonce used in this example as per IEEE802.15.4 */
dwt_aes_job_t   aes_job_tx,aes_job_rx;
int8_t          status;

// === 추가 함수 선언 ===
void sortAnchorsByDistance(AnchorData anchors[], int n);
void trilaterate(AnchorData anchor1, AnchorData anchor2, AnchorData anchor3, double* tag_x, double* tag_y);
// === 추가 함수 선언 끝 ===


void setup() {
  Serial.begin(115200); // 시리얼 통신 초기화
  // UART_init(); // 기존 함수가 Serial.begin을 포함하지 않으면 주석 처리 또는 제거
  test_run_info((unsigned char *)APP_NAME);

  /* Configure SPI rate, DW3000 supports up to 38 MHz */
  /* Reset DW IC */
  spiBegin(PIN_IRQ, PIN_RST);
  spiSelect(PIN_SS);

  delay(2); // Time needed for DW3000 to start up (transition from INIT_RC to IDLE_RC, or could wait for SPIRDY event)

  while (!dwt_checkidlerc()) // Need to make sure DW IC is in IDLE_RC before proceeding
  {
    Serial.println("IDLE FAILED"); // UART_puts 대신 Serial 사용
    while (1) ;
  }

  if (dwt_initialise(DWT_DW_INIT) == DWT_ERROR)
  {
    Serial.println("INIT FAILED"); // UART_puts 대신 Serial 사용
    while (1) ;
  }

  // Enabling LEDs here for debug so that for each TX the D1 LED will flash on DW3000 red eval-shield boards.
  dwt_setleds(DWT_LEDS_ENABLE | DWT_LEDS_INIT_BLINK);

  /* Configure DW IC. See NOTE 14 below. */
    if(dwt_configure(&config)) /* if the dwt_configure returns DWT_ERROR either the PLL or RX calibration has failed the host should reset the device */
    {
        test_run_info((unsigned char *)"CONFIG FAILED     ");
        while (1)
        { };
    }

    /* Configure the TX spectrum parameters (power, PG delay and PG count) */
    dwt_configuretxrf(&txconfig_options);

    /* Apply default antenna delay value. See NOTE 2 below. */
    dwt_setrxantennadelay(RX_ANT_DLY);
    dwt_settxantennadelay(TX_ANT_DLY);

    /* Set expected response's delay and timeout. See NOTE 1 and 5 below.
     * This example is paired with the SS-TWR responder and if delays/timings need to be changed
     * they must be changed in both to match. */
    dwt_setrxaftertxdelay(POLL_TX_TO_RESP_RX_DLY_UUS);
    dwt_setrxtimeout(RESP_RX_TIMEOUT_UUS);

    /* Next can enable TX/RX states output on GPIOs 5 and 6 to help debug */
    dwt_setlnapamode(DWT_LNA_ENABLE | DWT_PA_ENABLE);

    /*Configure the TX and RX AES jobs, the TX job is used to encrypt the Poll message,
     * the RX job is used to decrypt the Response message */
    aes_job_tx.mode        = AES_Encrypt;     /* this is encryption job */
    aes_job_tx.src_port    = AES_Src_Tx_buf;  /* dwt_do_aes will take plain text to the TX buffer */
    aes_job_tx.dst_port    = AES_Dst_Tx_buf;  /* dwt_do_aes will replace the original plain text TX buffer with encrypted one */
    aes_job_tx.nonce       = nonce;          /* pointer to the nonce structure*/
    aes_job_tx.header      = (uint8_t *)MHR_802_15_4_PTR(&mac_frame);/* plain-text header which will not be encrypted */
    aes_job_tx.header_len  = MAC_FRAME_HEADER_SIZE(&mac_frame);
    aes_job_tx.payload     = tx_poll_msg;    /* payload to be encrypted */
    aes_job_tx.payload_len = sizeof(tx_poll_msg); /* size of payload to be encrypted */

    aes_job_rx.mode        = AES_Decrypt;      /* this is decryption job */
    aes_job_rx.src_port    = AES_Src_Rx_buf_0; /* The source of the data to be decrypted is the IC RX buffer */
    aes_job_rx.dst_port    = AES_Dst_Rx_buf_0; /* Decrypt the encrypted data to the IC RX buffer : this will destroy original RX frame */
    aes_job_rx.header_len  = aes_job_tx.header_len;
    aes_job_rx.header      = aes_job_tx.header;/* plain-text header which will not be encrypted */
    aes_job_rx.payload     = rx_buffer;        /* pointer to where the decrypted data will be copied to when read from the IC*/

}

void loop() {
  // 측정 데이터 초기화 (매 루프 시작 시)
  for (int i = 0; i < NUM_ANCHORS; i++) {
    measuredAnchors[i] = knownAnchors[i]; // 앵커 정보 복사 (주소, 좌표)
    measuredAnchors[i].distance = 9999.0; // 매우 큰 값으로 초기화 (정렬 용이)
    measuredAnchors[i].valid = false;     // 유효성 플래그 초기화
  }

  Serial.println("--- Starting Ranging Cycle ---");

  // 모든 알려진 앵커에 대해 거리 측정 시도
  for (int i = 0; i < NUM_ANCHORS; i++) {
    uint64_t current_dest_addr = knownAnchors[i].address;

    Serial.print("Ranging with Anchor ");
    Serial.print(i + 1);
    Serial.print(" (");
    // 주소 출력 (간단히 상위 16비트만)
    Serial.print((uint16_t)(current_dest_addr >> 48), HEX);
    Serial.print("...): ");

    /* 사용할 키 프로그래밍 */
    dwt_set_keyreg_128(&keys_options[INITIATOR_KEY_INDEX - 1]);
    /* 프레임에 사용할 키 인덱스 설정 */
    MAC_FRAME_AUX_KEY_IDENTIFY_802_15_4(&mac_frame) = INITIATOR_KEY_INDEX;

    /* MHR을 올바른 SRC 및 DEST 주소로 업데이트하고 13바이트 nonce 생성 */
    mac_frame_set_pan_ids_and_addresses_802_15_4(&mac_frame, DEST_PAN_ID, current_dest_addr, SRC_ADDR);
    mac_frame_get_nonce(&mac_frame, nonce);

    /* AES TX 설정 */
    aes_job_tx.mic_size = mac_frame_get_aux_mic_size(&mac_frame);
    aes_config.mode = AES_Encrypt;
    aes_config.mic = dwt_mic_size_from_bytes(aes_job_tx.mic_size);
    dwt_configure_aes(&aes_config);

    /* AES 암호화 수행 */
    status = dwt_do_aes(&aes_job_tx, aes_config.aes_core_type);
    if (status < 0) {
      Serial.println(" AES length error");
      continue; // 다음 앵커로 넘어감
    } else if (status & AES_ERRORS) {
      Serial.println(" ERROR AES");
      continue; // 다음 앵커로 넘어감
    }

    /* 전송 프레임 설정 및 전송 시작 */
    dwt_writetxfctrl(aes_job_tx.header_len + aes_job_tx.payload_len + aes_job_tx.mic_size + FCS_LEN, 0, 1);
    dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED);

    /* 응답 수신 대기 (성공, 타임아웃 또는 오류) */
    while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG_BIT_MASK | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR))) {
    };

    /* 시퀀스 번호 및 프레임 카운터 증가 */
    MAC_FRAME_SEQ_NUM_802_15_4(&mac_frame) = ++seq_cnt;
    mac_frame_update_aux_frame_cnt(&mac_frame, ++frame_cnt);

    /* 응답을 성공적으로 수신한 경우 */
    if (status_reg & SYS_STATUS_RXFCG_BIT_MASK) {
      uint32_t frame_len;

      /* 상태 레지스터 클리어 */
      dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG_BIT_MASK);

      /* 수신된 데이터 길이 읽기 */
      frame_len = dwt_read32bitreg(RX_FINFO_ID) & RXFLEN_MASK;

      /* AES RX 설정 */
      aes_config.mode = AES_Decrypt;
      PAYLOAD_PTR_802_15_4(&mac_frame) = rx_buffer; /* MAC 페이로드 포인터 설정 */

      /* AES 복호화 시도 (기대하는 소스 주소 확인 포함) */
      // rx_aes_802_15_4 함수는 내부적으로 수신된 프레임의 SRC 주소가 예상하는 DEST 주소(current_dest_addr)와 일치하는지 확인합니다.
      status = rx_aes_802_15_4(&mac_frame, frame_len, &aes_job_rx, sizeof(rx_buffer), keys_options, current_dest_addr, SRC_ADDR, &aes_config);

      if (status == AES_RES_OK) {
        /* 페이로드 내용 확인 (타임스탬프 제외) */
        if (memcmp(&rx_buffer[START_RECEIVE_DATA_LOCATION], &rx_resp_msg[START_RECEIVE_DATA_LOCATION],
                   aes_job_rx.payload_len - START_RECEIVE_DATA_LOCATION) == 0)
        {
          uint32_t poll_tx_ts, resp_rx_ts, poll_rx_ts, resp_tx_ts;
          int32_t rtd_init, rtd_resp;
          float clockOffsetRatio;

          /* 타임스탬프 추출 */
          poll_tx_ts = dwt_readtxtimestamplo32();
          resp_rx_ts = dwt_readrxtimestamplo32();

          /* 클럭 오프셋 계산 */
          clockOffsetRatio = ((float)dwt_readclockoffset()) / (uint32_t)(1 << 26);

          /* 응답 메시지에서 타임스탬프 추출 */
          resp_msg_get_ts(&rx_buffer[RESP_MSG_POLL_RX_TS_IDX], &poll_rx_ts);
          resp_msg_get_ts(&rx_buffer[RESP_MSG_RESP_TX_TS_IDX], &resp_tx_ts);

          /* ToF 및 거리 계산 */
          rtd_init = resp_rx_ts - poll_tx_ts;
          rtd_resp = resp_tx_ts - poll_rx_ts;
          tof = ((rtd_init - rtd_resp * (1 - clockOffsetRatio)) / 2.0) * DWT_TIME_UNITS;
          distance = tof * SPEED_OF_LIGHT;

          /* 결과 저장 */
          measuredAnchors[i].distance = distance;
          measuredAnchors[i].valid = true;
          Serial.print(" Success! Dist: ");
          Serial.print(distance);
          Serial.println(" m");

        } else {
          Serial.println(" Payload mismatch.");
          // 유효하지 않은 페이로드, valid는 false로 유지됨
        }
      } else {
        // 복호화 실패 또는 다른 오류 처리 (예: 프레임 무시)
         Serial.print(" AES Decrypt Error/Ignore (Status: ");
         Serial.print(status);
         Serial.println(")");
         // 오류 발생 시 valid는 false로 유지됨
      }
    } else {
      /* 타임아웃 또는 수신 오류 */
      dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);
      Serial.println(" RX Timeout/Error.");
      // 타임아웃/오류 시 valid는 false로 유지됨
    }

    // 다음 앵커 시도 전 짧은 지연 (선택 사항, UWB 통신 안정성에 도움 줄 수 있음)
    delay(50);

  } // End of anchor loop

  Serial.println("--- Ranging Cycle Complete ---");

  // --- 정렬 및 삼각측량 로직 시작 ---

  // 1. 거리 기준으로 앵커 정렬 (삽입 정렬 사용)
  sortAnchorsByDistance(measuredAnchors, NUM_ANCHORS);

  // 2. 유효하고 가장 가까운 앵커 3개 찾기
  AnchorData closestAnchors[3];
  int validCount = 0;
  for (int i = 0; i < NUM_ANCHORS && validCount < 3; i++) {
    if (measuredAnchors[i].valid) {
      closestAnchors[validCount++] = measuredAnchors[i];
    }
  }

  // 3. 유효한 앵커가 3개 이상인지 확인 후 삼각측량 수행
  if (validCount >= 3) {
    Serial.println("Found 3 closest valid anchors:");
    for(int k=0; k<3; k++){
        Serial.print("  Anchor Addr: ...");
        Serial.print((uint16_t)(closestAnchors[k].address >> 48), HEX);
        Serial.print(" ("); Serial.print(closestAnchors[k].x);
        Serial.print(", "); Serial.print(closestAnchors[k].y);
        Serial.print("), Dist: "); Serial.println(closestAnchors[k].distance);
    }

    // 삼각측량 함수 호출
    trilaterate(closestAnchors[0], closestAnchors[1], closestAnchors[2], &tagX, &tagY);

    // 계산된 위치 출력
    Serial.print("Calculated Tag Position: (");
    Serial.print(tagX);
    Serial.print(", ");
    Serial.print(tagY);
    Serial.println(")");

  } else {
    Serial.print("Could not find 3 valid anchors for trilateration (Found: ");
    Serial.print(validCount);
    Serial.println(")");
    // 위치 계산 불가, 이전 값 유지 또는 기본값 설정
    tagX = NAN; // Not a Number
    tagY = NAN;
  }

  // --- 정렬 및 삼각측량 로직 끝 ---


  /* 다음 거리 측정 주기까지 대기 */
  delay(RNG_DELAY_MS); // Arduino의 delay 사용 (Sleep 함수 대신)
}


// === 추가 함수 정의 ===

/**
 * @brief 삽입 정렬을 사용하여 AnchorData 배열을 거리(distance) 기준으로 오름차순 정렬합니다.
 *        유효하지 않은(valid=false) 앵커는 배열 뒤쪽으로 보냅니다.
 *        시간 복잡도: 평균 O(n^2), 최선 O(n)
 * @param anchors 정렬할 AnchorData 배열
 * @param n 배열의 크기
 */
void sortAnchorsByDistance(AnchorData anchors[], int n) {
  int i, j;
  AnchorData key;
  for (i = 1; i < n; i++) {
    key = anchors[i];
    j = i - 1;

    // 유효한 앵커는 유효하지 않은 앵커보다 항상 앞에 오도록 처리
    // 또는 두 앵커 모두 유효할 경우 거리가 더 짧은 앵커가 앞에 오도록 처리
    while (j >= 0 && (!anchors[j].valid || (key.valid && anchors[j].distance > key.distance))) {
      anchors[j + 1] = anchors[j];
      j = j - 1;
    }
    // 유효하지 않은 앵커들 사이에서는 순서 유지 (상대적 안정성)
    // 또는 key가 유효하지 않고 anchors[j]가 유효한 경우 key를 뒤로 보냄
     while (j >= 0 && !key.valid && anchors[j].valid) {
         anchors[j + 1] = anchors[j];
         j = j - 1;
     }

    anchors[j + 1] = key;
  }
}


/**
 * @brief 2D 평면에서 세 앵커의 위치와 각 앵커까지의 거리를 이용하여 태그의 위치를 계산합니다 (삼각측량).
 *        참고: https://en.wikipedia.org/wiki/Trilateration#Mathematical_basis_and_solution
 *             (여기서는 단순화된 2D 버전을 사용합니다)
 * @param anchor1 첫 번째 앵커 데이터
 * @param anchor2 두 번째 앵커 데이터
 * @param anchor3 세 번째 앵커 데이터
 * @param tag_x 계산된 태그의 X 좌표를 저장할 포인터
 * @param tag_y 계산된 태그의 Y 좌표를 저장할 포인터
 */
void trilaterate(AnchorData anchor1, AnchorData anchor2, AnchorData anchor3, double* tag_x, double* tag_y) {
    double x1 = anchor1.x, y1 = anchor1.y, r1 = anchor1.distance;
    double x2 = anchor2.x, y2 = anchor2.y, r2 = anchor2.distance;
    double x3 = anchor3.x, y3 = anchor3.y, r3 = anchor3.distance;

    // 중간 계산 변수들
    double A = 2 * (x2 - x1);
    double B = 2 * (y2 - y1);
    double C = r1*r1 - r2*r2 - x1*x1 + x2*x2 - y1*y1 + y2*y2;
    double D = 2 * (x3 - x2);
    double E = 2 * (y3 - y2);
    double F = r2*r2 - r3*r3 - x2*x2 + x3*x3 - y2*y2 + y3*y3;

    // 분모 계산 (0이 되는 경우 - 앵커들이 일직선 상에 있는 등 - 계산 불가)
    double denominator = (A * E - B * D);

    if (abs(denominator) < 1e-6) { // 매우 작은 값으로 0에 가까운지 확인
        Serial.println("Trilateration failed: Denominator is close to zero (anchors might be collinear).");
        *tag_x = NAN; // 계산 실패 시 Not a Number 반환
        *tag_y = NAN;
        return;
    }

    // 태그 좌표 계산
    *tag_x = (C * E - F * B) / denominator;
    *tag_y = (C * D - A * F) / (B * D - A * E); // 분모 부호 주의하여 계산 (위와 동일하게 A*E - B*D 사용)
    *tag_y = (A * F - C * D) / denominator; // 위 식과 동일

}


// 기존 유틸리티 함수 (필요시 유지)
// 예: test_run_info, resp_msg_get_ts 등


/* Helper function to print informational messages */
void test_run_info(unsigned char *message)
{
    Serial.print(message); // Use Serial.print for output
    Serial.print("
");
}

/* Helper function to extract timestamp from response message */
void resp_msg_get_ts(uint8_t *ts_field, uint32_t *ts)
{
    int i;
    *ts = 0;
    for (i = 0; i < RESP_MSG_TS_LEN; i++)
    {
        *ts += ts_field[i] << (i * 8);
    }
}


// 기존 노트 주석 (필요시 유지)
/*****************************************************************************************************************************************************
 * NOTES:
 * (기존 노트 내용 생략 - 필요시 복원)
 * 15. When CCM core type is used, AES_KEY_Load needs to be set prior to each encryption/decryption operation, even if the AES KEY used has not changed.
 * 16. (추가) 이 코드는 2D 삼각측량을 가정합니다. 3D 측량을 위해서는 앵커 4개와 Z 좌표, 수정된 삼각측량 알고리즘이 필요합니다.
 * 17. (추가) 삽입 정렬은 구현이 간단하지만 앵커 수가 많아지면 성능이 저하될 수 있습니다. 더 효율적인 정렬 (예: 퀵 정렬, 병합 정렬)을 고려할 수 있습니다. O(n log n)
 * 18. (추가) 삼각측량은 측정 오차에 민감합니다. 칼만 필터 등 필터링 기법을 적용하여 위치 추정 정확도를 향상시킬 수 있습니다.
 * 19. (추가) `test_run_info` 함수와 `resp_msg_get_ts` 함수는 원본 예제에 있었던 것으로 가정하고 유지했습니다. 만약 없다면 해당 함수 정의가 필요합니다. (위 코드에 추가됨)
 * 20. (추가) `delay()` 함수 대신 `millis()`를 사용한 비차단 방식으로 구현하면 다른 작업을 동시에 수행하는 데 유리합니다.
 * 21. (추가) 실제 환경에서는 앵커 주소 (`knownAnchors`)와 좌표를 정확하게 설정해야 합니다.
 *****************************************************************************************************************************************************/
