#include "dw3000.h"
#include "BluetoothSerial.h"

#define PIN_RST 27
#define PIN_IRQ 34
#define PIN_SS 4

#define RNG_DELAY_MS 100
#define TX_ANT_DLY 16385
#define RX_ANT_DLY 16385
#define ALL_MSG_COMMON_LEN 10
#define ALL_MSG_SN_IDX 2
#define RESP_MSG_POLL_RX_TS_IDX 10
#define RESP_MSG_RESP_TX_TS_IDX 14
#define RESP_MSG_TS_LEN 4
#define POLL_TX_TO_RESP_RX_DLY_UUS 240
#define RESP_RX_TIMEOUT_UUS 400

BluetoothSerial SerialBT;

/* Default communication configuration. We use default non-STS DW mode. */
static dwt_config_t config = {
    5,                /* Channel number. */
    DWT_PLEN_128,     /* Preamble length. Used in TX only. */
    DWT_PAC8,         /* Preamble acquisition chunk size. Used in RX only. */
    9,                /* TX preamble code. Used in TX only. */
    9,                /* RX preamble code. Used in RX only. */
    1,                /* 0 to use standard 8 symbol SFD, 1 to use non-standard 8 symbol, 2 for non-standard 16 symbol SFD and 3 for 4z 8 symbol SDF type */
    DWT_BR_6M8,       /* Data rate. */
    DWT_PHRMODE_STD,  /* PHY header mode. */
    DWT_PHRRATE_STD,  /* PHY header rate. */
    (129 + 8 - 8),    /* SFD timeout (preamble length + 1 + SFD length - PAC size). Used in RX only. */
    DWT_STS_MODE_OFF, /* STS disabled */
    DWT_STS_LEN_64,   /* STS length see allowed values in Enum dwt_sts_lengths_e */
    DWT_PDOA_M0       /* PDOA mode off */
};

static uint8_t tx_poll_msg_t0[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'T', 'A', 'G', '0', 0xE0, 0, 0};
static uint8_t rx_resp_msg_a0[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'A', 'N', 'C', '0', 0xE1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
static uint8_t rx_resp_msg_a1[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'A', 'N', 'C', '1', 0xE1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
static uint8_t rx_resp_msg_a2[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'A', 'N', 'C', '2', 0xE1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
static uint8_t rx_resp_msg_a3[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'A', 'N', 'C', '3', 0xE1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
static uint8_t frame_seq_nb = 0;
static uint8_t rx_buffer[20];
static uint32_t status_reg = 0;
static double tof;
static double distance;
extern dwt_txconfig_t txconfig_options;

// Fixed locations of the 
float fx0 = 0.0, fy0 = 0.0, a0 = 0.0;  // random number used to disqualify uwb anchor point
float fx1 = 0.0, fy1 = 4.23, a1 = 0.0;
float fx2 = 7.04, fy2 = 4.23, a2 = 0.0;
float fx3 = 0.0, fy3 = 0.0, a3 = 0.0;
float resultArray[12];

// 정렬용 구조체
typedef struct {
    float a;  // a 값
    float fx; // fx 값
    float fy; // fy 값
} Value;

Value values[4] = {
          {a0, fx0, fy0},
          {a1, fx1, fy1},
          {a2, fx2, fy2},
          {a3, fx3, fy3}
      };

void setup()
{
  SerialBT.begin("ESP32_BT_Module"); // 블루투스 장치 이름 설정
  Serial.begin(115200); // 디버깅용 시리얼 통신 시작
  
  UART_init();

  spiBegin(PIN_IRQ, PIN_RST);
  spiSelect(PIN_SS);

  delay(2); // Time needed for DW3000 to start up (transition from INIT_RC to IDLE_RC, or could wait for SPIRDY event)

  while (!dwt_checkidlerc()) // Need to make sure DW IC is in IDLE_RC before proceeding
  {
    UART_puts("IDLE FAILED\r\n");
    while (1)
      ;
  }

  if (dwt_initialise(DWT_DW_INIT) == DWT_ERROR)
  {
    UART_puts("INIT FAILED\r\n");
    while (1)
      ;
  }

  // Enabling LEDs here for debug so that for each TX the D1 LED will flash on DW3000 red eval-shield boards.
  dwt_setleds(DWT_LEDS_ENABLE | DWT_LEDS_INIT_BLINK);

  /* Configure DW IC. See NOTE 6 below. */
  if (dwt_configure(&config)) // if the dwt_configure returns DWT_ERROR either the PLL or RX calibration has failed the host should reset the device
  {
    UART_puts("CONFIG FAILED\r\n");
    while (1)
      ;
  }

  /* Configure the TX spectrum parameters (power, PG delay and PG count) */
  dwt_configuretxrf(&txconfig_options);

  /* Apply default antenna delay value. See NOTE 2 below. */
  dwt_setrxantennadelay(RX_ANT_DLY);
  dwt_settxantennadelay(TX_ANT_DLY);

  /* Set expected response's delay and timeout. See NOTE 1 and 5 below.
   * As this example only handles one incoming frame with always the same delay and timeout, those values can be set here once for all. */
  dwt_setrxaftertxdelay(POLL_TX_TO_RESP_RX_DLY_UUS);
  dwt_setrxtimeout(RESP_RX_TIMEOUT_UUS);

  /* Next can enable TX/RX states output on GPIOs 5 and 6 to help debug, and also TX/RX LEDs
   * Note, in real low power applications the LEDs should not be used. */
  dwt_setlnapamode(DWT_LNA_ENABLE | DWT_PA_ENABLE);

  //Serial.println("Tag T0 Setup Complete...");

}

void loop()
{
  //***********************Anchor A0******************************************
  //Serial.println("Probing Anchor A0...");
  //a0 = 12.3;
  // probe_anchor(tx_poll_msg_t0,rx_resp_msg_a0);
  /* Write frame data to DW IC and prepare transmission. See NOTE 7 below. */
  tx_poll_msg_t0[ALL_MSG_SN_IDX] = frame_seq_nb;
  dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS_BIT_MASK);
  dwt_writetxdata(sizeof(tx_poll_msg_t0), tx_poll_msg_t0, 0); /* Zero offset in TX buffer. */
  dwt_writetxfctrl(sizeof(tx_poll_msg_t0), 0, 1);          /* Zero offset in TX buffer, ranging. */

  /* Start transmission, indicating that a response is expected so that reception is enabled automatically after the frame is sent and the delay
   * set by dwt_setrxaftertxdelay() has elapsed. */
  dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED);

  /* We assume that the transmission is achieved correctly, poll for reception of a frame or error/timeout. See NOTE 8 below. */
  while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG_BIT_MASK | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR)))
  {
  };

  /* Increment frame sequence number after transmission of the poll message (modulo 256). */
  frame_seq_nb++;

  if (status_reg & SYS_STATUS_RXFCG_BIT_MASK)
  {
    uint32_t frame_len;

    /* Clear good RX frame event in the DW IC status register. */
    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG_BIT_MASK);

    /* A frame has been received, read it into the local buffer. */
    frame_len = dwt_read32bitreg(RX_FINFO_ID) & RXFLEN_MASK;
    if (frame_len <= sizeof(rx_buffer))
    {
      dwt_readrxdata(rx_buffer, frame_len, 0);

      /* Check that the frame is the expected response from the companion "SS TWR responder" example.
       * As the sequence number field of the frame is not relevant, it is cleared to simplify the validation of the frame. */
      rx_buffer[ALL_MSG_SN_IDX] = 0;
      if (memcmp(rx_buffer, rx_resp_msg_a0, ALL_MSG_COMMON_LEN) == 0)
      {
        uint32_t poll_tx_ts, resp_rx_ts, poll_rx_ts, resp_tx_ts;
        int32_t rtd_init, rtd_resp;
        float clockOffsetRatio;

        /* Retrieve poll transmission and response reception timestamps. See NOTE 9 below. */
        poll_tx_ts = dwt_readtxtimestamplo32();
        resp_rx_ts = dwt_readrxtimestamplo32();

        /* Read carrier integrator value and calculate clock offset ratio. See NOTE 11 below. */
        clockOffsetRatio = ((float)dwt_readclockoffset()) / (uint32_t)(1 << 26);

        /* Get timestamps embedded in response message. */
        resp_msg_get_ts(&rx_buffer[RESP_MSG_POLL_RX_TS_IDX], &poll_rx_ts);
        resp_msg_get_ts(&rx_buffer[RESP_MSG_RESP_TX_TS_IDX], &resp_tx_ts);

        /* Compute time of flight and distance, using clock offset ratio to correct for differing local and remote clock rates */
        rtd_init = resp_rx_ts - poll_tx_ts;
        rtd_resp = resp_tx_ts - poll_rx_ts;

        tof = ((rtd_init - rtd_resp * (1 - clockOffsetRatio)) / 2.0) * DWT_TIME_UNITS;
        distance = tof * SPEED_OF_LIGHT;

        /* Display computed distance on LCD. */
        // snprintf(dist_str, sizeof(dist_str), "DIST: %3.2f m", distance);
        snprintf(dist_str, sizeof(dist_str), "%3.2f", distance);
        test_run_info((unsigned char *)dist_str);

        a0 = atof(dist_str);
        delay(RNG_DELAY_MS);
      }
    }
  }
  else
  {
    /* Clear RX error/timeout events in the DW IC status register. */
    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);
  }


  //***********************Anchor A1******************************************
  //Serial.println("Probing Anchor A1...");
  //a1 = 12.3;
  // probe_anchor(tx_poll_msg_t0,rx_resp_msg_a1);
  /* Write frame data to DW IC and prepare transmission. See NOTE 7 below. */
  tx_poll_msg_t0[ALL_MSG_SN_IDX] = frame_seq_nb;
  dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS_BIT_MASK);
  dwt_writetxdata(sizeof(tx_poll_msg_t0), tx_poll_msg_t0, 0); /* Zero offset in TX buffer. */
  dwt_writetxfctrl(sizeof(tx_poll_msg_t0), 0, 1);          /* Zero offset in TX buffer, ranging. */

  /* Start transmission, indicating that a response is expected so that reception is enabled automatically after the frame is sent and the delay
   * set by dwt_setrxaftertxdelay() has elapsed. */
  dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED);

  /* We assume that the transmission is achieved correctly, poll for reception of a frame or error/timeout. See NOTE 8 below. */
  while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG_BIT_MASK | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR)))
  {
  };

  /* Increment frame sequence number after transmission of the poll message (modulo 256). */
  frame_seq_nb++;

  if (status_reg & SYS_STATUS_RXFCG_BIT_MASK)
  {
    uint32_t frame_len;

    /* Clear good RX frame event in the DW IC status register. */
    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG_BIT_MASK);

    /* A frame has been received, read it into the local buffer. */
    frame_len = dwt_read32bitreg(RX_FINFO_ID) & RXFLEN_MASK;
    if (frame_len <= sizeof(rx_buffer))
    {
      dwt_readrxdata(rx_buffer, frame_len, 0);

      /* Check that the frame is the expected response from the companion "SS TWR responder" example.
       * As the sequence number field of the frame is not relevant, it is cleared to simplify the validation of the frame. */
      rx_buffer[ALL_MSG_SN_IDX] = 0;
      if (memcmp(rx_buffer, rx_resp_msg_a1, ALL_MSG_COMMON_LEN) == 0)
      {
        uint32_t poll_tx_ts, resp_rx_ts, poll_rx_ts, resp_tx_ts;
        int32_t rtd_init, rtd_resp;
        float clockOffsetRatio;

        /* Retrieve poll transmission and response reception timestamps. See NOTE 9 below. */
        poll_tx_ts = dwt_readtxtimestamplo32();
        resp_rx_ts = dwt_readrxtimestamplo32();

        /* Read carrier integrator value and calculate clock offset ratio. See NOTE 11 below. */
        clockOffsetRatio = ((float)dwt_readclockoffset()) / (uint32_t)(1 << 26);

        /* Get timestamps embedded in response message. */
        resp_msg_get_ts(&rx_buffer[RESP_MSG_POLL_RX_TS_IDX], &poll_rx_ts);
        resp_msg_get_ts(&rx_buffer[RESP_MSG_RESP_TX_TS_IDX], &resp_tx_ts);

        /* Compute time of flight and distance, using clock offset ratio to correct for differing local and remote clock rates */
        rtd_init = resp_rx_ts - poll_tx_ts;
        rtd_resp = resp_tx_ts - poll_rx_ts;

        tof = ((rtd_init - rtd_resp * (1 - clockOffsetRatio)) / 2.0) * DWT_TIME_UNITS;
        distance = tof * SPEED_OF_LIGHT;

        /* Display computed distance on LCD. */
        // snprintf(dist_str, sizeof(dist_str), "DIST: %3.2f m", distance);
        snprintf(dist_str, sizeof(dist_str), "%3.2f", distance);
        test_run_info((unsigned char *)dist_str);

        a1 = atof(dist_str);
        delay(RNG_DELAY_MS);
      }
    }
  }
  else
  {
    /* Clear RX error/timeout events in the DW IC status register. */
    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);
  }



  //***********************Anchor A2******************************************
  //Serial.println("Probing Anchor A2...");
  //a2 = 12.3;
  // probe_anchor(tx_poll_msg_t0,rx_resp_msg_a1);
  /* Write frame data to DW IC and prepare transmission. See NOTE 7 below. */
  tx_poll_msg_t0[ALL_MSG_SN_IDX] = frame_seq_nb;
  dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS_BIT_MASK);
  dwt_writetxdata(sizeof(tx_poll_msg_t0), tx_poll_msg_t0, 0); /* Zero offset in TX buffer. */
  dwt_writetxfctrl(sizeof(tx_poll_msg_t0), 0, 1);          /* Zero offset in TX buffer, ranging. */

  /* Start transmission, indicating that a response is expected so that reception is enabled automatically after the frame is sent and the delay
   * set by dwt_setrxaftertxdelay() has elapsed. */
  dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED);

  /* We assume that the transmission is achieved correctly, poll for reception of a frame or error/timeout. See NOTE 8 below. */
  while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG_BIT_MASK | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR)))
  {
  };

  /* Increment frame sequence number after transmission of the poll message (modulo 256). */
  frame_seq_nb++;

  if (status_reg & SYS_STATUS_RXFCG_BIT_MASK)
  {
    uint32_t frame_len;

    /* Clear good RX frame event in the DW IC status register. */
    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG_BIT_MASK);

    /* A frame has been received, read it into the local buffer. */
    frame_len = dwt_read32bitreg(RX_FINFO_ID) & RXFLEN_MASK;
    if (frame_len <= sizeof(rx_buffer))
    {
      dwt_readrxdata(rx_buffer, frame_len, 0);

      /* Check that the frame is the expected response from the companion "SS TWR responder" example.
       * As the sequence number field of the frame is not relevant, it is cleared to simplify the validation of the frame. */
      rx_buffer[ALL_MSG_SN_IDX] = 0;
      if (memcmp(rx_buffer, rx_resp_msg_a2, ALL_MSG_COMMON_LEN) == 0)
      {
        uint32_t poll_tx_ts, resp_rx_ts, poll_rx_ts, resp_tx_ts;
        int32_t rtd_init, rtd_resp;
        float clockOffsetRatio;

        /* Retrieve poll transmission and response reception timestamps. See NOTE 9 below. */
        poll_tx_ts = dwt_readtxtimestamplo32();
        resp_rx_ts = dwt_readrxtimestamplo32();

        /* Read carrier integrator value and calculate clock offset ratio. See NOTE 11 below. */
        clockOffsetRatio = ((float)dwt_readclockoffset()) / (uint32_t)(1 << 26);

        /* Get timestamps embedded in response message. */
        resp_msg_get_ts(&rx_buffer[RESP_MSG_POLL_RX_TS_IDX], &poll_rx_ts);
        resp_msg_get_ts(&rx_buffer[RESP_MSG_RESP_TX_TS_IDX], &resp_tx_ts);

        /* Compute time of flight and distance, using clock offset ratio to correct for differing local and remote clock rates */
        rtd_init = resp_rx_ts - poll_tx_ts;
        rtd_resp = resp_tx_ts - poll_rx_ts;

        tof = ((rtd_init - rtd_resp * (1 - clockOffsetRatio)) / 2.0) * DWT_TIME_UNITS;
        distance = tof * SPEED_OF_LIGHT;

        /* Display computed distance on LCD. */
        // snprintf(dist_str, sizeof(dist_str), "DIST: %3.2f m", distance);
        snprintf(dist_str, sizeof(dist_str), "%3.2f", distance);
        test_run_info((unsigned char *)dist_str);

        a2 = atof(dist_str);
        delay(RNG_DELAY_MS);
      }
    }
  }
  else
  {
    /* Clear RX error/timeout events in the DW IC status register. */
    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);
  }



  //***********************Anchor A3******************************************
  //Serial.println("Probing Anchor A3...");
  //a3 = 12.3;
  // probe_anchor(tx_poll_msg_t0,rx_resp_msg_a1);
  /* Write frame data to DW IC and prepare transmission. See NOTE 7 below. */
  tx_poll_msg_t0[ALL_MSG_SN_IDX] = frame_seq_nb;
  dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS_BIT_MASK);
  dwt_writetxdata(sizeof(tx_poll_msg_t0), tx_poll_msg_t0, 0); /* Zero offset in TX buffer. */
  dwt_writetxfctrl(sizeof(tx_poll_msg_t0), 0, 1);          /* Zero offset in TX buffer, ranging. */

  /* Start transmission, indicating that a response is expected so that reception is enabled automatically after the frame is sent and the delay
   * set by dwt_setrxaftertxdelay() has elapsed. */
  dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED);

  /* We assume that the transmission is achieved correctly, poll for reception of a frame or error/timeout. See NOTE 8 below. */
  while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG_BIT_MASK | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR)))
  {
  };

  /* Increment frame sequence number after transmission of the poll message (modulo 256). */
  frame_seq_nb++;

  if (status_reg & SYS_STATUS_RXFCG_BIT_MASK)
  {
    uint32_t frame_len;

    /* Clear good RX frame event in the DW IC status register. */
    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG_BIT_MASK);

    /* A frame has been received, read it into the local buffer. */
    frame_len = dwt_read32bitreg(RX_FINFO_ID) & RXFLEN_MASK;
    if (frame_len <= sizeof(rx_buffer))
    {
      dwt_readrxdata(rx_buffer, frame_len, 0);

      /* Check that the frame is the expected response from the companion "SS TWR responder" example.
       * As the sequence number field of the frame is not relevant, it is cleared to simplify the validation of the frame. */
      rx_buffer[ALL_MSG_SN_IDX] = 0;
      if (memcmp(rx_buffer, rx_resp_msg_a3, ALL_MSG_COMMON_LEN) == 0)
      {
        uint32_t poll_tx_ts, resp_rx_ts, poll_rx_ts, resp_tx_ts;
        int32_t rtd_init, rtd_resp;
        float clockOffsetRatio;

        /* Retrieve poll transmission and response reception timestamps. See NOTE 9 below. */
        poll_tx_ts = dwt_readtxtimestamplo32();
        resp_rx_ts = dwt_readrxtimestamplo32();

        /* Read carrier integrator value and calculate clock offset ratio. See NOTE 11 below. */
        clockOffsetRatio = ((float)dwt_readclockoffset()) / (uint32_t)(1 << 26);

        /* Get timestamps embedded in response message. */
        resp_msg_get_ts(&rx_buffer[RESP_MSG_POLL_RX_TS_IDX], &poll_rx_ts);
        resp_msg_get_ts(&rx_buffer[RESP_MSG_RESP_TX_TS_IDX], &resp_tx_ts);

        /* Compute time of flight and distance, using clock offset ratio to correct for differing local and remote clock rates */
        rtd_init = resp_rx_ts - poll_tx_ts;
        rtd_resp = resp_tx_ts - poll_rx_ts;

        tof = ((rtd_init - rtd_resp * (1 - clockOffsetRatio)) / 2.0) * DWT_TIME_UNITS;
        distance = tof * SPEED_OF_LIGHT;

        /* Display computed distance on LCD. */
        // snprintf(dist_str, sizeof(dist_str), "DIST: %3.2f m", distance);
        snprintf(dist_str, sizeof(dist_str), "%3.2f", distance);
        test_run_info((unsigned char *)dist_str);

        a3 = atof(dist_str);
        delay(RNG_DELAY_MS);
      }
    }
  }
  else
  {
    /* Clear RX error/timeout events in the DW IC status register. */
    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);
  }

  SerialBT.print("a0 = ");
  SerialBT.println(a0);
  SerialBT.print("a1 = ");
  SerialBT.println(a1);
  SerialBT.print("a2 = ");
  SerialBT.println(a2);
  SerialBT.print("a3 = ");
  SerialBT.println(a3);

  Value values[4] = {
          {a0, fx0, fy0},
          {a1, fx1, fy1},
          {a2, fx2, fy2},
          {a3, fx3, fy3}
      };

    // 유효한 데이터를 저장할 배열
    Value filteredValues[4];
    int validCount = 0;

    // 유효한 데이터 필터링
    for (int i = 0; i < 4; i++) {
        if (values[i].a != 0 || values[i].fx != 0 || values[i].fy != 0) {
            filteredValues[validCount++] = values[i];
        }
    }

    // 정렬 (유효한 데이터만 정렬)
    qsort(filteredValues, validCount, sizeof(Value), compare);

    // 결과 배열 생성
    float resultArray[validCount * 3];
    for (int i = 0; i < validCount; i++) {
        resultArray[i * 3] = filteredValues[i].fx;
        resultArray[i * 3 + 1] = filteredValues[i].fy;
        resultArray[i * 3 + 2] = filteredValues[i].a;
    }

    // tag_location 함수 호출 (유효한 데이터만 전달)
    if (validCount >= 3) { // 최소 3개의 유효 데이터 필요
        tag_location(resultArray[0], resultArray[1], resultArray[2],  // 첫 번째 세트
                     resultArray[3], resultArray[4], resultArray[5],  // 두 번째 세트
                     resultArray[6], resultArray[7], resultArray[8]);
    } else {
        SerialBT.println("유효한 데이터가 부족합니다.");
    }
}

// 정렬 함수
int compare(const void *a, const void *b) {
    Value *val1 = (Value *)a;
    Value *val2 = (Value *)b;
    return (val1->a > val2->a) - (val1->a < val2->a); // a 값을 기준으로 오름차순 정렬
}

void tag_location(float ax1, float ay1, float ar1, float ax2, float ay2, float ar2, float ax3, float ay3, float ar3) {
    float tA = 2 * ax2 - 2 * ax1;
    float tB = 2 * ay2 - 2 * ay1;
    float tC = (ar1 * ar1) - (ar2 * ar2) - (ax1 * ax1) + (ax2 * ax2) - (ay1 * ay1) + (ay2 * ay2);
    float tD = 2 * ax3 - 2 * ax2;
    float tE = 2 * ay3 - 2 * ay2;
    float tF = (ar2 * ar2) - (ar3 * ar3) - (ax2 * ax2) + (ax3 * ax3) - (ay2 * ay2) + (ay3 * ay3);

    // 분모가 0인지 확인
    float denominator_x = tE * tA - tB * tD;
    float denominator_y = tB * tD - tA * tE;

    if (denominator_x == 0 || denominator_y == 0) {
        return; // 계산을 중단
    }

    float tx = (tC * tE - tF * tB) / denominator_x;
    float ty = (tC * tD - tA * tF) / denominator_y;

    SerialBT.print("(x,y) = (");
    SerialBT.print(tx);
    SerialBT.print(",");
    SerialBT.print(ty);
    SerialBT.println(")");
}
  

// int probe_anchor(uint8_t tx_poll_msg_ax[],uint8_t rx_resp_msg_ax[]){
//   /* Write frame data to DW IC and prepare transmission. See NOTE 7 below. */
//   tx_poll_msg_ax[ALL_MSG_SN_IDX] = frame_seq_nb;
//   dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS_BIT_MASK);
//   dwt_writetxdata(sizeof(tx_poll_msg_ax), tx_poll_msg_ax, 0); /* Zero offset in TX buffer. */
//   dwt_writetxfctrl(sizeof(tx_poll_msg_ax), 0, 1);          /* Zero offset in TX buffer, ranging. */

//   /* Start transmission, indicating that a response is expected so that reception is enabled automatically after the frame is sent and the delay
//    * set by dwt_setrxaftertxdelay() has elapsed. */
//   dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED);

//   /* We assume that the transmission is achieved correctly, poll for reception of a frame or error/timeout. See NOTE 8 below. */
//   while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG_BIT_MASK | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR)))
//   {
//   };

//   /* Increment frame sequence number after transmission of the poll message (modulo 256). */
//   frame_seq_nb++;

//   if (status_reg & SYS_STATUS_RXFCG_BIT_MASK)
//   {
//     uint32_t frame_len;

//     /* Clear good RX frame event in the DW IC status register. */
//     dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG_BIT_MASK);

//     /* A frame has been received, read it into the local buffer. */
//     frame_len = dwt_read32bitreg(RX_FINFO_ID) & RXFLEN_MASK;
//     if (frame_len <= sizeof(rx_buffer))
//     {
//       dwt_readrxdata(rx_buffer, frame_len, 0);

//       /* Check that the frame is the expected response from the companion "SS TWR responder" example.
//        * As the sequence number field of the frame is not relevant, it is cleared to simplify the validation of the frame. */
//       rx_buffer[ALL_MSG_SN_IDX] = 0;
//       if (memcmp(rx_buffer, rx_resp_msg_ax, ALL_MSG_COMMON_LEN) == 0)
//       {
//         uint32_t poll_tx_ts, resp_rx_ts, poll_rx_ts, resp_tx_ts;
//         int32_t rtd_init, rtd_resp;
//         float clockOffsetRatio;

//         /* Retrieve poll transmission and response reception timestamps. See NOTE 9 below. */
//         poll_tx_ts = dwt_readtxtimestamplo32();
//         resp_rx_ts = dwt_readrxtimestamplo32();

//         /* Read carrier integrator value and calculate clock offset ratio. See NOTE 11 below. */
//         clockOffsetRatio = ((float)dwt_readclockoffset()) / (uint32_t)(1 << 26);

//         /* Get timestamps embedded in response message. */
//         resp_msg_get_ts(&rx_buffer[RESP_MSG_POLL_RX_TS_IDX], &poll_rx_ts);
//         resp_msg_get_ts(&rx_buffer[RESP_MSG_RESP_TX_TS_IDX], &resp_tx_ts);

//         /* Compute time of flight and distance, using clock offset ratio to correct for differing local and remote clock rates */
//         rtd_init = resp_rx_ts - poll_tx_ts;
//         rtd_resp = resp_tx_ts - poll_rx_ts;

//         tof = ((rtd_init - rtd_resp * (1 - clockOffsetRatio)) / 2.0) * DWT_TIME_UNITS;
//         distance = tof * SPEED_OF_LIGHT;

//         /* Display computed distance on LCD. */
//         // snprintf(dist_str, sizeof(dist_str), "DIST: %3.2f m", distance);
//         snprintf(dist_str, sizeof(dist_str), "%3.2f", distance);
//         test_run_info((unsigned char *)dist_str);
//       }
//     }
//   }
//   else
//   {
//     /* Clear RX error/timeout events in the DW IC status register. */
//     dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);
//   }
// }
