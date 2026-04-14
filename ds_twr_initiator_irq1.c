/*
 * @file    ds_twr_initiator_irq1.c
 * @brief   DS-TWR initiator with Token-Passing (MASTER)
 *
 * Init_1 is the master node: boots immediately into ranging.
 * Sequence: Res_1 DS-TWR → Res_2 DS-TWR → TURN broadcast → wait for TURN → repeat
 * If Init_2 is offline, 50ms timeout triggers self-restart.
 */

#include "deca_probe_interface.h"
#include "config_options.h"
#include "deca_device_api.h"
#include "deca_spi.h"
#include "example_selection.h"
#include "port.h"
#include "shared_defines.h"
#include "shared_functions.h"
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stm32f4xx_hal.h>
#include "main.h"
#include "usart.h"
#include "tim.h"
#include "ds_twr.h"
#include "ds_twr_fuc.h"
#include "examples_defines.h"


#if defined(TEST_DS_TWR_INITIATOR_irq)

extern void test_run_info(unsigned char *data);
extern uint8_t tag_macro;
extern uint8_t latest_imu_data[IMU_DATA_SIZE];

#define APP_NAME "DS TWR INIT IRQ v2.0 (Token-Passing MASTER)"

/* Default antenna delay values for 64 MHz PRF. */
#define TX_ANT_DLY (16365)
#define RX_ANT_DLY (16365)

/* UWB configuration - matched to responder settings */
static dwt_config_t config = {
    5,                    /* Channel number. */
    DWT_PLEN_128,         /* Preamble length. Used in TX only. */
    DWT_PAC8,             /* Preamble acquisition chunk size. Used in RX only. */
    9,                    /* TX preamble code. Used in TX only. */
    9,                    /* RX preamble code. Used in RX only. */
    1,                    /* 0: standard 8 symbol SFD, 1: non-standard 8 symbol SFD */
    DWT_BR_6M8,           /* Data rate. */
    DWT_PHRMODE_STD,      /* PHY header mode. */
    DWT_PHRRATE_STD,      /* PHY header rate. */
    (128 + 1 + 8 - 8),   /* SFD timeout (preamble length + 1 + SFD length - PAC size). */
    DWT_STS_MODE_OFF,     /* STS disabled */
    DWT_STS_LEN_64,       /* STS length */
    DWT_PDOA_M0           /* PDOA mode off */
};

/* DS-TWR 3-way 메시지 버퍼 (Poll에 IMU 12바이트 탑재) */
static uint8_t tx_poll_msg[ALL_MSG_COMMON_LEN + IMU_DATA_SIZE]  = { 0x41, 0x88, 0, 0xCA, 0xDE, 0x01, 0x00, 'V', 'E', 0x21 };
static uint8_t rx_resp_msg[]  = { 0x41, 0x88, 0, 0xCA, 0xDE, 0x01, 0x00, 'W', 'A', 0x10, 0x02, 0, 0 };
static uint8_t tx_final_msg[] = { 0x41, 0x88, 0, 0xCA, 0xDE, 0x01, 0x00, 'V', 'E', 0x23,
                                   0, 0, 0, 0, 0,   /* poll_tx_ts placeholder (5 bytes) */
                                   0, 0, 0, 0, 0,   /* resp_rx_ts placeholder (5 bytes) */
                                   0, 0, 0, 0, 0 }; /* final_tx_ts placeholder (5 bytes) */

/* TURN 메시지: byte[5]=0xFF (Responder가 무시), 'T','U',0x52 = TURN 식별자 */
static uint8_t tx_turn_msg[] = { 0x41, 0x88, 0, 0xCA, 0xDE, 0xFF, 0x00, 'T', 'U', 0x52 };

/* Frame sequence number */
static uint8_t frame_seq_nb = 0;

static uint8_t target_anchor = 1;

/* 현재 대상 Responder (1 또는 2) */
static uint8_t current_target = 1;

/* WAIT_TURN 소프트웨어 타임아웃용 */
static uint32_t wait_turn_start = 0;
#define WAIT_TURN_TIMEOUT_MS 50

/* RX 수신 버퍼 */
static uint8_t rx_buffer[RX_BUF_LEN];

/* TRX 딜레이 */
#define TRX_TO_TX_DLY_UUS (5000)
#define TRX_TO_RX_DLY_UUS (2000)
#define PRE_TIMEOUT 0

static int ret;
static uint64_t latest_rx_time;
static uint32_t final_tx_time;
static uint64_t poll_tx_ts;

/* Callback function declarations */
static void rx_ok_cb(const dwt_cb_data_t *cb_data);
static void rx_to_cb(const dwt_cb_data_t *cb_data);
static void rx_err_cb(const dwt_cb_data_t *cb_data);
static void tx_conf_cb(const dwt_cb_data_t *cb_data);

static void send_poll_immediate(void);
static void send_final(void);
static void send_turn(void);
static void enter_wait_turn(void);
static void advance_target(void);
static void start_ranging(void);
static void set_poll_msg_anchor_id(uint8_t anchor_id);
static void set_resp_msg_anchor_id(uint8_t anchor_id);
static void set_final_msg_anchor_id(uint8_t anchor_id);
static void set_all_msg_anchor_id(uint8_t anchor_id);

extern dwt_txconfig_t txconfig_options;

static dwt_callbacks_s cbs = {NULL};


int ds_twr_initiator_irq(void)
{
    raspi_send((unsigned char *)"\r\n=== INIT_1 Token-Passing MASTER ===\r\n");

    raspi_send((unsigned char *)"1. SPI Set Fastrate...\r\n");
    port_set_dw_ic_spi_fastrate();

    raspi_send((unsigned char *)"2. UWB Hardware Resetting...\r\n");
    reset_DWIC();
    Sleep(2);
    raspi_send((unsigned char *)"3. UWB Reset OK!\r\n");

    cbs.cbTxDone = tx_conf_cb;
    cbs.cbRxOk = rx_ok_cb;
    cbs.cbRxTo = rx_to_cb;
    cbs.cbRxErr = rx_err_cb;

    raspi_send((unsigned char *)"4. UWB Probe Start...\r\n");
    dwt_probe((struct dwt_probe_s *)&dw3000_probe_interf);

    raspi_send((unsigned char *)"5. Waiting for UWB IDLE state...\r\n");
    while (!dwt_checkidlerc()) { };

    raspi_send((unsigned char *)"6. UWB Init Start...\r\n");
    if (dwt_initialise(DWT_DW_INIT) == DWT_ERROR) {
        raspi_send((unsigned char *)"INIT FAILED\r\n");
        while (1) { };
    }

    raspi_send((unsigned char *)"7. UWB Config Start...\r\n");
    dwt_configure(&config);
    dwt_configuretxrf(&txconfig_options);
    dwt_setcallbacks(&cbs);
    dwt_setinterrupt(DWT_INT_TXFRS_BIT_MASK | DWT_INT_RXFCG_BIT_MASK | DWT_INT_RXFTO_BIT_MASK |
                     DWT_INT_RXPTO_BIT_MASK | DWT_INT_RXPHE_BIT_MASK | DWT_INT_RXFCE_BIT_MASK |
                     DWT_INT_RXFSL_BIT_MASK | DWT_INT_RXSTO_BIT_MASK, 0, DWT_ENABLE_INT);

    dwt_setrxantennadelay(RX_ANT_DLY);
    dwt_settxantennadelay(TX_ANT_DLY);

    HAL_TIM_Base_Stop_IT(&htim2);
    port_set_dwic_isr(dwt_isr);

    raspi_send((unsigned char *)"8. ALL SETUP OK! Starting Token-Passing...\r\n");

    /* 마스터: 즉시 Res_1부터 시작 */
    start_ranging();

    while (1) {
        /* S_WAIT_TURN 소프트웨어 타임아웃: 상대 Init이 죽었을 때 자력 복구 */
        if (state == S_WAIT_TURN) {
            if ((HAL_GetTick() - wait_turn_start) > WAIT_TURN_TIMEOUT_MS) {
                start_ranging();
            }
        }
    }
}


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    /* 타이머 인터럽트 무시 */
}


/* RX 성공 콜백 */
static void rx_ok_cb(const dwt_cb_data_t *cb_data)
{
    (void)cb_data;
    uint16_t frame_len;
    uint8_t anchor_id;

    dwt_writesysstatuslo(SYS_STATUS_ALL_RX_GOOD);

    /* 1) POLL 전송 후 → RESPONSE 수신 대기 상태 */
    if (state == S_POLL_TX) {
        frame_len = dwt_getframelength(0);
        if (frame_len > RX_BUF_LEN) {
            advance_target();
            return;
        }
        dwt_readrxdata(rx_buffer, frame_len, 0);

        rx_buffer[ALL_MSG_SN_IDX] = 0;
        anchor_id = rx_buffer[5];

        if (anchor_id != target_anchor) {
            advance_target();
            return;
        }

        /* tag ID 확인 */
        if (rx_buffer[6] != tag_macro) {
            advance_target();
            return;
        }
        rx_buffer[6] = 0; /* memcmp용 복원 */

        if (memcmp(rx_buffer, rx_resp_msg, ALL_MSG_COMMON_LEN) != 0) {
            advance_target();
            return;
        }

        /* RESPONSE 수신 성공 → FINAL 전송 */
        send_final();
    }
    /* 2) TURN 대기 중 → TURN 메시지 수신 */
    else if (state == S_WAIT_TURN) {
        frame_len = dwt_getframelength(0);
        if (frame_len <= RX_BUF_LEN) {
            dwt_readrxdata(rx_buffer, frame_len, 0);
        } else {
            enter_wait_turn();
            return;
        }

        /* TURN 메시지 확인: 'T', 'U', 0x52 */
        if (rx_buffer[7] == 'T' && rx_buffer[8] == 'U' && rx_buffer[9] == 0x52) {
            /* 상대 Init 완료 → 내 차례 시작 */
            start_ranging();
        } else {
            /* TURN이 아닌 다른 패킷 → RX만 다시 활성화 (타이머 리셋 안 함) */
            dwt_rxenable(DWT_START_RX_IMMEDIATE);
        }
    }
    else {
        /* 예상치 못한 수신 → 무시 */
        dwt_forcetrxoff();
    }
}


/* RX 타임아웃 콜백 */
static void rx_to_cb(const dwt_cb_data_t *cb_data)
{
    (void)cb_data;
    dwt_writesysstatuslo(SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);

    if (state == S_WAIT_TURN) {
        /* TURN 타임아웃 → 상대 Init 오프라인 → 자력 시작 */
        start_ranging();
    } else {
        /* DS-TWR 중 타임아웃 → 다음 타겟으로 진행 */
        advance_target();
    }
}


/* RX 에러 콜백 */
static void rx_err_cb(const dwt_cb_data_t *cb_data)
{
    (void)cb_data;
    dwt_writesysstatuslo(SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);

    if (state == S_WAIT_TURN) {
        dwt_rxenable(DWT_START_RX_IMMEDIATE);
    } else {
        advance_target();
    }
}


/* TX 완료 콜백 */
static void tx_conf_cb(const dwt_cb_data_t *cb_data)
{
    (void)cb_data;
    dwt_writesysstatuslo(DWT_INT_TXFRS_BIT_MASK);

    if (state == S_POLL_TX) {
        /* POLL 전송 완료 → 타임스탬프 저장 (RX 대기 중) */
        poll_tx_ts = get_tx_timestamp_u64();
    }
    else if (state == S_FINAL_TX) {
        /* FINAL 전송 완료 → 다음 동작 결정 */
        frame_seq_nb++;

        if (current_target == 1) {
            /* Res_1 완료 → Res_2 시작 */
            current_target = 2;
            set_all_msg_anchor_id(2);
            send_poll_immediate();
        } else {
            /* Res_2 완료 → TURN 전송 */
            send_turn();
        }
    }
    else if (state == S_TURN_TX) {
        /* TURN 전송 완료 → RX 대기 모드 진입 */
        enter_wait_turn();
    }
}


/*=============================================================================
 * 토큰 패싱 핵심 함수들
 *===========================================================================*/

/* 레인징 시작: Res_1부터 Poll 전송 */
static void start_ranging(void)
{
    dwt_forcetrxoff();
    current_target = 1;
    set_all_msg_anchor_id(1);
    send_poll_immediate();
}


/* 에러 시 다음 타겟으로 진행, 또는 TURN 전송 */
static void advance_target(void)
{
    dwt_forcetrxoff();

    if (current_target == 1) {
        /* Res_1 실패 → Res_2 시도 */
        current_target = 2;
        set_all_msg_anchor_id(2);
        send_poll_immediate();
    } else {
        /* Res_2도 완료(또는 실패) → TURN 전송하여 상대에게 넘김 */
        send_turn();
    }
}


/* TURN 메시지 브로드캐스트 */
static void send_turn(void)
{
    dwt_forcetrxoff();

    tx_turn_msg[6] = tag_macro;
    tx_turn_msg[ALL_MSG_SN_IDX] = frame_seq_nb;
    dwt_writetxdata(sizeof(tx_turn_msg), tx_turn_msg, 0);
    dwt_writetxfctrl(sizeof(tx_turn_msg) + FCS_LEN, 0, 1);

    ret = dwt_starttx(DWT_START_TX_IMMEDIATE);

    if (ret != DWT_SUCCESS) {
        /* TX 실패 시 바로 WAIT_TURN 진입 */
        enter_wait_turn();
    } else {
        state = S_TURN_TX;
    }
}


/* TURN 대기 모드 진입 */
static void enter_wait_turn(void)
{
    dwt_forcetrxoff();
    dwt_setrxaftertxdelay(0);
    dwt_setrxtimeout(0);  /* 하드웨어 타임아웃 없음 (소프트웨어로 관리) */
    dwt_setpreambledetecttimeout(0);

    ret = dwt_rxenable(DWT_START_RX_IMMEDIATE);
    if (ret != DWT_SUCCESS) {
        /* RX 활성화 실패 시 재시도 */
        dwt_forcetrxoff();
        ret = dwt_rxenable(DWT_START_RX_IMMEDIATE);
    }

    wait_turn_start = HAL_GetTick();
    state = S_WAIT_TURN;
}


/*=============================================================================
 * DS-TWR 메시지 전송 함수들 (기존과 동일)
 *===========================================================================*/

/* Poll 메시지 즉시 전송, RESPONSE 대기 활성화 */
static void send_poll_immediate(void)
{
    tx_poll_msg[6] = tag_macro;
    tx_poll_msg[ALL_MSG_SN_IDX] = frame_seq_nb;

    /* IMU 데이터 페이로드 복사 */
    memcpy(&tx_poll_msg[MSG_IMU_IDX], latest_imu_data, IMU_DATA_SIZE);

    dwt_writetxdata(sizeof(tx_poll_msg), tx_poll_msg, 0);
    dwt_writetxfctrl(sizeof(tx_poll_msg) + FCS_LEN, 0, 1);

    dwt_setrxaftertxdelay(TRX_TO_RX_DLY_UUS);
    dwt_setrxtimeout(7000);
    dwt_setpreambledetecttimeout(PRE_TIMEOUT);

    /* 전송 후 RESPONSE 수신 대기 */
    ret = dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED);

    if (ret != DWT_SUCCESS) {
        advance_target();
    } else {
        state = S_POLL_TX;
    }
}


/* Final 메시지 전송 */
static void send_final(void)
{
    uint64_t final_tx_ts;

    latest_rx_time = get_rx_timestamp_u64();

    final_tx_time = (uint32_t)((latest_rx_time + TRX_TO_TX_DLY_UUS * UUS_TO_DWT_TIME) >> 8);
    dwt_setdelayedtrxtime(final_tx_time);

    /* 실제 안테나 딜레이를 고려한 최종 TX 예측 타임스탬프 계산 */
    final_tx_ts = (((uint64_t)(final_tx_time & 0xFFFFFFFEUL)) << 8) + TX_ANT_DLY;

    /* 타임스탬프 필드들 실제 값으로 채워넣기 */
    final_msg_set_ts(&tx_final_msg[FINAL_MSG_POLL_TX_TS_IDX], poll_tx_ts);
    final_msg_set_ts(&tx_final_msg[FINAL_MSG_RESP_RX_TS_IDX], latest_rx_time);
    final_msg_set_ts(&tx_final_msg[FINAL_MSG_FINAL_TX_TS_IDX], final_tx_ts);

    tx_final_msg[6] = tag_macro;
    tx_final_msg[ALL_MSG_SN_IDX] = frame_seq_nb;
    dwt_writetxdata(sizeof(tx_final_msg), tx_final_msg, 0);
    dwt_writetxfctrl(sizeof(tx_final_msg) + FCS_LEN, 0, 1);

    ret = dwt_starttx(DWT_START_TX_DELAYED);

    if (ret != DWT_SUCCESS) {
        advance_target();
    } else {
        state = S_FINAL_TX;
    }
}


/*=============================================================================
 * 메시지 anchor ID 설정 함수들
 *===========================================================================*/

static void set_poll_msg_anchor_id(uint8_t anchor_id)
{
    tx_poll_msg[5] = anchor_id;
}

static void set_resp_msg_anchor_id(uint8_t anchor_id)
{
    rx_resp_msg[5] = anchor_id;
}

static void set_final_msg_anchor_id(uint8_t anchor_id)
{
    tx_final_msg[5] = anchor_id;
}

static void set_all_msg_anchor_id(uint8_t anchor_id)
{
    target_anchor = anchor_id;
    set_poll_msg_anchor_id(anchor_id);
    set_resp_msg_anchor_id(anchor_id);
    set_final_msg_anchor_id(anchor_id);
}

#endif
