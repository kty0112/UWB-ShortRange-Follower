/* ds_twr_responder_irq.c - 메모리 오버플로우 완벽 해결 버전 */
#include "deca_probe_interface.h"
#include "config_options.h"
#include "deca_device_api.h"
#include "deca_spi.h"
#include "example_selection.h"
#include "port.h"
#include "shared_defines.h"
#include "shared_functions.h"
#include <stdio.h>
#include <math.h>
#include <stdint.h>
#include "dw3000_deca_regs.h"
#include "ds_twr.h"
#include "ds_twr_fuc.h"
#include "examples_defines.h"

#if defined(TEST_DS_TWR_RESPONDER_irq)
extern void test_run_info(unsigned char *data);
#define APP_NAME "DS TWR RESP (Back Car - Calc)"

static dwt_config_t config = {
    5,                 // Channel
    DWT_PLEN_128,      // ★ Preamble Length (단거리용으로 축소)
    DWT_PAC8,          // ★ PAC Size (짧은 프리앰블에 맞춤)
    9,                 // TX Preamble code (PRF 64MHz용)
    9,                 // RX Preamble code (PRF 64MHz용)
    1,                 // 1 = 표준 802.15.4a SFD 사용 (기존 2 였다면 1 권장)
    DWT_BR_6M8,        // ★ Data Rate (6.8Mbps 고속 통신)
    DWT_PHRMODE_STD,
    DWT_PHRRATE_STD,
    (128 + 1 + 8 - 8), // ★ SFD Timeout (PLEN 128 기준 재계산)
    DWT_STS_MODE_OFF,
    DWT_STS_LEN_64,
    DWT_PDOA_M0
};

#define TX_ANT_DLY (16365)
#define RX_ANT_DLY (16365)

#define TRX_TO_TX_DLY_UUS (5000)
#define TRX_TO_RX_DLY_UUS (2000)
#define PRE_TIMEOUT 0

#define TS_POLL_TX_IDX FINAL_MSG_POLL_TX_TS_IDX
#define TS_RESP_RX_IDX FINAL_MSG_RESP_RX_TS_IDX
#define TS_FINAL_TX_IDX FINAL_MSG_FINAL_TX_TS_IDX

static void rx_ok_cb(const dwt_cb_data_t *cb_data);
static void rx_to_cb(const dwt_cb_data_t *cb_data);
static void rx_err_cb(const dwt_cb_data_t *cb_data);
static void tx_conf_cb(const dwt_cb_data_t *cb_data);
static void set_all_msg_anchor_id(uint8_t anchor_id);
static void uwb_enable_receive(void);

extern dwt_txconfig_t txconfig_options;

static uint8_t rx_poll_msg[] = { 0x41, 0x88, 0, 0xCA, 0xDE, 0x01, 0x00, 'V', 'E', 0x21 };
static uint8_t tx_resp_msg[] = { 0x41, 0x88, 0, 0xCA, 0xDE, 0x01, 0x00, 'W', 'A', 0x10, 0x02, 0, 0 };
static uint8_t rx_final_msg[] = { 0x41, 0x88, 0, 0xCA, 0xDE, 0x01, 0x00, 'V', 'E', 0x23, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

static uint8_t frame_seq_nb = 0;
static uint8_t rx_buffer[RX_BUF_LEN];

static float dist_init1_res1 = 0.0f; // 1번 데이터
static float dist_init2_res1 = 0.0f; // 3번 데이터

/* IMU 데이터 수신 저장용 (UWB_ver1 UWB_Leader_Res_1 방식 이식) */
static int16_t global_imu_vals[6] = {0,};
static volatile uint8_t imu_ready_to_print = 0;
static char imu_dbg_str_global[128];

static uint64_t poll_rx_ts[NUM_ANCHORS] = {0,};
static uint64_t resp_tx_ts[NUM_ANCHORS] = {0,};
static uint64_t final_rx_ts[NUM_ANCHORS] = {0,};

static uint16_t frame_len;
static uint32_t resp_tx_time;
static int ret;

/* Hz 측정용 변수 */
static uint32_t hz_start_tick = 0;
static uint32_t hz_count = 0;
static uint8_t  hz_measuring = 1; // 1: 10초 측정 중, 0: 정상 출력 모드

int ds_twr_responder_irq(void)
{
    dwt_callbacks_s cbs = {NULL};
    port_set_dw_ic_spi_fastrate();
    reset_DWIC(); 
    Sleep(2); 

    dwt_probe((struct dwt_probe_s *)&dw3000_probe_interf);
    while (!dwt_checkidlerc()) { };
    if (dwt_initialise(DWT_DW_INIT) == DWT_ERROR) { while (1) { }; }

    dwt_configure(&config);
    dwt_configuretxrf(&txconfig_options);

    cbs.cbTxDone = tx_conf_cb;
    cbs.cbRxOk = rx_ok_cb;
    cbs.cbRxTo = rx_to_cb;
    cbs.cbRxErr = rx_err_cb;
    dwt_setcallbacks(&cbs);

    dwt_setinterrupt(DWT_INT_TXFRS_BIT_MASK | DWT_INT_RXFCG_BIT_MASK | DWT_INT_RXFTO_BIT_MASK | DWT_INT_RXPTO_BIT_MASK | DWT_INT_RXPHE_BIT_MASK |
    				 DWT_INT_RXFCE_BIT_MASK | DWT_INT_RXFSL_BIT_MASK | DWT_INT_RXSTO_BIT_MASK | DWT_INT_RXOVRR_BIT_MASK,
					 0, DWT_ENABLE_INT);

    port_set_dwic_isr(dwt_isr);
	dwt_writesysstatuslo(DWT_INT_RCINIT_BIT_MASK | DWT_INT_SPIRDY_BIT_MASK);
	dwt_writesysstatuslo(SYS_STATUS_ALL_RX_GOOD | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);

    dwt_setrxantennadelay(RX_ANT_DLY);
    dwt_settxantennadelay(TX_ANT_DLY);
	  //dwt_setlnapamode(DWT_LNA_ENABLE | DWT_PA_ENABLE);
    //dwt_settxpower(0xfdfdfdfd);  

    set_all_msg_anchor_id(anchor_macro);

    raspi_send((unsigned char *)"\r\n[SYSTEM] 뒷차(Res) V2.3 (오버플로우 해결완료!)\r\n");
    raspi_send((unsigned char *)"[HZ TEST] 10초 동안 데이터 수신 중... \r\n");
    hz_start_tick = HAL_GetTick();
    uwb_enable_receive();
    
    while (1) {
        if (port_CheckEXT_IRQ() != 0) {
            process_deca_irq();
        }
    }
}

static void rx_ok_cb(const dwt_cb_data_t *cb_data)
{
	(void)cb_data;
	dwt_writesysstatuslo(SYS_STATUS_ALL_RX_GOOD);

	if (state == S_POLL_WAIT) {
		frame_len = dwt_getframelength(0);
		if (frame_len <= RX_BUF_LEN) {
			dwt_readrxdata(rx_buffer, frame_len, 0);
		} else {
			uwb_enable_receive();
			return;
		}

        /* DATA 패킷(Res_2가 보낸 보낸 2, 4번 거리 데이터)인지 확인: 매직넘버 'D', 'T', 0x44 */
		if (rx_buffer[5] == 0x01 && rx_buffer[6] == 0x02 && 
		    rx_buffer[7] == 'D' && rx_buffer[8] == 'T' && rx_buffer[9] == 0x44) {
			
			float res2_dists[2];
			memcpy(res2_dists, &rx_buffer[10], 8);

			if (hz_measuring) {
				/* 10초 측정 단계 */
				hz_count++;
				uint32_t elapsed = HAL_GetTick() - hz_start_tick;
				if (elapsed >= 10000) {
					static char hz_buf[64];
					float hz = (float)hz_count / 10.0f;
					sprintf(hz_buf, "[HZ RESULT] %lu cycles / 10s = %.2f Hz", hz_count, hz);
					raspi_send((unsigned char *)hz_buf);
					hz_measuring = 0;
				}
			} else {
				/* 정상 출력 모드 */
				static char dbgd[200];
				sprintf(dbgd, "[DIST] I1-R1: %.2f | I1-R2: %.2f | I2-R1: %.2f | I2-R2: %.2f",
					    dist_init1_res1, res2_dists[0], dist_init2_res1, res2_dists[1]);
				raspi_send((unsigned char *)dbgd);
				sprintf(dbgd, "[IMU] AccX:%d, AccY:%d, AccZ:%d, Roll:%d, Pitch:%d, Yaw:%d",
					    global_imu_vals[0], global_imu_vals[1], global_imu_vals[2],
					    global_imu_vals[3], global_imu_vals[4], global_imu_vals[5]);
				raspi_send((unsigned char *)dbgd);
			}

			uwb_enable_receive();
			return;
		}

		rx_buffer[ALL_MSG_SN_IDX] = 0;
		uint8_t received_tag = rx_buffer[6];
		rx_buffer[6] = 0;
		if (memcmp(rx_buffer, rx_poll_msg, ALL_MSG_COMMON_LEN) == 0) {

			/* Poll 메시지에 실린 IMU 데이터 추출 (UWB_ver1 방식) */
			if (frame_len >= ALL_MSG_COMMON_LEN + IMU_DATA_SIZE) {
				memcpy(global_imu_vals, &rx_buffer[MSG_IMU_IDX], IMU_DATA_SIZE);
			}

			poll_rx_ts[0] = get_rx_timestamp_u64();
			resp_tx_time = (uint32_t)((poll_rx_ts[0] + TRX_TO_TX_DLY_UUS * UUS_TO_DWT_TIME) >> 8);
			dwt_setdelayedtrxtime(resp_tx_time);

			dwt_setrxaftertxdelay(TRX_TO_RX_DLY_UUS);
			dwt_setrxtimeout(TRX_TO_TX_DLY_UUS * 2);
			dwt_setpreambledetecttimeout(PRE_TIMEOUT);

			tx_resp_msg[6] = received_tag;
			tx_resp_msg[ALL_MSG_SN_IDX] = frame_seq_nb;
			dwt_writetxdata(sizeof(tx_resp_msg), tx_resp_msg, 0);
			dwt_writetxfctrl(sizeof(tx_resp_msg) + FCS_LEN, 0, 1);

			ret = dwt_starttx(DWT_START_TX_DELAYED | DWT_RESPONSE_EXPECTED);
			if (ret != DWT_SUCCESS) {
				uwb_enable_receive();
			}
		} else {
			uwb_enable_receive();
		}
	}
	else if (state == S_FINAL_WAIT) {
		frame_len = dwt_getframelength(0);
		if (frame_len <= RX_BUF_LEN) {
			dwt_readrxdata(rx_buffer, frame_len, 0);
		} else {
			uwb_enable_receive();
			return;
		}

		rx_buffer[ALL_MSG_SN_IDX] = 0;
		uint8_t target_init_id = rx_buffer[6];
		rx_buffer[6] = 0;
		if (memcmp(rx_buffer, rx_final_msg, ALL_MSG_COMMON_LEN) == 0) {
			final_rx_ts[0] = get_rx_timestamp_u64();

            // 🌟 1. 함수가 요구하는 정품 8바이트 바구니 준비
			uint64_t init_poll_tx_64 = 0, init_resp_rx_64 = 0, init_final_tx_64 = 0;

            // 🌟 2. 강제 형변환 없이 8바이트로 안전하게 데이터 수령
			final_msg_get_ts(&rx_buffer[TS_POLL_TX_IDX], &init_poll_tx_64);
			final_msg_get_ts(&rx_buffer[TS_RESP_RX_IDX], &init_resp_rx_64);
			final_msg_get_ts(&rx_buffer[TS_FINAL_TX_IDX], &init_final_tx_64);

            // 🌟 3. 계산을 위해 4바이트로 변환
			uint32_t init_poll_tx = (uint32_t)init_poll_tx_64;
			uint32_t init_resp_rx = (uint32_t)init_resp_rx_64;
			uint32_t init_final_tx = (uint32_t)init_final_tx_64;

			uint32_t res_poll_rx = (uint32_t)poll_rx_ts[0];
			uint32_t res_resp_tx = (uint32_t)resp_tx_ts[0];
			uint32_t res_final_rx = (uint32_t)final_rx_ts[0];

			double Ra = (double)((uint32_t)(init_resp_rx - init_poll_tx));
			double Rb = (double)((uint32_t)(res_final_rx - res_resp_tx));
			double Da = (double)((uint32_t)(init_final_tx - init_resp_rx));
			double Db = (double)((uint32_t)(res_resp_tx - res_poll_rx));

			int64_t tof_dtu = (int64_t)((Ra * Rb - Da * Db) / (Ra + Rb + Da + Db));
			double tof = tof_dtu * DWT_TIME_UNITS;
			double distance = tof * SPEED_OF_LIGHT;
			float dist_f = (float)distance;

			// 각각의 Init에 대한 거리 저장
			if (target_init_id == 1) {
				dist_init1_res1 = dist_f; 
			} else if (target_init_id == 2) {
				dist_init2_res1 = dist_f; 
			}



			uwb_enable_receive();
		} else {
			uwb_enable_receive();
		}
	}
}

static void rx_to_cb(const dwt_cb_data_t *cb_data) {
    (void)cb_data;
	dwt_writesysstatuslo(SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);
	uwb_enable_receive();
}

static void rx_err_cb(const dwt_cb_data_t *cb_data) {
    (void)cb_data;
    dwt_writesysstatuslo(SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);
    uwb_enable_receive();
}

static void tx_conf_cb(const dwt_cb_data_t *cb_data) {
    (void)cb_data;
	if(state == S_POLL_WAIT) {
        resp_tx_ts[0] = get_tx_timestamp_u64();
		state = S_FINAL_WAIT;
	}
	dwt_writesysstatuslo(DWT_INT_TXFRS_BIT_MASK);
}

static void uwb_enable_receive(void) {
	dwt_forcetrxoff(); 
	dwt_setrxaftertxdelay(TRX_TO_RX_DLY_UUS);
    dwt_setpreambledetecttimeout(PRE_TIMEOUT);
    dwt_setrxtimeout(0);
	do {
		ret = dwt_rxenable(DWT_START_RX_IMMEDIATE);
	} while (ret != DWT_SUCCESS);
    state = S_POLL_WAIT;
}

static void set_all_msg_anchor_id(uint8_t anchor_id) {
    tx_resp_msg[5] = anchor_id;
    rx_poll_msg[5] = anchor_id;
    rx_final_msg[5] = anchor_id;
}
#endif
