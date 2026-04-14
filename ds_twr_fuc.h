/*
 * ds_twr_fuc.h
 *
 *  Created on: Sep 23, 2025
 *      Author: jjw
 */

#ifndef DS_TWR_FUC_H_
#define DS_TWR_FUC_H_


#include <stdio.h>
#include <stm32f4xx_hal.h>    // STM32 HAL 드라이버
#include <string.h>           // strlen 함수 사용
#include <stdlib.h>           // 추가: malloc, free 등을 위해
#include "examples_defines.h" // Decawave 예제 관련 정의 (example_pointer 등)
#include "port.h"             // Decawave 포팅 관련 함수 (port_DisableEXT_IRQ 등)
#include "ds_twr.h"           // custom code 헤더
#include <assert.h>
#include <example_selection.h>
#include "spi.h"
#include "tim.h"
#include "deca_probe_interface.h"
#include "config_options.h"
#include "deca_device_api.h"
#include "deca_spi.h"
#include "shared_defines.h"
#include "shared_functions.h"

#define CALI_NUM 1000 // 보정 횟수

#define TEST_DS_TWR_INITIATOR_irq_anchor_map


void responder_init(void);
void initiator_init(void);
void initiator_map_init(void);
void initiator_single_init(void);
void responder_single_init(void);
void SPI_SetPrescaler_ReInit(SPI_HandleTypeDef *hspi, uint32_t prescaler);

extern int ds_twr_responder_irq(void);
extern int ds_twr_initiator_irq(void);
extern int ds_twr_initiator_irq_anchor_map(void);
extern int ds_twr_initiator_irq_single(void);
extern int ds_twr_responder_irq_single(void);


extern example_ptr example_pointer;

extern uint32_t num_cali; // 보정 횟수
extern uint32_t num_cali_2; // 보정 횟수(삼각측량 두번째 거리)
extern double tmp_cali_1[CALI_NUM];
extern double tmp_cali_2[CALI_NUM];

typedef enum {
	S_IDLE,
    S_POLL_TX,
    S_FINAL_TX,
    S_POLL_WAIT,
	S_FINAL_WAIT,
	S_DATA_TX,
	S_WAIT_TURN,
	S_TURN_TX
} dwr_state;

extern volatile dwr_state state;

extern uint8_t anchor;


#endif /* DS_TWR_FUC_H_ */
