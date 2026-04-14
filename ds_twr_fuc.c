/*
 * ds_twr_fuc.c
 *
 * Created on: Sep 23, 2025
 * Author: jjw
 */

#include "ds_twr_fuc.h"

// 함수 원형 선언 (extern으로 다른 파일의 함수 연결)
extern int ds_twr_responder_irq(void);
extern int ds_twr_initiator_irq(void);

example_ptr example_pointer;

// 🌟 수정 1: 과거의 잔재인 ANCHOR 매크로 삭제하고, 동적 변수 연결
extern uint8_t anchor_macro;

uint32_t num_cali = 0; // 보정 횟수
uint32_t num_cali_2 = 0; // 보정 횟수(삼각측량 두번째 거리)

double tmp_cali_1[CALI_NUM];
double tmp_cali_2[CALI_NUM];

// 🌟 수정 2: 앵커의 위치 좌표를 NUM_ANCHORS(2)에 맞게 정리
Point2D pos_anchors[NUM_ANCHORS] = {
    {0.0f, 0.0f}, // 앵커1 위치 (index 0)
    {0.0f, 0.0f}  // 앵커2 위치 (index 1)
};

volatile dwr_state state = S_IDLE;

void responder_init(void)
{
//    dwt_forcetrxoff(); // 현재 송수신 중지

    //example_pointer = ds_twr_responder_irq;

    SPI_SetPrescaler_ReInit(&hspi1, 32); // SPI 속도 느리게 설정

    HAL_TIM_Base_Stop_IT(&htim2);
    HAL_TIM_Base_Stop_IT(&htim3);

    example_pointer();
}

void initiator_init(void)
{
//    dwt_forcetrxoff(); // 현재 송수신 중지
    
    // 🌟 수정 3: 치명적 오류 수정! (ds_twr_responder_irq로 잘못 연결되어 있었음)
    example_pointer = ds_twr_initiator_irq;

    SPI_SetPrescaler_ReInit(&hspi1, 32); // SPI 속도 느리게 설정

    HAL_TIM_Base_Stop_IT(&htim2);
    HAL_TIM_Base_Stop_IT(&htim3);

    example_pointer();
}

void initiator_map_init(void)
{
//    dwt_forcetrxoff(); // 현재 송수신 중지

    example_pointer = ds_twr_initiator_irq; // 🌟 여기도 initiator로 수정

    SPI_SetPrescaler_ReInit(&hspi1, 32); // SPI 속도 느리게 설정

    HAL_TIM_Base_Stop_IT(&htim2);
    HAL_TIM_Base_Stop_IT(&htim3);

    example_pointer();
}

void initiator_single_init(void)
{
//    dwt_forcetrxoff(); // 현재 송수신 중지

    example_pointer = ds_twr_initiator_irq; // 🌟 여기도 initiator로 수정

    SPI_SetPrescaler_ReInit(&hspi1, 32); // SPI 속도 느리게 설정

    HAL_TIM_Base_Stop_IT(&htim2);
    HAL_TIM_Base_Stop_IT(&htim3);

    example_pointer();
}

void responder_single_init(void)
{
//    dwt_forcetrxoff(); // 현재 송수신 중지

    //example_pointer = ds_twr_responder_irq;

    SPI_SetPrescaler_ReInit(&hspi1, 32); // SPI 속도 느리게 설정

    HAL_TIM_Base_Stop_IT(&htim2);
    HAL_TIM_Base_Stop_IT(&htim3);

    example_pointer();
}

void SPI_SetPrescaler_ReInit(SPI_HandleTypeDef *hspi, uint32_t prescaler)
{
    // 진행 중 전송이 있으면 안전하게 중단
    HAL_SPI_Abort(hspi);

    hspi->Init.BaudRatePrescaler = prescaler;

    HAL_SPI_DeInit(hspi);
    if (HAL_SPI_Init(hspi) != HAL_OK) {
        Error_Handler(); // 사용자 정의 에러 처리
    }
}
