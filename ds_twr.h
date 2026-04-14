/*
 * ds_twr.h
 *
 *  Created on: Sep 23, 2025
 *      Author: jjw
 */

#ifndef DS_TWR_H_
#define DS_TWR_H_
#define NUM_ANCHORS 2 // 앵커 개수
//#define ANCHOR 0 // 이 앵커의 번호 , 0 이면 initiator

/* Length of the common part of the message (up to and including the function code, see NOTE 2 below). */
#define ALL_MSG_COMMON_LEN 10

#define IMU_DATA_SIZE 12
#define MSG_IMU_IDX   ALL_MSG_COMMON_LEN

#define ALL_MSG_SN_IDX            2
#define FINAL_MSG_POLL_TX_TS_IDX  10
#define FINAL_MSG_RESP_RX_TS_IDX  15
#define FINAL_MSG_FINAL_TX_TS_IDX 20

#define FINAL_MSG_POS_X_IDX 25
#define FINAL_MSG_POS_Y_IDX 30

/* Buffer to store received response message.
 * Its size is adjusted to longest frame that this example code is supposed to handle. */
#define RX_BUF_LEN 40  // resp msg와 final_ts msg 길이 이상으로 설정

extern Point2D pos_anchors[NUM_ANCHORS];


#endif /* DS_TWR_H_ */
