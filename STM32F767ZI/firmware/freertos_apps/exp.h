#ifndef EXP_H
#define EXP_H

#include "stdio.h"
#include "stdlib.h"
#include "stdbool.h"

//数据缓冲器大小
#define BUFFER_SIZE         300
//回调的执行时间us
#define EXECUTE_TIME        10000
//任务链的个数
#define CHAIN_NUM           3
//PQNode结点中数据指针数组的最大长度
#define EXAMPLE_MAX_NUM     5
//PQNode_Pool中PQNode结点的最大个数
#define PQNODE_MAX_NUM      10

#define UART_DMA_BUFFER_SIZE    4096
#define UART_RX_BUFFER_SIZE     1024
extern uint8_t dma_buffer[UART_DMA_BUFFER_SIZE];
extern size_t dma_head, dma_tail, temp_head, temp_tail;
extern uint8_t UART_RX_BUF[UART_RX_BUFFER_SIZE];
extern size_t UART_RX_STA;
extern int init_flag;

extern uint32_t execute_start, execute_end, transmit_end, receive_start, action_start;
extern int flag;

extern int first_flag;
extern int64_t union_next_call_time;
extern int64_t union_last_call_time;

#define HEADER_SIZE         15

//开启TIDE执行器
#define TIDE_EXECUTOR       1
//开启Parallel Transmission
#define DMA_DAEMON_SEND     1
//开启Interrupt-based data reception
#define DMA_DAEMON_RECEIVE  1

extern uint8_t pro20[500], pro40[500], pro60[500], pro80[500];
extern int index1, index2, index3;
extern int flag1, flag2, flag3;
extern bool global_start;

#define RELIABLE            0
#define SUCCESS_PROBAILITY  0

#endif