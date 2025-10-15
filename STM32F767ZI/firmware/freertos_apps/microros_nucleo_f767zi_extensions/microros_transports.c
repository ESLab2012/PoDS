#include <uxr/client/transport.h>
#include "stm32f7xx_hal.h"
#include "stm32f7xx_hal_dma.h"

#include <unistd.h>
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <stdint.h>
#include "malloc.h"

#include "microros_transports.h"

#include "FreeRTOS.h"
#include "task.h"

// Replace with the correct absolute path
#include "~/firmware/freertos_apps/exp.h"

// --- micro-ROS Transports ---
int init_flag = 0;

int first_flag = 0;
int64_t union_next_call_time;
int64_t union_last_call_time;

uint8_t dma_buffer[UART_DMA_BUFFER_SIZE];
size_t dma_head = 0, dma_tail = 0, temp_head, temp_tail;

uint8_t UART_RX_BUF[UART_RX_BUFFER_SIZE];
size_t UART_RX_STA = 0;

static UART_HandleTypeDef* global_uart;

uint32_t execute_start, execute_end, transmit_end, receive_start, action_start;
int flag = 1;

uint8_t pro20[500] = {
0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 1, 1, 1, 1, 0, 0, 1, 0, 1, 1, 0, 1, 0, 1, 1, 1, 1, 0, 0, 0, 0, 1, 1, 1, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0,
0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 0, 0, 0, 1, 0, 0, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0,
0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0,
0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 1, 0, 0, 1, 1, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0,
0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 1, 0, 1, 0, 0, 0, 0, 0, 1, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 1, 0, 1, 0, 0, 0,
0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0,
1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 1, 1, 1, 1, 1, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1, 0, 1, 0, 0, 0, 1, 1, 0, 0, 1, 1, 0, 1, 0, 0, 0, 0
};

uint8_t pro40[500] = {
0, 0, 0, 0, 0, 1, 1, 0, 0, 1, 0, 0, 1, 0, 1, 0, 1, 0, 0, 1, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 0, 1, 0, 1, 1, 0, 1, 0, 1, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0,
1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 1, 0, 0, 1, 0, 1, 0, 0, 1, 1, 0, 1, 0, 0, 0, 0, 0, 1, 0, 1, 0, 0, 1, 0, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 1, 0,
1, 1, 0, 0, 0, 1, 1, 1, 0, 1, 0, 0, 1, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 1, 0, 0, 0, 0, 0, 1, 0, 1, 0, 0, 1, 0, 1, 1, 0, 1, 0, 0, 0, 1, 1, 1, 1,
0, 1, 0, 0, 1, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 1, 0, 0, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 1, 1, 1, 0, 0, 0,
1, 1, 0, 1, 0, 1, 1, 0, 0, 1, 0, 1, 1, 0, 1, 0, 1, 0, 0, 0, 1, 0, 1, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 1, 0, 0, 1, 0, 0, 1, 0, 0, 0, 0,
0, 1, 1, 0, 0, 1, 0, 0, 1, 1, 0, 0, 0, 1, 0, 0, 1, 0, 1, 1, 1, 0, 1, 0, 1, 1, 1, 0, 0, 0, 0, 1, 0, 0, 1, 0, 1, 1, 0, 1, 1, 0, 0, 1, 0, 0, 0, 1, 0, 0,
0, 0, 0, 0, 0, 1, 0, 1, 0, 1, 1, 0, 0, 1, 0, 0, 1, 1, 0, 1, 1, 1, 0, 1, 0, 1, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 1, 1, 1, 1, 0, 0, 1, 1, 1, 1, 1, 0, 0,
1, 1, 1, 1, 1, 0, 0, 1, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 0, 1, 1, 0, 0, 0, 1, 0, 1, 0, 1, 0, 0, 0, 1, 0, 1, 1, 0, 1, 1, 1, 0, 0, 0, 0,
0, 0, 0, 0, 0, 1, 0, 0, 0, 1, 1, 0, 0, 0, 0, 1, 0, 0, 1, 0, 0, 1, 0, 0, 1, 0, 0, 0, 1, 1, 1, 0, 0, 0, 1, 0, 1, 0, 1, 1, 0, 1, 1, 0, 0, 1, 0, 1, 1, 0,
0, 1, 1, 1, 0, 1, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 0, 0, 0, 1, 1, 0, 1, 1, 0, 0, 0, 1, 0, 1, 1, 0, 0, 0, 1, 1, 0, 0, 1, 0, 1, 0, 1, 1, 1, 0, 0, 0, 0, 0
};

uint8_t pro60[500] = {
1, 0, 1, 0, 1, 1, 0, 0, 1, 1, 1, 0, 1, 1, 0, 0, 1, 1, 1, 1, 0, 0, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 0, 0, 1, 1, 1, 0, 1, 0, 0, 0, 1, 1, 1, 1, 1, 0, 1, 0,
1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 1, 1, 0, 1, 1, 1, 0, 0, 0, 1, 0, 0, 1, 0, 1, 1, 0, 0, 1, 1, 1, 1, 0, 0, 0, 1, 1, 0, 0, 0, 1,
1, 0, 1, 1, 0, 1, 1, 0, 1, 1, 1, 1, 0, 0, 1, 1, 0, 1, 1, 1, 0, 1, 0, 1, 0, 1, 1, 1, 0, 0, 1, 0, 0, 0, 0, 1, 0, 1, 0, 1, 1, 0, 1, 0, 1, 1, 0, 1, 1, 0,
0, 0, 0, 1, 0, 1, 1, 0, 0, 0, 1, 1, 1, 0, 0, 1, 0, 0, 1, 1, 1, 1, 1, 1, 0, 0, 0, 1, 1, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 0, 1, 0, 0, 1, 1, 1, 1, 1, 0, 0,
1, 1, 1, 1, 0, 1, 0, 1, 1, 0, 0, 0, 0, 1, 1, 1, 0, 0, 1, 1, 1, 1, 0, 1, 1, 0, 1, 0, 1, 1, 1, 0, 1, 0, 0, 1, 1, 0, 1, 1, 0, 1, 1, 0, 0, 0, 0, 0, 1, 0,
1, 0, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 0, 1, 1, 1, 0, 1, 1, 0, 0, 1, 1, 1, 1, 1, 0, 1, 0, 0, 1,
0, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 1, 1, 1, 1, 0, 1, 1, 0, 1, 0, 0, 0, 1, 1, 1, 0, 1, 0, 0, 0, 1, 1, 1, 0, 1, 1, 0, 0, 1, 1, 1, 1, 0, 0, 0, 0, 0,
1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 1, 1, 1, 0, 1, 1, 0, 1, 1, 0, 0, 0, 0, 1, 1, 0, 1, 0, 1, 0, 1, 1, 0, 1, 1, 0, 1, 1, 1, 0, 0, 0, 1, 0, 0, 0, 1, 1, 0, 0,
0, 0, 1, 1, 0, 1, 1, 0, 0, 1, 0, 0, 0, 1, 0, 1, 1, 0, 1, 1, 0, 1, 1, 0, 1, 0, 1, 0, 0, 0, 0, 1, 1, 0, 1, 0, 1, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 1, 1, 1,
1, 0, 1, 0, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 1, 1, 0, 1, 1, 1, 0, 0, 0, 0, 1, 0, 1, 1, 1, 0, 1, 0, 0, 1, 1, 0, 1, 1, 0, 0, 1, 1, 1, 0
};

uint8_t pro80[500] = {
0, 1, 0, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 1, 0, 1, 1, 1, 1, 1, 1,
1, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 0, 0, 0, 1, 0, 1, 1, 0, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 0, 0, 1, 1, 1, 1, 1, 1, 1, 0,
1, 1, 1, 0, 0, 1, 1, 1, 1, 1, 1, 0, 1, 1, 0, 0, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 0, 1, 1, 1, 1, 0, 1, 1, 0, 0, 1, 1, 1, 1, 0, 1, 0, 1, 1, 0, 1, 1, 1, 1,
1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 0, 1, 0, 1, 1, 0, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1,
1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
0, 1, 1, 1, 1, 1, 0, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 0, 0, 1, 1, 1, 1, 0, 1, 1, 0, 1, 1, 0, 1, 1, 0, 1, 0, 1, 1, 1, 1,
1, 1, 0, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 0, 1, 0, 1, 0, 1, 1, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1,
1, 0, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 0, 0, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 1, 1, 1, 1, 0, 1,
0, 0, 1, 1, 1, 1, 0, 1, 1, 1, 0, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 1,
1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 0, 0, 1, 1, 1, 1, 0, 0, 1, 1, 0, 0, 1, 0, 1, 1, 1, 0, 1, 1, 0, 1, 1
};

int index1 = 0, index2 = 0, index3 = 0;
bool global_start = false;
int flag1 = 0, flag2 = 0, flag3 = 0;
//=================================================================================================
//                                       dma_daemon_send
//=================================================================================================
struct ListNode{
    //记录数据包对应的回调的优先级
    int priority;
    //记录数据包入队的时间戳
    uint32_t time_stamp;
    //记录数据包和数据包长度
    uint8_t data[BUFFER_SIZE];
    size_t data_length;
    //记录数据包对应的回调的QoS
    //为简便，qos==0表示Best-effort，qos==1表示Reliable
    uint8_t qos;
    //用来记录是否应该发送正确的消息
    int count;
    //用来发送错误的数据包
    uint8_t false_data[BUFFER_SIZE];
    struct ListNode *pre;
    struct ListNode *next;
};
typedef struct ListNode DPNode;

static DPNode *DataPool, *ExecutionBuffer, *PendingBuffer, *current_execution;

static void DP_Init(){
    ExecutionBuffer = (DPNode*)malloc(sizeof(DPNode));
    ExecutionBuffer->next = NULL;
    ExecutionBuffer->pre = NULL;
    PendingBuffer = (DPNode*)malloc(sizeof(DPNode));
    PendingBuffer->next = NULL;
    PendingBuffer->pre = NULL;
    DataPool = (DPNode*)malloc(sizeof(DPNode));
    DPNode *pre = DataPool;
    for(int i = 0; i < PQNODE_MAX_NUM; ++i){
        pre->next = (DPNode*)malloc(sizeof(DPNode));
        pre->next->pre = pre;
        pre = pre->next;
        for(int i = 0;i < BUFFER_SIZE;++i){
            pre->false_data[i] = 0;
        }
    }
    pre->next = NULL;
}

static DPNode* get_DPNode(){
    if(DataPool->next == NULL){
        // printf("No idle DPNode in DataPool !!!\n");
        return NULL;
    }
    DPNode *temp = DataPool->next;
    DataPool->next = temp->next;
    temp->next->pre = DataPool;
    temp->next = temp->pre = NULL;
    return temp;
}

//将DPNode结点p加入DataPool中
static void put_DPNode(DPNode *p){
    p->next = DataPool->next;
    p->next->pre = p;
    p->pre = DataPool;
    DataPool->next = p;
}

static void En_ExecutionBuffer(DPNode *p){
    __disable_irq();
    DPNode* pre = ExecutionBuffer;
    //ExecutionBuffer按照优先级递减排序
    while(pre->next != NULL && pre->next->priority >= p->priority){
        pre = pre->next;
    }
    if(pre->next == NULL){
        pre->next = p;
        p->pre = pre;
        p->next = NULL;
    }else{
        p->next = pre->next;
        p->next->pre = p;
        p->pre = pre;
        pre->next = p;
    }
    __enable_irq();
}

static void En_PendingBuffer(DPNode *p){
    DPNode* pre = PendingBuffer;
    //PendingBuffer按照时间戳递增排序
    while(pre->next != NULL && pre->next->time_stamp < p->time_stamp){
        pre = pre->next;
    }
    if(pre->next == NULL){
        pre->next = p;
        p->pre = pre;
        p->next = NULL;
    }else{
        p->next = pre->next;
        p->next->pre = p;
        p->pre = pre;
        pre->next = p;
    }
}
//=================================================================================================

bool freertos_serial_open(struct uxrCustomTransport * transport){
    UART_HandleTypeDef * uart = (UART_HandleTypeDef*) transport->args;
    global_uart = uart;
#if(DMA_DAEMON_SEND == 1)
    DP_Init();
#endif
#if(DMA_DAEMON_RECEIVE == 0)
    HAL_UART_Receive_DMA(uart, dma_buffer, UART_DMA_BUFFER_SIZE);
#elif(DMA_DAEMON_RECEIVE == 1)
    HAL_UART_Receive_DMA(uart, UART_RX_BUF, UART_RX_BUFFER_SIZE);
    __HAL_UART_ENABLE_IT(uart, UART_IT_IDLE);
#endif
    return true;
}

bool freertos_serial_close(struct uxrCustomTransport * transport){
    UART_HandleTypeDef * uart = (UART_HandleTypeDef*) transport->args;
    HAL_UART_DMAStop(uart);
    return true;
}

size_t freertos_serial_write(struct uxrCustomTransport* transport, const uint8_t * buf, size_t len, uint8_t * err){
    UART_HandleTypeDef * uart = (UART_HandleTypeDef*) transport->args;

    HAL_StatusTypeDef ret;
    if (uart->gState == HAL_UART_STATE_READY){
        ret = HAL_UART_Transmit_DMA(uart, buf, len);
        while (ret == HAL_OK && uart->gState != HAL_UART_STATE_READY){
        osDelay(1);
        }
        
        return (ret == HAL_OK) ? len : 0;
    }else{
        return 0;
    }
}

static bool en_flag = false;
//micro-ROS与ROS 2 Agent建立链接时，DMA传输通道工作完成后不能调用完成回调函数
size_t freertos_serial_write_new(struct uxrCustomTransport* transport, const uint8_t * buf, size_t len, uint8_t * err, int priority, uint8_t QoS){
    //1.从DataPool中申请一个DPNode结点
    en_flag = true;//变相关闭中断
    DPNode* temp = get_DPNode();
    en_flag = false;//变相开启中断

    uint32_t current_time;
    while(temp == NULL){
        //当前DataPool中没有可用的DPNode结点
        if(PendingBuffer->next == NULL){
            //如果此时PendingBuffer为空
            //等待可用的DPNode结点的出现，或者等待PendingBuffer中时间戳最早的结点过期
            osDelay(1);
        }else{
            //如果此时PendingBuffer不为空，检查Pending Buffer中时间戳最早的结点
            DPNode* temp2 = PendingBuffer->next;
            current_time = xTaskGetTickCount();
            if(current_time - temp2->time_stamp >= 100){
                //Pending Buffer中时间戳最早的结点距离当前时间超过1ms，视为过期
                printf("A message has not received confirmation and has expired !!!\n");
                //将该结点从PendingBuffer上取下来
                PendingBuffer->next = temp2->next;
                if(temp2->next != NULL){
                    temp2->next->pre = PendingBuffer;
                }
                temp = temp2;
                break;
            }else{
                osDelay(1);
            }
        }
        temp = get_DPNode();
    }
    //2.填充DPNode结点
    temp->data_length = len;
    memcpy(temp->data, buf, len);
    temp->time_stamp = xTaskGetTickCount();
    temp->priority = priority;
    temp->qos = QoS;
#if(RELIABLE == 1)
    current_execution->qos = 1;  
    uint8_t flag = 0;
    switch (SUCCESS_PROBAILITY)
    {
        case 0:
            flag = 0;
            break;
        case 20:
            if(flag1 == 1){
                flag = pro20[index1];
                index1++;
            }else if(flag2 == 1){
                flag = pro20[index2];
                index2++;
            }else if(flag3 == 1){
                flag = pro20[index3];
                index3++;
            }
            break;
        case 40:
            if(flag1 == 1){
                flag = pro40[index1];
                index1++;
            }else if(flag2 == 1){
                flag = pro40[index2];
                index2++;
            }else if(flag3 == 1){
                flag = pro40[index3];
                index3++;
            }
            break;
        case 60:
            if(flag1 == 1){
                flag = pro60[index1];
                index1++;
            }else if(flag2 == 1){
                flag = pro60[index2];
                index2++;
            }else if(flag3 == 1){
                flag = pro60[index3];
                index3++;
            }
            break;
        case 80:
            if(flag1 == 1){
                flag = pro80[index1];
                index1++;
            }else if(flag2 == 1){
                flag = pro80[index2];
                index2++;
            }else if(flag3 == 1){
                flag = pro80[index3];
                index3++;
            }
            break;
        case 100:
            flag = 1;
            break;
    }
    if(flag == 1){
        current_execution->count = 1;
    }else{
        current_execution->count = 2;
    }
#endif

    en_flag = true;//变相关闭中断
    En_ExecutionBuffer(temp);
    en_flag = false;//变相开启中断

    //4.检查DMA传输通道的状态
    UART_HandleTypeDef * uart = (UART_HandleTypeDef*) transport->args;
    if (uart->gState == HAL_UART_STATE_READY){
        //ExecutionBuffer不可能为空
        if(init_flag == 0){
            init_flag++;
        }
        //记录正在传输数据的当前DPNode结点
        current_execution = ExecutionBuffer->next;
        //将当前DPNode结点从ExecutionBuffer上取下来
        ExecutionBuffer->next = current_execution->next;
        if(current_execution->next != NULL){
            current_execution->next->pre = ExecutionBuffer;
        }
        if(current_execution->qos == 1 && current_execution->count != 1){
            HAL_UART_Transmit_DMA(uart, current_execution->false_data, current_execution->data_length);
        }else if(current_execution->qos == 1 && current_execution->count == 1){
            HAL_UART_Transmit_DMA(uart, current_execution->data, current_execution->data_length);
        }else{
            HAL_UART_Transmit_DMA(uart, current_execution->data, current_execution->data_length);
        }
        return len;
    }
    //如果当前DMA传输通道不空闲，不用进行任何操作，DMA传输通道工作完成后自动调用完成回调函数
    return len;
}

#if(DMA_DAEMON_SEND == 1)
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
// void UART_DMATransmitCplt(UART_HandleTypeDef *huart){
    if(init_flag != 0 && en_flag == false){
        //init_flag == 0表明连接尚未成功建立
        //en_flag == true表明正在操作current_execution，不能去打扰
        //判断传输数据完成的当前DPNode结点的QoS
        if(current_execution->qos == 0 && current_execution->priority != 0){
            //直接回收当前DPNode结点
            current_execution->priority = 0;
            put_DPNode(current_execution);
        }else if(current_execution->qos == 1){
            //将当前DPNode结点加入PendingBuffer
            // En_PendingBuffer(current_execution);
            if(current_execution->count == 1){
                //直接回收当前结点
                put_DPNode(current_execution);
            }else{
                current_execution->count--;
                //将当前结点重新加入发送队列中
                en_flag = true;//变相关闭中断
                En_ExecutionBuffer(current_execution);
                en_flag = false;//变相开启中断
            }
        }
        //如果当前ExecutionBuffer不为NULL，自动配置并激活DMA传输通道
        if(ExecutionBuffer->next != NULL){
            current_execution = ExecutionBuffer->next;
            //将当前DPNode结点从ExecutionBuffer上取下来
            ExecutionBuffer->next = current_execution->next;
            if(current_execution->next != NULL){
                current_execution->next->pre = ExecutionBuffer;
            }
            if(current_execution->qos == 1 && current_execution->count != 1){
                HAL_UART_Transmit_DMA(huart, current_execution->false_data, current_execution->data_length);
            }else if(current_execution->qos == 1 && current_execution->count == 1){
                HAL_UART_Transmit_DMA(huart, current_execution->data, current_execution->data_length);
            }else{
                HAL_UART_Transmit_DMA(huart, current_execution->data, current_execution->data_length);
            }
        }
    }
}
#endif

bool Check_DMA_buffer(){

#if(DMA_DAEMON_RECEIVE == 0)
    dma_tail = UART_DMA_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(global_uart->hdmarx);
#endif

    return dma_head != dma_tail;
}

size_t freertos_serial_read(struct uxrCustomTransport* transport, uint8_t* buf, size_t len, int timeout, uint8_t* err){

#if(DMA_DAEMON_RECEIVE == 0)
    UART_HandleTypeDef * uart = (UART_HandleTypeDef*) transport->args;
    __disable_irq();
    dma_tail = UART_DMA_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(uart->hdmarx);
    __enable_irq();
#endif

    size_t wrote = 0;
    while ((dma_head != dma_tail) && (wrote < len)){
        buf[wrote] = dma_buffer[dma_head];
        dma_head = (dma_head + 1) % UART_DMA_BUFFER_SIZE;
        wrote++;
    }
    return wrote;
}

void freertos_serial_read_new(uint8_t* src_buf, int len, int pos){
    //将起始位置未temp_head,长度为len的数据复制到src_buf中
    int i = 0;
    while(i < len){
        src_buf[i] = dma_buffer[(pos + i) % UART_DMA_BUFFER_SIZE];
        i++;
    }
}

void write_to_dma_buffer(int pos, uint8_t data){
    dma_buffer[pos % UART_DMA_BUFFER_SIZE] = data;
}