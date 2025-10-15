#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/header.h>

#include <stdio.h>
#include <unistd.h>
#include <time.h>

// Replace with the correct absolute path
#include "~/firmware/freertos_apps/exp.h"
#include "~/firmware/mcu_ws/uros/rclc/rclc/include/rclc/priority_queue.h"

#ifdef ESP_PLATFORM
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#endif

#define STRING_BUFFER_LEN BUFFER_SIZE

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Aborting.\n",__LINE__,(int)temp_rc); vTaskDelete(NULL);}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Continuing.\n",__LINE__,(int)temp_rc);}}

rcl_publisher_t sensor_publisher;
rcl_subscription_t action_subscriber;
std_msgs__msg__Header action_incoming;
std_msgs__msg__Header sensor_data;

rcl_publisher_t second_publisher;
rcl_subscription_t second_subscriber;
std_msgs__msg__Header second_incoming;
std_msgs__msg__Header second_data;

rcl_publisher_t third_publisher;
rcl_subscription_t third_subscriber;
std_msgs__msg__Header third_incoming;
std_msgs__msg__Header third_data;

rcl_publisher_t four_publisher;
rcl_subscription_t four_subscriber;
std_msgs__msg__Header four_incoming;
std_msgs__msg__Header four_data;

rcl_publisher_t five_publisher;
rcl_subscription_t five_subscriber;
std_msgs__msg__Header five_incoming;
std_msgs__msg__Header five_data;

int num = 0;
uint32_t start1, end1;
void sensor_timer_callback(rcl_timer_t * timer, int64_t last_call_time){
	RCLC_UNUSED(last_call_time);
	if(timer != NULL){
		global_start = true;
		flag1 = 1;
		start1 = xTaskGetTickCount();

		usleep(5000);

		sprintf(sensor_data.frame_id.data, "%-10d", num);
		sensor_data.frame_id.size = strlen(sensor_data.frame_id.data);
#if(DMA_DAEMON_SEND == 0)
		RCSOFTCHECK(rcl_publish(&sensor_publisher, (const void*)&sensor_data, NULL));
		flag1 = 0;
#endif
#if(DMA_DAEMON_SEND == 1)
		RCSOFTCHECK(rcl_publish_new(&sensor_publisher, (const void*)&sensor_data, NULL, 51));
		flag1 = 0;
#endif

		num++;
	}
}

void action_subscription_callback(const void * msgin){
	const std_msgs__msg__Header * msg = (const std_msgs__msg__Header *)msgin;
	int receive_num = 0;
	for(int i = 0; i < msg->frame_id.size; ++i){
		if(msg->frame_id.data[i] >= '0' && msg->frame_id.data[i] <= '9'){
			receive_num = receive_num * 10 + (msg->frame_id.data[i] - '0');
		}else{
			break;
		}
	}
	usleep(8000);

	end1 = xTaskGetTickCount();

	printf("==========|11111| %d : %d\n", receive_num, end1 - start1);
}

int num2 = 0;
uint32_t start2, end2;
void second_timer_callback(rcl_timer_t * timer, int64_t last_call_time){
	RCLC_UNUSED(last_call_time);
	if(timer != NULL){
		flag2 = 1;
		start2 = xTaskGetTickCount();

		usleep(10000);

		sprintf(second_data.frame_id.data, "%-15d", num2);
		second_data.frame_id.size = strlen(second_data.frame_id.data);
#if(DMA_DAEMON_SEND == 0)
		RCSOFTCHECK(rcl_publish(&second_publisher, (const void*)&second_data, NULL));
		flag2 = 0;
#endif
#if(DMA_DAEMON_SEND == 1)
		RCSOFTCHECK(rcl_publish_new(&second_publisher, (const void*)&second_data, NULL, 41));
		flag2 = 0;
#endif
		num2++;
	}
}

void second_subscription_callback(const void * msgin){
	const std_msgs__msg__Header * msg = (const std_msgs__msg__Header *)msgin;
	int receive_num = 0;
	for(int i = 0; i < msg->frame_id.size; ++i){
		if(msg->frame_id.data[i] >= '0' && msg->frame_id.data[i] <= '9'){
			receive_num = receive_num * 10 + (msg->frame_id.data[i] - '0');
		}else{
			break;
		}
	}

	usleep(6000);

	end2 = xTaskGetTickCount();
	printf("==========|22222| %d : %d\n", receive_num, end2 - start2);
}

int num3 = 0;
uint32_t start3, end3;
void third_timer_callback(rcl_timer_t * timer, int64_t last_call_time){
	RCLC_UNUSED(last_call_time);
	if(timer != NULL){
		flag3 = 1;
		start3 = xTaskGetTickCount();

		usleep(18000);

		sprintf(third_data.frame_id.data, "%-20d", num3);
		third_data.frame_id.size = strlen(third_data.frame_id.data);
#if(DMA_DAEMON_SEND == 0)
		RCSOFTCHECK(rcl_publish(&third_publisher, (const void*)&third_data, NULL));
		flag3 = 0;
#endif
#if(DMA_DAEMON_SEND == 1)
		RCSOFTCHECK(rcl_publish_new(&third_publisher, (const void*)&third_data, NULL, 31));
		flag3 = 0;
#endif
		num3++;
	}
}

void third_subscription_callback(const void * msgin){
	const std_msgs__msg__Header * msg = (const std_msgs__msg__Header *)msgin;
	int receive_num = 0;
	for(int i = 0; i < msg->frame_id.size; ++i){
		if(msg->frame_id.data[i] >= '0' && msg->frame_id.data[i] <= '9'){
			receive_num = receive_num * 10 + (msg->frame_id.data[i] - '0');
		}else{
			break;
		}
	}

	usleep(7000);

	uint32_t end3 = xTaskGetTickCount();
	printf("==========|33333| %d : %d\n", receive_num, end3 - start3);
}

int num4 = 0;
uint32_t start4, end4;
void four_timer_callback(rcl_timer_t * timer, int64_t last_call_time){
	RCLC_UNUSED(last_call_time);
	if(timer != NULL){
		start4 = xTaskGetTickCount();

		usleep(18000);

		sprintf(four_data.frame_id.data, "%d", num4);
		four_data.frame_id.size = strlen(four_data.frame_id.data);
#if(DMA_DAEMON_SEND == 0)
		RCSOFTCHECK(rcl_publish(&four_publisher, (const void*)&four_data, NULL));
#endif
#if(DMA_DAEMON_SEND == 1)
		RCSOFTCHECK(rcl_publish_new(&four_publisher, (const void*)&four_data, NULL, 21));
#endif
		num4++;
	}
}

void four_subscription_callback(const void * msgin){
	const std_msgs__msg__Header * msg = (const std_msgs__msg__Header *)msgin;
	int receive_num = 0;
	for(int i = 0; i < msg->frame_id.size; ++i){
		if(msg->frame_id.data[i] >= '0' && msg->frame_id.data[i] <= '9'){
			receive_num = receive_num * 10 + (msg->frame_id.data[i] - '0');
		}else{
			break;
		}
	}

	usleep(7000);

	end4 = xTaskGetTickCount();
	printf("==========|44444| %d : %d\n", receive_num, end4 - start4);
}

int num5 = 0;
uint32_t start5, end5;
void five_timer_callback(rcl_timer_t * timer, int64_t last_call_time){
	RCLC_UNUSED(last_call_time);
	if(timer != NULL){
		uint32_t start5 = xTaskGetTickCount();

		usleep(EXECUTE_TIME);

		sprintf(five_data.frame_id.data, "%d", num5);
		five_data.frame_id.size = strlen(five_data.frame_id.data);
#if(DMA_DAEMON_SEND == 0)
		RCSOFTCHECK(rcl_publish(&five_publisher, (const void*)&five_data, NULL));
#endif
#if(DMA_DAEMON_SEND == 1)
		RCSOFTCHECK(rcl_publish_new(&five_publisher, (const void*)&five_data, NULL, 11));
#endif
		num5++;
	}
}

void five_subscription_callback(const void * msgin){
	const std_msgs__msg__Header * msg = (const std_msgs__msg__Header *)msgin;
	int receive_num = 0;
	for(int i = 0; i < msg->frame_id.size; ++i){
		if(msg->frame_id.data[i] >= '0' && msg->frame_id.data[i] <= '9'){
			receive_num = receive_num * 10 + (msg->frame_id.data[i] - '0');
		}else{
			break;
		}
	}

	usleep(EXECUTE_TIME);

	uint32_t end5 = xTaskGetTickCount();
	printf("==========|55555| %d : %d\n", receive_num, end5 - start5);
}

void appMain(void *argument)
{
	rcl_allocator_t allocator = rcl_get_default_allocator();
	rclc_support_t support;

	// create init_options
	RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

	// create node
	rcl_node_t node;
	RCCHECK(rclc_node_init_default(&node, "pingpong_node", "", &support));

#if( CHAIN_NUM >= 1 )
	RCCHECK(rclc_publisher_init_best_effort(&sensor_publisher, &node,
	 	ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Header), "/microROS/sensor"));
	// Create a best effort action subscriber
	RCCHECK(rclc_subscription_init_best_effort_new(&action_subscriber, &node, 52,
		ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Header), "/microROS/action"));
	rcl_timer_t sensor_timer;
	RCCHECK(rclc_timer_init_default(&sensor_timer, &support, RCL_MS_TO_NS(300), sensor_timer_callback));
#endif
#if( CHAIN_NUM >= 2 )
	RCCHECK(rclc_publisher_init_best_effort(&second_publisher, &node,
 	 	ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Header), "/microROS/second_publish"));
	RCCHECK(rclc_subscription_init_best_effort_new(&second_subscriber, &node, 42,
		ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Header), "/microROS/second_receive"));
	rcl_timer_t second_timer;
	RCCHECK(rclc_timer_init_default(&second_timer, &support, RCL_MS_TO_NS(300), second_timer_callback));
#endif
#if( CHAIN_NUM >= 3 )
	RCCHECK(rclc_publisher_init_best_effort(&third_publisher, &node,
 		ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Header), "/microROS/third_publish"));
	RCCHECK(rclc_subscription_init_best_effort_new(&third_subscriber, &node, 32,
		ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Header), "/microROS/third_receive"));
	rcl_timer_t third_timer;
	RCCHECK(rclc_timer_init_default(&third_timer, &support, RCL_MS_TO_NS(300), third_timer_callback));
#endif
#if( CHAIN_NUM >= 4 )
	RCCHECK(rclc_publisher_init_best_effort(&four_publisher, &node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Header), "/microROS/four_publish"));
	RCCHECK(rclc_subscription_init_best_effort_new(&four_subscriber, &node, 22,
		ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Header), "/microROS/four_receive"));
	rcl_timer_t four_timer;
	RCCHECK(rclc_timer_init_default(&four_timer, &support, RCL_MS_TO_NS(300), four_timer_callback));
#endif
#if( CHAIN_NUM >= 5 )
	RCCHECK(rclc_publisher_init_best_effort(&five_publisher, &node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Header), "/microROS/five_publish"));
	RCCHECK(rclc_subscription_init_best_effort_new(&five_subscriber, &node, 12,
		ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Header), "/microROS/five_receive"));
	rcl_timer_t five_timer;
	RCCHECK(rclc_timer_init_default(&five_timer, &support, RCL_MS_TO_NS(300), five_timer_callback));
#endif

	// Create executor
	rclc_executor_t executor;
	RCCHECK(rclc_executor_init(&executor, &support.context, 10, &allocator));

#if( TIDE_EXECUTOR == 0 )
#if( CHAIN_NUM >= 1 )
	RCCHECK(rclc_executor_add_subscription(&executor, &action_subscriber, &action_incoming,
		&action_subscription_callback, ON_NEW_DATA));
	RCCHECK(rclc_executor_add_timer(&executor, &sensor_timer));
#endif
#if( CHAIN_NUM >= 2 )
	RCCHECK(rclc_executor_add_subscription(&executor, &second_subscriber, &second_incoming,
		&second_subscription_callback, ON_NEW_DATA));
	RCCHECK(rclc_executor_add_timer(&executor, &second_timer));
#endif
#if( CHAIN_NUM >= 3 )
	RCCHECK(rclc_executor_add_subscription(&executor, &third_subscriber, &third_incoming,
		&third_subscription_callback, ON_NEW_DATA));
	RCCHECK(rclc_executor_add_timer(&executor, &third_timer));
#endif
#if( CHAIN_NUM >= 4 )
	RCCHECK(rclc_executor_add_subscription(&executor, &four_subscriber, &four_incoming,
		&four_subscription_callback, ON_NEW_DATA));
	RCCHECK(rclc_executor_add_timer(&executor, &four_timer));
#endif
#if( CHAIN_NUM >= 5 )
	RCCHECK(rclc_executor_add_subscription(&executor, &five_subscriber, &five_incoming,
		&five_subscription_callback, ON_NEW_DATA));
	RCCHECK(rclc_executor_add_timer(&executor, &five_timer));
#endif
#endif

#if( TIDE_EXECUTOR == 1 )
#if( CHAIN_NUM >= 1 )
	RCCHECK(rclc_executor_add_timer_new(&executor, &sensor_timer, 51));
	RCCHECK(rclc_executor_add_subscription_new(&executor, &action_subscriber, &action_incoming, 52,
		&action_subscription_callback, ON_NEW_DATA));
#endif
#if( CHAIN_NUM >= 2 )
	RCCHECK(rclc_executor_add_timer_new(&executor, &second_timer, 41));
	RCCHECK(rclc_executor_add_subscription_new(&executor, &second_subscriber, &second_incoming, 42,
		&second_subscription_callback, ON_NEW_DATA));
#endif
#if( CHAIN_NUM >= 3 )
	RCCHECK(rclc_executor_add_timer_new(&executor, &third_timer, 31));
	RCCHECK(rclc_executor_add_subscription_new(&executor, &third_subscriber, &third_incoming, 32,
		&third_subscription_callback, ON_NEW_DATA));
#endif
#if( CHAIN_NUM >= 4 )
	RCCHECK(rclc_executor_add_timer_new(&executor, &four_timer, 21));
	RCCHECK(rclc_executor_add_subscription_new(&executor, &four_subscriber, &four_incoming, 22,
		&four_subscription_callback, ON_NEW_DATA));
#endif
#if( CHAIN_NUM >= 5 )
	RCCHECK(rclc_executor_add_timer_new(&executor, &five_timer, 11));
	RCCHECK(rclc_executor_add_subscription_new(&executor, &five_subscriber, &five_incoming, 12,
		&five_subscription_callback, ON_NEW_DATA));
#endif
	//Init TimerList and ReadyList
	PQ_Init();

	//Add all timer handles to TimerList
	rclc_executor_add_timer_handle_new(&executor);
#endif

	char sensor_buffer[STRING_BUFFER_LEN];
	sensor_data.frame_id.data = sensor_buffer;
	sensor_data.frame_id.capacity = STRING_BUFFER_LEN;

	char action_buffer[STRING_BUFFER_LEN];
	action_incoming.frame_id.data = action_buffer;
	action_incoming.frame_id.capacity = STRING_BUFFER_LEN;

	char second_buffer[STRING_BUFFER_LEN];
	second_data.frame_id.data = second_buffer;
	second_data.frame_id.capacity = STRING_BUFFER_LEN;

	char second2_buffer[STRING_BUFFER_LEN];
	second_incoming.frame_id.data = second2_buffer;
	second_incoming.frame_id.capacity = STRING_BUFFER_LEN;

	char third_buffer[STRING_BUFFER_LEN];
	third_data.frame_id.data = third_buffer;
	third_data.frame_id.capacity = STRING_BUFFER_LEN;

	char third2_buffer[STRING_BUFFER_LEN];
	third_incoming.frame_id.data = third2_buffer;
	third_incoming.frame_id.capacity = STRING_BUFFER_LEN;

	char four_buffer[STRING_BUFFER_LEN];
	four_data.frame_id.data = four_buffer;
	four_data.frame_id.capacity = STRING_BUFFER_LEN;

	char four2_buffer[STRING_BUFFER_LEN];
	four_incoming.frame_id.data = four2_buffer;
	four_incoming.frame_id.capacity = STRING_BUFFER_LEN;

	char five_buffer[STRING_BUFFER_LEN];
	five_data.frame_id.data = five_buffer;
	five_data.frame_id.capacity = STRING_BUFFER_LEN;

	char five2_buffer[STRING_BUFFER_LEN];
	five_incoming.frame_id.data = five2_buffer;
	five_incoming.frame_id.capacity = STRING_BUFFER_LEN;

	while(1){
#if( TIDE_EXECUTOR == 0 )
		rclc_executor_spin_some(&executor, RCL_MS_TO_NS(0));
#endif

#if( TIDE_EXECUTOR == 1 )
		rclc_executor_spin_some_new(&executor, RCL_MS_TO_NS(0));

#endif
		// usleep(10000);
	}
	// Free resources
	RCCHECK(rcl_publisher_fini(&sensor_publisher, &node));
	RCCHECK(rcl_subscription_fini(&action_subscriber, &node));
	RCCHECK(rcl_publisher_fini(&second_publisher, &node));
	RCCHECK(rcl_subscription_fini(&second_subscriber, &node));
	RCCHECK(rcl_publisher_fini(&third_publisher, &node));
	RCCHECK(rcl_subscription_fini(&third_subscriber, &node));
	RCCHECK(rcl_publisher_fini(&four_publisher, &node));
	RCCHECK(rcl_subscription_fini(&four_subscriber, &node));
	RCCHECK(rcl_publisher_fini(&five_publisher, &node));
	RCCHECK(rcl_subscription_fini(&five_subscriber, &node));
	RCCHECK(rcl_node_fini(&node));
}
