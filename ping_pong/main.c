#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/header.h>

#include <stdio.h>
#include <unistd.h>
#include <time.h>

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Aborting.\n",__LINE__,(int)temp_rc); return 1;}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Continuing.\n",__LINE__,(int)temp_rc);}}

#define CHAIN_NUM       5
#define SENSORDATASIZE	1000
#define EXECUTE_TIME	10000

rcl_publisher_t action_publisher;
rcl_subscription_t sensor_subscriber;
std_msgs__msg__Header sensor_incoming;
std_msgs__msg__Header action_outcoming;

rcl_publisher_t second_publisher;
rcl_subscription_t second_subscriber;
std_msgs__msg__Header second_incoming;
std_msgs__msg__Header second_outcoming;

rcl_publisher_t third_publisher;
rcl_subscription_t third_subscriber;
std_msgs__msg__Header third_incoming;
std_msgs__msg__Header third_outcoming;

rcl_publisher_t four_publisher;
rcl_subscription_t four_subscriber;
std_msgs__msg__Header four_incoming;
std_msgs__msg__Header four_outcoming;

rcl_publisher_t five_publisher;
rcl_subscription_t five_subscriber;
std_msgs__msg__Header five_incoming;
std_msgs__msg__Header five_outcoming;

void sensor_subscription_callback(const void * msgin){
	const std_msgs__msg__Header * msg = (const std_msgs__msg__Header *)msgin;
	int receive_num = 0;
	for(int i = 0;i <  msg->frame_id.size; ++i ){
		if(msg->frame_id.data[i] >= '0' && msg->frame_id.data[i] <= '9'){
			receive_num = receive_num * 10 + (msg->frame_id.data[i] - '0');
		}else{
			break;
		}
	}
	printf("==========|ROS 11111 Reveived|%d\t%ld\n", receive_num, msg->frame_id.size);

	sprintf(action_outcoming.frame_id.data, "%d", receive_num);
	action_outcoming.frame_id.size = strlen(action_outcoming.frame_id.data);

	RCSOFTCHECK(rcl_publish(&action_publisher, (const void *)&action_outcoming, NULL));
}

void second_subscription_callback(const void * msgin){
	const std_msgs__msg__Header * msg = (const std_msgs__msg__Header *)msgin;
	int receive_num = 0;
	for(int i = 0;i <  msg->frame_id.size; ++i ){
		if(msg->frame_id.data[i] >= '0' && msg->frame_id.data[i] <= '9'){
			receive_num = receive_num * 10 + (msg->frame_id.data[i] - '0');
		}else{
			break;
		}
	}

	printf("==========|ROS 22222 Reveived|%d\t%ld\n", receive_num, msg->frame_id.size);

	sprintf(second_outcoming.frame_id.data, "%d", receive_num);
	second_outcoming.frame_id.size = strlen(second_outcoming.frame_id.data);

	RCSOFTCHECK(rcl_publish(&second_publisher, (const void *)&second_outcoming, NULL));
}

void third_subscription_callback(const void * msgin){
	const std_msgs__msg__Header * msg = (const std_msgs__msg__Header *)msgin;
	int receive_num = 0;
	for(int i = 0;i <  msg->frame_id.size; ++i ){
		if(msg->frame_id.data[i] >= '0' && msg->frame_id.data[i] <= '9'){
			receive_num = receive_num * 10 + (msg->frame_id.data[i] - '0');
		}else{
			break;
		}
	}

	printf("==========|ROS 33333 Reveived|%d\t%ld\n", receive_num, msg->frame_id.size);

	sprintf(third_outcoming.frame_id.data, "%d", receive_num);
	third_outcoming.frame_id.size = strlen(third_outcoming.frame_id.data);

	RCSOFTCHECK(rcl_publish(&third_publisher, (const void *)&third_outcoming, NULL));
}

void four_subscription_callback(const void * msgin){
	const std_msgs__msg__Header * msg = (const std_msgs__msg__Header *)msgin;
	int receive_num = 0;
	for(int i = 0;i <  msg->frame_id.size; ++i ){
		if(msg->frame_id.data[i] >= '0' && msg->frame_id.data[i] <= '9'){
			receive_num = receive_num * 10 + (msg->frame_id.data[i] - '0');
		}else{
			break;
		}
	}

	printf("==========|ROS 44444 Reveived|%d\t%ld\n", receive_num, msg->frame_id.size);

	sprintf(four_outcoming.frame_id.data, "%d", receive_num);
	four_outcoming.frame_id.size = strlen(four_outcoming.frame_id.data);

	RCSOFTCHECK(rcl_publish(&four_publisher, (const void *)&four_outcoming, NULL));
}

void five_subscription_callback(const void * msgin){
	const std_msgs__msg__Header * msg = (const std_msgs__msg__Header *)msgin;
	int receive_num = 0;
	for(int i = 0;i <  msg->frame_id.size; ++i ){
		if(msg->frame_id.data[i] >= '0' && msg->frame_id.data[i] <= '9'){
			receive_num = receive_num * 10 + (msg->frame_id.data[i] - '0');
		}else{
			break;
		}
	}

	printf("==========|ROS 55555 Reveived|%d\t%ld\n", receive_num, msg->frame_id.size);

	sprintf(five_outcoming.frame_id.data, "%d", receive_num);
	five_outcoming.frame_id.size = strlen(five_outcoming.frame_id.data);

	RCSOFTCHECK(rcl_publish(&five_publisher, (const void *)&five_outcoming, NULL));
}

int main()
{
	rcl_allocator_t allocator = rcl_get_default_allocator();
	rclc_support_t support;

	// create init_options
	RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

	// create node
	rcl_node_t node;
	RCCHECK(rclc_node_init_default(&node, "ROS2", "", &support));

	RCCHECK(rclc_publisher_init_default(&action_publisher, &node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Header), "/microROS/action"));

	RCCHECK(rclc_subscription_init_best_effort(&sensor_subscriber, &node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Header), "/microROS/sensor"));

	RCCHECK(rclc_publisher_init_best_effort(&second_publisher, &node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Header), "/microROS/second_receive"));

	RCCHECK(rclc_subscription_init_best_effort(&second_subscriber, &node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Header), "/microROS/second_publish"));

	RCCHECK(rclc_publisher_init_best_effort(&third_publisher, &node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Header), "/microROS/third_receive"));

	RCCHECK(rclc_subscription_init_best_effort(&third_subscriber, &node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Header), "/microROS/third_publish"));
	
	RCCHECK(rclc_publisher_init_best_effort(&four_publisher, &node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Header), "/microROS/four_receive"));

	RCCHECK(rclc_subscription_init_best_effort(&four_subscriber, &node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Header), "/microROS/four_publish"));
	
	RCCHECK(rclc_publisher_init_best_effort(&five_publisher, &node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Header), "/microROS/five_receive"));
	
	RCCHECK(rclc_subscription_init_best_effort(&five_subscriber, &node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Header), "/microROS/five_publish"));


	// Create executor
	rclc_executor_t executor = rclc_executor_get_zero_initialized_executor();
	RCCHECK(rclc_executor_init(&executor, &support.context, CHAIN_NUM, &allocator));
#if( CHAIN_NUM >= 1 )
	RCCHECK(rclc_executor_add_subscription(&executor, &sensor_subscriber, &sensor_incoming,
		&sensor_subscription_callback, ON_NEW_DATA));
#endif
#if( CHAIN_NUM >= 2 )
	RCCHECK(rclc_executor_add_subscription(&executor, &second_subscriber, &second_incoming,
		&second_subscription_callback, ON_NEW_DATA));
#endif
#if( CHAIN_NUM >= 3 )
	RCCHECK(rclc_executor_add_subscription(&executor, &third_subscriber, &third_incoming,
		&third_subscription_callback, ON_NEW_DATA));
#endif
#if( CHAIN_NUM >= 4 )
	RCCHECK(rclc_executor_add_subscription(&executor, &four_subscriber, &four_incoming,
		&four_subscription_callback, ON_NEW_DATA));
#endif
#if( CHAIN_NUM >= 5 )
	RCCHECK(rclc_executor_add_subscription(&executor, &five_subscriber, &five_incoming,
		&five_subscription_callback, ON_NEW_DATA));
#endif

	char sensor_buffer[SENSORDATASIZE];
	sensor_incoming.frame_id.data = sensor_buffer;
	sensor_incoming.frame_id.capacity = SENSORDATASIZE;
	char action_buffer[SENSORDATASIZE];
	action_outcoming.frame_id.data = action_buffer;
	action_outcoming.frame_id.capacity = SENSORDATASIZE;
	char second1_buffer[SENSORDATASIZE];
	second_incoming.frame_id.data = second1_buffer;
	second_incoming.frame_id.capacity = SENSORDATASIZE;
	char second2_buffer[SENSORDATASIZE];
	second_outcoming.frame_id.data = second2_buffer;
	second_outcoming.frame_id.capacity = SENSORDATASIZE;
	char third1_buffer[SENSORDATASIZE];
	third_incoming.frame_id.data = third1_buffer;
	third_incoming.frame_id.capacity = SENSORDATASIZE;
	char third2_buffer[SENSORDATASIZE];
	third_outcoming.frame_id.data = third2_buffer;
	third_outcoming.frame_id.capacity = SENSORDATASIZE;
	char four_buffer[SENSORDATASIZE];
	four_incoming.frame_id.data = four_buffer;
	four_incoming.frame_id.capacity = SENSORDATASIZE;
	char four2_buffer[SENSORDATASIZE];
	four_outcoming.frame_id.data = four2_buffer;
	four_outcoming.frame_id.capacity = SENSORDATASIZE;
	char five_buffer[SENSORDATASIZE];
	five_incoming.frame_id.data = five_buffer;
	five_incoming.frame_id.capacity = SENSORDATASIZE;
	char five2_buffer[SENSORDATASIZE];
	five_outcoming.frame_id.data = five2_buffer;
	five_outcoming.frame_id.capacity = SENSORDATASIZE;

	while(1){
		rclc_executor_spin_some(&executor, RCL_MS_TO_NS(0));
	}

	// Free resources
	RCCHECK(rcl_publisher_fini(&action_publisher, &node));
	RCCHECK(rcl_subscription_fini(&sensor_subscriber, &node));
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