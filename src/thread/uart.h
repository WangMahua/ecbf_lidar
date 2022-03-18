#include <nav_msgs/Odometry.h>      

#ifndef __UART_H__
#define __UART_H__

typedef struct {
	
	float acc[3];

	float gyrop[3];

	volatile int buf_pos;

	double deviation_acc;	

	uint8_t buf[];
	
} imu_t ;

/* extern var will send to lidar.cpp */
extern float rc_value[4]; // roll pitch yaw throttle 
extern int rc_ecbf_mode;

int uart_thread_entry();

uint8_t generate_imu_checksum_byte(uint8_t *, int);

int imu_decode(uint8_t *);

void imu_buf_push(uint8_t);

#endif
