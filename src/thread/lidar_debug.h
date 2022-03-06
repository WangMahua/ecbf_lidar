#ifndef __LIDAR_H__
#define __LIDAR_H__


#define IMU_SERIAL_MSG_SIZE 27 
#define IMU_CHECKSUM_INIT_VAL 19

#define MASS 1.39f
#define GRAVITY 9.8f

extern double roll_d;
extern double pitch_d;
extern double throttle_d;
extern double pub_to_controller[3];


typedef struct {
	
	float acc[3];

	float gyrop[3];

	volatile int buf_pos;

	double deviation_acc;	

	uint8_t buf[];
	
} imu_t ;

typedef struct {
	
	float roll;

	float pitch;

	float yaw;

	float throttle;

	int mode;
	
} rc_data ;

uint8_t generate_imu_checksum_byte(uint8_t *, int);

int imu_decode(uint8_t *);

void imu_buf_push(uint8_t);

int lidar_thread_debug_entry();

#endif