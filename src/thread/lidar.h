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
	
	float roll;

	float pitch;

	float yaw;

	float throttle;

	int mode;
	
} rc_data ;

int lidar_thread_entry();

#endif
