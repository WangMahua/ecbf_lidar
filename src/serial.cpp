#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <inttypes.h>
#include <string>
#include "ros/ros.h"

using namespace std;

int serial_fd = 0;

void serial_init(char *port_name, int baudrate)
{
	//open the port
	serial_fd = open(port_name, O_RDWR | O_NOCTTY /*| O_NONBLOCK*/);

	if(serial_fd == -1) {
		ROS_FATAL("Failed to open the serial port.");
		exit(0);
	}

	//config the port
	struct termios options;

	tcgetattr(serial_fd, &options);

	options.c_cflag = CS8 | CLOCAL | CREAD;
	options.c_iflag = IGNPAR;
	options.c_oflag = 0;
	options.c_lflag = 0;

	switch(baudrate) {
	case 9600:
		options.c_cflag |= B9600;
		break;
	case 57600:
		options.c_cflag |= B57600;
		break;
	case 115200:
		options.c_cflag |= B115200;
		break;
	default:
		ROS_FATAL("Unknown baudrate. try 9600, 57600, 115200 or check \"serial.cpp\".");
		exit(0);
	}

	tcflush(serial_fd, TCIFLUSH);
	tcsetattr(serial_fd, TCSANOW, &options);
}
int check_rigid_body_name(char *name, int *id)
{
	char tracker_id_s[100] = {0};

	if(name[0] == 'M' && name[1] == 'A' && name[2] == 'V') {
		strncpy(tracker_id_s, name + 3, strlen(name) - 3);
	}

	//ROS_INFO("%s -> %s", name, tracker_id_s);

	char *end;
	int tracker_id = std::strtol(tracker_id_s, &end, 10);
	if (*end != '\0' || end == tracker_id_s) { //FIXME
		ROS_FATAL("Invalid tracker name %s, correct format: MAV + number, e.g: MAV1", name);
		return 1;
	}

	*id = tracker_id;

	return 0;
}


void serial_puts(char *s, size_t size)
{
	write(serial_fd, s, size);
}

#define VINS_MONO_CHECKSUM_INIT_VAL 19
static uint8_t generate_vins_mono_checksum_byte(uint8_t *payload, int payload_count)
{
	uint8_t result = VINS_MONO_CHECKSUM_INIT_VAL;

	int i;
	for(i = 0; i < payload_count; i++)
		result ^= payload[i];

	return result;
}
#define VINS_MONO_SERIAL_MSG_SIZE 44
#define DRONE_ID 1
void send_pose_to_serial(float pos_x_m, float pos_y_m, float pos_z_m,
			 float quat_x, float quat_y, float quat_z, float quat_w,
			 float vel_x,float vel_y,float vel_z)
{
/*
	ROS_INFO("[%fHz], position=(x:%.2f, y:%.2f, z:%.2f), "
                 "orientation=(x:%.2f, y:%.2f, z:%.2f, w:%.2f),"
		"velocity=(x:%.2f,y:%.2f,z:%.2f)",
        	 real_freq,
             	pos_x_m * 100.0f, pos_y_m * 100.0f, pos_z_m * 100.0f,
                 quat_x *100.0f , quat_y *100.0f, quat_z *100.0f, quat_w,vel_x,vel_y,vel_z);
*/
	char msg_buf[VINS_MONO_SERIAL_MSG_SIZE] = {0};
	int msg_pos = 0;

	/* reserve 2 for start byte and checksum byte as header */
	msg_buf[msg_pos] = '@'; //start byte
	msg_pos += sizeof(uint8_t);
	msg_buf[msg_pos] = 0;
	msg_pos += sizeof(uint8_t);
	msg_buf[msg_pos] = DRONE_ID;//tracker_id
	msg_pos += sizeof(uint8_t);

	/* pack payloads */
	//pose enu
	memcpy(msg_buf + msg_pos, &pos_x_m, sizeof(float));
	msg_pos += sizeof(float);
	memcpy(msg_buf + msg_pos, &pos_y_m, sizeof(float));
	msg_pos += sizeof(float);
	memcpy(msg_buf + msg_pos, &pos_z_m, sizeof(float));
	msg_pos += sizeof(float);
	//velocity enu
	memcpy(msg_buf + msg_pos, &vel_x, sizeof(float));
	msg_pos += sizeof(float);
	memcpy(msg_buf + msg_pos, &vel_y, sizeof(float));
	msg_pos += sizeof(float);
	memcpy(msg_buf + msg_pos, &vel_z, sizeof(float));
	msg_pos += sizeof(float);
	//rotation_q ned
	memcpy(msg_buf + msg_pos, &quat_w, sizeof(float));
	msg_pos += sizeof(float);
	memcpy(msg_buf + msg_pos, &quat_x, sizeof(float));
	msg_pos += sizeof(float);
	memcpy(msg_buf + msg_pos, &quat_y, sizeof(float));
	msg_pos += sizeof(float);
	memcpy(msg_buf + msg_pos, &quat_z, sizeof(float));
	msg_pos += sizeof(float);

        msg_buf[msg_pos] = '+'; //end byte
	msg_pos += sizeof(uint8_t);

	/* generate and fill the checksum field */
	msg_buf[1] = generate_vins_mono_checksum_byte((uint8_t *)&msg_buf[3], VINS_MONO_SERIAL_MSG_SIZE - 4);

	serial_puts(msg_buf, VINS_MONO_SERIAL_MSG_SIZE);
}

int serial_getc(char *c)
{
	return read(serial_fd, c, 1);
}
