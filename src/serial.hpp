#ifndef __SERIAL_HPP__
#define __SERIAL_HPP__

void serial_init(char *port_name, int baudrate);
void send_pose_to_serial(char *tracker_name, float pos_x_cm, float pos_y_cm, float pos_z_cm,
			 float quat_x, float quat_y, float quat_z, float quat_w);

void send_pose_to_serial(float pos_x_cm, float pos_y_cm, float pos_z_cm,
			 float quat_x, float quat_y, float quat_z, float quat_w, 
			float vel_x, float vel_y, float vel_z);
int serial_getc(char *c);

#endif
