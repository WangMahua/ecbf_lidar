#include <nav_msgs/Odometry.h>      
#ifndef __VEL_H__
#define __VEL_H__
extern double pos[3];
extern double vel[3];
void odom_callback(nav_msgs::Odometry odom); 
int vel_thread_debug_entry();

#endif
