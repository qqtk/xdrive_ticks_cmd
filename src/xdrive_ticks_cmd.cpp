
#include <vector>
#include <string>
#include <iostream>

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <unistd.h>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <signal.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
// #include "xdrive_ticks_cmd/WheelSpeed.h"
#include "robbase_msg/WheelSpeed.h"
#include "robbase_msg/encoders.h"
// #include "encoder_test/ticks.h"

// #include <map>
#include "xdrive_ticks_cmd/xdriver.h"
// #include "xdriver.h"

ros::Time current_time, last_time;

#define M_PIpi 3.1415926
#define wheel_base 0.496
int i_gear_ratio;
double d_drive_freq;
double d_wheel_diameter;
double d_diam;

ros::NodeHandle *private_n;
robbase_msg::WheelSpeed wheelspeed_msg;
// xdrive_ticks_cmd::WheelSpeed wheelspeed_msg;

using namespace std;
int right_wheel_speed;
int left_wheel_speed;
unsigned int numA;
unsigned int numB;

float var_Vlin, var_Vang;

//
typedef struct {
	float Vlin;
} velx_struct;
velx_struct vel_x_struct;

typedef struct {
	float Vang;
} velw_struct;
velw_struct vel_w_struct;

// typedef struct { float Vlin; float Vang; } twist_struct;
// twist_struct twist_vec_struct;

typedef struct{
	int tick_rf;
	int tick_lb;
	int tick_lf;
	int tick_rb;
} tick_vec_struct;
tick_vec_struct tick_vec;

float lwheelmotor, rwheelmotor;
float reduction_ratio;
  ros::Publisher cmd_vel_pub;

/*
void wheel_update_Speed_callback(const xdrive_unit::WheelSpeed& vel_motors_msg){
    d_diam = 0.14;
    left_wheel_speed = vel_motors_msg.lwheelmotor/d_diam*M_PIpi*reduction_ratio*60;
    right_wheel_speed = vel_motors_msg.rwheelmotor/d_diam*M_PIpi*reduction_ratio*60;
} */
// #if 1

void cmdvel_Callback(const geometry_msgs::Twist::ConstPtr& cmdvel_msg)
{
  // vel_x_struct.Vlin = cmdvel_msg->linear.x;
  // vel_w_struct.Vang = cmdvel_msg->angular.z;
  var_Vlin = cmdvel_msg->linear.x;
  var_Vang = cmdvel_msg->angular.z;

  // Vlinr = (lmotor + rmotor) / 2
  // Vangular = (rmotor - lmotor) / w
  lwheelmotor = var_Vlin - var_Vang * wheel_base *0.5;
  rwheelmotor = var_Vlin + var_Vang * wheel_base *0.5;
  // .header.frame_id = "/qch";
  wheelspeed_msg.lwheel = lwheelmotor /d_diam*M_PIpi*reduction_ratio*60 ;
  wheelspeed_msg.rwheel = rwheelmotor /d_diam*M_PIpi*reduction_ratio*60;
  wheelspeed_msg.header.stamp = ros::Time::now();
  // header
  cmd_vel_pub.publish(wheelspeed_msg);
}
// #if 1 # endif // a=JunZhen

int main(int argc, char** argv)
{
  char strBuf[100];

	// struct twist_struct vel_9_struct;
	 var_Vlin =0.0;
	 var_Vang =0.0;

    ros::init(argc, argv, "xdrive_ticks_cmd_node" );
    ROS_INFO("xdrive_ticks_cmd_node starting");

    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~").
    // private_n= new ros::NodeHandle("~");

    double d_exec_rate;
    nh_private.param("exec_rate", d_exec_rate, 20); 

    // float reduction_ratio;
    nh_private.param("f_reduction_ratio", reduction_ratio, 65); 

    float d_diam;
    nh_private.param("wheel_diam", d_diam, 0.262); 
    // if(!private_n->getParam("wheel_diam", d_diam)) 
    //    d_diam  = 0.262; ROS_WARN("Not provided: d_diam. Default=0.262"); 

    bool rwheeltick_positive_;
    int rwheeltick_positive_factor =1;
    // {rwheeltick_positive, lwheeltick_negative}, vice versa.
    nh_private.param("rwheeltick_positive_flag", rwheeltick_positive_, true); 
    if (rwheeltick_positive_ == true)
      rwheeltick_positive_factor = 1;
    else if (rwheeltick_positive_ == false)
      rwheeltick_positive_factor = -1;

  ROS_INFO("serial connection _ not-use");
  ros::Subscriber cmdvel_sub = nh.subscribe("cmd_vel", 20, cmdvel_Callback);

  //  ros::Publisher ticksLR_pub = nh.advertise<robbase_msg::ticks>("/ticks", 20);
  ros::Publisher ticksLR_pub = nh.advertise<robbase_msg::encoders>("/ticksLR", 20);
  // ros::Publisher ticksLR4_pub = nh.advertise<encoder_test::ticks>("/ticks", 20);
  //  ros::Publisher ticksMLR_pub = nh.advertise<robbase_msg::RazorImu>("/ticksMLR", 20);

  // showing motor_speed:
  cmd_vel_pub  = nh.advertise<robbase_msg::WheelSpeed>("/wheelspeed", 10);
  //  cmd_vel_pub  = nh.advertise<robbase_msg::WheelSpeed>("/wheelspeed", 10);

  // struct qch.rwheelmotor

  // ros::Duration dur_time;
  robbase_msg::encoders ticksMsg;
  //  encoder_test::ticks ticksMsg;
  ros::Rate loop_rate(d_exec_rate);
  ros::Time read_time = ros::Time::now();

    while(ros::ok())
    {
        // xdriver_struct_set("Vlin",&vel_x_struct);
        // xdriver_struct_set("Vang",&vel_w_struct;
        xdriver_setValue_float("Vlin", var_Vlin);
        xdriver_setValue_float("Vang", var_Vang);

        tick_vec.tick_lb = xdriver_getValue("ticklb") * rwheeltick_positive_factor * (-1);
        tick_vec.tick_lf  = xdriver_getValue("ticklf") * rwheeltick_positive_factor * (-1);
        tick_vec.tick_rb =  xdriver_getValue("tickrb") * rwheeltick_positive_factor;
        tick_vec.tick_rf = xdriver_getValue("tickrf") * rwheeltick_positive_factor;

        ticksMsg.header.stamp = ros::Time::now();
        ticksMsg.ticks_l = tick_vec.tick_lb;
        ticksMsg.ticks_r = tick_vec.tick_rb;
        // ticksMsg.ticks_l = int( (tick_vec.tick_lb + tick_vec.tick_lf )*0.5 );
        // ticksMsg.ticks_r = int( (tick_vec.tick_rb + tick_vec.tick_rf) *0.5 );

        // xdrive_motor();
        // ticksMsg.rticks = 99; // numA; // 'L'+numA==rwheel
        ticksLR_pub.publish(ticksMsg);

        ros::spinOnce();
        loop_rate.sleep();
    }// end.mainloop _ ( ros::ok() )
}// end.main()
