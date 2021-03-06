/*
 * teleop_uav_controller
 *
 * Based on Original "teleop_pr2_keyboard" 
 * MODIFIED with TAKE-OFF, LAND and HOVER Commands*
 */

// Author: David Asare 

#include <termios.h>
#include <signal.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Empty.h>
#include <pkg_uav_control/collision_msg.h>

#define KEYCODE_A 0x61
#define KEYCODE_D 0x64
#define KEYCODE_S 0x73
#define KEYCODE_W 0x77 
#define KEYCODE_Q 0x71
#define KEYCODE_E 0x65
#define KEYCODE_O 0X6F
#define KEYCODE_H 0x68
#define KEYCODE_L 0x6c
#define KEYCODE_T 0x74

#define KEYCODE_A_CAP 0x41
#define KEYCODE_D_CAP 0x44
#define KEYCODE_S_CAP 0x53
#define KEYCODE_W_CAP 0x57
#define KEYCODE_Q_CAP 0x51
#define KEYCODE_E_CAP 0x45

#define KEYCODE_H_CAP 0x48
#define KEYCODE_L_CAP 0x4c
#define KEYCODE_T_CAP 0x54

class TeleopUAVController
{
  private:
  double walk_vel, run_vel, yaw_rate, yaw_rate_run;
 
 geometry_msgs::Twist cmd;

  ros::NodeHandle n_;
  ros::Publisher vel_pub_;
  ros::Subscriber ls_sub_;
  bool intro_;
  int number;

  public:
  void init(){

	 ls_sub_ = n_.subscribe("ls_info", 1, &TeleopUAVController::laserInfoCallback, this);

	 cmd.linear.x = cmd.linear.y = cmd.angular.z = 0;

    vel_pub_ = n_.advertise<geometry_msgs::Twist>("/uav1/cmd_vel", 1);

    ros::NodeHandle n_private("~");
    n_private.param("walk_vel", walk_vel, 1.0);
    n_private.param("run_vel", run_vel, 1.0);
    n_private.param("yaw_rate", yaw_rate, 1.0);
    n_private.param("yaw_run_rate", yaw_rate_run, 1.5);
    
    walk_vel = 1.0;
    run_vel = 1.0;
    yaw_rate= 1.0;
    yaw_rate_run = 1.5;
    number = 0;

  }
  
  ~TeleopUAVController()   { }
  void keyboardLoop();
  void laserInfoCallback(const pkg_uav_control::collision_msg&);

};

int kfd = 0;
struct termios cooked, raw;

void quit(int sig)
{
  tcsetattr(kfd, TCSANOW, &cooked);
  exit(0);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "teleopUAVController");

  TeleopUAVController tuc;
  tuc.init();

  signal(SIGINT,quit);

  tuc.keyboardLoop();

  return(0);
}


void TeleopUAVController::laserInfoCallback(const pkg_uav_control::collision_msg&){

}


void TeleopUAVController::keyboardLoop(){
  char c;
  bool dirty=false;

  // get the console in raw mode
  tcgetattr(kfd, &cooked);
  memcpy(&raw, &cooked, sizeof(struct termios));
  raw.c_lflag &=~ (ICANON | ECHO);
  // Setting a new line, then end of file
  raw.c_cc[VEOL] = 1;
  raw.c_cc[VEOF] = 2;
  tcsetattr(kfd, TCSANOW, &raw);

  intro_ = true;

  puts("Reading from keyboard");
  puts("---------------------------");
  puts("***************************");
  puts("Press Characters Once");
  puts("***************************");
  puts("");
  puts("Use 'WASD' to translate");
  puts("Use 'T' to take off");
  puts("Use 'L' to land");
  puts("Use 'H' to hover");
  puts("Use 'QE' to yaw");
  puts("Press 'Shift' to run");
  puts("");
  puts("---------------------------");

  for(;;)
  {
    // get the next event from the keyboard
    if(read(kfd, &c, 1) < 0)
    {
      perror("read():");
      exit(-1);
    }

    cmd.linear.z = cmd.linear.x = cmd.linear.y = cmd.angular.z = 0;

    switch(c)
    {
      // Walking
    case KEYCODE_T:
      cmd.linear.z = walk_vel;
      ROS_INFO_STREAM("TAKE OFF!!");
      dirty = true;
      break;
    case KEYCODE_L:
      cmd.linear.z = - walk_vel;
      ROS_INFO_STREAM("LAND!!");
      dirty = true;
      break;
    case KEYCODE_H:
    	ROS_INFO_STREAM("** HOVERING **");
      dirty = true;
      break;
    case KEYCODE_W:
      cmd.linear.x = walk_vel;
	ROS_INFO_STREAM("FORWARD");
      dirty = true;
      break;
    case KEYCODE_S:
      cmd.linear.x = - walk_vel;
      ROS_INFO_STREAM("BACKWARD");
      dirty = true;
      break;
    case KEYCODE_A:
      cmd.linear.y = walk_vel;
      ROS_INFO_STREAM("LEFT <-");
      dirty = true;
      break;
    case KEYCODE_D:
      cmd.linear.y = - walk_vel;
      ROS_INFO_STREAM("RIGHT ->");
      dirty = true;
      break;
    case KEYCODE_Q:
      cmd.angular.z = yaw_rate;
      dirty = true;
      break;
    case KEYCODE_E:
      cmd.angular.z = - yaw_rate;
      dirty = true;
      break;

      // Running 
    case KEYCODE_W_CAP:
      cmd.linear.x = run_vel;

      dirty = true;
      break;
    case KEYCODE_S_CAP:
      cmd.linear.x = - run_vel;
      dirty = true;
      break;
    case KEYCODE_A_CAP:
      cmd.linear.y = run_vel;
      dirty = true;
      break;
    case KEYCODE_D_CAP:
      cmd.linear.y = - run_vel;
      dirty = true;
      break;
    case KEYCODE_Q_CAP:
      cmd.angular.z = yaw_rate_run;
      dirty = true;
      break;
    case KEYCODE_E_CAP:
      cmd.angular.z = - yaw_rate_run;
      dirty = true;
      break;
    }

    
    if (dirty == true)
    {
      vel_pub_.publish(cmd);
    }

  }
}
