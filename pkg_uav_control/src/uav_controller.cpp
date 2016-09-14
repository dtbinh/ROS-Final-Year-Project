/*
 * uav_controller.cpp
 *
 *  Created on: 22 Mar 2014
 *      Author: optimus
 */

#include <pkg_uav_control/uav_controller.hpp>

namespace uav{
	UavController::UavController(){
		uav1_cmd_topic_="/uav1/cmd_vel";
		uav2_cmd_topic_="/uav2/cmd_vel";
		uav3_cmd_topic_="/uav3/cmd_vel";

		nh_st_="UavController";

		subAll();

	}

	UavController::~UavController(){

	}

	void UavController::subAll(){
		nh = new ros::NodeHandle(nh_st_);
		cmd_subscriber_ = nh->subscribe(uav1_cmd_topic_, 1, &UavController::Uav1CmdCallback, this);

		ROS_INFO_STREAM("Subscribed to cmd_vel topics");

		cmd_1_publisher_ = nh->advertise<geometry_msgs::Twist>(uav2_cmd_topic_, 1);
		cmd_2_publisher_ = nh->advertise<geometry_msgs::Twist>(uav3_cmd_topic_, 1);

		ROS_INFO_STREAM("Publishing to cmd_vel topics");
	}

	void UavController::Uav1CmdCallback(const geometry_msgs::TwistConstPtr& gmt){
		cmd_1_publisher_.publish(gmt);
		cmd_2_publisher_.publish(gmt);

		ROS_INFO_STREAM("Published Commands To UAV's");
	}


}
int main(int argc, char** argv)
{
  ros::init(argc, argv, "UavController");
  uav::UavController uc;
  ros::spin();
  return 0;
}




