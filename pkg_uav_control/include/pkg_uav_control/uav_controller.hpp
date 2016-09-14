/*
 * uav_controller.hpp
 *
 *  Created on: 22 Mar 2014
 *      Author: optimus
 */

#ifndef UAV_CONTROLLER_HPP_
#define UAV_CONTROLLER_HPP_

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>

#include <std_msgs/Empty.h>
namespace uav {

class UavController {

public:
	UavController();
	~UavController();

private:

	ros::NodeHandle* nh;
	ros::Subscriber cmd_subscriber_;
	ros::Publisher cmd_1_publisher_;
	ros::Publisher cmd_2_publisher_;


	//callbacks
	void Uav1CmdCallback(const geometry_msgs::TwistConstPtr&);

	//general functions
	void subAll();
	void calcLaser(const sensor_msgs::LaserScanConstPtr&, std::string);

	geometry_msgs::Twist gmttwist;

	std::string uav1_cmd_topic_;
	std::string uav2_cmd_topic_;
	std::string uav3_cmd_topic_;

	std::string nh_st_;

};
}


#endif /* UAV_CONTROLLER_HPP_ */
