/*
 * collision_detection.hpp
 *
 *  Created on: 23 Apr 2014
 *      Author: optimus
 */

#ifndef COLLISION_DETECTION_HPP_
#define COLLISION_DETECTION_HPP_

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Range.h>
#include <std_msgs/Empty.h>

#include <pkg_uav_control/collision_msg.h>


namespace uav {


class CollisionDetection {


public:
	CollisionDetection();
	~CollisionDetection();

private:

	ros::NodeHandle* nh;

	ros::Subscriber ls1_sub_;
	ros::Subscriber ls2_sub_;
	ros::Subscriber ls3_sub_;

	ros::Subscriber s1_sub_;
	ros::Subscriber s2_sub_;
	ros::Subscriber s3_sub_;

	ros::Subscriber ls1_out_sub_;

	ros::Subscriber uav_h_sub_;
	ros::Subscriber stop_sub_;

	ros::Publisher ls_info_pub_;
	ros::Publisher stop_pub_;

	//callbacks
	void laserscanCallback1(const sensor_msgs::LaserScanConstPtr&);
	void laserscanCallback2(const sensor_msgs::LaserScanConstPtr&);
	void laserscanCallback3(const sensor_msgs::LaserScanConstPtr&);

	void lsOutCallback(const std_msgs::EmptyConstPtr&);

	void uavHeightCallback(const geometry_msgs::PoseStamped&);

	void sonarRangeCallback1(const sensor_msgs::RangeConstPtr&);
	void sonarRangeCallback2(const sensor_msgs::RangeConstPtr&);
	void sonarRangeCallback3(const sensor_msgs::RangeConstPtr&);

	void quantifyLaser(const sensor_msgs::LaserScanConstPtr&, std::string);
	void quantifySonar(const sensor_msgs::RangeConstPtr&, std::string);

	void readCmdCallback(const geometry_msgs::TwistConstPtr&);

	//void objDetectCallback1(const pkg_uav_control::collision_msg&);

	void cdInit();

	sensor_msgs::LaserScanConstPtr ls1_;
	sensor_msgs::LaserScanConstPtr ls2_;
	sensor_msgs::LaserScanConstPtr ls3_;

	sensor_msgs::RangeConstPtr sr1_;
	sensor_msgs::RangeConstPtr sr2_;
	sensor_msgs::RangeConstPtr sr3_;

	geometry_msgs::TwistConstPtr cmdt_;

	pkg_uav_control::collision_msg cmsg1;
	pkg_uav_control::collision_msg cmsg2;
	pkg_uav_control::collision_msg cmsg3;

	std::string ls1_topic_;
	std::string ls2_topic_;
	std::string ls3_topic_;

	std::string sonar1_topic_;
	std::string sonar2_topic_;
	std::string sonar3_topic_;

	std::string uav_h_topic_;
	std::string ls_info_topic;

	bool fwd_obst;
	bool left_obst;
	bool right_obst;
	bool back_obst;
	bool bot_obst;

	std::string nh_st_;

	float current_uav_h;

	double cmd_z;
	double cmd_y;
	double cmd_x;

};



}





#endif /* COLLISION_DETECTION_HPP_ */
