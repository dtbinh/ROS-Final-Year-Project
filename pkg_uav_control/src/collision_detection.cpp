/*
 * collision_detection.cpp
 *
 *  Created on: 24 Apr 2014
 *      Author: optimus
 */


#include <pkg_uav_control/collision_detection.hpp>


namespace uav {

	CollisionDetection::CollisionDetection(){
		ls1_topic_ = "/uav1/scan";
		ls2_topic_ = "/uav2/scan";
		ls3_topic_ = "/uav3/scan";

		sonar1_topic_ = "/uav1/sonar_height";
		sonar2_topic_ = "/uav2/sonar_height";
		sonar3_topic_ = "/uav3/sonar_height";

		uav_h_topic_ = "/uav1/ground_truth_to_tf/pose";
		ls_info_topic = "/ls_info";


		nh_st_= "CollisionDetection";





		cdInit();

	}


	CollisionDetection::~CollisionDetection(){

	}

	void CollisionDetection::cdInit(){
		std_msgs::Empty empt;

		nh = new ros::NodeHandle(nh_st_);

		ls1_sub_ = nh->subscribe(ls1_topic_, 1, &CollisionDetection::laserscanCallback1, this);
		ls2_sub_ = nh->subscribe(ls2_topic_, 1, &CollisionDetection::laserscanCallback2, this);
		ls3_sub_ = nh->subscribe(ls3_topic_, 1, &CollisionDetection::laserscanCallback3, this);

		ls1_out_sub_ = nh->subscribe("/ls_out", 1, &CollisionDetection::lsOutCallback, this);

		s1_sub_ = nh->subscribe(sonar1_topic_, 1, &CollisionDetection::sonarRangeCallback1, this);
		s2_sub_ = nh->subscribe(sonar2_topic_, 1, &CollisionDetection::sonarRangeCallback2, this);
		s3_sub_ = nh->subscribe(sonar3_topic_, 1, &CollisionDetection::sonarRangeCallback3, this);

		uav_h_sub_ = nh->subscribe(uav_h_topic_,1 , &CollisionDetection::uavHeightCallback, this);

		ls_info_pub_ = nh->advertise<pkg_uav_control::collision_msg>(ls_info_topic, 1);

		stop_sub_ = nh->subscribe("/uav1/cmd_vel",1 , &CollisionDetection::readCmdCallback, this);

		stop_pub_ = nh->advertise<geometry_msgs::Twist>("/uav1/cmd_vel", 1);

		ROS_INFO_STREAM("Subscribed to Laser, Sonar and Pose topics");


		//ls_info_pub_->empt;
	}

	void CollisionDetection::readCmdCallback(const geometry_msgs::TwistConstPtr& gmt1){
		cmd_z = gmt1->linear.z; // TAKEOFF AND LAND
		cmd_y = gmt1->linear.y; // LEFT AND RIGHT
		cmd_x = gmt1->linear.x; // FORWARD AND BACKWARD
	}

	void CollisionDetection::sonarRangeCallback1(const sensor_msgs::RangeConstPtr& sr_){
		quantifySonar(sr_, "uav1");
	}

	void CollisionDetection::sonarRangeCallback2(const sensor_msgs::RangeConstPtr& sr_){
		quantifySonar(sr_, "uav2");
	}

	void CollisionDetection::sonarRangeCallback3(const sensor_msgs::RangeConstPtr& sr_){
		quantifySonar(sr_, "uav3");
	}

	void CollisionDetection::laserscanCallback1(const sensor_msgs::LaserScanConstPtr& laser1){
			ls1_ = laser1;

			quantifyLaser(laser1, "uav1");
	}

	void CollisionDetection::laserscanCallback2(const sensor_msgs::LaserScanConstPtr& laser2){
			ls2_ = laser2;

			quantifyLaser(laser2, "uav2");
	}

	void CollisionDetection::laserscanCallback3(const sensor_msgs::LaserScanConstPtr& laser3){

			ls3_ = laser3;

			quantifyLaser(laser3, "uav3");
	}

	void CollisionDetection::uavHeightCallback(const geometry_msgs::PoseStamped& uhz){
			current_uav_h = uhz.pose.position.z;

	}

	void CollisionDetection::quantifySonar(const sensor_msgs::RangeConstPtr& sr, std::string uav_){
			float sonar_ = sr->range;

			//ROS_INFO_STREAM(":"<< current_uav_h );

			if(current_uav_h > 1){
				if(sonar_ < 1){
					ROS_WARN_STREAM("["<< uav_ << "] " << "BELOW: TOO CLOSE TO OBJECT");
					ROS_WARN_STREAM("[uav2] " << "BELOW: TOO CLOSE TO OBJECT");
					ROS_WARN_STREAM("[uav3] " << "BELOW: TOO CLOSE TO OBJECT");
				}
			}
	}

	void CollisionDetection::quantifyLaser(const sensor_msgs::LaserScanConstPtr& in_lsr, std::string uav_){
		sensor_msgs::LaserScanConstPtr lsr = in_lsr;
		pkg_uav_control::collision_msg new_cmsg;

		std::vector<float> ranges = lsr->ranges;
		geometry_msgs::Twist new_gmt;

		int int_ran;


		float fwd_temp_r= ranges[475];
		float lft_temp_r= ranges[202];
		float rgt_temp_r= ranges[742];
		float bak_temp_r= ranges[68];

		fwd_obst= false;
		left_obst= false;
		right_obst= false;
		back_obst= false;


		new_gmt.linear.z = 0.0;
		new_gmt.linear.x = 0.0;
		new_gmt.linear.y = 0.0;
		new_gmt.angular.z = 0.0;

		for (int c1 = 0; c1 <= ranges.size(); c1++){

			//Forward Sensing
			if(c1 >= 475 && c1 <=608){
				if (fwd_temp_r >= ranges[c1]){
					fwd_temp_r = ranges[c1];
				}
			}

			//Left Sensing
			if(c1 >=202 && c1 <=338 ){
				if (lft_temp_r >= ranges[c1]){
					lft_temp_r = ranges[c1];
				}
			}

			//Right Sensing
			if(c1 >=742 && c1 <=878){
				if (rgt_temp_r >= ranges[c1]){
					rgt_temp_r = ranges[c1];
				}
			}

			//Backward Sensing
			if((c1 > 0 && c1 <= 68) || (c1 >= 1013 && c1 <1081)){
				if(bak_temp_r >= ranges[c1]){
					bak_temp_r = ranges[c1];
				}
			}

		}

		if (fwd_temp_r <= 1.4){	ROS_WARN_STREAM("["<< uav_ << "] "<<"FORWARD: TOO CLOSE TO OBJECT (Approx: " << fwd_temp_r << "m)"); fwd_obst= true;}
		if (lft_temp_r <= 1.4){	ROS_WARN_STREAM("["<< uav_ << "] "<<"LEFT: TOO CLOSE TO OBJECT (Approx: " << lft_temp_r << "m)"); left_obst= true;}
		if (rgt_temp_r <= 1.4){	ROS_WARN_STREAM("["<< uav_ << "] "<<"RIGHT: TOO CLOSE TO OBJECT (Approx: " << rgt_temp_r << "m)"); right_obst= true;}
		if (bak_temp_r <= 1.4){	ROS_WARN_STREAM("["<< uav_ << "] "<<"BACK: TOO CLOSE TO OBJECT (Approx: " << bak_temp_r << "m)"); back_obst= true;}

		new_cmsg.uav_name = uav_;

		if(fwd_obst){new_cmsg.front = 1; if(cmd_x == 1){stop_pub_.publish(new_gmt); ROS_WARN_STREAM("**_ SYSTEM OVERRIDE _** : TELEOPERATOR COMMAND");}}else{new_cmsg.front = 0;}
		if(left_obst){new_cmsg.left = 1; if(cmd_y == 1){stop_pub_.publish(new_gmt); ROS_WARN_STREAM("**_ SYSTEM OVERRIDE _** : TELEOPERATOR COMMAND");}}else{new_cmsg.left = 0;}
		if(right_obst){new_cmsg.right = 1; if(cmd_y == -1){stop_pub_.publish(new_gmt); ROS_WARN_STREAM("**_ SYSTEM OVERRIDE _** : TELEOPERATOR COMMAND"); }}else{new_cmsg.right = 0;}
		if(back_obst){new_cmsg.back = 1; if(cmd_x == -1){stop_pub_.publish(new_gmt); ROS_WARN_STREAM("**_ SYSTEM OVERRIDE _** : TELEOPERATOR COMMAND");}}else{new_cmsg.back = 0;}
		if(bot_obst){new_cmsg.bottom = 1; }else{new_cmsg.bottom = 0;}

		ls_info_pub_.publish(new_cmsg);

	}

	void CollisionDetection::lsOutCallback(const std_msgs::EmptyConstPtr& em){
			sensor_msgs::LaserScanConstPtr lsr = ls1_;

			float angle_min = lsr->angle_min;
			float angle_max= lsr->angle_max;
			float angle_increment = lsr->angle_increment;
			float time_increment = lsr->time_increment;
			float scan_time  = lsr->scan_time;
			float range_min = lsr->range_min;
			float range_max = lsr->range_max;
			std::vector<float> ranges = lsr->ranges;
			std::vector<float> intensities  = lsr->intensities;

			std::string range_str;
			std::ostringstream ss;
			int int_ran;

			float temprange = ranges[475];

			for (int count1 = 0; count1 <= ranges.size(); count1++){
				//int_ran = ranges[count1];

				if (count1 >= 472 && count1 <=611 ){

					if (temprange >= ranges[count1]){

						ROS_INFO_STREAM("[" << count1 << "] " << ranges[count1]);
						temprange = ranges[count1];
					}

					ss << "! [" << count1 << "] : " << ranges[count1] << "?";
					range_str =  ss.str();

				}
			}

			ss.str("");
			ss.clear();

			ss << "Average : " <<  temprange;

			ROS_INFO_STREAM("*************************");
			ROS_INFO_STREAM("angle_min : " << angle_min);
			ROS_INFO_STREAM("angle_max : " << angle_max);
			ROS_INFO_STREAM("angle_increment : " << angle_increment);
			ROS_INFO_STREAM("time_increment : "  << time_increment);
			ROS_INFO_STREAM("scan_time : " << scan_time);
			ROS_INFO_STREAM("range_min : " << range_min);
			ROS_INFO_STREAM("range_max : " << range_max);
			ROS_INFO_STREAM("range size : " << ranges.size());
			ROS_INFO_STREAM("inten size : " << intensities.size());
			ROS_INFO_STREAM("ranges : " << range_str);

			if (temprange <= 0.46)	ROS_INFO_STREAM("TOOO CLOSE TO OBSTRACTION");
			ROS_INFO_STREAM("" << ss.str());

			ROS_INFO_STREAM("*************************");

			ROS_INFO_STREAM("Forward Range H : " << ranges[475]);
			ROS_INFO_STREAM("Forward Range L : " << ranges[611]);
			ROS_INFO_STREAM("*************************");

			ROS_INFO_STREAM("Right Range H : " << ranges[742]);
			ROS_INFO_STREAM("Right Range L : " << ranges[878]);
			ROS_INFO_STREAM("*************************");

			ROS_INFO_STREAM("Left Range H : " << ranges[338]);
			ROS_INFO_STREAM("Left Range L : " << ranges[202]);

			ROS_INFO_STREAM("Backward Range H : " << ranges[68]);
			ROS_INFO_STREAM("Backward Range L : " << ranges[1013]);
	}
}

int main(int argc, char** argv){

	ros::init(argc, argv, "CollisionDection");
	uav::CollisionDetection cd;

	ros::spin();
	return 0;

}


