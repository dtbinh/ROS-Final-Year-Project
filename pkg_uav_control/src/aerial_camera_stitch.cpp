/*
 * aerial_camera_stitch.cpp
 *
 *  Created on: 18 Mar 2014
 *      Author: optimus
 */

#include <pkg_uav_control/aerial_camera_stitch.h>

#include <string>
using namespace cv;

int MAX_KERNEL_LENGTH = 31;

namespace uav {

AerialCameraStitch::AerialCameraStitch(){
	uav1_cam_topic_= "/uav1/downward_cam/camera/image";
	uav2_cam_topic_= "/uav2/downward_cam/camera/image";
	uav3_cam_topic_= "/uav3/downward_cam/camera/image";
	uav4_cam_topic_= "/uav4/downward_cam/camera/image";
	uav5_cam_topic_= "/uav5/downward_cam/camera/image";

	cam_uav_out_= "/image_converter/images";
	nh_str_ = "AerialCameraStitch";

	img_stitch_topic_ = "/stitch_images";
	uavpose_topic_ = "/uav1/ground_truth_to_tf/pose";
	uavpose_z_ = "/pose_z_value";

	
	subCameraImages();

}


AerialCameraStitch::~AerialCameraStitch(){
	destroyAllWindows();
}


void AerialCameraStitch::UAVPoseCallback(const geometry_msgs::PoseStamped& uavpose){


	uavpose_z = uavpose.pose.position.z;
	int current_pose;

	cmd.linear.z = cmd.linear.x = cmd.linear.y = cmd.angular.z = 0;

	current_pose = uavpose_z;

	if(opt_pose){

		if(current_pose == uav_int_pose){
			ROS_INFO_STREAM("OPTIMAL LEVEL REACHED");
			ROS_INFO_STREAM("Re-attempting stitching process ...");
			opt_pose = false;
			cmd.linear.z = 0;

			cmd_pub.publish(cmd);

			stitchCameraImages();

			rtn_pose=true;

			ROS_INFO_STREAM("Returning back to Approximate Original Position");
			ROS_INFO_STREAM("Translating ...");

		}else{
			 cmd.linear.z = 1;
			 cmd_pub.publish(cmd);
		}


	}else if(rtn_pose){

		if(current_pose == (uav_int_pose - 5)){
			ROS_INFO_STREAM("RETURNED BACK TO ORIGINAL POSITION");

			rtn_pose=false;

			cmd.linear.z = 0;
			cmd_pub.publish(cmd);
		}else{
			cmd.linear.z = -1;
			cmd_pub.publish(cmd);
		}
	}

}


void AerialCameraStitch::UAVPoseOUTCallback(const std_msgs::EmptyConstPtr& posez){
	ROS_INFO_STREAM("height : " << uavpose_z);
}
void AerialCameraStitch::subCameraImages(){

	nh = new ros::NodeHandle(nh_str_);

	it_ = new image_transport::ImageTransport(*nh);
	
	uavpose = nh->subscribe(uavpose_topic_, 1, &AerialCameraStitch::UAVPoseCallback,this);
	uavpose_ = nh->subscribe(uavpose_z_, 1, &AerialCameraStitch::UAVPoseOUTCallback,this);
	cmd_pub = nh->advertise<geometry_msgs::Twist>("/uav1/cmd_vel", 1);

	it_uav1_sub_ = it_->subscribe(uav1_cam_topic_, 1, &AerialCameraStitch::Camera1Callback,this);
	ROS_INFO_NAMED("image_convert", "Subscribed to %s", uav1_cam_topic_.c_str());

	it_uav2_sub_ = it_->subscribe(uav2_cam_topic_, 1, &AerialCameraStitch::Camera2Callback,this);
	ROS_INFO_NAMED("image_convert", "Subscribed to %s", uav2_cam_topic_.c_str());


	it_uav3_sub_ = it_->subscribe(uav3_cam_topic_, 1, &AerialCameraStitch::Camera3Callback,this);
	ROS_INFO_NAMED("image_convert", "Subscribed to %s", uav3_cam_topic_.c_str());


	imgstitch_subscriber_ = nh->subscribe(img_stitch_topic_, 1, &AerialCameraStitch::ImageStitchCallback,this);
	


	ROS_INFO_STREAM("Aerial Stitch Initialised");
	ROS_INFO_STREAM("************************************");
}

void AerialCameraStitch::ImageStitchCallback(const std_msgs::EmptyConstPtr& stitch){

	destroyAllWindows();

	stitchCameraImages();
	//panoImages();

}


void AerialCameraStitch::stitchCameraImages(){
	cv::Stitcher imgstitcher = Stitcher::createDefault(true);
	vector< Mat > tmp_uav_cam_img;
		Mat out_stitch_img;

		Mat tmp_img1;
		Mat tmp_img2;
		Mat tmp_img3;

		Mat tmp_img1_;
		Mat tmp_img2_;
		Mat tmp_img3_;

		opt_pose = false;

		ROS_INFO_STREAM("STITCHING STARTED");

		try{
			destroyAllWindows();
			tmp_img1 = uav1_cam_img;
			tmp_img2 = uav2_cam_img;
			tmp_img3 = uav3_cam_img;


			cv::resize(tmp_img1, tmp_img1_, cv::Size2i(tmp_img1.cols/2, tmp_img1.rows/2));
			cv::resize(tmp_img2, tmp_img2_, cv::Size2i(tmp_img2.cols/2, tmp_img2.rows/2));
			cv::resize(tmp_img3, tmp_img3_, cv::Size2i(tmp_img3.cols/2, tmp_img3.rows/2));

			tmp_img1 = tmp_img1_;
			tmp_img2 = tmp_img2_;
			tmp_img3 = tmp_img3_;

			tmp_uav_cam_img.push_back(tmp_img1);
			tmp_uav_cam_img.push_back(tmp_img2);
			tmp_uav_cam_img.push_back(tmp_img3);


			ROS_INFO_STREAM("UAV 1 : "<<tmp_uav_cam_img[0].size());
			ROS_INFO_STREAM("UAV 2 : "<<tmp_uav_cam_img[1].size());
			ROS_INFO_STREAM("UAV 3 : "<<tmp_uav_cam_img[2].size());


			unsigned long AAtime=0, BBtime=0; //check processing time
			AAtime = getTickCount(); //check processing time

			Stitcher::Status status = imgstitcher.stitch(tmp_uav_cam_img, out_stitch_img);


			 BBtime = getTickCount(); //check processing time
			 printf("Time Taken: %.2lf sec \n",  (BBtime - AAtime)/getTickFrequency() ); //check processing time


			cv::Size out_size =  out_stitch_img.size();
			int out_row = out_size.height;
			int out_cols = out_size.width;

			 if (status != Stitcher::OK || (out_row == 1 && out_cols == 1))
				 {
				 	 ROS_INFO_STREAM("IMAGES DIDNT STITCH - Not Enough Common Features Detected : "<< out_stitch_img.size());

				 	 ROS_INFO_STREAM("Increasing height to optimum level, if possible");

				 	 opt_pose = true;
				 	 uav_int_pose = uavpose_z + 5;

				 	 ROS_INFO_STREAM("Optimal Height : " <<  uav_int_pose);
				 	 ROS_INFO_STREAM("Translating ... ");

				 }else{
					 ROS_INFO_STREAM("STITCHING RESULT: "<< out_size);
					 ROS_INFO_STREAM("SUCCESSFUL ");

					 namedWindow("Stitching Result", WINDOW_AUTOSIZE);
					 imshow("Stitching Result", out_stitch_img);

				 }
		}catch(cv::Exception& e){
			const char* err_msg = e.what();
			ROS_INFO_STREAM("SOMETHING WENT WRONG!!!");
			ROS_INFO_STREAM("exception caught : " << err_msg);

		}
		 waitKey(0);

		 out_stitch_img.release();
		 tmp_img1.release();
		 tmp_img2.release();
		 tmp_img3.release();

		 tmp_img1_.release();
		 tmp_img2_.release();
		 tmp_img3_.release();


		 tmp_uav_cam_img[0].release();
		 tmp_uav_cam_img[1].release();
		 tmp_uav_cam_img[2].release();



}


void AerialCameraStitch::Camera1Callback(const sensor_msgs::ImageConstPtr& msg){
	 cv_bridge::CvImagePtr cv_ptr;

	 try{
		 if(sensor_msgs::image_encodings::isColor(msg->encoding)){
	 		//ROS_INFO_STREAM("BGR8");
			cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
		}else{
			//ROS_INFO_STREAM("MONO");
			cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
	 	}
	 }
	 catch (cv_bridge::Exception& e){
	        ROS_ERROR("cv_bridge exception: %s", e.what());
	        return;
	 }

		uav1_cam_img =  cv_ptr->image.clone();

}


void AerialCameraStitch::Camera2Callback(const sensor_msgs::ImageConstPtr& msg){
	 cv_bridge::CvImagePtr cv_ptr;

	 try{

		if(sensor_msgs::image_encodings::isColor(msg->encoding)){
			 	//ROS_INFO_STREAM("BGR8");
				cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
		}else{
				//ROS_INFO_STREAM("MONO");
				cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
			 }
	 }
	 catch (cv_bridge::Exception& e){
	        ROS_ERROR("cv_bridge exception: %s", e.what());
	        return;
	 }

	 uav2_cam_img =  cv_ptr->image.clone();


}

void AerialCameraStitch::Camera3Callback(const sensor_msgs::ImageConstPtr& msg){
	 cv_bridge::CvImagePtr cv_ptr;


	 try{

	 		if(sensor_msgs::image_encodings::isColor(msg->encoding)){
	 			 	//ROS_INFO_STREAM("BGR8");
	 				cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
	 		}else{
	 				//ROS_INFO_STREAM("MONO");
	 				cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
	 			 }
	 	 }
	 	 catch (cv_bridge::Exception& e){
	 	        ROS_ERROR("cv_bridge exception: %s", e.what());
	 	        return;
	 	 }

	 uav3_cam_img =  cv_ptr->image.clone();

}


}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "AerialCameraStitch");
  uav::AerialCameraStitch acs;
  ros::spin();
  return 0;
}

