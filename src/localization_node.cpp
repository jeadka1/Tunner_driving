#include "ros/ros.h"
#include "nodelet/nodelet.h"
#include <pluginlib/class_list_macros.h>

#include <tf/transform_listener.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>

enum MODE_{
//	MOBILE_STOP,
	//MOBILE_MOVING,
	//MOBILE_ROTATING,
	//MOBILE_ALIGN,
	CHARGE_MODE,
	STANDBY_MODE,
	DOCKING_MODE,
	DOCK_IN_MODE,
	DOCK_OUT_MODE,
	DOCK_OUT_PRE_MODE,
	DOCK_IN_PRE_MODE,
	AUTO_PRE_IMAGE_MODE,
	AUTO_PRE_LIDAR_MODE,
	AUTO_IMAGE_MODE,
	AUTO_LIDAR_MODE,
	MANUAL_MODE,
	STOP_MODE,
	TURN_MODE,
};


namespace auto_driving {

class LocalizationNode : public nodelet::Nodelet {

public:
	LocalizationNode() = default;

private:
	virtual void onInit() {
		ros::NodeHandle nh = getNodeHandle();
		ros::NodeHandle nhp = getPrivateNodeHandle();
		
		// Configuration //
		nhp.param("global_dist_boundary", config_.global_dist_boundary_, 0.3);
		nhp.param("global_angle_boundary", config_.global_angle_boundary_, 0.05);

		sub_goal_ = nhp.subscribe<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1, &LocalizationNode::setGoal, this);    
		sub_pose_ = nhp.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/amcl_pose", 10, &LocalizationNode::poseCallback, this);
		
		pub_localization_ = nhp.advertise<std_msgs::Float32MultiArray>("/localization_data", 10); 
		// [0]: global dist error, [1]: global angle error, [2]: arrival flag, [3]: rotating flag
	};
	
	void setGoal(const geometry_msgs::PoseStamped::ConstPtr& click_msg)
	{
		//geometry_msgs::PoseStamped print_point;
		ROS_INFO("%d th goal is set", goal_count_);
		//ROS_INFO("x: %d, y: %d", click_msg.pose.position.x, click_msg.pose.position.y);
		goal_set_.push_back(*click_msg);
    goal_count_ ++;
	}

	void poseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& pose_msg)
	{
		std_msgs::Float32MultiArray localization_msgs;
		localization_msgs.data.clear();
		if (goal_count_ < 2)
		{
			ROS_INFO_ONCE("Waiting for inserting goal... ");
			localization_msgs.data.push_back(10000); // global_dist_err
			localization_msgs.data.push_back(10000); // global_ang_err
			localization_msgs.data.push_back(0); // Postech mode
			//localization_msgs.data.push_back(true); // is_arrived -> Not moving
			localization_msgs.data.push_back(false); // is_rotating

			localization_msgs.data.push_back(0); // global_x_err
			localization_msgs.data.push_back(0); // global_y_err
			localization_msgs.data.push_back(0); // global_theta_err
			localization_msgs.data.push_back(0); // global_c_theta_err

			localization_msgs.data.push_back(0); // 
			localization_msgs.data.push_back(0); // 
			localization_msgs.data.push_back(0); //
			localization_msgs.data.push_back(0); // 

			localization_msgs.data.push_back(0); // line y pose

			localization_msgs.data.push_back(false); // init_flag for odom
			pub_localization_.publish(localization_msgs);
			return;
		}
		bool is_arrived = false;
		current_goal_ = goal_set_[goal_index_ % goal_count_];
		ROS_INFO_ONCE("Goal is set: %f, %f", current_goal_.pose.position.x, current_goal_.pose.position.y);

		geometry_msgs::PoseStamped print_point;
		print_point =goal_set_[0];
		ROS_INFO_ONCE("Goal 1 is set: %f, %f", print_point.pose.position.x, print_point.pose.position.y);
		print_point =goal_set_[1];
		ROS_INFO_ONCE("Goal 2 is set: %f, %f", print_point.pose.position.x, print_point.pose.position.y);
		
		//Initialize when the mobile robot arrives at home		
		//TODO It should operate with QR code
		/*if (goal_index_ % goal_count_ ==0)// && init_flag_==false)
		{
			// Only once when it's arrived at home.
			init_start_ = true;
			//system("rosservice call /odom_init 0.0 0.0 0.0");
			//system("rosservice call /reset_odom");
			//system("rosservice call /pose_update 0.0 0.0 0.0");
		}
		else
		{
			//init_flag_ = false;
			init_start_ = false;
		}*/
		

		// 1. Calculate Global Error
		float global_x_err = current_goal_.pose.position.x - pose_msg->pose.pose.position.x;
		float global_y_err = current_goal_.pose.position.y - pose_msg->pose.pose.position.y;
		double global_dist_err = sqrt(global_x_err*global_x_err + global_y_err*global_y_err);	
		double global_ang_err;
		double static_x_err,static_y_err;
		std::cout << "goal (x,y): " <<"(" <<current_goal_.pose.position.x << ", " <<current_goal_.pose.position.y << ")" <<std::endl;		
		std::cout << "curr (x,y): " <<"(" <<pose_msg->pose.pose.position.x << ", " << pose_msg->pose.pose.position.y << ")" <<std::endl;
		std::cout << "distance :" << global_dist_err <<std::endl;
		std::cout <<" " <<std::endl;


		//instead of in else loop
		tf::StampedTransform transform;
		tf::TransformListener tf_listener;
		if (!tf_listener.waitForTransform("/odom", "/base_link", ros::Time(0), ros::Duration(0.5), ros::Duration(0.01))) 
		{
			ROS_ERROR("Unable to get pose from TF");
			return;
		}
		try 
		{
			tf_listener.lookupTransform("/odom", "/base_link", ros::Time(0), transform);
		}
		catch (const tf::TransformException &e) {
			ROS_ERROR("%s",e.what());
		} 			
		tf2::Quaternion orientation (transform.getRotation().x(), transform.getRotation().y(), transform.getRotation().z(), transform.getRotation().w());
		tf2::Matrix3x3 m(orientation);

		double roll, pitch, yaw;
		m.getRPY(roll, pitch, yaw);
		g_x_err= global_x_err;
		g_y_err= global_y_err;

		g_ctheta = yaw;
		//check once

		if (goal_index_ % goal_count_ ==0 && g_rtheta_flag==true)
		{
			g_rtheta_flag = false;
      g_rtheta = atan2(global_y_err , global_x_err);
      //y = a(x-current_goal_.pose.position.x) + current_goal_.pose.position.y
      //ax-y + a*current_goal_.pose.position.x+ current_goal_.pose.position.y =0
      //fabs(current_x*a  +current_y*1 +a*current_goal_.pose.position.x+ current_goal_.pose.position.y)/sqrt(a^2+1);
//      line_y_pose = g_rtheta*current_goal_.pose.position.x+ current_goal_.pose.position.y;
		}
		else if( goal_index_ % goal_count_ ==1 && g_rtheta_flag==false)
		{
			//init_flag_ = false;
			g_rtheta_flag = true;
      g_rtheta = atan2(global_y_err , global_x_err);
      //line_y_pose = g_rtheta*current_goal_.pose.position.x+ current_goal_.pose.position.y;
		}
//    line_y_pose = fabs(pose_msg->pose.pose.position.x*g_rtheta  +pose_msg->pose.pose.position.y +g_rtheta*current_goal_.pose.position.x+ current_goal_.pose.position.y)/sqrt(g_rtheta*g_rtheta+1);
//    line_y_pose =(pose_msg->pose.pose.position.x*g_rtheta  +pose_msg->pose.pose.position.y +g_rtheta*current_goal_.pose.position.x+ current_goal_.pose.position.y)/sqrt(g_rtheta*g_rtheta+1);
		line_y_pose = -pose_msg->pose.pose.position.x *sin(g_rtheta) + pose_msg->pose.pose.position.y *cos(g_rtheta); //rtheta_global theta

/*
    //Print goal index
		if(goal_index_ % goal_count_ ==0)
			g_rtheta = atan2(goal_set_[1].pose.position.y - goal_set_[0].pose.position.y , goal_set_[1].pose.position.x-goal_set_[0].pose.position.x);
		else if(goal_index_ % goal_count_ ==1)
			g_rtheta = atan2(goal_set_[0].pose.position.y - goal_set_[1].pose.position.y , goal_set_[0].pose.position.x-goal_set_[1].pose.position.x);
*/
		// 2.1 Not Arrived to the goal position
		if (global_dist_err > config_.global_dist_boundary_ && !is_rotating_) 
		{
			is_arrived = false;
			global_ang_err = M_PI;
			postect_mode = AUTO_LIDAR_MODE;//moving
		}
	
		// 2.2 Arrived to the goal position
		else
		{
			if(arrvial_flag)
			{
				arrvial_flag = false;
				postect_mode = STOP_MODE;//to finish rotate (stop)
			}
			is_arrived = true;
			/*tf::StampedTransform transform;
			tf::TransformListener tf_listener;
			if (!tf_listener.waitForTransform("/odom", "/base_link", ros::Time(0), ros::Duration(0.5), ros::Duration(0.01))) 
			{
				ROS_ERROR("Unable to get pose from TF");
				return;
			}
			try 
			{
				tf_listener.lookupTransform("/odom", "/base_link", ros::Time(0), transform);
			}
			catch (const tf::TransformException &e) {
				ROS_ERROR("%s",e.what());
			} 				
			// angle
			
			tf2::Quaternion orientation (transform.getRotation().x(), transform.getRotation().y(), transform.getRotation().z(), transform.getRotation().w());
			tf2::Matrix3x3 m(orientation);
			double roll, pitch, yaw;
			m.getRPY(roll, pitch, yaw);*/
		
			if(!is_rotating_) 
			{
				std::cout<<"**Arrived to the goal position: "<<global_dist_err<<std::endl;
				static_x = pose_msg->pose.pose.position.x;
				static_y = pose_msg->pose.pose.position.y;
				goal_yaw = yaw + M_PI; // save current when start rotating
				if(goal_yaw > M_PI)
					goal_yaw -= 2*M_PI;
				else if(goal_yaw < -M_PI)
					goal_yaw += 2*M_PI;
				is_rotating_ = true;
				postect_mode =TURN_MODE; //rotating
			}
			static_x_err = static_x - pose_msg->pose.pose.position.x;
			static_y_err = static_y - pose_msg->pose.pose.position.y;
			static_t = goal_yaw;
			static_ct = yaw;
				
			// 2.2.1 Check whether robot should rotate
				//To rotate 180 degree from the position where the mobile robot is arrived. // goalyaw mean "arrival yaw"
			global_ang_err = goal_yaw - yaw;  
			std::cout<<  "start yaw: " << goal_yaw  <<", cur yaw: " << yaw<<std::endl;
			if(global_ang_err > M_PI)
				global_ang_err -= 2*M_PI;
			else if(global_ang_err < -M_PI)
				global_ang_err += 2*M_PI;
			
			std::cout<<"rotating ... bounded_angle_err: "<<global_ang_err<<std::endl;
			if(abs(global_ang_err) < config_.global_angle_boundary_)
			{
				std::cout<<"finish rotation"<<std::endl;
				goal_index_++;
				is_rotating_ = false; //To go another position 
				is_arrived = false;
				postect_mode = STOP_MODE;//to finish rotate (stop) (AUTO_PRE_LIDAR_MODE)
			}
		}
		localization_msgs.data.push_back(global_dist_err);
		localization_msgs.data.push_back(global_ang_err);
		localization_msgs.data.push_back(postect_mode);
		//localization_msgs.data.push_back(is_arrived);
		localization_msgs.data.push_back(is_rotating_);

		localization_msgs.data.push_back(static_x_err);
		localization_msgs.data.push_back(static_y_err);
		localization_msgs.data.push_back(static_t);
		localization_msgs.data.push_back(static_ct);

		localization_msgs.data.push_back(g_x_err);
		localization_msgs.data.push_back(g_y_err);
		localization_msgs.data.push_back(g_rtheta);
		localization_msgs.data.push_back(g_ctheta);

		localization_msgs.data.push_back(init_start_);

		localization_msgs.data.push_back(line_y_pose);
		pub_localization_.publish(localization_msgs);
	}

private:
	ros::Subscriber sub_pose_;
	ros::Subscriber sub_goal_;
	ros::Publisher pub_localization_;
	
	bool is_rotating_ = false;
	bool init_flag_ = false;
	bool init_start_ = false;
  bool g_rtheta_flag =true;


	// GOAL
	int goal_index_ = 0;
	int goal_count_ = 0;    
	double goal_yaw = M_PI;	
	double static_x=0.0, static_y=0.0,static_t=0.0,static_ct=0.0;
	double g_x_err,g_y_err, g_rtheta,g_ctheta;
	int postect_mode;
	bool arrvial_flag =true;
  double line_y_pose = 0;
		
	std::vector<geometry_msgs::PoseStamped> goal_set_;
	geometry_msgs::PoseStamped current_goal_;

	/** configuration parameters */
	typedef struct
	{
		double global_dist_boundary_;
		double global_angle_boundary_;
	} Config;
	Config config_;

};
}
PLUGINLIB_EXPORT_CLASS(auto_driving::LocalizationNode, nodelet::Nodelet);

