#include "ros/ros.h"
#include "nodelet/nodelet.h"
#include <pluginlib/class_list_macros.h>


#include <sensor_msgs/Joy.h>
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
#include <std_msgs/Int32.h>
#include <std_msgs/Empty.h>
#include "std_srvs/Empty.h"
#include <leo_driving/charging_done.h>

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
		nhp.param("HJ_MODE", config_.HJ_MODE_, 0);
		nhp.param("Without_QR_move", config_.Without_QR_move_, false);


		sub_joy_ = nhp.subscribe<std_msgs::Empty>("/Doclking_done", 10, &LocalizationNode::DockingCallback, this); // Temporary
		sub_goal_ = nhp.subscribe<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1, &LocalizationNode::setGoal, this);    
		sub_pose_ = nhp.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/amcl_pose", 10, &LocalizationNode::poseCallback, this);

		sub_area_ = nhp.subscribe<std_msgs::Float32MultiArray> ("/fiducial_area_d", 1, &LocalizationNode::areaDataCallback, this);
		sub_mode_ = nhp.subscribe("/mode/low", 10, &LocalizationNode::DecisionpublishCmd, this);

		sub_docking_done_ = nhp.advertiseService("charge_done", &LocalizationNode::docking_done, this);
		
		pub_localization_ = nhp.advertise<std_msgs::Float32MultiArray>("/localization_data", 10); 
		pub_robot_pose_ = nhp.advertise<geometry_msgs::PoseStamped>("/state/pose", 10); 
		pub_QR_= nhp.advertise<std_msgs::Int32>("/QR_mode", 10);
		// [0]: global dist error, [1]: global angle error, [2]: arrival flag, [3]: rotating flag
	};

	void DockingCallback(const std_msgs::Empty::ConstPtr& joy_msg)
	{
		ROS_INFO("Joy A: Behavior ++");
		if(config_.HJ_MODE_!=0)
			behavior_cnt++;
	}
	
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
		pub_robot_pose_.publish(pose_msg);
		std_msgs::Float32MultiArray localization_msgs;
		localization_msgs.data.clear();

		/*char fileName[128];
		FILE *DATA_LOGGER;
		if(FIRST_START_FLAG)
		{
			FIRST_START_FLAG =false;
			sprintf(fileName, "Datalogger_pose.scv");
			DATA_LOGGER = fopen(fileName,"w+");
			fprintf(DATA_LOGGER, "MODE, refx, y, yaw, curx, y, yaw \n");
			fflush(DATA_LOGGER);
		}*/

/*
		geometry_msgs::PoseStamped saved_point1, saved_point2;
		goal_count=2;
		if(goal_index_ % goal_count_ ==0)
		{
			saved_point1.pose.position.x = 2.91127443314;
			saved_point1.pose.position.y = -0.030445933342;
		}
		else
		{
			saved_point2.pose.position.x = 0.498938083649;
			saved_point2.pose.position.y = 0.00525569915771;
		}
*/
		if (goal_count_ < 2)
		{
			ROS_INFO_ONCE("Waiting for inserting goal... ");
			localization_msgs.data.push_back(10000); // global_dist_err
			localization_msgs.data.push_back(10000); // global_ang_err
			localization_msgs.data.push_back(0); // Postech mode
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
		}
		else if( goal_index_ % goal_count_ ==1 && g_rtheta_flag==false)
		{
			g_rtheta_flag = true;
      g_rtheta = atan2(global_y_err , global_x_err);
		}
		line_y_pose = -pose_msg->pose.pose.position.x *sin(g_rtheta) + pose_msg->pose.pose.position.y *cos(g_rtheta); //rtheta_global theta rotation matrix

/*
    //Print goal index
		if(goal_index_ % goal_count_ ==0)
			g_rtheta = atan2(goal_set_[1].pose.position.y - goal_set_[0].pose.position.y , goal_set_[1].pose.position.x-goal_set_[0].pose.position.x);
		else if(goal_index_ % goal_count_ ==1)
			g_rtheta = atan2(goal_set_[0].pose.position.y - goal_set_[1].pose.position.y , goal_set_[0].pose.position.x-goal_set_[1].pose.position.x);
*/
		// 2.1 Not Arrived to the goal position


//----------------------------------------------- current // is_rotating is essential
		//HJ_mode

		std_msgs::Int32 QR_msg;
		
		if(config_.HJ_MODE_==0){
		switch(HJ_mode) //To decide what the mobile robot does
		{
		case AUTO_LIDAR_MODE:
			is_rotating_ = false;
			STOP_cnt =0;
			postech_mode = AUTO_LIDAR_MODE;//moving
			if(fid_ID==1 && fid_area >=5000)
			{
				QR_msg.data =1;
				pub_QR_.publish(QR_msg);//
			}
			else if (fid_ID==2 && fid_area >=5000)
			{
				QR_msg.data =2;
				pub_QR_.publish(QR_msg);//
			}
			break;

		case TURN_MODE:
			if(STOP_cnt<20)
			{
				STOP_cnt++;
				postech_mode = STOP_MODE;//moving
			}
			else
			{
				postech_mode = TURN_MODE;//moving
				if(!is_rotating_)
				{
					is_rotating_ =true;
					std::cout<<"**Arrived to the goal position: "<<global_dist_err<<std::endl;
					static_x = pose_msg->pose.pose.position.x;
					static_y = pose_msg->pose.pose.position.y;
					goal_yaw = yaw + M_PI; // save current when start rotating
					if(goal_yaw > M_PI)
						goal_yaw -= 2*M_PI;
					else if(goal_yaw < -M_PI)
						goal_yaw += 2*M_PI;
				}
				global_ang_err = goal_yaw - yaw;  
				std::cout<<  "start yaw: " << goal_yaw  <<", cur yaw: " << yaw<< ", yaw err:"<<global_ang_err<<std::endl;
				if(global_ang_err > M_PI)
					global_ang_err -= 2*M_PI;
				else if(global_ang_err < -M_PI)
					global_ang_err += 2*M_PI;

				if(abs(global_ang_err) < config_.global_angle_boundary_) // Ending turn
				{
					is_rotating_ =false;
					STOP_cnt =0;
					goal_index_++;
				}

				static_x_err = static_x - pose_msg->pose.pose.position.x;
				static_y_err = static_y - pose_msg->pose.pose.position.y;
				static_t = goal_yaw;
				static_ct = yaw;
			}
			
			break;

		case DOCK_IN_MODE:
			postech_mode = DOCK_IN_MODE;//moving
			is_rotating_ = false;
			STOP_cnt =0;
			break;

		case DOCK_OUT_MODE:
			postech_mode = DOCK_OUT_MODE;//moving
			is_rotating_ = false;
			STOP_cnt =0;
			if(fid_ID==4 && fid_area >=5000)
			{
				init_start_ = true;
				
				goal_index_ ++; //TODO ?? 
				QR_msg.data =4;
				pub_QR_.publish(QR_msg);//
			}
			break;

		case STOP_MODE:
			postech_mode = STOP_MODE;//moving
			is_rotating_ = false;
			STOP_cnt =0;
			break;


		default :
			is_rotating_ = false;
			STOP_cnt =0;
			break;
		}
	}
//----------------------------------------------- postech mode
	else if(config_.HJ_MODE_==1){ //With docking postech mode
		switch(behavior_cnt) // Behave in turn
		{
		case 0://moving to go turning point: start straight
			init_start_ = false;
			postech_mode = AUTO_LIDAR_MODE;
			if(global_dist_err < config_.global_dist_boundary_ || (fid_area >=5000 && fid_ID == 1))
			{
				behavior_cnt++;
				postech_mode = STOP_MODE;
			}
			break;

		case 1://turn : QR or goal pose
			if(STOP_cnt<20)
			{
				STOP_cnt++;
				postech_mode = STOP_MODE;//moving
			}
			else
			{
				postech_mode = TURN_MODE;
				
				if(!is_rotating_)
				{
					is_rotating_ =true;
					std::cout<<"**Arrived to the goal position: "<<global_dist_err<<std::endl;
					static_x = pose_msg->pose.pose.position.x;
					static_y = pose_msg->pose.pose.position.y;
					goal_yaw = yaw + M_PI; // save current when start rotating
					if(goal_yaw > M_PI)
						goal_yaw -= 2*M_PI;
					else if(goal_yaw < -M_PI)
						goal_yaw += 2*M_PI;
				}

				global_ang_err = goal_yaw - yaw;  
				std::cout<<  "start yaw: " << goal_yaw  <<", cur yaw: " << yaw<< ", yaw err:"<<global_ang_err<<std::endl;
				if(global_ang_err > M_PI)
					global_ang_err -= 2*M_PI;
				else if(global_ang_err < -M_PI)
					global_ang_err += 2*M_PI;
				if(abs(global_ang_err) < config_.global_angle_boundary_) // Ending turn
				{
					behavior_cnt++;
					postech_mode = STOP_MODE; //TODO depending on what we're gonna use the sensor (change to publish, rotating done)

					is_rotating_ =false;
					goal_index_++;			
					STOP_cnt=0;	
				}
				static_x_err = static_x - pose_msg->pose.pose.position.x;
				static_y_err = static_y - pose_msg->pose.pose.position.y;
				static_t = goal_yaw;
				static_ct = yaw;
			}
			break;

		case 2: //moving back to home
			postech_mode = AUTO_LIDAR_MODE;
			
			if(global_dist_err < config_.global_dist_boundary_ || (fid_area >=5000 && fid_ID == 2) )
			{
				behavior_cnt++;
				postech_mode = STOP_MODE;
			}
			break;

		case 3://stop -> turn : QR or goal pose
			if(STOP_cnt<20)
			{
				STOP_cnt++;
				postech_mode = STOP_MODE;//moving
			}
			else
			{			
				postech_mode = TURN_MODE;
				if(!is_rotating_)
				{
					is_rotating_ =true;
					std::cout<<"**Arrived to the goal position: "<<global_dist_err<<std::endl;
					static_x = pose_msg->pose.pose.position.x;
					static_y = pose_msg->pose.pose.position.y;
					goal_yaw = yaw + M_PI; // save current when start rotating
					if(goal_yaw > M_PI)
						goal_yaw -= 2*M_PI;
					else if(goal_yaw < -M_PI)
						goal_yaw += 2*M_PI;
				}

				global_ang_err = goal_yaw - yaw;  
				std::cout<<  "start yaw: " << goal_yaw  <<", cur yaw: " << yaw<< ", yaw err:"<<global_ang_err<<std::endl;
				if(global_ang_err > M_PI)
					global_ang_err -= 2*M_PI;
				else if(global_ang_err < -M_PI)
					global_ang_err += 2*M_PI;

				if(abs(global_ang_err) < config_.global_angle_boundary_) // Ending turn
				{
					behavior_cnt++;
					postech_mode = STOP_MODE; 

					is_rotating_ =false;
					STOP_cnt =0;
				}
				static_x_err = static_x - pose_msg->pose.pose.position.x;
				static_y_err = static_y - pose_msg->pose.pose.position.y;
				static_t = goal_yaw;
				static_ct = yaw;
			}
			break;

		case 4:// docking in : 
			postech_mode = DOCK_IN_MODE;//moving
			if(fid_area >=5000 && fid_ID == 3)
			{
				behavior_cnt++;
				postech_mode = STOP_MODE; 
			}
			break;

		case 5://wait : before resservice call
			postech_mode = STOP_MODE; 
			if(Charging_done_flag)//rosserive call
			{
				behavior_cnt++;
				postech_mode = STOP_MODE; 

				Charging_done_flag =0;
			}
			break;

		case 6://dokcing_out : rostopic pub ending_charging
			postech_mode = DOCK_OUT_MODE;//moving
			if(fid_area >=5000 && fid_ID == 4)// To use AMCL pose probably
			{
				behavior_cnt=0;
				postech_mode = STOP_MODE; 
				init_start_ = true;
				goal_index_ ++;
			}
			break;
		default:
			//behavior_cnt=0;
			postech_mode = STOP_MODE;
			ROS_INFO("Behavior error");
			break;
		}
		ROS_INFO("Behavior count: %d, %d", behavior_cnt, behavior_decision);
	}
//-----------------------------------------------------------------------------------------------------
	else if(config_.HJ_MODE_==2){ // without docking
		switch(behavior_cnt) // Behave in turn
		{
		case 0://moving to go turning point: start straight
			init_start_ = false;
			postech_mode = AUTO_LIDAR_MODE;
			if(global_dist_err < config_.global_dist_boundary_ || (fid_area >=5000 && fid_ID == 1))
			{
				behavior_cnt++;
				postech_mode = STOP_MODE;
			}
			break;

		case 1://turn : QR or goal pose
			if(STOP_cnt<20)
			{
				STOP_cnt++;
				postech_mode = STOP_MODE;//moving
			}
			else
			{
				postech_mode = TURN_MODE;//moving
				if(!is_rotating_)
				{
					is_rotating_ =true;
					std::cout<<"**Arrived to the goal position: "<<global_dist_err<<std::endl;
					static_x = pose_msg->pose.pose.position.x;
					static_y = pose_msg->pose.pose.position.y;
					goal_yaw = yaw + M_PI; // save current when start rotating
					if(goal_yaw > M_PI)
						goal_yaw -= 2*M_PI;
					else if(goal_yaw < -M_PI)
						goal_yaw += 2*M_PI;
				}
				global_ang_err = goal_yaw - yaw;  
				std::cout<<  "start yaw: " << goal_yaw  <<", cur yaw: " << yaw<< ", yaw err:"<<global_ang_err<<std::endl;
				if(global_ang_err > M_PI)
					global_ang_err -= 2*M_PI;
				else if(global_ang_err < -M_PI)
					global_ang_err += 2*M_PI;
				if(abs(global_ang_err) < config_.global_angle_boundary_) // Ending turn
				{
					STOP_cnt=0;
					behavior_cnt++;
					postech_mode = STOP_MODE; //TODO depending on what we're gonna use the sensor (change to publish, rotating done)

					is_rotating_ =false;
					goal_index_++;				
				}
				static_x_err = static_x - pose_msg->pose.pose.position.x;
				static_y_err = static_y - pose_msg->pose.pose.position.y;
				static_t = goal_yaw;
				static_ct = yaw;
			}

			
			break;

		case 2: //moving back to home
			postech_mode = AUTO_LIDAR_MODE;
			
			if(global_dist_err < config_.global_dist_boundary_ || (fid_area >=5000 && fid_ID == 2) )
			{
				behavior_cnt++;
				postech_mode = STOP_MODE;
			}
			break;

		case 3://stop -> turn : QR or goal pose
			if(STOP_cnt<20)
			{
				STOP_cnt++;
				postech_mode = STOP_MODE;//moving
			}
			else
			{
				postech_mode = TURN_MODE;//moving
				if(!is_rotating_)
				{
					is_rotating_ =true;
					std::cout<<"**Arrived to the goal position: "<<global_dist_err<<std::endl;
					static_x = pose_msg->pose.pose.position.x;
					static_y = pose_msg->pose.pose.position.y;
					goal_yaw = yaw + M_PI; // save current when start rotating
					if(goal_yaw > M_PI)
						goal_yaw -= 2*M_PI;
					else if(goal_yaw < -M_PI)
						goal_yaw += 2*M_PI;
				}
				global_ang_err = goal_yaw - yaw;  
				std::cout<<  "start yaw: " << goal_yaw  <<", cur yaw: " << yaw<< ", yaw err:"<<global_ang_err<<std::endl;
				if(global_ang_err > M_PI)
					global_ang_err -= 2*M_PI;
				else if(global_ang_err < -M_PI)
					global_ang_err += 2*M_PI;

				if(abs(global_ang_err) < config_.global_angle_boundary_) // Ending turn
				{
					STOP_cnt=0;
					behavior_cnt++;
					postech_mode = STOP_MODE; 

					is_rotating_ =false;
					goal_index_++;

					//Without docking & no QR
					if(config_.Without_QR_move_)
					{
						behavior_cnt=0;
						postech_mode = STOP_MODE;
					}
					
				}
				static_x_err = static_x - pose_msg->pose.pose.position.x;
				static_y_err = static_y - pose_msg->pose.pose.position.y;
				static_t = goal_yaw;
				static_ct = yaw;
			}
			break;
		case 4://Before starting, initialize the position
			postech_mode = AUTO_LIDAR_MODE;
			if(fid_area >=5000 && fid_ID == 3)
			{
				behavior_cnt=0;
				init_start_ =true;
				postech_mode = STOP_MODE;
				ROS_INFO("fid_area: %f", fid_area);
			}
			break;
		default:
			//behavior_cnt=0;
			postech_mode = STOP_MODE;
			ROS_INFO("Behavior error");
			break;
		}
		ROS_INFO("Behavior count: %d, %d", behavior_cnt, behavior_decision);
	}

//-----------------------------------------------
		localization_msgs.data.push_back(global_dist_err);
		localization_msgs.data.push_back(global_ang_err);
		localization_msgs.data.push_back(postech_mode);
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
	/*
		float Data_log[7];
		Data_log[0] = postech_mode;
		if(postech_mode == TURN_MODE)
		{
			Data_log[1] = static_x;
			Data_log[2] = static_y;
			Data_log[3] = goal_yaw;
		}
		else
		{
			Data_log[1] = current_goal_.pose.position.x;
			Data_log[2] = current_goal_.pose.position.y;
			Data_log[3] = goal_yaw;
		}

		Data_log[4] = pose_msg->pose.pose.position.x;
		Data_log[5] = pose_msg->pose.pose.position.y;
		Data_log[6] = yaw;
		fprintf(DATA_LOGGER, "%f,%f,%f,%f,%f,%f,%f\n",Data_log[0],Data_log[1],Data_log[2],Data_log[3],Data_log[4],Data_log[5],Data_log[6]);
		fflush(DATA_LOGGER);*/
	}

//Previous thoughts	
	void DecisionpublishCmd(const std_msgs::Int32::ConstPtr &mode_call);
	void areaDataCallback(const std_msgs::Float32MultiArray::ConstPtr& area_msgs);
//current thoughts
	bool docking_done(std_srvs::Empty::Request &req, std_srvs::Empty::Response &resp);
//	bool docking_done(leo_driving::charging_done::Request& req, leo_driving::charging_done::Response& res);

private:
	ros::Subscriber sub_joy_;
	ros::Subscriber sub_pose_;
	ros::Subscriber sub_goal_;
	ros::Subscriber sub_area_;
	ros::Subscriber sub_mode_;
	ros::ServiceServer sub_docking_done_;
	ros::Publisher pub_localization_;
	ros::Publisher pub_robot_pose_;
	ros::Publisher pub_QR_;
	
	bool is_rotating_ = false;
	bool init_start_ = false;
  bool g_rtheta_flag =true;
	bool FIRST_START_FLAG =true;


	// GOAL
	int goal_index_ = 0;
	int goal_count_ = 0;    
	double goal_yaw = M_PI;	
	double static_x=0.0, static_y=0.0,static_t=0.0,static_ct=0.0;
	double g_x_err,g_y_err, g_rtheta,g_ctheta;
	int postech_mode;
	bool arrvial_flag =true;
  double line_y_pose = 0;

	//Cmaera
	int fid_ID=0;
	float fid_area=0;
	bool turn_mode_start = false;

	//Decision
	unsigned int behavior_cnt =0;
	int behavior_decision=STOP_MODE;
	bool Charging_done_flag =false;
	int HJ_mode=STOP_MODE;

	unsigned int STOP_cnt =0;
		
	std::vector<geometry_msgs::PoseStamped> goal_set_;
	geometry_msgs::PoseStamped current_goal_;

	/** configuration parameters */
	typedef struct
	{
		double global_dist_boundary_;
		double global_angle_boundary_;
		int HJ_MODE_;
		bool Without_QR_move_;
	} Config;
	Config config_;

};
void LocalizationNode::areaDataCallback(const std_msgs::Float32MultiArray::ConstPtr& area_msgs)
{
	fid_ID = (int) area_msgs->data[0];
	fid_area =area_msgs->data[1];
	//ROS_INFO("fid_ID: %d, are: %f",fid_ID, fid_area);
}
void LocalizationNode::DecisionpublishCmd(const std_msgs::Int32::ConstPtr &mode_call)
{
	HJ_mode = mode_call->data;
}
bool LocalizationNode::docking_done(std_srvs::Empty::Request &req, std_srvs::Empty::Response &resp)
//bool LocalizationNode::docking_done(leo_driving::charging_done::Request& req, leo_driving::charging_done::Response& res)
{
	//Charging_done_flag = true;
	behavior_cnt++;
	return true;
}
}
PLUGINLIB_EXPORT_CLASS(auto_driving::LocalizationNode, nodelet::Nodelet);

