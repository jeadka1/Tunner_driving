#include "ros/ros.h"
#include "nodelet/nodelet.h"
#include <pluginlib/class_list_macros.h>

#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <signal.h>

namespace auto_driving {

class MaincommandNode : public nodelet::Nodelet {

public:
	MaincommandNode() = default;

private:
	virtual void onInit() {
		ros::NodeHandle nh = getNodeHandle();
		ros::NodeHandle nhp = getPrivateNodeHandle();
		
		// Configuration //
		nhp.param("obstacle_coefficient", config_.obstacle_coefficient_, 0.005);
		nhp.param("front_obstacle_dist", config_.front_obstacle_dist_, 0.1);
//    sub_qr_ = nhp.subscribe<std_msgs::Bool> ("/lidar_driving", 10, &CmdPublishNode::publishCmd, this);
		sub_start_ = nhp.subscribe<std_msgs::Bool> ("/main_flag", 10, &MaincommandNode::MainCallback, this);
		sub_kill_ = nhp.subscribe<std_msgs::Bool> ("/kill_flag", 10, &MaincommandNode::killCallback, this);

   	sub_pub_cmd_ = nhp.advertise<std_msgs::Float32MultiArray> ("/main_test", 10);
	};

private:
	ros::Subscriber sub_start_;
	ros::Subscriber sub_kill_;
	ros::Publisher sub_pub_cmd_;

	int pid_id[10];	

	bool start_up=false;
	void MainCallback(const std_msgs::Bool::ConstPtr &main_msg);
	void killCallback(const std_msgs::Bool::ConstPtr &kill_msg);


	/** configuration parameters */
	typedef struct
	{
		double obstacle_coefficient_;
		double front_obstacle_dist_;
	} Config;
	Config config_;
};
void MaincommandNode::killCallback(const std_msgs::Bool::ConstPtr &kill_msg)
{

	//pid = fork();
  kill(pid_id[0], SIGKILL);
  printf("killed process group %d\n", pid_id[0]);
}
void MaincommandNode::MainCallback(const std_msgs::Bool::ConstPtr &main_msg)
{
	std_msgs::Float32MultiArray data_test;

	data_test.data.clear();
	//data_test.data.push_back(trans_msg); //
	data_test.data.push_back(100); //
	if(!start_up)
	{
		pid_t pid;
		pid = fork();
		if(pid>0)
		{
			
		}
		else if(pid==0)
		{
			pid_id[0] = getpid();
			std::cout<<"child process: "<<pid_id[0]<<std::endl;
			system("cd ~/catkin_ws; roslaunch roverrobotics_driver zero2.launch");
			start_up = true;
		}
		else if (pid ==-1)
		{
			std::cout<<"error"<<std::endl;
		}
	}
	sub_pub_cmd_.publish(data_test);
}
}

PLUGINLIB_EXPORT_CLASS(auto_driving::MaincommandNode, nodelet::Nodelet);

