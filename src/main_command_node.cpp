#include "ros/ros.h"
#include "nodelet/nodelet.h"
#include <pluginlib/class_list_macros.h>

#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>
#include <stdio.h>
#include <stdlib.h>


namespace auto_driving {

class MainNode : public nodelet::Nodelet {

public:
	MainNode() = default;

private:
	virtual void onInit() {
		ros::NodeHandle nh = getNodeHandle();
		ros::NodeHandle nhp = getPrivateNodeHandle();
		
		// Configuration //
		nhp.param("obstacle_coefficient", config_.obstacle_coefficient_, 0.005);
		nhp.param("front_obstacle_dist", config_.front_obstacle_dist_, 0.1);

		sub_qr_ = nhp.subscribe("/main_flag", 10, &MainNode::qrCallback, this);

   	sub_pub_cmd_ = nhp.advertise<std_msgs::Float32MultiArray> ("/qr_cmd", 10);
	};

private:
	ros::Subscriber sub_qr_;
	ros::Publisher sub_pub_cmd_;


	void qrCallback(const std_msgs::Float32MultiArray::ConstPtr& trans_msg);


	/** configuration parameters */
	typedef struct
	{
		double obstacle_coefficient_;
		double front_obstacle_dist_;
	} Config;
	Config config_;
};
void MainNode::qrCallback(const std_msgs::Float32MultiArray::ConstPtr& trans_msg)
{
	std_msgs::Float32MultiArray data_test;

	data_test.data.clear();
	data_test.data.push_back(trans_msg->data[0]); // global_dist_err
	data_test.data.push_back(trans_msg->data[1]); // global_dist_err

	system("ls");
	sub_pub_cmd_.publish(data_test);
}
}

PLUGINLIB_EXPORT_CLASS(auto_driving::MainNode, nodelet::Nodelet);

