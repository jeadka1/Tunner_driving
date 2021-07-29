#include "ros/ros.h"
#include "nodelet/nodelet.h"
#include <pluginlib/class_list_macros.h>

#include <fiducial_msgs/Fiducial.h>
#include <fiducial_msgs/FiducialTransform.h>
#include <fiducial_msgs/FiducialTransformArray.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Float32.h>
//QR 1,2 AMDCL end or start
//QR 3,4 Docking station arrival

//to stop eps > QR 1,2 fiducial_area to road
//to stop QR 3,4 eps >QR 3,4 at station

namespace auto_driving {

class ArucoNode : public nodelet::Nodelet {

public:
	ArucoNode() = default;

private:
	virtual void onInit() {
		ros::NodeHandle nh = getNodeHandle();
		ros::NodeHandle nhp = getPrivateNodeHandle();
		
		// Configuration //
		nhp.param("obstacle_coefficient", config_.obstacle_coefficient_, 0.005);
		nhp.param("front_obstacle_dist", config_.front_obstacle_dist_, 0.1);

		//sub_qr_ = nhp.subscribe("/fiducial_transforms", 1, &ArucoNode::qrCallback, this);
		sub_qr_ = nhp.subscribe("/fiducial_area_d", 1, &ArucoNode::qrCallback, this);

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
void ArucoNode::qrCallback(const std_msgs::Float32MultiArray::ConstPtr& trans_msg)
{
	std_msgs::Float32MultiArray data_test;

	data_test.data.clear();
	data_test.data.push_back(trans_msg->data[0]); // global_dist_err
	data_test.data.push_back(trans_msg->data[1]); // global_dist_err

	std::cout<<"ID: "<<trans_msg->data[0]<<std::endl;
	std::cout<<"Area: "<<trans_msg->data[1]<<std::endl;
	sub_pub_cmd_.publish(data_test);
}
}

PLUGINLIB_EXPORT_CLASS(auto_driving::ArucoNode, nodelet::Nodelet);

