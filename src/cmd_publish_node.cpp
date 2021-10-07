#include "ros/ros.h"
#include "nodelet/nodelet.h"
#include <pluginlib/class_list_macros.h>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <algorithm>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <leo_driving/Mode.h>


using namespace message_filters;

enum MODE_{
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

class CmdPublishNode : public nodelet::Nodelet {
	bool joy_driving_ = false; // even: auto, odd: joy control
	// Obs
	float obs_x_ = 10000;
	float obs_y_ = 10000;	
	bool was_obs_in_aisle = false;
	double spare_length = 0;
  float temp_y_err_local = 0;
	float Max_speed = 0.5;
	unsigned int align_cnt=0;
	// Amcl
	float global_dist_err_ = 0;
	float global_ang_err_ = 0;  
	float global_x_err_ =0;  
	float global_y_err_ =0,global_r_theta_=0.0,global_c_theta_=0.0;

	float g_x_err_=0, g_y_err_=0,g_rtheta_=0, g_ctheta_ =0;

	float liney_pose=0;
	float Docking_speed=0;

	bool init_call = false;
	unsigned int init_cnt=0;
	bool once_flag = false;
	bool RP_MODE= true;
	bool local_data_receive = false;

	// Aisle
	float line_start_y_ = -30; 
	float line_end_y_ = 30;
	float ref_y_ = 0;
  float ref_x_ = 0;
	float near_y_ = 0;

	//int fid_ID=0;
	//float fid_area=0;
	int postech_mode_=0;
	int HJ_mode_low=STOP_MODE;
	unsigned int HJ_mode_cnt =0;

	// 
	bool is_rotating_ = false;
	bool is_arrived_ = true;
	//bool is_linetracking = false;
	bool gmapping_go =false;



public:
	CmdPublishNode() = default;

private:
	virtual void onInit() {
	ros::NodeHandle nh = getNodeHandle();
	ros::NodeHandle nhp = getPrivateNodeHandle();

    // Configuration
    nhp.param("Kpx_param", config_.Kpx_param_, 2.0);
    nhp.param("Kpy_param", config_.Kpy_param_, 1.1);
    nhp.param("Kpy_param_rot", config_.Kpy_param_rot_, 0.01);
    nhp.param("linear_vel", config_.linear_vel_, 0.0);
    nhp.param("robot_width", config_.robot_width_, 0.45);
    nhp.param("line_width_min", config_.line_width_min_, 0.7);
    nhp.param("line_width_max", config_.line_width_max_, 1.0);
    nhp.param("obs_coefficient", config_.obs_coefficient_, 0.5);
    nhp.param("front_obs", config_.front_obs_, 0.6);
    nhp.param("boundary_percent", config_.boundary_percent_, 0.02);
    nhp.param("spare_length", config_.spare_length_, 0.3);
    nhp.param("amcl_driving", config_.amcl_driving_, true);
    nhp.param("check_obstacles", config_.check_obstacles_, false);
    nhp.param("rot_kx", config_.rot_kx_, 0.1);
    nhp.param("rot_ky", config_.rot_ky_, 0.1);
    nhp.param("rot_kt", config_.rot_kt_, 0.3);
    nhp.param("min_vel", config_.min_vel_, 0.05);
    nhp.param("min_rot", config_.min_rot_, 0.05);
    nhp.param("max_vel", config_.max_vel_, 0.5);
    nhp.param("max_rot", config_.max_rot_, 1.0);
    nhp.param("straight", config_.straight_, false);
    nhp.param("Kpy_param_straight", config_.Kpy_param_straight_, 1.1);
    nhp.param("Postech_code", config_.Postech_code_, false);
    nhp.param("straight_align_bound", config_.straight_align_bound_, 0.1);
    nhp.param("obs_avoidance_distance", config_.obs_avoidance_distance_, 0.1);




    // // Subscriber & Publisher
    sub_joy_ = nhp.subscribe<sensor_msgs::Joy>("/joystick", 1, &CmdPublishNode::joyCallback, this);
    sub_obs_dists_ = nhp.subscribe<std_msgs::Float32MultiArray> ("/obs_dists", 10, &CmdPublishNode::obsCallback, this);
    sub_aisle_ = nhp.subscribe<sensor_msgs::PointCloud2> ("/aisle_points", 10, &CmdPublishNode::aisleCallback, this);    


    sub_localization_ = nhp.subscribe<std_msgs::Float32MultiArray> ("/localization_data", 10, &CmdPublishNode::localDataCallback, this);
    sub_mode_call_ = nhp.subscribe<std_msgs::Int32> ("/mode/low", 10, &CmdPublishNode::modeCallback, this);
    sub_driving_ = nhp.subscribe<std_msgs::Int32> ("/cmd_publish", 10, &CmdPublishNode::publishCmd, this);
/*
		message_filters::Subscriber<std_msgs::Float32MultiArray> sub_localization_(nhp, "localization_data", 10);
		message_filters::Subscriber<std_msgs::Int32> sub_driving_(nhp, "/mode/low", 10);
		TimeSynchronizer<std_msgs::Float32MultiArray, std_msgs::Int32> sync(sub_localization_, sub_driving_, 10);
		sync.registerCallback(boost::bind(&CmdPublishNode::publishCmd,this, _1, _2));
*/


    sub_speed_ = nhp.subscribe("/mission/setspeed", 1, &CmdPublishNode::SpeedCallback, this);
    //sub_integratedpose_ = nhp.subscribe("/state/pose", 1, &CmdPublishNode::PoseCallback, this);

    //sub_area_ = nhp.subscribe<std_msgs::Float32MultiArray> ("/fiducial_area_d", 1, &CmdPublishNode::areaDataCallback, this);

    pub_cmd_ = nhp.advertise<geometry_msgs::Twist> ("/cmd_vel", 10);
    pub_docking_end_ = nhp.advertise<std_msgs::Int32> ("/Doclking_done", 1); // Temperary To finish with joystick
    pub_prelidar_end_ = nhp.advertise<std_msgs::Empty> ("/auto_pre_lidar_mode/end", 10);
	};

    void joyCallback(const sensor_msgs::Joy::ConstPtr& joy_msg)
    {
        std_msgs::Int32 joy_msg_to_node;
        //Button "B" : driving mode change -->   even: auto, odd: joy control
        if (joy_msg->buttons[1] == 1)	
        {
            std::cout<<"(push) B "<<std::endl;
            joy_driving_ = !joy_driving_;
            gmapping_go = true; //Keep TRUE
            ros::Duration(0.5).sleep();
        }
        if(joy_msg->buttons[0] == 1)	 //
        {
            std::cout<<"(push) A "<<std::endl;
						joy_msg_to_node.data=0;
						pub_docking_end_.publish(joy_msg_to_node);
						ros::Duration(0.5).sleep();
        }
				if(joy_msg->buttons[3] == 1)	 //
        {
            std::cout<<"(push) X "<<std::endl;
						joy_msg_to_node.data=3;
						pub_docking_end_.publish(joy_msg_to_node);
						ros::Duration(0.5).sleep();
        }
				if(joy_msg->buttons[3] == 4)	 //
        {
            std::cout<<"(push) Y "<<std::endl;
						init_call = true;
						//joy_msg_to_node.data=3;
						//pub_docking_end_.publish(joy_msg_to_node);
						ros::Duration(0.5).sleep();
        }
        if(joy_driving_)
        { 
            geometry_msgs::Twist cmd_vel;
            cmd_vel.linear.x = joy_msg -> axes[1] * 1.0;
            cmd_vel.angular.z = joy_msg -> axes[0] * 1.0;
            pub_cmd_.publish(cmd_vel);
            return;
        }
				
    }

	void obsCallback(const std_msgs::Float32MultiArray::ConstPtr& dists_msg)
    {
		if (dists_msg->data.size())
		{
		    obs_x_ = dists_msg->data[0];
		    obs_y_ = dists_msg->data[1];
		}
		else
		{
		    obs_x_ = 1000000;
		    obs_y_ = 1000000;
		}
    }

    void aisleCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg)
    {
        pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2; 
        pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
        // Convert to PCL data type
        pcl_conversions::toPCL(*cloud_msg, *cloud);
        pcl::PointCloud<pcl::PointXYZ>::Ptr data_cloud(new pcl::PointCloud<pcl::PointXYZ>); 
        pcl::fromPCLPointCloud2(*cloud, *data_cloud); 

        near_y_ = data_cloud->points[0].y;
        ref_y_ = data_cloud->points[1].y;
        ref_x_ = data_cloud->points[1].x;
        line_start_y_ = data_cloud->points[2].y;
        line_end_y_ = data_cloud->points[3].y;        
    }
    void SpeedCallback(const std_msgs::Float32::ConstPtr& speed_msgs)
    {
        config_.linear_vel_ = speed_msgs->data;
    }


    void localDataCallback(const std_msgs::Float32MultiArray::ConstPtr& local_msgs)
    {
        global_dist_err_ = local_msgs->data[0]; 
        global_ang_err_ = local_msgs->data[1];
        //is_arrived_ = local_msgs->data[2];
        postech_mode_ = local_msgs->data[2];
        is_rotating_ = local_msgs->data[3];

        global_x_err_ = local_msgs->data[4];
        global_y_err_ = local_msgs->data[5];
        global_r_theta_ = local_msgs->data[6];
        global_c_theta_ = local_msgs->data[7];

        g_x_err_ = local_msgs->data[8];
        g_y_err_ = local_msgs->data[9];
        g_rtheta_ = local_msgs->data[10];
        g_ctheta_ = local_msgs->data[11];

        init_call = local_msgs->data[12];

        liney_pose = local_msgs->data[13];
				local_data_receive =true;
    }




    void modeCallback(const std_msgs::Int32::ConstPtr &Mode_value)
		{
			HJ_mode_low = Mode_value->data;
			HJ_mode_cnt =0;
		}
    void publishCmd(const std_msgs::Int32::ConstPtr &driving_start)
		{

        std_msgs::Empty EmptyMsg;
        geometry_msgs::Twist cmd_vel;
        int Mode_type;
				HJ_mode_cnt++;
				
				if(config_.Postech_code_)
            Mode_type  = postech_mode_;
				else
				{
					if(HJ_mode_cnt >=10)
						HJ_mode_low = STOP_MODE;
					Mode_type = HJ_mode_low;
				}		

				if(config_.amcl_driving_ ==false)
				{
					Mode_type = AUTO_LIDAR_MODE;
					if(!gmapping_go)  
            return;
				}
				
        if(joy_driving_ || driving_start->data == MANUAL_MODE)
            Mode_type = MANUAL_MODE;


        //std::cout<<"Mode_type: "<<Mode_type<<std::endl;
        if(init_call || init_cnt !=0)
        {
						init_call = false;
						init_cnt++;
            cmd_vel.linear.x = 0.0;
            cmd_vel.linear.z = 0.0;
            pub_cmd_.publish(cmd_vel);
						if(init_cnt==10)
						{
							system("rosservice call /odom_init 0.0 0.0 0.0"); //Intialize Encoder
		          system("rosservice call /reset_odom"); //Intialize IMU
		          system("rosservice call /pose_update 0.0 0.0 0.0"); //Intialize AMCL
						}
						if(init_cnt==80)
							init_cnt =0;
						return;
        }
        
        //// 2. Autonomous Driving

        float y_err_local = ref_y_ - near_y_;
        // 2.1 Check Obstacles

        if (config_.check_obstacles_ && (Mode_type == AUTO_LIDAR_MODE || Mode_type == AUTO_IMAGE_MODE))
        {
            float line_length = line_end_y_ - line_start_y_;
            float left_boundary = line_start_y_ - (line_length * config_.boundary_percent_ + 0.5 * config_.robot_width_);
            float right_boundary = line_end_y_ + (line_length * config_.boundary_percent_ + 0.5 * config_.robot_width_);
            bool is_obs_in_aisle = obs_y_ > line_end_y_ && obs_y_ < line_start_y_;
            // (0) Front Obstacle Update
        		
						//std::cout << "Obs_x : " << obs_x_ << ", ref_x : " <<  ref_x_<<std::endl;
/*
            if (obs_x_ < config_.front_obs_ && abs(obs_y_) < config_.robot_width_/4 && !is_rotating_)
            {
                cmd_vel.linear.x = 0.0;
                cmd_vel.linear.z = 0.0;
                pub_cmd_.publish(cmd_vel);
                std::cout<<"Front obstacle is deteced"<<std::endl;
                return;
            }*/ // below  else if
            
        // (1) Right Obstacle Update	(y:오른쪽이 음수)
            if(obs_y_ < 0 && obs_y_ > -1 && obs_x_< 0.6)
            {	
//Start- end = length
//robot_length = 0.5m
//0.1 m + 0.1m
                std::cout << "Right obstacle is detected, distance = " << obs_y_ << ", x = " <<  obs_x_<<std::endl;
                //float shift = config_.obs_coefficient_*(line_end_y_ - obs_y_);
                //y_err_local = (near_y_ + shift > left_boundary) ? left_boundary - near_y_ : y_err_local + shift;
                //y_err_local = (line_end_y_+obs_y_)/2 - near_y_;
								y_err_local = ref_y_-config_.obs_avoidance_distance_ - near_y_;
                temp_y_err_local = 1;
                was_obs_in_aisle = true;
                spare_length = 0;
            }
            // (2) Left Obstacle Update (y:왼쪽이 양수)
            else if(obs_y_ > 0 && obs_y_ < 1 && obs_x_< 0.6)
            {
                std::cout << "Left obstacle is detected, distance = " << obs_y_ << ", x = " <<  obs_x_<<std::endl;
                //float shift = config_.obs_coefficient_*(line_start_y_ - obs_y_);
                //y_err_local = (near_y_ + shift < right_boundary) ? right_boundary - near_y_ : y_err_local + shift;
//                y_err_local = (obs_y_+line_start_y_)/2 - near_y_;
								y_err_local = ref_y_+config_.obs_avoidance_distance_ - near_y_;
                temp_y_err_local = -1;
                was_obs_in_aisle = true;
                spare_length = 0;
            }
            // (3) After obs disappear, go further 'spare_length'
            if(!is_obs_in_aisle && was_obs_in_aisle)
            { 
                spare_length += config_.linear_vel_ * 0.1;
                //y_err_local = temp_y_err_local;
		if(temp_y_err_local ==1)
			y_err_local = ref_y_ - config_.obs_avoidance_distance_ - near_y_;
		else if (temp_y_err_local == -1)
			y_err_local = ref_y_ + config_.obs_avoidance_distance_ - near_y_;
                std::cout<< "straight foward of spare distance" <<std::endl;
                if(spare_length > config_.spare_length_)
                {
                    spare_length = 0;
                    was_obs_in_aisle = false;
                    std::cout<<"spare finish"<<std::endl;
                }
            }

        }
        

        float straight_l_xerr, straight_l_yerr;
        float comy_yerr;
        float l_xerr,l_yerr;

        switch(Mode_type)
        {
            case MANUAL_MODE:
            break;
            case STOP_MODE:
                cmd_vel.linear.x = 0.0;
                cmd_vel.linear.z = 0.0;
                pub_cmd_.publish(cmd_vel);
            break;

            case AUTO_LIDAR_MODE:

                straight_l_xerr= g_x_err_ *cos(g_ctheta_) + g_y_err_ *sin(g_ctheta_);
                straight_l_yerr= -g_x_err_ *sin(g_ctheta_) + g_y_err_ *cos(g_ctheta_);
                //comy_yerr= -g_x_err_ *sin(g_rtheta_) + g_y_err_ *cos(g_rtheta_); //rtheta_global theta
                comy_yerr= liney_pose; //rtheta_global theta
                //comy_yerr= g_rtheta_-atan2(g_y_err_,g_x_err_);

                /*
                    std::cout<< "g_x_err_: " <<g_x_err_ <<" , g_y_err_: " <<g_y_err_ <<std::endl;
                    std::cout<< "rtheta: " <<g_rtheta_ <<" , ctheta: " <<g_ctheta_ <<std::endl;
                    std::cout<< "l_x: " <<straight_l_xerr <<", l_y:" << straight_l_yerr <<std::endl;
                    std::cout<< "comp y: "<< comy_yerr <<std::endl;
                */
                if(config_.straight_) // straight test
                {
			            std::cout<<"sy err: "<<straight_l_yerr<<std::endl;
			            std::cout<<"compy err: "<<comy_yerr<<", liney_pose: "<<liney_pose<<std::endl;
                    cmd_vel.linear.x = config_.linear_vel_*straight_l_xerr; // To stop slowly when arriving at the point
                //cmd_vel.linear.x = config_.linear_vel_;
                //cmd_vel.angular.z = config_.Kpy_param_ * straight_l_yerr + config_.Kpy_param_straight_*comy_yerr; //g_rtheta
                    cmd_vel.angular.z = config_.Kpy_param_ * straight_l_yerr - config_.Kpy_param_straight_*comy_yerr; //liney_pose
                /*
                        if(comy_yerr<config_.straight_align_bound_ && comy_yerr>-config_.straight_align_bound_)
                {
                            cmd_vel.angular.z = config_.Kpy_param_*straight_l_yerr;
                ROS_INFO("con 1");
                }
                        else
                {
                            cmd_vel.angular.z = -config_.Kpy_param_straight_*comy_yerr; 
                ROS_INFO("con 2");
                }
                */

                    //Saturation parts due to Zero's deadline from VESC
                    if(cmd_vel.linear.x> config_.max_vel_)
                        cmd_vel.linear.x = config_.max_vel_;
                    else if(cmd_vel.linear.x< -config_.max_vel_)
                        cmd_vel.linear.x = -config_.max_vel_;


                    if(cmd_vel.angular.z> config_.max_rot_)
                        cmd_vel.angular.z = config_.max_rot_;
                    else if(cmd_vel.angular.z< -config_.max_rot_)
                        cmd_vel.angular.z = -config_.max_rot_;				        				        

                    if(cmd_vel.linear.x< config_.min_vel_ && cmd_vel.linear.x>0)
                        cmd_vel.linear.x = config_.min_vel_;
                    else if(cmd_vel.linear.x> -config_.min_vel_ && cmd_vel.linear.x<0)
                        cmd_vel.linear.x = -config_.min_vel_;

                //Saturation to move correctly
            /*
                    if(cmd_vel.angular.z< config_.min_rot_ && cmd_vel.angular.z>0)//To rotate minimum speed at cw
                        cmd_vel.angular.z = config_.min_rot_;
                    else if(cmd_vel.angular.z> config_.max_rot_)
                        cmd_vel.angular.z = config_.max_rot_;
            if(cmd_vel.angular.z> -config_.min_rot_ && cmd_vel.angular.z<0) //To rotate minimum speed at ccw
                        cmd_vel.angular.z = -config_.min_rot_;
                    else if(cmd_vel.angular.z< -config_.max_rot_)
                        cmd_vel.angular.z = -config_.max_rot_;*/
                }
                else // tunnel AMCL test
                {
                //cmd_vel.linear.x = config_.linear_vel_*straight_l_xerr; // To stop slowly when arriving at the point
                    cmd_vel.linear.x = config_.linear_vel_;
                    cmd_vel.angular.z = -config_.Kpy_param_ * y_err_local; 

                    //Saturation parts due to Zero's deadline from VESC
                    if(cmd_vel.linear.x< config_.min_vel_ && cmd_vel.linear.x>0)
                        cmd_vel.linear.x = config_.min_vel_;


                    if(cmd_vel.linear.x> config_.max_vel_)
                        cmd_vel.linear.x = config_.max_vel_;
                    else if(cmd_vel.linear.x< -config_.max_vel_)
                        cmd_vel.linear.x = -config_.max_vel_;

                    if(cmd_vel.angular.z> config_.max_rot_)
                        cmd_vel.angular.z = config_.max_rot_;
                    else if(cmd_vel.angular.z< -config_.max_rot_)
                        cmd_vel.angular.z = -config_.max_rot_;
                    //std::cout<<"x: "<<cmd_vel.linear.x<< ", z: " << cmd_vel.angular.z<<std::endl;
                    //Saturation of 'cmd_vel.linear.x' doesn't need because when the x fits, it's not necessary to move
                }
            pub_cmd_.publish(cmd_vel);
            break;

            case TURN_MODE:
								if(local_data_receive)
								{
									local_data_receive =false;
		//global_x_err_ -= 0.08*cos(global_c_theta_);
		//global_y_err_ -= 0.08*sin(global_c_theta_);
                l_xerr= global_x_err_ *cos(global_c_theta_) + global_y_err_ *sin(global_c_theta_);
                l_yerr= -global_x_err_ *sin(global_c_theta_) + global_y_err_ *cos(global_c_theta_);
                //std::cout<< "static_x: " <<l_xerr <<", static_y:" << l_yerr <<std::endl;
								//l_xerr -= 0.03;
								if(fabs(global_c_theta_ - global_c_theta_)< 0.4 || fabs(global_c_theta_ - global_c_theta_)> 2.7)
								{
								cmd_vel.linear.x = config_.rot_kx_*l_xerr;
	              cmd_vel.angular.z = config_.rot_ky_ *l_yerr + config_.rot_kt_ *(global_r_theta_ - global_c_theta_);
								}
								else
								{
									cmd_vel.linear.x = 0.0;
		              cmd_vel.angular.z = config_.rot_ky_ *l_yerr + config_.rot_kt_ *(global_r_theta_ - global_c_theta_);
								}
/*
								if(fabs(l_xerr)>0.05)
								{
									cmd_vel.linear.x = config_.rot_kx_*l_xerr;
	                cmd_vel.angular.z = 0.0;
								}
								else
								{
		              cmd_vel.linear.x = 0.0;
		              cmd_vel.angular.z = config_.rot_ky_ *l_yerr + config_.rot_kt_ *(global_r_theta_ - global_c_theta_);
								}
*/

                //Saturation parts due to Zero's deadline from VESC
                //Saturation of 'cmd_vel.linear.x' doesn't need because when the x fits, it's not necessary to move
                if(cmd_vel.angular.z< config_.min_rot_ && cmd_vel.angular.z>0)//To rotate minimum speed at cw
                    cmd_vel.angular.z = config_.min_rot_;
                else if(cmd_vel.angular.z> -config_.min_rot_ && cmd_vel.angular.z<0) //To rotate minimum speed at ccw
                    cmd_vel.angular.z = -config_.min_rot_;

                if(cmd_vel.angular.z> config_.max_rot_)
                    cmd_vel.angular.z = config_.max_rot_;
                else if(cmd_vel.angular.z< -config_.max_rot_)
                    cmd_vel.angular.z = -config_.max_rot_;
                pub_cmd_.publish(cmd_vel);
								}
            break;

            case AUTO_PRE_LIDAR_MODE:
                align_cnt++;
                cmd_vel.linear.x = 0.0;
                cmd_vel.angular.z = -config_.Kpy_param_ * y_err_local; 
                if(cmd_vel.angular.z< config_.min_rot_ && cmd_vel.angular.z>0)//To rotate minimum speed at cw
                    cmd_vel.angular.z = config_.min_rot_;
                else if(cmd_vel.angular.z> config_.max_rot_)
                    cmd_vel.angular.z = config_.max_rot_;
                if(cmd_vel.angular.z> -config_.min_rot_ && cmd_vel.angular.z<0) //To rotate minimum speed at ccw
                    cmd_vel.angular.z = -config_.min_rot_;
                else if(cmd_vel.angular.z< -config_.max_rot_)
                    cmd_vel.angular.z = -config_.max_rot_;

                pub_cmd_.publish(cmd_vel);
                if(y_err_local < 0.05 || align_cnt >=70 )//0.05 cm
                {
                    align_cnt=0;
                    pub_prelidar_end_.publish(EmptyMsg);
                }
            break;

            case DOCK_IN_MODE: //Hanjeon gives us the err of y
                cmd_vel.linear.x = -0.1;
                cmd_vel.angular.z = -config_.Kpy_param_ * y_err_local; 

                if(cmd_vel.linear.x> config_.max_vel_)
                    cmd_vel.linear.x = config_.max_vel_;
                else if(cmd_vel.linear.x< -config_.max_vel_)
                    cmd_vel.linear.x = -config_.max_vel_;

                if(cmd_vel.angular.z> config_.max_rot_)
                    cmd_vel.angular.z = config_.max_rot_;
                else if(cmd_vel.angular.z< -config_.max_rot_)
                    cmd_vel.angular.z = -config_.max_rot_;

                pub_cmd_.publish(cmd_vel);
            break;
            case DOCK_OUT_MODE: // with RPlidar
                cmd_vel.linear.x = 0.1;
                cmd_vel.angular.z = -config_.Kpy_param_ * y_err_local; 

                if(cmd_vel.linear.x> config_.max_vel_)
                    cmd_vel.linear.x = config_.max_vel_;
                else if(cmd_vel.linear.x< -config_.max_vel_)
                    cmd_vel.linear.x = -config_.max_vel_;

                if(cmd_vel.angular.z> config_.max_rot_)
                    cmd_vel.angular.z = config_.max_rot_;
                else if(cmd_vel.angular.z< -config_.max_rot_)
                    cmd_vel.angular.z = -config_.max_rot_;

                pub_cmd_.publish(cmd_vel);
            break;					

            default:
            break;
        }
				//std::cout<< "static_g_x_err_: "<< global_x_err_ <<", static_g_y_err_: " << global_y_err_<< std::endl;
				//std::cout<< "l_xerr: "<< l_xerr <<", l_yerr: " << l_yerr<< std::endl;
				//std::cout<< "goal_yaw: "<< global_r_theta_ <<", yaw: " << global_c_theta_<< std::endl;
    }

private:
	// Publisher & Subscriber
	ros::Subscriber sub_joy_;
    ros::Subscriber sub_obs_dists_;
	ros::Subscriber sub_aisle_;
	ros::Subscriber sub_localization_;
	ros::Subscriber sub_driving_;
	ros::Subscriber sub_mode_call_;
	//ros::Subscriber sub_area_;
	ros::Subscriber sub_speed_;
	//ros::Subscriber sub_integratedpose_;
    
	ros::Publisher pub_cmd_;
	ros::Publisher pub_prelidar_end_;
	ros::Publisher pub_docking_end_;



	/** configuration parameters */
	typedef struct
	{
		double Kpx_param_;
		double Kpy_param_;
		double Kpy_param_rot_;
    double linear_vel_;
    double robot_width_;
    double obs_coefficient_;
    double front_obs_;
    double spare_length_;
    double boundary_percent_;;
    double line_width_min_;
    double line_width_max_;
    bool amcl_driving_;
    bool check_obstacles_;
		
		double rot_kx_;
		double rot_ky_;
		double rot_kt_;
		double min_vel_;
		double min_rot_;
		double max_vel_;
		double max_rot_;
		bool straight_;
		double Kpy_param_straight_;
		bool Postech_code_;
    double straight_align_bound_;
		double obs_avoidance_distance_;
	} Config;
	Config config_;
};
}
PLUGINLIB_EXPORT_CLASS(auto_driving::CmdPublishNode, nodelet::Nodelet);
