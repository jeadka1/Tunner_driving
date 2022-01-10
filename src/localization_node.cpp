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
#include <nav_msgs/MapMetaData.h>
#include "nav_msgs/Odometry.h"
#include "time.h"

#include <leo_driving/charging_done.h>
#include <leo_driving/PlotMsg.h>

#define STOP_MAX 30


#define TorotateAtHome 0.3 //m
#define QR_DISTANCE 2000
#define QR_reset    100//To initialize Docking out
#define QR_home     3//101 To rotate at home
#define QR_end      4//102 To rotate to go home

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
        nhp.param("Main_start_x", config_.Main_start_x_, 0.0);
        nhp.param("Main_start_y", config_.Main_start_y_, 0.0);
        nhp.param("Main_goal_x", config_.Main_goal_x_, 0.0);
        nhp.param("Main_goal_y", config_.Main_goal_y_, 0.0);


        sub_joy_ = nhp.subscribe<std_msgs::Int32>("/joy_from_cmd", 10, &LocalizationNode::DockingCallback, this); // Joystick data from cmd_node



        sub_goal_ = nhp.subscribe<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1, &LocalizationNode::setGoal, this);//Clicked point from RVIZ
        sub_pose_ = nhp.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/amcl_pose", 10, &LocalizationNode::UpdateposeCallback, this);//Pose callback from amcl node
        sub_pose_driving_ = nhp.subscribe<std_msgs::Int32> ("/cmd_publish", 10, &LocalizationNode::poseCallback, this);
        //	sub_pose_ = nhp.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/cmd_publish", 10, &LocalizationNode::poseCallback, this);

        sub_area_ = nhp.subscribe<std_msgs::Float32MultiArray> ("/fiducial_area_d", 1, &LocalizationNode::areaDataCallback, this);
        sub_mode_ = nhp.subscribe("/mode/low", 10, &LocalizationNode::DecisionpublishCmd, this); //To get a mode/low from Hanjeon

        sub_QRinit_ = nhp.subscribe("/QR_TEST", 1, &LocalizationNode::QRtestCallback, this); //While HJ_mode==1 or 2, the mode is changed to another HJ_mode ==2 or 1
        sub_odom_ = nhp.subscribe("/odom", 1, &LocalizationNode::OdomCallback, this);

        sub_predone_ = nhp.subscribe("/auto_pre_lidar_mode/end", 1, &LocalizationNode::predoneCallback, this);//After finishing AUTO_PRE_LIDAR MODE

        sub_mode_decision_ = nhp.subscribe<std_msgs::Int32>("/Mode_Decision", 1, &LocalizationNode::ModedecisionCallback, this);//To jump behavior_cnd



        //sub_docking_done_ = nhp.advertiseService("charge_done", &LocalizationNode::docking_done, this); //Docking done from robot-station

        //pcw for QR local

        pub_for_test_QR_local = nhp.advertise<geometry_msgs::Point>("QR_local_pose", 10); //To communicate with cmd_node
        //

        pub_localization_ = nhp.advertise<std_msgs::Float32MultiArray>("/localization_data", 10); //To communicate with cmd_node
        pub_robot_pose_ = nhp.advertise<geometry_msgs::PoseStamped>("/state/pose", 10);//To send to robot-station or Hanjeon
        pub_QR_= nhp.advertise<std_msgs::Int32>("/QR_mode", 10); //To send to robot-station or Hanjeon
        pub_log_data_= nhp.advertise<leo_driving::PlotMsg>("/PlotMsg_data", 10); //To save the data to plot
        pub_mode_call_= nhp.advertise<std_msgs::Int32>("/mode/low", 10); //To use Dock in wiht Hanjeon
        \
    }


    void DockingCallback(const std_msgs::Int32::ConstPtr& joy_msg)
    {
        if(config_.HJ_MODE_!=0)
        {
            switch(joy_msg->data)
            {
            case 0:
                //behavior_cnt++;
                Next_step = true;
                ROS_INFO("Joy A: Behavior ++");
                break;
            case 1:
                Joy_mode=true;
                break;
            case 10:
                Joy_mode=false;
                break;
            case 3:
                behavior_cnt=0;
                ROS_INFO("Joy X: Behavior 0");
                break;

            }
        }
    }



    void setGoal(const geometry_msgs::PoseStamped::ConstPtr& click_msg)
    {
        ROS_INFO("%d th goal is set", goal_count_);
        //ROS_INFO("x: %d, y: %d", click_msg.pose.position.x, click_msg.pose.position.y);
        goal_set_.clear();
        goal_set_.push_back(*click_msg);
        goal_count_ ++;
        new_goal_flag=true; //TODO When the goal is set to basic goal, this parameter should be 'false'
    }
    void UpdateposeCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& amcl_pose_msg)
    {
        pub_robot_pose_.publish(amcl_pose_msg);
        current_pose = *amcl_pose_msg;
        //system("rosservice call /odom_init 0.0 0.0 0.0"); //Intialize Encoder
    }
    void poseCallback(const std_msgs::Int32::ConstPtr &empty_pose_msg)
    {
        if(FIRST_START_FLAG==true)
        {
            postech_mode = STOP_MODE;
            FIRST_START_FLAG=false;
            if(config_.HJ_MODE_==1)//To start from Docking station
                behavior_cnt =6;
            else //To start iterative driving
                behavior_cnt =0;
        }


        geometry_msgs::PoseWithCovarianceStamped pose_msg;
        pose_msg = current_pose;
        std_msgs::Int32 mode_dockin;
        std_msgs::Float32MultiArray localization_msgs;
        localization_msgs.data.clear();


        if(start_wait<150)
        {
            start_wait++;
            ROS_INFO_ONCE("Waiting for inserting goal... ");
            localization_msgs.data.push_back(0); // Postech mode


            localization_msgs.data.push_back(0); // global_x_err
            localization_msgs.data.push_back(0); // global_y_err
            localization_msgs.data.push_back(0); // global_theta_err
            localization_msgs.data.push_back(0); // global_c_theta_err

            localization_msgs.data.push_back(0); // initcall
            localization_msgs.data.push_back(0); // dokcking_cmd

            pub_localization_.publish(localization_msgs);
            return;
        }
        else
        {
            ROS_INFO_ONCE("Goal is set");
            goal_count_ =2;
            if(behavior_cnt==2 ||  behavior_cnt ==6)
            {
                current_goal_.pose.position.x = config_.Main_start_x_;
                current_goal_.pose.position.y = config_.Main_start_y_;
            }
            else if(behavior_cnt ==0)
            {
                if(new_goal_flag)
                {
                    current_goal_.pose.position.x = goal_set_[0].pose.position.x;
                    current_goal_.pose.position.y = goal_set_[0].pose.position.y;
                }
                else
                {
                    current_goal_.pose.position.x = config_.Main_goal_x_;
                    current_goal_.pose.position.y = config_.Main_goal_y_;
                }
            }
        }


        // 1. Calculate Global Error
        float global_x_err = current_goal_.pose.position.x - pose_msg.pose.pose.position.x;
        float global_y_err = current_goal_.pose.position.y - pose_msg.pose.pose.position.y;
        float global_dist_err = sqrt(global_x_err*global_x_err + global_y_err*global_y_err);
        float global_ang_err;
        float static_x_err,static_y_err;
        std::cout << "goal (x,y): " <<"(" <<current_goal_.pose.position.x << ", " <<current_goal_.pose.position.y << ")" <<std::endl;
        std::cout << "curr (x,y): " <<"(" <<pose_msg.pose.pose.position.x << ", " << pose_msg.pose.pose.position.y << ")" <<std::endl;
        std::cout << "distance :" << global_dist_err <<std::endl;
        std::cout << "tunnel pose: " << tunnel_pose << std::endl;

        //std::cout <<" " <<std::endl;


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

        double roll, pitch, yaw; //double
        m.getRPY(roll, pitch, yaw);
        g_x_err= global_x_err;
        g_y_err= global_y_err;

        g_ctheta = (float) yaw;//TODO for straight control, have to change atan2
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
        line_y_pose = -pose_msg.pose.pose.position.x *sin(g_rtheta) + pose_msg.pose.pose.position.y *cos(g_rtheta); //rtheta_global theta rotation matrix
        //std::cout<<"position amcl: "<< pose_msg.pose.pose.position.x<< ", "<<pose_msg.pose.pose.position.y <<", " << yaw<<std::endl;
        //std::cout<<"position test: "<< pose_msg.pose.pose.position.x<< ", "<<pose_msg.pose.pose.position.y <<", " << asin(pose_msg.pose.pose.orientation.z)*2<<std::endl;
        //----------------------------------------------- current // is_rotating is essential
        //HJ_mode

        std_msgs::Int32 QR_msg;

        if(config_.HJ_MODE_==0){
            switch(HJ_mode_low) //To decide what the mobile robot does
            {
            case AUTO_LIDAR_MODE:
                init_start_ = false;
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
                init_start_ = false;
                if(STOP_cnt<STOP_MAX)
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
                        static_x = pose_msg.pose.pose.position.x;
                        static_y = pose_msg.pose.pose.position.y;
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
                    static_x_err = static_x - pose_msg.pose.pose.position.x;
                    static_y_err = static_y - pose_msg.pose.pose.position.y;
                    static_t = goal_yaw;
                    static_ct = yaw;
                }

                break;

            case DOCK_IN_MODE:
                postech_mode = DOCK_IN_MODE;//moving
                init_start_ = false;
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
                init_start_ = false;
                is_rotating_ = false;
                STOP_cnt =0;
                break;


            default :
                is_rotating_ = false;
                init_start_ = false;
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
                mobile_direction =1;
                if(fabs(current_goal_.pose.position.x - tunnel_pose) < config_.global_dist_boundary_){
                    Next_step = true;
                    ROS_INFO("Tunnel POse STOP !!!!");
                }
                if(Next_step||global_dist_err < config_.global_dist_boundary_ || (fid_area >=QR_DISTANCE && fid_ID == QR_end))
                {
                    Next_step=false;
                    behavior_cnt++;
                    postech_mode = STOP_MODE;
                }
                break;

            case 1://turn : QR or goal pose
                if(STOP_cnt<STOP_MAX)
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
                        static_x = pose_msg.pose.pose.position.x;
                        static_y = pose_msg.pose.pose.position.y;
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
                    if(fabs(encoder_angle - 3.05) < config_.global_angle_boundary_){
                        Next_step = true;
                        ROS_INFO("Tunnel POse STOP !!!!");
                    }
                    if(Next_step||abs(global_ang_err) < config_.global_angle_boundary_) // Ending turn
                    {
                        Next_step=false;
                        behavior_cnt++;
                        postech_mode = STOP_MODE; //TODO depending on what we're gonna use the sensor (change to publish, rotating done)

                        is_rotating_ =false;
                        //                        goal_index_++;
                        STOP_cnt=0;
                    }
                    static_x_err = static_x - pose_msg.pose.pose.position.x;
                    static_y_err = static_y - pose_msg.pose.pose.position.y;
                    static_t = goal_yaw;
                    static_ct = yaw;
                }
                break;

            case 2: //moving back to home
                postech_mode = AUTO_LIDAR_MODE;
                mobile_direction  = -1;
                if(fabs(current_goal_.pose.position.x - tunnel_pose) < config_.global_dist_boundary_){
                    Next_step = true;
                    ROS_INFO("Tunnel POse STOP !!!!");
                }
                if(Next_step||global_dist_err < config_.global_dist_boundary_ || (fid_area >=QR_DISTANCE && fid_ID == QR_home) )
                /*if(fid_area >=QR_DISTANCE && fid_ID == QR_home)
                {
                    if(home_arrival_flag)
                    {
                        home_arrival_flag =false;
                        current_tunnel_pose =tunnel_pose;
                    }
                    if(fabs(current_tunnel_pose - tunnel_pose) > TorotateAtHome)
                        Next_step = true;
                }

                if(Next_step)
*/
                {
                    home_arrival_flag = true;
                    Next_step=false;
                    behavior_cnt++;
                    postech_mode = STOP_MODE;
                }
                break;

            case 3://stop -> turn : QR or goal pose
                if(STOP_cnt<STOP_MAX)
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
                        static_x = pose_msg.pose.pose.position.x;
                        static_y = pose_msg.pose.pose.position.y;
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
                    if(fabs(encoder_angle - 3.05) < config_.global_angle_boundary_){
                        Next_step = true;
                        ROS_INFO("Tunnel POse STOP !!!!");
                    }
                    if(Next_step||abs(global_ang_err) < config_.global_angle_boundary_) // Ending turn
                    {
                        Next_step=false;
                        behavior_cnt++;
                        postech_mode = STOP_MODE;

                        is_rotating_ =false;
                        STOP_cnt =0;
                    }
                    static_x_err = static_x - pose_msg.pose.pose.position.x;
                    static_y_err = static_y - pose_msg.pose.pose.position.y;
                    static_t = goal_yaw;
                    static_ct = yaw;
                }
                break;

            case 4:// docking in :
                postech_mode = DOCK_IN_MODE;//moving
		mobile_direction = 1;
                if(Joy_mode==false)
                {
                    mode_dockin.data = 4;
                    pub_mode_call_.publish(mode_dockin);
                }
                else if(Joy_mode==true) //To stop dock in node
                {
                    mode_dockin.data = 0;
                    pub_mode_call_.publish(mode_dockin);
                }

                if(Next_step||fid_area >=QR_DISTANCE && fid_ID == 104) //TODO
                {
                    mode_dockin.data = 0;
                    pub_mode_call_.publish(mode_dockin);//To stop dock in.
                    Next_step =false;
                    behavior_cnt++;
                    postech_mode = STOP_MODE;
                }
                break;

            case 5://wait : before resservice call
                postech_mode = STOP_MODE;
                if(Next_step||Charging_done_flag)//rosserive call
                {
                    Next_step=false;
                    behavior_cnt++;
                    postech_mode = STOP_MODE;

                    Charging_done_flag =0;
                }
                break;

            case 6://dokcing_out : rostopic pub ending_charging
		//mobile_direction =1;
		if(docking_out_flag)
		{
			docking_out_flag = false;
			init_start_=true;
			tunnel_pose = 0;
                }
		else
		{
			init_start_=false;
		}
		postech_mode = DOCK_OUT_MODE;
/*              camera_on_cnt++;
                if(camera_on_cnt>20)
                {
                    postech_mode = MANUAL_MODE;
                    if(Joy_mode==false)
                    {
                        //only camera dock out
                        mode_dockin.data = 5;
                        pub_mode_call_.publish(mode_dockin);
                    }
                    else if(Joy_mode==true) //To stop dock in node
                    {
                        mode_dockin.data = 0;
                        pub_mode_call_.publish(mode_dockin);
                    }
                }
                else
                {
                    std::cout<<  "RP Docking out: " <<std::endl;
                    postech_mode = DOCK_OUT_MODE;
                }
*/

                if(fabs(current_goal_.pose.position.x - tunnel_pose) < config_.global_dist_boundary_){
                    Next_step = true;
                    ROS_INFO("Tunnel POse STOP !!!!");
                }
                if(Next_step|| global_dist_err<config_.global_dist_boundary_ || fid_area >=QR_DISTANCE && fid_ID == QR_reset)// To use AMCL pose probably
                {
                    mode_dockin.data = 0;
                    pub_mode_call_.publish(mode_dockin);//To stop dock in.
                    Next_step=false;
                    behavior_cnt=0;
                    postech_mode = STOP_MODE;
                    //init_start_ = true;
		    docking_out_flag = true;
                    //goal_index_ ++;
                    camera_on_cnt=0;
                }
                break;
            default:
                //behavior_cnt=0;
                postech_mode = STOP_MODE;
                ROS_INFO("Behavior error: rostopic pub /Mode_Decision std_msgs/Int32 data: #number // ex) data: 0");
                break;
            }
            ROS_INFO("Behavior count: %d", behavior_cnt);

        }
        //-----------------
        else if(config_.HJ_MODE_ ==2){ //Without docking postech mode
            switch(behavior_cnt) // Behave in turn
            {
            case 0://moving to go turning point: start straight
                init_start_ = false;
                postech_mode = AUTO_LIDAR_MODE;
                if(Next_step||global_dist_err < config_.global_dist_boundary_ || (fid_area >=5000 && fid_ID == 1))
                {
                    Next_step=false;
                    behavior_cnt++;
                    postech_mode = STOP_MODE;
                }
                break;

            case 1://turn : QR or goal pose
                if(STOP_cnt<STOP_MAX)
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
                        static_x = pose_msg.pose.pose.position.x;
                        static_y = pose_msg.pose.pose.position.y;
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
                    if(Next_step||abs(global_ang_err) < config_.global_angle_boundary_) // Ending turn
                    {
                        Next_step=false;
                        behavior_cnt++;
                        postech_mode = STOP_MODE; //TODO depending on what we're gonna use the sensor (change to publish, rotating done)

                        is_rotating_ =false;
                        //goal_index_++;
                        STOP_cnt=0;
                    }
                    static_x_err = static_x - pose_msg.pose.pose.position.x;
                    static_y_err = static_y - pose_msg.pose.pose.position.y;
                    static_t = goal_yaw;
                    static_ct = yaw;
                }
                break;

            case 2: //moving back to home
                postech_mode = AUTO_LIDAR_MODE;

                if(Next_step||global_dist_err < config_.global_dist_boundary_ || (fid_area >=5000 && fid_ID == 2) )
                {
                    Next_step=false;
                    behavior_cnt++;
                    postech_mode = STOP_MODE;
                }
                break;

            case 3://stop -> turn : QR or goal pose
                if(STOP_cnt<STOP_MAX)
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
                        static_x = pose_msg.pose.pose.position.x;
                        static_y = pose_msg.pose.pose.position.y;
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

                    if(Next_step||abs(global_ang_err) < config_.global_angle_boundary_) // Ending turn
                    {
                        Next_step=false;
                        behavior_cnt++;
                        postech_mode = STOP_MODE;

                        is_rotating_ =false;
                        STOP_cnt =0;

                        //Without docking & no QR
                        if(config_.Without_QR_move_)
                        {
                            behavior_cnt=0;
                            postech_mode = STOP_MODE;
                        }
                    }
                    static_x_err = static_x - pose_msg.pose.pose.position.x;
                    static_y_err = static_y - pose_msg.pose.pose.position.y;
                    static_t = goal_yaw;
                    static_ct = yaw;
                }
                break;

            case 4:// docking in :
                postech_mode = DOCK_OUT_MODE;//moving

                if(Next_step||fid_area >=5000 && fid_ID == 4)
                {
                    Next_step=false;
                    behavior_cnt=0;
                    init_start_ =true;
                    postech_mode = STOP_MODE;
                    ROS_INFO("fid_area: %f", fid_area);
                }
                break;
            default:
                postech_mode = STOP_MODE;
                ROS_INFO("Behavior error: rostopic pub /Mode_Decision std_msgs/Int32 data: #number // ex) data: 0");
                break;
            }
            ROS_INFO("Behavior count: %d", behavior_cnt);
        }
        //-------------------NEW for auto pre lidar mode only in postech code
        else if(config_.HJ_MODE_==3){ //With docking postech mode
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
                if(STOP_cnt<STOP_MAX)
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
                        static_x = pose_msg.pose.pose.position.x;
                        static_y = pose_msg.pose.pose.position.y;
                        goal_yaw = yaw + M_PI; // save current when start rotating
                        if(goal_yaw > M_PI)
                            goal_yaw -= 2*M_PI;
                        else if(goal_yaw < -M_PI)
                            goal_yaw += 2*M_PI;
                    }

                    global_ang_err = goal_yaw - yaw;
                    if(global_ang_err > M_PI)
                        global_ang_err -= 2*M_PI;
                    else if(global_ang_err < -M_PI)
                        global_ang_err += 2*M_PI;
                    std::cout<<  "start yaw: " << goal_yaw  <<", cur yaw: " << yaw<< ", yaw err:"<<global_ang_err<<std::endl;
                    if(abs(global_ang_err) < config_.global_angle_boundary_) // Ending turn
                    {
                        //behavior_cnt++;
                        postech_mode = AUTO_PRE_LIDAR_MODE; //TODO depending on what we're gonna use the sensor (change to publish, rotating done)
                        ROS_INFO("Mobile align");
                    }
                    static_x_err = static_x - pose_msg.pose.pose.position.x;
                    static_y_err = static_y - pose_msg.pose.pose.position.y;
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
                if(STOP_cnt<STOP_MAX)
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
                        static_x = pose_msg.pose.pose.position.x;
                        static_y = pose_msg.pose.pose.position.y;
                        goal_yaw = yaw + M_PI; // save current when start rotating
                        if(goal_yaw > M_PI)
                            goal_yaw -= 2*M_PI;
                        else if(goal_yaw < -M_PI)
                            goal_yaw += 2*M_PI;
                    }

                    global_ang_err = goal_yaw - yaw;
                    if(global_ang_err > M_PI)
                        global_ang_err -= 2*M_PI;
                    else if(global_ang_err < -M_PI)
                        global_ang_err += 2*M_PI;
                    std::cout<<  "start yaw: " << goal_yaw  <<", cur yaw: " << yaw<< ", yaw err:"<<global_ang_err<<std::endl;
                    if(abs(global_ang_err) < config_.global_angle_boundary_) // Ending turn
                    {
                        //behavior_cnt++;
                        postech_mode = AUTO_PRE_LIDAR_MODE;
                        ROS_INFO("Mobile align");

                    }
                    static_x_err = static_x - pose_msg.pose.pose.position.x;
                    static_y_err = static_y - pose_msg.pose.pose.position.y;
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
                }
                break;
            default:
                //behavior_cnt=0;
                postech_mode = STOP_MODE;
                ROS_INFO("Behavior error");
                break;
            }
            ROS_INFO("Behavior count: %d", behavior_cnt);
        }

        //-----------------------------------------------

        float send_x, send_y, send_ref_yaw, send_yaw;
        if(postech_mode ==TURN_MODE)
        {
            send_x = static_x_err;
            send_y = static_y_err;
            send_ref_yaw = static_t;
            send_yaw = static_ct;
        }
        else
        {
            send_x = global_x_err;
            send_y = global_y_err;
            send_ref_yaw = g_rtheta;
            send_yaw = g_ctheta;
        }

        localization_msgs.data.push_back(postech_mode);

        localization_msgs.data.push_back(send_x);
        localization_msgs.data.push_back(send_y);
        localization_msgs.data.push_back(send_ref_yaw);
        localization_msgs.data.push_back(send_yaw);

        localization_msgs.data.push_back(init_start_);

        localization_msgs.data.push_back(Docking_out_cmd);

        pub_localization_.publish(localization_msgs);
        fid_area = 0; // To reset for the fid_area. because if the ID is not detected the previous data is still in.
        std::cout<< "\n\n"<<std::endl;

        leo_driving::PlotMsg Save_log;
        Save_log.x = pose_msg.pose.pose.position.x;
        Save_log.y = pose_msg.pose.pose.position.y;
        if(postech_mode==TURN_MODE)
        {
            std::cout <<"static x: " << static_x_err *cos(static_ct) + static_y_err *sin(static_ct)<< ", static y: "<<  -static_x_err *sin(static_ct) + static_y_err *cos(static_ct) << std::endl;
            Save_log.r_x = static_x;
            Save_log.r_y = static_y;
            Save_log.s_x = static_x_err *cos(static_ct) + static_y_err *sin(static_ct);
            Save_log.s_y = -static_x_err *sin(static_ct) + static_y_err *cos(static_ct);
            Save_log.rt = goal_yaw;
            Save_log.ct = yaw;
        }
        else
        {
            Save_log.r_x = current_goal_.pose.position.x;
            Save_log.r_y = current_goal_.pose.position.y;
        }


        pub_log_data_.publish(Save_log);


    }
    void SetQRPose(int id){//pcw for QR local
        //just for test have to modify to real value
        //if(id <100)
        if(id==1 || id ==2)
            tunnel_pose = id*100;
    }

    //Previous thoughts
    void DecisionpublishCmd(const std_msgs::Int32::ConstPtr &mode_call);
    void QRtestCallback(const std_msgs::Float32::ConstPtr& QR_flag_msgs);
    void OdomCallback(const nav_msgs::Odometry::ConstPtr& odom_msgs);

    void areaDataCallback(const std_msgs::Float32MultiArray::ConstPtr& area_msgs);
    //current thoughts
    bool docking_done(std_srvs::Empty::Request &req, std_srvs::Empty::Response &resp);
    void ModedecisionCallback(const std_msgs::Int32::ConstPtr &msg_cnt);

    void predoneCallback(const std_msgs::Empty::ConstPtr &msg_empty);


private:
    ros::Subscriber sub_joy_;
    ros::Subscriber sub_pose_;
    ros::Subscriber sub_pose_driving_;
    ros::Subscriber sub_goal_;
    ros::Subscriber sub_front_camera;


    ros::Subscriber sub_area_;
    ros::Subscriber sub_mode_;
    ros::Subscriber sub_QRinit_;
    ros::Subscriber sub_odom_;
    ros::Subscriber sub_predone_;
    ros::Subscriber sub_mode_decision_;
    ros::Subscriber sub_pose_dt_from_linearvel; //pcw for QR local

    ros::ServiceServer sub_docking_done_;
    ros::Publisher pub_localization_;
    ros::Publisher pub_robot_pose_;
    ros::Publisher pub_QR_;
    ros::Publisher pub_log_data_;
    ros::Publisher pub_mode_call_;
    ros::Publisher pub_for_test_QR_local;//pcw for QR local

    bool is_rotating_ = false;
    bool init_start_ = false; //To initial odom and IMU
    bool g_rtheta_flag =true;
    bool FIRST_START_FLAG =true; //To set a behavior_cnt at first
    bool new_goal_flag =false;//For a event goal

    bool Joy_mode= false;
    // GOAL
    unsigned int start_wait=0;
    int goal_index_ = 0;
    int goal_count_ = 0;
    float goal_yaw = M_PI;
    float static_x=0.0, static_y=0.0,static_t=0.0,static_ct=0.0;
    float g_x_err,g_y_err, g_rtheta,g_ctheta;
    int postech_mode;
    float line_y_pose = 0;//not necessary

    //Cmaera
    int fid_ID=0;
    float fid_area=0;

    //Linear vel pose from roverroboics_ros_drier
    //pcw for QR local
    double dt_x_by_linearvel = 0;
    double dt_y_by_linearvel = 0;
    double pos_x_by_linearvel = 0;
    double pos_y_by_linearvel = 0;
    //

    //Decision
    unsigned int behavior_cnt =6;
    bool Charging_done_flag =false;
    int HJ_mode_low=STOP_MODE;
    bool Next_step =false;

    unsigned int STOP_cnt =0;

    //To recognize only one press for joystick
    unsigned int switch_flag0 =0;
    unsigned int switch_flag1 =0;

    double past_time=0;
    double tunnel_pose =0, current_tunnel_pose =0;
    int mobile_direction =1;

    bool home_arrival_flag =true;
    bool docking_out_flag = true;
    float Docking_out_cmd =0;

    double encoder_angle = 0;
    double encoder_angular = 0;

    unsigned int camera_on_cnt =0;

    std::vector<geometry_msgs::PoseStamped> goal_set_;
    geometry_msgs::PoseStamped current_goal_;

    geometry_msgs::PoseWithCovarianceStamped current_pose;

    /** configuration parameters */
    typedef struct
    {
        double global_dist_boundary_;
        double global_angle_boundary_;
        int HJ_MODE_;
        bool Without_QR_move_;
        double Main_start_x_;
        double Main_start_y_;
        double Main_goal_x_;
        double Main_goal_y_;
    } Config;
    Config config_;


};
void LocalizationNode::areaDataCallback(const std_msgs::Float32MultiArray::ConstPtr& area_msgs) //QR detection Aurco realsense for a front camera.
{
    fid_ID = (int) area_msgs->data[0];
    fid_area =area_msgs->data[1];
    //ROS_INFO("fid_ID: %d, are: %f",fid_ID, fid_area);

    SetQRPose(fid_ID);
}
void LocalizationNode::DecisionpublishCmd(const std_msgs::Int32::ConstPtr &mode_call)
{
    HJ_mode_low = mode_call->data;
    ROS_INFO("Mode: %d",HJ_mode_low);
}
void LocalizationNode::ModedecisionCallback(const std_msgs::Int32::ConstPtr &msg_cnt)//To jump the behavior of user
{
    behavior_cnt = msg_cnt->data;
}
void LocalizationNode::predoneCallback(const std_msgs::Empty::ConstPtr &msg_empty)
{
    if(behavior_cnt==1)
    {
        behavior_cnt =2; //The pre lidar is finished; thus next step
        postech_mode = STOP_MODE;

        is_rotating_ =false; //Those two parameters for waiting before rotating
        STOP_cnt =0;
    }
    else if (behavior_cnt==3)
    {
        behavior_cnt =4; //The pre lidar is finished; thus next step
        postech_mode = STOP_MODE;

        is_rotating_ =false;
        STOP_cnt =0;
    }
}

bool LocalizationNode::docking_done(std_srvs::Empty::Request &req, std_srvs::Empty::Response &resp)
{
    Charging_done_flag = true;
    return true;
}
void LocalizationNode::QRtestCallback(const std_msgs::Float32::ConstPtr& QR_flag_msgs)
{
    if(QR_flag_msgs->data ==0)
    {
        config_.Without_QR_move_ = false;
        ROS_INFO("When HJ_MODE==2, with QR initialzation");
    }
    else if(QR_flag_msgs->data ==1)
    {
        config_.Without_QR_move_ = true;
        ROS_INFO("When HJ_MODE==2, without QR initialzation");
    }
}
void LocalizationNode::OdomCallback(const nav_msgs::Odometry::ConstPtr& odom_msgs)
{
    double dt = 0;
    ros::Time ros_now_time = ros::Time::now();
    double now_time = ros_now_time.toSec();
    //tf2_ros::TransformBroadcaster odom_broadcaster;

    dt = now_time - past_time;
    past_time = now_time;

    tunnel_pose += mobile_direction*odom_msgs->twist.twist.linear.x * dt ;
    encoder_angular = odom_msgs->twist.twist.angular.z;
    if(behavior_cnt == 1 || behavior_cnt == 3){
        encoder_angle += encoder_angular;
    }
    else{
        encoder_angle = 0;
    }
    geometry_msgs::Point tunnel_pos_pub;
    tunnel_pos_pub.x= tunnel_pose;
    tunnel_pos_pub.z = odom_msgs->twist.twist.angular.z;
    pub_for_test_QR_local.publish(tunnel_pos_pub);
}
}
PLUGINLIB_EXPORT_CLASS(auto_driving::LocalizationNode, nodelet::Nodelet);

