
#include <laser_geometry/laser_geometry.h>
#include "ros/ros.h"
#include "nodelet/nodelet.h"
#include <pluginlib/class_list_macros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Float32.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/filters/conditional_removal.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <tf/transform_listener.h>

//CMD
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int32.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>

#include <algorithm>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
// #include <leo_driving/Mode.h>

#define INIT_WAIT 10
#define RATIO_VEL 1.5

using namespace message_filters;
using namespace std;

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

class AisleDetectNode : public nodelet::Nodelet {

public:
    AisleDetectNode() = default;

private:
    virtual void onInit() {
        ros::NodeHandle nh = getNodeHandle();
        ros::NodeHandle nhp = getPrivateNodeHandle();
        
        // Configuration //
        nhp.param("line_thresh", config_.line_thresh_, 0.5);
        nhp.param("aisle_width", config_.aisle_width_, 0.6);

        //cmd
        nhp.param("Kpx_param", config_.Kpx_param_, 2.0);
        nhp.param("Kpy_param", config_.Kpy_param_, 1.1);
        nhp.param("Kpy_param_rot", config_.Kpy_param_rot_, 0.01);
        nhp.param("Kpy_param_boundary_gain", config_.Kpy_param_boundary_gain_, 0.05);
        nhp.param("theta_ratio", config_.theta_ratio_, 0.8);
        nhp.param("tunnel_gain_boundary", config_.tunnel_gain_boundary_, 0.01);
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
        nhp.param("Postech_code", config_.Postech_code_, false);
        nhp.param("obs_avoidance_distance", config_.obs_avoidance_distance_, 0.1);

        // Subscriber & Publisher
        sub_scan_ = nhp.subscribe("/rp/scan", 20, &AisleDetectNode::scanCallback, this);

        pub_line_ = nhp.advertise<sensor_msgs::PointCloud2>("/cluster_line", 10);
        pub_points_ = nhp.advertise<sensor_msgs::PointCloud2> ("/aisle_points", 10);

        pub_prelidar_fail_ = nhp.advertise<std_msgs::Empty> ("/auto_pre_lidar_mode/fail", 10);
        pub_lidar_fail_ = nhp.advertise<std_msgs::Empty> ("/auto_pre_lidar_mode/fail", 10);


        //cmd
        sub_joy_ = nhp.subscribe<sensor_msgs::Joy>("/joystick", 1, &AisleDetectNode::joyCallback, this);
        sub_localization_ = nhp.subscribe<std_msgs::Float32MultiArray> ("/localization_data", 10, &AisleDetectNode::localDataCallback, this);
        sub_mode_call_ = nhp.subscribe<std_msgs::Int32> ("/mode/low", 10, &AisleDetectNode::modeCallback, this);
        sub_speed_ = nhp.subscribe("/mission/setspeed", 1, &AisleDetectNode::SpeedCallback, this);


        pub_cmd_ = nhp.advertise<geometry_msgs::Twist> ("/cmd_vel", 20);
        pub_docking_end_ = nhp.advertise<std_msgs::Int32> ("/joy_from_cmd", 1); // Temperary To finish with joystick
        pub_prelidar_end_ = nhp.advertise<std_msgs::Empty> ("/auto_pre_lidar_mode/end", 10);
    };

    void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan_msg)
    {
        // double dt = 0;
        // ros::Time ros_now_time = ros::Time::now();
        // double now_time = ros_now_time.toSec();

        

        std_msgs::Empty EmptyMsg;
        // 1. Data type conversions (laser scan -> pointcloud2)
        laser_geometry::LaserProjection projector;
        sensor_msgs::PointCloud2 cloud_msg;
        projector.projectLaser(*scan_msg, cloud_msg);

        pcl::PCLPointCloud2::Ptr temp_cloud (new pcl::PCLPointCloud2);
        pcl_conversions::toPCL(cloud_msg, *temp_cloud); // save cloud message to cloud2
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromPCLPointCloud2(*temp_cloud, *cloud);

        // 2. Crop Point Cloud
        pcl::ConditionAnd<pcl::PointXYZ>::Ptr range_condition(new pcl::ConditionAnd<pcl::PointXYZ> ());
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_inrange(new pcl::PointCloud<pcl::PointXYZ>); // <- cropped cloud
        // set condition
        range_condition->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZ>("y", pcl::ComparisonOps::GT, -config_.aisle_width_)));
        range_condition->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZ>("y", pcl::ComparisonOps::LT, config_.aisle_width_)));
        range_condition->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZ>("x", pcl::ComparisonOps::LT, 0.0)));
        // conditional removal
        pcl::ConditionalRemoval<pcl::PointXYZ> condrem;
        condrem.setInputCloud(cloud);
        condrem.setCondition(range_condition);
        condrem.setKeepOrganized(true);
        condrem.filter(*cloud_inrange);
        if (cloud_inrange->size() == 0)
        {
            this->pub_lidar_fail_.publish(EmptyMsg); //TODO FAIL Pub
            this->pub_prelidar_fail_.publish(EmptyMsg); //TODO FAIL Pub
            ROS_WARN("all points are cropped");
            return;
        }

        // 3. EXTRACT LINE (RANSAC ALGORITHM)
        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
        pcl::SACSegmentation<pcl::PointXYZ> seg;
        pcl::ExtractIndices<pcl::PointXYZ> extract;
        seg.setOptimizeCoefficients(true);
        seg.setModelType(pcl::SACMODEL_LINE); // <- extract model setting
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setDistanceThreshold(config_.line_thresh_); // <- threshold (line width) // 0.5
        seg.setInputCloud(cloud_inrange);
        seg.segment(*inliers, *coefficients);
        extract.setInputCloud(cloud_inrange);
        extract.setIndices(inliers);
        extract.setNegative(false); //<- if true, it returns point cloud except the line.
        extract.filter(*cloud_inrange);

        // 4. Extract Line Cluster
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree_cluster(new pcl::search::KdTree<pcl::PointXYZ>);
        tree_cluster->setInputCloud(cloud_inrange);
        std::vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
        ec.setInputCloud(cloud_inrange);
        ec.setClusterTolerance(0.05); // <- If the two points have distance bigger than this tolerance, then points go to different clusters.
        ec.setMinClusterSize(30);
        ec.setMaxClusterSize(800);;
        ec.setSearchMethod(tree_cluster);
        ec.extract(cluster_indices);

        // extract first clustering (center cluster)
        int j = 0;
        std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin ();
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
        for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
        {
            if (j == 0) {
                for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
                {
                    cloud_cluster->points.push_back (cloud_inrange->points[*pit]);
                }
                cloud_cluster->width = cloud_cluster->points.size ();
                cloud_cluster->height = 1;
                cloud_cluster->is_dense = true;
            }
            j++;
        }
        if((*cloud_cluster).size() == 0)
        {
            this->pub_lidar_fail_.publish(EmptyMsg); //TODO FAIL Pub
            this->pub_prelidar_fail_.publish(EmptyMsg); //TODO FAIL Pub
            ROS_WARN("Not enough points!");
            return;
        }

        // 5. FIND NEAREST POINT FROM THE ORIGIN ( => /nearest_point)
        pcl::PointCloud<pcl::PointXYZ> point_set; // Data to be published

        pcl::PointXYZ origin(0, 0, 0);
        pcl::KdTree<pcl::PointXYZ>::Ptr tree_(new pcl::KdTreeFLANN<pcl::PointXYZ>);
        tree_->setInputCloud(cloud_cluster);
        std::vector<int> nn_indices(1);
        std::vector<float> nn_dists(1);
        tree_->nearestKSearch(origin, 1, nn_indices, nn_dists); //<- finds the most closest sing point: save points index to "nn_indices", and distance to "nn_dists"
        point_set.push_back(cloud_cluster->points[nn_indices[0]]); // [0]: closest

        // 6. Calculate the Reference point
        float sum_x = 0;
        int num_points = 0;

        // Update Line min & Line max
        float line_start_y = 0;
        float line_end_y = 1000;
        pcl::PointCloud<pcl::PointXYZ> line_cloud;

        for (int i = 0; i < (*cloud_cluster).size(); i++)
        {
            sum_x += cloud_cluster->points[i].x;
            num_points ++;
            if (line_end_y > cloud_cluster->points[i].y)
                line_end_y = cloud_cluster->points[i].y;
            if (line_start_y < cloud_cluster->points[i].y)
                line_start_y = cloud_cluster->points[i].y;
        }
        pcl::PointXYZ reference (sum_x / (float)num_points, (line_start_y + line_end_y)/2, 0);//Jinsuk
        point_set.push_back(reference); // [1]: reference

        pcl::PointXYZ line_start_point (sum_x / (float)num_points, line_start_y, 0);
        pcl::PointXYZ line_end_point (sum_x / (float)num_points, line_end_y, 0);
        point_set.push_back(line_start_point); //[2]: line start point
        point_set.push_back(line_end_point); //[3]: line end point



        // Publish ROS Topics
        sensor_msgs::PointCloud2 points_msg;
        sensor_msgs::PointCloud2 points_line;

        pcl::toROSMsg((*cloud_cluster), points_line);
        pcl::toROSMsg(point_set, points_msg);
        points_line.header.frame_id = scan_msg->header.frame_id;
        points_msg.header.frame_id = scan_msg->header.frame_id;
        this->pub_points_.publish(points_msg);
        this->pub_line_.publish(points_line);

        //cmd
        near_y_ = cloud_cluster->points[nn_indices[0]].y;
        ref_y_ = reference.y;
        ref_x_ = reference.x;
        line_start_y_ = line_start_point.y;
        line_end_y_ = line_end_point.y;
        geometry_msgs::Twist cmd_vel;
        int Mode_type;
        HJ_mode_cnt++;

        //cout<<"1. reference = "<<reference<<endl;
        //cout<<"2. near_point_y = "<<near_y_<<endl;


        //To stop when the communication is delayed or failed
        if(config_.Postech_code_)
            Mode_type  = postech_mode_;
        else
        {
            if(HJ_mode_cnt >=10) //If there is no mode subscribe, the mode stops. It should be deleted
                HJ_mode_low = STOP_MODE;
            Mode_type = HJ_mode_low;
        }
        //To interrupt mode_type, program here->.
        //To operate the gmapping
        if(config_.amcl_driving_ ==false)
        {
            Mode_type = AUTO_LIDAR_MODE;
            if(!gmapping_go)
                return;
        }
        //To operate manual mode
        if(joy_driving_ || HJ_mode_low == MANUAL_MODE)
            Mode_type = MANUAL_MODE;


        //std::cout<<"Mode_type: "<<Mode_type<<std::endl;
        if(init_call || init_cnt !=0)
        {
            init_call = false; //Without this, it keeps initializing due to delayed subscribe from loclizaiton node.
            init_cnt++;
            cmd_vel.linear.x = 0.0;
            cmd_vel.linear.z = 0.0;
            pub_cmd_.publish(cmd_vel);
            if(init_cnt==INIT_WAIT)
            {
                //system("rosservice call /odom_init 0.0 0.0 0.0"); //Intialize Encoder
                //system("rosservice call /reset_odom"); //Intialize IMU
                system("rosservice call /pose_update 0.0 0.0 0.0"); //Intialize AMCL
            }
            else if(init_cnt==INIT_WAIT*3)
            {
                system("rosservice call /reset_odom"); //Intialize IMU
            }
            else if(init_cnt==INIT_WAIT*6)
            {
                system("rosservice call /odom_init 0.0 0.0 0.0"); //Intialize Encoder
            }
            if(init_cnt==INIT_WAIT*12)
                init_cnt =0;
            return;
        }

        //// 2. Autonomous Driving

        float y_err_local = ref_y_ - near_y_;
        float line_length_obs = copysign(fabs(line_start_y_ - line_end_y_),y_err_local);
        float y_err_theta = config_.theta_ratio_*atan2(y_err_local,0.2) + (1-config_.theta_ratio_)*atan2(line_length_obs,0.8);
        // std::cout<< "y_err_local" <<y_err_local<<std::endl;
        // std::cout<< "line_length_obs" <<line_length_obs<<std::endl;
        cmd_vel.linear.x = config_.linear_vel_;
	    double temp_angular_max;
        //std::cout<<"---------------------------ais: " << y_err_local<<std::endl;
        // 2.1 Check Obstacles

        if (config_.check_obstacles_ && (Mode_type == AUTO_LIDAR_MODE || Mode_type == AUTO_IMAGE_MODE))
        {
            float line_length = line_end_y_ - line_start_y_;
            float left_boundary = line_start_y_ - (line_length * config_.boundary_percent_ + 0.5 * config_.robot_width_);
            float right_boundary = line_end_y_ + (line_length * config_.boundary_percent_ + 0.5 * config_.robot_width_);
            bool is_obs_in_aisle = obs_y_ > line_end_y_ && obs_y_ < line_start_y_;
            // (0) Front Obstacle Update

            //std::cout << "Obs_x : " << obs_x_ << ", ref_x : " <<  ref_x_<<std::endl;

            if (obs_x_ < config_.front_obs_ && abs(obs_y_) < config_.robot_width_/4 )
            {
                cmd_vel.linear.x = 0.0;
                cmd_vel.linear.z = 0.0;
                pub_cmd_.publish(cmd_vel);
                std::cout<<"Front obstacle is deteced"<<std::endl;
                return;
            } // below  else if

            // (1) Right Obstacle Update	(y:오른쪽이 음수)
            else if(obs_y_ < 0 && obs_y_ > -1 && obs_x_< 1)
            {
                cmd_vel.linear.x =0.2;
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
            else if(obs_y_ > 0 && obs_y_ < 1 && obs_x_< 1)
            {
                cmd_vel.linear.x =0.2;
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
                cmd_vel.linear.x =0.2;
                spare_length += cmd_vel.linear.x * 0.1;
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
                    cmd_vel.linear.x = config_.linear_vel_;
                }
            }

        }


        float straight_l_xerr;

        float l_xerr,l_yerr;
        float tunnel_angle_diff;

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

            //cmd_vel.linear.x = config_.linear_vel_*straight_l_xerr; // To stop slowly when arriving at the point

            //previous controller
//            if(y_err_local<config_.tunnel_gain_boundary_)
//                cmd_vel.angular.z = -config_.Kpy_param_boundary_gain_ * y_err_local -config_.Kpy_param_rot_*(y_err_local - pre_y_err)*10;
//            else
//                cmd_vel.angular.z = -config_.Kpy_param_ * y_err_local -config_.Kpy_param_rot_*(y_err_local - pre_y_err)*10;
//            pre_y_err = y_err_local;

        // if (was_obs_in_aisle){ //for obstacle avoid
        if(1){
            cmd_vel.angular.z = -config_.Kpy_param_ * y_err_local -config_.Kpy_param_rot_*(y_err_local - pre_y_err)*10;
            pre_y_err = y_err_local;
        }
        else{ //for normal driving by theta 

            cmd_vel.angular.z= -config_.Kpy_param_*y_err_theta-config_.Kpy_param_rot_*(y_err_theta - pre_y_err)*10; // rad
            pre_y_err = y_err_theta;
        }
            

		cmd_vel.linear.x = config_.linear_vel_;//(tunnel_angle_diff);

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

		// if(fabs(cmd_vel.angular.z) >RATIO_VEL)
		// 	temp_angular_max = RATIO_VEL;
		// else
		// 	temp_angular_max = fabs(cmd_vel.angular.z);

		// cmd_vel.linear.x = cmd_vel.linear.x*(1- temp_angular_max);

            //std::cout<<"x: "<<cmd_vel.linear.x<< ", z: " << cmd_vel.angular.z<<std::endl;
            //Saturation of 'cmd_vel.linear.x' doesn't need because when the x fits, it's not necessary to move

            pub_cmd_.publish(cmd_vel);
            break;

        case TURN_MODE:
            // if(local_data_receive)
            // {
            //     local_data_receive =false;
                l_xerr= global_x_err_ *cos(global_c_theta_) + global_y_err_ *sin(global_c_theta_);
                l_yerr= -global_x_err_ *sin(global_c_theta_) + global_y_err_ *cos(global_c_theta_);
                //std::cout<< "static_x: " <<l_xerr <<", static_y:" << l_yerr <<std::endl;
                l_xerr = (tan(global_r_theta_)*-global_x_err_ -global_y_err_)/(sqrt(tan(global_r_theta_)*tan(global_r_theta_)+(float)1));//new l_xerr


                cmd_vel.linear.x = config_.rot_kx_;//config_.rot_kx_*l_xerr;
                cmd_vel.angular.z = config_.rot_ky_ *l_yerr + config_.rot_kt_ *(global_r_theta_ - global_c_theta_);

                /*
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
            //}
            break;

        case AUTO_PRE_LIDAR_MODE:
            align_cnt++;
            cmd_vel.linear.x = 0.0;
            cmd_vel.angular.z = -config_.Kpy_param_ * y_err_local;
            if(cmd_vel.angular.z< config_.min_rot_ && cmd_vel.angular.z>0)//To rotate minimum speed at cw
                cmd_vel.angular.z = config_.min_rot_;
            else if(cmd_vel.angular.z> -config_.min_rot_ && cmd_vel.angular.z<0) //To rotate minimum speed at ccw
                cmd_vel.angular.z = -config_.min_rot_;
            if(cmd_vel.angular.z> config_.max_rot_)
                cmd_vel.angular.z = config_.max_rot_;
            else if(cmd_vel.angular.z< -config_.max_rot_)
                cmd_vel.angular.z = -config_.max_rot_;

            pub_cmd_.publish(cmd_vel);

            if(y_err_local < 0.02 || align_cnt >=70 )//0.05 cm
            {
                align_cnt=0;
                pub_prelidar_end_.publish(EmptyMsg);
            }
            break;

        case DOCK_IN_MODE: //Hanjeon gives us the err of y
            //            cmd_vel.linear.x = -0.1;
            //            cmd_vel.angular.z = -config_.Kpy_param_ * y_err_local;

            //            if(cmd_vel.linear.x> config_.max_vel_)
            //                cmd_vel.linear.x = config_.max_vel_;
            //            else if(cmd_vel.linear.x< -config_.max_vel_)
            //                cmd_vel.linear.x = -config_.max_vel_;

            //            if(cmd_vel.angular.z> config_.max_rot_)
            //                cmd_vel.angular.z = config_.max_rot_;
            //            else if(cmd_vel.angular.z< -config_.max_rot_)
            //                cmd_vel.angular.z = -config_.max_rot_;

            //pub_cmd_.publish(cmd_vel);
            break;
        case DOCK_OUT_MODE: // with RPlidar
            cmd_vel.linear.x = 0.1;
            cmd_vel.angular.z = -config_.Kpy_param_ * y_err_local/0.6;

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
        //cout<<"3. cmd_vel = "<<cmd_vel<<endl;
        // ros::Time ros_end_time = ros::Time::now();
        // double end_time = ros_now_time.toSec();
        // dt = end_time - now_time;
        // ROS_INFO("Current, during TIME: %f, %f",now_time,dt);
    }

    void joyCallback(const sensor_msgs::Joy::ConstPtr& joy_msg)
    {
        std_msgs::Int32 joy_msg_to_node;
        //B
        if (joy_msg->buttons[1] == 1 && switch_flag1 ==0)
        {
            switch_flag1 = 1;
            std::cout<<"(push) B "<<std::endl;
            joy_driving_ = !joy_driving_;
            gmapping_go = true; //Keep TRUE

            if(joy_driving_==true)
            {
                ROS_INFO("Joystick mode");
                joy_msg_to_node.data=1;
                pub_docking_end_.publish(joy_msg_to_node);
            }
            else if(joy_driving_==false)
            {
                joy_msg_to_node.data=10;
                pub_docking_end_.publish(joy_msg_to_node);
            }
        }
        else if(joy_msg->buttons[1] == 0 && switch_flag1 ==1)
        {
            switch_flag1 =0;
        }
        //A
        if (joy_msg->buttons[0] == 1 && switch_flag0 ==0)
        {
            switch_flag0 = 1;
            std::cout<<"(push) A "<<std::endl;

            joy_msg_to_node.data=0;
            pub_docking_end_.publish(joy_msg_to_node);
        }
        else if(joy_msg->buttons[0] == 0 && switch_flag0 ==1)
        {
            switch_flag0 =0;
        }
        /*
        if(joy_msg->buttons[3] == 1)	 //
        {
            std::cout<<"(push) X "<<std::endl;
            joy_msg_to_node.data=3;
            pub_docking_end_.publish(joy_msg_to_node);
            ros::Duration(0.5).sleep();
        }*/
        if(joy_msg->buttons[3] == 4)	 //
        {
            std::cout<<"(push) Y "<<std::endl;
            init_call = true;
            //joy_msg_to_node.data=3;
            //pub_docking_end_.publish(joy_msg_to_node);
            ros::Duration(0.5).sleep();
        }
        //Velocity maximum
        // left right
        if ((joy_msg->axes[6] == 1 || joy_msg->axes[6] == -1) && axes_flag[6] ==0)
        {
            axes_flag[6] = 1;
            std::cout<<"(push) axes up "<<std::endl;
            if(joy_msg->axes[6] == 1)
                manual_ratio += 0.1;
            else if(joy_msg->axes[6] == -1)
                manual_ratio -= 0.1;
            if(manual_ratio >= 1.5)
                manual_ratio = 1.5;
            else if(manual_ratio <= 0.5)
                manual_ratio = 0.5;
        }
        else if(joy_msg->axes[6] == 0 && axes_flag[6] ==1)
        {
            axes_flag[6] =0;
        }

        // up down
        if ((joy_msg->axes[7] == 1 || joy_msg->axes[7] == -1) && axes_flag[7] ==0)
        {
            axes_flag[7] = 1;
            std::cout<<"(push) axes up "<<std::endl;
            if(joy_msg->axes[7] == 1)
                config_.linear_vel_ += 0.1;
            else if(joy_msg->axes[7] == -1)
                config_.linear_vel_ -= 0.1;
            if(config_.linear_vel_ >= 1.5)
                config_.linear_vel_ = 1.5;
            else if(config_.linear_vel_ <= 0.2)
                config_.linear_vel_ = 0.2;
        }
        else if(joy_msg->axes[7] == 0 && axes_flag[7] ==1)
        {
            axes_flag[7] =0;
        }

        if(joy_driving_)
        {
            geometry_msgs::Twist cmd_vel;
            cmd_vel.linear.x = joy_msg -> axes[1] * manual_ratio;
            cmd_vel.angular.z = joy_msg -> axes[0] * manual_ratio;
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

    void SpeedCallback(const std_msgs::Float32::ConstPtr& speed_msgs)
    {
        config_.linear_vel_ = speed_msgs->data;
    }


    void localDataCallback(const std_msgs::Float32MultiArray::ConstPtr& local_msgs)
    {
        postech_mode_ = local_msgs->data[0];

        global_x_err_ = local_msgs->data[1];
        global_y_err_ = local_msgs->data[2];
        global_r_theta_ = local_msgs->data[3];
        global_c_theta_ = local_msgs->data[4];

        init_call = local_msgs->data[5];

        docking_out_flag = local_msgs->data[6];

        local_data_receive =true;
    }




    void modeCallback(const std_msgs::Int32::ConstPtr &Mode_value)
    {
        HJ_mode_low = Mode_value->data;
        HJ_mode_cnt =0;
    }


private:
    // Publisher & Subscriber
    ros::Subscriber sub_scan_;
    ros::Publisher pub_line_;
    ros::Publisher pub_points_;
    ros::Publisher pub_prelidar_fail_;
    ros::Publisher pub_lidar_fail_;

    ros::Subscriber sub_joy_;//Joystic
    ros::Subscriber sub_obs_dists_;//Obs
    ros::Subscriber sub_aisle_;//RPLidar for aisle
    ros::Subscriber sub_localization_;//From localization node
    ros::Subscriber sub_driving_;
    ros::Subscriber sub_mode_call_;
    ros::Subscriber sub_speed_;//For reference speed
    ros::Subscriber sub_gmapping;

    ros::Publisher pub_cmd_;
    ros::Publisher pub_prelidar_end_;
    ros::Publisher pub_docking_end_;

    bool joy_driving_ = false; // even: auto, odd: joy control
    // Obs
    float obs_x_ = 10000;
    float obs_y_ = 10000;
    bool was_obs_in_aisle = false;
    double spare_length = 0;
    float temp_y_err_local = 0;
    float pre_y_err = 0.0;
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
    unsigned int gmapping_cnt=0;
    unsigned int gmapping_start_cnt=0;

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

    unsigned int switch_flag0 = true;
    unsigned int switch_flag1 = true;
    bool axes_flag[7] ={true};
    float manual_ratio = 1.0;
    unsigned int odom_update_cnt=0;//To update encoder odom while mapping
    float docking_out_flag=0;
    /** configuration parameters */
    typedef struct
    {
        double line_thresh_;
        double aisle_width_;

        double Kpx_param_;
        double Kpy_param_;
        double Kpy_param_rot_;
        double Kpy_param_boundary_gain_;
        double theta_ratio_;

        double tunnel_gain_boundary_;
        double linear_vel_;
        double robot_width_;
        double obs_coefficient_;
        double front_obs_;
        double spare_length_;
        double boundary_percent_;
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
        bool Postech_code_;
        double obs_avoidance_distance_;
    } Config;
    Config config_;
};
}
PLUGINLIB_EXPORT_CLASS(auto_driving::AisleDetectNode, nodelet::Nodelet);

