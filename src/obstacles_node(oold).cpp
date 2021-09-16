#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl_conversions/pcl_conversions.h>

// PCL specific includes
#include <pcl/conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>

#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>

#include <iostream>
#include <fstream>
#include <cmath>


ros::Publisher pub;
ros::Publisher pub2;

// This is to save on typing
typedef pcl::PointCloud<pcl::PointXYZ> point_cloud_t;
int id_=0;
void cloud_cb (const sensor_msgs::PointCloud2& ros_pc)
{
	bool new_obs_ =false;
	float temp_distance_ = 0;
	float h_velo = 0.33;  //0.34	//바뀌는 로봇에 따라 벨로다인 설치 높이를 측정하고, 
	//point cloud에서 바닥의 z값이 어느정도 되는지를 반영, but 너무 배수로 위쪽까지 하면 모바일 로봇 주행하면서 높이 변화 생겼을 경우 배수로가 아니라 낮은 높이의 좌표를 인식해버릴수도있음
	//그리고 배수로보다 그런 일반 지면의 값이 더 많이 나오면 평면의 방향이 아예 틀려질 수 있다.
	float w_drain = 0.2;
	float left_angle,right_angle,left_distance,right_distance;
	pcl::PCLPointCloud2 pcl_pc; // temporary PointCloud2 intermediary
	
	pcl_conversions::toPCL(ros_pc, pcl_pc);
	// Convert point cloud to PCL native point cloud
	point_cloud_t::Ptr input_ptr(new point_cloud_t());
	pcl::fromPCLPointCloud2(pcl_pc, *input_ptr);

	// Create output point cloud
	point_cloud_t::Ptr output_ptr(new point_cloud_t());    	    
	point_cloud_t::Ptr output_left(new point_cloud_t());    	    
	point_cloud_t::Ptr output_right(new point_cloud_t());
	point_cloud_t::Ptr output(new point_cloud_t());     	    
	point_cloud_t::Ptr output_obs(new point_cloud_t());     	    
	point_cloud_t::Ptr output_obs_selected(new point_cloud_t());  

	pcl::PassThrough<pcl::PointXYZ> pass;
	pass.setInputCloud (input_ptr);         
	pass.setFilterFieldName ("z");         
//	pass.setFilterLimits (-(h_velo+0.3), -h_velo);    회전할때조차 아래에 이상한 포인트를 안보기 위해
	//회전할땐 어차피 point cloud로 안하니까, 기본 주행할때는 맘껏 볼 수 있도록
	pass.setFilterLimits (-1, -h_velo);    
	//pass.setFilterLimitsNegative (false);  
	pass.filter (*output_ptr);              

	
	// x<2 (front)
	pass.setInputCloud (output_ptr);         
	pass.setFilterFieldName ("x");         
	pass.setFilterLimits (0, 1.8); //2.0   
	//pass.setFilterLimitsNegative (false);  
	pass.filter (*output_ptr);              

	// y>0 - left
	pass.setInputCloud (output_ptr);         
	pass.setFilterFieldName ("y");         
	pass.setFilterLimits (0, 1.0);    
	//pass.setFilterLimitsNegative (false);  
	pass.filter (*output_left);              

	// y<0 - right
	pass.setInputCloud (output_ptr);         
	pass.setFilterFieldName ("y");         
	pass.setFilterLimits (-1.0, 0);    
	//pass.setFilterLimitsNegative (false);  
	pass.filter (*output_right);              


/*	
	  // 배수로 데이터 출
        std::cout << "Loaded_left :" << output_left->width * output_left->height  << std::endl;
        std::cout << "Loaded_right :" << output_right->width * output_right->height  << std::endl;
*/
	

	//RANSAC -이상치제거
	pcl::ModelCoefficients::Ptr coefficients_left (new pcl::ModelCoefficients ());
	pcl::ModelCoefficients::Ptr coefficients_right (new pcl::ModelCoefficients ());
			
	if(output_left->size() > 10 && output_right->size() > 10 ){
		
		//left
		pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
									
		pcl::SACSegmentation<pcl::PointXYZ> seg;
		seg.setOptimizeCoefficients (true);      
		seg.setInputCloud (output_left);                 
		seg.setModelType (pcl::SACMODEL_PLANE);    
		seg.setMethodType (pcl::SAC_RANSAC);      
		seg.setMaxIterations (1000);              
		seg.setDistanceThreshold (0.01);          
		seg.segment (*inliers, *coefficients_left);    

		pcl::ExtractIndices<pcl::PointXYZ> extract;
		extract.setInputCloud (output_left);
		extract.setIndices (inliers);
		extract.setNegative (false);//false
		extract.filter (*output_left);
	
		//right		
		//pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
									
		//pcl::SACSegmentation<pcl::PointXYZ> seg;
		seg.setOptimizeCoefficients (true);      
		seg.setInputCloud (output_right);                 
		seg.setModelType (pcl::SACMODEL_PLANE);    
		seg.setMethodType (pcl::SAC_RANSAC);      
		seg.setMaxIterations (1000);              
		seg.setDistanceThreshold (0.01);          
		seg.segment (*inliers, *coefficients_right);    

		//pcl::ExtractIndices<pcl::PointXYZ> extract;
		extract.setInputCloud (output_right);
		extract.setIndices (inliers);
		extract.setNegative (false);//false
		extract.filter (*output_right);
	}
	else{
	//아무것도 못찾았을 경우 어떻게할까? 
		std::cerr << "Not enough point" << std::endl;
		return;
	}

	//fail&safe (양쪽 배수로의 평면 방정식중 , y가 0.85보다 작으면, 즉 옆을 바라보고 있지 않으면 fail
	if(abs(coefficients_left->values[1]) < 0.85 || abs(coefficients_right->values[1]) < 0.85){
		std::cerr << "Wrong direction plane" << std::endl;		
		return;
	}
/*
	// 배수로 ransac 이후의 유효 데이터 수
        std::cout << "Processed _left:" << output_left->width * output_left->height  << std::endl;
        std::cout << "Processed _right:" << output_right->width * output_right->height  << std::endl;
*/
	//추정된 평면 파라미터 출력 (eg. ax + by + cz + d = 0 ).
  	std::cerr << "Model coefficients_left: " << coefficients_left->values[0] << " " 
                                      << coefficients_left->values[1] << " "
                                      << coefficients_left->values[2] << " " 
                                      << coefficients_left->values[3] << std::endl;

  	std::cerr << "Model coefficients_right: " << coefficients_right->values[0] << " " 
                                      << coefficients_right->values[1] << " "
                                      << coefficients_right->values[2] << " " 
                                      << coefficients_right->values[3] << std::endl;

	left_angle  = atan(-coefficients_left->values[0] / coefficients_left->values[1]) *180/3.141592;
	right_angle  = atan(-coefficients_right->values[0] / coefficients_right->values[1]) *180/3.141592;
	
	//제자리 회전 이후 양쪽 angle중 (둘 중 하나라도/둘 다) 특정각(ex-4도) 이하일 때 출발하도록
	//양쪽의 angle 차이가 10도 이상 나면 return?(양쪽중 하나는 실패한거니까
	std::cerr << "left line angle : " << left_angle <<std::endl;
	std::cerr << "right line angle : " << right_angle <<std::endl;
	

	//배수로 양옆의 평면의 방정식에 로봇에서 1m앞을 본 좌표 넣어준 값을 출력(1,0,-h_velo)
	left_distance = coefficients_left->values[0] + coefficients_left->values[2]*(-h_velo) + coefficients_left->values[3];
	right_distance = coefficients_right->values[0] + coefficients_right->values[2]*(-h_velo) + coefficients_right->values[3];
	//left dist>0, right >0, 
	//left+right>0면 왼쪽배수로까지의 거리가 먼거니까 오른쪽으로 치우침
	//left+right<0면 오른쪽배수로까지의 거리가 먼거니까 왼쪽으로 치우침
	//몇번 wrong direction, not enough point 나오는지 count해주고, 5회 이상 연속으로 나오면 error flag 주고 정지.
	//아니면 직전에 있었던 difference 값을 바탕으로 로봇 회전 제어 
	std::cerr << "left line distance : " << left_distance <<std::endl;
	std::cerr << "right line distance : " << right_distance <<std::endl;
	std::cerr << "differnece btw lines : " << left_distance +right_distance <<std::endl;

		
	*output = *output_left + *output_right;


	//z축에 해당하는거 빼버리고 xy 2차원 직선 2개 구하고, 그걸 기준으로 obs detection
	//obs 후보 고르고, 직선 사이에 있는애들만 유효 obs로 판단
	pass.setInputCloud (input_ptr);         
	pass.setFilterFieldName ("z");         
	pass.setFilterLimits (-(h_velo-0.15), 0.2);    
	//pass.setFilterLimitsNegative (false);  
	pass.filter (*output_obs);

	// x<2 (front)
	pass.setInputCloud (output_obs);         
	pass.setFilterFieldName ("x");         
	pass.setFilterLimits (0.5, 2.0);    
	//pass.setFilterLimitsNegative (false);  
	pass.filter (*output_obs);  


	float temp_x,temp_y;
	int temp_count = 0;
	for(int i = 0; i < output_obs->points.size(); ++i){
		temp_x = output_obs->points[i].x;
		temp_y = output_obs->points[i].y;
		if(coefficients_left->values[0]*temp_x + coefficients_left->values[1]*temp_y + coefficients_left->values[3] -w_drain >0 
		&& coefficients_right->values[0]*temp_x + coefficients_right->values[1]*temp_y + coefficients_right->values[3] +w_drain <0)
		{ 
			output_obs_selected->push_back(output_obs->points[i]);
		}
	}
	

	// Convert data type PCL to ROS
	sensor_msgs::PointCloud2 ros_output;
	pcl::toPCLPointCloud2(*output, pcl_pc);
	pcl_conversions::fromPCL(pcl_pc, ros_output);

	// Publish the data	
	pub.publish(ros_output);
	
	if(output_obs_selected->size() >5)
	{
		sensor_msgs::PointCloud2 ros_obs;
		pcl::toPCLPointCloud2(*output_obs_selected, pcl_pc);
		pcl_conversions::fromPCL(pcl_pc, ros_obs);
		ros_obs.header.frame_id=ros_pc.header.frame_id;
		pub2.publish(ros_obs);
	}
}

int main (int argc, char** argv)
{
    // Initialize ROS
    ros::init (argc, argv, "pcl_voxel");
    ros::NodeHandle nh;

    // Create a ROS subscriber for the input point cloud
    ros::Subscriber sub = nh.subscribe("/velodyne_points", 1, cloud_cb);

    // Create a ROS publisher for the output point cloud
    pub = nh.advertise<sensor_msgs::PointCloud2>("/velodyne_points_line", 1);
    pub2 = nh.advertise<sensor_msgs::PointCloud2>("/obs_dists", 1);

    // Spin
    ros::spin ();
}
