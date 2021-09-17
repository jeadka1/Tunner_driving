#include "ros/ros.h"
#include "nodelet/nodelet.h"
#include <pluginlib/class_list_macros.h>

#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Float32MultiArray.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>


namespace auto_driving {

class ObstaclesNode : public nodelet::Nodelet {

public:
	ObstaclesNode() = default;

private:
	virtual void onInit() {
		ros::NodeHandle nh = getNodeHandle();
		ros::NodeHandle nhp = getPrivateNodeHandle();
		
		// Configuration //
		nhp.param("obstacle_coefficient", config_.obstacle_coefficient_, 0.005);
		nhp.param("front_obstacle_dist", config_.front_obstacle_dist_, 0.1);

		sub_pointcloud_ = nhp.subscribe("/velodyne_points", 10, &ObstaclesNode::cloudCallback, this);
		pub_obs_ = nhp.advertise<sensor_msgs::PointCloud2> ("/cropped_obs", 10);
        	pub_obs_dists_ = nhp.advertise<std_msgs::Float32MultiArray> ("/obs_dists", 10);
		pub_drain_line_ = nhp.advertise<sensor_msgs::PointCloud2> ("/drain_line", 10);
	};
	
	void cloudCallback(const sensor_msgs::PointCloud2::ConstPtr& pc_msg)
	{
    	std_msgs::Float32MultiArray dists;
	dists.data.clear();
		
    	pcl::PCLPointCloud2 pcl_pc; // temporary PointCloud2 intermediary
		pcl_conversions::toPCL(*pc_msg, pcl_pc);
		// Convert point cloud to PCL native point cloud
		pcl::PointCloud<pcl::PointXYZ>::Ptr input_ptr(new pcl::PointCloud<pcl::PointXYZ>());
		pcl::fromPCLPointCloud2(pcl_pc, *input_ptr);
		// Create output point cloud
		pcl::PointCloud<pcl::PointXYZ>::Ptr output_ptr(new pcl::PointCloud<pcl::PointXYZ>());   	    
		pcl::PointCloud<pcl::PointXYZ>::Ptr output_left(new pcl::PointCloud<pcl::PointXYZ>());  	    
		pcl::PointCloud<pcl::PointXYZ>::Ptr output_right(new pcl::PointCloud<pcl::PointXYZ>());
		pcl::PointCloud<pcl::PointXYZ>::Ptr output(new pcl::PointCloud<pcl::PointXYZ>());
		pcl::PointCloud<pcl::PointXYZ>::Ptr output_obs(new pcl::PointCloud<pcl::PointXYZ>());
		pcl::PointCloud<pcl::PointXYZ>::Ptr output_obs_selected(new pcl::PointCloud<pcl::PointXYZ>());

		//Drain line detection using Velodyne
		float h_velo = 0.40;  //0.34	//바뀌는 로봇에 따라 벨로다인 설치 높이를 측정하고, 
		//point cloud에서 바닥의 z값이 어느정도 되는지를 반영, but 너무 배수로 위쪽까지 하면 모바일 로봇 주행하면서 높이 변화 생겼을 경우 배수로가 아니라 낮은 높이의 좌표를 인식해버릴수도있음
		//그리고 배수로보다 그런 일반 지면의 값이 더 많이 나오면 평면의 방향이 아예 틀려질 수 있다.
		float w_drain = 0.2;
		

		pcl::PassThrough<pcl::PointXYZ> pass;
		pass.setInputCloud (input_ptr);         
		pass.setFilterFieldName ("z");         
		//pass.setFilterLimits (-(h_velo+0.3), -h_velo);    
		//회전할때조차 아래에 이상한 포인트를 안보기 위해
		//회전할땐 어차피 point cloud로 안하니까, 기본 주행할때는 맘껏 볼 수 있도록
		//pass.setFilterLimits (-1, -h_velo);    
		pass.setFilterLimits (-h_velo+0.03,-h_velo+0.09 );
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
			//std::cerr << "Not enough point for finding drain line" << std::endl;
			return;
		}

		//fail&safe (양쪽 배수로의 평면 방정식중 , y가 0.85보다 작으면, 즉 옆을 바라보고 있지 않으면 fail
		/*if(abs(coefficients_left->values[1]) < 0.85 || abs(coefficients_right->values[1]) < 0.85){
			std::cerr << "Wrong direction plane for finding drain line" << std::endl;		
			return;
		}*/


		//추정된 평면 파라미터 출력 (eg. ax + by + cz + d = 0 ).
	  	/*std::cerr << "Model coefficients_left: " << coefficients_left->values[0] << " " 
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
*/
		//drain	line		
		*output = *output_left + *output_right;


		//OBSTACLE DETECTION

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
		
		// |y|<0.3 (left,right)
		pass.setInputCloud (output_obs);         
		pass.setFilterFieldName ("y");         
		pass.setFilterLimits (-0.3, 0.3);    
		//pass.setFilterLimitsNegative (false);  
		pass.filter (*output_obs_selected);  

		/*float temp_x,temp_y;
		int temp_count = 0;
		for(int i = 0; i < output_obs->points.size(); ++i){
			temp_x = output_obs->points[i].x;
			temp_y = output_obs->points[i].y;
			if(coefficients_left->values[0]*temp_x + coefficients_left->values[1]*temp_y + coefficients_left->values[3] -w_drain >0 
			&& coefficients_right->values[0]*temp_x + coefficients_right->values[1]*temp_y + coefficients_right->values[3] +w_drain <0)
			{ 
				output_obs_selected->push_back(output_obs->points[i]);
			}
		}*/


	if(output_obs_selected->size() != 0)
        {
			pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
			sor.setInputCloud (output_obs_selected);  
			sor.setMeanK (50);               
			sor.setStddevMulThresh (1.0);    
			sor.filter (*output_obs_selected);        
		    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);	    
			pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
			tree->setInputCloud (output_obs_selected);  
			std::vector<int> nn_indices(30);
			std::vector<float> nn_dists(30);
			pcl::PointXYZ origin(0, 0, 0);

			tree->nearestKSearch(origin, 30, nn_indices, nn_dists);

			float min_negative = -999, min_positive = 999, movement = 0, min_positive_point =0, min_negative_point =0;
			int count_negative=0, count_positive = 0, count_center = 0;
			float point_x, point_y; 
			//front_obstacle_ = false;
			if(nn_indices.size() > 5)
            {
				for (int i = 0; i < nn_indices.size(); i++)
				{
					point_y = output_obs_selected->points[nn_indices[i]].y;
					point_x = output_obs_selected->points[nn_indices[i]].x;
					
					if(point_y > 0) //obstacles in left side
					{			
						if(min_positive >point_y)
						{
							min_positive = point_y;
							min_positive_point = i;
						}
						count_positive += 1;
					}
					else //obstacles in right side
					{
						if(min_negative < point_y)
						{	
							min_negative = point_y;
							min_negative_point = i;
						}
						count_negative += 1;
					}
				}

				if(count_positive > count_negative) //obstacles in left side
				{
					// shift_position_ = std::min(params_.obstacle_coefficient_/output_obs_selected->points[nn_indices[min_positive_point]].x, 0.01);
					//std::cout<<"[LEFT OBSTACLES] Distance to obstacles(m): "<<abs(output_obs_selected->points[nn_indices[min_positive_point]].x) <<std::endl;	
					dists.data.push_back(output_obs_selected->points[nn_indices[min_positive_point]].x);
					dists.data.push_back(output_obs_selected->points[nn_indices[min_positive_point]].y);
				}
				else	//obstacles in right side
				{	
					// shift_position_ = std::max(-params_.obstacle_coefficient_/output_obs_selected->points[nn_indices[min_negative_point]].x, -0.01);
					//std::cout<<"[RIGHT OBSTACLES] Distance to obstacles(m): "<<abs(output_obs_selected->points[nn_indices[min_negative_point]].x) <<std::endl;
					dists.data.push_back(output_obs_selected->points[nn_indices[min_negative_point]].x);
					dists.data.push_back(output_obs_selected->points[nn_indices[min_negative_point]].y);
				}	
            
	        }
        }
        	// Convert data type PCL to ROS
		sensor_msgs::PointCloud2 ros_output;
		pcl::toPCLPointCloud2(*output_obs_selected, pcl_pc);
		pcl_conversions::fromPCL(pcl_pc, ros_output);

		sensor_msgs::PointCloud2 ros_line_output;
		pcl::toPCLPointCloud2(*output, pcl_pc);
		pcl_conversions::fromPCL(pcl_pc, ros_line_output);

		// Publish the data
		pub_obs_.publish(ros_output);
		pub_obs_dists_.publish(dists);
		pub_drain_line_.publish(ros_line_output);
    }

private:
	ros::Subscriber sub_pointcloud_;
	ros::Publisher pub_obs_;
	ros::Publisher pub_obs_dists_;
	ros::Publisher pub_drain_line_;
	 

	/** configuration parameters */
	typedef struct
	{
		double obstacle_coefficient_;
		double front_obstacle_dist_;
	} Config;
	Config config_;
};
}
PLUGINLIB_EXPORT_CLASS(auto_driving::ObstaclesNode, nodelet::Nodelet);

