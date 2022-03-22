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
		nhp.param("h_velo", config_.h_velo_, 0.005);
		nhp.param("front_limit", config_.front_limit_, 1.5);
		nhp.param("side_limit", config_.side_limit_, 0.25);

		sub_pointcloud_ = nhp.subscribe("/velodyne_points", 10, &ObstaclesNode::cloudCallback, this);
		pub_obs_ = nhp.advertise<sensor_msgs::PointCloud2> ("/cropped_obs", 10);
        pub_obs_dists_ = nhp.advertise<std_msgs::Float32MultiArray> ("/obs_dists", 10);
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
		pcl::PointCloud<pcl::PointXYZ>::Ptr output(new pcl::PointCloud<pcl::PointXYZ>());
		pcl::PointCloud<pcl::PointXYZ>::Ptr output_obs(new pcl::PointCloud<pcl::PointXYZ>());
		pcl::PointCloud<pcl::PointXYZ>::Ptr output_obs_selected(new pcl::PointCloud<pcl::PointXYZ>());

		//OBSTACLE DETECTION

		pcl::PassThrough<pcl::PointXYZ> pass;
		pass.setInputCloud (input_ptr);         
		pass.setFilterFieldName ("z");         
		pass.setFilterLimits (-(config_.h_velo_-0.15), 0.2);
		pass.filter (*output_obs);

		// x<1.5 (front)
		pass.setInputCloud (output_obs);         
		pass.setFilterFieldName ("x");         
		pass.setFilterLimits (0.1, config_.front_limit_);
		pass.filter (*output_obs);  
		
		// |y|<0.3 (left,right)
		pass.setInputCloud (output_obs);         
		pass.setFilterFieldName ("y");         
        pass.setFilterLimits (-config_.side_limit_, config_.side_limit_);
		pass.filter (*output_obs);  

		*output_obs_selected = *output_obs;

    //classify nearest obstacle
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
			if(nn_indices.size() > 5)
            {
				for (int i = 0; i < nn_indices.size(); i++){
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
					dists.data.push_back(output_obs_selected->points[nn_indices[min_positive_point]].x);
					dists.data.push_back(output_obs_selected->points[nn_indices[min_positive_point]].y);
				}
				else	//obstacles in right side
				{	
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
    }

private:
	ros::Subscriber sub_pointcloud_;
	ros::Publisher pub_obs_;
	ros::Publisher pub_obs_dists_;
	 

	/** configuration parameters */
	typedef struct
	{
	    double h_velo_;
	    double front_limit_;
	    double side_limit_;
	} Config;
	Config config_;
};
}
PLUGINLIB_EXPORT_CLASS(auto_driving::ObstaclesNode, nodelet::Nodelet);

