#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <geometry_msgs/Point32.h>
#include <tf/transform_listener.h>
#include <costmap_2d/costmap_2d.h>

#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/filters/passthrough.h>



class StepCostmap
{
public:
	StepCostmap();
private:
	ros::NodeHandle nh_;
	ros::Subscriber cloud_sub_;
	ros::Publisher cloud_pub_;
	tf::TransformListener tf_listener_;
	void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& msgs);
	costmap_2d::Costmap2D costmap_;
	
	std::string sensor_frame_, topic_name_;
	double sensor_range_x_min_, sensor_range_x_max_, sensor_range_y_min_, sensor_range_y_max_, sensor_range_z_min_, sensor_range_z_max_;
};

StepCostmap::StepCostmap()
{
	std::cout << "start Step Costmap class" << std::endl;
	
	ros::NodeHandle private_nh("~");
	private_nh.param("sensor_frame", sensor_frame_, std::string("/velodyne"));
	private_nh.param("topic_name", topic_name_, std::string("velodyne_points"));
	private_nh.param("sensor_range_x_min", sensor_range_x_min_, 0.0);
	private_nh.param("sensor_range_x_max", sensor_range_x_max_, 2.0);
	private_nh.param("sensor_range_y_min", sensor_range_y_min_, -2.0);
	private_nh.param("sensor_range_y_max", sensor_range_y_max_, 2.0);
	private_nh.param("sensor_range_z_min", sensor_range_z_min_, -0.6);
	private_nh.param("sensor_range_z_max", sensor_range_z_max_, -0.3);
	
	costmap_.setDefaultValue(0);
	costmap_.resizeMap(40, 40, 0.1, 0, 0);
	

	cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/step_cloud", 1, false);
	cloud_sub_ = nh_.subscribe(topic_name_, 1, &StepCostmap::cloudCallback, this);
}

void StepCostmap::cloudCallback(const sensor_msgs::PointCloud2ConstPtr& msgs)
{
	std::cout << "cloud callback" << std::endl;
	
	double robot_x, robot_y;
	double new_origin_x, new_origin_y;

	unsigned int mx, my;
	double wx, wy, wz;
	
	try
	{	
		tf::StampedTransform trans;
		tf_listener_.waitForTransform("odom", "base_link", ros::Time(0), ros::Duration(0.5));
		tf_listener_.lookupTransform("odom", "base_link", ros::Time(0), trans);
		robot_x = trans.getOrigin().x();
		robot_y = trans.getOrigin().y();
	}
	catch(tf::TransformException &e)
	{
		ROS_WARN("%s", e.what());
	}
		
	new_origin_x = robot_x - step_costmap_.getSizeInMetersX() / 2;
	new_origin_y = robot_y - step_costmap_.getSizeInMetersY() / 2;
	
	
	costmap_.updateOrigin(new_origin_x, new_origin_y);
	
	pcl::PointCloud<pcl::PointXYZI> pcl_cloud;
	
	pcl::fromROSMsg (*msgs, pcl_cloud);
	
	//debug//
	std::cout << "pcl_cloud size 1:" << pcl_cloud.width << std::endl;

	pcl::PassThrough<pcl::PointXYZI> pass;
	pass.setInputCloud (pcl_cloud.makeShared());
	pass.setFilterFieldName ("x");
	pass.setFilterLimits (sensor_range_x_min_, sensor_range_x_max_);
	pass.filter (pcl_cloud);
	
	pass.setInputCloud (pcl_cloud.makeShared());
	pass.setFilterFieldName ("y");
	pass.setFilterLimits (sensor_range_y_min_, sensor_range_y_max_);
	pass.filter (pcl_cloud);
	
	pass.setInputCloud (pcl_cloud.makeShared());
	pass.setFilterFieldName ("z");
	pass.setFilterLimits (sensor_range_z_min_, sensor_range_z_max_);
	pass.filter (pcl_cloud);
	
	//debug//
	std::cout << "pcl_cloud size:" << pcl_cloud.width << std::endl;

	try
	{
		tf::StampedTransform trans;
		tf_listener_.waitForTransform("odom", sensor_frame_, ros::Time(0), ros::Duration(0.5));
		tf_listener_.lookupTransform(sensor_frame_, "odom", ros::Time(0), trans);
		pcl_ros::transformPointCloud("odom", pcl_cloud, pcl_cloud,tf_listener_);
	}
	catch(tf::TransformException &e)
	{
		ROS_WARN("%s", e.what());
	}
	
}

int main(int argc, char **argv)
{
    std::cout << "start step_costmap" << std::endl;
	ros::init(argc, argv, "step_costmap");
	StepCostmap sc;
	ros::Rate r(5);
	while(ros::ok()){
		ros::spinOnce();
		r.sleep();
	}
	return 0;
}
