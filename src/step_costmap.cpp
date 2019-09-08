#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <geometry_msgs/Point32.h>
#include <tf/transform_listener.h>
#include <costmap_2d/costmap_2d.h>

#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>



class StepCostmap
{
public:
	StepCostmap();
private:
	ros::NodeHandle nh_;
	ros::Subscriber cloud_sub_;
	ros::Publisher cloud_pub1_;
	ros::Publisher cloud_pub2_;
	tf::TransformListener tf_listener_;
	void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& msgs);
	pcl::PointCloud<pcl::PointXYZI> stored_pcl_cloud_;
	costmap_2d::Costmap2D costmap_;
	
	std::string sensor_frame_, topic_name_;
	double sensor_range_x_min_, sensor_range_x_max_, sensor_range_y_min_, sensor_range_y_max_, sensor_range_z_min_, sensor_range_z_max_;
    double z_th_;
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

	private_nh.param("z_th", z_th_, 0.1);
	
	costmap_.setDefaultValue(0);
	costmap_.resizeMap(40, 40, 0.1, 0, 0);
	

	cloud_pub1_ = nh_.advertise<sensor_msgs::PointCloud2>("/cloud1", 1, false);
	cloud_pub2_ = nh_.advertise<sensor_msgs::PointCloud2>("/cloud2", 1, false);
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
		tf_listener_.waitForTransform("odom", "base_link", ros::Time(0), ros::Duration(1));
		tf_listener_.lookupTransform("odom", "base_link", ros::Time(0), trans);
		robot_x = trans.getOrigin().x();
		robot_y = trans.getOrigin().y();
	}
	catch(tf::TransformException &e)
	{
		ROS_WARN("%s", e.what());
	}
		
	new_origin_x = robot_x - costmap_.getSizeInMetersX() / 2;
	new_origin_y = robot_y - costmap_.getSizeInMetersY() / 2;
	
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
	std::cout << "pcl_cloud size2:" << pcl_cloud.width << std::endl;


	pcl::PointCloud<pcl::PointXYZI> pcl_cloud_odom;
	try
	{
		tf::StampedTransform trans;
		//tf_listener_.waitForTransform(sensor_frame_, "odom", ros::Time(0), ros::Duration(10.0));
		tf_listener_.lookupTransform(sensor_frame_, "odom", ros::Time(0), trans);
		pcl_ros::transformPointCloud("odom", pcl_cloud, pcl_cloud_odom,tf_listener_);
	}
	catch(tf::TransformException &e)
	{
		ROS_WARN("%s", e.what());
	}

    stored_pcl_cloud_ += pcl_cloud_odom;
    pcl::VoxelGrid<pcl::PointXYZI> sor;
    sor.setInputCloud(stored_pcl_cloud_.makeShared());
    sor.setLeafSize(0.01f, 0.01f, 0.01f);
    sor.filter (stored_pcl_cloud_);

	pass.setInputCloud (stored_pcl_cloud_.makeShared());
	pass.setFilterFieldName ("x");
	pass.setFilterLimits (robot_x - 2.0, robot_x + 2.0);
	pass.filter (stored_pcl_cloud_);
	
	pass.setInputCloud (stored_pcl_cloud_.makeShared());
	pass.setFilterFieldName ("y");
	pass.setFilterLimits (robot_y - 2.0, robot_y + 2.0);
	pass.filter (stored_pcl_cloud_);


    sensor_msgs::PointCloud2 cloud1;
    pcl::toROSMsg(stored_pcl_cloud_, cloud1);
    cloud1.header.frame_id = "odom";
    cloud_pub1_.publish(cloud1);

    pcl::PointCloud<pcl::PointXYZI> step_pcl_cloud;
    pcl::PointCloud<pcl::PointXYZI> region_y_pcl_cloud;
    pcl::PointCloud<pcl::PointXYZI> region_pcl_cloud;

    double grid_width = 0.1;
    
    for (int i=0;i<(4.0/grid_width);i++)
    {
        double x_min = robot_x - 2.0 + i*grid_width;
        pass.setInputCloud (stored_pcl_cloud_.makeShared());
        pass.setFilterFieldName ("x");
        pass.setFilterLimits (x_min, x_min + grid_width);
        pass.filter (region_y_pcl_cloud);
        
        for (int j=0;j<(4.0/grid_width);j++){

            double y_min = robot_y - 2.0 + j*grid_width;
            pass.setInputCloud (region_y_pcl_cloud.makeShared());
            pass.setFilterFieldName ("y");
            pass.setFilterLimits (y_min, y_min + grid_width);
            pass.filter (region_pcl_cloud);
            
            //std::cout << "region_pcl_cloud size " << 40*i + j << ":" <<  region_pcl_cloud.size() << std::endl;

            if (region_pcl_cloud.size() != 0)
            {
                double z_min=region_pcl_cloud.points[0].z;
                double z_max=region_pcl_cloud.points[0].z;
                double z_now;
                for (int k=0;k<region_pcl_cloud.size();k++){
                    z_now = region_pcl_cloud.points[k].z;
                    if (z_min > z_now)z_min = z_now;
                    if (z_max < z_now)z_max = z_now;
                    //std::cout << "z_min " << z_min << std::endl;
                }
                std::cout << z_max - z_min << std::endl;
                if ((z_max - z_min) > z_th_)step_pcl_cloud += region_pcl_cloud;
            }
        }
    }

    sensor_msgs::PointCloud2 cloud2;
    pcl::toROSMsg(step_pcl_cloud, cloud2);
    std::cout << "step_pcl_cloud size:" << step_pcl_cloud.size() << std::endl;
    step_pcl_cloud.clear();
    cloud2.header.frame_id = "odom";
    cloud_pub2_.publish(cloud2);




}

int main(int argc, char **argv)
{
    std::cout << "start step_costmap" << std::endl;
	ros::init(argc, argv, "step_costmap");
	StepCostmap sc;
	ros::Rate r(10);
	while(ros::ok()){
		ros::spinOnce();
		r.sleep();
	}
	return 0;
}
