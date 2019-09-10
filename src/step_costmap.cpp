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
#include <pcl/point_types.h>
#include <pcl/common/common.h>


#define MIN_Z 0
#define MAX_Z 1
#define HAVE_VALUE 2

#define HIGH 1
#define MIDDLE 2
#define LOW 3
#define ROAD 4
#define NONE 5



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
	private_nh.param("sensor_range_x_min", sensor_range_x_min_, -8.0);
	private_nh.param("sensor_range_x_max", sensor_range_x_max_, 8.0);
	private_nh.param("sensor_range_y_min", sensor_range_y_min_, -8.0);
	private_nh.param("sensor_range_y_max", sensor_range_y_max_, 8.0);

	private_nh.param("z_th", z_th_, 0.1);
	
	costmap_.setDefaultValue(0);
	costmap_.resizeMap(160, 160, 0.1, 0, 0);
	

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

	std::cout << "pcl_cloud size 2:" << pcl_cloud.width << std::endl;


    //region_pcl
    double height_map[160][160][3];//min_z,max_z,have_value
	pcl::PointCloud<pcl::PointXYZI> region_cloud1;
	pcl::PointCloud<pcl::PointXYZI> region_cloud2;
    pcl::PointXYZI minPt, maxPt;
    static int size_sum;

    for (int i=0;i<160;i++){

        double min_x = sensor_range_x_min_ + 0.1*i;

        pass.setInputCloud (pcl_cloud.makeShared());
        pass.setFilterFieldName ("x");
        pass.setFilterLimits (min_x, min_x + 0.1 );
        pass.filter (region_cloud1);


        for (int j=0;j<160;j++){
           
            double min_y = sensor_range_y_min_ + 0.1*j;

            pass.setInputCloud (region_cloud1.makeShared());
            pass.setFilterFieldName ("y");
            pass.setFilterLimits (min_y, min_y+0.1 );
            pass.filter (region_cloud2);

            if (region_cloud2.size() > 0){
                pcl::getMinMax3D (region_cloud2, minPt, maxPt);
                height_map[i][j][MIN_Z] = minPt.z;
                height_map[i][j][MAX_Z] = maxPt.z;
                height_map[i][j][HAVE_VALUE] = 1;
            }
            else{
                height_map[i][j][MIN_Z] = 0;
                height_map[i][j][MAX_Z] = 0;
                height_map[i][j][HAVE_VALUE] = 0;

            }
            size_sum += region_cloud2.size();//debug: region size sum
            //std::cout << height_map[i][j][1] << " ";
            //std::cout << height_map[i][j][2] << " ";
        }
        //std::cout << std::endl;
    }
    //std::cout << "size sum:" << size_sum << std::endl;//debug: region size sum
    size_sum = 0;

    double diff_map[160][160];
    
    for (int i=0;i<160;i++){
        for (int j=0;j<160;j++){
            if (i == 0 || j == 0 || i == 159 || j == 160){
                diff_map[i][j] = 0;
            }
            else if (height_map[i][j][HAVE_VALUE] == 1){
                double min_z = 0;
                for (int k=0;k<3;k++){
                    for (int l=0;l<3;l++){
                        if (height_map[i+k-1][j+l-1][HAVE_VALUE] == 1){
                            if (min_z > height_map[i+k-1][j+l-1][MIN_Z]){
                                min_z = height_map[i+k-1][j+l-1][MIN_Z];
                            }
                        }
                    }
                }
                diff_map[i][j] = height_map[i][j][MAX_Z] - min_z;
            }
            //std::cout << diff_map[i][j] << " ";
        }
        //std::cout << std::endl;
    }

    double type_map[160][160];
    for (int i=0;i<160;i++){
        for (int j=0;j<160;j++){
            double diff_value = diff_map[i][j];
            if (0 < diff_value && diff_value < 0.1)type_map[i][j] = ROAD;
            else if (diff_value < 0.3)type_map[i][j] = LOW;
            else if (diff_value < 0.4)type_map[i][j] = MIDDLE;
            else if (diff_value >= 0.4)type_map[i][j] = HIGH;
            std::cout << type_map[i][j] << " ";
        }
        std::cout << std::endl;
    }

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

    sensor_msgs::PointCloud2 cloud1;
    pcl::toROSMsg(pcl_cloud_odom, cloud1);
    cloud1.header.frame_id = "odom";
    cloud_pub1_.publish(cloud1);

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
