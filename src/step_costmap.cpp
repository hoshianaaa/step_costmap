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
#include <pcl/PCLPointCloud2.h>

#include <cmath>


#define MIN_Z 0
#define MAX_Z 1
#define HAVE_VALUE 2

#define LOW 1
#define MIDDLE 2
#define HIGH 3
#define ROAD 4
#define NONE 0

const double D[4] = {300, 300, 550, 700}; //normal distance of ring(i) ~ ring(i+1)
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
    void pcl_z_merge_sort(pcl::PointCloud<pcl::PointXYZ>& pcl_cloud, int left, int right);
    void pcl_z_merge(pcl::PointCloud<pcl::PointXYZ>& pcl_cloud, int left, int mid, int right);
    bool pcl_z_binary_search(const pcl::PointCloud<pcl::PointXYZ>& pcl_cloud, const double z, pcl::PointXYZ& near_point);//0.0017rad = 0.1deg
    void mapToWorld(const int map_x, const int map_y, double& world_x, double& world_y);
    void worldToMap(const double world_x, const double world_y, int& map_x, int& map_y);
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

void StepCostmap::pcl_z_merge_sort(pcl::PointCloud<pcl::PointXYZ>& pcl_cloud, int left, int right){
    if (left + 1 < right) {
        int mid;
        mid = (left + right) / 2;

        //devide
        pcl_z_merge_sort(pcl_cloud, left, mid);
        pcl_z_merge_sort(pcl_cloud, mid, right);

        //conquer
        pcl_z_merge(pcl_cloud, left, mid, right);
    }
}


void StepCostmap::pcl_z_merge(pcl::PointCloud<pcl::PointXYZ>& pcl_cloud, int left, int mid, int right){

    int i, j, k;
    int n1, n2;
    const int INFT = 10000;

    n1 = mid - left;
    n2 = right - mid;

    pcl::PointCloud<pcl::PointXYZ> L, R;

    for (i=0; i < n1; i++) L.push_back(pcl_cloud.at(left + i));
    for (i=0; i < n2; i++) R.push_back(pcl_cloud.at(mid + i));
    
    pcl::PointXYZ pt;
    pt.x = 0;
    pt.y = 0;
    pt.z = INFT;
    
    L.push_back(pt);
    R.push_back(pt);

    //sort

    j = 0;
    k = 0;

    for (i = left; i < right; i++){
        if (L.points[j].z <= R.points[k].z) {

            pcl_cloud.at(i) = L.at(j);
            j++;
        } else {
            pcl_cloud.at(i) = R.at(k);
            k++;
        }
    }
}


bool StepCostmap::pcl_z_binary_search(const pcl::PointCloud<pcl::PointXYZ>& pcl_cloud, const double z, pcl::PointXYZ& near_point){

    const double range = 0.0017;
    double d = 100.0;
    int m, r, l;

    r = 0;
    l = pcl_cloud.size();

    while (1){
        m = (r + l)/2;

        d = std::abs(pcl_cloud.points[m].z - z);
        std::cout << "z - z:" << pcl_cloud.points[m].z - z << std::endl;
        std::cout << "d:" << d << std::endl;
        if (d < range)break;

        if ( pcl_cloud.points[m].z > z)l = m;
        else r = m;
        
        if (r+1 >= l)return 0;
    }
    near_point = pcl_cloud.points[m];
    return 1;
}


void StepCostmap::mapToWorld(const int map_x, const int map_y, double& world_x, double& world_y){
    world_x = map_x * 0.1 - 8.0 + 0.1/2;
    world_y = map_y * 0.1 - 8.0 + 0.1/2;
}

void StepCostmap::worldToMap(const double world_x, const double world_y, int& map_x, int& map_y){
    map_x = int((world_x + 8.0) * 10);
    map_y = int((world_y + 8.0) * 10);
    if (map_x < 0)map_x = 0;
    if (map_y < 0)map_y = 0;
    if (map_x > 159)map_x = 159;
    if (map_y > 159)map_y = 159;
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
	
	pcl::PassThrough<pcl::PointXYZI> pass;
	pass.setInputCloud (pcl_cloud.makeShared());
	pass.setFilterFieldName ("x");
	pass.setFilterLimits (sensor_range_x_min_, sensor_range_x_max_);
	pass.filter (pcl_cloud);
	
	pass.setInputCloud (pcl_cloud.makeShared());
	pass.setFilterFieldName ("y");
	pass.setFilterLimits (sensor_range_y_min_, sensor_range_y_max_);
	pass.filter (pcl_cloud);

   

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
            diff_map[i][j] = 0;
            if (i == 0 || j == 0 || i == 159 || j == 159){
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
            if (diff_value < 0)type_map[i][j] = -1; //error
            else if (diff_value == 0)type_map[i][j] = NONE;
            else if (0 < diff_value && diff_value < 0.1)type_map[i][j] = ROAD;
            else if (diff_value < 0.3)type_map[i][j] = LOW;
            else if (diff_value < 0.4)type_map[i][j] = MIDDLE;
            else if (diff_value >= 0.4)type_map[i][j] = HIGH;
            //std::cout << type_map[i][j] << " ";
        }
        //std::cout << std::endl;
    }

    //** publish low object **//
	pcl::PointCloud<pcl::PointXYZ> low_object_pcl_cloud;
    pcl::PointCloud<pcl::PointXYZ> one_point_pcl_cloud;
    one_point_pcl_cloud.width = 1;
    one_point_pcl_cloud.height = 1;
    one_point_pcl_cloud.is_dense = false;
    one_point_pcl_cloud.resize (1);

    for (int i=0;i<160;i++){
        for (int j=0;j<160;j++){
            if(type_map[i][j] == LOW){
                double world_x,world_y;
                mapToWorld(i,j,world_x,world_y);
                one_point_pcl_cloud.points[0].x = world_x;
                one_point_pcl_cloud.points[0].y = world_y;
                one_point_pcl_cloud.points[0].z = 0;
                low_object_pcl_cloud += one_point_pcl_cloud;
            }
        }
    }

    sensor_msgs::PointCloud2 cloud1;
    pcl::toROSMsg(low_object_pcl_cloud, cloud1);
    cloud1.header.frame_id = sensor_frame_;
    cloud_pub1_.publish(cloud1);

    //** ring_filter_map **//
    std::cout << "ring_filter_map" << std::endl;
	pcl::PointCloud<pcl::PointXYZ> ring_pcl_cloud[16];
    for (int i=0; i<msgs->height * msgs->width; i++){
        int arrayPosX = i * msgs->point_step + msgs->fields[0].offset;
        int arrayPosY = i * msgs->point_step + msgs->fields[1].offset;
        int arrayPosZ = i * msgs->point_step + msgs->fields[2].offset;
        int ring = msgs->data[i * msgs->point_step + msgs->fields[4].offset];

        float x;
        float y;
        float z;

        memcpy(&x, &msgs->data[arrayPosX], sizeof(float));
        memcpy(&y, &msgs->data[arrayPosY], sizeof(float));

        pcl::PointXYZ Pt;
        Pt.x = x;
        Pt.y = y;
        Pt.z = std::atan2(y,x);   //degree 
        ring_pcl_cloud[ring].push_back(Pt);

    }

    //merge sort pcl
    for (int i=0;i<8;i++){
        pcl_z_merge_sort(ring_pcl_cloud[i], 0, ring_pcl_cloud[i].size());
    }

    /*
    for (int i=0;i<ring_pcl_cloud[0].size();i++){
        std::cout << ring_pcl_cloud[0].points[i].z << " ";
    }
    std::cout << std::endl;
    */


    pcl::PointXYZ p;
    
    std::cout << "ok:" << pcl_z_binary_search(ring_pcl_cloud[0] ,0.5, p);
    std::cout << " binary search 0.5rad " << " x:" << p.x << " y:" << p.y << " z:" << p.z << std::endl;

    //for (int i=0;i<8;i++)std::cout << ring_pcl_cloud[i].size() << " ";
    //std::cout << std::endl;

    bool ring_filter_map[160][160];
    for (int i=0;i<160;i++){
        for (int j=0;j<160;j++){
            ring_filter_map[i][j] = 0;
        }
    }

    /* 
    for (int i=0;i<3;i++){
        for (int j=0;j<ring_pcl_cloud[i].size();j++){
            double d;
            double d1, d2;
            double x1, x2, y1, y2

            tree[i].radiusSearch(p, radius, k_indices, k_sqr_distances, max_nn);
            if (k_indices.size() == 0)continue;

            x1 = ring_pcl_cloud[i].points[j].x;
            y1 = ring_pcl_cloud[i].points[j].y;
            d1 = std::sqrt(x1*x1 + y1*y1);
            
            x2 = ring_pcl_cloud[i+1].points[k_indices].x;
            y2 = ring_pcl_cloud[i+1].points[k_indices].y;
            d2 = std::sqrt(x2*x2 + y2*y2);

            d = d2 - d1;

            int mx,my;
            if (d/D[i] < 0.1){
                worldToMap(x1, x2, mx, my);
                ring_filter_map[mx][my] = 1;
            }
        }
    }

    for (int i=0;i<160;i++){
        for (int j=0;j<160;j++){
            std::cout << ring_filter_map[i][j] << " ";
        }
        std::cout << std::endl;
    }
    */



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
