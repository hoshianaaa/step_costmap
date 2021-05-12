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
#include <pcl/kdtree/kdtree_flann.h>

#include <cmath>


/* 
---- map ----
diff_map: diff max height and around min height

-------------
*/

std::string DEBUG_MAP = "diff_map";

/*
---- debug mode : how to output difference debug cloud map value -----
color : change color
height : change
----------------------------------------------------------------------
*/

std::string DEBUG_MODE = "color";

#define MIN_Z 0
#define MAX_Z 1
#define HAVE_VALUE 2

#define LOW 1
#define MIDDLE 2
#define HIGH 3 
#define ROAD 4
#define NONE 0

#define FOR(i,a,b) for(int i = (a); i < (b); ++i)
#define rep(i,n) FOR(i,0,n)

class StepCostmap
{
public:
	StepCostmap();
private:
    ros::NodeHandle nh_;
    ros::Subscriber cloud_sub_;
    ros::Publisher high_cloud_pub_;
    ros::Publisher deleate_cloud_pub_;
    ros::Publisher low_cloud_pub_;
    ros::Publisher road_cloud_pub_;
    ros::Publisher ring_cloud_pub_;
	  ros::Publisher nowOGM_cloud_pub_;
	  ros::Publisher step_cloud_pub_;
	  ros::Publisher debug_cloud_pub_;
	  tf::TransformListener tf_listener_;
    void pcl_z_merge_sort(pcl::PointCloud<pcl::PointXYZ>& pcl_cloud, int left, int right);
    void pcl_z_merge(pcl::PointCloud<pcl::PointXYZ>& pcl_cloud, int left, int mid, int right);
    bool pcl_z_binary_search(const pcl::PointCloud<pcl::PointXYZ>& pcl_cloud, const double z, pcl::PointXYZ& near_point);//0.0017rad = 0.1deg
    bool mapToWorld(const int map_x, const int map_y, double& world_x, double& world_y);
    bool worldToMap(const double world_x, const double world_y, int& map_x, int& map_y);
    double road_probability(const int map_x, const int map_y);
	void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& msgs);
	costmap_2d::Costmap2D costmap_;
    unsigned char probToCost(const double prob);
    double costToProb(unsigned char cost);
	std::string sensor_frame_, topic_name_;
	double sensor_range_x_min_, sensor_range_x_max_, sensor_range_y_min_, sensor_range_y_max_, sensor_range_z_min_, sensor_range_z_max_;
    double z_th_;

    void mapToPointCloud(double map[][160], pcl::PointCloud<pcl::PointXYZI>& pcl_cloud);
    void debugMapPublish(double map[][160]);

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

	private_nh.param("z_th", z_th_, 0.05);
	
	costmap_.setDefaultValue(probToCost(0));
	costmap_.resizeMap(160, 160, 0.1, 0, 0);
	

    high_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/high_cloud", 1, false);
    deleate_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/deleate_cloud", 1, false);
    low_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/low_cloud", 1, false);
	road_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/road_cloud", 1, false);
	ring_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/ring_cloud", 1, false);
	nowOGM_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/ogm", 1, false);
	step_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/step_costmap", 1, false);
	debug_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/debug_map", 1, false);
	cloud_sub_ = nh_.subscribe(topic_name_, 1, &StepCostmap::cloudCallback, this);
}

void StepCostmap::mapToPointCloud(double map[][160], pcl::PointCloud<pcl::PointXYZI>& pcl_cloud)
{

  pcl::PointCloud<pcl::PointXYZI> cloud;
  pcl::PointXYZI p;

  double wx,wy;

  rep(i, 160){
    rep(j, 160){
      mapToWorld(i,j,wx,wy);
      p.x = wx;
      p.y = wy;

      if (DEBUG_MODE=="color")
      {
        p.z = 0;
        p.intensity = map[i][j];
      }
      else
      {
        p.z = map[i][j];
        p.intensity = 0;
      }

      cloud.push_back(p);
    }
  }

  pcl_cloud = cloud;

  return;
}

void StepCostmap::debugMapPublish(double map[][160])
{
  pcl::PointCloud<pcl::PointXYZI> cloud;
  mapToPointCloud(map,cloud);

  sensor_msgs::PointCloud2 ros_cloud;
  pcl::toROSMsg(cloud, ros_cloud);

  ros_cloud.header.frame_id = sensor_frame_;
  debug_cloud_pub_.publish(ros_cloud);
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

    const double range = 0.017;
    double d = 100.0;
    int m, r, l;

    r = 0;
    l = pcl_cloud.size();

    while (1){
        m = (r + l)/2;

        d = std::abs(pcl_cloud.points[m].z - z);
        if (d < range)break;

        if ( pcl_cloud.points[m].z > z)l = m;
        else r = m;
        
        if (r+1 >= l)return 0;
    }
    near_point = pcl_cloud.points[m];
    return 1;
}


bool StepCostmap::mapToWorld(const int map_x, const int map_y, double& world_x, double& world_y){
    world_x = map_x * 0.1 - 8.0 + 0.1/2;
    world_y = map_y * 0.1 - 8.0 + 0.1/2;
    if (world_x > 8.0 || world_x < -8.0  || world_y > 8.0 || world_y < -8.0)return 0;
    return 1;
}

bool StepCostmap::worldToMap(const double world_x, const double world_y, int& map_x, int& map_y){
    map_x = int((world_x + 8.0) * 10);
    map_y = int((world_y + 8.0) * 10);
    if (map_x < 0 || map_x > 159 || map_y < 0 || map_y > 159)return 0;
    return 1;
}

double StepCostmap::road_probability(const int map_x, const int map_y){
    double world_x, world_y;
    double d;
    double prob;

    /*
    mapToWorld(map_x, map_y, world_x, world_y);
    d = std::sqrt(world_x * world_x + world_y + world_y);
    prob = (0.3 * d / (8 * std::sqrt(2.0))+0.3); 
    if (prob < 0)prob = 0.5;
    if (prob > 0.5)prob = 0.5;
    if (std::isnan(prob))prob = 0.5;
    */

    prob = 0.4;
    return prob;
} 

unsigned char StepCostmap::probToCost(const double prob){
    // -1~1 -> 0~255
    if (prob < -1)return 0;
    if (prob > 1)return 255;
    return int((int)(255/2)*prob+(int)(255/2));
}

double StepCostmap::costToProb(unsigned char cost){
    // 0~255 -> -1~1
    if (cost > 255){
        return 1.0;
    }

    if (cost < 0){
        return -1.0;
    }
    return ((double)cost-(int)(255/2))/(int(255/2));
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

    //filter1:deleate near point cloud to sensor 
    /*
    const double filter1_distance = 1.5;
	  pcl::PointCloud<pcl::PointXYZI> filtered_pcl_cloud;

    for (int i=0;i<pcl_cloud.size();i++){
        double px, py;
        double d;

        px = pcl_cloud.points[i].x;
        py = pcl_cloud.points[i].y;

        d = std::sqrt(px*px + py*py);

        if (d > filter1_distance){
            pcl::PointXYZI p;
            p.x = px;
            p.y = py;
            p.z = pcl_cloud.points[i].z;
            filtered_pcl_cloud.push_back(p);
        }
    }
    */

    //filter2:deleate high object like a human
    //pcl::PointCloud<pcl::PointXYZI> filtered_pcl_cloud;
    /*
    pcl::PointCloud<pcl::PointXYZI> high_pcl_cloud;

    pass.setInputCloud (filtered_pcl_cloud.makeShared());
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (0.3, 1.0);
    pass.filter (high_pcl_cloud);

    sensor_msgs::PointCloud2 high_cloud;
    pcl::toROSMsg(high_pcl_cloud, high_cloud);

    high_cloud.header.frame_id = sensor_frame_;
    high_cloud_pub_.publish(high_cloud);

    std::cout << "high_pcl_cloud size:" << high_pcl_cloud.size() << std::endl;

    bool deleate_map[160][160];
    int deleate_map_counter; 

    for (int i=0;i<160;i++){
        for (int j=0;j<160;j++){
            deleate_map[i][j] = 0;
        }
    }

    for (int i=0;i<pcl_cloud.size();i++){
        int mx,my;
        double wx,wy;

        wx = pcl_cloud.points[i].x;
        wy = pcl_cloud.points[i].y;
        if(worldToMap(wx, wy, mx, my)){
          deleate_map[mx][my] = 1;
          //std::cout << "wx:" << wx << " wy:" << wy << " mx:" << mx << " my:" << my << std::endl;
        }
    }

    deleate_map_counter = 0;
    for (int i=0;i<160;i++){
        for (int j=0;j<160;j++){
            if (deleate_map[i][j] == 1)deleate_map_counter++;
        }
    }
    std::cout << "deleate_map num(before expand):" << deleate_map_counter << std::endl;
 
    //expand region deleate point
    bool expanded_deleate_map[160][160];
    for (int i=0;i<160;i++){
        for (int j=0;j<160;j++){
            expanded_deleate_map[i][j] = 0;
        }
    }

    const int expand_num = 3;
    for (int i=0;i<160;i++){
        for (int j=0;j<160;j++){
            if (deleate_map[i][j] == 1){
                for (int k=0;k<expand_num*2+1;k++){
                    for (int l=0;l<expand_num*2+1;l++){
                            int x,y;
                            x = i - expand_num + k;
                            y = j - expand_num + l;
                            if (x<0 || x>160 || y<0 || y>160)continue;
                            expanded_deleate_map[x][y] = 1;
                    }
                }
            }
        }
    }

    deleate_map_counter = 0;
    for (int i=0;i<160;i++){
        for (int j=0;j<160;j++){
            if (expanded_deleate_map[i][j] == 1)deleate_map_counter++;
        }
    }
    std::cout << "deleate_map num(after expand):" << deleate_map_counter << std::endl;

    pcl::PointCloud<pcl::PointXYZ> deleate_pcl_cloud;
    for (int i=0;i<160;i++){
        for (int j=0;j<160;j++){
            if (expanded_deleate_map[i][j] == 1){
                double wx, wy;
                pcl::PointXYZ p;
                mapToWorld(i,j,wx,wy);
                p.x = wx;
                p.y = wy;
                p.z = 0;
                deleate_pcl_cloud.push_back(p);
                //std::cout << "expand deleate map wx:" << wx << " wy:" << wy << " mx:" << mx << " my:" << my << std::endl;
            }
        }
    }

    sensor_msgs::PointCloud2 deleate_cloud;
    pcl::toROSMsg(deleate_pcl_cloud, deleate_cloud);
    deleate_cloud.header.frame_id = sensor_frame_;

    deleate_cloud_pub_.publish(deleate_cloud);
    */

    //region_pcl
    double height_map[160][160][3];//min_z,max_z,have_value
	  pcl::PointCloud<pcl::PointXYZI> region_cloud1;
	  pcl::PointCloud<pcl::PointXYZI> region_cloud2;
    pcl::PointXYZI minPt, maxPt;

    for (int i=0;i<160;i++){

        double min_x = -8.0 + 0.1*i;

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
        }
    }

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
        }
    }

    if(DEBUG_MAP == "diff_map")debugMapPublish(diff_map);

    double type_map[160][160];
    for (int i=0;i<160;i++){
        for (int j=0;j<160;j++){
            double diff_value = diff_map[i][j];
            //if (expanded_deleate_map[i][j] == 1)type_map[i][j] = NONE;
            if(diff_value < 0)type_map[i][j] = -1; //error
            else if (diff_value == 0)type_map[i][j] = NONE;
            else if (0 < diff_value && diff_value < z_th_)type_map[i][j] = ROAD;
            else type_map[i][j] = LOW;
            //else if (diff_value < 0.3)type_map[i][j] = LOW;
            //else if (diff_value < 0.4)type_map[i][j] = MIDDLE;
        }
    }

    

    //** publish low object **//

	pcl::PointCloud<pcl::PointXYZ> low_pcl_cloud;
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
                //std::cout << "low map wx:" << world_x << " wy:" << world_y << " mx:" << i << " my:" << j << std::endl;
                one_point_pcl_cloud.points[0].x = world_x;
                one_point_pcl_cloud.points[0].y = world_y;
                one_point_pcl_cloud.points[0].z = 0;
                low_pcl_cloud += one_point_pcl_cloud;
            }
        }
    }

    sensor_msgs::PointCloud2 low_cloud;
    pcl::toROSMsg(low_pcl_cloud, low_cloud);
    low_cloud.header.frame_id = sensor_frame_;
    low_cloud_pub_.publish(low_cloud);

    //** publish road object **//
	pcl::PointCloud<pcl::PointXYZ> road_pcl_cloud;
    
    for (int i=0;i<160;i++){
        for (int j=0;j<160;j++){
            if(type_map[i][j] == ROAD){
                double world_x,world_y;
                mapToWorld(i,j,world_x,world_y);
                one_point_pcl_cloud.points[0].x = world_x;
                one_point_pcl_cloud.points[0].y = world_y;
                one_point_pcl_cloud.points[0].z = 0;
                road_pcl_cloud += one_point_pcl_cloud;
            }
        }
    }

    sensor_msgs::PointCloud2 road_cloud;
    pcl::toROSMsg(road_pcl_cloud, road_cloud);
    road_cloud.header.frame_id = sensor_frame_;
    road_cloud_pub_.publish(road_cloud);
 
    //** occupancy grid map **//
    double nowOGM[160][160];
    double prob;
    for (int i=0;i<160;i++){
        for (int j=0;j<160;j++){
            if (type_map[i][j] == LOW){nowOGM[i][j] = std::log(0.7/(1-0.7));
            }else if (type_map[i][j] == ROAD){
                prob = road_probability(i,j);
                if (prob >= 0.5)prob = 0.5;
                nowOGM[i][j] = std::log(prob/(1-prob));
            }else{nowOGM[i][j] = std::log(0.5/(1-0.5));}
          
            if (nowOGM[i][j] < -1)nowOGM[i][j] = -1;
            if (nowOGM[i][j] > 1)nowOGM[i][j] = 1;
        }
    }
    
    
    /*
    for (int i=0;i<160;i++){
        for (int j=0;j<160;j++){
            if (nowOGM[i][j] < 0)std::cout << (double)nowOGM[i][j] << " ";
        }
    }
    std::cout << std::endl;
    */

	pcl::PointCloud<pcl::PointXYZI> now_ogm_pcl_cloud;
    now_ogm_pcl_cloud.width = 160*160;
    now_ogm_pcl_cloud.height = 1;
    now_ogm_pcl_cloud.is_dense = false;
    now_ogm_pcl_cloud.resize (160*160);


    for (int i=0;i<160;i++){
        for (int j=0;j<160;j++){
                double world_x,world_y;
                mapToWorld(i,j,world_x,world_y);
                now_ogm_pcl_cloud.points[160*i+j].x = world_x;
                now_ogm_pcl_cloud.points[160*i+j].y = world_y;
                now_ogm_pcl_cloud.points[160*i+j].z = 0;
                now_ogm_pcl_cloud.points[160*i+j].intensity= nowOGM[i][j];
        }
    }


    sensor_msgs::PointCloud2 now_ogm_cloud;
    pcl::toROSMsg(now_ogm_pcl_cloud, now_ogm_cloud);

    now_ogm_cloud.header.frame_id = sensor_frame_;
    sensor_msgs::PointCloud2 now_ogm_cloud_odom;


    try
    {
        tf::StampedTransform trans;
        tf_listener_.waitForTransform("odom", sensor_frame_, ros::Time(0), ros::Duration(0.5));
        tf_listener_.lookupTransform("odom", sensor_frame_, ros::Time(0), trans);
        pcl_ros::transformPointCloud("odom", trans, now_ogm_cloud, now_ogm_cloud_odom);
    }
    catch(tf::TransformException &e)
    {
        ROS_WARN("%s", e.what());
    }

   
    now_ogm_cloud.header.frame_id = "odom";
    nowOGM_cloud_pub_.publish(now_ogm_cloud_odom);
    
 
	pcl::PointCloud<pcl::PointXYZI> now_ogm_pcl_cloud_odom;
    pcl::fromROSMsg (now_ogm_cloud_odom, now_ogm_pcl_cloud_odom); 

    int low_counter = 0;
    int road_counter = 0;
    for (int i=0;i<now_ogm_pcl_cloud_odom.size();i++){
        prob = now_ogm_pcl_cloud_odom.points[i].intensity;
        if (prob < 0.0)road_counter++;
        if (prob > 0.0)low_counter++;
    }
    std::cout << "r:" << road_counter <<" ";
    std::cout << "l:" << low_counter << std::endl;

   
    for (int i=0;i<now_ogm_pcl_cloud_odom.size();i++){
        unsigned int mx,my;
        unsigned char cost;
        
        double prob = now_ogm_pcl_cloud_odom.points[i].intensity;

        if(costmap_.worldToMap(now_ogm_pcl_cloud_odom.points[i].x, now_ogm_pcl_cloud_odom.points[i].y, mx, my)){
            //if (prob > 0)std::cout << " cp:"<<costToProb(costmap_.getCost(mx, my)) << "+"<< "p:"<<now_ogm_pcl_cloud_odom.points[i].intensity;

            //if ((int)costmap_.getCost(mx,my)>127)std::cout <<"c:"<< (int)costmap_.getCost(mx,my)<<" p:"<<prob;

            cost =  probToCost(costToProb(costmap_.getCost(mx, my)) + now_ogm_pcl_cloud_odom.points[i].intensity);
            //if ((int)costmap_.getCost(mx,my)>127)std::cout <<" after_c:"<<(int)cost<<std::endl;
            //if (prob > 0)std::cout <<" ="<<(int)cost<<std::endl;
            //if (prob > 0)std::cout << " ="<<costToProb(costmap_.getCost(mx, my)) + now_ogm_pcl_cloud_odom.points[i].intensity<< std::endl;
            costmap_.setCost(mx, my, cost);
        }
    }
    //std::cout << std::endl;
    /*
    for (int i=-255;i<500;i++){
        std::cout << i << ":";
        std::cout << costToProb(i) << std::endl;
    }
    */

    std::cout << "cost"<< (int)probToCost(0.5) << std::endl;

	pcl::PointCloud<pcl::PointXYZ> step_pcl_cloud;
    pcl::PointCloud<pcl::PointXYZ> one_point_pcl_cloud3;
    one_point_pcl_cloud3.width = 1;
    one_point_pcl_cloud3.height = 1;
    one_point_pcl_cloud3.is_dense = false;
    one_point_pcl_cloud3.resize (1);


    for (int i=0;i<160;i++){
        for (int j=0;j<160;j++){
 //           std::cout << (int)costmap_.getCost(i,j) << " ";
            if(costmap_.getCost(i,j) >= probToCost(0.5)){
                double world_x,world_y;
                costmap_.mapToWorld(i,j,world_x,world_y);
                one_point_pcl_cloud3.points[0].x = world_x;
                one_point_pcl_cloud3.points[0].y = world_y;
                one_point_pcl_cloud3.points[0].z = 0.2;
                step_pcl_cloud += one_point_pcl_cloud3;
            }
        }
//       std::cout << std::endl;
    }

    sensor_msgs::PointCloud2 step_cloud;
    pcl::toROSMsg(step_pcl_cloud, step_cloud);

    try
    {
        tf::StampedTransform trans;
        tf_listener_.waitForTransform(sensor_frame_, "odom", ros::Time(0), ros::Duration(0.5));
        tf_listener_.lookupTransform(sensor_frame_, "odom", ros::Time(0), trans);
        pcl_ros::transformPointCloud(sensor_frame_, trans, step_cloud, step_cloud);
    }
    catch(tf::TransformException &e)
    {
        ROS_WARN("%s", e.what());
    }


    step_cloud.header.frame_id = sensor_frame_;
    step_cloud_pub_.publish(step_cloud);

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
