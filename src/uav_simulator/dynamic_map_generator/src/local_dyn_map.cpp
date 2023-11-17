#include <ros/console.h>
#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <Eigen/Eigen>
#include <math.h>
#include <string>
#include <iostream>

using namespace std;

ros::Publisher local_map_pub_;
ros::Publisher fov_pub;
ros::Subscriber global_map_sub_;
ros::Subscriber odom_sub_;
tf::StampedTransform globalToLocalTransform;
tf::TransformListener* listener;

std::string local_frame = "base";  // 设置local_frame坐标系的名称
std::string world_frame = "world";  // 设置world_frame坐标系的名称
//
Eigen::Vector3d uav_pos_;
Eigen::Quaternionf uav_att_; 
sensor_msgs::PointCloud2 local_map_msg_;
sensor_msgs::PointCloud2 global_map_msg_;
std::vector<geometry_msgs::Point> pyramid_;// for local point cloud

//param for local map;
double fov_width;  // Field of view width in meters
double fov_height; // Field of view height in meters
double fov_depth;  // Field of view depth in meters


//func
void transformPointsToWorldFrame(std::vector<geometry_msgs::Point>& pyramid_, const std::string& local_frame, const std::string& world_frame) 
{  
    try {
        listener->lookupTransform("world", "base", ros::Time(0), globalToLocalTransform);
    } catch (tf::TransformException &ex) {
        ROS_WARN("Failed to lookup transform: %s", ex.what());
        return;
    }
    for (auto& point : pyramid_) {
        tf::Vector3 transformed_point(point.x, point.y, point.z);
        transformed_point = globalToLocalTransform * transformed_point;
        point.x = transformed_point.x();
        point.y = transformed_point.y();
        point.z = transformed_point.z();
    }
}

void fov_vis(const double fov_width, const double fov_height, const double fov_depth)
{
    visualization_msgs::Marker fov_marker;
    fov_marker.header.frame_id = local_frame;
    fov_marker.header.stamp = ros::Time::now();
    fov_marker.ns = "fov";
    fov_marker.action = visualization_msgs::Marker::ADD;
    fov_marker.type = visualization_msgs::Marker::LINE_LIST;  
    fov_marker.scale.x = 0.1;
    fov_marker.scale.y = 0.1;
    fov_marker.scale.z = 0.1;
    fov_marker.color.a = 0.5;  
    fov_marker.color.r = 0.0;
    fov_marker.color.g = 0.0;
    fov_marker.color.b = 0.0;

    geometry_msgs::Point vertex1, vertex2, vertex3, vertex4, vertex5;
    vertex1.x = 0;
    vertex1.y = 0;
    vertex1.z = 0;

    vertex2.x = fov_depth;
    vertex2.y = fov_width / 2;
    vertex2.z = fov_height / 2;

    vertex3.x = fov_depth;
    vertex3.y = -fov_width / 2;
    vertex3.z = fov_height / 2;

    vertex4.x = fov_depth;
    vertex4.y = fov_width / 2;
    vertex4.z = -fov_height / 2;

    vertex5.x = fov_depth;
    vertex5.y = -fov_width / 2;
    vertex5.z = -fov_height / 2;

    // Add vertices to the marker
    fov_marker.points.push_back(vertex1);
    fov_marker.points.push_back(vertex2);
    fov_marker.points.push_back(vertex1);
    fov_marker.points.push_back(vertex3);
    fov_marker.points.push_back(vertex1);
    fov_marker.points.push_back(vertex4);
    fov_marker.points.push_back(vertex1);
    fov_marker.points.push_back(vertex5);
    fov_marker.points.push_back(vertex2);
    fov_marker.points.push_back(vertex3);
    fov_marker.points.push_back(vertex2);
    fov_marker.points.push_back(vertex4);
    fov_marker.points.push_back(vertex3);
    fov_marker.points.push_back(vertex5);
    fov_marker.points.push_back(vertex4);
    fov_marker.points.push_back(vertex5);

    fov_pub.publish(fov_marker);

    pyramid_.clear();
    pyramid_.push_back(vertex1);
    pyramid_.push_back(vertex2);
    pyramid_.push_back(vertex3);
    pyramid_.push_back(vertex4);
    pyramid_.push_back(vertex5);
    transformPointsToWorldFrame(pyramid_, local_frame, world_frame);

    // for (size_t i = 0; i< pyramid_.size(); i++){
    // std::cout << "Vertex: (" << pyramid_[i].x << ", " << pyramid_[i].y << ", " << pyramid_[i].z << ")" << std::endl;}
    // for (const auto& vertex : pyramid_) {
    // std::cout << "Vertex: (" << vertex.x << ", " << vertex.y << ", " << vertex.z << ")" << std::endl;}

}

void simPoseCallback(const geometry_msgs::PoseStamped &msg)
{
    uav_pos_(0) = msg.pose.position.x;
    uav_pos_(1) = msg.pose.position.y;
    uav_pos_(2) = msg.pose.position.z;
    uav_att_.w() = msg.pose.orientation.w;
    uav_att_.x() = msg.pose.orientation.x;
    uav_att_.y() = msg.pose.orientation.y;
    uav_att_.z() = msg.pose.orientation.z;
    fov_vis(fov_width, fov_height, fov_depth);
}

void cloudCallback(const sensor_msgs::PointCloud2& cloud)
{
    global_map_msg_ = cloud;
}

geometry_msgs::Point crossProduct(const geometry_msgs::Point& AB, const geometry_msgs::Point& AC) {
    geometry_msgs::Point result;

    result.x = AB.y * AC.z - AB.z * AC.y;
    result.y = AB.z * AC.x - AB.x * AC.z;
    result.z = AB.x * AC.y - AB.y * AC.x;

    return result;
}

float calcdotProduct(const geometry_msgs::Point& A, const geometry_msgs::Point& B) {
    return A.x * B.x + A.y * B.y + A.z * B.z;
}

geometry_msgs::Point calcvector(const geometry_msgs::Point& A, const geometry_msgs::Point& B) {
    geometry_msgs::Point result;

    result.x = B.x - A.x;
    result.y = B.y - A.y;
    result.z = B.z - A.z;

    return result;
}

bool isInsidePyramid(const pcl::PointXYZ& point, const std::vector<geometry_msgs::Point>& pyramid) 
{   
    // Check if the pyramid has enough points
    if (pyramid.size()!=5)
    {
        ROS_ERROR("MISS POINTS, RETURN!");
        return false;
    }
    // cout << "-------" <<endl;
    // cout << point <<endl;

    // Calculate normal vector of the base
    geometry_msgs::Point O = pyramid[0];
    geometry_msgs::Point A = pyramid[1];
    geometry_msgs::Point B = pyramid[3];
    geometry_msgs::Point C = pyramid[2];
    geometry_msgs::Point D = pyramid[4];
    geometry_msgs::Point P;   
    P.x = point.x; P.y =  point.y; P.z = point.z;
   
    // Calculate normal vector of the base
    geometry_msgs::Point AB = calcvector(A,B);
    geometry_msgs::Point AC = calcvector(A,C);
    geometry_msgs::Point normal = crossProduct(AB,AC);
  
    // Check if point is on the same side of the base
    geometry_msgs::Point AP = calcvector(A,P);

    float dotProduct = calcdotProduct(AP,normal);
    if (dotProduct < 0) {
        return false;  // Point is on the opposite side of the base
    }

    // Check if point is inside the four triangular sides
    //  A ... C
    //  .  O  .
    //  B ... D
    geometry_msgs::Point OA,OB,OC,OD,OP;
    OA = calcvector(O,A);
    OB = calcvector(O,B);
    OC = calcvector(O,C);
    OD = calcvector(O,D);
    OP = calcvector(O,P);

    geometry_msgs::Point crossOAB = crossProduct(OB,OA);
    geometry_msgs::Point crossOAC = crossProduct(OA,OC);
    geometry_msgs::Point crossOCD = crossProduct(OC,OD);
    geometry_msgs::Point crossOBD = crossProduct(OD,OB);

    if(calcdotProduct(crossOAB,OP)>0 && calcdotProduct(crossOAC,OP)>0 
        && calcdotProduct(crossOCD,OP)>0 && calcdotProduct(crossOBD,OP)>0)
        return true;
    else
        return false;

}

void updatelocalmap()
{
    //update local map
    pcl::PointCloud<pcl::PointXYZ> global_cloud;
    pcl::fromROSMsg(global_map_msg_, global_cloud);
    pcl::PointCloud<pcl::PointXYZ> local_cloud;

    for (const auto& point : global_cloud.points) {
        if (isInsidePyramid(point, pyramid_)) {
            local_cloud.push_back(point);
        }
    }

    pcl::toROSMsg(local_cloud, local_map_msg_);
    local_map_msg_.header = global_map_msg_.header;
    local_map_msg_.header.frame_id = "world";  
    // pub
    local_map_pub_.publish(local_map_msg_);

    // pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
    // voxel_filter.setInputCloud(local_cloud.makeShared());
    // voxel_filter.setLeafSize(0.1, 0.1, 0.1); 
    // voxel_filter.filter(local_cloud);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "local_click_map");
  ros::NodeHandle nh("~");
  ros::Rate loop_rate(20.0);
    
    while (!ros::isInitialized()) {
        ros::spinOnce();
        loop_rate.sleep();
    }

  // Get parameters for fov size
  nh.param("fov_width", fov_width, 10.0);
  nh.param("fov_height", fov_height, 4.8);
  nh.param("fov_depth", fov_depth, 5.0);    

  listener = new tf::TransformListener();
  global_map_sub_ = 
      nh.subscribe("/map_generator/click_map", 1, cloudCallback);
  odom_sub_ = 
      nh.subscribe("/odom_visualization/pose", 1, simPoseCallback);

  local_map_pub_ =
      nh.advertise<sensor_msgs::PointCloud2>("/map_generator/local_click_map", 10);
  fov_pub = 
      nh.advertise<visualization_msgs::Marker>("/fov", 1);
  

  ROS_INFO("\033[1;32m Ready to pub local map now.  \033[0m");

    //loop
    while (ros::ok()){
        updatelocalmap();
        ros::spinOnce();
        loop_rate.sleep();
    }
}
