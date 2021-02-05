#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/transforms.h>
#include <ros/ros.h>
#include <ros/console.h>
#include <pcl/registration/icp.h>
#include <tf/transform_listener.h>
#include <pcl/registration/icp.h>
#include <geometry_msgs/PointStamped.h>
#include <nav_msgs/Odometry.h>
#include <pcl/filters/passthrough.h>
#include <Eigen/Geometry>
#include <fstream>
#include <math.h>
#include <string>


pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_source (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_target (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_tf (new pcl::PointCloud<pcl::PointXYZ>);
nav_msgs::Odometry localization;
Eigen::Matrix4f initial_guess;
bool first_time = true;
bool score_is_bad = false;
tf::StampedTransform transform;
ros::Publisher pub_1;
ros::Publisher pub_2;
ros::Publisher pub_3;
int count = 1;
int count_gps = 1;
float gps_data[3] = {0};
std::ofstream myfile;

std::string* map_select(float x, float y);
pcl::PCLPointCloud2::Ptr merge_maps(const std::string* map_name);

void lidar_scan_cb (const sensor_msgs::PointCloud2::ConstPtr& pc2_msg);

void gps_cb(const geometry_msgs::PointStamped::ConstPtr& gps);

//void odom_cb (const nav_msgs::Odometry::ConstPtr& odom);

pcl::PointCloud<pcl::PointXYZ> ::Ptr icp_algo(){
  std::cerr << "Start ICP." << std::endl;
  
  pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
  // Set the input source and target
  icp.setInputSource (cloud_source);
  icp.setInputTarget (cloud_target);

  // Set the max correspondence distance to 5cm (e.g., correspondences with higher distances will be ignored) 
  icp.setMaxCorrespondenceDistance (1.0);
  // Set the maximum number of iterations (criterion 1)
  icp.setMaximumIterations (150);
  // Set the transformation epsilon (criterion 2)
  icp.setTransformationEpsilon (1e-16);
  // Set the euclidean distance difference epsilon (criterion 3)
  icp.setEuclideanFitnessEpsilon (1e-4);

  // Perform the alignment
  pcl::PointCloud<pcl::PointXYZ> ::Ptr output_register (new pcl::PointCloud<pcl::PointXYZ>);
  icp.align (*output_register, initial_guess);
  std::cerr << "has converged:" << icp.hasConverged() << " score: " <<
      icp.getFitnessScore() << std::endl;

  // Obtain the transformation that aligned cloud_source to cloud_source_registered
  initial_guess = icp.getFinalTransformation ();
  if(icp.getFitnessScore()>0.75)
    score_is_bad = true;
  
  std::cerr << "ICP was done." << std::endl;

  return output_register;
}

void publish_topic(sensor_msgs::PointCloud2 result_pcl, nav_msgs::Odometry localization_data){
  pub_2.publish (result_pcl);
  pub_3.publish (localization_data);
}

int main (int argc, char** argv)
{
  ros::init (argc, argv, "my_pcl");
  ros::NodeHandle nh;
  ros::Subscriber sub_1 = nh.subscribe ("gps", 500, gps_cb);
  ros::Subscriber sub_2 = nh.subscribe ("lidar_points", 500, lidar_scan_cb);
  //ros::Subscriber sub_3 = nh.subscribe ("wheel_odometry", 500, odom_cb);
  pub_1 = nh.advertise<sensor_msgs::PointCloud2> ("map", 1000);
  pub_2 = nh.advertise<sensor_msgs::PointCloud2> ("lidar_scan", 1000);
  pub_3 = nh.advertise<nav_msgs::Odometry> ("ICP_Localization", 1000);
  
  
  myfile.open("/home/jimmy/catkin_ws/src/localization_competition/test/nuScenes_result1.csv", std::ios::out | std::ios::trunc);
  myfile << "id,x,y,z,yaw,pitch,roll\n";

  tf::TransformListener tf_listener;

  try
  {
    tf_listener.waitForTransform("/car", "/nuscenes_lidar", ros::Time(0), ros::Duration(10.0) );
    tf_listener.lookupTransform("/car", "/nuscenes_lidar", ros::Time(0), transform);
  }
  catch (tf::TransformException &ex) 
  {
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
  }

  //ros::Rate loop_rate(1);
  
  while (count < 395)
  {
    //std::cerr << count << std::endl;
    ros::spinOnce ();
  }
  
  myfile.close();
  std::cerr << "The program is done." << std::endl;
  return (0);
}

void lidar_scan_cb (const sensor_msgs::PointCloud2::ConstPtr& pc2_msg)
{
  std::cerr << "Lidar data are obtained." << std::endl;
  //std::cerr << pc2_msg->header.seq << std::endl;  //Check if the seq of lidar points matches that of GPS
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_F_1 (new pcl::PointCloud<pcl::PointXYZ>);
  

  pcl::fromROSMsg ( *pc2_msg, *cloud_in );
  
  // Passthrough the pointcloud to reject some outliers out of maps
  
  pcl::PassThrough<pcl::PointXYZ> pass_z;
  
  pass_z.setInputCloud (cloud_in);
  pass_z.setFilterFieldName ("z");
  pass_z.setFilterLimits (1.0, 15.0);
  //pass_x.setNegative (true);
  pass_z.filter (*cloud_tf);
  /*
  pass_x.setInputCloud (cloud_F_1);
  pass_x.setFilterFieldName ("x");
  pass_x.setFilterLimits (-10, -4.0);
  pass_x.setNegative (true);
  pass_x.filter (*cloud_tf);
 
  pcl::PassThrough<pcl::PointXYZ> pass_y;
  pass_y.setInputCloud (cloud_F_1);
  pass_y.setFilterFieldName ("y");
  pass_y.setFilterLimits (-30.0, 30.0);
  //pass_y.setNegative (true);
  pass_y.filter (*cloud_tf);
  */
  pcl_ros::transformPointCloud(*cloud_tf, *cloud_source, transform);

  if (first_time){
    initial_guess(0,3) = 1774.84;
    initial_guess(1,3) = 866.367;
    initial_guess(2,3) = 0.015302;
    //0 rad/0 degree
    initial_guess(0,0) = 1;
    initial_guess(0,1) = 0;
    initial_guess(1,1) = 1;
    initial_guess(1,0) = 0;

    initial_guess(0,2) = 0;
    initial_guess(1,2) = 0;
    initial_guess(2,0) = 0;
    initial_guess(2,1) = 0;
    initial_guess(3,0) = 0;
    initial_guess(3,1) = 0;
    initial_guess(3,2) = 0;

    initial_guess(2,2) = 1;
    initial_guess(3,3) = 1;

    first_time = false;
  }

  //pcl_ros::transformPointCloud(*cloud_in, *cloud_source, transform);
  pcl::PCLPointCloud2::Ptr cloud (new pcl::PCLPointCloud2 ());
  sensor_msgs::PointCloud2 pcl_target;

  std::string* maps = map_select(initial_guess(0,3),initial_guess(1,3));
  cloud = merge_maps(maps);
  std::cerr << "PointCloud after filtering: " << cloud->width * cloud->height 
       << " data points (" << pcl::getFieldsList (*cloud) << ")." << std::endl;

  pcl_conversions::fromPCL(*cloud, pcl_target); 
  pcl_target.header.frame_id = "world";
  pcl::fromROSMsg ( pcl_target, *cloud_target );

  pub_1.publish(pcl_target);

  pcl::PointCloud<pcl::PointXYZ> ::Ptr final(new pcl::PointCloud<pcl::PointXYZ>);
  final = icp_algo();

  sensor_msgs::PointCloud2 result;
  pcl::PointCloud<pcl::PointXYZ> ::Ptr output (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::transformPointCloud( *cloud_source, *output, initial_guess);

  pcl::toROSMsg (*output, result);
  result.header.frame_id = "world";

  localization.header.frame_id = "world";
  localization.child_frame_id = "car";

  localization.pose.pose.position.x = initial_guess(0,3);
  localization.pose.pose.position.y = initial_guess(1,3);
  localization.pose.pose.position.z = 0;

  Eigen::Matrix3f Rotation = initial_guess.topLeftCorner<3,3>();
  Eigen::Quaternionf q(Rotation);
  localization.pose.pose.orientation.x = q.x();
  localization.pose.pose.orientation.y = q.y();
  localization.pose.pose.orientation.z = q.z();
  localization.pose.pose.orientation.w = q.w();
  /*
  float yaw = atan2(2.0*(q.x()*q.y() + q.w()*q.z()), -1 + 2.0*(q.w()*q.w() + q.x()*q.x()));
  float pitch = asin(-2.0*(q.x()*q.z() - q.w()*q.y()));
  float roll = atan2(2.0*(q.z()*q.y() + q.w()*q.x()), 1 - 2.0*(q.x()*q.x() + q.y()*q.y()));
  */
  Eigen::Vector3f euler = q.toRotationMatrix().eulerAngles(2, 1, 0);
  float yaw = euler[0]; 
  float pitch = euler[1]; 
  float roll = euler[2];

  std::cerr << count << std::endl;
  myfile << count << "," << localization.pose.pose.position.x << "," << localization.pose.pose.position.y << "," << 
  localization.pose.pose.position.z << "," << yaw << "," << pitch << "," << roll << "\n";
  count++;

  
  float pos_error = sqrt(pow(initial_guess(0,3) - gps_data[0],2) + pow(initial_guess(1,3) - gps_data[1],2) + 
                    pow(initial_guess(2,3) - gps_data[2],2));
  if (pos_error > 5 && abs(count - count_gps) < 3)
    score_is_bad = true;
  
  
  publish_topic(result, localization);


  
}

void gps_cb(const geometry_msgs::PointStamped::ConstPtr& gps)
{
  std::cerr << "GPS data are obtained." << std::endl;
  //store the most recent GPS data
  gps_data[0] = gps->point.x;
  gps_data[1] = gps->point.y;
  gps_data[2] = gps->point.z;
  count_gps++;
  if(first_time){
    std::cerr << gps->point.x << ";" << gps->point.y << ";" << gps->point.z <<std::endl; 
    //std::cerr << gps->header.seq<<std::endl;  //Check if the sequence matches that of lidar points.
    initial_guess(0,3) = gps->point.x;
    initial_guess(1,3) = gps->point.y;
    initial_guess(2,3) = gps->point.z;
    //0 rad/0 degree
    initial_guess(0,0) = 1;
    initial_guess(0,1) = 0;
    initial_guess(1,1) = 1;
    initial_guess(1,0) = 0;

    initial_guess(0,2) = 0;
    initial_guess(1,2) = 0;
    initial_guess(2,0) = 0;
    initial_guess(2,1) = 0;
    initial_guess(3,0) = 0;
    initial_guess(3,1) = 0;
    initial_guess(3,2) = 0;

    initial_guess(2,2) = 1;
    initial_guess(3,3) = 1;

    first_time = false;
  }
  if(score_is_bad){
    initial_guess(0,3) = gps_data[0];
    initial_guess(1,3) = gps_data[1];
    initial_guess(2,3) = gps_data[2];
    score_is_bad = false;
  }
}
/*
void odom_cb (const nav_msgs::Odometry::ConstPtr& odom){
  std::cerr<<"Odom data are obtained."<<std::endl;
  Eigen::Quaternionf q_correct(odom->pose.pose.orientation.w, 
                               odom->pose.pose.orientation.x,
                               odom->pose.pose.orientation.y,
                               odom->pose.pose.orientation.z);
  Eigen::Matrix3f rot_correct = q_correct.normalized().toRotationMatrix();
  initial_guess.topLeftCorner<3,3>() = rot_correct;
}
*/
