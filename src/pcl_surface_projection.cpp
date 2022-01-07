#include <iostream>
#include <ros/ros.h>
#include <vector>
#include <math.h>

#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h> //for sensor_msgs::PointCloud2
#include <geometry_msgs/TransformStamped.h>

#include <pcl/point_types.h> // for pcl::PointXYZ
#include <pcl_conversions/pcl_conversions.h> // for pcl::fromROSMsg()

#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
// #include <pcl_ros/transforms.hpp>


#include <pcl/segmentation/sac_segmentation.h> //for SACSegmentation
#include <pcl/filters/extract_indices.h> //for ExtractIndices
#include <pcl/filters/statistical_outlier_removal.h>

#include <tf2_ros/transform_listener.h>
#include <tf/transform_listener.h>



#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>


#include <Eigen/Dense>

ros::Publisher org_low_frq_pub;
ros::Publisher processed_pcl_pub;

// with PointCloud2Ptr not working, but with PointCloud2::ConstPtr it works 
// void sync_callback(const boost::shared_ptr<const sensor_msgs::PointCloud2>& pcl_in, const nav_msgs::Odometry::ConstPtr &pose_in)


void sync_callback(const  sensor_msgs::PointCloud2::ConstPtr &pcl_in, const nav_msgs::Odometry::ConstPtr &pose_in)
{
  // Solve all of perception here...
  ROS_INFO("\n\nrcvd msg ... ");

  ROS_INFO_STREAM("pcl  time stamp: "<< pcl_in->header.stamp);
  ROS_INFO_STREAM("pose time stamp: "<< pose_in->header.stamp); 


  //convert the input into a pcl::PointCloud< pcl::PointXYZ> object
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloudPtr(new pcl::PointCloud<pcl::PointXYZI>);


  // variables  for filtered/transformed/low_frq  pcl 
  pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_cloudPtr(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::fromROSMsg(*pcl_in, *cloudPtr);



  //*********** pcl to std vector ***********
  pcl::PointCloud<pcl::PointXYZI> org_pcl, fr_pcl, sph_pcl, cart_pcl;
  std::copy(cloudPtr->points.begin(), cloudPtr->points.end(),std::back_inserter(org_pcl));



  // ROS_INFO_STREAM("hight: "<< pcl_in->height);
  // ROS_INFO_STREAM("width: "<< pcl_in->width);
  // ROS_INFO_STREAM("point_step: "<< pcl_in->point_step);
  // ROS_INFO_STREAM("row_step: "<< pcl_in->row_step);
  // ROS_INFO_STREAM("cloudPtr [0]: "<< cloudPtr->points.size());
  // ROS_INFO_STREAM("org_pcl: "<< org_pcl.size() );
  // ROS_INFO_STREAM("org_pcl : 1st pt: "<<org_pcl[0].x<<", "<< org_pcl[0].y<<", "<<org_pcl[0].z<<", "<<org_pcl[0].intensity);


  //*********** clustering ***********
  // next step



  //*********** from cartesian to spherical ***********
  // dist = np.sqrt(x**2 + y**2 + z**2)
  // theta = np.arctan2(y, x)
  // alpha = np.arccos(z / dist)

  std::vector<pcl::PointXYZI> sph_pcl_vec, cart_pcl_vec;
  float rds = 5.0, rds1= 5.5; // surface radius
  for(auto pt: org_pcl){
    // ROS_INFO_STREAM("pt: " << pt.x);
    float dst = sqrt(pt.x*pt.x + pt.y*pt.y + pt.z*pt.z );
    float th = atan2(pt.y , pt.x );
    float al = acos(pt.z / dst );
    pcl::PointXYZI shp_pt;
    shp_pt.x = th;
    shp_pt.y = al;
    shp_pt.z = rds;
    shp_pt.intensity = dst;
    sph_pcl_vec.push_back(shp_pt);

    float x = rds1 * sin(al) * cos(th);
    float y = rds1 * sin(al) * sin(th);
    float z = rds1 * cos(al); 
    pcl::PointXYZI cart_pt;
    cart_pt.x = x;
    cart_pt.y = y;
    cart_pt.z = z;
    cart_pt.intensity = dst;
    cart_pcl_vec.push_back(cart_pt);
  }
  ROS_INFO_STREAM("sph_pcl_vec: "<< sph_pcl_vec.size() );
  ROS_INFO_STREAM("cart_pcl_vec: "<< cart_pcl_vec.size() );




  //*********** std vector to pcl ***********
  std::copy(sph_pcl_vec.begin(), sph_pcl_vec.end(),std::back_inserter(sph_pcl));
  sph_pcl.header.frame_id = "velodyne"; 

  std::copy(cart_pcl_vec.begin(), cart_pcl_vec.end(),std::back_inserter(cart_pcl));
  cart_pcl.header.frame_id = "velodyne"; 

  ROS_INFO_STREAM("sph_pcl: "<< sph_pcl.size());
  ROS_INFO_STREAM("cart_pcl: "<< cart_pcl.size());


  //*********** from velodyne to world frame ***********
  // Eigen::Quaternionf rotation(pose_in->pose.pose.orientation.w, pose_in->pose.pose.orientation.x, pose_in->pose.pose.orientation.y,
                              // pose_in->pose.pose.orientation.z);    
  // Eigen::Vector3f origin(pose_in->pose.pose.position.x, pose_in->pose.pose.position.y, pose_in->pose.pose.position.z+0.322);
  

  // Eigen::Quaternionf rotation(1,0,0,0);    
  // Eigen::Vector3f origin(0,0,0);
  
  // transformPointCloud(org_pcl, *transformed_cloudPtr, origin, rotation);
  // transformed_cloudPtr->header.frame_id = "world"; //should be after transform otherwise aha
  // transformed_cloudPtr->header.frame_id = "velodyne"; //should be after transform otherwise aha

  // ROS_INFO_STREAM("transformed_cloudPtr: "<< transformed_cloudPtr->points.size());
  // ROS_INFO_STREAM("transformed_cloudPtr: "<< transformed_cloudPtr->header.frame_id);




//*********** from pcl to ros msg ***********
  sensor_msgs::PointCloud2 sph_pcl_msg;
  pcl::toROSMsg(sph_pcl, sph_pcl_msg);
  sph_pcl_msg.header.stamp = pcl_in->header.stamp; //.toSec(); 
  processed_pcl_pub.publish(sph_pcl_msg);
 
  // ROS_INFO_STREAM("hight: "<< sph_pcl_msg.height);
  // ROS_INFO_STREAM("width: "<< sph_pcl_msg.width);
  // ROS_INFO_STREAM("point_step: "<< sph_pcl_msg.point_step);
  // ROS_INFO_STREAM("row_step: "<< sph_pcl_msg.row_step);




  sensor_msgs::PointCloud2 cart_pcl_msg;
  pcl::toROSMsg(cart_pcl, cart_pcl_msg);
  sph_pcl_msg.header.stamp = pcl_in->header.stamp;//.toSec(); 
  org_low_frq_pub.publish(cart_pcl_msg);
  ros::Duration(0.3).sleep();

}




int main(int argc, char** argv)
{
  ros::init(argc, argv, "pcl_surface_projection");
  ROS_INFO("pcl_surface_projection node ... ");

  ros::NodeHandle nh;

  //create publisher object
  processed_pcl_pub = nh.advertise<sensor_msgs::PointCloud2>("/pcl_in_wrld_fr", 1);
  org_low_frq_pub = nh.advertise<sensor_msgs::PointCloud2>("/org_low_frq_pcl", 1);

  // message filter time synchronization
  message_filters::Subscriber<nav_msgs::Odometry> pose_sub(nh, "/ground_truth/state", 1);
  message_filters::Subscriber<sensor_msgs::PointCloud2> pcl_sub(nh, "/mid/points", 1);
  message_filters::TimeSynchronizer<sensor_msgs::PointCloud2, nav_msgs::Odometry> sync(pcl_sub, pose_sub, 10);
 
  sync.registerCallback(boost::bind(&sync_callback, _1, _2)); // try std instead boost not working as well
  ros::spin();

  return 0;
}