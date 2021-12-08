
#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h> //for sensor_msgs::PointCloud2
#include <pcl/point_types.h> // for pcl::PointXYZ
#include <pcl_conversions/pcl_conversions.h> // for pcl::fromROSMsg()
#include <pcl_ros/point_cloud.h>

#include <pcl/segmentation/sac_segmentation.h> //for SACSegmentation
#include <pcl/filters/extract_indices.h> //for ExtractIndices
#include <pcl/filters/statistical_outlier_removal.h>


// Plane (ground) remover class
class PclPlaneRemover {
private:
  ros::Publisher pub; //member variable stores publisher

public:
  //constructor
  PclPlaneRemover (const ros::Publisher& pub)
  :pub(pub){}

  //callback function Executed every time a pointCloud message is recieved
  void raw_pcl_callback(const sensor_msgs::PointCloud2& msgIn){
    ROS_INFO("PointCloud2 message recieved by pcl_solution_node");

    //convert the input into a pcl::PointCloud< pcl::PointXYZ> object
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPtr(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloudPtr(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(msgIn, *cloudPtr);

    // ROS_INFO_STREAM("size cloud before removing groundplane:" << (cloudPtr->width));

    //GROUND PLANE REMOVAL 
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    // Optional
    seg.setOptimizeCoefficients (true);
    // Mandatory
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(0.3);

    seg.setInputCloud(cloudPtr);
    seg.segment(*inliers, *coefficients);

    //filter inliers (=groundplane) from cloud
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(cloudPtr);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*cloudPtr);
    // ROS_INFO_STREAM("size cloud after removing groundplane:" << (cloudPtr->width));

    // pcl::PointCloud<pcl::PointXYZ> segmented_pcl = *cloudPtr;

    // outlier remover
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud (cloudPtr);
    sor.setMeanK (50);
    sor.setStddevMulThresh (1.0);
    sor.filter (*filtered_cloudPtr);





    sensor_msgs::PointCloud2 pcl_msg;
    pcl::toROSMsg(*filtered_cloudPtr, pcl_msg);
   
    //publish message
    pub.publish(pcl_msg);
    ros::Duration(0.5).sleep();
  }

};



int main(int argc, char **argv) {
  //Initiate the ROS system and become a node.
  ros::init(argc, argv, "pcl_plane_remover");
  ros::NodeHandle nh;

  //create publisher object
  ros::Publisher processed_pcl_pub = nh.advertise<sensor_msgs::PointCloud2>("/segmented_pcl", 1);
  PclPlaneRemover pclpr(processed_pcl_pub);

  //create subscriber object
  // ros::Subscriber sub = nh.subscribe("/mid/points", 1, &PclPlaneRemover::raw_pcl_callback, &pclpr);
  ros::Subscriber sub = nh.subscribe("/velodyne_points", 1, &PclPlaneRemover::raw_pcl_callback, &pclpr);

  ROS_INFO("pcl_solution_node started");
  // let ROS take over
  ros::spin();

  return 0;
}