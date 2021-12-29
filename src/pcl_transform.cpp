
#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h> //for sensor_msgs::PointCloud2
#include <pcl/point_types.h> // for pcl::PointXYZ
#include <pcl_conversions/pcl_conversions.h> // for pcl::fromROSMsg()
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>

#include <pcl/segmentation/sac_segmentation.h> //for SACSegmentation
#include <pcl/filters/extract_indices.h> //for ExtractIndices
#include <pcl/filters/statistical_outlier_removal.h>

#include <tf2_ros/transform_listener.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>




// Plane (ground) remover class
class PclPlaneRemover {
private:
  ros::Publisher pub_low_frq_org; //member variable stores publisher
  ros::Publisher pub; //member variable stores publisher

public:
  //constructor
  PclPlaneRemover (const ros::Publisher& pub, const ros::Publisher& pub_low_frq_org)
  :pub(pub), pub_low_frq_org(pub_low_frq_org){}

  //callback function Executed every time a pointCloud message is recieved
  void raw_pcl_callback(const sensor_msgs::PointCloud2& msgIn){
    ROS_INFO("PointCloud2 message recieved by pcl_solution_node");

    //convert the input into a pcl::PointCloud< pcl::PointXYZ> object
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPtr(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloudPtr(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloudPtr(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr low_frq_cloudPtr(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(msgIn, *cloudPtr);
    pcl::fromROSMsg(msgIn, *low_frq_cloudPtr);

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



    // #################### create transform listener and put the transform in stamped_transform 
    // transform from frame to another frame 
    // tf::TransformListener listener;
    // tf::Transform transform;
    // tf::StampedTransform transform;


    // ************ one way for transform listener 
    // listener.lookupTransform("odom", "/mid/points", ros::Time(0), transform);
    // ROS_INFO(" listen to transform 1");

    // ************ second way for transform listener 
    // try{
    //   listener.lookupTransform("/odom", "/mid/points", ros::Time(0), transform);
    // }
    // catch (tf::TransformException ex){
    //   ROS_ERROR("%s",ex.what());
    //   ros::Duration(1.0).sleep();
    // }
    // ROS_INFO(" listen to transform 2");

  // ************ third  way for transform       
   // try{
   //    listener.waitForTransform(target_frame, original_frame, time, ros::Duration(10.0));
   //    listener.lookupTransform(target_frame, original_frame, time, transform);
   //    break;
   //  }
   // catch (tf::TransformException ex){
   //      ROS_ERROR("%s",ex.what());
   // }
    // ROS_INFO(" listen to transform 3");


 //************ last available transform ************* 
  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);
  geometry_msgs::TransformStamped transformStamped;
  // tf2::Transform transformStamped;

  try{
      // transformStamped = tfBuffer.lookupTransform("target_frame", "source_frame", ros::Time(0));
      // transformStamped = tfBuffer.lookupTransform("odom", "velodyne", ros::Time(0), ros::Duration(1.0));
      transformStamped = tfBuffer.lookupTransform("odom", "velodyne", msgIn.header.stamp + ros::Duration(0.3), ros::Duration(1.0));
      // ROS_INFO(" ahahahahahahaha");
    }
    catch (tf2::TransformException &ex) {
      ROS_WARN("%s",ex.what());
    }





    // ros::Duration(0.3).sleep();
// #################### transorm pointcloud ####################
// check https://github.com/ros-perception/perception_pcl/pull/231/files

// ************ one way  
// transformPointCloud (const std::string &target_frame, const tf::Transform &net_transform, const sensor_msgs::PointCloud2 &in, sensor_msgs::PointCloud2 &out)
    // pcl_ros::transformPointCloud("odom", *filtered_cloudPtr, *transformed_cloudPtr, tfBuffer);


  // pcl_ros::transformPointCloud ("odom", msgIn.header.stamp+ ros::Duration(0.3), *filtered_cloudPtr,"velodyne",*transformed_cloudPtr,tfBuffer);
  pcl_ros::transformPointCloud ("odom", msgIn.header.stamp+ ros::Duration(0.3), *low_frq_cloudPtr,"velodyne",*transformed_cloudPtr,tfBuffer);


  // pcl_ros::transformPointCloud ("odom", ros::Time(0), *filtered_cloudPtr,"velodyne",*transformed_cloudPtr,tfBuffer);




// ************ another way 
// bool pcl_ros::transformPointCloud ( const std::string &   target_frame,
//                                      const pcl::PointCloud< PointT > &   cloud_in, 
//                                     pcl::PointCloud< PointT > &   cloud_out, 
//                                     const tf::TransformListener &   tf_listener ) 

// transformPointCloud (const std::string &target_frame,
//                    const pcl::PointCloud <PointT> &cloud_in,
//                    pcl::PointCloud <PointT> &cloud_out,
//                    const tf2_ros::Buffer &tf_buffer);

// transformPointCloud (const std::string &target_frame,
//                      const sensor_msgs::PointCloud2 &in,
//                      sensor_msgs::PointCloud2 &out,
//                      const tf2_ros::Buffer &tf_buffer);



// transformPointCloud (const pcl::PointCloud <PointT> &cloud_in,
//                      pcl::PointCloud <PointT> &cloud_out,
//                      const geometry_msgs::Transform &transform);


// transformPointCloud (const std::string &target_frame,
//                      const geometry_msgs::Transform &net_transform,
//                      const sensor_msgs::PointCloud2 &in,
//                      sensor_msgs::PointCloud2 &out);




// #################### convert pointcloud to ros msg ####################
    sensor_msgs::PointCloud2 low_frq_pcl_msg;
    sensor_msgs::PointCloud2 sgmntd_pcl_msg;

    pcl::toROSMsg(*filtered_cloudPtr, sgmntd_pcl_msg);
    pcl::toROSMsg(*transformed_cloudPtr, low_frq_pcl_msg);

    //publish message
    pub.publish(sgmntd_pcl_msg);
    // pub_low_frq_org.publish(low_frq_pcl_msg);
    pub_low_frq_org.publish(msgIn);
    ros::Duration(0.6).sleep();
  }

};



int main(int argc, char **argv) {
  //Initiate the ROS system and become a node.
  ros::init(argc, argv, "pcl_plane_remover");
  ros::NodeHandle nh;

  //create publisher object
  ros::Publisher processed_pcl_pub = nh.advertise<sensor_msgs::PointCloud2>("/segmented_pcl", 1);
  ros::Publisher org_low_frq_pub = nh.advertise<sensor_msgs::PointCloud2>("/org_low_frq_pcl", 1);

  PclPlaneRemover pclpr(processed_pcl_pub, org_low_frq_pub);

  //create subscriber object
  ros::Subscriber sub = nh.subscribe("/mid/points", 1, &PclPlaneRemover::raw_pcl_callback, &pclpr);
  // ros::Subscriber sub = nh.subscribe("/velodyne_points", 1, &PclPlaneRemover::raw_pcl_callback, &pclpr);

  ROS_INFO("pcl_solution_node started");
  // let ROS take over
  ros::spin();

  return 0;
}