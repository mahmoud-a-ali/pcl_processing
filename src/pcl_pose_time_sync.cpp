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
  // ROS_INFO_STREAM("pcl  frame     : "<< pcl_in->header.frame_id);
  // ROS_INFO_STREAM("pose position  : "<< pose_in->pose.pose.position.x <<" "<<
                                      // pose_in->pose.pose.position.y <<" "<< pose_in->pose.pose.position.z);
  // ROS_INFO_STREAM("pose orientatio: "<< pose_in->pose.pose.orientation.x <<" "<<pose_in->pose.pose.orientation.y <<" "<<
                                      // pose_in->pose.pose.orientation.z <<" "<< pose_in->pose.pose.orientation.w);



  //convert the input into a pcl::PointCloud< pcl::PointXYZ> object
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloudPtr(new pcl::PointCloud<pcl::PointXYZI>);


  // variables  for filtered/transformed/low_frq  pcl 
  pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloudPtr(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_cloudPtr(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr low_frq_cloudPtr(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::fromROSMsg(*pcl_in, *cloudPtr);


  //*********** outlier remover ***********
  // pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
  // sor.setInputCloud (cloudPtr);
  // sor.setMeanK (50);
  // sor.setStddevMulThresh (1.0);
  // sor.filter (*filtered_cloudPtr);


  //*********** pcl to std vector ***********
  pcl::PointCloud<pcl::PointXYZI> org_pcl_vec, fr_pcl_vec;
  std::copy(cloudPtr->points.begin(), cloudPtr->points.end(),std::back_inserter(org_pcl_vec));



  ROS_INFO_STREAM("hight: "<< pcl_in->height);
  ROS_INFO_STREAM("width: "<< pcl_in->width);
  ROS_INFO_STREAM("point_step: "<< pcl_in->point_step);
  ROS_INFO_STREAM("row_step: "<< pcl_in->row_step);
  ROS_INFO_STREAM("cloudPtr [0]: "<< cloudPtr->points.size());
  ROS_INFO_STREAM("org_pcl_vec: "<< org_pcl_vec.size() );
  // ROS_INFO_STREAM("org_pcl_vec : 1st pt: "<<org_pcl_vec[0].x<<", "<< org_pcl_vec[0].y<<", "<<org_pcl_vec[0].z<<", "<<org_pcl_vec[0].intensity);


  //*********** clustering ***********
  // next step



  //*********** adding free points ***********
  const float rslujn = 0.2; 
  for(int pt=0; pt<org_pcl_vec.size(); pt++){
    org_pcl_vec[pt].intensity = 1.0;
    float bm_lngth = sqrt(org_pcl_vec[pt].x*org_pcl_vec[pt].x + org_pcl_vec[pt].y*org_pcl_vec[pt].y + org_pcl_vec[pt].z*org_pcl_vec[pt].z );
    float bm_nrml[3] = {org_pcl_vec[pt].x/bm_lngth, org_pcl_vec[pt].y/bm_lngth, org_pcl_vec[pt].z/bm_lngth }; 
    pcl::PointXYZI fr_pt; 

    if(bm_lngth < (2*rslujn)){
      fr_pt.x = bm_nrml[0]*bm_lngth/2.0;
      fr_pt.y = bm_nrml[1]*bm_lngth/2.0;
      fr_pt.z = bm_nrml[2]*bm_lngth/2.0;
      fr_pt.intensity = 0.0;
      fr_pcl_vec.push_back(fr_pt);
    }
    else{
      // fr_pt.x = bm_nrml[0]*rslujn;
      // fr_pt.y = bm_nrml[1]*rslujn;
      // fr_pt.z = bm_nrml[2]*rslujn;
      // fr_pt.intensity = 0.0;
      // fr_pcl_vec.push_back(fr_pt);

      fr_pt.x = bm_nrml[0]*(bm_lngth - rslujn);
      fr_pt.y = bm_nrml[1]*(bm_lngth - rslujn);
      fr_pt.z = bm_nrml[2]*(bm_lngth - rslujn);
      fr_pt.intensity = 0.0;
      fr_pcl_vec.push_back(fr_pt);

    }
  }
  ROS_INFO_STREAM("fr_pcl_vec: "<< fr_pcl_vec.size() );



  //*********** add fr tp org pcl_vec ***********
  // std::copy (b.begin(), b.end(), std::back_inserter(a));
  // a.insert(a.end(), b.begin(), b.end());
  org_pcl_vec.insert(org_pcl_vec.end(), fr_pcl_vec.begin(), fr_pcl_vec.end());


  ROS_INFO_STREAM("org+fr pcl_vec: "<< org_pcl_vec.size() );






    // dist = np.sqrt(x**2 + y**2 + z**2)
    // theta = np.arctan2(y, x)
    // alpha = np.arccos(z / dist)

    // x = np.array( dist * np.sin(alpha) * np.cos(theta), dtype='float32')
    // y = np.array( dist * np.sin(alpha) * np.sin(theta), dtype='float32')
    // z = np.array( dist * np.cos(alpha) , dtype='float32')


  //*********** std vector to pcl ***********
  // std::vector<pcl::PointXYZI> org_pcl_vector; 
  // std::copy(org_pcl_vec.begin(), org_pcl_vec.end(),std::back_inserter(org_pcl_vector));
  // ROS_INFO_STREAM("org_pcl_vector: "<< org_pcl_vector.size());


  //*********** from velodyne to world frame ***********
  Eigen::Quaternionf rotation(pose_in->pose.pose.orientation.w, pose_in->pose.pose.orientation.x, pose_in->pose.pose.orientation.y,
                              pose_in->pose.pose.orientation.z);    
  Eigen::Vector3f origin(pose_in->pose.pose.position.x, pose_in->pose.pose.position.y, pose_in->pose.pose.position.z+0.322);
  
  // Eigen::Quaternionf rotation(1,0,0,0);    
  // Eigen::Vector3f origin(0,0,0);
  
  transformPointCloud(org_pcl_vec, *transformed_cloudPtr, origin, rotation);
  transformed_cloudPtr->header.frame_id = "world"; //should be after transform otherwise aha
  // transformed_cloudPtr->header.frame_id = "velodyne"; //should be after transform otherwise aha

  // ROS_INFO_STREAM("transformed_cloudPtr: "<< transformed_cloudPtr->points.size());
  // ROS_INFO_STREAM("transformed_cloudPtr: "<< transformed_cloudPtr->header.frame_id);




//*********** from pcl to ros msg ***********
  sensor_msgs::PointCloud2 pcl_msg_org_fr;
  pcl::toROSMsg(*transformed_cloudPtr, pcl_msg_org_fr);
  processed_pcl_pub.publish(pcl_msg_org_fr);
 
  // ROS_INFO_STREAM("hight: "<< pcl_msg_org_fr.height);
  // ROS_INFO_STREAM("width: "<< pcl_msg_org_fr.width);
  // ROS_INFO_STREAM("point_step: "<< pcl_msg_org_fr.point_step);
  // ROS_INFO_STREAM("row_step: "<< pcl_msg_org_fr.row_step);





    // sensor_msgs::PointCloud2 low_frq_pcl_msg;
    // pcl::toROSMsg(*filtered_cloudPtr, low_frq_pcl_msg);
    // org_low_frq_pub.publish(low_frq_pcl_msg);
    ros::Duration(0.3).sleep();

}




int main(int argc, char** argv)
{
  ros::init(argc, argv, "pcl_pose_time_sync_node");

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