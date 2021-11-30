
#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h> //for sensor_msgs::PointCloud2
#include <pcl/point_types.h> // for pcl::PointXYZ
#include <pcl_conversions/pcl_conversions.h> // for pcl::fromROSMsg()
#include <pcl_ros/point_cloud.h>

#include <pcl/segmentation/sac_segmentation.h> //for SACSegmentation
#include <pcl/filters/extract_indices.h> //for ExtractIndices

// #include <gp/gp.h>

void pcl2dataset_callback(const sensor_msgs::PointCloud2& msgIn){
  //ROS_INFO("PointCloud2 message recieved by pcl_solution_node");

  //convert the input into a pcl::PointCloud< pcl::PointXYZ> object
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPtr(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(msgIn, *cloudPtr);

  ROS_INFO_STREAM("PCL size:" << (cloudPtr->width));

  // for(int pt=0; pt < cloudPtr->points.size(); pt++){
  //   ROS_INFO_STREAM("x, y, z:" << cloudPtr->points[pt].x <<", "<< cloudPtr->points[pt].y <<", "<< cloudPtr->points[pt].z);
  // }
 
 // generateed datasets names and directory 
  std::string dataset_dir = "/home/alimaa/github/datasets_gp/";
  std::string xyz_dataset = "xyz_pcl_data.txt";
  std::string sph_dataset = "sph_pcl_data.txt";
  


 // Write entire point clouds to a .txt file
    std::ofstream xyz_pcl_file;
    std::ofstream sph_pcl_file;

 // Write entire point clouds to a .txt file
    sph_pcl_file.open (dataset_dir+sph_dataset);
    xyz_pcl_file.open (dataset_dir+xyz_dataset);

    double dist = 0;
    ROS_INFO_STREAM("file: "<< dataset_dir+sph_dataset << " is opened = "<< sph_pcl_file.is_open());
    if (sph_pcl_file.is_open()) {
      for(int pt=0; pt < cloudPtr->points.size(); pt++){
      
        dist = std::sqrt( cloudPtr->points[pt].x * cloudPtr->points[pt].x 
                        + cloudPtr->points[pt].y * cloudPtr->points[pt].y 
                        + cloudPtr->points[pt].z * cloudPtr->points[pt].z );

        xyz_pcl_file << " " << cloudPtr->points[pt].x
                     << " " << cloudPtr->points[pt].y
                     << " " << cloudPtr->points[pt].z << std::endl;

        sph_pcl_file << " " << std::atan2(cloudPtr->points[pt].y, cloudPtr->points[pt].x) 
                     << " " << std::acos(cloudPtr->points[pt].z/dist)
                     << " " << dist << std::endl;
      
      }
      sph_pcl_file.close();
      xyz_pcl_file.close();
      ROS_INFO_STREAM("pcl points saved to file: "<< dataset_dir);
    }

    exit(0);
}

// X = cos(theta) * cos(phi) * radius
// Y = sin(theta) * cos(phi) * radius
// Z = sin(phi) * radius
// radius = sqrt(X * X + Y * Y + Z * Z)
// theta = atan2(Y, X)
// phi = acos(Z / radius)

int main(int argc, char **argv) {
  //Initiate the ROS system and become a node.
  ros::init(argc, argv, "pcl_to_dataset");
  ros::NodeHandle nh;

  //pcl subscriber 
  ros::Subscriber sub = nh.subscribe("/segmented_pcl", 1000, pcl2dataset_callback);

  ROS_INFO("pcl_solution_node started");
  
  ros::spin();

  return 0;
}