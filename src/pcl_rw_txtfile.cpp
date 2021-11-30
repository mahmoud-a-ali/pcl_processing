
#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h> //for sensor_msgs::PointCloud2
#include <pcl/point_types.h> // for pcl::PointXYZ
#include <pcl_conversions/pcl_conversions.h> // for pcl::fromROSMsg()
#include <pcl_ros/point_cloud.h>

#include <pcl/segmentation/sac_segmentation.h> //for SACSegmentation
#include <pcl/filters/extract_indices.h> //for ExtractIndices

#include <fstream>
#include <vector>
#include <string>
#include <math.h> 

//create publisher object
// ros::Publisher pcl_pub;


// read training data set
void pcl_from_txtfile( std::string filename, 
                          std::vector< std::vector<double> > &pcl_vec, 
                          pcl::PointCloud<pcl::PointXYZ> &cloud, bool shperical=1){

  std::ifstream infile;
  std::string line; 
  infile.open(filename);// file containing xyz in 3 columns 
 
  pcl::PointXYZ point;
  std::vector<double> theta, phi, dist;  
  pcl_vec.resize(3);
 ROS_INFO_STREAM( "shperical: "<<shperical);
  while (std::getline(infile, line))
  {
    std::stringstream ss(line);
    float a, b, c;

    if (ss >> a >> b >> c)
    {
      float x,y,z;
      if(!shperical){
        x=a;
        y=b;
        z=c;
      }else{
        x = c * sin(b) * cos(a);
        y = c * sin(b) * sin(a);
        z = c * cos(b);
        // ROS_INFO("sh to xyz");
        // ROS_INFO_STREAM("x, y, z:" << x <<", "<<y <<", " <<z);

      }
      pcl_vec[0].push_back(x);
      pcl_vec[1].push_back(y);
      pcl_vec[2].push_back(z);

      point.x = x;
      point.y = y;
      point.z = z;
      cloud.points.push_back(point);
    }
  }
}


// x = r sinϕ cosθ
// y = r sinϕ sinθ
// z = r cosϕ 


int main(int argc, char **argv) {
  //Initiate the ROS system and become a node.
  ros::init(argc, argv, "pcl_readfile");
  ros::NodeHandle nh;
  ROS_INFO("pcl_readfile started");
  ros::Publisher pcl_pub = nh.advertise<sensor_msgs::PointCloud2>("/pcl_from_txt", 1000);

  // dataset directory and name 
  std::string dataset_dir = "/home/alimaa/github/datasets_gp/";
  std::string dataset = "sph_sampled.txt";
  std::string filename = dataset_dir + dataset;
  
  std::vector< std::vector<double> > pcl_vec;
  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl_from_txtfile(filename, pcl_vec, cloud);

  ROS_INFO_STREAM("dataset name: " << dataset );
  ROS_INFO_STREAM("dataset size: " << cloud.points.size());

  sensor_msgs::PointCloud2 pcl_msg;
  pcl::toROSMsg(cloud, pcl_msg);
  pcl_msg.header.frame_id = "mid_mount";

  while (ros::ok()){
    pcl_pub.publish(pcl_msg);    
  }

  ros::spin();

  return 0;
}
