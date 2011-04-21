// Filename: kinect_transformer.cpp
// Author: Jason Tennyson
// Data: 4-13-11
// 
// This file reads a text file that contains file names.
// The file names point to point clouds that have previously
// been recorded, and the x, y, z, theta values that define
// each cloud's offset from 0,0,0,0,0,0 is included with those files.

// testing1,-0.1212,0,0,0,-30,0
// testing2,0.1212,0,0,0,30,0

#include <ros/ros.h>
#include <fstream>
#include <cstdlib>
#include <math.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <boost/algorithm/string.hpp>
#include <Eigen/Geometry>
#include "pcl/common/transform.hpp"

#define PCDS_FILE ("pcds_file.txt")
#define DEG_TO_RAD (0.0174533)

// The templated pcl object type.
typedef pcl::PointXYZRGB PointT;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "kinect_transform_publisher");
  ros::NodeHandle node;

  // This node publishes on the same topic as a Kinect. This is done so that a user can use the
  // same point cloud functions that work directly from the Kinect as they use to read and
  // interpret this transformed cloud.
  ros::Publisher kinect_pub = node.advertise<sensor_msgs::PointCloud2>("camera/depth/points2", 1000);

  // Create our pcd reading object.
  pcl::PCDReader reader;

  // Create a cloud pointer that points to a blank cloud.
  pcl::PointCloud<PointT>::Ptr final_cloud (new pcl::PointCloud<PointT> ());

  // Open the user-created data file that points to the point clouds.
  std::ifstream pcds_in(PCDS_FILE);

  while(!pcds_in.eof())
  {
    // This stores the current line we're working with from the text file.
    std::string pcd_info;

    // Grab a line of pcd information from the file.
    getline(pcds_in, pcd_info);

    // Create an array of string parameters from the info we obtained.
    // All of the info should be comma delimited in the file.
    std::vector<std::string> params;
    boost::split(params,pcd_info,boost::is_any_of(","));

    if(params.size() == 7)
    {
      // Each line has a file name that contains point cloud data.
      std::string filename;

      // Each line also has floating point values for x,y,z,x theta, y theta, and z theta(meters and degrees).
      // The x axis runs left to right, the y axis top to bottom, and the z axis forward and back.
      float x,y,z,x_theta,y_theta,z_theta;

      // The first parameter of an info line should be the file name.
      filename = params[0];

      // Convert the separated offset parameters from the text file to floats.
      x = atof(params[1].c_str());
      y = atof(params[2].c_str());
      z = atof(params[3].c_str());
      x_theta = atof(params[4].c_str());
      y_theta = atof(params[5].c_str());
      z_theta = atof(params[6].c_str());

      // Create a transformation object based on the data we received.
      Eigen::Affine3f transform;
      pcl::getTransformation (x, y, z, (DEG_TO_RAD*x_theta), (DEG_TO_RAD*y_theta), (DEG_TO_RAD*z_theta), transform);

      // Store off the previous cloud's width and total size.
      unsigned int prev_width = final_cloud->width;
      unsigned int prev_size = final_cloud->points.size();

      // Create a blank cloud for storing the next set of data.
      pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT> ());

      // Read in a cloud.
      reader.read (filename+".pcd", *cloud);

      // Resize our destination cloud so that we can add the new cloud info to it.
      // We have to do this because ROS doesn't like width*height != points.size().
      final_cloud->width = cloud->width + prev_width;
      final_cloud->height = cloud->height;
      final_cloud->points.resize(final_cloud->width * final_cloud->height);

      // Manipulate the input cloud and store it.
      for(unsigned int i = 0; i < cloud->points.size(); i++)
      {
        final_cloud->points[i+prev_size] = pcl::transformXYZ(transform, cloud->points[i]);
      }
    }
  }

  // Close our input file.
  pcds_in.close();

  // Create a blank sensor message.
  sensor_msgs::PointCloud2 transformed_cloud;

  // Fill that sensor message with our cloud data.
  pcl::toROSMsg(*final_cloud,transformed_cloud);

  // We continue to publish this data over and over again. This is to duplicate what a Kinect does.
  while(ros::ok())
  {
    // Send out the unified point cloud.
    kinect_pub.publish(transformed_cloud);
  }

  return 0;
};

