// Filename: kinect_pcd.cpp
// Author: Jason Tennyson
// Data: 4-14-11
// 
// This file is designed to be used after the openni_camera driver is launched
// for use with the Microsoft KINECT.  It reads a single point cloud message
// from the KINECT, formats the data, and stores it with a .pcd extension.

#include <ros/ros.h>
#include <time.h>
#include <unistd.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#define CAM_DELAY (5)

// The templated pcl object type.
typedef pcl::PointXYZRGB PointT;

// This boolean value is set to true when we get a point cloud.
// The program loops until it gets a point cloud and sets this to true.
bool DATA_FULL = false;

// The user-specified filename that we are going to save the point cloud data to.
std::string FILE_NAME = "";

// The file path that we extract from the launch file if we run through it.
std::string FILE_PATH = "";

// The name of the folder in the root directory where all of the pcd files are stored.
const std::string FILE_FOLDER = "PCDs";

void pcdCallback(const sensor_msgs::PointCloud2::Ptr msg)
{
  if(!DATA_FULL)
  {
    // Used to store a point cloud to a file.
    pcl::PCDWriter writer;

    // An empty cloud.
    pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT> ());

    // Fill the empty cloud with data from the kinect.
    pcl::fromROSMsg(*msg,*cloud);

    // Changed directories to the folder where we keep the .pcd files.
    chdir(FILE_PATH.c_str());

    // Write the point cloud data to disk.
    writer.write(FILE_NAME+".pcd", *cloud, false);

    // We are done running the program.
    ROS_INFO("Done! Press Ctrl+C to shut down the kinect driver...");
    DATA_FULL = true;
  }
}

int main (int argc, char** argv)
{
  // Check to see that the user specified a file name.
  if(argc < 2)
  {
    ROS_INFO("No file name given!");
    return(-1);
  }
  else
  {
    // We parse through the argument and drop off extensions in case the user added one.
    for(int i = 0; ((argv[1][i] != '.') && (argv[1][i] != '\0')); i++)
    {
      // Since we have not hit a period or the end of the argument, tack on the char.
      FILE_NAME += argv[1][i];
    }

    // The launch file sends the path to the root folder of this package as an argument.
    if(argc > 2)
    {
      FILE_PATH = argv[2];
      FILE_PATH += "/";
      FILE_PATH += FILE_FOLDER;
    }
    else
    {
      FILE_PATH = FILE_FOLDER;
    }

    // This time delay allows you to start the program and walk away if it's in view of the Kinect.
    ROS_INFO("You now have %d seconds to back away!",CAM_DELAY);
    sleep(CAM_DELAY);
    ROS_INFO("Time is up!");

    // Initializes the node as a kinect listener.
    ros::init(argc, argv, "kinect_to_pcd");

    // Create an empty node handle.
    ros::NodeHandle n;

    // Create a subscriber on the depth topic.
    ros::Subscriber sub = n.subscribe("camera/depth/points2", 1000, pcdCallback);

    // Waits for new data. Then, the callback function is executed.
    while((!DATA_FULL) && ros::ok())
    {
      ros::spinOnce();
    }
  }

  return (0);
}
