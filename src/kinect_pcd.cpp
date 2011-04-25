// Filename: kinect_pcd.cpp
// Author: Jason Tennyson
// Date: 4-25-11
// 
// This file is designed to be used after the openni_camera driver is launched
// for use with the Microsoft Kinect.  It reads a single point cloud message
// from the Kinect, formats the data, and stores it with a .pcd extension.

#include <ros/ros.h>
#include <time.h>
#include <cv.h>
#include <highgui.h>
#include <unistd.h>
#include <sys/stat.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

// The delay time in seconds between when the program starts and when a cloud is taken.
#define CAM_DELAY (5)

// These are the defined folder names for all files.
#define PCD_FOLDER ("pcd_files")
#define IMG_FOLDER ("image_files")

// The file extension of the image that we save from the kinect.
#define IMG_EXTENSION ("bmp")

// The templated pcl object type.
typedef pcl::PointXYZRGB PointT;

// This boolean value is set to true when we get a point cloud.
// The program loops until it gets a point cloud and sets this to true.
bool DATA_FULL = false;

// The user-specified filename that we are going to save the point cloud data to.
std::string FILE_NAME = "";

// The file path that we extract from the launch file if we run through it.
std::string FILE_PATH = "";

// Accepts a point cloud of type XYZRGB from the callback function and saves
// its RGB data to a file with the extension given above.
void saveImage(const pcl::PointCloud<PointT>::Ptr cloud);

// This function is called as soon as the kinect driver pushes out a new
// point cloud message.
void pcdCallback(const sensor_msgs::PointCloud2::Ptr msg);

int main(int argc, char** argv)
{
  // Check to see that the user specified a file name.
  if(argc < 2)
  {
    ROS_INFO("No file name given!");
    return(-1);
  }
  else
  {
    // Store the file name given by the launch file.
    FILE_NAME += argv[1];

    // Store the path to the root package directory given by the launch file.
    FILE_PATH = argv[2];

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

void saveImage(const pcl::PointCloud<PointT>::Ptr cloud)
{
  // Integer return value when attempting to change a directory.
  int good_dir = 0;

  // Change directories to the root folder.
  if(chdir(FILE_PATH.c_str()) == 0)
  {
    good_dir = chdir(IMG_FOLDER);

    // If we can't change directories to the image folder, we assume it doesn't exist and make it.
    if(good_dir != 0)
    {
      // Make the image folder.
      if(mkdir(IMG_FOLDER, 0755) == 0)
      {
        // If successful, change directories into this image folder.
        good_dir = chdir(IMG_FOLDER);
      }
      else
      {
        ROS_INFO("Program failed, likely due to improper read/write permissions in this directory.");
      }
    }

    // If we are in the right directory, save the image file.
    if(good_dir == 0)
    {
      // Create a blank cv image object.
      IplImage* img = cvCreateImage(cvSize(cloud->width,cloud->height), IPL_DEPTH_8U, 3);

      // Populate the image object with rgb data from the point cloud we received.
      for(unsigned int i = 0; i < cloud->height; i++)
      {
        for(unsigned int j = 0; j < cloud->width; j++)
        {
          // Store the floating point rgb value as an integer for bit masking.
          int rgb = *reinterpret_cast<int*>(&cloud->points[i*cloud->width + j].rgb);
          ((uchar *)(img->imageData + i*img->widthStep))[j*img->nChannels + 0] = (rgb & 0xff);
          ((uchar *)(img->imageData + i*img->widthStep))[j*img->nChannels + 1] = ((rgb >> 8) & 0xff);
          ((uchar *)(img->imageData + i*img->widthStep))[j*img->nChannels + 2] = ((rgb >> 16) & 0xff);
        }
      }

      // Create a string and add the file name and its desired image extension type.
      std::string img_filename = FILE_NAME;
      img_filename += ".";
      img_filename += IMG_EXTENSION;

      // Save the image object to a file with the given name.
      cvSaveImage(img_filename.c_str(),img);

      // Delete the image object.
      cvReleaseImage(&img);
    }
  }
}

void pcdCallback(const sensor_msgs::PointCloud2::Ptr msg)
{
  int good_dir = 0;

  if(!DATA_FULL)
  {
    // Used to store a point cloud to a file.
    pcl::PCDWriter writer;

    // An empty cloud.
    pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT> ());

    // Fill the empty cloud with data from the kinect.
    pcl::fromROSMsg(*msg,*cloud);

    // If our root path is correct, continue.
    if(chdir(FILE_PATH.c_str()) == 0)
    {
      good_dir = chdir(PCD_FOLDER);

      // If we aren't in the right directory, make the directory.
      if(good_dir != 0)
      {
        // If we are successful in making the directory, change directories to that directory.
        if(mkdir(PCD_FOLDER, 0755) == 0)
        {
          good_dir = chdir(PCD_FOLDER);
        }
        else
        {
          ROS_INFO("Program failed, likely due to improper read/write permissions in this directory.");
        }
      }

      // If we are now in the correct directory, write the data to disk.
      if(good_dir == 0)
      {
        // Write the point cloud data to disk.
        writer.write(FILE_NAME+".pcd", *cloud, false);

        // Write the RGB image to disk.
        saveImage(cloud);

        // We are done running the program.
        ROS_INFO("Done! Data was saved as %s.pcd and %s.%s!",FILE_NAME.c_str(),FILE_NAME.c_str(),IMG_EXTENSION);
        ROS_INFO("Press Ctrl+C to shut down the Kinect driver...");

        // We have received our data and are done here.
        DATA_FULL = true;
      }
    }
    else
    {
      ROS_INFO("Incorrect root path given!");
    }
  }
}
