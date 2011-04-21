// Filename: kinect_transformer.cpp
// Author: Jason Tennyson
// Date: 4-21-11
// 
// This file reads a text file that contains file names.
// The file names point to point clouds that have previously
// been recorded, and the x, y, z, theta values that define
// each cloud's offset from 0,0,0,0,0,0 is included with those files.

#include <ros/ros.h>
#include <fstream>
#include <cstdlib>
#include <math.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <boost/algorithm/string.hpp>
#include <Eigen/Geometry>
#include "pcl/common/transform.hpp"

// The file that the file names and parameters are read from.
#define PCDS_FILE ("pcds_file.txt")

// The folders that all of the files are separated into.
// These file folders are assumed to be in the root package directory.
#define PCDS_FOLDER ("pcd_files")
#define PTS_FOLDER ("pts_files")
#define VRML_FOLDER ("vrml_files")
#define PCD_INFO_FOLDER (".")

// These are our file extensions
#define VRML_EXTENSION (".wrl")
#define PTS_EXTENSION (".pts")
#define PCD_EXTENSION (".pcd")

// These defines are used to convert units so users can input familiar units.
#define DEG_TO_RAD (0.0174533)

// This is the size of the tabbed white space that we use
// to signify that we are inside of a certain vrml scope.
#define	TAB_SIZE (2)

// The templated pcl object type.
typedef pcl::PointXYZRGB PointT;

// Merges the data and writes a file with the .pts file extension.
void writePTS(const char* pts_filename, const char* root);

// Simply takes the .pts file output and converts it to vrml.
void writeVRML(const char* pts_filename, const char* vrml_filename, const char* root);

int main(int argc, char** argv)
{
  // Store the name of the output files that the user specified and add their extensions.
  std::string pts_file = argv[1];
  std::string vrml_file = argv[1];
  pts_file += PTS_EXTENSION;
  vrml_file += VRML_EXTENSION;

  // Store the root package path argument sent by the launch file.
  const std::string ROOT_PATH = argv[2];

  // Create the pts file.
  writePTS(pts_file.c_str(), ROOT_PATH.c_str());

  // Now we convert the pts file to vrml.
  writeVRML(pts_file.c_str(), vrml_file.c_str(), ROOT_PATH.c_str());

  // The transform is complete.
  ROS_INFO("Transformation complete! Press Ctrl+C to shut down the ros core.");

  return 0;
}

void writePTS(const char* pts_filename, const char* root)
{
  // Create our pcd reading object.
  pcl::PCDReader reader;

  // Change directories to where the pcd info file is.
  chdir(root);
  chdir(PCD_INFO_FOLDER);

  // Open an input file stream from the user-edited data file.
  std::ifstream pcds_in(PCDS_FILE);

  // Change directories to the pts folder.
  chdir(root);
  chdir(PTS_FOLDER);

  // Create a file stream to the new pts file.
  std::ofstream pts_out(pts_filename);

  // Change directories to the PCD folder to read in the clouds.
  chdir(root);
  chdir(PCDS_FOLDER);

  // Read clouds until we have read them all.
  do
  {
    // This stores the current line we're working with from the pcd info file.
    std::string pcd_info;

    // Grab a line of pcd information from the info file.
    getline(pcds_in, pcd_info);

    // Create an array of string parameters from the info we obtained.
    // All of the info should be comma delimited in the file.
    std::vector<std::string> params;
    boost::split(params,pcd_info,boost::is_any_of(","));

    // If we have a valid line of 7 parameters.
    if(params.size() == 7)
    {
      // The first parameter of an info line should be the file name.
      std::string filename = params[0];

      // Convert the separated offset parameters from the text file to floats.
      float x = atof(params[1].c_str());
      float y = atof(params[2].c_str());
      float z = atof(params[3].c_str());
      float x_theta = atof(params[4].c_str());
      float y_theta = atof(params[5].c_str());
      float z_theta = atof(params[6].c_str());

      // Create a transformation object based on the data we received.
      Eigen::Affine3f transform;
      pcl::getTransformation (x, y, z, (DEG_TO_RAD*x_theta), (DEG_TO_RAD*y_theta), (DEG_TO_RAD*z_theta), transform);

      // Create a blank cloud for storing the next set of data.
      pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT> ());

      // Read in the data to the blank cloud.
      reader.read (filename+PCD_EXTENSION, *cloud);

      // Manipulate the input cloud and store it.
      for(unsigned int i = 0; i < cloud->points.size(); i++)
      {
        // If x y and z are numbers, we print the line.
        if(!((cloud->points[i].x != cloud->points[i].x) ||
             (cloud->points[i].y != cloud->points[i].y) ||
             (cloud->points[i].z != cloud->points[i].z)))
        {
          // Transform the point and store it back to the cloud object.
          cloud->points[i] = pcl::transformXYZ(transform, cloud->points[i]);

          // Print the distance information to the pts file followed by a trash buffer value.
          pts_out << cloud->points[i].x << " " << cloud->points[i].y << " " << cloud->points[i].z << " 0 ";

          // Print the RGB information and finish with a new line character.
          int rgb = *reinterpret_cast<int*>(&cloud->points[i].rgb);
          pts_out << ((rgb >> 16) & 0xff) << " " << ((rgb >> 8) & 0xff) << " " << (rgb & 0xff) << "\n";
        }
      }
    }
  }while(!pcds_in.eof());

  // Print out one last new line at the end of the pts file.
  pts_out << "\n";

  // Close our input and output files.
  pcds_in.close();
  pts_out.close();
}

void writeVRML(const char* pts_filename, const char* vrml_filename, const char* root)
{
  // The number of spaces we print before we start to write a field name in the vrml file.
  // The number of spaces starts at 0 and varies by +/- TAB_SIZE defined above.
  int numSpaces = 0;

  // Change directories to the pts folder and create the input stream.
  chdir(root);
  chdir(PTS_FOLDER);
  std::ifstream pts_in(pts_filename);

  // Change directories to the vrml folder and create the output stream.
  chdir(root);
  chdir(VRML_FOLDER);
  std::ofstream vrml_out(vrml_filename);

  // Write the header and types that wrap the points list in the wrl file.
  vrml_out << "#VRML V2.0 utf8\n";
  vrml_out << "# Automatically generated by kinect_transformer in ROS.\n\n";
  vrml_out << "Group {\n";
  numSpaces += TAB_SIZE;
  for(int i = 0; i < numSpaces; i++) { vrml_out << " "; }
  vrml_out << "children [\n";
  numSpaces += TAB_SIZE;
  for(int i = 0; i < numSpaces; i++) { vrml_out << " "; }
  vrml_out << "Transform {\n";
  numSpaces += TAB_SIZE;
  for(int i = 0; i < numSpaces; i++) { vrml_out << " "; }
  vrml_out << "scale 1 1 1\n";
  for(int i = 0; i < numSpaces; i++) { vrml_out << " "; }
  vrml_out << "children [\n";
  numSpaces += TAB_SIZE;
  for(int i = 0; i < numSpaces; i++) { vrml_out << " "; }
  vrml_out << "Shape {\n";
  numSpaces += TAB_SIZE;
  for(int i = 0; i < numSpaces; i++) { vrml_out << " "; }
  vrml_out << "geometry PointSet {\n";
  numSpaces += TAB_SIZE;
  for(int i = 0; i < numSpaces; i++) { vrml_out << " "; }
  vrml_out << "coord Coordinate {\n";
  numSpaces += TAB_SIZE;
  for(int i = 0; i < numSpaces; i++) { vrml_out << " "; }
  vrml_out << "point [\n";
  numSpaces += TAB_SIZE;

  // Read in the lines from the pts file and format them for the vrml file.
  do
  {
    // This stores the current line we're working with from the text file.
    std::string pts_line;

    // Read a line.
    getline(pts_in, pts_line);

    // Break up the parameters in the line of point data.
    std::vector<std::string> params;
    boost::split(params,pts_line,boost::is_any_of(" "));

    // If we have 7 values, we assume valid data.
    if(params.size() == 7)
    {
      // Print only the points to the vrml file.
      for(int k = 0; k < numSpaces; k++) { vrml_out << " "; }
      vrml_out << params[0] << " " << params[1] << " " << params[2] << ",\n";
    }
  }while(!pts_in.eof());

  // Reset the pts file reader to the beginning of the file.
  pts_in.clear();
  pts_in.seekg(0);

  // Wrap the other side of the points list with their end brackets.
  numSpaces -= TAB_SIZE;
  for(int i = 0; i < numSpaces; i++) { vrml_out << " "; }
  vrml_out << "] # end of point\n";
  numSpaces -= TAB_SIZE;
  for(int i = 0; i < numSpaces; i++) { vrml_out << " "; }
  vrml_out << "} # end of Coordinate\n";

  // Print the colors field indicators.
  for(int i = 0; i < numSpaces; i++) { vrml_out << " "; }
  vrml_out << "color Color {\n";
  numSpaces += TAB_SIZE;
  for(int i = 0; i < numSpaces; i++) { vrml_out << " "; }
  vrml_out << "color [\n";
  numSpaces += TAB_SIZE;

  // Read in a line from the pts file and format it for the vrml file.
  do
  {
    // This stores the current line we're working with from the text file.
    std::string pts_line;

    // Read a line.
    getline(pts_in, pts_line);

    // Break up the parameters in the line of point data.
    std::vector<std::string> params;
    boost::split(params,pts_line,boost::is_any_of(" "));

    if(params.size() == 7)
    {
      // Print only the colors to the vrml file.
      for(int k = 0; k < numSpaces; k++) { vrml_out << " "; }
      vrml_out << atof(params[4].c_str())/255.0 << " "
               << atof(params[5].c_str())/255.0 << " "
               << atof(params[6].c_str())/255.0 << ",\n";
    }
  }while(!pts_in.eof());

  // Print the end brackets for all of the fields that we opened.
  numSpaces -= TAB_SIZE;
  for(int i = 0; i < numSpaces; i++) { vrml_out << " "; }
  vrml_out << "] # end of color\n";
  numSpaces -= TAB_SIZE;
  for(int i = 0; i < numSpaces; i++) { vrml_out << " "; }
  vrml_out << "} # end of Color\n";
  numSpaces -= TAB_SIZE;
  for(int i = 0; i < numSpaces; i++) { vrml_out << " "; }
  vrml_out << "} # end of PointSet\n";
  numSpaces -= TAB_SIZE;
  for(int i = 0; i < numSpaces; i++) { vrml_out << " "; }
  vrml_out << "} # end of Shape\n";
  numSpaces -= TAB_SIZE;
  for(int i = 0; i < numSpaces; i++) { vrml_out << " "; }
  vrml_out << "] # end of children\n";
  numSpaces -= TAB_SIZE;
  for(int i = 0; i < numSpaces; i++) { vrml_out << " "; }
  vrml_out << "} # end of Transform\n";
  numSpaces -= TAB_SIZE;
  for(int i = 0; i < numSpaces; i++) { vrml_out << " "; }
  vrml_out << "] # end of children\n";
  numSpaces -= TAB_SIZE;
  for(int i = 0; i < numSpaces; i++) { vrml_out << " "; }
  vrml_out << "} # end of Group\n";

  // Close the files properly.
  vrml_out.close();
  pts_in.close();
}
