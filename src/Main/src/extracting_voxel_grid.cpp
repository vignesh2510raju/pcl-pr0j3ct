#include "/home/vignesh/pcl-proyect/src/Main/header/extracting_voxel_grid.h"
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>

pcl::PointCloud<pcl::PointXYZ>::Ptr extracting_voxel_grid (pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, bool SWITH_VOXEL_GRID)
{
  // Create the filtering object: downsample the dataset using a leaf size of 1cm
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
  if (SWITH_VOXEL_GRID)
  {
    pcl::VoxelGrid<pcl::PointXYZ> vg;
    vg.setInputCloud (cloud);
    vg.setLeafSize (0.095f, 0.095f, 0.095f);
    vg.filter (*cloud_filtered);
    std::cout << "PointCloud after filtering has: " << cloud_filtered->points.size ()  << " data points." << std::endl; 
  }
  else
  {
    cloud_filtered= cloud;
    std::cout << "PointCloud not filted - same as original cloud" << std::endl; 
  }

  return cloud_filtered;
}