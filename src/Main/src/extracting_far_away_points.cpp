#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include "/home/vignesh/pcl-proyect/src/Main/header/extracting_far_away_points.h"

pcl::PointCloud<pcl::PointXYZ>::Ptr extracting_far_away_points (pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int xlim, int ylim, int zlim)
{ 

  // eliminate far-away x points
  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud (cloud);
  pass.setFilterFieldName ("x");
  pass.setFilterLimits (-xlim, xlim);
  //pass.setFilterLimitsNegative (true);
  pass.filter (*cloud);

  // eliminate far-away y points
  pass.setInputCloud (cloud);
  pass.setFilterFieldName ("y");
  pass.setFilterLimits (-ylim, ylim);
  //pass.setFilterLimitsNegative (true);
  pass.filter (*cloud);
  
  // eliminate far-away z points
  pass.setInputCloud (cloud);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (-zlim, 14.0);
  //pass.setFilterLimitsNegative (true);
  pass.filter (*cloud);

  return cloud;
}