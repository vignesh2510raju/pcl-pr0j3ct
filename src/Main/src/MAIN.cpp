#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/sac_model_cylinder.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <string>
#include <pcl/filters/passthrough.h>
#include <pcl/common/common.h>
#include <pcl/common/distances.h>
#include <pcl/common/common_headers.h>
#include <iomanip>
#include <sstream>
#include <fstream>
#include <iostream>
#include <map>
#include <Eigen/Dense>
#include <vector>
#include <pwd.h>
#include "../header/read_matrices_pose.h"
#include "../header/read_velo_to_cam.h"
#include "../header/read_transformations.h"
#include "../header/visualize.h"
#include "../header/user_input.h"
#include "../header/extracting_far_away_points.h"
#include "../header/extracting_voxel_grid.h"
#include "../header/cluster_extraction.h"
#include "../header/plane_from_cluster.h"
#include "../header/cylinder_segmentation.h"
#include "../header/store_values_in_vector_of_maps.h"

namespace patch
{
    template < typename T > std::string to_string( const T& n )
    {
        std::ostringstream stm ;
        stm << n ;
        return stm.str() ;
    }
}

int main (int argc, char** argv)
{
  
  int counter= 15; //Counter for number of PCD files that need to be executed


  float number_of_cylinders, number_of_matched_lm , number_of_expected_lm , number_of_readings;

  std::map< int , std::vector<float> > mapping_landmarks;
  std::map< int , std::vector<float> > mapping_coord_landmarks;
  std::vector<float> storeco;

  // User input
  int j, min_cluster_size, min_plane_size, min_cylinder_size; 
  bool options_flag= false;
  std::string filename;
  float max_radius, nd_weight, min_cluster_distance, min_density, xlim, ylim, zlim;  

  max_radius= 0.12f;            
  nd_weight= 0.1f;              
  min_cluster_size= 50;         
  min_plane_size= 250;           
  min_cluster_distance= 0.15;    
  min_density= 400;             
  min_cylinder_size= 25;        
  xlim= 14;                     
  ylim= 14;                     
  zlim= 0.5;

  // Parameters
  bool SWITH_VOXEL_GRID= false; // Downsample the point cloud
  bool SWITH_WRITE_CLUSTERS= false; // Write the clusters to disk 

  // To get the pose
  std::vector<Eigen::Matrix4d> T;
  T = read_transformations();  
  
 for (int fi= 0; fi <= counter; ++fi)
 {
 
   std::string filename2; 
  std::stringstream sa;
  sa << setw(6) << setfill('0') << fi;
  filename2= sa.str();

  // Read in the cloud data
  pcl::PCDReader reader;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>), cloud_f (new pcl::PointCloud<pcl::PointXYZ>);
  std::cout << "File currently being used :"<< filename2 << ".pcd" << endl;
  reader.read ("../Data/pcd-files/Test/" + filename2 + ".pcd", *cloud);
  //std::cout << "PointCloud before filtering has: " << cloud->points.size () << " data points." << std::endl; //*

  // Eliminate far-away points
  cloud = extracting_far_away_points(cloud, xlim, ylim, zlim);

  // Create the filtering object: downsample the dataset using a leaf size of 0.75cm
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
  cloud_filtered = extracting_voxel_grid(cloud, SWITH_VOXEL_GRID);

  // Creating the KdTree object for the search method of the extraction of clusters
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  //tree->setInputCloud (cloud_filtered);

  // Clustering 
  std::map<int, pcl::PointCloud<pcl::PointXYZ>::Ptr > clusters;
  clusters = cluster_extraction (cloud_filtered, min_cluster_distance, min_cluster_size, SWITH_WRITE_CLUSTERS);

  // Extract Planes from clusters
  clusters = plane_from_cluster(clusters, min_cluster_size, min_plane_size, min_density);

  // Cylinder Segmentation 
  std::map<int, pcl::PointCloud<pcl::PointXYZ>::Ptr > cylinders;
  cylinders = cylinder_segmentation(clusters, nd_weight, max_radius, min_cylinder_size, counter, fi);

  // Visualization
  pcl::visualization::PCLVisualizer viewer("PCL Viewer");
  viewer.setBackgroundColor( 0.0, 0.0, 0.0 );
  //viewer.setFullScreen(true); 
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> white_color (cloud, 255, 255, 255);
  viewer.addPointCloud <pcl::PointXYZ>(cloud_filtered, white_color, "cloud");
  viewer = visualize(viewer, cylinders, "cylinders", true, true);

  // Run the viewer
  //while (!viewer.wasStopped ())
  // {
     viewer.spinOnce (0.01);
  // }
 }  
  return (0);
}
