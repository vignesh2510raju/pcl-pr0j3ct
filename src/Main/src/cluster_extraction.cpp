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
#include "/home/vignesh/pcl-proyect/src/Main/header/cluster_extraction.h"

std::map<int, pcl::PointCloud<pcl::PointXYZ>::Ptr > cluster_extraction (pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered, float min_cluster_distance, int min_cluster_size, 
                                                                                    bool SWITH_WRITE_CLUSTERS)
{   

  // Creating the KdTree object for the search method of the extraction of clusters
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud (cloud_filtered);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance (min_cluster_distance);
  ec.setMinClusterSize (min_cluster_size);
  // ec.setMaxClusterSize (250000);  // very high value
  ec.setSearchMethod (tree);
  ec.setInputCloud (cloud_filtered);
  ec.extract (cluster_indices);

  // clusters contains all clusters in a map
  std::map<int, pcl::PointCloud<pcl::PointXYZ>::Ptr > clusters;
  int j = 0;
  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
    for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
      cloud_cluster->points.push_back (cloud_filtered->points[*pit]); 
    cloud_cluster->width = cloud_cluster->points.size ();
    cloud_cluster->height = 1;
    cloud_cluster->is_dense = true;

    std::cout << "Cluster " <<j << " has " << cloud_cluster->points.size () << " points." << std::endl;

    // If we want to save to clusters to disk - false default
    if (SWITH_WRITE_CLUSTERS)
    {
      pcl::PCDWriter writer;
      std::stringstream ss;
      ss << "cloud_cluster_" << j << ".pcd";
      writer.write<pcl::PointXYZ> (ss.str (), *cloud_cluster, false); 
    }

    // save the cluster in the map
    clusters[j] = cloud_cluster;

    j++;
  } 
  cout<< "map size before eliminating the planes is: "<< clusters.size()<< endl;

  return clusters;

}