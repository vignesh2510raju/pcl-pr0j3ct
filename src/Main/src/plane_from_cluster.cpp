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
#include "/home/vignesh/pcl-proyect/src/Main/header/plane_from_cluster.h"

std::map<int, pcl::PointCloud<pcl::PointXYZ>::Ptr > plane_from_cluster (std::map<int, pcl::PointCloud<pcl::PointXYZ>::Ptr > clusters, 
                                                                                                int min_cluster_size, int min_plane_size, float min_density)
{
   // Extract Planes from clusters
   pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal> seg1;
   pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
   pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
   pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZ> ());
   seg1.setOptimizeCoefficients (true);
   seg1.setModelType (pcl::SACMODEL_NORMAL_PLANE);
   seg1.setMethodType (pcl::SAC_RANSAC);
   seg1.setMaxIterations (1000);
   seg1.setDistanceThreshold (0.01);
   seg1.setNormalDistanceWeight (0.1);

   // Create the normal estimation class, and pass the input dataset to it
   pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;

   int j= 0;
   while(j < clusters.size())
   {
     while (clusters[j]->points.size () > min_cluster_size)
     {
        ne.setInputCloud (clusters[j]);

        // Create an empty kdtree representation, and pass it to the normal estimation object.
        // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
        
        ne.setSearchMethod (tree);

        // Output datasets
        pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);

        // Use all neighbors in a sphere of radius 3cm
        ne.setRadiusSearch (0.03);

        // Compute the features
        ne.compute (*cloud_normals);

        // Segment the largest planar component from the remaining cluster cloud
        seg1.setInputCloud (clusters[j]);
        seg1.setInputNormals (cloud_normals);

        seg1.segment (*inliers, *coefficients);
        
        if (inliers->indices.size () == 0)
        {
          std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
          break;
        }

        // Extract the planar inliers from the input cloud
        pcl::ExtractIndices<pcl::PointXYZ> extract;
        
        extract.setInputCloud (clusters[j]);
        extract.setIndices (inliers);
        extract.setNegative (false);

        // Get the points associated with the planar surface
        extract.filter (*cloud_plane);

        // Extract plane
        if (cloud_plane->points.size () > min_plane_size)
        {
          // Remove the planar inliers, extract the rest
          extract.setNegative (true);
          extract.filter (*clusters[j]);
          std::cout << "PointCloud representing the planar component from cluster "<< j <<" has: " << cloud_plane->points.size () << " data points." << std::endl;
          // If the remainder is small -> remove this cluster b/c there is no cylinder
          if (clusters[j]->points.size() < min_cluster_size)
          {
            clusters[j].reset();
            break; // out of the current cluster loop
          }
        }
        else
        {
          //cout<< "The plane has too few points"<< endl;
          break;
        } 
      }

      j++;
    }  

   cout<< "map size after eliminating the small clusters is: "<< clusters.size()<< endl;

    // Check the density of clustes, if it's too low -> eliminate the cluster
    pcl::PointXYZ pmin, pmax;
    double maxDistance, area, density;

    j= 0;
    while(j < clusters.size())
    {
      // maxDistance= pcl::getMaxSegment(*clusters[j], pmin, pmax);
      area= pcl::calculatePolygonArea (*clusters[j]);
      density= clusters[j]->points.size() / area;

      std::cout<<"Cluster " << j<< "   #points: "<<clusters[j]->points.size()<< "     Density: "<< density<< "     Area: "<< area<< endl;

      if (density < min_density)
      { 
        clusters[j].reset();
      }
      j++;
    }

    return clusters;
}