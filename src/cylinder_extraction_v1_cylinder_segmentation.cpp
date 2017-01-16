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
#include "cylinder_extraction_v1_cylinder_segmentation.h"

std::map<int, pcl::PointCloud<pcl::PointXYZ>::Ptr > cylinder_extraction_v1_cylinder_segmentation (std::map<int, pcl::PointCloud<pcl::PointXYZ>::Ptr > clusters, 
                                                                                                float nd_weight, float max_radius, int min_cylinder_size, int counter, int fi)
{

  float imagearray[counter][50];

  // Fit cylinders to the remainding clusters
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
  pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal> seg_normals; 
  pcl::ExtractIndices<pcl::PointXYZ> extract;

  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
  pcl::ModelCoefficients::Ptr coefficients_cylinder (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers_cylinder (new pcl::PointIndices);

  // Estimate point normals
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  ne.setSearchMethod (tree);

  // Create the segmentation object for cylinder segmentation and set all the parameters
  seg_normals.setOptimizeCoefficients (true);
  seg_normals.setModelType (pcl::SACMODEL_CYLINDER);
  seg_normals.setMethodType (pcl::SAC_RANSAC);
  seg_normals.setNormalDistanceWeight (nd_weight); // 0.1
  seg_normals.setMaxIterations (10000);
  seg_normals.setDistanceThreshold (0.15); // 0.2
  seg_normals.setRadiusLimits (0.0, max_radius); // 0.1
  seg_normals.setAxis (Eigen::Vector3f (0.0, 0.0, 1.0));
  seg_normals.setEpsAngle (pcl::deg2rad (15.0f));

  // Obtain the cylinder inliers and coefficients for each cluster (just one cylinder per cluster)
  std::map<int, pcl::PointCloud<pcl::PointXYZ>::Ptr > cylinders;

  int j, k = 0;

  j= 0;
  while(j < clusters.size() )
  {
    // allocate new memory to the cloud cylinder, the one before has been saved to cloud cylinder
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cylinder (new pcl::PointCloud<pcl::PointXYZ> ());

    if (clusters[j] != NULL)
    {
      // compute normals
      ne.setInputCloud (clusters[j]);
      ne.setKSearch (20); // 50
      ne.compute (*cloud_normals);

      // Segment cylinder
      seg_normals.setInputCloud (clusters[j]);
      seg_normals.setInputNormals (cloud_normals);
      seg_normals.segment (*inliers_cylinder, *coefficients_cylinder);

      // Extract indices
      extract.setInputCloud (clusters[j]);
      extract.setIndices (inliers_cylinder);
      extract.setNegative (false);
      extract.filter (*cloud_cylinder);
      if (cloud_cylinder->points.empty ()) 
      {
        //std::cerr << "Can't find the cylindrical component." << std::endl;
        cylinders[j].reset();
      }
      else
      {
        std::cout << "PointCloud representing the cylindrical component: " << cloud_cylinder->points.size () << " data points." << std::endl;
        // std::cout<< "Coefficients of the cylinder: "<< *coefficients_cylinder << endl;
        if (cloud_cylinder->points.size() > min_cylinder_size)
        {
          cylinders[j]= cloud_cylinder;

          pcl::CentroidPoint<pcl::PointXYZ> centroid;

          for (int z = 0; z < min_cylinder_size ; z++)
          {
              centroid.add (pcl::PointXYZ ( cloud_cylinder->points[ j + z ].x , cloud_cylinder->points[ j + z ].y , cloud_cylinder->points[ j + z ].z ));              
          }
          pcl::PointXYZ cl;
          centroid.get (cl);
          
          //Storing the values of the Centroid into the Image Array Matrix
          imagearray[fi][k] = ((int)(cl.x * 10 + 0.5) / 10.0 ); //rounding off to 1 decimal place
          imagearray[fi][k+1] = ((int)(cl.y * 10 + 0.5) / 10.0 ); //rounding off to 1 decimal place

          std::cout << "Centroid is : " << imagearray[fi][k] << ',' << imagearray[fi][k+1] << ' ' << std::endl;

          k = k+2;

          double dist;
          pcl::PointXYZ dist2;
          dist2.x = 0.0;
          dist2.y = 0.0;
          dist2.z = 0.0;
          dist = pcl::euclideanDistance ( cl , dist2 );
          dist = ((int)(dist * 10 + 0.5) / 10.0 ); 
          std::cout << "Distance from the car - " << dist << std::endl;
        }
        else
        {
          cylinders[j].reset();
        }
      }
    }

    j++;
  }

  return cylinders;
}