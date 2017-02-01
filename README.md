README

This is a brief explanation of what you'll find in this folder. You don't need to go through all of it, that would be a wasted of time. Please refer to the PCL documentation to make this programs run.

The folder structure is as follows :

CmakeFiles: No need to look in here. Files needed for compilation.

Data : 
Calibration: this folder contains transformation matrices needed. Much more information can be found on the KITTI page: http://www.cvlibs.net/datasets/kitti/. Please take a look at the “Odometry” and “Raw data” tabs in the link above. You'll find plenty of information there, here you have two links to papers explaining how the KITTI dataset is organized:
- Odometry: http://www.cvlibs.net/publications/Geiger2012CVPR.pdf
- Raw data: http://www.cvlibs.net/publications/Geiger2013IJRR.pdf
pcd-files/ Test : This are the binary files in the PCL format which is “.pcd”. You load this files one by one when using the LiDAR for navigation.
poses : stores the ground truth, which is a combination of GPS and IMU (inertial measure system). Each of the 10 files contatined in this folder specifies a few thousands of poses. At this point we are just working with “00.txt”. Again, more information about how the poses are stored can be find in the papers and the online documentation of the KITTI dataset.

build: This folder contains the executable files specified by “CmakeLists.txt” which is located in the “pcl-pr0j3ct” directory. 

Structure of MAIN.cpp (read this when you are reading about MAIN.cpp) :
Read the PCD files
Extract and remove far away points
Downsample using Voxel grid with a predefined leaf size
Create a KDTree object to extract clusters 
Extract planes from the clusters and remove them
Run the cylinder segmentation algorithm to extract cylinders
Visualize the cylinders
Repeat for other PCD files

src :
The main folder contains examples that can be found in PCL tutorials page. The ones with the words “read” or “write” are not very interesting, some of them are the ones I used to read the information from the text and binary files of the KITTI dataset. The ones related to features extraction or segmentation are:
- bare_earth: extracts the ground points
- don_segmentation: DoN filter explained here :http://pointclouds.org/documentation/tutorials/don_segmentation.php.
- example_sift_keypoint_estimation / example_sift_normal_keypoint_estimation /example_sift_z_keypoint_estimation: these three files use different variants of the SIFT algorithm. SIFT is a very popular algorithm in computer vision that has been adapted to 3D lidar feature extraction. I don't know exactly how it works at this point. This examples where taken from https://github.com/otherlab/pcl/tree/master/examples/keypoints. There are more examples there from many other algorithms, take a look.
- extract_ground_and_don: The idea here was combine the “bare_earth.cpp” algorithm and the don segmentation together, but I never finish it....
- narf_keypoint_extraction: There is a PCL  tutorial that explains how to extract NARF keypoints, it's here http://pointclouds.org/documentation/tutorials/narf_keypoint_extraction.php.
- planar_segmentation: This algorithm uses RANSAC to extract planes from the laser scan. There are a few parameters to tune here, but I never made it work properly. There is a tutorial that teaches you how to do this, here http://pointclouds.org/documentation/tutorials/planar_segmentation.php and here http://www.pointclouds.org/documentation/tutorials/cluster_extraction.php.
- range_image_border_extraction: There is a tutorial for this too. Uses a technique very similar to the one used in NARF to extract the borders. NARF keypoints start from here to extract the keypoints later. The tutorial is here http://pointclouds.org/documentation/tutorials/range_image_border_extraction.php.
Main 
src : Contains C++ files. The MAIN.cpp is the code for cylinder extraction from KITTI dataset. The other cpp files in this folder are functions that are called by MAIN.cpp. Feel free to get back to me in case I’ve missed comments for certain functions.
cluster_extraction.cpp : You can find the explanation here. http://www.pointclouds.org/documentation/tutorials/cluster_extraction.php
cylinder_segmentation.cpp : http://pointclouds.org/documentation/tutorials/cylinder_segmentation.php
extracting_voxel_grid.cpp : We use a leaf size of 0.75cm to downsample the point cloud http://pointclouds.org/documentation/tutorials/voxel_grid.php
plane_from_cluster.cpp :  http://www.pointclouds.org/documentation/tutorials/planar_segmentation.php
read_matrices_pose.cpp : code to read the pose of the car
visualize.cpp : Code to visualize the point clouds. http://pointclouds.org/documentation/tutorials/pcl_visualizer.php
    -   header : Contains the header files of the src folder	

