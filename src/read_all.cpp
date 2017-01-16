#include <iostream>
#include <map>
#include <Eigen/Dense>
#include "read_matrices_pose.h"
#include "read_velo_to_cam.h"


using namespace std;

using namespace std;


int main(int argc, char const *argv[])
{
	map<int, Eigen::Matrix4d> matrices;
	read_matrices_pose(matrices);

	Eigen::Affine3d transform;
	read_velo_to_cam(transform);

	return 0;
}

