#include <iostream>
#include <map>
#include <Eigen/Dense>
#include "read_matrices_pose.h"
#include "read_velo_to_cam.h"
#include "read_transformations.h"
#include <vector>

using namespace std;

int main(int argc, char const *argv[])
{
	vector<Eigen::Matrix4d> T;
	T= read_transformations();

	for (int i = 0; i < 20; ++i)
	{
		cout<< T[i]<<endl<<endl;
	}
}