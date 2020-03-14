#include <iostream>
#include <fstream>
#include "param.h"
#include <vector>
#include <array>
#include <sstream>

using std::string;
using std::vector;
using std::stringstream;

int main()
{
	params param;
	vector<float> timestamp;
	vector<std::array<float, 3>> selfCarPose;
	std::ifstream timeStream;
	std::ifstream poseStream;
	string lineStr;
	char ch;
	string timeFileDir = param.kitti_base_velo_dir + "20/info/timestamp.txt";
	std::cout << timeFileDir <<std::endl;
	timeStream.open(timeFileDir, std::ios::in);
	if (timeStream.fail())
	{
		fprintf(stderr, "open timeStream error\n");
		return 0;
	}
	// while(timeStream >> ch)
	while(timeStream >> lineStr)
	{
		// std::cout << ch << std::endl;
		// std::cout << lineStr << std::endl;
		timestamp.emplace_back(std::stof(lineStr));
	}
	// for (int idx = 0; idx < timestamp.size(); ++idx)
	// {
	// 	fprintf(stderr, "%f\n", timestamp[idx]);
	// }
	// fprintf(stderr, "num of timestamp : %d\n", timestamp.size());
	timeStream.close();
	// -------------------------------------------------------------
	string poseFileDir = param.kitti_base_velo_dir + "20/info/pose.txt";
	poseStream.open(poseFileDir, std::ios::in);
	if (poseStream.fail())
	{
		fprintf(stderr, "open poseStream error\n");
		return 0;
	}
	int count = 0;
	while (getline(poseStream, lineStr, '\n'))
	{
		string str;
		int strIdx = 0;
		std::array<float, 3> ps;
		stringstream ss(lineStr);
		// while(getline(ss, str, ' '))
		// {
		// 	ps[strIdx] = std::stof(str);
		// 	strIdx++;
		// }
		ss >> ps[0] >> ps[1] >> ps[2];
		selfCarPose.emplace_back(ps);
	}
	// --------------------------------------------------------------
	// for (int idx = 0; idx < 100; ++idx)
	// {
	// 	fprintf(stderr, "%f, %f, %f\n", selfCarPose[idx][0], selfCarPose[idx][1], selfCarPose[idx][2]);
	// }
	poseStream.close();
	// --------------------------------------------------------------

	fprintf(stderr, "test2.cpp done\n"); 
	return 0;
}