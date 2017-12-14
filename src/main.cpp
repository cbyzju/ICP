
#include "icp.h"
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include "display.h"
#include "timer.h"
using namespace nanoflann;
using namespace std;

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloud;

void PointCloudToEigenMatrix(const PointCloud::Ptr& source, Eigen::Matrix<double, Eigen::Dynamic, 3>& destination);
void EigenMatrixToPointCloud(const Eigen::Matrix<double, Eigen::Dynamic, 3>& source, PointCloud::Ptr& destination);
int  loadPointCloud(const std::string& source_path, const PointCloud::Ptr& source);
void EigenMatrixToTxt(const Eigen::Matrix<double, Eigen::Dynamic, 3>& source, const std::string& file_path);

int main()
{
	const std::string target_path = "../../dataset/cloud0000.pcd";
	const std::string source_path = "../../dataset/cloud0033.pcd";
	//const std::string target_path = "../../dataset/cloud0.pcd";
	//const std::string source_path = "../../dataset/cloud20.pcd";
	//const std::string target_path = "../../dataset/yuanlin18SourcePoints.xyz";
	//const std::string source_path = "../../dataset/yuanlin18modelPoints.xyz";
	
	PointCloud::Ptr target(new PointCloud());
	PointCloud::Ptr source(new PointCloud());

	if (loadPointCloud(target_path, target) < 0){std::cerr << "ERROR: load target " << target_path << " error." << std::endl;return -1;}
	if (loadPointCloud(source_path, source) < 0){std::cerr << "ERROR: load source " << source_path << " error." << std::endl;return -1;}

	Eigen::Matrix<double, Eigen::Dynamic, 3> targetPoints;
	targetPoints.resize(target->points.size(), 3);
	Eigen::Matrix<double, Eigen::Dynamic, 3> sourcePoints;
	sourcePoints.resize(source->points.size(), 3);

	//cout << "target size = " << target->points.size() << endl;
	//cout << "source size = " << source->points.size() << endl;
	PointCloudToEigenMatrix(target, targetPoints);
	PointCloudToEigenMatrix(source, sourcePoints);

	//EigenMatrixToTxt(targetPoints, "target.txt");
	//EigenMatrixToTxt(sourcePoints, "source.txt");
	
	timer icpTime;
	netease::icp myicp(targetPoints, sourcePoints);
	myicp.setSourceSamplingProb(0.2);
	myicp.setTargetSamplingProb(0.2);
	//myicp.setMaximumIterations(20);
	//myicp.setTrimmedDistOutlierFilterRatio(0.8);
	Eigen::Matrix<double, 4, 4> transformation = myicp.compute();
	std::cout << "cost time = "<< icpTime.getClock() <<" ms." << endl;
	std::cout << "final transformation:\n" << transformation << std::endl;
	netease::icpResult result = myicp.getResult();
	std::cout << " iteration = " << result.iterations << ", MSE = " << result.MSE <<", epsilon = "<<result.epsilon<< endl;
	PointCloud::Ptr cloud_transformed(new PointCloud());
	pcl::transformPointCloud(*source, *cloud_transformed, transformation);

	pcl::PLYWriter writer;
	writer.write("../../dataset/result.ply", *cloud_transformed);

	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	display PointCloudDisplay;
	std::vector<cloud> cloudShow12;
	cloudShow12.push_back(cloud(target, position(0.0, 0.5, 0.5, 1.0), color(1, 1, 1), color(255, 0, 0), 3, "target", true, 0));
	cloudShow12.push_back(cloud(source, position(0.5, 0.5, 1.0, 1.0), color(1, 1, 1), color(0, 255, 0), 3, "source", true, 1));
	PointCloudDisplay.show(viewer, cloudShow12);
	std::vector<cloud> cloudShow3;
	cloudShow3.push_back(cloud(target, color(255, 0, 0), 3, "cloud_target", false));
	cloudShow3.push_back(cloud(source, color(0, 255, 0), 3, "cloud_source", false));
	PointCloudDisplay.show(viewer, cloudShow3, position(0.0, 0.0, 0.5, 0.5), color(1, 1, 1), 3);
	std::vector<cloud> cloudShow4;
	cloudShow4.push_back(cloud(target, color(255, 0, 0), 3, "together_target", false));
	cloudShow4.push_back(cloud(cloud_transformed, color(0, 255, 0), 3, "together_source", false));
	PointCloudDisplay.show(viewer, cloudShow4, position(0.5, 0.0, 1.0, 0.5), color(1, 1, 1), 4);

	while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}

	return 0;
}


int loadPointCloud(const std::string& source_path, const PointCloud::Ptr& source)
{
	const std::string source_ext = source_path.substr(source_path.size() - 3, source_path.size());
	if (source_ext.compare("pcd") == 0)
	{
		if (pcl::io::loadPCDFile(source_path, *source) < 0)
		{
			std::cerr << "Error loading " << source_path << std::endl;
			return -1;
		}
	}
	else if (source_ext.compare("ply") == 0)
	{
		if (pcl::io::loadPLYFile(source_path, *source) < 0)
		{
			std::cerr << "Error loading " << source_path << std::endl;
			return -1;
		}
	}
	else if (source_ext.compare("xyz") == 0 || source_ext.compare("txt") == 0)
	{
		ifstream infile;
		infile.open(source_path);
		if (!infile) { cout << "failed to open " + source_path << endl; return -1; }
		float x, y, z;
		int npts = 0;
		while (infile >> x >> y >> z)
		{
			PointT point;
			point.x = x;
			point.y = y;
			point.z = z;
			//point.b = 255;
			//point.g = 255;
			//point.r = 0;
			source->points.push_back(point);
			npts++;
		}
		infile.close();
		source->height = 1;
		source->width = npts;
		source->points.resize(npts);
	}
	else
	{
		std::cerr << "we don't support this file format right now, please use ply/pcd format!" << std::endl;
		return -1;
	}
	return 0;
}

void PointCloudToEigenMatrix(const PointCloud::Ptr& source, Eigen::Matrix<double, Eigen::Dynamic, 3>& destination)
{
	for (int i = 0; i < source->points.size(); ++i)
	{
		destination(i, 0) = source->points[i].x;
		destination(i, 1) = source->points[i].y;
		destination(i, 2) = source->points[i].z;
	}
}

void EigenMatrixToPointCloud(const Eigen::Matrix<double, Eigen::Dynamic, 3>& source, PointCloud::Ptr& destination)
{
	int rows = source.rows();
	destination->height = 1;
	destination->width = rows;
	destination->resize(rows);
	for (int i = 0; i < rows; ++i)
	{
			destination->points[i].x = source(i, 0);
			destination->points[i].y = source(i, 1);
			destination->points[i].z = source(i, 2);
	}
}

void EigenMatrixToTxt(const Eigen::Matrix<double, Eigen::Dynamic, 3>& source, const std::string& file_path)
{
	std::ofstream outfile;
	outfile.open(file_path, 'w');
	if (!outfile) { std::cout << "failed to open " << file_path << std::endl; return; }
	//clock_t start = clock();
	//#pragma omp parallel for
	for (int i = 0; i < source.rows(); i++)
	{
		outfile << source(i, 0) << " " << source(i, 1) << " " << source(i, 2) << std::endl;
	}
	//clock_t end = clock();
	//std::cout << "cost time = " << (float(end - start)) / CLOCKS_PER_SEC << std::endl;
	outfile.close();
}
