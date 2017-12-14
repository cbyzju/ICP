#pragma once
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/point_types.h>
typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloud;

struct position {
	float x_min;
	float y_min;
	float x_max;
	float y_max;
	position(float x_min_, float y_min_, float x_max_, float y_max_) : x_min(x_min_), y_min(y_min_), x_max(x_max_), y_max(y_max_) {}
};

struct color {
	float r;
	float g;
	float b;
	color(float r_, float g_, float b_) : r(r_), g(g_), b(b_) {}
};

struct cloud {
	PointCloud::Ptr cloudPtr;
	position pos = position(0, 0, 1, 1);
	color backgroundColor = color(0, 0, 0);
	color pointColor = color(128, 128, 128);
	int pointSize = 3;
	std::string context = "PointCloud";
	bool showOriginColor = true;
	int viewport = 0;
	cloud(const PointCloud::Ptr cloudPtr_, const position& pos_, const color& backgroundColor_, const color& pointColor_, int pointSize_, const std::string& context_, bool _showOriginColor, int viewport_) :
		cloudPtr(cloudPtr_), pos(pos_), backgroundColor(backgroundColor_), pointColor(pointColor_), pointSize(pointSize_), context(context_), showOriginColor(_showOriginColor), viewport(viewport_) {}
	cloud(const PointCloud::Ptr cloudPtr_, const color& pointColor_, int pointSize_, const std::string& context_, bool _showOriginColor) :
		cloudPtr(cloudPtr_), pointColor(pointColor_), pointSize(pointSize_), context(context_), showOriginColor(_showOriginColor) {}
	cloud(const PointCloud::Ptr cloudPtr_, int pointSize_, const std::string& context_, bool _showOriginColor) :
		cloudPtr(cloudPtr_), pointSize(pointSize_), context(context_), showOriginColor(_showOriginColor) {}
	cloud(const PointCloud::Ptr cloudPtr_, int pointSize_, const std::string& context_) :
		cloudPtr(cloudPtr_), pointSize(pointSize_), context(context_) {}
};

class display {
	public:
		//void show(const boost::shared_ptr<pcl::visualization::PCLVisualizer>& viewer, const PointCloud::Ptr& cloud, const position& pos = position(0, 0, 1, 1), const color& backgroundColor = color(0, 0, 0),  const color& pointColor = color(128, 128, 128), int pointSize = 3, const std::string& context = "PointCloud", int viewport = 0);
		void show(const boost::shared_ptr<pcl::visualization::PCLVisualizer>& viewer, std::vector<cloud> cloudProperty);
		void show(const boost::shared_ptr<pcl::visualization::PCLVisualizer>& viewer, std::vector<cloud> cloudProperty, const position& pos, const color& backgroundColor = color(0, 0, 0), int viewport = 0);
};
