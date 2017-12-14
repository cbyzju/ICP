#include "display.h"

void display::show(const boost::shared_ptr<pcl::visualization::PCLVisualizer>& viewer, std::vector<cloud> cloudProperty) 
{
	for (int i = 0; i < cloudProperty.size(); ++i)
	{
		if (cloudProperty[i].showOriginColor)
		{
			viewer->createViewPort(cloudProperty[i].pos.x_min, cloudProperty[i].pos.y_min, cloudProperty[i].pos.x_max, cloudProperty[i].pos.y_max, cloudProperty[i].viewport);
			viewer->setBackgroundColor(cloudProperty[i].backgroundColor.r, cloudProperty[i].backgroundColor.g, cloudProperty[i].backgroundColor.b, cloudProperty[i].viewport);
			viewer->addText(cloudProperty[i].context, 10, 10, 15, 0.5, 0.5, 0.5, cloudProperty[i].context[i] + "_cloud", cloudProperty[i].viewport);
			viewer->addPointCloud<PointT>(cloudProperty[i].cloudPtr, cloudProperty[i].context + "_point", cloudProperty[i].viewport);
			viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, cloudProperty[i].pointSize, cloudProperty[i].context + "_point", cloudProperty[i].viewport);
		}
		else
		{
			viewer->createViewPort(cloudProperty[i].pos.x_min, cloudProperty[i].pos.y_min, cloudProperty[i].pos.x_max, cloudProperty[i].pos.y_max, cloudProperty[i].viewport);
			viewer->setBackgroundColor(cloudProperty[i].backgroundColor.r, cloudProperty[i].backgroundColor.g, cloudProperty[i].backgroundColor.b, cloudProperty[i].viewport);
			viewer->addText(cloudProperty[i].context, 10, 10, 15, 0.5, 0.5, 0.5, cloudProperty[i].context[i] + "_cloud", cloudProperty[i].viewport);
			pcl::visualization::PointCloudColorHandlerCustom<PointT> scolor(cloudProperty[i].cloudPtr, cloudProperty[i].pointColor.r, cloudProperty[i].pointColor.g, cloudProperty[i].pointColor.b);
			viewer->addPointCloud<PointT>(cloudProperty[i].cloudPtr, scolor, cloudProperty[i].context + "_point", cloudProperty[i].viewport);
			viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, cloudProperty[i].pointSize, cloudProperty[i].context + "_point", cloudProperty[i].viewport);
		}
		
	}
}

void display::show(const boost::shared_ptr<pcl::visualization::PCLVisualizer>& viewer, std::vector<cloud> cloudProperty, const position& pos, const color& backgroundColor, int viewport)
{
	viewer->createViewPort(pos.x_min, pos.y_min, pos.x_max, pos.y_max, viewport);
	viewer->setBackgroundColor(backgroundColor.r, backgroundColor.g, backgroundColor.b, viewport);
	
	for (int i = 0; i < cloudProperty.size(); ++i)
	{
		if (cloudProperty[i].showOriginColor)
		{
			viewer->addPointCloud<PointT>(cloudProperty[i].cloudPtr, cloudProperty[i].context + "_morepoint", viewport);
			viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, cloudProperty[i].pointSize, cloudProperty[i].context + "_morepoint", viewport);
		}
		else
		{
			pcl::visualization::PointCloudColorHandlerCustom<PointT> scolor(cloudProperty[i].cloudPtr, cloudProperty[i].pointColor.r, cloudProperty[i].pointColor.g, cloudProperty[i].pointColor.b);
			viewer->addPointCloud<PointT>(cloudProperty[i].cloudPtr, scolor, cloudProperty[i].context + "_morepoint", viewport);
			viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, cloudProperty[i].pointSize, cloudProperty[i].context + "_morepoint", viewport);
		}	
	}
}