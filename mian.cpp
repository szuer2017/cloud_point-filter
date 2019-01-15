#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/median_filter.h>
#include <pcl/filters/convolution_3d.h>
#include <iostream>
#include <opencv.hpp>

using namespace std;
using namespace cv;

int main(int argc, char** argv)
{

	pcl::PointCloud<pcl::PointXYZ>::Ptr  cloud(new pcl::PointCloud<pcl::PointXYZ>), rad(new pcl::PointCloud<pcl::PointXYZ>), output(new pcl::PointCloud<pcl::PointXYZ>);

	pcl::PointCloud<pcl::PointXYZ>::Ptr  cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>), cloud_pass(new pcl::PointCloud<pcl::PointXYZ>), removeliqun(new pcl::PointCloud<pcl::PointXYZ>);

	pcl::io::loadPCDFile("3d_data.pcd", *cloud);

	boost::shared_ptr<pcl::visualization::PCLVisualizer>  viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	viewer->setBackgroundColor(0, 0, 0);
	viewer->addPointCloud<pcl::PointXYZ>(cloud, "phoneshell");
	std::cout << "pointcloud before filtered has :" << cloud->points.size() << "data points" << std::endl;

	clock_t start, end;
	start = clock();
	std::cout << "begin at" << start << endl;

	//double t1 = GetTickCount();
	//double t2 = (GetTickCount() - t1) / getTickFrequency();

	pcl::VoxelGrid<pcl::PointXYZ> vg;//体素滤波器，精简点云数据
		//pcl::PointCloud<pcl::PointXYZ>::Ptr  cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
		vg.setInputCloud(cloud);
		vg.setLeafSize(0.1f, 0.1f, 0.1f);
		vg.filter(*cloud_filtered);
		std::cout << "pointcloud after VoxelGrid filtered has :" << cloud_filtered->points.size() << "data points" << std::endl;

	int v1(0);
	viewer->createViewPort(0.0, 0.5, 0.5, 1.0, v1);
	viewer->setBackgroundColor(0., 0, 0, v1);
	viewer->addPointCloud<pcl::PointXYZ >(cloud, "originalpoint", v1);

	pcl::PassThrough<pcl::PointXYZ> passt;//直通滤波器
	passt.setInputCloud(cloud_filtered);
	//passt.setFilterFieldName("z");
	passt.setFilterLimits(0.0, 15.0);
	passt.filter(*cloud_pass);
	std::cout << "pointcloud after passfiltered has :" << cloud_pass->points.size() << "data points" << std::endl;

	pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;//移除离群点
	sor.setInputCloud(cloud_pass);
	sor.setMeanK(50);
	sor.setStddevMulThresh(1);
	sor.filter(*removeliqun);
	std::cout << "pointcloud after StatisticalOutlierRemoval  has :" << removeliqun->points.size() << "data points" << std::endl;

	int v2(0);
	viewer->createViewPort(0.5, 0.5, 1.0, 1.0, v2);
	viewer->setBackgroundColor(0, 0.1, 0, v2);
	viewer->addPointCloud<pcl::PointXYZ >(cloud_filtered, "downsample", v2);

	pcl::RadiusOutlierRemoval<pcl::PointXYZ> radiusor;
	radiusor.setInputCloud(removeliqun);
	radiusor.setRadiusSearch(1.0);
	radiusor.setMinNeighborsInRadius(400);  
	radiusor.filter(*rad);
	std::cout << "pointcloud after Radius_filter filtered has :" << rad->points.size() << "data points" << std::endl;


	//实现类似pcl::PointCloud::Ptr和pcl::PointCloud的两个类相互转换
	/*pcl::PointCloud<pcl::PointXYZ> cloud1;
	cloud1 = *rad;
	rad = cloud1.makeShared();
	std::vector<pcl::PointXYZ, Eigen::aligned_allocator<pcl::PointXYZ>> data = cloud1.points;*/

	int v3(0);
	viewer->createViewPort(0.0, 0.0, 0.5, 0.5, v3);
	viewer->setBackgroundColor(0.1, 0, 0, v3);
	viewer->addPointCloud<pcl::PointXYZ >(removeliqun, "radius", v3);


	//pcl::MedianFilter<pcl::PointXYZ> median;// 中值滤波 ，需要有序点云才能进行滤波
	//median.setInputCloud(rad);
	//median.setWindowSize(5);
	//median.setMaxAllowedMovement(0.1);
	//median.applyFilter(*median_filtered);

	/*pcl::filters::GaussianKernel<pcl::PointXYZ, pcl::PointXYZ>::Ptr guass_kernel(new pcl::filters::GaussianKernel<pcl::PointXYZ, pcl::PointXYZ>);

	(*guass_kernel).setSigma(3.375);
	(*guass_kernel).setThresholdRelativeToSigma(3);
	std::cout << "Gaussian_Kernel made" << std::endl;

	pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZ>);
	(*kdtree).setInputCloud(rad);
	pcl::filters::Convolution3D <pcl::PointXYZ, pcl::PointXYZ, pcl::filters::GaussianKernel<pcl::PointXYZ, pcl::PointXYZ>> convolution;
	convolution.setKernel(*guass_kernel);
	convolution.setInputCloud(rad);
	convolution.setSearchMethod(kdtree);
	convolution.setRadiusSearch(3);
	std::cout << "start convolution" << endl;
	convolution.convolve(*output);
	std::cout << "convolution finish" <<endl;
	std::cout << "pointcloud after Guassian_filter filtered has :" << output->points.size() << "data points" << std::endl;
	std::cout << "gaussian_filtered come out" << endl;*/
	
	end = clock();
	std::cout << "End at" << end << endl;
	float duration = float(end - start) / CLOCKS_PER_SEC;
	std::cout << "filtered used " << duration << "second" << endl;

	int v4(0);
	viewer->createViewPort(0.5, 0.0, 1.0, 0.5, v4);
	viewer->setBackgroundColor(0.1,0.1, 0.4, v4);
	viewer->addPointCloud<pcl::PointXYZ >(rad, "cloud_simplify", v4);


	pcl::PointCloud<pcl::PointXYZ>& cloud1 = *rad;
	cloud1.width = cloud1.size();
    cloud1.height = 1;
	cloud1.is_dense = false;
	cloud1.points.resize(cloud1.width * cloud1.height);
	pcl::io::savePCDFileASCII("filtered_cloud.pcd", cloud1);
	std::cout << "saved the filtered pcd data" << endl;

	while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);
	}
}