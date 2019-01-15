#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/bilateral.h>
#include <pcl/filters/fast_bilateral.h>
#include <pcl/filters/fast_bilateral_omp.h>  
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <vtkSmartPointer.h>
#include <vtkPolyData.h>
#include <pcl/io/vtk_lib_io.h>

using namespace std;

int main()
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr  cloud(new pcl::PointCloud<pcl::PointXYZ>);

	//pcl::io::loadPCDFile("F:\\MATLAB projects\\use ply\\bunny.ply", *cloud);
	

	pcl::PolygonMesh mesh;
	vtkSmartPointer<vtkPolyData> polydata = vtkSmartPointer<vtkPolyData>::New();
	pcl::io::loadPolygonFilePLY("F:\\MATLAB projects\\use ply\\bunny.ply", mesh);
	pcl::io::mesh2vtk(mesh, polydata);
	pcl::io::vtkPolyDataToPointCloud(polydata, *cloud);
	pcl::io::savePCDFileASCII("/bunny.pcd", *cloud);

	return 0;
	
	//pcl::FastBilateralFilter<pcl::PointXYZ> bifilter;
	//bifilter.setInputCloud(cloud);

	////bifilter->setSearchMethod(tree);
	//bifilter.setSigmaS(0.5);
	//bifilter.setSigmaR(0.03);
	//bifilter.applyFilter(*out);
	//std::cout << "pointcloud after bifilter filtered has :" << out->points.size() << "data points" << std::endl;


}