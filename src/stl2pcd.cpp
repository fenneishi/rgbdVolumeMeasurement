
#include <iostream>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/common/io.h>
#include <pcl/point_cloud.h>

#include <pcl/PolygonMesh.h>
#include <vtkSmartPointer.h>
#include <vtkPolyData.h>
#include <pcl/io/vtk_lib_io.h>

#include<pcl/io/io.h>
#include<pcl/io/pcd_io.h>//pcd 读写类相关的头文件。
#include<pcl/io/ply_io.h>
#include<pcl/point_types.h> //PCL中支持的点类型头文件。


# include <string>
# include <sstream>
# include "confi.h"
# include <vector>

int user_data;
using namespace std;


const string stlDir=Confi::dirPath_stl;
const string pcdDir=Confi::dirPath_pcd;


int main(int argc,char const *argv[])
{
    // steps
    vector<int> steps{0,15,30,45,60,90,180,360};
	// stl2pcd
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());//创建点云对象
	pcl::PolygonMesh mesh;
	vtkSmartPointer<vtkPolyData> polydata = vtkSmartPointer<vtkPolyData>::New();
	for(auto step:steps)
	{
		string stl_file=stlDir+"step_"+to_string(step)+".stl";
		string pcd_file=pcdDir+"step_"+to_string(step)+".pcd";
		pcl::io::loadPolygonFileSTL(stl_file,mesh);										//PCL利用VTK的IO接口，可以直接读取stl,ply,obj等格式的三维点云数据,传给PolygonMesh对象
		pcl::io::mesh2vtk(mesh, polydata);												//将PolygonMesh对象转化为vtkPolyData对象
		pcl::io::vtkPolyDataToPointCloud(polydata, *cloud);								//获取点云
		pcl::io::savePCDFileASCII(pcd_file, *cloud);//存储为pcb文件						
	}
	return 0;
}