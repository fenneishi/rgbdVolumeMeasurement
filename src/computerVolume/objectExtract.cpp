#include <iostream>
#include <string>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/extract_indices.h>
#include "confi.h"
using namespace std;

const string inputPCD(dirPath_pcd);
const string outputPCD_ground(dirPath_pcd_ground);
const string outputPCD_object(dirPath_pcd_object);
const string outputPCD_object(dirPath_pcd_other);
 
void viewerOneOff(pcl::visualization::PCLVisualizer& viewer)
{
    viewer.setBackgroundColor(0,0,1);
}
 
int main (int argc, char** argv)
{
    
    // 建立点云对象（用来保存输入pcd文件）
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    // 建立点云对象（用来保存最后分割出的平面 or Object）
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    // 读入点云PCD文件 
    pcl::PCDReader reader;
    reader.read(inputPCD,*cloud);
    // 输出原始点云规模
    // std::cerr << "Point cloud data: " << cloud->points.size () << " points" << std::endl;
    // 创建一个模型参数对象，用于记录结果(平面方程)
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    // inliers表示误差能容忍的点 记录的是点云的序号
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    // Create the segmentation object(创建一个分割器)
    pcl::SACSegmentation<pcl::PointXYZ> seg;


    // 分割器设置
    // Optional-这个设置可以选定结果平面展示的点是分割掉的点还是分割剩下的点。(true是展示剩下的点)
    seg.setOptimizeCoefficients (true);
    // Mandatory-设置目标几何形状
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    // 距离阈值 单位m
    seg.setDistanceThreshold (0.005);
    // 输入点云
    seg.setInputCloud (cloud);

    //分割
    //分割点云，同时获得平面和法向量
    seg.segment (*inliers, *coefficients);
    if (inliers->indices.size () == 0)
    {
        PCL_ERROR ("Could not estimate a planar model for the given dataset.");
        return (-1);
    }
 

    // 结果显示与保存
    // 打印出平面方程
    std::cout<<"the plane is ax+by+cz+d=0"<<std::endl;
	std::cout << "a：" << coefficients->values[0] << endl;
	std::cout << "b：" << coefficients->values[1] << endl;
	std::cout << "c：" << coefficients->values[2] << endl;
    std::cout << "d：" << coefficients->values[3] << endl;
    // 提取地面and write to file
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud (cloud);
    extract.setIndices (inliers);
    extract.filter (*cloud_filtered);
    // std::cerr << "Ground cloud after filtering: " << std::endl;
    // std::cerr << *cloud_filtered << std::endl;
    pcl::PCDWriter writer;
    writer.write<pcl::PointXYZ> (outputPCD_ground, *cloud_filtered, false);
    // 提取除地面外的物体and write to file
    extract.setNegative (true);
    extract.filter (*cloud_filtered);
    // std::cerr << "Object cloud after filtering: " << std::endl;
    // std::cerr << *cloud_filtered << std::endl;
    writer.write<pcl::PointXYZ> (outputPCD_object, *cloud_filtered, false);
    // // 点云可视化
    // pcl::visualization::CloudViewer viewer("Filtered");
    // viewer.showCloud(cloud_filtered);
    // viewer.runOnVisualizationThreadOnce(viewerOneOff);
    // while(!viewer.wasStopped()){
    // }
	//显示对象创建
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("viewer"));
	//全局点云添加
	viewer->addPointCloud(cloud, "data");
	//将object点标为红色添加
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> red(cloud_filtered, 255, 0, 0);
	std::string cloudName="object";
	viewer->addPointCloud(cloud_filtered, red, cloudName);
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, cloudName);
	//保持窗体，知道显示窗体退出
	while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}


    return (0);
}




