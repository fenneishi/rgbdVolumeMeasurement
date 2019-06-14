#include <iostream>
#include <string>
#include <vector>
#include <fstream>
#include <algorithm>
#include <set>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/common/transforms.h>
#include <opencv.hpp>
// #include <pcl/filters/statistical_outlier_removal.h>
#include "confi.h"
#include "CloudPoint3DSplice.h"

using namespace std;
typedef pcl::PointCloud<pcl::PointXYZ>::iterator itertatorPCL;
 
void viewerOneOff(pcl::visualization::PCLVisualizer& viewer)
{
    viewer.setBackgroundColor(0,0,1);
}

int _segment(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,pcl::PointIndices::Ptr inliers,pcl::ModelCoefficients::Ptr coefficients )
{

    // Create the segmentation object(创建一个分割器)
    pcl::SACSegmentation<pcl::PointXYZ> seg;
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
    //分割点云，同时获得平面和 (a,b,c)
    seg.segment (*inliers, *coefficients);
    if (inliers->indices.size () == 0)
    {
        PCL_ERROR ("Could not estimate a planar model for the given dataset.");
        return (-1);
    }
    return 0;
}


int _computerTransform(pcl::ModelCoefficients::Ptr coefficients,Eigen::Matrix3f R_plane,Eigen::Matrix<float,3,1> M_plane,Eigen::Matrix4f T_plane)
{
     // ------------------坐标系转换----------------------
    // 地面单位法向量
    Eigen::Vector3d n(0,0,1);
    double X=coefficients->values[0];
    double Y=coefficients->values[1];
    double Z=coefficients->values[2];
    double length=pow((X*X+Y*Y+Z*Z),1.0/2);
    n=Eigen::Vector3d(X/length,Y/length,Z/length);
    // 地面坐标系单位基向量
    Eigen::Vector3f x;
    Eigen::Vector3f y;
    Eigen::Vector3f z;
    //computer z;
    z<<n[0],n[1],n[2];
    //computer y;
    y<<1,1,(-(z[0]+z[1])/z[2] );
    double y_length=pow((y[0]*y[0]+y[1]*y[1]+y[2]*y[2]),1.0/2);
    y<<y[0]/y_length,y[1]/y_length,y[2]/y_length;
    //computer x;
    x= y.cross(z);
    double x_length=pow((x[0]*x[0]+x[1]*x[1]+x[2]*x[2]),1.0/2);
    x<<x[0]/x_length,x[1]/x_length,x[2]/x_length;
    // 构造出旋转矩阵
    // Eigen::Matrix3f R_plane= Eigen::Matrix3f::Identity();
    R_plane (0,0) = x[0];
    R_plane (0,1) = x[1];
    R_plane (0,2) = x[2];
    R_plane (1,0) = y[0];
    R_plane (1,1) = y[1];
    R_plane (1,2) = y[2];
    R_plane (2,0) = z[0];
    R_plane (2,1) = z[1];
    R_plane (2,2) = z[2];
    // cout<<"rotation:"<<endl<<R_plane<<endl;
    // 再构造出平移向量
    // Eigen::Matrix<float,3,1> M_plane;
    M_plane[0] = 0;
    M_plane[1] = 0;
    M_plane[2] = -coefficients->values[3]/coefficients->values[2];
    M_plane=-R_plane*M_plane;
    // 最后构造出转换矩阵
    // Eigen::Matrix4f T_plane = Eigen::Matrix4f::Identity();
    // rotation分量
    T_plane (0,0) = x[0];
    T_plane (1,0) = x[1];
    T_plane (2,0) = x[2];
    T_plane (0,1) = y[0];
    T_plane (1,1) = y[1];
    T_plane (2,1) = y[2];
    T_plane (0,2) = z[0];
    T_plane (1,2) = z[1];
    T_plane (2,2) = z[2];
    // move分量
    T_plane (0,3) = M_plane[0];
    T_plane (1,3) = M_plane[1];
    T_plane (2,3) = M_plane[2];
    return 0;
}

int _Transform(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,Eigen::Matrix3f R_plane,Eigen::Matrix<float,3,1> M_plane,Eigen::Matrix4f T_plane)
{
    Eigen::Matrix<float,3,1> point_every;
    for(int i=0;i<cloud->points.size();i++)
    {
        point_every <<  cloud->points[i].x,
                        cloud->points[i].y,
                        cloud->points[i].z;
        point_every = R_plane * point_every + M_plane;
        cloud->points[i].x = point_every[0];
        cloud->points[i].y = point_every[1];
        cloud->points[i].z = -point_every[2];
    }
}

// 坐标变换前的点云展示
int _show_1(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,pcl::PointCloud<pcl::PointXYZ>::Ptr object,pcl::ModelCoefficients::Ptr coefficients)
{
    //显示对象创建
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("segment"));
	//全局点云添加
	viewer->addPointCloud(cloud, "data");
	//将object点标为红色添加
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> red(object, 255, 0, 0);
	viewer->addPointCloud(object, red,"object");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "object");
    viewer->addCoordinateSystem(0.1,"cloud",0);
    // // 显示平面法向量
    // pcl::PointXYZ origin,end_x,end_y,end_z;
    // origin.x=0;origin.y=0;origin.z=0;
    // end_z.x=coefficients->values[0];
    // end_z.y=coefficients->values[1];
    // end_z.z=coefficients->values[2];
    // end_x.x=x[0];
    // end_x.y=x[1];
    // end_x.z=x[2];
    // end_y.x=y[0];
    // end_y.y=y[1];
    // end_y.z=y[2];
    // viewer->addLine<pcl::PointXYZ> (origin,end_x,255,0,0,"x");
    // viewer->addLine<pcl::PointXYZ> (origin,end_y,0,255,0,"y");
    // viewer->addLine<pcl::PointXYZ> (origin,end_z,0,0,255,"z");
	//保持窗体，知道显示窗体退出
	while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}
    return 0;
}

// 坐标变换后，滤波前点云展示
int _show_2(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,pcl::PointCloud<pcl::PointXYZ>::Ptr object)
{
    //显示对象创建
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer2(new pcl::visualization::PCLVisualizer("transformation"));
	//全局点云添加
	viewer2->addPointCloud(cloud, "data");
	//将object点标为红色添加
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> red2(object, 255, 0, 0);
	viewer2->addPointCloud(object, red2, "object");
	viewer2->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "object");
    viewer2->addCoordinateSystem(0.1,"cloud",0);
	//保持窗体，知道显示窗体退出
	while (!viewer2->wasStopped())
	{
		viewer2->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}
    return 0;
}


int segment(string PCDfileLocation,string outputPCD_object_before,string outputPCD_ground)
{
    cout<<"----------------segment---------------------"<<endl;

    // -----------------------工具---------------------------
    pcl::PCDWriter writer; 
   
    
    // -----------------------建立点云------------------------------
    // 输入点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    // object点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr object(new pcl::PointCloud<pcl::PointXYZ>);
    // 地面点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr ground(new pcl::PointCloud<pcl::PointXYZ>);
    // 读入点云PCD文件 
    pcl::PCDReader reader;
    reader.read(PCDfileLocation,*cloud);


    // --------—----------------剔除地面--------------------------
    // 平面方程
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    // 内点序号
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    // 找到平面
    _segment(cloud,inliers,coefficients);
    // 提取器
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    // 平面点提取到ground
    extract.setInputCloud (cloud);
    extract.setIndices (inliers);
    extract.filter (*ground);
    // 地面之外所有点提取到object
    extract.setNegative (true);
    extract.filter (*object); 
    
    // -------------------------计算转换矩阵--------------------------
    Eigen::Matrix3f R_plane;
    Eigen::Matrix<float,3,1> M_plane;
    Eigen::Matrix4f T_plane;
    _computerTransform( coefficients,R_plane,M_plane,T_plane);


    // --------------------------first show------------------------------
    // _show_1(cloud,object,coefficients);
    

    // --------------------------坐标系转换------------------------
    _Transform(object,R_plane,M_plane,T_plane);
    _Transform(ground,R_plane,M_plane,T_plane);
    _Transform(cloud,R_plane,M_plane,T_plane);

    // --------------------------second show-------------------------
    _show_2(cloud,object);

    //-------------------------写入文件----------------------------
    writer.write<pcl::PointXYZ> (outputPCD_ground,*ground,false);
    writer.write<pcl::PointXYZ> (outputPCD_object_before,*object,false);
}























struct compare_1
{
    bool operator()(const pcl::PointXYZ &P1,const pcl::PointXYZ &P2)
    {
        return (P1.z<P2.z||P1.z==P2.z);
    }
};

int _show_3(pcl::PointCloud<pcl::PointXYZ>::Ptr after)
{
     // 显示对象创建
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer3(new pcl::visualization::PCLVisualizer("Filter"));
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> green3(after, 0, 255, 0); 
    viewer3->addPointCloud(after,green3,"filter");
	viewer3->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "object");
    viewer3->addCoordinateSystem(0.1,"cloud",0);
	//保持窗体，知道显示窗体退出
	while (!viewer3->wasStopped())
	{
		viewer3->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}
}

int filter(string outputPCD_object_before,string outputPCD_object_after)
{
    cout<<"----------------Filter---------------------"<<endl;


    pcl::PointCloud<pcl::PointXYZ>::Ptr before(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr after(new pcl::PointCloud<pcl::PointXYZ>);
    double size; // 点云大小
    pcl::PCDWriter writer;
   
    // ---------------------------------------读入点云-------------------------------------------
    pcl::PCDReader reader;
    reader.read(outputPCD_object_before,*before);
                                                                                        cout<<"(before filter)the points cloud size is :"<<(size=before->points.size())<<endl;
    
    
    
    // ----------------------------------------滤波---------------------------------------------
    for(int i=0;i<size;i++)
    {
        if
        (
            before->points[i].z < 0||
            before->points[i].z > 1||
            before->points[i].z < 0.1||
            before->points[i].x < -1||
            before->points[i].y > 0||
            before->points[i].y < -2||
            before->points[i].x > 0.8
        )
        {
            continue;
        }
        else
        {
            after->points.push_back(before->points[i]);
        }  
    }        
                                                                                            cout<<"(after filter)the points cloud size is :"<<(size=after->points.size())<<endl; // size 
    
    
    
    // ---------------------------------------写入文件----------------------------------------------
    if(after->points.size()==0)
    {
        cout<<"after points is 0"<<endl;
        return -1;
    }
    after->width =1;
    after->height = after->points.size();
    writer.write<pcl::PointXYZ> (outputPCD_object_after, *after, false);



    // -------------------------------------third show------------------------------------------
    _show_3(after);


    return 0;
}


int measurement(string objectFile_after,int step,fstream &fout)
{
    cout<<"-------------------------measurement-------------------------"<<endl;
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr after(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PCDReader reader;
    reader.read(objectFile_after,*after);
    double size;

    // ------------------------------------------点云排序(按照Z从小到大)------------------------------------------
    size=after->points.size();
    set<pcl::PointXYZ,compare_1> after_order;
    for(int i=0;i<size;i++)
    {
        after_order.insert(after->points[i]);
    }
    after->clear();
    fstream Pout1("/home/qilong/Desktop/project/rgbdVolumeMeasurement/data/pointsSort1.txt",std::ios_base::binary | std::ios_base::trunc | std::ios_base::out);
    for(auto &p:after_order) 
    {
        Pout1<<p<<endl;          
        after->points.push_back(p);
    }                                                                                         cout<<"(after ths sort)the size is :"<<(size=after->size())<<endl;
    Pout1.close();
    fstream Pout2("/home/qilong/Desktop/project/rgbdVolumeMeasurement/data/pointsSort2.txt",std::ios_base::binary | std::ios_base::trunc | std::ios_base::out);
    for(int i=0;i<size;i++)
    {
        Pout2<<after->points[i]<<endl;
    }
    Pout2.close();




    // ------------------------------------------计算高度----------------------------------------------------------
    double aver;
    for(int i=size;i>(size*0.999);i--)
    {
        aver+=after->points[i].z;
    }
    aver=aver/size;
    fout<<to_string(step)<<":Hight:"<<aver<<endl;
    cout<<to_string(step)<<":Hight:"<<aver<<endl;
    
   


    // -------------------------------------------计算体积-------------------------------------------------------
    // std::vector<cv::Point3f> points;
    // cv::Point3f box_vertex[8];
    // cv::Point3f salient_points[6];
	// float length = 1; 
    // float width = 0;
    // float height = 0;
    // float sp = 0;
    // cv::Point3f point;
    // cout<<"----------------------------------------------------------------------------------"<<endl;
    // for(int i=0;i<100;i++)
    // {
    //     cout<<after->points[i].x<<","<<after->points[i].y<<","<<after->points[i].z<<endl;
    // }
    // for(int i=0;i<size;i++)
    // {
    //     point.x=after->points[i].x*1000;
    //     point.y=after->points[i].y*1000;
    //     point.z=after->points[i].z*1000;
    //     points.push_back(point);
    // }
    // getMinBoundingBoxByRC(points,box_vertex,salient_points,&length,&width,&height,sp);
    // cout<<length<<endl;
    // cout<<width<<endl;
    // cout<<height<<endl;
    // cout<<"the valume is:"<<length*width*height<<endl;

}

















int main (int argc, char** argv)
{
    // ,15,30,45,60,90,180,360
    vector<int> steps{0};
    string resFile=Confi::projectPath+"./data/"+"res.txt";
    fstream fout(resFile,std::ios_base::binary | std::ios_base::trunc | std::ios_base::out);

    for(auto step:steps)
    {
        cout<<"step:"<<step<<endl;
        string PCDfileLocation=Confi::projectPath+"./data/pcd/"+"step_"+to_string(step)+".pcd";
        string objectFile_before = Confi::projectPath+"./data/pcd/object_before/"+"step_"+to_string(step)+".pcd";
        string objectFile_after= Confi::projectPath+"./data/pcd/object_after/"+"step_"+to_string(step)+".pcd";
        string groundFile = Confi::projectPath+"./data/pcd/ground/"+"step_"+to_string(step)+".pcd";
        segment(PCDfileLocation,objectFile_before,groundFile);
        filter(objectFile_before,objectFile_after);
        measurement(objectFile_after,step,fout);
    }
    
    fout.close();
    return (0);
}






 // // Create the segmentation object(创建一个分割器)
    // pcl::SACSegmentation<pcl::PointXYZ> seg;
    // // Optional-这个设置可以选定结果平面展示的点是分割掉的点还是分割剩下的点。(true是展示剩下的点)
    // seg.setOptimizeCoefficients (true);
    // // Mandatory-设置目标几何形状
    // seg.setModelType (pcl::SACMODEL_PLANE);
    // seg.setMethodType (pcl::SAC_RANSAC);
    // // 距离阈值 单位m
    // seg.setDistanceThafterhold (0.005);
    // // 输入点云
    // seg.setInputCloud (cloud);
    // //分割
    // //分割点云，同时获得平面和 (a,b,c)
    // seg.segment (*inliers, *coefficients);
    // if (inliers->indices.size () == 0)
    // {
    //     PCL_ERROR ("Could not estimate a planar model for the given dataset.");
    //     return (-1);
    // }



// 打印出平面方程
    // std::cout<<"the plane is ax+by+cz+d=0"<<std::endl;
	// std::cout << "a：" << coefficients->values[0] << endl;
	// std::cout << "b：" << coefficients->values[1] << endl;
	// std::cout << "c：" << coefficients->values[2] << endl;
    // std::cout << "d：" << coefficients->values[3] << endl;
	