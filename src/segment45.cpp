#include <iostream>
#include <string>
#include <vector>
#include <fstream>
#include "confi.h"
#include "CloudPoint3DSplice.h"


#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/common/transforms.h>
#include <pcl/kdtree/flann.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>


// #include <opencv.hpp>
using namespace std;



typedef pcl::PointCloud<pcl::PointXYZ>::iterator itertatorPCL;
 
void viewerOneOff(pcl::visualization::PCLVisualizer& viewer)
{
    viewer.setBackgroundColor(0,0,1);

}


int filterBeforeSegment(pcl::PointCloud<pcl::PointXYZ>::Ptr source,pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    if(0==source->points.size())
    {
        cout<<"source points cloud is zero!"<<endl;
        return -1;
    }
    for(int i=0;i<source->points.size();i++)
    {
        if
        (
            !(
                source->points[i].y<-1||
                source->points[i].x<-1||
                source->points[i].y>1||
                source->points[i].x>1||
                 source->points[i].z>2
            )
        )
        {
            cloud->points.push_back(source->points[i]);
        }
    }
    return 0;

}

int segment(string PCDfileLocation,string outputPCD_object_before,string outputPCD_ground)
{
    cout<<"segment........................"<<endl;
    
    // 建立点云对象（用来保存输入pcd文件）
    pcl::PointCloud<pcl::PointXYZ>::Ptr source(new pcl::PointCloud<pcl::PointXYZ>);
    // 建立点云对象（用来保存最后分割出的平面 or Object）
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    // 读入点云PCD文件 
    pcl::PCDReader reader;
    reader.read(PCDfileLocation,*source);
    

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    filterBeforeSegment(source,cloud);

    // -------------剔除地面-------------------
    // 创建一个模型参数对象，用于记录结果(平面方程)
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    // inliers表示误差能容忍的点 记录的是点云的序号
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
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

    // 提取地面and write to file
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud (cloud);
    extract.setIndices (inliers);
    extract.filter (*cloud_filtered);
    pcl::PointCloud<pcl::PointXYZ>::Ptr ground(cloud_filtered);
    pcl::PCDWriter writer;
    writer.write<pcl::PointXYZ> (outputPCD_ground, *ground, false);
    // 提取除地面外的物体and write to file
    extract.setNegative (true);
    extract.filter (*cloud_filtered); // 所以最终cloud_filtered保存的是剔除地面后剩余的点

    // 打印出平面方程
    // std::cout<<"the plane is ax+by+cz+d=0"<<std::endl;
	// std::cout << "a：" << coefficients->values[0] << endl;
	// std::cout << "b：" << coefficients->values[1] << endl;
	// std::cout << "c：" << coefficients->values[2] << endl;
    // std::cout << "d：" << coefficients->values[3] << endl;
	




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
    Eigen::Matrix3f rotation_1= Eigen::Matrix3f::Identity();
    rotation_1 (0,0) = x[0];
    rotation_1 (0,1) = x[1];
    rotation_1 (0,2) = x[2];
    rotation_1 (1,0) = y[0];
    rotation_1 (1,1) = y[1];
    rotation_1 (1,2) = y[2];
    rotation_1 (2,0) = z[0];
    rotation_1 (2,1) = z[1];
    rotation_1 (2,2) = z[2];
    // cout<<"rotation:"<<endl<<rotation_1<<endl;
    // 再构造出平移向量
    Eigen::Matrix<float,3,1> move_1;
    move_1[0] = 0;
    move_1[1] = 0;
    move_1[2] = -coefficients->values[3]/coefficients->values[2];
    // cout<<"move:"<<endl<<move_1<<endl;
    move_1=-rotation_1*move_1;
    // cout<<"move:"<<move_1<<endl;
    // 最后构造出转矩阵
    Eigen::Matrix4f transform_1 = Eigen::Matrix4f::Identity();
    // rotation 
    transform_1 (0,0) = x[0];
    transform_1 (1,0) = x[1];
    transform_1 (2,0) = x[2];
    transform_1 (0,1) = y[0];
    transform_1 (1,1) = y[1];
    transform_1 (2,1) = y[2];
    transform_1 (0,2) = z[0];
    transform_1 (1,2) = z[1];
    transform_1 (2,2) = z[2];
    // 平移
    transform_1 (0,3) = move_1[0];
    transform_1 (1,3) = move_1[1];
    transform_1 (2,3) = move_1[2];
    // cout<<"transform_1"<<endl<<transform_1<<endl;


    // //显示对象创建
	// boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("segment"));
	// //全局点云添加
	// viewer->addPointCloud(cloud, "data");
	// //将object点标为红色添加
	// pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> red(cloud_filtered, 255, 0, 0);
	// viewer->addPointCloud(cloud_filtered, red,"object");
	// viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "object");
    // viewer->addCoordinateSystem(0.1,"cloud",0);
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

	// //保持窗体，知道显示窗体退出
	// while (!viewer->wasStopped())
	// {
	// 	viewer->spinOnce(100);
	// 	boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	// }


    // 刚体变换(方法2)
    Eigen::Matrix<float,3,1> point_every;
    pcl::PointXYZ PCLPOINT;
    for(int i=0;i<cloud_filtered->points.size();i++)
    {
        point_every <<  cloud_filtered->points[i].x,
                        cloud_filtered->points[i].y,
                        cloud_filtered->points[i].z;
        point_every = rotation_1 * point_every + move_1;
        cloud_filtered->points[i].x = point_every[0];
        cloud_filtered->points[i].y = point_every[1];
        cloud_filtered->points[i].z = -point_every[2];
    }//转换object
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_T(new pcl::PointCloud<pcl::PointXYZ>);
    for(int i=0;i<cloud->points.size();i++)
    {
        point_every <<  cloud->points[i].x,
                        cloud->points[i].y,
                        cloud->points[i].z;
        point_every = rotation_1 * point_every + move_1;
        cloud->points[i].x = point_every[0];
        cloud->points[i].y = point_every[1];
        cloud->points[i].z = -point_every[2];
    }//转换原始点云

    // //显示对象创建
	// boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer2(new pcl::visualization::PCLVisualizer("transformation"));
	// //全局点云添加
	// viewer2->addPointCloud(cloud, "data");
	// //将object点标为红色添加
	// pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> red2(cloud_filtered, 255, 0, 0);
	// viewer2->addPointCloud(cloud_filtered, red2, "object");
	// viewer2->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "object");
    // viewer2->addCoordinateSystem(0.1,"cloud",0);
	// //保持窗体，知道显示窗体退出
	// while (!viewer2->wasStopped())
	// {
	// 	viewer2->spinOnce(100);
	// 	boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	// }


    writer.write<pcl::PointXYZ> (outputPCD_object_before, *cloud_filtered, false);
}



int filter(string outputPCD_object_before,string outputPCD_object_after,string resFile,int step,fstream &fout)
{
    cout<<"Filting-"<<endl;
    pcl::PointCloud<pcl::PointXYZ>::Ptr before(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr res(new pcl::PointCloud<pcl::PointXYZ>);
    // 读入
    pcl::PCDReader reader;
    reader.read(outputPCD_object_before,*before);
    // 滤波
    double size=before->points.size ();
    for(int i=0;i<size;i++)
    {
        if
        (
            !(
                before->points[i].z < 0||
                // before->points[i].z > 1||
                before->points[i].x < -0.3||
                before->points[i].x > 0.3||
                before->points[i].y > 0.3||
                before->points[i].y < -0.5
            )
        )
        {
            res->points.push_back(before->points[i]);
        }
    }
    
    
    // 统计滤波
    pcl::PointCloud<pcl::PointXYZ>::Ptr res_2(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;   //创建滤波器对象
    sor.setInputCloud (res);                           //设置待滤波的点云
    sor.setMeanK (50);                               //设置在进行统计时考虑的临近点个数
    sor.setStddevMulThresh (1.0);                      //设置判断是否为离群点的阀值，用来倍乘标准差，也就是上面的std_mul
    sor.filter (*res_2);                    //滤波结果存储到cloud_filtered



    // // 高度
    // double aver;
    // for(int i=0;i<size;i+=100)
    // {
    //     if(res->points[i].z>0.03)
    //     aver+=res->points[i].z;
    // }
    // aver=aver/(size/100);
    // fout<<to_string(step)<<":Hight:"<<aver<<endl;
    // cout<<to_string(step)<<":Hight:"<<aver<<endl;
    

    



    // 显示对象创建
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer3(new pcl::visualization::PCLVisualizer("Filter"));
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> green3(res_2, 0, 255, 0); 
    viewer3->addPointCloud(res_2,green3,"filter");
	viewer3->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "object");
    viewer3->addCoordinateSystem(0.1,"cloud",0);
	//保持窗体，知道显示窗体退出
	while (!viewer3->wasStopped())
	{
		viewer3->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}



    // 体积测量
    std::vector<cv::Point3f> points;
    cv::Point3f box_vertex[8];
    cv::Point3f salient_points[6];
	float length = 1; 
    float width = 0;
    float height = 0;
    float sp = 0.05;
    cv::Point3f point;
    size=res_2->points.size();
    for(int i=0;i<size;i++)
    {
        point.x=res_2->points[i].x;
        point.y=res_2->points[i].y;
        point.z=-res_2->points[i].z; //  getMinBoundingBoxByRC中的坐标系Z周向下的。
        points.push_back(point);
    }

    getMinBoundingBoxByRC(points,box_vertex,salient_points,&length,&width,&height,sp);
    // salient_points[0]～salient_points[4]是前后左右四个面的突点
    // salient_points[4]是上表面（z轴朝下时）最靠下(最靠箱子内部）的前1%的点的平均值。
    // salient_points[0]是下表面（z轴朝下时) 最靠下（最靠箱子外面）的前1%的点的平均值。
    cout<<":Hight(height):"<<height<<endl;
    cout<<":Hight(salient_points[0].z):"<<-salient_points[0].z<<endl;
    cout<<":Hight(salient_points[1].z):"<<-salient_points[1].z<<endl;
    cout<<":Hight(salient_points[2].z):"<<-salient_points[2].z<<endl;
    cout<<":Hight(salient_points[3].z):"<<-salient_points[3].z<<endl;
    cout<<":Hight(salient_points[4].z):"<<-salient_points[4].z<<endl;
    cout<<":Hight(salient_points[5].z):"<<-salient_points[5].z<<endl;
    fout<<to_string(step)<<":length:"<<length<<endl;
    fout<<to_string(step)<<":width:"<<width<<endl;
    fout<<to_string(step)<<":Hight:"<<-salient_points[4].z<<endl;






    // 写入pcd
    if(res_2->points.size()==0)
    {
        cout<<"res points is 0"<<endl;
        return -1;
    }
    pcl::PCDWriter writer;
    res_2->width =1;
    res_2->height = res_2->points.size();
    writer.write<pcl::PointXYZ> (outputPCD_object_after, *res_2, false);

    return 0;
}



int main (int argc, char** argv)
{
    vector<int> steps{15};
    // vector<int> steps{0};
    fstream fout(Confi::projectPath+"/data/res.txt", std::ios_base::binary | std::ios_base::trunc | std::ios_base::out);

    for(auto step:steps)
    {
        cout<<"*****************************************************************************"<<endl;
        cout<<"step:"<<step<<endl;
        string PCDfileLocation=Confi::projectPath+"./data/pcd/"+"step_"+to_string(step)+".pcd";
        string objectFile_before = Confi::projectPath+"./data/pcd/object_before/"+"step_"+to_string(step)+".pcd";
        string objectFile_after= Confi::projectPath+"./data/pcd/object_after/"+"step_"+to_string(step)+".pcd";
        string groundFile = Confi::projectPath+"./data/pcd/ground/"+"step_"+to_string(step)+".pcd";
        string resFile=Confi::projectPath+"./data/"+"res.txt";
        segment(PCDfileLocation,objectFile_before,groundFile);
        filter(objectFile_before,objectFile_after,resFile,step,fout);
    }
    
    fout.close();
    return (0);
}




