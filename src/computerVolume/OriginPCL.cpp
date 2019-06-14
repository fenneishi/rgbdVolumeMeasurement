# include "OriginPCL.h"
const double DistanceThreshold=Confi::DistanceThreshold;



//0.construct
OriginPCL::OriginPCL(const string fileLocation);
{
    // 读入点云PCD文件 
    pcl::PCDReader reader;
    reader.read(fileLocation,OriginCloud);
}



//1.ground segment
void OriginPCL::segment_ground()
{
    // 平面方程参数
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    // 点云序号:inliers表示误差能容忍的点(后面用来记录记录分割署来平面（地面）点的序号）
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    // 分割器
    pcl::SACSegmentation<pcl::PointXYZ> seg;


    // 分割器设置
    // Optional-这个设置可以选定结果平面展示的点是分割掉的点还是分割剩下的点。(true是展示剩下的点)
    seg.setOptimizeCoefficients (true);
    // Mandatory-设置目标几何形状
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    // 距离阈值 单位m
    seg.setDistanceThreshold (DistanceThreshold);
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
 

    // 提取地面
    pcl::PointCloud<pcl::PointXYZ>::Ptr ground(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud (cloud);
    extract.setIndices (inliers);
    extract.filter (*ground);
    _ground=Ground(*ground,coefficients);



    // 提取物体(除了地面外剩余的)
    pcl::PointCloud<pcl::PointXYZ>::Ptr object(new pcl::PointCloud<pcl::PointXYZ>);
    extract.setNegative (true);
    extract.filter (*object);
    _object=Object(*object);
    
    return (0);
}



//2.coordinate transformation
void OriginPCL::coordinateTransformation()
{
    // 构造转换矩阵
    Eigen::Matrix4f transform_1 = Eigen::Matrix4f::Identity();
    // rotation (关键是横纵坐标搞清楚，你怎么保存到，接口要的有什么形式)
    transform_1 (0,0) = _ground.get_GroundCoordinateXYZ()(0,0);
    transform_1 (0,1) = _ground.get_GroundCoordinateXYZ()(0,1);
    transform_1 (0,2) = _ground.get_GroundCoordinateXYZ()(0,2);
    transform_1 (1,0) = _ground.get_GroundCoordinateXYZ()(1,0);
    transform_1 (1,1) = _ground.get_GroundCoordinateXYZ()(1,1);
    transform_1 (1,2) = _ground.get_GroundCoordinateXYZ()(1,2);
    transform_1 (2,0) = _ground.get_GroundCoordinateXYZ()(2,0);
    transform_1 (2,1) = _ground.get_GroundCoordinateXYZ()(2,1);
    transform_1 (2,2) = _ground.get_GroundCoordinateXYZ()(2,2);
    // 平移
    transform_1 (0,3) = _ground.get_GroundCenter()[0];
    transform_1 (1,3) = _ground.get_GroundCenter()[1];
    transform_1 (2,3) = _ground.get_GroundCenter()[2];
    // 进行转换
    pcl::PointCloud<pcl::PointXYZ> OldOriginCloud(OriginCloud);
    pcl::transformPointCloud(OldOriginCloud,OriginCloud,transform_1); 
    // 重新分割(进行整体更新)
    segment_ground();
}



//3.filter;
void OriginPCL::ObjectFilter()
{
    // z轴方向进行滤波
    _Object.Filter_z();
    // 根据地面中心进行滤波(结合先验信息)
    _Object.Filter_xy_ground(_ground);
    // 根据object顶部平面进行滤波(结合先验信息)
    _Object.Filter_xy();
}



//4.computer volume
double OriginPCL::ComputerVolume()
{
    return _Object.compute_volume();
}



//5.save;
void OriginPCL::saveToFile () const 
{

}



//6.show
void show () const 
{

}


