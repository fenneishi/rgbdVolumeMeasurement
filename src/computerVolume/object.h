# pragma once

#include <iostream>
#include <string>
#include "confi.h"
#include "ground.h"
// includes pcl 
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/extract_indices.h>
// eigen
#include <Eigen/Core>
#include <Eigen/Geometry>


using namespace std;



class Object
{
public:
    // Filter
    // 过滤掉z≈0，z<0,z>0但是值过大的。
    void Filter_z();
    // 过滤掉距离地面中心过远的点。
    void Filter_xy_ground(Ground _ground);
    // 过滤掉距离距离object顶面中心过远的点。
    void Filter_xy();

    //  volume 
    double compute_volume();

    
    Object(Object &&) = delete;
    Object &operator=(Object &&) = delete;
    Object &operator=(const Object &) = delete;
    ~Object() = default;
    // construct
    Object() = default;
    Object(const pcl::PointCloud<pcl::PointXYZ> &ObjectPointXYZ):
         _ObjectPointXYZ(ObjectPointXYZ),_PrioriSizeXYZ(-1,-1,-1)
    {
        _compute_ObjectCenter();
        _compute_TopSurfacePointXYZ();
        _compute_TopSurfaceCenter();
    };
    Object(const pcl::PointCloud<pcl::PointXYZ> &ObjectPointXYZ,const Eigen::Vector3d &PrioriSizeXYZ):
         _ObjectPointXYZ(ObjectPointXYZ),_PrioriSizeXYZ(PrioriSizeXYZ)
    {
        _compute_ObjectCenter();
        _compute_TopSurfacePointXYZ();
        _compute_TopSurfaceCenter();
    };
    // copy construct 
    Object(const Object &object)
    {
        _PrioriSizeXYZ=object.get_PrioriSizeXYZ();
        _ObjectCenter =object.get_ObjectCenter();
        _TopSurfacePointXYZ=object.get_TopSurfacePointXYZ();
        _TopSurfaceCenter=object.get_TopSurfaceCenter();
        _volume=object.get_volume();
    }
    // copy operator
    Object &operator=(const Object &object) 
    {
        _PrioriSizeXYZ=object.get_PrioriSizeXYZ();
        _ObjectCenter =object.get_ObjectCenter();
        _TopSurfacePointXYZ=object.get_TopSurfacePointXYZ();
        _TopSurfaceCenter=object.get_TopSurfaceCenter();
        _volume=object.get_volume();
        return  *this;
    }
    
    


    // get value
    // _ObjectPointXYZ
    pcl::PointCloud<pcl::PointXYZ> & get_ObjectPointXYZ(){return _ObjectPointXYZ};
    // _PrioriSizeXYZ
    Eigen::Vector3d get_PrioriSizeXYZ(){return _PrioriSizeXYZ};
    // _TopSurfacePointXYZ
    pcl::PointCloud<pcl::PointXYZ> get_TopSurfacePointXYZ(){return  _TopSurfacePointXYZ};
    // _TopSurfaceCenter
    Eigen::Vector3d get_TopSurfaceCenter(){return _TopSurfaceCenter};
    // colume
    double get_volume(){return _volume;};



    //change value
    void change_ObjectPointXYZ(const pcl::PointCloud<pcl::PointXYZ> &New){_ObjectPointXYZ=New;};
    void change_PrioriSizeXY(const Eigen::Vector3d &New){_PrioriSizeXYZ=New;};
    void change_TopSurfacePointXYZ(const pcl::PointCloud<pcl::PointXYZ> &New){_TopSurfacePointXYZ=New;};
    void change_TopSurfaceCenter(const  Eigen::Vector3d &New){_TopSurfaceCenter=New;};
    void change_volume(double new){_volume=New;};


    
private:

    // attribute
    // 点云
    pcl::PointCloud<pcl::PointXYZ> _ObjectPointXYZ();
    // 先验：先验尺寸X=lenth(x轴方向) Y=width(y轴方向) Z=high(Z轴方向)
    Eigen::Vector3d _PrioriSizeXYZ(0,0,0);
    // 顶面
    pcl::PointCloud<pcl::PointXYZ> _TopSurfacePointXYZ();
    Eigen::Vector3d _TopSurfaceCenter(0,0,0);
    // 体积
    double _volume;

    
    // computer attribute
    void _compute_TopSurfacePointXYZ();
    void _compute_TopSurfaceCenter();
    

    // status 
    // 若没有进行滤波，则状态为flase;
    // 若进行了滤波，则状态为true;
    bool if_filter=false;



    
};