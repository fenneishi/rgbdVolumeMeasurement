#pragma once


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
// other
#include <iostream>
#include <string>
#include "confi.h"
#include "ground.h"
#include "object.h"

using namespace std;


class Ground
{
public:
    // get value
    pcl::PointCloud<pcl::PointXYZ> get_GroundPointXYZ() const {return  _GroundPointXYZ};
    pcl::ModelCoefficients get_Coefficients() const {return _Coefficients};
    Eigen::Vector3d get_UnitNormalVector() const {return _UnitNormalVector};
    Eigen::Vector3d get_GroundCenter() const {return _GroundCenter};
    Eigen::Matrix3d get_GroundCoordinateXYZ() const {return _GroundCoordinateXYZ};

    // change value
    void change_GroundPointXYZ(const pcl::PointCloud<pcl::PointXYZ> &New){_GroundPointXYZ=New;};
    void change_Coefficients(const pcl::ModelCoefficients &New){_Coefficients=New;};
    void change_UnitNormalVector(const Eigen::Vector3d &New){_UnitNormalVector=New;};
    void change_GroundCenter(const Eigen::Vector3d &New){_GroundCenter=New;};
    void change_roundCoordinateXYZ(const Eigen::Matrix3d &New){_GroundCoordinateXYZ=New;};






    Ground() = default;
    Ground(Ground &&) = delete;
    Ground &operator=(Ground &&) = delete;
    Ground &operator=(const Ground &) = delete;
    ~Ground() =default;
    // construct
    Ground(const pcl::PointCloud<pcl::PointXYZ> &GroundPointXYZ, const pcl::ModelCoefficients &Coefficients):
        _GroundPointXYZ(GroundPointXYZ),_Coefficients(Coefficients)
    {
        _compute_UnitNormalVector();
        _compute_GroundCenter();
        _compute_GroundCoordinateXYZ()
    };
    // copy construct
    Ground(const Ground &ground)
    {
        _GroundPointXYZ=ground.get_GroundPointXYZ();
        _Coefficients=ground.get_Coefficients();
        _UnitNormalVector=ground.get_UnitNormalVector();
        _GroundCenter=ground.get_GroundCenter();
        _GroundCoordinateXYZ=ground.get_GroundCoordinateXYZ();
    }
    // copy operator
    Ground &operator=(const Ground &ground) 
    {
        _GroundPointXYZ=ground.get_GroundPointXYZ();
        _Coefficients=ground.get_Coefficients();
        _UnitNormalVector=ground.get_UnitNormalVector();
        _GroundCenter=ground.get_GroundCenter();
        _GroundCoordinateXYZ=ground.get_GroundCoordinateXYZ();
        return *this;
    }


    

private:
    // attribute
    // 地面点云
    pcl::PointCloud<pcl::PointXYZ> _GroundPointXYZ;
    // 地面方程
    pcl::ModelCoefficients _Coefficients;
    // 地面法向量
    Eigen::Vector3d _UnitNormalVector(0,0,1);
    // 地面点云中心(x,y,z)
    Eigen::Vector3d _GroundCenter(0,0,0);
    // 以点云法向量为Z轴正方向的坐标系单位基向量(在原坐标系中的坐标)
    Eigen::Matrix3d _GroundCoordinateXYZ(1,0,0,0,1,0,0,0,1);
    
    // computer attribute
    void _compute_UnitNormalVector();
    void _compute_GroundCenter();
    void _compute_GroundCoordinateXYZ();s
    

};