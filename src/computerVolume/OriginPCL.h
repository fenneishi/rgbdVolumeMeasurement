# pragma once

#include <iostream>
#include <string>
#include "confi.h"
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

class OriginPCL
{
public:
    //0.construct
    OriginPCL(const string fileLocation);
    //1.ground segment
    void segment_ground();
    //2.coordinate transformation
    void coordinateTransformation();
    //3.filter;
    void ObjectFilter();
    //4.computer volume
    double ComputerVolume(){return _Object.compute_volume();};
    //5.save;
    void saveToFile () const ;
    // 6.show
    void show () const ;



    OriginPCL() = default;
    OriginPCL(OriginPCL &&) = delete;
    OriginPCL(const OriginPCL &) = delete;
    OriginPCL &operator=(OriginPCL &&) = delete;
    OriginPCL &operator=(const OriginPCL &) = delete;
    ~OriginPCL() = default;

    
private:
    pcl::PointCloud<pcl::PointXYZ> OriginCloud;
    Ground _ground;
    Object _Object;
};