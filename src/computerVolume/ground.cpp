# include "ground.h"
# include <cmath>
using namespace std;


// UnitNormalVector
void Ground::_compute_UnitNormalVector()
{
    double x=_Coefficients->values[0];
    double y=_Coefficients->values[1];
    double z=_Coefficients->values[2];
    double length=pow((x*x+y*y+z*z),1.0/2);
    _UnitNormalVector=Eigen::Vector3d(x/length,y/length,z/length);
}


// center of the plane point cloud
void Ground::_compute_GroundCenter()
{
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(_GroundPointXYZ, centroid);
    _GroundCenter<<centroid[0],centroid[1],centroid[2];
}

// 地面坐标系单位基向量，顺序是x,y,z;
void Ground::_compute_GroundCoordinateXYZ()
{
    Eigen::Vector3f x;
    Eigen::Vector3f y;
    Eigen::Vector3f z;

    //computer z;
    z<<_UnitNormalVector[0],_UnitNormalVector[1],_UnitNormalVector[2];

    //computer x;
    x<<1,1,(-(z[0]+z[1])/z[2] );
    double x_length=pow((x[0]*x[0]+x[1]*x[1]+x[2]*x[2]),1.0/2);
    x<<x[0]/x_length,x[1]/x_length,x[2]/x_length;

    //computer y;
    y= z.cross(x);
    double y_length=pow((y[0]*y[0]+y[1]*y[1]+y[2]*y[2]),1.0/2);
    y<<y[0]/y_length,y[1]/y_length,y[2]/y_length;

    // 这里还差一个输入到最终结果中
}
