

# include "object.h"

typedef pcl::PointCloud<pcl::PointXYZ>::iterator itertatorPCL;

void Filter_z()
{

    for(itertatorPCL index=_ObjectPointXYZ->begin();index!=_ObjectPointXYZ->end();index++)
    {
        if(_ObjectPointXYZ[index].z<0||_ObjectPointXYZ[index].z>10)
            _ObjectPointXYZ->erase(index);

    }
}