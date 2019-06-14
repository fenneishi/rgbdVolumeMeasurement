#ifndef __CLOUDPOINT3DSPLICE_H__
#define __CLOUDPOINT3DSPLICE_H__

#ifdef _WIN32
#ifdef FOURIERMELLINREGISTRATION_EXPORTS
#define FOURIERMELLINREGISTRATION_API __declspec(dllexport)
#else
#define FOURIERMELLINREGISTRATION_API __declspec(dllimport)
#endif
#else
#define FOURIERMELLINREGISTRATION_API 
#endif

#ifndef EXTERN_C
	#ifdef __cplusplus
		#define EXTERN_C extern "C"
	#else
		#define EXTERN_C 
	#endif
#endif

#include <opencv.hpp>

//旋转卡壳方式计算最小包围盒
//box_vertex:0--3表示已最低点Z坐标为基准的xoy平面最小包围盒的4个角点。4--5表示最低点和最高点
EXTERN_C FOURIERMELLINREGISTRATION_API int getMinBoundingBoxByRC(const std::vector<cv::Point3f>& points, cv::Point3f box_vertex[8], cv::Point3f salient_points[6]
	,float* length = NULL, float* width = NULL, float* height = NULL,float sp = 0.0f);


//根据相机姿态计算相机正射变换矩阵：标高为当前相机高度
EXTERN_C FOURIERMELLINREGISTRATION_API int getTransByPose(const cv::Vec4f& camera_pose, cv::Mat& mat);

#endif