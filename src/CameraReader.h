#ifndef __CAMERAREADER_H__
#define __CAMERAREADER_H__
#include <fstream>
#include <opencv.hpp>


class CameraReader
{
public:
	CameraReader();
	~CameraReader();
	bool open(const char* name);
	void close();
protected:
	std::fstream m_stream;
};

typedef struct _CamPara_
{
	float depth_inner[4];
	float color_inner[4];
	float c2d_r[9];
	float c2d_t[3];
}CamPara,*CamParaPtr;

class CamParaReader : public CameraReader
{
public:
	CamParaReader();
	~CamParaReader();
	bool next(CamPara& cam_para);
};

class RgbdReader : public CameraReader
{
public:
	RgbdReader();
	~RgbdReader();
	bool next(cv::Mat& img, const cv::Size& size, int type);
};

#endif


