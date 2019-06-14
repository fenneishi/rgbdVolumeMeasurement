#include "CameraReader.h"



CameraReader::CameraReader()
{
}


CameraReader::~CameraReader()
{
}

bool CameraReader::open(const char* name)
{
	if (m_stream.is_open())
		m_stream.close();
	if (nullptr == name)
		return false;
	m_stream.open(name, std::ios_base::binary | std::ios_base::in);
	return m_stream.is_open();
}

void CameraReader::close()
{
	if (m_stream.is_open())
		m_stream.close();
}


CamParaReader::CamParaReader():
	CameraReader()
{
}


CamParaReader::~CamParaReader()
{
}

bool CamParaReader::next(CamPara& cam_para)
{
	if (!m_stream.is_open())
		return false;
	if (m_stream.eof())
		return false;
	int get_size = sizeof(cam_para);
	m_stream.read((char*)&cam_para.depth_inner[0], get_size);
	if (m_stream.gcount() < get_size)
		return false;
	return true;
}

RgbdReader::RgbdReader():
	CameraReader()
{
}

RgbdReader::~RgbdReader()
{
}

bool RgbdReader::next(cv::Mat& img, const cv::Size& size, int type)
{
	if (!m_stream.is_open())
		return false;
	if (size.width <= 0 || size.height <= 0)
		return false;
	int get_size = 0;
	switch (type)
	{
	case CV_8UC1:
		get_size = size.width * size.height;
		break;
	case CV_8UC3:
		get_size = size.width * size.height * 3;
		break;
	case CV_16UC1:
		get_size = size.width * size.height * 2;
		break;
	default:
		break;
	}
	if (get_size <= 0)
		return false;
	if (m_stream.eof())
		return false;
	cv::Mat data = cv::Mat::zeros(size, type);
	m_stream.read((char*)data.data, get_size);
	if (m_stream.gcount() < get_size)
		return false;
	img = data;
	return true;
}
