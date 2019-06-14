// RgbdViewer.cpp : 此文件包含 "main" 函数。程序执行将在此处开始并结束。
//
#include <iostream>
#include <unordered_map>
#include <fstream>
#include <opencv.hpp>
// #ifdef _WIN32
// #include <direct.h>
// #elif _linux
// #include <unistd.h>
// #endif //_DEBUG
# include<sys/stat.h>
#include <unistd.h>
#include <io.h> 
#include "confi.h"
#include "CameraReader.h"
#ifdef _WIN32
#ifdef _DEBUG
#pragma comment(lib, "opencv_core341d.lib")
#pragma comment(lib, "opencv_imgproc341d.lib")
#pragma comment(lib, "opencv_highgui341d.lib")
#pragma comment(lib, "opencv_imgcodecs341d.lib")
#else
#pragma comment(lib, "opencv_core341.lib")
#pragma comment(lib, "opencv_imgproc341.lib")
#pragma comment(lib, "opencv_highgui341.lib")
#pragma comment(lib, "opencv_imgcodecs341.lib")
#endif
#endif
using namespace std;



// the size of frames(both colour image and depth image)
const int Width = Confi::ImageWidth;
const int High = Confi::ImageHigh;
// the input file location;
const string CamPara_Readlocation = Confi::fileLocation_camParaBinary ;
const string Frames_Readlocation = Confi::fileLocation_rgbdBinary;
// the save path 
const string CamePara_SavePath = Confi::dirPath_frames;
const string Frames_SavePath_pre = Confi::dirPath_frames;
// the file save name(or just name postfix)
const string CamePara_SaveName = Confi::fileName_campara;
const string rgbFrames_SaveNamePostfix = Confi::postfixName_colorFrame;
const string depthFrames_SaveNamePostfix = Confi::postfixName_depthFrame;



int readCamPara();
int readRgbds();
int countFrames(); //count Frames
bool dirExists(const std::string &file_name); // check if the dir is exist
int  dirCreate(const std::string& dir_name); 



int main(int argc,char const *argv[])
{
	cout<<"total cout of the frames is :"<<countFrames()<<endl;
	readCamPara();
    readRgbds();
}




int readCamPara() 
{
	// read
	CamParaReader cam_para_reader;
	if (!cam_para_reader.open(CamPara_Readlocation.c_str()))
		return -1;

	// write
	CamPara cam_para; // const char* rn = "\r\n";
	if (cam_para_reader.next(cam_para))
	{
		// dir
		dirCreate(CamePara_SavePath.c_str());
		// write
		std::fstream out_stream(CamePara_SavePath+CamePara_SaveName, std::ios_base::binary | std::ios_base::trunc | std::ios_base::out);
		if (out_stream.is_open())
		{
			out_stream << Width << " " << High << endl;
			// out_stream << cam_para.color_inner[0] << " " << cam_para.color_inner[1] << endl;
			// out_stream << cam_para.color_inner[2] << " " << cam_para.color_inner[3] << endl;
			out_stream << cam_para.depth_inner[0] << " " << cam_para.depth_inner[1] << endl;
			out_stream << cam_para.depth_inner[2] << " " << cam_para.depth_inner[3] << endl;
			out_stream << endl;
			out_stream << Width << " " << High << endl;
			// out_stream << cam_para.depth_inner[0] << " " << cam_para.depth_inner[1] << endl;
			// out_stream << cam_para.depth_inner[2] << " " << cam_para.depth_inner[3] << endl;
			out_stream << cam_para.color_inner[0] << " " << cam_para.color_inner[1] << endl;
			out_stream << cam_para.color_inner[2] << " " << cam_para.color_inner[3] << endl;
			out_stream << endl;
			for (int i = 0; i < 3; ++i)
			{
				out_stream << cam_para.c2d_r[i * 3] << " " << cam_para.c2d_r[i * 3 + 1]
						   << cam_para.c2d_r[i * 3 + 2] << " " << cam_para.c2d_t[i] << endl;
			}
			out_stream << endl;
			out_stream << 0.0 << " " << 0.0 << endl;
		}
		//close
		out_stream.close();
	}

	// close
	cam_para_reader.close();
	return 1;
}



int readRgbds()
{
	// size
	const cv::Size size(Width, High);
	
	// steps(rotation step map)
	const int totalFrames=countFrames();
    map<int,int> steps;
	steps[0]	=1;
    steps[15]	=totalFrames/24;
    steps[30]	=totalFrames/12;
    steps[45]	=totalFrames/8;
    steps[60]	=totalFrames/6;
    steps[90]	=totalFrames/4;
    steps[180]	=totalFrames/2;
    steps[360]	=totalFrames/1;


	// write(according step)
	std::string out_dir;
	for(auto step:steps)
	{
		// reader
		RgbdReader reader;
		if (!reader.open(Frames_Readlocation.c_str()))
			return -1;
		cout<<step.first<<"-"<<step.second<<endl;
		// out_dir
		out_dir= Frames_SavePath_pre + "Frames_" + to_string(step.first) + "/";
		dirCreate(out_dir.c_str());
		// read and write;
		int index_source= 0;
		int index_write=0;
		bool exit = false;
		while (!exit)
		{
			// get current Frame;
			cv::Mat color, depth;
			if (!reader.next(color, size, CV_8UC3))
			{
				cout<<"break"<<endl;
				break;
			}
				
			if (!reader.next(depth, size, CV_16UC1))
				break;
			cv::imshow("COLOR", color);
			cv::imshow("DEPTH", depth);
			exit = ('q' == cv::waitKey(1));

			//  current Frame writed to file
			if (index_source % step.second  == 0)
			{
				// cout<<"---------------index_source:"<<index_source<<endl;
				// cout<<"---------------index_write:"<<index_write<<endl;
				// get the str type of index_write
				std::ostringstream os;os.fill('0');os.width(5);
				os << index_write;
				// write colour image
				cv::imwrite(out_dir + os.str() + ".ppm", color);
				// write depth image
				cv::imwrite(out_dir + os.str() + ".pgm", depth);
				// increase
				++index_source;
				++index_write;
			}
			else
			{
				++index_source;
				continue;
			}
		}
		// close reader
		reader.close();
	}
	
	return 1;
}



int countFrames() //count Frames
{
    
	// load frame file(saved as binary file)
    RgbdReader reader;
	if (!reader.open(Frames_Readlocation.c_str()))
		return -1;

    // computer the number of Frames
    const cv::Size size(Width, High);
	bool exit = false;
    int count=0;
    while(!exit)
    {
        //read image
        cv::Mat color,depth;
        if (!reader.next(color, size, CV_8UC3))
			break;
        if (!reader.next(depth, size, CV_16UC1))
			break;
        count++;
        exit = ('q' == cv::waitKey(1));
    }

    // close the frame file 
	reader.close();
	return count;
}



bool  dirExists(const std::string &file_name)
{

// #ifdef _WIN32
// 	int ftyp = _access(file_name.c_str(), 0);
// # elif linux
// 	int ftyp = access(file_name.c_str(), 0);
// # endif 

	int ftyp = access(file_name.c_str(), 0);
	if (0 == ftyp)
		return true;   // this is a directory!  
	else
		return false;    // this is not a directory!  
}




int dirCreate(const std::string& dir_name)
{
	if (!dirExists(dir_name))
	{
		std::cout << "now make it" << std::endl;
		// #ifdef _WIN32  
		// 		int flag = _mkdir(dir_name.c_str());
		// #elif linux   
		// 		int flag = _mkdir(dir.c_str(), 0777);
		// #endif  

		int flag = mkdir(dir_name.c_str(), 0777);
		if (flag != 0)
		{
			std::cout << "make errorly" << std::endl;
			return -1;
		}
		return 0;
	}
	return 1;
}





