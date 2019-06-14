# include "ground.h"
# include "object.h"
# include "OriginPCL.h"
# include "confi.h"
# include <string>
# include <iostream>
# include <cmath>
# include <vector>
# include <fstream>
using namespace std;

const string PCDdir=Confi::dirPath_pcd;
const string resDir=Confi::dirPath_res;
const double realVolme=Confi::realVolme;


double computerVolume(const string PCDfileLocation)
{
    OriginPCL origin_PCL(PCDfileLocation);
    origin_PCL.segment_ground();
    origin_PCL.ObjectFilter();
    origin_PCL.saveToFile();
    return origin_PCL.ComputerVolume();
}

double  Accuracy(double measurementVolme,double realVolume)
{
    if(realVolme==0)
    {
        cerr<<"wrong realVolume"<<endl;
        return 0;
    }
    return abs(measurementVolme-realVolme)/realVolume;
}



int main(int argc, char const *argv[])
{
    vector<int> steps{0,15,30,45,60,90,180,360};
    
    for(auto step:steps)
    {
        // 文件路径
            // pcd文件
        const string PCDfileLocation=PCDdir+"step_"+to_string(step)+".pcd";
            // 结果文件（测量出来的精度保存位置）
        const string resFileLocation=resDir+"step_"+to_string(step)+".txt";

        // 测量
            // 体积
        double measurementVolme=computerVolume(PCDfileLocation);
            // 精度
        double accuracy=Accuracy(measurementVolme,realVolme);

        // 输出
            // 屏幕
        const string result="step_"+to_string(step)+".pcd"+"-------"+to_string(accuracy*100)+"%";
        cout<<result<<endl;
            // 文件
        fstream fout(resFileLocation);
        fout<<result<<endl;
        fout.close();
    }

        
    return 0;
}
