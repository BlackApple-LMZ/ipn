#include <iostream>
#include <string>
#include <vector>
#include <fstream>

#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>

#include <chrono>

static const double SCALE = 0.416667; // 1.5*10^8*1000/f/30000 f=12*10^6
static const int ROWS = 240;
static const int COLS = 320;

union two_bytes
{
    uint16_t uint;
    uint8_t bytes[2];
};


cv::Mat LoadData(std::string str)
{
    FILE* fp = std::fopen(str.c_str(), "r");
    if(!fp) {
        std::perror("File opening failed");
        return cv::Mat();
    }

    cv::Mat img(ROWS, COLS, CV_8UC3, cv::Scalar(0,0,0));
    
    int c(0); // note: int, not char, required to handle EOF
    int count(0);
    float distance(0.0);
    union two_bytes tmp;
    int i(ROWS-1), j(0);
    while ((c = std::fgetc(fp)) != EOF) { 
        if(count%2){
            
            distance += c<<8;
            //tmp.bytes[1] = c;
            distance = distance * SCALE;
            
            img.ptr<cv::Vec3b>(i)[j][0] = 255*distance/12500.0;
            img.ptr<cv::Vec3b>(i)[j][1] = 255*(1-abs(distance-6250)/6250.0);
            img.ptr<cv::Vec3b>(i)[j][2] = 255*(1-distance/12500.0);
            
            if(distance<0.000001)
                img.ptr<cv::Vec3b>(i)[j][2] = 0;

            j++;
            if(j >= COLS){
                j = 0;
                i--;
            }
        }
        else
            distance = c;
            //tmp.bytes[0] = c;
        
        count++;
    }
    
    std::cout<<count<<std::endl;
    if (std::ferror(fp))
        std::puts("I/O error when reading");
    else if (std::feof(fp))
        std::puts("End of file reached successfully");
 
    std::fclose(fp);

    return img;
}

int main(){

    std::chrono::time_point<std::chrono::system_clock> start, end;
	start = std::chrono::system_clock::now();
	
	cv::Mat img = LoadData("/home/lmz/ros_ws/src/ipn_pointcloud/tests/12M.txt");
	
	end = std::chrono::system_clock::now();
	std::cout<<"time: "<<(std::chrono::duration_cast<std::chrono::microseconds> (end - start)).count()<<std::endl;
    
    cv::imshow("Distance Image", img);
    cv::waitKey(0);

    return 0;
}









