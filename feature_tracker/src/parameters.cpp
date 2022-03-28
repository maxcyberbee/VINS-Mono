#include "parameters.h"
#include <iostream>
#include <string>
std::string IMAGE_TOPIC = "/image";
std::string IMU_TOPIC;
std::vector<std::string> CAM_NAMES;
std::string FISHEYE_MASK;
int MAX_CNT;
int MIN_DIST;
int WINDOW_SIZE;
int FREQ;
double F_THRESHOLD;
int SHOW_TRACK;
int STEREO_TRACK;
int EQUALIZE;
int ROW = 640;
int COL = 512;
int FOCAL_LENGTH;
int FISHEYE;
bool PUB_THIS_FRAME;

void readParameters()
{
    IMAGE_TOPIC = "/image";
    IMU_TOPIC = "/imu";
    CAM_NAMES.push_back("KANNALA_BRANDT");
    FISHEYE_MASK = "config/fisheye_mask.jpg";;
    MAX_CNT = 300;
    FREQ = 10;
    F_THRESHOLD = 1;
    SHOW_TRACK = 0;
    EQUALIZE = 0;
    ROW = 640;
    COL = 512;
    FISHEYE = 0;
    WINDOW_SIZE = 20;
    STEREO_TRACK = false;
    FOCAL_LENGTH = 460;
    PUB_THIS_FRAME = false;
}