#include <opencv2/opencv.hpp>
#include "timer.h"
#include <iostream>
#include <string>
#include "NN3D.h"

cv::Mat ortho_cam(double yaw, double pitch, double scale, cv::Vec3d trans);
cv::Mat ortho_cam(cv::Vec3d look, cv::Vec3d up, double scale, cv::Vec3d trans);
cv::Mat reproject(Pose solution, Pose original, cv::Mat camera, cv::Vec2i size);
void mouse_callback(int event, int x, int y, int flags, void * userdata);

struct Results {
    Mat img;
    Pose solution;
    Pose original;
    Mat camera; double yaw, pitch;
    bool project;
    Extractor * extractor;
};

//what you call
Results * initialize_parameters();

