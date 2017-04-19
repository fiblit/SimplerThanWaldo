#ifndef POSE_2D_H
#define POSE_2D_H

#include <opencv2/opencv.hpp>

struct Pose_2D
{
    int Initialized_Positions;
    cv::Point Head_Top;
    cv::Point Head_Bot;
    cv::Point C_Chest;
    cv::Point L_Chest;
    cv::Point R_Chest;
    cv::Point C_Hip;
    cv::Point L_Hip;
    cv::Point R_Hip;
    cv::Point L_Elbow;
    cv::Point R_Elbow;
    cv::Point L_Wrist;
    cv::Point R_Wrist;
    cv::Point L_Knee;
    cv::Point R_Knee;
    cv::Point L_Ankle;
    cv::Point R_Ankle;
};

#endif//POSE_2D_H