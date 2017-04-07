#ifndef NN3D_H
#define NN3D_H
#pragma once

#include <opencv2/opencv.hpp>
#include "MotionParser.h"
#include "Pose.h"

/*
Perform ANN on the pose database using MotionParser to read in motionfiles/searchposes and Pose to represent poses.
Once the nearest pose has been found, I'll need to properly project it into 3D space using the 2D pose.
I may pre-read the poses so as to structure them into a spatial data-structure.
I'll actually do this tomorrow morning. Been a bit busy.
*/

typedef std::vector<Pose> MotionDB;
MotionDB createDB(std::string databasepath);

//some "main" function that takes 2D points and gives back a Pose
Pose extract3D(std::vector<jointnames::jointnames> labels, std::vector<cv::Point2f> points, std::string databasepath);
//some projection function from 3D points to 2D Mat image
cv::Mat reproject(Pose solution, cv::Mat virtualCamera, int outW, int outH);

//see jiang
double comparePose(cv::Mat poseDescriptor1, cv::Mat poseDescriptor2);
double findANN(Pose p, MotionDB db);

#endif//NN3D_H
