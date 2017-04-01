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

Pose filter_CMU(Pose cmuSkele);
//some "main" function that takes 2D points and gives back 3D points (i.e. a Pose)
//points Pose is known to have z = 0, else it is assumed to be that :P
//I will make a 2DPose class just for NN3D
Pose extract3D(Pose points);
//some projection function from 3D points to 2D Mat image
cv::Mat project(Pose solution);


//TODO: everything else...

#endif//NN3D_H
