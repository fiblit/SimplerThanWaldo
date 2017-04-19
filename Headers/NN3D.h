#ifndef NN3D_H
#define NN3D_H
#pragma once

#include <opencv2/opencv.hpp>
#include "MotionParser.h"
#include "Pose.h"
#include "kd_tree.h"
#include "Pose_2D.h"

/*
Perform ANN on the pose database using MotionParser to read in motionfiles/searchposes and Pose to represent poses.
Once the nearest pose has been found, I'll need to properly project it into 3D space using the 2D pose.
I may pre-read the poses so as to structure them into a spatial data-structure.
I'll actually do this tomorrow morning. Been a bit busy.
*/

MotionDB createDB(std::string databasepath);

//some "main" function that takes 2D points and gives back a Pose
Pose extract3D_from_Pose_2D(Pose_2D pose, std::string databasepath);
Pose extract3D(std::vector<jointnames::jointnames> labels, std::vector<cv::Point2d> points, std::string databasepath);
//some projection function from 3D points to 2D Mat image
cv::Mat reproject(Pose solution, cv::Mat virtualCamera, cv::Vec2i size);

//see jiang
double pose_similar(cv::Mat poseDescriptor1, cv::Mat poseDescriptor2);
double pose_distant(cv::Mat poseDescriptor1, cv::Mat poseDescriptor2);
double findANN(Pose p, kd_tree * db);
double findANN_old(Pose p, MotionDB db);
std::pair<Pose, double> find_proj_ANN(Pose P, PoseDB * db);

double get_scale_3D_construct(std::vector<Vec3d> joints2D, std::vector<Bone> bones2D);
std::vector<double> get_depthdiff_3D_construct(std::vector<Vec3d> joints2D, std::vector<Bone> bones2D);
Pose search_possible_3D(std::vector<Vec3d> joints2D, std::vector<Bone> bones2D, MotionDB db);
Pose search_reprojections(std::vector<Vec3d> joints2D, std::vector<Bone> bones2D, PoseDB db);

#endif//NN3D_H
