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
enum class EXTRACT { BY_REPROJECTION, BY_ITERATIVE_3D, BY_ITERATIVE_KD};
struct Extractor {
    PoseDB pdb;
    MotionDB mdb;
    kd_tree * kddb;
    EXTRACT method;
};
Extractor * init_3D_extractor(std::string databasepath, EXTRACT method, int increment);
Pose extract3D_from_Pose_2D(Extractor * e, Pose_2D pose);
Pose extract3D(Extractor * e, std::vector<jointnames::jointnames> labels, std::vector<cv::Point2d> points);

//see jiang
double pose_similar(cv::Mat poseDescriptor1, cv::Mat poseDescriptor2);
double pose_distant(cv::Mat poseDescriptor1, cv::Mat poseDescriptor2);
double findANN(Pose p, kd_tree * db);
double findANN_end(Pose p, MotionDB db);
double findANN_old(Pose p, MotionDB db);

double get_scale_3D_construct(std::vector<cv::Vec3d> joints2D, std::vector<Bone> bones2D, std::vector<double> avgBoneLength);
std::vector<double> get_depthdiff_3D_construct(std::vector<cv::Vec3d> joints2D, std::vector<Bone> bones2D, double scale, std::vector<double> avgBoneLength);
Pose search_possible_3D(std::vector<cv::Vec3d> joints2D, std::vector<Bone> bones2D, MotionDB db);
Pose search_possible_3D_by_kd(std::vector<cv::Vec3d> joints2D, std::vector<Bone> bones2D, std::vector<double> avgBoneLength, kd_tree * kddb);

Pose search_reprojections(std::vector<cv::Vec3d> joints2D, std::vector<Bone> bones2D, PoseDB db);

std::pair<std::vector<jointnames::jointnames>, std::vector<cv::Point2d>> Pose_2D_to_labeled_joints(Pose_2D p);
std::pair<std::vector<jointnames::jointnames>, std::vector<cv::Vec3d>> Pose2D_to_labeled_3Djoints(Pose_2D p);
#endif//NN3D_H
