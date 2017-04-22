#ifndef MOTIONPARSER_H
#define MOTIONPARSER_H
#pragma once

#include <fstream>
#include <vector>
#include <opencv2/opencv.hpp>
#include "Pose.h"

// time does not matter to me, but most of these motion files were 120FPS, otherwise
// they were 60FPS. I could theoretically make a lookup-table to determine the FPS.
// motion file format: 
// bonename:x, bonename:y, bonename:z, ...
// x-val, y-val, z-val, ...
// ..., ..., ..., ...


struct MotionDB {
    std::vector<cv::Mat> descs; //todo vector -> kd-tree
    //descs are 30x1 row vectors
    std::vector<double> avgBoneLength;
};

struct PoseDB {
    std::vector<Pose> poses;
    std::vector<double> avgBoneLength;
};


class MotionParser {
public:
    MotionParser();
    MotionParser(std::string path);
    ~MotionParser();

    void open(std::string path);
    Pose getNextPose();
    std::vector<Pose> getAllPoses();
    std::vector<cv::Mat> getAllPoseDescriptors();

    void MotionParser::updateMotionDB(MotionDB * db, int increment);
    void MotionParser::updatePoseDB(PoseDB * db, int increment);

    //deprecated
    MotionDB getMiniMotionDB();
    void mergeMotionDB(MotionDB * db1, MotionDB db2);
private:
    std::ifstream currentFile;
    std::vector<std::string> currentJointNames;

    void close();
    std::vector<std::string> getJointNames(std::string header);
    std::vector<cv::Vec3d> getJointPositions(std::string line);
};

std::vector<std::string> split(std::string s, char delim);

#endif//MOTIONPARSER_H
