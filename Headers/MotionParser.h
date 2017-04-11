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
    std::vector<cv::Mat> descs;
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
    MotionDB getMiniMotionDB();
    MotionDB mergeMotionDB(MotionDB db1, MotionDB db2);
private:
    std::ifstream currentFile;
    std::vector<std::string> currentJointNames;

    void close();
    std::vector<std::string> getJointNames(std::string header);
    std::vector<cv::Vec3f> getJointPositions(std::string line);
};

std::vector<std::string> split(std::string s, char delim);

#endif//MOTIONPARSER_H
