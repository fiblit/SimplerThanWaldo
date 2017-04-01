#ifndef MOTIONPARSER_H
#define MOTIONPARSER_H
#pragma once

#include <fstream>
#include <vector>
#include "Pose.h"

// time does not matter to me, but most of these motion files were 120FPS, otherwise
// they were 60FPS. I could theoretically make a lookup-table to determine the FPS.
// motion file format: 
// bonename:x, bonename:y, bonename:z, ...
// x-val, y-val, z-val, ...
// ..., ..., ..., ...

class MotionParser {
public:
    //MotionParser();
    MotionParser(std::string path);
    ~MotionParser();

    void open(std::string path);
    Pose getNextPose();
    std::vector<Pose> getAllPoses();
private:
    std::ifstream currentFile;
    std::vector<std::string> currentJointNames;

    void close();
    std::vector<std::string> getJointNames(std::string header);
    std::vector<float[3]> getJointPositions(std::string line);
};

#endif//MOTIONPARSER_H
