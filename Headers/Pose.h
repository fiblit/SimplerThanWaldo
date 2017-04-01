#ifndef POSE_H
#define POSE_H
#pragma once

#include <vector>
#include <string>
#include <unordered_map>

class Pose {
public:
    Pose(std::vector<std::string> names, std::vector<float[3]> positions);
    ~Pose();
    const float* getJointPosition(std::string jointName);
    std::vector<std::string> getJointNames();
    std::vector<float[3]> getJointPositions();
private:
    std::vector<float[3]> positer;
    std::vector<std::string> nameiter;
    std::unordered_map<std::string, float *> joints;
};

#endif//POSE_H
