#include "Pose.h"

using namespace std;

Pose::Pose(vector<string> names, vector<float[3]> positions) {
    this->nameiter = names;
    this->positer = positions;
    for (int i = 0; i < names.size(); i++)
        this->joints[names[i]] = positions[i];
}


Pose::~Pose() {
    //joints.clear();
    //nameiter.clear();
}


std::vector<std::string> Pose::getJointNames() {
    return this->nameiter;
}

std::vector<float[3]> Pose::getJointPositions() {
    return this->positer;
}

const float* Pose::getJointPosition(string jointName) {
    return this->joints[jointName];
}
