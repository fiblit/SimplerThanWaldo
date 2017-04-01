#include "Pose.h"

using namespace std;

Pose::Pose(vector<string> names, vector<float[3]> positions) {
    for (int i = 0; i < names.size(); i++)
        this->joints[names[i]] = positions[i];
}


Pose::~Pose() {
    //joints.clear();
}

const float* Pose::getJointPosition(string jointName) {
    return this->joints[jointName];
}
