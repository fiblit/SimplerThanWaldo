#ifndef POSE_H
#define POSE_H
#pragma once

#include <opencv2\opencv.hpp>
#include <vector>
#include <string>
#include <unordered_map>

const int NUMBONES = 10;
enum bonenames { HEAD, TORSO, LUPARM, LLOARM, LUPLEG, LLOLEG, RUPARM, RLOARM, RUPLEG, RLOLEG };
const int NUMJOINTS = 35;
enum jointnames { HIP, LFEMUR, LTIBIA, LFOOT, LTOES, LTOES_END, RFEMUR, RTIBIA, RFOOT, RTOES,
    RTOES_END, LOWERBACK, UPPERBACK, THORAX, LOWERNECK, UPPERNECK, HEAD, HEAD_END, CLAVICLE,
    LHUMERUS, LRADIUS, LWRIST, LHAND, LFINGERS, LFINGERS_END, LTHUMB, LTHUMB_END, RHUMERUS,
    RRADIUS, RWRIST, RHAND, RFINGERS, RFINGERS_END, RTHUMB, RTHUMB_END
};

struct Bone {
    jointnames start;
    jointnames end;
    Bone(jointnames start, jointnames end) {
        this->start = start;
        this->end = end;
    };
};

class Pose {
public:
    Pose();
    Pose(std::vector<std::string> names, std::vector<cv::Vec3f> positions);
    //I might need a copy constructor...
    ~Pose();
    const cv::Vec3f getJointPosition(jointnames joint);
    //const cv::Vec3f getJointPosition(std::string jointName);
    std::vector<Bone> getBones();
    std::vector<cv::Vec3f> getJoints();

    //should probably be in NN3D
    cv::Mat getLocalInverse();
    void normLocToHip();
    cv::Mat getDescriptor();

    bool isNull; // dumb, weird hack
private:
    std::vector<cv::Vec3f> filterJoints(std::vector<std::string> names, std::vector<cv::Vec3f> positions);
    std::vector<Bone> filterBones();

    bonenames strToBone(std::string name);
    jointnames strToJoint(std::string name);

    //these are ordered by their enumerators
    //float is always float[3]
    std::vector<cv::Vec3f> ordered_positions;
    std::vector<Bone> ordered_bones;
};

#endif//POSE_H
