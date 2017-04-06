#ifndef POSE_H
#define POSE_H
#pragma once

#include <opencv2\opencv.hpp>
#include <vector>
#include <string>
#include <unordered_map>

namespace bonenames {
    const int NUMBONES = 10;
    //keep the bonenames such that the hierarchy is preserved: (parent before child)
    enum bonenames { TORSO, HEAD, LUPARM, LLOARM, LUPLEG, LLOLEG, RUPARM, RLOARM, RUPLEG, RLOLEG };
}
namespace jointnames {
    const int NUMJOINTS = 35;
    enum jointnames {
        HIP, LFEMUR, LTIBIA, LFOOT, LTOES, LTOES_END, RFEMUR, RTIBIA, RFOOT, RTOES,
        RTOES_END, LOWERBACK, UPPERBACK, THORAX, LOWERNECK, UPPERNECK, HEAD, HEAD_END, CLAVICLE,
        LHUMERUS, LRADIUS, LWRIST, LHAND, LFINGERS, LFINGERS_END, LTHUMB, LTHUMB_END, RHUMERUS,
        RRADIUS, RWRIST, RHAND, RFINGERS, RFINGERS_END, RTHUMB, RTHUMB_END
    };
}

struct Bone {
    int/*jn*/ start;
    int/*jn*/ end;
    Bone() { start = 0; end = 0;}
    Bone(jointnames::jointnames start, jointnames::jointnames end) {
        this->start = start;
        this->end = end;
    };
};

class Pose {
public:
    Pose();
    Pose(std::vector<std::string> names, std::vector<cv::Vec3f> positions);
    
    void jointInit(std::vector<int/*jn*/> labels, std::vector<cv::Vec3f> positions);
    Pose(std::vector<cv::Vec3f> positions);
    Pose(std::vector<int/*jn*/> labels, std::vector<cv::Vec3f> positions);

    //I might need a copy constructor...
    ~Pose();
    const cv::Vec3f getJointPosition(jointnames::jointnames joint);
    //const cv::Vec3f getJointPosition(std::string jointName);
    std::vector<Bone> getBones();
    std::vector<cv::Vec3f> getJoints();

    //should probably be in NN3D
    cv::Mat getLocalInverse();
    void normLocToHip();
    cv::Mat getDescriptor();

private:
    std::vector<cv::Vec3f> filterJoints(std::vector<std::string> names, std::vector<cv::Vec3f> positions);
    std::vector<Bone> filterBones();

    bonenames::bonenames strToBone(std::string name);
    jointnames::jointnames strToJoint(std::string name);

    //these are ordered by their enumerators
    //float is always float[3]
    std::vector<cv::Vec3f> ordered_positions;
    std::vector<Bone> ordered_bones;
};

#endif//POSE_H
