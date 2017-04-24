#ifndef POSE_H
#define POSE_H
#pragma once

#include <opencv2/opencv.hpp>
#include <vector>
#include <string>
#include <unordered_map>

namespace bonenames {
    const int NUMBONES = 10;
    //keep the bonenames such that the hierarchy is preserved: (parent before child)
    enum bonenames { TORSO, HEAD, LUPARM, LLOARM, LUPLEG, LLOLEG, RUPARM, RLOARM, RUPLEG, RLOLEG, NIL};
}
namespace jointnames {
    const int NUMJOINTS = 35;
    enum jointnames {
        HIP, LFEMUR, LTIBIA, LFOOT, LTOES, LTOES_END, RFEMUR, RTIBIA, RFOOT, RTOES,
        RTOES_END, LOWERBACK, UPPERBACK, THORAX, LOWERNECK, UPPERNECK, HEAD, HEAD_END, CLAVICLE,
        LHUMERUS, LRADIUS, LWRIST, LHAND, LFINGERS, LFINGERS_END, LTHUMB, LTHUMB_END, RHUMERUS,
        RRADIUS, RWRIST, RHAND, RFINGERS, RFINGERS_END, RTHUMB, RTHUMB_END, NIL
    };
}

struct Bone {
    jointnames::jointnames start;
    jointnames::jointnames end;
    Bone() { start = (jointnames::jointnames)0; end = (jointnames::jointnames)0;}
    Bone(jointnames::jointnames start, jointnames::jointnames end) {
        this->start = start;
        this->end = end;
    };
};

class Pose {
public:
    Pose();
    Pose(std::vector<std::string> names, std::vector<cv::Vec3d> positions);
    
    void jointInit(std::vector<jointnames::jointnames> labels, std::vector<cv::Vec3d> positions);
    Pose(std::vector<cv::Vec3d> positions);
    Pose(std::vector<jointnames::jointnames> labels, std::vector<cv::Vec3d> positions);

    //I might need a copy constructor...
    ~Pose();
    const cv::Vec3d getJointPosition(jointnames::jointnames joint);
    //const cv::Vec3d getJointPosition(std::string jointName);
    std::vector<Bone> getBones();
    std::vector<cv::Vec3d> getJoints();

    //should probably be in NN3D
    cv::Mat getLocalInverse();
    std::vector<cv::Vec3d> normLocToHip();
    cv::Mat getDescriptor();
    cv::Mat getEndpointDescriptor();

    //enum converters
    static bonenames::bonenames strToBone(std::string name);
    static std::string bonetoStr(bonenames::bonenames bone);
    static jointnames::jointnames strToJoint(std::string name);

    void print();

private:
    std::vector<cv::Vec3d> filterJoints(std::vector<std::string> names, std::vector<cv::Vec3d> positions);
    std::vector<Bone> filterBones();

    //these are ordered by their enumerators
    //float is always float[3]
    std::vector<cv::Vec3d> ordered_positions;
    std::vector<Bone> ordered_bones;
};

#endif//POSE_H
