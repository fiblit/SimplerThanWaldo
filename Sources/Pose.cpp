#include "Pose.h"
#include <algorithm>

using namespace std;
using namespace cv;

Pose::Pose(vector<string> names, vector<Vec3f> positions) {
    this->ordered_positions = this->filterJoints(names, positions);
    this->ordered_bones = this->filterBones();
}

Pose::Pose() {
}


void Pose::jointInit(vector<jointnames::jointnames> labels, vector<Vec3f> positions) {
    if (labels.size() != positions.size())
        throw runtime_error("must be equal joints");

    this->ordered_positions = vector<Vec3f>(jointnames::NUMJOINTS);
    for (int i = 0; i < labels.size(); i++)
        ordered_positions[labels[i]] = positions[i];
    this->ordered_bones = this->filterBones();
}

Pose::Pose(vector<jointnames::jointnames> labels, vector<Vec3f> positions) {
    this->jointInit(labels, positions);
}

Pose::Pose(vector<Vec3f> positions) {
    vector<jointnames::jointnames> labels = vector<jointnames::jointnames>(jointnames::NUMJOINTS);
    for (int i = 0; i < jointnames::NUMJOINTS; i++)
        labels[i] = (jointnames::jointnames)i;
    this->jointInit(labels, positions);
}

Pose::~Pose() {
    //joints.clear();
    //nameiter.clear();
}

vector<Vec3f> Pose::filterJoints(vector<string> names, vector<Vec3f> positions) {
    if (names.size() != positions.size())
        throw runtime_error("must be equal joints");

    vector<Vec3f> filteredPositions = vector<Vec3f>(jointnames::NUMJOINTS);
    for (int i = 0; i < names.size(); i++) {
        jointnames::jointnames currentJoint;
        try {
            currentJoint = this->strToJoint(names[i]);
        }
        catch (domain_error&) {
            continue;
        }
        filteredPositions[currentJoint] = positions[i];
    }
    return filteredPositions;
}

vector<Bone> Pose::filterBones() {
    vector<Bone> filteredBones = vector<Bone>(bonenames::NUMBONES);

    filteredBones[bonenames::HEAD] = Bone(jointnames::CLAVICLE, jointnames::HEAD_END);
    filteredBones[bonenames::TORSO] = Bone(jointnames::HIP, jointnames::CLAVICLE);
    filteredBones[bonenames::LUPARM] = Bone(jointnames::LHUMERUS, jointnames::LRADIUS);
    filteredBones[bonenames::LLOARM] = Bone(jointnames::LRADIUS, jointnames::LWRIST);
    filteredBones[bonenames::LUPLEG] = Bone(jointnames::LFEMUR, jointnames::LTIBIA);
    filteredBones[bonenames::LLOLEG] = Bone(jointnames::LTIBIA, jointnames::LFOOT);
    filteredBones[bonenames::RUPARM] = Bone(jointnames::RHUMERUS, jointnames::RRADIUS);
    filteredBones[bonenames::RLOARM] = Bone(jointnames::RRADIUS, jointnames::RWRIST);
    filteredBones[bonenames::RUPLEG] = Bone(jointnames::RFEMUR, jointnames::RTIBIA);
    filteredBones[bonenames::RLOLEG] = Bone(jointnames::RTIBIA, jointnames::RFOOT);

    return filteredBones;
}

void Pose::normLocToHip() {
    for (int i = 0; i < this->ordered_positions.size(); i++)
        this->ordered_positions[i] -= this->ordered_positions[jointnames::HIP];
}

// see jiang "A" matrixB
Mat Pose::getLocalInverse() {
    Vec3f temp_xAxis = this->ordered_positions[jointnames::LFEMUR] - this->ordered_positions[jointnames::HIP];
    Vec3f yAxis = this->ordered_positions[jointnames::LOWERBACK] - this->ordered_positions[jointnames::HIP];
    Vec3f zAxis = temp_xAxis.cross(yAxis);
    Vec3f xAxis = yAxis.cross(zAxis);

    //Ainv
    return (Mat_<float>(3,3)
        << *xAxis.row(0).val, *xAxis.row(1).val, *xAxis.row(2).val,
           *yAxis.row(0).val, *yAxis.row(1).val, *yAxis.row(2).val,
           *zAxis.row(0).val, *zAxis.row(1).val, *zAxis.row(2).val);
}
//see jiang Vk descriptor
//to compare how close two descriptors are, you simply take the dot product of them.
//the closer this value is to ... NUMBONES=10, the better. (since there are NUMBONES unit vectors)
Mat Pose::getDescriptor() {
    Mat descriptor = Mat(3 * bonenames::NUMBONES, 1, CV_64F);

    //the math says this may be unnecessary.
    this->normLocToHip();

    Mat Ainv = this->getLocalInverse();

    for (int i = 0; i < this->ordered_bones.size(); i++) {
        Bone u = this->ordered_bones[i];
        Vec3f uk = this->ordered_positions[u.end] - this->ordered_positions[u.start];

        //this is one spot where jiang's method simplifies things. Normalizing the torso's 
        //orientation allows for the algorithm to ignore camera orientation relative to the person.
        Mat vk = Ainv * Mat(-uk);
        vk = vk / sqrt(vk.dot(vk));
        for (int axis = 0; axis < 3; axis++)
            //I think this is what I wanted. OpenCV seems a little arcane at times
            //at least for directly manipulating matrices.
            descriptor.at<float>(i + axis, 1) = vk.at<float>(axis, 1);
    }

    return descriptor;
}

//only for queries
vector<Vec3f> Pose::getJoints() {
    return this->ordered_positions;
}
vector<Bone> Pose::getBones() {
    return this->ordered_bones;
}
const Vec3f Pose::getJointPosition(jointnames::jointnames joint) {
    return this->ordered_positions[joint];
}
/*
const Vec3f Pose::getJointPosition(string jointName) {
    //I should be doing a try-catch block, but I'm assuming I'll never hardcode an invalid
    //get. What I'm worried about is the CMU DB changing their names on me.
    return this->ordered_positions[this->strToJoint(jointName)];
}
*/

//enum converters
bonenames::bonenames Pose::strToBone(std::string name) {
    transform(name.begin(), name.end(), name.begin(), ::tolower);

    if      (name == "head")      return bonenames::HEAD;
    else if (name == "torso")     return bonenames::TORSO;
    else if (name == "luparm")    return bonenames::LUPARM;
    else if (name == "lloarm")    return bonenames::LLOARM;
    else if (name == "lupleg")    return bonenames::LUPLEG;
    else if (name == "lloleg")    return bonenames::LLOLEG;
    else if (name == "ruparm")    return bonenames::RUPARM;
    else if (name == "rloarm")    return bonenames::RLOARM;
    else if (name == "rupleg")    return bonenames::RUPLEG;
    else if (name == "rloleg")    return bonenames::RLOLEG;
    else                          throw domain_error("invalid bone name");
}
jointnames::jointnames Pose::strToJoint(std::string name) {
    transform(name.begin(), name.end(), name.begin(), ::tolower);

    //ignore root
    if      (name == "lhipjoint")     return jointnames::HIP;//lhipjoint == rhipjoint
    else if (name == "lfemur")        return jointnames::LFEMUR;
    else if (name == "ltibia")        return jointnames::LTIBIA;
    else if (name == "lfoot")         return jointnames::LFOOT;
    else if (name == "ltoes")         return jointnames::LTOES;
    else if (name == "ltoes_end")     return jointnames::LTOES_END;
    else if (name == "rfemur")        return jointnames::RFEMUR;
    else if (name == "rtibia")        return jointnames::RTIBIA;
    else if (name == "rfoot")         return jointnames::RFOOT;
    else if (name == "rtoes")         return jointnames::RTOES;
    else if (name == "rtoes_end")     return jointnames::RTOES_END;
    else if (name == "lowerback")     return jointnames::LOWERBACK;
    else if (name == "upperback")     return jointnames::UPPERBACK;
    else if (name == "thorax")        return jointnames::THORAX;
    else if (name == "lowerneck")     return jointnames::LOWERNECK;
    else if (name == "upperneck")     return jointnames::UPPERNECK;
    else if (name == "head")          return jointnames::HEAD;
    else if (name == "head_end")      return jointnames::HEAD_END;
    else if (name == "lclavicle")     return jointnames::CLAVICLE;//lclavicle == rclavicle
    else if (name == "lhumerus")      return jointnames::LHUMERUS;
    else if (name == "lradius")       return jointnames::LRADIUS;
    else if (name == "lwrist")        return jointnames::LWRIST;
    else if (name == "lhand")         return jointnames::LHAND;
    else if (name == "lfingers")      return jointnames::LFINGERS;
    else if (name == "lfingers_end")  return jointnames::LFINGERS_END;
    else if (name == "lthumb")        return jointnames::LTHUMB;
    else if (name == "lthumb_end")    return jointnames::LTHUMB_END;
    else if (name == "rhumerus")      return jointnames::RHUMERUS;
    else if (name == "rradius")       return jointnames::RRADIUS;
    else if (name == "rwrist")        return jointnames::RWRIST;
    else if (name == "rhand")         return jointnames::RHAND;
    else if (name == "rfingers")      return jointnames::RFINGERS;
    else if (name == "rfingers_end")  return jointnames::RFINGERS_END;
    else if (name == "rthumb")        return jointnames::RTHUMB;
    else if (name == "rthumb_end")    return jointnames::RTHUMB_END;
    else                              throw domain_error("invalid joint name");
}
