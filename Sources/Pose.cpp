#include "Pose.h"
#include <algorithm>

using namespace std;
using namespace cv;

Pose::Pose(vector<string> names, vector<Vec3d> positions) {
    this->ordered_positions = this->filterJoints(names, positions);
    this->ordered_bones = this->filterBones();
}

Pose::Pose() {
}

void Pose::jointInit(vector<jointnames::jointnames> labels, vector<Vec3d> positions) {
    if (labels.size() != positions.size())
        throw runtime_error("must be equal joints");

    this->ordered_positions = vector<Vec3d>(jointnames::NUMJOINTS);
    for (int i = 0; i < labels.size(); i++)
        ordered_positions[labels[i]] = positions[i];
    this->ordered_bones = this->filterBones();
}

Pose::Pose(vector<jointnames::jointnames> labels, vector<Vec3d> positions) {
    this->jointInit(labels, positions);
}

Pose::Pose(vector<Vec3d> positions) {
    vector<jointnames::jointnames> labels = vector<jointnames::jointnames>(jointnames::NUMJOINTS);
    for (int i = 0; i < jointnames::NUMJOINTS; i++)
        labels[i] = (jointnames::jointnames)i;
    this->jointInit(labels, positions);
}

Pose::~Pose() {
    //joints.clear();
    //nameiter.clear();
}

vector<Vec3d> Pose::filterJoints(vector<string> names, vector<Vec3d> positions) {
    //trade safety for speed
    //if (names.size() != positions.size())
    //    throw runtime_error("must be equal joints");
    vector<Vec3d> filteredPositions(jointnames::NUMJOINTS);
    for (int i = 0; i < names.size(); i++) {
        jointnames::jointnames currentJoint;
        currentJoint = this->strToJoint(names[i]);
        if (currentJoint == jointnames::NIL)
            continue;
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

vector<Vec3d> Pose::normLocToHip() {
    vector<Vec3d> normed;
    for (int i = 0; i < this->ordered_positions.size(); i++)
        normed.push_back(this->ordered_positions[i] - this->ordered_positions[jointnames::HIP]);
    return normed;
}

// see jiang "A" matrixB
Mat Pose::getLocalInverse() {
    Vec3d temp_xAxis = this->ordered_positions[jointnames::LFEMUR] - this->ordered_positions[jointnames::HIP];
    temp_xAxis /= norm(temp_xAxis);
    Vec3d yAxis = this->ordered_positions[jointnames::CLAVICLE] - this->ordered_positions[jointnames::HIP];
    yAxis /= norm(yAxis);
    Vec3d zAxis = temp_xAxis.cross(yAxis);
    Vec3d xAxis = yAxis.cross(zAxis);

    //Ainv
    return (Mat_<double>(3,3)
        << *xAxis.row(0).val, *xAxis.row(1).val, *xAxis.row(2).val,
           *yAxis.row(0).val, *yAxis.row(1).val, *yAxis.row(2).val,
           *zAxis.row(0).val, *zAxis.row(1).val, *zAxis.row(2).val);
}
//see jiang Vk descriptor
//to compare how close two descriptors are, you simply take the dot product of them.
//the closer this value is to ... NUMBONES=10, the better. (since there are NUMBONES unit vectors)
Mat Pose::getDescriptor() {
    //30-dimensional (ish), sheesh
    Mat descriptor = Mat(3 * bonenames::NUMBONES, 1, CV_64F);

    //the math says this may be unnecessary.
    //vector<Vec3d> normOrderedPos = this->normLocToHip();

    //it's okay that this doesn't use normOrderedPos as it handles that within it
    Mat Ainv = this->getLocalInverse();

    for (int i = 0; i < this->ordered_bones.size(); i++) {
        Bone u = this->ordered_bones[i];
        Vec3d uk = this->ordered_positions[u.end] - this->ordered_positions[u.start];
        //this is one spot where jiang's method simplifies things. Normalizing the torso's 
        //orientation allows for the algorithm to ignore camera orientation relative to the person.
        Mat vk = Ainv * Mat(uk);
        vk /= norm(vk);
        for (int axis = 0; axis < 3; axis++) {
            //I think this is what I wanted. OpenCV seems a little arcane at times
            //at least for directly manipulating matrices.
            descriptor.at<double>(3.*i + axis, 0) = vk.at<double>(static_cast<double>(axis), 0);
        }
    }

    return descriptor;
}

//only for queries
vector<Vec3d> Pose::getJoints() {
    return this->ordered_positions;
}
vector<Bone> Pose::getBones() {
    return this->ordered_bones;
}
const Vec3d Pose::getJointPosition(jointnames::jointnames joint) {
    return this->ordered_positions[joint];
}
/*
const Vec3d Pose::getJointPosition(string jointName) {
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
std::string Pose::bonetoStr(bonenames::bonenames bone) {
    if      (bone == bonenames::HEAD)    return "  head";
    else if (bone == bonenames::TORSO)   return " torso";
    else if (bone == bonenames::LUPARM)  return "luparm";
    else if (bone == bonenames::LLOARM)  return "lloarm";
    else if (bone == bonenames::LUPLEG)  return "lupleg";
    else if (bone == bonenames::LLOLEG)  return "lloleg";
    else if (bone == bonenames::RUPARM)  return "ruparm";
    else if (bone == bonenames::RLOARM)  return "rloarm";
    else if (bone == bonenames::RUPLEG)  return "rupleg";
    else if (bone == bonenames::RLOLEG)  return "rloleg";
    else return "";
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
    else                              return jointnames::NIL;
}

void Pose::print() {
    vector<Bone> bones = this->getBones();
    vector<Vec3d> joints = this->getJoints();
    for (int k = 0; k < bones.size(); k++)
        cout << joints[bones[k].start] << "->" << joints[bones[k].end] << endl;

    Mat desc = this->getDescriptor();
    cout << "desc\n";
    for (int i = 0; i < desc.rows; i++) {
        if (i % 3 == 0)
            cout << bonetoStr((bonenames::bonenames)(i/3));
        cout << desc.row(i);
        if (i % 3 == 2)
            cout << endl;
    }
}
