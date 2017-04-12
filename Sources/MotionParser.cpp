#include "MotionParser.h"
#include <sstream>
#include <iostream>

// TODO: comments/documentation

// do not use outside of a .cpp or function
using namespace std;
using namespace cv;

MotionParser::MotionParser() {
    this->currentFile = ifstream();
    this->currentJointNames = vector<string>();
}
MotionParser::MotionParser(string path) {
    this->open(path);
}
MotionParser::~MotionParser() {
    this->currentFile = ifstream();
    this->currentJointNames = vector<string>();
}

void MotionParser::open(string path) {
    this->currentFile = ifstream(path);
    string header;
    getline(this->currentFile, header);
    this->currentJointNames = this->getJointNames(header);
}

Pose MotionParser::getNextPose() {
    std::string line;
    if (getline(this->currentFile, line))
        return Pose(this->currentJointNames, this->getJointPositions(line));
    else
        throw overflow_error("EOF");
}

MotionDB MotionParser::getMiniMotionDB() {
    MotionDB mini_db;

    mini_db.avgBoneLength = vector<double>(bonenames::NUMBONES, 0.0);
    int poseCount = 0;
    
    string file = static_cast<stringstream const&>(stringstream() << this->currentFile.rdbuf()).str();
    
    //for each pose
    size_t afterLastSplit = 0, nextSplitEnd = 0;
    while((nextSplitEnd = file.find("\n", afterLastSplit)) != string::npos) {
        if (nextSplitEnd == afterLastSplit)//EOF
            break;

        Pose p = Pose(this->currentJointNames, this->getJointPositions(file.substr(afterLastSplit, nextSplitEnd - afterLastSplit)));
        mini_db.descs.push_back(p.getDescriptor());

        vector<Bone> bones = p.getBones();
        vector<Vec3f> joints = p.getJoints();
        for (int i = 0; i < bonenames::NUMBONES; i++) {
            Vec3f diff = joints[bones[i].end] - joints[bones[i].start];
            double len = sqrt(diff.dot(diff));
            mini_db.avgBoneLength[i] = (len + poseCount * mini_db.avgBoneLength[i]) / (poseCount + 1);
        }
        poseCount++;

        afterLastSplit = nextSplitEnd + 1;
    }

    return mini_db;
}

MotionDB MotionParser::mergeMotionDB(MotionDB db1, MotionDB db2) {
    MotionDB db;
    //I hope these next two aren't super slow...
    //ideally they are replaced by a kd-tree
    db.descs.reserve(db1.descs.size() + db2.descs.size()); // preallocate memory
    db.descs.insert(db.descs.end(), db1.descs.begin(), db1.descs.end());
    db.descs.insert(db.descs.end(), db2.descs.begin(), db2.descs.end());
    

    db.avgBoneLength = vector<double>(bonenames::NUMBONES);
    for (int i = 0; i < bonenames::NUMBONES; i++)
        db.avgBoneLength[i] =
            (db1.avgBoneLength[i] * db1.descs.size() + db2.avgBoneLength[i] * db2.descs.size()) 
            / (db1.descs.size() + db2.descs.size());
    cout << db1.descs.size() << "+" << db2.descs.size() << "=" << db.descs.size() << "\n";
    return db;
}

vector<Mat> MotionParser::getAllPoseDescriptors() {
    string file = static_cast<stringstream const&>(stringstream() << this->currentFile.rdbuf()).str();
    vector<string> lines = split(file, '\n');
    vector<Mat> poseDescs = vector<Mat>(lines.size());
    for (int i = 0; i < lines.size(); i++)
        poseDescs[i] = Pose(this->currentJointNames, this->getJointPositions(lines[i])).getDescriptor();

    return poseDescs;
}

vector<Pose> MotionParser::getAllPoses() {
    string file = static_cast<stringstream const&>(stringstream() << this->currentFile.rdbuf()).str();
    vector<string> lines = split(file, '\n');
    vector<Pose> poses = vector<Pose>(lines.size());
    for (int i = 0; i < lines.size(); i++)
        poses[i] = Pose(this->currentJointNames, this->getJointPositions(lines[i]));

    return poses;
}

void MotionParser::close() {
    this->currentFile.close();
    this->currentJointNames = vector<string>();
}

//http://stackoverflow.com/questions/14265581/parse-split-a-string-in-c-using-string-delimiter-standard-c
vector<string> split(string s, char delim) {
    vector<string> toks;
    size_t last = 0;
    size_t next = 0;
    while ((next = s.find(delim, last)) != string::npos) {
        toks.push_back(s.substr(last, next - last));
        last = next + 1;
    }
    toks.push_back(s.substr(last));

    return toks;
}

vector<string> MotionParser::getJointNames(string header) {
    vector<string> vals = split(header, ',');
    vector<string> names;
    //remember the format is name:x name:y name:z
    for (int i = 0; i < vals.size(); i += 3)
        if (vals[i] != "")
            names.push_back(vals[i].substr(0, vals[i].size() - 2));

    return names;
}

vector<Vec3f> MotionParser::getJointPositions(string line) {
    vector<string> vals = split(line, ',');
    vector<Vec3f> positions;

    for (int i = 0; i+2 < vals.size(); i += 3) {
        float posf[3] = {0 ,0, 0};// x y z (will this even work?, being static and all?)
        for (int off = 0; off < 3; off++)
            if (vals[i + off] != "")
                posf[off] = stof(vals[i + off]);

        Vec3f pos = Vec3f(posf[0], posf[1], posf[2]);
        positions.push_back(pos);
    }
    
    return positions;
}
