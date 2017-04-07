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

vector<Pose> MotionParser::getAllPoses() {
    std::string line;
    vector<Pose> poses = vector<Pose>();
    while (getline(this->currentFile, line))
        poses.push_back(Pose(this->currentJointNames, this->getJointPositions(line)));

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
    while ((next = s.find(',', last)) != string::npos) {
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
