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

//deprecated; please don't use
Pose MotionParser::getNextPose() {
    std::string line;
    if (getline(this->currentFile, line))
        return Pose(this->currentJointNames, this->getJointPositions(line));
    else
        throw overflow_error("EOF");
}

void MotionParser::updatePoseDB(PoseDB * db, int increment) {
    string file = static_cast<stringstream const&>(stringstream() << this->currentFile.rdbuf()).str();

    int count = 0;

    for (size_t afterLastSplit = 0, nextSplitEnd = 0;
        (nextSplitEnd = file.find('\n', afterLastSplit)) != string::npos
        && nextSplitEnd != afterLastSplit;
        afterLastSplit = nextSplitEnd + 1) {

        if ((count++) % increment == 0) {
            count = 1;

            string line = file.substr(afterLastSplit, nextSplitEnd - afterLastSplit);
            vector<Vec3d> vjp = this->getJointPositions(line);
            Pose p = Pose(this->currentJointNames, vjp);

            vector<Bone> bones = p.getBones();
            vector<Vec3d> joints = p.getJoints();
            for (int i = 0; i < bonenames::NUMBONES; i++) {
                Vec3d diff = joints[bones[i].end] - joints[bones[i].start];
                double len = sqrt(diff.dot(diff));
                db->avgBoneLength[i] = (len + db->poses.size() * db->avgBoneLength[i]) / (db->poses.size() + 1);
            }

            db->poses.push_back(p);
        }
    }
}

void MotionParser::updateMotionDB(MotionDB * db, int increment) {
    string file = static_cast<stringstream const&>(stringstream() << this->currentFile.rdbuf()).str();

    //compare in aggregate case for with/without
    /*/
    size_t lineCount = count(file.begin(), file.end(), '\n');
    for (int i = 0; i < file.size(); i++)
        if (file[i] == '\n')
            lineCount++;
    size_t originSize = db->descs.size();

    db->descs.reserve(originSize + lineCount);
    //*/
     
    //for each pose
    //int i = 0;

    int count = 0;

    for ( size_t afterLastSplit = 0, nextSplitEnd = 0;
        //I should time that find
          (nextSplitEnd = file.find('\n', afterLastSplit)) != string::npos 
            && nextSplitEnd != afterLastSplit;//eof
          afterLastSplit = nextSplitEnd + 1) {

        if ((count++) % increment == 0) {
            count = 1;

            string line = file.substr(afterLastSplit, nextSplitEnd - afterLastSplit);

            vector<Vec3d> vjp = this->getJointPositions(line);

            Pose p = Pose(this->currentJointNames, vjp);

            vector<Bone> bones = p.getBones();
            vector<Vec3d> joints = p.getJoints();
            for (int i = 0; i < bonenames::NUMBONES; i++) {
                Vec3d diff = joints[bones[i].end] - joints[bones[i].start];
                double len = sqrt(diff.dot(diff));
                db->avgBoneLength[i] = (len + db->descs.size() * db->avgBoneLength[i]) / (db->descs.size() + 1);
            }

            Mat desc = p.getDescriptor();//.getEndpointDescriptor();//getDescriptor();

            db->descs.push_back(desc);
        }
    }
}

//deprecated; please don't use
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
        vector<Vec3d> joints = p.getJoints();
        for (int i = 0; i < bonenames::NUMBONES; i++) {
            Vec3d diff = joints[bones[i].end] - joints[bones[i].start];
            double len = sqrt(diff.dot(diff));
            mini_db.avgBoneLength[i] = (len + poseCount * mini_db.avgBoneLength[i]) / (poseCount + 1);
        }
        poseCount++;

        afterLastSplit = nextSplitEnd + 1;
    }

    return mini_db;
}


//deprecated; please don't use
void MotionParser::mergeMotionDB(MotionDB * db1, MotionDB db2) {
    //I hope these next two aren't super slow...
    //ideally they are replaced by a kd-tree
    int size = db1->descs.size();
    db1->descs.insert(db1->descs.end(), db2.descs.begin(), db2.descs.end());

    for (int i = 0; i < bonenames::NUMBONES; i++)
        db1->avgBoneLength[i] =
            (db1->avgBoneLength[i] * size + db2.avgBoneLength[i] * db2.descs.size()) 
            / (size + db2.descs.size());
    cout << size << "+" << db2.descs.size() << "=" << db1->descs.size() << "\n";

    //return db1;
}

//deprecated; please don't use
vector<Mat> MotionParser::getAllPoseDescriptors() {
    string file = static_cast<stringstream const&>(stringstream() << this->currentFile.rdbuf()).str();
    vector<string> lines = split(file, '\n');
    vector<Mat> poseDescs = vector<Mat>(lines.size());
    for (int i = 0; i < lines.size(); i++)
        poseDescs[i] = Pose(this->currentJointNames, this->getJointPositions(lines[i])).getDescriptor();

    return poseDescs;
}

//deprecated; please don't use
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

vector<Vec3d> MotionParser::getJointPositions(string line) {
    vector<Vec3d> positions;

    double posf[3] = {0,0,0};
    int i = 0;
    for (size_t afterLastSplit = 0, nextSplitEnd = 0;
        (nextSplitEnd = line.find(',', afterLastSplit)) != string::npos
        && nextSplitEnd != afterLastSplit;//eol
        afterLastSplit = nextSplitEnd + 1) {
        string val = line.substr(afterLastSplit, nextSplitEnd - afterLastSplit);
        posf[i++] = stod(val);

        if (i == 3) {
            positions.push_back(Vec3d(posf[0], posf[1], posf[2]));
            i = 0;
        }
    }

    return positions;
}
