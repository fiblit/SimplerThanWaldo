#include "NN3D.h"
#include <experimental\filesystem>
#include <bitset>

using namespace std;
using namespace cv;
using namespace std::experimental::filesystem;

//I'm pretty sure they all have the same length... but I might as well take the average.
//also this should only ever need to be done once.
vector<double> getAvgBoneLength(string databasepath) {
    vector<double> runningAvg = vector<double>(bonenames::NUMBONES);
    int poseCount = 0;
    for (int i = 0; i < bonenames::NUMBONES; i++)
        runningAvg[i] = 0;

    //while there are motion files
    for (auto &p : recursive_directory_iterator(databasepath)) {
        //load file into a MotionParser
        MotionParser currentMotion(p.path().filename().string());

        //for each pose in currentMotion
        while (true) {
            Pose current;
            try {
                current = currentMotion.getNextPose();
            }
            catch (overflow_error&) {
                break;
            }

            vector<Bone> bones = current.getBones();
            vector<Vec3f> joints = current.getJoints();
            for (int i = 0; i < bonenames::NUMBONES; i++) {
                Vec3f diff = joints[bones[i].end] - joints[bones[i].start];
                double len = sqrt(diff.dot(diff));
                runningAvg[i] = (len + poseCount * runningAvg[i]) / (poseCount + 1);
            }
            poseCount++;
        }
    }

    return runningAvg;
}

//based on jiang
//labels.size() MUST equal points.size()
Pose extract3D(vector<jointnames::jointnames> labels, vector<Point2f> points, string databasepath) {
    //create 2D Pose
    vector<Vec3f> points3D = vector<Vec3f>(points.size());
    for (int i = 0; i < points.size(); i++)
        //the hip should be at 0 depth
        points3D[i] = Vec3f(points[i].x, points[i].y, 0);

    Pose estimate2D(labels, points3D);

    //calculate bone lengths
    vector<double> bonelength = getAvgBoneLength(databasepath);
    //calculate orthographic scale, s
    vector<Bone> bones2D = estimate2D.getBones();
    vector<Vec3f> joints2D = estimate2D.getJoints();//different order than points3D
    double s = 1;
    for (int k = 0; k < bonenames::NUMBONES; k++) {
        Vec3f start = joints2D[bones2D[k].start];
        Vec3f end = joints2D[bones2D[k].end];
        double dx = end[0] - start[0];
        double dy = end[1] - start[1];
        //I don't know what jiang was doing, but I'm 95% sure this is wrong.
        //Shouldn't it by dx^2 PLUS dy^2 ? There's no way minus makes sense...
        s = max(s, sqrt(dx*dx + dy*dy) / bonelength[k]);
    }

    //calculate projected depth difference (dZ) of each endpoint
    //these aren't bones
    vector<double> depthDiff = vector<double>(bonenames::NUMBONES);
    for (int k = 0; k < bonenames::NUMBONES; k++) {
        double scaledLen = s*bonelength[k];
        Vec3f start = joints2D[bones2D[k].start];
        Vec3f end = joints2D[bones2D[k].end];
        double dx = end[0] - start[0];
        double dy = end[1] - start[1];
        depthDiff[k] = sqrt(scaledLen*scaledLen - dx*dx - dy*dy);
    }
   
    Pose final;
    double mostSimilar = numeric_limits<double>::lowest();
    //for each possible flipping of the points' 3D coordinates signs
    //a bit-level 0 means negative, a 1 means positive
    for (unsigned short boneDepthSign = 0; boneDepthSign < (1 << bonenames::NUMBONES); boneDepthSign++) {
        vector<Vec3f> guessPositions = joints2D;
        vector<Bone> guessBones = bones2D;
        for (int k = 0; k < bonenames::NUMBONES; k++) {
            bool sign = bitset<bonenames::NUMBONES>(boneDepthSign)[k];
            //the bonenames are ordered in such a way that this will work fine
            //end.z = (sign)dZ + start.z
            guessPositions[guessBones[k].end][2] = static_cast<float>((sign ? 1 : -1) * depthDiff[k] + guessPositions[guessBones[k].start][2]);
        }

        Pose guess(guessPositions);
        double similarity = findANN(guess, databasepath);
        if (similarity > mostSimilar) {
            final = guess;
            mostSimilar = similarity;
        }
    }

    return final;
}

Mat reproject(Pose solution, Mat camera) {
    //draw each bone as a projected line using the camera matrix as the projection
    //undoubtedly need more parameters, I'm just lazy.
    //The resulting mat is a 2D image. Should probably just be the projected space of
    //the pose, and another function should actually draw it.
    return Mat();
}

//the closer to NUMBONES (i.e. the larger) the better.
double comparePose(Mat poseDescriptor1, Mat poseDescriptor2) {
    return poseDescriptor1.dot(poseDescriptor2);
}

//not actually *approximate* yet. I was going to read some locality-sensitive hashing stuff for that.
double findANN(Pose guess, string databasepath) {
    double closest = numeric_limits<double>::lowest();
    Mat guessDesc = guess.getDescriptor();

    //while there are motion files
    for (auto &p: recursive_directory_iterator(databasepath)) {
        //load file into a MotionParser
        MotionParser currentMotion(p.path().filename().string());

        //for each pose in currentMotion
        while (true) {
            Pose actual;
            try {
                actual = currentMotion.getNextPose();
            }
            catch (overflow_error&) {
                break;
            }

            // get pose descriptor for current pose
            Mat actualDesc = actual.getDescriptor();

            //nearest neighbour's distance; jiang's method doesn't care what the real pose looks like.
            //His method just cares that the guessed pose is close to a real pose.
            closest = max(closest, comparePose(guessDesc, actualDesc));
        }
    }
    return closest;
}
