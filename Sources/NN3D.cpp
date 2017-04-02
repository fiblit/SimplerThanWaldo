#include "NN3D.h"
#include <experimental\filesystem>

using namespace std;
using namespace cv;
using namespace std::experimental::filesystem;


//I'm pretty sure they all have the same length... but I might as well take the average.
//also this should only ever need to be done once.
vector<double> getAvgBoneLength(string databasepath) {
    vector<double> runningAvg = vector<double>(NUMBONES);
    int poseCount = 0;
    for (int i = 0; i < NUMBONES; i++)
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
            catch (overflow_error& e) {
                break;
            }

            vector<Bone> bones = current.getBones();
            vector<Vec3f> joints = current.getJoints();
            for (int i = 0; i < NUMBONES; i++) {
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
Pose extract3D(vector<jointnames> labels, vector<Point2f> points, string databasepath) {
    //calculate bone lengths
    vector<double> bonelength = getAvgBoneLength(databasepath);
    //calculate orthographic scale, s
    //s = max_k(sqrt(dx_k^2 - dy_k^2)/bonelength(k))
    //calculate projected depth difference (dZ) of each point
        //dZ_k^2 = (s*l_k)^2 - dx_k^2 - dy_k^2
    //create Pose "final"
    //mostSimilar = -inf
    //for each possible flipping of the points' 3D coordinates signs
        //create Pose "guess"
        //similarity = findANN(guess, DB)
        //if similarity > mostSimilar
            //final = guess
            //mostSimilar = similarity
    //return final
}

Mat reproject(Pose solution, Mat camera) {
    //draw each bone as a projected line using the camera matrix as the projection
    //undoubtedly need more parameters, I'm just lazy.
    //The resulting mat is a 2D image. Should probably just be the projected space of
    //the pose, and another function should actually draw it.
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
            catch (overflow_error& e) {
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
