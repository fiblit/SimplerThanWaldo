#include "NN3D.h"
#include <experimental\filesystem>
#include <bitset>

using namespace std;
using namespace cv;
using namespace std::experimental::filesystem;

#include <chrono>
typedef std::chrono::high_resolution_clock Clock;

//TODO: also return meta information(like avg bone lengths)/an actual class/struct for the DB
MotionDB createDB(string path) {
    MotionDB db;
    db.descs = vector<Mat>();
    db.avgBoneLength = vector<double>(bonenames::NUMBONES, 0.0);

    //while there are motion files
    cout << "i/o start" << endl;
    auto t1 = Clock::now();
    for (auto &p : recursive_directory_iterator(path)) {
        MotionParser mp;
        //load file into MotionParser
        if (!is_directory(p.path())) {

            //cout << "\tmp start " << p.path().filename().string() << endl;
            //auto mpt1 = Clock::now();

            mp.open(p.path().string());

            //cout << "\t\tmp update start" << p.path().filename().string() << endl;
            //auto mpt_1 = Clock::now();
            mp.updateMotionDB(&db);
            //auto mpt_2 = Clock::now();
            //cout << "\t\tmp update in "
            //    << chrono::duration_cast<chrono::nanoseconds>(mpt_2 - mpt_1).count()
            //    << endl;

            /*
            cout << "\t\tmp get start " << p.path().filename().string() << endl;
            auto mpt_1 = Clock::now();
            MotionDB poses = mp.getMiniMotionDB();
            auto mpt_2 = Clock::now();
            cout << "\t\tmp get in "
                << chrono::duration_cast<chrono::nanoseconds>(mpt_2 - mpt_1).count()
                << endl;

            cout << "\t\tmp merge start " << p.path().filename().string() << endl;
            mpt_1 = Clock::now();
            mp.mergeMotionDB(&db, poses);
            mpt_2 = Clock::now();
            cout << "\t\tmp merge in "
                << chrono::duration_cast<chrono::nanoseconds>(mpt_2 - mpt_1).count()
                << endl;

            auto mpt2 = Clock::now();
            cout << "\tmp in "
                << chrono::duration_cast<chrono::nanoseconds>(mpt2 - mpt1).count()
                << endl;
            */
            
        }
    }
    auto t2 = Clock::now();
    cout << "i/o in "
        << chrono::duration_cast<chrono::nanoseconds>(t2 - t1).count() / 1000000000.
        << endl;

    return db;
}

//I'm pretty sure they all have the same length... but I might as well take the average.
//also this should only ever need to be done once.
/*
vector<double> getAvgBoneLength(MotionDB db) {
    vector<double> runningAvg = vector<double>(bonenames::NUMBONES);
    int poseCount = 0;
    for (int i = 0; i < bonenames::NUMBONES; i++)
        runningAvg[i] = 0;

    auto t1 = Clock::now();
    //while there are poses
    for (Pose p : db) {
        vector<Bone> bones = p.getBones();
        vector<Vec3f> joints = p.getJoints();
        for (int i = 0; i < bonenames::NUMBONES; i++) {
            Vec3f diff = joints[bones[i].end] - joints[bones[i].start];
            double len = sqrt(diff.dot(diff));
            runningAvg[i] = (len + poseCount * runningAvg[i]) / (poseCount + 1);
        }
        poseCount++;
    }
    auto t2 = Clock::now();
    cout << "search"
        << chrono::duration_cast<chrono::nanoseconds>(t2 - t1).count()
        << endl;

    return runningAvg;
}
*/

#include <chrono>
typedef std::chrono::high_resolution_clock Clock;

//based on jiang
//labels.size() MUST equal points.size()
Pose extract3D(vector<jointnames::jointnames> labels, vector<Point2f> points, string dbpath) {
    auto t_start = Clock::now();

    //create 2D Pose
    vector<Vec3f> points3D = vector<Vec3f>(points.size());
    for (int i = 0; i < points.size(); i++)
        //the hip should be at 0 depth
        points3D[i] = Vec3f(points[i].x, points[i].y, 0);

    Pose estimate2D(labels, points3D);
    //estimate2D.print();

    //calculate bone lengths
    auto t0 = Clock::now();
    cout << "after create 2d skele in 3d in"
        << chrono::duration_cast<chrono::nanoseconds>(t0 - t_start).count() / 1000000000.
        << endl;

    //release mode breakpoint; because debug is 10x slower

    MotionDB db = createDB(dbpath);
    auto t_ = Clock::now();
    cout << "after db in " 
        << chrono::duration_cast<chrono::nanoseconds>(t_ - t0).count() / 1000000000.
        << endl;

    vector<double> bonelength = db.avgBoneLength;
    //calculate orthographic scale, s
    auto t1 = Clock::now();
    cout << "after get avg bone length in " 
        << chrono::duration_cast<chrono::nanoseconds>(t1 - t_).count() / 1000000000.
        << endl;
    vector<Bone> bones2D = estimate2D.getBones();
    vector<Vec3f> joints2D = estimate2D.getJoints();//different order than points3D
    auto t2 = Clock::now();
    cout << "after get bones & joints in " 
        << chrono::duration_cast<chrono::nanoseconds>(t2 - t1).count() / 1000000000.
        << endl;
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

    auto t3 = Clock::now();
    cout << "after get scale in " 
        << chrono::duration_cast<chrono::nanoseconds>(t3 - t2).count() / 1000000000.
        << endl;

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
   
    auto t4 = Clock::now();
    cout << "after orthographic depth diff in " 
        << chrono::duration_cast<chrono::nanoseconds>(t4 - t3).count() / 1000000000.
        << endl;

    Pose finalPose;
    double mostSimilar = numeric_limits<double>::lowest();
    //for each possible flipping of the points' 3D coordinates signs
    //a bit-level 0 means negative, a 1 means positive

    //could be parallelised, however the biggest speed up would come from a spatial data strucutre as that's O(n) -> O(log n)
    for (unsigned short boneDepthSign = 0; boneDepthSign < (1 << bonenames::NUMBONES); boneDepthSign++) {
        vector<Vec3f> guessPositions = joints2D;
        vector<Bone> guessBones = bones2D;

        auto ann1 = Clock::now();
        cout << "\tguess generation" << endl;
        for (int k = 0; k < bonenames::NUMBONES; k++) {
            bool sign = bitset<bonenames::NUMBONES>(boneDepthSign)[k];
            //the bonenames are ordered in such a way that this will work fine
            //end.z = (sign)dZ + start.z
            guessPositions[guessBones[k].end][2] = static_cast<float>((sign ? 1 : -1) * depthDiff[k] + guessPositions[guessBones[k].start][2]);
        }

        Pose guess(guessPositions);
        auto ann2 = Clock::now();
        cout << "\t in "
            << chrono::duration_cast<chrono::nanoseconds>(ann2 - ann1).count() / 1000000000.
            << endl;

        auto ann3 = Clock::now();
        cout << "\tfind ann" << endl;
        double similarity = findANN(guess, db);
        if (similarity > mostSimilar) {
            finalPose = guess;
            mostSimilar = similarity;
        }
        auto ann4 = Clock::now();
        cout << "\t in "
            << chrono::duration_cast<chrono::nanoseconds>(ann4 - ann3).count() / 1000000000.
            << endl;
    }

    auto f = Clock::now();
    cout << "after ANN search over all possible depth placements in " 
        << chrono::duration_cast<chrono::nanoseconds>(f - t4).count() / 1000000000.
        << endl;

    cout << "------------------------" << endl;
    finalPose.print();

    return finalPose;
}

Mat reproject(Pose solution, Mat camera, int outW, int outH) {
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
double findANN(Pose guess, MotionDB db) {

    //auto t1 = Clock::now();
    double closest = numeric_limits<double>::lowest();
    Mat guessDesc = guess.getDescriptor();

    //while there are motion files
    for (Mat desc : db.descs) {
        //uto t1 = Clock::now();

        //nearest neighbour's distance; jiang's method doesn't care what the real pose looks like.
        //His method just cares that the guessed pose is close to a real pose.
        closest = max(closest, comparePose(guessDesc, desc));
     //   auto t2 = Clock::now();;
     //   cout << "\tNNcomp in "
     //       << chrono::duration_cast<chrono::nanoseconds>(t2 - t1).count()
    //        << endl;
    }

    //auto t2 = Clock::now();
    //cout << "NN in "
    //    << chrono::duration_cast<chrono::nanoseconds>(t2 - t1).count() / 1000000000.
    //    << endl;
    return closest;
}
