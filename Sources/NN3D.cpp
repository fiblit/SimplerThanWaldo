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
            mp.open(p.path().string());
            mp.updateMotionDB(&db);
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

    //while there are poses
    for (Pose p : db) {
        vector<Bone> bones = p.getBones();
        vector<Vec3d> joints = p.getJoints();
        for (int i = 0; i < bonenames::NUMBONES; i++) {
            Vec3d diff = joints[bones[i].end] - joints[bones[i].start];
            double len = sqrt(diff.dot(diff));
            runningAvg[i] = (len + poseCount * runningAvg[i]) / (poseCount + 1);
        }
        poseCount++;
    }

    return runningAvg;
}
*/

#include <chrono>
typedef std::chrono::high_resolution_clock Clock;

//based on jiang
//labels.size() MUST equal points.size()
Pose extract3D(vector<jointnames::jointnames> labels, vector<Point2d> points, string dbpath) {
    auto t_start = Clock::now();

    //create 2D Pose
    vector<Vec3d> points3D = vector<Vec3d>(points.size());
    for (int i = 0; i < points.size(); i++)
        //the hip should be at 0 depth
        points3D[i] = Vec3d(points[i].x, points[i].y, 0);

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
    cout << "avg bone lengths : ";
    for (int i = 0; i < bonelength.size(); i++)
        cout << "bone " << Pose::bonetoStr((bonenames::bonenames)i) << ": " << bonelength[i] << endl;

    cout << "after get avg bone length in " 
        << chrono::duration_cast<chrono::nanoseconds>(t1 - t_).count() / 1000000000.
        << endl;
    vector<Bone> bones2D = estimate2D.getBones();
    vector<Vec3d> joints2D = estimate2D.getJoints();//different order than points3D
    auto t2 = Clock::now();
    cout << "bones & joints: ";
    estimate2D.print();
    cout << endl;
    cout << "after get bones & joints in " 
        << chrono::duration_cast<chrono::nanoseconds>(t2 - t1).count() / 1000000000.
        << endl;
    double s = 1;
    for (int k = 0; k < bonenames::NUMBONES; k++) {
        Vec3d start = joints2D[bones2D[k].start];
        Vec3d end = joints2D[bones2D[k].end];
        double dx = end[0] - start[0];
        double dy = end[1] - start[1];
        //I don't know what jiang was doing, but I'm 95% sure this is wrong.
        //Shouldn't it by dx^2 PLUS dy^2 ? There's no way minus makes sense...
        s = max(s, sqrt(dx*dx + dy*dy) / bonelength[k]);
    }

    auto t3 = Clock::now();
    cout << "scale : " << s << endl;
    cout << "after get scale in "
        << chrono::duration_cast<chrono::nanoseconds>(t3 - t2).count() / 1000000000.
        << endl;

    //calculate projected depth difference (dZ) of each endpoint
    //these aren't bones
    vector<double> depthDiff = vector<double>(bonenames::NUMBONES);
    for (int k = 0; k < bonenames::NUMBONES; k++) {
        double scaledLen = s*bonelength[k];
        Vec3d start = joints2D[bones2D[k].start];
        Vec3d end = joints2D[bones2D[k].end];
        double dx = end[0] - start[0];
        double dy = end[1] - start[1];
        depthDiff[k] = sqrt(scaledLen*scaledLen - dx*dx - dy*dy);
        cout << Pose::bonetoStr((bonenames::bonenames)k) << " " << depthDiff[k] << endl;
    }
   
    auto t4 = Clock::now();
    cout << "after orthographic depth diff in " 
        << chrono::duration_cast<chrono::nanoseconds>(t4 - t3).count() / 1000000000.
        << endl;

    //kd_tree * kd_tree_of_db = new kd_tree(db.descs, 30, pose_distant);//pretty sure since that's squared distance I need to change the search
    //cout << "max_depth: " << kd_tree::max_depth << endl;
    //auto t4_1 = Clock::now();
    //cout << "after kd tree construction in "
    //    << chrono::duration_cast<chrono::nanoseconds>(t4_1 - t4).count() / 1000000000.
    //    << endl;

    Pose finalPose;
    double closest = numeric_limits<double>::infinity();
    //for each possible flipping of the points' 3D coordinates signs
    //a bit-level 0 means negative, a 1 means positive

    //could be parallelised, however the biggest speed up would come from a spatial data strucutre as that's O(n) -> O(log n)
    for (unsigned short boneDepthSign = 0; boneDepthSign < (1 << bonenames::NUMBONES); boneDepthSign++) {
        vector<Vec3d> guessPositions = joints2D;
        vector<Bone> guessBones = bones2D;

        auto ann1 = Clock::now();
        cout << "\tguess generation" << endl;
        for (int k = 0; k < bonenames::NUMBONES; k++) {
            bool sign = bitset<bonenames::NUMBONES>(boneDepthSign)[k];
            //the bonenames are ordered in such a way that this will work fine
            //end.z = (sign)dZ + start.z
            guessPositions[guessBones[k].end][2] = static_cast<float>((sign ? 1 : -1) * depthDiff[k] + guessPositions[guessBones[k].start][2]);
            if ((bonenames::bonenames)k == bonenames::TORSO) {
                guessPositions[guessBones[bonenames::LUPARM].start][2] += static_cast<float>(sign ? 1 : -1) * depthDiff[k];
                guessPositions[guessBones[bonenames::RUPARM].start][2] += static_cast<float>(sign ? 1 : -1) * depthDiff[k];
            }
        }

        Pose guess(guessPositions);
        auto ann2 = Clock::now();
        cout << "\t in "
            << chrono::duration_cast<chrono::nanoseconds>(ann2 - ann1).count() / 1000000000.
            << endl;

        auto ann3 = Clock::now();
        cout << "\tfind ann" << endl;
        double distance = findANN_old(guess, db);//kd_tree_of_db);
        if (distance < closest) {
            finalPose = guess;//should this maybe instead be the pose found in the ANN?
            closest = distance;
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

static cv::Scalar boneToColor(bonenames::bonenames bone) {

    if (bone == bonenames::HEAD)    return Scalar(.95,.5,.5);
    else if (bone == bonenames::TORSO)   return Scalar(.45,.5,.5);
    else if (bone == bonenames::LUPARM)  return Scalar(.85,.5,.5);
    else if (bone == bonenames::LLOARM)  return Scalar(.35,.5,.5);
    else if (bone == bonenames::LUPLEG)  return Scalar(.75,.5,.5);
    else if (bone == bonenames::LLOLEG)  return Scalar(.05,.5,.5);
    else if (bone == bonenames::RUPARM)  return Scalar(.65,.5,.5);
    else if (bone == bonenames::RLOARM)  return Scalar(.25,.5,.5);
    else if (bone == bonenames::RUPLEG)  return Scalar(.55, .5, .5);
    else if (bone == bonenames::RLOLEG)  return Scalar(.15, .5, .5);
    else return Scalar(0,0,0);
}

Mat reproject(Pose solution, Mat camera, Vec2i size) {
    Mat out = Mat::zeros(size[1], size[0], CV_8UC3);
    //draw each bone as a projected line using the camera matrix as the projection
    //undoubtedly need more parameters, I'm just lazy.
    //The resulting mat is a 2D image. Should probably just be the projected space of
    //the pose, and another function should actually draw it.
    
    vector<Vec3d> joints = solution.getJoints();

    for (int j = 0; j < joints.size(); j++) {
        Mat p = camera * Mat(Vec4d(joints[j][0], joints[j][1], joints[j][2], 1));
        joints[j] = Vec3d(p.at<double>(0,0), p.at<double>(1,0), p.at<double>(2,0));
    }

    vector<Bone> bones = solution.getBones();
    Vec2i center(size[0] / 2, size[1] / 2);
    for (int k = 0; k < bones.size(); k++) {
        Point3d start = joints[bones[k].start];
        Point3d end = joints[bones[k].end];

        Point2d orth_start = Point2d(start.x + center[0], size[1]-(start.y + center[1]));
        Point2d orth_end = Point2d(end.x + center[0], size[1]-(end.y + center[1]));
        //cout << start << " " << end << endl;
        //cout << " " << orth_start << " " << orth_end << endl;
        cv::line(out, orth_start, orth_end, boneToColor((bonenames::bonenames)k).mul(Scalar(180,255,255)), 5);
    }

    Mat hsl_out;
    cv::cvtColor(out, hsl_out, CV_HLS2BGR);

    return hsl_out;
}

//the closer to NUMBONES (i.e. the larger) the better.
double pose_similar(Mat poseDescriptor1, Mat poseDescriptor2) {
    return poseDescriptor1.dot(poseDescriptor2);
}

//the closer to 0, the better.
double pose_distant(Mat pd1, Mat pd2) {
    Mat diff = pd1 - pd2;
    return diff.dot(diff); //==||diff||^2
}

//not actually *approximate* yet. I was going to read some locality-sensitive hashing stuff for that.
double findANN(Pose guess, kd_tree * db) {
    Mat guessDesc = guess.getDescriptor();
    pair<cv::Mat, double> result = db->nn_search(guessDesc);
    return result.second;
}


double findANN_old(Pose guess, MotionDB db) {
    double closest = numeric_limits<double>::lowest();
    Mat guessDesc = guess.getDescriptor();

    //while there are motion files
    for (Mat desc : db.descs) {
        //nearest neighbour's distance; jiang's method doesn't care what the real pose looks like.
        //His method just cares that the guessed pose is close to a real pose.
        closest = max(closest, pose_similar(guessDesc, desc));
    }
    return closest;
}

