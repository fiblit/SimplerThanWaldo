#include "NN3D.h"
#include "timer.h"

#include <experimental\filesystem>
#include <bitset>

using namespace std;
using namespace cv;
using namespace std::experimental::filesystem;

static pair<vector<jointnames::jointnames>, vector<Point2d>> Pose_2D_to_labeled_joints(Pose_2D p) {
    namespace j = jointnames;
    vector<j::jointnames> labels = {
        j::HIP, j::CLAVICLE, j::HEAD_END, //spine, there is also a HEAD joint, that I may switch HEAD_END to
        j::LHUMERUS, j::LRADIUS, j::LWRIST, //l arm
        j::LFEMUR, j::LTIBIA, j::LFOOT, //l leg
        j::RHUMERUS, j::RRADIUS, j::RWRIST, //r arm
        j::RFEMUR, j::RTIBIA, j::RFOOT //r leg
    };

    vector<Point2d> joints = {
        Point2d(p.C_Hip), Point2d(p.C_Chest), Point2d(p.Head_Top),//spine
        Point2d(p.L_Chest), Point2d(p.L_Elbow), Point2d(p.L_Wrist),//l arm
        Point2d(p.L_Hip), Point2d(p.L_Knee), Point2d(p.L_Ankle),//l leg
        Point2d(p.R_Chest), Point2d(p.R_Elbow), Point2d(p.R_Wrist),//r arm
        Point2d(p.R_Hip), Point2d(p.R_Knee), Point2d(p.R_Ankle)//r leg
    };

    return make_pair(labels, joints);
}

Pose extract3D_from_Pose_2D(Pose_2D pose, std::string databasepath) {
    pair<vector<jointnames::jointnames>, vector<Point2d>> result = Pose_2D_to_labeled_joints(pose);
    return extract3D(result.first, result.second, databasepath);
}

PoseDB create_proj_DB(string path) {
    PoseDB db;
    db.poses = vector<Pose>();
    db.avgBoneLength = vector<double>(bonenames::NUMBONES, 0.0);

    //while there are motion files
    timer::start(2, "i/o v2");
    for (auto &p : recursive_directory_iterator(path)) {
        MotionParser mp;
        //load file into MotionParser
        if (!is_directory(p.path())) {
            mp.open(p.path().string());
            mp.updatePoseDB(&db);
        }
    }
    timer::stop(2);

    return db;
}

//TODO: also return meta information(like avg bone lengths)/an actual class/struct for the DB
MotionDB createDB(string path) {
    MotionDB db;
    db.descs = vector<Mat>();
    db.avgBoneLength = vector<double>(bonenames::NUMBONES, 0.0);

    //while there are motion files
    timer::start(2, "i/o v1");
    for (auto &p : recursive_directory_iterator(path)) {
        MotionParser mp;
        //load file into MotionParser
        if (!is_directory(p.path())) {
            mp.open(p.path().string());
            mp.updateMotionDB(&db);
        }
    }
    timer::stop(2);

    return db;
}

//based on jiang
//labels.size() MUST equal points.size()
Pose extract3D(vector<jointnames::jointnames> labels, vector<Point2d> points, string dbpath) {
    timer::start(1, "create ortho projection of 2D pose");
    //create 2D Pose
    vector<Vec3d> points3D = vector<Vec3d>(points.size());
    for (int i = 0; i < points.size(); i++)
        //the hip should be at 0 depth
        points3D[i] = Vec3d(points[i].x, points[i].y, 0);

    Pose estimate2D(labels, points3D);
    //estimate2D.print();
    timer::stop(1);

    //timer::start(1, "create DB");
    //MotionDB db = createDB(dbpath);
    timer::start(1, "create projected db");
    PoseDB db = create_proj_DB(dbpath);
    timer::stop(1);

    vector<double> bonelength = db.avgBoneLength;
    cout << "avg bone lengths:\n";
    for (int i = 0; i < bonelength.size(); i++)
        cout << Pose::bonetoStr((bonenames::bonenames)i) << ": " << bonelength[i] << "\n";

    timer::start(1, "get 2D bones & joints");
    vector<Bone> bones2D = estimate2D.getBones();
    vector<Vec3d> joints2D = estimate2D.getJoints();//different order than points3D
    estimate2D.print();
    timer::stop(1);

    //timer::start(1, "create kd tree of db");
    //kd_tree * kd_tree_of_db = new kd_tree(db.descs, 30, pose_distant);//pretty sure since that's squared distance I need to change the search
    //cout << "max_depth: " << kd_tree::max_depth << endl;
    //timer::stop(1);

    //timer::start(1, "search db (by guessing 3D)");
    //Pose finalPose = search_possible_3D(joints2D, bones2D, db);//motionDB/kd
    timer::start(1, "search db (by reprojections)");
    Pose finalPose = search_reprojections(joints2D, bones2D, db);//poseDB
    timer::stop(1);

    return finalPose;
}

Pose search_reprojections(std::vector<Vec3d> joints2D, std::vector<Bone> bones2D, PoseDB db) {
    //find the bone whose axis is most near-parallel to the image plane
    //Vec3d parallel_dir_front(0, 0, 0);
    //Vec3d parallel_dir_back(0, 0, 0);
    timer::start(2, "find parallel bone");
    bonenames::bonenames b = bonenames::NIL;
    Vec3d dir_2d(0, 0, 0);//pretend it's in 2d
    double best_parallel = -numeric_limits<double>::infinity();
    for (int k = 0; k < bones2D.size(); k++) {
        Vec3d dir_k = joints2D[bones2D[k].end] - joints2D[bones2D[k].start];
        double l = db.avgBoneLength[k];
           
        double parallel_val = (dir_k[0]*dir_k[0] + dir_k[1]*dir_k[1]) / (l*l);
        if (parallel_val > best_parallel) {
            best_parallel = parallel_val;
            b = (bonenames::bonenames)k;
            dir_2d = dir_k;
            //double depth_diff;
            //parallel_dir_front = ;
            //parallel_dir_back = ;
        }
    }
    timer::stop(2);

    //generate cameras circling around the bone's axis most near-parallel to the image plane
    timer::start(2, "generate cameras");
    double dir_len = norm(dir_2d);
    double s = dir_len / db.avgBoneLength[b];
    Mat scale = (Mat_<double>(3, 3) <<
        s, 0, 0,
        0, s, 0,
        0, 0, s);
    dir_2d /= dir_len;
    Vec3d x_new = dir_2d.cross(Vec3d(0,0,1));//right = up x back
    Mat roll_to_dir = (Mat_<double>(3, 3) <<
        x_new[0], dir_2d[0], 0,
        x_new[1], dir_2d[1], 0,
        x_new[2], dir_2d[2], 1);
    const int num_cameras = 10;
    vector<Mat> cameras = vector<Mat>(num_cameras);//orthographic
    for (int i = 0; i < num_cameras; i++) {
        double yaw = (360. / static_cast<double>(num_cameras)) * static_cast<double>(i);
        double syaw = sin(yaw);
        double cyaw = cos(yaw);
        Mat r_yaw = (Mat_<double>(3, 3) <<
             cyaw,  0,  syaw,
             0,     1,  0,
            -syaw,  0,  cyaw);
        cameras[i] = r_yaw * roll_to_dir * scale;
    }
    timer::stop(2);

    timer::start(2, "reprojection of db");
    double least_squares = numeric_limits<double>::infinity();
    Pose bestPose;
    //for (Pose pose : db.poses) {
    for (Mat cam : cameras) {
        timer::start(3, "reprojecting db poses");
        //for (Mat cam : cameras) {
        for (Pose pose : db.poses) {
            vector<Vec3d> joints = pose.getJoints();//TODO: there are more joints here than I actually need I think...
            vector<Bone> bones = pose.getBones();
            vector<bool> projected(joints.size(), false);

            //get reprojection error
            double reproj_err = 0;
            for (int k = 0; k < bones.size(); k++) {
                Vec3d diffe(0,0,0), diffs(0,0,0);
                if (!projected[bones[k].end]) {
                    //reproject joint
                    projected[bones[k].end] = true;
                    Mat j_new = cam * Mat(joints[bones[k].end] - joints[jointnames::HIP]);
                    joints[bones[k].end] = Vec3d(j_new.at<double>(0, 0), j_new.at<double>(1, 0), j_new.at<double>(2, 0));
                    diffe = Vec3d(joints2D[bones[k].end] - joints[bones[k].end]);
                }
                if (!projected[bones[k].start]) {
                    //reproject joint
                    projected[bones[k].start] = true;
                    Mat j_new = cam * Mat(joints[bones[k].start] - joints[jointnames::HIP]);
                    joints[bones[k].start] = Vec3d(j_new.at<double>(0, 0), j_new.at<double>(1, 0), j_new.at<double>(2, 0));
                    diffs = (joints2D[bones[k].start] - joints[bones[k].start]);
                }
                reproj_err += diffe.dot(diffe) + diffs.dot(diffs);//||diff end||^2 + ||diff start||^2
            }
                 
            //compare reprojection to Pose_2D(via LS)
            if (least_squares > reproj_err) {
                least_squares = reproj_err;
                bestPose = pose;
            }
        }
        timer::stop(3);
    }
    timer::stop(2);
    return bestPose;
}

double get_scale_3D_construct(std::vector<Vec3d> joints2D, std::vector<Bone> bones2D, std::vector<double> avgBoneLength) {
    double s = 1;
    for (int k = 0; k < bonenames::NUMBONES; k++) {
        Vec3d start = joints2D[bones2D[k].start];
        Vec3d end = joints2D[bones2D[k].end];
        double dx = end[0] - start[0];
        double dy = end[1] - start[1];
        s = max(s, sqrt(dx*dx + dy*dy) / avgBoneLength[k]);
    }
    return s;
}

std::vector<double> get_depthdiff_3D_construct(std::vector<Vec3d> joints2D, std::vector<Bone> bones2D, double scale, std::vector<double> avgBoneLength) {
    //calculate projected depth difference (dZ) of each endpoint
    //these aren't bones
    vector<double> depthDiff = vector<double>(bonenames::NUMBONES);
    cout << "depth diffs" << endl;
    for (int k = 0; k < bonenames::NUMBONES; k++) {
        double scaledLen = scale*avgBoneLength[k];
        Vec3d start = joints2D[bones2D[k].start];
        Vec3d end = joints2D[bones2D[k].end];
        double dx = end[0] - start[0];
        double dy = end[1] - start[1];
        depthDiff[k] = sqrt(scaledLen*scaledLen - dx*dx - dy*dy);
       cout << Pose::bonetoStr((bonenames::bonenames)k) << " " << depthDiff[k] << endl;
    }
    return depthDiff;
}

//this is based on jiang's algorithm
Pose search_possible_3D(vector<Vec3d> joints2D, vector<Bone> bones2D, MotionDB db) {
    double scale = get_scale_3D_construct(joints2D, bones2D, db.avgBoneLength);
    vector<double> depthDiff = get_depthdiff_3D_construct(joints2D, bones2D, scale, db.avgBoneLength);

    Pose finalPose;
    double closest = numeric_limits<double>::infinity();
    //for each possible flipping of the points' 3D coordinates signs
    //a bit-level 0 means negative, a 1 means positive

    //could be parallelised, however the biggest speed up would come from a spatial data strucutre as that's O(n) -> O(log n)
    for (unsigned short boneDepthSign = 0; boneDepthSign < (1 << bonenames::NUMBONES); boneDepthSign++) {
        vector<Vec3d> guessPositions = joints2D;
        vector<Bone> guessBones = bones2D;

        timer::start(2, "generate a guess");
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
        timer::stop(2);

        timer::start(2, "find ANN");
        double distance = findANN_old(guess, db);//kd_tree_of_db);
        if (distance < closest) {
            finalPose = guess;//should this maybe instead be the pose found in the ANN?
            closest = distance;
        }
        timer::stop(2);
    }

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
