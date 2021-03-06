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

Pose extract3D_from_Pose_2D(Extractor * e, Pose_2D pose) {
    pair<vector<jointnames::jointnames>, vector<Point2d>> result = Pose_2D_to_labeled_joints(pose);
    return extract3D(e, result.first, result.second);
}

PoseDB create_proj_DB(string path, int increment) {
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
            mp.updatePoseDB(&db, increment);
        }
    }
    timer::stop(2);

    return db;
}

//TODO: also return meta information(like avg bone lengths)/an actual class/struct for the DB
MotionDB createDB(string path, int increment) {
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
            mp.updateMotionDB(&db, increment);
        }
    }
    timer::stop(2);

    return db;
}

//based on jiang
//labels.size() MUST equal points.size()
Extractor * init_3D_extractor(string dbpath, EXTRACT method, int increment) {
    Extractor * e = new Extractor();
    e->method = method;

    vector<double> bonelength;
    if (method == EXTRACT::BY_ITERATIVE_3D) {
        timer::start(1, "create DB");
        e->mdb = createDB(dbpath, increment);
        bonelength = e->mdb.avgBoneLength;
    }
    else if (method == EXTRACT::BY_ITERATIVE_KD) {
        timer::start(1, "create db for kd tree");
        e->mdb = createDB(dbpath, increment);
        bonelength = e->mdb.avgBoneLength;
        timer::stop(1);
        timer::start(1, "create kd tree of db");
        e->kddb = new kd_tree(e->mdb.descs, 30, pose_distant);//pretty sure since that's squared distance I need to change the search
        cout << "max_depth: " << kd_tree::max_depth << endl;
    }
    else {
        timer::start(1, "create projected db");
        e->pdb = create_proj_DB(dbpath, increment);
        bonelength = e->pdb.avgBoneLength;
    }
    timer::stop(1);

    cout << "avg bone lengths:\n";
    for (int i = 0; i < bonelength.size(); i++)
        cout << Pose::bonetoStr((bonenames::bonenames)i) << ": " << bonelength[i] << "\n";

    return e;
}

Pose extract3D(Extractor * e, vector<jointnames::jointnames> labels, vector<Point2d> points) {
    timer::start(1, "create ortho projection of 2D pose");
    //create 2D Pose
    vector<Vec3d> points3D = vector<Vec3d>(points.size());
    for (int i = 0; i < points.size(); i++)
        //the hip should be at 0 depth1
        points3D[i] = Vec3d(points[i].x, points[i].y, 0);

    Pose estimate2D(labels, points3D);
    //estimate2D.print();
    timer::stop(1);

    timer::start(1, "get 2D bones & joints");
    vector<Bone> bones2D = estimate2D.getBones();
    vector<Vec3d> joints2D = estimate2D.getJoints();//different order than points3D
    estimate2D.print();
    timer::stop(1);

    Pose finalPose;
    if (e->method == EXTRACT::BY_ITERATIVE_3D) {
        timer::start(1, "search db (by iterative 3D)");
        finalPose = search_possible_3D(joints2D, bones2D, e->mdb);//motionDB/kd
    }
    else if (e->method == EXTRACT::BY_ITERATIVE_KD) {
        timer::start(1, "search db (by iterative KD)");
        finalPose = search_possible_3D_by_kd(joints2D, bones2D, e->mdb.avgBoneLength, e->kddb);
    }
    else {
        timer::start(1, "search db (by reprojections)");
        finalPose = search_reprojections(joints2D, bones2D, e->pdb);//poseDB
    }
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
    Mat roll_to_dir_inv = (Mat_<double>(3, 3) <<
        x_new[0], x_new[1], x_new[2],
        dir_2d[0], dir_2d[1], dir_2d[2],
        0, 0, 1);
    const int num_cameras = 60;
    vector<Mat> cameras = vector<Mat>(num_cameras);//orthographic
    for (int i = 0; i < num_cameras; i++) {
        double yaw = (360. / static_cast<double>(num_cameras)) * static_cast<double>(i);
        double syaw = sin(yaw);
        double cyaw = cos(yaw);
        Mat r_yaw = (Mat_<double>(3, 3) <<
             cyaw,  0,  syaw,
             0,     1,  0,
            -syaw,  0,  cyaw);
        cameras[i] = roll_to_dir_inv * r_yaw * roll_to_dir * scale;
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
            Vec3d hip = joints[jointnames::HIP];

            //get reprojection error
            double reproj_err = 0;
            for (int k = 0; k < bones.size(); k++) {
                Vec3d diffe(0,0,0), diffs(0,0,0);
                if (!projected[bones[k].start]) {
                    //reproject joint
                    projected[bones[k].start] = true;
                    Mat j_new = cam * Mat(joints[bones[k].start] - hip);
                    joints[bones[k].start] = Vec3d(j_new.at<double>(0, 0), j_new.at<double>(1, 0), j_new.at<double>(2, 0));
                    diffs = (joints2D[bones[k].start] - joints[bones[k].start]);
                }
                if (!projected[bones[k].end]) {
                    //reproject joint
                    projected[bones[k].end] = true;
                    Mat j_new = cam * Mat(joints[bones[k].end] - hip);
                    joints[bones[k].end] = Vec3d(j_new.at<double>(0, 0), j_new.at<double>(1, 0), j_new.at<double>(2, 0));
                    diffe = Vec3d(joints2D[bones[k].end] - joints[bones[k].end]);
                }
                //||diff end||^2 + ||diff start||^2
                reproj_err += diffe.dot(diffe) + diffs.dot(diffs);
            }
                 
            //compare reprojection to Pose_2D(via LS)
            if (least_squares > reproj_err) {
                least_squares = reproj_err;
                bestPose = Pose(joints);
            }
        }
        timer::stop(3);
    }
    timer::stop(2);
    //return allThePoses;
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
        //timer::start(2, "generate a guess");
        vector<Vec3d> guessPositions = joints2D;
        vector<Bone> guessBones = bones2D;
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
        //timer::stop(2);

        //timer::start(2, "find ANN");
        double distance = findANN_old(guess, db);//kd_tree_of_db);
        if (distance < closest) {
            finalPose = guess;//should this maybe instead be the pose found in the ANN?
            closest = distance;
        }
        //timer::stop(2);
    }

    return finalPose;
}

//this is based on jiang's algorithm
Pose search_possible_3D_by_kd(vector<Vec3d> joints2D, vector<Bone> bones2D, vector<double> avgBoneLength, kd_tree * kddb) {
    double scale = get_scale_3D_construct(joints2D, bones2D, avgBoneLength);
    vector<double> depthDiff = get_depthdiff_3D_construct(joints2D, bones2D, scale, avgBoneLength);

    Pose finalPose;
    double closest = numeric_limits<double>::infinity();
    //for each possible flipping of the points' 3D coordinates signs
    //a bit-level 0 means negative, a 1 means positive

    //could be parallelised, however the biggest speed up would come from a spatial data strucutre as that's O(n) -> O(log n)
    for (unsigned short boneDepthSign = 0; boneDepthSign < (1 << bonenames::NUMBONES); boneDepthSign++) {
        //timer::start(2, "generate a guess");
        vector<Vec3d> guessPositions = joints2D;
        vector<Bone> guessBones = bones2D;
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
        //timer::stop(2);

        //timer::start(2, "find ANN");
        double distance = findANN(guess, kddb);
        if (distance < closest) {
            finalPose = guess;//should this maybe instead be the pose found in the ANN?
            closest = distance;
        }
        //timer::stop(2);
    }

    return finalPose;
}
//the closer to NUMBONES (i.e. the larger) the better.
double pose_similar(Mat poseDescriptor1, Mat poseDescriptor2) {
    return poseDescriptor1.dot(poseDescriptor2);
}

//the closer to 0, the better.
double pose_distant(Mat pd1, Mat pd2) {
    pd2 = pd1 - pd2;
    return pd2.dot(pd2); //==||diff||^2
}

//not actually *approximate* yet. I was going to read some locality-sensitive hashing stuff for that.
double findANN(Pose guess, kd_tree * db) {
    Mat guessDesc = guess.getDescriptor();
    pair<cv::Mat, double> result = db->nn_search(guessDesc);
    return result.second;
}

double findANN_end(Pose guess, MotionDB db) {
    double closest = numeric_limits<double>::infinity();
    Mat guessDesc = guess.getEndpointDescriptor();
    for (Mat desc : db.descs) {
        closest = min(closest, pose_distant(guessDesc, desc));//turns out this is ||end1||^2 + ||end2||^2 + ...
    }
    return closest;
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
