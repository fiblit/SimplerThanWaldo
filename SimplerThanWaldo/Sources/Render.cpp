#include "Render.h"
#include "timer.h"

#include <opencv2/opencv.hpp>

#include <iostream>
#include <string>

//only use this in .cpp or functions
using namespace cv;
using namespace std;

#include "NN3D.h"

Mat ortho_cam(double yaw, double pitch, double scale, Vec3d trans) {
    double cos_yaw = cos(yaw);
    double sin_yaw = sin(yaw);
    double cos_pitch = cos(pitch);
    double sin_pitch = sin(pitch);
    Mat r_y_yaw = (Mat_<double>(3, 3) <<
        cos_yaw, 0, sin_yaw,
        0, 1, 0,
        -sin_yaw, 0, cos_yaw);
    Mat r_x_pitch = (Mat_<double>(3, 3) <<
        1, 0, 0,
        0, cos_pitch, -sin_pitch,
        0, sin_pitch, cos_pitch);

    Mat r = r_x_pitch * r_y_yaw;

    return scale*(Mat_<double>(3, 4) <<
        r.at<double>(0, 0), r.at<double>(0, 1), r.at<double>(0, 2), trans[0],
        r.at<double>(1, 0), r.at<double>(1, 1), r.at<double>(1, 2), trans[1],
        r.at<double>(2, 0), r.at<double>(2, 1), r.at<double>(2, 2), trans[2]);
}

Mat ortho_cam(Vec3d look, Vec3d up, double scale, Vec3d trans) {
    Vec3d x_new = look.cross(up);
    Vec3d y_new = x_new.cross(look);
    Vec3d z_new = x_new.cross(y_new);

    Mat cam = (Mat_<double>(3, 4) <<
        x_new[0], y_new[0], z_new[0], trans[0],
        x_new[1], y_new[1], z_new[1], trans[1],
        x_new[2], y_new[2], z_new[2], trans[2]);
    cam = scale * cam;

    return cam;
}

struct Results {
    Mat img;
    Pose solution;
    Pose original;
    Mat camera; double yaw, pitch;
};

static cv::Scalar boneToColor(bonenames::bonenames bone) {
    if (bone == bonenames::HEAD)    return Scalar(.95, .5, .5);
    else if (bone == bonenames::TORSO)   return Scalar(.45, .5, .5);
    else if (bone == bonenames::LUPARM)  return Scalar(.85, .5, .5);
    else if (bone == bonenames::LLOARM)  return Scalar(.35, .5, .5);
    else if (bone == bonenames::LUPLEG)  return Scalar(.75, .5, .5);
    else if (bone == bonenames::LLOLEG)  return Scalar(.05, .5, .5);
    else if (bone == bonenames::RUPARM)  return Scalar(.65, .5, .5);
    else if (bone == bonenames::RLOARM)  return Scalar(.25, .5, .5);
    else if (bone == bonenames::RUPLEG)  return Scalar(.55, .5, .5);
    else if (bone == bonenames::RLOLEG)  return Scalar(.15, .5, .5);
    else return Scalar(1, .5, .5);
}

Mat reproject(Pose solution, Pose original, Mat camera, Vec2i size) {
    Mat out = Mat::zeros(size[1], size[0], CV_8UC3);
    //draw each bone as a projected line using the camera matrix as the projection
    //undoubtedly need more parameters, I'm just lazy.
    //The resulting mat is a 2D image. Should probably just be the projected space of
    //the pose, and another function should actually draw it.

    vector<Vec3d> o_j = original.getJoints();
    vector<Vec3d> joints = solution.getJoints();
    vector<Vec3d> planar = solution.getJoints();

    for (int j = 0; j < joints.size(); j++) {
        planar[j] = Vec3d(0, joints[j][0], joints[j][2]);

        Mat p = camera * Mat(Vec4d(joints[j][0], joints[j][1], joints[j][2], 1));
        joints[j] = Vec3d(p.at<double>(0, 0), p.at<double>(1, 0), p.at<double>(2, 0));
        p = camera * Mat(Vec4d(planar[j][0], planar[j][1], planar[j][2], 1));
        planar[j] = Vec3d(p.at<double>(0, 0), p.at<double>(1, 0), p.at<double>(2, 0));
        p = camera * Mat(Vec4d(o_j[j][0], o_j[j][1], o_j[j][2], 1));
        o_j[j] = Vec3d(p.at<double>(0, 0), p.at<double>(1, 0), p.at<double>(2, 0));
    }

    vector<Bone> bones = solution.getBones();
    bones.push_back(Bone(jointnames::HIP, jointnames::LFEMUR));
    bones.push_back(Bone(jointnames::HIP, jointnames::RFEMUR));
    bones.push_back(Bone(jointnames::CLAVICLE, jointnames::RHUMERUS));
    bones.push_back(Bone(jointnames::CLAVICLE, jointnames::LHUMERUS));
    Vec2i center(size[0] / 2, size[1] / 2);
    for (int k = 0; k < bones.size(); k++) {
        Point3d start = joints[bones[k].start];
        Point3d end = joints[bones[k].end];
        Point3d start_o = solution.getJoints()[bones[k].start];
        Point3d end_o = solution.getJoints()[bones[k].end];
        Point3d s_o = original.getJoints()[bones[k].start];
        Point3d e_o = original.getJoints()[bones[k].end];
        Point3d r_s_o = o_j[bones[k].start];
        Point3d r_e_o = o_j[bones[k].end];
        Point3d pl_s = planar[bones[k].start];
        Point3d pl_e = planar[bones[k].end];

        Point2d orth_start = Point2d(start.x + center[0], size[1] - (start.y + center[1]));
        Point2d orth_end = Point2d(end.x + center[0], size[1] - (end.y + center[1]));
        Point2d orth_start_o = Point2d(start_o.x + center[0], size[1] - (start_o.y + center[1]));
        Point2d orth_end_o = Point2d(end_o.x + center[0], size[1] - (end_o.y + center[1]));
        Point2d orth_s_o = Point2d(s_o.x + center[0], size[1] - (s_o.y + center[1]));
        Point2d orth_e_o = Point2d(e_o.x + center[0], size[1] - (e_o.y + center[1]));
        Point2d orth_r_s_o = Point2d(r_s_o.x + center[0], size[1] - (r_s_o.y + center[1]));
        Point2d orth_r_e_o = Point2d(r_e_o.x + center[0], size[1] - (r_e_o.y + center[1]));
        Point2d orth_pl_s = Point2d(pl_s.x + center[0], size[1] - (pl_s.y + center[1]));
        Point2d orth_pl_e = Point2d(pl_e.x + center[0], size[1] - (pl_s.y + center[1]));

        cv::line(out, orth_r_s_o, orth_r_e_o, boneToColor((bonenames::bonenames)k).mul(Scalar(180, 255, 255)), 1);
        cv::line(out, orth_s_o, orth_e_o, boneToColor((bonenames::bonenames)k).mul(Scalar(180, 255, 255)), 1);
        cv::line(out, orth_start, orth_end, boneToColor((bonenames::bonenames)k).mul(Scalar(180, 255, 255)), 5);
        //cv::line(out, orth_start_o, orth_end_o, boneToColor((bonenames::bonenames)k).mul(Scalar(180, 255, 255)), 3);
        cv::line (out, orth_pl_s, orth_pl_e, boneToColor((bonenames::bonenames)k).mul(Scalar(180, 255, 255)), 5);
    }

    Mat p = camera * Mat(Vec4d(joints[jointnames::HIP][0], joints[jointnames::HIP][1], joints[jointnames::HIP][2], 1));
    Point2d axis_forward_s(p.at<double>(0,0) + center[0], size[1] - (p.at<double>(1,0) + center[1]));
    double dist_from_hip_to_femur = norm(joints[jointnames::LFEMUR] - joints[jointnames::HIP]);
    p = camera * Mat(Vec4d(joints[jointnames::HIP][0], joints[jointnames::HIP][1], joints[jointnames::HIP][2] - dist_from_hip_to_femur, 1));
    Point2d axis_forward_e(p.at<double>(0, 0) + center[0], size[1] - (p.at<double>(1, 0) + center[1]));
    cv::arrowedLine(out, axis_forward_s, axis_forward_e, Scalar(.5, .5, .5).mul(Scalar(180, 255, 255)), 3, 8, 0, 0.3);

    Mat hsl_out;
    cv::cvtColor(out, hsl_out, CV_HLS2BGR);

    return hsl_out;
}

void mouse_callback(int event, int x, int y, int flags, void * userdata) {
    static int prev_x = -1;
    static int prev_y = -1;
    static bool down = false;

    Results * r = static_cast<Results *>(userdata);

    if (event == EVENT_LBUTTONDOWN)
        down = true;
    else if (event == EVENT_LBUTTONUP)
        down = false;
    else if (event == EVENT_MOUSEMOVE) {
        if (down) {
            int dx = x - prev_x;
            int dy = y - prev_y;
            r->yaw += dx * 3.14159 / 180.;
            r->pitch += dy * 3.14159 / 180.;
            r->camera = ortho_cam(r->yaw, r->pitch, 1.0, Vec3d(0, 0, 0));
            r->img = reproject(r->solution, r->original, r->camera, Vec2i(800, 600));
            imshow("3D Pose", r->img);
        }
        prev_x = x;
        prev_y = y;
    }
}

Extractor * initialize_parameters() {
    timer::init_layers(10);//layers 0 through 9 allowed
    string databasepath = (PROJECT_SOURCE_DIR)+(std::string)"/../csvpose_mini";
    cout << "parameter initialization complete" << endl;

    Extractor * e = init_3D_extractor(databasepath, EXTRACT::BY_ITERATIVE_KD, 1);

    return e;
}

int main(int argc, char** argv) {

    Extractor * e = initialize_parameters();

    //init extract3D
    namespace j = jointnames;
    vector<j::jointnames> labels = {
        j::HIP, j::CLAVICLE, j::HEAD_END, //spine, there is also a HEAD joint, that I may switch HEAD_END to
        j::LHUMERUS, j::LRADIUS, j::LWRIST, //l arm
        j::LFEMUR, j::LTIBIA, j::LFOOT, //l leg
        j::RHUMERUS, j::RRADIUS, j::RWRIST, //r arm
        j::RFEMUR, j::RTIBIA, j::RFOOT //r leg
    };

    //double s = 25.; //pretty sure it doesn't matter
    vector<Point2d> points = {
        Point2d( 0,  0), Point2d( 19,  133), Point2d( 28,  247),//spine
        Point2d( 94,  154), Point2d( 91,  45), Point2d( 101,  -8),//l arm
        Point2d( 32, -37), Point2d( 95, -175), Point2d( 34, -207),//l leg
        Point2d(-44,  167), Point2d(-54,  54), Point2d(-108,  2),//r arm
        Point2d(-26, -35), Point2d(12, -157), Point2d(-10, -247)//r leg
    };
    int xt = 0; int yt = 0;
    for (int i = 0; i < points.size(); i++) {
        points[i].x += xt;
        points[i].y += yt;
    }

    vector<Vec3d> points3D = vector<Vec3d>(points.size());
    for (int i = 0; i < points.size(); i++)
        //the hip should be at 0 depth
        points3D[i] = Vec3d(points[i].x, points[i].y, 0);
    Pose estimate2D(labels, points3D);

    timer::start(0, "find solution");
    //obtain Pose
    Pose solution = extract3D(e, labels, points);//this is just for me to hard code
    //Pose solution = extract3D_from_Pose_2D(e, pose_2d); //you will use this
    delete e;

    vector<Vec3d> joints = solution.getJoints();
    for (Vec3d j : joints)
        j -= joints[jointnames::HIP];
    solution = Pose(joints);
    timer::stop(0);

    cout << "----------------\n";
    cout << "solution:\n";
    solution.print();

    cout << "\n DONE \n" << flush;

    /*** INIT RASTER ***/
    //raster solution
    double yaw = 0, pitch = 0;
    Vec3d up(0, 1, 0);
    Vec3d T(0, 0, 0);
    Mat virtualCamera = ortho_cam(yaw, pitch, 1.0, T);

    int outW = 800, outH = 600;
    // So, for this step, I'd prefer if we could make it an interactive camera
    Mat out = reproject(solution, estimate2D, virtualCamera, Vec2i(outW, outH));
    namedWindow("3D Pose", WINDOW_AUTOSIZE);
    Results r;
    r.img = out;
    r.solution = solution;
    r.original = estimate2D;
    r.camera = virtualCamera; r.yaw = yaw; r.pitch = pitch;
    setMouseCallback("3D Pose", mouse_callback, &r);

    imshow("3D Pose", out);
    /*** END INIT RASTER ***/

    //no accidental quitting allowed
    for (;;)
        waitKey(0);
}
