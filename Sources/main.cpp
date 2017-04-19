/**
TODO replace all code with the actual main. ATM this is just a junk file.
**/

#include <opencv2/opencv.hpp>

#include <iostream>
#include <string>
#include <chrono>

//only use this in .cpp or functions
using namespace cv;
using namespace std;

typedef std::chrono::high_resolution_clock Clock;

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

    return (Mat_<double>(3, 4) <<
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
    Mat camera; double yaw, pitch;
};

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
            r->pitch += dy* 3.14159 / 180.;
            r->camera = ortho_cam(r->yaw, r->pitch, 1.0, Vec3d(0, 0, 0));
            r->img = reproject(r->solution, r->camera, Vec2i(800, 600));
            imshow("3D Pose", r->img);
        }
        prev_x = x;
        prev_y = y;
    }
}

int main(int argc, char** argv) {

    //init extract3D
    namespace j = jointnames;
    vector<j::jointnames> labels = {
        j::HIP, j::CLAVICLE, j::HEAD_END, //spine, there is also a HEAD joint, that I may switch HEAD_END to
        j::LHUMERUS, j::LRADIUS, j::LWRIST, //l arm
        j::LFEMUR, j::LTIBIA, j::LFOOT, //l leg
        j::RHUMERUS, j::RRADIUS, j::RWRIST, //r arm
        j::RFEMUR, j::RTIBIA, j::RFOOT //r leg
    };

    double s = 50.; //pretty sure it doesn't matter
    vector<Point2d> points = {
        Point2d( 0*s,  0*s), Point2d( 0*s,  5*s), Point2d( 0*s,  6*s),//spine
        Point2d( 1*s,  5*s), Point2d( 2*s,  3*s), Point2d( 2*s,  0*s),//l arm
        Point2d( 1*s, -1*s), Point2d( 1*s, -3*s), Point2d( 1*s, -5*s),//l leg
        Point2d(-1*s,  5*s), Point2d(-2*s,  3*s), Point2d(-2*s,  0*s),//r arm
        Point2d(-1*s, -1*s), Point2d(-1*s, -3*s), Point2d(-1*s, -5*s)//r leg
    };

    string databasepath = (PROJECT_SOURCE_DIR)+(std::string)"/../csvpose_mini";

    cout << "initialization complete" << endl;

    auto t1 = Clock::now();
    //obtain Pose
    Pose solution = extract3D(labels, points, databasepath);
    auto t2 = Clock::now();

    cout << "solution found in " 
         << chrono::duration_cast<chrono::nanoseconds>(t2 - t1).count()/1000000000.
         << "s" << endl;

    //temporary debug output
    vector<Vec3d> outJoints = solution.getJoints();
    vector<Bone> outBones = solution.getBones();
    for (int k = 0; k < outBones.size(); k++) {
        cout << "start: " << outJoints[outBones[k].start];
        cout << "  end: " << outJoints[outBones[k].end] << endl;
    }

    //raster solution
    double yaw = 0, pitch = 0;
    Vec3d up(0, 1, 0);
    Vec3d T(0, 0, 0);
    Mat virtualCamera = ortho_cam(yaw, pitch, 1.0, T);

    int outW = 800, outH = 600;
    // So, for this step, I'd prefer if we could make it an interactive camera
    Mat out = reproject(solution, virtualCamera, Vec2i(outW, outH));
    namedWindow("3D Pose", WINDOW_AUTOSIZE);
    Results r;
    r.img = out;
    r.solution = solution;
    r.camera = virtualCamera; r.yaw = yaw; r.pitch = pitch;
    setMouseCallback("3D Pose", mouse_callback, &r);

    cout << "\n DONE \n" << flush;
    imshow("3D Pose", out);

    for (;;)
        waitKey(0);
}
