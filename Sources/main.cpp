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

    string databasepath = (PROJECT_SOURCE_DIR) + (std::string)"/../csvpose_mini";

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
    Mat virtualCamera;
    int outW = 800, outH = 600;
    // So, for this step, I'd prefer if we could make it an interactive camera
    Mat out = reproject(solution, virtualCamera, outW, outH);
    namedWindow("3D Pose", WINDOW_AUTOSIZE);
    imshow("3D Pose", out);

    cout << "\n DONE \n" << flush;
    for(;;)
        //because waiting 2 hours and then having the window close FUCKING SUCKS
        waitKey(0);

}

int test(int argc, char** argv) {
    //String imageName("../data/HappyFish.jpg"); // by default
    if (argc != 2) {
        cout << "Usage: " << argv[0] << " <imagename>" << endl;
        exit(EXIT_SUCCESS);
    }
    String imageName = argv[1];

    Mat image;
    // Read the file
    image = imread(imageName, IMREAD_COLOR);

    // Check for invalid input
    if (image.empty()) {
        cout << "Could not open or find the image" << endl;
        return -1;
    }

    Mat gray_image;
    cvtColor(image, gray_image, COLOR_BGR2GRAY);

    imwrite((std::string)PROJECT_SOURCE_DIR + "/Gray_Image.jpg", gray_image);

    // Create a window for display.
    namedWindow(imageName, WINDOW_AUTOSIZE);
    namedWindow("Gray image", WINDOW_AUTOSIZE);

    // Show our image inside it.
    imshow(imageName, image);
    imshow("Gray image", gray_image);

    // Wait for a keystroke in the window
    waitKey(0);

    return 0;
}
