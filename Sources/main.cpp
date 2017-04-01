/**
TODO replace all code with the actual main. ATM this is just a junk file.
**/

#include <opencv2/opencv.hpp>

#include <iostream>
#include <string>

//only use this in .cpp or functions
using namespace cv;
using namespace std;

int main(int argc, char** argv) {
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
