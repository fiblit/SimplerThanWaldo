# SimplerThanWaldo

## Main Project Goal: 
Human 3D Pose Estimation from Monocular 2D Images

## Contributors
~~Jacquelyn Sloan~~, Kyle Fox, Dalton Hildreth

## How-to-build:
* Install the latest version of [CMake](https://cmake.org/download/) (I have 3.7)
* Install [OpenCV](http://opencv.org/) as appropriate to your system
* Clone the repo
* Try to just use unix: `$ cmake .   $ make`
* If that fails, Open the CMake-Gui
 * Set the sources directory to the repo
 * Set the build directory to a new subdirectory of the repo called Build/ or build/
 * Hit configure and select your compiler
 * Add an entry called "CMAKE_PREFIX_PATH" and set it to the build folder of your installed OpenCV (I'm not sure if you actually need to do this or if my system is just wonky, for me I set it to this: absolutepath..."opencv/build")
 * Hit configure again and check for errors
 * If there are no errors, hit generate (and open the project if you're using Visual Studio...)

## Goals
### Necessary Goals for Core Idea:
#### Notes
Without these our project would not seem complete
#### Task List
* Human Pose Estimation from Monocular
* 3D pose estimation via various methods

### Highly Desired Goals:
#### Notes
Goals which could lead to improved performance or a more interesting project
#### Task List
* Efficient post-processing for real-time
* Stereo or alternative methods for more efficient/viable 3D estimation

### Stretch Goals:
#### Notes
Try not to take these last few too seriously.
#### Task List
* Moving camera setup (attached to robot)
* Robot collision avoidance/threading the needle (don’t get kicked!)
* Iteratively make part of an Autocar’s vision.
 
## Division Of Labor
* Jacqui: Methods of extracting human features from 2-D images & Image pre-processing
* Kyle: Extraction of 2D pose estimates from detected human features.
* Dalton: Using 2D poses and camera information to estimate and reconstruct 3D human poses
