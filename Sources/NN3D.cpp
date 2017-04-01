#include "NN3D.h"

using namespace std;

Pose filter_CMU(Pose cmuSkele) {
    vector<string> names = cmuSkele.getJointNames();
    vector<float[3]> positions = cmuSkele.getJointPositions();

    //TODO: actually filter to the correct bones.
    Pose stdSkele(names, positions);
    
    return stdSkele;
}
