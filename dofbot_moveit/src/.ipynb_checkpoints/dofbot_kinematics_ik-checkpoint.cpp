#include "dofbot_kinemarics.h"
#include <iostream>

using namespace KDL;
using namespace std;

Dofbot dofbot = Dofbot();

const float RA2DE = 180.0f / M_PI;

const float DE2RA = M_PI / 180.0f;
const char *urdf_file = "/home/chen/ws/dofbot_ws/src/dofbot_moveit/urdf/dofbot.urdf";

/*

 */
int main(int argc, char **argv) {

    double Roll = -135;
    double Pitch = 0;
    double Yaw = 0;

    double x = 0;
    double y = 5.5;
    double z = 17.375;

    double xyz[]{x, y, z};

    double rpy[]{Roll , Pitch, Yaw };

    vector<double> outjoints;

    vector<double> targetXYZ;

    vector<double> targetRPY;

    for (int k = 0; k < 3; ++k) targetXYZ.push_back(xyz[k] / 100);

    for (int l = 0; l < 3; ++l) targetRPY.push_back(rpy[l] * DE2RA);

    dofbot.dofbot_getIK(urdf_file, targetXYZ, targetRPY, outjoints);

    cout << "---------------" << endl;
    for (int i = 0; i < 5; i++) cout << outjoints.at(i) * RA2DE + 90 << "\t";
    return 0;
}