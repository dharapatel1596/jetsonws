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

//    double joints[]{90, 90, 90, 90, 90};
    double joints[]{90, 135, 0, 0, 90};

    vector<double> initjoints;

    vector<double> initpos;

    for (int i = 0; i < 5; ++i) initjoints.push_back((joints[i] - 90) * DE2RA);

    dofbot.dofbot_getFK(urdf_file, initjoints, initpos);
    cout <<fixed<< "FK kinematics result : " << endl;
    cout << "X coordinates(cm)： " << initpos.at(0) * 100 << "\t"
         << "Y coordinates (cm)： " << initpos.at(1) * 100 << "\t"
         << "Z coordinates (cm)： " << initpos.at(2) * 100 << endl;
    cout << "Roll  (°)： " << initpos.at(3) * RA2DE << "\t"
         << "Pitch (°)： " << initpos.at(4) * RA2DE << "\t"
         << "Yaw   (°)： " << initpos.at(5) * RA2DE << endl;
    return 0;
}