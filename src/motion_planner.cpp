#include "ur5/ur5_motion_library.h"

using namespace std;

int main(int argc,char** argv){
    ROS_INFO("Starting motion planner");
    InverseDifferential(argc,argv);
    return 0;
}