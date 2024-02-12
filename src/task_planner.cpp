#include "ur5/ur5_task_library.h"

int main(int argc, char** argv){
    ros::init(argc, argv, "task_planner");
    ros::NodeHandle n;

    //initialize convertion matrix
    WORLD_TO_ROBOT << 1, 0, 0, WORLD_TO_ROBOT_X,
                        0, -1, 0, WORLD_TO_ROBOT_Y,
                        0, 0, -1, WORLD_TO_ROBOT_Z,
                        0, 0, 0, 1;


    //set initial state and number of blocks to be moved
    state=start;
    // n_classes=6;///< set the number of blocks that are positioned on the workspace

   

    ROS_INFO("Starting task planner");

    while(next_state!=no_more_blocks && ros::ok()){
        stateHandler(state,n); 
    }
    
    ros::Duration(1).sleep();
    ROS_INFO("Terminating task planner");

    return 0;
}