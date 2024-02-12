/**
 * the file contains all the functions
 * that are used in order to compute the trajectory of the robot
 * and also make some checks in order to avoid singularities and go outside
 * the designed workin area.
 */

#ifndef UR5_INVDIFF_LIBRARY_H
#define UR5_INVDIFF_LIBRARY_H

#include "ros/ros.h"
#include "std_msgs/Float64MultiArray.h"
#include "sensor_msgs/JointState.h"
#include "ur5/ServiceMessage.h"    
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <algorithm>
#include <iterator>
#include <sstream>
#include <string.h>
#include <math.h>
#include <list>
#include <iostream>
#define INTERMEDIATE_POINT 1///< 0 to use damping factor, 1 to use intermediate point

using namespace std;
using namespace Eigen;

class InverseDifferential   
{
    const double Tf=10.0;
    const double Tb=0;
    const double deltaT=0.1;//the smaller, the more precise but more computation
    

    const std::string TOPIC = std::string("/ur5/joint_group_pos_controller/command");///< the topic to send the new joints configurations
    const std::string TOPIC_SUB = std::string("/ur5/joint_states");///< the topic to read the joints states

    

    const static int JOINT_NAMES = 6;
    const double SCALAR_FACTOR = 1.0;
    const double DAMPING_FACTOR = 0.095;///< the damping value used in damped pseudoinverse matrix
    const double ALMOST_ZERO = 1e-7;///< threshold when a values is recognized as zero
    const int RATE = 700;//set to 100 to slow down

    //global values
    Quaterniond q0,qf;
    Vector3d xe0,xef,phie0,phief; //xe_intermediate,phie_intermediate;
    Vector2d gripper;
    VectorXd q, q_des;
    VectorXd A, D, ALPHA;

    ros::Publisher joint_pub;///< publisher node to send the new joints' values
    ros::Subscriber sub;///< subscriber node to read the joints' values
    std::string joint_names [JOINT_NAMES];///< contains the name of all the joints

    char **argv;
    int argc;

    //ack is used to syncronize the service 
    
    int ack=1;
    int error=0;//no error at start
    int final_end=0;//different from 0 if there are no other blocks
    int counter;

    /**
     * define the workin area of the manipulator
     * w.r.t the robot frame
    */
    const double MAX_X = 0.5;
    const double MIN_X = -0.5;
    const double MAX_Y = 0.12;
    const double MIN_Y = -0.45;
    const double MAX_Z = 0.733;
    const double MIN_Z = 0.15;

    public:
        /**
         * the constructor initializes all the varaibles
         * and starts the motion node
         * @param argc_ the argc of the main 
         * @param argv_ the argv of the main
        */
        InverseDifferential(int argc_, char** argv_);

        //ROS functions

        /**
         * a service that receives the pose in which the robot must arrive
         * and responds if the position has been reached or not
         * 
         * @param req the request message
         * @param res the response that the service will generate
         * 
         * @return true if the motion has been done without problems, false otherwise
         */
        bool motionPlannerToTaskPlannerServiceResponse(ur5::ServiceMessage::Request &req,ur5::ServiceMessage::Response &res);

        /**
         * function that given the joint's configuration, sends it through the topic
         * 
         * @param joint_pub the publisher node that is linked to the topic
         * @param q_des the joint's configuration
         */
        void send_des_jstate(ros::Publisher joint_pub, Eigen::VectorXd q_des);

        /**
         * read the joints values from the topic
         * 
         * @param msg the message received from the topic
         */
        void receive_jstate(const sensor_msgs::JointState::ConstPtr &msg);

        /**
         * main function,
         * initialize nodes, reading and sending the new joint states
         */
        int talker();

        /**
         * function called from the talker.
         * given the initial and final pose, compute the trajectory 
         * trying to stay away from singularities
         * 
         * @return true if no problems occured, false otherwise
         */
        bool invDiff();


        //Convertion functions

        /**
         * convert the euler angles in a rotation matrix
         * using the XYZ convention
         * 
         * @param euler vector containing the euler angles in XYZ order
         * 
         * @return the corresponding rotation matrix
         */
        Eigen::Matrix3d eulerAnglesToRotationMatrix(const Eigen::Vector3d &euler);

        /**
         * given the rotation matrix, return the euler angles in XYZ convention
         * 
         * @param rotationMatrix the rotation matrix
         * 
         * @return the vector containing the euler angles in XYZ order
         */
        Eigen::Vector3d rotationMatrixToEulerAngles(const Eigen::Matrix3d &rotationMatrix);

        /**
         * convert the euler angles in XYZ convention in a quaternion
         * 
         * @param euler vector containing the euler angles in XYZ order
         * 
         * @return the corresponding quaternion
         */
        Eigen::Quaterniond eulerAnglesToQuaternion(const Eigen::Vector3d &euler);

        /**
         * convert the rotation matrix to the corresponding quaternion
         * 
         * @param rotationMatrix the rotation matrix
         * 
         * @return the corresponding quaternion
        */
        Eigen::Quaterniond rotationMatrixToQuaternion(const Eigen::Matrix3d &rotationMatrix);


        //Inverse differential functions

        /**
         * given the joints values, return the end effector position
         * and the rotation matrix
         * 
         * @param xe vector containing the end effector position
         * @param Re matrix corresponding to the orientation of end effector
         * 
         * @param q_des the joint values used to calculate the corresponing pose
         */
        void ur5Direct(Eigen::Vector3d &xe, Eigen::Matrix3d &Re, const Eigen::VectorXd q_des);

        /**
         * given the joints values, returns the computed jacobian for the ur5
         * 
         * @param v vector containing the joint's values
         */
        Eigen::MatrixXd ur5Jac(Eigen::VectorXd v);

        /**
         * first of the two functions that generates the trajectory for the robot
         * 
         * @param TH0 the initial joint configuration
         * @param T vector containing the time intervals
         * @param Kp correction matrix for the velocity component
         * @param Kphi correction matrix for the orientation component
         * 
         * @return a list of joint configuration for each time interval
         */
        list<Eigen::VectorXd> invDiffKinematic(Eigen::VectorXd TH0,Eigen::VectorXd T,Eigen::Matrix3d Kp,Eigen::Matrix3d Kphi);

        /**
         * second of the two functions that generates the trajectory for the robot
         * implements the control for possible singularity using the damped pseudo inverse matrix
         * 
         * @param qk the joint configuration
         * @param xe the position given from direct kinematic
         * @param xd the desired velocity at a specific time
         * @param vd the difference of velocity from the actual and the previous instance of time
         * @param omegad the difference of orientation from the actual and the previous instance of time
         * @param qe the rotation matrix from direct kinematic converted in a quaternion
         * @param qd the desired quaternion at a specific time
         * @param Kp correction matrix for the velocity component
         * @param Kphi correction mateix for the orientation component
         * 
         * @return the joint velocities
         */
        Eigen::VectorXd invDiffKinematicControl(Eigen::VectorXd qk, Eigen::Vector3d xe, Eigen::Vector3d xd, Eigen::Vector3d vd, Eigen::Vector3d omegad, Eigen::Quaterniond qe, Eigen::Quaterniond qd, Eigen::Matrix3d Kp, Eigen::Matrix3d Kphi);

        /**
         * position as a function over time using linear interpolation between 2 points
         * 
         * @param ti time instance
         * 
         * @return xyz position in a vector
         */
        Eigen::Vector3d xd(double ti);

        /**
         * implementation of SLERP
         * 
         * @param ti time instance
         * 
         * @return quaternion corresponding to a rotation
         */
        Eigen::Quaterniond qd(double ti);

        /**
         * given position and orientation, returns the possible joints' configurations
         * 
         * @param p60 the desired position
         * @param Re the desired rotation 
         * 
         * @return a matrix containing the solutions, saving them in columns order
         */
        Eigen::MatrixXd ur5Inverse(Eigen::Vector3d &p60, Eigen::Matrix3d &Re);

        /**
         * compute the homogeneous matrix for the computation of the direct kinematic
         * 
         * @param th rotation about z axis
         * @param alpha rotation about x axis
         * @param d translation about z axis
         * @param a translation about x axis
         * 
         * @return the homogeneous matrix
         */
        Eigen::Matrix4d getRotationMatrix(double th, double alpha, double d, double a);

        /**
         * remove all the NON possible solutions from the inverse problem
         * checking if each columns contains at least a NaN value
         * 
         * @param matrix the returned matrix from the ur5Inverse function
         * 
         * @return a possible resized matrix which doesn't contain non valid solutions
         */
        Eigen::MatrixXd purgeNanColumn(Eigen::MatrixXd matrix);


        //Other functions

        /**
         * function that controls if the end effector is inside the working area
         * 
         * @param position the xyz coordinates
         * 
         * @return true if inside the working area, false otherwise
         */
        bool checkWorkArea(const Eigen::Vector3d &position);

        /**
         * function that controls if the given joint configuration, the end
         * effector in inside the working area
         * 
         * @param joints the current joint values of the manipulator
         * 
         * @return true if inside the working area, false otherwise
         */
        bool checkWorkArea(const Eigen::VectorXd &joints);

        /**
         * function that checks if a value is close to zero
         * 
         * @param value
         * 
         * @return true if the values is close to zero, false otherwise
         */
        bool almostZero(double value);

        /**
         * function that checks if the angle is close to zero.
         * if so, set the value to zero
         * 
         * @param angle 
         */
        void angleCorrection(double &angle);

        /**
         * the wirst's joints can reach its limits, given the desired configuration
         * bring back in the valid range the joints values in the range [-2pi;2pi]
         * 
         * @param joints the complete joint configuration
         */
        void fixWirstJointLimits(Eigen::VectorXd &joints);
};

#endif