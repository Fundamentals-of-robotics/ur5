/**
 * the file contains the implementation of all functions
 * defined in the ur5_invDiff_library.h file
 */

#include "ur5/ur5_motion_library.h"

InverseDifferential::InverseDifferential(int argc_, char** argv_) :
joint_names{"shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"}
{
    //just copy the argc and argv of main to initialize the nodes in talker function
    argc = argc_;
    argv = argv_;

    //initialize all the vector
    A.resize(JOINT_NAMES);
    D.resize(JOINT_NAMES);
    ALPHA.resize(JOINT_NAMES);
    q.resize(JOINT_NAMES);
    q_des.resize(JOINT_NAMES);

    //set values of the D-H parameters
    //A - D -> distance between joints
    //ALPHA -> rotation about x
    A << 0, -0.425, -0.3922, 0, 0, 0;///!<the D-H parameter
    D << 0.1625, 0, 0, 0.1333, 0.0997, 0.0996;///< the D-H parameter
    ALPHA << M_PI/2, 0, 0, M_PI/2, -M_PI/2, 0;///< the D-H parameter
    A *= SCALAR_FACTOR;
    D *= SCALAR_FACTOR;
    
    

    talker();    
}


//ROS functions
bool InverseDifferential::motionPlannerToTaskPlannerServiceResponse(ur5::ServiceMessage::Request &req, ur5::ServiceMessage::Response &res){
    error=0;
    final_end=req.end;
    ack=req.ack;

    xef << req.xef1, req.xef2, req.xef3;
    phief << req.phief1, req.phief2, req.phief3;
    gripper << req.gripper, req.gripper;

    ROS_INFO("Request received");


    if(!checkWorkArea(xef)){
        ROS_INFO("The request is not inside the working area"); 
        res.error = 1;//block unreachable 
        return true;
    }

    //if everything ok, calculate trajectory
    //and give back feedback
    bool result = invDiff();
    res.error= result ? 0 : 2;//0 - ok; 2 - error in motion

    if(result) ROS_INFO("Motion completed");
    else ROS_INFO("Error in motion/Near singularity");

    if(final_end != 0) {
        ROS_INFO("Terminating motion planner");
        ros::shutdown();
    }
    return true;
}

void InverseDifferential::send_des_jstate(ros::Publisher joint_pub, VectorXd q_des){
    std_msgs::Float64MultiArray msg;

    q_des.conservativeResize(q_des.size() + 2);
    q_des(q_des.size()-2) = gripper[0];
    q_des(q_des.size()-1) = gripper[1];

    std::vector<double> v3(&q_des[0], q_des.data()+q_des.cols()*q_des.rows());
    msg.data = v3;

    joint_pub.publish(msg);
}

void InverseDifferential::receive_jstate(const sensor_msgs::JointState::ConstPtr &msg){
    for(int msg_idx=0; msg_idx<msg->name.size(); ++msg_idx){
        for(int joint_idx=0; joint_idx<JOINT_NAMES; ++joint_idx){
            if(joint_names[joint_idx].compare(msg->name[msg_idx]) == 0){
                q[joint_idx] = msg->position[msg_idx];
            }
        }
    }
}

int InverseDifferential::talker(){
    ros::init(argc, argv, "motion_planner");
    ros::NodeHandle n;
    
    while(final_end == 0 && ros::ok()){
        
        ros::ServiceServer service4 = n.advertiseService("tp_mp_communication", &InverseDifferential::motionPlannerToTaskPlannerServiceResponse, this);
        
        ros::spin();
    }

    return 0;
}

bool InverseDifferential::invDiff(){
    ros::init(argc, argv, "ur5_invDiff");
    ros::NodeHandle n;
    ros::Publisher joint_pub = n.advertise<std_msgs::Float64MultiArray>(TOPIC, 1000);
    ros::Subscriber sub = n.subscribe(TOPIC_SUB, 1000, &InverseDifferential::receive_jstate, this);
    ros::Rate loop_rate(RATE);

    ros::Duration(1).sleep();
    ros::spinOnce();//IMPORTANT!!! to make sure that the initial configuration is read from the subscriber

    //check first if the final position is possible
    Matrix3d Rf = eulerAnglesToRotationMatrix(phief);
    MatrixXd TH0 = ur5Inverse(xef, Rf);
    MatrixXd M = purgeNanColumn(TH0);
    if(M.cols() == 0) {ROS_INFO("Motion not possible"); return false;}

    //print initial q state
    VectorXd qstart(6);
    for(int i=0; i<q.size(); ++i){
        qstart(i) = q(i);
    }

    //get initial pose of the robot knowing the initial joint states
    Matrix3d Re;
    ur5Direct(xe0, Re, qstart);
    phie0 = rotationMatrixToEulerAngles(Re);
    
    //calculate quaternions
    //from the initial configuration
    q0 = rotationMatrixToQuaternion(Re);
    //from the desired final configuration
    qf = rotationMatrixToQuaternion(Rf);

    //calculate the trajectory of the robot by a function of time
    //and quaternions
    VectorXd T;
    double L=(Tf-Tb)/deltaT;
    T.resize(L+1);
    double time=0;

    for(int i=0;i<=L;i++){
        T(i)=time;
        time +=0.1;
    } 

    Matrix3d Kp = 10*MatrixXd::Identity(3,3);
    Matrix3d Kq = 10*MatrixXd::Identity(3,3);
    
     list <VectorXd> l;

    l = invDiffKinematic(qstart, T, Kp, Kq); 

    //#if INTERMEDIATE_POINT == 1
    if(error == 2){
        //during the computation, we reached a singularity
        return false;
    }
    //#endif
    

    MatrixXd v1;
    v1.resize(JOINT_NAMES, (Tf - Tb) / deltaT);
    int k =0;
    auto iterator = l.begin();

    //from list l to matrix of solutions
    for(auto i:l){
        v1.col(k) = i;
        k++;
    }

    //check if the final position is valid    
    if(!checkWorkArea(q))return false;

    //send values to joints
    ROS_INFO("Starting motion");
    int i=0;

    //first send the final configuration with the gripper values only
    VectorXd firstConfiguration = v1.col(0);
    send_des_jstate(joint_pub, firstConfiguration);
    ros::Duration(1).sleep();

    

    //send all the calculated configurations
    while(ros::ok()){
        send_des_jstate(joint_pub, v1.col(i++));
        ros::spinOnce();
        loop_rate.sleep();

        if(i >= (Tf - Tb) / deltaT) break;
    }

    //to make sure that we reached precisley the final configuration
    //and the wirst hasn't reached its joints' limits
    VectorXd lastConfiguration = v1.col(((Tf - Tb) / deltaT) - 1);
    fixWirstJointLimits(lastConfiguration);
    send_des_jstate(joint_pub, lastConfiguration);
    ros::Duration(1).sleep();
    ros::spinOnce();

    //check if the final position is valid
    if(!checkWorkArea(q))return false;
    return true;
}


//Convertion functions
Quaterniond InverseDifferential::eulerAnglesToQuaternion(const Vector3d &euler) {
    Quaterniond quaternion;

    quaternion=eulerAnglesToRotationMatrix(euler);

    return quaternion;
}

Vector3d InverseDifferential::rotationMatrixToEulerAngles(const Matrix3d &rotationMatrix) {
    Vector3d euler;

    euler=rotationMatrix.eulerAngles(0,1,2);
    
    for(int j=0;j<3;j++){
        angleCorrection(euler(j));
    }
    return euler;
}

Matrix3d InverseDifferential::eulerAnglesToRotationMatrix(const Vector3d &euler) {
    Matrix3d rotationMatrix;
    
    double roll = euler[0];
    double pitch = euler[1];
    double yaw = euler[2];
    

    //rotation about x axis (Roll)
    Matrix3d rotationX;
    rotationX << 1, 0, 0,
                  0, cos(roll), -sin(roll),
                  0, sin(roll), cos(roll);

    //rotation about y axis (Pithc)
    Matrix3d rotationY;
    rotationY << cos(pitch), 0, sin(pitch),
                  0, 1, 0,
                  -sin(pitch), 0, cos(pitch);

    //rotation about z axis (Yaw)
    Matrix3d rotationZ;
    rotationZ << cos(yaw), -sin(yaw), 0,
                  sin(yaw), cos(yaw), 0,
                  0, 0, 1;

    rotationMatrix = rotationX * rotationY * rotationZ;
    
    //Delete precision errors
    for(int i=0;i<3;i++){
        for(int j=0;j<3;j++){
            angleCorrection(rotationMatrix(i,j));   
        }
    }

    return rotationMatrix;
}

Quaterniond InverseDifferential::rotationMatrixToQuaternion(const Matrix3d &rotationMatrix) {
    Quaterniond quaternion;
    quaternion=rotationMatrix;
    return quaternion;
}


//Inverse differential functions
void InverseDifferential::ur5Direct(Vector3d &xe, Matrix3d &Re, VectorXd q_des){
    
    Matrix4d t10 = getRotationMatrix(q_des(0), ALPHA(0), D(0), A(0));
    Matrix4d t21 = getRotationMatrix(q_des(1), ALPHA(1), D(1), A(1));
    Matrix4d t32 = getRotationMatrix(q_des(2), ALPHA(2), D(2), A(2));
    Matrix4d t43 = getRotationMatrix(q_des(3), ALPHA(3), D(3), A(3));
    Matrix4d t54 = getRotationMatrix(q_des(4), ALPHA(4), D(4), A(4));
    Matrix4d t65 = getRotationMatrix(q_des(5), ALPHA(5), D(5), A(5));
    
    Matrix4d t60 = t10*t21*t32*t43*t54*t65;

    xe = t60.block(0, 3, 3, 1);
    Re = t60.block(0, 0, 3, 3);
}

MatrixXd InverseDifferential::ur5Jac(VectorXd v){
    VectorXd q_des(6);

    q_des << v(0), v(1),  v(2), v(3), v(4), v(5);

    Matrix4d t10 = getRotationMatrix(q_des(0), ALPHA(0), D(0), A(0));
    Matrix4d t21 = getRotationMatrix(q_des(1), ALPHA(1), D(1), A(1));
    Matrix4d t32 = getRotationMatrix(q_des(2), ALPHA(2), D(2), A(2));
    Matrix4d t43 = getRotationMatrix(q_des(3), ALPHA(3), D(3), A(3));
    Matrix4d t54 = getRotationMatrix(q_des(4), ALPHA(4), D(4), A(4));
    Matrix4d t65 = getRotationMatrix(q_des(5), ALPHA(5), D(5), A(5));

    Matrix4d t20 = t10*t21;
    Matrix4d t30 = t10*t21*t32;
    Matrix4d t40 = t10*t21*t32*t43;
    Matrix4d t50 = t10*t21*t32*t43*t54;
    Matrix4d t60 = t10*t21*t32*t43*t54*t65;
    
    Vector3d z0(0,0,1);
    Vector3d z1=t10.block(0,2,3,1);
    Vector3d z2=t20.block(0,2,3,1);
    Vector3d z3=t30.block(0,2,3,1);
    Vector3d z4=t40.block(0,2,3,1);
    Vector3d z5=t50.block(0,2,3,1);

    Vector3d p0(0,0,0);
    Vector3d p1=t10.block(0,3,3,1);
    Vector3d p2=t20.block(0,3,3,1);
    Vector3d p3=t30.block(0,3,3,1);
    Vector3d p4=t40.block(0,3,3,1);
    Vector3d p5=t50.block(0,3,3,1);
    Vector3d P=t60.block(0,3,3,1);

    MatrixXd J;///< the jacobian matrix of the ur5 manipulator
    J.resize(6,6);

    J.col(0)<<(z0.cross(P-p0)), z0;
    J.col(1)<<(z1.cross(P-p1)), z1;
    J.col(2)<<(z2.cross(P-p2)), z2;
    J.col(3)<<(z3.cross(P-p3)), z3;
    J.col(4)<<(z4.cross(P-p4)), z4;
    J.col(5)<<(z5.cross(P-p5)), z5;
    
    return J;  
}

list<VectorXd> InverseDifferential::invDiffKinematic(VectorXd TH0, VectorXd T, Matrix3d Kp, Matrix3d Kphi){
    counter=0;
    Vector3d xe;
    Matrix3d Re;
    VectorXd qk(6);
    qk=TH0;
    double t;
    list <VectorXd> q;
    q.push_back(qk);
    
    //remove second condition when NOT using intermediate point
    for(int l=1;l<T.size()-1 && error != 2;l++){
        t=T(l);
        ur5Direct(xe,Re,qk);

        
        Quaterniond qe=rotationMatrixToQuaternion(Re);

        Vector3d vd=(xd(t)-xd(t-deltaT))/deltaT;

        Quaterniond work = (qd(t+deltaT)*qd(t).conjugate());          
        work.coeffs()*= (2/deltaT);                              
        
        Vector3d omegad= work.vec();                                 
       
        Vector3d xd_t=xd(t);
        Quaterniond qd_t=qd(t);
        VectorXd dotqk(6);
        
        dotqk = invDiffKinematicControl(qk, xe, xd_t, vd, omegad, qe, qd_t, Kp, Kphi); 
           
        VectorXd qk1(6);
        qk1= qk + dotqk*deltaT;

        if(!checkWorkArea(qk1))error=2;

        q.push_back(qk1);
        qk=qk1;    
    }
    return q;
}

VectorXd InverseDifferential::invDiffKinematicControl(VectorXd qk, Vector3d xe, Vector3d xd, Vector3d vd, Vector3d omegad, Quaterniond qe, Quaterniond qd, Matrix3d Kp, Matrix3d Kphi){
    MatrixXd J=ur5Jac(qk);     
    
    Quaterniond qp = qd*qe.conjugate();
    
    Vector3d eo=qp.vec();
                                                
    Vector3d corrected_vd= vd+Kp*(xd-xe);
    Vector3d corrected_omegad= omegad+Kphi*eo;
    VectorXd vd_wd(6);
    for(int i=0; i<3;i++){
        vd_wd(i)=corrected_vd(i);
        vd_wd(i+3)=corrected_omegad(i);
    }
      

    VectorXd dotQ(6);
    MatrixXd identity = MatrixXd::Identity(6,6);

    double determinant = abs(J.determinant());
    
    #if INTERMEDIATE_POINT == 1
    if(determinant<=1e-2){ 
        //cout<<"NEAR SINGULARITY"<<endl;                                                    
        error = 2;//using intermediate point
    }
    dotQ = J.partialPivLu().solve(vd_wd);
    #else
    //dyanmic damping factor using eigenvalues
    if(determinant<1e-2){
        if((counter++)==0   ROS_INFO("NEAR SINGULARITY");
        
        VectorXd eigenvalues = (J * J.transpose()).eigenvalues().real();
        double dampingFactor = DAMPING_FACTOR / eigenvalues.maxCoeff();
        J = J.transpose() * ((J * J.transpose() + pow(dampingFactor, 2) * identity).inverse());
        dotQ = J * vd_wd;
    }else {
        dotQ = J.partialPivLu().solve(vd_wd);
    }
    #endif

    return dotQ;
}

Vector3d InverseDifferential::xd(double ti){
     Vector3d xd;
    double t = ti/Tf;
    if (t >= 1){
        xd = xef;
    }             
    else{
        xd = t*xef + (1-t)*xe0;
    }
    return xd;
    
}

Quaterniond InverseDifferential::qd(double ti){
    double t=ti/Tf;
    Quaterniond qd;
    if(t>=1){
        qd=qf;
    }
    else{    
        qd=q0.slerp(t,qf);            
    }
    return qd;
}
  
MatrixXd InverseDifferential::ur5Inverse(Vector3d &p60, Matrix3d &Re){
    MatrixXd Th(6, 8);
    
    Affine3d hmTransf = Affine3d::Identity();
    hmTransf.translation() = p60;
    hmTransf.linear() = Re;
    Matrix4d T60 = hmTransf.matrix();
  
            
    //finding th1
    Vector4d data(0, 0, -D(5), 1);
    Vector4d p50 = (T60 * data);

    double psi = atan2(p50(1), p50(0));
    // angleCorrection(psi);
    double p50xy = hypot(p50(1), p50(0));
                
                
    if(p50xy < D(3)){
        MatrixXd ones(6,1);
        ones.setOnes();
        Th = ones * NAN;
        cout << "Position request in the unreachable cylinder" << endl;
        return Th;
    }

    double phi1_1 = acos(D(3) / p50xy);
    
    double phi1_2 = -phi1_1;
                
    double th1_1 = psi + phi1_1 + M_PI/2;
    double th1_2 = psi + phi1_2 + M_PI/2;

    double s_th1_1=sin(th1_1);
    double c_th1_1=cos(th1_1);
    double s_th1_2=sin(th1_2);
    double c_th1_2=cos(th1_2);
    

    double p61z_1 = p60(0) * s_th1_1 - p60(1) * c_th1_1;
    double p61z_2 = p60(0) * s_th1_2 - p60(1) * c_th1_2;

    double th5_1_1 = acos((p61z_1 - D(3)) / D(5));
    double th5_1_2 = -acos((p61z_1 - D(3)) / D(5));
    double th5_2_1 = acos((p61z_2 - D(3)) / D(5));
    double th5_2_2 = -acos((p61z_2 - D(3)) / D(5));
    

   
    Matrix4d T10_1 = getRotationMatrix(th1_1, ALPHA(0), D(0), A(0));
    Matrix4d T10_2 = getRotationMatrix(th1_2, ALPHA(0), D(0), A(0));

    Matrix4d T16_1 = (T10_1.inverse()*T60).inverse();
    Matrix4d T16_2 = (T10_2.inverse()*T60).inverse();

    double zy_1 = T16_1(1,2);
    double zx_1 = T16_1(0,2);

    double zy_2 = T16_2(1,2);
    double zx_2 = T16_2(0,2);
    double th6_1_1, th6_1_2, th6_2_1, th6_2_2;

    if(almostZero(sin(th5_1_1)) || (almostZero(zy_1) && almostZero(zx_1))){
        cout << "singular configuration. Choosing arbitrary th6" << endl;
        th6_1_1 = 0;
    } else {
        th6_1_1 = atan2((-zy_1 / sin(th5_1_1)), (zx_1 / sin(th5_1_1)));
    }

    if(almostZero(sin(th5_1_2)) || (almostZero(zy_1) && almostZero(zx_1))){
        cout << "singular configuration. Choosing arbitrary th6" << endl;
        th6_1_2 = 0;
    } else {
        th6_1_2 = atan2((-zy_1 / sin(th5_1_2)), (zx_1 / sin(th5_1_2)));
    }

    if(almostZero(sin(th5_2_1)) || (almostZero(zy_2) && almostZero(zx_2))){
        cout << "singular configuration. Choosing arbitrary th6" << endl;
        th6_2_1 = 0;
    } else {
        th6_2_1 = atan2((-zy_2 / sin(th5_2_1)), (zx_2 / sin(th5_2_1)));
    }

    if(almostZero(sin(th5_2_2)) || (almostZero(zy_2) && almostZero(zx_2))){
        cout << "singular configuration. Choosing arbitrary th6" << endl;
        th6_2_2 = 0;
    } else {
        th6_2_2 = atan2((-zy_2 / sin(th5_2_2)), (zx_2 / sin(th5_2_2)));
    }

    Matrix4d T61_1 = T16_1.inverse();
    Matrix4d T61_2 = T16_2.inverse();

    Matrix4d T54_1_1 = getRotationMatrix(th5_1_1, ALPHA(4), D(4), A(4));
    Matrix4d T54_1_2 = getRotationMatrix(th5_1_2, ALPHA(4), D(4), A(4));
    Matrix4d T54_2_1 = getRotationMatrix(th5_2_1, ALPHA(4), D(4), A(4));
    Matrix4d T54_2_2 = getRotationMatrix(th5_2_2, ALPHA(4), D(4), A(4));

    Matrix4d T65_1_1 = getRotationMatrix(th6_1_1, ALPHA(5), D(5), A(5));
    Matrix4d T65_1_2 = getRotationMatrix(th6_1_2, ALPHA(5), D(5), A(5));
    Matrix4d T65_2_1 = getRotationMatrix(th6_2_1, ALPHA(5), D(5), A(5));
    Matrix4d T65_2_2 = getRotationMatrix(th6_2_2, ALPHA(5), D(5), A(5));

    Matrix4d T41_1_1 = T61_1 * (T54_1_1 * T65_1_1).inverse();
    Matrix4d T41_1_2 = T61_1 * (T54_1_2 * T65_1_2).inverse();
    Matrix4d T41_2_1 = T61_2 * (T54_2_1 * T65_2_1).inverse();
    Matrix4d T41_2_2 = T61_2 * (T54_2_2 * T65_2_2).inverse();
   
    data << 0, -D(3), 0, 1;
    Vector4d P = (T41_1_1 * data);
    Vector3d P31_1_1(P(0),P(1),P(2));

    P = (T41_1_2 * data);
    Vector3d P31_1_2 (P(0),P(1),P(2));
    
    P = (T41_2_1 * data);
    Vector3d P31_2_1 (P(0),P(1),P(2));
            
    P = (T41_2_2 * data);
    Vector3d P31_2_2 (P(0),P(1),P(2));
            
    double th3_1_1_1, th3_1_1_2, th3_1_2_1, th3_1_2_2, 
    th3_2_1_1, th3_2_1_2, th3_2_2_1, th3_2_2_2;

    double C = (pow(P31_1_1.norm(), 2) - A(1) * A(1) - A(2) * A(2)) / (2 * A(1) * A(2));
    if(abs(C) > 1){
        cout << "Point out of the work space for th3_1_1\n";
        th3_1_1_1 = NAN;
        th3_1_1_2 = NAN;
    } else {
        th3_1_1_1 = acos(C);
        th3_1_1_2 = -acos(C);
    }

    C = (pow(P31_1_2.norm(), 2) - A(1) * A(1) - A(2) * A(2)) / (2 * A(1) * A(2));
    if(abs(C) > 1){
        cout << "Point out of the work space for th3_1_2\n";
        th3_1_2_1 = NAN;
        th3_1_2_2 = NAN;
    } else {
        th3_1_2_1 = acos(C);
        th3_1_2_2 = -acos(C);
    }


    C = (pow(P31_2_1.norm(), 2) - A(1) * A(1) - A(2) * A(2)) / (2 * A(1) * A(2));
    if(abs(C) > 1){
        cout << "Point out of the work space for th3_2_1\n";
        th3_2_1_1 = NAN;
        th3_2_1_2 = NAN;
    } else {
        th3_2_1_1 = acos(C);
        th3_2_1_2 = -acos(C);
    }


    C = (pow(P31_2_2.norm(), 2) - A(1) * A(1) - A(2) * A(2)) / (2 * A(1) * A(2));
    if(abs(C) > 1){
        cout << "Point out of the work space for th3_2_2\n";
        th3_2_2_1 = NAN;
        th3_2_2_2 = NAN;
    } else {
        th3_2_2_1 = acos(C);
        th3_2_2_2 = -acos(C);
    }

    double th2_1_1_1, th2_1_1_2, th2_1_2_1, th2_1_2_2,
    th2_2_1_1, th2_2_1_2, th2_2_2_1, th2_2_2_2;

    th2_1_1_1 = -atan2(P31_1_1(1), -P31_1_1(0))+asin((A(2)*sin(th3_1_1_1))/P31_1_1.norm());
    th2_1_1_2 = -atan2(P31_1_1(1), -P31_1_1(0))+asin((A(2)*sin(th3_1_1_2))/P31_1_1.norm());
    th2_1_2_1 = -atan2(P31_1_2(1), -P31_1_2(0))+asin((A(2)*sin(th3_1_2_1))/P31_1_2.norm());
    th2_1_2_2 = -atan2(P31_1_2(1), -P31_1_2(0))+asin((A(2)*sin(th3_1_2_2))/P31_1_2.norm());
    th2_2_1_1 = -atan2(P31_2_1(1), -P31_2_1(0))+asin((A(2)*sin(th3_2_1_1))/P31_2_1.norm());
    th2_2_1_2 = -atan2(P31_2_1(1), -P31_2_1(0))+asin((A(2)*sin(th3_2_1_2))/P31_2_1.norm());
    th2_2_2_1 = -atan2(P31_2_2(1), -P31_2_2(0))+asin((A(2)*sin(th3_2_2_1))/P31_2_2.norm());
    th2_2_2_2 = -atan2(P31_2_2(1), -P31_2_2(0))+asin((A(2)*sin(th3_2_2_2))/P31_2_2.norm());
             
    Matrix4d T21 = getRotationMatrix(th2_1_1_1, ALPHA(1), D(1), A(1));
    Matrix4d T32 = getRotationMatrix(th3_1_1_1, ALPHA(2), D(2), A(2));
    Matrix4d T41 = T41_1_1;
    Matrix4d T43 = (T21 * T32).inverse() * T41;
    double xy = T43(1, 0);
    double xx = T43(0, 0);
    double th4_1_1_1 = atan2(xy, xx);

    T21 = getRotationMatrix(th2_1_1_2, ALPHA(1), D(1), A(1));
    T32 = getRotationMatrix(th3_1_1_2, ALPHA(2), D(2), A(2));
    T41 = T41_1_1;
    T43 = (T21 * T32).inverse() * T41;
    xy = T43(1, 0);
    xx = T43(0, 0);
    double th4_1_1_2 = atan2(xy, xx);

    T21 = getRotationMatrix(th2_1_2_1, ALPHA(1), D(1), A(1));
    T32 = getRotationMatrix(th3_1_2_1, ALPHA(2), D(2), A(2));
    T41 = T41_1_2;
    T43 = (T21 * T32).inverse() * T41;
    xy = T43(1, 0);
    xx = T43(0, 0);
    double th4_1_2_1 = atan2(xy, xx);

    T21 = getRotationMatrix(th2_1_2_2, ALPHA(1), D(1), A(1));
    T32 = getRotationMatrix(th3_1_2_2, ALPHA(2), D(2), A(2));
    T41 = T41_1_2;
    T43 = (T21 * T32).inverse() * T41;
    xy = T43(1, 0);
    xx = T43(0, 0);
    double th4_1_2_2 = atan2(xy, xx);

    T21 = getRotationMatrix(th2_2_1_1, ALPHA(1), D(1), A(1));
    T32 = getRotationMatrix(th3_2_1_1, ALPHA(2), D(2), A(2));
    T41 = T41_2_1;
    T43 = (T21 * T32).inverse() * T41;
    xy = T43(1, 0);
    xx = T43(0, 0);
    double th4_2_1_1 = atan2(xy, xx);

    T21 = getRotationMatrix(th2_2_1_2, ALPHA(1), D(1), A(1));
    T32 = getRotationMatrix(th3_2_1_2, ALPHA(2), D(2), A(2));
    T41 = T41_2_1;
    T43 = (T21 * T32).inverse() * T41;
    xy = T43(1, 0);
    xx = T43(0, 0);
    double th4_2_1_2 = atan2(xy, xx);

    T21 = getRotationMatrix(th2_2_2_1, ALPHA(1), D(1), A(1));
    T32 = getRotationMatrix(th3_2_2_1, ALPHA(2), D(2), A(2));
    T41 = T41_2_2;
    T43 = (T21 * T32).inverse() * T41;
    xy = T43(1, 0);
    xx = T43(0, 0);
    double th4_2_2_1 = atan2(xy, xx);

    T21 = getRotationMatrix(th2_2_2_2, ALPHA(1), D(1), A(1));
    T32 = getRotationMatrix(th3_2_2_2, ALPHA(2), D(2), A(2));
    T41 = T41_2_2;
    T43 = (T21 * T32).inverse() * T41;
    xy = T43(1, 0);
    xx = T43(0, 0);
    double th4_2_2_2 = atan2(xy, xx);
    
    Th.resize(6, 8);
    Th.row(0) << th1_1, th1_1, th1_1, th1_1, th1_2, th1_2, th1_2, th1_2;
    Th.row(1) << th2_1_1_1, th2_1_1_2, th2_1_2_1, th2_1_2_2, th2_2_2_1, th2_2_1_2, th2_2_2_1, th2_2_2_2;
    Th.row(2) << th3_1_1_1, th3_1_1_2, th3_1_2_1, th3_1_2_2, th3_2_2_1, th3_2_1_2, th3_2_2_1, th3_2_2_2;
    Th.row(3) << th4_1_1_1, th4_1_1_2, th4_1_2_1, th4_1_2_2, th4_2_2_1, th4_2_1_2, th4_2_2_1, th4_2_2_2;
    Th.row(4) << th5_1_1, th5_1_1, th5_1_2, th5_1_2, th5_2_2, th5_2_1, th5_2_2, th5_2_2;
    Th.row(5) << th6_1_1, th6_1_1, th6_1_2, th6_1_2, th6_2_2, th6_2_1, th6_2_2, th6_2_2;

    return Th;
}

Matrix4d InverseDifferential::getRotationMatrix(double th, double alpha, double d, double a){
    Matrix4d rotation;
    rotation << cos(th), -sin(th)*cos(alpha), sin(th)*sin(alpha), a*cos(th),
        sin(th), cos(th)*cos(alpha), -cos(th)*sin(alpha), a*sin(th),
        0, sin(alpha), cos(alpha), d,
        0, 0, 0, 1;

    
    for(int i=0;i<rotation.rows();i++){
        for(int j=0;j<rotation.cols();j++){
            angleCorrection(rotation(i,j));
        }
    }
    return rotation;                                        
}

MatrixXd InverseDifferential::purgeNanColumn(MatrixXd matrix){
    MatrixXd newMatrix(6, 0);
    int nColumns = 0;

    for (int col = 0; col < matrix.cols(); ++col) {
        bool hasNaN = matrix.col(col).array().isNaN().any();

        if (!hasNaN) {
            newMatrix.conservativeResize(matrix.rows(), nColumns + 1);
            newMatrix.col(nColumns) = matrix.col(col);
            ++nColumns;
        }
    }

    return newMatrix;
}


//Other functions
bool InverseDifferential::checkWorkArea(const Vector3d &position){
    //approximation to 3 decimal to exlude precision errors
    double x = floor((position[0]/SCALAR_FACTOR)*100)/100;
    double y = floor((position[1]/SCALAR_FACTOR)*100)/100;
    double z = floor((position[2]/SCALAR_FACTOR)*100)/100;

    if(MIN_X <= x && x <= MAX_X){
        if(MIN_Y <= y && y <= MAX_Y){
            if(MIN_Z <= z && z <= MAX_Z){
                return true;
            }
        }
    }

    return false;
}

bool InverseDifferential::checkWorkArea(const VectorXd &joints){
    //use direct kinematic to find end effector position
    Vector3d xe;
    Matrix3d Re;

    ur5Direct(xe, Re, joints);

    return checkWorkArea(xe);
}

bool InverseDifferential::almostZero(double value){
    return abs(value)<ALMOST_ZERO;
};

void InverseDifferential::angleCorrection(double &angle){
    if(almostZero(angle*pow(10,9))){
        angle=0;
    }
}

void InverseDifferential::fixWirstJointLimits(Eigen::VectorXd& joints){
    for(int i=3; i<=5; ++i){
        joints[i] = fmod(joints[i] + M_PI, 2 * M_PI) - M_PI;
    }        
}