
#include "CRobot.h"

CRobot::CRobot()
{
}

CRobot::CRobot(const CRobot& orig)
{
}

CRobot::~CRobot()
{
}

void CRobot::setRobotModel(Model* getModel)
{
    //Quaternion QQ(0, 0, 0, 1);
    
//    printf("[test1]\n");

    m_pModel = getModel;
    m_pModel->gravity = Vector3d(0., 0., -9.81);
    nDOF = m_pModel->dof_count - 6; //* get Degree of freedom, Except x,y,z,roll,pitch,yaw of the robot
    joint = new JOINT[nDOF]; //* only joint of the robot excepting x,y,z,roll,pitch,yaw of the robot
    RobotState = VectorNd::Zero(20);
    RobotStatedot = VectorNd::Zero(19);
    RobotState2dot = VectorNd::Zero(19);
    BasePosOri = VectorNd::Zero(6);
    BaseVel = VectorNd::Zero(6);
    JointAngle = VectorNd::Zero(nDOF);
    JointVel = VectorNd::Zero(nDOF);
    
//    printf("[test2]\n");

    base.ID = m_pModel->GetBodyId("REAR_BODY");
    front_body.ID = m_pModel->GetBodyId("FRONT_BODY");
    FR.ID = m_pModel->GetBodyId("FR_CALF");
    FL.ID = m_pModel->GetBodyId("FL_CALF");
    RR.ID = m_pModel->GetBodyId("RR_CALF");
    RL.ID = m_pModel->GetBodyId("RL_CALF");

    QQ << 0, 0, 0, 1;
    m_pModel->SetQuaternion(base.ID, QQ, RobotState);
    
//    printf("[test3]\n");

    init_target_pos << 0, -45, 90, 0, -45, 90, 0, 0, 45, -90, 0, 45, -90;
//    goal_EP << 0,  0.15378, -0.5050, 0.218, -0.45, 0, -0.218, -0.45, 0.7, 0.218, -0.45, 0.7, -0.218, -0.45;
    goal_EP << 0, 0.15378, -0.50, 0, -0.15378, -0.50, 0.0, 0.15378, -0.50, 0.0, -0.15378, -0.50;
      
//    Kp_EP << 18000, 18000, 18000, 18000, 18000, 18000, 18000, 18000, 18000, 18000, 18000, 18000; //2,3,10, 2,3,10, 2,3,10, 2,3,10;
//    Kd_EP << 500, 500, 500, 500, 500, 500, 500, 500, 500, 500, 500, 500;
    
    Kp_q << 18000, 18000, 18000, 18000, 18000, 18000, 18000, 18000, 18000, 18000, 18000, 18000, 18000; //2,3,10, 2,3,10, 2,3,10, 2,3,10;
    Kd_q << 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200;
    
    home_pos_time = 5;
    init_pos_time = 5;
    
//    printf("[test4]\n");
    
//    Kp_EP << 5000, 5000, 15000, 5000, 5000, 15000, 5000, 5000, 15000, 5000, 5000, 15000; //2,3,10, 2,3,10, 2,3,10, 2,3,10;
//    Kd_EP << 200, 200, 600, 200, 200, 600, 200, 200, 600, 200, 200, 600;
}

void CRobot::getCurrentJoint(VectorNd Angle, VectorNd Vel)
{
    for (int nJoint = 0; nJoint < nDOF; nJoint++) {
        joint[nJoint].currentAngle = Angle(nJoint);
        joint[nJoint].currentVel = Vel(nJoint);
    }
}

void CRobot::getRobotState(VectorNd basePosOri, VectorNd baseVel, VectorNd jointAngle, VectorNd jointVel)
{
    BasePosOri = basePosOri;
    BaseVel = baseVel;

    base.currentX = BasePosOri(AXIS_X);
    base.currentY = BasePosOri(AXIS_Y);
    base.currentZ = BasePosOri(AXIS_Z);
    base.currentRoll = BasePosOri(AXIS_Roll);
    base.currentPitch = BasePosOri(AXIS_Pitch);
    base.currentYaw = BasePosOri(AXIS_Yaw);

    base.currentXvel = BaseVel(AXIS_X);
    base.currentYvel = BaseVel(AXIS_Y);
    base.currentZvel = BaseVel(AXIS_Z);
    base.currentRollvel = BaseVel(AXIS_Roll);
    base.currentPitchvel = BaseVel(AXIS_Pitch);
    base.currentYawvel = BaseVel(AXIS_Yaw);

    JointAngle = jointAngle;
    JointVel = jointVel;

    getCurrentJoint(JointAngle, JointVel);
}

//void CRobot::ComputeTorqueControl()
//{
//    RobotState(AXIS_X) = base.currentX;
//    RobotState(AXIS_Y) = base.currentY;
//    RobotState(AXIS_Z) = base.currentZ;
//    RobotState(AXIS_Roll) = base.currentRoll;
//    RobotState(AXIS_Pitch) = base.currentPitch;
//    RobotState(AXIS_Yaw) = base.currentYaw;
//    RobotStatedot(AXIS_X) = base.currentXvel;
//    RobotStatedot(AXIS_Y) = base.currentYvel;
//    RobotStatedot(AXIS_Z) = base.currentZvel;
//    RobotStatedot(AXIS_Roll) = base.currentRollvel;
//    RobotStatedot(AXIS_Pitch) = base.currentPitchvel;
//    RobotStatedot(AXIS_Yaw) = base.currentYawvel;
//
//    for (int nJoint = 0; nJoint < nDOF; nJoint++) {
//        RobotState(6 + nJoint) = joint[nJoint].currentAngle;
//        RobotStatedot(6 + nJoint) = joint[nJoint].currentVel;
//    }
//
//    EP_RL = CalcBodyToBaseCoordinates(*m_pModel, RobotState, RL.ID, EP_OFFSET_RL, true);
//    EP_RR = CalcBodyToBaseCoordinates(*m_pModel, RobotState, RR.ID, EP_OFFSET_RR, true);
//    EP_FL = CalcBodyToBaseCoordinates(*m_pModel, RobotState, FL.ID, EP_OFFSET_FL, true);
//    EP_FR = CalcBodyToBaseCoordinates(*m_pModel, RobotState, FR.ID, EP_OFFSET_FR, true);
//
//    actual_EP.block<3, 1>(0, 0) = EP_RL;
//    actual_EP.block<3, 1>(3, 0) = EP_RR;
//    actual_EP.block<3, 1>(6, 0) = EP_FL;
//    actual_EP.block<3, 1>(9, 0) = EP_FR;
//
//    CalcPointJacobian(*m_pModel, RobotState, RL.ID, EP_OFFSET_RL, J_RL, true);
//    CalcPointJacobian(*m_pModel, RobotState, RR.ID, EP_OFFSET_RR, J_RR, true);
//    CalcPointJacobian(*m_pModel, RobotState, FL.ID, EP_OFFSET_FL, J_FL, true);
//    CalcPointJacobian(*m_pModel, RobotState, FR.ID, EP_OFFSET_FR, J_FR, true);
//    CalcPointJacobian6D(*m_pModel, RobotState, front_body.ID, Originbase, J_FRONT_BODY, true);
//    CalcPointJacobian6D(*m_pModel, RobotState, base.ID, Originbase, J_BASE, true);
//
//    J_A.block<6, 19>(0, 0) = J_BASE;
//    J_A.block<1, 19>(6, 0) = J_FRONT_BODY.block<1, 19>(2, 0); // only yaw
//    J_A.block<3, 19>(7, 0) = J_RL;
//    J_A.block<3, 19>(10, 0) = J_RR;
//    J_A.block<3, 19>(13, 0) = J_FL;
//    J_A.block<3, 19>(16, 0) = J_FR;
//
//    x_dot = J_A*RobotStatedot;
//
//    //actual_EP_vel = x_dot.block<12, 1>(7, 0);
//    actual_EP_vel = x_dot.block(7, 0, 12, 1);
//
//    // ====================== Get dJdQ for CTC ===================== //
//
//    base_dJdQ = CalcPointAcceleration6D(*m_pModel, RobotState, RobotStatedot, ddqZero, base.ID, Originbase, true);
//    FRONT_BODY_dJdQ = CalcPointAcceleration6D(*m_pModel, RobotState, RobotStatedot, ddqZero, front_body.ID, Originbase, true);
//    RL_dJdQ = CalcPointAcceleration(*m_pModel, RobotState, RobotStatedot, ddqZero, RL.ID, EP_OFFSET_RL, true);
//    RR_dJdQ = CalcPointAcceleration(*m_pModel, RobotState, RobotStatedot, ddqZero, RR.ID, EP_OFFSET_RR, true);
//    FL_dJdQ = CalcPointAcceleration(*m_pModel, RobotState, RobotStatedot, ddqZero, FL.ID, EP_OFFSET_FL, true);
//    FR_dJdQ = CalcPointAcceleration(*m_pModel, RobotState, RobotStatedot, ddqZero, FR.ID, EP_OFFSET_FR, true);
//
//    dJdQ.block<6, 1>(0, 0)  = base_dJdQ;
//    dJdQ.block<1, 1>(6, 0)  = FRONT_BODY_dJdQ.block<1, 1>(2, 0);
//    dJdQ.block<3, 1>(7, 0)  = RL_dJdQ;
//    dJdQ.block<3, 1>(10, 0) = RR_dJdQ;
//    dJdQ.block<3, 1>(13, 0) = FL_dJdQ;
//    dJdQ.block<3, 1>(16, 0) = FR_dJdQ;
//
//
//    Fc << 0, 0, 0, 0, 0, 0, 0, 0, 0, Fc_RL, 0, 0, Fc_RR, 0, 0, Fc_FL, 0, 0, Fc_FR;
//
//    for (unsigned int i = 0; i < 7; ++i) {
//        x_2dot_cp[i] = 0;
//
//        if (i == 6) { // waist
//            x_2dot_cp[i] = 0 + 10000 * (0 - RobotState[12]) + 100 * (0 - RobotStatedot[12]);
//            //            cout << "x_2dot_cp[6] = "<< x_2dot_cp[6] << endl;
//        }
//    }
//
////    target_EP.block<12, 1>(0, 0) = goal_EP;
//
//    for (unsigned int i = 0; i < 12; ++i) {
//        x_2dot_cp[i + 7] = target_EP_acc[i] + Kp_EP[i]*(target_EP[i] - actual_EP[i]) + Kd_EP[i]*(target_EP_vel[i] - actual_EP_vel[i]);
//    }
//    
//    printf("x error = %f, y error = %f, z error = %f\n",target_EP[0] - actual_EP[0], target_EP[1] - actual_EP[1], target_EP[2] - actual_EP[2]);
//
//
//    RobotState2dot = J_A.inverse()*(x_2dot_cp - dJdQ);
//
//    CompositeRigidBodyAlgorithm(*m_pModel, RobotState, M_term, true);
//    NonlinearEffects(*m_pModel, RobotState, RobotStatedot, hatNonLinearEffects);
//    NonlinearEffects(*m_pModel, RobotState, VectorNd::Zero(m_pModel->dof_count), G_term);
//
//    C_term = hatNonLinearEffects - G_term;
//
//    CTC_Torque = M_term * RobotState2dot + C_term + G_term + J_A.transpose() * Fc;
//    for (int nJoint = 0; nJoint < nDOF; nJoint++) {
//        joint[nJoint].torque = CTC_Torque(6 + nJoint);
//    }
//}
void CRobot::ComputeTorqueControl()
{
    RobotState(AXIS_X) = base.currentX;
    RobotState(AXIS_Y) = base.currentY;
    RobotState(AXIS_Z) = base.currentZ;
    RobotState(AXIS_Roll) = base.currentRoll;
    RobotState(AXIS_Pitch) = base.currentPitch;
    RobotState(AXIS_Yaw) = base.currentYaw;
    RobotStatedot(AXIS_X) = base.currentXvel;
    RobotStatedot(AXIS_Y) = base.currentYvel;
    RobotStatedot(AXIS_Z) = base.currentZvel;
    RobotStatedot(AXIS_Roll) = base.currentRollvel;
    RobotStatedot(AXIS_Pitch) = base.currentPitchvel;
    RobotStatedot(AXIS_Yaw) = base.currentYawvel;

    for (int nJoint = 0; nJoint < nDOF; nJoint++) {
        RobotState(6 + nJoint) = actual_pos[nJoint];
        RobotStatedot(6 + nJoint) = actual_vel[nJoint];  
    }
    
    Math::Quaternion QQ(0,0,0,1);
    m_pModel->SetQuaternion(base.ID,QQ,RobotState);
    
    cout << "actual_pos = " << actual_pos.transpose() << endl;
    
    actual_EP = FK1(actual_pos);
    
//    target_EP = FK1(target_pos);
    
    cout << "actual_EP = " << actual_EP.transpose() << endl;
    
    if(ControlMode == INITIALIZE){
        
//        Init_Pos_Traj();
//        ctc_cnt = 0;
//        ctc_cnt2 = 0;
//        home_init_flag = true;
//        trot_init_flag = true;
//        turning_init_flag = true;
//        turning_init_flag2 = true;
//        jumping_init_flag = true;
//        forward_init_flag = true;
// 
////      for(unsigned int i=0; i<12; ++i){
////          RB_CON.target_EP[i] = RB_CON.goal_EP[i];
////          RB_CON.target_EP_vel[i] = 0;
////          RB_CON.target_EP_acc[i] = 0;
////      }
////
////      // waist
////      RB_CON.target_pos[6] = RB_CON.goal_pos[6];
    }
    else if(ControlMode == HOME_POS){
        if(home_init_flag == true){
            ctc_cnt = 0;
            home_init_flag = false;
        }
        Home_Pos_Traj();
    }
    else if(ControlMode == POS_INIT){
//        Pos_Init_Traj();
    }
    else if(ControlMode == TROT){
//        if(trot_init_flag == true){
//            ctc_cnt = 0;
//            ctc_cnt2 = 0;
//            step_cnt = 0;
//        }
//        Trot_Traj();
    }
    else if(ControlMode == FORWARD){
//        if(forward_init_flag == true){
//            ctc_cnt = 0;
//            ctc_cnt2 = 0;
//            step_cnt = 0;
//        }
//        Forward_Traj();
    }
    
    
    
     target_pos = IK1(target_EP);
     
     cout << "init_EP from IK = " << target_pos.transpose() <<  endl;
    
    

//    EP_RL = CalcBodyToBaseCoordinates(*m_pModel, RobotState, RL.ID, EP_OFFSET_RL, true);
//    EP_RR = CalcBodyToBaseCoordinates(*m_pModel, RobotState, RR.ID, EP_OFFSET_RR, true);
//    EP_FL = CalcBodyToBaseCoordinates(*m_pModel, RobotState, FL.ID, EP_OFFSET_FL, true);
//    EP_FR = CalcBodyToBaseCoordinates(*m_pModel, RobotState, FR.ID, EP_OFFSET_FR, true);
//
//    actual_EP.block<3, 1>(0, 0) = EP_RL;
//    actual_EP.block<3, 1>(3, 0) = EP_RR;
//    actual_EP.block<3, 1>(6, 0) = EP_FL;
//    actual_EP.block<3, 1>(9, 0) = EP_FR;
    
    for(unsigned int i=0;i<13;++i){
        target_vel[i] = (target_pos[i]-pre_target_pos[i])/dt;
        pre_target_pos[i] = target_pos[i];
 
        target_acc[i] = (target_vel[i]-pre_target_vel[i])/dt;
        pre_target_vel[i] = target_vel[i];
    }
     
     cout << "target_pos = " << target_pos.transpose() << endl;
     cout << "target_vel = " << target_vel.transpose() << endl;
     cout << "target_acc = " << target_acc.transpose() << endl <<  endl;
     
    

    CalcPointJacobian(*m_pModel, RobotState, RL.ID, EP_OFFSET_RL, J_RL, true);
    CalcPointJacobian(*m_pModel, RobotState, RR.ID, EP_OFFSET_RR, J_RR, true);
    CalcPointJacobian(*m_pModel, RobotState, FL.ID, EP_OFFSET_FL, J_FL, true);
    CalcPointJacobian(*m_pModel, RobotState, FR.ID, EP_OFFSET_FR, J_FR, true);
    CalcPointJacobian6D(*m_pModel, RobotState, front_body.ID, Originbase, J_FRONT_BODY, true);
    CalcPointJacobian6D(*m_pModel, RobotState, base.ID, Originbase, J_BASE, true);

    J_A.block<6, 19>(0, 0) = J_BASE;
    J_A.block<1, 19>(6, 0) = J_FRONT_BODY.block<1, 19>(2, 0); // only yaw
    J_A.block<3, 19>(7, 0) = J_RL;
    J_A.block<3, 19>(10, 0) = J_RR;
    J_A.block<3, 19>(13, 0) = J_FL;
    J_A.block<3, 19>(16, 0) = J_FR;

//    x_dot = J_A*RobotStatedot;

    //actual_EP_vel = x_dot.block<12, 1>(7, 0);
//    actual_EP_vel = x_dot.block(7, 0, 12, 1);

    // ====================== Get dJdQ for CTC ===================== //

//    base_dJdQ = CalcPointAcceleration6D(*m_pModel, RobotState, RobotStatedot, ddqZero, base.ID, Originbase, true);
//    FRONT_BODY_dJdQ = CalcPointAcceleration6D(*m_pModel, RobotState, RobotStatedot, ddqZero, front_body.ID, Originbase, true);
//    RL_dJdQ = CalcPointAcceleration(*m_pModel, RobotState, RobotStatedot, ddqZero, RL.ID, EP_OFFSET_RL, true);
//    RR_dJdQ = CalcPointAcceleration(*m_pModel, RobotState, RobotStatedot, ddqZero, RR.ID, EP_OFFSET_RR, true);
//    FL_dJdQ = CalcPointAcceleration(*m_pModel, RobotState, RobotStatedot, ddqZero, FL.ID, EP_OFFSET_FL, true);
//    FR_dJdQ = CalcPointAcceleration(*m_pModel, RobotState, RobotStatedot, ddqZero, FR.ID, EP_OFFSET_FR, true);
//
//    dJdQ.block<6, 1>(0, 0)  = base_dJdQ;
//    dJdQ.block<1, 1>(6, 0)  = FRONT_BODY_dJdQ.block<1, 1>(2, 0);
//    dJdQ.block<3, 1>(7, 0)  = RL_dJdQ;
//    dJdQ.block<3, 1>(10, 0) = RR_dJdQ;
//    dJdQ.block<3, 1>(13, 0) = FL_dJdQ;
//    dJdQ.block<3, 1>(16, 0) = FR_dJdQ;
    
    Fc_RL = -70;
    Fc_RR = -70;
    Fc_FL = -70;
    Fc_FR = -70;


    Fc << 0, 0, 0, 0, 0, 0, 0, 0, 0, Fc_RL, 0, 0, Fc_RR, 0, 0, Fc_FL, 0, 0, Fc_FR;

//    for (unsigned int i = 0; i < 7; ++i) {
//        x_2dot_cp[i] = 0;
//
//        if (i == 6) { // waist
//            x_2dot_cp[i] = 0 + 10000 * (0 - RobotState[12]) + 100 * (0 - RobotStatedot[12]);
//            //            cout << "x_2dot_cp[6] = "<< x_2dot_cp[6] << endl;
//        }
//    }

//    target_EP.block<12, 1>(0, 0) = goal_EP;

//    for (unsigned int i = 0; i < 12; ++i) {
//        x_2dot_cp[i + 7] = target_EP_acc[i] + Kp_EP[i]*(target_EP[i] - actual_EP[i]) + Kd_EP[i]*(target_EP_vel[i] - actual_EP_vel[i]);
//    }
//    
//    printf("x error = %f, y error = %f, z error = %f\n",target_EP[0] - actual_EP[0], target_EP[1] - actual_EP[1], target_EP[2] - actual_EP[2]);


//    RobotState2dot = J_A.inverse()*(x_2dot_cp - dJdQ);
    
    for (unsigned int i = 0; i < 6; ++i) {
        RobotState2dot[i + 6] = 0;
    }
    for (unsigned int i = 0; i < 13; ++i) {
        RobotState2dot[i + 6] = target_acc[i] + Kp_q[i]*(target_pos[i] - actual_pos[i]) + Kd_q[i]*(target_vel[i] - actual_vel[i]);
    }

    

    CompositeRigidBodyAlgorithm(*m_pModel, RobotState, M_term, true);
    NonlinearEffects(*m_pModel, RobotState, RobotStatedot, hatNonLinearEffects);
    NonlinearEffects(*m_pModel, RobotState, VectorNd::Zero(m_pModel->dof_count), G_term);

    C_term = hatNonLinearEffects - G_term;

    CTC_Torque = M_term * RobotState2dot + C_term + G_term + J_A.transpose() * Fc;
    for (int nJoint = 0; nJoint < nDOF; nJoint++) {
        target_tor[nJoint] = CTC_Torque(6 + nJoint);
    }
}

void CRobot::FTsensorTransformation()
{
    RL.T_matrix = CalcBodyWorldOrientation(*m_pModel, RobotState, RL.ID, true);
    RR.T_matrix = CalcBodyWorldOrientation(*m_pModel, RobotState, RR.ID, true);
    FL.T_matrix = CalcBodyWorldOrientation(*m_pModel, RobotState, FL.ID, true);
    FR.T_matrix = CalcBodyWorldOrientation(*m_pModel, RobotState, FR.ID, true);
}



VectorNd CRobot::FK1(VectorNd jointAngle){
 
    const double L1 = 0.15378;
    const double L2 = 0.305;
    const double L3 = 0.305;
 
    static double q1 = 0;//-RB_CON.actual_pos[0];//joint[0].currentAngle;
    static double q2 = 0;// RB_CON.actual_pos[1];//joint[1].currentAngle;
    static double q3 = 0;// RB_CON.actual_pos[2];//joint[2].currentAngle;
 
    q1 =  jointAngle[0];
    q2 =  jointAngle[1];
    q3 =  jointAngle[2];
 
    actual_EP[0] = -L3*(cos(q2)*sin(q3) + cos(q3)*sin(q2)) - L2*sin(q2);
    actual_EP[1] =  L1*cos(q1) - L3*(sin(q1)*sin(q2)*sin(q3) - cos(q2)*cos(q3)*sin(q1)) + L2*cos(q2)*sin(q1);
    actual_EP[2] =  L1*sin(q1) - L3*(cos(q1)*cos(q2)*cos(q3) - cos(q1)*sin(q2)*sin(q3)) - L2*cos(q1)*cos(q2);
 
//  RB_CON.actual_EP[0] = -L3*(cos(q2)*sin(q3) + cos(q3)*sin(q2)) - L2*sin(q2);
//  RB_CON.actual_EP[1] = -(L1*cos(q1) - L3*(sin(q1)*sin(q2)*sin(q3) - cos(q2)*cos(q3)*sin(q1)) + L2*cos(q2)*sin(q1));
//  RB_CON.actual_EP[2] =  L1*sin(q1) - L3*(cos(q1)*cos(q2)*cos(q3) - cos(q1)*sin(q2)*sin(q3)) - L2*cos(q1)*cos(q2);
 
 
//  printf("[FK][RL] x=%f,y=%f,z=%f\n",RB_CON.actual_EP[0],RB_CON.actual_EP[1],RB_CON.actual_EP[2]);
 
    q1 = -jointAngle[3];//joint[3].currentAngle;
    q2 = jointAngle[4];//joint[4].currentAngle;
    q3 = jointAngle[5];//joint[5].currentAngle;
 
//  printf("[RR]q1=%f,q2=%f,q3=%f\n",q1*R2D,q2*R2D,q3*R2D);
 
//  q1 = -11.886108*PI/180;
//  q2 = -29.362610*PI/180;
//  q3 = 59.845605*PI/180;
 
    actual_EP[3] = -L3*(cos(q2)*sin(q3) + cos(q3)*sin(q2)) - L2*sin(q2);
    actual_EP[4] = -(L1*cos(q1) - L3*(sin(q1)*sin(q2)*sin(q3) - cos(q2)*cos(q3)*sin(q1)) + L2*cos(q2)*sin(q1));
    actual_EP[5] =  L1*sin(q1) - L3*(cos(q1)*cos(q2)*cos(q3) - cos(q1)*sin(q2)*sin(q3)) - L2*cos(q1)*cos(q2);
 
//  printf("[FK][RR] x=%f,y=%f,z=%f\n",RB_CON.actual_EP[3],RB_CON.actual_EP[4],RB_CON.actual_EP[5]);
 
 
    q1 = jointAngle[7];//joint[6].currentAngle;
    q2 = jointAngle[8];//joint[7].currentAngle;
    q3 = jointAngle[9];//joint[8].currentAngle;
 
//  printf("[FL]q1=%f,q2=%f,q3=%f\n",q1*R2D,q2*R2D,q3*R2D);
//  q1 = -(11.886108*PI/180);
//  q2 = 29.362610*PI/180;
//  q3 = -59.845605*PI/180;
 
    actual_EP[6] = -(-L3*(cos(q2)*sin(q3) + cos(q3)*sin(q2)) - L2*sin(q2));
    actual_EP[7] = (L1*cos(q1) - L3*(sin(q1)*sin(q2)*sin(q3) - cos(q2)*cos(q3)*sin(q1)) + L2*cos(q2)*sin(q1));
    actual_EP[8] =  L1*sin(q1) - L3*(cos(q1)*cos(q2)*cos(q3) - cos(q1)*sin(q2)*sin(q3)) - L2*cos(q1)*cos(q2);
 
//  printf("[FK][FL] x=%f,y=%f,z=%f\n",RB_CON.actual_EP[6],RB_CON.actual_EP[7],RB_CON.actual_EP[8]);
 
    q1 = -jointAngle[10];//joint[9].currentAngle;
    q2 = jointAngle[11];//joint[10].currentAngle;
    q3 = jointAngle[12];//joint[11].currentAngle;
 
//  printf("[FR]q1=%f,q2=%f,q3=%f\n",q1*R2D,q2*R2D,q3*R2D);
 
//  q1 = -11.886108*PI/180;
//  q2 = 29.362610*PI/180;
//  q3 = -59.845605*PI/180;
 
    actual_EP[9] = -(-L3*(cos(q2)*sin(q3) + cos(q3)*sin(q2)) - L2*sin(q2));
    actual_EP[10] = -(L1*cos(q1) - L3*(sin(q1)*sin(q2)*sin(q3) - cos(q2)*cos(q3)*sin(q1)) + L2*cos(q2)*sin(q1));
    actual_EP[11] =  L1*sin(q1) - L3*(cos(q1)*cos(q2)*cos(q3) - cos(q1)*sin(q2)*sin(q3)) - L2*cos(q1)*cos(q2);
 
//  printf("[FK][FR] x=%f,y=%f,z=%f\n",RB_CON.actual_EP[9],RB_CON.actual_EP[10],RB_CON.actual_EP[11]);
 
    return actual_EP;
}



VectorNd CRobot::IK1(VectorNd EP){
 
    const double L1 = 0.15378;
    const double L2 = 0.305;
    const double L3 = 0.305;
//  const double L1 = 0.1;//0.15378;
//  const double L2 = 0.3;//0.305;
//  const double L3 = 0.3;//0.305;
 
    static double x = 0;
    static double y = 0;
    static double z = 0;
 
//  static double q1, q2, q3;
 
 
    x = EP[0];
    y = EP[1];
    z = EP[2];
 
//  x = -0.005169;//-0.05;
//  y = 0.041596;//0.05;
//  z = 0.548999;//-0.4;
 
    target_pos[0] = (atan(y/abs(z)) - PI/2 + acos(L1/(sqrt(pow(y,2) + pow(z,2)))));
    target_pos[1] = - atan(x/sqrt(abs(-pow(L1,2) + pow(y,2) + pow(z,2)))) - acos((- pow(L1,2) + pow(L2,2) - pow(L3,2) + pow(x,2) + pow(y,2) + pow(z,2))/(2*L2*sqrt(-pow(L1,2) + pow(x,2) + pow(y,2) + pow(z,2))));
    target_pos[2] = PI - acos((pow(L1,2) + pow(L2,2) + pow(L3,2) - pow(x,2) - pow(y,2) - pow(z,2))/(2*L2*L3));
 
//  printf("[RL]q1=%f,q2=%f,q3=%f\n",RB_CON.target_pos[0]*180/PI,RB_CON.target_pos[1]*180/PI,RB_CON.target_pos[2]*180/PI);
 
 
    x =  EP[3];
    y = -EP[4];
    z =  EP[5];
 
//  x = -0.005169;//-0.05;
//  y = -(-0.041596);//0.05;
//  z = 0.548999;//-0.4;
 
    target_pos[3] = -((atan(y/abs(z)) - PI/2 + acos(L1/(sqrt(pow(y,2) + pow(z,2))))));
    target_pos[4] = - atan(x/sqrt(abs(-pow(L1,2) + pow(y,2) + pow(z,2)))) - acos((- pow(L1,2) + pow(L2,2) - pow(L3,2) + pow(x,2) + pow(y,2) + pow(z,2))/(2*L2*sqrt(-pow(L1,2) + pow(x,2) + pow(y,2) + pow(z,2))));
    target_pos[5] = PI - acos((pow(L1,2) + pow(L2,2) + pow(L3,2) - pow(x,2) - pow(y,2) - pow(z,2))/(2*L2*L3));
 
//  printf("[RR]q1=%f,q2=%f,q3=%f\n",RB_CON.target_pos[3]*180/PI,RB_CON.target_pos[4]*180/PI,RB_CON.target_pos[5]*180/PI);
 
 
//  RB_CON.target_pos[6] = 0;
 
    x = EP[6];
    y = EP[7];
    z = EP[8];
 
//  x = -0.005169;//-0.05;
//  y = 0.041596;//0.05;
//  z = 0.548999;//-0.4;
 
    target_pos[7] = (atan(y/abs(z)) - PI/2 + acos(L1/(sqrt(pow(y,2) + pow(z,2)))));
    target_pos[8] = -(- atan(x/sqrt(abs(-pow(L1,2) + pow(y,2) + pow(z,2)))) - acos((- pow(L1,2) + pow(L2,2) - pow(L3,2) + pow(x,2) + pow(y,2) + pow(z,2))/(2*L2*sqrt(-pow(L1,2) + pow(x,2) + pow(y,2) + pow(z,2)))));
    target_pos[9] = -(PI - acos((pow(L1,2) + pow(L2,2) + pow(L3,2) - pow(x,2) - pow(y,2) - pow(z,2))/(2*L2*L3)));
 
//  printf("[FL]q1=%f,q2=%f,q3=%f\n",RB_CON.target_pos[6]*180/PI,RB_CON.target_pos[7]*180/PI,RB_CON.target_pos[8]*180/PI);
 
 
    x = EP[9];
    y = -EP[10];
    z = EP[11];
 
//  x = -0.005169;//-0.05;
//  y = -(-0.041596);//0.05;
//  z = 0.548999;//-0.4;
 
    target_pos[10] = -((atan(y/abs(z)) - PI/2 + acos(L1/(sqrt(pow(y,2) + pow(z,2))))));
    target_pos[11] = -(- atan(x/sqrt(abs(-pow(L1,2) + pow(y,2) + pow(z,2)))) - acos((- pow(L1,2) + pow(L2,2) - pow(L3,2) + pow(x,2) + pow(y,2) + pow(z,2))/(2*L2*sqrt(-pow(L1,2) + pow(x,2) + pow(y,2) + pow(z,2)))));
    target_pos[12] = -(PI - acos((pow(L1,2) + pow(L2,2) + pow(L3,2) - pow(x,2) - pow(y,2) - pow(z,2))/(2*L2*L3)));
 
//  printf("[FR]q1=%f,q2=%f,q3=%f\n",RB_CON.target_pos[9]*180/PI,RB_CON.target_pos[10]*180/PI,RB_CON.target_pos[11]*180/PI);
 
    return target_pos;
}


void CRobot::Init_Pos_Traj(void)
{
    target_tor[0] = 200 * (init_target_pos[0]*D2R - actual_pos[0]) + 20 * (0 - actual_vel[0]); //RL_HIP
    target_tor[1] = 400 * (init_target_pos[1]*D2R - actual_pos[1]) + 20 * (0 - actual_vel[1]); //RL_THIGH
    target_tor[2] = 400 * (init_target_pos[2]*D2R - actual_pos[2]) + 20 * (0 - actual_vel[2]); //RL_CALF
    target_tor[3] = 200 * (init_target_pos[3]*D2R - actual_pos[3]) + 20 * (0 - actual_vel[3]); //RR_HIP
    target_tor[4] = 400 * (init_target_pos[4]*D2R - actual_pos[4]) + 20 * (0 - actual_vel[4]); //RR_THIGH
    target_tor[5] = 400 * (init_target_pos[5]*D2R - actual_pos[5]) + 20 * (0 - actual_vel[5]); //RR_CALF

    target_tor[6] = 200 * (init_target_pos[6]*D2R - actual_pos[6]) + 20 * (0 - actual_vel[6]); //WAIST

    target_tor[7] = 200 * (init_target_pos[7]*D2R - actual_pos[7]) + 20 * (0 - actual_vel[7]); //FL_HIP
    target_tor[8] = 400 * (init_target_pos[8]*D2R - actual_pos[8]) + 20 * (0 - actual_vel[8]); //FL_THIGH
    target_tor[9] = 400 * (init_target_pos[9]*D2R - actual_pos[9]) + 20 * (0 - actual_vel[9]); //FL_CALF
    target_tor[10] = 200 * (init_target_pos[10]*D2R - actual_pos[10]) + 20 * (0 - actual_vel[10]); //FR_HIP
    target_tor[11] = 400 * (init_target_pos[11]*D2R - actual_pos[11]) + 20 * (0 - actual_vel[11]); //FR_THIGH
    target_tor[12] = 400 * (init_target_pos[12]*D2R - actual_pos[12]) + 20 * (0 - actual_vel[12]); //FR_CALF
}



void CRobot::Home_Pos_Traj(void){
 
//  static double home_pos_time = 5;
 
    TROT_PHASE = INIT_Fc;
 
    if(ctc_cnt == 0){
        
        for(unsigned int i=0; i<12; ++i){
            init_EP[i] = actual_EP[i];
            target_EP[i] = actual_EP[i];
            target_EP_vel[i] = 0;
            target_EP_acc[i] = 0;
        }
 
        cout << "init_EP = " << init_EP << endl;
        
        
        // waist
        init_pos[6] = actual_pos[6];
        target_pos[6] = actual_pos[6];
 
        ctc_cnt++;
    }
    else if(ctc_cnt <= (unsigned int)(home_pos_time/dt)){
        
        for(unsigned int i=0; i<12; ++i){
            target_EP[i] = init_EP[i] + (goal_EP[i] - init_EP[i])/2.0*(1-cos(PI2/(home_pos_time*2)*(double)(ctc_cnt)*dt));
            target_EP_vel[i] = (goal_EP[i] - init_EP[i])/2.0*PI2/(home_pos_time*2)*(sin(PI2/(home_pos_time*2)*(double)(ctc_cnt)*dt));
            target_EP_acc[i] = (goal_EP[i] - init_EP[i])/2.0*PI2/(home_pos_time*2)*PI2/(home_pos_time*2)*(cos(PI2/(home_pos_time*2)*(double)(ctc_cnt)*dt));
        }
 
        // waist
        target_pos[6] = init_pos[6] + (0 - init_pos[6])/2.0*(1-cos(PI2/(home_pos_time*2)*(double)(ctc_cnt)*dt));
 
 
        ctc_cnt++;
    }
    else{
        
        for(unsigned int i=0; i<12; ++i){
            target_EP[i] = goal_EP[i];
            target_EP_vel[i] = 0;
            target_EP_acc[i] = 0;
        }
 
        // waist
        target_pos[6] = 0;
    }
}