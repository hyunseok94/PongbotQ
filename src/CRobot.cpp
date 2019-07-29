
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
    goal_EP << 0, 0.15378, -0.40, 0, -0.15378, -0.40, 0.0, 0.15378, -0.40, 0.0, -0.15378, -0.40;

    //    Kp_EP << 18000, 18000, 18000, 18000, 18000, 18000, 18000, 18000, 18000, 18000, 18000, 18000; //2,3,10, 2,3,10, 2,3,10, 2,3,10;
    //    Kd_EP << 500, 500, 500, 500, 500, 500, 500, 500, 500, 500, 500, 500;

    //Kp_q << 18000, 18000, 18000, 18000, 18000, 18000, 18000, 18000, 18000, 18000, 18000, 18000, 18000; //2,3,10, 2,3,10, 2,3,10, 2,3,10;
    //Kd_q << 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200;

    //Kp_q << 120000, 120000, 120000, 120000, 120000, 120000, 10000, 120000, 120000, 120000, 120000, 120000, 120000; //2,3,10, 2,3,10, 2,3,10, 2,3,10;
    //Kd_q << 1200, 1200, 1200, 1200, 1200, 1200, 100, 1200, 1200, 1200, 1200, 1200, 1200;
    Kp_q << 50000, 50000, 50000, 50000, 50000, 50000, 10000, 50000, 50000, 50000, 50000, 50000, 50000; //2,3,10, 2,3,10, 2,3,10, 2,3,10;
    Kd_q << 500, 500, 500, 500, 500, 500,      100,     500, 500, 500, 500, 500, 500;
    init_Kp_q<< 50000, 50000, 50000, 50000, 50000, 50000, 10000, 50000, 50000, 50000, 50000, 50000, 50000; //2,3,10, 2,3,10, 2,3,10, 2,3,10;
    init_Kd_q<< 500, 500, 500, 500, 500, 500,       100      , 500, 500, 500, 500, 500, 500;
    goal_Kp_q << 60000, 60000, 60000, 60000, 60000, 60000, 10000, 60000, 60000, 60000, 60000, 60000, 60000; //2,3,10, 2,3,10, 2,3,10, 2,3,10;
    goal_Kd_q << 600, 600, 600, 600, 600, 600,     100,       600, 600, 600, 600, 600, 600;
    
//    goal_Kp_q << 50000, 50000, 50000, 50000, 50000, 50000, 10000, 50000, 50000, 50000, 50000, 50000, 50000; //2,3,10, 2,3,10, 2,3,10, 2,3,10;
//    goal_Kd_q << 500, 500, 500, 500, 500, 500,     100,       500, 500, 500, 500, 500, 500;
    
    // Kp_q << 100000, 100000, 100000, 100000, 100000, 100000, 100000, 100000, 100000, 100000, 100000, 100000, 100000; //2,3,10, 2,3,10, 2,3,10, 2,3,10;
    // Kd_q << 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000;
    
    
    init_pos_time = 1;
    home_pos_time = 5;
    
    ts = 0.20;
    tf = dsp_time - ts;
    
    Traj_gen();

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

    Math::Quaternion QQ(0, 0, 0, 1);
    m_pModel->SetQuaternion(base.ID, QQ, RobotState);

    actual_EP = FK1(actual_pos);

    if (ControlMode == INITIALIZE) {
                ctc_cnt = 0;
                ctc_cnt2 = 0;
                home_init_flag = true;
                trot_init_flag = true;
                forward_init_flag = true;
                flying_trot_init_flag = true;
    }
    else if (ControlMode == HOME_POS) {
        if (home_init_flag == true) {
            ctc_cnt = 0;
            home_init_flag = false;
        }
        Home_Pos_Traj();
    }
    else if (ControlMode == POS_INIT) {

    }
    else if (ControlMode == TROT) {
        if(forward_init_flag == true){
            ctc_cnt2 = 0;
            step_cnt = 0;
        }
        //        Trot_Traj();
        
        Forward_Traj();
    }
    else if (ControlMode == FORWARD) {
        //        if(forward_init_flag == true){
        //            ctc_cnt = 0;
        //            ctc_cnt2 = 0;
        //            step_cnt = 0;
        //        }
        //        Forward_Traj();
    }
    else if (ControlMode == FLYING_TROT) {
        if(flying_trot_init_flag == true){
            ctc_cnt2 = 0;
            step_cnt = 0;
        }
        
        Flying_Trot_Traj();
    }
    



    target_pos = IK1(target_EP);
    
    
//    cout << "EP ERROR = " << (target_EP - actual_EP).transpose() << endl;

//    cout << "init_EP from IK = " << target_pos.transpose() << endl;



    //    EP_RL = CalcBodyToBaseCoordinates(*m_pModel, RobotState, RL.ID, EP_OFFSET_RL, true);
    //    EP_RR = CalcBodyToBaseCoordinates(*m_pModel, RobotState, RR.ID, EP_OFFSET_RR, true);
    //    EP_FL = CalcBodyToBaseCoordinates(*m_pModel, RobotState, FL.ID, EP_OFFSET_FL, true);
    //    EP_FR = CalcBodyToBaseCoordinates(*m_pModel, RobotState, FR.ID, EP_OFFSET_FR, true);
    //
    //    actual_EP.block<3, 1>(0, 0) = EP_RL;
    //    actual_EP.block<3, 1>(3, 0) = EP_RR;
    //    actual_EP.block<3, 1>(6, 0) = EP_FL;
    //    actual_EP.block<3, 1>(9, 0) = EP_FR;

    for (unsigned int i = 0; i < 13; ++i) {
        target_vel[i] = (target_pos[i] - pre_target_pos[i]) / dt;
        pre_target_pos[i] = target_pos[i];

        target_acc[i] = (target_vel[i] - pre_target_vel[i]) / dt;
        pre_target_vel[i] = target_vel[i];
    }

//    cout << "target_pos = " << target_pos.transpose() << endl;
//    cout << "target_vel = " << target_vel.transpose() << endl;
//    cout << "target_acc = " << target_acc.transpose() << endl << endl;

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

//    Fc_RL = -70;
//    Fc_RR = -70;
//    Fc_FL = -70;
//    Fc_FR = -70;
//
//
//    Fc << 0, 0, 0, 0, 0, 0, 0, 0, 0, Fc_RL, 0, 0, Fc_RR, 0, 0, Fc_FL, 0, 0, Fc_FR;
    
    Cal_Fc();

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
//        if(ControlMode==HOME_POS){
//          RobotState2dot[i + 6] = target_acc[i] + home_Kp_q[i]*(target_pos[i] - actual_pos[i]) + home_Kd_q[i]*(target_vel[i] - actual_vel[i]);  
//        }
//        else{
//            RobotState2dot[i + 6] = target_acc[i] + Kp_q[i]*(target_pos[i] - actual_pos[i]) + Kd_q[i]*(target_vel[i] - actual_vel[i]);
//        }
        
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

VectorNd CRobot::FK1(VectorNd jointAngle)
{

    const double L1 = 0.15378;
    const double L2 = 0.305;
    const double L3 = 0.305;

    static double q1 = 0; //-RB_CON.actual_pos[0];//joint[0].currentAngle;
    static double q2 = 0; // RB_CON.actual_pos[1];//joint[1].currentAngle;
    static double q3 = 0; // RB_CON.actual_pos[2];//joint[2].currentAngle;

    q1 = jointAngle[0];
    q2 = jointAngle[1];
    q3 = jointAngle[2];

    actual_EP[0] = -L3 * (cos(q2) * sin(q3) + cos(q3) * sin(q2)) - L2 * sin(q2);
    actual_EP[1] = L1 * cos(q1) - L3 * (sin(q1) * sin(q2) * sin(q3) - cos(q2) * cos(q3) * sin(q1)) + L2 * cos(q2) * sin(q1);
    actual_EP[2] = L1 * sin(q1) - L3 * (cos(q1) * cos(q2) * cos(q3) - cos(q1) * sin(q2) * sin(q3)) - L2 * cos(q1) * cos(q2);

    //  RB_CON.actual_EP[0] = -L3*(cos(q2)*sin(q3) + cos(q3)*sin(q2)) - L2*sin(q2);
    //  RB_CON.actual_EP[1] = -(L1*cos(q1) - L3*(sin(q1)*sin(q2)*sin(q3) - cos(q2)*cos(q3)*sin(q1)) + L2*cos(q2)*sin(q1));
    //  RB_CON.actual_EP[2] =  L1*sin(q1) - L3*(cos(q1)*cos(q2)*cos(q3) - cos(q1)*sin(q2)*sin(q3)) - L2*cos(q1)*cos(q2);


    //  printf("[FK][RL] x=%f,y=%f,z=%f\n",RB_CON.actual_EP[0],RB_CON.actual_EP[1],RB_CON.actual_EP[2]);

    q1 = -jointAngle[3]; //joint[3].currentAngle;
    q2 = jointAngle[4]; //joint[4].currentAngle;
    q3 = jointAngle[5]; //joint[5].currentAngle;

    //  printf("[RR]q1=%f,q2=%f,q3=%f\n",q1*R2D,q2*R2D,q3*R2D);

    //  q1 = -11.886108*PI/180;
    //  q2 = -29.362610*PI/180;
    //  q3 = 59.845605*PI/180;

    actual_EP[3] = -L3 * (cos(q2) * sin(q3) + cos(q3) * sin(q2)) - L2 * sin(q2);
    actual_EP[4] = -(L1 * cos(q1) - L3 * (sin(q1) * sin(q2) * sin(q3) - cos(q2) * cos(q3) * sin(q1)) + L2 * cos(q2) * sin(q1));
    actual_EP[5] = L1 * sin(q1) - L3 * (cos(q1) * cos(q2) * cos(q3) - cos(q1) * sin(q2) * sin(q3)) - L2 * cos(q1) * cos(q2);

    //  printf("[FK][RR] x=%f,y=%f,z=%f\n",RB_CON.actual_EP[3],RB_CON.actual_EP[4],RB_CON.actual_EP[5]);


    q1 = jointAngle[7]; //joint[6].currentAngle;
    q2 = jointAngle[8]; //joint[7].currentAngle;
    q3 = jointAngle[9]; //joint[8].currentAngle;

    //  printf("[FL]q1=%f,q2=%f,q3=%f\n",q1*R2D,q2*R2D,q3*R2D);
    //  q1 = -(11.886108*PI/180);
    //  q2 = 29.362610*PI/180;
    //  q3 = -59.845605*PI/180;

    actual_EP[6] = -(-L3 * (cos(q2) * sin(q3) + cos(q3) * sin(q2)) - L2 * sin(q2));
    actual_EP[7] = (L1 * cos(q1) - L3 * (sin(q1) * sin(q2) * sin(q3) - cos(q2) * cos(q3) * sin(q1)) + L2 * cos(q2) * sin(q1));
    actual_EP[8] = L1 * sin(q1) - L3 * (cos(q1) * cos(q2) * cos(q3) - cos(q1) * sin(q2) * sin(q3)) - L2 * cos(q1) * cos(q2);

    //  printf("[FK][FL] x=%f,y=%f,z=%f\n",RB_CON.actual_EP[6],RB_CON.actual_EP[7],RB_CON.actual_EP[8]);

    q1 = -jointAngle[10]; //joint[9].currentAngle;
    q2 = jointAngle[11]; //joint[10].currentAngle;
    q3 = jointAngle[12]; //joint[11].currentAngle;

    //  printf("[FR]q1=%f,q2=%f,q3=%f\n",q1*R2D,q2*R2D,q3*R2D);

    //  q1 = -11.886108*PI/180;
    //  q2 = 29.362610*PI/180;
    //  q3 = -59.845605*PI/180;

    actual_EP[9] = -(-L3 * (cos(q2) * sin(q3) + cos(q3) * sin(q2)) - L2 * sin(q2));
    actual_EP[10] = -(L1 * cos(q1) - L3 * (sin(q1) * sin(q2) * sin(q3) - cos(q2) * cos(q3) * sin(q1)) + L2 * cos(q2) * sin(q1));
    actual_EP[11] = L1 * sin(q1) - L3 * (cos(q1) * cos(q2) * cos(q3) - cos(q1) * sin(q2) * sin(q3)) - L2 * cos(q1) * cos(q2);

    //  printf("[FK][FR] x=%f,y=%f,z=%f\n",RB_CON.actual_EP[9],RB_CON.actual_EP[10],RB_CON.actual_EP[11]);

    return actual_EP;
}

VectorNd CRobot::IK1(VectorNd EP)
{

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

    target_pos[0] = (atan(y / abs(z)) - PI / 2 + acos(L1 / (sqrt(pow(y, 2) + pow(z, 2)))));
    target_pos[1] = -atan(x / sqrt(abs(-pow(L1, 2) + pow(y, 2) + pow(z, 2)))) - acos((-pow(L1, 2) + pow(L2, 2) - pow(L3, 2) + pow(x, 2) + pow(y, 2) + pow(z, 2)) / (2 * L2 * sqrt(-pow(L1, 2) + pow(x, 2) + pow(y, 2) + pow(z, 2))));
    target_pos[2] = PI - acos((pow(L1, 2) + pow(L2, 2) + pow(L3, 2) - pow(x, 2) - pow(y, 2) - pow(z, 2)) / (2 * L2 * L3));

    //  printf("[RL]q1=%f,q2=%f,q3=%f\n",RB_CON.target_pos[0]*180/PI,RB_CON.target_pos[1]*180/PI,RB_CON.target_pos[2]*180/PI);


    x = EP[3];
    y = -EP[4];
    z = EP[5];

    //  x = -0.005169;//-0.05;
    //  y = -(-0.041596);//0.05;
    //  z = 0.548999;//-0.4;

    target_pos[3] = -((atan(y / abs(z)) - PI / 2 + acos(L1 / (sqrt(pow(y, 2) + pow(z, 2))))));
    target_pos[4] = -atan(x / sqrt(abs(-pow(L1, 2) + pow(y, 2) + pow(z, 2)))) - acos((-pow(L1, 2) + pow(L2, 2) - pow(L3, 2) + pow(x, 2) + pow(y, 2) + pow(z, 2)) / (2 * L2 * sqrt(-pow(L1, 2) + pow(x, 2) + pow(y, 2) + pow(z, 2))));
    target_pos[5] = PI - acos((pow(L1, 2) + pow(L2, 2) + pow(L3, 2) - pow(x, 2) - pow(y, 2) - pow(z, 2)) / (2 * L2 * L3));

    //  printf("[RR]q1=%f,q2=%f,q3=%f\n",RB_CON.target_pos[3]*180/PI,RB_CON.target_pos[4]*180/PI,RB_CON.target_pos[5]*180/PI);


    //  RB_CON.target_pos[6] = 0;

    x = EP[6];
    y = EP[7];
    z = EP[8];

    //  x = -0.005169;//-0.05;
    //  y = 0.041596;//0.05;
    //  z = 0.548999;//-0.4;

    target_pos[7] = (atan(y / abs(z)) - PI / 2 + acos(L1 / (sqrt(pow(y, 2) + pow(z, 2)))));
    target_pos[8] = -(-atan(x / sqrt(abs(-pow(L1, 2) + pow(y, 2) + pow(z, 2)))) - acos((-pow(L1, 2) + pow(L2, 2) - pow(L3, 2) + pow(x, 2) + pow(y, 2) + pow(z, 2)) / (2 * L2 * sqrt(-pow(L1, 2) + pow(x, 2) + pow(y, 2) + pow(z, 2)))));
    target_pos[9] = -(PI - acos((pow(L1, 2) + pow(L2, 2) + pow(L3, 2) - pow(x, 2) - pow(y, 2) - pow(z, 2)) / (2 * L2 * L3)));

    //  printf("[FL]q1=%f,q2=%f,q3=%f\n",RB_CON.target_pos[6]*180/PI,RB_CON.target_pos[7]*180/PI,RB_CON.target_pos[8]*180/PI);


    x = EP[9];
    y = -EP[10];
    z = EP[11];

    //  x = -0.005169;//-0.05;
    //  y = -(-0.041596);//0.05;
    //  z = 0.548999;//-0.4;

    target_pos[10] = -((atan(y / abs(z)) - PI / 2 + acos(L1 / (sqrt(pow(y, 2) + pow(z, 2))))));
    target_pos[11] = -(-atan(x / sqrt(abs(-pow(L1, 2) + pow(y, 2) + pow(z, 2)))) - acos((-pow(L1, 2) + pow(L2, 2) - pow(L3, 2) + pow(x, 2) + pow(y, 2) + pow(z, 2)) / (2 * L2 * sqrt(-pow(L1, 2) + pow(x, 2) + pow(y, 2) + pow(z, 2)))));
    target_pos[12] = -(PI - acos((pow(L1, 2) + pow(L2, 2) + pow(L3, 2) - pow(x, 2) - pow(y, 2) - pow(z, 2)) / (2 * L2 * L3)));

    //  printf("[FR]q1=%f,q2=%f,q3=%f\n",RB_CON.target_pos[9]*180/PI,RB_CON.target_pos[10]*180/PI,RB_CON.target_pos[11]*180/PI);

    return target_pos;
}

void CRobot::Init_Pos_Traj(void)
{
    target_tor[0] = 500 * (init_target_pos[0] * D2R - actual_pos[0]) + 10 * (0 - actual_vel[0]); //RL_HIP
    target_tor[1] = 500 * (init_target_pos[1] * D2R - actual_pos[1]) + 10 * (0 - actual_vel[1]); //RL_THIGH
    target_tor[2] = 500 * (init_target_pos[2] * D2R - actual_pos[2]) + 10 * (0 - actual_vel[2]); //RL_CALF
    target_tor[3] = 500 * (init_target_pos[3] * D2R - actual_pos[3]) + 10 * (0 - actual_vel[3]); //RR_HIP
    target_tor[4] = 500 * (init_target_pos[4] * D2R - actual_pos[4]) + 10 * (0 - actual_vel[4]); //RR_THIGH
    target_tor[5] = 500 * (init_target_pos[5] * D2R - actual_pos[5]) + 10 * (0 - actual_vel[5]); //RR_CALF

    target_tor[6] = 500 * (init_target_pos[6] * D2R - actual_pos[6]) + 10 * (0 - actual_vel[6]); //WAIST

    target_tor[7] = 500 * (init_target_pos[7] * D2R - actual_pos[7]) + 10 * (0 - actual_vel[7]); //FL_HIP
    target_tor[8] = 500 * (init_target_pos[8] * D2R - actual_pos[8]) + 10 * (0 - actual_vel[8]); //FL_THIGH
    target_tor[9] = 500 * (init_target_pos[9] * D2R - actual_pos[9]) + 10 * (0 - actual_vel[9]); //FL_CALF
    target_tor[10] = 500 * (init_target_pos[10] * D2R - actual_pos[10]) + 10 * (0 - actual_vel[10]); //FR_HIP
    target_tor[11] = 500 * (init_target_pos[11] * D2R - actual_pos[11]) + 10 * (0 - actual_vel[11]); //FR_THIGH
    target_tor[12] = 500 * (init_target_pos[12] * D2R - actual_pos[12]) + 10 * (0 - actual_vel[12]); //FR_CALF
}

void CRobot::Home_Pos_Traj(void)
{
//    cout << "," << endl;
    //  static double home_pos_time = 5;

    TROT_PHASE = INIT_Fc;

    if (ctc_cnt == 0) {
//        cout << "[1]" << endl;
        for (unsigned int i = 0; i < 12; ++i) {
            init_EP[i] = actual_EP[i];
            target_EP[i] = actual_EP[i];
            target_EP_vel[i] = 0;
            target_EP_acc[i] = 0;
        }

//        cout << "init_EP = " << init_EP << endl;


        // waist
        init_pos[6] = actual_pos[6];
        target_pos[6] = actual_pos[6];

        for(unsigned int i = 0; i < 13; ++i){
            Kp_q[i] = init_Kp_q[i];
            Kd_q[i] = init_Kd_q[i];
        }
        
        ctc_cnt++;
    }
    else if (ctc_cnt <= (unsigned int) (home_pos_time / dt)) {
//        cout << "[2]" << endl;
        for (unsigned int i = 0; i < 12; ++i) {
            target_EP[i] = init_EP[i] + (goal_EP[i] - init_EP[i]) / 2.0 * (1 - cos(PI2 / (home_pos_time * 2)*(double) (ctc_cnt) * dt));
            target_EP_vel[i] = (goal_EP[i] - init_EP[i]) / 2.0 * PI2 / (home_pos_time * 2)*(sin(PI2 / (home_pos_time * 2)*(double) (ctc_cnt) * dt));
            target_EP_acc[i] = (goal_EP[i] - init_EP[i]) / 2.0 * PI2 / (home_pos_time * 2) * PI2 / (home_pos_time * 2)*(cos(PI2 / (home_pos_time * 2)*(double) (ctc_cnt) * dt));
        }

        // waist
        target_pos[6] = init_pos[6] + (0 - init_pos[6]) / 2.0 * (1 - cos(PI2 / (home_pos_time * 2)*(double) (ctc_cnt) * dt));
        
        ctc_cnt++;
        
        
        for(unsigned int i = 0; i < 13; ++i){
            Kp_q[i] = init_Kp_q[i] + (goal_Kp_q[i] - init_Kp_q[i]) / 2.0 * (1 - cos(PI2 / (home_pos_time * 2)*(double) (ctc_cnt) * dt));
            Kd_q[i] = init_Kd_q[i] + (goal_Kd_q[i] - init_Kd_q[i]) / 2.0 * (1 - cos(PI2 / (home_pos_time * 2)*(double) (ctc_cnt) * dt));
        }
        
        
        
    }
    else {
//        cout << "[3]" << endl;
        for (unsigned int i = 0; i < 12; ++i) {
            target_EP[i] = goal_EP[i];
            target_EP_vel[i] = 0;
            target_EP_acc[i] = 0;
        }
        
        for(unsigned int i = 0; i < 13; ++i){
            Kp_q[i] = goal_Kp_q[i];
            Kd_q[i] = goal_Kd_q[i];
        }

        // waist
        target_pos[6] = 0;
    }
    
//    printf("Kp_q[0]=%f]n",Kp_q[0]);
}

void CRobot::TROT_Traj(void)
{
    const double dsp_time = 0.25;
    const double fsp_time = 0.05;
    const double step_time = dsp_time + fsp_time;
    
    if (ctc_cnt2 == 0) {
        TROT_PHASE = STOP;
        ctc_cnt2++;
    }
    else if (ctc_cnt2 <= (unsigned int) (step_time / dt)) {
        if (ctc_cnt2 <= (unsigned int) (dsp_time / dt)) {
            TROT_PHASE = STANCE_RRFL;
        }
        else {
            TROT_PHASE = STOP; 
        }
        ctc_cnt2++;
    }
    else if (ctc_cnt2 <= (unsigned int) ((step_time * 2) / dt)) {
        if (ctc_cnt2 <= (unsigned int) ((step_time + dsp_time) / dt)) {
            TROT_PHASE = STANCE_RLFR;
        }
        else {
            TROT_PHASE = STOP;
        }

        ctc_cnt2++;
    }
    else {
        TROT_PHASE = STOP;

        static unsigned int step_cnt = 0;
        static unsigned int step_num = 5;

        if (step_cnt < step_num - 1) {
            ctc_cnt2 = 1;
        }

        step_cnt++;
        ctc_cnt = 0;
    }


    switch (TROT_PHASE) {

    case STOP:
        if (trot_init_flag == true) {
            for (unsigned int i = 0; i < 12; ++i) {
                init_EP[i] = target_EP[i];
                target_EP_vel[i] = 0;
                target_EP_acc[i] = 0;
            }

            trot_init_flag = false;
        }

        for (unsigned int i = 0; i < 12; ++i) {
            if (i == 2 || i == 5 || i == 8 || i == 11) {
                trot_goal_EP[i] = init_EP[i] + 0.10;
            }
            else {
                trot_goal_EP[i] = init_EP[i];
            }
        }

        break;

    case STANCE_RRFL:

        for (unsigned int i = 0; i < 3; ++i) {
            target_EP[i] = init_EP[i] + (trot_goal_EP[i] - init_EP[i]) / 2.0 * (1 - cos(PI2 / (dsp_time)*(double) (ctc_cnt2) * dt));
            target_EP_vel[i] = (trot_goal_EP[i] - init_EP[i]) / 2.0 * PI2 / (dsp_time)*(sin(PI2 / (dsp_time)*(double) (ctc_cnt2) * dt));
            target_EP_acc[i] = (trot_goal_EP[i] - init_EP[i]) / 2.0 * PI2 / (dsp_time) * PI2 / (dsp_time)*(cos(PI2 / (dsp_time)*(double) (ctc_cnt2) * dt));
        }

        for (unsigned int i = 3; i < 6; ++i) {
            target_EP[i] = init_EP[i];
            target_EP_vel[i] = 0;
            target_EP_acc[i] = 0;
        }

        for (unsigned int i = 6; i < 9; ++i) {
            target_EP[i] = init_EP[i];
            target_EP_vel[i] = 0;
            target_EP_acc[i] = 0;
        }

        for (unsigned int i = 9; i < 12; ++i) {

            target_EP[i] = init_EP[i] + (trot_goal_EP[i] - init_EP[i]) / 2.0 * (1 - cos(PI2 / (dsp_time)*(double) (ctc_cnt2) * dt));
            target_EP_vel[i] = (trot_goal_EP[i] - init_EP[i]) / 2.0 * PI2 / (dsp_time)*(sin(PI2 / (dsp_time)*(double) (ctc_cnt2) * dt));
            target_EP_acc[i] = (trot_goal_EP[i] - init_EP[i]) / 2.0 * PI2 / (dsp_time) * PI2 / (dsp_time)*(cos(PI2 / (dsp_time)*(double) (ctc_cnt2) * dt));
        }

  
        break;


    case STANCE_RLFR:
        for (unsigned int i = 0; i < 3; ++i) {
            target_EP[i] = init_EP[i];
            target_EP_vel[i] = 0;
            target_EP_acc[i] = 0;
        }

        for (unsigned int i = 3; i < 6; ++i) {
            target_EP[i] = init_EP[i] + (trot_goal_EP[i] - init_EP[i]) / 2.0 * (1 - cos(PI2 / (dsp_time)*((double) ((ctc_cnt2) * dt) - step_time)));
            target_EP_vel[i] = (trot_goal_EP[i] - init_EP[i]) / 2.0 * PI2 / (dsp_time)*(sin(PI2 / (dsp_time)*((double) ((ctc_cnt2) * dt) - step_time)));
            target_EP_acc[i] = (trot_goal_EP[i] - init_EP[i]) / 2.0 * PI2 / (dsp_time) * PI2 / (dsp_time)*(cos(PI2 / (dsp_time)*((double) ((ctc_cnt2) * dt) - step_time)));
        }

        for (unsigned int i = 6; i < 9; ++i) {
            target_EP[i] = init_EP[i] + (trot_goal_EP[i] - init_EP[i]) / 2.0 * (1 - cos(PI2 / (dsp_time)*((double) ((ctc_cnt2) * dt) - step_time)));
            target_EP_vel[i] = (trot_goal_EP[i] - init_EP[i]) / 2.0 * PI2 / (dsp_time)*(sin(PI2 / (dsp_time)*((double) ((ctc_cnt2) * dt) - step_time)));
            target_EP_acc[i] = (trot_goal_EP[i] - init_EP[i]) / 2.0 * PI2 / (dsp_time) * PI2 / (dsp_time)*(cos(PI2 / (dsp_time)*((double) ((ctc_cnt2) * dt) - step_time)));
        }

        for (unsigned int i = 9; i < 12; ++i) {
            target_EP[i] = init_EP[i];
            target_EP_vel[i] = 0;
            target_EP_acc[i] = 0;
        }
        break;
    }
}





void CRobot::Flying_Trot_Traj(void){
 
//    cout << "ctc_cnt2 = " << ctc_cnt2 << endl;
    
    if(ctc_cnt2 == 0){
        FORWARD_PHASE = INIT_FORWARD;
        TROT_PHASE = STOP;
        ctc_cnt2++;
        tmp_cnt2 = 0;
    }
    else if(ctc_cnt2 <= (unsigned int)(dsp_time/dt)){
        if(ctc_cnt2 <= (unsigned int)(dsp_time/dt)){
            FORWARD_PHASE = INIT_STANCE_RRFL;
            TROT_PHASE = STANCE_RRFL;
        }

        ctc_cnt2++;
    }
    else if(ctc_cnt2 <= (unsigned int)((dsp_time*2)/dt)){
        if(ctc_cnt2 <= (unsigned int)((dsp_time+dsp_time)/dt)){
            FORWARD_PHASE = TROT_STANCE_RLFR;
            TROT_PHASE = STANCE_RLFR;
        }

        ctc_cnt2++;
    }
    else if(ctc_cnt2 <= (unsigned int)((dsp_time*3)/dt)){
 
        if(ctc_cnt2 <= (unsigned int)((dsp_time*2+dsp_time)/dt)){
            FORWARD_PHASE = TROT_STANCE_RRFL;
            TROT_PHASE = STANCE_RRFL;
        }
 
 
        ctc_cnt2++;
    }
    else if(ctc_cnt2 <= (unsigned int)((dsp_time*4)/dt)){
        if(ctc_cnt2 <= (unsigned int)((dsp_time*3+dsp_time)/dt)){
            FORWARD_PHASE = FINAL_STANCE_RLFR;
            TROT_PHASE = STANCE_RLFR;
        }
 
        ctc_cnt2++;
    }
    else{
        FORWARD_PHASE = FINAL_STANCE_FOUR;
        TROT_PHASE = STOP;

    }
 
 
    switch(FORWARD_PHASE){
 
        case INIT_FORWARD:
 
            if(flying_trot_init_flag == true){
                for(unsigned int i=0; i<12; ++i){
                    init_goal_EP[i] = goal_EP[i];//RB_CON.target_EP[i];
                    target_EP_vel[i] = 0;
                    target_EP_acc[i] = 0;
                }
 
                step_cnt = 0;
                flying_trot_init_flag = false;
            }
            break;
 
        case INIT_STANCE_RRFL:
 
            tmp_time = (double)(ctc_cnt2)*dt;
 
            for(unsigned int i=0; i<3; ++i){
 
                if(i == 0){
                    target_EP[i] =  + x_init1[0] + x_init1[1]*tmp_time + x_init1[2]*pow(tmp_time,2) + x_init1[3]*pow(tmp_time,3) + x_init1[4]*pow(tmp_time,4) + x_init1[5]*pow(tmp_time,5);
                    target_EP_vel[i] = x_init1[1] + 2*x_init1[2]*pow(tmp_time,1) + 3*x_init1[3]*pow(tmp_time,2) + 4*x_init1[4]*pow(tmp_time,3) + 5*x_init1[5]*pow(tmp_time,4);
                    target_EP_acc[i] = 2*x_init1[2] + 6*x_init1[3]*pow(tmp_time,1) + 12*x_init1[4]*pow(tmp_time,2) + 20*x_init1[5]*pow(tmp_time,3);
                }
                else if(i == 1){
                    target_EP[i] = init_goal_EP[i];
                    target_EP_vel[i] = 0;
                    target_EP_acc[i] = 0;
                }
                else{ // z
                    if(ctc_cnt2 <= (unsigned int)(ts/dt)){
                        target_EP[i] = init_goal_EP[i] + z_f1[0] + z_f1[1]*tmp_time + z_f1[2]*pow(tmp_time,2) + z_f1[3]*pow(tmp_time,3) + z_f1[4]*pow(tmp_time,4) + z_f1[5]*pow(tmp_time,5);
                        target_EP_vel[i] = z_f1[1] + 2*z_f1[2]*pow(tmp_time,1) + 3*z_f1[3]*pow(tmp_time,2) + 4*z_f1[4]*pow(tmp_time,3) + 5*z_f1[5]*pow(tmp_time,4);
                        target_EP_acc[i] = 2*z_f1[2] + 6*z_f1[3]*pow(tmp_time,1) + 12*z_f1[4]*pow(tmp_time,2) + 20*z_f1[5]*pow(tmp_time,3);
                    }
                    else{
                        tmp_time2 = tmp_time - ts;
 
                        target_EP[i] = init_goal_EP[i] + z_f2[0] + z_f2[1]*tmp_time2 + z_f2[2]*pow(tmp_time2,2) + z_f2[3]*pow(tmp_time2,3) + z_f2[4]*pow(tmp_time2,4) + z_f2[5]*pow(tmp_time2,5);
                        target_EP_vel[i] = z_f2[1] + 2*z_f2[2]*pow(tmp_time2,1) + 3*z_f2[3]*pow(tmp_time2,2) + 4*z_f2[4]*pow(tmp_time2,3) + 5*z_f2[5]*pow(tmp_time2,4);
                        target_EP_acc[i] = 2*z_f2[2] + 6*z_f2[3]*pow(tmp_time2,1) + 12*z_f2[4]*pow(tmp_time2,2) + 20*z_f2[5]*pow(tmp_time2,3);
                    }
                }
            }
 
            for(unsigned int i=3; i<6; ++i){
 
                if(i == 3){
                    target_EP[i] =  + x_init2[0] + x_init2[1]*tmp_time + x_init2[2]*pow(tmp_time,2) + x_init2[3]*pow(tmp_time,3) + x_init2[4]*pow(tmp_time,4) + x_init2[5]*pow(tmp_time,5);
                    target_EP_vel[i] = x_init2[1] + 2*x_init2[2]*pow(tmp_time,1) + 3*x_init2[3]*pow(tmp_time,2) + 4*x_init2[4]*pow(tmp_time,3) + 5*x_init2[5]*pow(tmp_time,4);
                    target_EP_acc[i] = 2*x_init2[2] + 6*x_init2[3]*pow(tmp_time,1) + 12*x_init2[4]*pow(tmp_time,2) + 20*x_init2[5]*pow(tmp_time,3);
                }
                else if(i == 4){
                    target_EP[i] = init_goal_EP[i];
                    target_EP_vel[i] = 0;
                    target_EP_acc[i] = 0;
                }
                else{ // z
                    if(ctc_cnt2 <= (unsigned int)(ts/dt)){
                        target_EP[i] = init_goal_EP[i] + z_s1[0] + z_s1[1]*tmp_time + z_s1[2]*pow(tmp_time,2) + z_s1[3]*pow(tmp_time,3) + z_s1[4]*pow(tmp_time,4) + z_s1[5]*pow(tmp_time,5);
                        target_EP_vel[i] = z_s1[1] + 2*z_s1[2]*pow(tmp_time,1) + 3*z_s1[3]*pow(tmp_time,2) + 4*z_s1[4]*pow(tmp_time,3) + 5*z_s1[5]*pow(tmp_time,4);
                        target_EP_acc[i] = 2*z_s1[2] + 6*z_s1[3]*pow(tmp_time,1) + 12*z_s1[4]*pow(tmp_time,2) + 20*z_s1[5]*pow(tmp_time,3);
                    }
                    else{
                        tmp_time2 = tmp_time - ts;
 
                        target_EP[i] = init_goal_EP[i] + z_s2[0] + z_s2[1]*tmp_time2 + z_s2[2]*pow(tmp_time2,2) + z_s2[3]*pow(tmp_time2,3) + z_s2[4]*pow(tmp_time2,4) + z_s2[5]*pow(tmp_time2,5);
                        target_EP_vel[i] = z_s2[1] + 2*z_s2[2]*pow(tmp_time2,1) + 3*z_s2[3]*pow(tmp_time2,2) + 4*z_s2[4]*pow(tmp_time2,3) + 5*z_s2[5]*pow(tmp_time2,4);
                        target_EP_acc[i] = 2*z_s2[2] + 6*z_s2[3]*pow(tmp_time2,1) + 12*z_s2[4]*pow(tmp_time2,2) + 20*z_s2[5]*pow(tmp_time2,3);
                    }
                }
            }
 
            for(unsigned int i=6; i<9; ++i){
 
                if(i == 6){
                    target_EP[i] =  + x_init2[0] + x_init2[1]*tmp_time + x_init2[2]*pow(tmp_time,2) + x_init2[3]*pow(tmp_time,3) + x_init2[4]*pow(tmp_time,4) + x_init2[5]*pow(tmp_time,5);
                    target_EP_vel[i] = x_init2[1] + 2*x_init2[2]*pow(tmp_time,1) + 3*x_init2[3]*pow(tmp_time,2) + 4*x_init2[4]*pow(tmp_time,3) + 5*x_init2[5]*pow(tmp_time,4);
                    target_EP_acc[i] = 2*x_init2[2] + 6*x_trot_dsp1[3]*pow(tmp_time,1) + 12*x_init2[4]*pow(tmp_time,2) + 20*x_init2[5]*pow(tmp_time,3);
                }
                else if(i == 7){
                    target_EP[i] = init_goal_EP[i];
                    target_EP_vel[i] = 0;
                    target_EP_acc[i] = 0;
                }
                else{ // z
                    if(ctc_cnt2 <= (unsigned int)(ts/dt)){
                        target_EP[i] = init_goal_EP[i] + z_s1[0] + z_s1[1]*tmp_time + z_s1[2]*pow(tmp_time,2) + z_s1[3]*pow(tmp_time,3) + z_s1[4]*pow(tmp_time,4) + z_s1[5]*pow(tmp_time,5);
                        target_EP_vel[i] = z_s1[1] + 2*z_s1[2]*pow(tmp_time,1) + 3*z_s1[3]*pow(tmp_time,2) + 4*z_s1[4]*pow(tmp_time,3) + 5*z_s1[5]*pow(tmp_time,4);
                        target_EP_acc[i] = 2*z_s1[2] + 6*z_s1[3]*pow(tmp_time,1) + 12*z_s1[4]*pow(tmp_time,2) + 20*z_s1[5]*pow(tmp_time,3);
                    }
                    else{
                        tmp_time2 = tmp_time - ts;
 
                        target_EP[i] = init_goal_EP[i] + z_s2[0] + z_s2[1]*tmp_time2 + z_s2[2]*pow(tmp_time2,2) + z_s2[3]*pow(tmp_time2,3) + z_s2[4]*pow(tmp_time2,4) + z_s2[5]*pow(tmp_time2,5);
                        target_EP_vel[i] = z_s2[1] + 2*z_s2[2]*pow(tmp_time2,1) + 3*z_s2[3]*pow(tmp_time2,2) + 4*z_s2[4]*pow(tmp_time2,3) + 5*z_s2[5]*pow(tmp_time2,4);
                        target_EP_acc[i] = 2*z_s2[2] + 6*z_s2[3]*pow(tmp_time2,1) + 12*z_s2[4]*pow(tmp_time2,2) + 20*z_s2[5]*pow(tmp_time2,3);
                    }
                    
//                    target_EP[i] = init_goal_EP[i];
//                    target_EP_vel[i] = 0;
//                    target_EP_acc[i] = 0;
                }
            }
 
            for(unsigned int i=9; i<12; ++i){
 
                if(i == 9){
                    target_EP[i] =  + x_init1[0] + x_init1[1]*tmp_time + x_init1[2]*pow(tmp_time,2) + x_init1[3]*pow(tmp_time,3) + x_init1[4]*pow(tmp_time,4) + x_init1[5]*pow(tmp_time,5);
                    target_EP_vel[i] = x_init1[1] + 2*x_init1[2]*pow(tmp_time,1) + 3*x_init1[3]*pow(tmp_time,2) + 4*x_init1[4]*pow(tmp_time,3) + 5*x_init1[5]*pow(tmp_time,4);
                    target_EP_acc[i] = 2*x_init1[2] + 6*x_init1[3]*pow(tmp_time,1) + 12*x_init1[4]*pow(tmp_time,2) + 20*x_init1[5]*pow(tmp_time,3);
                }
                else if(i == 10){
                    target_EP[i] = init_goal_EP[i];
                    target_EP_vel[i] = 0;
                    target_EP_acc[i] = 0;
                }
                else{ // z
                    if(ctc_cnt2 <= (unsigned int)(ts/dt)){
                        target_EP[i] = init_goal_EP[i] + z_f1[0] + z_f1[1]*tmp_time + z_f1[2]*pow(tmp_time,2) + z_f1[3]*pow(tmp_time,3) + z_f1[4]*pow(tmp_time,4) + z_f1[5]*pow(tmp_time,5);
                        target_EP_vel[i] = z_f1[1] + 2*z_f1[2]*pow(tmp_time,1) + 3*z_f1[3]*pow(tmp_time,2) + 4*z_f1[4]*pow(tmp_time,3) + 5*z_f1[5]*pow(tmp_time,4);
                        target_EP_acc[i] = 2*z_f1[2] + 6*z_f1[3]*pow(tmp_time,1) + 12*z_f1[4]*pow(tmp_time,2) + 20*z_f1[5]*pow(tmp_time,3);
                    }
                    else{
                        tmp_time2 = tmp_time - ts;
 
                        target_EP[i] = init_goal_EP[i] + z_f2[0] + z_f2[1]*tmp_time2 + z_f2[2]*pow(tmp_time2,2) + z_f2[3]*pow(tmp_time2,3) + z_f2[4]*pow(tmp_time2,4) + z_f2[5]*pow(tmp_time2,5);
                        target_EP_vel[i] = z_f2[1] + 2*z_f2[2]*pow(tmp_time2,1) + 3*z_f2[3]*pow(tmp_time2,2) + 4*z_f2[4]*pow(tmp_time2,3) + 5*z_f2[5]*pow(tmp_time2,4);
                        target_EP_acc[i] = 2*z_f2[2] + 6*z_f2[3]*pow(tmp_time2,1) + 12*z_f2[4]*pow(tmp_time2,2) + 20*z_f2[5]*pow(tmp_time2,3);
                    }
                }
            }
 
 
            break;
 
        
 
        case TROT_STANCE_RLFR:
 
            tmp_cnt2 = ctc_cnt2 - (int)(dsp_time/dt);
            tmp_time = (double)((ctc_cnt2)*dt)- dsp_time;//(double)(ctc_cnt2)*RB_CON.dt - dsp_time/2;
//
//
            for(unsigned int i=0; i<3; ++i){
 
                if(i == 0){
                    target_EP[i] =  x_trot_dsp1[0] + x_trot_dsp1[1]*tmp_time + x_trot_dsp1[2]*pow(tmp_time,2) + x_trot_dsp1[3]*pow(tmp_time,3) + x_trot_dsp1[4]*pow(tmp_time,4) + x_trot_dsp1[5]*pow(tmp_time,5);
                    target_EP_vel[i] = x_trot_dsp1[1] + 2*x_trot_dsp1[2]*pow(tmp_time,1) + 3*x_trot_dsp1[3]*pow(tmp_time,2) + 4*x_trot_dsp1[4]*pow(tmp_time,3) + 5*x_trot_dsp1[5]*pow(tmp_time,4);
                    target_EP_acc[i] = 2*x_trot_dsp1[2] + 6*x_trot_dsp1[3]*pow(tmp_time,1) + 12*x_trot_dsp1[4]*pow(tmp_time,2) + 20*x_trot_dsp1[5]*pow(tmp_time,3);
                }
                else if(i == 1){
                    target_EP[i] = init_goal_EP[i];
                    target_EP_vel[i] = 0;
                    target_EP_acc[i] = 0;
                }
                else{ // z
                    
                    if(tmp_cnt2 <= (int)(ts/dt)){
                        target_EP[i] = init_goal_EP[i] + z_f3[0] + z_f3[1]*tmp_time + z_f3[2]*pow(tmp_time,2) + z_f3[3]*pow(tmp_time,3) + z_f3[4]*pow(tmp_time,4) + z_f3[5]*pow(tmp_time,5);
                        target_EP_vel[i] = z_f3[1] + 2*z_f3[2]*pow(tmp_time,1) + 3*z_f3[3]*pow(tmp_time,2) + 4*z_f3[4]*pow(tmp_time,3) + 5*z_f3[5]*pow(tmp_time,4);
                        target_EP_acc[i] = 2*z_f3[2] + 6*z_f3[3]*pow(tmp_time,1) + 12*z_f3[4]*pow(tmp_time,2) + 20*z_f3[5]*pow(tmp_time,3);
                    }
                    else{
                        tmp_time2 = tmp_time - ts;
                        
                        target_EP[i] = init_goal_EP[i] + z_s2[0] + z_s2[1]*tmp_time2 + z_s2[2]*pow(tmp_time2,2) + z_s2[3]*pow(tmp_time2,3) + z_s2[4]*pow(tmp_time2,4) + z_s2[5]*pow(tmp_time2,5);
                        target_EP_vel[i] = z_s2[1] + 2*z_s2[2]*pow(tmp_time2,1) + 3*z_s2[3]*pow(tmp_time2,2) + 4*z_s2[4]*pow(tmp_time2,3) + 5*z_s2[5]*pow(tmp_time2,4);
                        target_EP_acc[i] = 2*z_s2[2] + 6*z_s2[3]*pow(tmp_time2,1) + 12*z_s2[4]*pow(tmp_time2,2) + 20*z_s2[5]*pow(tmp_time2,3);
                    
//                        cout << "tmp_time2=" << tmp_time2 << "target_EP[2]=" << target_EP[2] << endl;
                    
                    }
                }
            }
 
 
            for(unsigned int i=3; i<6; ++i){
 
                if(i == 3){
                    target_EP[i] =  + x_trot_dsp2[0] + x_trot_dsp2[1]*tmp_time + x_trot_dsp2[2]*pow(tmp_time,2) + x_trot_dsp2[3]*pow(tmp_time,3) + x_trot_dsp2[4]*pow(tmp_time,4) + x_trot_dsp2[5]*pow(tmp_time,5);
                    target_EP_vel[i] = x_trot_dsp2[1] + 2*x_trot_dsp2[2]*pow(tmp_time,1) + 3*x_trot_dsp2[3]*pow(tmp_time,2) + 4*x_trot_dsp2[4]*pow(tmp_time,3) + 5*x_trot_dsp2[5]*pow(tmp_time,4);
                    target_EP_acc[i] = 2*x_trot_dsp2[2] + 6*x_trot_dsp2[3]*pow(tmp_time,1) + 12*x_trot_dsp2[4]*pow(tmp_time,2) + 20*x_trot_dsp2[5]*pow(tmp_time,3);
                }
                else if(i == 4){
                    target_EP[i] = init_goal_EP[i];
                    target_EP_vel[i] = 0;
                    target_EP_acc[i] = 0;
                }
                else{ // z
                    
                    if(tmp_cnt2 <= (int)(ts/dt)){
                        target_EP[i] = init_goal_EP[i] + z_f1[0] + z_f1[1]*tmp_time + z_f1[2]*pow(tmp_time,2) + z_f1[3]*pow(tmp_time,3) + z_f1[4]*pow(tmp_time,4) + z_f1[5]*pow(tmp_time,5);
                        target_EP_vel[i] = z_f1[1] + 2*z_f1[2]*pow(tmp_time,1) + 3*z_f1[3]*pow(tmp_time,2) + 4*z_f1[4]*pow(tmp_time,3) + 5*z_f1[5]*pow(tmp_time,4);
                        target_EP_acc[i] = 2*z_f1[2] + 6*z_f1[3]*pow(tmp_time,1) + 12*z_f1[4]*pow(tmp_time,2) + 20*z_f1[5]*pow(tmp_time,3);
                    }
                    else{
                        tmp_time2 = tmp_time - ts;
 
                        target_EP[i] = init_goal_EP[i] + z_f2[0] + z_f2[1]*tmp_time2 + z_f2[2]*pow(tmp_time2,2) + z_f2[3]*pow(tmp_time2,3) + z_f2[4]*pow(tmp_time2,4) + z_f2[5]*pow(tmp_time2,5);
                        target_EP_vel[i] = z_f2[1] + 2*z_f2[2]*pow(tmp_time2,1) + 3*z_f2[3]*pow(tmp_time2,2) + 4*z_f2[4]*pow(tmp_time2,3) + 5*z_f2[5]*pow(tmp_time2,4);
                        target_EP_acc[i] = 2*z_f2[2] + 6*z_f2[3]*pow(tmp_time2,1) + 12*z_f2[4]*pow(tmp_time2,2) + 20*z_f2[5]*pow(tmp_time2,3);
                    }
                }
            }
 
 
            for(unsigned int i=6; i<9; ++i){
 
                if(i == 6){
                    target_EP[i] =  + x_trot_dsp2[0] + x_trot_dsp2[1]*tmp_time + x_trot_dsp2[2]*pow(tmp_time,2) + x_trot_dsp2[3]*pow(tmp_time,3) + x_trot_dsp2[4]*pow(tmp_time,4) + x_trot_dsp2[5]*pow(tmp_time,5);
                    target_EP_vel[i] = x_trot_dsp2[1] + 2*x_trot_dsp2[2]*pow(tmp_time,1) + 3*x_trot_dsp2[3]*pow(tmp_time,2) + 4*x_trot_dsp2[4]*pow(tmp_time,3) + 5*x_trot_dsp2[5]*pow(tmp_time,4);
                    target_EP_acc[i] = 2*x_trot_dsp2[2] + 6*x_trot_dsp2[3]*pow(tmp_time,1) + 12*x_trot_dsp2[4]*pow(tmp_time,2) + 20*x_trot_dsp2[5]*pow(tmp_time,3);
                }
                else if(i == 7){
                    target_EP[i] = init_goal_EP[i];
                    target_EP_vel[i] = 0;
                    target_EP_acc[i] = 0;
                }
                else{ // z
                    
                    if(tmp_cnt2 <= (int)(ts/dt)){
                        target_EP[i] = init_goal_EP[i] + z_f1[0] + z_f1[1]*tmp_time + z_f1[2]*pow(tmp_time,2) + z_f1[3]*pow(tmp_time,3) + z_f1[4]*pow(tmp_time,4) + z_f1[5]*pow(tmp_time,5);
                        target_EP_vel[i] = z_f1[1] + 2*z_f1[2]*pow(tmp_time,1) + 3*z_f1[3]*pow(tmp_time,2) + 4*z_f1[4]*pow(tmp_time,3) + 5*z_f1[5]*pow(tmp_time,4);
                        target_EP_acc[i] = 2*z_f1[2] + 6*z_f1[3]*pow(tmp_time,1) + 12*z_f1[4]*pow(tmp_time,2) + 20*z_f1[5]*pow(tmp_time,3);
                    }
                    else{
                        tmp_time2 = tmp_time - ts;
 
                        target_EP[i] = init_goal_EP[i] + z_f2[0] + z_f2[1]*tmp_time2 + z_f2[2]*pow(tmp_time2,2) + z_f2[3]*pow(tmp_time2,3) + z_f2[4]*pow(tmp_time2,4) + z_f2[5]*pow(tmp_time2,5);
                        target_EP_vel[i] = z_f2[1] + 2*z_f2[2]*pow(tmp_time2,1) + 3*z_f2[3]*pow(tmp_time2,2) + 4*z_f2[4]*pow(tmp_time2,3) + 5*z_f2[5]*pow(tmp_time2,4);
                        target_EP_acc[i] = 2*z_f2[2] + 6*z_f2[3]*pow(tmp_time2,1) + 12*z_f2[4]*pow(tmp_time2,2) + 20*z_f2[5]*pow(tmp_time2,3);
                    }
                    
                }
            }
 
 
            for(unsigned int i=9; i<12; ++i){
                if(i == 9){
                    target_EP[i] =  x_trot_dsp1[0] + x_trot_dsp1[1]*tmp_time + x_trot_dsp1[2]*pow(tmp_time,2) + x_trot_dsp1[3]*pow(tmp_time,3) + x_trot_dsp1[4]*pow(tmp_time,4) + x_trot_dsp1[5]*pow(tmp_time,5);
                    target_EP_vel[i] = x_trot_dsp1[1] + 2*x_trot_dsp1[2]*pow(tmp_time,1) + 3*x_trot_dsp1[3]*pow(tmp_time,2) + 4*x_trot_dsp1[4]*pow(tmp_time,3) + 5*x_trot_dsp1[5]*pow(tmp_time,4);
                    target_EP_acc[i] = 2*x_trot_dsp1[2] + 6*x_trot_dsp1[3]*pow(tmp_time,1) + 12*x_trot_dsp1[4]*pow(tmp_time,2) + 20*x_trot_dsp1[5]*pow(tmp_time,3);
                }
                else if(i == 10){
                    target_EP[i] = init_goal_EP[i];
                    target_EP_vel[i] = 0;
                    target_EP_acc[i] = 0;
                }
                else{ // z
                    if(tmp_cnt2 <= (int)(ts/dt)){
                        target_EP[i] = init_goal_EP[i] + z_f3[0] + z_f3[1]*tmp_time + z_f3[2]*pow(tmp_time,2) + z_f3[3]*pow(tmp_time,3) + z_f3[4]*pow(tmp_time,4) + z_f3[5]*pow(tmp_time,5);
                        target_EP_vel[i] = z_f3[1] + 2*z_f3[2]*pow(tmp_time,1) + 3*z_f3[3]*pow(tmp_time,2) + 4*z_f3[4]*pow(tmp_time,3) + 5*z_f3[5]*pow(tmp_time,4);
                        target_EP_acc[i] = 2*z_f3[2] + 6*z_f3[3]*pow(tmp_time,1) + 12*z_f3[4]*pow(tmp_time,2) + 20*z_f3[5]*pow(tmp_time,3);
                    }
                    else{
                        tmp_time2 = tmp_time - ts;
 
                        target_EP[i] = init_goal_EP[i] + z_s2[0] + z_s2[1]*tmp_time2 + z_s2[2]*pow(tmp_time2,2) + z_s2[3]*pow(tmp_time2,3) + z_s2[4]*pow(tmp_time2,4) + z_s2[5]*pow(tmp_time2,5);
                        target_EP_vel[i] = z_s2[1] + 2*z_s2[2]*pow(tmp_time2,1) + 3*z_s2[3]*pow(tmp_time2,2) + 4*z_s2[4]*pow(tmp_time2,3) + 5*z_s2[5]*pow(tmp_time2,4);
                        target_EP_acc[i] = 2*z_s2[2] + 6*z_s2[3]*pow(tmp_time2,1) + 12*z_s2[4]*pow(tmp_time2,2) + 20*z_s2[5]*pow(tmp_time2,3);
                    }
                }
            }
 
            break;
 
 
        case TROT_STANCE_RRFL:
 
            tmp_cnt2 = ctc_cnt2 - (int)(dsp_time*2/dt);
            tmp_time = (double)((ctc_cnt2)*dt)-dsp_time*2;//(double)(ctc_cnt2)*RB_CON.dt - dsp_time/2;
//
//
            for(unsigned int i=0; i<3; ++i){
 
                if(i == 0){
                    target_EP[i] =  + x_trot_dsp2[0] + x_trot_dsp2[1]*tmp_time + x_trot_dsp2[2]*pow(tmp_time,2) + x_trot_dsp2[3]*pow(tmp_time,3) + x_trot_dsp2[4]*pow(tmp_time,4) + x_trot_dsp2[5]*pow(tmp_time,5);
                    target_EP_vel[i] = x_trot_dsp2[1] + 2*x_trot_dsp2[2]*pow(tmp_time,1) + 3*x_trot_dsp2[3]*pow(tmp_time,2) + 4*x_trot_dsp2[4]*pow(tmp_time,3) + 5*x_trot_dsp2[5]*pow(tmp_time,4);
                    target_EP_acc[i] = 2*x_trot_dsp2[2] + 6*x_trot_dsp2[3]*pow(tmp_time,1) + 12*x_trot_dsp2[4]*pow(tmp_time,2) + 20*x_trot_dsp2[5]*pow(tmp_time,3);
                }
                else if(i == 1){
                    target_EP[i] = init_goal_EP[i];
                    target_EP_vel[i] = 0;
                    target_EP_acc[i] = 0;
                }
                else{ // z
 
                    if(tmp_cnt2 <= (int)(ts/dt)){
                        target_EP[i] = init_goal_EP[i] + z_f1[0] + z_f1[1]*tmp_time + z_f1[2]*pow(tmp_time,2) + z_f1[3]*pow(tmp_time,3) + z_f1[4]*pow(tmp_time,4) + z_f1[5]*pow(tmp_time,5);
                        target_EP_vel[i] = z_f1[1] + 2*z_f1[2]*pow(tmp_time,1) + 3*z_f1[3]*pow(tmp_time,2) + 4*z_f1[4]*pow(tmp_time,3) + 5*z_f1[5]*pow(tmp_time,4);
                        target_EP_acc[i] = 2*z_f1[2] + 6*z_f1[3]*pow(tmp_time,1) + 12*z_f1[4]*pow(tmp_time,2) + 20*z_f1[5]*pow(tmp_time,3);
                    }
                    else{
                        tmp_time2 = tmp_time - ts;
 
                        target_EP[i] = init_goal_EP[i] + z_f2[0] + z_f2[1]*tmp_time2 + z_f2[2]*pow(tmp_time2,2) + z_f2[3]*pow(tmp_time2,3) + z_f2[4]*pow(tmp_time2,4) + z_f2[5]*pow(tmp_time2,5);
                        target_EP_vel[i] = z_f2[1] + 2*z_f2[2]*pow(tmp_time2,1) + 3*z_f2[3]*pow(tmp_time2,2) + 4*z_f2[4]*pow(tmp_time2,3) + 5*z_f2[5]*pow(tmp_time2,4);
                        target_EP_acc[i] = 2*z_f2[2] + 6*z_f2[3]*pow(tmp_time2,1) + 12*z_f2[4]*pow(tmp_time2,2) + 20*z_f2[5]*pow(tmp_time2,3);
                    }
 
                }
            }
 
 
            for(unsigned int i=3; i<6; ++i){
 
                if(i == 3){
                    target_EP[i] =  x_trot_dsp1[0] + x_trot_dsp1[1]*tmp_time + x_trot_dsp1[2]*pow(tmp_time,2) + x_trot_dsp1[3]*pow(tmp_time,3) + x_trot_dsp1[4]*pow(tmp_time,4) + x_trot_dsp1[5]*pow(tmp_time,5);
                    target_EP_vel[i] = x_trot_dsp1[1] + 2*x_trot_dsp1[2]*pow(tmp_time,1) + 3*x_trot_dsp1[3]*pow(tmp_time,2) + 4*x_trot_dsp1[4]*pow(tmp_time,3) + 5*x_trot_dsp1[5]*pow(tmp_time,4);
                    target_EP_acc[i] = 2*x_trot_dsp1[2] + 6*x_trot_dsp1[3]*pow(tmp_time,1) + 12*x_trot_dsp1[4]*pow(tmp_time,2) + 20*x_trot_dsp1[5]*pow(tmp_time,3);
                }
                else if(i == 4){
                    target_EP[i] = init_goal_EP[i];
                    target_EP_vel[i] = 0;
                    target_EP_acc[i] = 0;
                }
                else{ // z
                    if(tmp_cnt2 <= (int)(ts/dt)){
                        target_EP[i] = init_goal_EP[i] + z_f3[0] + z_f3[1]*tmp_time + z_f3[2]*pow(tmp_time,2) + z_f3[3]*pow(tmp_time,3) + z_f3[4]*pow(tmp_time,4) + z_f3[5]*pow(tmp_time,5);
                        target_EP_vel[i] = z_f3[1] + 2*z_f3[2]*pow(tmp_time,1) + 3*z_f3[3]*pow(tmp_time,2) + 4*z_f3[4]*pow(tmp_time,3) + 5*z_f3[5]*pow(tmp_time,4);
                        target_EP_acc[i] = 2*z_f3[2] + 6*z_f3[3]*pow(tmp_time,1) + 12*z_f3[4]*pow(tmp_time,2) + 20*z_f3[5]*pow(tmp_time,3);
                    }
                    else{
                        tmp_time2 = tmp_time - ts;
 
                        target_EP[i] = init_goal_EP[i] + z_s2[0] + z_s2[1]*tmp_time2 + z_s2[2]*pow(tmp_time2,2) + z_s2[3]*pow(tmp_time2,3) + z_s2[4]*pow(tmp_time2,4) + z_s2[5]*pow(tmp_time2,5);
                        target_EP_vel[i] = z_s2[1] + 2*z_s2[2]*pow(tmp_time2,1) + 3*z_s2[3]*pow(tmp_time2,2) + 4*z_s2[4]*pow(tmp_time2,3) + 5*z_s2[5]*pow(tmp_time2,4);
                        target_EP_acc[i] = 2*z_s2[2] + 6*z_s2[3]*pow(tmp_time2,1) + 12*z_s2[4]*pow(tmp_time2,2) + 20*z_s2[5]*pow(tmp_time2,3);
                    }
                }
            }
 
 
            for(unsigned int i=6; i<9; ++i){
 
                if(i == 6){
                    target_EP[i] =  x_trot_dsp1[0] + x_trot_dsp1[1]*tmp_time + x_trot_dsp1[2]*pow(tmp_time,2) + x_trot_dsp1[3]*pow(tmp_time,3) + x_trot_dsp1[4]*pow(tmp_time,4) + x_trot_dsp1[5]*pow(tmp_time,5);
                    target_EP_vel[i] = x_trot_dsp1[1] + 2*x_trot_dsp1[2]*pow(tmp_time,1) + 3*x_trot_dsp1[3]*pow(tmp_time,2) + 4*x_trot_dsp1[4]*pow(tmp_time,3) + 5*x_trot_dsp1[5]*pow(tmp_time,4);
                    target_EP_acc[i] = 2*x_trot_dsp1[2] + 6*x_trot_dsp1[3]*pow(tmp_time,1) + 12*x_trot_dsp1[4]*pow(tmp_time,2) + 20*x_trot_dsp1[5]*pow(tmp_time,3);
                }
                else if(i == 7){
                    target_EP[i] = init_goal_EP[i];
                    target_EP_vel[i] = 0;
                    target_EP_acc[i] = 0;
                }
                else{ // z
                    
                    if(tmp_cnt2 <= (int)(ts/dt)){
                        target_EP[i] = init_goal_EP[i] + z_f3[0] + z_f3[1]*tmp_time + z_f3[2]*pow(tmp_time,2) + z_f3[3]*pow(tmp_time,3) + z_f3[4]*pow(tmp_time,4) + z_f3[5]*pow(tmp_time,5);
                        target_EP_vel[i] = z_f3[1] + 2*z_f3[2]*pow(tmp_time,1) + 3*z_f3[3]*pow(tmp_time,2) + 4*z_f3[4]*pow(tmp_time,3) + 5*z_f3[5]*pow(tmp_time,4);
                        target_EP_acc[i] = 2*z_f3[2] + 6*z_f3[3]*pow(tmp_time,1) + 12*z_f3[4]*pow(tmp_time,2) + 20*z_f3[5]*pow(tmp_time,3);
                    }
                    else{
                        tmp_time2 = tmp_time - ts;
 
                        target_EP[i] = init_goal_EP[i] + z_s2[0] + z_s2[1]*tmp_time2 + z_s2[2]*pow(tmp_time2,2) + z_s2[3]*pow(tmp_time2,3) + z_s2[4]*pow(tmp_time2,4) + z_s2[5]*pow(tmp_time2,5);
                        target_EP_vel[i] = z_s2[1] + 2*z_s2[2]*pow(tmp_time2,1) + 3*z_s2[3]*pow(tmp_time2,2) + 4*z_s2[4]*pow(tmp_time2,3) + 5*z_s2[5]*pow(tmp_time2,4);
                        target_EP_acc[i] = 2*z_s2[2] + 6*z_s2[3]*pow(tmp_time2,1) + 12*z_s2[4]*pow(tmp_time2,2) + 20*z_s2[5]*pow(tmp_time2,3);
                    }
                }
            }
 
 
            for(unsigned int i=9; i<12; ++i){
                if(i == 9){
                    target_EP[i] =  + x_trot_dsp2[0] + x_trot_dsp2[1]*tmp_time + x_trot_dsp2[2]*pow(tmp_time,2) + x_trot_dsp2[3]*pow(tmp_time,3) + x_trot_dsp2[4]*pow(tmp_time,4) + x_trot_dsp2[5]*pow(tmp_time,5);
                    target_EP_vel[i] = x_trot_dsp2[1] + 2*x_trot_dsp2[2]*pow(tmp_time,1) + 3*x_trot_dsp2[3]*pow(tmp_time,2) + 4*x_trot_dsp2[4]*pow(tmp_time,3) + 5*x_trot_dsp2[5]*pow(tmp_time,4);
                    target_EP_acc[i] = 2*x_trot_dsp2[2] + 6*x_trot_dsp2[3]*pow(tmp_time,1) + 12*x_trot_dsp2[4]*pow(tmp_time,2) + 20*x_trot_dsp2[5]*pow(tmp_time,3);
                }
                else if(i == 10){
                    target_EP[i] = init_goal_EP[i];
                    target_EP_vel[i] = 0;
                    target_EP_acc[i] = 0;
                }
                else{ // z
                    if(tmp_cnt2 <= (int)(ts/dt)){
                        target_EP[i] = init_goal_EP[i] + z_f1[0] + z_f1[1]*tmp_time + z_f1[2]*pow(tmp_time,2) + z_f1[3]*pow(tmp_time,3) + z_f1[4]*pow(tmp_time,4) + z_f1[5]*pow(tmp_time,5);
                        target_EP_vel[i] = z_f1[1] + 2*z_f1[2]*pow(tmp_time,1) + 3*z_f1[3]*pow(tmp_time,2) + 4*z_f1[4]*pow(tmp_time,3) + 5*z_f1[5]*pow(tmp_time,4);
                        target_EP_acc[i] = 2*z_f1[2] + 6*z_f1[3]*pow(tmp_time,1) + 12*z_f1[4]*pow(tmp_time,2) + 20*z_f1[5]*pow(tmp_time,3);
                    }
                    else{
                        tmp_time2 = tmp_time - ts;
 
                        target_EP[i] = init_goal_EP[i] + z_f2[0] + z_f2[1]*tmp_time2 + z_f2[2]*pow(tmp_time2,2) + z_f2[3]*pow(tmp_time2,3) + z_f2[4]*pow(tmp_time2,4) + z_f2[5]*pow(tmp_time2,5);
                        target_EP_vel[i] = z_f2[1] + 2*z_f2[2]*pow(tmp_time2,1) + 3*z_f2[3]*pow(tmp_time2,2) + 4*z_f2[4]*pow(tmp_time2,3) + 5*z_f2[5]*pow(tmp_time2,4);
                        target_EP_acc[i] = 2*z_f2[2] + 6*z_f2[3]*pow(tmp_time2,1) + 12*z_f2[4]*pow(tmp_time2,2) + 20*z_f2[5]*pow(tmp_time2,3);
                    }
                }
            }
 
            break;
 
 
        case FINAL_STANCE_RLFR:
 
            tmp_cnt2 = ctc_cnt2 - (int)(dsp_time*3/dt);
            tmp_time = (double)((ctc_cnt2)*dt)-dsp_time*3;//(double)(ctc_cnt2)*RB_CON.dt - dsp_time/2;
//
//
            for(unsigned int i=0; i<3; ++i){
 
                if(i == 0){
                    target_EP[i] =  x_final1[0] + x_final1[1]*tmp_time + x_final1[2]*pow(tmp_time,2) + x_final1[3]*pow(tmp_time,3) + x_final1[4]*pow(tmp_time,4) + x_final1[5]*pow(tmp_time,5);
                    target_EP_vel[i] = x_final1[1] + 2*x_final1[2]*pow(tmp_time,1) + 3*x_final1[3]*pow(tmp_time,2) + 4*x_final1[4]*pow(tmp_time,3) + 5*x_final1[5]*pow(tmp_time,4);
                    target_EP_acc[i] = 2*x_final1[2] + 6*x_final1[3]*pow(tmp_time,1) + 12*x_final1[4]*pow(tmp_time,2) + 20*x_final1[5]*pow(tmp_time,3);
                }
                else if(i == 1){
                    target_EP[i] = init_goal_EP[i];
                    target_EP_vel[i] = 0;
                    target_EP_acc[i] = 0;
                }
                else{ // z
                    if(tmp_cnt2 <= (int)(ts/dt)){
                        target_EP[i] = init_goal_EP[i] + z_f3[0] + z_f3[1]*tmp_time + z_f3[2]*pow(tmp_time,2) + z_f3[3]*pow(tmp_time,3) + z_f3[4]*pow(tmp_time,4) + z_f3[5]*pow(tmp_time,5);
                        target_EP_vel[i] = z_f3[1] + 2*z_f3[2]*pow(tmp_time,1) + 3*z_f3[3]*pow(tmp_time,2) + 4*z_f3[4]*pow(tmp_time,3) + 5*z_f3[5]*pow(tmp_time,4);
                        target_EP_acc[i] = 2*z_f3[2] + 6*z_f3[3]*pow(tmp_time,1) + 12*z_f3[4]*pow(tmp_time,2) + 20*z_f3[5]*pow(tmp_time,3);
                    }
                    else{
                        tmp_time2 = tmp_time - ts;
 
                        target_EP[i] = init_goal_EP[i] + z_s2[0] + z_s2[1]*tmp_time2 + z_s2[2]*pow(tmp_time2,2) + z_s2[3]*pow(tmp_time2,3) + z_s2[4]*pow(tmp_time2,4) + z_s2[5]*pow(tmp_time2,5);
                        target_EP_vel[i] = z_s2[1] + 2*z_s2[2]*pow(tmp_time2,1) + 3*z_s2[3]*pow(tmp_time2,2) + 4*z_s2[4]*pow(tmp_time2,3) + 5*z_s2[5]*pow(tmp_time2,4);
                        target_EP_acc[i] = 2*z_s2[2] + 6*z_s2[3]*pow(tmp_time2,1) + 12*z_s2[4]*pow(tmp_time2,2) + 20*z_s2[5]*pow(tmp_time2,3);
                    }
                }
            }
 
 
            for(unsigned int i=3; i<6; ++i){
 
                if(i == 3){
                    target_EP[i] =  + x_final2[0] + x_final2[1]*tmp_time + x_final2[2]*pow(tmp_time,2) + x_final2[3]*pow(tmp_time,3) + x_final2[4]*pow(tmp_time,4) + x_final2[5]*pow(tmp_time,5);
                    target_EP_vel[i] = x_final2[1] + 2*x_final2[2]*pow(tmp_time,1) + 3*x_final2[3]*pow(tmp_time,2) + 4*x_final2[4]*pow(tmp_time,3) + 5*x_final2[5]*pow(tmp_time,4);
                    target_EP_acc[i] = 2*x_final2[2] + 6*x_final2[3]*pow(tmp_time,1) + 12*x_final2[4]*pow(tmp_time,2) + 20*x_final2[5]*pow(tmp_time,3);
                }
                else if(i == 4){
                    target_EP[i] = init_goal_EP[i];
                    target_EP_vel[i] = 0;
                    target_EP_acc[i] = 0;
                }
                else{ // z
                    if(tmp_cnt2 <= (int)(ts/dt)){
                        target_EP[i] = init_goal_EP[i] + z_f1[0] + z_f1[1]*tmp_time + z_f1[2]*pow(tmp_time,2) + z_f1[3]*pow(tmp_time,3) + z_f1[4]*pow(tmp_time,4) + z_f1[5]*pow(tmp_time,5);
                        target_EP_vel[i] = z_f1[1] + 2*z_f1[2]*pow(tmp_time,1) + 3*z_f1[3]*pow(tmp_time,2) + 4*z_f1[4]*pow(tmp_time,3) + 5*z_f1[5]*pow(tmp_time,4);
                        target_EP_acc[i] = 2*z_f1[2] + 6*z_f1[3]*pow(tmp_time,1) + 12*z_f1[4]*pow(tmp_time,2) + 20*z_f1[5]*pow(tmp_time,3);
                    }
                    else{
                        tmp_time2 = tmp_time - ts;
 
                        target_EP[i] = init_goal_EP[i] + z_final1[0] + z_final1[1]*tmp_time2 + z_final1[2]*pow(tmp_time2,2) + z_final1[3]*pow(tmp_time2,3) + z_final1[4]*pow(tmp_time2,4) + z_final1[5]*pow(tmp_time2,5);
                        target_EP_vel[i] = z_final1[1] + 2*z_final1[2]*pow(tmp_time2,1) + 3*z_final1[3]*pow(tmp_time2,2) + 4*z_final1[4]*pow(tmp_time2,3) + 5*z_final1[5]*pow(tmp_time2,4);
                        target_EP_acc[i] = 2*z_final1[2] + 6*z_final1[3]*pow(tmp_time2,1) + 12*z_final1[4]*pow(tmp_time2,2) + 20*z_final1[5]*pow(tmp_time2,3);
                    }
                }
            }
 
 
            for(unsigned int i=6; i<9; ++i){
 
                if(i == 6){
                    target_EP[i] =  + x_final2[0] + x_final2[1]*tmp_time + x_final2[2]*pow(tmp_time,2) + x_final2[3]*pow(tmp_time,3) + x_final2[4]*pow(tmp_time,4) + x_final2[5]*pow(tmp_time,5);
                    target_EP_vel[i] = x_final2[1] + 2*x_final2[2]*pow(tmp_time,1) + 3*x_final2[3]*pow(tmp_time,2) + 4*x_final2[4]*pow(tmp_time,3) + 5*x_final2[5]*pow(tmp_time,4);
                    target_EP_acc[i] = 2*x_final2[2] + 6*x_final2[3]*pow(tmp_time,1) + 12*x_final2[4]*pow(tmp_time,2) + 20*x_final2[5]*pow(tmp_time,3);
                }
                else if(i == 7){
                    target_EP[i] = init_goal_EP[i];
                    target_EP_vel[i] = 0;
                    target_EP_acc[i] = 0;
                }
                else{ // z
                    if(tmp_cnt2 <= (int)(ts/dt)){
                        target_EP[i] = init_goal_EP[i] + z_f1[0] + z_f1[1]*tmp_time + z_f1[2]*pow(tmp_time,2) + z_f1[3]*pow(tmp_time,3) + z_f1[4]*pow(tmp_time,4) + z_f1[5]*pow(tmp_time,5);
                        target_EP_vel[i] = z_f1[1] + 2*z_f1[2]*pow(tmp_time,1) + 3*z_f1[3]*pow(tmp_time,2) + 4*z_f1[4]*pow(tmp_time,3) + 5*z_f1[5]*pow(tmp_time,4);
                        target_EP_acc[i] = 2*z_f1[2] + 6*z_f1[3]*pow(tmp_time,1) + 12*z_f1[4]*pow(tmp_time,2) + 20*z_f1[5]*pow(tmp_time,3);
                    }
                    else{
                        tmp_time2 = tmp_time - ts;
 
                        target_EP[i] = init_goal_EP[i] + z_final1[0] + z_final1[1]*tmp_time2 + z_final1[2]*pow(tmp_time2,2) + z_final1[3]*pow(tmp_time2,3) + z_final1[4]*pow(tmp_time2,4) + z_final1[5]*pow(tmp_time2,5);
                        target_EP_vel[i] = z_final1[1] + 2*z_final1[2]*pow(tmp_time2,1) + 3*z_final1[3]*pow(tmp_time2,2) + 4*z_final1[4]*pow(tmp_time2,3) + 5*z_final1[5]*pow(tmp_time2,4);
                        target_EP_acc[i] = 2*z_final1[2] + 6*z_final1[3]*pow(tmp_time2,1) + 12*z_final1[4]*pow(tmp_time2,2) + 20*z_final1[5]*pow(tmp_time2,3);
                    }
                }
            }
 
 
            for(unsigned int i=9; i<12; ++i){
                if(i == 9){
                    target_EP[i] =  x_final1[0] + x_final1[1]*tmp_time + x_final1[2]*pow(tmp_time,2) + x_final1[3]*pow(tmp_time,3) + x_final1[4]*pow(tmp_time,4) + x_final1[5]*pow(tmp_time,5);
                    target_EP_vel[i] = x_final1[1] + 2*x_final1[2]*pow(tmp_time,1) + 3*x_final1[3]*pow(tmp_time,2) + 4*x_final1[4]*pow(tmp_time,3) + 5*x_final1[5]*pow(tmp_time,4);
                    target_EP_acc[i] = 2*x_final1[2] + 6*x_final1[3]*pow(tmp_time,1) + 12*x_final1[4]*pow(tmp_time,2) + 20*x_final1[5]*pow(tmp_time,3);
                }
                else if(i == 10){
                    target_EP[i] = init_goal_EP[i];
                    target_EP_vel[i] = 0;
                    target_EP_acc[i] = 0;
                }
                else{ // z
                    if(tmp_cnt2 <= (int)(ts/dt)){
                        target_EP[i] = init_goal_EP[i] + z_f3[0] + z_f3[1]*tmp_time + z_f3[2]*pow(tmp_time,2) + z_f3[3]*pow(tmp_time,3) + z_f3[4]*pow(tmp_time,4) + z_f3[5]*pow(tmp_time,5);
                        target_EP_vel[i] = z_f3[1] + 2*z_f3[2]*pow(tmp_time,1) + 3*z_f3[3]*pow(tmp_time,2) + 4*z_f3[4]*pow(tmp_time,3) + 5*z_f3[5]*pow(tmp_time,4);
                        target_EP_acc[i] = 2*z_f3[2] + 6*z_f3[3]*pow(tmp_time,1) + 12*z_f3[4]*pow(tmp_time,2) + 20*z_f3[5]*pow(tmp_time,3);
                    }
                    else{
                        tmp_time2 = tmp_time - ts;
 
                        target_EP[i] = init_goal_EP[i] + z_s2[0] + z_s2[1]*tmp_time2 + z_s2[2]*pow(tmp_time2,2) + z_s2[3]*pow(tmp_time2,3) + z_s2[4]*pow(tmp_time2,4) + z_s2[5]*pow(tmp_time2,5);
                        target_EP_vel[i] = z_s2[1] + 2*z_s2[2]*pow(tmp_time2,1) + 3*z_s2[3]*pow(tmp_time2,2) + 4*z_s2[4]*pow(tmp_time2,3) + 5*z_s2[5]*pow(tmp_time2,4);
                        target_EP_acc[i] = 2*z_s2[2] + 6*z_s2[3]*pow(tmp_time2,1) + 12*z_s2[4]*pow(tmp_time2,2) + 20*z_s2[5]*pow(tmp_time2,3);
                    }
                }
            }
 
            break;
 
        case FINAL_STANCE_FOUR:
            for(unsigned int i=0; i<12; ++i){
                target_EP[i] = init_goal_EP[i];
                target_EP_vel[i] = 0;
                target_EP_acc[i] = 0;
            }
            
            break;
    }
 
 
    if(ctc_cnt2 == (unsigned int)((dsp_time*3)/dt)){
        step_num = 10;
 
        if(step_cnt < step_num-1){
            ctc_cnt2 = (unsigned int)(dsp_time/dt);
        }
 
        step_cnt++;
 
        printf("\n===================== step_num = %d, step_cnt = %d ======================\n",step_num,step_cnt);
 
    }
 
    target_pos[6] = 0;
 
}


void CRobot::Forward_Traj(void){
 
//    cout << "ctc_cnt2 = " << ctc_cnt2 << endl;
    
    if(ctc_cnt2 == 0){
        FORWARD_PHASE = INIT_FORWARD;
        TROT_PHASE = STOP;
        ctc_cnt2++;
        tmp_cnt2 = 0;
    }
    else if(ctc_cnt2 <= (unsigned int)(step_time/dt)){
        if(ctc_cnt2 <= (unsigned int)(dsp_time/dt)){
            FORWARD_PHASE = INIT_STANCE_RRFL;
            TROT_PHASE = STANCE_RRFL;
        }
        else{
            FORWARD_PHASE = INIT_STANCE_FOUR_AFTER_RRFL;
            TROT_PHASE = STOP;
        }
        ctc_cnt2++;
    }
    else if(ctc_cnt2 <= (unsigned int)((step_time*2)/dt)){
        if(ctc_cnt2 <= (unsigned int)((step_time+dsp_time)/dt)){
            FORWARD_PHASE = TROT_STANCE_RLFR;
            TROT_PHASE = STANCE_RLFR;
        }
 
        else{
            FORWARD_PHASE = TROT_STANCE_FOUR_AFTER_RLFR;
            TROT_PHASE = STOP;
        }
        ctc_cnt2++;
    }
    else if(ctc_cnt2 <= (unsigned int)((step_time*3)/dt)){
 
        if(ctc_cnt2 <= (unsigned int)((step_time*2+dsp_time)/dt)){
            FORWARD_PHASE = TROT_STANCE_RRFL;
            TROT_PHASE = STANCE_RRFL;
        }
 
        else{
            FORWARD_PHASE = TROT_STANCE_FOUR_AFTER_RRFL;
            TROT_PHASE = STOP;
        }
 
        ctc_cnt2++;
    }
    else if(ctc_cnt2 <= (unsigned int)((step_time*4)/dt)){
        if(ctc_cnt2 <= (unsigned int)((step_time*3+dsp_time)/dt)){
            FORWARD_PHASE = FINAL_STANCE_RLFR;
            TROT_PHASE = STANCE_RLFR;
        }
 
        else{
            FORWARD_PHASE = FINAL_STANCE_FOUR;
            TROT_PHASE = STOP;
        }
        ctc_cnt2++;
    }
    else{
        FORWARD_PHASE = FINAL_STANCE_FOUR;
        TROT_PHASE = STOP;

    }
 
 
    //  else if(ctc_cnt2 <= (unsigned int)((step_time*4)/RB_CON.dt)){
    //      if(ctc_cnt2 <= (unsigned int)((step_time*3+dsp_time)/RB_CON.dt)){
    //          FORWARD_PHASE = TROT_STANCE_RLFR;
    //      }
    //
    //      else{
    //          FORWARD_PHASE = TROT_STANCE_FOUR;
    //      }
    //      ctc_cnt2++;
    //  }
 
//    printf("FORWARD_PHASE = %d\n",FORWARD_PHASE);
 
    switch(FORWARD_PHASE){
 
        case INIT_FORWARD:
 
            if(forward_init_flag == true){
                for(unsigned int i=0; i<12; ++i){
                    init_goal_EP[i] = goal_EP[i];//RB_CON.target_EP[i];
                    target_EP_vel[i] = 0;
                    target_EP_acc[i] = 0;
                }
 
                step_cnt = 0;
                forward_init_flag = false;
            }
 
            break;
 
        case INIT_STANCE_RRFL:
 
            tmp_time = (double)(ctc_cnt2)*dt;
 
            for(unsigned int i=0; i<3; ++i){
 
                if(i == 0){
                    target_EP[i] =  + x_init1[0] + x_init1[1]*tmp_time + x_init1[2]*pow(tmp_time,2) + x_init1[3]*pow(tmp_time,3) + x_init1[4]*pow(tmp_time,4) + x_init1[5]*pow(tmp_time,5);
                    target_EP_vel[i] = x_init1[1] + 2*x_init1[2]*pow(tmp_time,1) + 3*x_init1[3]*pow(tmp_time,2) + 4*x_init1[4]*pow(tmp_time,3) + 5*x_init1[5]*pow(tmp_time,4);
                    target_EP_acc[i] = 2*x_init1[2] + 6*x_init1[3]*pow(tmp_time,1) + 12*x_init1[4]*pow(tmp_time,2) + 20*x_init1[5]*pow(tmp_time,3);
                }
                else if(i == 1){
                    target_EP[i] = init_goal_EP[i];
                    target_EP_vel[i] = 0;
                    target_EP_acc[i] = 0;
                }
                else{ // z
                    if(ctc_cnt2 <= (unsigned int)((dsp_time/2)/dt)){
                        target_EP[i] = init_goal_EP[i] + z_f1[0] + z_f1[1]*tmp_time + z_f1[2]*pow(tmp_time,2) + z_f1[3]*pow(tmp_time,3) + z_f1[4]*pow(tmp_time,4) + z_f1[5]*pow(tmp_time,5);
                        target_EP_vel[i] = z_f1[1] + 2*z_f1[2]*pow(tmp_time,1) + 3*z_f1[3]*pow(tmp_time,2) + 4*z_f1[4]*pow(tmp_time,3) + 5*z_f1[5]*pow(tmp_time,4);
                        target_EP_acc[i] = 2*z_f1[2] + 6*z_f1[3]*pow(tmp_time,1) + 12*z_f1[4]*pow(tmp_time,2) + 20*z_f1[5]*pow(tmp_time,3);
                    }
                    else{
                        tmp_time2 = tmp_time - dsp_time/2;
 
                        target_EP[i] = init_goal_EP[i] + z_f2[0] + z_f2[1]*tmp_time2 + z_f2[2]*pow(tmp_time2,2) + z_f2[3]*pow(tmp_time2,3) + z_f2[4]*pow(tmp_time2,4) + z_f2[5]*pow(tmp_time2,5);
                        target_EP_vel[i] = z_f2[1] + 2*z_f2[2]*pow(tmp_time2,1) + 3*z_f2[3]*pow(tmp_time2,2) + 4*z_f2[4]*pow(tmp_time2,3) + 5*z_f2[5]*pow(tmp_time2,4);
                        target_EP_acc[i] = 2*z_f2[2] + 6*z_f2[3]*pow(tmp_time2,1) + 12*z_f2[4]*pow(tmp_time2,2) + 20*z_f2[5]*pow(tmp_time2,3);
                    }
                }
            }
 
            for(unsigned int i=3; i<6; ++i){
 
                if(i == 3){
                    target_EP[i] =  + x_init2[0] + x_init2[1]*tmp_time + x_init2[2]*pow(tmp_time,2) + x_init2[3]*pow(tmp_time,3) + x_init2[4]*pow(tmp_time,4) + x_init2[5]*pow(tmp_time,5);
                    target_EP_vel[i] = x_init2[1] + 2*x_init2[2]*pow(tmp_time,1) + 3*x_init2[3]*pow(tmp_time,2) + 4*x_init2[4]*pow(tmp_time,3) + 5*x_init2[5]*pow(tmp_time,4);
                    target_EP_acc[i] = 2*x_init2[2] + 6*x_init2[3]*pow(tmp_time,1) + 12*x_init2[4]*pow(tmp_time,2) + 20*x_init2[5]*pow(tmp_time,3);
                }
                else if(i == 4){
                    target_EP[i] = init_goal_EP[i];
                    target_EP_vel[i] = 0;
                    target_EP_acc[i] = 0;
                }
                else{ // z
                    target_EP[i] = init_goal_EP[i];
                    target_EP_vel[i] = 0;
                    target_EP_acc[i] = 0;
                }
            }
 
            for(unsigned int i=6; i<9; ++i){
 
                if(i == 6){
                    target_EP[i] =  + x_init2[0] + x_init2[1]*tmp_time + x_init2[2]*pow(tmp_time,2) + x_init2[3]*pow(tmp_time,3) + x_init2[4]*pow(tmp_time,4) + x_init2[5]*pow(tmp_time,5);
                    target_EP_vel[i] = x_init2[1] + 2*x_init2[2]*pow(tmp_time,1) + 3*x_init2[3]*pow(tmp_time,2) + 4*x_init2[4]*pow(tmp_time,3) + 5*x_init2[5]*pow(tmp_time,4);
                    target_EP_acc[i] = 2*x_init2[2] + 6*x_trot_dsp1[3]*pow(tmp_time,1) + 12*x_init2[4]*pow(tmp_time,2) + 20*x_init2[5]*pow(tmp_time,3);
                }
                else if(i == 7){
                    target_EP[i] = init_goal_EP[i];
                    target_EP_vel[i] = 0;
                    target_EP_acc[i] = 0;
                }
                else{ // z
                    target_EP[i] = init_goal_EP[i];
                    target_EP_vel[i] = 0;
                    target_EP_acc[i] = 0;
                }
            }
 
            for(unsigned int i=9; i<12; ++i){
 
                if(i == 9){
                    target_EP[i] =  + x_init1[0] + x_init1[1]*tmp_time + x_init1[2]*pow(tmp_time,2) + x_init1[3]*pow(tmp_time,3) + x_init1[4]*pow(tmp_time,4) + x_init1[5]*pow(tmp_time,5);
                    target_EP_vel[i] = x_init1[1] + 2*x_init1[2]*pow(tmp_time,1) + 3*x_init1[3]*pow(tmp_time,2) + 4*x_init1[4]*pow(tmp_time,3) + 5*x_init1[5]*pow(tmp_time,4);
                    target_EP_acc[i] = 2*x_init1[2] + 6*x_init1[3]*pow(tmp_time,1) + 12*x_init1[4]*pow(tmp_time,2) + 20*x_init1[5]*pow(tmp_time,3);
                }
                else if(i == 10){
                    target_EP[i] = init_goal_EP[i];
                    target_EP_vel[i] = 0;
                    target_EP_acc[i] = 0;
                }
                else{ // z
                    if(ctc_cnt2 <= (unsigned int)((dsp_time/2)/dt)){
                        target_EP[i] = init_goal_EP[i] + z_f1[0] + z_f1[1]*tmp_time + z_f1[2]*pow(tmp_time,2) + z_f1[3]*pow(tmp_time,3) + z_f1[4]*pow(tmp_time,4) + z_f1[5]*pow(tmp_time,5);
                        target_EP_vel[i] = z_f1[1] + 2*z_f1[2]*pow(tmp_time,1) + 3*z_f1[3]*pow(tmp_time,2) + 4*z_f1[4]*pow(tmp_time,3) + 5*z_f1[5]*pow(tmp_time,4);
                        target_EP_acc[i] = 2*z_f1[2] + 6*z_f1[3]*pow(tmp_time,1) + 12*z_f1[4]*pow(tmp_time,2) + 20*z_f1[5]*pow(tmp_time,3);
                    }
                    else{
                        tmp_time2 = tmp_time - dsp_time/2;
 
                        target_EP[i] = init_goal_EP[i] + z_f2[0] + z_f2[1]*tmp_time2 + z_f2[2]*pow(tmp_time2,2) + z_f2[3]*pow(tmp_time2,3) + z_f2[4]*pow(tmp_time2,4) + z_f2[5]*pow(tmp_time2,5);
                        target_EP_vel[i] = z_f2[1] + 2*z_f2[2]*pow(tmp_time2,1) + 3*z_f2[3]*pow(tmp_time2,2) + 4*z_f2[4]*pow(tmp_time2,3) + 5*z_f2[5]*pow(tmp_time2,4);
                        target_EP_acc[i] = 2*z_f2[2] + 6*z_f2[3]*pow(tmp_time2,1) + 12*z_f2[4]*pow(tmp_time2,2) + 20*z_f2[5]*pow(tmp_time2,3);
                    }
                }
            }
 
 
            break;
 
        case INIT_STANCE_FOUR_AFTER_RRFL:
//          printf(".");
 
            tmp_time = (double)(ctc_cnt2)*dt - dsp_time;
 
//          printf("tmp_time = %f\n",tmp_time);
 
            for(unsigned int i=0; i<3; ++i){
                if(i == 0){
                    target_EP[i] =  + x_trot_fsp1[0] + x_trot_fsp1[1]*tmp_time + x_trot_fsp1[2]*pow(tmp_time,2) + x_trot_fsp1[3]*pow(tmp_time,3) + x_trot_fsp1[4]*pow(tmp_time,4) + x_trot_fsp1[5]*pow(tmp_time,5);
                    target_EP_vel[i] = x_trot_fsp1[1] + 2*x_trot_fsp1[2]*pow(tmp_time,1) + 3*x_trot_fsp1[3]*pow(tmp_time,2) + 4*x_trot_fsp1[4]*pow(tmp_time,3) + 5*x_trot_fsp1[5]*pow(tmp_time,4);
                    target_EP_acc[i] = 2*x_trot_fsp1[2] + 6*x_trot_fsp1[3]*pow(tmp_time,1) + 12*x_trot_fsp1[4]*pow(tmp_time,2) + 20*x_trot_fsp1[5]*pow(tmp_time,3);
                }
                else if(i == 1){
                    target_EP[i] = init_goal_EP[i];
                    target_EP_vel[i] = 0;
                    target_EP_acc[i] = 0;
                }
                else{ // z
                    target_EP[i] = init_goal_EP[i];
                    target_EP_vel[i] = 0;
                    target_EP_acc[i] = 0;
                }
            }
 
            for(unsigned int i=3; i<6; ++i){
                if(i == 3){
                    target_EP[i] =  + x_trot_fsp2[0] + x_trot_fsp2[1]*tmp_time + x_trot_fsp2[2]*pow(tmp_time,2) + x_trot_fsp2[3]*pow(tmp_time,3) + x_trot_fsp2[4]*pow(tmp_time,4) + x_trot_fsp2[5]*pow(tmp_time,5);
                    target_EP_vel[i] = x_trot_fsp2[1] + 2*x_trot_fsp2[2]*pow(tmp_time,1) + 3*x_trot_fsp2[3]*pow(tmp_time,2) + 4*x_trot_fsp2[4]*pow(tmp_time,3) + 5*x_trot_fsp2[5]*pow(tmp_time,4);
                    target_EP_acc[i] = 2*x_trot_fsp2[2] + 6*x_trot_fsp2[3]*pow(tmp_time,1) + 12*x_trot_fsp2[4]*pow(tmp_time,2) + 20*x_trot_fsp2[5]*pow(tmp_time,3);
                }
                else if(i == 4){
                    target_EP[i] = init_goal_EP[i];
                    target_EP_vel[i] = 0;
                    target_EP_acc[i] = 0;
                }
                else{ // z
                    target_EP[i] = init_goal_EP[i];
                    target_EP_vel[i] = 0;
                    target_EP_acc[i] = 0;
                }
            }
 
            for(unsigned int i=6; i<9; ++i){
                if(i == 6){
                    target_EP[i] =  + x_trot_fsp2[0] + x_trot_fsp2[1]*tmp_time + x_trot_fsp2[2]*pow(tmp_time,2) + x_trot_fsp2[3]*pow(tmp_time,3) + x_trot_fsp2[4]*pow(tmp_time,4) + x_trot_fsp2[5]*pow(tmp_time,5);
                    target_EP_vel[i] = x_trot_fsp2[1] + 2*x_trot_fsp2[2]*pow(tmp_time,1) + 3*x_trot_fsp2[3]*pow(tmp_time,2) + 4*x_trot_fsp2[4]*pow(tmp_time,3) + 5*x_trot_fsp2[5]*pow(tmp_time,4);
                    target_EP_acc[i] = 2*x_trot_fsp2[2] + 6*x_trot_fsp2[3]*pow(tmp_time,1) + 12*x_trot_fsp2[4]*pow(tmp_time,2) + 20*x_trot_fsp2[5]*pow(tmp_time,3);
                }
                else if(i == 7){
                    target_EP[i] = init_goal_EP[i];
                    target_EP_vel[i] = 0;
                    target_EP_acc[i] = 0;
                }
                else{ // z
                    target_EP[i] = init_goal_EP[i];
                    target_EP_vel[i] = 0;
                    target_EP_acc[i] = 0;
                }
            }
 
 
            for(unsigned int i=9; i<12; ++i){
                if(i == 9){
                    target_EP[i] =  + x_trot_fsp1[0] + x_trot_fsp1[1]*tmp_time + x_trot_fsp1[2]*pow(tmp_time,2) + x_trot_fsp1[3]*pow(tmp_time,3) + x_trot_fsp1[4]*pow(tmp_time,4) + x_trot_fsp1[5]*pow(tmp_time,5);
                    target_EP_vel[i] = x_trot_fsp1[1] + 2*x_trot_fsp1[2]*pow(tmp_time,1) + 3*x_trot_fsp1[3]*pow(tmp_time,2) + 4*x_trot_fsp1[4]*pow(tmp_time,3) + 5*x_trot_fsp1[5]*pow(tmp_time,4);
                    target_EP_acc[i] = 2*x_trot_fsp1[2] + 6*x_trot_fsp1[3]*pow(tmp_time,1) + 12*x_trot_fsp1[4]*pow(tmp_time,2) + 20*x_trot_fsp1[5]*pow(tmp_time,3);
                }
                else if(i == 10){
                    target_EP[i] = init_goal_EP[i];
                    target_EP_vel[i] = 0;
                    target_EP_acc[i] = 0;
                }
                else{ // z
                    target_EP[i] = init_goal_EP[i];
                    target_EP_vel[i] = 0;
                    target_EP_acc[i] = 0;
                }
            }
 
            break;
 
        case TROT_STANCE_RLFR:
 
            tmp_cnt2 = ctc_cnt2 - (int)(step_time/dt);
            tmp_time = (double)((ctc_cnt2)*dt)-step_time;//(double)(ctc_cnt2)*RB_CON.dt - dsp_time/2;
//
//
            for(unsigned int i=0; i<3; ++i){
 
                if(i == 0){
                    target_EP[i] =  x_trot_dsp1[0] + x_trot_dsp1[1]*tmp_time + x_trot_dsp1[2]*pow(tmp_time,2) + x_trot_dsp1[3]*pow(tmp_time,3) + x_trot_dsp1[4]*pow(tmp_time,4) + x_trot_dsp1[5]*pow(tmp_time,5);
                    target_EP_vel[i] = x_trot_dsp1[1] + 2*x_trot_dsp1[2]*pow(tmp_time,1) + 3*x_trot_dsp1[3]*pow(tmp_time,2) + 4*x_trot_dsp1[4]*pow(tmp_time,3) + 5*x_trot_dsp1[5]*pow(tmp_time,4);
                    target_EP_acc[i] = 2*x_trot_dsp1[2] + 6*x_trot_dsp1[3]*pow(tmp_time,1) + 12*x_trot_dsp1[4]*pow(tmp_time,2) + 20*x_trot_dsp1[5]*pow(tmp_time,3);
                }
                else if(i == 1){
                    target_EP[i] = init_goal_EP[i];
                    target_EP_vel[i] = 0;
                    target_EP_acc[i] = 0;
                }
                else{ // z
                    target_EP[i] = init_goal_EP[i];
                    target_EP_vel[i] = 0;
                    target_EP_acc[i] = 0;
                }
            }
 
 
            for(unsigned int i=3; i<6; ++i){
 
                if(i == 3){
                    target_EP[i] =  + x_trot_dsp2[0] + x_trot_dsp2[1]*tmp_time + x_trot_dsp2[2]*pow(tmp_time,2) + x_trot_dsp2[3]*pow(tmp_time,3) + x_trot_dsp2[4]*pow(tmp_time,4) + x_trot_dsp2[5]*pow(tmp_time,5);
                    target_EP_vel[i] = x_trot_dsp2[1] + 2*x_trot_dsp2[2]*pow(tmp_time,1) + 3*x_trot_dsp2[3]*pow(tmp_time,2) + 4*x_trot_dsp2[4]*pow(tmp_time,3) + 5*x_trot_dsp2[5]*pow(tmp_time,4);
                    target_EP_acc[i] = 2*x_trot_dsp2[2] + 6*x_trot_dsp2[3]*pow(tmp_time,1) + 12*x_trot_dsp2[4]*pow(tmp_time,2) + 20*x_trot_dsp2[5]*pow(tmp_time,3);
                }
                else if(i == 4){
                    target_EP[i] = init_goal_EP[i];
                    target_EP_vel[i] = 0;
                    target_EP_acc[i] = 0;
                }
                else{ // z
                    if(tmp_cnt2 <= (int)((dsp_time/2)/dt)){
                        target_EP[i] = init_goal_EP[i] + z_f1[0] + z_f1[1]*tmp_time + z_f1[2]*pow(tmp_time,2) + z_f1[3]*pow(tmp_time,3) + z_f1[4]*pow(tmp_time,4) + z_f1[5]*pow(tmp_time,5);
                        target_EP_vel[i] = z_f1[1] + 2*z_f1[2]*pow(tmp_time,1) + 3*z_f1[3]*pow(tmp_time,2) + 4*z_f1[4]*pow(tmp_time,3) + 5*z_f1[5]*pow(tmp_time,4);
                        target_EP_acc[i] = 2*z_f1[2] + 6*z_f1[3]*pow(tmp_time,1) + 12*z_f1[4]*pow(tmp_time,2) + 20*z_f1[5]*pow(tmp_time,3);
                    }
                    else{
                        tmp_time2 = tmp_time - dsp_time/2;
 
                        target_EP[i] = init_goal_EP[i] + z_f2[0] + z_f2[1]*tmp_time2 + z_f2[2]*pow(tmp_time2,2) + z_f2[3]*pow(tmp_time2,3) + z_f2[4]*pow(tmp_time2,4) + z_f2[5]*pow(tmp_time2,5);
                        target_EP_vel[i] = z_f2[1] + 2*z_f2[2]*pow(tmp_time2,1) + 3*z_f2[3]*pow(tmp_time2,2) + 4*z_f2[4]*pow(tmp_time2,3) + 5*z_f2[5]*pow(tmp_time2,4);
                        target_EP_acc[i] = 2*z_f2[2] + 6*z_f2[3]*pow(tmp_time2,1) + 12*z_f2[4]*pow(tmp_time2,2) + 20*z_f2[5]*pow(tmp_time2,3);
                    }
                }
            }
 
 
            for(unsigned int i=6; i<9; ++i){
 
                if(i == 6){
                    target_EP[i] =  + x_trot_dsp2[0] + x_trot_dsp2[1]*tmp_time + x_trot_dsp2[2]*pow(tmp_time,2) + x_trot_dsp2[3]*pow(tmp_time,3) + x_trot_dsp2[4]*pow(tmp_time,4) + x_trot_dsp2[5]*pow(tmp_time,5);
                    target_EP_vel[i] = x_trot_dsp2[1] + 2*x_trot_dsp2[2]*pow(tmp_time,1) + 3*x_trot_dsp2[3]*pow(tmp_time,2) + 4*x_trot_dsp2[4]*pow(tmp_time,3) + 5*x_trot_dsp2[5]*pow(tmp_time,4);
                    target_EP_acc[i] = 2*x_trot_dsp2[2] + 6*x_trot_dsp2[3]*pow(tmp_time,1) + 12*x_trot_dsp2[4]*pow(tmp_time,2) + 20*x_trot_dsp2[5]*pow(tmp_time,3);
                }
                else if(i == 7){
                    target_EP[i] = init_goal_EP[i];
                    target_EP_vel[i] = 0;
                    target_EP_acc[i] = 0;
                }
                else{ // z
                    if(tmp_cnt2 <= (int)((dsp_time/2)/dt)){
                        target_EP[i] = init_goal_EP[i] + z_f1[0] + z_f1[1]*tmp_time + z_f1[2]*pow(tmp_time,2) + z_f1[3]*pow(tmp_time,3) + z_f1[4]*pow(tmp_time,4) + z_f1[5]*pow(tmp_time,5);
                        target_EP_vel[i] = z_f1[1] + 2*z_f1[2]*pow(tmp_time,1) + 3*z_f1[3]*pow(tmp_time,2) + 4*z_f1[4]*pow(tmp_time,3) + 5*z_f1[5]*pow(tmp_time,4);
                        target_EP_acc[i] = 2*z_f1[2] + 6*z_f1[3]*pow(tmp_time,1) + 12*z_f1[4]*pow(tmp_time,2) + 20*z_f1[5]*pow(tmp_time,3);
                    }
                    else{
                        tmp_time2 = tmp_time - dsp_time/2;
 
                        target_EP[i] = init_goal_EP[i] + z_f2[0] + z_f2[1]*tmp_time2 + z_f2[2]*pow(tmp_time2,2) + z_f2[3]*pow(tmp_time2,3) + z_f2[4]*pow(tmp_time2,4) + z_f2[5]*pow(tmp_time2,5);
                        target_EP_vel[i] = z_f2[1] + 2*z_f2[2]*pow(tmp_time2,1) + 3*z_f2[3]*pow(tmp_time2,2) + 4*z_f2[4]*pow(tmp_time2,3) + 5*z_f2[5]*pow(tmp_time2,4);
                        target_EP_acc[i] = 2*z_f2[2] + 6*z_f2[3]*pow(tmp_time2,1) + 12*z_f2[4]*pow(tmp_time2,2) + 20*z_f2[5]*pow(tmp_time2,3);
                    }
                }
            }
 
 
            for(unsigned int i=9; i<12; ++i){
                if(i == 9){
                    target_EP[i] =  x_trot_dsp1[0] + x_trot_dsp1[1]*tmp_time + x_trot_dsp1[2]*pow(tmp_time,2) + x_trot_dsp1[3]*pow(tmp_time,3) + x_trot_dsp1[4]*pow(tmp_time,4) + x_trot_dsp1[5]*pow(tmp_time,5);
                    target_EP_vel[i] = x_trot_dsp1[1] + 2*x_trot_dsp1[2]*pow(tmp_time,1) + 3*x_trot_dsp1[3]*pow(tmp_time,2) + 4*x_trot_dsp1[4]*pow(tmp_time,3) + 5*x_trot_dsp1[5]*pow(tmp_time,4);
                    target_EP_acc[i] = 2*x_trot_dsp1[2] + 6*x_trot_dsp1[3]*pow(tmp_time,1) + 12*x_trot_dsp1[4]*pow(tmp_time,2) + 20*x_trot_dsp1[5]*pow(tmp_time,3);
                }
                else if(i == 10){
                    target_EP[i] = init_goal_EP[i];
                    target_EP_vel[i] = 0;
                    target_EP_acc[i] = 0;
                }
                else{ // z
                    target_EP[i] = init_goal_EP[i];
                    target_EP_vel[i] = 0;
                    target_EP_acc[i] = 0;
                }
            }
 
            break;
 
 
 
        case TROT_STANCE_FOUR_AFTER_RLFR:
//          printf(".");
 
            tmp_time = (double)(ctc_cnt2)*dt - step_time - dsp_time;
 
//          printf("tmp_time = %f\n",tmp_time);
 
            for(unsigned int i=0; i<3; ++i){
                if(i == 0){
                    target_EP[i] =  + x_trot_fsp2[0] + x_trot_fsp2[1]*tmp_time + x_trot_fsp2[2]*pow(tmp_time,2) + x_trot_fsp2[3]*pow(tmp_time,3) + x_trot_fsp2[4]*pow(tmp_time,4) + x_trot_fsp2[5]*pow(tmp_time,5);
                    target_EP_vel[i] = x_trot_fsp2[1] + 2*x_trot_fsp2[2]*pow(tmp_time,1) + 3*x_trot_fsp2[3]*pow(tmp_time,2) + 4*x_trot_fsp2[4]*pow(tmp_time,3) + 5*x_trot_fsp2[5]*pow(tmp_time,4);
                    target_EP_acc[i] = 2*x_trot_fsp2[2] + 6*x_trot_fsp2[3]*pow(tmp_time,1) + 12*x_trot_fsp2[4]*pow(tmp_time,2) + 20*x_trot_fsp2[5]*pow(tmp_time,3);
                }
                else if(i == 1){
                    target_EP[i] = init_goal_EP[i];
                    target_EP_vel[i] = 0;
                    target_EP_acc[i] = 0;
                }
                else{ // z
                    target_EP[i] = init_goal_EP[i];
                    target_EP_vel[i] = 0;
                    target_EP_acc[i] = 0;
                }
            }
 
            for(unsigned int i=3; i<6; ++i){
                if(i == 3){
                    target_EP[i] =  + x_trot_fsp1[0] + x_trot_fsp1[1]*tmp_time + x_trot_fsp1[2]*pow(tmp_time,2) + x_trot_fsp1[3]*pow(tmp_time,3) + x_trot_fsp1[4]*pow(tmp_time,4) + x_trot_fsp1[5]*pow(tmp_time,5);
                    target_EP_vel[i] = x_trot_fsp1[1] + 2*x_trot_fsp1[2]*pow(tmp_time,1) + 3*x_trot_fsp1[3]*pow(tmp_time,2) + 4*x_trot_fsp1[4]*pow(tmp_time,3) + 5*x_trot_fsp1[5]*pow(tmp_time,4);
                    target_EP_acc[i] = 2*x_trot_fsp1[2] + 6*x_trot_fsp1[3]*pow(tmp_time,1) + 12*x_trot_fsp1[4]*pow(tmp_time,2) + 20*x_trot_fsp1[5]*pow(tmp_time,3);
                }
                else if(i == 4){
                    target_EP[i] = init_goal_EP[i];
                    target_EP_vel[i] = 0;
                    target_EP_acc[i] = 0;
                }
                else{ // z
                    target_EP[i] = init_goal_EP[i];
                    target_EP_vel[i] = 0;
                    target_EP_acc[i] = 0;
                }
            }
 
            for(unsigned int i=6; i<9; ++i){
                if(i == 6){
                    target_EP[i] =  + x_trot_fsp1[0] + x_trot_fsp1[1]*tmp_time + x_trot_fsp1[2]*pow(tmp_time,2) + x_trot_fsp1[3]*pow(tmp_time,3) + x_trot_fsp1[4]*pow(tmp_time,4) + x_trot_fsp1[5]*pow(tmp_time,5);
                    target_EP_vel[i] = x_trot_fsp1[1] + 2*x_trot_fsp1[2]*pow(tmp_time,1) + 3*x_trot_fsp1[3]*pow(tmp_time,2) + 4*x_trot_fsp1[4]*pow(tmp_time,3) + 5*x_trot_fsp1[5]*pow(tmp_time,4);
                    target_EP_acc[i] = 2*x_trot_fsp1[2] + 6*x_trot_fsp1[3]*pow(tmp_time,1) + 12*x_trot_fsp1[4]*pow(tmp_time,2) + 20*x_trot_fsp1[5]*pow(tmp_time,3);
                }
                else if(i == 7){
                    target_EP[i] = init_goal_EP[i];
                    target_EP_vel[i] = 0;
                    target_EP_acc[i] = 0;
                }
                else{ // z
                    target_EP[i] = init_goal_EP[i];
                    target_EP_vel[i] = 0;
                    target_EP_acc[i] = 0;
                }
            }
 
 
            for(unsigned int i=9; i<12; ++i){
                if(i == 9){
                    target_EP[i] =  + x_trot_fsp2[0] + x_trot_fsp2[1]*tmp_time + x_trot_fsp2[2]*pow(tmp_time,2) + x_trot_fsp2[3]*pow(tmp_time,3) + x_trot_fsp2[4]*pow(tmp_time,4) + x_trot_fsp2[5]*pow(tmp_time,5);
                    target_EP_vel[i] = x_trot_fsp2[1] + 2*x_trot_fsp2[2]*pow(tmp_time,1) + 3*x_trot_fsp2[3]*pow(tmp_time,2) + 4*x_trot_fsp2[4]*pow(tmp_time,3) + 5*x_trot_fsp2[5]*pow(tmp_time,4);
                    target_EP_acc[i] = 2*x_trot_fsp2[2] + 6*x_trot_fsp2[3]*pow(tmp_time,1) + 12*x_trot_fsp2[4]*pow(tmp_time,2) + 20*x_trot_fsp2[5]*pow(tmp_time,3);
                }
                else if(i == 10){
                    target_EP[i] = init_goal_EP[i];
                    target_EP_vel[i] = 0;
                    target_EP_acc[i] = 0;
                }
                else{ // z
                    target_EP[i] = init_goal_EP[i];
                    target_EP_vel[i] = 0;
                    target_EP_acc[i] = 0;
                }
            }
 
            break;
 
 
 
        case TROT_STANCE_RRFL:
 
            tmp_cnt2 = ctc_cnt2 - (int)(step_time*2/dt);
            tmp_time = (double)((ctc_cnt2)*dt)-step_time*2;//(double)(ctc_cnt2)*RB_CON.dt - dsp_time/2;
//
//
            for(unsigned int i=0; i<3; ++i){
 
                if(i == 0){
                    target_EP[i] =  + x_trot_dsp2[0] + x_trot_dsp2[1]*tmp_time + x_trot_dsp2[2]*pow(tmp_time,2) + x_trot_dsp2[3]*pow(tmp_time,3) + x_trot_dsp2[4]*pow(tmp_time,4) + x_trot_dsp2[5]*pow(tmp_time,5);
                    target_EP_vel[i] = x_trot_dsp2[1] + 2*x_trot_dsp2[2]*pow(tmp_time,1) + 3*x_trot_dsp2[3]*pow(tmp_time,2) + 4*x_trot_dsp2[4]*pow(tmp_time,3) + 5*x_trot_dsp2[5]*pow(tmp_time,4);
                    target_EP_acc[i] = 2*x_trot_dsp2[2] + 6*x_trot_dsp2[3]*pow(tmp_time,1) + 12*x_trot_dsp2[4]*pow(tmp_time,2) + 20*x_trot_dsp2[5]*pow(tmp_time,3);
                }
                else if(i == 1){
                    target_EP[i] = init_goal_EP[i];
                    target_EP_vel[i] = 0;
                    target_EP_acc[i] = 0;
                }
                else{ // z
 
                    if(tmp_cnt2 <= (int)((dsp_time/2)/dt)){
                        target_EP[i] = init_goal_EP[i] + z_f1[0] + z_f1[1]*tmp_time + z_f1[2]*pow(tmp_time,2) + z_f1[3]*pow(tmp_time,3) + z_f1[4]*pow(tmp_time,4) + z_f1[5]*pow(tmp_time,5);
                        target_EP_vel[i] = z_f1[1] + 2*z_f1[2]*pow(tmp_time,1) + 3*z_f1[3]*pow(tmp_time,2) + 4*z_f1[4]*pow(tmp_time,3) + 5*z_f1[5]*pow(tmp_time,4);
                        target_EP_acc[i] = 2*z_f1[2] + 6*z_f1[3]*pow(tmp_time,1) + 12*z_f1[4]*pow(tmp_time,2) + 20*z_f1[5]*pow(tmp_time,3);
                    }
                    else{
                        tmp_time2 = tmp_time - dsp_time/2;
 
                        target_EP[i] = init_goal_EP[i] + z_f2[0] + z_f2[1]*tmp_time2 + z_f2[2]*pow(tmp_time2,2) + z_f2[3]*pow(tmp_time2,3) + z_f2[4]*pow(tmp_time2,4) + z_f2[5]*pow(tmp_time2,5);
                        target_EP_vel[i] = z_f2[1] + 2*z_f2[2]*pow(tmp_time2,1) + 3*z_f2[3]*pow(tmp_time2,2) + 4*z_f2[4]*pow(tmp_time2,3) + 5*z_f2[5]*pow(tmp_time2,4);
                        target_EP_acc[i] = 2*z_f2[2] + 6*z_f2[3]*pow(tmp_time2,1) + 12*z_f2[4]*pow(tmp_time2,2) + 20*z_f2[5]*pow(tmp_time2,3);
                    }
 
                }
            }
 
 
            for(unsigned int i=3; i<6; ++i){
 
                if(i == 3){
                    target_EP[i] =  x_trot_dsp1[0] + x_trot_dsp1[1]*tmp_time + x_trot_dsp1[2]*pow(tmp_time,2) + x_trot_dsp1[3]*pow(tmp_time,3) + x_trot_dsp1[4]*pow(tmp_time,4) + x_trot_dsp1[5]*pow(tmp_time,5);
                    target_EP_vel[i] = x_trot_dsp1[1] + 2*x_trot_dsp1[2]*pow(tmp_time,1) + 3*x_trot_dsp1[3]*pow(tmp_time,2) + 4*x_trot_dsp1[4]*pow(tmp_time,3) + 5*x_trot_dsp1[5]*pow(tmp_time,4);
                    target_EP_acc[i] = 2*x_trot_dsp1[2] + 6*x_trot_dsp1[3]*pow(tmp_time,1) + 12*x_trot_dsp1[4]*pow(tmp_time,2) + 20*x_trot_dsp1[5]*pow(tmp_time,3);
                }
                else if(i == 4){
                    target_EP[i] = init_goal_EP[i];
                    target_EP_vel[i] = 0;
                    target_EP_acc[i] = 0;
                }
                else{ // z
                    target_EP[i] = init_goal_EP[i];
                    target_EP_vel[i] = 0;
                    target_EP_acc[i] = 0;
                }
            }
 
 
            for(unsigned int i=6; i<9; ++i){
 
                if(i == 6){
                    target_EP[i] =  x_trot_dsp1[0] + x_trot_dsp1[1]*tmp_time + x_trot_dsp1[2]*pow(tmp_time,2) + x_trot_dsp1[3]*pow(tmp_time,3) + x_trot_dsp1[4]*pow(tmp_time,4) + x_trot_dsp1[5]*pow(tmp_time,5);
                    target_EP_vel[i] = x_trot_dsp1[1] + 2*x_trot_dsp1[2]*pow(tmp_time,1) + 3*x_trot_dsp1[3]*pow(tmp_time,2) + 4*x_trot_dsp1[4]*pow(tmp_time,3) + 5*x_trot_dsp1[5]*pow(tmp_time,4);
                    target_EP_acc[i] = 2*x_trot_dsp1[2] + 6*x_trot_dsp1[3]*pow(tmp_time,1) + 12*x_trot_dsp1[4]*pow(tmp_time,2) + 20*x_trot_dsp1[5]*pow(tmp_time,3);
                }
                else if(i == 7){
                    target_EP[i] = init_goal_EP[i];
                    target_EP_vel[i] = 0;
                    target_EP_acc[i] = 0;
                }
                else{ // z
                    target_EP[i] = init_goal_EP[i];
                    target_EP_vel[i] = 0;
                    target_EP_acc[i] = 0;
                }
            }
 
 
            for(unsigned int i=9; i<12; ++i){
                if(i == 9){
                    target_EP[i] =  + x_trot_dsp2[0] + x_trot_dsp2[1]*tmp_time + x_trot_dsp2[2]*pow(tmp_time,2) + x_trot_dsp2[3]*pow(tmp_time,3) + x_trot_dsp2[4]*pow(tmp_time,4) + x_trot_dsp2[5]*pow(tmp_time,5);
                    target_EP_vel[i] = x_trot_dsp2[1] + 2*x_trot_dsp2[2]*pow(tmp_time,1) + 3*x_trot_dsp2[3]*pow(tmp_time,2) + 4*x_trot_dsp2[4]*pow(tmp_time,3) + 5*x_trot_dsp2[5]*pow(tmp_time,4);
                    target_EP_acc[i] = 2*x_trot_dsp2[2] + 6*x_trot_dsp2[3]*pow(tmp_time,1) + 12*x_trot_dsp2[4]*pow(tmp_time,2) + 20*x_trot_dsp2[5]*pow(tmp_time,3);
                }
                else if(i == 10){
                    target_EP[i] = init_goal_EP[i];
                    target_EP_vel[i] = 0;
                    target_EP_acc[i] = 0;
                }
                else{ // z
                    if(tmp_cnt2 <= (int)((dsp_time/2)/dt)){
                        target_EP[i] = init_goal_EP[i] + z_f1[0] + z_f1[1]*tmp_time + z_f1[2]*pow(tmp_time,2) + z_f1[3]*pow(tmp_time,3) + z_f1[4]*pow(tmp_time,4) + z_f1[5]*pow(tmp_time,5);
                        target_EP_vel[i] = z_f1[1] + 2*z_f1[2]*pow(tmp_time,1) + 3*z_f1[3]*pow(tmp_time,2) + 4*z_f1[4]*pow(tmp_time,3) + 5*z_f1[5]*pow(tmp_time,4);
                        target_EP_acc[i] = 2*z_f1[2] + 6*z_f1[3]*pow(tmp_time,1) + 12*z_f1[4]*pow(tmp_time,2) + 20*z_f1[5]*pow(tmp_time,3);
                    }
                    else{
                        tmp_time2 = tmp_time - dsp_time/2;
 
                        target_EP[i] = init_goal_EP[i] + z_f2[0] + z_f2[1]*tmp_time2 + z_f2[2]*pow(tmp_time2,2) + z_f2[3]*pow(tmp_time2,3) + z_f2[4]*pow(tmp_time2,4) + z_f2[5]*pow(tmp_time2,5);
                        target_EP_vel[i] = z_f2[1] + 2*z_f2[2]*pow(tmp_time2,1) + 3*z_f2[3]*pow(tmp_time2,2) + 4*z_f2[4]*pow(tmp_time2,3) + 5*z_f2[5]*pow(tmp_time2,4);
                        target_EP_acc[i] = 2*z_f2[2] + 6*z_f2[3]*pow(tmp_time2,1) + 12*z_f2[4]*pow(tmp_time2,2) + 20*z_f2[5]*pow(tmp_time2,3);
                    }
                }
            }
 
            break;
 
 
 
        case TROT_STANCE_FOUR_AFTER_RRFL:
//          printf(".");
 
            tmp_time = (double)(ctc_cnt2)*dt -step_time*2 - dsp_time;
 
//          printf("tmp_time = %f\n",tmp_time);
 
            for(unsigned int i=0; i<3; ++i){
                if(i == 0){
                    target_EP[i] =  + x_trot_fsp1[0] + x_trot_fsp1[1]*tmp_time + x_trot_fsp1[2]*pow(tmp_time,2) + x_trot_fsp1[3]*pow(tmp_time,3) + x_trot_fsp1[4]*pow(tmp_time,4) + x_trot_fsp1[5]*pow(tmp_time,5);
                    target_EP_vel[i] = x_trot_fsp1[1] + 2*x_trot_fsp1[2]*pow(tmp_time,1) + 3*x_trot_fsp1[3]*pow(tmp_time,2) + 4*x_trot_fsp1[4]*pow(tmp_time,3) + 5*x_trot_fsp1[5]*pow(tmp_time,4);
                    target_EP_acc[i] = 2*x_trot_fsp1[2] + 6*x_trot_fsp1[3]*pow(tmp_time,1) + 12*x_trot_fsp1[4]*pow(tmp_time,2) + 20*x_trot_fsp1[5]*pow(tmp_time,3);
                }
                else if(i == 1){
                    target_EP[i] = init_goal_EP[i];
                    target_EP_vel[i] = 0;
                    target_EP_acc[i] = 0;
                }
                else{ // z
                    target_EP[i] = init_goal_EP[i];
                    target_EP_vel[i] = 0;
                    target_EP_acc[i] = 0;
                }
            }
 
            for(unsigned int i=3; i<6; ++i){
                if(i == 3){
                    target_EP[i] =  + x_trot_fsp2[0] + x_trot_fsp2[1]*tmp_time + x_trot_fsp2[2]*pow(tmp_time,2) + x_trot_fsp2[3]*pow(tmp_time,3) + x_trot_fsp2[4]*pow(tmp_time,4) + x_trot_fsp2[5]*pow(tmp_time,5);
                    target_EP_vel[i] = x_trot_fsp2[1] + 2*x_trot_fsp2[2]*pow(tmp_time,1) + 3*x_trot_fsp2[3]*pow(tmp_time,2) + 4*x_trot_fsp2[4]*pow(tmp_time,3) + 5*x_trot_fsp2[5]*pow(tmp_time,4);
                    target_EP_acc[i] = 2*x_trot_fsp2[2] + 6*x_trot_fsp2[3]*pow(tmp_time,1) + 12*x_trot_fsp2[4]*pow(tmp_time,2) + 20*x_trot_fsp2[5]*pow(tmp_time,3);
                }
                else if(i == 4){
                    target_EP[i] = init_goal_EP[i];
                    target_EP_vel[i] = 0;
                    target_EP_acc[i] = 0;
                }
                else{ // z
                    target_EP[i] = init_goal_EP[i];
                    target_EP_vel[i] = 0;
                    target_EP_acc[i] = 0;
                }
            }
 
            for(unsigned int i=6; i<9; ++i){
                if(i == 6){
                    target_EP[i] =  + x_trot_fsp2[0] + x_trot_fsp2[1]*tmp_time + x_trot_fsp2[2]*pow(tmp_time,2) + x_trot_fsp2[3]*pow(tmp_time,3) + x_trot_fsp2[4]*pow(tmp_time,4) + x_trot_fsp2[5]*pow(tmp_time,5);
                    target_EP_vel[i] = x_trot_fsp2[1] + 2*x_trot_fsp2[2]*pow(tmp_time,1) + 3*x_trot_fsp2[3]*pow(tmp_time,2) + 4*x_trot_fsp2[4]*pow(tmp_time,3) + 5*x_trot_fsp2[5]*pow(tmp_time,4);
                    target_EP_acc[i] = 2*x_trot_fsp2[2] + 6*x_trot_fsp2[3]*pow(tmp_time,1) + 12*x_trot_fsp2[4]*pow(tmp_time,2) + 20*x_trot_fsp2[5]*pow(tmp_time,3);
                }
                else if(i == 7){
                    target_EP[i] = init_goal_EP[i];
                    target_EP_vel[i] = 0;
                    target_EP_acc[i] = 0;
                }
                else{ // z
                    target_EP[i] = init_goal_EP[i];
                    target_EP_vel[i] = 0;
                    target_EP_acc[i] = 0;
                }
            }
 
 
            for(unsigned int i=9; i<12; ++i){
                if(i == 9){
                    target_EP[i] =  + x_trot_fsp1[0] + x_trot_fsp1[1]*tmp_time + x_trot_fsp1[2]*pow(tmp_time,2) + x_trot_fsp1[3]*pow(tmp_time,3) + x_trot_fsp1[4]*pow(tmp_time,4) + x_trot_fsp1[5]*pow(tmp_time,5);
                    target_EP_vel[i] = x_trot_fsp1[1] + 2*x_trot_fsp1[2]*pow(tmp_time,1) + 3*x_trot_fsp1[3]*pow(tmp_time,2) + 4*x_trot_fsp1[4]*pow(tmp_time,3) + 5*x_trot_fsp1[5]*pow(tmp_time,4);
                    target_EP_acc[i] = 2*x_trot_fsp1[2] + 6*x_trot_fsp1[3]*pow(tmp_time,1) + 12*x_trot_fsp1[4]*pow(tmp_time,2) + 20*x_trot_fsp1[5]*pow(tmp_time,3);
                }
                else if(i == 10){
                    target_EP[i] = init_goal_EP[i];
                    target_EP_vel[i] = 0;
                    target_EP_acc[i] = 0;
                }
                else{ // z
                    target_EP[i] = init_goal_EP[i];
                    target_EP_vel[i] = 0;
                    target_EP_acc[i] = 0;
                }
            }
 
            break;
 
 
 
 
        case FINAL_STANCE_RLFR:
 
            tmp_cnt2 = ctc_cnt2 - (int)(step_time*3/dt);
            tmp_time = (double)((ctc_cnt2)*dt)-step_time*3;//(double)(ctc_cnt2)*RB_CON.dt - dsp_time/2;
//
//
            for(unsigned int i=0; i<3; ++i){
 
                if(i == 0){
                    target_EP[i] =  x_final1[0] + x_final1[1]*tmp_time + x_final1[2]*pow(tmp_time,2) + x_final1[3]*pow(tmp_time,3) + x_final1[4]*pow(tmp_time,4) + x_final1[5]*pow(tmp_time,5);
                    target_EP_vel[i] = x_final1[1] + 2*x_final1[2]*pow(tmp_time,1) + 3*x_final1[3]*pow(tmp_time,2) + 4*x_final1[4]*pow(tmp_time,3) + 5*x_final1[5]*pow(tmp_time,4);
                    target_EP_acc[i] = 2*x_final1[2] + 6*x_final1[3]*pow(tmp_time,1) + 12*x_final1[4]*pow(tmp_time,2) + 20*x_final1[5]*pow(tmp_time,3);
                }
                else if(i == 1){
                    target_EP[i] = init_goal_EP[i];
                    target_EP_vel[i] = 0;
                    target_EP_acc[i] = 0;
                }
                else{ // z
                    target_EP[i] = init_goal_EP[i];
                    target_EP_vel[i] = 0;
                    target_EP_acc[i] = 0;
                }
            }
 
 
            for(unsigned int i=3; i<6; ++i){
 
                if(i == 3){
                    target_EP[i] =  + x_final2[0] + x_final2[1]*tmp_time + x_final2[2]*pow(tmp_time,2) + x_final2[3]*pow(tmp_time,3) + x_final2[4]*pow(tmp_time,4) + x_final2[5]*pow(tmp_time,5);
                    target_EP_vel[i] = x_final2[1] + 2*x_final2[2]*pow(tmp_time,1) + 3*x_final2[3]*pow(tmp_time,2) + 4*x_final2[4]*pow(tmp_time,3) + 5*x_final2[5]*pow(tmp_time,4);
                    target_EP_acc[i] = 2*x_final2[2] + 6*x_final2[3]*pow(tmp_time,1) + 12*x_final2[4]*pow(tmp_time,2) + 20*x_final2[5]*pow(tmp_time,3);
                }
                else if(i == 4){
                    target_EP[i] = init_goal_EP[i];
                    target_EP_vel[i] = 0;
                    target_EP_acc[i] = 0;
                }
                else{ // z
                    if(tmp_cnt2 <= (int)((dsp_time/2)/dt)){
                        target_EP[i] = init_goal_EP[i] + z_f1[0] + z_f1[1]*tmp_time + z_f1[2]*pow(tmp_time,2) + z_f1[3]*pow(tmp_time,3) + z_f1[4]*pow(tmp_time,4) + z_f1[5]*pow(tmp_time,5);
                        target_EP_vel[i] = z_f1[1] + 2*z_f1[2]*pow(tmp_time,1) + 3*z_f1[3]*pow(tmp_time,2) + 4*z_f1[4]*pow(tmp_time,3) + 5*z_f1[5]*pow(tmp_time,4);
                        target_EP_acc[i] = 2*z_f1[2] + 6*z_f1[3]*pow(tmp_time,1) + 12*z_f1[4]*pow(tmp_time,2) + 20*z_f1[5]*pow(tmp_time,3);
                    }
                    else{
                        tmp_time2 = tmp_time - dsp_time/2;
 
                        target_EP[i] = init_goal_EP[i] + z_f2[0] + z_f2[1]*tmp_time2 + z_f2[2]*pow(tmp_time2,2) + z_f2[3]*pow(tmp_time2,3) + z_f2[4]*pow(tmp_time2,4) + z_f2[5]*pow(tmp_time2,5);
                        target_EP_vel[i] = z_f2[1] + 2*z_f2[2]*pow(tmp_time2,1) + 3*z_f2[3]*pow(tmp_time2,2) + 4*z_f2[4]*pow(tmp_time2,3) + 5*z_f2[5]*pow(tmp_time2,4);
                        target_EP_acc[i] = 2*z_f2[2] + 6*z_f2[3]*pow(tmp_time2,1) + 12*z_f2[4]*pow(tmp_time2,2) + 20*z_f2[5]*pow(tmp_time2,3);
                    }
                }
            }
 
 
            for(unsigned int i=6; i<9; ++i){
 
                if(i == 6){
                    target_EP[i] =  + x_final2[0] + x_final2[1]*tmp_time + x_final2[2]*pow(tmp_time,2) + x_final2[3]*pow(tmp_time,3) + x_final2[4]*pow(tmp_time,4) + x_final2[5]*pow(tmp_time,5);
                    target_EP_vel[i] = x_final2[1] + 2*x_final2[2]*pow(tmp_time,1) + 3*x_final2[3]*pow(tmp_time,2) + 4*x_final2[4]*pow(tmp_time,3) + 5*x_final2[5]*pow(tmp_time,4);
                    target_EP_acc[i] = 2*x_final2[2] + 6*x_final2[3]*pow(tmp_time,1) + 12*x_final2[4]*pow(tmp_time,2) + 20*x_final2[5]*pow(tmp_time,3);
                }
                else if(i == 7){
                    target_EP[i] = init_goal_EP[i];
                    target_EP_vel[i] = 0;
                    target_EP_acc[i] = 0;
                }
                else{ // z
                    if(tmp_cnt2 <= (int)((dsp_time/2)/dt)){
                        target_EP[i] = init_goal_EP[i] + z_f1[0] + z_f1[1]*tmp_time + z_f1[2]*pow(tmp_time,2) + z_f1[3]*pow(tmp_time,3) + z_f1[4]*pow(tmp_time,4) + z_f1[5]*pow(tmp_time,5);
                        target_EP_vel[i] = z_f1[1] + 2*z_f1[2]*pow(tmp_time,1) + 3*z_f1[3]*pow(tmp_time,2) + 4*z_f1[4]*pow(tmp_time,3) + 5*z_f1[5]*pow(tmp_time,4);
                        target_EP_acc[i] = 2*z_f1[2] + 6*z_f1[3]*pow(tmp_time,1) + 12*z_f1[4]*pow(tmp_time,2) + 20*z_f1[5]*pow(tmp_time,3);
                    }
                    else{
                        tmp_time2 = tmp_time - dsp_time/2;
 
                        target_EP[i] = init_goal_EP[i] + z_f2[0] + z_f2[1]*tmp_time2 + z_f2[2]*pow(tmp_time2,2) + z_f2[3]*pow(tmp_time2,3) + z_f2[4]*pow(tmp_time2,4) + z_f2[5]*pow(tmp_time2,5);
                        target_EP_vel[i] = z_f2[1] + 2*z_f2[2]*pow(tmp_time2,1) + 3*z_f2[3]*pow(tmp_time2,2) + 4*z_f2[4]*pow(tmp_time2,3) + 5*z_f2[5]*pow(tmp_time2,4);
                        target_EP_acc[i] = 2*z_f2[2] + 6*z_f2[3]*pow(tmp_time2,1) + 12*z_f2[4]*pow(tmp_time2,2) + 20*z_f2[5]*pow(tmp_time2,3);
                    }
                }
            }
 
 
            for(unsigned int i=9; i<12; ++i){
                if(i == 9){
                    target_EP[i] =  x_final1[0] + x_final1[1]*tmp_time + x_final1[2]*pow(tmp_time,2) + x_final1[3]*pow(tmp_time,3) + x_final1[4]*pow(tmp_time,4) + x_final1[5]*pow(tmp_time,5);
                    target_EP_vel[i] = x_final1[1] + 2*x_final1[2]*pow(tmp_time,1) + 3*x_final1[3]*pow(tmp_time,2) + 4*x_final1[4]*pow(tmp_time,3) + 5*x_final1[5]*pow(tmp_time,4);
                    target_EP_acc[i] = 2*x_final1[2] + 6*x_final1[3]*pow(tmp_time,1) + 12*x_final1[4]*pow(tmp_time,2) + 20*x_final1[5]*pow(tmp_time,3);
                }
                else if(i == 10){
                    target_EP[i] = init_goal_EP[i];
                    target_EP_vel[i] = 0;
                    target_EP_acc[i] = 0;
                }
                else{ // z
                    target_EP[i] = init_goal_EP[i];
                    target_EP_vel[i] = 0;
                    target_EP_acc[i] = 0;
                }
            }
 
            break;
 
        case FINAL_STANCE_FOUR:
 
            break;
    }
 
 
    if(ctc_cnt2 == (unsigned int)((step_time*3)/dt)){
        step_num = 10;
 
        if(step_cnt < step_num-1){
            ctc_cnt2 = (unsigned int)(step_time/dt);
        }
 
        step_cnt++;
 
        printf("\n===================== step_num = %d, step_cnt = %d ======================\n",step_num,step_cnt);
 
    }
  
    target_pos[6] = 0;
 
}



void CRobot::Traj_gen(void){
    
// ===================== Vertical Foot Trajectory Generation ==================== //
    static double init_x[3] = {0,0,0};
    static double final_x[3] = {0,0,0};
    static double _t = 0;
    static double _out[6] = {0,0,0,0,0,0};
    static double foot_height = 0.03;
    static double sf_z_length = 0.04;
    double landing_vel = 0.3;
    double take_off_speed = 0.1;
 
    to_height = 0 - take_off_speed*tf + 9.81/2.0*tf*tf;
    
    cout << "to_height = " << to_height << endl;
    
//  double dsp_time = 0.25, fsp_time = 0.05;
 
//    tf = dsp_time - ts;
    
    init_x[0] = 0;
    init_x[1] = 0;
    init_x[2] = 0;
 
    final_x[0] = foot_height;
    final_x[1] = 0;
    final_x[2] = 0;
 
    _t = ts;
 
    coefficient_5thPoly(init_x, final_x, _t, _out);
 
    z_f1[0] = _out[0];
    z_f1[1] = _out[1];
    z_f1[2] = _out[2];
    z_f1[3] = _out[3];
    z_f1[4] = _out[4];
    z_f1[5] = _out[5];
 
    // -------------------------------------------------
    
    init_x[0] = foot_height;
    init_x[1] = 0;
    init_x[2] = 0;
 
    final_x[0] = -sf_z_length;
    final_x[1] = landing_vel;
    final_x[2] = 9.81;
 
    _t = tf;
 
    coefficient_5thPoly(init_x, final_x, _t, _out);
 
    z_f2[0] = _out[0];
    z_f2[1] = _out[1];
    z_f2[2] = _out[2];
    z_f2[3] = _out[3];
    z_f2[4] = _out[4];
    z_f2[5] = _out[5];
    
    // -------------------------------------------------
    
    init_x[0] = -sf_z_length;
    init_x[1] = landing_vel;
    init_x[2] = 9.81;
 
    final_x[0] = -to_height;
    final_x[1] = -take_off_speed;
    final_x[2] = 9.81;
 
    _t = ts;
 
    coefficient_5thPoly(init_x, final_x, _t, _out);
 
    z_f3[0] = _out[0];
    z_f3[1] = _out[1];
    z_f3[2] = _out[2];
    z_f3[3] = _out[3];
    z_f3[4] = _out[4];
    z_f3[5] = _out[5];
    
    // -------------------------------------------------
    
//    init_x[0] = -sf_z_length;
//    init_x[1] = 0;
//    init_x[2] = 0;
// 
//    final_x[0] = -to_height;
//    final_x[1] = -take_off_speed;
//    final_x[2] = 9.81;
// 
//    _t = ts;
// 
//    coefficient_5thPoly(init_x, final_x, _t, _out);
// 
//    z_f4[0] = _out[0];
//    z_f4[1] = _out[1];
//    z_f4[2] = _out[2];
//    z_f4[3] = _out[3];
//    z_f4[4] = _out[4];
//    z_f4[5] = _out[5];
    
    // -------------------------------------------------
    // -------------------------------------------------

    
    init_x[0] = 0;
    init_x[1] = 0;
    init_x[2] = 0;
 
    final_x[0] = -to_height;
    final_x[1] = -take_off_speed;
    final_x[2] = 9.81;
 
    _t = ts;//dsp_time/2;
 
    coefficient_5thPoly(init_x, final_x, _t, _out);
 
    z_s1[0] = _out[0];
    z_s1[1] = _out[1];
    z_s1[2] = _out[2];
    z_s1[3] = _out[3];
    z_s1[4] = _out[4];
    z_s1[5] = _out[5];
    
    
    // -------------------------------------------------
 
    init_x[0] = -to_height;
    init_x[1] = -take_off_speed;
    init_x[2] = 9.81;
 
    final_x[0] = 0;
    final_x[1] = 0;
    final_x[2] = 0;
 
    _t = tf;//dsp_time/2;
 
    coefficient_5thPoly(init_x, final_x, _t, _out);
 
    z_s2[0] = _out[0];
    z_s2[1] = _out[1];
    z_s2[2] = _out[2];
    z_s2[3] = _out[3];
    z_s2[4] = _out[4];
    z_s2[5] = _out[5];
    
    // -------------------------------------------------
 
//    init_x[0] = 0;//final_x[0];
//    init_x[1] = landing_vel;//final_x[1];
//    init_x[2] = 9.81;//final_x[2];
// 
//    final_x[0] = -to_height;
//    final_x[1] = -take_off_speed;
//    final_x[2] = 9.81;
// 
//    _t = ts;//dsp_time/2;
// 
//    coefficient_5thPoly(init_x, final_x, _t, _out);
// 
//    z_s3[0] = _out[0];
//    z_s3[1] = _out[1];
//    z_s3[2] = _out[2];
//    z_s3[3] = _out[3];
//    z_s3[4] = _out[4];
//    z_s3[5] = _out[5];
    
    // -------------------------------------------------
 
    init_x[0] = foot_height;
    init_x[1] = 0;
    init_x[2] = 0;
 
    final_x[0] = 0;
    final_x[1] = 0;
    final_x[2] = 0;
 
    _t = tf;//dsp_time/2;
 
    coefficient_5thPoly(init_x, final_x, _t, _out);
 
    z_final1[0] = _out[0];
    z_final1[1] = _out[1];
    z_final1[2] = _out[2];
    z_final1[3] = _out[3];
    z_final1[4] = _out[4];
    z_final1[5] = _out[5];
    
    // -------------------------------------------------
 
//    init_x[0] = final_x[0];
//    init_x[1] = final_x[1];
//    init_x[2] = final_x[2];
// 
//    final_x[0] = 0;
//    final_x[1] = 0;
//    final_x[2] = 0;
// 
//    _t = tf;//dsp_time/2;
// 
//    coefficient_5thPoly(init_x, final_x, _t, _out);
// 
//    z_final2[0] = _out[0];
//    z_final2[1] = _out[1];
//    z_final2[2] = _out[2];
//    z_final2[3] = _out[3];
//    z_final2[4] = _out[4];
//    z_final2[5] = _out[5];

    // ===================== Vertical Foot Trajectory Generation End ==================== //
 
 
    // ===================== Horizontal Foot Trajectory Generation ==================== //
 
//  double step_length = 0.05;
    double moving_speed = 0.0; // [m/s]
    double x_step = step_time*moving_speed;
    double x_fsp = fsp_time*moving_speed;
 
    init_x[0] = 0;
    init_x[1] = 0;
    init_x[2] = 0;
 
    final_x[0] = x_step + x_fsp;//step_length;
    final_x[1] = -moving_speed;
    final_x[2] = 0;
 
    _t = dsp_time;
 
    coefficient_5thPoly(init_x, final_x, _t, _out);
 
    x_init1[0] = _out[0];
    x_init1[1] = _out[1];
    x_init1[2] = _out[2];
    x_init1[3] = _out[3];
    x_init1[4] = _out[4];
    x_init1[5] = _out[5];
 
    // --------------------------------------
 
    init_x[0] = final_x[0];
    init_x[1] = final_x[1];
    init_x[2] = final_x[2];
 
    final_x[0] = x_step - x_fsp;
    final_x[1] = -moving_speed;
    final_x[2] = 0;
 
    _t = fsp_time;
 
    coefficient_5thPoly(init_x, final_x, _t, _out);
 
    x_trot_fsp1[0] = _out[0];
    x_trot_fsp1[1] = _out[1];
    x_trot_fsp1[2] = _out[2];
    x_trot_fsp1[3] = _out[3];
    x_trot_fsp1[4] = _out[4];
    x_trot_fsp1[5] = _out[5];
 
 
    // --------------------------------------
 
    init_x[0] = 0;
    init_x[1] = 0;
    init_x[2] = 0;
 
    final_x[0] = -x_step + x_fsp;//-step_length;
    final_x[1] = -moving_speed;
    final_x[2] = 0;
 
    _t = dsp_time;
 
    coefficient_5thPoly(init_x, final_x, _t, _out);
 
    x_init2[0] = _out[0];
    x_init2[1] = _out[1];
    x_init2[2] = _out[2];
    x_init2[3] = _out[3];
    x_init2[4] = _out[4];
    x_init2[5] = _out[5];
 
 
    // --------------------------------------
 
    init_x[0] = final_x[0];
    init_x[1] = final_x[1];
    init_x[2] = final_x[2];
 
    final_x[0] = -x_step - x_fsp;//-step_length;
    final_x[1] = -moving_speed;
    final_x[2] = 0;
 
    _t = fsp_time;
 
    coefficient_5thPoly(init_x, final_x, _t, _out);
 
    x_trot_fsp2[0] = _out[0];
    x_trot_fsp2[1] = _out[1];
    x_trot_fsp2[2] = _out[2];
    x_trot_fsp2[3] = _out[3];
    x_trot_fsp2[4] = _out[4];
    x_trot_fsp2[5] = _out[5];
 
 
    // --------------------------------------
 
    init_x[0] = x_step - x_fsp;
    init_x[1] = -moving_speed;
    init_x[2] = 0;
 
    final_x[0] = -x_step + x_fsp;
    final_x[1] = -moving_speed;
    final_x[2] = 0;
 
    _t = dsp_time;
 
    coefficient_5thPoly(init_x, final_x, _t, _out);
 
    x_trot_dsp1[0] = _out[0];
    x_trot_dsp1[1] = _out[1];
    x_trot_dsp1[2] = _out[2];
    x_trot_dsp1[3] = _out[3];
    x_trot_dsp1[4] = _out[4];
    x_trot_dsp1[5] = _out[5];
 
 
    // --------------------------------------
 
    init_x[0] = -x_step - x_fsp;
    init_x[1] = -moving_speed;
    init_x[2] = 0;
 
    final_x[0] = x_step + x_fsp;
    final_x[1] = -moving_speed;
    final_x[2] = 0;
 
    _t = dsp_time;
 
    coefficient_5thPoly(init_x, final_x, _t, _out);
 
    x_trot_dsp2[0] = _out[0];
    x_trot_dsp2[1] = _out[1];
    x_trot_dsp2[2] = _out[2];
    x_trot_dsp2[3] = _out[3];
    x_trot_dsp2[4] = _out[4];
    x_trot_dsp2[5] = _out[5];
 
    // --------------------------------------
 
    init_x[0] = x_step - x_fsp;
    init_x[1] = -moving_speed;
    init_x[2] = 0;
 
    final_x[0] = 0;
    final_x[1] = 0;
    final_x[2] = 0;
 
    _t = dsp_time;
 
    coefficient_5thPoly(init_x, final_x, _t, _out);
 
    x_final1[0] = _out[0];
    x_final1[1] = _out[1];
    x_final1[2] = _out[2];
    x_final1[3] = _out[3];
    x_final1[4] = _out[4];
    x_final1[5] = _out[5];
 
    // --------------------------------------
 
    init_x[0] = -x_step - x_fsp;;
    init_x[1] = -moving_speed;
    init_x[2] = 0;
 
    final_x[0] = 0;
    final_x[1] = 0;
    final_x[2] = 0;
 
    _t = dsp_time;
 
    coefficient_5thPoly(init_x, final_x, _t, _out);
 
    x_final2[0] = _out[0];
    x_final2[1] = _out[1];
    x_final2[2] = _out[2];
    x_final2[3] = _out[3];
    x_final2[4] = _out[4];
    x_final2[5] = _out[5];
 
 
    // ===================== Horizontal Foot Trajectory Generation End ==================== //
 
 
 
 
}

void CRobot::ballistics(double flight_time, double landing_height, double take_off_speed)
{
  double g = 9.81;
  double time1 = (take_off_speed)/9.81;
  double time2 = flight_time - time1;//(flight_time - 0.01) - time1; //- 0.012
  double top_height = landing_height + 0.5*g*pow(time2,2);
  to_height = top_height + 0.5*g*pow(time1,2) - take_off_speed*time1;
}


void CRobot::coefficient_5thPoly(double *init_x, double *final_x, double tf, double *output)
{
//   MatrixXd R(6,1), A(6,6), P(6,1);
 
   R << init_x[0], final_x[0], init_x[1], final_x[1], init_x[2], final_x[2];
 
//   cout << "R = " << R << endl;
 
   temp_t1 = 0;
   temp_t2 = tf;
 
//   cout << "temp_t1 = " << temp_t1 << "temp_t2 = " << temp_t2 << endl;
 
   A << 1, temp_t1, pow(temp_t1,2), pow(temp_t1,3), pow(temp_t1,4), pow(temp_t1,5),
        1, temp_t2, pow(temp_t2,2), pow(temp_t2,3), pow(temp_t2,4), pow(temp_t2,5),
        0,  1   , 2*pow(temp_t1,1), 3*pow(temp_t1,2), 4*pow(temp_t1,3), 5*pow(temp_t1,4),
        0,  1   , 2*pow(temp_t2,1), 3*pow(temp_t2,2), 4*pow(temp_t2,3), 5*pow(temp_t2,4),
        0,  0   , 2 , 6*pow(temp_t1,1), 12*pow(temp_t1,2), 20*pow(temp_t1,3),
        0,  0   , 2 , 6*pow(temp_t2,1), 12*pow(temp_t2,2), 20*pow(temp_t2,3);
 
//   cout << "A = " << A << endl;
 
   P = A.inverse() * R;
 
//   cout << "P = " << P << endl;
 
   output[0] = P(0,0);
   output[1] = P(1,0);
   output[2] = P(2,0);
   output[3] = P(3,0);
   output[4] = P(4,0);
   output[5] = P(5,0);
}


void CRobot::Cal_Fc(void){
 
//  static double Kp_imp = 100, Kd_imp = 1;
    static unsigned int Fc_cnt = 0;
    static double tmp_Fc1 = 80;//80;//50; // 50
    static double tmp_Fc2 = tmp_Fc1*2.0;
 
 
    if(TROT_PHASE == INIT_Fc){
        Fc_RL = -tmp_Fc1/2.0*(1-cos(PI2/(home_pos_time*2)*(double)(ctc_cnt)*dt));
        Fc_RR = -tmp_Fc1/2.0*(1-cos(PI2/(home_pos_time*2)*(double)(ctc_cnt)*dt));
        Fc_FL = -tmp_Fc1/2.0*(1-cos(PI2/(home_pos_time*2)*(double)(ctc_cnt)*dt));
        Fc_FR = -tmp_Fc1/2.0*(1-cos(PI2/(home_pos_time*2)*(double)(ctc_cnt)*dt));
 
    }else{
        Fc_RL = -tmp_Fc1;
        Fc_RR = -tmp_Fc1;
        Fc_FL = -tmp_Fc1;
        Fc_FR = -tmp_Fc1;
 
        Fc_cnt = 0;
    }
 
    if(TROT_PHASE == STOP){
        Fc_RL = -tmp_Fc1;
        Fc_RR = -tmp_Fc1;
        Fc_FL = -tmp_Fc1;
        Fc_FR = -tmp_Fc1;
 
        Fc_cnt = 0;
    }
    else if(TROT_PHASE == STANCE_RLFR){
        Fc_RL = -tmp_Fc2;//-100 + Kp_imp*(RB_CON.goal_EP[2] - RB_CON.actual_EP[2]) + Kd_imp*(0 - RB_CON.actual_EP_vel[2]);
        Fc_RR = 0;
        Fc_FL = 0;
        Fc_FR = -tmp_Fc2;//-100 + Kp_imp*(RB_CON.goal_EP[11] - RB_CON.actual_EP[11]) + Kd_imp*(0 - RB_CON.actual_EP_vel[11]);
 
        Fc_cnt = 0;
//      printf("[1] Fc_RL = %f, Fc_FR = %f\n",Fc_RL, Fc_FR);
    }
    else if(TROT_PHASE == STANCE_RRFL){
        Fc_RL = 0;
        Fc_RR = -tmp_Fc2;//-100 + Kp_imp*(RB_CON.goal_EP[5] - RB_CON.actual_EP[5]) + Kd_imp*(0 - RB_CON.actual_EP_vel[5]);
        Fc_FL = -tmp_Fc2;//-100 + Kp_imp*(RB_CON.goal_EP[8] - RB_CON.actual_EP[8]) + Kd_imp*(0 - RB_CON.actual_EP_vel[8]);
        Fc_FR = 0;
 
        Fc_cnt = 0;
//      printf("[2] Fc_RR = %f, Fc_FL = %f\n",Fc_RR, Fc_FL);
    }
 
 
    Fc   << 0, 0, 0, 0, 0, 0, 0 ,0, 0, Fc_RL,  0, 0, Fc_RR, 0, 0, Fc_FL, 0, 0, Fc_FR;
 
}