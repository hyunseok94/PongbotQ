
#include <stdio.h>
#include <math.h>
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

    base.ID = m_pModel->GetBodyId("REAR_BODY");
    front_body.ID = m_pModel->GetBodyId("FRONT_BODY");
    FR.ID = m_pModel->GetBodyId("FR_CALF");
    FL.ID = m_pModel->GetBodyId("FL_CALF");
    RR.ID = m_pModel->GetBodyId("RR_CALF");
    RL.ID = m_pModel->GetBodyId("RL_CALF");

    QQ << 0, 0, 0, 1;
    m_pModel->SetQuaternion(base.ID, QQ, RobotState);

    init_target_pos << 0, -45, 90, 0, -45, 90, 0, 0, 45, -90, 0, 45, -90;
    goal_EP << 0, 0.15378, -0.45, 0, -0.15378, -0.45, 0.0, 0.15378, -0.45, 0.0, -0.15378, -0.45;

    // 2019.08.19
//    Kp_q << 700, 900, 800, 700, 900, 800, 20000, 700, 900, 800, 700, 900, 800;
//	Kd_q << 15,  15,  30, 15,  15,  30, 200, 15,  15,  30, 15,  15,  30;

    // DH : Actual robot
//    Kp_q << 400, 400, 400, 400, 400, 400, 20000, 400, 400, 400, 400, 400, 400; //100
//    Kd_q << 10,  10,  10, 10, 10,  10, 200, 10, 10,  10, 10,  10,  10;
//
//    Kp_vsd << 2000, 2000, 2000, 2000, 2000, 2000, 2000, 2000, 2000, 2000, 2000, 2000;
//    Kd_vsd << 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20;

    // DH : GAZEBO

    Kp_q << 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000; //100
    Kd_q << 10,  10,  10, 10, 10,  10, 10, 10, 10,  10, 10,  10,  10;

    Kp_vsd << 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10;
    Kd_vsd << 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1;
    
//    init_Kp_q << 50000, 50000, 50000, 50000, 50000, 50000, 10000, 50000, 50000, 50000, 50000, 50000, 50000; //2,3,10, 2,3,10, 2,3,10, 2,3,10;
//    init_Kd_q << 500, 500, 500, 500, 500, 500, 100, 500, 500, 500, 500, 500, 500;
//    goal_Kp_q << 60000, 60000, 60000, 60000, 60000, 60000, 10000, 60000, 60000, 60000, 60000, 60000, 60000; //2,3,10, 2,3,10, 2,3,10, 2,3,10;
//    goal_Kd_q << 600, 600, 600, 600, 600, 600, 100, 600, 600, 600, 600, 600, 600;
//    Kp_f << 10000, 300, 10000, 10000, 300, 10000, 10000, 300, 10000, 10000, 300, 10000; //2,3,10, 2,3,10, 2,3,10, 2,3,10;
//    Kd_f << 1000, 30, 1000, 1000, 30, 1000, 1000, 30, 1000, 1000, 30, 1000;

    //Kp_f << 5000, 300, 1000, 5000, 300, 1000, 5000, 300, 1000, 5000, 300, 1000; //2,3,10, 2,3,10, 2,3,10, 2,3,10;   //kp of force added by HSKIM
    //Kd_f << 500, 30, 100, 500, 30, 100, 500, 30, 100, 500, 30, 100;  //kd of force added by HSKIM

    //    goal_Kp_q << 50000, 50000, 50000, 50000, 50000, 50000, 10000, 50000, 50000, 50000, 50000, 50000, 50000; //2,3,10, 2,3,10, 2,3,10, 2,3,10;
    //    goal_Kd_q << 500, 500, 500, 500, 500, 500,     100,       500, 500, 500, 500, 500, 500;

    // Kp_q << 100000, 100000, 100000, 100000, 100000, 100000, 100000, 100000, 100000, 100000, 100000, 100000, 100000; //2,3,10, 2,3,10, 2,3,10, 2,3,10;
    // Kd_q << 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000;

    home_pos_time = 5;

//    ts = 0.20;
//    tf = dsp_time - ts;

    for(unsigned int i = 0; i<13 ; ++i){
		joint[i].torque = 0;
	}

    Traj_gen();
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

    target_pos = IK1(target_EP);

    for (unsigned int i = 0; i < 13; ++i) {
        target_vel[i] = (target_pos[i] - pre_target_pos[i]) / dt;
        pre_target_pos[i] = target_pos[i];

        target_acc[i] = (target_vel[i] - pre_target_vel[i]) / dt;
        pre_target_vel[i] = target_vel[i];
    }

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

    x_dot = J_A*RobotStatedot;

    //actual_EP_vel = x_dot.block<12, 1>(7, 0);
    actual_EP_vel = x_dot.block(7, 0, 12, 1);

    for (unsigned int i = 0; i < 12; ++i) {
        EP_vel_err[i] = target_EP_vel[i] - actual_EP_vel[i];
        EP_err[i] = target_EP[i] - actual_EP[i];
    }

    Cal_CP();

    Cal_Fc();

    for (unsigned int i = 0; i < 6; ++i) {
        RobotState2dot[i + 6] = 0;
    }
    for (unsigned int i = 0; i < 13; ++i) {
    	RobotState2dot[i + 6] = Kp_q[i]*(target_pos[i] - actual_pos[i]) + Kd_q[i]*(target_vel[i] - actual_vel[i]);
    }

    CompositeRigidBodyAlgorithm(*m_pModel, RobotState, M_term, true);
    NonlinearEffects(*m_pModel, RobotState, RobotStatedot, hatNonLinearEffects);
    NonlinearEffects(*m_pModel, RobotState, VectorNd::Zero(m_pModel->dof_count), G_term);

    C_term = hatNonLinearEffects - G_term;

//    CTC_Torque = M_term * RobotState2dot + C_term + G_term + J_A.transpose() * Fc; // old
    CTC_Torque = RobotState2dot + C_term + G_term + J_A.transpose() * Fc;
    for (int nJoint = 0; nJoint < nDOF; nJoint++) {
    	joint[nJoint].torque = CTC_Torque(6 + nJoint);
    }
    
    joint[6].torque = Kp_q[6]*(target_pos[6] - actual_pos[6]) + Kd_q[6]*(0 - actual_vel[6]);
//    joint[6].torque = (int)(RB_CON.Kp_q[6]*(RB_CON.target_pos[6] - RB_CON.actual_pos[6]) + RB_CON.Kd_q[6]*(0 - RB_CON.actual_vel[6]));
    //


}

void CRobot::FTsensorTransformation()
{
    RL.T_matrix = CalcBodyWorldOrientation(*m_pModel, RobotState, RL.ID, true);
    RR.T_matrix = CalcBodyWorldOrientation(*m_pModel, RobotState, RR.ID, true);
    FL.T_matrix = CalcBodyWorldOrientation(*m_pModel, RobotState, FL.ID, true);
    FR.T_matrix = CalcBodyWorldOrientation(*m_pModel, RobotState, FR.ID, true);
}

VectorNd CRobot::FK1(VectorNd q)
{

    const double L1 = 0.15378;
    const double L2 = 0.305;
    const double L3 = 0.305;

    static double q1 = 0;
    static double q2 = 0;
    static double q3 = 0;

    q1 = q[0];
    q2 = q[1]; // Actual : -q[1]
    q3 = q[2]; // Actual : -q[2]

    actual_EP[0] = -(-L3*(cos(q2)*sin(q3) + cos(q3)*sin(q2)) - L2*sin(q2));
    actual_EP[1] =  L1*cos(q1) - L3*(sin(q1)*sin(q2)*sin(q3) - cos(q2)*cos(q3)*sin(q1)) + L2*cos(q2)*sin(q1);
    actual_EP[2] =  L1*sin(q1) - L3*(cos(q1)*cos(q2)*cos(q3) - cos(q1)*sin(q2)*sin(q3)) - L2*cos(q1)*cos(q2);

//    printf("[RL] x=%f, y=%f, z=%f\n",actual_EP[0],actual_EP[1],actual_EP[2]);
    q1 = q[3];
    q2 = q[4]; // Actual : -q[4]
    q3 = q[5]; // Actual : -q[5]

    actual_EP[3] = -(- L3*(cos(q2)*sin(q3) + cos(q3)*sin(q2)) - L2*sin(q2));
    actual_EP[4] =   L2*cos(q2)*sin(q1) - L1*cos(q1) - L3*(sin(q1)*sin(q2)*sin(q3) - cos(q2)*cos(q3)*sin(q1));
    actual_EP[5] = - L1*sin(q1) - L3*(cos(q1)*cos(q2)*cos(q3) - cos(q1)*sin(q2)*sin(q3)) - L2*cos(q1)*cos(q2);

    q1 = q[7]; //joint[6].currentAngle;
    q2 = -q[8]; //joint[7].currentAngle;
    q3 = -q[9]; //joint[8].currentAngle;

    actual_EP[6] = -(-L3*(cos(q2)*sin(q3) + cos(q3)*sin(q2)) - L2*sin(q2));
    actual_EP[7] =  L1*cos(q1) - L3*(sin(q1)*sin(q2)*sin(q3) - cos(q2)*cos(q3)*sin(q1)) + L2*cos(q2)*sin(q1);
    actual_EP[8] =  L1*sin(q1) - L3*(cos(q1)*cos(q2)*cos(q3) - cos(q1)*sin(q2)*sin(q3)) - L2*cos(q1)*cos(q2);

    q1 = q[10]; //joint[9].currentAngle;  //revised by HSKIM
    q2 = -q[11]; //joint[10].currentAngle;
    q3 = -q[12]; //joint[11].currentAngle;

    actual_EP[9] =  -(- L3*(cos(q2)*sin(q3) + cos(q3)*sin(q2)) - L2*sin(q2));
    actual_EP[10] =   L2*cos(q2)*sin(q1) - L1*cos(q1) - L3*(sin(q1)*sin(q2)*sin(q3) - cos(q2)*cos(q3)*sin(q1));
    actual_EP[11] = - L1*sin(q1) - L3*(cos(q1)*cos(q2)*cos(q3) - cos(q1)*sin(q2)*sin(q3)) - L2*cos(q1)*cos(q2);

    return actual_EP;
}


VectorNd CRobot::IK1(VectorNd EP)
{
    const double L1 = 0.15378;
    const double L2 = 0.305;
    const double L3 = 0.305;

    static double x = 0;
    static double y = 0;
    static double z = 0;

    x = EP[0]; // Actual : -EP[0], target_pos[1] = -(), , target_pos[2] = -()
    y = EP[1];
    z = EP[2];

    target_pos[0] = atan(y/abs(z)) - PI/2 + acos(L1/(sqrt(pow(y,2) + pow(z,2))));
    target_pos[1] = (- atan(x/sqrt(abs(-pow(L1,2) + pow(y,2) + pow(z,2)))) - acos((- pow(L1,2) + pow(L2,2) - pow(L3,2) + pow(x,2) + pow(y,2) + pow(z,2))/(2*L2*sqrt(-pow(L1,2) + pow(x,2) + pow(y,2) + pow(z,2)))));
    target_pos[2] = (PI - acos((pow(L1,2) + pow(L2,2) + pow(L3,2) - pow(x,2) - pow(y,2) - pow(z,2))/(2*L2*L3)));

//    printf("[RL] q1=%f, q2=%f, q3=%f\n\n",target_pos[0],target_pos[1],target_pos[2]);
    
    x = EP[3]; // Actual : -EP[3]
    y = EP[4];
    z = EP[5];

    target_pos[3] = PI/2 + atan(y/abs(z)) - acos(L1/sqrt(pow(y,2) + pow(z,2)));//PI/2 - acos(L1/sqrt(pow(y,2) + pow(z,2))) - atan(abs(y)/abs(z)); //-((atan(y/abs(z)) - PI/2 + acos(L1/(sqrt(pow(y,2) + pow(z,2))))));
    target_pos[4] = (- atan(x/sqrt(abs(-pow(L1,2) + pow(y,2) + pow(z,2)))) - acos((- pow(L1,2) + pow(L2,2) - pow(L3,2) + pow(x,2) + pow(y,2) + pow(z,2))/(2*L2*sqrt(-pow(L1,2) + pow(x,2) + pow(y,2) + pow(z,2)))));
    target_pos[5] = (PI - acos((pow(L1,2) + pow(L2,2) + pow(L3,2) - pow(x,2) - pow(y,2) - pow(z,2))/(2*L2*L3)));

    x = -EP[6];
    y =  EP[7];
    z =  EP[8];

    target_pos[7] = atan(y/abs(z)) - PI/2 + acos(L1/(sqrt(pow(y,2) + pow(z,2))));
    target_pos[8] = -(- atan(x/sqrt(abs(-pow(L1,2) + pow(y,2) + pow(z,2)))) - acos((- pow(L1,2) + pow(L2,2) - pow(L3,2) + pow(x,2) + pow(y,2) + pow(z,2))/(2*L2*sqrt(-pow(L1,2) + pow(x,2) + pow(y,2) + pow(z,2)))));
    target_pos[9] = -(PI - acos((pow(L1,2) + pow(L2,2) + pow(L3,2) - pow(x,2) - pow(y,2) - pow(z,2))/(2*L2*L3)));

    x = -EP[9];
    y =  EP[10];
    z =  EP[11];

    target_pos[10] = PI/2 + atan(y/abs(z)) - acos(L1/sqrt(pow(y,2) + pow(z,2)));//PI/2 - acos(L1/sqrt(pow(y,2) + pow(z,2))) - atan(abs(y)/abs(z)); //-((atan(y/abs(z)) - PI/2 + acos(L1/(sqrt(pow(y,2) + pow(z,2))))));
    target_pos[11] = -(- atan(x/sqrt(abs(-pow(L1,2) + pow(y,2) + pow(z,2)))) - acos((- pow(L1,2) + pow(L2,2) - pow(L3,2) + pow(x,2) + pow(y,2) + pow(z,2))/(2*L2*sqrt(-pow(L1,2) + pow(x,2) + pow(y,2) + pow(z,2)))));
    target_pos[12] = -(PI - acos((pow(L1,2) + pow(L2,2) + pow(L3,2) - pow(x,2) - pow(y,2) - pow(z,2))/(2*L2*L3)));

    return target_pos;
}

void CRobot::Init_Pos_Traj(void)
{
//    target_tor[0] = 500 * (init_target_pos[0] * D2R - actual_pos[0]) + 10 * (0 - actual_vel[0]); //RL_HIP
    joint[0].torque = 500 * (init_target_pos[0] * D2R - actual_pos[0]) + 10 * (0 - actual_vel[0]); //RL_HIP
    joint[1].torque = 500 * (init_target_pos[1] * D2R - actual_pos[1]) + 10 * (0 - actual_vel[1]); //RL_THIGH
    joint[2].torque = 500 * (init_target_pos[2] * D2R - actual_pos[2]) + 10 * (0 - actual_vel[2]); //RL_CALF
    joint[3].torque = 500 * (init_target_pos[3] * D2R - actual_pos[3]) + 10 * (0 - actual_vel[3]); //RR_HIP
    joint[4].torque = 500 * (init_target_pos[4] * D2R - actual_pos[4]) + 10 * (0 - actual_vel[4]); //RR_THIGH
    joint[5].torque = 500 * (init_target_pos[5] * D2R - actual_pos[5]) + 10 * (0 - actual_vel[5]); //RR_CALF

    joint[6].torque = 500 * (init_target_pos[6] * D2R - actual_pos[6]) + 10 * (0 - actual_vel[6]); //WAIST

    joint[7].torque = 500 * (init_target_pos[7] * D2R - actual_pos[7]) + 10 * (0 - actual_vel[7]); //FL_HIP
    joint[8].torque = 500 * (init_target_pos[8] * D2R - actual_pos[8]) + 10 * (0 - actual_vel[8]); //FL_THIGH
    joint[9].torque = 500 * (init_target_pos[9] * D2R - actual_pos[9]) + 10 * (0 - actual_vel[9]); //FL_CALF
    joint[10].torque = 500 * (init_target_pos[10] * D2R - actual_pos[10]) + 10 * (0 - actual_vel[10]); //FR_HIP
    joint[11].torque = 500 * (init_target_pos[11] * D2R - actual_pos[11]) + 10 * (0 - actual_vel[11]); //FR_THIGH
    joint[12].torque = 500 * (init_target_pos[12] * D2R - actual_pos[12]) + 10 * (0 - actual_vel[12]); //FR_CALF

}

void CRobot::Home_Pos_Traj(void)
{
    FC_PHASE = INIT_Fc;                                           // this is commented out by HSKIM(to subdivide the forces depending on the Modes.)

    if (ctc_cnt == 0) {

    	actual_EP = FK1(actual_pos);

        for (unsigned int i = 0; i < 12; ++i) {
            init_EP[i] = actual_EP[i];
            target_EP[i] = actual_EP[i];

            target_EP_vel[i] = 0;
            target_EP_acc[i] = 0;
        }

        // waist
        init_pos[6] = actual_pos[6];
        target_pos[6] = actual_pos[6];

        ctc_cnt++;
    }

    else if (ctc_cnt <= (unsigned int) (home_pos_time / dt)) {

    	for (unsigned int i = 0; i < 12; ++i) {
            target_EP[i] = init_EP[i] + (goal_EP[i] - init_EP[i]) / 2.0 * (1 - cos(PI2 / (home_pos_time * 2)*(double) (ctc_cnt) * dt));
            target_EP_vel[i] = (goal_EP[i] - init_EP[i]) / 2.0 * PI2 / (home_pos_time * 2)*(sin(PI2 / (home_pos_time * 2)*(double) (ctc_cnt) * dt));
            target_EP_acc[i] = (goal_EP[i] - init_EP[i]) / 2.0 * PI2 / (home_pos_time * 2) * PI2 / (home_pos_time * 2)*(cos(PI2 / (home_pos_time * 2)*(double) (ctc_cnt) * dt));
        }

        // waist
        target_pos[6] = init_pos[6] + (0 - init_pos[6]) / 2.0 * (1 - cos(PI2 / (home_pos_time * 2)*(double) (ctc_cnt) * dt));

        ctc_cnt++;
    }

    else {
        //        cout << "[3]" << endl;
        for (unsigned int i = 0; i < 12; ++i) {
            target_EP[i] = goal_EP[i];
            target_EP_vel[i] = 0;
            target_EP_acc[i] = 0;
        }
        // waist
        target_pos[6] = 0;
    }
    
//    printf("target_pos[6] = %f, actual_pos[6] = %f\n",target_pos[6],actual_pos[6]);
}

void CRobot::TROT_Traj(void)
{

    if(ctc_cnt2 == 0){
    	TROT_PHASE = INIT_FORWARD;
    	FC_PHASE = STOP;
		tmp_cnt2 = 0;
	}
	else if(ctc_cnt2 < step_cnt){
		if(ctc_cnt2 <= dsp_cnt){
			TROT_PHASE = INIT_STANCE_RRFL;
			FC_PHASE = STANCE_RRFL;
		}
		else{
			TROT_PHASE = INIT_STANCE_FOUR_AFTER_RRFL;
			FC_PHASE = STOP;
		}
	}
	else if(ctc_cnt2 < step_cnt*2){
		if(ctc_cnt2 <= step_cnt + dsp_cnt){
			TROT_PHASE = TROT_STANCE_RLFR;
			FC_PHASE = STANCE_RLFR;
		}

		else{
			TROT_PHASE = TROT_STANCE_FOUR_AFTER_RLFR;
			FC_PHASE = STOP;
		}
	}
	else if(ctc_cnt2 < step_cnt*3){

		if(ctc_cnt2 <= step_cnt*2 + dsp_cnt){
			TROT_PHASE = TROT_STANCE_RRFL;
			FC_PHASE = STANCE_RRFL;
		}

		else{
			TROT_PHASE = TROT_STANCE_FOUR_AFTER_RRFL;
			FC_PHASE = STOP;
		}
	}
	else if(ctc_cnt2 <= step_cnt*4){

		if(ctc_cnt2 <= step_cnt*3 + dsp_cnt){
			TROT_PHASE = TROT_STANCE_RLFR2;
			FC_PHASE = STANCE_RLFR;
		}
		else{
			TROT_PHASE = TROT_STANCE_FOUR_AFTER_RLFR2;
			FC_PHASE = STOP;
		}
	}

	else if(ctc_cnt2 < step_cnt*5){
		if(ctc_cnt2 <= step_cnt*4 + dsp_cnt){
			TROT_PHASE = FINAL_STANCE_RRFL;
			FC_PHASE = STANCE_RRFL;
		}
		else{
			TROT_PHASE = FINAL_STANCE_FOUR;
			FC_PHASE = STOP;
		}
	}
	else{
		TROT_PHASE = FINAL_STANCE_FOUR;
		FC_PHASE = STOP;
	}


    switch(TROT_PHASE){

    	case INIT_FORWARD:

    		if(forward_init_flag == true){
    			for(unsigned int i=0; i<12; ++i){
					init_goal_EP[i] = goal_EP[i];//RB_CON.target_EP[i];
					target_EP_vel[i] = 0;
					target_EP_acc[i] = 0;
				}

    			forward_init_flag = false;
    		}

    		break;

    	case INIT_STANCE_RRFL:

    		tmp_time = (double)(ctc_cnt2)*dt;

    		for(unsigned int i=0; i<3; ++i){

    			if(i == 0){
    				target_EP[i] =  + c_xl1[0] + c_xl1[1]*tmp_time + c_xl1[2]*pow(tmp_time,2) + c_xl1[3]*pow(tmp_time,3) + c_xl1[4]*pow(tmp_time,4) + c_xl1[5]*pow(tmp_time,5);
					target_EP_vel[i] = c_xl1[1] + 2*c_xl1[2]*pow(tmp_time,1) + 3*c_xl1[3]*pow(tmp_time,2) + 4*c_xl1[4]*pow(tmp_time,3) + 5*c_xl1[5]*pow(tmp_time,4);
					target_EP_acc[i] = 2*c_xl1[2] + 6*c_xl1[3]*pow(tmp_time,1) + 12*c_xl1[4]*pow(tmp_time,2) + 20*c_xl1[5]*pow(tmp_time,3);
				}
				else if(i == 1){
					target_EP[i] = init_goal_EP[i];
					target_EP_vel[i] = 0;
					target_EP_acc[i] = 0;
				}
				else{ // z
					if(ctc_cnt2 <= (unsigned int)((dsp_time/2)/dt)){
						target_EP[i] = init_goal_EP[i] + c_z1[0] + c_z1[1]*tmp_time + c_z1[2]*pow(tmp_time,2) + c_z1[3]*pow(tmp_time,3) + c_z1[4]*pow(tmp_time,4) + c_z1[5]*pow(tmp_time,5);
						target_EP_vel[i] = c_z1[1] + 2*c_z1[2]*pow(tmp_time,1) + 3*c_z1[3]*pow(tmp_time,2) + 4*c_z1[4]*pow(tmp_time,3) + 5*c_z1[5]*pow(tmp_time,4);
						target_EP_acc[i] = 2*c_z1[2] + 6*c_z1[3]*pow(tmp_time,1) + 12*c_z1[4]*pow(tmp_time,2) + 20*c_z1[5]*pow(tmp_time,3);
					}
					else{
						tmp_time2 = tmp_time - dsp_time/2;

						target_EP[i] = init_goal_EP[i] + c_z2[0] + c_z2[1]*tmp_time2 + c_z2[2]*pow(tmp_time2,2) + c_z2[3]*pow(tmp_time2,3) + c_z2[4]*pow(tmp_time2,4) + c_z2[5]*pow(tmp_time2,5);
						target_EP_vel[i] = c_z2[1] + 2*c_z2[2]*pow(tmp_time2,1) + 3*c_z2[3]*pow(tmp_time2,2) + 4*c_z2[4]*pow(tmp_time2,3) + 5*c_z2[5]*pow(tmp_time2,4);
						target_EP_acc[i] = 2*c_z2[2] + 6*c_z2[3]*pow(tmp_time2,1) + 12*c_z2[4]*pow(tmp_time2,2) + 20*c_z2[5]*pow(tmp_time2,3);
					}
				}
    		}

    		for(unsigned int i=3; i<6; ++i){

				if(i == 3){
					target_EP[i] =  + c_xr1[0] + c_xr1[1]*tmp_time + c_xr1[2]*pow(tmp_time,2) + c_xr1[3]*pow(tmp_time,3) + c_xr1[4]*pow(tmp_time,4) + c_xr1[5]*pow(tmp_time,5);
					target_EP_vel[i] = c_xr1[1] + 2*c_xr1[2]*pow(tmp_time,1) + 3*c_xr1[3]*pow(tmp_time,2) + 4*c_xr1[4]*pow(tmp_time,3) + 5*c_xr1[5]*pow(tmp_time,4);
					target_EP_acc[i] = 2*c_xr1[2] + 6*c_xr1[3]*pow(tmp_time,1) + 12*c_xr1[4]*pow(tmp_time,2) + 20*c_xr1[5]*pow(tmp_time,3);
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
					target_EP[i] =  + c_xr1[0] + c_xr1[1]*tmp_time + c_xr1[2]*pow(tmp_time,2) + c_xr1[3]*pow(tmp_time,3) + c_xr1[4]*pow(tmp_time,4) + c_xr1[5]*pow(tmp_time,5);
					target_EP_vel[i] = c_xr1[1] + 2*c_xr1[2]*pow(tmp_time,1) + 3*c_xr1[3]*pow(tmp_time,2) + 4*c_xr1[4]*pow(tmp_time,3) + 5*c_xr1[5]*pow(tmp_time,4);
					target_EP_acc[i] = 2*c_xr1[2] + 6*c_xr1[3]*pow(tmp_time,1) + 12*c_xr1[4]*pow(tmp_time,2) + 20*c_xr1[5]*pow(tmp_time,3);
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
					target_EP[i] =  + c_xl1[0] + c_xl1[1]*tmp_time + c_xl1[2]*pow(tmp_time,2) + c_xl1[3]*pow(tmp_time,3) + c_xl1[4]*pow(tmp_time,4) + c_xl1[5]*pow(tmp_time,5);
					target_EP_vel[i] = c_xl1[1] + 2*c_xl1[2]*pow(tmp_time,1) + 3*c_xl1[3]*pow(tmp_time,2) + 4*c_xl1[4]*pow(tmp_time,3) + 5*c_xl1[5]*pow(tmp_time,4);
					target_EP_acc[i] = 2*c_xl1[2] + 6*c_xl1[3]*pow(tmp_time,1) + 12*c_xl1[4]*pow(tmp_time,2) + 20*c_xl1[5]*pow(tmp_time,3);
				}
				else if(i == 10){
					target_EP[i] = init_goal_EP[i];
					target_EP_vel[i] = 0;
					target_EP_acc[i] = 0;
				}
				else{ // z
					if(ctc_cnt2 <= (unsigned int)((dsp_time/2)/dt)){
						target_EP[i] = init_goal_EP[i] + c_z1[0] + c_z1[1]*tmp_time + c_z1[2]*pow(tmp_time,2) + c_z1[3]*pow(tmp_time,3) + c_z1[4]*pow(tmp_time,4) + c_z1[5]*pow(tmp_time,5);
						target_EP_vel[i] = c_z1[1] + 2*c_z1[2]*pow(tmp_time,1) + 3*c_z1[3]*pow(tmp_time,2) + 4*c_z1[4]*pow(tmp_time,3) + 5*c_z1[5]*pow(tmp_time,4);
						target_EP_acc[i] = 2*c_z1[2] + 6*c_z1[3]*pow(tmp_time,1) + 12*c_z1[4]*pow(tmp_time,2) + 20*c_z1[5]*pow(tmp_time,3);
					}
					else{
						tmp_time2 = tmp_time - dsp_time/2;

						target_EP[i] = init_goal_EP[i] + c_z2[0] + c_z2[1]*tmp_time2 + c_z2[2]*pow(tmp_time2,2) + c_z2[3]*pow(tmp_time2,3) + c_z2[4]*pow(tmp_time2,4) + c_z2[5]*pow(tmp_time2,5);
						target_EP_vel[i] = c_z2[1] + 2*c_z2[2]*pow(tmp_time2,1) + 3*c_z2[3]*pow(tmp_time2,2) + 4*c_z2[4]*pow(tmp_time2,3) + 5*c_z2[5]*pow(tmp_time2,4);
						target_EP_acc[i] = 2*c_z2[2] + 6*c_z2[3]*pow(tmp_time2,1) + 12*c_z2[4]*pow(tmp_time2,2) + 20*c_z2[5]*pow(tmp_time2,3);
					}
				}
			}


			break;

    	case INIT_STANCE_FOUR_AFTER_RRFL:
//    		printf(".");

    		tmp_time = (double)(ctc_cnt2)*dt - dsp_time;

//    		printf("tmp_time = %f\n",tmp_time);

			for(unsigned int i=0; i<3; ++i){
				if(i == 0){
					target_EP[i] =  + c_xl2[0] + c_xl2[1]*tmp_time + c_xl2[2]*pow(tmp_time,2) + c_xl2[3]*pow(tmp_time,3) + c_xl2[4]*pow(tmp_time,4) + c_xl2[5]*pow(tmp_time,5);
					target_EP_vel[i] = c_xl2[1] + 2*c_xl2[2]*pow(tmp_time,1) + 3*c_xl2[3]*pow(tmp_time,2) + 4*c_xl2[4]*pow(tmp_time,3) + 5*c_xl2[5]*pow(tmp_time,4);
					target_EP_acc[i] = 2*c_xl2[2] + 6*c_xl2[3]*pow(tmp_time,1) + 12*c_xl2[4]*pow(tmp_time,2) + 20*c_xl2[5]*pow(tmp_time,3);
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
					target_EP[i] =  + c_xr2[0] + c_xr2[1]*tmp_time + c_xr2[2]*pow(tmp_time,2) + c_xr2[3]*pow(tmp_time,3) + c_xr2[4]*pow(tmp_time,4) + c_xr2[5]*	pow(tmp_time,5);
					target_EP_vel[i] = c_xr2[1] + 2*c_xr2[2]*pow(tmp_time,1) + 3*c_xr2[3]*pow(tmp_time,2) + 4*c_xr2[4]*pow(tmp_time,3) + 5*c_xr2[5]*pow(tmp_time,4);
					target_EP_acc[i] = 2*c_xr2[2] + 6*c_xr2[3]*pow(tmp_time,1) + 12*c_xr2[4]*pow(tmp_time,2) + 20*c_xr2[5]*pow(tmp_time,3);
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
					target_EP[i] =  + c_xr2[0] + c_xr2[1]*tmp_time + c_xr2[2]*pow(tmp_time,2) + c_xr2[3]*pow(tmp_time,3) + c_xr2[4]*pow(tmp_time,4) + c_xr2[5]*	pow(tmp_time,5);
					target_EP_vel[i] = c_xr2[1] + 2*c_xr2[2]*pow(tmp_time,1) + 3*c_xr2[3]*pow(tmp_time,2) + 4*c_xr2[4]*pow(tmp_time,3) + 5*c_xr2[5]*pow(tmp_time,4);
					target_EP_acc[i] = 2*c_xr2[2] + 6*c_xr2[3]*pow(tmp_time,1) + 12*c_xr2[4]*pow(tmp_time,2) + 20*c_xr2[5]*pow(tmp_time,3);
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
					target_EP[i] =  + c_xl2[0] + c_xl2[1]*tmp_time + c_xl2[2]*pow(tmp_time,2) + c_xl2[3]*pow(tmp_time,3) + c_xl2[4]*pow(tmp_time,4) + c_xl2[5]*pow(tmp_time,5);
					target_EP_vel[i] = c_xl2[1] + 2*c_xl2[2]*pow(tmp_time,1) + 3*c_xl2[3]*pow(tmp_time,2) + 4*c_xl2[4]*pow(tmp_time,3) + 5*c_xl2[5]*pow(tmp_time,4);
					target_EP_acc[i] = 2*c_xl2[2] + 6*c_xl2[3]*pow(tmp_time,1) + 12*c_xl2[4]*pow(tmp_time,2) + 20*c_xl2[5]*pow(tmp_time,3);
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
    		tmp_time = (double)((ctc_cnt2)*dt)-step_time;//(double)(ctc_cnt2)*dt - dsp_time/2;
//
//
    		for(unsigned int i=0; i<3; ++i){

    			if(i == 0){
    				target_EP[i] =  + c_xl3[0] + c_xl3[1]*tmp_time + c_xl3[2]*pow(tmp_time,2) + c_xl3[3]*pow(tmp_time,3) + c_xl3[4]*pow(tmp_time,4) + c_xl3[5]*pow(tmp_time,5);
					target_EP_vel[i] = c_xl3[1] + 2*c_xl3[2]*pow(tmp_time,1) + 3*c_xl3[3]*pow(tmp_time,2) + 4*c_xl3[4]*pow(tmp_time,3) + 5*c_xl3[5]*pow(tmp_time,4);
					target_EP_acc[i] = 2*c_xl3[2] + 6*c_xl3[3]*pow(tmp_time,1) + 12*c_xl3[4]*pow(tmp_time,2) + 20*c_xl3[5]*pow(tmp_time,3);
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
					target_EP[i] =  + c_xr3[0] + c_xr3[1]*tmp_time + c_xr3[2]*pow(tmp_time,2) + c_xr3[3]*pow(tmp_time,3) + c_xr3[4]*pow(tmp_time,4) + c_xr3[5]*pow(tmp_time,5);
					target_EP_vel[i] = c_xr3[1] + 2*c_xr3[2]*pow(tmp_time,1) + 3*c_xr3[3]*pow(tmp_time,2) + 4*c_xr3[4]*pow(tmp_time,3) + 5*c_xr3[5]*pow(tmp_time,4);
					target_EP_acc[i] = 2*c_xr3[2] + 6*c_xr3[3]*pow(tmp_time,1) + 12*c_xr3[4]*pow(tmp_time,2) + 20*c_xr3[5]*pow(tmp_time,3);
				}
				else if(i == 4){
					target_EP[i] = init_goal_EP[i];
					target_EP_vel[i] = 0;
					target_EP_acc[i] = 0;
				}
				else{ // z
    				if(tmp_cnt2 <= (int)((dsp_time/2)/dt)){
						target_EP[i] = init_goal_EP[i] + c_z1[0] + c_z1[1]*tmp_time + c_z1[2]*pow(tmp_time,2) + c_z1[3]*pow(tmp_time,3) + c_z1[4]*pow(tmp_time,4) + c_z1[5]*pow(tmp_time,5);
						target_EP_vel[i] = c_z1[1] + 2*c_z1[2]*pow(tmp_time,1) + 3*c_z1[3]*pow(tmp_time,2) + 4*c_z1[4]*pow(tmp_time,3) + 5*c_z1[5]*pow(tmp_time,4);
						target_EP_acc[i] = 2*c_z1[2] + 6*c_z1[3]*pow(tmp_time,1) + 12*c_z1[4]*pow(tmp_time,2) + 20*c_z1[5]*pow(tmp_time,3);
					}
					else{
						tmp_time2 = tmp_time - dsp_time/2;

						target_EP[i] = init_goal_EP[i] + c_z2[0] + c_z2[1]*tmp_time2 + c_z2[2]*pow(tmp_time2,2) + c_z2[3]*pow(tmp_time2,3) + c_z2[4]*pow(tmp_time2,4) + c_z2[5]*pow(tmp_time2,5);
						target_EP_vel[i] = c_z2[1] + 2*c_z2[2]*pow(tmp_time2,1) + 3*c_z2[3]*pow(tmp_time2,2) + 4*c_z2[4]*pow(tmp_time2,3) + 5*c_z2[5]*pow(tmp_time2,4);
						target_EP_acc[i] = 2*c_z2[2] + 6*c_z2[3]*pow(tmp_time2,1) + 12*c_z2[4]*pow(tmp_time2,2) + 20*c_z2[5]*pow(tmp_time2,3);
					}
				}
			}


    		for(unsigned int i=6; i<9; ++i){

				if(i == 6){
					target_EP[i] =  + c_xr3[0] + c_xr3[1]*tmp_time + c_xr3[2]*pow(tmp_time,2) + c_xr3[3]*pow(tmp_time,3) + c_xr3[4]*pow(tmp_time,4) + c_xr3[5]*pow(tmp_time,5);
					target_EP_vel[i] = c_xr3[1] + 2*c_xr3[2]*pow(tmp_time,1) + 3*c_xr3[3]*pow(tmp_time,2) + 4*c_xr3[4]*pow(tmp_time,3) + 5*c_xr3[5]*pow(tmp_time,4);
					target_EP_acc[i] = 2*c_xr3[2] + 6*c_xr3[3]*pow(tmp_time,1) + 12*c_xr3[4]*pow(tmp_time,2) + 20*c_xr3[5]*pow(tmp_time,3);
				}
				else if(i == 7){
					target_EP[i] = init_goal_EP[i];
					target_EP_vel[i] = 0;
					target_EP_acc[i] = 0;
				}
				else{ // z
					if(tmp_cnt2 <= (int)((dsp_time/2)/dt)){
						target_EP[i] = init_goal_EP[i] + c_z1[0] + c_z1[1]*tmp_time + c_z1[2]*pow(tmp_time,2) + c_z1[3]*pow(tmp_time,3) + c_z1[4]*pow(tmp_time,4) + c_z1[5]*pow(tmp_time,5);
						target_EP_vel[i] = c_z1[1] + 2*c_z1[2]*pow(tmp_time,1) + 3*c_z1[3]*pow(tmp_time,2) + 4*c_z1[4]*pow(tmp_time,3) + 5*c_z1[5]*pow(tmp_time,4);
						target_EP_acc[i] = 2*c_z1[2] + 6*c_z1[3]*pow(tmp_time,1) + 12*c_z1[4]*pow(tmp_time,2) + 20*c_z1[5]*pow(tmp_time,3);
					}
					else{
						tmp_time2 = tmp_time - dsp_time/2;

						target_EP[i] = init_goal_EP[i] + c_z2[0] + c_z2[1]*tmp_time2 + c_z2[2]*pow(tmp_time2,2) + c_z2[3]*pow(tmp_time2,3) + c_z2[4]*pow(tmp_time2,4) + c_z2[5]*pow(tmp_time2,5);
						target_EP_vel[i] = c_z2[1] + 2*c_z2[2]*pow(tmp_time2,1) + 3*c_z2[3]*pow(tmp_time2,2) + 4*c_z2[4]*pow(tmp_time2,3) + 5*c_z2[5]*pow(tmp_time2,4);
						target_EP_acc[i] = 2*c_z2[2] + 6*c_z2[3]*pow(tmp_time2,1) + 12*c_z2[4]*pow(tmp_time2,2) + 20*c_z2[5]*pow(tmp_time2,3);
					}
				}
			}


    		for(unsigned int i=9; i<12; ++i){
				if(i == 9){
					target_EP[i] =  + c_xl3[0] + c_xl3[1]*tmp_time + c_xl3[2]*pow(tmp_time,2) + c_xl3[3]*pow(tmp_time,3) + c_xl3[4]*pow(tmp_time,4) + c_xl3[5]*pow(tmp_time,5);
					target_EP_vel[i] = c_xl3[1] + 2*c_xl3[2]*pow(tmp_time,1) + 3*c_xl3[3]*pow(tmp_time,2) + 4*c_xl3[4]*pow(tmp_time,3) + 5*c_xl3[5]*pow(tmp_time,4);
					target_EP_acc[i] = 2*c_xl3[2] + 6*c_xl3[3]*pow(tmp_time,1) + 12*c_xl3[4]*pow(tmp_time,2) + 20*c_xl3[5]*pow(tmp_time,3);
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
//    		printf(".");

			tmp_time = (double)(ctc_cnt2)*dt - step_time - dsp_time;

//    		printf("tmp_time = %f\n",tmp_time);

			for(unsigned int i=0; i<3; ++i){
				if(i == 0){
					target_EP[i] =  + c_xl4[0] + c_xl4[1]*tmp_time + c_xl4[2]*pow(tmp_time,2) + c_xl4[3]*pow(tmp_time,3) + c_xl4[4]*pow(tmp_time,4) + c_xl4[5]*pow(tmp_time,5);
					target_EP_vel[i] = c_xl4[1] + 2*c_xl4[2]*pow(tmp_time,1) + 3*c_xl4[3]*pow(tmp_time,2) + 4*c_xl4[4]*pow(tmp_time,3) + 5*c_xl4[5]*pow(tmp_time,4);
					target_EP_acc[i] = 2*c_xl4[2] + 6*c_xl4[3]*pow(tmp_time,1) + 12*c_xl4[4]*pow(tmp_time,2) + 20*c_xl4[5]*pow(tmp_time,3);
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
					target_EP[i] =  + c_xr4[0] + c_xr4[1]*tmp_time + c_xr4[2]*pow(tmp_time,2) + c_xr4[3]*pow(tmp_time,3) + c_xr4[4]*pow(tmp_time,4) + c_xr4[5]*pow(tmp_time,5);
					target_EP_vel[i] = c_xr4[1] + 2*c_xr4[2]*pow(tmp_time,1) + 3*c_xr4[3]*pow(tmp_time,2) + 4*c_xr4[4]*pow(tmp_time,3) + 5*c_xr4[5]*pow(tmp_time,4);
					target_EP_acc[i] = 2*c_xr4[2] + 6*c_xr4[3]*pow(tmp_time,1) + 12*c_xr4[4]*pow(tmp_time,2) + 20*c_xr4[5]*pow(tmp_time,3);
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
					target_EP[i] =  + c_xr4[0] + c_xr4[1]*tmp_time + c_xr4[2]*pow(tmp_time,2) + c_xr4[3]*pow(tmp_time,3) + c_xr4[4]*pow(tmp_time,4) + c_xr4[5]*pow(tmp_time,5);
					target_EP_vel[i] = c_xr4[1] + 2*c_xr4[2]*pow(tmp_time,1) + 3*c_xr4[3]*pow(tmp_time,2) + 4*c_xr4[4]*pow(tmp_time,3) + 5*c_xr4[5]*pow(tmp_time,4);
					target_EP_acc[i] = 2*c_xr4[2] + 6*c_xr4[3]*pow(tmp_time,1) + 12*c_xr4[4]*pow(tmp_time,2) + 20*c_xr4[5]*pow(tmp_time,3);
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
					target_EP[i] =  + c_xl4[0] + c_xl4[1]*tmp_time + c_xl4[2]*pow(tmp_time,2) + c_xl4[3]*pow(tmp_time,3) + c_xl4[4]*pow(tmp_time,4) + c_xl4[5]*pow(tmp_time,5);
					target_EP_vel[i] = c_xl4[1] + 2*c_xl4[2]*pow(tmp_time,1) + 3*c_xl4[3]*pow(tmp_time,2) + 4*c_xl4[4]*pow(tmp_time,3) + 5*c_xl4[5]*pow(tmp_time,4);
					target_EP_acc[i] = 2*c_xl4[2] + 6*c_xl4[3]*pow(tmp_time,1) + 12*c_xl4[4]*pow(tmp_time,2) + 20*c_xl4[5]*pow(tmp_time,3);
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

			tmp_cnt2 = ctc_cnt2 - step_cnt*2;//(int)(step_time*2/dt);
			tmp_time = (double)(tmp_cnt2)*dt;//(double)(ctc_cnt2)*dt - dsp_time/2;
//
//
			for(unsigned int i=0; i<3; ++i){

				if(i == 0){
					target_EP[i] =  + c_xr3[0] + c_xr3[1]*tmp_time + c_xr3[2]*pow(tmp_time,2) + c_xr3[3]*pow(tmp_time,3) + c_xr3[4]*pow(tmp_time,4) + c_xr3[5]*pow(tmp_time,5);
					target_EP_vel[i] = c_xr3[1] + 2*c_xr3[2]*pow(tmp_time,1) + 3*c_xr3[3]*pow(tmp_time,2) + 4*c_xr3[4]*pow(tmp_time,3) + 5*c_xr3[5]*pow(tmp_time,4);
					target_EP_acc[i] = 2*c_xr3[2] + 6*c_xr3[3]*pow(tmp_time,1) + 12*c_xr3[4]*pow(tmp_time,2) + 20*c_xr3[5]*pow(tmp_time,3);
				}
				else if(i == 1){
					target_EP[i] = init_goal_EP[i];
					target_EP_vel[i] = 0;
					target_EP_acc[i] = 0;
				}
				else{ // z
					if(tmp_cnt2 <= (int)((dsp_time/2)/dt)){
						target_EP[i] = init_goal_EP[i] + c_z1[0] + c_z1[1]*tmp_time + c_z1[2]*pow(tmp_time,2) + c_z1[3]*pow(tmp_time,3) + c_z1[4]*pow(tmp_time,4) + c_z1[5]*pow(tmp_time,5);
						target_EP_vel[i] = c_z1[1] + 2*c_z1[2]*pow(tmp_time,1) + 3*c_z1[3]*pow(tmp_time,2) + 4*c_z1[4]*pow(tmp_time,3) + 5*c_z1[5]*pow(tmp_time,4);
						target_EP_acc[i] = 2*c_z1[2] + 6*c_z1[3]*pow(tmp_time,1) + 12*c_z1[4]*pow(tmp_time,2) + 20*c_z1[5]*pow(tmp_time,3);
					}
					else{
						tmp_time2 = tmp_time - dsp_time/2;

						target_EP[i] = init_goal_EP[i] + c_z2[0] + c_z2[1]*tmp_time2 + c_z2[2]*pow(tmp_time2,2) + c_z2[3]*pow(tmp_time2,3) + c_z2[4]*pow(tmp_time2,4) + c_z2[5]*pow(tmp_time2,5);
						target_EP_vel[i] = c_z2[1] + 2*c_z2[2]*pow(tmp_time2,1) + 3*c_z2[3]*pow(tmp_time2,2) + 4*c_z2[4]*pow(tmp_time2,3) + 5*c_z2[5]*pow(tmp_time2,4);
						target_EP_acc[i] = 2*c_z2[2] + 6*c_z2[3]*pow(tmp_time2,1) + 12*c_z2[4]*pow(tmp_time2,2) + 20*c_z2[5]*pow(tmp_time2,3);
					}
				}
			}


			for(unsigned int i=3; i<6; ++i){

				if(i == 3){
					target_EP[i] =  + c_xl3[0] + c_xl3[1]*tmp_time + c_xl3[2]*pow(tmp_time,2) + c_xl3[3]*pow(tmp_time,3) + c_xl3[4]*pow(tmp_time,4) + c_xl3[5]*pow(tmp_time,5);
					target_EP_vel[i] = c_xl3[1] + 2*c_xl3[2]*pow(tmp_time,1) + 3*c_xl3[3]*pow(tmp_time,2) + 4*c_xl3[4]*pow(tmp_time,3) + 5*c_xl3[5]*pow(tmp_time,4);
					target_EP_acc[i] = 2*c_xl3[2] + 6*c_xl3[3]*pow(tmp_time,1) + 12*c_xl3[4]*pow(tmp_time,2) + 20*c_xl3[5]*pow(tmp_time,3);
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
					target_EP[i] =  + c_xl3[0] + c_xl3[1]*tmp_time + c_xl3[2]*pow(tmp_time,2) + c_xl3[3]*pow(tmp_time,3) + c_xl3[4]*pow(tmp_time,4) + c_xl3[5]*pow(tmp_time,5);
					target_EP_vel[i] = c_xl3[1] + 2*c_xl3[2]*pow(tmp_time,1) + 3*c_xl3[3]*pow(tmp_time,2) + 4*c_xl3[4]*pow(tmp_time,3) + 5*c_xl3[5]*pow(tmp_time,4);
					target_EP_acc[i] = 2*c_xl3[2] + 6*c_xl3[3]*pow(tmp_time,1) + 12*c_xl3[4]*pow(tmp_time,2) + 20*c_xl3[5]*pow(tmp_time,3);
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
					target_EP[i] =  + c_xr3[0] + c_xr3[1]*tmp_time + c_xr3[2]*pow(tmp_time,2) + c_xr3[3]*pow(tmp_time,3) + c_xr3[4]*pow(tmp_time,4) + c_xr3[5]*pow(tmp_time,5);
					target_EP_vel[i] = c_xr3[1] + 2*c_xr3[2]*pow(tmp_time,1) + 3*c_xr3[3]*pow(tmp_time,2) + 4*c_xr3[4]*pow(tmp_time,3) + 5*c_xr3[5]*pow(tmp_time,4);
					target_EP_acc[i] = 2*c_xr3[2] + 6*c_xr3[3]*pow(tmp_time,1) + 12*c_xr3[4]*pow(tmp_time,2) + 20*c_xr3[5]*pow(tmp_time,3);
				}
				else if(i == 10){
					target_EP[i] = init_goal_EP[i];
					target_EP_vel[i] = 0;
					target_EP_acc[i] = 0;
				}
				else{ // z
					if(tmp_cnt2 <= (int)((dsp_time/2)/dt)){
						target_EP[i] = init_goal_EP[i] + c_z1[0] + c_z1[1]*tmp_time + c_z1[2]*pow(tmp_time,2) + c_z1[3]*pow(tmp_time,3) + c_z1[4]*pow(tmp_time,4) + c_z1[5]*pow(tmp_time,5);
						target_EP_vel[i] = c_z1[1] + 2*c_z1[2]*pow(tmp_time,1) + 3*c_z1[3]*pow(tmp_time,2) + 4*c_z1[4]*pow(tmp_time,3) + 5*c_z1[5]*pow(tmp_time,4);
						target_EP_acc[i] = 2*c_z1[2] + 6*c_z1[3]*pow(tmp_time,1) + 12*c_z1[4]*pow(tmp_time,2) + 20*c_z1[5]*pow(tmp_time,3);
					}
					else{
						tmp_time2 = tmp_time - dsp_time/2;

						target_EP[i] = init_goal_EP[i] + c_z2[0] + c_z2[1]*tmp_time2 + c_z2[2]*pow(tmp_time2,2) + c_z2[3]*pow(tmp_time2,3) + c_z2[4]*pow(tmp_time2,4) + c_z2[5]*pow(tmp_time2,5);
						target_EP_vel[i] = c_z2[1] + 2*c_z2[2]*pow(tmp_time2,1) + 3*c_z2[3]*pow(tmp_time2,2) + 4*c_z2[4]*pow(tmp_time2,3) + 5*c_z2[5]*pow(tmp_time2,4);
						target_EP_acc[i] = 2*c_z2[2] + 6*c_z2[3]*pow(tmp_time2,1) + 12*c_z2[4]*pow(tmp_time2,2) + 20*c_z2[5]*pow(tmp_time2,3);
					}
				}
			}

			break;



    	case TROT_STANCE_FOUR_AFTER_RRFL:
//    		printf(".");
    		tmp_cnt2 = ctc_cnt2 - step_cnt*2 - dsp_cnt;
			tmp_time = (double)(tmp_cnt2)*dt;// -step_time*2 - dsp_time;

//    		printf("tmp_time = %f\n",tmp_time);

			for(unsigned int i=0; i<3; ++i){
				if(i == 0){
					target_EP[i] =  + c_xr4[0] + c_xr4[1]*tmp_time + c_xr4[2]*pow(tmp_time,2) + c_xr4[3]*pow(tmp_time,3) + c_xr4[4]*pow(tmp_time,4) + c_xr4[5]*pow(tmp_time,5);
					target_EP_vel[i] = c_xr4[1] + 2*c_xr4[2]*pow(tmp_time,1) + 3*c_xr4[3]*pow(tmp_time,2) + 4*c_xr4[4]*pow(tmp_time,3) + 5*c_xr4[5]*pow(tmp_time,4);
					target_EP_acc[i] = 2*c_xr4[2] + 6*c_xr4[3]*pow(tmp_time,1) + 12*c_xr4[4]*pow(tmp_time,2) + 20*c_xr4[5]*pow(tmp_time,3);
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
					target_EP[i] =  + c_xl4[0] + c_xl4[1]*tmp_time + c_xl4[2]*pow(tmp_time,2) + c_xl4[3]*pow(tmp_time,3) + c_xl4[4]*pow(tmp_time,4) + c_xl4[5]*pow(tmp_time,5);
					target_EP_vel[i] = c_xl4[1] + 2*c_xl4[2]*pow(tmp_time,1) + 3*c_xl4[3]*pow(tmp_time,2) + 4*c_xl4[4]*pow(tmp_time,3) + 5*c_xl4[5]*pow(tmp_time,4);
					target_EP_acc[i] = 2*c_xl4[2] + 6*c_xl4[3]*pow(tmp_time,1) + 12*c_xl4[4]*pow(tmp_time,2) + 20*c_xl4[5]*pow(tmp_time,3);
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
					target_EP[i] =  + c_xl4[0] + c_xl4[1]*tmp_time + c_xl4[2]*pow(tmp_time,2) + c_xl4[3]*pow(tmp_time,3) + c_xl4[4]*pow(tmp_time,4) + c_xl4[5]*pow(tmp_time,5);
					target_EP_vel[i] = c_xl4[1] + 2*c_xl4[2]*pow(tmp_time,1) + 3*c_xl4[3]*pow(tmp_time,2) + 4*c_xl4[4]*pow(tmp_time,3) + 5*c_xl4[5]*pow(tmp_time,4);
					target_EP_acc[i] = 2*c_xl4[2] + 6*c_xl4[3]*pow(tmp_time,1) + 12*c_xl4[4]*pow(tmp_time,2) + 20*c_xl4[5]*pow(tmp_time,3);
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
					target_EP[i] =  + c_xr4[0] + c_xr4[1]*tmp_time + c_xr4[2]*pow(tmp_time,2) + c_xr4[3]*pow(tmp_time,3) + c_xr4[4]*pow(tmp_time,4) + c_xr4[5]*pow(tmp_time,5);
					target_EP_vel[i] = c_xr4[1] + 2*c_xr4[2]*pow(tmp_time,1) + 3*c_xr4[3]*pow(tmp_time,2) + 4*c_xr4[4]*pow(tmp_time,3) + 5*c_xr4[5]*pow(tmp_time,4);
					target_EP_acc[i] = 2*c_xr4[2] + 6*c_xr4[3]*pow(tmp_time,1) + 12*c_xr4[4]*pow(tmp_time,2) + 20*c_xr4[5]*pow(tmp_time,3);
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


    	case TROT_STANCE_RLFR2:

			tmp_cnt2 = ctc_cnt2 - step_cnt*3;//(int)(step_time*3/dt);
			tmp_time = (double)(tmp_cnt2)*dt;//((ctc_cnt2)*dt)-step_time*3;//(double)(ctc_cnt2)*dt - dsp_time/2;
//
//
			for(unsigned int i=0; i<3; ++i){

				if(i == 0){
					target_EP[i] =  + c_xl3[0] + c_xl3[1]*tmp_time + c_xl3[2]*pow(tmp_time,2) + c_xl3[3]*pow(tmp_time,3) + c_xl3[4]*pow(tmp_time,4) + c_xl3[5]*pow(tmp_time,5);
					target_EP_vel[i] = c_xl3[1] + 2*c_xl3[2]*pow(tmp_time,1) + 3*c_xl3[3]*pow(tmp_time,2) + 4*c_xl3[4]*pow(tmp_time,3) + 5*c_xl3[5]*pow(tmp_time,4);
					target_EP_acc[i] = 2*c_xl3[2] + 6*c_xl3[3]*pow(tmp_time,1) + 12*c_xl3[4]*pow(tmp_time,2) + 20*c_xl3[5]*pow(tmp_time,3);
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
					target_EP[i] =  + c_xr3[0] + c_xr3[1]*tmp_time + c_xr3[2]*pow(tmp_time,2) + c_xr3[3]*pow(tmp_time,3) + c_xr3[4]*pow(tmp_time,4) + c_xr3[5]*pow(tmp_time,5);
					target_EP_vel[i] = c_xr3[1] + 2*c_xr3[2]*pow(tmp_time,1) + 3*c_xr3[3]*pow(tmp_time,2) + 4*c_xr3[4]*pow(tmp_time,3) + 5*c_xr3[5]*pow(tmp_time,4);
					target_EP_acc[i] = 2*c_xr3[2] + 6*c_xr3[3]*pow(tmp_time,1) + 12*c_xr3[4]*pow(tmp_time,2) + 20*c_xr3[5]*pow(tmp_time,3);
				}
				else if(i == 4){
					target_EP[i] = init_goal_EP[i];
					target_EP_vel[i] = 0;
					target_EP_acc[i] = 0;
				}
				else{ // z
					if(tmp_cnt2 <= (int)((dsp_time/2)/dt)){
						target_EP[i] = init_goal_EP[i] + c_z1[0] + c_z1[1]*tmp_time + c_z1[2]*pow(tmp_time,2) + c_z1[3]*pow(tmp_time,3) + c_z1[4]*pow(tmp_time,4) + c_z1[5]*pow(tmp_time,5);
						target_EP_vel[i] = c_z1[1] + 2*c_z1[2]*pow(tmp_time,1) + 3*c_z1[3]*pow(tmp_time,2) + 4*c_z1[4]*pow(tmp_time,3) + 5*c_z1[5]*pow(tmp_time,4);
						target_EP_acc[i] = 2*c_z1[2] + 6*c_z1[3]*pow(tmp_time,1) + 12*c_z1[4]*pow(tmp_time,2) + 20*c_z1[5]*pow(tmp_time,3);
					}
					else{
						tmp_time2 = tmp_time - dsp_time/2;

						target_EP[i] = init_goal_EP[i] + c_z2[0] + c_z2[1]*tmp_time2 + c_z2[2]*pow(tmp_time2,2) + c_z2[3]*pow(tmp_time2,3) + c_z2[4]*pow(tmp_time2,4) + c_z2[5]*pow(tmp_time2,5);
						target_EP_vel[i] = c_z2[1] + 2*c_z2[2]*pow(tmp_time2,1) + 3*c_z2[3]*pow(tmp_time2,2) + 4*c_z2[4]*pow(tmp_time2,3) + 5*c_z2[5]*pow(tmp_time2,4);
						target_EP_acc[i] = 2*c_z2[2] + 6*c_z2[3]*pow(tmp_time2,1) + 12*c_z2[4]*pow(tmp_time2,2) + 20*c_z2[5]*pow(tmp_time2,3);
					}
				}
			}


			for(unsigned int i=6; i<9; ++i){

				if(i == 6){
					target_EP[i] =  + c_xr3[0] + c_xr3[1]*tmp_time + c_xr3[2]*pow(tmp_time,2) + c_xr3[3]*pow(tmp_time,3) + c_xr3[4]*pow(tmp_time,4) + c_xr3[5]*pow(tmp_time,5);
					target_EP_vel[i] = c_xr3[1] + 2*c_xr3[2]*pow(tmp_time,1) + 3*c_xr3[3]*pow(tmp_time,2) + 4*c_xr3[4]*pow(tmp_time,3) + 5*c_xr3[5]*pow(tmp_time,4);
					target_EP_acc[i] = 2*c_xr3[2] + 6*c_xr3[3]*pow(tmp_time,1) + 12*c_xr3[4]*pow(tmp_time,2) + 20*c_xr3[5]*pow(tmp_time,3);
				}
				else if(i == 7){
					target_EP[i] = init_goal_EP[i];
					target_EP_vel[i] = 0;
					target_EP_acc[i] = 0;
				}
				else{ // z
					if(tmp_cnt2 <= (int)((dsp_time/2)/dt)){
						target_EP[i] = init_goal_EP[i] + c_z1[0] + c_z1[1]*tmp_time + c_z1[2]*pow(tmp_time,2) + c_z1[3]*pow(tmp_time,3) + c_z1[4]*pow(tmp_time,4) + c_z1[5]*pow(tmp_time,5);
						target_EP_vel[i] = c_z1[1] + 2*c_z1[2]*pow(tmp_time,1) + 3*c_z1[3]*pow(tmp_time,2) + 4*c_z1[4]*pow(tmp_time,3) + 5*c_z1[5]*pow(tmp_time,4);
						target_EP_acc[i] = 2*c_z1[2] + 6*c_z1[3]*pow(tmp_time,1) + 12*c_z1[4]*pow(tmp_time,2) + 20*c_z1[5]*pow(tmp_time,3);
					}
					else{
						tmp_time2 = tmp_time - dsp_time/2;

						target_EP[i] = init_goal_EP[i] + c_z2[0] + c_z2[1]*tmp_time2 + c_z2[2]*pow(tmp_time2,2) + c_z2[3]*pow(tmp_time2,3) + c_z2[4]*pow(tmp_time2,4) + c_z2[5]*pow(tmp_time2,5);
						target_EP_vel[i] = c_z2[1] + 2*c_z2[2]*pow(tmp_time2,1) + 3*c_z2[3]*pow(tmp_time2,2) + 4*c_z2[4]*pow(tmp_time2,3) + 5*c_z2[5]*pow(tmp_time2,4);
						target_EP_acc[i] = 2*c_z2[2] + 6*c_z2[3]*pow(tmp_time2,1) + 12*c_z2[4]*pow(tmp_time2,2) + 20*c_z2[5]*pow(tmp_time2,3);
					}
				}
			}


			for(unsigned int i=9; i<12; ++i){
				if(i == 9){
					target_EP[i] =  + c_xl3[0] + c_xl3[1]*tmp_time + c_xl3[2]*pow(tmp_time,2) + c_xl3[3]*pow(tmp_time,3) + c_xl3[4]*pow(tmp_time,4) + c_xl3[5]*pow(tmp_time,5);
					target_EP_vel[i] = c_xl3[1] + 2*c_xl3[2]*pow(tmp_time,1) + 3*c_xl3[3]*pow(tmp_time,2) + 4*c_xl3[4]*pow(tmp_time,3) + 5*c_xl3[5]*pow(tmp_time,4);
					target_EP_acc[i] = 2*c_xl3[2] + 6*c_xl3[3]*pow(tmp_time,1) + 12*c_xl3[4]*pow(tmp_time,2) + 20*c_xl3[5]*pow(tmp_time,3);
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


		case TROT_STANCE_FOUR_AFTER_RLFR2:

			tmp_cnt2 = ctc_cnt2 - step_cnt*3 - dsp_cnt;
			tmp_time = (double)(tmp_cnt2)*dt;


			for(unsigned int i=0; i<3; ++i){
				if(i == 0){
					target_EP[i] =  + c_xl4[0] + c_xl4[1]*tmp_time + c_xl4[2]*pow(tmp_time,2) + c_xl4[3]*pow(tmp_time,3) + c_xl4[4]*pow(tmp_time,4) + c_xl4[5]*pow(tmp_time,5);
					target_EP_vel[i] = c_xl4[1] + 2*c_xl4[2]*pow(tmp_time,1) + 3*c_xl4[3]*pow(tmp_time,2) + 4*c_xl4[4]*pow(tmp_time,3) + 5*c_xl4[5]*pow(tmp_time,4);
					target_EP_acc[i] = 2*c_xl4[2] + 6*c_xl4[3]*pow(tmp_time,1) + 12*c_xl4[4]*pow(tmp_time,2) + 20*c_xl4[5]*pow(tmp_time,3);
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
					target_EP[i] =  + c_xr4[0] + c_xr4[1]*tmp_time + c_xr4[2]*pow(tmp_time,2) + c_xr4[3]*pow(tmp_time,3) + c_xr4[4]*pow(tmp_time,4) + c_xr4[5]*pow(tmp_time,5);
					target_EP_vel[i] = c_xr4[1] + 2*c_xr4[2]*pow(tmp_time,1) + 3*c_xr4[3]*pow(tmp_time,2) + 4*c_xr4[4]*pow(tmp_time,3) + 5*c_xr4[5]*pow(tmp_time,4);
					target_EP_acc[i] = 2*c_xr4[2] + 6*c_xr4[3]*pow(tmp_time,1) + 12*c_xr4[4]*pow(tmp_time,2) + 20*c_xr4[5]*pow(tmp_time,3);
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
					target_EP[i] =  + c_xr4[0] + c_xr4[1]*tmp_time + c_xr4[2]*pow(tmp_time,2) + c_xr4[3]*pow(tmp_time,3) + c_xr4[4]*pow(tmp_time,4) + c_xr4[5]*pow(tmp_time,5);
					target_EP_vel[i] = c_xr4[1] + 2*c_xr4[2]*pow(tmp_time,1) + 3*c_xr4[3]*pow(tmp_time,2) + 4*c_xr4[4]*pow(tmp_time,3) + 5*c_xr4[5]*pow(tmp_time,4);
					target_EP_acc[i] = 2*c_xr4[2] + 6*c_xr4[3]*pow(tmp_time,1) + 12*c_xr4[4]*pow(tmp_time,2) + 20*c_xr4[5]*pow(tmp_time,3);
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
					target_EP[i] =  + c_xl4[0] + c_xl4[1]*tmp_time + c_xl4[2]*pow(tmp_time,2) + c_xl4[3]*pow(tmp_time,3) + c_xl4[4]*pow(tmp_time,4) + c_xl4[5]*pow(tmp_time,5);
					target_EP_vel[i] = c_xl4[1] + 2*c_xl4[2]*pow(tmp_time,1) + 3*c_xl4[3]*pow(tmp_time,2) + 4*c_xl4[4]*pow(tmp_time,3) + 5*c_xl4[5]*pow(tmp_time,4);
					target_EP_acc[i] = 2*c_xl4[2] + 6*c_xl4[3]*pow(tmp_time,1) + 12*c_xl4[4]*pow(tmp_time,2) + 20*c_xl4[5]*pow(tmp_time,3);
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




    	case FINAL_STANCE_RRFL:

    		tmp_cnt2 = ctc_cnt2 - step_cnt*4;
			tmp_time = (double)(tmp_cnt2)*dt;

			for(unsigned int i=0; i<3; ++i){

				if(i == 0){
					target_EP[i] =  c_xl_f[0] + c_xl_f[1]*tmp_time + c_xl_f[2]*pow(tmp_time,2) + c_xl_f[3]*pow(tmp_time,3) + c_xl_f[4]*pow(tmp_time,4) + c_xl_f[5]*pow(tmp_time,5);
					target_EP_vel[i] = c_xl_f[1] + 2*c_xl_f[2]*pow(tmp_time,1) + 3*c_xl_f[3]*pow(tmp_time,2) + 4*c_xl_f[4]*pow(tmp_time,3) + 5*c_xl_f[5]*pow(tmp_time,4);
					target_EP_acc[i] = 2*c_xl_f[2] + 6*c_xl_f[3]*pow(tmp_time,1) + 12*c_xl_f[4]*pow(tmp_time,2) + 20*c_xl_f[5]*pow(tmp_time,3);
				}
				else if(i == 1){
					target_EP[i] = init_goal_EP[i];
					target_EP_vel[i] = 0;
					target_EP_acc[i] = 0;
				}
				else{ // z
					if(tmp_cnt2 <= (int)((dsp_time/2)/dt)){
						target_EP[i] = init_goal_EP[i] + c_z1[0] + c_z1[1]*tmp_time + c_z1[2]*pow(tmp_time,2) + c_z1[3]*pow(tmp_time,3) + c_z1[4]*pow(tmp_time,4) + c_z1[5]*pow(tmp_time,5);
						target_EP_vel[i] = c_z1[1] + 2*c_z1[2]*pow(tmp_time,1) + 3*c_z1[3]*pow(tmp_time,2) + 4*c_z1[4]*pow(tmp_time,3) + 5*c_z1[5]*pow(tmp_time,4);
						target_EP_acc[i] = 2*c_z1[2] + 6*c_z1[3]*pow(tmp_time,1) + 12*c_z1[4]*pow(tmp_time,2) + 20*c_z1[5]*pow(tmp_time,3);
					}
					else{
						tmp_time2 = tmp_time - dsp_time/2;

						target_EP[i] = init_goal_EP[i] + c_z2[0] + c_z2[1]*tmp_time2 + c_z2[2]*pow(tmp_time2,2) + c_z2[3]*pow(tmp_time2,3) + c_z2[4]*pow(tmp_time2,4) + c_z2[5]*pow(tmp_time2,5);
						target_EP_vel[i] = c_z2[1] + 2*c_z2[2]*pow(tmp_time2,1) + 3*c_z2[3]*pow(tmp_time2,2) + 4*c_z2[4]*pow(tmp_time2,3) + 5*c_z2[5]*pow(tmp_time2,4);
						target_EP_acc[i] = 2*c_z2[2] + 6*c_z2[3]*pow(tmp_time2,1) + 12*c_z2[4]*pow(tmp_time2,2) + 20*c_z2[5]*pow(tmp_time2,3);
					}

				}
			}


			for(unsigned int i=3; i<6; ++i){

				if(i == 3){
					target_EP[i] =  c_xr_f[0] + c_xr_f[1]*tmp_time + c_xr_f[2]*pow(tmp_time,2) + c_xr_f[3]*pow(tmp_time,3) + c_xr_f[4]*pow(tmp_time,4) + c_xr_f[5]*pow(tmp_time,5);
					target_EP_vel[i] = c_xr_f[1] + 2*c_xr_f[2]*pow(tmp_time,1) + 3*c_xr_f[3]*pow(tmp_time,2) + 4*c_xr_f[4]*pow(tmp_time,3) + 5*c_xr_f[5]*pow(tmp_time,4);
					target_EP_acc[i] = 2*c_xr_f[2] + 6*c_xr_f[3]*pow(tmp_time,1) + 12*c_xr_f[4]*pow(tmp_time,2) + 20*c_xr_f[5]*pow(tmp_time,3);
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
					target_EP[i] =  c_xr_f[0] + c_xr_f[1]*tmp_time + c_xr_f[2]*pow(tmp_time,2) + c_xr_f[3]*pow(tmp_time,3) + c_xr_f[4]*pow(tmp_time,4) + c_xr_f[5]*pow(tmp_time,5);
					target_EP_vel[i] = c_xr_f[1] + 2*c_xr_f[2]*pow(tmp_time,1) + 3*c_xr_f[3]*pow(tmp_time,2) + 4*c_xr_f[4]*pow(tmp_time,3) + 5*c_xr_f[5]*pow(tmp_time,4);
					target_EP_acc[i] = 2*c_xr_f[2] + 6*c_xr_f[3]*pow(tmp_time,1) + 12*c_xr_f[4]*pow(tmp_time,2) + 20*c_xr_f[5]*pow(tmp_time,3);
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
					target_EP[i] =  c_xl_f[0] + c_xl_f[1]*tmp_time + c_xl_f[2]*pow(tmp_time,2) + c_xl_f[3]*pow(tmp_time,3) + c_xl_f[4]*pow(tmp_time,4) + c_xl_f[5]*pow(tmp_time,5);
					target_EP_vel[i] = c_xl_f[1] + 2*c_xl_f[2]*pow(tmp_time,1) + 3*c_xl_f[3]*pow(tmp_time,2) + 4*c_xl_f[4]*pow(tmp_time,3) + 5*c_xl_f[5]*pow(tmp_time,4);
					target_EP_acc[i] = 2*c_xl_f[2] + 6*c_xl_f[3]*pow(tmp_time,1) + 12*c_xl_f[4]*pow(tmp_time,2) + 20*c_xl_f[5]*pow(tmp_time,3);
				}
				else if(i == 10){
					target_EP[i] = init_goal_EP[i];
					target_EP_vel[i] = 0;
					target_EP_acc[i] = 0;
				}
				else{ // z
					if(tmp_cnt2 <= (int)((dsp_time/2)/dt)){
						target_EP[i] = init_goal_EP[i] + c_z1[0] + c_z1[1]*tmp_time + c_z1[2]*pow(tmp_time,2) + c_z1[3]*pow(tmp_time,3) + c_z1[4]*pow(tmp_time,4) + c_z1[5]*pow(tmp_time,5);
						target_EP_vel[i] = c_z1[1] + 2*c_z1[2]*pow(tmp_time,1) + 3*c_z1[3]*pow(tmp_time,2) + 4*c_z1[4]*pow(tmp_time,3) + 5*c_z1[5]*pow(tmp_time,4);
						target_EP_acc[i] = 2*c_z1[2] + 6*c_z1[3]*pow(tmp_time,1) + 12*c_z1[4]*pow(tmp_time,2) + 20*c_z1[5]*pow(tmp_time,3);
					}
					else{
						tmp_time2 = tmp_time - dsp_time/2;

						target_EP[i] = init_goal_EP[i] + c_z2[0] + c_z2[1]*tmp_time2 + c_z2[2]*pow(tmp_time2,2) + c_z2[3]*pow(tmp_time2,3) + c_z2[4]*pow(tmp_time2,4) + c_z2[5]*pow(tmp_time2,5);
						target_EP_vel[i] = c_z2[1] + 2*c_z2[2]*pow(tmp_time2,1) + 3*c_z2[3]*pow(tmp_time2,2) + 4*c_z2[4]*pow(tmp_time2,3) + 5*c_z2[5]*pow(tmp_time2,4);
						target_EP_acc[i] = 2*c_z2[2] + 6*c_z2[3]*pow(tmp_time2,1) + 12*c_z2[4]*pow(tmp_time2,2) + 20*c_z2[5]*pow(tmp_time2,3);
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


	if(ctc_cnt2 == step_cnt*4){

		if(sub_ctrl_flag != true){

			pre_foot_l[0] = -x_step - x_fsp;//[ -x_step - x_fsp,-moving_speed,0];
			pre_foot_l[1] = -moving_speed;
			pre_foot_l[2] = 0;

			pre_foot_r[0] = x_step - x_fsp;//[  x_step - x_fsp,-moving_speed,0];
			pre_foot_r[1] = -moving_speed;
			pre_foot_r[2] = 0;

//			moving_speed = m_udp_recvData.rec_data1;//0.2; // [m/s]
			x_step = step_time*moving_speed/2.0;
			x_fsp = fsp_time*moving_speed/2.0;


			xl1[0] = pre_foot_l[0]; xl1[1] = pre_foot_l[1]; xl1[2] = pre_foot_l[2]; xl1[3] =  x_step + x_fsp; xl1[4] = -moving_speed; xl1[5] = 0; xl1[6] = dsp_time;
			xl2[0] = x_step + x_fsp; xl2[1] = -moving_speed; xl2[2] = 0; xl2[3] =  x_step - x_fsp; xl2[4] = -moving_speed; xl2[5] = 0; xl2[6] = fsp_time;
			xl3[0] = x_step - x_fsp; xl3[1] = -moving_speed; xl3[2] = 0; xl3[3] =  -x_step + x_fsp; xl3[4] = -moving_speed; xl3[5] = 0; xl3[6] = dsp_time;
			xl4[0] = -x_step + x_fsp; xl4[1] = -moving_speed; xl4[2] = 0; xl4[3] =  -x_step - x_fsp; xl4[4] = -moving_speed; xl4[5] = 0; xl4[6] = fsp_time;

			xr1[0] = pre_foot_r[0]; xr1[1] = pre_foot_r[1]; xr1[2] = pre_foot_r[2]; xr1[3] =  -x_step + x_fsp; xr1[4] = -moving_speed; xr1[5] = 0; xr1[6] = dsp_time;
			xr2[0] = -x_step + x_fsp; xr2[1] = -moving_speed; xr2[2] = 0; xr2[3] =  -x_step - x_fsp; xr2[4] = -moving_speed; xr2[5] = 0; xr2[6] = fsp_time;
			xr3[0] = -x_step - x_fsp; xr3[1] = -moving_speed; xr3[2] = 0; xr3[3] =  x_step + x_fsp; xr3[4] = -moving_speed; xr3[5] = 0; xr3[6] = dsp_time;
			xr4[0] = x_step + x_fsp; xr4[1] = -moving_speed; xr4[2] = 0; xr4[3] =  x_step - x_fsp; xr4[4] = -moving_speed; xr4[5] = 0; xr4[6] = fsp_time;

			Hori_X_Traj_Gen(xl1,xl2,xl3,xl4,xr1,xr2,xr3,xr4);

			ctc_cnt2 = 0;
		}
		else{
			xl_f[0] = -x_step - x_fsp; xl_f[1] = -moving_speed; xl_f[2] = 0; xl_f[3] = 0; xl_f[4] = 0; xl_f[5] = 0; xl_f[6] = dsp_time;
			xr_f[0] =  x_step - x_fsp; xr_f[1] = -moving_speed; xr_f[2] = 0; xr_f[3] = 0; xr_f[4] = 0; xr_f[5] = 0; xr_f[6] = dsp_time;

			Hori_X_Final_Traj_Gen(xl_f, xr_f);

		}

	}
	ctc_cnt2++;

	target_pos[6] = 0;//goal_pos[6];

}

void CRobot::Traj_gen(void) // common trot
{
	// ===================== Vertical Foot Trajectory Generation ==================== //
		double foot_height = 0.05;
		double dsp_t1 = dsp_time/2;
		double dsp_t2 = dsp_time - dsp_t1;

		double z1[7] = {0,0,0,foot_height,0,0,dsp_t1};
		double z2[7] = {foot_height,0,0,0,0,0,dsp_t2};

		Vertical_Traj_Gen(z1, z2);

		// ===================== Vertical Foot Trajectory Generation End ==================== //


		// ===================== Horizontal Foot Trajectory Generation ==================== //

		moving_speed = 0; // [m/s]
		x_step = step_time*moving_speed/2.0;
		x_fsp = fsp_time*moving_speed/2.0;

		pre_foot_l[0] = 0;//[ -x_step - x_fsp,-moving_speed,0];
		pre_foot_l[1] = 0;
		pre_foot_l[2] = 0;

		pre_foot_r[0] = 0;//[  x_step - x_fsp,-moving_speed,0];
		pre_foot_r[1] = 0;
		pre_foot_r[2] = 0;

		xl1[0] = pre_foot_l[0]; xl1[1] = pre_foot_l[1]; xl1[2] = pre_foot_l[2]; xl1[3] =  x_step + x_fsp; xl1[4] = -moving_speed; xl1[5] = 0; xl1[6] = dsp_time;
		xl2[0] = x_step + x_fsp; xl2[1] = -moving_speed; xl2[2] = 0; xl2[3] =  x_step - x_fsp; xl2[4] = -moving_speed; xl2[5] = 0; xl2[6] = fsp_time;
		xl3[0] = x_step - x_fsp; xl3[1] = -moving_speed; xl3[2] = 0; xl3[3] =  -x_step + x_fsp; xl3[4] = -moving_speed; xl3[5] = 0; xl3[6] = dsp_time;
		xl4[0] = -x_step + x_fsp; xl4[1] = -moving_speed; xl4[2] = 0; xl4[3] =  -x_step - x_fsp; xl4[4] = -moving_speed; xl4[5] = 0; xl4[6] = fsp_time;

		xr1[0] = pre_foot_r[0]; xr1[1] = pre_foot_r[1]; xr1[2] = pre_foot_r[2]; xr1[3] =  -x_step + x_fsp; xr1[4] = -moving_speed; xr1[5] = 0; xr1[6] = dsp_time;
		xr2[0] = -x_step + x_fsp; xr2[1] = -moving_speed; xr2[2] = 0; xr2[3] =  -x_step - x_fsp; xr2[4] = -moving_speed; xr2[5] = 0; xr2[6] = fsp_time;
		xr3[0] = -x_step - x_fsp; xr3[1] = -moving_speed; xr3[2] = 0; xr3[3] =  x_step + x_fsp; xr3[4] = -moving_speed; xr3[5] = 0; xr3[6] = dsp_time;
		xr4[0] = x_step + x_fsp; xr4[1] = -moving_speed; xr4[2] = 0; xr4[3] =  x_step - x_fsp; xr4[4] = -moving_speed; xr4[5] = 0; xr4[6] = fsp_time;

		Hori_X_Traj_Gen(xl1,xl2,xl3,xl4,xr1,xr2,xr3,xr4);

		// ===================== Horizontal Foot Trajectory Generation End ==================== //
}

void CRobot::ballistics(double flight_time, double landing_height, double take_off_speed)
{
    double g = 9.81;
    double time1 = (take_off_speed) / 9.81;
    double time2 = flight_time - time1; //(flight_time - 0.01) - time1; //- 0.012
    double top_height = landing_height + 0.5 * g * pow(time2, 2);
    to_height = top_height + 0.5 * g * pow(time1, 2) - take_off_speed*time1;
}

void CRobot::coefficient_5thPoly(double *init_x, double *final_x, double tf, double *output)
{
    //   MatrixXd R(6,1), A(6,6), P(6,1);

    R << init_x[0], final_x[0], init_x[1], final_x[1], init_x[2], final_x[2];

    //   cout << "R = " << R << endl;

    temp_t1 = 0;
    temp_t2 = tf;

    //   cout << "temp_t1 = " << temp_t1 << "temp_t2 = " << temp_t2 << endl;

    A << 1, temp_t1, pow(temp_t1, 2), pow(temp_t1, 3), pow(temp_t1, 4), pow(temp_t1, 5),
            1, temp_t2, pow(temp_t2, 2), pow(temp_t2, 3), pow(temp_t2, 4), pow(temp_t2, 5),
            0, 1, 2 * pow(temp_t1, 1), 3 * pow(temp_t1, 2), 4 * pow(temp_t1, 3), 5 * pow(temp_t1, 4),
            0, 1, 2 * pow(temp_t2, 1), 3 * pow(temp_t2, 2), 4 * pow(temp_t2, 3), 5 * pow(temp_t2, 4),
            0, 0, 2, 6 * pow(temp_t1, 1), 12 * pow(temp_t1, 2), 20 * pow(temp_t1, 3),
            0, 0, 2, 6 * pow(temp_t2, 1), 12 * pow(temp_t2, 2), 20 * pow(temp_t2, 3);

    //   cout << "A = " << A << endl;

    P = A.inverse() * R;
    //   cout << "P = " << P << endl;

    output[0] = P(0, 0);
    output[1] = P(1, 0);
    output[2] = P(2, 0);
    output[3] = P(3, 0);
    output[4] = P(4, 0);
    output[5] = P(5, 0);
}

void CRobot::Cal_Fc(void)
{
		static double tmp_Fc1 = 70;//50; // 50
		static double tmp_Fc2 = tmp_Fc1*2.0;


		if(FC_PHASE == STOP){
			Fc_RL_z = -tmp_Fc1;
			Fc_RR_z = -tmp_Fc1;
			Fc_FL_z = -tmp_Fc1;
			Fc_FR_z = -tmp_Fc1;
		}
		else if(FC_PHASE == INIT_Fc){
			Fc_RL_z = -tmp_Fc1/2.0*(1-cos(PI2/(home_pos_time*2)*(double)(ctc_cnt)*dt));
			Fc_RR_z = -tmp_Fc1/2.0*(1-cos(PI2/(home_pos_time*2)*(double)(ctc_cnt)*dt));
			Fc_FL_z = -tmp_Fc1/2.0*(1-cos(PI2/(home_pos_time*2)*(double)(ctc_cnt)*dt));
			Fc_FR_z = -tmp_Fc1/2.0*(1-cos(PI2/(home_pos_time*2)*(double)(ctc_cnt)*dt));
		}
		else if(FC_PHASE == STANCE_RLFR){
			Fc_RL_z = -tmp_Fc2;
			Fc_RR_z = 0;
			Fc_FL_z = 0;
			Fc_FR_z = -tmp_Fc2;
		}
		else if(FC_PHASE == STANCE_RRFL){
			Fc_RL_z = 0;
			Fc_RR_z = -tmp_Fc2;
			Fc_FL_z = -tmp_Fc2;
			Fc_FR_z = 0;

		}

		// ==================== VSD ======================= //
		for(unsigned int i=0;i<12;++i){
			Fc_vsd[i] = Kp_vsd[i]*EP_err[i] + Kd_vsd[i]*EP_vel_err[i];
		}
		// ==================== VSD END ====================== //

		// ==================== Force Distribution Control ==================== //
//		static double k_fd = 100;
//
//		F_fd_y = k_fd*CP_y;

		static double k_fd = 2000;

		static double ki = 1500;
		static double sum_CP_y_err = 0.;

		sum_CP_y_err = sum_CP_y_err + (CP_y - 0)*dt;

//		printf("sum_CP_y_err = %f\n",sum_CP_y_err);

//		if(CP_y > 0){
//			F_fd_l = -0.5*k_fd*(CP_y - 0);// - ki*sum_CP_y_err;
//			F_fd_r =  0.5*k_fd*(CP_y - 0);// + ki*sum_CP_y_err;
//		}else{
//			F_fd_l = -0.5*k_fd*(CP_y - 0);// - ki*sum_CP_y_err;
//			F_fd_r =  0.5*k_fd*(CP_y - 0);// + ki*sum_CP_y_err;
//		}

		F_fd_l = -0.5*k_fd*(CP_y - 0) - ki*sum_CP_y_err;
		F_fd_r =  0.5*k_fd*(CP_y - 0) + ki*sum_CP_y_err;

//		static double Ki = -0.002;
//		static double sum_Roll_err = 0, sum_Pitch_err = 0;
//
//		sum_Roll_err = sum_Roll_err + RB_CON.dt*(-0.0 - IMURoll);
//		sum_Pitch_err = sum_Pitch_err + RB_CON.dt*(0 - IMUPitch);
//
//		RB_CON.target_EP[2] = RB_CON.goal_EP[2] - Ki*sum_Roll_err + Ki*sum_Pitch_err;
//		RB_CON.target_EP[8] = RB_CON.goal_EP[8] - Ki*sum_Roll_err;
//		RB_CON.target_EP[5] = RB_CON.goal_EP[5] + Ki*sum_Pitch_err;
//
//	    static double y;
//	    static double sume = 0.;
//	    const double KI = 0.01;
//
//	    y = 0.0*(_ref - _angle) + KI*sume;
//
//	    sume += _ref - _angle;



		// ==================== Force Distribution Control END ==================== //


//		Fc   << 0, 0, 0, 0, 0, 0, 0 ,Fc_vsd[0], Fc_vsd[1] + F_fd_y, Fc_RL_z + Fc_vsd[2] ,Fc_vsd[3], Fc_vsd[4] + F_fd_y, Fc_RR_z + Fc_vsd[5],  Fc_vsd[6], Fc_vsd[7] + F_fd_y, Fc_FL_z + Fc_vsd[8],Fc_vsd[9], Fc_vsd[10] + F_fd_y, Fc_FR_z + Fc_vsd[11];
		Fc   << 0, 0, 0, 0, 0, 0, 0 ,Fc_vsd[0], Fc_vsd[1], Fc_RL_z + Fc_vsd[2] + F_fd_l ,Fc_vsd[3], Fc_vsd[4], Fc_RR_z + Fc_vsd[5] + F_fd_r,  Fc_vsd[6], Fc_vsd[7], Fc_FL_z + Fc_vsd[8] + F_fd_l, Fc_vsd[9], Fc_vsd[10], Fc_FR_z + Fc_vsd[11] + F_fd_r;

}


void CRobot::Torque_off(void)
{
	for (int i=0; i<nDOF; ++i){
		computed_tor[i] = 0;
	}
}

void CRobot::Vertical_Traj_Gen(double *z1, double *z2){

	init_x[0] = z1[0];
	init_x[1] = z1[1];
	init_x[2] = z1[2];

	final_x[0] = z1[3];
	final_x[1] = z1[4];
	final_x[2] = z1[5];

	_t = z1[6];

	coefficient_5thPoly(init_x, final_x, _t, _out);

	c_z1[0] = _out[0];
	c_z1[1] = _out[1];
	c_z1[2] = _out[2];
	c_z1[3] = _out[3];
	c_z1[4] = _out[4];
	c_z1[5] = _out[5];


	init_x[0] = z2[0];
	init_x[1] = z2[1];
	init_x[2] = z2[2];

	final_x[0] = z2[3];
	final_x[1] = z2[4];
	final_x[2] = z2[5];

	_t = z2[6];

	coefficient_5thPoly(init_x, final_x, _t, _out);

	c_z2[0] = _out[0];
	c_z2[1] = _out[1];
	c_z2[2] = _out[2];
	c_z2[3] = _out[3];
	c_z2[4] = _out[4];
	c_z2[5] = _out[5];

}

void CRobot::Hori_X_Traj_Gen(double *xl1,double *xl2,double *xl3,double *xl4,double *xr1,double *xr2,double *xr3,double *xr4)
{
	init_x[0] = xl1[0];
	init_x[1] = xl1[1];
	init_x[2] = xl1[2];

	final_x[0] = xl1[3];//step_length;
	final_x[1] = xl1[4];
	final_x[2] = xl1[5];

	_t = xl1[6];

	coefficient_5thPoly(init_x, final_x, _t, _out);

	c_xl1[0] = _out[0];
	c_xl1[1] = _out[1];
	c_xl1[2] = _out[2];
	c_xl1[3] = _out[3];
	c_xl1[4] = _out[4];
	c_xl1[5] = _out[5];

	// --------------------------------------

	init_x[0] = xl2[0];
	init_x[1] = xl2[1];
	init_x[2] = xl2[2];

	final_x[0] = xl2[3];
	final_x[1] = xl2[4];
	final_x[2] = xl2[5];

	_t = xl2[6];

	coefficient_5thPoly(init_x, final_x, _t, _out);

	c_xl2[0] = _out[0];
	c_xl2[1] = _out[1];
	c_xl2[2] = _out[2];
	c_xl2[3] = _out[3];
	c_xl2[4] = _out[4];
	c_xl2[5] = _out[5];


	// --------------------------------------

	init_x[0] = xl3[0];
	init_x[1] = xl3[1];
	init_x[2] = xl3[2];

	final_x[0] = xl3[3];//-step_length;
	final_x[1] = xl3[4];
	final_x[2] = xl3[5];

	_t = xl3[6];

	coefficient_5thPoly(init_x, final_x, _t, _out);

	c_xl3[0] = _out[0];
	c_xl3[1] = _out[1];
	c_xl3[2] = _out[2];
	c_xl3[3] = _out[3];
	c_xl3[4] = _out[4];
	c_xl3[5] = _out[5];


	// --------------------------------------

	init_x[0] = xl4[0];
	init_x[1] = xl4[1];
	init_x[2] = xl4[2];

	final_x[0] = xl4[3];//-step_length;
	final_x[1] = xl4[4];
	final_x[2] = xl4[5];

	_t = xl4[6];

	coefficient_5thPoly(init_x, final_x, _t, _out);

	c_xl4[0] = _out[0];
	c_xl4[1] = _out[1];
	c_xl4[2] = _out[2];
	c_xl4[3] = _out[3];
	c_xl4[4] = _out[4];
	c_xl4[5] = _out[5];

	// --------------------------------------

	init_x[0] = xr1[0];
	init_x[1] = xr1[1];
	init_x[2] = xr1[2];

	final_x[0] = xr1[3];
	final_x[1] = xr1[4];
	final_x[2] = xr1[5];

	_t = xr1[6];

	coefficient_5thPoly(init_x, final_x, _t, _out);

	c_xr1[0] = _out[0];
	c_xr1[1] = _out[1];
	c_xr1[2] = _out[2];
	c_xr1[3] = _out[3];
	c_xr1[4] = _out[4];
	c_xr1[5] = _out[5];


	// --------------------------------------

	init_x[0] = xr2[0];
	init_x[1] = xr2[1];
	init_x[2] = xr2[2];

	final_x[0] = xr2[3];
	final_x[1] = xr2[4];
	final_x[2] = xr2[5];

	_t = xr2[6];

	coefficient_5thPoly(init_x, final_x, _t, _out);

	c_xr2[0] = _out[0];
	c_xr2[1] = _out[1];
	c_xr2[2] = _out[2];
	c_xr2[3] = _out[3];
	c_xr2[4] = _out[4];
	c_xr2[5] = _out[5];

	// --------------------------------------

	init_x[0] = xr3[0];
	init_x[1] = xr3[1];
	init_x[2] = xr3[2];

	final_x[0] = xr3[3];
	final_x[1] = xr3[4];
	final_x[2] = xr3[5];

	_t = xr3[6];

	coefficient_5thPoly(init_x, final_x, _t, _out);

	c_xr3[0] = _out[0];
	c_xr3[1] = _out[1];
	c_xr3[2] = _out[2];
	c_xr3[3] = _out[3];
	c_xr3[4] = _out[4];
	c_xr3[5] = _out[5];

	// --------------------------------------

	init_x[0] = xr4[0];
	init_x[1] = xr4[1];
	init_x[2] = xr4[2];

	final_x[0] = xr4[3];
	final_x[1] = xr4[4];
	final_x[2] = xr4[5];

	_t = xr4[6];

	coefficient_5thPoly(init_x, final_x, _t, _out);

	c_xr4[0] = _out[0];
	c_xr4[1] = _out[1];
	c_xr4[2] = _out[2];
	c_xr4[3] = _out[3];
	c_xr4[4] = _out[4];
	c_xr4[5] = _out[5];
}


void CRobot::Hori_X_Final_Traj_Gen(double *xl_f, double *xr_f){

	init_x[0] = xl_f[0];
	init_x[1] = xl_f[1];
	init_x[2] = xl_f[2];

	final_x[0] = xl_f[3];
	final_x[1] = xl_f[4];
	final_x[2] = xl_f[5];

	_t = xl_f[6];

	coefficient_5thPoly(init_x, final_x, _t, _out);

	c_xl_f[0] = _out[0];
	c_xl_f[1] = _out[1];
	c_xl_f[2] = _out[2];
	c_xl_f[3] = _out[3];
	c_xl_f[4] = _out[4];
	c_xl_f[5] = _out[5];

	// --------------------------------------

	init_x[0] = xr_f[0];
	init_x[1] = xr_f[1];
	init_x[2] = xr_f[2];

	final_x[0] = xr_f[3];
	final_x[1] = xr_f[4];
	final_x[2] = xr_f[5];

	_t = xr_f[6];

	coefficient_5thPoly(init_x, final_x, _t, _out);

	c_xr_f[0] = _out[0];
	c_xr_f[1] = _out[1];
	c_xr_f[2] = _out[2];
	c_xr_f[3] = _out[3];
	c_xr_f[4] = _out[4];
	c_xr_f[5] = _out[5];

}

void CRobot::Cal_CP(void){

	static double CP_alpha1 = 0.05, CP_alpha2 = 0.003;

	COM_Height = 0.45;
	COM_y = COM_Height*IMURoll*PI/180;
	COM_y_dot = COM_Height*IMURoll_dot*PI/180;

	lpf_COM_y = (1-CP_alpha1)*lpf_COM_y + CP_alpha1*COM_y;
	lpf_COM_y_dot = (1-CP_alpha2)*lpf_COM_y_dot + CP_alpha2*COM_y_dot;

	natural_freq = sqrt(COM_Height/GRAVITY);

	CP_y = lpf_COM_y + 1/natural_freq * lpf_COM_y_dot;

}
