
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
    // ============== Controller OnOff ================ //
//    Foot_Height_Control_OnOff_Flag = true;//false;//true;
//    VSD_Contrl_OnOff_Flag = false;
    // ============== Controller OnOff End ================ //
    
//	Mode = MODE_ACTUAL_ROBOT;
	Mode = MODE_SIMULATION;
        
    if(Mode == MODE_SIMULATION){
    	foot_z_offset << 0.0, 0.0, 0.00, 0.00;
        foot_height = 0.05;

    	Kp_q << 100, 400, 400, 100, 400, 400, 20000, 100, 400, 400, 100, 400, 400;
    	Kd_q <<   1,  15,  15,   1,  15,  15,   200,   1,  15,  15,   1,  15,  15;
        
        //2019.11.20
//    	Kp_q << 100, 400, 400, 100, 400, 400, 20000, 100, 400, 400, 100, 400, 400;
//    	Kd_q <<   2,  15,  15,   2,  15,  15,   200,   2,  15,  15,   2,  15,  15;

//        Kp_q << 100, 400, 400, 100, 400, 400, 20000, 100, 400, 400, 100, 400, 400;
//    	Kd_q <<   1,  10,  10,   1,  10,  10,   200,   1,  10,  10,   1,  10,  10;

        Kp_y = 0.001;
        Ki_y = 0.003;

        target_kp_roll = 0;
        target_kd_roll = 0.0;
        
//        target_kp_roll = 2;
//        target_kd_roll = 0.01;
    }
    else if(Mode == MODE_ACTUAL_ROBOT){
    	foot_z_offset << 0.0, -0.0, -0.0, -0.0 ;
        foot_height = 0.05;

//        Kp_q << 30, 500, 600, 30, 500, 600, 20000, 30, 500, 600, 30, 500, 600;
//        Kd_q <<  1,  10,  20,  1,  10,  20,   200,  1,  10,  20,  1,  10,  20;

        Kp_q << 30, 400, 400, 30, 400, 400, 20000, 30, 400, 400, 30, 400, 400;
        Kd_q <<  0.5,  10,  10,  0.5,  10,  10,   200,  0.5,  10,  10,  0.5,  10,  10;

        Kp_y = 0.0005;
        Ki_y = 0.003;

        target_kp_roll = 5;
        target_kd_roll = 0.03;
    }
    
//    foot_height = 0.05;
    com_height = 0.45; // 0.47 // 0.43
    moving_speed = 0;
    
    init_foot_l_pos << 0,  0.105,    0;
    init_foot_r_pos << 0, -0.105,    0;
    init_com_pos    << 0,     0, com_height;

//    foot_z_offset << 0.0, 0.0, -0.0, 0.00;
//    foot_z_offset << 0.0, -0.01, -0.0, -0.0 ;
    
    VectorNd tmp_init_foot_l_pos = VectorNd::Zero(3);
    VectorNd tmp_init_foot_r_pos = VectorNd::Zero(3);

    tmp_init_foot_l_pos = init_foot_l_pos - init_com_pos;
    tmp_init_foot_r_pos = init_foot_r_pos - init_com_pos;

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

    init_target_pos << 0, 45, -90, 0, 45, -90, 0, 0, 45, -90, 0, 45, -90;
    goal_EP << tmp_init_foot_l_pos(0), tmp_init_foot_l_pos(1), tmp_init_foot_l_pos(2) + foot_z_offset(0), tmp_init_foot_r_pos(0), tmp_init_foot_r_pos(1), tmp_init_foot_r_pos(2) + foot_z_offset(1), tmp_init_foot_l_pos(0), tmp_init_foot_l_pos(1), tmp_init_foot_l_pos(2) + foot_z_offset(2), tmp_init_foot_r_pos(0), tmp_init_foot_r_pos(1), tmp_init_foot_r_pos(2) + foot_z_offset(3);
    
    pre_target_EP = goal_EP;

    Get_gain(); // for preview control

    home_pos_time = 2;

    for (unsigned int i = 0; i < 13; ++i) {
        joint[i].torque = 0;
    }
    
    IMURoll = 0;
    IMUPitch = 0;
    IMUYaw = 0;
    
// =============== Flying trot parameters initialize =============== //
  
    ts = 0.10;
    tf = 0.07;
    ft_step_time = ts + tf;
    
    ts_cnt = 100;
    tf_cnt = 70;
    ft_step_cnt = ts_cnt + tf_cnt;
    
    
    h_0 = init_com_pos(2);
    v_0 = 0;
    a_0 = 0;
    
    v_1 = 0.1;
    a_1 = -GRAVITY;
    
    h_2 = init_com_pos(2);
    v_2 = -0.0; // -0.3
    a_2 = -GRAVITY;
    
    h_3 = init_com_pos(2);
    v_3 = 0;
    a_3 = 0;
    
    h_1 = 0.5*GRAVITY*tf*tf - v_1*tf + h_2;
    
    swing_foot_height = 0.05;
    
    flying_trot_final_flag = false;    
    CP_check_flag = true;
    CP_PHASE = 0;
    get_cp_done_flag = false;
//    CP_first_step_flag = true;
    CP_move_step = 0;
    CP_move_done_flag = false;
    
    cp_y_limit = 0.08;
    cp_com_limit = 0.015;
    
    jump_ready_height = 0.35;

    jump_ready_time = 2, jump_ready_cnt = 2000;
    jump_stance_time = 0.3, jump_stance_cnt = 300;
    jump_flight_time = 0.2, jump_flight_cnt = 200;
    jump_landing_time = 0.2, jump_landing_cnt = 200;
    
    tmp_Fc1 = 100;//100;
    tmp_Fc2 = tmp_Fc1 * 2.0;

    Kp_cp = 0.2;
     
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

    if(Body_Ori_Con_onoff_flag == true){
        Body_Ori_Con2();
    }
    
    tmp_target_EP = target_EP + target_EP_offset;
    
    actual_EP = FK1(actual_pos);

    target_pos = IK1(tmp_target_EP);
    
    Damping_con();
    
    Get_act_com();
    
    

    for (unsigned int i = 0; i < 13; ++i) {
        target_vel[i] = (target_pos_with_con[i] - pre_target_pos[i]) / dt;
        pre_target_pos[i] = target_pos_with_con[i];

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

    actual_EP_vel = x_dot.block(7, 0, 12, 1);

    for (unsigned int i = 0; i < 12; ++i) {
//        EP_vel_err[i] = target_EP_vel[i] - actual_EP_vel[i];
        EP_err[i] = tmp_target_EP[i] - actual_EP[i];
        tmp_data[i] = EP_err[i];
    }

    Cal_CP2();

    Cal_Fc();
    
    for (unsigned int i = 0; i < 6; ++i) {
        RobotState2dot[i] = 0;
        pd_con[i] = 0;
    }
    for (unsigned int i = 0; i < 13; ++i) {
        RobotState2dot[i + 6] = target_acc[i];
        pd_con[i + 6] = Kp_q[i]*(target_pos_with_con[i] - actual_pos[i]) + Kd_q[i]*(target_vel[i] - actual_vel[i]);
//        RobotState2dot[i + 6] = Kp_q[i]*(target_pos[i] - actual_pos[i]) + Kd_q[i]*(target_vel[i] - lpf_actual_vel[i]);
//        RobotState2dot[i + 6] = Kp_q[i]*(target_pos[i] - actual_pos[i]) + Kd_q[i]*(0 - actual_vel[i]);
    }

    CompositeRigidBodyAlgorithm(*m_pModel, RobotState, M_term, true);
    NonlinearEffects(*m_pModel, RobotState, RobotStatedot, hatNonLinearEffects);
    NonlinearEffects(*m_pModel, RobotState, VectorNd::Zero(m_pModel->dof_count), G_term);

    C_term = hatNonLinearEffects - G_term;

//    CTC_Torque = M_term * RobotState2dot + C_term + G_term + J_A.transpose() * Fc; // old
//    CTC_Torque = RobotState2dot + C_term + G_term + J_A.transpose() * Fc;
    CTC_Torque = C_term + G_term + J_A.transpose() * Fc + pd_con;
    for (int nJoint = 0; nJoint < nDOF; nJoint++) {
        joint[nJoint].torque = CTC_Torque(6 + nJoint);
    }
}

void CRobot::FTsensorTransformation()
{
//    RL.T_matrix = CalcBodyWorldOrientation(*m_pModel, RobotState, RL.ID, true);
//    RR.T_matrix = CalcBodyWorldOrientation(*m_pModel, RobotState, RR.ID, true);
//    FL.T_matrix = CalcBodyWorldOrientation(*m_pModel, RobotState, FL.ID, true);
//    FR.T_matrix = CalcBodyWorldOrientation(*m_pModel, RobotState, FR.ID, true);
    
    C_I_roll<<1,0,0,0,cos(base.currentRoll),-sin(base.currentRoll),0,sin(base.currentRoll),cos(base.currentRoll);
    
    C_I_pitch<<cos(base.currentPitch),0,sin(base.currentPitch),0,1,0,-sin(base.currentPitch),0,cos(base.currentPitch);
  
    RL_C_I_HP<<1,0,0,0,cos(actual_pos[0]),-sin(actual_pos[0]),0,sin(actual_pos[0]),cos(actual_pos[0]);
    RL_C_HP_HR<<cos(actual_pos[1]),0,sin(actual_pos[1]),0,1,0,-sin(actual_pos[1]),0,cos(actual_pos[1]);
    RL_C_HR_KN<<cos(actual_pos[2]),0,sin(actual_pos[2]),0,1,0,-sin(actual_pos[2]),0,cos(actual_pos[2]);
    RL_C_KN_TIP << cos(0), 0, sin(0), 0, 1, 0, -sin(0), 0, cos(0);
    RL.T_matrix=C_I_roll*C_I_pitch*RL_C_I_HP * RL_C_HP_HR * RL_C_HR_KN * RL_C_KN_TIP;
    
    RR_C_I_HP<<1,0,0,0,cos(actual_pos[3]),-sin(actual_pos[3]),0,sin(actual_pos[3]),cos(actual_pos[3]);
    RR_C_HP_HR<<cos(actual_pos[4]),0,sin(actual_pos[4]),0,1,0,-sin(actual_pos[4]),0,cos(actual_pos[4]);
    RR_C_HR_KN<<cos(actual_pos[5]),0,sin(actual_pos[5]),0,1,0,-sin(actual_pos[5]),0,cos(actual_pos[5]);
    RR_C_KN_TIP << cos(0), 0, sin(0), 0, 1, 0, -sin(0), 0, cos(0);
    RR.T_matrix=C_I_roll*C_I_pitch*RR_C_I_HP * RR_C_HP_HR * RR_C_HR_KN * RR_C_KN_TIP;
    
    FL_C_I_HP<<1,0,0,0,cos(actual_pos[7]),-sin(actual_pos[7]),0,sin(actual_pos[7]),cos(actual_pos[7]);
    FL_C_HP_HR<<cos(actual_pos[8]),0,sin(actual_pos[8]),0,1,0,-sin(actual_pos[8]),0,cos(actual_pos[8]);
    FL_C_HR_KN<<cos(actual_pos[9]),0,sin(actual_pos[9]),0,1,0,-sin(actual_pos[9]),0,cos(actual_pos[9]);
    FL_C_KN_TIP << cos(0), 0, sin(0), 0, 1, 0, -sin(0), 0, cos(0);
    FL.T_matrix=C_I_roll*C_I_pitch*FL_C_I_HP * FL_C_HP_HR * FL_C_HR_KN * FL_C_KN_TIP;
    
    FR_C_I_HP<<1,0,0,0,cos(actual_pos[10]),-sin(actual_pos[10]),0,sin(actual_pos[10]),cos(actual_pos[10]);
    FR_C_HP_HR<<cos(actual_pos[11]),0,sin(actual_pos[11]),0,1,0,-sin(actual_pos[11]),0,cos(actual_pos[11]);
    FR_C_HR_KN<<cos(actual_pos[12]),0,sin(actual_pos[12]),0,1,0,-sin(actual_pos[12]),0,cos(actual_pos[12]);
    FR_C_KN_TIP << cos(0), 0, sin(0), 0, 1, 0, -sin(0), 0, cos(0);
    FR.T_matrix=C_I_roll*C_I_pitch*FR_C_I_HP * FR_C_HP_HR * FR_C_HR_KN * FR_C_KN_TIP;
    
}

VectorNd CRobot::FK1(VectorNd q)
{
    const double L1 = 0.105;
    const double L2 = 0.305;
    const double L3 = 0.30946;//0.305;

    static double q1 = 0;
    static double q2 = 0;
    static double q3 = 0;

    q1 =  q[0];
    q2 = -q[1];
    q3 = -q[2];

    actual_EP[0] = -(-L3 * (cos(q2) * sin(q3) + cos(q3) * sin(q2)) - L2 * sin(q2));
    actual_EP[1] = L1 * cos(q1) - L3 * (sin(q1) * sin(q2) * sin(q3) - cos(q2) * cos(q3) * sin(q1)) + L2 * cos(q2) * sin(q1);
    actual_EP[2] = L1 * sin(q1) - L3 * (cos(q1) * cos(q2) * cos(q3) - cos(q1) * sin(q2) * sin(q3)) - L2 * cos(q1) * cos(q2);

    q1 =  q[3];
    q2 = -q[4];
    q3 = -q[5];

    actual_EP[3] = -(-L3 * (cos(q2) * sin(q3) + cos(q3) * sin(q2)) - L2 * sin(q2));
    actual_EP[4] = L2 * cos(q2) * sin(q1) - L1 * cos(q1) - L3 * (sin(q1) * sin(q2) * sin(q3) - cos(q2) * cos(q3) * sin(q1));
    actual_EP[5] = -L1 * sin(q1) - L3 * (cos(q1) * cos(q2) * cos(q3) - cos(q1) * sin(q2) * sin(q3)) - L2 * cos(q1) * cos(q2);

    q1 =  q[7];
    q2 = -q[8];
    q3 = -q[9];

    actual_EP[6] = -(-L3 * (cos(q2) * sin(q3) + cos(q3) * sin(q2)) - L2 * sin(q2));
    actual_EP[7] = (L1 * cos(q1) - L3 * (sin(q1) * sin(q2) * sin(q3) - cos(q2) * cos(q3) * sin(q1)) + L2 * cos(q2) * sin(q1));
    actual_EP[8] = L1 * sin(q1) - L3 * (cos(q1) * cos(q2) * cos(q3) - cos(q1) * sin(q2) * sin(q3)) - L2 * cos(q1) * cos(q2);

    q1 =  q[10];
    q2 = -q[11];
    q3 = -q[12];
    actual_EP[9] = -(-L3 * (cos(q2) * sin(q3) + cos(q3) * sin(q2)) - L2 * sin(q2));
    actual_EP[10] = L2 * cos(q2) * sin(q1) - L1 * cos(q1) - L3 * (sin(q1) * sin(q2) * sin(q3) - cos(q2) * cos(q3) * sin(q1));
    actual_EP[11] = -L1 * sin(q1) - L3 * (cos(q1) * cos(q2) * cos(q3) - cos(q1) * sin(q2) * sin(q3)) - L2 * cos(q1) * cos(q2);

    return actual_EP;
}

VectorNd CRobot::IK1(VectorNd EP)
{
    const double L1 = 0.105;
    const double L2 = 0.305;
    const double L3 = 0.30946;//0.305;

    static double x = 0;
    static double y = 0;
    static double z = 0;

    x = -EP[0];
    y = EP[1];
    z = EP[2];

    target_pos[0] = atan(y / abs(z)) - PI / 2 + acos(L1 / (sqrt(pow(y, 2) + pow(z, 2))));
    target_pos[1] = -(-atan(x / sqrt(abs(-pow(L1, 2) + pow(y, 2) + pow(z, 2)))) - acos((-pow(L1, 2) + pow(L2, 2) - pow(L3, 2) + pow(x, 2) + pow(y, 2) + pow(z, 2)) / (2 * L2 * sqrt(-pow(L1, 2) + pow(x, 2) + pow(y, 2) + pow(z, 2)))));
    target_pos[2] = -(PI - acos((pow(L1, 2) + pow(L2, 2) + pow(L3, 2) - pow(x, 2) - pow(y, 2) - pow(z, 2)) / (2 * L2 * L3)));

    x = -EP[3];
    y = EP[4];
    z = EP[5];

    target_pos[3] = PI / 2 + atan(y / abs(z)) - acos(L1 / sqrt(pow(y, 2) + pow(z, 2))); //PI/2 - acos(L1/sqrt(pow(y,2) + pow(z,2))) - atan(abs(y)/abs(z)); //-((atan(y/abs(z)) - PI/2 + acos(L1/(sqrt(pow(y,2) + pow(z,2))))));
    target_pos[4] = -(-atan(x / sqrt(abs(-pow(L1, 2) + pow(y, 2) + pow(z, 2)))) - acos((-pow(L1, 2) + pow(L2, 2) - pow(L3, 2) + pow(x, 2) + pow(y, 2) + pow(z, 2)) / (2 * L2 * sqrt(-pow(L1, 2) + pow(x, 2) + pow(y, 2) + pow(z, 2)))));
    target_pos[5] = -(PI - acos((pow(L1, 2) + pow(L2, 2) + pow(L3, 2) - pow(x, 2) - pow(y, 2) - pow(z, 2)) / (2 * L2 * L3)));

    x = -EP[6];
    y = EP[7];
    z = EP[8];

    target_pos[7] = atan(y / abs(z)) - PI / 2 + acos(L1 / (sqrt(pow(y, 2) + pow(z, 2))));
    target_pos[8] = -(-atan(x / sqrt(abs(-pow(L1, 2) + pow(y, 2) + pow(z, 2)))) - acos((-pow(L1, 2) + pow(L2, 2) - pow(L3, 2) + pow(x, 2) + pow(y, 2) + pow(z, 2)) / (2 * L2 * sqrt(-pow(L1, 2) + pow(x, 2) + pow(y, 2) + pow(z, 2)))));
    target_pos[9] = -(PI - acos((pow(L1, 2) + pow(L2, 2) + pow(L3, 2) - pow(x, 2) - pow(y, 2) - pow(z, 2)) / (2 * L2 * L3)));

    x = -EP[9];
    y = EP[10];
    z = EP[11];

    target_pos[10] = PI / 2 + atan(y / abs(z)) - acos(L1 / sqrt(pow(y, 2) + pow(z, 2))); //PI/2 - acos(L1/sqrt(pow(y,2) + pow(z,2))) - atan(abs(y)/abs(z)); //-((atan(y/abs(z)) - PI/2 + acos(L1/(sqrt(pow(y,2) + pow(z,2))))));
    target_pos[11] = -(-atan(x / sqrt(abs(-pow(L1, 2) + pow(y, 2) + pow(z, 2)))) - acos((-pow(L1, 2) + pow(L2, 2) - pow(L3, 2) + pow(x, 2) + pow(y, 2) + pow(z, 2)) / (2 * L2 * sqrt(-pow(L1, 2) + pow(x, 2) + pow(y, 2) + pow(z, 2)))));
    target_pos[12] = -(PI - acos((pow(L1, 2) + pow(L2, 2) + pow(L3, 2) - pow(x, 2) - pow(y, 2) - pow(z, 2)) / (2 * L2 * L3)));

    return target_pos;
}

void CRobot::Init_Pos_Traj(void)
{
    joint[0].torque = 500 * (init_target_pos[0] * D2R - actual_pos[0]) + 5 * (0 - actual_vel[0]); //RL_HIP
    joint[1].torque = 500 * (init_target_pos[1] * D2R - actual_pos[1]) + 5 * (0 - actual_vel[1]); //RL_THIGH
    joint[2].torque = 500 * (init_target_pos[2] * D2R - actual_pos[2]) + 5 * (0 - actual_vel[2]); //RL_CALF
    joint[3].torque = 500 * (init_target_pos[3] * D2R - actual_pos[3]) + 5 * (0 - actual_vel[3]); //RR_HIP
    joint[4].torque = 500 * (init_target_pos[4] * D2R - actual_pos[4]) + 5 * (0 - actual_vel[4]); //RR_THIGH
    joint[5].torque = 500 * (init_target_pos[5] * D2R - actual_pos[5]) + 5 * (0 - actual_vel[5]); //RR_CALF
    
    joint[6].torque = 500 * (init_target_pos[6] * D2R - actual_pos[6]) + 5 * (0 - actual_vel[6]); //WAIST

    joint[7].torque = 500 * (init_target_pos[7] * D2R - actual_pos[7]) + 5 * (0 - actual_vel[7]); //FL_HIP
    joint[8].torque = 500 * (init_target_pos[8] * D2R - actual_pos[8]) + 5 * (0 - actual_vel[8]); //FL_THIGH
    joint[9].torque = 500 * (init_target_pos[9] * D2R - actual_pos[9]) + 5 * (0 - actual_vel[9]); //FL_CALF
    joint[10].torque = 500 * (init_target_pos[10] * D2R - actual_pos[10]) + 5 * (0 - actual_vel[10]); //FR_HIP
    joint[11].torque = 500 * (init_target_pos[11] * D2R - actual_pos[11]) + 5 * (0 - actual_vel[11]); //FR_THIGH
    joint[12].torque = 500 * (init_target_pos[12] * D2R - actual_pos[12]) + 5 * (0 - actual_vel[12]); //FR_CALF
}

void CRobot::Home_Pos_Traj(void)
{
    // this is commented out by HSKIM(to subdivide the forces depending on the Modes.)

    if (ctc_cnt == 0) {
        FC_PHASE = INIT_Fc; 
        
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



        kp_roll = target_kp_roll / 2.0 * (1 - cos(PI2 / (home_pos_time * 2)*(double) (ctc_cnt) * dt));
        kd_roll = target_kd_roll / 2.0 * (1 - cos(PI2 / (home_pos_time * 2)*(double) (ctc_cnt) * dt));

        ctc_cnt++;
        
        if(ctc_cnt == (unsigned int) (home_pos_time / dt)){
            walk_ready_moving_done_flag = true;
            CP_moving_flag = false;
            CP_init_flag = true;
            CP_moving_start_flag = true;
            cout << "!! moving done !!" << endl;
            
            FC_PHASE = STOP; 
        }

    }

    else {
        for (unsigned int i = 0; i < 12; ++i) {
            target_EP[i] = goal_EP[i];
            target_EP_vel[i] = 0;
            target_EP_acc[i] = 0;
        }
        // waist
        target_pos[6] = 0;
    }
}

void CRobot::One_Step_Standing_Jump(void){

    const double tmp_t1 = jump_ready_cnt;
    const double tmp_t2 = tmp_t1 + jump_stance_cnt;
    const double tmp_t3 = tmp_t2 + jump_flight_cnt;
    const double tmp_t4 = tmp_t3 + jump_landing_cnt;
    const double tmp_t5 = tmp_t4 + jump_ready_cnt;
    static int tmp_jump_cnt = 0;
    static double tmp_jump_time = 0;
    
    if(moving_cnt < tmp_t1){
        JUMP_PHASE = 1;
    }
    else if(moving_cnt < tmp_t2){
        JUMP_PHASE = 2; 
    }
    else if(moving_cnt < tmp_t3){
        JUMP_PHASE = 3;
    }
    else if(moving_cnt < tmp_t4){
        JUMP_PHASE = 4;
    }
    else if(moving_cnt < tmp_t5){
        JUMP_PHASE = 5;
    }
    else{
        JUMP_PHASE = 6;
    }

    
    
    switch (JUMP_PHASE) {
    case 1: // Jump Ready
        tmp_jump_cnt = moving_cnt;
        tmp_jump_time = (double)tmp_jump_cnt*dt;
        
        if(tmp_jump_cnt == 0){
            foot_l_pos = init_foot_l_pos;
            foot_r_pos = init_foot_r_pos;
            com_pos = init_com_pos;
            
            Jump_COM_Z_Traj_Gen();
        }
        else{
            com_pos[2] = init_com_pos[2] + (jump_ready_height - init_com_pos[2])/2.0*(1-cos(PI2 / (jump_ready_time * 2)*tmp_jump_time)); 
        }
        break;
        
    case 2: // Take-off
        tmp_jump_cnt = moving_cnt - tmp_t1;
        tmp_jump_time = (double)tmp_jump_cnt*dt;
        
        foot_l_pos = init_foot_l_pos;
        foot_r_pos = init_foot_r_pos;
        com_pos = init_com_pos;
            
        com_pos[2] = jump_z1[5]*pow(tmp_jump_time,5) + jump_z1[4]*pow(tmp_jump_time,4) + jump_z1[3]*pow(tmp_jump_time,3) + jump_z1[2]*pow(tmp_jump_time,2) + jump_z1[1]*pow(tmp_jump_time,1) + jump_z1[0];
        
        break;
        
    case 3: // Flight
        tmp_jump_cnt = moving_cnt - tmp_t2;
        tmp_jump_time = (double)tmp_jump_cnt*dt;
        
        foot_l_pos = init_foot_l_pos;
        foot_r_pos = init_foot_r_pos;
        com_pos = init_com_pos;
            
        com_pos[2] = jump_z2[5]*pow(tmp_jump_time,5) + jump_z2[4]*pow(tmp_jump_time,4) + jump_z2[3]*pow(tmp_jump_time,3) + jump_z2[2]*pow(tmp_jump_time,2) + jump_z2[1]*pow(tmp_jump_time,1) + jump_z2[0];
        
        break;
        
    case 4: // Landing
        tmp_jump_cnt = moving_cnt - tmp_t3;
        tmp_jump_time = (double)tmp_jump_cnt*dt;
        
        foot_l_pos = init_foot_l_pos;
        foot_r_pos = init_foot_r_pos;
        com_pos = init_com_pos;
            
        com_pos[2] = jump_z4[5]*pow(tmp_jump_time,5) + jump_z4[4]*pow(tmp_jump_time,4) + jump_z4[3]*pow(tmp_jump_time,3) + jump_z4[2]*pow(tmp_jump_time,2) + jump_z4[1]*pow(tmp_jump_time,1) + jump_z4[0];
        
        break;
        
        
    case 5: // Walk Ready
        tmp_jump_cnt = moving_cnt - tmp_t4;
        tmp_jump_time = (double)tmp_jump_cnt*dt;
        
        foot_l_pos = init_foot_l_pos;
        foot_r_pos = init_foot_r_pos;
        com_pos = init_com_pos;
        
        com_pos[2] = jump_ready_height + (init_com_pos[2] - jump_ready_height)/2.0*(1-cos(PI2 / (jump_ready_time * 2)*(double) (tmp_jump_cnt) * dt));
        
        break;
        
    }
    moving_cnt++;
    
    // ============================ target_EP ========================== //
    local_foot_l_pos = foot_l_pos - com_pos; // foot position from global to local
    local_foot_r_pos = foot_r_pos - com_pos;
    
    
    target_EP[0] =   local_foot_l_pos[0];
    target_EP[1] =   local_foot_l_pos[1];
    target_EP[2] =   local_foot_l_pos[2] + foot_z_offset(0);
    target_EP[3] =   local_foot_r_pos[0];
    target_EP[4] =   local_foot_r_pos[1];
    target_EP[5] =   local_foot_r_pos[2] + foot_z_offset(1);
    target_EP[6] =   local_foot_r_pos[0];
    target_EP[7] =  -local_foot_r_pos[1];
    target_EP[8] =   local_foot_r_pos[2] + foot_z_offset(2);
    target_EP[9] =   local_foot_l_pos[0];
    target_EP[10] = -local_foot_l_pos[1];
    target_EP[11] =  local_foot_l_pos[2] + foot_z_offset(3);

    ctc_cnt2++;

    target_pos[6] = 0; //goal_pos[6];
}

void CRobot::Jump_COM_Z_Traj_Gen(void){
    
    const double jump_h_0 = jump_ready_height;
    const double jump_v_0 = 0;
    const double jump_a_0 = 0;
    
    const double jump_v_1 = 0.5;
    const double jump_a_1 = -GRAVITY;
    
    const double jump_h_2 = jump_h_0;
    const double jump_v_2 = -0.1;
    const double jump_a_2 = -GRAVITY;
    
    const double jump_h_3 = jump_h_0;
    const double jump_v_3 = 0;
    const double jump_a_3 = 0;
    
    const double jump_h_1 = 0.5*GRAVITY*jump_flight_time*jump_flight_time - jump_v_1*jump_flight_time + jump_h_2;
      
    init_x[0] = jump_h_0;
    init_x[1] = jump_v_0;
    init_x[2] = jump_a_0;

    final_x[0] = jump_h_1;
    final_x[1] = jump_v_1;
    final_x[2] = jump_a_1;

    coefficient_5thPoly(init_x, final_x, jump_stance_time, jump_z1);
    
    init_x[0] = jump_h_1;
    init_x[1] = jump_v_1;
    init_x[2] = jump_a_1;

    final_x[0] = jump_h_2;
    final_x[1] = jump_v_2;
    final_x[2] = jump_a_2;

    coefficient_5thPoly(init_x, final_x, jump_flight_time, jump_z2);
    
    init_x[0] = jump_h_2;
    init_x[1] = jump_v_2;
    init_x[2] = jump_a_2;

    final_x[0] = jump_h_1;
    final_x[1] = jump_v_1;
    final_x[2] = jump_a_1;

    coefficient_5thPoly(init_x, final_x, jump_stance_time, jump_z3);
    
    init_x[0] = jump_h_2;
    init_x[1] = jump_v_2;
    init_x[2] = jump_a_2;

    final_x[0] = jump_h_3;
    final_x[1] = jump_v_3;
    final_x[2] = jump_a_3;

    coefficient_5thPoly(init_x, final_x, jump_landing_time, jump_z4);
}


void CRobot::CP_Con(void){
    if(walk_ready_moving_done_flag == true){
    
        if(CP_init_flag == true){
        
            cout << "CP init!!" << endl;

            for (unsigned int i = 0; i < 12; ++i) {
                init_EP[i] = target_EP[i];
            }

            CP_init_flag = false;
        }
            
//        cout << "[1] CP_y = " << CP_y << endl;
        
        if((CP_y > cp_y_limit || CP_y < -cp_y_limit) && CP_moving_start_flag == true){
//        if((actual_com_pos[1] > cp_com_limit || actual_com_pos[1] < -cp_com_limit) && CP_moving_start_flag == true){
            
            cout << "CP_y = " << CP_y << endl;

            CP_moving_flag = true;
            CP_moving_start_flag = false;
        }
        
        if(CP_moving_flag == true){
            CP_foot_traj_gen();
        }
        
        target_EP[0]  = init_EP[0] + cp_foot_l_3d[0];
        target_EP[1]  = init_EP[1] + cp_foot_l_3d[1];
        target_EP[2]  = init_EP[2] + cp_foot_l_3d[2];
        target_EP[3]  = init_EP[3] + cp_foot_r_3d[0];
        target_EP[4]  = init_EP[4] + cp_foot_r_3d[1];
        target_EP[5]  = init_EP[5] + cp_foot_r_3d[2];
        target_EP[6]  = init_EP[6] + cp_foot_r_3d[0];
        target_EP[7]  = init_EP[7] + cp_foot_r_3d[1];
        target_EP[8]  = init_EP[8] + cp_foot_r_3d[2];
        target_EP[9]  = init_EP[9] + cp_foot_l_3d[0];
        target_EP[10] = init_EP[10] + cp_foot_l_3d[1];
        target_EP[11] = init_EP[11] + cp_foot_l_3d[2];

    }
}

void CRobot::CP_foot_traj_gen(void){

    static unsigned int cp_cnt = 0;
    static double cp_foot_height = 0.06;
    static double cp_foot_pos_y = 0;
    static double cp_limit_y = 0.15;
    
    if(cp_cnt == 0){
        FC_PHASE = STANCE_RRFL;
        
        init_cp_foot_l_3d << 0,0,0;
        init_cp_foot_r_3d << 0,0,0;
        
        cp_foot_l_3d = init_cp_foot_l_3d;
        cp_foot_r_3d = init_cp_foot_r_3d;

        if(CP_y > cp_y_limit){
            cp_foot_pos_y = cp_y_limit;
        }
        else if(CP_y < -cp_y_limit){
            cp_foot_pos_y = -cp_y_limit;
        }
        else{
            cp_foot_pos_y = CP_y;
        }
        cp_foot_pos_y = cp_foot_pos_y*0.5;//1.0;
        
        cout << "<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< cp_foot_pos_y = " << cp_foot_pos_y  << ">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>"  << endl;
        
        cp_cnt++;
    }
    else if(cp_cnt <= dsp_cnt){
        
        FC_PHASE = STANCE_RRFL;
        
        cp_foot_offset_y = Kp_cp*(CP_y-cp_foot_pos_y*2)/2.0;
        
        printf("cp_foot_offset_y = %f\n",cp_foot_offset_y);
        
        cp_foot_l_3d[0] = init_cp_foot_l_3d[0];
        cp_foot_l_3d[1] = init_cp_foot_l_3d[1] + (cp_foot_pos_y)/2.0*(1-cos(PI2 / (dsp_time * 2)*(double) (cp_cnt) * dt)) + cp_foot_offset_y;
        cp_foot_l_3d[2] = init_cp_foot_l_3d[2] + (cp_foot_height)/2.0*(1-cos(PI2 / (dsp_time)*(double) (cp_cnt) * dt));

        cp_foot_r_3d[0] = init_cp_foot_r_3d[0];
        cp_foot_r_3d[1] = init_cp_foot_r_3d[1] - (cp_foot_pos_y)/2.0*(1-cos(PI2 / (dsp_time * 2)*(double) (cp_cnt) * dt)) - cp_foot_offset_y;
        cp_foot_r_3d[2] = init_cp_foot_r_3d[2]; //init_foot_r_pos[2];

        cp_cnt++;
    }
    else if(cp_cnt <= step_cnt){
        
        FC_PHASE = STOP;

        cp_foot_l_3d[0] = init_cp_foot_l_3d[0];
        cp_foot_l_3d[1] = init_cp_foot_l_3d[1] + cp_foot_pos_y + cp_foot_offset_y;
        cp_foot_l_3d[2] = init_cp_foot_l_3d[2];

        cp_foot_r_3d[0] = init_cp_foot_r_3d[0];
        cp_foot_r_3d[1] = init_cp_foot_r_3d[1] - cp_foot_pos_y - cp_foot_offset_y;
        cp_foot_r_3d[2] = init_cp_foot_r_3d[2];
        
        cp_cnt++;
    }
    else if(cp_cnt <= step_cnt + dsp_cnt){

        FC_PHASE = STANCE_RLFR;
        
        cp_foot_l_3d[0] = init_cp_foot_l_3d[0];
        cp_foot_l_3d[1] = init_cp_foot_l_3d[1] + cp_foot_pos_y + cp_foot_offset_y - (cp_foot_pos_y + cp_foot_offset_y)/2.0*(1-cos(PI2 / (dsp_time * 2)*(double) (cp_cnt - step_cnt) * dt));
        cp_foot_l_3d[2] = init_cp_foot_l_3d[2];

        cp_foot_r_3d[0] = init_cp_foot_r_3d[0];
        cp_foot_r_3d[1] = init_cp_foot_r_3d[1] - cp_foot_pos_y - cp_foot_offset_y + (cp_foot_pos_y + cp_foot_offset_y)/2.0*(1-cos(PI2 / (dsp_time * 2)*(double) (cp_cnt - step_cnt) * dt));
        cp_foot_r_3d[2] = init_cp_foot_r_3d[2] + (cp_foot_height)/2.0*(1-cos(PI2 / (dsp_time)*(double) (cp_cnt - step_cnt) * dt));

        cp_cnt++;
    }
    else if(cp_cnt <= step_cnt*2){
        FC_PHASE = STOP;
        
        cp_foot_l_3d[0] = init_cp_foot_l_3d[0];
        cp_foot_l_3d[1] = init_cp_foot_l_3d[1];
        cp_foot_l_3d[2] = init_cp_foot_l_3d[2];

        cp_foot_r_3d[0] = init_cp_foot_r_3d[0];
        cp_foot_r_3d[1] = init_cp_foot_r_3d[1];
        cp_foot_r_3d[2] = init_cp_foot_r_3d[2];
        
        cp_cnt++;
    }
    
    else if(cp_cnt <= step_cnt*6){ //12

        FC_PHASE = STOP;
        
        cp_foot_l_3d[0] = init_cp_foot_l_3d[0];
        cp_foot_l_3d[1] = init_cp_foot_l_3d[1];
        cp_foot_l_3d[2] = init_cp_foot_l_3d[2];

        cp_foot_r_3d[0] = init_cp_foot_r_3d[0];
        cp_foot_r_3d[1] = init_cp_foot_r_3d[1];
        cp_foot_r_3d[2] = init_cp_foot_r_3d[2];
        
        cp_cnt++;
    }
    else{
        FC_PHASE = STOP;
//        CP_moving_flag = false;
        cp_foot_l_3d[0] = init_cp_foot_l_3d[0];
        cp_foot_l_3d[1] = init_cp_foot_l_3d[1];
        cp_foot_l_3d[2] = init_cp_foot_l_3d[2];

        cp_foot_r_3d[0] = init_cp_foot_r_3d[0];
        cp_foot_r_3d[1] = init_cp_foot_r_3d[1];
        cp_foot_r_3d[2] = init_cp_foot_r_3d[2];
        
        cp_cnt = 0;
//        CP_moving_start_flag = true;
        
        if(CP_y < cp_y_limit && CP_y > -cp_y_limit){
//        if(actual_com_pos[1] < cp_com_limit && actual_com_pos[1] > -cp_com_limit){
            CP_moving_flag = false;
            CP_moving_start_flag = true;
            cout << "CP CON DONE!~~~~~~~~~~~~~~" << endl;
        }
    }
    
//    cout << "cp_cnt = " << cp_cnt << "cp_foot_l_3d[1] = " << cp_foot_l_3d[1] << "cp_foot_r_3d[1] = " << cp_foot_r_3d[1] << endl;
    
}


















// ====================== flying trot trajectory generation ===================== //
void CRobot::Flying_Trot_Running(void)
{
    ft_time2=(double)ft_cnt2*dt;
    ft_cnt2++;
    ft_time = (double)ft_cnt*dt;
    
    if(ft_cnt == 0){
        FC_PHASE = STANCE_RRFL;
        
        com_pos = init_com_pos;
        foot_l_pos = init_foot_l_pos;
        foot_r_pos = init_foot_r_pos;   
        
        moving_speed = 0;// initial moving speed
        
        COM_FT_X_Traj_Gen();
        COM_FT_Z_Traj_Gen();
        SF_FT_X_Traj_Gen();
        SF_FT_Z_Traj_Gen();
    
    }
    else if(ft_cnt < ts_cnt){
        FC_PHASE = STANCE_RRFL;
        t2 = ft_time;
        
        com_pos[0] = c_com_x1[5]*pow(t2,5) + c_com_x1[4]*pow(t2,4) + c_com_x1[3]*pow(t2,3) + c_com_x1[2]*pow(t2,2) + c_com_x1[1]*pow(t2,1) + c_com_x1[0]; 
        com_pos[1] = init_com_pos(1);
        com_pos[2] = c_com_z1[5]*pow(t2,5) + c_com_z1[4]*pow(t2,4) + c_com_z1[3]*pow(t2,3) + c_com_z1[2]*pow(t2,2) + c_com_z1[1]*pow(t2,1) + c_com_z1[0];
        
        foot_l_pos[0] = c_sf_x1[5]*pow(t2,5) + c_sf_x1[4]*pow(t2,4) + c_sf_x1[3]*pow(t2,3) + c_sf_x1[2]*pow(t2,2) + c_sf_x1[1]*pow(t2,1) + c_sf_x1[0]; 
        foot_l_pos[1] = init_foot_l_pos(1);
        foot_l_pos[2] = c_sf_z1[5]*pow(t2,5) + c_sf_z1[4]*pow(t2,4) + c_sf_z1[3]*pow(t2,3) + c_sf_z1[2]*pow(t2,2) + c_sf_z1[1]*pow(t2,1) + c_sf_z1[0]; 
        
        foot_r_pos[0] = init_foot_r_pos(0);
        foot_r_pos[1] = init_foot_r_pos(1);
        foot_r_pos[2] = init_foot_r_pos(2);
        
        if(ft_cnt == ts_cnt - 1){
            flying_trot_init_flag = false;
        }
                
    }
    else if(ft_cnt < ft_step_cnt){
        FC_PHASE = ZERO;
        t2 = ft_time - ts;
        
//        printf("[1] t2 = %f\n",t2);
        
        com_pos[0] = moving_speed*(ts/2 + t2);
        com_pos[1] = init_com_pos(1);
        com_pos[2] = c_com_z2[5]*pow(t2,5) + c_com_z2[4]*pow(t2,4) + c_com_z2[3]*pow(t2,3) + c_com_z2[2]*pow(t2,2) + c_com_z2[1]*pow(t2,1) + c_com_z2[0];
        
        foot_l_pos[0] = moving_speed*ft_step_time; 
        foot_l_pos[1] = init_foot_l_pos(1);
        foot_l_pos[2] = c_sf_z3[5]*pow(t2,5) + c_sf_z3[4]*pow(t2,4) + c_sf_z3[3]*pow(t2,3) + c_sf_z3[2]*pow(t2,2) + c_sf_z3[1]*pow(t2,1) + c_sf_z3[0]; 
        
        foot_r_pos[0] = init_foot_r_pos(0);
        foot_r_pos[1] = init_foot_r_pos(1);
        foot_r_pos[2] = c_sf_z2[5]*pow(t2,5) + c_sf_z2[4]*pow(t2,4) + c_sf_z2[3]*pow(t2,3) + c_sf_z2[2]*pow(t2,2) + c_sf_z2[1]*pow(t2,1) + c_sf_z2[0]; 
    
    }
    else if(ft_cnt < ft_step_cnt + ts_cnt){
        FC_PHASE = STANCE_RLFR;
        t2 = ft_time - ft_step_time;
        
        com_pos[0] = moving_speed*(ts/2 + tf + t2);
        com_pos[1] = init_com_pos(1);
        com_pos[2] = c_com_z3[5]*pow(t2,5) + c_com_z3[4]*pow(t2,4) + c_com_z3[3]*pow(t2,3) + c_com_z3[2]*pow(t2,2) + c_com_z3[1]*pow(t2,1) + c_com_z3[0];
        
        foot_l_pos[0] = moving_speed*ft_step_time; 
        foot_l_pos[1] = init_foot_l_pos(1);
        foot_l_pos[2] = init_foot_l_pos(2);
        
        foot_r_pos[0] = c_sf_x2[5]*pow(t2,5) + c_sf_x2[4]*pow(t2,4) + c_sf_x2[3]*pow(t2,3) + c_sf_x2[2]*pow(t2,2) + c_sf_x2[1]*pow(t2,1) + c_sf_x2[0]; 
        foot_r_pos[1] = init_foot_r_pos(1);
        foot_r_pos[2] = swing_foot_height;
    
    }
    else if(ft_cnt < 2*ft_step_cnt){
        FC_PHASE = ZERO;
        t2 = ft_time - ft_step_time - ts;
        
        com_pos[0] = moving_speed*(ts/2 + ft_step_time + t2);
        com_pos[1] = init_com_pos(1);
        com_pos[2] = c_com_z2[5]*pow(t2,5) + c_com_z2[4]*pow(t2,4) + c_com_z2[3]*pow(t2,3) + c_com_z2[2]*pow(t2,2) + c_com_z2[1]*pow(t2,1) + c_com_z2[0];
        
        foot_l_pos[0] = moving_speed*ft_step_time; 
        foot_l_pos[1] = init_foot_l_pos(1);
        foot_l_pos[2] = c_sf_z2[5]*pow(t2,5) + c_sf_z2[4]*pow(t2,4) + c_sf_z2[3]*pow(t2,3) + c_sf_z2[2]*pow(t2,2) + c_sf_z2[1]*pow(t2,1) + c_sf_z2[0]; 
        
        foot_r_pos[0] = moving_speed*ft_step_time*2;  
        foot_r_pos[1] = init_foot_r_pos(1);
        foot_r_pos[2] = c_sf_z3[5]*pow(t2,5) + c_sf_z3[4]*pow(t2,4) + c_sf_z3[3]*pow(t2,3) + c_sf_z3[2]*pow(t2,2) + c_sf_z3[1]*pow(t2,1) + c_sf_z3[0]; 
    
    }
    else if(ft_cnt < 2*ft_step_cnt + ts_cnt){
        FC_PHASE = STANCE_RRFL;
        t2 = ft_time - 2*ft_step_time;
        
        com_pos[0] = moving_speed*(ts/2 + ft_step_time + tf + t2);
        com_pos[1] = init_com_pos(1);
        com_pos[2] = c_com_z3[5]*pow(t2,5) + c_com_z3[4]*pow(t2,4) + c_com_z3[3]*pow(t2,3) + c_com_z3[2]*pow(t2,2) + c_com_z3[1]*pow(t2,1) + c_com_z3[0];
        
        foot_l_pos[0] = moving_speed*ft_step_time + c_sf_x2[5]*pow(t2,5) + c_sf_x2[4]*pow(t2,4) + c_sf_x2[3]*pow(t2,3) + c_sf_x2[2]*pow(t2,2) + c_sf_x2[1]*pow(t2,1) + c_sf_x2[0]; 
        foot_l_pos[1] = init_foot_l_pos(1);
        foot_l_pos[2] = swing_foot_height;
        
        foot_r_pos[0] = moving_speed*ft_step_time*2;
        foot_r_pos[1] = init_foot_r_pos(1);
        foot_r_pos[2] = init_foot_r_pos(2);
    
    }
    else if(ft_cnt < 3*ft_step_cnt){
        FC_PHASE = ZERO;
        t2 = ft_time - 2*ft_step_time - ts;
        
        com_pos[0] = moving_speed*(ts/2 + ft_step_time*2 + t2);
        com_pos[1] = init_com_pos(1);
        com_pos[2] = c_com_z2[5]*pow(t2,5) + c_com_z2[4]*pow(t2,4) + c_com_z2[3]*pow(t2,3) + c_com_z2[2]*pow(t2,2) + c_com_z2[1]*pow(t2,1) + c_com_z2[0];
        
        foot_l_pos[0] = moving_speed*ft_step_time*3; 
        foot_l_pos[1] = init_foot_l_pos(1);
        foot_l_pos[2] = c_sf_z3[5]*pow(t2,5) + c_sf_z3[4]*pow(t2,4) + c_sf_z3[3]*pow(t2,3) + c_sf_z3[2]*pow(t2,2) + c_sf_z3[1]*pow(t2,1) + c_sf_z3[0];
        
        foot_r_pos[0] = moving_speed*ft_step_time*2;  
        foot_r_pos[1] = init_foot_r_pos(1);
        foot_r_pos[2] = c_sf_z2[5]*pow(t2,5) + c_sf_z2[4]*pow(t2,4) + c_sf_z2[3]*pow(t2,3) + c_sf_z2[2]*pow(t2,2) + c_sf_z2[1]*pow(t2,1) + c_sf_z2[0];
        
        
        if(ft_cnt == 3*ft_step_cnt - 1){

            pre_com_pos[0] = moving_speed*(ts/2 + ft_step_time*2 + tf);
            pre_com_pos[1] = init_com_pos(1);
            pre_com_pos[2] = c_com_z2[5]*pow(t2+dt,5) + c_com_z2[4]*pow(t2+dt,4) + c_com_z2[3]*pow(t2+dt,3) + c_com_z2[2]*pow(t2+dt,2) + c_com_z2[1]*pow(t2+dt,1) + c_com_z2[0];

            pre_foot_l_pos[0] = moving_speed*ft_step_time*3; 
            pre_foot_l_pos[1] = init_foot_l_pos(1);
            pre_foot_l_pos[2] = c_sf_z3[5]*pow(t2+dt,5) + c_sf_z3[4]*pow(t2+dt,4) + c_sf_z3[3]*pow(t2+dt,3) + c_sf_z3[2]*pow(t2+dt,2) + c_sf_z3[1]*pow(t2+dt,1) + c_sf_z3[0];

            pre_foot_r_pos[0] = moving_speed*ft_step_time*2;  
            pre_foot_r_pos[1] = init_foot_r_pos(1);
            pre_foot_r_pos[2] = c_sf_z2[5]*pow(t2+dt,5) + c_sf_z2[4]*pow(t2+dt,4) + c_sf_z2[3]*pow(t2+dt,3) + c_sf_z2[2]*pow(t2+dt,2) + c_sf_z2[1]*pow(t2+dt,1) + c_sf_z2[0];

            
            pre_moving_speed = moving_speed;
            moving_speed = pre_moving_speed + 0.1; // tmp_moving_speed
            
            // ====== trajectory is modified ====== //
            init_x[0] = 0;
            init_x[1] = pre_moving_speed;
            init_x[2] = 0;

            final_x[0] = pre_moving_speed*(ts)/2.0 + moving_speed*(ts)/2.0;
            final_x[1] = moving_speed;
            final_x[2] = 0;

            coefficient_5thPoly(init_x, final_x, ts, c_com_x4);
            
            init_x[0] = 0;
            init_x[1] = moving_speed;
            init_x[2] = 0;

            final_x[0] = moving_speed*(ts)/2.0;
            final_x[1] = 0;
            final_x[2] = 0;

            coefficient_5thPoly(init_x, final_x, ts, c_com_x5);
            
            init_x[0] = 0;
            init_x[1] = 0;
            init_x[2] = 0;

            final_x[0] = pre_moving_speed*(ft_step_time) + moving_speed*(ft_step_time);
            final_x[1] = 0;
            final_x[2] = 0;

            coefficient_5thPoly(init_x, final_x, ts, c_sf_x2);
            
            init_x[0] = 0;
            init_x[1] = 0;
            init_x[2] = 0;

            final_x[0] = moving_speed*(ft_step_time)*2;
            final_x[1] = 0;
            final_x[2] = 0;

            coefficient_5thPoly(init_x, final_x, ts, c_sf_x3);
                   
            init_x[0] = 0;
            init_x[1] = 0;
            init_x[2] = 0;

            final_x[0] = moving_speed*(ft_step_time);
            final_x[1] = 0;
            final_x[2] = 0;

            coefficient_5thPoly(init_x, final_x, ts, c_sf_x4);
             
        }
    }
    
    else if(ft_cnt < ft_step_cnt*3 + ts_cnt){
        FC_PHASE = STANCE_RLFR;
        t2 = ft_time - ft_step_time*3;
        
        com_pos[0] = pre_com_pos[0] + c_com_x4[5]*pow(t2,5) + c_com_x4[4]*pow(t2,4) + c_com_x4[3]*pow(t2,3) + c_com_x4[2]*pow(t2,2) + c_com_x4[1]*pow(t2,1) + c_com_x4[0];//moving_speed*(ts/2 + tf + t2);
        com_pos[1] = init_com_pos(1);
        com_pos[2] = c_com_z3[5]*pow(t2,5) + c_com_z3[4]*pow(t2,4) + c_com_z3[3]*pow(t2,3) + c_com_z3[2]*pow(t2,2) + c_com_z3[1]*pow(t2,1) + c_com_z3[0];
        
        foot_l_pos[0] = pre_foot_l_pos[0]; 
        foot_l_pos[1] = pre_foot_l_pos[1];
        foot_l_pos[2] = pre_foot_l_pos[2];
        
        foot_r_pos[0] = pre_foot_r_pos[0] + c_sf_x2[5]*pow(t2,5) + c_sf_x2[4]*pow(t2,4) + c_sf_x2[3]*pow(t2,3) + c_sf_x2[2]*pow(t2,2) + c_sf_x2[1]*pow(t2,1) + c_sf_x2[0]; 
        foot_r_pos[1] = pre_foot_r_pos[1];//init_foot_r_pos(1);
        foot_r_pos[2] = pre_foot_r_pos[2];//swing_foot_height;
    
    }
    else if(ft_cnt < 4*ft_step_cnt){
        FC_PHASE = ZERO;
        t2 = ft_time - ft_step_time*3 - ts;
        
        com_pos[0] = pre_com_pos[0] + pre_moving_speed*ts/2.0 + moving_speed*ts/2.0 + moving_speed*t2;//moving_speed*(ts/2 + ft_step_time + t2);
        com_pos[1] = init_com_pos(1);
        com_pos[2] = c_com_z2[5]*pow(t2,5) + c_com_z2[4]*pow(t2,4) + c_com_z2[3]*pow(t2,3) + c_com_z2[2]*pow(t2,2) + c_com_z2[1]*pow(t2,1) + c_com_z2[0];
        
        foot_l_pos[0] = pre_foot_l_pos[0];
        foot_l_pos[1] = pre_foot_l_pos[1];
        foot_l_pos[2] = c_sf_z2[5]*pow(t2,5) + c_sf_z2[4]*pow(t2,4) + c_sf_z2[3]*pow(t2,3) + c_sf_z2[2]*pow(t2,2) + c_sf_z2[1]*pow(t2,1) + c_sf_z2[0]; 
        
        foot_r_pos[0] = pre_foot_r_pos[0] + pre_moving_speed*ft_step_time + moving_speed*ft_step_time;  
        foot_r_pos[1] = pre_foot_r_pos[1];
        foot_r_pos[2] = c_sf_z3[5]*pow(t2,5) + c_sf_z3[4]*pow(t2,4) + c_sf_z3[3]*pow(t2,3) + c_sf_z3[2]*pow(t2,2) + c_sf_z3[1]*pow(t2,1) + c_sf_z3[0]; 
    
    }
    else if(ft_cnt < 4*ft_step_cnt + ts_cnt){
        FC_PHASE = STANCE_RRFL;
        t2 = ft_time - 4*ft_step_time;
        
        com_pos[0] = pre_com_pos[0] + pre_moving_speed*ts/2.0 + moving_speed*ts/2.0 + moving_speed*(tf + t2); //(ts/2 + ft_step_time + tf + t2);
        com_pos[1] = init_com_pos(1);
        com_pos[2] = c_com_z3[5]*pow(t2,5) + c_com_z3[4]*pow(t2,4) + c_com_z3[3]*pow(t2,3) + c_com_z3[2]*pow(t2,2) + c_com_z3[1]*pow(t2,1) + c_com_z3[0];
        
        foot_l_pos[0] = pre_foot_l_pos[0] + c_sf_x3[5]*pow(t2,5) + c_sf_x3[4]*pow(t2,4) + c_sf_x3[3]*pow(t2,3) + c_sf_x3[2]*pow(t2,2) + c_sf_x3[1]*pow(t2,1) + c_sf_x3[0]; 
        foot_l_pos[1] = init_foot_l_pos(1);
        foot_l_pos[2] = swing_foot_height;
        
        foot_r_pos[0] = pre_foot_r_pos[0] + pre_moving_speed*ft_step_time + moving_speed*ft_step_time;//moving_speed*ft_step_time*2;
        foot_r_pos[1] = init_foot_r_pos(1);
        foot_r_pos[2] = init_foot_r_pos(2);
    
    }
    else if(ft_cnt < 5*ft_step_cnt){
        FC_PHASE = ZERO;
        t2 = ft_time - 4*ft_step_time - ts;
        
        com_pos[0] =  pre_com_pos[0] + pre_moving_speed*ts/2.0 + moving_speed*ts/2.0 + moving_speed*(ft_step_time + t2);//moving_speed*(ts/2 + ft_step_time*2 + t2);
        com_pos[1] = init_com_pos(1);
        com_pos[2] = c_com_z2[5]*pow(t2,5) + c_com_z2[4]*pow(t2,4) + c_com_z2[3]*pow(t2,3) + c_com_z2[2]*pow(t2,2) + c_com_z2[1]*pow(t2,1) + c_com_z2[0];
        
        foot_l_pos[0] = pre_foot_l_pos[0] + moving_speed*ft_step_time*2; 
        foot_l_pos[1] = init_foot_l_pos(1);
        foot_l_pos[2] = c_sf_z3[5]*pow(t2,5) + c_sf_z3[4]*pow(t2,4) + c_sf_z3[3]*pow(t2,3) + c_sf_z3[2]*pow(t2,2) + c_sf_z3[1]*pow(t2,1) + c_sf_z3[0];
        
        foot_r_pos[0] = pre_foot_r_pos[0] + pre_moving_speed*ft_step_time + moving_speed*ft_step_time;//moving_speed*ft_step_time*2;  
        foot_r_pos[1] = init_foot_r_pos(1);
        foot_r_pos[2] = c_sf_z2[5]*pow(t2,5) + c_sf_z2[4]*pow(t2,4) + c_sf_z2[3]*pow(t2,3) + c_sf_z2[2]*pow(t2,2) + c_sf_z2[1]*pow(t2,1) + c_sf_z2[0];
        
        
        if(ft_cnt == 5*ft_step_cnt - 1){
            
            if(sub_ctrl_flag == false){
                ft_cnt = 3*ft_step_cnt - 1;
                
                pre_com_pos[0] = pre_com_pos[0] + pre_moving_speed*ts/2.0 + moving_speed*ts/2.0 + moving_speed*(ft_step_time + t2 + dt);
                pre_com_pos[1] = init_com_pos(1);
                pre_com_pos[2] = c_com_z2[5]*pow(t2+dt,5) + c_com_z2[4]*pow(t2+dt,4) + c_com_z2[3]*pow(t2+dt,3) + c_com_z2[2]*pow(t2+dt,2) + c_com_z2[1]*pow(t2+dt,1) + c_com_z2[0];

                pre_foot_l_pos[0] = pre_foot_l_pos[0] + moving_speed*ft_step_time*2; moving_speed*ft_step_time*3; 
                pre_foot_l_pos[1] = init_foot_l_pos(1);
                pre_foot_l_pos[2] = c_sf_z3[5]*pow(t2+dt,5) + c_sf_z3[4]*pow(t2+dt,4) + c_sf_z3[3]*pow(t2+dt,3) + c_sf_z3[2]*pow(t2+dt,2) + c_sf_z3[1]*pow(t2+dt,1) + c_sf_z3[0];

                pre_foot_r_pos[0] = pre_foot_r_pos[0] + pre_moving_speed*ft_step_time + moving_speed*ft_step_time;//moving_speed*ft_step_time*2;  
                pre_foot_r_pos[1] = init_foot_r_pos(1);
                pre_foot_r_pos[2] = c_sf_z2[5]*pow(t2+dt,5) + c_sf_z2[4]*pow(t2+dt,4) + c_sf_z2[3]*pow(t2+dt,3) + c_sf_z2[2]*pow(t2+dt,2) + c_sf_z2[1]*pow(t2+dt,1) + c_sf_z2[0];


//                pre_moving_speed = moving_speed;
//                moving_speed = tmp_moving_speed;
                
                pre_moving_speed = moving_speed;
                moving_speed = pre_moving_speed + 0.1; //
                
                if(moving_speed > 2.2){
                    moving_speed = 2.2;
                }
                // ====== trajectory is modified ====== //
                init_x[0] = 0;
                init_x[1] = pre_moving_speed;
                init_x[2] = 0;

                final_x[0] = pre_moving_speed*(ts)/2.0 + moving_speed*(ts)/2.0;
                final_x[1] = moving_speed;
                final_x[2] = 0;

                coefficient_5thPoly(init_x, final_x, ts, c_com_x4);

                init_x[0] = 0;
                init_x[1] = moving_speed;
                init_x[2] = 0;

                final_x[0] = moving_speed*(ts)/2.0;
                final_x[1] = 0;
                final_x[2] = 0;

                coefficient_5thPoly(init_x, final_x, ts, c_com_x5);

                init_x[0] = 0;
                init_x[1] = 0;
                init_x[2] = 0;

                final_x[0] = pre_moving_speed*(ft_step_time) + moving_speed*(ft_step_time);
                final_x[1] = 0;
                final_x[2] = 0;

                coefficient_5thPoly(init_x, final_x, ts, c_sf_x2);

                init_x[0] = 0;
                init_x[1] = 0;
                init_x[2] = 0;

                final_x[0] = moving_speed*(ft_step_time)*2;
                final_x[1] = 0;
                final_x[2] = 0;

                coefficient_5thPoly(init_x, final_x, ts, c_sf_x3);

                init_x[0] = 0;
                init_x[1] = 0;
                init_x[2] = 0;

                final_x[0] = moving_speed*(ft_step_time);
                final_x[1] = 0;
                final_x[2] = 0;

                coefficient_5thPoly(init_x, final_x, ts, c_sf_x4); 
            }
            
            printf("step_num = %d, moving_speed = %f\n",step_num,moving_speed);
            step_num++;
           
        } 
    }
    
    
    
    else if(ft_cnt < 5*ft_step_cnt + ts_cnt){
        FC_PHASE = STANCE_RLFR;
        t2 = ft_time - 5*ft_step_time;
        
        if(ft_cnt == 5*ft_step_cnt){
            flying_trot_final_flag = true;
            
//            printf("[1] flying_trot_final_flag = %d\n",flying_trot_final_flag);
        }

        
        com_pos[0] = pre_com_pos[0] + pre_moving_speed*ts/2.0 + moving_speed*ts/2.0 + moving_speed*(ft_step_time + tf) + c_com_x5[5]*pow(t2,5) + c_com_x5[4]*pow(t2,4) + c_com_x5[3]*pow(t2,3) + c_com_x5[2]*pow(t2,2) + c_com_x5[1]*pow(t2,1) + c_com_x5[0];// moving_speed*(ts/2 + ft_step_time*2 + tf) + c_com_x3[5]*pow(t2,5) + c_com_x3[4]*pow(t2,4) + c_com_x3[3]*pow(t2,3) + c_com_x3[2]*pow(t2,2) + c_com_x3[1]*pow(t2,1) + c_com_x3[0];
        com_pos[1] = init_com_pos(1);
        com_pos[2] = c_com_z4[5]*pow(t2,5) + c_com_z4[4]*pow(t2,4) + c_com_z4[3]*pow(t2,3) + c_com_z4[2]*pow(t2,2) + c_com_z4[1]*pow(t2,1) + c_com_z4[0];
        
        foot_l_pos[0] = pre_foot_l_pos[0] + moving_speed*ft_step_time*2;
        foot_l_pos[1] = init_foot_l_pos(1);
        foot_l_pos[2] = init_foot_l_pos(2);
        
        foot_r_pos[0] = pre_foot_r_pos[0] + pre_moving_speed*ft_step_time + moving_speed*ft_step_time + c_sf_x4[5]*pow(t2,5) + c_sf_x4[4]*pow(t2,4) + c_sf_x4[3]*pow(t2,3) + c_sf_x4[2]*pow(t2,2) + c_sf_x4[1]*pow(t2,1) + c_sf_x4[0];
        foot_r_pos[1] = init_foot_r_pos(1);
        foot_r_pos[2] = c_sf_z4[5]*pow(t2,5) + c_sf_z4[4]*pow(t2,4) + c_sf_z4[3]*pow(t2,3) + c_sf_z4[2]*pow(t2,2) + c_sf_z4[1]*pow(t2,1) + c_sf_z4[0];
        
    }
    else{
        FC_PHASE = STOP;
        
        if(ft_cnt == 5*ft_step_cnt + ts_cnt - 1){
            flying_trot_final_flag = false;
            
        }
        
        com_pos[0] = pre_com_pos[0] + pre_moving_speed*ts/2.0 + moving_speed*ft_step_time*2;//moving_speed*ft_step_time*3;
        com_pos[1] = init_com_pos(1);
        com_pos[2] = init_com_pos(2);
        
        foot_l_pos[0] = pre_foot_l_pos[0] + moving_speed*ft_step_time*2;//moving_speed*ft_step_time*3;
        foot_l_pos[1] = init_foot_l_pos(1);
        foot_l_pos[2] = init_foot_l_pos(2);
        
        foot_r_pos[0] = pre_foot_r_pos[0] + pre_moving_speed*ft_step_time + moving_speed*ft_step_time*2;//moving_speed*ft_step_time*3;
        foot_r_pos[1] = init_foot_r_pos(1);
        foot_r_pos[2] = init_foot_r_pos(2);
    
    }
    
    if(ft_cnt > 0){
        target_com_vel[0] = (com_pos[0] - old_com_pos[0])/dt;
        old_com_pos[0] = com_pos[0];
        if(ft_cnt > 1){
            target_com_acc[0] = (target_com_vel[0] - old_com_vel[0])/dt;
            old_com_vel[0] = target_com_vel[0];
        }
    }
    
    // ============================ target_EP ========================== //
    local_foot_l_pos = foot_l_pos - com_pos; // foot position from global to local
    local_foot_r_pos = foot_r_pos - com_pos;
    
    
    target_EP[0] =   local_foot_l_pos[0];
    target_EP[1] =   local_foot_l_pos[1];
    target_EP[2] =   local_foot_l_pos[2] + foot_z_offset(0);
    target_EP[3] =   local_foot_r_pos[0];
    target_EP[4] =   local_foot_r_pos[1];
    target_EP[5] =   local_foot_r_pos[2] + foot_z_offset(1);
    target_EP[6] =   local_foot_r_pos[0];
    target_EP[7] =  -local_foot_r_pos[1];
    target_EP[8] =   local_foot_r_pos[2] + foot_z_offset(2);
    target_EP[9] =   local_foot_l_pos[0];
    target_EP[10] = -local_foot_l_pos[1];
    target_EP[11] =  local_foot_l_pos[2] + foot_z_offset(3);

    ft_cnt++;

    target_pos[6] = 0; //goal_pos[6];

}


void CRobot::COM_FT_X_Traj_Gen(void){
    init_x[0] = 0;
    init_x[1] = 0;
    init_x[2] = 0;

    final_x[0] = moving_speed*(ts)/2.0;
    final_x[1] = moving_speed;
    final_x[2] = 0;

    coefficient_5thPoly(init_x, final_x, ts, c_com_x1);
    
    init_x[0] = 0;
    init_x[1] = moving_speed;
    init_x[2] = 0;

    final_x[0] = moving_speed*(ts);
    final_x[1] = 0;
    final_x[2] = 0;

    coefficient_5thPoly(init_x, final_x, ts, c_com_x2);
    
    init_x[0] = 0;
    init_x[1] = moving_speed;
    init_x[2] = 0;

    final_x[0] = moving_speed*(ts)/2.0;
    final_x[1] = 0;
    final_x[2] = 0;

    coefficient_5thPoly(init_x, final_x, ts, c_com_x3);

}

void CRobot::COM_FT_Z_Traj_Gen(void){
    init_x[0] = h_0;
    init_x[1] = v_0;
    init_x[2] = a_0;

    final_x[0] = h_1;
    final_x[1] = v_1;
    final_x[2] = a_1;

    coefficient_5thPoly(init_x, final_x, ts, c_com_z1);
    
    init_x[0] = h_1;
    init_x[1] = v_1;
    init_x[2] = a_1;

    final_x[0] = h_2;
    final_x[1] = v_2;
    final_x[2] = a_2;

    coefficient_5thPoly(init_x, final_x, tf, c_com_z2);
    
    init_x[0] = h_2;
    init_x[1] = v_2;
    init_x[2] = a_2;

    final_x[0] = h_1;
    final_x[1] = v_1;
    final_x[2] = a_1;

    coefficient_5thPoly(init_x, final_x, ts, c_com_z3);
    
    init_x[0] = h_2;
    init_x[1] = v_2;
    init_x[2] = a_2;

    final_x[0] = h_3;
    final_x[1] = v_3;
    final_x[2] = a_3;

    coefficient_5thPoly(init_x, final_x, ts, c_com_z4);

}

void CRobot::SF_FT_X_Traj_Gen(void){
    init_x[0] = 0;
    init_x[1] = 0;
    init_x[2] = 0;

    final_x[0] = moving_speed*(ft_step_time);
    final_x[1] = 0;
    final_x[2] = 0;

    coefficient_5thPoly(init_x, final_x, ts, c_sf_x1);
    
    init_x[0] = 0;
    init_x[1] = 0;
    init_x[2] = 0;

    final_x[0] = moving_speed*(2*ft_step_time);
    final_x[1] = 0;
    final_x[2] = 0;

    coefficient_5thPoly(init_x, final_x, ts, c_sf_x2);
}

void CRobot::SF_FT_Z_Traj_Gen(void){
    init_x[0] = 0;
    init_x[1] = 0;
    init_x[2] = 0;

    final_x[0] = swing_foot_height;
    final_x[1] = 0;
    final_x[2] = 0;

    coefficient_5thPoly(init_x, final_x, ts, c_sf_z1);
    
    init_x[0] = 0;
    init_x[1] = 0;
    init_x[2] = 0;

    final_x[0] = swing_foot_height;
    final_x[1] = 0;
    final_x[2] = 0;

    coefficient_5thPoly(init_x, final_x, tf, c_sf_z2);
    
    init_x[0] = swing_foot_height;
    init_x[1] = 0;
    init_x[2] = 0;

    final_x[0] = 0;
    final_x[1] = 0;
    final_x[2] = 0;

    coefficient_5thPoly(init_x, final_x, tf, c_sf_z3);
    
    init_x[0] = swing_foot_height;
    init_x[1] = 0;
    init_x[2] = 0;

    final_x[0] = 0;
    final_x[1] = 0;
    final_x[2] = 0;

    coefficient_5thPoly(init_x, final_x, ts, c_sf_z4);
}


void CRobot::Flying_Trot_Running_Traj(unsigned int i)
{

}

void CRobot::Flying_Trot_Running_Traj_Final(unsigned int i)
{

}


void CRobot::COM_Flying_Trot_Z_Traj_Gen(void){
    
    init_x[0] = h_0;
    init_x[1] = v_0;
    init_x[2] = a_0;

    final_x[0] = h_1;
    final_x[1] = v_1;
    final_x[2] = a_1;

    coefficient_5thPoly(init_x, final_x, ts, c_com_z1);
    
    init_x[0] = h_1;
    init_x[1] = v_1;
    init_x[2] = a_1;

    final_x[0] = h_2;
    final_x[1] = v_2;
    final_x[2] = a_2;

    coefficient_5thPoly(init_x, final_x, tf, c_com_z2);
    
    init_x[0] = h_2;
    init_x[1] = v_2;
    init_x[2] = a_2;

    final_x[0] = h_1;
    final_x[1] = v_1;
    final_x[2] = a_1;

    coefficient_5thPoly(init_x, final_x, ts, c_com_z3);
    
    init_x[0] = h_2;
    init_x[1] = v_2;
    init_x[2] = a_2;

    final_x[0] = h_3;
    final_x[1] = v_3;
    final_x[2] = a_3;

    coefficient_5thPoly(init_x, final_x, ts, c_com_z4);
    
    
}

void CRobot::SF_Flying_Trot_Z_Traj_Gen(void){
    
    init_x[0] = 0;
    init_x[1] = 0;
    init_x[2] = 0;

    final_x[0] = swing_foot_height;
    final_x[1] = 0;
    final_x[2] = 0;

    coefficient_5thPoly(init_x, final_x, ts, c_sf_z1);
    
    init_x[0] = 0;
    init_x[1] = 0;
    init_x[2] = 0;

    final_x[0] = swing_foot_height;
    final_x[1] = 0;
    final_x[2] = 0;

    coefficient_5thPoly(init_x, final_x, tf, c_sf_z2);
    
    init_x[0] = swing_foot_height;
    init_x[1] = 0;
    init_x[2] = 0;

    final_x[0] = 0;
    final_x[1] = 0;
    final_x[2] = 0;

    coefficient_5thPoly(init_x, final_x, tf, c_sf_z3);
    
    init_x[0] = swing_foot_height;
    init_x[1] = 0;
    init_x[2] = 0;

    final_x[0] = 0;
    final_x[1] = 0;
    final_x[2] = 0;

    coefficient_5thPoly(init_x, final_x, ts, c_sf_z4);
}







// ====================== Normal trot walking trajectory generation ===================== //
void CRobot::Trot_Walking(void)
{
    moving_speed = tmp_moving_speed;
//    moving_speed = 1.0; // m/s
//    y_moving_speed = 0;//tmp_moving_speed2;
    
    if(ctc_cnt2 < preview_cnt){
        Trot_Walking_Traj_First(ctc_cnt2);
        stop_flag = false;
    }
    else if(ctc_cnt2 < preview_cnt*2){
        
        if(ctc_cnt2 == preview_cnt){
            pre_com_pos = com_pos;
        }
        
        Trot_Walking_Traj(ctc_cnt2-preview_cnt); 
        
        if(ctc_cnt2 == preview_cnt*2 - 1){
 
            if (sub_ctrl_flag != true){
                stop_flag = false;
                ctc_cnt2 = preview_cnt - 1;
            }
            else{
                stop_flag = true;
                if(traj_stop_flag == true){
                    ctc_cnt2 = preview_cnt - 1;
                    traj_stop_flag = false;
                }
            }
        }   
    }

    else if(ctc_cnt2 < preview_cnt*3){
        stop_flag = true;
        moving_speed = 0;
        Trot_Walking_Traj_Final(ctc_cnt2-preview_cnt*2);    
    }
    
    local_foot_l_pos = foot_l_pos - com_pos; // foot position from global to local
    local_foot_r_pos = foot_r_pos - com_pos;
    
    // ======= Turning trajectory generation ======= //
    
    const double turn_l = 0.4134; // sqrt(0.35^2 + (0.115 + 0.105)^2)
    const double x1 = 0.35, y1 = 0.22, turn_theta1 = 57.8477*D2R;
    static double x2 = 0.35, y2 = 0.22, turn_theta2 = 0;
    static double del_x = 0, del_y = 0;
    
//    turn_theta2 = turn_theta1 + des_theta; // [rad]
//    x2 = turn_l*sin(turn_theta2);
//    y2 = turn_l*cos(turn_theta2);
//    del_x = x2 - x1;
//    del_y = y2 - y1;
    
//    printf("des_theta = %f, del_x = %f, del_y = %f\n",des_theta*R2D,del_x,del_y);
    
    // turn left
    if(des_theta > 0.01*D2R && (ctc_cnt2 < preview_cnt*3)){
        
        if(turn_mode == 0 && FC_PHASE == STANCE_RLFR){
            turn_start_flag = true;
            turn_mode = 1;

            turn_theta2 = turn_theta1 - des_theta; // [rad]
            x2 = turn_l*sin(turn_theta2);
            y2 = turn_l*cos(turn_theta2);
            del_x = x2 - x1;
            del_y = y2 - y1;
            
            printf("des_theta = %f, del_x = %f, del_y = %f\n",des_theta*R2D,del_x,del_y);
        }
        

    }
    else if(des_theta < -0.01*D2R && (ctc_cnt2 < preview_cnt*2)){
        
        if(turn_mode == 0 && FC_PHASE == STANCE_RRFL){
            turn_start_flag = true;
            turn_mode = 3;

            turn_theta2 = turn_theta1 + des_theta; // [rad]
            x2 = turn_l*sin(turn_theta2);
            y2 = turn_l*cos(turn_theta2);
            del_x = x2 - x1;
            del_y = y2 - y1;
            
            printf("des_theta = %f, del_x = %f, del_y = %f\n",des_theta*R2D,del_x,del_y);
        }
    }

    if(turn_start_flag == true){
//            turn_start_flag = true;
		if(turn_mode == 1){
			if(turn_cnt <= dsp_cnt){
				turn_xl_EP = del_x / 2.0 * (1-cos(PI2 / (2*dsp_time) * (double)turn_cnt*dt));
				turn_yl_EP = del_y / 2.0 * (1-cos(PI2 / (2*dsp_time) * (double)turn_cnt*dt));
				turn_xr_EP = 0;
				turn_yr_EP = 0;

				turn_cnt++;
			}
			else{
				turn_xl_EP = del_x;
				turn_yl_EP = del_y;
				turn_xr_EP = 0;
				turn_yr_EP = 0;
			}

			if(FC_PHASE == STANCE_RRFL){
				turn_mode = 2;
				turn_cnt = 0;
			}
		}
		else if(turn_mode == 2){
			if(turn_cnt <= dsp_cnt){
				turn_xl_EP = del_x + (0 - del_x) / 2.0 * (1-cos(PI2 / (2*dsp_time) * (double)turn_cnt*dt));
				turn_yl_EP = del_y + (0 - del_y) / 2.0 * (1-cos(PI2 / (2*dsp_time) * (double)turn_cnt*dt));
				turn_xr_EP = 0;
				turn_yr_EP = 0;

				turn_cnt++;
			}
			else{
				turn_xl_EP = 0;
				turn_yl_EP = 0;
				turn_xr_EP = 0;
				turn_yr_EP = 0;
			}

			if(FC_PHASE == STOP){
				// Initialization
				turn_mode = 0;
				turn_cnt = 0;
				turn_start_flag = false;
			}
		}
		else if(turn_mode == 3){//) && (ctc_cnt2 < preview_cnt*2)){
			if(turn_cnt <= dsp_cnt){
				turn_xr_EP = del_x / 2.0 * (1-cos(PI2 / (2*dsp_time) * (double)turn_cnt*dt));
				turn_yr_EP = del_y / 2.0 * (1-cos(PI2 / (2*dsp_time) * (double)turn_cnt*dt));
				turn_xl_EP = 0;
				turn_yl_EP = 0;

				turn_cnt++;
			}
			else{
				turn_xr_EP = del_x;
				turn_yr_EP = del_y;
				turn_xl_EP = 0;
				turn_yl_EP = 0;
			}

			if(FC_PHASE == STANCE_RLFR){
				turn_mode = 4;
				turn_cnt = 0;
			}
		}
		else if(turn_mode == 4){
			if(turn_cnt <= dsp_cnt){
				turn_xr_EP = del_x + (0 - del_x) / 2.0 * (1-cos(PI2 / (2*dsp_time) * (double)turn_cnt*dt));
				turn_yr_EP = del_y + (0 - del_y) / 2.0 * (1-cos(PI2 / (2*dsp_time) * (double)turn_cnt*dt));
				turn_xl_EP = 0;
				turn_yl_EP = 0;

				turn_cnt++;
			}
			else{
				turn_xr_EP = 0;
				turn_yr_EP = 0;
				turn_xl_EP = 0;
				turn_yl_EP = 0;
			}

			if(FC_PHASE == STOP){
				// Initialization
				turn_mode = 0;
				turn_cnt = 0;
				turn_start_flag = false;
			}
		}

	}
    else{
        turn_mode = 0;
        turn_cnt = 0;
//        turn_start_flag = false;

        turn_xl_EP = 0;
        turn_yl_EP = 0;
        turn_xr_EP = 0;
        turn_yr_EP = 0;
    }

    // ======= Turning trajectory generation  END ======= //

    
    target_EP[0] =   local_foot_l_pos[0] - turn_xr_EP;
    target_EP[1] =   local_foot_l_pos[1] + turn_yr_EP + cp_foot_l_3d[1];
    target_EP[2] =   local_foot_l_pos[2] + foot_z_offset(0);
    target_EP[3] =   local_foot_r_pos[0] - turn_xl_EP;
    target_EP[4] =   local_foot_r_pos[1] - turn_yl_EP + cp_foot_r_3d[1];
    target_EP[5] =   local_foot_r_pos[2] + foot_z_offset(1);
    target_EP[6] =   local_foot_r_pos[0] + turn_xl_EP;
    target_EP[7] =  -local_foot_r_pos[1] + turn_yl_EP + cp_foot_r_3d[1];
    target_EP[8] =   local_foot_r_pos[2] + foot_z_offset(2);
    target_EP[9] =   local_foot_l_pos[0] + turn_xr_EP;
    target_EP[10] = -local_foot_l_pos[1] - turn_yr_EP + cp_foot_l_3d[1];
    target_EP[11] =  local_foot_l_pos[2] + foot_z_offset(3);
    
    ctc_cnt2++;

    target_pos[6] = 0; //goal_pos[6];

}

void CRobot::Trot_Walking_Traj_First(unsigned int i)
{
    if(i == 0){
        
        FC_PHASE = STOP;
        
        sum_e = 0;
        
        init_foot_l_2d << init_foot_l_pos(0), init_foot_l_pos(1);
        init_foot_r_2d << init_foot_r_pos(0), init_foot_r_pos(1);
        
        Foot_step_planner_first(init_foot_l_2d,init_foot_r_2d);
        
        final_foot_l_2d << foot_l_2d(4,0),foot_l_2d(4,1); 
        final_foot_r_2d << foot_r_2d(4,0),foot_r_2d(4,1); 
        
//        cout << "[1] foot_l_2d = " << foot_l_2d << endl;
//        cout << "[1] foot_r_2d = " << foot_r_2d << endl;
        
        pre_foot_l_2d = foot_l_2d;
        pre_foot_r_2d = foot_r_2d;
        
        X_new << 0,0,0;
        
        foot_l_pos(0) = init_foot_l_pos(0);
        foot_l_pos(1) = init_foot_l_pos(1);
        foot_l_pos(2) = init_foot_l_pos(2);
        
        foot_r_pos(0) = init_foot_r_pos(0);
        foot_r_pos(1) = init_foot_r_pos(1);
        foot_r_pos(2) = init_foot_r_pos(2);
 
    }
    else{
        foot_l_pos(0) = init_foot_l_pos(0);
        foot_l_pos(1) = init_foot_l_pos(1);
        foot_l_pos(2) = init_foot_l_pos(2);
        
        foot_r_pos(0) = init_foot_r_pos(0);
        foot_r_pos(1) = init_foot_r_pos(1);
        foot_r_pos(2) = init_foot_r_pos(2);
    }
    
    COM_X_Traj_Gen(i);
        
    com_pos(0) = X_new(0);
    com_pos(1) = 0;
    com_pos(2) = com_height;
    tmp_zmp_x_ref = zmp_x_ref;
     
}

void CRobot::Trot_Walking_Traj(unsigned int i)
{    
    walk_time = i*dt;
    
    if(i == 0){
        FC_PHASE = STANCE_RRFL;
       
        Foot_step_planner(final_foot_l_2d,final_foot_r_2d);
                
        final_foot_l_2d << foot_l_2d(4,0),foot_l_2d(4,1); 
        final_foot_r_2d << foot_r_2d(4,0),foot_r_2d(4,1); 
       
        SF_Z_Traj_Gen();
        SF_X_Traj_Gen();
        
        foot_l_pos(0) = pre_foot_l_2d(0,0);
        foot_l_pos(1) = pre_foot_l_2d(0,1);
        foot_l_pos(2) = init_foot_l_pos(2);
        
        foot_r_pos(0) = pre_foot_r_2d(0,0);
        foot_r_pos(1) = pre_foot_r_2d(0,1);
        foot_r_pos(2) = init_foot_r_pos(2);
    }
    else if(i < dsp_cnt){
        FC_PHASE = STANCE_RRFL;
        
        t2 = walk_time;
        foot_l_pos(0) = x1[5]*pow(t2,5) + x1[4]*pow(t2,4) + x1[3]*pow(t2,3) + x1[2]*pow(t2,2) + x1[1]*pow(t2,1) + x1[0];
        foot_l_pos(1) = pre_foot_l_2d(0,1);
        
        foot_r_pos(0) = pre_foot_r_2d(0,0);
        foot_r_pos(1) = pre_foot_r_2d(0,1);
        foot_r_pos(2) = init_foot_r_pos(2);
        
        if(i < dsp_t1/dt){
            t1 = t2;
            foot_l_pos(2) = init_foot_l_pos(2) + z1[5]*pow(t1,5) + z1[4]*pow(t1,4) + z1[3]*pow(t1,3) + z1[2]*pow(t1,2) + z1[1]*pow(t1,1) + z1[0];
        }
        else{
            t1 = t2 - dsp_t1;
            foot_l_pos(2) = init_foot_l_pos(2) + z2[5]*pow(t1,5) + z2[4]*pow(t1,4) + z2[3]*pow(t1,3) + z2[2]*pow(t1,2) + z2[1]*pow(t1,1) + z2[0];
        }        
    }
    else if(i < step_cnt){
        FC_PHASE = STOP;
        foot_l_pos(0) = pre_foot_l_2d(1,0);
        foot_l_pos(1) = pre_foot_l_2d(1,1);
        foot_l_pos(2) = init_foot_l_pos(2);
        
        foot_r_pos(0) = pre_foot_r_2d(1,0);
        foot_r_pos(1) = pre_foot_r_2d(1,1);
        foot_r_pos(2) = init_foot_r_pos(2);   
        
    }
    else if(i < step_cnt + dsp_cnt){
        FC_PHASE = STANCE_RLFR;
        
        t2 = walk_time - step_time;

        foot_l_pos(0) = pre_foot_l_2d(1,0);
        foot_l_pos(1) = pre_foot_l_2d(1,1);
        foot_l_pos(2) = init_foot_l_pos(2);
        
        foot_r_pos(0) = x2[5]*pow(t2,5) + x2[4]*pow(t2,4) + x2[3]*pow(t2,3) + x2[2]*pow(t2,2) + x2[1]*pow(t2,1) + x2[0];
        foot_r_pos(1) = pre_foot_r_2d(1,1);//init_foot_r_pos(1);
        
        if(i < step_cnt + dsp_t1/dt){
            t1 = t2;
            foot_r_pos(2) = init_foot_r_pos(2) + z1[5]*pow(t1,5) + z1[4]*pow(t1,4) + z1[3]*pow(t1,3) + z1[2]*pow(t1,2) + z1[1]*pow(t1,1) + z1[0];
        }
        else{
            t1 = t2 - dsp_t1;
            foot_r_pos(2) = init_foot_r_pos(2) + z2[5]*pow(t1,5) + z2[4]*pow(t1,4) + z2[3]*pow(t1,3) + z2[2]*pow(t1,2) + z2[1]*pow(t1,1) + z2[0];
        } 
        
    }
    
    else if(i < step_cnt*2){
        FC_PHASE = STOP;
        
        foot_l_pos(0) = pre_foot_l_2d(2,0);
        foot_l_pos(1) = pre_foot_l_2d(2,1);
        foot_l_pos(2) = init_foot_l_pos(2);
        
        foot_r_pos(0) = pre_foot_r_2d(2,0);
        foot_r_pos(1) = pre_foot_r_2d(2,1);
        foot_r_pos(2) = init_foot_r_pos(2);    
        
    }
    else if(i < step_cnt*2 + dsp_cnt){
        FC_PHASE = STANCE_RRFL;
        
        t2 = walk_time - step_time*2;
        
        foot_l_pos(0) = x3[5]*pow(t2,5) + x3[4]*pow(t2,4) + x3[3]*pow(t2,3) + x3[2]*pow(t2,2) + x3[1]*pow(t2,1) + x3[0];
        foot_l_pos(1) = pre_foot_l_2d(2,1);
        foot_l_pos(2) = init_foot_l_pos(2);
        
        foot_r_pos(0) = pre_foot_r_2d(2,0);
        foot_r_pos(1) = pre_foot_r_2d(2,1);
        foot_r_pos(2) = init_foot_r_pos(2);
        
        if(i < step_cnt*2 + dsp_t1/dt){
            t1 = t2;
            foot_l_pos(2) = init_foot_l_pos(2) + z1[5]*pow(t1,5) + z1[4]*pow(t1,4) + z1[3]*pow(t1,3) + z1[2]*pow(t1,2) + z1[1]*pow(t1,1) + z1[0];
        }
        else{
            t1 = t2 - dsp_t1;
            foot_l_pos(2) = init_foot_l_pos(2) + z2[5]*pow(t1,5) + z2[4]*pow(t1,4) + z2[3]*pow(t1,3) + z2[2]*pow(t1,2) + z2[1]*pow(t1,1) + z2[0];
        }
        
    }
    else if(i < step_cnt*3){
        FC_PHASE = STOP;
        foot_l_pos(0) = pre_foot_l_2d(3,0);
        foot_l_pos(1) = pre_foot_l_2d(3,1);
        foot_l_pos(2) = init_foot_l_pos(2);
        
        foot_r_pos(0) = pre_foot_r_2d(3,0);
        foot_r_pos(1) = pre_foot_r_2d(3,1);
        foot_r_pos(2) = init_foot_r_pos(2); 
          
    }
    else if(i < step_cnt*3 + dsp_cnt){
        FC_PHASE = STANCE_RLFR;
        
        t2 = walk_time - step_time*3;
        
        foot_l_pos(0) = pre_foot_l_2d(3,0);
        foot_l_pos(1) = init_foot_l_pos(1);
        foot_l_pos(2) = init_foot_l_pos(2);
        
        foot_r_pos(0) = x4[5]*pow(t2,5) + x4[4]*pow(t2,4) + x4[3]*pow(t2,3) + x4[2]*pow(t2,2) + x4[1]*pow(t2,1) + x4[0];
        foot_r_pos(1) = init_foot_r_pos(1);
        
        if(i < step_cnt*3 + dsp_t1/dt){
            t1 = t2;
            foot_r_pos(2) = init_foot_r_pos(2) + z1[5]*pow(t1,5) + z1[4]*pow(t1,4) + z1[3]*pow(t1,3) + z1[2]*pow(t1,2) + z1[1]*pow(t1,1) + z1[0];
        }
        else{
            t1 = t2 - dsp_t1;
            foot_r_pos(2) = init_foot_r_pos(2) + z2[5]*pow(t1,5) + z2[4]*pow(t1,4) + z2[3]*pow(t1,3) + z2[2]*pow(t1,2) + z2[1]*pow(t1,1) + z2[0];
        } 

    }
    
    else{
        FC_PHASE = STOP;
        foot_l_pos(0) = pre_foot_l_2d(4,0);
        foot_l_pos(1) = pre_foot_l_2d(4,1);//init_foot_l_pos(1);
        foot_l_pos(2) = init_foot_l_pos(2);
        
        foot_r_pos(0) = pre_foot_r_2d(4,0);
        foot_r_pos(1) = pre_foot_r_2d(4,1);//init_foot_r_pos(1);
        foot_r_pos(2) = init_foot_r_pos(2);
        
        if(i == step_cnt*4 - 1){
            pre_foot_l_2d = foot_l_2d;
            pre_foot_r_2d = foot_r_2d;
        }     
    }
    
    CP_Con_trot(i);
        
    COM_X_Traj_Gen(i);
    
    com_pos(0) = X_new(0);
    com_pos(1) = 0;
    com_pos(2) = com_height;
    tmp_zmp_x_ref = zmp_x_ref; 
}

void CRobot::check_CP(void){
    static double tmp_cp_y_lower_limit = 0.07;
    static double tmp_cp_y_upper_limit = 0.15;

//    cout << "[0] CP_y = " << CP_y << endl;
    
    if(CP_y > tmp_cp_y_lower_limit){
        cout << "CP_y = " << CP_y << endl;

        if(CP_y < tmp_cp_y_upper_limit){
            tmp_cp_foot_pos_y = CP_y;
        }
        else{
            tmp_cp_foot_pos_y = tmp_cp_y_upper_limit;
        }

        get_cp_done_flag = true;
    }
    else if(CP_y < -tmp_cp_y_lower_limit){
        cout << "CP_y = " << CP_y << endl;

        if(CP_y > -tmp_cp_y_upper_limit){
            tmp_cp_foot_pos_y = CP_y;
        }
        else{
            tmp_cp_foot_pos_y = -tmp_cp_y_upper_limit;
        }

        get_cp_done_flag = true;
    }
    else if(get_cp_done_flag == false){
        tmp_cp_foot_pos_y = 0;
    }
}

void CRobot::CP_Con_trot(unsigned int i){
    
    static int tmp_cp_cnt = 0;
    static int cp_on_cnt = 0;
    static double tmp_cp_time = 0;
    // ==================== CP CONTROLLER =================== //
    
    // CP PHASE
    // 0 : Normal walking
    // 1 : CASE1
    // 2 : CASE2
    
    switch (CP_PHASE) {
    case 0:
        // Normal walking
        check_CP();
        if (i%step_cnt <= dsp_cnt/2 - 1) {
            // 1. check CP
            if(get_cp_done_flag == true){
                CP_PHASE = 1;//1;
                target_cp_foot_pos_y = tmp_cp_foot_pos_y/2.0;
                tmp_cp_cnt = 0;
                cp_on_cnt = i%step_cnt;
                get_cp_done_flag = false;
            }
            else{
                target_cp_foot_pos_y = 0;
            }
        }
        else if (i%step_cnt == step_cnt - 1) { // dsp_cnt or step_cnt
            if(get_cp_done_flag == true){
                CP_PHASE = 2;
                target_cp_foot_pos_y = tmp_cp_foot_pos_y/2.0;
                tmp_cp_cnt = 0;
                get_cp_done_flag = false;
            }
            else{
                target_cp_foot_pos_y = 0;
            }
        }
        
        break;
        
    case 1:
        // swing foot can be located at CP during current phase.

        if(CP_move_step == 0){
            if(i%step_cnt < dsp_cnt){
                tmp_t = (double)(i%step_cnt - cp_on_cnt)*dt; // now time
                tmp_cp_time = (double)(dsp_cnt - cp_on_cnt)*dt; // stepping time
                
                cout << "tmp_t = " << tmp_t << ", tmp_cp_time = " << tmp_cp_time << ", cp_on_cnt = "<< cp_on_cnt << endl;
                
//                cout << "[case.1][CP_move_step.1]tmp_t = " << tmp_t << ", target_cp_foot_pos_y = " << target_cp_foot_pos_y << endl;
                
                if(FC_PHASE == STANCE_RRFL){
                    cp_foot_offset_y = Kp_cp*(CP_y-target_cp_foot_pos_y*2)/2.0;
                    
                    cp_foot_l_3d[1] = init_cp_foot_l_3d[1] + (target_cp_foot_pos_y) / 2.0 * (1 - cos(PI2 / (2*tmp_cp_time) * tmp_t)) + cp_foot_offset_y;
                    cp_foot_r_3d[1] = init_cp_foot_r_3d[1] - (target_cp_foot_pos_y) / 2.0 * (1 - cos(PI2 / (2*tmp_cp_time) * tmp_t)) - cp_foot_offset_y; 
                }
                else if(FC_PHASE == STANCE_RLFR){
                    cp_foot_offset_y = Kp_cp*(CP_y-target_cp_foot_pos_y*2)/2.0;
                    
                    cp_foot_l_3d[1] = init_cp_foot_l_3d[1] - (target_cp_foot_pos_y) / 2.0 * (1 - cos(PI2 / (2*tmp_cp_time) * tmp_t)) - cp_foot_offset_y;
                    cp_foot_r_3d[1] = init_cp_foot_r_3d[1] + (target_cp_foot_pos_y) / 2.0 * (1 - cos(PI2 / (2*tmp_cp_time) * tmp_t)) + cp_foot_offset_y; 
                }   
                printf("[1] cp_foot_offset_y = %f, cp_foot_l_3d[1] = %f\n",cp_foot_offset_y,cp_foot_l_3d[1]);
            }
            else{
                if(FC_PHASE == STANCE_RRFL){
                cp_foot_l_3d[1] = init_cp_foot_l_3d[1] + (target_cp_foot_pos_y) + cp_foot_offset_y;
                cp_foot_r_3d[1] = init_cp_foot_r_3d[1] - (target_cp_foot_pos_y) - cp_foot_offset_y;
                }
                else if(FC_PHASE == STANCE_RLFR){
                cp_foot_l_3d[1] = init_cp_foot_l_3d[1] - (target_cp_foot_pos_y) - cp_foot_offset_y;
                cp_foot_r_3d[1] = init_cp_foot_r_3d[1] + (target_cp_foot_pos_y) + cp_foot_offset_y;
                }
                
                if(i%step_cnt == step_cnt-1){
                    CP_move_step = 1;
                }
            }
        }
        else if(CP_move_step == 1){
            
            if(i%step_cnt < dsp_cnt){
                tmp_t = (double)(i%step_cnt)*dt;
//                cout << "[case.1][CP_move_step.2] tmp_t = " << tmp_t << ", target_cp_foot_pos_y = " << target_cp_foot_pos_y << endl;
                
                if(FC_PHASE == STANCE_RRFL){
                    cp_foot_l_3d[1] = init_cp_foot_l_3d[1] - (target_cp_foot_pos_y + cp_foot_offset_y) + (target_cp_foot_pos_y + cp_foot_offset_y) / 2.0 * (1 - cos(PI2 / (dsp_time*2) * tmp_t));
                    cp_foot_r_3d[1] = init_cp_foot_r_3d[1] + (target_cp_foot_pos_y + cp_foot_offset_y) - (target_cp_foot_pos_y + cp_foot_offset_y) / 2.0 * (1 - cos(PI2 / (dsp_time*2) * tmp_t)); 
                }
                else if(FC_PHASE == STANCE_RLFR){
                    cp_foot_l_3d[1] = init_cp_foot_l_3d[1] + (target_cp_foot_pos_y + cp_foot_offset_y) - (target_cp_foot_pos_y + cp_foot_offset_y) / 2.0 * (1 - cos(PI2 / (dsp_time*2) * tmp_t));
                    cp_foot_r_3d[1] = init_cp_foot_r_3d[1] - (target_cp_foot_pos_y + cp_foot_offset_y) + (target_cp_foot_pos_y + cp_foot_offset_y) / 2.0 * (1 - cos(PI2 / (dsp_time*2) * tmp_t)); 
                }     
            }
            else{
                cp_foot_l_3d[1] = init_cp_foot_l_3d[1];
                cp_foot_r_3d[1] = init_cp_foot_r_3d[1];
                
                if(i%step_cnt == step_cnt-1){
                    CP_move_step = 2;
                }
            }
        }
        else if(CP_move_step == 2){
            if(i%step_cnt == step_cnt-1){
                if(i%step_cnt == step_cnt-1){
                    tmp_cp_cnt++;

                    cout << endl << " =========== tmp_cp_cnt = " << tmp_cp_cnt << " ===========" << endl;

                    if(tmp_cp_cnt == 6){ //3
                        CP_PHASE = 0;
                        CP_move_step = 0;
                        CP_move_done_flag = true;
                    }
                }
            }
        }
        break;
        
    case 2:
        // swing foot can be located at CP during next phase.
        
        if(CP_move_step == 0){
            if(i%step_cnt < dsp_cnt){
                tmp_t = (double)(i%step_cnt)*dt;
//                cout << "[case.2][CP_move_step.1]tmp_t = " << tmp_t << ", target_cp_foot_pos_y = " << target_cp_foot_pos_y << endl;
                
                if(FC_PHASE == STANCE_RRFL){
                    cp_foot_offset_y = Kp_cp*(CP_y-target_cp_foot_pos_y*2)/2.0;
                    
                    cp_foot_l_3d[1] = init_cp_foot_l_3d[1] + (target_cp_foot_pos_y) / 2.0 * (1 - cos(PI2 / (dsp_time*2) * tmp_t)) + cp_foot_offset_y;
                    cp_foot_r_3d[1] = init_cp_foot_r_3d[1] - (target_cp_foot_pos_y) / 2.0 * (1 - cos(PI2 / (dsp_time*2) * tmp_t)) - cp_foot_offset_y; 
                }
                else if(FC_PHASE == STANCE_RLFR){
                    cp_foot_offset_y = Kp_cp*(CP_y-target_cp_foot_pos_y*2)/2.0;
                    
                    cp_foot_l_3d[1] = init_cp_foot_l_3d[1] - (target_cp_foot_pos_y) / 2.0 * (1 - cos(PI2 / (dsp_time*2) * tmp_t)) - cp_foot_offset_y;
                    cp_foot_r_3d[1] = init_cp_foot_r_3d[1] + (target_cp_foot_pos_y) / 2.0 * (1 - cos(PI2 / (dsp_time*2) * tmp_t)) + cp_foot_offset_y; 
                }    
                
                printf("[2] cp_foot_offset_y = %f\n",cp_foot_offset_y);
            }
            else{
                if(FC_PHASE == STANCE_RRFL){
                cp_foot_l_3d[1] = init_cp_foot_l_3d[1] + (target_cp_foot_pos_y + cp_foot_offset_y);
                cp_foot_r_3d[1] = init_cp_foot_r_3d[1] - (target_cp_foot_pos_y + cp_foot_offset_y);
                }
                else if(FC_PHASE == STANCE_RLFR){
                cp_foot_l_3d[1] = init_cp_foot_l_3d[1] - (target_cp_foot_pos_y + cp_foot_offset_y);
                cp_foot_r_3d[1] = init_cp_foot_r_3d[1] + (target_cp_foot_pos_y + cp_foot_offset_y);
                }
                
                if(i%step_cnt == step_cnt-1){
                    CP_move_step = 1;
                }
            }
        }
        else if(CP_move_step == 1){
            if(i%step_cnt < dsp_cnt){
                tmp_t = (double)(i%step_cnt)*dt;
//                cout << "[case.2][CP_move_step.2] tmp_t = " << tmp_t << ", target_cp_foot_pos_y = " << target_cp_foot_pos_y << endl;
                
                if(FC_PHASE == STANCE_RRFL){
                    cp_foot_l_3d[1] = init_cp_foot_l_3d[1] - (target_cp_foot_pos_y + cp_foot_offset_y) + (target_cp_foot_pos_y + cp_foot_offset_y) / 2.0 * (1 - cos(PI2 / (dsp_time*2) * tmp_t));
                    cp_foot_r_3d[1] = init_cp_foot_r_3d[1] + (target_cp_foot_pos_y + cp_foot_offset_y) - (target_cp_foot_pos_y + cp_foot_offset_y) / 2.0 * (1 - cos(PI2 / (dsp_time*2) * tmp_t)); 
                }
                else if(FC_PHASE == STANCE_RLFR){
                    cp_foot_l_3d[1] = init_cp_foot_l_3d[1] + (target_cp_foot_pos_y + cp_foot_offset_y) - (target_cp_foot_pos_y + cp_foot_offset_y) / 2.0 * (1 - cos(PI2 / (dsp_time*2) * tmp_t));
                    cp_foot_r_3d[1] = init_cp_foot_r_3d[1] - (target_cp_foot_pos_y + cp_foot_offset_y) + (target_cp_foot_pos_y + cp_foot_offset_y) / 2.0 * (1 - cos(PI2 / (dsp_time*2) * tmp_t)); 
                }   
            }
            else{
                cp_foot_l_3d[1] = init_cp_foot_l_3d[1];
                cp_foot_r_3d[1] = init_cp_foot_r_3d[1];
                
                if(i%step_cnt == step_cnt-1){
                    CP_move_step = 2;
                }
            }
        }
        else if(CP_move_step == 2){
            if(i%step_cnt == step_cnt-1){
                if(i%step_cnt == step_cnt-1){
                    tmp_cp_cnt++;

                    cout << endl << " =========== tmp_cp_cnt = " << tmp_cp_cnt << " ===========" << endl;
                    
                    if(tmp_cp_cnt == 6){ //3
                        CP_PHASE = 0;
                        CP_move_step = 0;
                        CP_move_done_flag = true;
                    }
                }
            }
        }
        break;
    
    }
    
    // Get CP
    if(CP_con_onoff_flag == 0){
        cp_foot_l_3d << 0,0,0;
        cp_foot_r_3d << 0,0,0;
    }
}

void CRobot::Body_Ori_Con(void){
  
}


void CRobot::Body_Ori_Con2(void){
   
    static double sum_roll_err = 0.;
    static double del_L_fl = 0, del_L_fr = 0;//, del_L_rl = 0, del_L_rr = 0;
    const double limit_foot_z = 0.03;
    const double IMURoll_alpha = 0.01;
    static double lpf_IMURoll = 0;
    
    // ==================== Slop Compensation Control ==================== //

    // roll
    lpf_IMURoll = (1 - IMURoll_alpha) * lpf_IMURoll + IMURoll_alpha*IMURoll;
    
    sum_roll_err = sum_roll_err + (0 - lpf_IMURoll) * dt;

    del_L_fl = Kp_y * (0 - lpf_IMURoll) + Ki_y*sum_roll_err;
    del_L_fr = -Kp_y * (0 - lpf_IMURoll) - Ki_y*sum_roll_err;
//    del_L_rl = Kp_y * (0 - lpf_IMURoll) + Ki_y*sum_roll_err;
//    del_L_rr = -Kp_y * (0 - lpf_IMURoll) - Ki_y*sum_roll_err;

    if(del_L_fl > limit_foot_z){
    	del_L_fl = limit_foot_z;
    }
    else if(del_L_fl < -limit_foot_z){
    	del_L_fl = -limit_foot_z;
    }
    
    if(del_L_fr > limit_foot_z){
        del_L_fr = limit_foot_z;
    }
    else if(del_L_fr < -limit_foot_z){
        del_L_fr = -limit_foot_z;
    }

    target_EP_offset[2]  = -del_L_fl;//Kp_roll*IMURoll + Kd_roll*lpf_IMURoll_dot;
    target_EP_offset[8]  = -del_L_fl;//Kp_roll*IMURoll + Kd_roll*lpf_IMURoll_dot;
    target_EP_offset[5]  = -del_L_fr;//Kp_roll*IMURoll + Kd_roll*lpf_IMURoll_dot;
    target_EP_offset[11] = -del_L_fr;//Kp_roll*IMURoll + Kd_roll*lpf_IMURoll_dot;
    
}


void CRobot::Trot_Walking_Traj_Final(unsigned int i)
{
    walk_time = i*dt;
    
    if(i == 0){
        FC_PHASE = STANCE_RRFL;
               
        Foot_step_planner(final_foot_l_2d,final_foot_r_2d);
        
//        cout << "[3]foot_l_2d = " << foot_l_2d << endl;
//        cout << "[3]foot_r_2d = " << foot_r_2d << endl;

        SF_X_Traj_Gen_Final();
        
        foot_l_pos(0) = pre_foot_l_2d(0,0);
        foot_l_pos(1) = init_foot_l_pos(1);
        foot_l_pos(2) = init_foot_l_pos(2);
        
        foot_r_pos(0) = pre_foot_r_2d(0,0);
        foot_r_pos(1) = init_foot_r_pos(1);
        foot_r_pos(2) = init_foot_r_pos(2);
 
    }
    else if(i < dsp_cnt){
        FC_PHASE = STANCE_RRFL;
        t2 = walk_time;
        foot_l_pos(0) = x5[5]*pow(t2,5) + x5[4]*pow(t2,4) + x5[3]*pow(t2,3) + x5[2]*pow(t2,2) + x5[1]*pow(t2,1) + x5[0];
        foot_l_pos(1) = init_foot_l_pos(1);
        
        foot_r_pos(0) = pre_foot_r_2d(0,0);
        foot_r_pos(1) = init_foot_r_pos(1);
        foot_r_pos(2) = init_foot_r_pos(2);
        
        if(i < dsp_t1/dt){
            t1 = t2;
            foot_l_pos(2) = init_foot_l_pos(2) + z1[5]*pow(t1,5) + z1[4]*pow(t1,4) + z1[3]*pow(t1,3) + z1[2]*pow(t1,2) + z1[1]*pow(t1,1) + z1[0];
        }
        else{
            t1 = t2 - dsp_t1;
            foot_l_pos(2) = init_foot_l_pos(2) + z2[5]*pow(t1,5) + z2[4]*pow(t1,4) + z2[3]*pow(t1,3) + z2[2]*pow(t1,2) + z2[1]*pow(t1,1) + z2[0];
        }
    }
    
    else{
        FC_PHASE = STOP;
        foot_l_pos(0) = pre_foot_l_2d(4,0);
        foot_l_pos(1) = init_foot_l_pos(1);
        foot_l_pos(2) = init_foot_l_pos(2);
        
        foot_r_pos(0) = pre_foot_r_2d(4,0);
        foot_r_pos(1) = init_foot_r_pos(1);
        foot_r_pos(2) = init_foot_r_pos(2);
        
        if(i == step_cnt*4){
            pre_foot_l_2d = foot_l_2d;
            pre_foot_r_2d = foot_r_2d;
        }     
    }
    
    COM_X_Traj_Gen(i);
        
    com_pos(0) = X_new(0);//zmp_x_ref;//com_x_array(i,0);
    com_pos(1) = 0;
    com_pos(2) = com_height;
    tmp_zmp_x_ref = zmp_x_ref;//zmp_x_array(i);

}

void CRobot::SF_X_Traj_Gen_Final(void){

    init_x[0] = pre_foot_l_2d(0,0);
    init_x[1] = 0;
    init_x[2] = 0;

    final_x[0] = pre_foot_l_2d(1,0);
    final_x[1] = 0;
    final_x[2] = 0;

    coefficient_5thPoly(init_x, final_x, dsp_time, x5);
    
}

void CRobot::SF_X_Traj_Gen(void){

    init_x[0] = pre_foot_l_2d(0,0);
    init_x[1] = 0;
    init_x[2] = 0;

    final_x[0] = pre_foot_l_2d(1,0);
    final_x[1] = 0;
    final_x[2] = 0;

    coefficient_5thPoly(init_x, final_x, dsp_time, x1);
    
    init_x[0] = pre_foot_r_2d(1,0);
    init_x[1] = 0;
    init_x[2] = 0;

    final_x[0] = pre_foot_r_2d(2,0);
    final_x[1] = 0;
    final_x[2] = 0;

    coefficient_5thPoly(init_x, final_x, dsp_time, x2);
    
    init_x[0] = pre_foot_l_2d(2,0);
    init_x[1] = 0;
    init_x[2] = 0;

    final_x[0] = pre_foot_l_2d(3,0);
    final_x[1] = 0;
    final_x[2] = 0;

    coefficient_5thPoly(init_x, final_x, dsp_time, x3);
    
    init_x[0] = pre_foot_r_2d(3,0);
    init_x[1] = 0;
    init_x[2] = 0;

    final_x[0] = pre_foot_r_2d(4,0);
    final_x[1] = 0;
    final_x[2] = 0;

    coefficient_5thPoly(init_x, final_x, dsp_time, x4);
    
}

void CRobot::SF_Z_Traj_Gen(void){
    dsp_t1 = dsp_time/2.0f;
    dsp_t2 = dsp_time - dsp_t1;
    
    init_x[0] = 0;
    init_x[1] = 0;
    init_x[2] = 0;

    final_x[0] = foot_height;
    final_x[1] = 0;
    final_x[2] = 0;

    coefficient_5thPoly(init_x, final_x, dsp_t1, z1);
    
    init_x[0] = foot_height;
    init_x[1] = 0;
    init_x[2] = 0;

    final_x[0] = 0;
    final_x[1] = 0;
    final_x[2] = 0;

    coefficient_5thPoly(init_x, final_x, dsp_t2, z2);
}


void CRobot::Foot_step_planner(VectorNd init_foot_l_2d, VectorNd init_foot_r_2d){
    double x_dist = moving_speed*step_time;
    double y_dist = y_moving_speed*step_time;
    
    if(stop_flag == false){
       foot_l_2d << init_foot_l_2d[0],          init_foot_l_2d[1],
                    init_foot_r_2d[0]+x_dist,   init_foot_l_2d[1],
                    init_foot_r_2d[0]+x_dist,   init_foot_l_2d[1],
                    init_foot_r_2d[0]+x_dist*3, init_foot_l_2d[1],
                    init_foot_r_2d[0]+x_dist*3, init_foot_l_2d[1];

       foot_r_2d << init_foot_r_2d[0],          init_foot_r_2d[1],
                    init_foot_r_2d[0],          init_foot_r_2d[1],
                    init_foot_r_2d[0]+x_dist*2, init_foot_r_2d[1],
                    init_foot_r_2d[0]+x_dist*2, init_foot_r_2d[1],
                    init_foot_r_2d[0]+x_dist*4, init_foot_r_2d[1];

    }
    else{
       foot_l_2d << init_foot_l_2d[0],          init_foot_l_2d[1],
                    init_foot_l_2d[0]+x_dist,   init_foot_l_2d[1],
                    init_foot_l_2d[0]+x_dist,   init_foot_l_2d[1],
                    init_foot_l_2d[0]+x_dist,   init_foot_l_2d[1],
                    init_foot_l_2d[0]+x_dist,   init_foot_l_2d[1];

       foot_r_2d << init_foot_r_2d[0],          init_foot_r_2d[1],
                    init_foot_r_2d[0],          init_foot_r_2d[1],
                    init_foot_r_2d[0],          init_foot_r_2d[1],
                    init_foot_r_2d[0],          init_foot_r_2d[1],
                    init_foot_r_2d[0],          init_foot_r_2d[1];

    }
}

void CRobot::Foot_step_planner_first(VectorNd init_foot_l_2d, VectorNd init_foot_r_2d){
    double dist = moving_speed*step_time;
    
    foot_l_2d << init_foot_l_2d[0],        init_foot_l_2d[1],
                 init_foot_l_2d[0]+dist,   init_foot_l_2d[1],
                 init_foot_l_2d[0]+dist,   init_foot_l_2d[1],
                 init_foot_l_2d[0]+dist*3, init_foot_l_2d[1],
                 init_foot_l_2d[0]+dist*3, init_foot_l_2d[1];

    foot_r_2d << init_foot_r_2d[0],        init_foot_r_2d[1],
                 init_foot_r_2d[0],        init_foot_r_2d[1],
                 init_foot_r_2d[0]+dist*2, init_foot_r_2d[1],
                 init_foot_r_2d[0]+dist*2, init_foot_r_2d[1],
                 init_foot_r_2d[0]+dist*4, init_foot_r_2d[1];
}


void CRobot::COM_X_Traj_Gen(unsigned int i){
    
    static double tmp_zmp_ref = 0;
   
    if(i <= dsp_cnt){
        tmp_zmp_ref = foot_r_2d(0,0);
    }
    else if(i <= step_cnt){
        tmp_zmp_ref = foot_l_2d(1,0);
    }
    else if(i <= step_cnt+dsp_cnt){
        tmp_zmp_ref = foot_l_2d(1,0);
    }
    else if(i <= step_cnt*2){
        tmp_zmp_ref = foot_r_2d(2,0);
    }
    else if(i <= step_cnt*2+dsp_cnt){
        tmp_zmp_ref = foot_r_2d(2,0);
    }
    else if(i <= step_cnt*3){
        tmp_zmp_ref = foot_l_2d(3,0);
    }
    else if(i <= step_cnt*3+dsp_cnt){
        tmp_zmp_ref = foot_l_2d(3,0);
    }
    else{
        tmp_zmp_ref = foot_r_2d(4,0);
    }

    for(unsigned int j=0;j<=preview_cnt-2;++j){
        zmp_ref_array(j) = zmp_ref_array(j+1);
    }
    zmp_ref_array(preview_cnt-1) = tmp_zmp_ref;  

    Preview_con();
        
    zmp_x_ref = zmp_ref_array(0);

}

void CRobot::Preview_con(void)
{
    static double err = 0;
    static double sum_p = 0;
    static double u = 0;
    static double zmp_ref_old = 0;
    
    zmp_ref_old = CC.transpose()*X_new;
    
    err = zmp_ref_old - zmp_ref_array(0);
    sum_e = sum_e + err;
    sum_p = 0;
    
    for(unsigned int k=0;k<preview_cnt-1;++k){
        sum_p = sum_p + Gp(k)*zmp_ref_array(k);
    }
    
    u = -Gi*sum_e - Gx.transpose()*X_new - sum_p;
    
    X_new = AA*X_new + BB*u;
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

void CRobot::Cal_Fc(void){
    
//    static double tmp_Fc1 = 70;//100; //70; // 50
//    static double tmp_Fc2 = tmp_Fc1 * 2.0;

    static double fc_cnt = 0;

    static double tmp_fc_time = 0.01;
    static int tmp_fc_cnt = 10;
    

    
//    static double offset_fc = 30;

    if (FC_PHASE == STOP) {
        Fc_RL_z = -tmp_Fc1;
        Fc_RR_z = -tmp_Fc1;
        Fc_FL_z = -tmp_Fc1;
        Fc_FR_z = -tmp_Fc1;
        
        fc_cnt = 0;
    }
    else if (FC_PHASE == INIT_Fc) { // walk ready
        Fc_RL_z = -tmp_Fc1 / 2.0 * (1 - cos(PI2 / (home_pos_time * 2)*(double) (ctc_cnt) * dt));
        Fc_RR_z = -tmp_Fc1 / 2.0 * (1 - cos(PI2 / (home_pos_time * 2)*(double) (ctc_cnt) * dt));
        Fc_FL_z = -tmp_Fc1 / 2.0 * (1 - cos(PI2 / (home_pos_time * 2)*(double) (ctc_cnt) * dt));
        Fc_FR_z = -tmp_Fc1 / 2.0 * (1 - cos(PI2 / (home_pos_time * 2)*(double) (ctc_cnt) * dt));

    }
    else if (FC_PHASE == STANCE_RLFR) {

        if(CommandFlag == FLYING_TROT_RUNNING){
            
            if(flying_trot_final_flag == true){
//				printf("===================================== final_flag = true ====================================\n");
//                printf("fc_cnt = %f, Fc_RR_z = %f, Fc_RL_z = %f\n",fc_cnt,Fc_RR_z,Fc_RL_z);
                if (fc_cnt <= tmp_fc_cnt) {
                    Fc_RR_z = 0;
                    Fc_FL_z = 0;
                    Fc_RL_z = -tmp_Fc2*0.5 * (1 - cos(PI2 / (tmp_fc_time * 2)*(double) (fc_cnt) * dt));
                    Fc_FR_z = -tmp_Fc2*0.5 * (1 - cos(PI2 / (tmp_fc_time * 2)*(double) (fc_cnt) * dt));

                }

                else if (fc_cnt >= ts_cnt - tmp_fc_cnt) {
                    Fc_RR_z = -tmp_Fc1*0.5 * (1 - cos(PI2 / (tmp_fc_time * 2)*(double) (fc_cnt - (ts_cnt - tmp_fc_cnt)) * dt));
                    Fc_FL_z = -tmp_Fc1*0.5 * (1 - cos(PI2 / (tmp_fc_time * 2)*(double) (fc_cnt - (ts_cnt - tmp_fc_cnt)) * dt));
                    Fc_RL_z = -tmp_Fc2 + (-tmp_Fc1 + tmp_Fc2)*0.5 * (1 - cos(PI2 / (tmp_fc_time * 2)*(double) (fc_cnt - (ts_cnt - tmp_fc_cnt)) * dt));
                    Fc_FR_z = -tmp_Fc2 + (-tmp_Fc1 + tmp_Fc2)*0.5 * (1 - cos(PI2 / (tmp_fc_time * 2)*(double) (fc_cnt - (ts_cnt - tmp_fc_cnt)) * dt));
                }
            }
            else{

                if (fc_cnt <= tmp_fc_cnt) {
                        Fc_RR_z = 0;
                        Fc_FL_z = 0;
                        Fc_RL_z = -tmp_Fc2*0.5 * (1 - cos(PI2 / (tmp_fc_time * 2)*(double) (fc_cnt) * dt));
                        Fc_FR_z = -tmp_Fc2*0.5 * (1 - cos(PI2 / (tmp_fc_time * 2)*(double) (fc_cnt) * dt));

                }
                else if (fc_cnt >= ts_cnt - tmp_fc_cnt) {
                        Fc_RR_z = 0;
                        Fc_FL_z = 0;
                        Fc_RL_z = -tmp_Fc2 + tmp_Fc2*0.5 * (1 - cos(PI2 / (tmp_fc_time * 2)*(double) (fc_cnt - (ts_cnt - tmp_fc_cnt)) * dt));
                        Fc_FR_z = -tmp_Fc2 + tmp_Fc2*0.5 * (1 - cos(PI2 / (tmp_fc_time * 2)*(double) (fc_cnt - (ts_cnt - tmp_fc_cnt)) * dt));
                }

            }
//            if (fc_cnt <= tmp_fc_cnt) {
//                    Fc_RR_z = 0;
//                    Fc_FL_z = 0;
//                    Fc_RL_z = -tmp_Fc2*0.5 * (1 - cos(PI2 / (tmp_fc_time * 2)*(double) (fc_cnt) * dt));
//                    Fc_FR_z = -tmp_Fc2*0.5 * (1 - cos(PI2 / (tmp_fc_time * 2)*(double) (fc_cnt) * dt));
//
//            }
//            else if (fc_cnt >= ts_cnt - tmp_fc_cnt) {
//                    Fc_RR_z = 0;
//                    Fc_FL_z = 0;
//                    Fc_RL_z = -tmp_Fc2 + tmp_Fc2*0.5 * (1 - cos(PI2 / (tmp_fc_time * 2)*(double) (fc_cnt - (ts_cnt - tmp_fc_cnt)) * dt));
//                    Fc_FR_z = -tmp_Fc2 + tmp_Fc2*0.5 * (1 - cos(PI2 / (tmp_fc_time * 2)*(double) (fc_cnt - (ts_cnt - tmp_fc_cnt)) * dt));
//            }

        }
        else if(CommandFlag == NOMAL_TROT_WALKING){

            if (fc_cnt <= tmp_fc_cnt) {
                Fc_RR_z = -tmp_Fc1 + (tmp_Fc1)*0.5 * (1 - cos(PI2 / (tmp_fc_time * 2)*(double) (fc_cnt) * dt));
                Fc_FL_z = -tmp_Fc1 + (tmp_Fc1)*0.5 * (1 - cos(PI2 / (tmp_fc_time * 2)*(double) (fc_cnt) * dt));
                Fc_RL_z = -tmp_Fc1 + (tmp_Fc1 - tmp_Fc2)*0.5 * (1 - cos(PI2 / (tmp_fc_time * 2)*(double) (fc_cnt) * dt));
                Fc_FR_z = -tmp_Fc1 + (tmp_Fc1 - tmp_Fc2)*0.5 * (1 - cos(PI2 / (tmp_fc_time * 2)*(double) (fc_cnt) * dt));

            }
            else if (fc_cnt >= dsp_cnt - tmp_fc_cnt) {
                Fc_RR_z = -(tmp_Fc1)*0.5 * (1 - cos(PI2 / (tmp_fc_time * 2)*(double) (fc_cnt - (dsp_cnt - tmp_fc_cnt)) * dt));
                Fc_FL_z = -(tmp_Fc1)*0.5 * (1 - cos(PI2 / (tmp_fc_time * 2)*(double) (fc_cnt - (dsp_cnt - tmp_fc_cnt)) * dt));
                Fc_RL_z = -tmp_Fc2 + (-tmp_Fc1 + tmp_Fc2)*0.5 * (1 - cos(PI2 / (tmp_fc_time * 2)*(double) (fc_cnt - (dsp_cnt - tmp_fc_cnt)) * dt));
                Fc_FR_z = -tmp_Fc2 + (-tmp_Fc1 + tmp_Fc2)*0.5 * (1 - cos(PI2 / (tmp_fc_time * 2)*(double) (fc_cnt - (dsp_cnt - tmp_fc_cnt)) * dt));   
            }
            else{
                Fc_RL_z = -tmp_Fc2;
                Fc_RR_z =  0;
                Fc_FL_z =  0;
                Fc_FR_z = -tmp_Fc2;
            }
        }
        
        else{
            Fc_RL_z = -tmp_Fc2;
            Fc_RR_z = 0;
            Fc_FL_z = 0;
            Fc_FR_z = -tmp_Fc2;
        }

        fc_cnt++;
    }
    else if (FC_PHASE == STANCE_RRFL) {

        if(CommandFlag == FLYING_TROT_RUNNING){

            if(flying_trot_init_flag == true){
//				printf("=====================================init_flag = true====================================\n");
                if (fc_cnt <= tmp_fc_cnt) {
                    Fc_RR_z = -tmp_Fc1 + (tmp_Fc1 - tmp_Fc2)*0.5 * (1 - cos(PI2 / (tmp_fc_time * 2)*(double) (fc_cnt) * dt));
                    Fc_FL_z = -tmp_Fc1 + (tmp_Fc1 - tmp_Fc2)*0.5 * (1 - cos(PI2 / (tmp_fc_time * 2)*(double) (fc_cnt) * dt));
                    Fc_RL_z = -tmp_Fc1 + (tmp_Fc1)*0.5 * (1 - cos(PI2 / (tmp_fc_time * 2)*(double) (fc_cnt) * dt));
                    Fc_FR_z = -tmp_Fc1 + (tmp_Fc1)*0.5 * (1 - cos(PI2 / (tmp_fc_time * 2)*(double) (fc_cnt) * dt));

                }
                else if (fc_cnt >= ts_cnt - tmp_fc_cnt) {
                    Fc_RR_z = -tmp_Fc2 + tmp_Fc2*0.5 * (1 - cos(PI2 / (tmp_fc_time * 2)*(double) (fc_cnt - (ts_cnt - tmp_fc_cnt)) * dt));
                    Fc_FL_z = -tmp_Fc2 + tmp_Fc2*0.5 * (1 - cos(PI2 / (tmp_fc_time * 2)*(double) (fc_cnt - (ts_cnt - tmp_fc_cnt)) * dt));
                    Fc_RL_z = 0;
                    Fc_FR_z = 0;
                }
                
                
            }
            
            else{

                if (fc_cnt <= tmp_fc_cnt) {
                    Fc_RR_z = -tmp_Fc2*0.5 * (1 - cos(PI2 / (tmp_fc_time * 2)*(double) (fc_cnt) * dt));
                    Fc_FL_z = -tmp_Fc2*0.5 * (1 - cos(PI2 / (tmp_fc_time * 2)*(double) (fc_cnt) * dt));
                    Fc_RL_z = 0;
                    Fc_FR_z = 0;

                }
                else if (fc_cnt >= ts_cnt - tmp_fc_cnt) {
                    Fc_RR_z = -tmp_Fc2 + tmp_Fc2*0.5 * (1 - cos(PI2 / (tmp_fc_time * 2)*(double) (fc_cnt - (ts_cnt - tmp_fc_cnt)) * dt));
                    Fc_FL_z = -tmp_Fc2 + tmp_Fc2*0.5 * (1 - cos(PI2 / (tmp_fc_time * 2)*(double) (fc_cnt - (ts_cnt - tmp_fc_cnt)) * dt));
                    Fc_RL_z = 0;
                    Fc_FR_z = 0;
                }

            }

        }
        else if(CommandFlag == NOMAL_TROT_WALKING){

            if (fc_cnt <= tmp_fc_cnt) {
                Fc_RR_z = -tmp_Fc1 + (tmp_Fc1 - tmp_Fc2)*0.5 * (1 - cos(PI2 / (tmp_fc_time * 2)*(double) (fc_cnt) * dt));
                Fc_FL_z = -tmp_Fc1 + (tmp_Fc1 - tmp_Fc2)*0.5 * (1 - cos(PI2 / (tmp_fc_time * 2)*(double) (fc_cnt) * dt));
                Fc_RL_z = -tmp_Fc1 + (tmp_Fc1)*0.5 * (1 - cos(PI2 / (tmp_fc_time * 2)*(double) (fc_cnt) * dt));
                Fc_FR_z = -tmp_Fc1 + (tmp_Fc1)*0.5 * (1 - cos(PI2 / (tmp_fc_time * 2)*(double) (fc_cnt) * dt));

            }
            else if (fc_cnt >= dsp_cnt - tmp_fc_cnt) {
                Fc_RR_z = -tmp_Fc2 + (-tmp_Fc1 + tmp_Fc2)*0.5 * (1 - cos(PI2 / (tmp_fc_time * 2)*(double) (fc_cnt - (dsp_cnt - tmp_fc_cnt)) * dt));
                Fc_FL_z = -tmp_Fc2 + (-tmp_Fc1 + tmp_Fc2)*0.5 * (1 - cos(PI2 / (tmp_fc_time * 2)*(double) (fc_cnt - (dsp_cnt - tmp_fc_cnt)) * dt));
                Fc_RL_z = -(tmp_Fc1)*0.5 * (1 - cos(PI2 / (tmp_fc_time * 2)*(double) (fc_cnt - (dsp_cnt - tmp_fc_cnt)) * dt));
                Fc_FR_z = -(tmp_Fc1)*0.5 * (1 - cos(PI2 / (tmp_fc_time * 2)*(double) (fc_cnt - (dsp_cnt - tmp_fc_cnt)) * dt));
                
//                cout << "cnt = " << fc_cnt - (dsp_cnt - tmp_fc_cnt) << endl;
            }
            else{
                Fc_RL_z = 0;
                Fc_RR_z = -tmp_Fc2;
                Fc_FL_z = -tmp_Fc2;
                Fc_FR_z = 0;
            }
        }
        else{
            Fc_RL_z = 0;
            Fc_RR_z = -tmp_Fc2;
            Fc_FL_z = -tmp_Fc2;
            Fc_FR_z = 0;
        }

        fc_cnt++;
    }
    else if(FC_PHASE == ZERO){
        Fc_RL_z = 0;
        Fc_RR_z = 0;
        Fc_FL_z = 0;
        Fc_FR_z = 0;

        fc_cnt = 0;
    }

//		Fc   << 0, 0, 0, 0, 0, 0, 0 ,Fc_vsd[0], Fc_vsd[1] + F_fd_y, Fc_RL_z + Fc_vsd[2] ,Fc_vsd[3], Fc_vsd[4] + F_fd_y, Fc_RR_z + Fc_vsd[5],  Fc_vsd[6], Fc_vsd[7] + F_fd_y, Fc_FL_z + Fc_vsd[8],Fc_vsd[9], Fc_vsd[10] + F_fd_y, Fc_FR_z + Fc_vsd[11];
    //Fc << 0, 0, 0, 0, 0, 0, 0, Fc_vsd[0], Fc_vsd[1], Fc_RL_z + Fc_vsd[2], 0, 0, Fc_RR_z + Fc_vsd[5], 0, 0, Fc_FL_z + Fc_vsd[8],0,0, Fc_FR_z + Fc_vsd[11];
    Fc << 0, 0, 0, 0, 0, 0, 0, 0, 0, Fc_RL_z, 0, 0, Fc_RR_z, 0, 0, Fc_FL_z, 0, 0, Fc_FR_z;
}

void CRobot::Torque_off(void)
{
    for (int i = 0; i < nDOF; ++i) {
    	joint[i].torque = 0;
    }
}

void CRobot::Cal_CP(void){

//    static double CP_x_alpha = 0.05, CP_x_dot_alpha = 0.003;
//    static double CP_y_alpha = 0.03, CP_y_dot_alpha = 0.001;
//
//    natural_freq = sqrt(com_height / GRAVITY);
//
//    // roll
//    COM_y = -com_height * IMURoll * PI / 180;
//    COM_y_dot = -com_height * IMURoll_dot * PI / 180;
//
//    lpf_COM_y = (1 - CP_y_alpha) * lpf_COM_y + CP_y_alpha*COM_y;
//    lpf_COM_y_dot = (1 - CP_y_dot_alpha) * lpf_COM_y_dot + CP_y_dot_alpha*COM_y_dot;
//
//    CP_y = lpf_COM_y + 1 / natural_freq * lpf_COM_y_dot;
//
////    printf("IMURoll=%f\n",IMURoll);
//    
//    // pitch
//    COM_x = com_height * IMUPitch * PI / 180;
//    COM_x_dot = com_height * IMUPitch_dot * PI / 180;
//
//    lpf_COM_x = (1 - CP_x_alpha) * lpf_COM_x + CP_x_alpha*COM_x;
//    lpf_COM_x_dot = (1 - CP_x_dot_alpha) * lpf_COM_x_dot + CP_x_dot_alpha*COM_x_dot;
//
//    CP_x = lpf_COM_x + 1 / natural_freq * lpf_COM_x_dot;

}

void CRobot::Cal_CP2(void){

//    static double CP_x_alpha = 0.05, CP_x_dot_alpha = 0.003;
    static double CP_y_alpha = 0.03, CP_y_dot_alpha = 0.01;

    natural_freq = sqrt(com_height / GRAVITY);

    // roll
//    COM_y = -com_height * IMURoll * PI / 180;
//    COM_y_dot = -com_height * IMURoll_dot * PI / 180;
//
    lpf_COM_y = (1 - CP_y_alpha) * lpf_COM_y + CP_y_alpha*actual_com_pos[1];
    lpf_COM_y_dot = (1 - CP_y_dot_alpha) * lpf_COM_y_dot + CP_y_dot_alpha*actual_com_vel[1];

    CP_y = (lpf_COM_y + 1 / natural_freq * lpf_COM_y_dot)/5.0; // weighted CP_y
    
//    CP_y = actual_com_pos[1] + 1 / natural_freq * actual_com_vel[1]/10.0;
    
    
//    printf("IMURoll=%f\n",IMURoll);

//    // pitch
//    COM_x = com_height * IMUPitch * PI / 180;
//    COM_x_dot = com_height * IMUPitch_dot * PI / 180;
//
//    lpf_COM_x = (1 - CP_x_alpha) * lpf_COM_x + CP_x_alpha*COM_x;
//    lpf_COM_x_dot = (1 - CP_x_dot_alpha) * lpf_COM_x_dot + CP_x_dot_alpha*COM_x_dot;
//
//    CP_x = lpf_COM_x + 1 / natural_freq * lpf_COM_x_dot;

}

//void CRobot::Get_gain(void)
//{
//    double z_c = com_height;
//    int nCount = 0;
//    double temp_Gp_gain,temp_Gx_gain,temp_Gi_gain;
//
//    AA << 1, dt, dt*dt/2.0f ,
//              0, 1, dt,
//              0, 0, 1;
//
//    BB << dt*dt*dt/6.0f, dt*dt/2.0f, dt;
//
//    CC << 1, 0, -z_c/GRAVITY;
//
//    FILE *fp1;
//    FILE *fp2;
//    FILE *fp3;
//
//    fp1 = fopen("/home/user/Desktop/RcLab-PongBotQ2/src/gain/dh_Gp.txt","r");
//
//    if(fp1 == NULL)printf("CAN NOT OPEN Gp TEXT FILE \n");
//    while(fscanf(fp1,"%lf",&temp_Gp_gain)==1){pv_Gp[nCount] = temp_Gp_gain; nCount++;}
//    fclose(fp1);
//    nCount = 0;
//
//    fp2 = fopen("/home/user/Desktop/RcLab-PongBotQ2/src/gain/dh_Gx.txt","r");
//    if(fp2 == NULL)printf("CAN NOT OPEN Gx TEXT FILE \n");
//    while(fscanf(fp2,"%lf",&temp_Gx_gain)==1){pv_Gx[nCount] = temp_Gx_gain; nCount++;}
//    fclose(fp2);
//    nCount = 0;
//
//    fp3 = fopen("/home/user/Desktop/RcLab-PongBotQ2/src/gain/dh_Gi.txt","r");
//    if(fp3 == NULL)printf("CAN NOT OPEN Gi TEXT FILE \n");
//    while(fscanf(fp3,"%lf",&temp_Gi_gain)==1){pv_Gi[nCount] = temp_Gi_gain; nCount++;}
//    fclose(fp3);
//
//    Gi = pv_Gi[0];
//
//    Gx(0) = pv_Gx[0];
//    Gx(1) = pv_Gx[1];
//    Gx(2) = pv_Gx[2];
//
//    for(unsigned int i=0;i<preview_cnt-1;++i){
//      Gp(i) = pv_Gp[i];
//    }
//}


void CRobot::Get_gain(void)
{
    double z_c = com_height;
    int nCount = 0;
    double temp_Gp_gain,temp_Gx_gain,temp_Gi_gain;

    AA << 1, dt, dt*dt/2.0f ,
          0, 1, dt,
          0, 0, 1;

//    cout << "AA = " << AA << endl;

    BB << dt*dt*dt/6.0f, dt*dt/2.0f, dt;
//    cout << "BB = " << BB << endl;

    CC << 1, 0, -z_c/GRAVITY;
//    cout << "CC = " << CC << endl;

    FILE *fp1;
    FILE *fp2;
    FILE *fp3;

    if(Mode == MODE_ACTUAL_ROBOT){
        fp1 = fopen("/home/user/Desktop/RcLab-PongBotQ2/src/gain/dh_Gp.txt","r");

        if(fp1 == NULL)printf("CAN NOT OPEN Gp TEXT FILE \n");
        while(fscanf(fp1,"%lf",&temp_Gp_gain)==1){pv_Gp[nCount] = temp_Gp_gain; nCount++;}
        fclose(fp1);
        nCount = 0;

        fp2 = fopen("/home/user/Desktop/RcLab-PongBotQ2/src/gain/dh_Gx.txt","r");
        if(fp2 == NULL)printf("CAN NOT OPEN Gx TEXT FILE \n");
        while(fscanf(fp2,"%lf",&temp_Gx_gain)==1){pv_Gx[nCount] = temp_Gx_gain; nCount++;}
        fclose(fp2);
        nCount = 0;

        fp3 = fopen("/home/user/Desktop/RcLab-PongBotQ2/src/gain/dh_Gi.txt","r");
        if(fp3 == NULL)printf("CAN NOT OPEN Gi TEXT FILE \n");
        while(fscanf(fp3,"%lf",&temp_Gi_gain)==1){pv_Gi[nCount] = temp_Gi_gain; nCount++;}
        fclose(fp3);
    }
    else{ // simulation
    	fp1 = fopen("/root/catkin_ws/src/RcLab-PongBotQ2/src/gain/dh_Gp.txt","r");
        if(fp1 == NULL)printf("CAN NOT OPEN Gp TEXT FILE \n");
        while(fscanf(fp1,"%lf",&temp_Gp_gain)==1){pv_Gp[nCount] = temp_Gp_gain; nCount++;}
        fclose(fp1);
        nCount = 0;

        fp2 = fopen("/root/catkin_ws/src/RcLab-PongBotQ2/src/gain/dh_Gx.txt","r");
        if(fp2 == NULL)printf("CAN NOT OPEN Gx TEXT FILE \n");
        while(fscanf(fp2,"%lf",&temp_Gx_gain)==1){pv_Gx[nCount] = temp_Gx_gain; nCount++;}
        fclose(fp2);
        nCount = 0;

        fp3 = fopen("/root/catkin_ws/src/RcLab-PongBotQ2/src/gain/dh_Gi.txt","r");
        if(fp3 == NULL)printf("CAN NOT OPEN Gi TEXT FILE \n");
        while(fscanf(fp3,"%lf",&temp_Gi_gain)==1){pv_Gi[nCount] = temp_Gi_gain; nCount++;}
        fclose(fp3);
    }


    Gi = pv_Gi[0];
//        cout << pv_Gi[0] << endl;

    Gx(0) = pv_Gx[0];
    Gx(1) = pv_Gx[1];
    Gx(2) = pv_Gx[2];

//    cout << pv_Gx[0] << endl << pv_Gx[1] << endl << pv_Gx[2] << endl;

    for(unsigned int i=0;i<preview_cnt-1;++i){
      Gp(i) = pv_Gp[i];
    }

//    cout << pv_Gp[0] << endl << pv_Gp[1] << endl;
}


void CRobot::get_zmp(void)
{
    static double sum_F, sum_Mx, sum_My;
    static double l_RL_x, l_RL_y, l_RR_x, l_RR_y, l_FL_x, l_FL_y, l_FR_x, l_FR_y;
    const double alpha = 0.08;

    l_RL_x = -0.35 + target_EP[0];
    l_RR_x = -0.35 + target_EP[3];
    l_FL_x =  0.35 - target_EP[6];
    l_FR_x =  0.35 - target_EP[9];
    
//    cout << "l_RL_x="<<l_RL_x  << "l_RR_x="<< l_RR_x << "l_FL_x=" << l_FL_x << "l_FR_x=" << l_FR_x << endl;
    
    l_RL_y = 0;// 0.115 + target_EP[1];
    l_RR_y = 0;//-0.115 + target_EP[4];
    l_FL_y = 0;// 0.115 + target_EP[7];
    l_FR_y = 0;//-0.115 + target_EP[10];
    
    
    sum_Mx = 0;
    sum_My = l_RL_x*RL.ftSensor.Fz + l_RR_x*RR.ftSensor.Fz + l_FL_x*FL.ftSensor.Fz + l_FR_x*FR.ftSensor.Fz;
    sum_F = RL.ftSensor.Fz + RR.ftSensor.Fz + FL.ftSensor.Fz + FR.ftSensor.Fz;
    
    if(sum_F > 50){
        zmp_x = com_pos(0) + (sum_My)/(sum_F); // from local to global
        zmp_y = (sum_Mx)/(sum_F);
    }
    else{
        zmp_x = 0;
        zmp_y = 0;
    }
    
    lpf_zmp_x = (1-alpha)*old_lpf_zmp_x + alpha*zmp_x;

    old_lpf_zmp_x = lpf_zmp_x;
    
    
//    cout << "zmp_x = " << zmp_x << endl;
//    cout << "RLz = " << RL_Force_E[2] << "RRz = " << RR_Force_E[2] << "FLz = " << FL_Force_E[2] << "FRz = " << FR_Force_E[2] << endl;
}


void CRobot::Get_act_com(void){
    if(CommandFlag == GOTO_WALK_READY_POS){
//    	actual_com_pos[1] = -(actual_pos[0] + actual_pos[3] + actual_pos[7] + actual_pos[10])/ 4.0 * com_height;
//      actual_com_vel[1] = -(actual_vel[0] + actual_vel[3] + actual_vel[7] + actual_vel[10])/ 4.0 * com_height;


        if(FC_PHASE == STOP){
//            cout << "1" << endl;
            actual_com_pos[1] = -(actual_pos[0] + actual_pos[3] + actual_pos[7] + actual_pos[10])/ 4.0 * com_height;
            actual_com_vel[1] = -(actual_vel[0] + actual_vel[3] + actual_vel[7] + actual_vel[10])/ 4.0 * com_height;
        }
        else if(FC_PHASE == STANCE_RRFL){
            actual_com_pos[1] = -(actual_pos[3] + actual_pos[7])/ 2.0 * com_height;
            actual_com_vel[1] = -(actual_vel[3] + actual_vel[7])/ 2.0 * com_height;
        }
        else if(FC_PHASE == STANCE_RLFR){
            actual_com_pos[1] = -(actual_pos[0] + actual_pos[10])/ 2.0 * com_height;
            actual_com_vel[1] = -(actual_vel[0] + actual_vel[10])/ 2.0 * com_height;
        }

    }


    else if(CommandFlag == NOMAL_TROT_WALKING){
        if(FC_PHASE == STOP){
            actual_com_pos[1] = -(actual_pos[0] + actual_pos[3] + actual_pos[7] + actual_pos[10])/ 4.0 * com_height;
            actual_com_vel[1] = -(actual_vel[0] + actual_vel[3] + actual_vel[7] + actual_vel[10])/ 4.0 * com_height;
        }
        else if(FC_PHASE == STANCE_RRFL){
            actual_com_pos[1] = -(actual_pos[3] + actual_pos[7])/ 2.0 * com_height;
            actual_com_vel[1] = -(actual_vel[3] + actual_vel[7])/ 2.0 * com_height;
        }
        else if(FC_PHASE == STANCE_RLFR){
            actual_com_pos[1] = -(actual_pos[0] + actual_pos[10])/ 2.0 * com_height;
            actual_com_vel[1] = -(actual_vel[0] + actual_vel[10])/ 2.0 * com_height;
        }
    }
    else{
    	actual_com_pos[1] = 0;
        actual_com_vel[1] = 0;
    }
}

void CRobot::Damping_con(void){
    static double front_hip_roll_err = 0, rear_hip_roll_err = 0;
	static double target_front_hip_roll_err = 0, target_rear_hip_roll_err = 0;
    static double front_hip_roll_err_dot = 0, rear_hip_roll_err_dot = 0;

	target_front_hip_roll_err = target_pos[7] - target_pos[10];
	target_rear_hip_roll_err = target_pos[0] - target_pos[3];

	front_hip_roll_err = actual_pos[7] - actual_pos[10];
	rear_hip_roll_err = actual_pos[0] - actual_pos[3];

	front_hip_roll_err_dot = actual_vel[7] - actual_vel[10];
	rear_hip_roll_err_dot = actual_vel[0] - actual_vel[3];


	// hip roll error compensation
	target_pos_offset[7]  = +kp_roll*(target_front_hip_roll_err-front_hip_roll_err) + kd_roll*(0-front_hip_roll_err_dot);
	target_pos_offset[10] = -kp_roll*(target_front_hip_roll_err-front_hip_roll_err) - kd_roll*(0-front_hip_roll_err_dot);
	target_pos_offset[0]  = +kp_roll*(target_rear_hip_roll_err-rear_hip_roll_err) + kd_roll*(0-rear_hip_roll_err_dot);
	target_pos_offset[3]  = -kp_roll*(target_rear_hip_roll_err-rear_hip_roll_err) - kd_roll*(0-rear_hip_roll_err_dot);


	target_pos_with_con = target_pos + target_pos_offset;
}
