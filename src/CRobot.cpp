
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
    Mode = MODE_SIMULATION;
    //	Mode = MODE_ACTUAL_ROBOT;
    

    // ============== Controller OnOff ================ //
    Body_Ori_Con_onoff_flag = true; //false; //true; //true;
    CP_con_onoff_flag = false; //true; //false;//true;
    // ============== Controller OnOff End ================ //


    if (Mode == MODE_SIMULATION) {
        //        foot_z_offset << -0.03, -0.03, 0.00, 0.00;
        foot_height = 0.05;
        swing_foot_height = 0.05;


        //        Kp_q << 400, 600, 600, 400, 600, 600, 20000, 400, 600, 600, 400, 600, 600;
        //        Kd_q <<  10,  15,  15,  10,  15,  15,   200,  10,  15,  15,  10,  15,  15;

        //2019.11.20
        //    	Kp_q << 100, 400, 400, 100, 400, 400, 20000, 100, 400, 400, 100, 400, 400;
        //    	Kd_q <<   2,  15,  15,   2,  15,  15,   200,   2,  15,  15,   2,  15,  15;

        //        Kp_q << 100, 400, 400, 100, 400, 400, 20000, 100, 400, 400, 100, 400, 400;
        //    	Kd_q <<   1,  10,  10,   1,  10,  10,   200,   1,  10,  10,   1,  10,  10;

        //2019.12.10
        //        Kp_q << 100, 400, 400, 100, 400, 400, 20000, 100, 400, 400, 100, 400, 400;
        //        Kd_q << 1, 10, 10, 1, 10, 10, 200, 1, 10, 10, 1, 10, 10;

        //2019.12.17
//        Kp_q << 200, 400, 400, 200, 400, 400, 5000, 200, 400, 400, 200, 400, 400;
//        Kd_q << 2, 10, 10, 2, 10, 10, 100, 2, 10, 10, 2, 10, 10;
        
        //2020.01.21
//        Kp_q << 40, 40, 40, 40, 40, 40, 1000, 40, 40, 40, 40, 40, 40;
//        Kd_q << 1, 1, 1, 1, 1, 1, 20, 1, 1, 1, 1, 1, 1;
        
        Kp_q << 400, 500, 500, 400, 500, 500, 5000, 400, 500, 500, 400, 500, 500;
        Kd_q << 10, 12, 12, 10, 12, 12, 100, 10, 12, 12, 10, 12, 12;

        Kp_t << 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100;
        Kd_t << 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1;
        //2020.01.02
        //        Kp_q << 200, 300, 300, 200, 300, 300, 20000, 200, 300, 300, 200, 300, 300;
        //        Kd_q <<   2,   7,   7,   2,   7,   7,   200,   2,   7,   7,   2,   7,   7;

        //        Kp_q << 300, 300, 300, 300, 300, 300, 20000, 300, 300, 300, 300, 300, 300;
        //        Kd_q <<   7,   7,   7,   7,   7,   7,   200,   7,   7,   7,   7,   7,   7;

        //2020.01.02 v2
        //        Kp_q << 400, 500, 500, 400, 500, 500, 20000, 400, 500, 500, 400, 500, 500;
        //        Kd_q <<  10,  12,  12,  10,  12,  12,   200,  10,  12,  12,  10,  12,  12;
        //        
        FT_Kp_q << 400, 500, 500, 400, 500, 500, 5000, 400, 500, 500, 400, 500, 500;
        FT_Kd_q << 10, 12, 12, 10, 12, 12, 100, 10, 12, 12, 10, 12, 12;



        //        BOC_Kp_roll = 0.0005;
        //        BOC_Ki_roll = 0.003;
        //        BOC_Kp_pitch = 0.0005;
        //        BOC_Ki_pitch = 0.003;

        BOC_Kp_roll = 0.00010;
        BOC_Ki_roll = 0.010; //0.003
        BOC_Kp_pitch = 0.00010;
        BOC_Ki_pitch = 0.010;

        //        target_kp_roll = 0;
        //        target_kd_roll = 0.0;

        //        target_kp_roll = 0.5; //2;
        //        target_kd_roll = 0.005; //0.01;

        //        target_kp_roll = 2; //0.1; //2;
        //        target_kd_roll = 0.001; //0.001; //0.01;

        target_kp_roll = 0; //0.05;//0.5; //0.5;
        target_kd_roll = 0; //0.00005;//0.0001; //0.0001;

        com_height = 0.40;
        init_com_pos << 0.0, 0, com_height;

        //        init_RL_foot_pos << -0.02,  0.105, -0.0;
        //        init_RR_foot_pos << -0.02, -0.105, -0.0;
        //        init_FL_foot_pos << -0.00,  0.105, 0.0;
        //        init_FR_foot_pos << -0.00, -0.105, 0;

        //        init_RL_foot_pos << -0.01,  0.105, -0.02;
        //        init_RR_foot_pos << -0.01, -0.105, -0.02;
        //        init_FL_foot_pos <<  0.00,  0.105,  0.0;
        //        init_FR_foot_pos <<  0.00, -0.105,  0.0;

        init_RL_foot_pos << 0.05, 0.105, -0.03;
        init_RR_foot_pos << 0.05, -0.105, -0.03;
        init_FL_foot_pos << 0.05, 0.105, 0.0;
        init_FR_foot_pos << 0.05, -0.105, 0.0;



    }
    else if (Mode == MODE_ACTUAL_ROBOT) {
        //        foot_z_offset << -0.03, -0.03, 0.00, 0.00;
        //        foot_z_offset << 0.0, -0.0, -0.0, -0.0;
        foot_height = 0.05; //0.05
        swing_foot_height = 0.050;

        //        Kp_q << 30, 500, 600, 30, 500, 600, 20000, 30, 500, 600, 30, 500, 600;
        //        Kd_q <<  1,  10,  20,  1,  10,  20,   200,  1,  10,  20,  1,  10,  20;

        //        Kp_q << 30, 400, 400, 30, 400, 400, 20000, 30, 400, 400, 30, 400, 400;
        //        Kd_q << 0.5, 10, 10, 0.5, 10, 10, 200, 0.5, 10, 10, 0.5, 10, 10;

        //        Kp_q << 200, 500, 600, 200, 500, 600, 20000, 200, 500, 600, 200, 500, 600;
        //		Kd_q <<   3,  10,  20,   3,  10,  20,   200,   3,  10,  20,   3,  10,  20;

        // 2019.12.16
        //        Kp_q << 200, 400, 400, 200, 400, 400, 20000, 200, 400, 400, 200, 400, 400;
        //        Kd_q <<   3,  10,  10,   3,  10,  10,   200,   3,  10,  10,   3,  10,  10;
        // 2019.12.27
        //        Kp_q << 400, 500, 600, 400, 500, 600, 20000, 400, 500, 600, 400, 500, 600;
        //        Kd_q <<  10,  12,  20,  10,  12,  20,   200,  10,  12,  20,  10,  12,  20;
        // 2019.12.30
        //        Kp_q << 400, 400, 400, 400, 400, 400, 20000, 400, 400, 400, 400, 400, 400;
        //        Kd_q <<  10,  10,  10,  10,  10,  10,   200,  10,  10,  10,  10,  10,  10;
        // 2019.12.31
        //        Kp_q << 400, 500, 500, 400, 500, 500, 5000, 400, 500, 500, 400, 500, 500;
        //        Kd_q << 10, 12, 12, 10, 12, 12, 100, 10, 12, 12, 10, 12, 12;

        //        Kp_q << 200, 300, 300, 200, 300, 300, 20000, 200, 300, 300, 200, 300, 300;
        //        Kd_q <<   2,   7,   7,   2,   7,   7,   200,   2,   7,   7,   2,   7,   7;

        Kp_q << 200, 400, 400, 200, 400, 400, 5000, 200, 400, 400, 200, 400, 400;
        Kd_q << 3, 10, 10, 3, 10, 10, 100, 3, 10, 10, 3, 10, 10;

        FT_Kp_q << 300, 500, 500, 300, 500, 500, 5000, 300, 500, 500, 300, 500, 500;
        FT_Kd_q << 5, 12, 12, 5, 12, 12, 100, 5, 12, 12, 5, 12, 12;




        //	  BOC_Kp_roll = 0.0004;
        //	  BOC_Ki_roll = 0.04; //0.003
        //	  BOC_Kp_pitch = 0.0004;
        //        BOC_Ki_pitch = 0.04;

        //        BOC_Kp_roll = 0.001;
        //        BOC_Ki_roll = 0.02; //0.003
        //        BOC_Kp_pitch = 0.001;
        //        BOC_Ki_pitch = 0.02;

        BOC_Kp_roll = 0.00010;
        BOC_Ki_roll = 0.010; //0.003
        BOC_Kp_pitch = 0.00010;
        BOC_Ki_pitch = 0.010;

        // Damping control : leg length constraint
        //        target_kp_roll = 5;
        //        target_kd_roll = 0.03;
        target_kp_roll = 0; //0.05;//0.5; //0.5;
        target_kd_roll = 0; //0.00005;//0.0001; //0.0001;

        com_height = 0.40;
        init_com_pos << 0.0, 0, com_height;

        //        init_RL_foot_pos << -0.00,  0.105, -0.038;
        //        init_RR_foot_pos << -0.00, -0.105, -0.035;
        //        init_FL_foot_pos << -0.02,  0.105, 0.0;
        //        init_FR_foot_pos << -0.02, -0.105, 0;

        //        init_RL_foot_pos << -0.01,  0.105, -0.035;
        //        init_RR_foot_pos << -0.01, -0.105, -0.03;
        //        init_FL_foot_pos <<  0.00,  0.105, -0.00;
        //        init_FR_foot_pos <<  0.00, -0.105, -0.0;

        init_RL_foot_pos << 0.03, 0.105, -0.030;
        init_RR_foot_pos << 0.03, -0.105, -0.030;
        init_FL_foot_pos << 0.03, 0.105, -0.0;
        init_FR_foot_pos << 0.03, -0.105, -0.0;
    }

    des_pitch_deg = 4.0;

    init_Kp_q = Kp_q;
    init_Kd_q = Kd_q;

    //    com_height = 0.40; // 0.40 // 0.43
    x_moving_speed = 0;
    y_moving_speed = 0;

    //    init_foot_l_pos << 0.0,  0.085, 0; //0, 0.105, 0;
    //    init_foot_r_pos << 0.0, -0.085, 0; //0, -0.105, 0;
    //    init_com_pos << 0.0, 0, com_height;
    //
    ////    init_RL_foot_pos << -0.00, 0.085, -0.0;
    ////    init_RR_foot_pos << -0.00, -0.085, -0.0;
    ////    init_FL_foot_pos << -0.00, 0.085, 0;
    ////    init_FR_foot_pos << -0.00, -0.085, 0;
    //
    //    init_RL_foot_pos << -0.00,  0.105, -0.038;
    //    init_RR_foot_pos << -0.00, -0.105, -0.035;
    //    init_FL_foot_pos << -0.02,  0.105, 0.0;
    //    init_FR_foot_pos << -0.02, -0.105, 0;

    VectorNd tmp_init_RL_foot_pos = VectorNd::Zero(3);
    VectorNd tmp_init_RR_foot_pos = VectorNd::Zero(3);
    VectorNd tmp_init_FL_foot_pos = VectorNd::Zero(3);
    VectorNd tmp_init_FR_foot_pos = VectorNd::Zero(3);

    tmp_init_RL_foot_pos = init_RL_foot_pos - init_com_pos;
    tmp_init_RR_foot_pos = init_RR_foot_pos - init_com_pos;
    tmp_init_FL_foot_pos = init_FL_foot_pos - init_com_pos;
    tmp_init_FR_foot_pos = init_FR_foot_pos - init_com_pos;

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
    goal_EP << tmp_init_RL_foot_pos(0), tmp_init_RL_foot_pos(1), tmp_init_RL_foot_pos(2), tmp_init_RR_foot_pos(0), tmp_init_RR_foot_pos(1), tmp_init_RR_foot_pos(2), tmp_init_FL_foot_pos(0), tmp_init_FL_foot_pos(1), tmp_init_FL_foot_pos(2), tmp_init_FR_foot_pos(0), tmp_init_FR_foot_pos(1), tmp_init_FR_foot_pos(2);

    //    Get_gain(); // for preview control

    home_pos_time = 2;

    for (unsigned int i = 0; i < 13; ++i) {
        joint[i].torque = 0;
    }

    IMURoll = 0;
    IMUPitch = 0;
    IMUYaw = 0;

    // =============== Flying trot parameters initialize =============== //

    //    ts = 0.10;
    //    tf = 0.07;
    //    ft_step_time = ts + tf;
    //
    //    ts_cnt = 100;
    //    tf_cnt = 70;
    //    ft_step_cnt = ts_cnt + tf_cnt;

    ts = 0.22; //0.22; //0.25;
    tf = 0.07; //0.07;
    ft_step_time = ts + tf;

    ts_cnt = 220; //220;
    tf_cnt = 70;
    ft_step_cnt = ts_cnt + tf_cnt;

    h_0 = init_com_pos(2);
    v_0 = 0;
    a_0 = 0;

    v_1 = 0.15; //0.10; //0.15;
    a_1 = -GRAVITY;

    h_2 = init_com_pos(2);
    v_2 = -0.0; //-0.05; // -0.3
    a_2 = -GRAVITY;

    h_3 = init_com_pos(2);
    v_3 = 0;
    a_3 = 0;

    h_1 = 0.5 * GRAVITY * tf * tf - v_1 * tf + h_2;

    flying_trot_final_flag = false;
    flying_trot_init_flag = false;
    // =============== Flying trot parameters initialize END =============== //

    // =============== CP Initialize =============== //
    CP_check_flag = true;
    CP_PHASE = 0;
    get_cp_done_flag = false;
    //    CP_first_step_flag = true;
    CP_move_step = 0;
    CP_move_done_flag = false;

    cp_y_limit = 0.03;

    // =============== CP Initialize END =============== //

    //    tar_Fc_RL = 0; //100;
    //    tar_Fc_RR = 0; //100;
    //    tar_Fc_FL = 200; //100;
    //    tar_Fc_FR = 200; //100;

    tar_Fc_RL = 120; //100;
    tar_Fc_RR = 120; //100;
    tar_Fc_FL = 120; //100;
    tar_Fc_FR = 120; //100;

    Kp_cp = 0.05; //0.05;

    moving_done_flag = true;
    walk_ready_moving_done_flag = false;

    init_cp_RL_foot_pos << 0, 0, 0;
    init_cp_RR_foot_pos << 0, 0, 0;
    init_cp_FL_foot_pos << 0, 0, 0;
    init_cp_FR_foot_pos << 0, 0, 0;

    ft_ready_flag = false;
    ft_finish_flag = false;
    
    for (unsigned int i = 0; i < 6; ++i) {
        pd_con_joint[i] = 0;
        pd_con_task[i] = 0;
    }
    
    tmp_CTC_Torque = CTC_Torque;
    
    // Selection matrix
    
    S_mat.block<6, 19>(0, 0) = MatrixNd::Zero(6, 19);
    S_mat.block<13, 6>(6, 0) = MatrixNd::Zero(13, 6);
//    S_mat.block<13, 13>(6, 6) = MatrixNd::Zero(13, 13);//MatrixNd::Identity(13, 13);
    S_mat.block<13, 13>(6, 6) = MatrixNd::Identity(13, 13);
}


// ============================================================================================ //

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
        RobotState(6 + nJoint) = target_pos[nJoint]; //actual_pos[nJoint];
        RobotStatedot(6 + nJoint) = target_vel[nJoint]; //actual_vel[nJoint];
        RobotState2dot(6 + nJoint) = target_acc[nJoint]; //actual_vel[nJoint];
    }

    Math::Quaternion QQ(0, 0, 0, 1);
    m_pModel->SetQuaternion(base.ID, QQ, RobotState);

    if ((Body_Ori_Con_onoff_flag == true && CommandFlag != FLYING_TROT_RUNNING) || (Body_Ori_Con_onoff_flag == true && CommandFlag == FLYING_TROT_RUNNING && ft_finish_flag == true)) {
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

    CalcPointJacobian6D(*m_pModel, RobotState, base.ID, Originbase, J_BASE, true);
    CalcPointJacobian6D(*m_pModel, RobotState, front_body.ID, Originbase, J_FRONT_BODY, true);
    CalcPointJacobian(*m_pModel, RobotState, RL.ID, EP_OFFSET_RL, J_RL, true);
    CalcPointJacobian(*m_pModel, RobotState, RR.ID, EP_OFFSET_RR, J_RR, true);
    CalcPointJacobian(*m_pModel, RobotState, FL.ID, EP_OFFSET_FL, J_FL, true);
    CalcPointJacobian(*m_pModel, RobotState, FR.ID, EP_OFFSET_FR, J_FR, true);
    
    J_A.block<6, 19>(0, 0) = J_BASE;
    J_A.block<1, 19>(6, 0) = J_FRONT_BODY.block<1, 19>(2, 0); // only yaw
    J_A.block<3, 19>(7, 0) = J_RL;
    J_A.block<3, 19>(10, 0) = J_RR;
    J_A.block<3, 19>(13, 0) = J_FL;
    J_A.block<3, 19>(16, 0) = J_FR;

    x_dot = J_A*RobotStatedot;

    actual_EP_vel = x_dot.block(7, 0, 12, 1);
    
    // ====================== Get dJdQ for CTC ===================== //
 
    base_dJdQ = CalcPointAcceleration6D(*m_pModel, RobotState, RobotStatedot, ddqZero, base.ID, Originbase, true);
    FRONT_BODY_dJdQ = CalcPointAcceleration6D(*m_pModel, RobotState, RobotStatedot, ddqZero, front_body.ID, Originbase, true);
    RL_dJdQ = CalcPointAcceleration(*m_pModel, RobotState, RobotStatedot, ddqZero, RL.ID, EP_OFFSET_RL, true);
    RR_dJdQ = CalcPointAcceleration(*m_pModel, RobotState, RobotStatedot, ddqZero, RR.ID, EP_OFFSET_RR, true);
    FL_dJdQ = CalcPointAcceleration(*m_pModel, RobotState, RobotStatedot, ddqZero, FL.ID, EP_OFFSET_FL, true);
    FR_dJdQ = CalcPointAcceleration(*m_pModel, RobotState, RobotStatedot, ddqZero, FR.ID, EP_OFFSET_FR, true);

    dJdQ.block<6, 1>(0, 0)  = base_dJdQ;
    dJdQ.block<1, 1>(6, 0)  = FRONT_BODY_dJdQ.block<1, 1>(2, 0);
    dJdQ.block<3, 1>(7, 0)  = RL_dJdQ;
    dJdQ.block<3, 1>(10, 0) = RR_dJdQ;
    dJdQ.block<3, 1>(13, 0) = FL_dJdQ;
    dJdQ.block<3, 1>(16, 0) = FR_dJdQ;
    
    
    for (unsigned int i = 0; i < 12; ++i) {
        //        EP_vel_err[i] = target_EP_vel[i] - actual_EP_vel[i];
        EP_err[i] = tmp_target_EP[i] - actual_EP[i];
//        tmp_data2[i] = EP_err[i];
    }

    Cal_CP2();

    if (CommandFlag != TEST_FLAG) {
        Cal_Fc();
    }


    for (unsigned int i = 0; i < 13; ++i) {
        pd_con_joint[i + 6] = Kp_q[i]*(target_pos_with_con[i] - actual_pos[i]) + Kd_q[i]*(target_vel[i] - actual_vel[i]);
    }
    
//    for (unsigned int i = 0; i < 12; ++i) {
//        pd_con_task[i + 6] = Kp_t[i]*(tmp_target_EP[i] - actual_EP[i]) + Kd_t[i]*(0 - actual_EP_vel[i]);
//     }
    

    CompositeRigidBodyAlgorithm(*m_pModel, RobotState, M_term, true);
    NonlinearEffects(*m_pModel, RobotState, RobotStatedot, hatNonLinearEffects);
    NonlinearEffects(*m_pModel, RobotState, VectorNd::Zero(m_pModel->dof_count), G_term);

    C_term = hatNonLinearEffects - G_term;
  
    Cal_JFc();
    
    CTC_Torque = C_term + G_term  + pd_con_joint + JFc;// - pd_con_task); //
    for (int nJoint = 0; nJoint < nDOF; nJoint++) {
        joint[nJoint].torque = CTC_Torque(6 + nJoint);
    }
    
    tmp_data2[0] = Fc2[16];
    tmp_data2[1] = Fc2[17];
    tmp_data2[2] = Fc2[18];
    tmp_data2[3] = CTC_Torque[16];
    tmp_data2[4] = CTC_Torque[17];
    tmp_data2[5] = CTC_Torque[18];
    tmp_data2[6] = C_term[16];
    tmp_data2[7] = C_term[17];
    tmp_data2[8] = C_term[18];
    tmp_data2[9] = G_term[16];
    tmp_data2[10] = G_term[17];
    tmp_data2[11] = G_term[18];
    tmp_data2[12] = pd_con_joint[16];
    tmp_data2[13] = pd_con_joint[17];
    tmp_data2[14] = pd_con_joint[18];
    tmp_data2[15] = JFc[16];
    tmp_data2[16] = JFc[17];
    tmp_data2[17] = JFc[18];    
        
    
    
    
//    cout << "J_A.transpose() * (Fc2) = " << J_A.transpose() * (Fc2) << endl;
    
    cout << "[2] CTC_Torque = " << CTC_Torque.transpose() << endl;
    
//    cout << endl << M_term << endl;
    
    
//    cout << S_mat << endl << endl;
//    S_mat = 
    

    
    
}

void CRobot::FTsensorTransformation()
{
    //    RL.T_matrix = CalcBodyWorldOrientation(*m_pModel, RobotState, RL.ID, true);
    //    RR.T_matrix = CalcBodyWorldOrientation(*m_pModel, RobotState, RR.ID, true);
    //    FL.T_matrix = CalcBodyWorldOrientation(*m_pModel, RobotState, FL.ID, true);
    //    FR.T_matrix = CalcBodyWorldOrientation(*m_pModel, RobotState, FR.ID, true);

    C_I_roll << 1, 0, 0, 0, cos(base.currentRoll), -sin(base.currentRoll), 0, sin(base.currentRoll), cos(base.currentRoll);

    C_I_pitch << cos(base.currentPitch), 0, sin(base.currentPitch), 0, 1, 0, -sin(base.currentPitch), 0, cos(base.currentPitch);

    RL_C_I_HP << 1, 0, 0, 0, cos(actual_pos[0]), -sin(actual_pos[0]), 0, sin(actual_pos[0]), cos(actual_pos[0]);
    RL_C_HP_HR << cos(actual_pos[1]), 0, sin(actual_pos[1]), 0, 1, 0, -sin(actual_pos[1]), 0, cos(actual_pos[1]);
    RL_C_HR_KN << cos(actual_pos[2]), 0, sin(actual_pos[2]), 0, 1, 0, -sin(actual_pos[2]), 0, cos(actual_pos[2]);
    RL_C_KN_TIP << cos(0), 0, sin(0), 0, 1, 0, -sin(0), 0, cos(0);
    RL.T_matrix = C_I_roll * C_I_pitch * RL_C_I_HP * RL_C_HP_HR * RL_C_HR_KN * RL_C_KN_TIP;

    RR_C_I_HP << 1, 0, 0, 0, cos(actual_pos[3]), -sin(actual_pos[3]), 0, sin(actual_pos[3]), cos(actual_pos[3]);
    RR_C_HP_HR << cos(actual_pos[4]), 0, sin(actual_pos[4]), 0, 1, 0, -sin(actual_pos[4]), 0, cos(actual_pos[4]);
    RR_C_HR_KN << cos(actual_pos[5]), 0, sin(actual_pos[5]), 0, 1, 0, -sin(actual_pos[5]), 0, cos(actual_pos[5]);
    RR_C_KN_TIP << cos(0), 0, sin(0), 0, 1, 0, -sin(0), 0, cos(0);
    RR.T_matrix = C_I_roll * C_I_pitch * RR_C_I_HP * RR_C_HP_HR * RR_C_HR_KN * RR_C_KN_TIP;

    FL_C_I_HP << 1, 0, 0, 0, cos(actual_pos[7]), -sin(actual_pos[7]), 0, sin(actual_pos[7]), cos(actual_pos[7]);
    FL_C_HP_HR << cos(actual_pos[8]), 0, sin(actual_pos[8]), 0, 1, 0, -sin(actual_pos[8]), 0, cos(actual_pos[8]);
    FL_C_HR_KN << cos(actual_pos[9]), 0, sin(actual_pos[9]), 0, 1, 0, -sin(actual_pos[9]), 0, cos(actual_pos[9]);
    FL_C_KN_TIP << cos(0), 0, sin(0), 0, 1, 0, -sin(0), 0, cos(0);
    FL.T_matrix = C_I_roll * C_I_pitch * FL_C_I_HP * FL_C_HP_HR * FL_C_HR_KN * FL_C_KN_TIP;

    FR_C_I_HP << 1, 0, 0, 0, cos(actual_pos[10]), -sin(actual_pos[10]), 0, sin(actual_pos[10]), cos(actual_pos[10]);
    FR_C_HP_HR << cos(actual_pos[11]), 0, sin(actual_pos[11]), 0, 1, 0, -sin(actual_pos[11]), 0, cos(actual_pos[11]);
    FR_C_HR_KN << cos(actual_pos[12]), 0, sin(actual_pos[12]), 0, 1, 0, -sin(actual_pos[12]), 0, cos(actual_pos[12]);
    FR_C_KN_TIP << cos(0), 0, sin(0), 0, 1, 0, -sin(0), 0, cos(0);
    FR.T_matrix = C_I_roll * C_I_pitch * FR_C_I_HP * FR_C_HP_HR * FR_C_HR_KN * FR_C_KN_TIP;

}

VectorNd CRobot::FK1(VectorNd q)
{
    const double L1 = 0.105;
    const double L2 = 0.305;
    const double L3 = 0.30946; //0.305;

    static double q1 = 0;
    static double q2 = 0;
    static double q3 = 0;

    q1 = q[0];
    q2 = -q[1];
    q3 = -q[2];

    actual_EP[0] = -(-L3 * (cos(q2) * sin(q3) + cos(q3) * sin(q2)) - L2 * sin(q2));
    actual_EP[1] = L1 * cos(q1) - L3 * (sin(q1) * sin(q2) * sin(q3) - cos(q2) * cos(q3) * sin(q1)) + L2 * cos(q2) * sin(q1);
    actual_EP[2] = L1 * sin(q1) - L3 * (cos(q1) * cos(q2) * cos(q3) - cos(q1) * sin(q2) * sin(q3)) - L2 * cos(q1) * cos(q2);

    q1 = q[3];
    q2 = -q[4];
    q3 = -q[5];

    actual_EP[3] = -(-L3 * (cos(q2) * sin(q3) + cos(q3) * sin(q2)) - L2 * sin(q2));
    actual_EP[4] = L2 * cos(q2) * sin(q1) - L1 * cos(q1) - L3 * (sin(q1) * sin(q2) * sin(q3) - cos(q2) * cos(q3) * sin(q1));
    actual_EP[5] = -L1 * sin(q1) - L3 * (cos(q1) * cos(q2) * cos(q3) - cos(q1) * sin(q2) * sin(q3)) - L2 * cos(q1) * cos(q2);

    q1 = q[7];
    q2 = -q[8];
    q3 = -q[9];

    actual_EP[6] = -(-L3 * (cos(q2) * sin(q3) + cos(q3) * sin(q2)) - L2 * sin(q2));
    actual_EP[7] = L1 * cos(q1) - L3 * (sin(q1) * sin(q2) * sin(q3) - cos(q2) * cos(q3) * sin(q1)) + L2 * cos(q2) * sin(q1);
    actual_EP[8] = L1 * sin(q1) - L3 * (cos(q1) * cos(q2) * cos(q3) - cos(q1) * sin(q2) * sin(q3)) - L2 * cos(q1) * cos(q2);

    q1 = q[10];
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
    const double L3 = 0.30946; //0.305;

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

    //    cout << "tar_pos = " << target_pos.transpose()*R2D << endl;
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

void CRobot::WalkReady_Pos_Traj(void)
{
    // this is commented out by HSKIM(to subdivide the forces depending on the Modes.)

    if (ctc_cnt == 0) {
        moving_done_flag = false;
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
            
//            target_EP[i] = init_EP[i];// + (goal_EP[i] - init_EP[i]) / 2.0 * (1 - cos(PI2 / (home_pos_time * 2)*(double) (ctc_cnt) * dt));
//            target_EP_vel[i] = 0;//(goal_EP[i] - init_EP[i]) / 2.0 * PI2 / (home_pos_time * 2)*(sin(PI2 / (home_pos_time * 2)*(double) (ctc_cnt) * dt));
//            target_EP_acc[i] = 0;//(goal_EP[i] - init_EP[i]) / 2.0 * PI2 / (home_pos_time * 2) * PI2 / (home_pos_time * 2)*(cos(PI2 / (home_pos_time * 2)*(double) (ctc_cnt) * dt));
        }

        // waist
        target_pos[6] = init_pos[6] + (0 - init_pos[6]) / 2.0 * (1 - cos(PI2 / (home_pos_time * 2)*(double) (ctc_cnt) * dt));

        kp_roll = target_kp_roll / 2.0 * (1 - cos(PI2 / (home_pos_time * 2)*(double) (ctc_cnt) * dt));
        kd_roll = target_kd_roll / 2.0 * (1 - cos(PI2 / (home_pos_time * 2)*(double) (ctc_cnt) * dt));

        ctc_cnt++;

        if (ctc_cnt == (unsigned int) (home_pos_time / dt)) {
            walk_ready_moving_done_flag = true;
            CP_moving_flag = false;
            CP_init_flag = true;
            CP_moving_start_flag = true;
            cout << "!! Walk Ready Done !!" << endl;

            moving_done_flag = true;

            FC_PHASE = STOP;
        }

    }

    else {
        for (unsigned int i = 0; i < 12; ++i) {
            target_EP[i] = goal_EP[i];
            target_EP_vel[i] = 0;
            target_EP_acc[i] = 0;
//            target_EP[i] = init_EP[i];
//            target_EP_vel[i] = 0;
//            target_EP_acc[i] = 0;
        }
        // waist
        target_pos[6] = 0;

        if (CP_con_onoff_flag == true) {
            CP_Con();
        }
    }
}

void CRobot::Pronk_Jump(void)
{
    // =============== Pronk Initialize =============== //

    const double jump_ready_height = com_height;

    const double jump_ready_time = 1.0;
    const int jump_ready_cnt = 1000;
    const double jump_stance_time = 0.300;
    const int jump_stance_cnt = 300;
    const double jump_flight_time = 0.20;
    const int jump_flight_cnt = 200;
    const double jump_landing_time = 0.3;
    const int jump_landing_cnt = 300;

    // =============== Pronk Initialize END =============== //

    const double tmp_t1 = jump_ready_cnt;
    const double tmp_t2 = tmp_t1 + jump_stance_cnt;
    const double tmp_t3 = tmp_t2 + jump_flight_cnt;
    const double tmp_t4 = tmp_t3 + jump_landing_cnt;
    const double tmp_t5 = tmp_t4 + jump_ready_cnt;
    static int tmp_jump_cnt = 0;
    static double tmp_jump_time = 0;

    static double tmp_t[4];
    const double tmp_flight_time = 0.10;
    const int tmp_flight_cnt = 100;

    if (moving_cnt < tmp_t1) {
        FC_PHASE = STOP;
        JUMP_PHASE = 1;

        if (moving_cnt == 0) {
            first_jump_flag = true;
            moving_done_flag = false;
            global_jump_flight_cnt = jump_flight_cnt;
        }
    }
    else if (moving_cnt < tmp_t2) {
        FC_PHASE = STOP;
        JUMP_PHASE = 2;
    }
    else if (moving_cnt < tmp_t3) {
        FC_PHASE = ZERO;
        JUMP_PHASE = 3;
    }
    else if (moving_cnt < tmp_t4) {
        FC_PHASE = STOP;
        JUMP_PHASE = 4;
    }
    else if (moving_cnt < tmp_t5) {
        FC_PHASE = STOP;
        JUMP_PHASE = 5;
    }
    else {
        FC_PHASE = STOP;
        JUMP_PHASE = 6;

        if (moving_cnt == tmp_t5) {
            moving_done_flag = true;
        }
    }

    switch (JUMP_PHASE) {
    case 1: // Jump Ready
        tmp_jump_cnt = moving_cnt;
        tmp_jump_time = (double) tmp_jump_cnt*dt;
        com_acc[2] = 0;

        if (tmp_jump_cnt == 0) {
            RL_foot_pos = init_RL_foot_pos;
            RR_foot_pos = init_RR_foot_pos;
            FL_foot_pos = init_FL_foot_pos;
            FR_foot_pos = init_FR_foot_pos;

            com_pos = init_com_pos;


            tmp_t[0] = jump_stance_time;
            tmp_t[1] = jump_flight_time; //jump_flight_time;
            tmp_t[2] = tmp_flight_time;
            tmp_t[3] = jump_landing_time;

            Jump_COM_Z_Traj_Gen(jump_ready_height, tmp_t);
        }
        else {
            com_pos[2] = init_com_pos[2] + (jump_ready_height - init_com_pos[2]) / 2.0 * (1 - cos(PI2 / (jump_ready_time * 2) * tmp_jump_time));
        }
        break;

    case 2: // Take-off
        tmp_jump_cnt = moving_cnt - tmp_t1;
        tmp_jump_time = (double) tmp_jump_cnt*dt;

        RL_foot_pos = init_RL_foot_pos;
        RR_foot_pos = init_RR_foot_pos;
        FL_foot_pos = init_FL_foot_pos;
        FR_foot_pos = init_FR_foot_pos;

        com_pos = init_com_pos;

        if (first_jump_flag == true) {
            com_pos[2] = jump_z1[5] * pow(tmp_jump_time, 5) + jump_z1[4] * pow(tmp_jump_time, 4) + jump_z1[3] * pow(tmp_jump_time, 3) + jump_z1[2] * pow(tmp_jump_time, 2) + jump_z1[1] * pow(tmp_jump_time, 1) + jump_z1[0];
            com_acc[2] = 20 * jump_z1[5] * pow(tmp_jump_time, 3) + 12 * jump_z1[4] * pow(tmp_jump_time, 2) + 6 * jump_z1[3] * pow(tmp_jump_time, 1) + 2 * jump_z1[2];

            if (tmp_jump_cnt == jump_stance_cnt - 1) {
                first_jump_flag = false;
            }
        }
        else {
            com_pos[2] = jump_z1[5] * pow(tmp_jump_time, 5) + jump_z1[4] * pow(tmp_jump_time, 4) + jump_z1[3] * pow(tmp_jump_time, 3) + jump_z1[2] * pow(tmp_jump_time, 2) + jump_z1[1] * pow(tmp_jump_time, 1) + jump_z1[0];
            com_acc[2] = 20 * jump_z1[5] * pow(tmp_jump_time, 3) + 12 * jump_z1[4] * pow(tmp_jump_time, 2) + 6 * jump_z1[3] * pow(tmp_jump_time, 1) + 2 * jump_z1[2];
        }
        break;

    case 3: // Flight
        tmp_jump_cnt = moving_cnt - tmp_t2;
        tmp_jump_time = (double) tmp_jump_cnt*dt;

        RL_foot_pos = init_RL_foot_pos;
        RR_foot_pos = init_RR_foot_pos;
        FL_foot_pos = init_FL_foot_pos;
        FR_foot_pos = init_FR_foot_pos;

        if (tmp_jump_cnt < tmp_flight_cnt) {
            com_pos[2] = jump_z2[5] * pow(tmp_jump_time, 5) + jump_z2[4] * pow(tmp_jump_time, 4) + jump_z2[3] * pow(tmp_jump_time, 3) + jump_z2[2] * pow(tmp_jump_time, 2) + jump_z2[1] * pow(tmp_jump_time, 1) + jump_z2[0];
        }
        else {
            com_pos[2] = jump_z5[5] * pow(tmp_jump_time - tmp_flight_time, 5) + jump_z5[4] * pow(tmp_jump_time - tmp_flight_time, 4) + jump_z5[3] * pow(tmp_jump_time - tmp_flight_time, 3) + jump_z5[2] * pow(tmp_jump_time - tmp_flight_time, 2) + jump_z5[1] * pow(tmp_jump_time - tmp_flight_time, 1) + jump_z5[0];
        }

        com_acc[2] = 0;
        break;

    case 4: // Landing
        tmp_jump_cnt = moving_cnt - tmp_t3;
        tmp_jump_time = (double) tmp_jump_cnt*dt;

        RL_foot_pos = init_RL_foot_pos;
        RR_foot_pos = init_RR_foot_pos;
        FL_foot_pos = init_FL_foot_pos;
        FR_foot_pos = init_FR_foot_pos;

        com_pos = init_com_pos;

        com_pos[2] = jump_z4[5] * pow(tmp_jump_time, 5) + jump_z4[4] * pow(tmp_jump_time, 4) + jump_z4[3] * pow(tmp_jump_time, 3) + jump_z4[2] * pow(tmp_jump_time, 2) + jump_z4[1] * pow(tmp_jump_time, 1) + jump_z4[0];
        com_acc[2] = 20 * jump_z4[5] * pow(tmp_jump_time, 3) + 12 * jump_z4[4] * pow(tmp_jump_time, 2) + 6 * jump_z4[3] * pow(tmp_jump_time, 1) + 2 * jump_z4[2];
        break;


    case 5: // Walk Ready
        tmp_jump_cnt = moving_cnt - tmp_t4;
        tmp_jump_time = (double) tmp_jump_cnt*dt;

        RL_foot_pos = init_RL_foot_pos;
        RR_foot_pos = init_RR_foot_pos;
        FL_foot_pos = init_FL_foot_pos;
        FR_foot_pos = init_FR_foot_pos;

        com_pos = init_com_pos;

        com_pos[2] = jump_ready_height + (init_com_pos[2] - jump_ready_height) / 2.0 * (1 - cos(PI2 / (jump_ready_time * 2)*(double) (tmp_jump_cnt) * dt));
        com_acc[2] = 0;

        break;
    }
    moving_cnt++;

    if (moving_cnt == tmp_t4) {
        if (sub_ctrl_flag == false) { //jump_stop_flag

            if (T_RL == true || T_RR == true || T_FL == true || T_FR == true) {
                moving_cnt = tmp_t1;

                jump_num++;

                printf("jump_num = %d\n", jump_num);
            }
            else {
                printf("Not land\n");
                moving_cnt--;
            }
        }
    }

    // ============================ target_EP ========================== //
    // foot position from global to local
    local_RL_foot_pos = RL_foot_pos - com_pos;
    local_RR_foot_pos = RR_foot_pos - com_pos;
    local_FL_foot_pos = FL_foot_pos - com_pos;
    local_FR_foot_pos = FR_foot_pos - com_pos;

    if (CP_con_onoff_flag == true) {
        if (moving_cnt > tmp_t5 + 1000) {
            CP_Con();
        }
    }
    else {
        cp_RL_foot_pos << 0, 0, 0;
        cp_RR_foot_pos << 0, 0, 0;
        cp_FL_foot_pos << 0, 0, 0;
        cp_FR_foot_pos << 0, 0, 0;
    }

    target_EP[0] = local_RL_foot_pos[0] + cp_RL_foot_pos[0];
    target_EP[1] = local_RL_foot_pos[1] + cp_RL_foot_pos[1];
    target_EP[2] = local_RL_foot_pos[2] + cp_RL_foot_pos[2];
    target_EP[3] = local_RR_foot_pos[0] + cp_RR_foot_pos[0];
    target_EP[4] = local_RR_foot_pos[1] + cp_RR_foot_pos[1];
    target_EP[5] = local_RR_foot_pos[2] + cp_RR_foot_pos[2];
    target_EP[6] = local_FL_foot_pos[0] + cp_FL_foot_pos[0];
    target_EP[7] = local_FL_foot_pos[1] + cp_FL_foot_pos[1];
    target_EP[8] = local_FL_foot_pos[2] + cp_FL_foot_pos[2];
    target_EP[9] = local_FR_foot_pos[0] + cp_FR_foot_pos[0];
    target_EP[10] = local_FR_foot_pos[1] + cp_FR_foot_pos[1];
    target_EP[11] = local_FR_foot_pos[2] + cp_FR_foot_pos[2];

    ctc_cnt2++;

    target_pos[6] = 0; //goal_pos[6];
}

void CRobot::Jump_COM_Z_Traj_Gen(double h0, double t[4])
{
    const double jump_h_0 = h0;
    const double jump_v_0 = 0;
    const double jump_a_0 = 0;

    const double jump_v_1 = 0.6; //1.2;
    const double jump_a_1 = -GRAVITY;

    const double jump_h_2 = jump_h_0;
    const double jump_v_2 = -0.10; //-0.05;
    const double jump_a_2 = -GRAVITY;

    const double jump_h_3 = jump_h_0;
    const double jump_v_3 = 0;
    const double jump_a_3 = 0;

    const double jump_h_1 = 0.5 * GRAVITY * t[1] * t[1] - jump_v_1 * t[1] + jump_h_2;

    // =============== Stance phase (First) =============== //
    init_x[0] = jump_h_0;
    init_x[1] = jump_v_0;
    init_x[2] = jump_a_0;

    final_x[0] = jump_h_1;
    final_x[1] = jump_v_1;
    final_x[2] = jump_a_1;

    coefficient_5thPoly(init_x, final_x, t[0], jump_z1);

    // =============== Flight phase (First) =============== //
    init_x[0] = jump_h_1;
    init_x[1] = jump_v_1;
    init_x[2] = jump_a_1;

    final_x[0] = jump_h_0;
    final_x[1] = jump_v_0;
    final_x[2] = jump_a_0;

    coefficient_5thPoly(init_x, final_x, t[2], jump_z2);

    // =============== Flight phase (Second) =============== //
    init_x[0] = jump_h_0;
    init_x[1] = jump_v_0;
    init_x[2] = jump_a_0;

    final_x[0] = jump_h_2;
    final_x[1] = jump_v_2;
    final_x[2] = jump_a_2;

    coefficient_5thPoly(init_x, final_x, t[1] - t[2], jump_z5);

    // =============== Stance phase (Second) =============== //
    init_x[0] = jump_h_2;
    init_x[1] = jump_v_2;
    init_x[2] = jump_a_2;

    final_x[0] = jump_h_1;
    final_x[1] = jump_v_1;
    final_x[2] = jump_a_1;

    coefficient_5thPoly(init_x, final_x, t[0], jump_z3);

    // =============== Landing phase =============== //
    init_x[0] = jump_h_2;
    init_x[1] = jump_v_2;
    init_x[2] = jump_a_2;

    final_x[0] = jump_h_3;
    final_x[1] = jump_v_3;
    final_x[2] = jump_a_3;

    coefficient_5thPoly(init_x, final_x, t[3], jump_z4);
}

void CRobot::CP_Con(void)
{
    if (walk_ready_moving_done_flag == true) {

        if (CP_init_flag == true) {

            cout << "CP init!!" << endl;

            for (unsigned int i = 0; i < 12; ++i) {
                init_EP[i] = target_EP[i];
            }

            CP_init_flag = false;
        }

        //        cout << "[1] CP_y = " << CP_y << endl;

        if ((CP_y > cp_y_limit || CP_y < -cp_y_limit) && CP_moving_start_flag == true) {

            cout << "CP_y = " << CP_y << endl;

            CP_moving_flag = true;
            CP_moving_start_flag = false;
        }

        if (CP_moving_flag == true) {
            CP_foot_traj_gen();
        }

        target_EP[0] = init_EP[0] + cp_RL_foot_pos[0];
        target_EP[1] = init_EP[1] + cp_RL_foot_pos[1];
        target_EP[2] = init_EP[2] + cp_RL_foot_pos[2];
        target_EP[3] = init_EP[3] + cp_RR_foot_pos[0];
        target_EP[4] = init_EP[4] + cp_RR_foot_pos[1];
        target_EP[5] = init_EP[5] + cp_RR_foot_pos[2];
        target_EP[6] = init_EP[6] + cp_FL_foot_pos[0];
        target_EP[7] = init_EP[7] + cp_FL_foot_pos[1];
        target_EP[8] = init_EP[8] + cp_FL_foot_pos[2];
        target_EP[9] = init_EP[9] + cp_FR_foot_pos[0];
        target_EP[10] = init_EP[10] + cp_FR_foot_pos[1];
        target_EP[11] = init_EP[11] + cp_FR_foot_pos[2];
    }
}

void CRobot::CP_foot_traj_gen(void)
{
    static unsigned int cp_cnt = 0;
    static double cp_foot_height = 0.05;
    //    static double cp_foot_pos_y = 0;
    //    static double cp_limit_y = 0.15;

    if (cp_cnt == 0) {
        if (cp_foot_pos_y > 0) {
            FC_PHASE = STANCE_RRFL;
        }
        else {
            FC_PHASE = STANCE_RLFR;
        }

        cp_RL_foot_pos = init_cp_RL_foot_pos;
        cp_RR_foot_pos = init_cp_RR_foot_pos;
        cp_FL_foot_pos = init_cp_FL_foot_pos;
        cp_FR_foot_pos = init_cp_FR_foot_pos;

        if (CP_y > cp_y_limit) {
            cp_foot_pos_y = cp_y_limit;
        }
        else if (CP_y < -cp_y_limit) {
            cp_foot_pos_y = -cp_y_limit;
        }
        else {
            cp_foot_pos_y = CP_y;
        }
        cp_foot_pos_y = cp_foot_pos_y * 0.5; //1.0;

        init_CP_y = CP_y;

        //        printf("[0] init_CP_y = %f\n",init_CP_y);

        cout << "<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< cp_foot_pos_y = " << cp_foot_pos_y << ">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>" << endl;

        cp_cnt++;
    }
    else if (cp_cnt <= dsp_cnt) {
        if (cp_foot_pos_y > 0) {
            FC_PHASE = STANCE_RRFL;

            cp_foot_offset_y = Kp_cp * (CP_y - init_CP_y) / 2.0;

            //            printf("[1] cp_foot_offset_y = %f\n", cp_foot_offset_y);

            cp_RL_foot_pos[0] = init_cp_RL_foot_pos[0];
            cp_RL_foot_pos[1] = init_cp_RL_foot_pos[1] + (cp_foot_pos_y) / 2.0 * (1 - cos(PI2 / (dsp_time * 2)*(double) (cp_cnt) * dt)) + cp_foot_offset_y;
            cp_RL_foot_pos[2] = init_cp_RL_foot_pos[2] + (cp_foot_height) / 2.0 * (1 - cos(PI2 / (dsp_time)*(double) (cp_cnt) * dt));

            cp_RR_foot_pos[0] = init_cp_RR_foot_pos[0];
            cp_RR_foot_pos[1] = init_cp_RR_foot_pos[1] - (cp_foot_pos_y) / 2.0 * (1 - cos(PI2 / (dsp_time * 2)*(double) (cp_cnt) * dt)) - cp_foot_offset_y;
            cp_RR_foot_pos[2] = init_cp_RR_foot_pos[2];

            cp_FL_foot_pos[0] = init_cp_FL_foot_pos[0];
            cp_FL_foot_pos[1] = init_cp_FL_foot_pos[1] - (cp_foot_pos_y) / 2.0 * (1 - cos(PI2 / (dsp_time * 2)*(double) (cp_cnt) * dt)) - cp_foot_offset_y;
            cp_FL_foot_pos[2] = init_cp_FL_foot_pos[2];

            cp_FR_foot_pos[0] = init_cp_FR_foot_pos[0];
            cp_FR_foot_pos[1] = init_cp_FR_foot_pos[1] + (cp_foot_pos_y) / 2.0 * (1 - cos(PI2 / (dsp_time * 2)*(double) (cp_cnt) * dt)) + cp_foot_offset_y;
            cp_FR_foot_pos[2] = init_cp_FR_foot_pos[2] + (cp_foot_height) / 2.0 * (1 - cos(PI2 / (dsp_time)*(double) (cp_cnt) * dt));

        }
        else {
            FC_PHASE = STANCE_RLFR;

            cp_foot_offset_y = Kp_cp * (CP_y - init_CP_y) / 2.0;

            //            printf("[LEFT] cp_foot_offset_y = %f\n", cp_foot_offset_y);

            cp_RL_foot_pos[0] = init_cp_RL_foot_pos[0];
            cp_RL_foot_pos[1] = init_cp_RL_foot_pos[1] - (cp_foot_pos_y) / 2.0 * (1 - cos(PI2 / (dsp_time * 2)*(double) (cp_cnt) * dt)) - cp_foot_offset_y;
            cp_RL_foot_pos[2] = init_cp_RL_foot_pos[2];

            cp_RR_foot_pos[0] = init_cp_RR_foot_pos[0];
            cp_RR_foot_pos[1] = init_cp_RR_foot_pos[1] + (cp_foot_pos_y) / 2.0 * (1 - cos(PI2 / (dsp_time * 2)*(double) (cp_cnt) * dt)) + cp_foot_offset_y;
            cp_RR_foot_pos[2] = init_cp_RR_foot_pos[2] + (cp_foot_height) / 2.0 * (1 - cos(PI2 / (dsp_time)*(double) (cp_cnt) * dt));

            cp_FL_foot_pos[0] = init_cp_FL_foot_pos[0];
            cp_FL_foot_pos[1] = init_cp_FL_foot_pos[1] + (cp_foot_pos_y) / 2.0 * (1 - cos(PI2 / (dsp_time * 2)*(double) (cp_cnt) * dt)) + cp_foot_offset_y;
            cp_FL_foot_pos[2] = init_cp_FL_foot_pos[2] + (cp_foot_height) / 2.0 * (1 - cos(PI2 / (dsp_time)*(double) (cp_cnt) * dt));

            cp_FR_foot_pos[0] = init_cp_FR_foot_pos[0];
            cp_FR_foot_pos[1] = init_cp_FR_foot_pos[1] - (cp_foot_pos_y) / 2.0 * (1 - cos(PI2 / (dsp_time * 2)*(double) (cp_cnt) * dt)) - cp_foot_offset_y;
            cp_FR_foot_pos[2] = init_cp_FR_foot_pos[2];

        }

        cp_cnt++;
    }
    else if (cp_cnt <= step_cnt) {

        FC_PHASE = STOP;

        if (cp_foot_pos_y > 0) {
            cp_RL_foot_pos[0] = init_cp_RL_foot_pos[0];
            cp_RL_foot_pos[1] = init_cp_RL_foot_pos[1] + cp_foot_pos_y + cp_foot_offset_y;
            cp_RL_foot_pos[2] = init_cp_RL_foot_pos[2];

            cp_RR_foot_pos[0] = init_cp_RR_foot_pos[0];
            cp_RR_foot_pos[1] = init_cp_RR_foot_pos[1] - cp_foot_pos_y - cp_foot_offset_y;
            cp_RR_foot_pos[2] = init_cp_RR_foot_pos[2];

            cp_FL_foot_pos[0] = init_cp_FL_foot_pos[0];
            cp_FL_foot_pos[1] = init_cp_FL_foot_pos[1] - cp_foot_pos_y - cp_foot_offset_y;
            cp_FL_foot_pos[2] = init_cp_FL_foot_pos[2];

            cp_FR_foot_pos[0] = init_cp_FR_foot_pos[0];
            cp_FR_foot_pos[1] = init_cp_FR_foot_pos[1] + cp_foot_pos_y + cp_foot_offset_y;
            cp_FR_foot_pos[2] = init_cp_FR_foot_pos[2];
        }
        else {
            cp_RL_foot_pos[0] = init_cp_RL_foot_pos[0];
            cp_RL_foot_pos[1] = init_cp_RL_foot_pos[1] - cp_foot_pos_y - cp_foot_offset_y;
            cp_RL_foot_pos[2] = init_cp_RL_foot_pos[2];

            cp_RR_foot_pos[0] = init_cp_RR_foot_pos[0];
            cp_RR_foot_pos[1] = init_cp_RR_foot_pos[1] + cp_foot_pos_y + cp_foot_offset_y;
            cp_RR_foot_pos[2] = init_cp_RR_foot_pos[2];

            cp_FL_foot_pos[0] = init_cp_FL_foot_pos[0];
            cp_FL_foot_pos[1] = init_cp_FL_foot_pos[1] + cp_foot_pos_y + cp_foot_offset_y;
            cp_FL_foot_pos[2] = init_cp_FL_foot_pos[2];

            cp_FR_foot_pos[0] = init_cp_FR_foot_pos[0];
            cp_FR_foot_pos[1] = init_cp_FR_foot_pos[1] - cp_foot_pos_y - cp_foot_offset_y;
            cp_FR_foot_pos[2] = init_cp_FR_foot_pos[2];
        }
        cp_cnt++;
    }
    else if (cp_cnt <= step_cnt + dsp_cnt) {

        if (cp_foot_pos_y > 0) {
            FC_PHASE = STANCE_RLFR;

            cp_RL_foot_pos[0] = init_cp_RL_foot_pos[0];
            cp_RL_foot_pos[1] = init_cp_RL_foot_pos[1] + cp_foot_pos_y + cp_foot_offset_y - (cp_foot_pos_y + cp_foot_offset_y) / 2.0 * (1 - cos(PI2 / (dsp_time * 2)*(double) (cp_cnt - step_cnt) * dt));
            cp_RL_foot_pos[2] = init_cp_RL_foot_pos[2];

            cp_RR_foot_pos[0] = init_cp_RR_foot_pos[0];
            cp_RR_foot_pos[1] = init_cp_RR_foot_pos[1] - cp_foot_pos_y - cp_foot_offset_y + (cp_foot_pos_y + cp_foot_offset_y) / 2.0 * (1 - cos(PI2 / (dsp_time * 2)*(double) (cp_cnt - step_cnt) * dt));
            cp_RR_foot_pos[2] = init_cp_RR_foot_pos[2] + (cp_foot_height) / 2.0 * (1 - cos(PI2 / (dsp_time)*(double) (cp_cnt - step_cnt) * dt));

            cp_FL_foot_pos[0] = init_cp_FL_foot_pos[0];
            cp_FL_foot_pos[1] = init_cp_FL_foot_pos[1] - cp_foot_pos_y - cp_foot_offset_y + (cp_foot_pos_y + cp_foot_offset_y) / 2.0 * (1 - cos(PI2 / (dsp_time * 2)*(double) (cp_cnt - step_cnt) * dt));
            cp_FL_foot_pos[2] = init_cp_FL_foot_pos[2] + (cp_foot_height) / 2.0 * (1 - cos(PI2 / (dsp_time)*(double) (cp_cnt - step_cnt) * dt));

            cp_FR_foot_pos[0] = init_cp_FR_foot_pos[0];
            cp_FR_foot_pos[1] = init_cp_FR_foot_pos[1] + cp_foot_pos_y + cp_foot_offset_y - (cp_foot_pos_y + cp_foot_offset_y) / 2.0 * (1 - cos(PI2 / (dsp_time * 2)*(double) (cp_cnt - step_cnt) * dt));
            cp_FR_foot_pos[2] = init_cp_FR_foot_pos[2];

        }
        else {
            FC_PHASE = STANCE_RRFL;

            cp_RL_foot_pos[0] = init_cp_RL_foot_pos[0];
            cp_RL_foot_pos[1] = init_cp_RL_foot_pos[1] - cp_foot_pos_y - cp_foot_offset_y + (cp_foot_pos_y + cp_foot_offset_y) / 2.0 * (1 - cos(PI2 / (dsp_time * 2)*(double) (cp_cnt - step_cnt) * dt));
            cp_RL_foot_pos[2] = init_cp_RL_foot_pos[2] + (cp_foot_height) / 2.0 * (1 - cos(PI2 / (dsp_time)*(double) (cp_cnt - step_cnt) * dt));

            cp_RR_foot_pos[0] = init_cp_RR_foot_pos[0];
            cp_RR_foot_pos[1] = init_cp_RR_foot_pos[1] + cp_foot_pos_y + cp_foot_offset_y - (cp_foot_pos_y + cp_foot_offset_y) / 2.0 * (1 - cos(PI2 / (dsp_time * 2)*(double) (cp_cnt - step_cnt) * dt));
            cp_RR_foot_pos[2] = init_cp_RR_foot_pos[2];

            cp_FL_foot_pos[0] = init_cp_FL_foot_pos[0];
            cp_FL_foot_pos[1] = init_cp_FL_foot_pos[1] + cp_foot_pos_y + cp_foot_offset_y - (cp_foot_pos_y + cp_foot_offset_y) / 2.0 * (1 - cos(PI2 / (dsp_time * 2)*(double) (cp_cnt - step_cnt) * dt));
            cp_FL_foot_pos[2] = init_cp_FL_foot_pos[2];

            cp_FR_foot_pos[0] = init_cp_FR_foot_pos[0];
            cp_FR_foot_pos[1] = init_cp_FR_foot_pos[1] - cp_foot_pos_y - cp_foot_offset_y + (cp_foot_pos_y + cp_foot_offset_y) / 2.0 * (1 - cos(PI2 / (dsp_time * 2)*(double) (cp_cnt - step_cnt) * dt));
            cp_FR_foot_pos[2] = init_cp_FR_foot_pos[2] + (cp_foot_height) / 2.0 * (1 - cos(PI2 / (dsp_time)*(double) (cp_cnt - step_cnt) * dt));

        }
        cp_cnt++;
    }
    else if (cp_cnt <= step_cnt * 2) {
        FC_PHASE = STOP;

        cp_RL_foot_pos[0] = init_cp_RL_foot_pos[0];
        cp_RL_foot_pos[1] = init_cp_RL_foot_pos[1];
        cp_RL_foot_pos[2] = init_cp_RL_foot_pos[2];

        cp_RR_foot_pos[0] = init_cp_RR_foot_pos[0];
        cp_RR_foot_pos[1] = init_cp_RR_foot_pos[1];
        cp_RR_foot_pos[2] = init_cp_RR_foot_pos[2];

        cp_FL_foot_pos[0] = init_cp_FL_foot_pos[0];
        cp_FL_foot_pos[1] = init_cp_FL_foot_pos[1];
        cp_FL_foot_pos[2] = init_cp_FL_foot_pos[2];

        cp_FR_foot_pos[0] = init_cp_FR_foot_pos[0];
        cp_FR_foot_pos[1] = init_cp_FR_foot_pos[1];
        cp_FR_foot_pos[2] = init_cp_FR_foot_pos[2];

        cp_cnt++;
    }

    else if (cp_cnt <= step_cnt * 4) { //12

        FC_PHASE = STOP;

        cp_RL_foot_pos[0] = init_cp_RL_foot_pos[0];
        cp_RL_foot_pos[1] = init_cp_RL_foot_pos[1];
        cp_RL_foot_pos[2] = init_cp_RL_foot_pos[2];

        cp_RR_foot_pos[0] = init_cp_RR_foot_pos[0];
        cp_RR_foot_pos[1] = init_cp_RR_foot_pos[1];
        cp_RR_foot_pos[2] = init_cp_RR_foot_pos[2];

        cp_FL_foot_pos[0] = init_cp_FL_foot_pos[0];
        cp_FL_foot_pos[1] = init_cp_FL_foot_pos[1];
        cp_FL_foot_pos[2] = init_cp_FL_foot_pos[2];

        cp_FR_foot_pos[0] = init_cp_FR_foot_pos[0];
        cp_FR_foot_pos[1] = init_cp_FR_foot_pos[1];
        cp_FR_foot_pos[2] = init_cp_FR_foot_pos[2];

        cp_cnt++;
    }
    else {
        FC_PHASE = STOP;
        //        CP_moving_flag = false;
        cp_RL_foot_pos[0] = init_cp_RL_foot_pos[0];
        cp_RL_foot_pos[1] = init_cp_RL_foot_pos[1];
        cp_RL_foot_pos[2] = init_cp_RL_foot_pos[2];

        cp_RR_foot_pos[0] = init_cp_RR_foot_pos[0];
        cp_RR_foot_pos[1] = init_cp_RR_foot_pos[1];
        cp_RR_foot_pos[2] = init_cp_RR_foot_pos[2];

        cp_FL_foot_pos[0] = init_cp_FL_foot_pos[0];
        cp_FL_foot_pos[1] = init_cp_FL_foot_pos[1];
        cp_FL_foot_pos[2] = init_cp_FL_foot_pos[2];

        cp_FR_foot_pos[0] = init_cp_FR_foot_pos[0];
        cp_FR_foot_pos[1] = init_cp_FR_foot_pos[1];
        cp_FR_foot_pos[2] = init_cp_FR_foot_pos[2];

        cp_cnt = 0;

        if (CP_y < cp_y_limit && CP_y > -cp_y_limit) {

            cp_foot_pos_y = 0;
            CP_moving_flag = false;
            CP_moving_start_flag = true;
            cout << "CP CON DONE!~~~~~~~~~~~~~~" << endl;
        }
    }
}


















// ====================== flying trot trajectory generation ===================== //

void CRobot::Flying_Trot_Running(void)
{
    //    ft_time2 = (double) ft_cnt2*dt; // for graph
    //    ft_cnt2++;

    if (ft_ready_flag == true) {
        //        cout << "[1] FT ready !" << endl;
        //        gain up!!
        FC_PHASE = STOP;

        com_acc[2] = 0;

        com_pos = init_com_pos;
        RL_foot_pos = init_RL_foot_pos;
        RR_foot_pos = init_RR_foot_pos;
        FL_foot_pos = init_FL_foot_pos;
        FR_foot_pos = init_FR_foot_pos;

        if (ft_ready_cnt <= 1000) {
            for (unsigned int i = 0; i < 13; ++i) {
                Kp_q[i] = init_Kp_q[i] + (FT_Kp_q[i] - init_Kp_q[i]) / 2.0 * (1 - cos(PI2 / (2 * 1.0)*(double) ft_ready_cnt * dt));
                Kd_q[i] = init_Kd_q[i] + (FT_Kd_q[i] - init_Kd_q[i]) / 2.0 * (1 - cos(PI2 / (2 * 1.0)*(double) ft_ready_cnt * dt));
            }
        }
        else {
            ft_ready_flag = false;
        }

        if (CP_con_onoff_flag == true) {
            CP_Con();
        }

        ft_ready_cnt++;
    }
    else if (ft_ready_flag == false && ft_finish_flag == false) {
        //        cout << "[2] FT Working !" << endl;
        FT_Traj_Gen();

        if (CP_con_onoff_flag == true) {

            if (ft_cnt < 4 * ft_step_cnt + 1) {
                CP_Con_FT();
            }
            else if (ft_cnt < 5 * ft_step_cnt + 1) {
                if (ft_cnt == 4 * ft_step_cnt + 1) {
                    tmp_cp_RL_foot_pos = cp_RL_foot_pos;
                    tmp_cp_RR_foot_pos = cp_RR_foot_pos;
                    tmp_cp_FL_foot_pos = cp_FL_foot_pos;
                    tmp_cp_FR_foot_pos = cp_FR_foot_pos;
                }
                else {
                    for (unsigned int i = 0; i < 3; i++) {
                        tmp_t = ft_time - 4 * ft_step_time;
                        cp_RL_foot_pos[i] = tmp_cp_RL_foot_pos[i] + (init_cp_RL_foot_pos[i] - tmp_cp_RL_foot_pos[i]) / 2.0 * (1 - cos(PI2 / (ft_step_time * 2) * tmp_t));
                        cp_RR_foot_pos[i] = tmp_cp_RR_foot_pos[i] + (init_cp_RR_foot_pos[i] - tmp_cp_RR_foot_pos[i]) / 2.0 * (1 - cos(PI2 / (ft_step_time * 2) * tmp_t));
                        cp_FL_foot_pos[i] = tmp_cp_FL_foot_pos[i] + (init_cp_FL_foot_pos[i] - tmp_cp_FL_foot_pos[i]) / 2.0 * (1 - cos(PI2 / (ft_step_time * 2) * tmp_t));
                        cp_FR_foot_pos[i] = tmp_cp_FR_foot_pos[i] + (init_cp_FR_foot_pos[i] - tmp_cp_FR_foot_pos[i]) / 2.0 * (1 - cos(PI2 / (ft_step_time * 2) * tmp_t));
                    }
                }
            }
            else {
                ft_finish_flag = true;

                if (CP_con_onoff_flag == true) {
                    CP_Con();
                }
            }
        }
        else {
            cp_RL_foot_pos << 0, 0, 0;
            cp_RR_foot_pos << 0, 0, 0;
            cp_FL_foot_pos << 0, 0, 0;
            cp_FR_foot_pos << 0, 0, 0;
        }

        FT_Turning_Traj_Gen();

        ft_cnt++;
    }
    else if (ft_finish_flag == true) {
        //        cout << "[3] Finish !" << endl;

        com_acc[2] = 0;

        com_pos = init_com_pos;
        RL_foot_pos = init_RL_foot_pos;
        RR_foot_pos = init_RR_foot_pos;
        FL_foot_pos = init_FL_foot_pos;
        FR_foot_pos = init_FR_foot_pos;

        if (ft_finish_cnt <= 1000) {
            for (unsigned int i = 0; i < 13; ++i) {
                Kp_q[i] = FT_Kp_q[i] + (init_Kp_q[i] - FT_Kp_q[i]) / 2.0 * (1 - cos(PI2 / (2 * 1.0)*(double) ft_finish_cnt * dt));
                Kd_q[i] = FT_Kd_q[i] + (init_Kd_q[i] - FT_Kd_q[i]) / 2.0 * (1 - cos(PI2 / (2 * 1.0)*(double) ft_finish_cnt * dt));
            }
        }
        if (CP_con_onoff_flag == true) {
            CP_Con();
        }

        ft_finish_cnt++;

    }

    //    printf("Kp_q[0] = %f\n",Kp_q[0]);

    // ============================ target_EP ========================== //

    local_RL_foot_pos = RL_foot_pos - com_pos;
    local_RR_foot_pos = RR_foot_pos - com_pos;
    local_FL_foot_pos = FL_foot_pos - com_pos;
    local_FR_foot_pos = FR_foot_pos - com_pos;

    target_EP[0] = local_RL_foot_pos[0] - turn_xr_EP + cp_RL_foot_pos[0];
    target_EP[1] = local_RL_foot_pos[1] - turn_yr_EP + cp_RL_foot_pos[1];
    target_EP[2] = local_RL_foot_pos[2] + cp_RL_foot_pos[2];
    target_EP[3] = local_RR_foot_pos[0] - turn_xl_EP + cp_RR_foot_pos[0];
    target_EP[4] = local_RR_foot_pos[1] - turn_yl_EP + cp_RR_foot_pos[1];
    target_EP[5] = local_RR_foot_pos[2] + cp_RR_foot_pos[2];
    target_EP[6] = local_FL_foot_pos[0] + turn_xl_EP + cp_FL_foot_pos[0];
    target_EP[7] = local_FL_foot_pos[1] + turn_yl_EP + cp_FL_foot_pos[1];
    target_EP[8] = local_FL_foot_pos[2] + cp_FL_foot_pos[2];
    target_EP[9] = local_FR_foot_pos[0] + turn_xr_EP + cp_FR_foot_pos[0];
    target_EP[10] = local_FR_foot_pos[1] + turn_yr_EP + cp_FR_foot_pos[1];
    target_EP[11] = local_FR_foot_pos[2] + cp_FR_foot_pos[2];



    target_pos[6] = 0; //goal_pos[6];

    //    printf("FC_PHASE = %d, ft_cnt = %d, x_moving_speed = %f\n",FC_PHASE,ft_cnt,x_moving_speed);

}

void CRobot::FT_Traj_Gen(void)
{
    ft_time = (double) ft_cnt*dt;

    if (ft_cnt == 0) {
        // ============ Initialize ============ //
        FC_PHASE = STANCE_RLFR;

        moving_done_flag = false;

        com_acc[2] = 0;

        com_pos = init_com_pos;
        RL_foot_pos = init_RL_foot_pos;
        RR_foot_pos = init_RR_foot_pos;
        FL_foot_pos = init_FL_foot_pos;
        FR_foot_pos = init_FR_foot_pos;

        old_com_pos = init_com_pos;

        x_moving_speed = 0; // initial moving speed

        flying_trot_init_flag = true;
        flying_trot_final_flag = false;

        COM_SF_FT_Z_Traj_Gen();

    }
    else if (ft_cnt < ts_cnt) {
        //        printf("[1] First Step (STANCE_RLFR)\n");
        // ============ First Step (STANCE_RLFR) ============= //
        FC_PHASE = STANCE_RLFR;
        tmp_t = ft_time;

        com_pos[0] = init_com_pos(0);
        com_pos[1] = init_com_pos(1);
        com_pos[2] = c_com_z1[5] * pow(tmp_t, 5) + c_com_z1[4] * pow(tmp_t, 4) + c_com_z1[3] * pow(tmp_t, 3) + c_com_z1[2] * pow(tmp_t, 2) + c_com_z1[1] * pow(tmp_t, 1) + c_com_z1[0];

        RL_foot_pos = init_RL_foot_pos;

        RR_foot_pos[0] = init_RR_foot_pos[0];
        RR_foot_pos[1] = init_RR_foot_pos[1];
        RR_foot_pos[2] = init_RR_foot_pos[2] + c_sf_z1[5] * pow(tmp_t, 5) + c_sf_z1[4] * pow(tmp_t, 4) + c_sf_z1[3] * pow(tmp_t, 3) + c_sf_z1[2] * pow(tmp_t, 2) + c_sf_z1[1] * pow(tmp_t, 1) + c_sf_z1[0];

        FL_foot_pos[0] = init_FL_foot_pos[0];
        FL_foot_pos[1] = init_FL_foot_pos[1];
        FL_foot_pos[2] = init_FL_foot_pos[2] + c_sf_z1[5] * pow(tmp_t, 5) + c_sf_z1[4] * pow(tmp_t, 4) + c_sf_z1[3] * pow(tmp_t, 3) + c_sf_z1[2] * pow(tmp_t, 2) + c_sf_z1[1] * pow(tmp_t, 1) + c_sf_z1[0];

        FR_foot_pos = init_FR_foot_pos;

        com_acc[2] = 20 * c_com_z1[5] * pow(tmp_t, 3) + 12 * c_com_z1[4] * pow(tmp_t, 2) + 6 * c_com_z1[3] * pow(tmp_t, 1) + 2 * c_com_z1[2];

        if (ft_cnt == ts_cnt - 1) {
            flying_trot_init_flag = false;
        }
        //        printf("RR_foot_pos[2] = %f\n",RR_foot_pos[2]);
    }
    else if (ft_cnt < ft_step_cnt) {
        //        printf("[2] First Step (FP)\n");
        // ============ First Step (FP) ============= //
        FC_PHASE = ZERO;
        tmp_t = ft_time - ts;

        com_pos[0] = init_com_pos(0);
        com_pos[1] = init_com_pos(1);
        com_pos[2] = c_com_z2[5] * pow(tmp_t, 5) + c_com_z2[4] * pow(tmp_t, 4) + c_com_z2[3] * pow(tmp_t, 3) + c_com_z2[2] * pow(tmp_t, 2) + c_com_z2[1] * pow(tmp_t, 1) + c_com_z2[0];

        RL_foot_pos[0] = init_RL_foot_pos[0];
        RL_foot_pos[1] = init_RL_foot_pos[1];
        RL_foot_pos[2] = init_RL_foot_pos[2] + c_sf_z2[5] * pow(tmp_t, 5) + c_sf_z2[4] * pow(tmp_t, 4) + c_sf_z2[3] * pow(tmp_t, 3) + c_sf_z2[2] * pow(tmp_t, 2) + c_sf_z2[1] * pow(tmp_t, 1) + c_sf_z2[0];
        //        RL_foot_pos[2] = init_RL_foot_pos[2] + swing_foot_height/2.0*(1-cos(PI2/(2*tf)*tmp_t));

        RR_foot_pos[0] = init_RR_foot_pos[0];
        RR_foot_pos[1] = init_RR_foot_pos[1];
        RR_foot_pos[2] = init_RR_foot_pos[2] + c_sf_z3[5] * pow(tmp_t, 5) + c_sf_z3[4] * pow(tmp_t, 4) + c_sf_z3[3] * pow(tmp_t, 3) + c_sf_z3[2] * pow(tmp_t, 2) + c_sf_z3[1] * pow(tmp_t, 1) + c_sf_z3[0];

        FL_foot_pos[0] = init_FL_foot_pos[0];
        FL_foot_pos[1] = init_FL_foot_pos[1];
        FL_foot_pos[2] = init_FL_foot_pos[2] + c_sf_z3[5] * pow(tmp_t, 5) + c_sf_z3[4] * pow(tmp_t, 4) + c_sf_z3[3] * pow(tmp_t, 3) + c_sf_z3[2] * pow(tmp_t, 2) + c_sf_z3[1] * pow(tmp_t, 1) + c_sf_z3[0];

        FR_foot_pos[0] = init_FR_foot_pos[0];
        FR_foot_pos[1] = init_FR_foot_pos[1];
        FR_foot_pos[2] = init_FR_foot_pos[2] + c_sf_z2[5] * pow(tmp_t, 5) + c_sf_z2[4] * pow(tmp_t, 4) + c_sf_z2[3] * pow(tmp_t, 3) + c_sf_z2[2] * pow(tmp_t, 2) + c_sf_z2[1] * pow(tmp_t, 1) + c_sf_z2[0];
        //        FR_foot_pos[2] = init_FR_foot_pos[2] + swing_foot_height/2.0*(1-cos(PI2/(2*tf)*tmp_t));

        com_acc[2] = 0;

        if (ft_cnt == ft_step_cnt - 1) {
            pre_x_moving_speed = x_moving_speed;
            x_moving_speed = tmp_x_moving_speed;

            pre_com_pos[0] = com_pos[0];
            pre_com_pos[1] = com_pos[1];
            pre_com_pos[2] = h_2;

            pre_RL_foot_pos[0] = RL_foot_pos[0];
            pre_RL_foot_pos[1] = RL_foot_pos[1];
            pre_RL_foot_pos[2] = init_RL_foot_pos[2] + swing_foot_height;

            pre_RR_foot_pos[0] = RR_foot_pos[0];
            pre_RR_foot_pos[1] = RR_foot_pos[1];
            pre_RR_foot_pos[2] = init_RR_foot_pos[2];

            pre_FL_foot_pos[0] = FL_foot_pos[0];
            pre_FL_foot_pos[1] = FL_foot_pos[1];
            pre_FL_foot_pos[2] = init_FL_foot_pos[2];

            pre_FR_foot_pos[0] = FR_foot_pos[0];
            pre_FR_foot_pos[1] = FR_foot_pos[1];
            pre_FR_foot_pos[2] = init_FR_foot_pos[2] + swing_foot_height;

            COM_SF_FT_X_Traj_Gen();
        }
    }

    else if (ft_cnt < ft_step_cnt + ts_cnt) {
        //        printf("[3] Second Step (STANCE_RRFL)\n");
        // ============ Second Step (STANCE_RRFL) ============= //
        // ============ Continuous Walking Start ============== //
        FC_PHASE = STANCE_RRFL;
        tmp_t = ft_time - ft_step_time;
        tmp_t2 = ft_time - ft_step_time;

        com_pos[0] = c_com_x1[5] * pow(tmp_t, 5) + c_com_x1[4] * pow(tmp_t, 4) + c_com_x1[3] * pow(tmp_t, 3) + c_com_x1[2] * pow(tmp_t, 2) + c_com_x1[1] * pow(tmp_t, 1) + c_com_x1[0]; //init_com_pos(0) + x_moving_speed * (ts / 2 + tf + t2);
        com_pos[1] = pre_com_pos(1);
        com_pos[2] = c_com_z3[5] * pow(tmp_t, 5) + c_com_z3[4] * pow(tmp_t, 4) + c_com_z3[3] * pow(tmp_t, 3) + c_com_z3[2] * pow(tmp_t, 2) + c_com_z3[1] * pow(tmp_t, 1) + c_com_z3[0];

        RL_foot_pos[0] = pre_RL_foot_pos[0] + c_sf_x1[5] * pow(tmp_t2, 5) + c_sf_x1[4] * pow(tmp_t2, 4) + c_sf_x1[3] * pow(tmp_t2, 3) + c_sf_x1[2] * pow(tmp_t2, 2) + c_sf_x1[1] * pow(tmp_t2, 1) + c_sf_x1[0];
        RL_foot_pos[1] = pre_RL_foot_pos[1];
        RL_foot_pos[2] = pre_RL_foot_pos[2];

        //        printf("RL_foot_pos[0] = %f\n",RL_foot_pos[0]);
        RR_foot_pos[0] = pre_RR_foot_pos[0];
        RR_foot_pos[1] = pre_RR_foot_pos[1];
        RR_foot_pos[2] = pre_RR_foot_pos[2];

        FL_foot_pos[0] = pre_FL_foot_pos[0];
        FL_foot_pos[1] = pre_FL_foot_pos[1];
        FL_foot_pos[2] = pre_FL_foot_pos[2];

        FR_foot_pos[0] = pre_FR_foot_pos[0] + c_sf_x1[5] * pow(tmp_t2, 5) + c_sf_x1[4] * pow(tmp_t2, 4) + c_sf_x1[3] * pow(tmp_t2, 3) + c_sf_x1[2] * pow(tmp_t2, 2) + c_sf_x1[1] * pow(tmp_t2, 1) + c_sf_x1[0];
        FR_foot_pos[1] = pre_FR_foot_pos[1];
        FR_foot_pos[2] = pre_FR_foot_pos[2];

        com_acc[2] = 20 * c_com_z3[5] * pow(tmp_t, 3) + 12 * c_com_z3[4] * pow(tmp_t, 2) + 6 * c_com_z3[3] * pow(tmp_t, 1) + 2 * c_com_z3[2];

        if (ft_cnt == ft_step_cnt + ts_cnt - 1) {
            pre_com_pos(0) = pre_com_pos[0] + pre_x_moving_speed * (ts) / 2.0 + x_moving_speed * (ts) / 2.0;
            pre_com_pos(1) = com_pos(1);
            pre_com_pos(2) = h_1; //com_pos(2);

            //            pre_RL_foot_pos[0] = pre_RL_foot_pos[0] + pre_x_moving_speed * ft_step_time + x_moving_speed * ft_step_time;//RL_foot_pos[0];//x_moving_speed * (ft_step_time);
            pre_RL_foot_pos[1] = RL_foot_pos[1];
            pre_RL_foot_pos[2] = RL_foot_pos[2];

            pre_RR_foot_pos[0] = RR_foot_pos[0];
            pre_RR_foot_pos[1] = RR_foot_pos[1];
            pre_RR_foot_pos[2] = RR_foot_pos[2];

            pre_FL_foot_pos[0] = FL_foot_pos[0];
            pre_FL_foot_pos[1] = FL_foot_pos[1];
            pre_FL_foot_pos[2] = FL_foot_pos[2];

            //            pre_FR_foot_pos[0] = pre_FR_foot_pos[0] + pre_x_moving_speed * ft_step_time + x_moving_speed * ft_step_time;//FR_foot_pos[0];
            pre_FR_foot_pos[1] = FR_foot_pos[1];
            pre_FR_foot_pos[2] = FR_foot_pos[2];

            //            printf("[final] FR_foot_pos[0] = %f, pre_FR_foot_pos[0] = %f\n",FR_foot_pos[0],pre_FR_foot_pos[0]);
        }

        //        printf("FR_foot_pos[0] = %f\n",FR_foot_pos[0]);

    }
    else if (ft_cnt < 2 * ft_step_cnt) {
        //        printf("[4] Second Step (FP)\n");
        // ============ Second Step (FP) ============= //
        FC_PHASE = ZERO;
        tmp_t = ft_time - ft_step_time - ts;
        tmp_t2 = ft_time - ft_step_time;

        com_pos[0] = pre_com_pos(0) + x_moving_speed*tmp_t;
        com_pos[1] = pre_com_pos(1);
        com_pos[2] = c_com_z2[5] * pow(tmp_t, 5) + c_com_z2[4] * pow(tmp_t, 4) + c_com_z2[3] * pow(tmp_t, 3) + c_com_z2[2] * pow(tmp_t, 2) + c_com_z2[1] * pow(tmp_t, 1) + c_com_z2[0];

        RL_foot_pos[0] = pre_RL_foot_pos[0] + c_sf_x1[5] * pow(tmp_t2, 5) + c_sf_x1[4] * pow(tmp_t2, 4) + c_sf_x1[3] * pow(tmp_t2, 3) + c_sf_x1[2] * pow(tmp_t2, 2) + c_sf_x1[1] * pow(tmp_t2, 1) + c_sf_x1[0];
        RL_foot_pos[1] = pre_RL_foot_pos[1];
        RL_foot_pos[2] = init_RL_foot_pos[2] + c_sf_z3[5] * pow(tmp_t, 5) + c_sf_z3[4] * pow(tmp_t, 4) + c_sf_z3[3] * pow(tmp_t, 3) + c_sf_z3[2] * pow(tmp_t, 2) + c_sf_z3[1] * pow(tmp_t, 1) + c_sf_z3[0];

        RR_foot_pos[0] = pre_RR_foot_pos[0];
        RR_foot_pos[1] = pre_RR_foot_pos[1];
        RR_foot_pos[2] = init_RR_foot_pos[2] + c_sf_z2[5] * pow(tmp_t, 5) + c_sf_z2[4] * pow(tmp_t, 4) + c_sf_z2[3] * pow(tmp_t, 3) + c_sf_z2[2] * pow(tmp_t, 2) + c_sf_z2[1] * pow(tmp_t, 1) + c_sf_z2[0];
        //        RR_foot_pos[2] = init_RR_foot_pos[2] + swing_foot_height/2.0*(1-cos(PI2/(2*tf)*tmp_t));

        FL_foot_pos[0] = pre_FL_foot_pos[0];
        FL_foot_pos[1] = pre_FL_foot_pos[1];
        FL_foot_pos[2] = init_FL_foot_pos[2] + c_sf_z2[5] * pow(tmp_t, 5) + c_sf_z2[4] * pow(tmp_t, 4) + c_sf_z2[3] * pow(tmp_t, 3) + c_sf_z2[2] * pow(tmp_t, 2) + c_sf_z2[1] * pow(tmp_t, 1) + c_sf_z2[0];
        //        FL_foot_pos[2] = init_FL_foot_pos[2] + swing_foot_height/2.0*(1-cos(PI2/(2*tf)*tmp_t));

        FR_foot_pos[0] = pre_FR_foot_pos[0] + c_sf_x1[5] * pow(tmp_t2, 5) + c_sf_x1[4] * pow(tmp_t2, 4) + c_sf_x1[3] * pow(tmp_t2, 3) + c_sf_x1[2] * pow(tmp_t2, 2) + c_sf_x1[1] * pow(tmp_t2, 1) + c_sf_x1[0];
        FR_foot_pos[1] = pre_FR_foot_pos[1];
        FR_foot_pos[2] = init_FR_foot_pos[2] + c_sf_z3[5] * pow(tmp_t, 5) + c_sf_z3[4] * pow(tmp_t, 4) + c_sf_z3[3] * pow(tmp_t, 3) + c_sf_z3[2] * pow(tmp_t, 2) + c_sf_z3[1] * pow(tmp_t, 1) + c_sf_z3[0];

        //        printf("FR_foot_pos[0] = %f\n",FR_foot_pos[0]);

        com_acc[2] = 0;

        if (ft_cnt == 2 * ft_step_cnt - 1) {
            tmp_t2 = ts + tf;

            pre_com_pos(0) = pre_com_pos(0) + x_moving_speed*tf;
            pre_com_pos(1) = com_pos(1);
            pre_com_pos(2) = h_2; //com_pos(2);

            pre_RL_foot_pos[0] = pre_RL_foot_pos[0] + c_sf_x1[5] * pow(tmp_t2, 5) + c_sf_x1[4] * pow(tmp_t2, 4) + c_sf_x1[3] * pow(tmp_t2, 3) + c_sf_x1[2] * pow(tmp_t2, 2) + c_sf_x1[1] * pow(tmp_t2, 1) + c_sf_x1[0]; //RL_foot_pos[0];//pre_RL_foot_pos(0) + pre_x_moving_speed * step_time + x_moving_speed * step_time;//RL_foot_pos[0];//x_moving_speed * (ft_step_time);
            pre_RL_foot_pos[1] = RL_foot_pos[1];
            pre_RL_foot_pos[2] = init_RL_foot_pos[2];

            pre_RR_foot_pos[0] = RR_foot_pos[0];
            pre_RR_foot_pos[1] = RR_foot_pos[1];
            pre_RR_foot_pos[2] = init_RR_foot_pos[2] + swing_foot_height;

            pre_FL_foot_pos[0] = FL_foot_pos[0];
            pre_FL_foot_pos[1] = FL_foot_pos[1];
            pre_FL_foot_pos[2] = init_FL_foot_pos[2] + swing_foot_height;

            pre_FR_foot_pos[0] = pre_FR_foot_pos[0] + c_sf_x1[5] * pow(tmp_t2, 5) + c_sf_x1[4] * pow(tmp_t2, 4) + c_sf_x1[3] * pow(tmp_t2, 3) + c_sf_x1[2] * pow(tmp_t2, 2) + c_sf_x1[1] * pow(tmp_t2, 1) + c_sf_x1[0];
            pre_FR_foot_pos[1] = FR_foot_pos[1];
            pre_FR_foot_pos[2] = init_FR_foot_pos[2];
        }
    }
    else if (ft_cnt < 2 * ft_step_cnt + ts_cnt) {
        //        printf("[5] Third Step (STANCE_RLFR)\n");
        // ============ Third Step (STANCE_RLFR) ============= //
        FC_PHASE = STANCE_RLFR;
        tmp_t = ft_time - 2 * ft_step_time;
        tmp_t2 = ft_time - 2 * ft_step_time;

        com_pos[0] = pre_com_pos(0) + x_moving_speed*tmp_t;
        com_pos[1] = pre_com_pos(1);
        com_pos[2] = c_com_z3[5] * pow(tmp_t, 5) + c_com_z3[4] * pow(tmp_t, 4) + c_com_z3[3] * pow(tmp_t, 3) + c_com_z3[2] * pow(tmp_t, 2) + c_com_z3[1] * pow(tmp_t, 1) + c_com_z3[0];

        RL_foot_pos[0] = pre_RL_foot_pos[0];
        RL_foot_pos[1] = pre_RL_foot_pos[1];
        RL_foot_pos[2] = pre_RL_foot_pos[2];

        RR_foot_pos[0] = pre_RR_foot_pos[0] + c_sf_x2[5] * pow(tmp_t2, 5) + c_sf_x2[4] * pow(tmp_t2, 4) + c_sf_x2[3] * pow(tmp_t2, 3) + c_sf_x2[2] * pow(tmp_t2, 2) + c_sf_x2[1] * pow(tmp_t2, 1) + c_sf_x2[0];
        RR_foot_pos[1] = pre_RR_foot_pos[1];
        RR_foot_pos[2] = pre_RR_foot_pos[2];

        FL_foot_pos[0] = pre_FL_foot_pos[0] + c_sf_x2[5] * pow(tmp_t2, 5) + c_sf_x2[4] * pow(tmp_t2, 4) + c_sf_x2[3] * pow(tmp_t2, 3) + c_sf_x2[2] * pow(tmp_t2, 2) + c_sf_x2[1] * pow(tmp_t2, 1) + c_sf_x2[0];
        FL_foot_pos[1] = pre_FL_foot_pos[1];
        FL_foot_pos[2] = pre_FL_foot_pos[2];

        FR_foot_pos[0] = pre_FR_foot_pos[0];
        FR_foot_pos[1] = pre_FR_foot_pos[1];
        FR_foot_pos[2] = pre_FR_foot_pos[2];

        com_acc[2] = 20 * c_com_z3[5] * pow(tmp_t, 3) + 12 * c_com_z3[4] * pow(tmp_t, 2) + 6 * c_com_z3[3] * pow(tmp_t, 1) + 2 * c_com_z3[2];

        //        printf("pre_RL_foot_pos[0] = %f, pre_RR_foot_pos[0] = %f\n",pre_RL_foot_pos[0],pre_RR_foot_pos[0]);   

        if (ft_cnt == 2 * ft_step_cnt + ts_cnt - 1) {
            //            cout << "test!!!!!!!!!!!!!!!!!!!!!!!!! " << endl;

            pre_com_pos[0] = pre_com_pos[0] + x_moving_speed * (ts);
            pre_com_pos[1] = com_pos(1);
            pre_com_pos[2] = h_1; //com_pos(2);

            pre_RL_foot_pos[0] = RL_foot_pos[0];
            pre_RL_foot_pos[1] = RL_foot_pos[1];
            pre_RL_foot_pos[2] = RL_foot_pos[2];

            //            pre_RR_foot_pos[0] = pre_RR_foot_pos(0) + x_moving_speed * ft_step_time*2;//RR_foot_pos[0];
            pre_RR_foot_pos[1] = RR_foot_pos[1];
            pre_RR_foot_pos[2] = RR_foot_pos[2];

            //            pre_FL_foot_pos[0] = pre_FL_foot_pos(0) + x_moving_speed * ft_step_time*2;//FL_foot_pos[0];
            pre_FL_foot_pos[1] = FL_foot_pos[1];
            pre_FL_foot_pos[2] = FL_foot_pos[2];

            pre_FR_foot_pos[0] = FR_foot_pos[0];
            pre_FR_foot_pos[1] = FR_foot_pos[1];
            pre_FR_foot_pos[2] = FR_foot_pos[2];

            //            printf("pre_RL_foot_pos[0] = %f, pre_RR_foot_pos[0] = %f\n",pre_RL_foot_pos[0],pre_RR_foot_pos[0]);   
        }

    }
    else if (ft_cnt < 3 * ft_step_cnt) {
        //        printf("[6] Third Step (FP)\n");
        // ============ Third Step (FP) ============= //
        FC_PHASE = ZERO;
        tmp_t = ft_time - 2 * ft_step_time - ts;
        tmp_t2 = ft_time - 2 * ft_step_time;

        com_pos[0] = pre_com_pos(0) + x_moving_speed*tmp_t;
        com_pos[1] = pre_com_pos(1);
        com_pos[2] = c_com_z2[5] * pow(tmp_t, 5) + c_com_z2[4] * pow(tmp_t, 4) + c_com_z2[3] * pow(tmp_t, 3) + c_com_z2[2] * pow(tmp_t, 2) + c_com_z2[1] * pow(tmp_t, 1) + c_com_z2[0];


        RL_foot_pos[0] = pre_RL_foot_pos[0];
        RL_foot_pos[1] = pre_RL_foot_pos[1];
        RL_foot_pos[2] = init_RL_foot_pos[2] + c_sf_z2[5] * pow(tmp_t, 5) + c_sf_z2[4] * pow(tmp_t, 4) + c_sf_z2[3] * pow(tmp_t, 3) + c_sf_z2[2] * pow(tmp_t, 2) + c_sf_z2[1] * pow(tmp_t, 1) + c_sf_z2[0];
        //        RL_foot_pos[2] = init_RL_foot_pos[2] + swing_foot_height/2.0*(1-cos(PI2/(2*tf)*tmp_t));

        RR_foot_pos[0] = pre_RR_foot_pos[0] + c_sf_x2[5] * pow(tmp_t2, 5) + c_sf_x2[4] * pow(tmp_t2, 4) + c_sf_x2[3] * pow(tmp_t2, 3) + c_sf_x2[2] * pow(tmp_t2, 2) + c_sf_x2[1] * pow(tmp_t2, 1) + c_sf_x2[0];
        RR_foot_pos[1] = pre_RR_foot_pos[1];
        RR_foot_pos[2] = init_RR_foot_pos[2] + c_sf_z3[5] * pow(tmp_t, 5) + c_sf_z3[4] * pow(tmp_t, 4) + c_sf_z3[3] * pow(tmp_t, 3) + c_sf_z3[2] * pow(tmp_t, 2) + c_sf_z3[1] * pow(tmp_t, 1) + c_sf_z3[0];

        FL_foot_pos[0] = pre_FL_foot_pos[0] + c_sf_x2[5] * pow(tmp_t2, 5) + c_sf_x2[4] * pow(tmp_t2, 4) + c_sf_x2[3] * pow(tmp_t2, 3) + c_sf_x2[2] * pow(tmp_t2, 2) + c_sf_x2[1] * pow(tmp_t2, 1) + c_sf_x2[0];
        FL_foot_pos[1] = pre_FL_foot_pos[1];
        FL_foot_pos[2] = init_FL_foot_pos[2] + c_sf_z3[5] * pow(tmp_t, 5) + c_sf_z3[4] * pow(tmp_t, 4) + c_sf_z3[3] * pow(tmp_t, 3) + c_sf_z3[2] * pow(tmp_t, 2) + c_sf_z3[1] * pow(tmp_t, 1) + c_sf_z3[0];

        FR_foot_pos[0] = pre_FR_foot_pos[0];
        FR_foot_pos[1] = pre_FR_foot_pos[1];
        FR_foot_pos[2] = init_FR_foot_pos[2] + c_sf_z2[5] * pow(tmp_t, 5) + c_sf_z2[4] * pow(tmp_t, 4) + c_sf_z2[3] * pow(tmp_t, 3) + c_sf_z2[2] * pow(tmp_t, 2) + c_sf_z2[1] * pow(tmp_t, 1) + c_sf_z2[0];
        //        FR_foot_pos[2] = init_FR_foot_pos[2] + swing_foot_height/2.0*(1-cos(PI2/(2*tf)*tmp_t));

        com_acc[2] = 0;

        if (ft_cnt == 3 * ft_step_cnt - 1) {

            pre_com_pos(0) = pre_com_pos[0] + x_moving_speed * tf;
            pre_com_pos(1) = pre_com_pos[1];
            pre_com_pos(2) = h_1;

            pre_RL_foot_pos[0] = RL_foot_pos[0];
            pre_RL_foot_pos[1] = RL_foot_pos[1];
            pre_RL_foot_pos[2] = init_RL_foot_pos[2] + swing_foot_height;

            pre_RR_foot_pos[0] = RR_foot_pos[0];
            pre_RR_foot_pos[1] = RR_foot_pos[1];
            pre_RR_foot_pos[2] = init_RR_foot_pos[2];

            pre_FL_foot_pos[0] = FL_foot_pos[0];
            pre_FL_foot_pos[1] = FL_foot_pos[1];
            pre_FL_foot_pos[2] = init_FL_foot_pos[2];

            pre_FR_foot_pos[0] = FR_foot_pos[0];
            pre_FR_foot_pos[1] = FR_foot_pos[1];
            pre_FR_foot_pos[2] = init_FR_foot_pos[2] + swing_foot_height;


            if (sub_ctrl_flag == false) {
                pre_x_moving_speed = x_moving_speed;
                x_moving_speed = tmp_x_moving_speed;

                COM_SF_FT_X_Traj_Gen();
                ft_cnt = ft_step_cnt - 1;
            }
            else {
                // =============== COM =============== //                
                init_x[0] = pre_com_pos[0];
                init_x[1] = x_moving_speed;
                init_x[2] = 0;

                final_x[0] = pre_com_pos[0] + x_moving_speed * (ts) / 2.0;
                final_x[1] = 0;
                final_x[2] = 0;

                coefficient_5thPoly(init_x, final_x, ts, c_com_x2);

                // =============== Swing Foot =============== //

                // Left (Second)
                init_x[0] = 0;
                init_x[1] = 0;
                init_x[2] = 0;

                final_x[0] = 0 + x_moving_speed * ft_step_time;
                final_x[1] = 0;
                final_x[2] = 0;

                coefficient_5thPoly(init_x, final_x, ts, c_sf_x5);
            }
        }
    }

    else if (ft_cnt < 3 * ft_step_cnt + ts_cnt) {
        //        printf("[7] Final step (STANCE_RR_FL)\n");
        // ============ Final step (STANCE_RR_FL)============== //
        // =========== [FT] Final Step ========== //
        FC_PHASE = STANCE_RRFL;
        tmp_t = ft_time - 3 * ft_step_time;

        if (ft_cnt == 3 * ft_step_cnt) {
            flying_trot_final_flag = true;
        }

        com_pos[0] = c_com_x2[5] * pow(tmp_t, 5) + c_com_x2[4] * pow(tmp_t, 4) + c_com_x2[3] * pow(tmp_t, 3) + c_com_x2[2] * pow(tmp_t, 2) + c_com_x2[1] * pow(tmp_t, 1) + c_com_x2[0];
        com_pos[1] = init_com_pos(1);
        com_pos[2] = c_com_z4[5] * pow(tmp_t, 5) + c_com_z4[4] * pow(tmp_t, 4) + c_com_z4[3] * pow(tmp_t, 3) + c_com_z4[2] * pow(tmp_t, 2) + c_com_z4[1] * pow(tmp_t, 1) + c_com_z4[0];

        RL_foot_pos[0] = pre_RL_foot_pos[0] + c_sf_x5[5] * pow(tmp_t, 5) + c_sf_x5[4] * pow(tmp_t, 4) + c_sf_x5[3] * pow(tmp_t, 3) + c_sf_x5[2] * pow(tmp_t, 2) + c_sf_x5[1] * pow(tmp_t, 1) + c_sf_x5[0];
        RL_foot_pos[1] = pre_RL_foot_pos[1];
        RL_foot_pos[2] = init_RL_foot_pos[2] + c_sf_z4[5] * pow(tmp_t, 5) + c_sf_z4[4] * pow(tmp_t, 4) + c_sf_z4[3] * pow(tmp_t, 3) + c_sf_z4[2] * pow(tmp_t, 2) + c_sf_z4[1] * pow(tmp_t, 1) + c_sf_z4[0];

        RR_foot_pos[0] = pre_RR_foot_pos[0];
        RR_foot_pos[1] = pre_RR_foot_pos[1];
        RR_foot_pos[2] = pre_RR_foot_pos[2];

        FL_foot_pos[0] = pre_FL_foot_pos[0];
        FL_foot_pos[1] = pre_FL_foot_pos[1];
        FL_foot_pos[2] = pre_FL_foot_pos[2];

        FR_foot_pos[0] = pre_FR_foot_pos[0] + c_sf_x5[5] * pow(tmp_t, 5) + c_sf_x5[4] * pow(tmp_t, 4) + c_sf_x5[3] * pow(tmp_t, 3) + c_sf_x5[2] * pow(tmp_t, 2) + c_sf_x5[1] * pow(tmp_t, 1) + c_sf_x5[0];
        FR_foot_pos[1] = pre_FR_foot_pos[1];
        FR_foot_pos[2] = init_FR_foot_pos[2] + c_sf_z4[5] * pow(tmp_t, 5) + c_sf_z4[4] * pow(tmp_t, 4) + c_sf_z4[3] * pow(tmp_t, 3) + c_sf_z4[2] * pow(tmp_t, 2) + c_sf_z4[1] * pow(tmp_t, 1) + c_sf_z4[0];

        com_acc[2] = 20 * c_com_z4[5] * pow(tmp_t, 3) + 12 * c_com_z4[4] * pow(tmp_t, 2) + 6 * c_com_z4[3] * pow(tmp_t, 1) + 2 * c_com_z4[2];

        if (ft_cnt == 3 * ft_step_cnt + ts_cnt - 1) {
            pre_com_pos[0] = pre_com_pos[0] + x_moving_speed * (ts) / 2.0;
            pre_com_pos[1] = com_pos(1);
            pre_com_pos[2] = h_0; //com_pos(2);

            pre_RL_foot_pos[0] = pre_RL_foot_pos(0) + x_moving_speed * ft_step_time; //init_RL_foot_pos[0];// + pre_x_moving_speed * step_time + x_moving_speed * step_time;//RL_foot_pos[0];//x_moving_speed * (ft_step_time);
            pre_RL_foot_pos[1] = pre_RL_foot_pos[1];
            pre_RL_foot_pos[2] = init_RL_foot_pos[2];

            pre_RR_foot_pos[0] = pre_RR_foot_pos[0]; //pre_RL_foot_pos(0) + pre_x_moving_speed * step_time + x_moving_speed * step_time;//RR_foot_pos[0];
            pre_RR_foot_pos[1] = pre_RR_foot_pos[1];
            pre_RR_foot_pos[2] = pre_RR_foot_pos[2];

            pre_FL_foot_pos[0] = pre_FL_foot_pos[0]; //pre_RL_foot_pos(0) + pre_x_moving_speed * step_time + x_moving_speed * step_time;//FL_foot_pos[0];
            pre_FL_foot_pos[1] = pre_FL_foot_pos[1];
            pre_FL_foot_pos[2] = pre_FL_foot_pos[2];

            pre_FR_foot_pos[0] = pre_FR_foot_pos(0) + x_moving_speed * ft_step_time; //init_FR_foot_pos[0];// + pre_x_moving_speed * step_time + x_moving_speed * step_time;//FR_foot_pos[0];
            pre_FR_foot_pos[1] = pre_FR_foot_pos[1];
            pre_FR_foot_pos[2] = init_FR_foot_pos[2];
        }
    }
    else if (ft_cnt < 4 * ft_step_cnt) {
        // ============ Final step (FSP)============== //
        //        printf("[8] Final step (FSP)\n");
        FC_PHASE = STOP;


        if (ft_cnt == 3 * ft_step_cnt + ts_cnt) {
            moving_done_flag = true;
        }


        com_pos = pre_com_pos;
        RL_foot_pos = pre_RL_foot_pos;
        RR_foot_pos = pre_RR_foot_pos;
        FL_foot_pos = pre_FL_foot_pos;
        FR_foot_pos = pre_FR_foot_pos;

        com_acc[2] = 0;
    }

    if (ft_cnt > 0) {
        target_com_vel[0] = (com_pos[0] - old_com_pos[0]) / dt;
        old_com_pos[0] = com_pos[0];
        if (ft_cnt > 1) {
            target_com_acc[0] = (target_com_vel[0] - old_com_vel[0]) / dt;
            old_com_vel[0] = target_com_vel[0];
        }
    }
}

void CRobot::check_CP_FT(void)
{
    static double tmp_cp_y_lower_limit = 0.08;
    static double tmp_cp_y_upper_limit = 0.20;

    if (CP_y > tmp_cp_y_lower_limit) {
        //        cout << "CP_y = " << CP_y << endl;

        if (CP_y < tmp_cp_y_upper_limit) {
            tmp_cp_foot_pos_y = CP_y;
        }
        else {
            tmp_cp_foot_pos_y = tmp_cp_y_upper_limit;
        }

        get_cp_done_flag = true;
    }
    else if (CP_y < -tmp_cp_y_lower_limit) {
        //        cout << "CP_y = " << CP_y << endl;

        if (CP_y > -tmp_cp_y_upper_limit) {
            tmp_cp_foot_pos_y = CP_y;
        }
        else {
            tmp_cp_foot_pos_y = -tmp_cp_y_upper_limit;
        }

        get_cp_done_flag = true;
    }
    else if (get_cp_done_flag == false) {
        tmp_cp_foot_pos_y = 0;
    }
}

void CRobot::CP_Con_FT(void)
{
    const double tmp_kp_roll = 0.01;
    const double tmp_kd_roll = 0.00001; //0.00002;
    const double tmp_kp_pitch = 0.005;
    const double tmp_kd_pitch = 0.000005;
    const double max_cp_foot_pos[3] = {0.05, 0.08, 0.05};
    const double IMURoll_alpha = 0.03, IMURoll_dot_alpha = 0.02, IMUPitch_alpha = 0.02, IMUPitch_dot_alpha = 0.01;
    static double lpf_IMURoll = 0, lpf_IMURoll_dot = 0, lpf_IMUPitch = 0, lpf_IMUPitch_dot = 0;
    static double tmp_u_r = 0, tmp_u_p = 0;
    static double cp_weight = 0;

    if (ft_cnt <= ft_step_cnt) {
        cp_weight = 0.5 * (1 - cos(PI2 / (ft_step_time * 2)*(double) ft_cnt * dt));
    }

    //    cout << "cp_weight = " << cp_weight << endl;

    lpf_IMURoll = (1 - IMURoll_alpha) * lpf_IMURoll + IMURoll_alpha*IMURoll;
    lpf_IMURoll_dot = (1 - IMURoll_dot_alpha) * lpf_IMURoll_dot + IMURoll_dot_alpha*IMURoll_dot;

    lpf_IMUPitch = (1 - IMUPitch_alpha) * lpf_IMUPitch + IMUPitch_alpha*IMUPitch;
    lpf_IMUPitch_dot = (1 - IMUPitch_dot_alpha) * lpf_IMUPitch_dot + IMUPitch_dot_alpha*IMUPitch_dot;

    // Roll
    tmp_u_r = cp_weight * (tmp_kp_roll * (0 - lpf_IMURoll) + tmp_kd_roll * (0 - lpf_IMURoll_dot));

    if (tmp_u_r > 0) {
        cp_RL_foot_pos[1] = tmp_u_r;
        cp_RR_foot_pos[1] = -tmp_u_r * 0.2;
        cp_FL_foot_pos[1] = tmp_u_r;
        cp_FR_foot_pos[1] = -tmp_u_r * 0.2;
    }
    else {
        cp_RL_foot_pos[1] = -tmp_u_r * 0.2;
        cp_RR_foot_pos[1] = tmp_u_r;
        cp_FL_foot_pos[1] = -tmp_u_r * 0.2;
        cp_FR_foot_pos[1] = tmp_u_r;
    }

    if (cp_RL_foot_pos[1] > max_cp_foot_pos[1]) {
        cp_RL_foot_pos[1] = max_cp_foot_pos[1];
    }
    else if (cp_RL_foot_pos[1] < -max_cp_foot_pos[1]) {
        cp_RL_foot_pos[1] = -max_cp_foot_pos[1];
    }
    if (cp_RR_foot_pos[1] > max_cp_foot_pos[1]) {
        cp_RR_foot_pos[1] = max_cp_foot_pos[1];
    }
    else if (cp_RR_foot_pos[1] < -max_cp_foot_pos[1]) {
        cp_RR_foot_pos[1] = -max_cp_foot_pos[1];
    }
    if (cp_FL_foot_pos[1] > max_cp_foot_pos[1]) {
        cp_FL_foot_pos[1] = max_cp_foot_pos[1];
    }
    else if (cp_FL_foot_pos[1] < -max_cp_foot_pos[1]) {
        cp_FL_foot_pos[1] = -max_cp_foot_pos[1];
    }
    if (cp_FR_foot_pos[1] > max_cp_foot_pos[1]) {
        cp_FR_foot_pos[1] = max_cp_foot_pos[1];
    }
    else if (cp_FR_foot_pos[1] < -max_cp_foot_pos[1]) {
        cp_FR_foot_pos[1] = -max_cp_foot_pos[1];
    }

    // Pitch
    tmp_u_p = 0; //cp_weight * (-tmp_kp_pitch * (des_pitch_deg - lpf_IMUPitch) - tmp_kd_pitch * (0 - lpf_IMUPitch_dot));



    if (tmp_u_p > 0) {
        cp_RL_foot_pos[0] = -tmp_u_p * 0.3;
        cp_RR_foot_pos[0] = -tmp_u_p * 0.3;
        cp_FL_foot_pos[0] = tmp_u_p;
        cp_FR_foot_pos[0] = tmp_u_p;
    }
    else {
        cp_RL_foot_pos[0] = tmp_u_p;
        cp_RR_foot_pos[0] = tmp_u_p;
        cp_FL_foot_pos[0] = -tmp_u_p * 0.3;
        cp_FR_foot_pos[0] = -tmp_u_p * 0.3;
    }


    if (cp_RL_foot_pos[0] > max_cp_foot_pos[0]) {
        cp_RL_foot_pos[0] = max_cp_foot_pos[0];
    }
    else if (cp_RL_foot_pos[0] < -max_cp_foot_pos[0]) {
        cp_RL_foot_pos[0] = -max_cp_foot_pos[0];
    }
    if (cp_RR_foot_pos[0] > max_cp_foot_pos[0]) {
        cp_RR_foot_pos[0] = max_cp_foot_pos[0];
    }
    else if (cp_RR_foot_pos[0] < -max_cp_foot_pos[0]) {
        cp_RR_foot_pos[0] = -max_cp_foot_pos[0];
    }
    if (cp_FL_foot_pos[0] > max_cp_foot_pos[0]) {
        cp_FL_foot_pos[0] = max_cp_foot_pos[0];
    }
    else if (cp_FL_foot_pos[0] < -max_cp_foot_pos[0]) {
        cp_FL_foot_pos[0] = -max_cp_foot_pos[0];
    }
    if (cp_FR_foot_pos[0] > max_cp_foot_pos[0]) {
        cp_FR_foot_pos[0] = max_cp_foot_pos[0];
    }
    else if (cp_FR_foot_pos[0] < -max_cp_foot_pos[0]) {
        cp_FR_foot_pos[0] = -max_cp_foot_pos[0];
    }

    //    cout << "[P] cp_RL_foot_pos[0] = " << cp_RL_foot_pos[0] << ", cp_FL_foot_pos[0] = " << cp_FL_foot_pos[0] << endl;
    //    cout << "[R] cp_RL_foot_pos[1] = " << cp_RL_foot_pos[1] << ", cp_RR_foot_pos[1] = " << cp_RR_foot_pos[1] << endl;

    //    cout << "cp_RL_foot_pos[1] = " << cp_RL_foot_pos[1] << ",cp_RR_foot_pos[1] = " << cp_RR_foot_pos[1] << endl;

}

//void CRobot::check_CP_FT(void)
//{
//    static double tmp_cp_y_lower_limit = 0.08;
//    static double tmp_cp_y_upper_limit = 0.20;
//
//    if (CP_y > tmp_cp_y_lower_limit) {
//        //        cout << "CP_y = " << CP_y << endl;
//
//        if (CP_y < tmp_cp_y_upper_limit) {
//            tmp_cp_foot_pos_y = CP_y;
//        }
//        else {
//            tmp_cp_foot_pos_y = tmp_cp_y_upper_limit;
//        }
//
//        get_cp_done_flag = true;
//    }
//    else if (CP_y < -tmp_cp_y_lower_limit) {
//        //        cout << "CP_y = " << CP_y << endl;
//
//        if (CP_y > -tmp_cp_y_upper_limit) {
//            tmp_cp_foot_pos_y = CP_y;
//        }
//        else {
//            tmp_cp_foot_pos_y = -tmp_cp_y_upper_limit;
//        }
//
//        get_cp_done_flag = true;
//    }
//    else if (get_cp_done_flag == false) {
//        tmp_cp_foot_pos_y = 0;
//    }
//}
//
//void CRobot::CP_Con_FT(void){
//    static int tmp_cp_cnt = 0;
//    static int cp_on_cnt = 0;
//    static double tmp_cp_time = 0;
//    // ==================== CP CONTROLLER =================== //
//
//    // CP PHASE
//    // 0 : Check CP
//    // 1 : CASE1
//    // 2 : CASE2
//
//    switch (CP_PHASE) {
//    case 0:
////        cout << "[CP_PHASE 0]" << endl;
//        check_CP_FT();
//        
//        if (ft_cnt % ft_step_cnt < ts_cnt) {
////        if (ft_cnt % ft_step_cnt == ts_cnt - 1) {
//            if (get_cp_done_flag == true) {
//                if(FC_PHASE == STANCE_RRFL){
//                    CP_PHASE = 1;
//                }
//                else{
//                    CP_PHASE = 2;
//                }
//                cp_on_cnt = ft_cnt % ft_step_cnt;
//                
//                cout << "cp_on_cnt = " << cp_on_cnt << endl;
//                
//                CP_move_step = 0;
//                cp_foot_pos_y = tmp_cp_foot_pos_y / 3.0;
//                tmp_cp_cnt = 0;
//                get_cp_done_flag = false;
//                
//                init_CP_y = CP_y;
//            }
//            else {
//                cp_foot_pos_y = 0;
//            }
//
////            if (tmp_y_moving_speed > 0.01 || tmp_y_moving_speed < -0.01) {
////                cp_foot_pos_y = 0;
////                CP_PHASE = 0;
////            }
////            else 
//            if (des_theta > 5 * D2R || des_theta < -5 * D2R) {
//                cp_foot_pos_y = 0;
//                CP_PHASE = 0;
//            }
//        }
//        break;
//
//    case 1:
//        cout << "[CP_PHASE 1] RRFL" << endl;
//        if (CP_move_step == 0) {
//            cout << "[CP_move_step == 0]" << endl;
//            if (ft_cnt % ft_step_cnt < ft_step_cnt - 1) {
//                cout << "[1]" << endl;
//                tmp_t = (double) (ft_cnt % ft_step_cnt - 1 - cp_on_cnt) * dt; // now time
//                tmp_cp_time = (double) (ft_step_cnt - cp_on_cnt) * dt; // stepping time
//                
//                cout << "ft_cnt % ft_step_cnt - 1 - cp_on_cnt = " << ft_cnt % ft_step_cnt - 1 - cp_on_cnt << endl;
//                cout << "tmp_t = " << tmp_t << ", tmp_cp_time = " << tmp_cp_time << endl;
//
//                cp_foot_offset_y = Kp_cp * (CP_y - init_CP_y) / 2.0;
//
//                cp_RL_foot_pos[1] = init_cp_RL_foot_pos[1] + (cp_foot_pos_y) / 2.0 * (1 - cos(PI2 / (2 * tmp_cp_time) * tmp_t)) + cp_foot_offset_y;
//                cp_RR_foot_pos[1] = init_cp_RR_foot_pos[1] - (cp_foot_pos_y) / 2.0 * (1 - cos(PI2 / (2 * tmp_cp_time) * tmp_t)) - cp_foot_offset_y;
//                cp_FL_foot_pos[1] = init_cp_FL_foot_pos[1] - (cp_foot_pos_y) / 2.0 * (1 - cos(PI2 / (2 * tmp_cp_time) * tmp_t)) - cp_foot_offset_y;
//                cp_FR_foot_pos[1] = init_cp_FR_foot_pos[1] + (cp_foot_pos_y) / 2.0 * (1 - cos(PI2 / (2 * tmp_cp_time) * tmp_t)) + cp_foot_offset_y;
//                
////                cp_RL_foot_pos[2] = 0 + 0.02 / 2.0 * (1 - cos(PI2 / (2 * tmp_cp_time) * tmp_t));
//            }
//            else {
//                CP_move_step = 1;
//            }
//        }
//        else if (CP_move_step == 1) {
//            cout << "[CP_move_step == 1]" << endl;
//            if (ft_cnt % ft_step_cnt < ft_step_cnt - 1) {
//                cout << "[3]" << endl;
//                tmp_t = (double) (ft_cnt % ft_step_cnt - 1) * dt;
//
//                cp_RL_foot_pos[1] = init_cp_RL_foot_pos[1] + (cp_foot_pos_y + cp_foot_offset_y) - (cp_foot_pos_y + cp_foot_offset_y) / 2.0 * (1 - cos(PI2 / (ft_step_time * 2) * tmp_t));
//                cp_RR_foot_pos[1] = init_cp_RR_foot_pos[1] - (cp_foot_pos_y + cp_foot_offset_y) + (cp_foot_pos_y + cp_foot_offset_y) / 2.0 * (1 - cos(PI2 / (ft_step_time * 2) * tmp_t));
//                cp_FL_foot_pos[1] = init_cp_FL_foot_pos[1] - (cp_foot_pos_y + cp_foot_offset_y) + (cp_foot_pos_y + cp_foot_offset_y) / 2.0 * (1 - cos(PI2 / (ft_step_time * 2) * tmp_t));
//                cp_FR_foot_pos[1] = init_cp_FR_foot_pos[1] + (cp_foot_pos_y + cp_foot_offset_y) - (cp_foot_pos_y + cp_foot_offset_y) / 2.0 * (1 - cos(PI2 / (ft_step_time * 2) * tmp_t));
//
//            }
//            else {
//                CP_move_step = 2;
//                cp_foot_offset_y = 0;
//                cp_foot_pos_y = 0;
//            }
//        }
//        else if (CP_move_step == 2) {
////            cout << "[CP_move_step == 2]" << "ft_cnt % ft_step_cnt = " << ft_cnt % ft_step_cnt << endl;
////            if (ft_cnt % ft_step_cnt == ft_step_cnt - 1) {
//            if (ft_cnt % ft_step_cnt == ft_step_cnt - 1) {
////                tmp_cp_cnt++;
////
////                cout << endl << " =========== tmp_cp_cnt = " << tmp_cp_cnt << " ===========" << endl;
//
////                    if (tmp_cp_cnt == 6) { //3
////                            cout << "[7]" << endl;
//                CP_PHASE = 0;
//                CP_move_step = 0;
//                CP_move_done_flag = true;
//                
////                    }
//            }
////            }
//        }
//        break;
//
//    case 2:
//        // swing foot can be located at CP during next phase.
//        cout << "[CP_PHASE 2] RLFR" << endl;
//        if (CP_move_step == 0) {
//            cout << "[CP_move_step == 0]" << endl;
//            if (ft_cnt % ft_step_cnt < ft_step_cnt - 1) {
////                tmp_t = (double) (ft_cnt % ft_step_cnt) * dt;
//                tmp_t = (double) (ft_cnt % ft_step_cnt - 1 - cp_on_cnt) * dt; // now time
//                tmp_cp_time = (double) (ft_step_cnt - cp_on_cnt) * dt; // stepping time
//                
//                cout << "tmp_t = " << tmp_t << ", tmp_cp_time = " << tmp_cp_time << endl;
//
//                cp_foot_offset_y = Kp_cp * (CP_y - init_CP_y) / 2.0;
//
//                cp_RL_foot_pos[1] = init_cp_RL_foot_pos[1] - (cp_foot_pos_y) / 2.0 * (1 - cos(PI2 / (tmp_cp_time * 2) * tmp_t)) - cp_foot_offset_y;
//                cp_RR_foot_pos[1] = init_cp_RR_foot_pos[1] + (cp_foot_pos_y) / 2.0 * (1 - cos(PI2 / (tmp_cp_time * 2) * tmp_t)) + cp_foot_offset_y;
//                cp_FL_foot_pos[1] = init_cp_FL_foot_pos[1] + (cp_foot_pos_y) / 2.0 * (1 - cos(PI2 / (tmp_cp_time * 2) * tmp_t)) + cp_foot_offset_y;
//                cp_FR_foot_pos[1] = init_cp_FR_foot_pos[1] - (cp_foot_pos_y) / 2.0 * (1 - cos(PI2 / (tmp_cp_time * 2) * tmp_t)) - cp_foot_offset_y;
//
////                    printf("[2] cp_foot_offset_y = %f\n", cp_foot_offset_y);
//            }
//            else {
//                CP_move_step = 1;
//            }
//        }
//        else if (CP_move_step == 1) {
//            cout << "[CP_move_step == 1]" << endl;
//            if (ft_cnt % ft_step_cnt < ft_step_cnt - 1) {
//                tmp_t = (double) (ft_cnt % ft_step_cnt - 1) * dt;
////                    cout << "[case.2][CP_move_step.2] tmp_t = " << tmp_t << ", cp_foot_pos_y = " << cp_foot_pos_y << endl;
//
//                cp_RL_foot_pos[1] = init_cp_RL_foot_pos[1] - (cp_foot_pos_y + cp_foot_offset_y) + (cp_foot_pos_y + cp_foot_offset_y) / 2.0 * (1 - cos(PI2 / (ft_step_time * 2) * tmp_t));
//                cp_RR_foot_pos[1] = init_cp_RR_foot_pos[1] + (cp_foot_pos_y + cp_foot_offset_y) - (cp_foot_pos_y + cp_foot_offset_y) / 2.0 * (1 - cos(PI2 / (ft_step_time * 2) * tmp_t));
//                cp_FL_foot_pos[1] = init_cp_FL_foot_pos[1] + (cp_foot_pos_y + cp_foot_offset_y) - (cp_foot_pos_y + cp_foot_offset_y) / 2.0 * (1 - cos(PI2 / (ft_step_time * 2) * tmp_t));
//                cp_FR_foot_pos[1] = init_cp_FR_foot_pos[1] - (cp_foot_pos_y + cp_foot_offset_y) + (cp_foot_pos_y + cp_foot_offset_y) / 2.0 * (1 - cos(PI2 / (ft_step_time * 2) * tmp_t));
//
//            }
//            else {
//                CP_move_step = 2;
//                cp_foot_pos_y = 0;
//                cp_foot_offset_y = 0;
//            }
//        }
//        else if (CP_move_step == 2) {
////            cout << "[CP_move_step == 2]" << "ft_cnt % ft_step_cnt = " << ft_cnt % ft_step_cnt << endl;
////            if (ft_cnt % ft_step_cnt == ft_step_cnt - 1) {
//            
//            if (ft_cnt % ft_step_cnt == ft_step_cnt - 1) {
////                tmp_cp_cnt++;
////                cout << endl << " =========== tmp_cp_cnt = " << tmp_cp_cnt << " ===========" << endl;
//
////                    if (tmp_cp_cnt == 6) { //3
//                CP_PHASE = 0;
//                CP_move_step = 0;
//                CP_move_done_flag = true;
//                
////                    }
//            }
////            }
//        }
//        break;
//    }
//
////    del_L_left = BOC_Kp_roll*(0 - lpf_IMURoll) + BOC_Ki_roll*sum_roll_err;
////    del_L_right = -BOC_Kp_roll*(0 - lpf_IMURoll) - BOC_Ki_roll*sum_roll_err;
//
////    cp_RL_foot_pos[2] = 0.01 * (0 - IMURoll) + 0.01 * (0 - IMURoll_dot);
//    
//    
////    cout << "cp_RL_foot_pos[1] = " << cp_RL_foot_pos[1] << ",cp_RR_foot_pos[1] = " << cp_RR_foot_pos[1] << endl;
//    
//}

//void CRobot::CP_Con_FT(void){
//    static int tmp_cp_cnt = 0;
//    static int cp_on_cnt = 0;
//    static double tmp_cp_time = 0;
//    // ==================== CP CONTROLLER =================== //
//
//    // CP PHASE
//    // 0 : Check CP
//    // 1 : CASE1
//    // 2 : CASE2
//
//    switch (CP_PHASE) {
//    case 0:
////        cout << "[CP_PHASE 0]" << endl;
//        check_CP_FT();
//        
//        if (ft_cnt % ft_step_cnt <= ts_cnt) {
//            if (get_cp_done_flag == true) {
//                if(FC_PHASE == STANCE_RRFL){
//                    CP_PHASE = 1;
//                }
//                else{
//                    CP_PHASE = 2;
//                }
//                cp_on_cnt = ft_cnt % ft_step_cnt;
//                
//                cout << "cp_on_cnt = " << cp_on_cnt << endl;
//                
//                CP_move_step = 0;
//                cp_foot_pos_y = tmp_cp_foot_pos_y / 3.0;
//                tmp_cp_cnt = 0;
//                get_cp_done_flag = false;
//                
//                init_CP_y = CP_y;
//            }
//            else {
//                cp_foot_pos_y = 0;
//            }
//
//            if (tmp_y_moving_speed > 0.01 || tmp_y_moving_speed < -0.01) {
//                cp_foot_pos_y = 0;
//                CP_PHASE = 0;
//            }
//            else if (des_theta > 5 * D2R || des_theta < -5 * D2R) {
//                cp_foot_pos_y = 0;
//                CP_PHASE = 0;
//            }
//        }
//        break;
//
//    case 1:
//        cout << "[CP_PHASE 1] RRFL" << endl;
//        if (CP_move_step == 0) {
//            cout << "[CP_move_step == 0]" << endl;
//            if (ft_cnt % ft_step_cnt < ft_step_cnt - 1) {
//                cout << "[1]" << endl;
//                tmp_t = (double) (ft_cnt % ft_step_cnt - 1 - cp_on_cnt) * dt; // now time
//                tmp_cp_time = (double) (ft_step_cnt - cp_on_cnt) * dt; // stepping time
//                
//                cout << "ft_cnt % ft_step_cnt - 1 - cp_on_cnt = " << ft_cnt % ft_step_cnt - 1 - cp_on_cnt << endl;
//                cout << "tmp_t = " << tmp_t << ", tmp_cp_time = " << tmp_cp_time << endl;
//
//                cp_foot_offset_y = Kp_cp * (CP_y - init_CP_y) / 2.0;
//
//                cp_RL_foot_pos[1] = init_cp_RL_foot_pos[1] + (cp_foot_pos_y) / 2.0 * (1 - cos(PI2 / (2 * tmp_cp_time) * tmp_t)) + cp_foot_offset_y;
//                cp_RR_foot_pos[1] = init_cp_RR_foot_pos[1] - (cp_foot_pos_y) / 2.0 * (1 - cos(PI2 / (2 * tmp_cp_time) * tmp_t)) - cp_foot_offset_y;
//                cp_FL_foot_pos[1] = init_cp_FL_foot_pos[1] - (cp_foot_pos_y) / 2.0 * (1 - cos(PI2 / (2 * tmp_cp_time) * tmp_t)) - cp_foot_offset_y;
//                cp_FR_foot_pos[1] = init_cp_FR_foot_pos[1] + (cp_foot_pos_y) / 2.0 * (1 - cos(PI2 / (2 * tmp_cp_time) * tmp_t)) + cp_foot_offset_y;
//            }
//            else {
//                CP_move_step = 1;
//            }
//        }
//        else if (CP_move_step == 1) {
//            cout << "[CP_move_step == 1]" << endl;
//            if (ft_cnt % ft_step_cnt < ft_step_cnt - 1) {
//                cout << "[3]" << endl;
//                tmp_t = (double) (ft_cnt % ft_step_cnt - 1) * dt;
//
//                cp_RL_foot_pos[1] = init_cp_RL_foot_pos[1] + (cp_foot_pos_y + cp_foot_offset_y) - (cp_foot_pos_y + cp_foot_offset_y) / 2.0 * (1 - cos(PI2 / (ft_step_time * 2) * tmp_t));
//                cp_RR_foot_pos[1] = init_cp_RR_foot_pos[1] - (cp_foot_pos_y + cp_foot_offset_y) + (cp_foot_pos_y + cp_foot_offset_y) / 2.0 * (1 - cos(PI2 / (ft_step_time * 2) * tmp_t));
//                cp_FL_foot_pos[1] = init_cp_FL_foot_pos[1] - (cp_foot_pos_y + cp_foot_offset_y) + (cp_foot_pos_y + cp_foot_offset_y) / 2.0 * (1 - cos(PI2 / (ft_step_time * 2) * tmp_t));
//                cp_FR_foot_pos[1] = init_cp_FR_foot_pos[1] + (cp_foot_pos_y + cp_foot_offset_y) - (cp_foot_pos_y + cp_foot_offset_y) / 2.0 * (1 - cos(PI2 / (ft_step_time * 2) * tmp_t));
//
//            }
//            else {
//                CP_move_step = 2;
//            }
//        }
//        else if (CP_move_step == 2) {
//                cout << "[CP_move_step == 2]" << endl;
////            if (ft_cnt % ft_step_cnt == ft_step_cnt - 1) {
////                if (ft_cnt % ft_step_cnt == ft_step_cnt - 1) {
////                    tmp_cp_cnt++;
////
////                        cout << endl << " =========== tmp_cp_cnt = " << tmp_cp_cnt << " ===========" << endl;
//
////                    if (tmp_cp_cnt == 6) { //3
////                            cout << "[7]" << endl;
//                        CP_PHASE = 0;
//                        CP_move_step = 0;
//                        CP_move_done_flag = true;
//                        cp_foot_offset_y = 0;
////                    }
////                }
////            }
//        }
//        break;
//
//    case 2:
//        // swing foot can be located at CP during next phase.
//        cout << "[CP_PHASE 2] RLFR" << endl;
//        if (CP_move_step == 0) {
//            cout << "[CP_move_step == 0]" << endl;
//            if (ft_cnt % ft_step_cnt < ft_step_cnt - 1) {
////                tmp_t = (double) (ft_cnt % ft_step_cnt) * dt;
//                tmp_t = (double) (ft_cnt % ft_step_cnt - 1 - cp_on_cnt) * dt; // now time
//                tmp_cp_time = (double) (ft_step_cnt - cp_on_cnt) * dt; // stepping time
//                
//                cout << "tmp_t = " << tmp_t << ", tmp_cp_time = " << tmp_cp_time << endl;
//
//                cp_foot_offset_y = Kp_cp * (CP_y - init_CP_y) / 2.0;
//
//                cp_RL_foot_pos[1] = init_cp_RL_foot_pos[1] - (cp_foot_pos_y) / 2.0 * (1 - cos(PI2 / (tmp_cp_time * 2) * tmp_t)) - cp_foot_offset_y;
//                cp_RR_foot_pos[1] = init_cp_RR_foot_pos[1] + (cp_foot_pos_y) / 2.0 * (1 - cos(PI2 / (tmp_cp_time * 2) * tmp_t)) + cp_foot_offset_y;
//                cp_FL_foot_pos[1] = init_cp_FL_foot_pos[1] + (cp_foot_pos_y) / 2.0 * (1 - cos(PI2 / (tmp_cp_time * 2) * tmp_t)) + cp_foot_offset_y;
//                cp_FR_foot_pos[1] = init_cp_FR_foot_pos[1] - (cp_foot_pos_y) / 2.0 * (1 - cos(PI2 / (tmp_cp_time * 2) * tmp_t)) - cp_foot_offset_y;
//
////                    printf("[2] cp_foot_offset_y = %f\n", cp_foot_offset_y);
//            }
//            else {
//                CP_move_step = 1;
//            }
//        }
//        else if (CP_move_step == 1) {
//            cout << "[CP_move_step == 1]" << endl;
//            if (ft_cnt % ft_step_cnt < ft_step_cnt - 1) {
//                tmp_t = (double) (ft_cnt % ft_step_cnt - 1) * dt;
////                    cout << "[case.2][CP_move_step.2] tmp_t = " << tmp_t << ", cp_foot_pos_y = " << cp_foot_pos_y << endl;
//
//                cp_RL_foot_pos[1] = init_cp_RL_foot_pos[1] - (cp_foot_pos_y + cp_foot_offset_y) + (cp_foot_pos_y + cp_foot_offset_y) / 2.0 * (1 - cos(PI2 / (ft_step_time * 2) * tmp_t));
//                cp_RR_foot_pos[1] = init_cp_RR_foot_pos[1] + (cp_foot_pos_y + cp_foot_offset_y) - (cp_foot_pos_y + cp_foot_offset_y) / 2.0 * (1 - cos(PI2 / (ft_step_time * 2) * tmp_t));
//                cp_FL_foot_pos[1] = init_cp_FL_foot_pos[1] + (cp_foot_pos_y + cp_foot_offset_y) - (cp_foot_pos_y + cp_foot_offset_y) / 2.0 * (1 - cos(PI2 / (ft_step_time * 2) * tmp_t));
//                cp_FR_foot_pos[1] = init_cp_FR_foot_pos[1] - (cp_foot_pos_y + cp_foot_offset_y) + (cp_foot_pos_y + cp_foot_offset_y) / 2.0 * (1 - cos(PI2 / (ft_step_time * 2) * tmp_t));
//
//            }
//            else {
//                CP_move_step = 2;
//            }
//        }
//        else if (CP_move_step == 2) {
//            cout << "[CP_move_step == 2]" << endl;
////            if (ft_cnt % ft_step_cnt == ft_step_cnt - 1) {
////                if (ft_cnt % ft_step_cnt == ft_step_cnt - 1) {
////                    tmp_cp_cnt++;
//
////                    if (tmp_cp_cnt == 6) { //3
//                        CP_PHASE = 0;
//                        CP_move_step = 0;
//                        CP_move_done_flag = true;
//                        cp_foot_offset_y = 0;
////                    }
////                }
////            }
//        }
//        break;
//    }
//
////    cout << "cp_RL_foot_pos[1] = " << cp_RL_foot_pos[1] << ",cp_RR_foot_pos[1] = " << cp_RR_foot_pos[1] << endl;
//    
//}

//void CRobot::CP_Con_FT(void){
//    static int tmp_cp_cnt = 0;
//    static int cp_on_cnt = 0;
//    static double tmp_cp_time = 0;
//    // ==================== CP CONTROLLER =================== //
//
//    // CP PHASE
//    // 0 : Check CP
//    // 1 : CASE1
//    // 2 : CASE2
//
//    switch (CP_PHASE) {
//    case 0:
//        cout << "[CP_PHASE 0]" << endl;
//        check_CP_FT();
//        
//        if (ft_cnt % ft_step_cnt <= ts_cnt) {
//            if (get_cp_done_flag == true) {
//                if(FC_PHASE == STANCE_RRFL){
//                    CP_PHASE = 1;
//                }
//                else{
//                    CP_PHASE = 2;
//                }
//                cp_on_cnt = ft_cnt % ft_step_cnt;
//                
//                cout << "cp_on_cnt = " << cp_on_cnt << endl;
//                
//                CP_move_step = 0;
//                cp_foot_pos_y = tmp_cp_foot_pos_y / 5.0;
//                tmp_cp_cnt = 0;
//                get_cp_done_flag = false;
//            }
//            else {
//                cp_foot_pos_y = 0;
//            }
//
//            if (tmp_y_moving_speed > 0.01 || tmp_y_moving_speed < -0.01) {
//                cp_foot_pos_y = 0;
//                CP_PHASE = 0;
//            }
//            else if (des_theta > 5 * D2R || des_theta < -5 * D2R) {
//                cp_foot_pos_y = 0;
//                CP_PHASE = 0;
//            }
//        }
//
//        break;
//
//    case 1:
//        cout << "[CP_PHASE 1] RRFL" << endl;
//        if (CP_move_step == 0) {
//            cout << "[CP_move_step == 0]" << endl;
//            if (ft_cnt % ft_step_cnt < ft_step_cnt - 1) {
//                cout << "[1]" << endl;
//                tmp_t = (double) (ft_cnt % ft_step_cnt - 1 - cp_on_cnt) * dt; // now time
//                tmp_cp_time = (double) (ft_step_cnt - 1 - cp_on_cnt) * dt; // stepping time
//                
//                cout << "ft_cnt % ft_step_cnt - 1 = " << ft_cnt % ft_step_cnt - 1 << endl;
//                cout << "tmp_t = " << tmp_t << ", tmp_cp_time = " << tmp_cp_time << endl;
//
//                cp_foot_offset_y = Kp_cp * (CP_y - cp_foot_pos_y * 2) / 2.0;
//                
////                if(cp_foot_pos_y > 0){
////                    cp_RL_foot_pos[1] = init_cp_RL_foot_pos[1] + (cp_foot_pos_y) / 2.0 * (1 - cos(PI2 / (2 * ft_step_time) * tmp_t)) + cp_foot_offset_y;
////                    cp_RR_foot_pos[1] = init_cp_RR_foot_pos[1] - (cp_foot_pos_y) / 2.0 * (1 - cos(PI2 / (2 * ft_step_time) * tmp_t)) - cp_foot_offset_y;
////                    cp_FL_foot_pos[1] = init_cp_FL_foot_pos[1] - (cp_foot_pos_y) / 2.0 * (1 - cos(PI2 / (2 * ft_step_time) * tmp_t)) - cp_foot_offset_y;
////                    cp_FR_foot_pos[1] = init_cp_FR_foot_pos[1] + (cp_foot_pos_y) / 2.0 * (1 - cos(PI2 / (2 * ft_step_time) * tmp_t)) + cp_foot_offset_y;
////                }
////                else{
////                    cp_RL_foot_pos[1] = init_cp_RL_foot_pos[1] + (cp_foot_pos_y) / 2.0 * (1 - cos(PI2 / (2 * ft_step_time) * tmp_t)) + cp_foot_offset_y;
////                    cp_RR_foot_pos[1] = init_cp_RR_foot_pos[1] - (cp_foot_pos_y) / 2.0 * (1 - cos(PI2 / (2 * ft_step_time) * tmp_t)) - cp_foot_offset_y;
////                    cp_FL_foot_pos[1] = init_cp_FL_foot_pos[1] - (cp_foot_pos_y) / 2.0 * (1 - cos(PI2 / (2 * ft_step_time) * tmp_t)) - cp_foot_offset_y;
////                    cp_FR_foot_pos[1] = init_cp_FR_foot_pos[1] + (cp_foot_pos_y) / 2.0 * (1 - cos(PI2 / (2 * ft_step_time) * tmp_t)) + cp_foot_offset_y;
////                }
//
//                cp_RL_foot_pos[1] = init_cp_RL_foot_pos[1] + (cp_foot_pos_y) / 2.0 * (1 - cos(PI2 / (2 * tmp_cp_time) * tmp_t)) + cp_foot_offset_y;
//                cp_RR_foot_pos[1] = init_cp_RR_foot_pos[1] - (cp_foot_pos_y) / 2.0 * (1 - cos(PI2 / (2 * tmp_cp_time) * tmp_t)) - cp_foot_offset_y;
//                cp_FL_foot_pos[1] = init_cp_FL_foot_pos[1] - (cp_foot_pos_y) / 2.0 * (1 - cos(PI2 / (2 * tmp_cp_time) * tmp_t)) - cp_foot_offset_y;
//                cp_FR_foot_pos[1] = init_cp_FR_foot_pos[1] + (cp_foot_pos_y) / 2.0 * (1 - cos(PI2 / (2 * tmp_cp_time) * tmp_t)) + cp_foot_offset_y;
//            }
//            else {
//                CP_move_step = 1;
//            }
//        }
//        else if (CP_move_step == 1) {
//            cout << "[CP_move_step == 1]" << endl;
//            if (ft_cnt % ft_step_cnt < ft_step_cnt - 1) {
//                cout << "[3]" << endl;
//                tmp_t = (double) (ft_cnt % ft_step_cnt - 1) * dt;
//
//                cp_RL_foot_pos[1] = init_cp_RL_foot_pos[1] + (cp_foot_pos_y + cp_foot_offset_y) - (cp_foot_pos_y + cp_foot_offset_y) / 2.0 * (1 - cos(PI2 / (ft_step_time * 2) * tmp_t));
//                cp_RR_foot_pos[1] = init_cp_RR_foot_pos[1] - (cp_foot_pos_y + cp_foot_offset_y) + (cp_foot_pos_y + cp_foot_offset_y) / 2.0 * (1 - cos(PI2 / (ft_step_time * 2) * tmp_t));
//                cp_FL_foot_pos[1] = init_cp_FL_foot_pos[1] - (cp_foot_pos_y + cp_foot_offset_y) + (cp_foot_pos_y + cp_foot_offset_y) / 2.0 * (1 - cos(PI2 / (ft_step_time * 2) * tmp_t));
//                cp_FR_foot_pos[1] = init_cp_FR_foot_pos[1] + (cp_foot_pos_y + cp_foot_offset_y) - (cp_foot_pos_y + cp_foot_offset_y) / 2.0 * (1 - cos(PI2 / (ft_step_time * 2) * tmp_t));
//
//            }
//            else {
//                CP_move_step = 2;
//            }
//        }
//        else if (CP_move_step == 2) {
//                cout << "[CP_move_step == 2]" << endl;
//            if (ft_cnt % ft_step_cnt == ft_step_cnt - 1) {
//                if (ft_cnt % ft_step_cnt == ft_step_cnt - 1) {
//                    tmp_cp_cnt++;
//
//                        cout << endl << " =========== tmp_cp_cnt = " << tmp_cp_cnt << " ===========" << endl;
//
//                    if (tmp_cp_cnt == 6) { //3
////                            cout << "[7]" << endl;
//                        CP_PHASE = 0;
//                        CP_move_step = 0;
//                        CP_move_done_flag = true;
//                    }
//                }
//            }
//        }
//        break;
//
//    case 2:
//        // swing foot can be located at CP during next phase.
//        cout << "[CP_PHASE 2] RLFR" << endl;
//        if (CP_move_step == 0) {
//            cout << "[CP_move_step == 0]" << endl;
//            if (ft_cnt % ft_step_cnt < ft_step_cnt - 1) {
////                tmp_t = (double) (ft_cnt % ft_step_cnt) * dt;
//                tmp_t = (double) (ft_cnt % ft_step_cnt - 1 - cp_on_cnt) * dt; // now time
//                tmp_cp_time = (double) (ft_step_cnt - 1 - cp_on_cnt) * dt; // stepping time
//                
//                cout << "tmp_t = " << tmp_t << ", tmp_cp_time = " << tmp_cp_time << endl;
//
//                cp_foot_offset_y = Kp_cp * (CP_y - cp_foot_pos_y * 2) / 2.0;
//
//                cp_RL_foot_pos[1] = init_cp_RL_foot_pos[1] - (cp_foot_pos_y) / 2.0 * (1 - cos(PI2 / (tmp_cp_time * 2) * tmp_t)) - cp_foot_offset_y;
//                cp_RR_foot_pos[1] = init_cp_RR_foot_pos[1] + (cp_foot_pos_y) / 2.0 * (1 - cos(PI2 / (tmp_cp_time * 2) * tmp_t)) + cp_foot_offset_y;
//                cp_FL_foot_pos[1] = init_cp_FL_foot_pos[1] + (cp_foot_pos_y) / 2.0 * (1 - cos(PI2 / (tmp_cp_time * 2) * tmp_t)) + cp_foot_offset_y;
//                cp_FR_foot_pos[1] = init_cp_FR_foot_pos[1] - (cp_foot_pos_y) / 2.0 * (1 - cos(PI2 / (tmp_cp_time * 2) * tmp_t)) - cp_foot_offset_y;
//
////                    printf("[2] cp_foot_offset_y = %f\n", cp_foot_offset_y);
//            }
//            else {
//                CP_move_step = 1;
//            }
//        }
//        else if (CP_move_step == 1) {
//            cout << "[CP_move_step == 1]" << endl;
//            if (ft_cnt % ft_step_cnt < ft_step_cnt - 1) {
//                tmp_t = (double) (ft_cnt % ft_step_cnt) * dt;
////                    cout << "[case.2][CP_move_step.2] tmp_t = " << tmp_t << ", cp_foot_pos_y = " << cp_foot_pos_y << endl;
//
//                cp_RL_foot_pos[1] = init_cp_RL_foot_pos[1] - (cp_foot_pos_y + cp_foot_offset_y) + (cp_foot_pos_y + cp_foot_offset_y) / 2.0 * (1 - cos(PI2 / (ft_step_time * 2) * tmp_t));
//                cp_RR_foot_pos[1] = init_cp_RR_foot_pos[1] + (cp_foot_pos_y + cp_foot_offset_y) - (cp_foot_pos_y + cp_foot_offset_y) / 2.0 * (1 - cos(PI2 / (ft_step_time * 2) * tmp_t));
//                cp_FL_foot_pos[1] = init_cp_FL_foot_pos[1] + (cp_foot_pos_y + cp_foot_offset_y) - (cp_foot_pos_y + cp_foot_offset_y) / 2.0 * (1 - cos(PI2 / (ft_step_time * 2) * tmp_t));
//                cp_FR_foot_pos[1] = init_cp_FR_foot_pos[1] - (cp_foot_pos_y + cp_foot_offset_y) + (cp_foot_pos_y + cp_foot_offset_y) / 2.0 * (1 - cos(PI2 / (ft_step_time * 2) * tmp_t));
//
//            }
//            else {
//                CP_move_step = 2;
//            }
//        }
//        else if (CP_move_step == 2) {
//            cout << "[CP_move_step == 2]" << endl;
//            if (ft_cnt % ft_step_cnt == ft_step_cnt - 1) {
//                if (ft_cnt % ft_step_cnt == ft_step_cnt - 1) {
//                    tmp_cp_cnt++;
//
//                    if (tmp_cp_cnt == 6) { //3
//                        CP_PHASE = 0;
//                        CP_move_step = 0;
//                        CP_move_done_flag = true;
//                    }
//                }
//            }
//        }
//        break;
//    }
//
//}

void CRobot::COM_SF_FT_X_Traj_Gen()
{
    // =============== COM =============== //
    init_x[0] = pre_com_pos[0];
    init_x[1] = pre_x_moving_speed;
    init_x[2] = 0;

    final_x[0] = pre_com_pos[0] + pre_x_moving_speed * (ts) / 2.0 + x_moving_speed * (ts) / 2.0;
    final_x[1] = x_moving_speed;
    final_x[2] = 0;

    coefficient_5thPoly(init_x, final_x, ts, c_com_x1);

    // ============ Swing Foot Pos. ============ //
    // Left (First)
    init_x[0] = 0;
    init_x[1] = 0;
    init_x[2] = 0;

    final_x[0] = 0 + pre_x_moving_speed * ft_step_time + x_moving_speed * ft_step_time;
    final_x[1] = 0;
    final_x[2] = 0;

    coefficient_5thPoly(init_x, final_x, ts + tf, c_sf_x1);

    // Right (First)
    init_x[0] = 0;
    init_x[1] = 0;
    init_x[2] = 0;

    final_x[0] = 0 + x_moving_speed * ft_step_time * 2;
    final_x[1] = 0;
    final_x[2] = 0;

    coefficient_5thPoly(init_x, final_x, ts + tf, c_sf_x2);

    // Left (Second)
    init_x[0] = 0 + pre_x_moving_speed * ft_step_time + x_moving_speed * ft_step_time;
    init_x[1] = 0;
    init_x[2] = 0;

    final_x[0] = 0 + pre_x_moving_speed * ft_step_time + x_moving_speed * ft_step_time * 3;
    final_x[1] = 0;
    final_x[2] = 0;

    coefficient_5thPoly(init_x, final_x, ts, c_sf_x3);

    // Right (Second)
    init_x[0] = 0 + x_moving_speed * ft_step_time * 2;
    init_x[1] = 0;
    init_x[2] = 0;

    final_x[0] = 0 + x_moving_speed * ft_step_time * 4;
    final_x[1] = 0;
    final_x[2] = 0;

    coefficient_5thPoly(init_x, final_x, ts, c_sf_x4);

}

void CRobot::COM_SF_FT_Z_Traj_Gen()
{
    // =============== COM =============== //
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


    // =============== Foot =============== //

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

void CRobot::FT_Turning_Traj_Gen(void)
{
    // ======= Turning trajectory generation ======= //
    //    printf("================= [FT] Turning trajectory generation ============== \n");

    const double turn_l = 0.4134; // sqrt(0.35^2 + (0.115 + 0.105)^2)
    const double x1 = 0.35, y1 = 0.22, turn_theta1 = 57.8477 * D2R;
    static double x2 = 0.35, y2 = 0.22; //, turn_theta2 = 0;
    static double del_x = 0, del_y = 0;
    static double target_theta = turn_theta1;
    static double tmp_target_theta = 0;

    // turn left
    if ((des_theta > 0.01) * D2R && (ft_cnt == 2 * ft_step_cnt)) {
        if (turn_mode == 0) {
            turn_start_flag = true;
            turn_mode = 1;

            target_theta = -des_theta / 2.0; // [rad]

            printf("[left] target_theta = %f\n", target_theta);
        }
    }
    else if ((des_theta < -0.01) * D2R && (ft_cnt == 1 * ft_step_cnt)) {
        if (turn_mode == 0) {
            turn_start_flag = true;
            turn_mode = 3;

            target_theta = des_theta / 2.0; // [rad]

            printf("[right] target_theta = %f\n", target_theta);
        }
    }

    if (turn_start_flag == true) {
        if (turn_mode == 1) {
            if (turn_cnt <= ts_cnt) {

                tmp_target_theta = turn_theta1 + target_theta / 2.0 * (1 - cos(PI2 / (2 * ts) * (double) turn_cnt * dt));

                x2 = turn_l * sin(tmp_target_theta);
                y2 = turn_l * cos(tmp_target_theta);

                del_x = x2 - x1;
                del_y = y2 - y1;

                turn_xl_EP = del_x;
                turn_yl_EP = del_y;
                turn_xr_EP = del_x;
                turn_yr_EP = -del_y;

                turn_cnt++;
            }
            else {
                tmp_target_theta = turn_theta1 + target_theta;

                x2 = turn_l * sin(tmp_target_theta);
                y2 = turn_l * cos(tmp_target_theta);

                del_x = x2 - x1;
                del_y = y2 - y1;

                turn_xl_EP = del_x;
                turn_yl_EP = del_y;
                turn_xr_EP = del_x;
                turn_yr_EP = -del_y;

                turn_cnt++;
            }

            if (FC_PHASE == STANCE_RRFL) {
                turn_mode = 2;
                turn_cnt = 0;
            }
        }
        else if (turn_mode == 2) {
            if (turn_cnt <= ts_cnt) {

                tmp_target_theta = turn_theta1 + target_theta + (0 - target_theta) / 2.0 * (1 - cos(PI2 / (2 * ts) * (double) turn_cnt * dt));

                x2 = turn_l * sin(tmp_target_theta);
                y2 = turn_l * cos(tmp_target_theta);

                del_x = x2 - x1;
                del_y = y2 - y1;

                turn_xl_EP = del_x;
                turn_yl_EP = del_y;
                turn_xr_EP = del_x;
                turn_yr_EP = -del_y;

                turn_cnt++;
            }
            else {
                tmp_target_theta = turn_theta1; // + target_theta + (0 - target_theta) / 2.0 * (1-cos(PI2 / (2*dsp_time) * (double)turn_cnt*dt));

                x2 = turn_l * sin(tmp_target_theta);
                y2 = turn_l * cos(tmp_target_theta);

                del_x = x2 - x1;
                del_y = y2 - y1;

                turn_xl_EP = del_x;
                turn_yl_EP = del_y;
                turn_xr_EP = del_x;
                turn_yr_EP = -del_y;

                turn_cnt++;

            }

            if (turn_cnt == ts_cnt + tf_cnt - 1) { //FC_PHASE == STOP
                // Initialization
                turn_mode = 0;
                turn_cnt = 0;
                turn_start_flag = false;
            }
        }
        else if (turn_mode == 3) {//) && (ctc_cnt2 < preview_cnt*2)){
            if (turn_cnt <= ts_cnt) {
                tmp_target_theta = turn_theta1 + target_theta / 2.0 * (1 - cos(PI2 / (2 * ts) * (double) turn_cnt * dt));

                x2 = turn_l * sin(tmp_target_theta);
                y2 = turn_l * cos(tmp_target_theta);

                del_x = x2 - x1;
                del_y = y2 - y1;

                turn_xl_EP = del_x;
                turn_yl_EP = del_y;
                turn_xr_EP = del_x;
                turn_yr_EP = -del_y;

                //                            printf("[2] tmp_target_theta = %f\n",tmp_target_theta*R2D);

                turn_cnt++;
            }
            else {
                tmp_target_theta = turn_theta1 + target_theta;

                x2 = turn_l * sin(tmp_target_theta);
                y2 = turn_l * cos(tmp_target_theta);

                del_x = x2 - x1;
                del_y = y2 - y1;

                turn_xl_EP = del_x;
                turn_yl_EP = del_y;
                turn_xr_EP = del_x;
                turn_yr_EP = -del_y;

                turn_cnt++;

            }

            if (FC_PHASE == STANCE_RLFR) {
                turn_mode = 4;
                turn_cnt = 0;
            }
        }
        else if (turn_mode == 4) {
            if (turn_cnt <= ts_cnt) {

                tmp_target_theta = turn_theta1 + target_theta + (0 - target_theta) / 2.0 * (1 - cos(PI2 / (2 * ts) * (double) turn_cnt * dt));

                x2 = turn_l * sin(tmp_target_theta);
                y2 = turn_l * cos(tmp_target_theta);

                del_x = x2 - x1;
                del_y = y2 - y1;

                turn_xl_EP = del_x;
                turn_yl_EP = del_y;
                turn_xr_EP = del_x;
                turn_yr_EP = -del_y;

                turn_cnt++;
            }
            else {
                tmp_target_theta = turn_theta1; // + target_theta + (0 - target_theta) / 2.0 * (1-cos(PI2 / (2*dsp_time) * (double)turn_cnt*dt));

                x2 = turn_l * sin(tmp_target_theta);
                y2 = turn_l * cos(tmp_target_theta);

                del_x = x2 - x1;
                del_y = y2 - y1;

                turn_xl_EP = del_x;
                turn_yl_EP = del_y;
                turn_xr_EP = del_x;
                turn_yr_EP = -del_y;

                turn_cnt++;
            }

            if (turn_cnt == ts_cnt + tf_cnt - 1) { // FC_PHASE == STOP
                // Initialization
                turn_mode = 0;
                turn_cnt = 0;
                turn_start_flag = false;
            }
        }
    }
    else {
        turn_mode = 0;
        turn_cnt = 0;

        turn_xl_EP = 0;
        turn_yl_EP = 0;
        turn_xr_EP = 0;
        turn_yr_EP = 0;
    }

    // ======= Turning trajectory generation  END ======= //
}
















// ====================== Normal trot walking trajectory generation ===================== //

void CRobot::Trot_Walking2(void)
{
    tw_time = (double) tw_cnt*dt;

    //    tmp_x_moving_speed = 0.2;

    if (tw_cnt == 0) {
        // ============ Initialize ============ //
        //        printf("[0] Initialize\n");
        FC_PHASE = STANCE_RLFR;

        moving_done_flag = false;

        com_acc[2] = 0;

        com_pos = init_com_pos;
        RL_foot_pos = init_RL_foot_pos;
        RR_foot_pos = init_RR_foot_pos;
        FL_foot_pos = init_FL_foot_pos;
        FR_foot_pos = init_FR_foot_pos;

        old_com_pos = init_com_pos;

        x_moving_speed = 0;

        SF_TW_Z_Traj_Gen();

    }
    else if (tw_cnt < dsp_cnt) {
        //        printf("[1] First Step (STANCE_RLFR)\n");
        // ============ First Step (STANCE_RLFR) ============= //
        FC_PHASE = STANCE_RLFR;
        tmp_t = tw_time;

        com_pos[0] = init_com_pos(0);
        com_pos[1] = init_com_pos(1);
        com_pos[2] = init_com_pos(2);

        RL_foot_pos = init_RL_foot_pos;

        RR_foot_pos[0] = init_RR_foot_pos[0];
        RR_foot_pos[1] = init_RR_foot_pos[1];

        FL_foot_pos[0] = init_FL_foot_pos[0];
        FL_foot_pos[1] = init_FL_foot_pos[1];

        FR_foot_pos = init_FR_foot_pos;

        if (tw_cnt <= dsp_cnt / 2) {
            RR_foot_pos[2] = init_RR_foot_pos[2] + c_sf_z1[5] * pow(tmp_t, 5) + c_sf_z1[4] * pow(tmp_t, 4) + c_sf_z1[3] * pow(tmp_t, 3) + c_sf_z1[2] * pow(tmp_t, 2) + c_sf_z1[1] * pow(tmp_t, 1) + c_sf_z1[0];
            FL_foot_pos[2] = init_FL_foot_pos[2] + c_sf_z1[5] * pow(tmp_t, 5) + c_sf_z1[4] * pow(tmp_t, 4) + c_sf_z1[3] * pow(tmp_t, 3) + c_sf_z1[2] * pow(tmp_t, 2) + c_sf_z1[1] * pow(tmp_t, 1) + c_sf_z1[0];
        }
        else {
            tmp_t2 = tmp_t - dsp_time / 2.0;
            RR_foot_pos[2] = init_RR_foot_pos[2] + c_sf_z2[5] * pow(tmp_t2, 5) + c_sf_z2[4] * pow(tmp_t2, 4) + c_sf_z2[3] * pow(tmp_t2, 3) + c_sf_z2[2] * pow(tmp_t2, 2) + c_sf_z2[1] * pow(tmp_t2, 1) + c_sf_z2[0];
            FL_foot_pos[2] = init_FL_foot_pos[2] + c_sf_z2[5] * pow(tmp_t2, 5) + c_sf_z2[4] * pow(tmp_t2, 4) + c_sf_z2[3] * pow(tmp_t2, 3) + c_sf_z2[2] * pow(tmp_t2, 2) + c_sf_z2[1] * pow(tmp_t2, 1) + c_sf_z2[0];
        }
        //        printf("RR_foot_pos[2] = %f\n",RR_foot_pos[2]);
    }
    else if (tw_cnt < step_cnt) {
        //        printf("[2] First Step (FSP)\n");
        // ============ First Step (FSP) ============= //
        FC_PHASE = STOP;
        tmp_t = tw_time - dsp_time;

        com_pos[0] = init_com_pos(0);
        com_pos[1] = init_com_pos(1);
        com_pos[2] = init_com_pos(2);

        RL_foot_pos = init_RL_foot_pos;
        RR_foot_pos = init_RR_foot_pos;
        FL_foot_pos = init_FL_foot_pos;
        FR_foot_pos = init_FR_foot_pos;

        if (tw_cnt == step_cnt - 1) {

            pre_x_moving_speed = x_moving_speed;
            x_moving_speed = tmp_x_moving_speed;

            pre_com_pos = com_pos;
            pre_RL_foot_pos = RL_foot_pos;
            pre_RR_foot_pos = RR_foot_pos;
            pre_FL_foot_pos = FL_foot_pos;
            pre_FR_foot_pos = FR_foot_pos;

            COM_SF_TW_X_Traj_Gen();
        }

    }
    else if (tw_cnt < step_cnt + dsp_cnt) {
        //        printf("[3] Second Step (STANCE_RRFL)\n");
        // ============ Second Step (STANCE_RRFL) ============= //
        // ============ Continuous Walking Start ============== //
        FC_PHASE = STANCE_RRFL;
        tmp_t = tw_time - step_time;
        tmp_cnt = tw_cnt - step_cnt;

        com_pos[0] = c_com_x1[5] * pow(tmp_t, 5) + c_com_x1[4] * pow(tmp_t, 4) + c_com_x1[3] * pow(tmp_t, 3) + c_com_x1[2] * pow(tmp_t, 2) + c_com_x1[1] * pow(tmp_t, 1) + c_com_x1[0];
        com_pos[1] = init_com_pos(1);
        com_pos[2] = init_com_pos(2);

        RL_foot_pos[0] = pre_RL_foot_pos[0] + c_sf_x1[5] * pow(tmp_t, 5) + c_sf_x1[4] * pow(tmp_t, 4) + c_sf_x1[3] * pow(tmp_t, 3) + c_sf_x1[2] * pow(tmp_t, 2) + c_sf_x1[1] * pow(tmp_t, 1) + c_sf_x1[0]; //init_RL_foot_pos[0];
        RL_foot_pos[1] = init_RL_foot_pos[1];

        RR_foot_pos = pre_RR_foot_pos;
        FL_foot_pos = pre_FL_foot_pos;

        FR_foot_pos[0] = pre_FR_foot_pos[0] + c_sf_x1[5] * pow(tmp_t, 5) + c_sf_x1[4] * pow(tmp_t, 4) + c_sf_x1[3] * pow(tmp_t, 3) + c_sf_x1[2] * pow(tmp_t, 2) + c_sf_x1[1] * pow(tmp_t, 1) + c_sf_x1[0];
        FR_foot_pos[1] = init_FR_foot_pos[1];

        //        printf("tmp_t = %f, RL_foot_pos[2] = %f\n",tmp_t,RL_foot_pos[2]);
        if (tmp_cnt <= dsp_cnt / 2) {
            //            tmp_t2 = tmp_t;
            RL_foot_pos[2] = init_RL_foot_pos[2] + c_sf_z1[5] * pow(tmp_t, 5) + c_sf_z1[4] * pow(tmp_t, 4) + c_sf_z1[3] * pow(tmp_t, 3) + c_sf_z1[2] * pow(tmp_t, 2) + c_sf_z1[1] * pow(tmp_t, 1) + c_sf_z1[0];
            FR_foot_pos[2] = init_FR_foot_pos[2] + c_sf_z1[5] * pow(tmp_t, 5) + c_sf_z1[4] * pow(tmp_t, 4) + c_sf_z1[3] * pow(tmp_t, 3) + c_sf_z1[2] * pow(tmp_t, 2) + c_sf_z1[1] * pow(tmp_t, 1) + c_sf_z1[0];
        }
        else {
            tmp_t2 = tmp_t - dsp_time / 2.0;
            RL_foot_pos[2] = init_RL_foot_pos[2] + c_sf_z2[5] * pow(tmp_t2, 5) + c_sf_z2[4] * pow(tmp_t2, 4) + c_sf_z2[3] * pow(tmp_t2, 3) + c_sf_z2[2] * pow(tmp_t2, 2) + c_sf_z2[1] * pow(tmp_t2, 1) + c_sf_z2[0];
            FR_foot_pos[2] = init_FR_foot_pos[2] + c_sf_z2[5] * pow(tmp_t2, 5) + c_sf_z2[4] * pow(tmp_t2, 4) + c_sf_z2[3] * pow(tmp_t2, 3) + c_sf_z2[2] * pow(tmp_t2, 2) + c_sf_z2[1] * pow(tmp_t2, 1) + c_sf_z2[0];

            if (tmp_cnt == dsp_cnt - 1) {
                //                cout << "test!!!!!!!!!!!!!!!!!!!!!!!!!1 " << endl;

                pre_com_pos(0) = pre_com_pos[0] + pre_x_moving_speed * (dsp_time) / 2.0 + x_moving_speed * (dsp_time) / 2.0;
                pre_com_pos(1) = com_pos(1);
                pre_com_pos(2) = com_pos(2);

                pre_RL_foot_pos = RL_foot_pos;
                pre_RR_foot_pos = RR_foot_pos;
                pre_FL_foot_pos = FL_foot_pos;
                pre_FR_foot_pos = FR_foot_pos;
            }
        }

    }
    else if (tw_cnt < step_cnt * 2) {
        //        printf("[4] Second Step (FSP)\n");
        // ============ Second Step (FSP) ============= //
        FC_PHASE = STOP;
        tmp_t = tw_time - step_time - dsp_time;

        com_pos[0] = pre_com_pos(0) + x_moving_speed*tmp_t;
        com_pos[1] = pre_com_pos(1);
        com_pos[2] = pre_com_pos(2);

        RL_foot_pos = pre_RL_foot_pos;
        RR_foot_pos = pre_RR_foot_pos;
        FL_foot_pos = pre_FL_foot_pos;
        FR_foot_pos = pre_FR_foot_pos;

        if (tw_cnt == step_cnt * 2 - 1) {

            pre_com_pos(0) = pre_com_pos(0) + x_moving_speed*fsp_time;
            pre_com_pos(1) = com_pos(1);
            pre_com_pos(2) = com_pos(2);
            pre_RL_foot_pos = RL_foot_pos;
            pre_RR_foot_pos = RR_foot_pos;
            pre_FL_foot_pos = FL_foot_pos;
            pre_FR_foot_pos = FR_foot_pos;
        }

    }
    else if (tw_cnt < step_cnt * 2 + dsp_cnt) {
        //        printf("[5] Third Step (STANCE_RLFR)\n");
        // ============ Third Step (STANCE_RLFR) ============= //
        FC_PHASE = STANCE_RLFR;
        tmp_t = tw_time - step_time * 2;
        tmp_cnt = tw_cnt - step_cnt * 2;

        com_pos[0] = pre_com_pos(0) + x_moving_speed*tmp_t;
        com_pos[1] = pre_com_pos(1);
        com_pos[2] = pre_com_pos(2);

        RL_foot_pos = pre_RL_foot_pos;

        RR_foot_pos[0] = pre_RR_foot_pos[0] + c_sf_x2[5] * pow(tmp_t, 5) + c_sf_x2[4] * pow(tmp_t, 4) + c_sf_x2[3] * pow(tmp_t, 3) + c_sf_x2[2] * pow(tmp_t, 2) + c_sf_x2[1] * pow(tmp_t, 1) + c_sf_x2[0];
        RR_foot_pos[1] = init_RR_foot_pos[1];

        FL_foot_pos[0] = pre_FL_foot_pos[0] + c_sf_x2[5] * pow(tmp_t, 5) + c_sf_x2[4] * pow(tmp_t, 4) + c_sf_x2[3] * pow(tmp_t, 3) + c_sf_x2[2] * pow(tmp_t, 2) + c_sf_x2[1] * pow(tmp_t, 1) + c_sf_x2[0];
        FL_foot_pos[1] = init_FL_foot_pos[1];

        FR_foot_pos = pre_FR_foot_pos;

        if (tmp_cnt <= dsp_cnt / 2) {
            RR_foot_pos[2] = init_RR_foot_pos[2] + c_sf_z1[5] * pow(tmp_t, 5) + c_sf_z1[4] * pow(tmp_t, 4) + c_sf_z1[3] * pow(tmp_t, 3) + c_sf_z1[2] * pow(tmp_t, 2) + c_sf_z1[1] * pow(tmp_t, 1) + c_sf_z1[0];
            FL_foot_pos[2] = init_FL_foot_pos[2] + c_sf_z1[5] * pow(tmp_t, 5) + c_sf_z1[4] * pow(tmp_t, 4) + c_sf_z1[3] * pow(tmp_t, 3) + c_sf_z1[2] * pow(tmp_t, 2) + c_sf_z1[1] * pow(tmp_t, 1) + c_sf_z1[0];
        }
        else {
            tmp_t2 = tmp_t - dsp_time / 2.0;
            RR_foot_pos[2] = init_RR_foot_pos[2] + c_sf_z2[5] * pow(tmp_t2, 5) + c_sf_z2[4] * pow(tmp_t2, 4) + c_sf_z2[3] * pow(tmp_t2, 3) + c_sf_z2[2] * pow(tmp_t2, 2) + c_sf_z2[1] * pow(tmp_t2, 1) + c_sf_z2[0];
            FL_foot_pos[2] = init_FL_foot_pos[2] + c_sf_z2[5] * pow(tmp_t2, 5) + c_sf_z2[4] * pow(tmp_t2, 4) + c_sf_z2[3] * pow(tmp_t2, 3) + c_sf_z2[2] * pow(tmp_t2, 2) + c_sf_z2[1] * pow(tmp_t2, 1) + c_sf_z2[0];

            if (tmp_cnt == dsp_cnt - 1) {
                //                cout << "test!!!!!!!!!!!!!!!!!!!!!!!!!1 " << endl;
                pre_com_pos(0) = pre_com_pos(0) + x_moving_speed*dsp_time;
                pre_com_pos(1) = com_pos(1);
                pre_com_pos(2) = com_pos(2);
                pre_RL_foot_pos = RL_foot_pos;
                pre_RR_foot_pos = RR_foot_pos;
                pre_FL_foot_pos = FL_foot_pos;
                pre_FR_foot_pos = FR_foot_pos;
            }
        }

    }
    else if (tw_cnt < step_cnt * 3) {
        //        printf("[6] Third Step (FSP)\n");
        // ============ Third Step (FSP) ============= //
        FC_PHASE = STOP;
        tmp_t = tw_time - step_time * 2 - dsp_time;

        com_pos[0] = pre_com_pos(0) + x_moving_speed*tmp_t;
        com_pos[1] = pre_com_pos(1);
        com_pos[2] = pre_com_pos(2);

        RL_foot_pos = pre_RL_foot_pos;
        RR_foot_pos = pre_RR_foot_pos;
        FL_foot_pos = pre_FL_foot_pos;
        FR_foot_pos = pre_FR_foot_pos;

        if (tw_cnt == step_cnt * 3 - 1) {

            pre_com_pos(0) = pre_com_pos(0) + x_moving_speed*fsp_time;
            pre_com_pos(1) = com_pos(1);
            pre_com_pos(2) = com_pos(2);
            pre_RL_foot_pos = RL_foot_pos;
            pre_RR_foot_pos = RR_foot_pos;
            pre_FL_foot_pos = FL_foot_pos;
            pre_FR_foot_pos = FR_foot_pos;

            pre_x_moving_speed = x_moving_speed;
            x_moving_speed = tmp_x_moving_speed;

            COM_SF_TW_X_Traj_Gen();

        }

        if (tw_cnt == step_cnt * 3 - 1) {
            if (sub_ctrl_flag == false) {
                tw_cnt = step_cnt - 1;
            }
            else {
                // =============== COM =============== //
                init_x[0] = pre_com_pos[0];
                init_x[1] = x_moving_speed;
                init_x[2] = 0;

                final_x[0] = pre_com_pos[0] + x_moving_speed * (dsp_time) / 2.0;
                final_x[1] = 0;
                final_x[2] = 0;

                coefficient_5thPoly(init_x, final_x, dsp_time, c_com_x2);

                // =============== Swing Foot =============== //

                // Left (Second)
                init_x[0] = 0;
                init_x[1] = 0;
                init_x[2] = 0;

                final_x[0] = 0 + x_moving_speed * step_time;
                final_x[1] = 0;
                final_x[2] = 0;

                coefficient_5thPoly(init_x, final_x, dsp_time, c_sf_x5);
            }
        }
        // ============ Continuous Walking End ============== //
    }
    else if (tw_cnt < step_cnt * 3 + dsp_cnt) {
        //        printf("[7] Final step (STANCE_RR_FL)\n");
        // ============ Final step (STANCE_RR_FL)============== //
        FC_PHASE = STANCE_RRFL;
        tmp_t = tw_time - step_time * 3;
        tmp_cnt = tw_cnt - step_cnt * 3;

        com_pos[0] = c_com_x2[5] * pow(tmp_t, 5) + c_com_x2[4] * pow(tmp_t, 4) + c_com_x2[3] * pow(tmp_t, 3) + c_com_x2[2] * pow(tmp_t, 2) + c_com_x2[1] * pow(tmp_t, 1) + c_com_x2[0]; //pre_com_pos(0);
        com_pos[1] = pre_com_pos(1);
        com_pos[2] = init_com_pos(2);

        RL_foot_pos[0] = pre_RL_foot_pos[0] + c_sf_x5[5] * pow(tmp_t, 5) + c_sf_x5[4] * pow(tmp_t, 4) + c_sf_x5[3] * pow(tmp_t, 3) + c_sf_x5[2] * pow(tmp_t, 2) + c_sf_x5[1] * pow(tmp_t, 1) + c_sf_x5[0];
        RL_foot_pos[1] = pre_RL_foot_pos[1];

        RR_foot_pos = pre_RR_foot_pos;
        FL_foot_pos = pre_FL_foot_pos;

        FR_foot_pos[0] = pre_FR_foot_pos[0] + c_sf_x5[5] * pow(tmp_t, 5) + c_sf_x5[4] * pow(tmp_t, 4) + c_sf_x5[3] * pow(tmp_t, 3) + c_sf_x5[2] * pow(tmp_t, 2) + c_sf_x5[1] * pow(tmp_t, 1) + c_sf_x5[0];
        FR_foot_pos[1] = pre_FR_foot_pos[1];


        if (tmp_cnt <= dsp_cnt / 2) {
            //            tmp_t2 = tmp_t;
            RL_foot_pos[2] = init_RL_foot_pos[2] + c_sf_z1[5] * pow(tmp_t, 5) + c_sf_z1[4] * pow(tmp_t, 4) + c_sf_z1[3] * pow(tmp_t, 3) + c_sf_z1[2] * pow(tmp_t, 2) + c_sf_z1[1] * pow(tmp_t, 1) + c_sf_z1[0];
            FR_foot_pos[2] = init_FR_foot_pos[2] + c_sf_z1[5] * pow(tmp_t, 5) + c_sf_z1[4] * pow(tmp_t, 4) + c_sf_z1[3] * pow(tmp_t, 3) + c_sf_z1[2] * pow(tmp_t, 2) + c_sf_z1[1] * pow(tmp_t, 1) + c_sf_z1[0];
        }
        else {
            tmp_t2 = tmp_t - dsp_time / 2.0;
            RL_foot_pos[2] = init_RL_foot_pos[2] + c_sf_z2[5] * pow(tmp_t2, 5) + c_sf_z2[4] * pow(tmp_t2, 4) + c_sf_z2[3] * pow(tmp_t2, 3) + c_sf_z2[2] * pow(tmp_t2, 2) + c_sf_z2[1] * pow(tmp_t2, 1) + c_sf_z2[0];
            FR_foot_pos[2] = init_FR_foot_pos[2] + c_sf_z2[5] * pow(tmp_t2, 5) + c_sf_z2[4] * pow(tmp_t2, 4) + c_sf_z2[3] * pow(tmp_t2, 3) + c_sf_z2[2] * pow(tmp_t2, 2) + c_sf_z2[1] * pow(tmp_t2, 1) + c_sf_z2[0];

            if (tmp_cnt == dsp_cnt - 1) {
                //                cout << "test!!!!!!!!!!!!!!!!!!!!!!!!!1 " << endl;
                //                pre_com_pos = com_pos;
                pre_com_pos(0) = pre_com_pos[0] + x_moving_speed * (dsp_time) / 2.0;
                pre_com_pos(1) = com_pos(1);
                pre_com_pos(2) = com_pos(2);

                pre_RL_foot_pos = RL_foot_pos;
                pre_RR_foot_pos = RR_foot_pos;
                pre_FL_foot_pos = FL_foot_pos;
                pre_FR_foot_pos = FR_foot_pos;
            }
        }


    }
    else if (tw_cnt < step_cnt * 4) {
        // ============ Final step (FSP)============== //
        //        printf("[8] Final step (FSP)\n");
        FC_PHASE = STOP;
        tmp_t = tw_time - step_time * 3 - dsp_time;


        com_pos = pre_com_pos;
        RL_foot_pos = pre_RL_foot_pos;
        RR_foot_pos = pre_RR_foot_pos;
        FL_foot_pos = pre_FL_foot_pos;
        FR_foot_pos = pre_FR_foot_pos;

        if (tw_cnt == step_cnt * 3 + dsp_cnt) {
            moving_done_flag = true;
        }
    }

    if (tw_cnt > 0) {
        target_com_vel[0] = (com_pos[0] - old_com_pos[0]) / dt;
        old_com_pos[0] = com_pos[0];
        if (tw_cnt > 1) {
            target_com_acc[0] = (target_com_vel[0] - old_com_vel[0]) / dt;
            old_com_vel[0] = target_com_vel[0];
        }
    }



    local_RL_foot_pos = RL_foot_pos - com_pos;
    local_RR_foot_pos = RR_foot_pos - com_pos;
    local_FL_foot_pos = FL_foot_pos - com_pos;
    local_FR_foot_pos = FR_foot_pos - com_pos;


    // =========================== Turning Pattern Generation ========================= //

    TW_Turning_Traj_Gen();

    // =========================== Turning Pattern Generation END ========================= //


    // ======================================= Cal CP ====================================== //
    if (CP_con_onoff_flag == true) {
        if (tw_cnt > step_cnt * 8) {
            CP_Con();
        }
        else {
            CP_Con_TW();
        }
    }
    else {
        cp_RL_foot_pos << 0, 0, 0;
        cp_RR_foot_pos << 0, 0, 0;
        cp_FL_foot_pos << 0, 0, 0;
        cp_FR_foot_pos << 0, 0, 0;
    }

    // ======================================= Cal CP END ====================================== //

    target_EP[0] = local_RL_foot_pos[0] - turn_xr_EP + cp_RL_foot_pos[0];
    target_EP[1] = local_RL_foot_pos[1] - turn_yr_EP + cp_RL_foot_pos[1];
    target_EP[2] = local_RL_foot_pos[2] + cp_RL_foot_pos[2];
    target_EP[3] = local_RR_foot_pos[0] - turn_xl_EP + cp_RR_foot_pos[0];
    target_EP[4] = local_RR_foot_pos[1] - turn_yl_EP + cp_RR_foot_pos[1];
    target_EP[5] = local_RR_foot_pos[2] + cp_RR_foot_pos[2];
    target_EP[6] = local_FL_foot_pos[0] + turn_xl_EP + cp_FL_foot_pos[0];
    target_EP[7] = local_FL_foot_pos[1] + turn_yl_EP + cp_FL_foot_pos[1];
    target_EP[8] = local_FL_foot_pos[2] + cp_FL_foot_pos[2];
    target_EP[9] = local_FR_foot_pos[0] + turn_xr_EP + cp_FR_foot_pos[0];
    target_EP[10] = local_FR_foot_pos[1] + turn_yr_EP + cp_FR_foot_pos[1];
    target_EP[11] = local_FR_foot_pos[2] + cp_FR_foot_pos[2];

    target_pos[6] = 0; //goal_pos[6];

    //    printf("tw_cnt = %d\n",tw_cnt);
    tw_cnt++;
}

void CRobot::TW_Turning_Traj_Gen(void)
{
    // ======= Turning trajectory generation ======= //
    //    printf("================= [TW] Turning trajectory generation ============== \n");

    const double turn_l = 0.4134; // sqrt(0.35^2 + (0.115 + 0.105)^2)
    const double x1 = 0.35, y1 = 0.22, turn_theta1 = 57.8477 * D2R;
    static double x2 = 0.35, y2 = 0.22, turn_theta2 = 0;
    static double del_x = 0, del_y = 0;
    static double target_theta = turn_theta1;
    static double tmp_target_theta = 0;

    // turn left
    if ((des_theta > 0.01) * D2R && (tw_cnt == 2 * step_cnt)) {
        if (turn_mode == 0) {
            turn_start_flag = true;
            turn_mode = 1;

            target_theta = -des_theta / 2.0; // [rad]

            printf("[left] target_theta = %f\n", target_theta);
        }
    }
    else if ((des_theta < -0.01) * D2R && (tw_cnt == step_cnt)) {
        if (turn_mode == 0) {
            turn_start_flag = true;
            turn_mode = 3;

            target_theta = des_theta / 2.0; // [rad]

            printf("[right] target_theta = %f\n", target_theta);
        }
    }

    if (turn_start_flag == true) {
        if (turn_mode == 1) {
            if (turn_cnt <= dsp_cnt) {

                tmp_target_theta = turn_theta1 + target_theta / 2.0 * (1 - cos(PI2 / (2 * dsp_time) * (double) turn_cnt * dt));

                x2 = turn_l * sin(tmp_target_theta);
                y2 = turn_l * cos(tmp_target_theta);

                del_x = x2 - x1;
                del_y = y2 - y1;

                turn_xl_EP = del_x;
                turn_yl_EP = del_y;
                turn_xr_EP = del_x;
                turn_yr_EP = -del_y;

                turn_cnt++;
            }
            else {
                tmp_target_theta = turn_theta1 + target_theta;

                x2 = turn_l * sin(tmp_target_theta);
                y2 = turn_l * cos(tmp_target_theta);

                del_x = x2 - x1;
                del_y = y2 - y1;

                turn_xl_EP = del_x;
                turn_yl_EP = del_y;
                turn_xr_EP = del_x;
                turn_yr_EP = -del_y;

                turn_cnt++;
            }

            if (FC_PHASE == STANCE_RRFL) {
                turn_mode = 2;
                turn_cnt = 0;
            }
        }
        else if (turn_mode == 2) {
            if (turn_cnt <= dsp_cnt) {

                tmp_target_theta = turn_theta1 + target_theta + (0 - target_theta) / 2.0 * (1 - cos(PI2 / (2 * dsp_time) * (double) turn_cnt * dt));

                x2 = turn_l * sin(tmp_target_theta);
                y2 = turn_l * cos(tmp_target_theta);

                del_x = x2 - x1;
                del_y = y2 - y1;

                turn_xl_EP = del_x;
                turn_yl_EP = del_y;
                turn_xr_EP = del_x;
                turn_yr_EP = -del_y;

                turn_cnt++;
            }
            else {
                tmp_target_theta = turn_theta1; // + target_theta + (0 - target_theta) / 2.0 * (1-cos(PI2 / (2*dsp_time) * (double)turn_cnt*dt));

                x2 = turn_l * sin(tmp_target_theta);
                y2 = turn_l * cos(tmp_target_theta);

                del_x = x2 - x1;
                del_y = y2 - y1;

                turn_xl_EP = del_x;
                turn_yl_EP = del_y;
                turn_xr_EP = del_x;
                turn_yr_EP = -del_y;

                turn_cnt++;

            }

            if (turn_cnt == step_cnt - 1) { //FC_PHASE == STOP
                // Initialization
                turn_mode = 0;
                turn_cnt = 0;
                turn_start_flag = false;
            }
        }
        else if (turn_mode == 3) {//) && (ctc_cnt2 < preview_cnt*2)){
            if (turn_cnt <= dsp_cnt) {
                tmp_target_theta = turn_theta1 + target_theta / 2.0 * (1 - cos(PI2 / (2 * dsp_time) * (double) turn_cnt * dt));

                x2 = turn_l * sin(tmp_target_theta);
                y2 = turn_l * cos(tmp_target_theta);

                del_x = x2 - x1;
                del_y = y2 - y1;

                turn_xl_EP = del_x;
                turn_yl_EP = del_y;
                turn_xr_EP = del_x;
                turn_yr_EP = -del_y;

                //                            printf("[2] tmp_target_theta = %f\n",tmp_target_theta*R2D);

                turn_cnt++;
            }
            else {
                tmp_target_theta = turn_theta1 + target_theta;

                x2 = turn_l * sin(tmp_target_theta);
                y2 = turn_l * cos(tmp_target_theta);

                del_x = x2 - x1;
                del_y = y2 - y1;

                turn_xl_EP = del_x;
                turn_yl_EP = del_y;
                turn_xr_EP = del_x;
                turn_yr_EP = -del_y;

                turn_cnt++;

            }

            //            printf("FC_PHASE = %d\n",FC_PHASE);
            if (FC_PHASE == STANCE_RLFR) {
                turn_mode = 4;
                turn_cnt = 0;
            }
        }
        else if (turn_mode == 4) {
            if (turn_cnt <= dsp_cnt) {

                tmp_target_theta = turn_theta1 + target_theta + (0 - target_theta) / 2.0 * (1 - cos(PI2 / (2 * dsp_time) * (double) turn_cnt * dt));

                x2 = turn_l * sin(tmp_target_theta);
                y2 = turn_l * cos(tmp_target_theta);

                del_x = x2 - x1;
                del_y = y2 - y1;

                turn_xl_EP = del_x;
                turn_yl_EP = del_y;
                turn_xr_EP = del_x;
                turn_yr_EP = -del_y;

                turn_cnt++;
            }
            else {
                tmp_target_theta = turn_theta1; // + target_theta + (0 - target_theta) / 2.0 * (1-cos(PI2 / (2*dsp_time) * (double)turn_cnt*dt));

                x2 = turn_l * sin(tmp_target_theta);
                y2 = turn_l * cos(tmp_target_theta);

                del_x = x2 - x1;
                del_y = y2 - y1;

                turn_xl_EP = del_x;
                turn_yl_EP = del_y;
                turn_xr_EP = del_x;
                turn_yr_EP = -del_y;

                turn_cnt++;
            }

            if (turn_cnt == step_cnt - 1) { // FC_PHASE == STOP
                // Initialization
                turn_mode = 0;
                turn_cnt = 0;
                turn_start_flag = false;
            }
        }
    }
    else {
        turn_mode = 0;
        turn_cnt = 0;

        turn_xl_EP = 0;
        turn_yl_EP = 0;
        turn_xr_EP = 0;
        turn_yr_EP = 0;
    }

    // ======= Turning trajectory generation  END ======= //
}

void CRobot::COM_SF_TW_X_Traj_Gen(void)
{
    // ============ COM Pos. ========== //
    init_x[0] = pre_com_pos[0];
    init_x[1] = pre_x_moving_speed;
    init_x[2] = 0;

    final_x[0] = pre_com_pos[0] + pre_x_moving_speed * (dsp_time) / 2.0 + x_moving_speed * (dsp_time) / 2.0;
    final_x[1] = x_moving_speed;
    final_x[2] = 0;

    coefficient_5thPoly(init_x, final_x, dsp_time, c_com_x1);

    // ============ Swing Foot Pos. ============ //
    // Left (First)
    init_x[0] = 0;
    init_x[1] = 0;
    init_x[2] = 0;

    final_x[0] = 0 + pre_x_moving_speed * step_time + x_moving_speed * step_time;
    final_x[1] = 0;
    final_x[2] = 0;

    coefficient_5thPoly(init_x, final_x, dsp_time, c_sf_x1);

    // Right (First)
    init_x[0] = 0;
    init_x[1] = 0;
    init_x[2] = 0;

    final_x[0] = 0 + x_moving_speed * step_time * 2;
    final_x[1] = 0;
    final_x[2] = 0;

    coefficient_5thPoly(init_x, final_x, dsp_time, c_sf_x2);

    // Left (Second)
    init_x[0] = 0 + pre_x_moving_speed * step_time + x_moving_speed * step_time;
    init_x[1] = 0;
    init_x[2] = 0;

    final_x[0] = 0 + pre_x_moving_speed * step_time + x_moving_speed * step_time * 3;
    final_x[1] = 0;
    final_x[2] = 0;

    coefficient_5thPoly(init_x, final_x, dsp_time, c_sf_x3);

    // Right (Second)
    init_x[0] = 0 + x_moving_speed * step_time * 2;
    init_x[1] = 0;
    init_x[2] = 0;

    final_x[0] = 0 + x_moving_speed * step_time * 4;
    final_x[1] = 0;
    final_x[2] = 0;

    coefficient_5thPoly(init_x, final_x, dsp_time, c_sf_x4);
}

//void CRobot::COM_TW_Y_Traj_Gen(void)
//{
//
//}
//
//void CRobot::SF_TW_X_Traj_Gen(void)
//{
//
//}

void CRobot::SF_TW_Z_Traj_Gen(void)
{
    init_x[0] = 0;
    init_x[1] = 0;
    init_x[2] = 0;

    final_x[0] = swing_foot_height;
    final_x[1] = 0;
    final_x[2] = 0;

    coefficient_5thPoly(init_x, final_x, dsp_time / 2.0, c_sf_z1);

    init_x[0] = swing_foot_height;
    init_x[1] = 0;
    init_x[2] = 0;

    final_x[0] = 0;
    final_x[1] = 0;
    final_x[2] = 0;

    coefficient_5thPoly(init_x, final_x, dsp_time / 2.0, c_sf_z2);

    //    init_x[0] = swing_foot_height;
    //    init_x[1] = 0;
    //    init_x[2] = 0;
    //
    //    final_x[0] = 0;
    //    final_x[1] = 0;
    //    final_x[2] = 0;
    //
    //    coefficient_5thPoly(init_x, final_x, tf, c_sf_z3);
    //
    //    init_x[0] = swing_foot_height;
    //    init_x[1] = 0;
    //    init_x[2] = 0;
    //
    //    final_x[0] = 0;
    //    final_x[1] = 0;
    //    final_x[2] = 0;
    //
    //    coefficient_5thPoly(init_x, final_x, ts, c_sf_z4);

}

void CRobot::Trot_Walking(void)
{
    x_moving_speed = tmp_x_moving_speed; //0.0;//tmp_moving_speed;
    y_moving_speed = tmp_y_moving_speed; //0.1;

    //    moving_speed = 1.0; // m/s
    //    y_moving_speed = 0;//tmp_moving_speed2;

    //    printf("[1]\n");

    if (ctc_cnt2 < preview_cnt) {
        if (ctc_cnt2 == 0) {
            moving_done_flag = false;
        }
        Trot_Walking_Traj_First(ctc_cnt2);
        stop_flag = false;
        //        printf("[1] done \n");
    }
    else if (ctc_cnt2 < preview_cnt * 2) {

        if (ctc_cnt2 == preview_cnt) {
            pre_com_pos = com_pos;
        }

        Trot_Walking_Traj(ctc_cnt2 - preview_cnt);

        if (ctc_cnt2 == preview_cnt * 2 - 1) {

            if (sub_ctrl_flag != true) {
                stop_flag = false;
                ctc_cnt2 = preview_cnt - 1;
            }
            else {
                stop_flag = true;
                if (traj_stop_flag == true) {
                    ctc_cnt2 = preview_cnt - 1;
                    traj_stop_flag = false;
                }
            }
        }
    }

    else if (ctc_cnt2 < preview_cnt * 3) {
        stop_flag = true;
        x_moving_speed = 0;
        Trot_Walking_Traj_Final(ctc_cnt2 - preview_cnt * 2);
    }
    else {
        moving_done_flag = true;
    }

    local_RL_foot_pos = RL_foot_pos - com_pos;
    local_RR_foot_pos = RR_foot_pos - com_pos;
    local_FL_foot_pos = FL_foot_pos - com_pos;
    local_FR_foot_pos = FR_foot_pos - com_pos;


    //    local_foot_l_pos = foot_l_pos - com_pos; // foot position from global to local
    //    local_foot_r_pos = foot_r_pos - com_pos;

    //    // foot position from global to local
    //    local_foot_l_pos(0) = foot_l_pos(0) - com_pos(0) - init_com_pos(0); 
    //    local_foot_l_pos(1) = foot_l_pos(1) - com_pos(1) - init_com_pos(1);
    //    local_foot_l_pos(2) = foot_l_pos(2) - com_pos(2);
    //    
    //    local_foot_r_pos(0) = foot_r_pos(0) - com_pos(0) - init_com_pos(0);
    //    local_foot_r_pos(1) = foot_r_pos(1) - com_pos(1) - init_com_pos(1);
    //    local_foot_r_pos(2) = foot_r_pos(2) - com_pos(2); 

    Turning_Traj_Gen();

    target_EP[0] = local_RL_foot_pos[0] - turn_xr_EP; // - init_com_pos(0);
    target_EP[1] = local_RL_foot_pos[1] - turn_yr_EP + cp_RL_foot_pos[1]; // - init_com_pos(1); local_foot_l_pos[1] 
    target_EP[2] = local_RL_foot_pos[2];
    target_EP[3] = local_RR_foot_pos[0] - turn_xl_EP; // - init_com_pos(0); 
    target_EP[4] = local_RR_foot_pos[1] - turn_yl_EP + cp_RR_foot_pos[1]; // - init_com_pos(1); 
    target_EP[5] = local_RR_foot_pos[2];
    target_EP[6] = local_FL_foot_pos[0] + turn_xl_EP; // - init_com_pos(0); 
    target_EP[7] = local_FL_foot_pos[1] + turn_yl_EP + cp_FL_foot_pos[1]; // - init_com_pos(1); 
    target_EP[8] = local_FL_foot_pos[2];
    target_EP[9] = local_FR_foot_pos[0] + turn_xr_EP; // - init_com_pos(0); 
    target_EP[10] = local_FR_foot_pos[1] + turn_yr_EP + cp_FR_foot_pos[1]; // - init_com_pos(1); 
    target_EP[11] = local_FR_foot_pos[2];

    ctc_cnt2++;

    target_pos[6] = 0; //goal_pos[6];

}

void CRobot::Turning_Traj_Gen(void)
{
    // ======= Turning trajectory generation ======= //

    const double turn_l = 0.4134; // sqrt(0.35^2 + (0.115 + 0.105)^2)
    const double x1 = 0.35, y1 = 0.22, turn_theta1 = 57.8477 * D2R;
    static double x2 = 0.35, y2 = 0.22, turn_theta2 = 0;
    static double del_x = 0, del_y = 0;
    static double target_theta = turn_theta1;
    static double tmp_target_theta = 0;

    // turn left
    if (des_theta > 0.01 * D2R && (ctc_cnt2 < preview_cnt * 3)) {

        if (turn_mode == 0 && FC_PHASE == STANCE_RLFR) {
            turn_start_flag = true;
            turn_mode = 1;

            target_theta = -des_theta / 2.0; // [rad]

            //            printf("[1] target_theta = %f\n", target_theta * R2D);
        }


    }
    else if (des_theta < -0.01 * D2R && (ctc_cnt2 < preview_cnt * 2)) {

        if (turn_mode == 0 && FC_PHASE == STANCE_RRFL) {
            turn_start_flag = true;
            turn_mode = 3;

            target_theta = des_theta / 2.0; // [rad]

            //            printf("[2] target_theta = %f\n", target_theta * R2D);
        }
    }

    if (turn_start_flag == true) {
        if (turn_mode == 1) {
            if (turn_cnt <= dsp_cnt) {

                tmp_target_theta = turn_theta1 + target_theta / 2.0 * (1 - cos(PI2 / (2 * dsp_time) * (double) turn_cnt * dt));

                x2 = turn_l * sin(tmp_target_theta);
                y2 = turn_l * cos(tmp_target_theta);

                del_x = x2 - x1;
                del_y = y2 - y1;

                turn_xl_EP = del_x;
                turn_yl_EP = del_y;
                turn_xr_EP = del_x;
                turn_yr_EP = -del_y;

                turn_cnt++;
            }
            else {
                tmp_target_theta = turn_theta1 + target_theta;

                x2 = turn_l * sin(tmp_target_theta);
                y2 = turn_l * cos(tmp_target_theta);

                del_x = x2 - x1;
                del_y = y2 - y1;

                turn_xl_EP = del_x;
                turn_yl_EP = del_y;
                turn_xr_EP = del_x;
                turn_yr_EP = -del_y;
            }

            if (FC_PHASE == STANCE_RRFL) {
                turn_mode = 2;
                turn_cnt = 0;
            }
        }
        else if (turn_mode == 2) {
            if (turn_cnt <= dsp_cnt) {

                tmp_target_theta = turn_theta1 + target_theta + (0 - target_theta) / 2.0 * (1 - cos(PI2 / (2 * dsp_time) * (double) turn_cnt * dt));

                x2 = turn_l * sin(tmp_target_theta);
                y2 = turn_l * cos(tmp_target_theta);

                del_x = x2 - x1;
                del_y = y2 - y1;

                turn_xl_EP = del_x;
                turn_yl_EP = del_y;
                turn_xr_EP = del_x;
                turn_yr_EP = -del_y;

                turn_cnt++;
            }
            else {
                tmp_target_theta = turn_theta1; // + target_theta + (0 - target_theta) / 2.0 * (1-cos(PI2 / (2*dsp_time) * (double)turn_cnt*dt));

                x2 = turn_l * sin(tmp_target_theta);
                y2 = turn_l * cos(tmp_target_theta);

                del_x = x2 - x1;
                del_y = y2 - y1;

                turn_xl_EP = del_x;
                turn_yl_EP = del_y;
                turn_xr_EP = del_x;
                turn_yr_EP = -del_y;

            }

            if (FC_PHASE == STOP) {
                // Initialization
                turn_mode = 0;
                turn_cnt = 0;
                turn_start_flag = false;
            }
        }
        else if (turn_mode == 3) {//) && (ctc_cnt2 < preview_cnt*2)){
            if (turn_cnt <= dsp_cnt) {
                tmp_target_theta = turn_theta1 + target_theta / 2.0 * (1 - cos(PI2 / (2 * dsp_time) * (double) turn_cnt * dt));

                x2 = turn_l * sin(tmp_target_theta);
                y2 = turn_l * cos(tmp_target_theta);

                del_x = x2 - x1;
                del_y = y2 - y1;

                turn_xl_EP = del_x;
                turn_yl_EP = del_y;
                turn_xr_EP = del_x;
                turn_yr_EP = -del_y;

                //                            printf("[2] tmp_target_theta = %f\n",tmp_target_theta*R2D);

                turn_cnt++;
            }
            else {
                tmp_target_theta = turn_theta1 + target_theta;

                x2 = turn_l * sin(tmp_target_theta);
                y2 = turn_l * cos(tmp_target_theta);

                del_x = x2 - x1;
                del_y = y2 - y1;

                turn_xl_EP = del_x;
                turn_yl_EP = del_y;
                turn_xr_EP = del_x;
                turn_yr_EP = -del_y;

            }

            if (FC_PHASE == STANCE_RLFR) {
                turn_mode = 4;
                turn_cnt = 0;
            }
        }
        else if (turn_mode == 4) {
            if (turn_cnt <= dsp_cnt) {

                tmp_target_theta = turn_theta1 + target_theta + (0 - target_theta) / 2.0 * (1 - cos(PI2 / (2 * dsp_time) * (double) turn_cnt * dt));

                x2 = turn_l * sin(tmp_target_theta);
                y2 = turn_l * cos(tmp_target_theta);

                del_x = x2 - x1;
                del_y = y2 - y1;

                turn_xl_EP = del_x;
                turn_yl_EP = del_y;
                turn_xr_EP = del_x;
                turn_yr_EP = -del_y;

                turn_cnt++;
            }
            else {
                tmp_target_theta = turn_theta1; // + target_theta + (0 - target_theta) / 2.0 * (1-cos(PI2 / (2*dsp_time) * (double)turn_cnt*dt));

                x2 = turn_l * sin(tmp_target_theta);
                y2 = turn_l * cos(tmp_target_theta);

                del_x = x2 - x1;
                del_y = y2 - y1;

                turn_xl_EP = del_x;
                turn_yl_EP = del_y;
                turn_xr_EP = del_x;
                turn_yr_EP = -del_y;
            }

            if (FC_PHASE == STOP) {
                // Initialization
                turn_mode = 0;
                turn_cnt = 0;
                turn_start_flag = false;
            }
        }
    }
    else {
        turn_mode = 0;
        turn_cnt = 0;
        //        turn_start_flag = false;

        turn_xl_EP = 0;
        turn_yl_EP = 0;
        turn_xr_EP = 0;
        turn_yr_EP = 0;
    }

    // ======= Turning trajectory generation  END ======= //
}

void CRobot::Trot_Walking_Traj_First(unsigned int i)
{
    //    if (i == 0) {
    //
    //        FC_PHASE = STOP;
    //
    //        sum_e << 0, 0;
    //
    //        init_foot_l_2d << init_foot_l_pos(0), init_foot_l_pos(1);
    //        init_foot_r_2d << init_foot_r_pos(0), init_foot_r_pos(1);
    //
    //        Foot_step_planner_first(init_foot_l_2d, init_foot_r_2d);
    //
    //        final_foot_l_2d << foot_l_2d(4, 0), foot_l_2d(4, 1);
    //        final_foot_r_2d << foot_r_2d(4, 0), foot_r_2d(4, 1);
    //
    //        pre_foot_l_2d = foot_l_2d;
    //        pre_foot_r_2d = foot_r_2d;
    //
    //        X_new << 0, 0, 0;
    //        Y_new << 0, 0, 0;
    ////        X_new << init_com_pos(0), 0, 0;
    ////        Y_new << init_com_pos(1), 0, 0;
    //
    //
    //        RL_foot_pos(0) = init_RL_foot_pos(0);
    //        RL_foot_pos(1) = init_RL_foot_pos(1);
    //        RL_foot_pos(2) = init_RL_foot_pos(2);
    //        
    //        RR_foot_pos(0) = init_RR_foot_pos(0);
    //        RR_foot_pos(1) = init_RR_foot_pos(1);
    //        RR_foot_pos(2) = init_RR_foot_pos(2);
    //        
    //        FL_foot_pos(0) = init_FL_foot_pos(0);
    //        FL_foot_pos(1) = init_FL_foot_pos(1);
    //        FL_foot_pos(2) = init_FL_foot_pos(2);
    //        
    //        FR_foot_pos(0) = init_FR_foot_pos(0);
    //        FR_foot_pos(1) = init_FR_foot_pos(1);
    //        FR_foot_pos(2) = init_FR_foot_pos(2);
    //
    ////        foot_r_pos(0) = init_foot_r_pos(0);
    ////        foot_r_pos(1) = init_foot_r_pos(1);
    ////        foot_r_pos(2) = init_foot_r_pos(2);
    //
    //
    //    }
    //    else {
    //        RL_foot_pos(0) = init_RL_foot_pos(0);
    //        RL_foot_pos(1) = init_RL_foot_pos(1);
    //        RL_foot_pos(2) = init_RL_foot_pos(2);
    //        
    //        RR_foot_pos(0) = init_RR_foot_pos(0);
    //        RR_foot_pos(1) = init_RR_foot_pos(1);
    //        RR_foot_pos(2) = init_RR_foot_pos(2);
    //        
    //        FL_foot_pos(0) = init_FL_foot_pos(0);
    //        FL_foot_pos(1) = init_FL_foot_pos(1);
    //        FL_foot_pos(2) = init_FL_foot_pos(2);
    //        
    //        FR_foot_pos(0) = init_FR_foot_pos(0);
    //        FR_foot_pos(1) = init_FR_foot_pos(1);
    //        FR_foot_pos(2) = init_FR_foot_pos(2);
    //
    //        //        printf("[1.4]\n");
    //    }
    //
    //    COM_X_Traj_Gen(i);
    //
    //    //    printf("[1.3]\n");
    //
    //    com_pos(0) = X_new(0);
    //    com_pos(1) = Y_new(0);
    //    com_pos(2) = com_height;
    //    //    tmp_zmp_x_ref = zmp_ref(0);

}

void CRobot::Trot_Walking_Traj(unsigned int i)
{
    //    walk_time = i*dt;
    //
    //    if (i == 0) {
    //        FC_PHASE = STANCE_RRFL;
    //
    //        Foot_step_planner(final_foot_l_2d, final_foot_r_2d);
    //
    //        final_foot_l_2d << foot_l_2d(4, 0), foot_l_2d(4, 1);
    //        final_foot_r_2d << foot_r_2d(4, 0), foot_r_2d(4, 1);
    //
    //        //        cout << "pre_foot_l_2d = " << endl << pre_foot_l_2d << endl;
    //        //        cout << "pre_foot_r_2d = " << endl << pre_foot_r_2d << endl;
    //
    //        SF_Z_Traj_Gen();
    //        SF_X_Traj_Gen();
    //
    //        foot_l_pos(0) = pre_foot_l_2d(0, 0);
    //        foot_l_pos(1) = pre_foot_l_2d(0, 1);
    //        foot_l_pos(2) = init_foot_l_pos(2);
    //
    //        foot_r_pos(0) = pre_foot_r_2d(0, 0);
    //        foot_r_pos(1) = pre_foot_r_2d(0, 1);
    //        foot_r_pos(2) = init_foot_r_pos(2);
    //
    //        pre_zmp_ref_y = tmp_final_zmp_ref_y;
    //
    //
    //        // y axis trajectory generation
    //        init_x[0] = pre_foot_l_2d(0, 1);
    //        init_x[1] = 0;
    //        init_x[2] = 0;
    //
    //        final_x[0] = pre_foot_l_2d(1, 1);
    //        final_x[1] = 0;
    //        final_x[2] = 0;
    //
    //        coefficient_5thPoly(init_x, final_x, dsp_time, y1);
    //
    //        init_x[0] = pre_foot_r_2d(1, 1);
    //        init_x[1] = 0;
    //        init_x[2] = 0;
    //
    //        final_x[0] = pre_foot_r_2d(2, 1);
    //        final_x[1] = 0;
    //        final_x[2] = 0;
    //
    //        coefficient_5thPoly(init_x, final_x, dsp_time, y2);
    //
    //        init_x[0] = pre_foot_l_2d(2, 1);
    //        init_x[1] = 0;
    //        init_x[2] = 0;
    //
    //        final_x[0] = pre_foot_l_2d(3, 1);
    //        final_x[1] = 0;
    //        final_x[2] = 0;
    //
    //        coefficient_5thPoly(init_x, final_x, dsp_time, y3);
    //
    //        init_x[0] = pre_foot_r_2d(3, 1);
    //        init_x[1] = 0;
    //        init_x[2] = 0;
    //
    //        final_x[0] = pre_foot_r_2d(4, 1);
    //        final_x[1] = 0;
    //        final_x[2] = 0;
    //
    //        coefficient_5thPoly(init_x, final_x, dsp_time, y4);
    //
    //
    //    }
    //    else if (i < dsp_cnt) {
    //        FC_PHASE = STANCE_RRFL;
    //
    //        t2 = walk_time;
    //        foot_l_pos(0) = x1[5] * pow(t2, 5) + x1[4] * pow(t2, 4) + x1[3] * pow(t2, 3) + x1[2] * pow(t2, 2) + x1[1] * pow(t2, 1) + x1[0];
    //        foot_l_pos(1) = y1[5] * pow(t2, 5) + y1[4] * pow(t2, 4) + y1[3] * pow(t2, 3) + y1[2] * pow(t2, 2) + y1[1] * pow(t2, 1) + y1[0];
    //
    //        foot_r_pos(0) = pre_foot_r_2d(0, 0);
    //        foot_r_pos(1) = pre_foot_r_2d(0, 1);
    //        foot_r_pos(2) = init_foot_r_pos(2);
    //
    //        if (i < dsp_t1 / dt) {
    //            t1 = t2;
    //            foot_l_pos(2) = init_foot_l_pos(2) + z1[5] * pow(t1, 5) + z1[4] * pow(t1, 4) + z1[3] * pow(t1, 3) + z1[2] * pow(t1, 2) + z1[1] * pow(t1, 1) + z1[0];
    //        }
    //        else {
    //            t1 = t2 - dsp_t1;
    //            foot_l_pos(2) = init_foot_l_pos(2) + z2[5] * pow(t1, 5) + z2[4] * pow(t1, 4) + z2[3] * pow(t1, 3) + z2[2] * pow(t1, 2) + z2[1] * pow(t1, 1) + z2[0];
    //        }
    //    }
    //    else if (i < step_cnt) {
    //        FC_PHASE = STOP;
    //        foot_l_pos(0) = pre_foot_l_2d(1, 0);
    //        foot_l_pos(1) = pre_foot_l_2d(1, 1);
    //        foot_l_pos(2) = init_foot_l_pos(2);
    //
    //        foot_r_pos(0) = pre_foot_r_2d(1, 0);
    //        foot_r_pos(1) = pre_foot_r_2d(1, 1);
    //        foot_r_pos(2) = init_foot_r_pos(2);
    //
    //    }
    //    else if (i < step_cnt + dsp_cnt) {
    //        FC_PHASE = STANCE_RLFR;
    //
    //        t2 = walk_time - step_time;
    //
    //        foot_l_pos(0) = pre_foot_l_2d(1, 0);
    //        foot_l_pos(1) = pre_foot_l_2d(1, 1);
    //        foot_l_pos(2) = init_foot_l_pos(2);
    //
    //        foot_r_pos(0) = x2[5] * pow(t2, 5) + x2[4] * pow(t2, 4) + x2[3] * pow(t2, 3) + x2[2] * pow(t2, 2) + x2[1] * pow(t2, 1) + x2[0];
    //        foot_r_pos(1) = y2[5] * pow(t2, 5) + y2[4] * pow(t2, 4) + y2[3] * pow(t2, 3) + y2[2] * pow(t2, 2) + y2[1] * pow(t2, 1) + y2[0]; //pre_foot_r_2d(1, 1); //init_foot_r_pos(1);
    //
    //        if (i < step_cnt + dsp_t1 / dt) {
    //            t1 = t2;
    //            foot_r_pos(2) = init_foot_r_pos(2) + z1[5] * pow(t1, 5) + z1[4] * pow(t1, 4) + z1[3] * pow(t1, 3) + z1[2] * pow(t1, 2) + z1[1] * pow(t1, 1) + z1[0];
    //        }
    //        else {
    //            t1 = t2 - dsp_t1;
    //            foot_r_pos(2) = init_foot_r_pos(2) + z2[5] * pow(t1, 5) + z2[4] * pow(t1, 4) + z2[3] * pow(t1, 3) + z2[2] * pow(t1, 2) + z2[1] * pow(t1, 1) + z2[0];
    //        }
    //
    //    }
    //
    //    else if (i < step_cnt * 2) {
    //        FC_PHASE = STOP;
    //
    //        foot_l_pos(0) = pre_foot_l_2d(2, 0);
    //        foot_l_pos(1) = pre_foot_l_2d(2, 1);
    //        foot_l_pos(2) = init_foot_l_pos(2);
    //
    //        foot_r_pos(0) = pre_foot_r_2d(2, 0);
    //        foot_r_pos(1) = pre_foot_r_2d(2, 1);
    //        foot_r_pos(2) = init_foot_r_pos(2);
    //
    //    }
    //    else if (i < step_cnt * 2 + dsp_cnt) {
    //        FC_PHASE = STANCE_RRFL;
    //
    //        t2 = walk_time - step_time * 2;
    //
    //        foot_l_pos(0) = x3[5] * pow(t2, 5) + x3[4] * pow(t2, 4) + x3[3] * pow(t2, 3) + x3[2] * pow(t2, 2) + x3[1] * pow(t2, 1) + x3[0];
    //        foot_l_pos(1) = y3[5] * pow(t2, 5) + y3[4] * pow(t2, 4) + y3[3] * pow(t2, 3) + y3[2] * pow(t2, 2) + y3[1] * pow(t2, 1) + y3[0]; //pre_foot_l_2d(2, 1);
    //        foot_l_pos(2) = init_foot_l_pos(2);
    //
    //        foot_r_pos(0) = pre_foot_r_2d(2, 0);
    //        foot_r_pos(1) = pre_foot_r_2d(2, 1);
    //        foot_r_pos(2) = init_foot_r_pos(2);
    //
    //        if (i < step_cnt * 2 + dsp_t1 / dt) {
    //            t1 = t2;
    //            foot_l_pos(2) = init_foot_l_pos(2) + z1[5] * pow(t1, 5) + z1[4] * pow(t1, 4) + z1[3] * pow(t1, 3) + z1[2] * pow(t1, 2) + z1[1] * pow(t1, 1) + z1[0];
    //        }
    //        else {
    //            t1 = t2 - dsp_t1;
    //            foot_l_pos(2) = init_foot_l_pos(2) + z2[5] * pow(t1, 5) + z2[4] * pow(t1, 4) + z2[3] * pow(t1, 3) + z2[2] * pow(t1, 2) + z2[1] * pow(t1, 1) + z2[0];
    //        }
    //
    //    }
    //    else if (i < step_cnt * 3) {
    //        FC_PHASE = STOP;
    //        foot_l_pos(0) = pre_foot_l_2d(3, 0);
    //        foot_l_pos(1) = pre_foot_l_2d(3, 1);
    //        foot_l_pos(2) = init_foot_l_pos(2);
    //
    //        foot_r_pos(0) = pre_foot_r_2d(3, 0);
    //        foot_r_pos(1) = pre_foot_r_2d(3, 1);
    //        foot_r_pos(2) = init_foot_r_pos(2);
    //
    //    }
    //    else if (i < step_cnt * 3 + dsp_cnt) {
    //        FC_PHASE = STANCE_RLFR;
    //
    //        t2 = walk_time - step_time * 3;
    //
    //        foot_l_pos(0) = pre_foot_l_2d(3, 0);
    //        foot_l_pos(1) = pre_foot_l_2d(3, 1);
    //        foot_l_pos(2) = init_foot_l_pos(2);
    //
    //        foot_r_pos(0) = x4[5] * pow(t2, 5) + x4[4] * pow(t2, 4) + x4[3] * pow(t2, 3) + x4[2] * pow(t2, 2) + x4[1] * pow(t2, 1) + x4[0];
    //        foot_r_pos(1) = y4[5] * pow(t2, 5) + y4[4] * pow(t2, 4) + y4[3] * pow(t2, 3) + y4[2] * pow(t2, 2) + y4[1] * pow(t2, 1) + y4[0]; //init_foot_r_pos(1);
    //
    //        if (i < step_cnt * 3 + dsp_t1 / dt) {
    //            t1 = t2;
    //            foot_r_pos(2) = init_foot_r_pos(2) + z1[5] * pow(t1, 5) + z1[4] * pow(t1, 4) + z1[3] * pow(t1, 3) + z1[2] * pow(t1, 2) + z1[1] * pow(t1, 1) + z1[0];
    //        }
    //        else {
    //            t1 = t2 - dsp_t1;
    //            foot_r_pos(2) = init_foot_r_pos(2) + z2[5] * pow(t1, 5) + z2[4] * pow(t1, 4) + z2[3] * pow(t1, 3) + z2[2] * pow(t1, 2) + z2[1] * pow(t1, 1) + z2[0];
    //        }
    //
    //    }
    //
    //    else {
    //        FC_PHASE = STOP;
    //        foot_l_pos(0) = pre_foot_l_2d(4, 0);
    //        foot_l_pos(1) = pre_foot_l_2d(4, 1); //init_foot_l_pos(1);
    //        foot_l_pos(2) = init_foot_l_pos(2);
    //
    //        foot_r_pos(0) = pre_foot_r_2d(4, 0);
    //        foot_r_pos(1) = pre_foot_r_2d(4, 1); //init_foot_r_pos(1);
    //        foot_r_pos(2) = init_foot_r_pos(2);
    //
    //        if (i == step_cnt * 4 - 1) {
    //            pre_foot_l_2d = foot_l_2d;
    //            pre_foot_r_2d = foot_r_2d;
    //        }
    //    }
    //
    //    if (CP_con_onoff_flag == true) {
    //        CP_Con_trot(i);
    //    }
    //    else {
    //        cp_foot_l_3d << 0, 0, 0;
    //        cp_foot_r_3d << 0, 0, 0;
    //    }
    //
    //
    //    COM_X_Traj_Gen(i);
    //
    //    com_pos(0) = X_new(0);
    //    com_pos(1) = Y_new(0);
    //    com_pos(2) = com_height;
    //    //    zmp_ref2 = zmp_ref;

}

void CRobot::Trot_Walking_Traj_Final(unsigned int i)
{
    //    walk_time = i*dt;
    //
    //    if (i == 0) {
    //        FC_PHASE = STANCE_RRFL;
    //
    //        Foot_step_planner(final_foot_l_2d, final_foot_r_2d);
    //
    //        //        cout << "[3]foot_l_2d = " << foot_l_2d << endl;
    //        //        cout << "[3]foot_r_2d = " << foot_r_2d << endl;
    //
    //        SF_X_Traj_Gen_Final();
    //
    //        foot_l_pos(0) = pre_foot_l_2d(0, 0);
    //        foot_l_pos(1) = pre_foot_l_2d(0, 1);
    //        foot_l_pos(2) = init_foot_l_pos(2);
    //
    //        foot_r_pos(0) = pre_foot_r_2d(0, 0);
    //        foot_r_pos(1) = pre_foot_r_2d(0, 1);
    //        foot_r_pos(2) = init_foot_r_pos(2);
    //
    //        pre_zmp_ref_y = tmp_final_zmp_ref_y;
    //
    //        // y axis trajectory generation
    //        init_x[0] = pre_foot_l_2d(0, 1);
    //        init_x[1] = 0;
    //        init_x[2] = 0;
    //
    //        final_x[0] = pre_foot_l_2d(1, 1);
    //        final_x[1] = 0;
    //        final_x[2] = 0;
    //
    //        coefficient_5thPoly(init_x, final_x, dsp_time, y5);
    //
    //    }
    //    else if (i < dsp_cnt) {
    //        FC_PHASE = STANCE_RRFL;
    //        t2 = walk_time;
    //        foot_l_pos(0) = x5[5] * pow(t2, 5) + x5[4] * pow(t2, 4) + x5[3] * pow(t2, 3) + x5[2] * pow(t2, 2) + x5[1] * pow(t2, 1) + x5[0];
    //        foot_l_pos(1) = y5[5] * pow(t2, 5) + y5[4] * pow(t2, 4) + y5[3] * pow(t2, 3) + y5[2] * pow(t2, 2) + y5[1] * pow(t2, 1) + y5[0];
    //
    //        foot_r_pos(0) = pre_foot_r_2d(0, 0);
    //        foot_r_pos(1) = pre_foot_r_2d(0, 1);
    //        foot_r_pos(2) = init_foot_r_pos(2);
    //
    //        if (i < dsp_t1 / dt) {
    //            t1 = t2;
    //            foot_l_pos(2) = init_foot_l_pos(2) + z1[5] * pow(t1, 5) + z1[4] * pow(t1, 4) + z1[3] * pow(t1, 3) + z1[2] * pow(t1, 2) + z1[1] * pow(t1, 1) + z1[0];
    //        }
    //        else {
    //            t1 = t2 - dsp_t1;
    //            foot_l_pos(2) = init_foot_l_pos(2) + z2[5] * pow(t1, 5) + z2[4] * pow(t1, 4) + z2[3] * pow(t1, 3) + z2[2] * pow(t1, 2) + z2[1] * pow(t1, 1) + z2[0];
    //        }
    //    }
    //
    //    else {
    //        FC_PHASE = STOP;
    //        foot_l_pos(0) = pre_foot_l_2d(4, 0);
    //        foot_l_pos(1) = pre_foot_l_2d(4, 1);
    //        foot_l_pos(2) = init_foot_l_pos(2);
    //
    //        foot_r_pos(0) = pre_foot_r_2d(4, 0);
    //        foot_r_pos(1) = pre_foot_r_2d(4, 1);
    //        foot_r_pos(2) = init_foot_r_pos(2);
    //
    //        if (i == step_cnt * 4) {
    //            pre_foot_l_2d = foot_l_2d;
    //            pre_foot_r_2d = foot_r_2d;
    //        }
    //    }
    //
    //    COM_X_Traj_Gen(i);
    //
    //    com_pos(0) = X_new(0); //zmp_x_ref;//com_x_array(i,0);
    //    com_pos(1) = Y_new(0);
    //    com_pos(2) = com_height;
    //    //    tmp_zmp_x_ref = zmp_ref(0); //zmp_x_array(i);

}

void CRobot::check_CP(void)
{
    static double tmp_cp_y_lower_limit = 0.10;
    static double tmp_cp_y_upper_limit = 0.20;

    //    if(Mode == MODE_SIMULATION){
    //        tmp_cp_y_lower_limit = 0.15;
    //        tmp_cp_y_upper_limit = 0.25;
    //    }
    //    else{
    //        tmp_cp_y_lower_limit = 0.07;
    //        tmp_cp_y_upper_limit = 0.10;
    //    }

    if (CP_y > tmp_cp_y_lower_limit) {
        //        cout << "CP_y = " << CP_y << endl;

        if (CP_y < tmp_cp_y_upper_limit) {
            tmp_cp_foot_pos_y = CP_y;
        }
        else {
            tmp_cp_foot_pos_y = tmp_cp_y_upper_limit;
        }

        get_cp_done_flag = true;
    }
    else if (CP_y < -tmp_cp_y_lower_limit) {
        //        cout << "CP_y = " << CP_y << endl;

        if (CP_y > -tmp_cp_y_upper_limit) {
            tmp_cp_foot_pos_y = CP_y;
        }
        else {
            tmp_cp_foot_pos_y = -tmp_cp_y_upper_limit;
        }

        get_cp_done_flag = true;
    }
    else if (get_cp_done_flag == false) {
        tmp_cp_foot_pos_y = 0;
    }
}

void CRobot::CP_Con_TW(void)
{
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
        if (tw_cnt % step_cnt <= dsp_cnt / 2 - 1) {
            // 1. check CP
            if (get_cp_done_flag == true) {
                CP_PHASE = 1; //1;
                cp_foot_pos_y = tmp_cp_foot_pos_y / 4.0; // weighting factor : 0.5
                tmp_cp_cnt = 0;
                cp_on_cnt = tw_cnt % step_cnt;
                get_cp_done_flag = false;
                init_CP_y = CP_y;
            }
            else {
                cp_foot_pos_y = 0;
            }

            if (tmp_y_moving_speed > 0.01 || tmp_y_moving_speed < -0.01) {
                cp_foot_pos_y = 0;
                CP_PHASE = 0;
            }
            else if (des_theta > 5 * D2R || des_theta < -5 * D2R) {
                cp_foot_pos_y = 0;
                CP_PHASE = 0;
            }

            //            printf("cp_foot_pos_y = %f\n",cp_foot_pos_y);
        }
        else if (tw_cnt % step_cnt == step_cnt - 1) { // dsp_cnt or step_cnt
            if (get_cp_done_flag == true) {
                CP_PHASE = 2;
                cp_foot_pos_y = tmp_cp_foot_pos_y / 4.0;
                tmp_cp_cnt = 0;
                get_cp_done_flag = false;
                init_CP_y = CP_y;
            }
            else {
                cp_foot_pos_y = 0;
            }

            if (tmp_y_moving_speed > 0.01 || tmp_y_moving_speed < -0.01) {
                cp_foot_pos_y = 0;
                CP_PHASE = 0;
            }
            else if (des_theta > 5 * D2R || des_theta < -5 * D2R) {
                cp_foot_pos_y = 0;
                CP_PHASE = 0;
            }

            //            printf("cp_foot_pos_y = %f\n",cp_foot_pos_y);
        }

        //        if(tmp_y_moving_speed > 0.05 || tmp_y_moving_speed < -0.05){
        //            cp_foot_pos_y = 0;
        //        }
        //        else if(des_theta > 5*D2R || des_theta < -5*D2R){
        //            cp_foot_pos_y = 0;
        //        }
        //        
        //        printf("cp_foot_pos_y = %f\n",cp_foot_pos_y);

        break;

    case 1:
        // swing foot can be located at CP during current phase.
        //                    cout << "[case 1]" << endl;
        if (CP_move_step == 0) {
            //                cout << "[CP_move_step == 0]" << endl;
            if (tw_cnt % step_cnt < dsp_cnt) {
                //                    cout << "[1]" << endl;
                tmp_t = (double) (tw_cnt % step_cnt - cp_on_cnt) * dt; // now time
                tmp_cp_time = (double) (dsp_cnt - cp_on_cnt) * dt; // stepping time

                //                cout << "tmp_t = " << tmp_t << ", tmp_cp_time = " << tmp_cp_time << ", cp_on_cnt = " << cp_on_cnt << endl;
                //                cout << "[case.1][CP_move_step.1]tmp_t = " << tmp_t << ", cp_foot_pos_y = " << cp_foot_pos_y << endl;

                if (FC_PHASE == STANCE_RRFL) {
                    cp_foot_offset_y = Kp_cp * (CP_y - init_CP_y) / 2.0;

                    cp_RL_foot_pos[1] = init_cp_RL_foot_pos[1] + (cp_foot_pos_y) / 2.0 * (1 - cos(PI2 / (2 * tmp_cp_time) * tmp_t)) + cp_foot_offset_y;
                    cp_RR_foot_pos[1] = init_cp_RR_foot_pos[1] - (cp_foot_pos_y) / 2.0 * (1 - cos(PI2 / (2 * tmp_cp_time) * tmp_t)) - cp_foot_offset_y;
                    cp_FL_foot_pos[1] = init_cp_FL_foot_pos[1] - (cp_foot_pos_y) / 2.0 * (1 - cos(PI2 / (2 * tmp_cp_time) * tmp_t)) - cp_foot_offset_y;
                    cp_FR_foot_pos[1] = init_cp_FR_foot_pos[1] + (cp_foot_pos_y) / 2.0 * (1 - cos(PI2 / (2 * tmp_cp_time) * tmp_t)) + cp_foot_offset_y;
                }
                else if (FC_PHASE == STANCE_RLFR) {
                    cp_foot_offset_y = Kp_cp * (CP_y - init_CP_y) / 2.0;

                    cp_RL_foot_pos[1] = init_cp_RL_foot_pos[1] - (cp_foot_pos_y) / 2.0 * (1 - cos(PI2 / (2 * tmp_cp_time) * tmp_t)) - cp_foot_offset_y;
                    cp_RR_foot_pos[1] = init_cp_RR_foot_pos[1] + (cp_foot_pos_y) / 2.0 * (1 - cos(PI2 / (2 * tmp_cp_time) * tmp_t)) + cp_foot_offset_y;
                    cp_FL_foot_pos[1] = init_cp_FL_foot_pos[1] + (cp_foot_pos_y) / 2.0 * (1 - cos(PI2 / (2 * tmp_cp_time) * tmp_t)) + cp_foot_offset_y;
                    cp_FR_foot_pos[1] = init_cp_FR_foot_pos[1] - (cp_foot_pos_y) / 2.0 * (1 - cos(PI2 / (2 * tmp_cp_time) * tmp_t)) - cp_foot_offset_y;
                }
                //                printf("[1] cp_foot_offset_y = %f, cp_foot_l_3d[1] = %f\n", cp_foot_offset_y, cp_foot_l_3d[1]);
            }
            else {
                //                    cout << "[2]" << endl;
                if (FC_PHASE == STANCE_RRFL) {
                    cp_RL_foot_pos[1] = init_cp_RL_foot_pos[1] + (cp_foot_pos_y) + cp_foot_offset_y;
                    cp_RR_foot_pos[1] = init_cp_RR_foot_pos[1] - (cp_foot_pos_y) - cp_foot_offset_y;
                    cp_FL_foot_pos[1] = init_cp_FL_foot_pos[1] - (cp_foot_pos_y) - cp_foot_offset_y;
                    cp_FR_foot_pos[1] = init_cp_FR_foot_pos[1] + (cp_foot_pos_y) + cp_foot_offset_y;
                }
                else if (FC_PHASE == STANCE_RLFR) {
                    cp_RL_foot_pos[1] = init_cp_RL_foot_pos[1] - (cp_foot_pos_y) - cp_foot_offset_y;
                    cp_RR_foot_pos[1] = init_cp_RR_foot_pos[1] + (cp_foot_pos_y) + cp_foot_offset_y;
                    cp_FL_foot_pos[1] = init_cp_FL_foot_pos[1] + (cp_foot_pos_y) + cp_foot_offset_y;
                    cp_FR_foot_pos[1] = init_cp_FR_foot_pos[1] - (cp_foot_pos_y) - cp_foot_offset_y;
                }

                if (tw_cnt % step_cnt == step_cnt - 1) {
                    CP_move_step = 1;
                }
            }
        }
        else if (CP_move_step == 1) {
            //                            cout << "[CP_move_step == 1]" << endl;
            if (tw_cnt % step_cnt < dsp_cnt) {
                //                    cout << "[3]" << endl;
                tmp_t = (double) (tw_cnt % step_cnt) * dt;
                //                    cout << "tmp_t = " << tmp_t << endl;
                //                    cout << "[case.1][CP_move_step.2] tmp_t = " << tmp_t << ", cp_foot_pos_y = " << cp_foot_pos_y << endl;

                if (FC_PHASE == STANCE_RRFL) {
                    cp_RL_foot_pos[1] = init_cp_RL_foot_pos[1] - (cp_foot_pos_y + cp_foot_offset_y) + (cp_foot_pos_y + cp_foot_offset_y) / 2.0 * (1 - cos(PI2 / (dsp_time * 2) * tmp_t));
                    cp_RR_foot_pos[1] = init_cp_RR_foot_pos[1] + (cp_foot_pos_y + cp_foot_offset_y) - (cp_foot_pos_y + cp_foot_offset_y) / 2.0 * (1 - cos(PI2 / (dsp_time * 2) * tmp_t));
                    cp_FL_foot_pos[1] = init_cp_FL_foot_pos[1] + (cp_foot_pos_y + cp_foot_offset_y) - (cp_foot_pos_y + cp_foot_offset_y) / 2.0 * (1 - cos(PI2 / (dsp_time * 2) * tmp_t));
                    cp_FR_foot_pos[1] = init_cp_FR_foot_pos[1] - (cp_foot_pos_y + cp_foot_offset_y) + (cp_foot_pos_y + cp_foot_offset_y) / 2.0 * (1 - cos(PI2 / (dsp_time * 2) * tmp_t));
                }
                else if (FC_PHASE == STANCE_RLFR) {
                    cp_RL_foot_pos[1] = init_cp_RL_foot_pos[1] + (cp_foot_pos_y + cp_foot_offset_y) - (cp_foot_pos_y + cp_foot_offset_y) / 2.0 * (1 - cos(PI2 / (dsp_time * 2) * tmp_t));
                    cp_RR_foot_pos[1] = init_cp_RR_foot_pos[1] - (cp_foot_pos_y + cp_foot_offset_y) + (cp_foot_pos_y + cp_foot_offset_y) / 2.0 * (1 - cos(PI2 / (dsp_time * 2) * tmp_t));
                    cp_FL_foot_pos[1] = init_cp_FL_foot_pos[1] - (cp_foot_pos_y + cp_foot_offset_y) + (cp_foot_pos_y + cp_foot_offset_y) / 2.0 * (1 - cos(PI2 / (dsp_time * 2) * tmp_t));
                    cp_FR_foot_pos[1] = init_cp_FR_foot_pos[1] + (cp_foot_pos_y + cp_foot_offset_y) - (cp_foot_pos_y + cp_foot_offset_y) / 2.0 * (1 - cos(PI2 / (dsp_time * 2) * tmp_t));
                }
            }
            else {
                //                    cout << "[4]" << endl;
                cp_RL_foot_pos[1] = init_cp_RL_foot_pos[1];
                cp_RR_foot_pos[1] = init_cp_RR_foot_pos[1];
                cp_FL_foot_pos[1] = init_cp_FL_foot_pos[1];
                cp_FR_foot_pos[1] = init_cp_FR_foot_pos[1];

                if (tw_cnt % step_cnt == step_cnt - 1) {
                    CP_move_step = 2;
                    cp_foot_offset_y = 0;
                }
            }
        }
        else if (CP_move_step == 2) {
            //                            cout << "[CP_move_step == 2]" << endl;
            if (tw_cnt % step_cnt == step_cnt - 1) {
                //                    cout << "[5]" << endl;
                if (tw_cnt % step_cnt == step_cnt - 1) {
                    tmp_cp_cnt++;

                    //                        cout << endl << " =========== tmp_cp_cnt = " << tmp_cp_cnt << " ===========" << endl;

                    if (tmp_cp_cnt == 6) { //3
                        //                            cout << "[7]" << endl;
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
        //        cout << "[case 2]" << endl;
        if (CP_move_step == 0) {
            //            cout << "[CP_move_step == 0]" << endl;
            if (tw_cnt % step_cnt < dsp_cnt) {
                tmp_t = (double) (tw_cnt % step_cnt) * dt;

                //                cout << "tmp_t = " << tmp_t << endl;
                //                    cout << "[case.2][CP_move_step.1]tmp_t = " << tmp_t << ", cp_foot_pos_y = " << cp_foot_pos_y << endl;

                if (FC_PHASE == STANCE_RRFL) {
                    cp_foot_offset_y = Kp_cp * (CP_y - init_CP_y) / 2.0;

                    cp_RL_foot_pos[1] = init_cp_RL_foot_pos[1] + (cp_foot_pos_y) / 2.0 * (1 - cos(PI2 / (dsp_time * 2) * tmp_t)) + cp_foot_offset_y;
                    cp_RR_foot_pos[1] = init_cp_RR_foot_pos[1] - (cp_foot_pos_y) / 2.0 * (1 - cos(PI2 / (dsp_time * 2) * tmp_t)) - cp_foot_offset_y;
                    cp_FL_foot_pos[1] = init_cp_FL_foot_pos[1] - (cp_foot_pos_y) / 2.0 * (1 - cos(PI2 / (dsp_time * 2) * tmp_t)) - cp_foot_offset_y;
                    cp_FR_foot_pos[1] = init_cp_FR_foot_pos[1] + (cp_foot_pos_y) / 2.0 * (1 - cos(PI2 / (dsp_time * 2) * tmp_t)) + cp_foot_offset_y;
                }
                else if (FC_PHASE == STANCE_RLFR) {
                    cp_foot_offset_y = Kp_cp * (CP_y - init_CP_y) / 2.0;

                    cp_RL_foot_pos[1] = init_cp_RL_foot_pos[1] - (cp_foot_pos_y) / 2.0 * (1 - cos(PI2 / (dsp_time * 2) * tmp_t)) - cp_foot_offset_y;
                    cp_RR_foot_pos[1] = init_cp_RR_foot_pos[1] + (cp_foot_pos_y) / 2.0 * (1 - cos(PI2 / (dsp_time * 2) * tmp_t)) + cp_foot_offset_y;
                    cp_FL_foot_pos[1] = init_cp_FL_foot_pos[1] + (cp_foot_pos_y) / 2.0 * (1 - cos(PI2 / (dsp_time * 2) * tmp_t)) + cp_foot_offset_y;
                    cp_FR_foot_pos[1] = init_cp_FR_foot_pos[1] - (cp_foot_pos_y) / 2.0 * (1 - cos(PI2 / (dsp_time * 2) * tmp_t)) - cp_foot_offset_y;
                }

                //                cout << "test1 = " << 1 / 2.0 * (1 - cos(PI2 / (dsp_time * 2) * tmp_t)) << endl;
                //                cout << init_cp_RL_foot_pos[1] - (cp_foot_pos_y) / 2.0 * (1 - cos(PI2 / (dsp_time * 2) * tmp_t)) << endl;
                //                cout << - cp_foot_offset_y << endl;

                //                    printf("[2] cp_foot_offset_y = %f\n", cp_foot_offset_y);
            }
            else {
                if (FC_PHASE == STANCE_RRFL) {
                    cp_RL_foot_pos[1] = init_cp_RL_foot_pos[1] + (cp_foot_pos_y + cp_foot_offset_y);
                    cp_RR_foot_pos[1] = init_cp_RR_foot_pos[1] - (cp_foot_pos_y + cp_foot_offset_y);
                    cp_FL_foot_pos[1] = init_cp_FL_foot_pos[1] - (cp_foot_pos_y + cp_foot_offset_y);
                    cp_FR_foot_pos[1] = init_cp_FR_foot_pos[1] + (cp_foot_pos_y + cp_foot_offset_y);
                }
                else if (FC_PHASE == STANCE_RLFR) {
                    cp_RL_foot_pos[1] = init_cp_RL_foot_pos[1] - (cp_foot_pos_y + cp_foot_offset_y);
                    cp_RR_foot_pos[1] = init_cp_RR_foot_pos[1] + (cp_foot_pos_y + cp_foot_offset_y);
                    cp_FL_foot_pos[1] = init_cp_FL_foot_pos[1] + (cp_foot_pos_y + cp_foot_offset_y);
                    cp_FR_foot_pos[1] = init_cp_FR_foot_pos[1] - (cp_foot_pos_y + cp_foot_offset_y);
                }

                if (tw_cnt % step_cnt == step_cnt - 1) {
                    CP_move_step = 1;
                }
            }
        }
        else if (CP_move_step == 1) {
            //                            cout << "[CP_move_step == 1]" << endl;
            if (tw_cnt % step_cnt < dsp_cnt) {
                tmp_t = (double) (tw_cnt % step_cnt) * dt;
                //                    cout << "[case.2][CP_move_step.2] tmp_t = " << tmp_t << ", cp_foot_pos_y = " << cp_foot_pos_y << endl;

                if (FC_PHASE == STANCE_RRFL) {
                    cp_RL_foot_pos[1] = init_cp_RL_foot_pos[1] - (cp_foot_pos_y + cp_foot_offset_y) + (cp_foot_pos_y + cp_foot_offset_y) / 2.0 * (1 - cos(PI2 / (dsp_time * 2) * tmp_t));
                    cp_RR_foot_pos[1] = init_cp_RR_foot_pos[1] + (cp_foot_pos_y + cp_foot_offset_y) - (cp_foot_pos_y + cp_foot_offset_y) / 2.0 * (1 - cos(PI2 / (dsp_time * 2) * tmp_t));
                    cp_FL_foot_pos[1] = init_cp_FL_foot_pos[1] + (cp_foot_pos_y + cp_foot_offset_y) - (cp_foot_pos_y + cp_foot_offset_y) / 2.0 * (1 - cos(PI2 / (dsp_time * 2) * tmp_t));
                    cp_FR_foot_pos[1] = init_cp_FR_foot_pos[1] - (cp_foot_pos_y + cp_foot_offset_y) + (cp_foot_pos_y + cp_foot_offset_y) / 2.0 * (1 - cos(PI2 / (dsp_time * 2) * tmp_t));
                }
                else if (FC_PHASE == STANCE_RLFR) {
                    cp_RL_foot_pos[1] = init_cp_RL_foot_pos[1] + (cp_foot_pos_y + cp_foot_offset_y) - (cp_foot_pos_y + cp_foot_offset_y) / 2.0 * (1 - cos(PI2 / (dsp_time * 2) * tmp_t));
                    cp_RR_foot_pos[1] = init_cp_RR_foot_pos[1] - (cp_foot_pos_y + cp_foot_offset_y) + (cp_foot_pos_y + cp_foot_offset_y) / 2.0 * (1 - cos(PI2 / (dsp_time * 2) * tmp_t));
                    cp_FL_foot_pos[1] = init_cp_FL_foot_pos[1] - (cp_foot_pos_y + cp_foot_offset_y) + (cp_foot_pos_y + cp_foot_offset_y) / 2.0 * (1 - cos(PI2 / (dsp_time * 2) * tmp_t));
                    cp_FR_foot_pos[1] = init_cp_FR_foot_pos[1] + (cp_foot_pos_y + cp_foot_offset_y) - (cp_foot_pos_y + cp_foot_offset_y) / 2.0 * (1 - cos(PI2 / (dsp_time * 2) * tmp_t));
                }
            }
            else {
                cp_RL_foot_pos[1] = init_cp_RL_foot_pos[1];
                cp_RR_foot_pos[1] = init_cp_RR_foot_pos[1];
                cp_FL_foot_pos[1] = init_cp_FL_foot_pos[1];
                cp_FR_foot_pos[1] = init_cp_FR_foot_pos[1];

                if (tw_cnt % step_cnt == step_cnt - 1) {
                    CP_move_step = 2;
                    cp_foot_offset_y = 0;
                }
            }
        }
        else if (CP_move_step == 2) {
            //                            cout << "[CP_move_step == 2]" << endl;
            if (tw_cnt % step_cnt == step_cnt - 1) {
                if (tw_cnt % step_cnt == step_cnt - 1) {
                    tmp_cp_cnt++;

                    if (tmp_cp_cnt == 6) { //3
                        CP_PHASE = 0;
                        CP_move_step = 0;
                        CP_move_done_flag = true;
                    }
                }
            }
        }
        break;
    }

    //        cout << "cp_RL_foot_pos[1] = "<< cp_RL_foot_pos[1] << endl;
}

void CRobot::Body_Ori_Con2(void)
{
    static double sum_roll_err = 0., sum_pitch_err = 0.;
    static double del_L_left = 0, del_L_right = 0, del_L_front = 0, del_L_rear = 0; //, del_L_rl = 0, del_L_rr = 0;
    const double limit_foot_z = 0.03;
    //    const double IMURoll_alpha = 1, IMUPitch_alpha = 1;
    static double lpf_IMURoll = 0, lpf_IMUPitch = 0;

    // ==================== Slop Compensation Control ==================== //

    // roll
    //    lpf_IMURoll = (1 - IMURoll_alpha) * lpf_IMURoll + IMURoll_alpha*IMURoll;

    lpf_IMURoll = IMURoll;

    if (del_L_left < limit_foot_z && del_L_left > -limit_foot_z) {
        sum_roll_err = sum_roll_err + (0 - lpf_IMURoll) * dt;
    }

    del_L_left = BOC_Kp_roll * (0 - lpf_IMURoll) + BOC_Ki_roll*sum_roll_err;
    del_L_right = -BOC_Kp_roll * (0 - lpf_IMURoll) - BOC_Ki_roll*sum_roll_err;

    // pitch
    //    lpf_IMUPitch = (1 - IMUPitch_alpha) * lpf_IMUPitch + IMUPitch_alpha*IMUPitch;
    lpf_IMUPitch = IMUPitch;

    if (del_L_front < limit_foot_z && del_L_front > -limit_foot_z) {
        sum_pitch_err = sum_pitch_err + (des_pitch_deg - lpf_IMUPitch) * dt;
    }

    del_L_front = BOC_Kp_pitch * (des_pitch_deg - lpf_IMUPitch) + BOC_Ki_pitch*sum_pitch_err;
    del_L_rear = -BOC_Kp_pitch * (des_pitch_deg - lpf_IMUPitch) - BOC_Ki_pitch*sum_pitch_err;

    //    cout << "del_L_front = " << del_L_front << ", del_L_rear = " << del_L_rear << endl;

    //    printf("lpf_IMUPitch = %f, del_L_front = %f\n",lpf_IMUPitch,del_L_front);

    target_EP_offset[2] = -del_L_left + del_L_rear;
    target_EP_offset[5] = -del_L_right + del_L_rear;
    target_EP_offset[8] = -del_L_left + del_L_front;
    target_EP_offset[11] = -del_L_right + del_L_front;

    //    printf("target_EP_offset[2] = %f\n",target_EP_offset[2]);


    if (target_EP_offset[2] > limit_foot_z) {
        target_EP_offset[2] = limit_foot_z;
    }
    else if (target_EP_offset[2] < -limit_foot_z) {
        target_EP_offset[2] = -limit_foot_z;
    }

    if (target_EP_offset[5] > limit_foot_z) {
        target_EP_offset[5] = limit_foot_z;
    }
    else if (target_EP_offset[5] < -limit_foot_z) {
        target_EP_offset[5] = -limit_foot_z;
    }

    if (target_EP_offset[8] > limit_foot_z) {
        target_EP_offset[8] = limit_foot_z;
    }
    else if (target_EP_offset[8] < -limit_foot_z) {
        target_EP_offset[8] = -limit_foot_z;
    }

    if (target_EP_offset[11] > limit_foot_z) {
        target_EP_offset[11] = limit_foot_z;
    }
    else if (target_EP_offset[11] < -limit_foot_z) {
        target_EP_offset[11] = -limit_foot_z;
    }
}

void CRobot::SF_X_Traj_Gen_Final(void)
{
    init_x[0] = pre_foot_l_2d(0, 0);
    init_x[1] = 0;
    init_x[2] = 0;

    final_x[0] = pre_foot_l_2d(1, 0);
    final_x[1] = 0;
    final_x[2] = 0;

    coefficient_5thPoly(init_x, final_x, dsp_time, x5);

}

void CRobot::SF_X_Traj_Gen(void)
{
    init_x[0] = pre_foot_l_2d(0, 0);
    init_x[1] = 0;
    init_x[2] = 0;

    final_x[0] = pre_foot_l_2d(1, 0);
    final_x[1] = 0;
    final_x[2] = 0;

    coefficient_5thPoly(init_x, final_x, dsp_time, x1);

    init_x[0] = pre_foot_r_2d(1, 0);
    init_x[1] = 0;
    init_x[2] = 0;

    final_x[0] = pre_foot_r_2d(2, 0);
    final_x[1] = 0;
    final_x[2] = 0;

    coefficient_5thPoly(init_x, final_x, dsp_time, x2);

    init_x[0] = pre_foot_l_2d(2, 0);
    init_x[1] = 0;
    init_x[2] = 0;

    final_x[0] = pre_foot_l_2d(3, 0);
    final_x[1] = 0;
    final_x[2] = 0;

    coefficient_5thPoly(init_x, final_x, dsp_time, x3);

    init_x[0] = pre_foot_r_2d(3, 0);
    init_x[1] = 0;
    init_x[2] = 0;

    final_x[0] = pre_foot_r_2d(4, 0);
    final_x[1] = 0;
    final_x[2] = 0;

    coefficient_5thPoly(init_x, final_x, dsp_time, x4);
}

void CRobot::SF_Z_Traj_Gen(void)
{
    dsp_t1 = dsp_time / 2.0f;
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

void CRobot::Foot_step_planner(VectorNd init_foot_l_2d, VectorNd init_foot_r_2d)
{
    x_dist = x_moving_speed*step_time;
    y_dist = y_moving_speed*step_time;

    if (stop_flag == false) {
        foot_l_2d << init_foot_l_2d[0], init_foot_l_2d[1],
                init_foot_r_2d[0] + x_dist, init_foot_r_2d[1] + y_dist + 0.105 * 2,
                init_foot_r_2d[0] + x_dist, init_foot_r_2d[1] + y_dist + 0.105 * 2,
                init_foot_r_2d[0] + x_dist * 3, init_foot_r_2d[1] + y_dist * 3 + 0.105 * 2,
                init_foot_r_2d[0] + x_dist * 3, init_foot_r_2d[1] + y_dist * 3 + 0.105 * 2;

        foot_r_2d << init_foot_r_2d[0], init_foot_r_2d[1],
                init_foot_r_2d[0], init_foot_r_2d[1],
                init_foot_r_2d[0] + x_dist * 2, init_foot_r_2d[1] + y_dist * 2,
                init_foot_r_2d[0] + x_dist * 2, init_foot_r_2d[1] + y_dist * 2,
                init_foot_r_2d[0] + x_dist * 4, init_foot_r_2d[1] + y_dist * 4;

    }
    else {
        foot_l_2d << init_foot_l_2d[0], init_foot_l_2d[1],
                init_foot_l_2d[0] + x_dist, init_foot_l_2d[1] + y_dist,
                init_foot_l_2d[0] + x_dist, init_foot_l_2d[1] + y_dist,
                init_foot_l_2d[0] + x_dist, init_foot_l_2d[1] + y_dist,
                init_foot_l_2d[0] + x_dist, init_foot_l_2d[1] + y_dist;

        foot_r_2d << init_foot_r_2d[0], init_foot_r_2d[1],
                init_foot_r_2d[0], init_foot_r_2d[1],
                init_foot_r_2d[0], init_foot_r_2d[1],
                init_foot_r_2d[0], init_foot_r_2d[1],
                init_foot_r_2d[0], init_foot_r_2d[1];
    }
}

void CRobot::Foot_step_planner_first(VectorNd init_foot_l_2d, VectorNd init_foot_r_2d)
{
    x_dist = x_moving_speed*step_time;
    y_dist = y_moving_speed*step_time;

    foot_l_2d << init_foot_l_2d[0], init_foot_l_2d[1],
            init_foot_l_2d[0] + x_dist, init_foot_l_2d[1] + y_dist,
            init_foot_l_2d[0] + x_dist, init_foot_l_2d[1] + y_dist,
            init_foot_l_2d[0] + x_dist * 3, init_foot_l_2d[1] + y_dist * 3,
            init_foot_l_2d[0] + x_dist * 3, init_foot_l_2d[1] + y_dist * 3;

    foot_r_2d << init_foot_r_2d[0], init_foot_r_2d[1],
            init_foot_r_2d[0], init_foot_r_2d[1],
            init_foot_r_2d[0] + x_dist * 2, init_foot_r_2d[1] + y_dist * 2,
            init_foot_r_2d[0] + x_dist * 2, init_foot_r_2d[1] + y_dist * 2,
            init_foot_r_2d[0] + x_dist * 4, init_foot_r_2d[1] + y_dist * 4;
}

void CRobot::COM_X_Traj_Gen(unsigned int i)
{
    init_zmp_ref_y = pre_zmp_ref_y;

    if (stop_flag == false) {
        if (i <= dsp_cnt) {
            tmp_zmp_ref(0) = foot_r_2d(0, 0);
            tmp_zmp_ref(1) = init_zmp_ref_y;
        }
        else if (i <= step_cnt) {
            tmp_zmp_ref(0) = foot_l_2d(1, 0);
            tmp_zmp_ref(1) = init_zmp_ref_y + y_dist;
        }
        else if (i <= step_cnt + dsp_cnt) {
            tmp_zmp_ref(0) = foot_l_2d(1, 0);
            tmp_zmp_ref(1) = init_zmp_ref_y + y_dist;
        }
        else if (i <= step_cnt * 2) {
            tmp_zmp_ref(0) = foot_r_2d(2, 0);
            tmp_zmp_ref(1) = init_zmp_ref_y + y_dist * 2;
        }
        else if (i <= step_cnt * 2 + dsp_cnt) {
            tmp_zmp_ref(0) = foot_r_2d(2, 0);
            tmp_zmp_ref(1) = init_zmp_ref_y + y_dist * 2;
        }
        else if (i <= step_cnt * 3) {
            tmp_zmp_ref(0) = foot_l_2d(3, 0);
            tmp_zmp_ref(1) = init_zmp_ref_y + y_dist * 3;
        }
        else if (i <= step_cnt * 3 + dsp_cnt) {
            tmp_zmp_ref(0) = foot_l_2d(3, 0);
            tmp_zmp_ref(1) = init_zmp_ref_y + y_dist * 3;
        }
        else {
            tmp_zmp_ref(0) = foot_r_2d(4, 0);
            tmp_zmp_ref(1) = init_zmp_ref_y + y_dist * 4;

            if (i == step_cnt * 4 - 1) {
                tmp_final_zmp_ref_y = tmp_zmp_ref(1);
            }
        }
    }
    else {
        if (i <= dsp_cnt) {
            tmp_zmp_ref(0) = foot_r_2d(0, 0);
            tmp_zmp_ref(1) = init_zmp_ref_y;
        }
        else if (i <= step_cnt) {
            tmp_zmp_ref(0) = foot_l_2d(1, 0);
            tmp_zmp_ref(1) = init_zmp_ref_y;
        }
        else if (i <= step_cnt + dsp_cnt) {
            tmp_zmp_ref(0) = foot_l_2d(1, 0);
            tmp_zmp_ref(1) = init_zmp_ref_y;
        }
        else if (i <= step_cnt * 2) {
            tmp_zmp_ref(0) = foot_r_2d(2, 0);
            tmp_zmp_ref(1) = init_zmp_ref_y;
        }
        else if (i <= step_cnt * 2 + dsp_cnt) {
            tmp_zmp_ref(0) = foot_r_2d(2, 0);
            tmp_zmp_ref(1) = init_zmp_ref_y;
        }
        else if (i <= step_cnt * 3) {
            tmp_zmp_ref(0) = foot_l_2d(3, 0);
            tmp_zmp_ref(1) = init_zmp_ref_y;
        }
        else if (i <= step_cnt * 3 + dsp_cnt) {
            tmp_zmp_ref(0) = foot_l_2d(3, 0);
            tmp_zmp_ref(1) = init_zmp_ref_y;
        }
        else {
            tmp_zmp_ref(0) = foot_r_2d(4, 0);
            tmp_zmp_ref(1) = init_zmp_ref_y;

            if (i == step_cnt * 4 - 1) {
                tmp_final_zmp_ref_y = tmp_zmp_ref(1);
            }
        }
    }

    for (unsigned int j = 0; j < 2; ++j) {
        for (unsigned int k = 0; k <= preview_cnt - 2; ++k) {
            zmp_ref_array(k, j) = zmp_ref_array(k + 1, j);
        }
        zmp_ref_array(preview_cnt - 1, j) = tmp_zmp_ref(j);
    }

    Preview_con_2d();

    zmp_ref = zmp_ref_array.block<1, 2>(0, 0); //(0);

}

void CRobot::Preview_con_2d(void)
{
    static double u_x = 0, u_y = 0;

    zmp_ref_old(0) = CC.transpose() * X_new;
    zmp_ref_old(1) = CC.transpose() * Y_new;

    pre_err = zmp_ref_old - zmp_ref_array.block<1, 2>(0, 0).transpose();
    sum_e = sum_e + pre_err;
    pre_sum_p << 0, 0;

    for (unsigned int j = 0; j < 2; ++j) {
        for (unsigned int k = 0; k < preview_cnt - 1; ++k) {
            pre_sum_p[j] = pre_sum_p[j] + Gp(k) * zmp_ref_array(k, j);
        }
    }

    u_x = -Gi * sum_e(0) - Gx.transpose() * X_new - pre_sum_p(0);
    u_y = -Gi * sum_e(1) - Gx.transpose() * Y_new - pre_sum_p(1);

    X_new = AA * X_new + BB*u_x;
    Y_new = AA * Y_new + BB*u_y;
}

void CRobot::coefficient_5thPoly(double *init_x, double *final_x, double tf, double *output)
{
    R << init_x[0], final_x[0], init_x[1], final_x[1], init_x[2], final_x[2];

    temp_t1 = 0;
    temp_t2 = tf;

    A << 1, temp_t1, pow(temp_t1, 2), pow(temp_t1, 3), pow(temp_t1, 4), pow(temp_t1, 5),
            1, temp_t2, pow(temp_t2, 2), pow(temp_t2, 3), pow(temp_t2, 4), pow(temp_t2, 5),
            0, 1, 2 * pow(temp_t1, 1), 3 * pow(temp_t1, 2), 4 * pow(temp_t1, 3), 5 * pow(temp_t1, 4),
            0, 1, 2 * pow(temp_t2, 1), 3 * pow(temp_t2, 2), 4 * pow(temp_t2, 3), 5 * pow(temp_t2, 4),
            0, 0, 2, 6 * pow(temp_t1, 1), 12 * pow(temp_t1, 2), 20 * pow(temp_t1, 3),
            0, 0, 2, 6 * pow(temp_t2, 1), 12 * pow(temp_t2, 2), 20 * pow(temp_t2, 3);

    P = A.inverse() * R;

    output[0] = P(0, 0);
    output[1] = P(1, 0);
    output[2] = P(2, 0);
    output[3] = P(3, 0);
    output[4] = P(4, 0);
    output[5] = P(5, 0);
}

void CRobot::Cal_Fc(void)
{
    static double fc_cnt = 0;

    static double tmp_fc_time = 0.01;
    static int tmp_fc_cnt = 10;
    static double tmp_Fc_RL_z = 0, tmp_Fc_RR_z = 0, tmp_Fc_FL_z = 0, tmp_Fc_FR_z = 0;
    //    static int pr_cnt = 0;
    //    static double init_Fc_RL_z = 0, init_Fc_RR_z = 0, init_Fc_FL_z = 0, init_Fc_FR_z = 0;

    if (FC_PHASE == STOP) {

        if (CommandFlag != PRONK_JUMP) {
            Fc_RL_z = tar_Fc_RL;
            Fc_RR_z = tar_Fc_RR;
            Fc_FL_z = tar_Fc_FL;
            Fc_FR_z = tar_Fc_FR;
        }
        else {
            Fc_RL_z = tar_Fc_RL + 12 * com_acc[2];
            Fc_RR_z = tar_Fc_RR + 12 * com_acc[2];
            Fc_FL_z = tar_Fc_FL + 12 * com_acc[2];
            Fc_FR_z = tar_Fc_FR + 12 * com_acc[2];
        }

        fc_cnt = 0;
    }
    else if (FC_PHASE == INIT_Fc) { // walk ready
        Fc_RL_z = tar_Fc_RL / 2.0 * (1 - cos(PI2 / (home_pos_time * 2)*(double) (ctc_cnt) * dt));
        Fc_RR_z = tar_Fc_RR / 2.0 * (1 - cos(PI2 / (home_pos_time * 2)*(double) (ctc_cnt) * dt));
        Fc_FL_z = tar_Fc_FL / 2.0 * (1 - cos(PI2 / (home_pos_time * 2)*(double) (ctc_cnt) * dt));
        Fc_FR_z = tar_Fc_FR / 2.0 * (1 - cos(PI2 / (home_pos_time * 2)*(double) (ctc_cnt) * dt));

    }
    else if (FC_PHASE == STANCE_RLFR) {

        if (CommandFlag == FLYING_TROT_RUNNING) {

            if (flying_trot_init_flag == true) {
                if (fc_cnt <= tmp_fc_cnt) {
                    tmp_Fc_RL_z = tar_Fc_RL + (tar_Fc_RL)*0.5 * (1 - cos(PI2 / (tmp_fc_time * 2)*(double) (fc_cnt) * dt));
                    tmp_Fc_RR_z = tar_Fc_RR - tar_Fc_RR * 0.5 * (1 - cos(PI2 / (tmp_fc_time * 2)*(double) (fc_cnt) * dt));
                    tmp_Fc_FL_z = tar_Fc_FL - tar_Fc_FL * 0.5 * (1 - cos(PI2 / (tmp_fc_time * 2)*(double) (fc_cnt) * dt));
                    tmp_Fc_FR_z = tar_Fc_FR + (tar_Fc_FR)*0.5 * (1 - cos(PI2 / (tmp_fc_time * 2)*(double) (fc_cnt) * dt));
                }
                else if (fc_cnt >= ts_cnt - tmp_fc_cnt) {
                    tmp_Fc_RL_z = tar_Fc_RL * 2 + (0 - tar_Fc_RL * 2)*0.5 * (1 - cos(PI2 / (tmp_fc_time * 2)*(double) (fc_cnt - (ts_cnt - tmp_fc_cnt)) * dt));
                    tmp_Fc_RR_z = 0;
                    tmp_Fc_FL_z = 0;
                    tmp_Fc_FR_z = tar_Fc_FR * 2 + (0 - tar_Fc_FR * 2)*0.5 * (1 - cos(PI2 / (tmp_fc_time * 2)*(double) (fc_cnt - (ts_cnt - tmp_fc_cnt)) * dt));
                }
            }
            else {
                if (fc_cnt <= tmp_fc_cnt) {
                    tmp_Fc_RL_z = tar_Fc_RL * 2 * 0.5 * (1 - cos(PI2 / (tmp_fc_time * 2)*(double) (fc_cnt) * dt));
                    tmp_Fc_RR_z = 0;
                    tmp_Fc_FL_z = 0;
                    tmp_Fc_FR_z = tar_Fc_FR * 2 * 0.5 * (1 - cos(PI2 / (tmp_fc_time * 2)*(double) (fc_cnt) * dt));
                }
                else if (fc_cnt >= ts_cnt - tmp_fc_cnt) {
                    tmp_Fc_RL_z = tar_Fc_RL * 2 - tar_Fc_RL * 2 * 0.5 * (1 - cos(PI2 / (tmp_fc_time * 2)*(double) (fc_cnt - (ts_cnt - tmp_fc_cnt)) * dt));
                    tmp_Fc_RR_z = 0;
                    tmp_Fc_FL_z = 0;
                    tmp_Fc_FR_z = tar_Fc_FR * 2 - tar_Fc_FR * 2 * 0.5 * (1 - cos(PI2 / (tmp_fc_time * 2)*(double) (fc_cnt - (ts_cnt - tmp_fc_cnt)) * dt));
                }
            }

            Fc_RL_z = tmp_Fc_RL_z + 20 * com_acc[2];
            Fc_RR_z = tmp_Fc_RR_z;
            Fc_FL_z = tmp_Fc_FL_z;
            Fc_FR_z = tmp_Fc_FR_z + 20 * com_acc[2];

        }
        else if (CommandFlag == NOMAL_TROT_WALKING) {

            if (fc_cnt <= tmp_fc_cnt) {
                Fc_RL_z = tar_Fc_RL + (tar_Fc_RL * 2 - tar_Fc_RL)*0.5 * (1 - cos(PI2 / (tmp_fc_time * 2)*(double) (fc_cnt) * dt));
                Fc_RR_z = tar_Fc_RR - (tar_Fc_RR)*0.5 * (1 - cos(PI2 / (tmp_fc_time * 2)*(double) (fc_cnt) * dt));
                Fc_FL_z = tar_Fc_FL - (tar_Fc_FL)*0.5 * (1 - cos(PI2 / (tmp_fc_time * 2)*(double) (fc_cnt) * dt));
                Fc_FR_z = tar_Fc_FR + (tar_Fc_FR * 2 - tar_Fc_FR)*0.5 * (1 - cos(PI2 / (tmp_fc_time * 2)*(double) (fc_cnt) * dt));
            }
            else if (fc_cnt >= dsp_cnt - tmp_fc_cnt) {
                Fc_RL_z = tar_Fc_RL * 2 + (tar_Fc_RL - tar_Fc_RL * 2)*0.5 * (1 - cos(PI2 / (tmp_fc_time * 2)*(double) (fc_cnt - (dsp_cnt - tmp_fc_cnt)) * dt));
                Fc_RR_z = (tar_Fc_RR)*0.5 * (1 - cos(PI2 / (tmp_fc_time * 2)*(double) (fc_cnt - (dsp_cnt - tmp_fc_cnt)) * dt));
                Fc_FL_z = (tar_Fc_FL)*0.5 * (1 - cos(PI2 / (tmp_fc_time * 2)*(double) (fc_cnt - (dsp_cnt - tmp_fc_cnt)) * dt));
                Fc_FR_z = tar_Fc_FR * 2 + (tar_Fc_FR - tar_Fc_FR * 2)*0.5 * (1 - cos(PI2 / (tmp_fc_time * 2)*(double) (fc_cnt - (dsp_cnt - tmp_fc_cnt)) * dt));
            }
            else {
                Fc_RL_z = tar_Fc_RL * 2;
                Fc_RR_z = 0;
                Fc_FL_z = 0;
                Fc_FR_z = tar_Fc_FR * 2;
            }
        }

        else {
            Fc_RL_z = tar_Fc_RL * 2;
            Fc_RR_z = 0;
            Fc_FL_z = 0;
            Fc_FR_z = tar_Fc_FR * 2;
        }

        fc_cnt++;
    }
    else if (FC_PHASE == STANCE_RRFL) {

        if (CommandFlag == FLYING_TROT_RUNNING) {
            if (flying_trot_final_flag == true) {
                if (fc_cnt <= tmp_fc_cnt) {
                    tmp_Fc_RL_z = 0;
                    tmp_Fc_RR_z = 0 + (tar_Fc_RR * 2 - 0)*0.5 * (1 - cos(PI2 / (tmp_fc_time * 2)*(double) (fc_cnt) * dt));
                    tmp_Fc_FL_z = 0 + (tar_Fc_FL * 2 - 0)*0.5 * (1 - cos(PI2 / (tmp_fc_time * 2)*(double) (fc_cnt) * dt));
                    tmp_Fc_FR_z = 0;
                }
                else if (fc_cnt >= ts_cnt - tmp_fc_cnt) {
                    tmp_Fc_RL_z = tar_Fc_RL * 0.5 * (1 - cos(PI2 / (tmp_fc_time * 2)*(double) (fc_cnt - (ts_cnt - tmp_fc_cnt)) * dt));
                    tmp_Fc_RR_z = tar_Fc_RR * 2 + (tar_Fc_RR - tar_Fc_RR * 2) * 0.5 * (1 - cos(PI2 / (tmp_fc_time * 2)*(double) (fc_cnt - (ts_cnt - tmp_fc_cnt)) * dt));
                    tmp_Fc_FL_z = tar_Fc_FL * 2 + (tar_Fc_FL - tar_Fc_FL * 2) * 0.5 * (1 - cos(PI2 / (tmp_fc_time * 2)*(double) (fc_cnt - (ts_cnt - tmp_fc_cnt)) * dt));
                    tmp_Fc_FR_z = tar_Fc_FR * 0.5 * (1 - cos(PI2 / (tmp_fc_time * 2)*(double) (fc_cnt - (ts_cnt - tmp_fc_cnt)) * dt));
                }
            }
            else {
                if (fc_cnt <= tmp_fc_cnt) {
                    tmp_Fc_RL_z = 0;
                    tmp_Fc_RR_z = tar_Fc_RR * 2 * 0.5 * (1 - cos(PI2 / (tmp_fc_time * 2)*(double) (fc_cnt) * dt));
                    tmp_Fc_FL_z = tar_Fc_FL * 2 * 0.5 * (1 - cos(PI2 / (tmp_fc_time * 2)*(double) (fc_cnt) * dt));
                    tmp_Fc_FR_z = 0;
                }
                else if (fc_cnt >= ts_cnt - tmp_fc_cnt) {
                    tmp_Fc_RL_z = 0;
                    tmp_Fc_RR_z = tar_Fc_RR * 2 + (0 - tar_Fc_RR * 2) * 0.5 * (1 - cos(PI2 / (tmp_fc_time * 2)*(double) (fc_cnt - (ts_cnt - tmp_fc_cnt)) * dt));
                    tmp_Fc_FL_z = tar_Fc_FL * 2 + (0 - tar_Fc_FL * 2) * 0.5 * (1 - cos(PI2 / (tmp_fc_time * 2)*(double) (fc_cnt - (ts_cnt - tmp_fc_cnt)) * dt));
                    tmp_Fc_FR_z = 0;
                }
            }

            Fc_RL_z = tmp_Fc_RL_z;
            Fc_RR_z = tmp_Fc_RR_z + 20 * com_acc[2];
            Fc_FL_z = tmp_Fc_FL_z + 20 * com_acc[2];
            Fc_FR_z = tmp_Fc_FR_z;
        }
        else if (CommandFlag == NOMAL_TROT_WALKING) {
            if (fc_cnt <= tmp_fc_cnt) {
                Fc_RL_z = tar_Fc_RL + (0 - tar_Fc_RL)*0.5 * (1 - cos(PI2 / (tmp_fc_time * 2)*(double) (fc_cnt) * dt));
                Fc_RR_z = tar_Fc_RR + (tar_Fc_RR * 2 - tar_Fc_RR)*0.5 * (1 - cos(PI2 / (tmp_fc_time * 2)*(double) (fc_cnt) * dt));
                Fc_FL_z = tar_Fc_FL + (tar_Fc_FL * 2 - tar_Fc_FL)*0.5 * (1 - cos(PI2 / (tmp_fc_time * 2)*(double) (fc_cnt) * dt));
                Fc_FR_z = tar_Fc_FR + (0 - tar_Fc_FR)*0.5 * (1 - cos(PI2 / (tmp_fc_time * 2)*(double) (fc_cnt) * dt));
            }
            else if (fc_cnt >= dsp_cnt - tmp_fc_cnt) {
                Fc_RL_z = (tar_Fc_RL)*0.5 * (1 - cos(PI2 / (tmp_fc_time * 2)*(double) (fc_cnt - (dsp_cnt - tmp_fc_cnt)) * dt));
                Fc_RR_z = tar_Fc_RR * 2 + (tar_Fc_RR - tar_Fc_RR * 2)*0.5 * (1 - cos(PI2 / (tmp_fc_time * 2)*(double) (fc_cnt - (dsp_cnt - tmp_fc_cnt)) * dt));
                Fc_FL_z = tar_Fc_FL * 2 + (tar_Fc_FL - tar_Fc_FL * 2)*0.5 * (1 - cos(PI2 / (tmp_fc_time * 2)*(double) (fc_cnt - (dsp_cnt - tmp_fc_cnt)) * dt));
                Fc_FR_z = (tar_Fc_FR)*0.5 * (1 - cos(PI2 / (tmp_fc_time * 2)*(double) (fc_cnt - (dsp_cnt - tmp_fc_cnt)) * dt));
            }
            else {
                Fc_RL_z = 0;
                Fc_RR_z = tar_Fc_RR * 2;
                Fc_FL_z = tar_Fc_FL * 2;
                Fc_FR_z = 0;
            }
        }
        else {
            Fc_RL_z = 0;
            Fc_RR_z = tar_Fc_RR * 2;
            Fc_FL_z = tar_Fc_FL * 2;
            Fc_FR_z = 0;
        }

        fc_cnt++;
    }
    else if (FC_PHASE == ZERO) {
        Fc_RL_z = 0;
        Fc_RR_z = 0;
        Fc_FL_z = 0;
        Fc_FR_z = 0;

        fc_cnt = 0;
    }

    if (Fc_RL_z < 0) {
        Fc_RL_z = 0;
    }
    if (Fc_RR_z < 0) {
        Fc_RR_z = 0;
    }
    if (Fc_FL_z < 0) {
        Fc_FL_z = 0;
    }
    if (Fc_FR_z < 0) {
        Fc_FR_z = 0;
    }

    //    printf("Fc_RL_z = %f\n",Fc_RL_z);
//    Fc << 0, 0, 0, 0, 0, 0, 0, 0, 0, Fc_RL_z, 0, 0, Fc_RR_z, 0, 0, Fc_FL_z, 0, 0, Fc_FR_z;
    //     For friction modeling
            Fc << 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;


}

void CRobot::Cal_Fc2(void)
{
    Fc << 0, 0, 0, 0, 0, 0, 0, 0, 0, Fc_RL_z, 0, 0, Fc_RR_z, 0, 0, Fc_FL_z, 0, 0, Fc_FR_z;
}

void CRobot::Torque_off(void)
{
    for (int i = 0; i < nDOF; ++i) {
        joint[i].torque = 0;
        target_vel[i] = 0;
        target_acc[i] = 0;
    }

}

void CRobot::Cal_CP(void)
{

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

void CRobot::Cal_CP2(void)
{
    static double CP_y_alpha = 0.07, CP_y_dot_alpha = 0.03;

    natural_freq = sqrt(com_height / GRAVITY);

    lpf_COM_y = (1 - CP_y_alpha) * lpf_COM_y + CP_y_alpha * actual_com_pos[1];
    lpf_COM_y_dot = (1 - CP_y_dot_alpha) * lpf_COM_y_dot + CP_y_dot_alpha * actual_com_vel[1];

    if (Mode == MODE_SIMULATION) {
        CP_y = (lpf_COM_y + 1 / natural_freq * lpf_COM_y_dot) / 15.0; // weighted CP_y
    }
    else {
        CP_y = (lpf_COM_y + 1 / natural_freq * lpf_COM_y_dot) / 5.0; // weighted CP_y
    }

}

void CRobot::Get_gain(void)
{
    double z_c = com_height;
    int nCount = 0;
    double temp_Gp_gain, temp_Gx_gain, temp_Gi_gain;

    AA << 1, dt, dt * dt / 2.0f,
            0, 1, dt,
            0, 0, 1;

    //    cout << "AA = " << AA << endl;

    BB << dt * dt * dt / 6.0f, dt * dt / 2.0f, dt;
    //    cout << "BB = " << BB << endl;

    CC << 1, 0, -z_c / GRAVITY;
    //    cout << "CC = " << CC << endl;

    FILE *fp1;
    FILE *fp2;
    FILE *fp3;

    if (Mode == MODE_ACTUAL_ROBOT) {
        fp1 = fopen("/home/user/Desktop/RcLab-PongBotQ2/src/gain/dh_Gp.txt", "r");

        if (fp1 == NULL)printf("CAN NOT OPEN Gp TEXT FILE \n");
        while (fscanf(fp1, "%lf", &temp_Gp_gain) == 1) {
            pv_Gp[nCount] = temp_Gp_gain;
            nCount++;
        }
        fclose(fp1);
        nCount = 0;

        fp2 = fopen("/home/user/Desktop/RcLab-PongBotQ2/src/gain/dh_Gx.txt", "r");
        if (fp2 == NULL)printf("CAN NOT OPEN Gx TEXT FILE \n");
        while (fscanf(fp2, "%lf", &temp_Gx_gain) == 1) {
            pv_Gx[nCount] = temp_Gx_gain;
            nCount++;
        }
        fclose(fp2);
        nCount = 0;

        fp3 = fopen("/home/user/Desktop/RcLab-PongBotQ2/src/gain/dh_Gi.txt", "r");
        if (fp3 == NULL)printf("CAN NOT OPEN Gi TEXT FILE \n");
        while (fscanf(fp3, "%lf", &temp_Gi_gain) == 1) {
            pv_Gi[nCount] = temp_Gi_gain;
            nCount++;
        }
        fclose(fp3);
    }
    else { // simulation
        fp1 = fopen("/root/catkin_ws/src/RcLab-PongBotQ2/src/gain/dh_Gp.txt", "r");
        if (fp1 == NULL)printf("CAN NOT OPEN Gp TEXT FILE \n");
        while (fscanf(fp1, "%lf", &temp_Gp_gain) == 1) {
            pv_Gp[nCount] = temp_Gp_gain;
            nCount++;
        }
        fclose(fp1);
        nCount = 0;

        fp2 = fopen("/root/catkin_ws/src/RcLab-PongBotQ2/src/gain/dh_Gx.txt", "r");
        if (fp2 == NULL)printf("CAN NOT OPEN Gx TEXT FILE \n");
        while (fscanf(fp2, "%lf", &temp_Gx_gain) == 1) {
            pv_Gx[nCount] = temp_Gx_gain;
            nCount++;
        }
        fclose(fp2);
        nCount = 0;

        fp3 = fopen("/root/catkin_ws/src/RcLab-PongBotQ2/src/gain/dh_Gi.txt", "r");
        if (fp3 == NULL)printf("CAN NOT OPEN Gi TEXT FILE \n");
        while (fscanf(fp3, "%lf", &temp_Gi_gain) == 1) {
            pv_Gi[nCount] = temp_Gi_gain;
            nCount++;
        }
        fclose(fp3);
    }


    Gi = pv_Gi[0];
    //        cout << pv_Gi[0] << endl;

    Gx(0) = pv_Gx[0];
    Gx(1) = pv_Gx[1];
    Gx(2) = pv_Gx[2];

    //    cout << pv_Gx[0] << endl << pv_Gx[1] << endl << pv_Gx[2] << endl;

    for (unsigned int i = 0; i < preview_cnt - 1; ++i) {
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
    l_FL_x = 0.35 - target_EP[6];
    l_FR_x = 0.35 - target_EP[9];

    //    cout << "l_RL_x="<<l_RL_x  << "l_RR_x="<< l_RR_x << "l_FL_x=" << l_FL_x << "l_FR_x=" << l_FR_x << endl;

    l_RL_y = 0; // 0.115 + target_EP[1];
    l_RR_y = 0; //-0.115 + target_EP[4];
    l_FL_y = 0; // 0.115 + target_EP[7];
    l_FR_y = 0; //-0.115 + target_EP[10];


    sum_Mx = 0;
    sum_My = l_RL_x * RL.ftSensor.Fz + l_RR_x * RR.ftSensor.Fz + l_FL_x * FL.ftSensor.Fz + l_FR_x * FR.ftSensor.Fz;
    sum_F = RL.ftSensor.Fz + RR.ftSensor.Fz + FL.ftSensor.Fz + FR.ftSensor.Fz;

    if (sum_F > 50) {
        zmp_x = com_pos(0) + (sum_My) / (sum_F); // from local to global
        zmp_y = (sum_Mx) / (sum_F);
    }
    else {
        zmp_x = 0;
        zmp_y = 0;
    }

    lpf_zmp_x = (1 - alpha) * old_lpf_zmp_x + alpha*zmp_x;

    old_lpf_zmp_x = lpf_zmp_x;

}

void CRobot::Get_act_com(void)
{
    if (CommandFlag == GOTO_WALK_READY_POS || CommandFlag == PRONK_JUMP) {

        if (FC_PHASE == STOP) {
            //            cout << "1" << endl;
            actual_com_pos[1] = -(actual_pos[0] + actual_pos[3] + actual_pos[7] + actual_pos[10]) / 4.0 * com_height;
            actual_com_vel[1] = -(actual_vel[0] + actual_vel[3] + actual_vel[7] + actual_vel[10]) / 4.0 * com_height;
        }
        else if (FC_PHASE == STANCE_RRFL) {
            actual_com_pos[1] = -(actual_pos[3] + actual_pos[7]) / 2.0 * com_height;
            actual_com_vel[1] = -(actual_vel[3] + actual_vel[7]) / 2.0 * com_height;
        }
        else if (FC_PHASE == STANCE_RLFR) {
            actual_com_pos[1] = -(actual_pos[0] + actual_pos[10]) / 2.0 * com_height;
            actual_com_vel[1] = -(actual_vel[0] + actual_vel[10]) / 2.0 * com_height;
        }
    }

    else if (CommandFlag == NOMAL_TROT_WALKING) {
        if (FC_PHASE == STOP) {
            actual_com_pos[1] = -(actual_pos[0] + actual_pos[3] + actual_pos[7] + actual_pos[10]) / 4.0 * com_height;
            actual_com_vel[1] = -(actual_vel[0] + actual_vel[3] + actual_vel[7] + actual_vel[10]) / 4.0 * com_height;
        }
        else if (FC_PHASE == STANCE_RRFL) {
            actual_com_pos[1] = -(actual_pos[3] + actual_pos[7]) / 2.0 * com_height;
            actual_com_vel[1] = -(actual_vel[3] + actual_vel[7]) / 2.0 * com_height;
        }
        else if (FC_PHASE == STANCE_RLFR) {
            actual_com_pos[1] = -(actual_pos[0] + actual_pos[10]) / 2.0 * com_height;
            actual_com_vel[1] = -(actual_vel[0] + actual_vel[10]) / 2.0 * com_height;
        }
    }
    else if (CommandFlag == FLYING_TROT_RUNNING) {
        //    static double CP_x_alpha = 0.05, CP_x_dot_alpha = 0.003;
        //    static double CP_y_alpha = 0.03, CP_y_dot_alpha = 0.001;
        //
        if (ft_cnt < 5 * ft_step_cnt + 1) {
            natural_freq = sqrt(com_height / GRAVITY);

            // roll
            actual_com_pos[1] = -com_height * IMURoll * PI / 180 * 1.0;
            actual_com_vel[1] = -com_height * IMURoll_dot * PI / 180 * 1.0;
        }
        else {
            if (FC_PHASE == STOP) {
                actual_com_pos[1] = -(actual_pos[0] + actual_pos[3] + actual_pos[7] + actual_pos[10]) / 4.0 * com_height;
                actual_com_vel[1] = -(actual_vel[0] + actual_vel[3] + actual_vel[7] + actual_vel[10]) / 4.0 * com_height;
            }
            else if (FC_PHASE == STANCE_RRFL) {
                actual_com_pos[1] = -(actual_pos[3] + actual_pos[7]) / 2.0 * com_height;
                actual_com_vel[1] = -(actual_vel[3] + actual_vel[7]) / 2.0 * com_height;
            }
            else if (FC_PHASE == STANCE_RLFR) {
                actual_com_pos[1] = -(actual_pos[0] + actual_pos[10]) / 2.0 * com_height;
                actual_com_vel[1] = -(actual_vel[0] + actual_vel[10]) / 2.0 * com_height;
            }
        }

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
    else {
        actual_com_pos[1] = 0;
        actual_com_vel[1] = 0;
    }
}

void CRobot::Damping_con(void)
{
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
    target_pos_offset[7] = +kp_roll * (target_front_hip_roll_err - front_hip_roll_err) + kd_roll * (0 - front_hip_roll_err_dot);
    target_pos_offset[10] = -kp_roll * (target_front_hip_roll_err - front_hip_roll_err) - kd_roll * (0 - front_hip_roll_err_dot);
    target_pos_offset[0] = +kp_roll * (target_rear_hip_roll_err - rear_hip_roll_err) + kd_roll * (0 - rear_hip_roll_err_dot);
    target_pos_offset[3] = -kp_roll * (target_rear_hip_roll_err - rear_hip_roll_err) - kd_roll * (0 - rear_hip_roll_err_dot);

    //	printf("target_front_hip_roll_err - front_hip_roll_err = %f\n",(target_front_hip_roll_err - front_hip_roll_err)*R2D);
    //	printf("target_pos_offset[7] = %f, target_pos_offset[0] = %f\n",target_pos_offset[7],target_pos_offset[0]);
    //    printf("kp_roll = %f, kd_roll = %f\n",kp_roll,kd_roll);

    target_pos_with_con = target_pos + target_pos_offset;
}
void CRobot::Cal_JFc(void)
{
    const double alpha = 0.002;
    tmp_CTC_Torque = (1-alpha)*tmp_CTC_Torque + (alpha)*CTC_Torque;
    
    Fc2 = (J_A*M_term.inverse()*J_A.transpose()).inverse()*(J_A*M_term.inverse()*(S_mat.transpose()*tmp_CTC_Torque - C_term - G_term) + dJdQ);  
    
    const double max_Fc_z = -20;
    const double min_Fc_z = -800;
    
    if(Fc2(7) > abs(Fc2(9))*0.6/sqrt(2)){
        Fc2(7) = abs(Fc2(9))*0.6/sqrt(2);
    }
    else if(Fc2(7) < -abs(Fc2(9))*0.6/sqrt(2)){
        Fc2(7) = -abs(Fc2(9))*0.6/sqrt(2);
    }
    
    if(Fc2(8) > abs(Fc2(9))*0.6/sqrt(2)){
        Fc2(8) = abs(Fc2(9))*0.6/sqrt(2);
    }
    else if(Fc2(8) < -abs(Fc2(9))*0.6/sqrt(2)){
        Fc2(8) = -abs(Fc2(9))*0.6/sqrt(2);
    }
    
    if(Fc2(9) > max_Fc_z){
        Fc2(9) = max_Fc_z;
    }
    else if(Fc2(9) < min_Fc_z){
        Fc2(9) = min_Fc_z;
    }
    
    if(Fc2(10) > abs(Fc2(12))*0.6/sqrt(2)){
        Fc2(10) = abs(Fc2(12))*0.6/sqrt(2);
    }
    else if(Fc2(10) < -abs(Fc2(12))*0.6/sqrt(2)){
        Fc2(10) = -abs(Fc2(12))*0.6/sqrt(2);
    }
    
    if(Fc2(11) > abs(Fc2(12))*0.6/sqrt(2)){
        Fc2(11) = abs(Fc2(12))*0.6/sqrt(2);
    }
    else if(Fc2(11) < -abs(Fc2(12))*0.6/sqrt(2)){
        Fc2(11) = -abs(Fc2(12))*0.6/sqrt(2);
    }
    
    if(Fc2(12) > max_Fc_z){
        Fc2(12) = max_Fc_z;
    }
    else if(Fc2(12) < min_Fc_z){
        Fc2(12) = min_Fc_z;
    }
    
    if(Fc2(13) > abs(Fc2(15))*0.6/sqrt(2)){
        Fc2(13) = abs(Fc2(15))*0.6/sqrt(2);
    }
    else if(Fc2(13) < -abs(Fc2(15))*0.6/sqrt(2)){
        Fc2(13) = -abs(Fc2(15))*0.6/sqrt(2);
    }
    
    if(Fc2(14) > abs(Fc2(15))*0.6/sqrt(2)){
        Fc2(14) = abs(Fc2(15))*0.6/sqrt(2);
    }
    else if(Fc2(14) < -abs(Fc2(15))*0.6/sqrt(2)){
        Fc2(14) = -abs(Fc2(15))*0.6/sqrt(2);
    }
    
    if(Fc2(15) > max_Fc_z){
        Fc2(15) = max_Fc_z;
    }
    else if(Fc2(15) < min_Fc_z){
        Fc2(15) = min_Fc_z;
    }
    
    if(Fc2(16) > abs(Fc2(18))*0.6/sqrt(2)){
        Fc2(16) = abs(Fc2(18))*0.6/sqrt(2);
    }
    else if(Fc2(16) < -abs(Fc2(18))*0.6/sqrt(2)){
        Fc2(16) = -abs(Fc2(18))*0.6/sqrt(2);
    }
    
    if(Fc2(17) > abs(Fc2(18))*0.6/sqrt(2)){
        Fc2(17) = abs(Fc2(18))*0.6/sqrt(2);
    }
    else if(Fc2(17) < -abs(Fc2(18))*0.6/sqrt(2)){
        Fc2(17) = -abs(Fc2(18))*0.6/sqrt(2);
    }
    
    if(Fc2(18) > max_Fc_z){
        Fc2(18) = max_Fc_z;
    }
    else if(Fc2(18) < min_Fc_z){
        Fc2(18) = min_Fc_z;
    }

    cout << "Fc2 = " << Fc2.transpose() << endl << endl ;
    
    JFc = J_A.transpose() * (Fc2);
}


