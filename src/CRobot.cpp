
#include <stdio.h>
#include <math.h>
#include "CRobot.h"
#include "QuadProgpp/Array.hh"
#include "QuadProgpp/QuadProg++.hh"


using namespace quadprogpp;

CRobot::CRobot() {
}

CRobot::CRobot(const CRobot& orig) {
}

CRobot::~CRobot() {
}

void CRobot::setRobotModel(Model* getModel) {
    cout << endl << "Set Robot Model Start !!" << endl;
    Mode = MODE_SIMULATION;
    //    Mode = MODE_ACTUAL_ROBOT;

    WH_Mode = QP_CON; //QP_CON;// MPC_CON

    // ============== Controller OnOff ================ //
    //    Base_Ori_Con_onoff_flag = false; //false; //true; //true;
    CP_con_onoff_flag = false; //true; //false;//true;
    Slope_con_onoff_flag = true; //true; //false;//true;
    gain_scheduling_flag = true;
    // ============== Controller OnOff End ================ //

    RL_base2hip_pos << -0.350, 0.115, -0.053;
    RR_base2hip_pos << -0.350, -0.115, -0.053;
    FL_base2hip_pos << 0.350, 0.115, -0.053;
    FR_base2hip_pos << 0.350, -0.115, -0.053;

    base2hip_pos << RL_base2hip_pos, RR_base2hip_pos, FL_base2hip_pos, FR_base2hip_pos;

    // global init foot position
    //    tar_init_RL_foot_pos << RL_base2hip_pos(0), RL_base2hip_pos(1) + 0.105 - 0.04, 0.0;
    //    tar_init_RR_foot_pos << RR_base2hip_pos(0), RR_base2hip_pos(1) - 0.105 + 0.04, 0.0;
    //    tar_init_FL_foot_pos << FL_base2hip_pos(0), FL_base2hip_pos(1) + 0.105 - 0.04, 0.0;
    //    tar_init_FR_foot_pos << FR_base2hip_pos(0), FR_base2hip_pos(1) - 0.105 + 0.04, 0.0;

    tar_init_RL_foot_pos << RL_base2hip_pos(0), RL_base2hip_pos(1) + 0.105 - 0.0, 0.0;
    tar_init_RR_foot_pos << RR_base2hip_pos(0), RR_base2hip_pos(1) - 0.105 + 0.0, 0.0;
    tar_init_FL_foot_pos << FL_base2hip_pos(0), FL_base2hip_pos(1) + 0.105 - 0.0, 0.0;
    tar_init_FR_foot_pos << FR_base2hip_pos(0), FR_base2hip_pos(1) - 0.105 + 0.0, 0.0;

    com_height = 0.42; //0.42;

    if (Mode == MODE_SIMULATION) {
        set_simul_para();
    } else if (Mode == MODE_ACTUAL_ROBOT) {
        set_act_robot_para();
    }

    RL_foot_pos = tar_init_RL_foot_pos;
    RR_foot_pos = tar_init_RR_foot_pos;
    FL_foot_pos = tar_init_FL_foot_pos;
    FR_foot_pos = tar_init_FR_foot_pos;

    init_base_pos << 0, 0, com_height;
    init_base_ori << 0, 0, 0;
    base_pos = init_base_pos;
    base_ori = init_base_ori;
    base_vel << 0, 0, 0;
    base_ori_dot << 0, 0, 0;

    init_Kp_q = Kp_q;
    init_Kd_q = Kd_q;

    x_moving_speed = 0;
    y_moving_speed = 0;

    // Link com position
    p_base2body_com << -0.038, 0, -0.01, 1;
    p_RL_hp_com << 0, -0.0029, 0, 1;
    p_RL_thigh_com << -0.001, -0.006, -0.0227, 1;
    p_RL_calf_com << 0, 0.003, -0.094, 1;
    p_RR_hp_com << 0, 0.0029, 0, 1;
    p_RR_thigh_com << -0.001, 0.006, -0.0227, 1;
    p_RR_calf_com << 0, -0.003, -0.094, 1;
    p_FL_hp_com << 0, -0.0029, 0, 1;
    p_FL_thigh_com << -0.001, -0.006, -0.0227, 1;
    p_FL_calf_com << 0, 0.003, -0.094, 1;
    p_FR_hp_com << 0, 0.0029, 0, 1;
    p_FR_thigh_com << -0.001, 0.006, -0.0227, 1;
    p_FR_calf_com << 0, -0.003, -0.094, 1;

    target_EP << tar_init_RL_foot_pos - init_base_pos, tar_init_RR_foot_pos - init_base_pos, tar_init_FL_foot_pos - init_base_pos, tar_init_FR_foot_pos - init_base_pos;
    target_pos = IK1(target_EP);
    base_pos_ori << init_base_pos, init_base_ori;

    VectorNd com_offset = VectorNd::Zero(3);
    VectorNd tmp_init_com_pos = VectorNd::Zero(3);

    com_offset << 0.0, 0, 0;
    //    com_offset << 0.02, 0, 0;
    //    com_offset << 0.0, 0, 0;

    tmp_init_com_pos = Get_COM(base_pos_ori, target_pos);
    init_com_pos = tmp_init_com_pos + com_offset;

    cout << "tmp_init_com_pos = " << tmp_init_com_pos << endl << ", init_com_pos=" << init_com_pos << endl;

    base_offset = init_base_pos - init_com_pos;

    cout << "base_offset = " << base_offset << endl;
    tar_init_com_pos << 0.0, 0.0, com_height;
    tar_init_com_vel << 0.0, 0.0, 0.0;

    contact_num = 4;

    // ================= For RBDL =================== //
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

    for (unsigned int i = 0; i < 13; ++i) {
        joint[i].torque = 0;
    }

    IMURoll = 0;
    IMUPitch = 0;
    IMUYaw = 0;

    //    A0A = 0.05;
    //    AB = 0.305;
    //    B0B = 0.025;
    //    A0B0 = 0.305;

    moving_done_flag = true;
    walk_ready_moving_done_flag = false;

    for (unsigned int i = 0; i < 6; ++i) {
        pd_con_joint[i] = 0;
        pd_con_task[i] = 0;
    }

    pd_con_task[6] = 0;
    tmp_CTC_Torque = CTC_Torque;


    if (WH_Mode == QP_CON) {
        QP_Con_Init();

        //        if(Mode == MODE_SIMULATION){
        //        	// Cleanup
        //			if (MPC_data) {
        //				if (MPC_data->A) c_free(MPC_data->A);
        //				if (MPC_data->P) c_free(MPC_data->P);
        //				c_free(MPC_data);
        //			}
        //			if (MPC_settings) c_free(MPC_settings);
        //				// Cleanup END
        //        }

    }
    //    else{
    //        MPC_Con_Init();
    //
    //        if(Mode == MODE_SIMULATION){
    //			// Cleanup
    //			if (QP_data) {
    //				if (QP_data->A) c_free(QP_data->A);
    //				if (QP_data->P) c_free(QP_data->P);
    //				c_free(QP_data);
    //			}
    //			if (QP_settings) c_free(QP_settings);
    //        }
    //    }
    Get_gain_HS();
    cout << endl << "Set Robot Model End !!" << endl;
}

// ====================================================================

void CRobot::set_simul_para(void) {
    if (CommandFlag == NO_ACT || CommandFlag == GOTO_WALK_READY_POS || CommandFlag == NOMAL_TROT_WALKING || CommandFlag == TEST_FLAG) {
        foot_height = 0.10;
        _alpha = 0.001;
        dsp_time = 0.20;
        fsp_time = 0.10; //0.07
        step_time = dsp_time + fsp_time;
        dsp_cnt = 200;
        fsp_cnt = 100; //70;
        step_cnt = dsp_cnt + fsp_cnt;

        tar_Kp_q << 400, 100, 100,
                400, 100, 100,
                15000,
                400, 100, 100,
                400, 100, 100;

        tar_Kd_q << 15, 5, 5,
                15, 5, 5,
                300,
                15, 5, 5,
                15, 5, 5;

        tar_Kp_t << 2000, 0, 500,
                2000, 0, 500,
                2000, 0, 500,
                2000, 0, 500;

        tar_Kd_t << 10, 0, 5,
                10, 0, 5,
                10, 0, 5,
                10, 0, 5;

        // for gain scheduling
        tar_Kp_q_low << 400, 50, 50,
                400, 50, 50,
                30000,
                400, 50, 50,
                400, 50, 50;

        tar_Kd_q_low << 15, 4, 4,
                15, 4, 4,
                1000,
                15, 4, 4,
                15, 4, 4;

        tar_Kp_t_low << 2000, 0, 0,
                2000, 0, 0,
                2000, 0, 0,
                2000, 0, 0;

        tar_Kd_t_low << 10, 0, 0,
                10, 0, 0,
                10, 0, 0,
                10, 0, 0;

        tar_Kp_x << 1, 0, 0,
                0, 50, 0,
                0, 0, 1;
        tar_Kd_x << 0.01, 0, 0,
                0, 5, 0,
                0, 0, 0.1;
        tar_Kp_w << 1000, 0, 0,
                0, 1000, 0,
                0, 0, 0;
        tar_Kd_w << 10, 0, 0,
                0, 10, 0,
                0, 0, 0;
    } else if (CommandFlag == STAIR_WALKING) {
        foot_height = 0.10;
        _alpha = 0.001;
        dsp_time = 0.20;
        fsp_time = 0.50; //0.07
        step_time = dsp_time + fsp_time;
        dsp_cnt = 200;
        fsp_cnt = 500; //70;
        step_cnt = dsp_cnt + fsp_cnt;

        tar_Kp_q << 400, 20, 20,
                400, 20, 20,
                15000,
                400, 20, 20,
                400, 20, 20;

        tar_Kd_q << 15, 2, 2,
                15, 2, 2,
                300,
                15, 2, 2,
                15, 2, 2;

        tar_Kp_t << 1000, 0, 100,
                1000, 0, 100,
                1000, 0, 100,
                1000, 0, 100;

        tar_Kd_t << 10, 0, 1,
                10, 0, 1,
                10, 0, 1,
                10, 0, 1;

        // for gain scheduling
        tar_Kp_q_low << 400, 10, 10,
                400, 10, 10,
                30000,
                400, 10, 10,
                400, 10, 10;

        tar_Kd_q_low << 15, 1, 1,
                15, 1, 1,
                1000,
                15, 1, 1,
                15, 1, 1;

        tar_Kp_t_low << 1000, 0, 0,
                1000, 0, 0,
                1000, 0, 0,
                1000, 0, 0;

        tar_Kd_t_low << 10, 0, 0,
                10, 0, 0,
                10, 0, 0,
                10, 0, 0;

        tar_Kp_x << 1, 0, 0,
                0, 50, 0,
                0, 0, 30;
        tar_Kd_x << 0.01, 0, 0,
                0, 5, 0,
                0, 0, 3;
        tar_Kp_w << 1000, 0, 0,
                0, 1000, 0,
                0, 0, 0;
        tar_Kd_w << 10, 0, 0,
                0, 10, 0,
                0, 0, 0;
    } else if (CommandFlag == FLYING_TROT_RUNNING || CommandFlag == PRONK_JUMP) {

        swing_foot_height = 0.06; // flying trot
        _alpha = 0.0001;

        // =============== Flying trot parameters initialize =============== //

        ts = 0.20; //0.22; //0.25;
        tf = 0.05; //0.07;
        ft_step_time = ts + tf;

        ts_cnt = 200; //220;
        tf_cnt = 50;
        ft_step_cnt = ts_cnt + tf_cnt;

        h_0 = tar_init_com_pos(2);
        v_0 = 0;
        a_0 = 0;

        v_1 = 0.2; //0.10; //0.10; //0.15;
        a_1 = -GRAVITY;

        h_2 = tar_init_com_pos(2);
        v_2 = -0.0; //-0.05; // -0.3
        a_2 = -GRAVITY;

        h_3 = tar_init_com_pos(2);
        v_3 = 0;
        a_3 = 0;

        h_1 = 0.5 * GRAVITY * tf * tf - v_1 * tf + h_2;

        // =============== Flying trot parameters initialize END =============== //

        // for gain scheduling
        tar_Kp_q_low << 300, 100, 100,
                300, 100, 100,
                30000,
                300, 100, 100,
                300, 100, 100;

        tar_Kd_q_low << 10, 5, 5,
                10, 5, 5,
                1000,
                10, 5, 5,
                10, 5, 5;

        tar_Kp_t_low << 2000, 3000, 1000,
                2000, 3000, 1000,
                2000, 3000, 1000,
                2000, 3000, 1000;

        tar_Kd_t_low << 15, 20, 10,
                15, 20, 10,
                15, 20, 10,
                15, 20, 10;

        tar_Kp_x << 5, 0, 0,
                0, 50, 0,
                0, 0, 5;
        tar_Kd_x << 0.5, 0, 0,
                0, 5, 0,
                0, 0, 0.1;
        tar_Kp_w << 1000, 0, 0,
                0, 1000, 0,
                0, 0, 0;
        tar_Kd_w << 10, 0, 0,
                0, 10, 0,
                0, 0, 0;
    }
}

void CRobot::set_act_robot_para(void) {
    if (CommandFlag == NO_ACT || CommandFlag == GOTO_WALK_READY_POS || CommandFlag == NOMAL_TROT_WALKING || CommandFlag == TEST_FLAG) {
        foot_height = 0.06; //0.10;
        _alpha = 0.001; //0.01; // 0.001
        dsp_time = 0.20;
        fsp_time = 0.10; //0.07
        step_time = dsp_time + fsp_time;
        dsp_cnt = 200;
        fsp_cnt = 100; //70;
        step_cnt = dsp_cnt + fsp_cnt;

        tar_Kp_q << 500, 150, 100,
                500, 150, 100,
                0,
                500, 150, 100,
                500, 150, 100;

        tar_Kd_q << 10, 6, 5,
                10, 6, 5,
                0,
                10, 6, 5,
                10, 6, 5;

        tar_Kp_t << 1000, 0, 100,
                1000, 0, 100,
                1000, 0, 100,
                1000, 0, 100;

        tar_Kd_t << 10, 0, 1,
                10, 0, 1,
                10, 0, 1,
                10, 0, 1;

        // for gain scheduling

        tar_Kp_q_low << 500, 80, 50,
                500, 80, 50,
                0,
                500, 80, 50,
                500, 80, 50;

        tar_Kd_q_low << 10, 4, 2,
                10, 4, 2,
                0,
                10, 4, 2,
                10, 4, 2;

        tar_Kp_t_low << 1000, 0, 0,
                1000, 0, 0,
                1000, 0, 0,
                1000, 0, 0;

        tar_Kd_t_low << 10, 0, 0,
                10, 0, 0,
                10, 0, 0,
                10, 0, 0;

        // ========= QP Gain ========= //
        tar_Kp_x << 1, 0, 0,
                0, 50, 0,
                0, 0, 1;
        tar_Kd_x << 0.01, 0, 0,
                0, 0.5, 0,
                0, 0, 0.01;

        tar_Kp_w << 1000, 0, 0,
                0, 1000, 0,
                0, 0, 0;
        tar_Kd_w << 10, 0, 0,
                0, 10, 0,
                0, 0, 0;

    } else if (CommandFlag == STAIR_WALKING) {

        foot_height = 0.12; //0.10;
        _alpha = 0.001; // 0.001
        dsp_time = 0.20;
        fsp_time = 0.60; //0.07
        step_time = dsp_time + fsp_time;
        dsp_cnt = 200;
        fsp_cnt = 600; //70;
        step_cnt = dsp_cnt + fsp_cnt;

        tar_Kp_q << 500, 20, 20,
                500, 20, 20,
                0,
                500, 20, 20,
                500, 20, 20;

        tar_Kd_q << 10, 2, 2,
                10, 2, 2,
                0,
                10, 2, 2,
                10, 2, 2;

        tar_Kp_t << 2000, 0, 100,
                2000, 0, 100,
                2000, 0, 100,
                2000, 0, 100;

        tar_Kd_t << 20, 0, 1,
                20, 0, 1,
                20, 0, 1,
                20, 0, 1;

        // for gain scheduling

        tar_Kp_q_low << 500, 10, 10,
                500, 10, 10,
                0,
                500, 10, 10,
                500, 10, 10;

        tar_Kd_q_low << 10, 1, 1,
                10, 1, 1,
                0,
                10, 1, 1,
                10, 1, 1;

        tar_Kp_t_low << 2000, 0, 0,
                2000, 0, 0,
                2000, 0, 0,
                2000, 0, 0;

        tar_Kd_t_low << 20, 0, 0,
                20, 0, 0,
                20, 0, 0,
                20, 0, 0;

        // ========= QP Gain ========= //
        tar_Kp_x << 1, 0, 0,
                0, 50, 0,
                0, 0, 10;
        tar_Kd_x << 0.1, 0, 0,
                0, 5, 0,
                0, 0, 1;

        tar_Kp_w << 1000, 0, 0,
                0, 1000, 0,
                0, 0, 0;
        tar_Kd_w << 10, 0, 0,
                0, 10, 0,
                0, 0, 0;

    } else if (CommandFlag == FLYING_TROT_RUNNING || CommandFlag == PRONK_JUMP) {

        swing_foot_height = 0.06; // flying trot
        _alpha = 0.001;

        // =============== Flying trot parameters initialize =============== //

        ts = 0.20; //0.22; //0.25;
        tf = 0.05; //0.07;
        ft_step_time = ts + tf;

        ts_cnt = 200; //220;
        tf_cnt = 50;
        ft_step_cnt = ts_cnt + tf_cnt;

        h_0 = tar_init_com_pos(2);
        v_0 = 0;
        a_0 = 0;

        v_1 = 0.20; //0.10; //0.15;
        a_1 = -GRAVITY;

        h_2 = tar_init_com_pos(2);
        v_2 = -0.0; //-0.05; // -0.3
        a_2 = -GRAVITY;

        h_3 = tar_init_com_pos(2);
        v_3 = 0;
        a_3 = 0;

        h_1 = 0.5 * GRAVITY * tf * tf - v_1 * tf + h_2;

        // =============== Flying trot parameters initialize END =============== //

        // for gain scheduling
        tar_Kp_q_low << 200, 100, 100,
                200, 100, 100,
                0,
                200, 100, 100,
                200, 100, 100;

        tar_Kd_q_low << 10, 5, 5,
                10, 5, 5,
                0,
                10, 5, 5,
                10, 5, 5;

        tar_Kp_t_low << 2000, 0, 0,
                2000, 0, 0,
                2000, 0, 0,
                2000, 0, 0;

        tar_Kd_t_low << 20, 0, 0,
                20, 0, 0,
                20, 0, 0,
                20, 0, 0;

        tar_Kp_x << 1, 0, 0,
                0, 30, 0,
                0, 0, 1;
        tar_Kd_x << 0.1, 0, 0,
                0, 3, 0,
                0, 0, 0.1;
        tar_Kp_w << 1000, 0, 0,
                0, 500, 0,
                0, 0, 0;
        tar_Kd_w << 10, 0, 0,
                0, 5, 0,
                0, 0, 0;
    }
}


// ============================================================================================ //

void CRobot::StateUpdate(void) {
    RobotState(AXIS_X) = base_pos(0); //base.currentX;
    RobotState(AXIS_Y) = base_pos(1); //base.currentY;
    RobotState(AXIS_Z) = base_pos(2); //base.currentZ;
    RobotState(AXIS_Roll) = base_ori(0); //act_base_ori(0);//base_ori(0); //base.currentRoll;
    RobotState(AXIS_Pitch) = base_ori(1); //act_base_ori(1);//base_ori(1); //base.currentPitch;
    RobotState(AXIS_Yaw) = base_ori(2); //base.currentYaw;
    RobotStatedot(AXIS_X) = base_vel(0); //base.currentXvel;
    RobotStatedot(AXIS_Y) = base_vel(1); //base.currentYvel;
    RobotStatedot(AXIS_Z) = base_vel(2); //base.currentZvel;
    RobotStatedot(AXIS_Roll) = base_ori_dot(0); //base.currentRollvel;
    RobotStatedot(AXIS_Pitch) = base_ori_dot(1); //base.currentPitchvel;
    RobotStatedot(AXIS_Yaw) = base_ori_dot(2); //base.currentYawvel;

    for (int nJoint = 0; nJoint < nDOF; nJoint++) {
        RobotState(6 + nJoint) = actual_pos[nJoint];
        RobotStatedot(6 + nJoint) = actual_vel[nJoint];
        RobotState2dot(6 + nJoint) = actual_acc[nJoint];
    }

    base_ori_quat = Math::Quaternion::fromXYZAngles(base_ori);
    Math::Quaternion QQ(base_ori_quat);
    m_pModel->SetQuaternion(base.ID, QQ, RobotState);

    FK2();

    Get_act_com();
}

void CRobot::ComputeTorqueControl() {
    // ================= Cal Jacobian ================= //
    //	cout << "1" << endl;

    CalcPointJacobian6D(*m_pModel, RobotState, base.ID, Originbase, J_BASE, true);
    CalcPointJacobian6D(*m_pModel, RobotState, front_body.ID, Originbase, J_FRONT_BODY, true);
    CalcPointJacobian(*m_pModel, RobotState, RL.ID, EP_OFFSET_RL, J_RL, true);
    CalcPointJacobian(*m_pModel, RobotState, RR.ID, EP_OFFSET_RR, J_RR, true);
    CalcPointJacobian(*m_pModel, RobotState, FL.ID, EP_OFFSET_FL, J_FL, true);
    CalcPointJacobian(*m_pModel, RobotState, FR.ID, EP_OFFSET_FR, J_FR, true);

    //   cout << J_RL << endl;
    J_A.block(0, 0, 6, 19) = J_BASE;
    J_A.block(6, 0, 1, 19) = J_FRONT_BODY.block(2, 0, 1, 19); // only yaw
    J_A.block(7, 0, 3, 19) = J_RL;
    J_A.block(10, 0, 3, 19) = J_RR;
    J_A.block(13, 0, 3, 19) = J_FL;
    J_A.block(16, 0, 3, 19) = J_FR;

    //    cout << J_RL.block(0,6,3,3) << endl;

    J_RL2 << J_RL.block(0, 6, 3, 3);
    J_RR2 << J_RR.block(0, 9, 3, 3);
    J_FL2 << J_FL.block(0, 13, 3, 3);
    J_FR2 << J_FR.block(0, 16, 3, 3);

    act_RL_q_dot << actual_vel[0], actual_vel[1], actual_vel[2];
    act_RR_q_dot << actual_vel[3], actual_vel[4], actual_vel[5];
    act_FL_q_dot << actual_vel[7], actual_vel[8], actual_vel[9];
    act_FR_q_dot << actual_vel[10], actual_vel[11], actual_vel[12];

    act_RL_foot_vel = J_RL2*act_RL_q_dot;
    act_RR_foot_vel = J_RR2*act_RR_q_dot;
    act_FL_foot_vel = J_FL2*act_FL_q_dot;
    act_FR_foot_vel = J_FR2*act_FR_q_dot;

    actual_EP_vel << act_RL_foot_vel, act_RR_foot_vel, act_FL_foot_vel, act_FR_foot_vel;

    // ================= Cal Jacobian END ================= //

    tar_RL_foot_pos_local = (RL_foot_pos - base_pos) + RL_foot_pos_local_offset;
    tar_RR_foot_pos_local = (RR_foot_pos - base_pos) + RR_foot_pos_local_offset;
    tar_FL_foot_pos_local = (FL_foot_pos - base_pos) + FL_foot_pos_local_offset;
    tar_FR_foot_pos_local = (FR_foot_pos - base_pos) + FR_foot_pos_local_offset;

    base_vel = com_vel;

    tar_RL_foot_vel_local = RL_foot_vel - base_vel;
    tar_RR_foot_vel_local = RR_foot_vel - base_vel;
    tar_FL_foot_vel_local = FL_foot_vel - base_vel;
    tar_FR_foot_vel_local = FR_foot_vel - base_vel;

    target_EP << tar_RL_foot_pos_local, tar_RR_foot_pos_local, tar_FL_foot_pos_local, tar_FR_foot_pos_local;
    actual_EP << act_RL_foot_pos_local, act_RR_foot_pos_local, act_FL_foot_pos_local, act_FR_foot_pos_local;

    target_EP_vel << tar_RL_foot_vel_local, tar_RR_foot_vel_local, tar_FL_foot_vel_local, tar_FR_foot_vel_local;

    target_pos = IK1(target_EP);

    tar_RL_q_dot = J_RL2.inverse() * tar_RL_foot_vel_local;
    tar_RR_q_dot = J_RR2.inverse() * tar_RR_foot_vel_local;
    tar_FL_q_dot = J_FL2.inverse() * tar_FL_foot_vel_local;
    tar_FR_q_dot = J_FR2.inverse() * tar_FR_foot_vel_local;

    target_vel << tar_RL_q_dot, tar_RR_q_dot, 0, tar_FL_q_dot, tar_FR_q_dot;

    for (unsigned int i = 0; i < 12; ++i) {
        //    pd_con_task[i + 7] = Kp_t[i]*(target_EP[i] - actual_EP[i]) + Kd_t[i]*(target_EP_vel[i] - actual_EP_vel[i]); // + target_EP_offset[i];
        pd_con_task[i + 7] = Kp_t[i]*(target_EP[i] - actual_EP[i]) + Kd_t[i]*(0 - actual_EP_vel[i]);

        tmp_data1[i] = Fc(i + 7);
        tmp_data1[i + 25] = target_EP_vel[i];
    }

    for (unsigned int i = 0; i < 13; ++i) {
        pd_con_joint[i + 6] = Kp_q[i]*(target_pos[i] - actual_pos[i]) + Kd_q[i]*(target_vel[i] - actual_vel[i]);

        tmp_data1[i + 12] = (target_pos[i] - actual_pos[i]) * R2D;
    }

    //    cout << "Kp_q[1] = " << Kp_q[1] << endl;
    //
    //    cout << "target_pos[1] = " << target_pos[1]*R2D << ", actual_pos[1] = " << actual_pos[1]*R2D << endl;
    //    cout << "target_vel[1] = " << target_vel[1]*R2D << ", actual_vel[1] = " << actual_vel[1]*R2D << endl;
    //
    //    cout << "pd_con_joint[7] = " << pd_con_joint[7] << ", pd_con_joint[10] = " << pd_con_joint[10] << endl;


    //    cout << "target_pos = " << target_pos.transpose()*R2D << endl;
    //    cout << "actual_pos = " << actual_pos.transpose()*R2D << endl;
    //
    //    cout << "target_vel = " << target_vel.transpose()*R2D << endl;
    //    cout << "actual_vel = " << actual_vel.transpose()*R2D << endl;

    CompositeRigidBodyAlgorithm(*m_pModel, RobotState, M_term, true);
    NonlinearEffects(*m_pModel, RobotState, RobotStatedot, hatNonLinearEffects);
    NonlinearEffects(*m_pModel, RobotState, VectorNd::Zero(m_pModel->dof_count), G_term);
    C_term = hatNonLinearEffects - G_term;

    //	Fc << 0,0,0,0,0,0,0, 0,0,110, 0,0,110, 0,0,110, 0,0,110;

    //    Fc << 0,0,0,0,0,0,0, 0,0,0, 0,0,0, 0,0,0, 0,0,0;

    //    MPC_Fc << 0,0,0,0,0,0,0, 0,tar_Fc_y/4,0, 0,tar_Fc_y/4,0, 0,tar_Fc_y/4,0, 0,tar_Fc_y/4,0;

    //    cout << "Torque by MPC_FC = " << - J_A.transpose() * (MPC_Fc) << endl;

    //    CTC_Torque = fc_weight * (C_term + G_term - J_A.transpose() * (Fc - pd_con_task + MPC_Fc*10));





    CTC_Torque = fc_weight * (C_term + G_term - J_A.transpose() * (Fc - pd_con_task)) + pd_con_joint;

    //    cout << "CTC_Torque[7] = " << CTC_Torque[7] << ", CTC_Torque[10] = " << CTC_Torque[10] << endl;

    //    cout << "Fc = " << Fc.transpose() << endl;
    ////    cout << "pd_con_task = " << pd_con_task.transpose() << endl;
    //    cout << "pd_con_joint = " << pd_con_joint.transpose() << endl;
    //    cout << "torque = " << CTC_Torque.transpose() << endl;

    for (int nJoint = 0; nJoint < nDOF; nJoint++) {
        joint[nJoint].torque = CTC_Torque(6 + nJoint);
        //        tmp_data1[nJoint + 25] = joint[nJoint].torque;
    }
    /*
        static int tmp_cnt5 = 0;
        tmp_data2[0] = (double) tmp_cnt5*dt;

        tmp_cnt5++;

    //    tmp_data2[1] = target_vel[1]*50*60/PI2;//com_pos(0);
    //    tmp_data2[2] = target_vel[2]*50*60/PI2;//com_pos(1);
    //    tmp_data2[3] = joint[1].torque/50;//com_pos(2);
    //    tmp_data2[4] = joint[2].torque/50;//act_com_pos(0);
        tmp_data2[1] = com_pos(0);
        tmp_data2[2] = com_pos(1);
        tmp_data2[3] = com_pos(2);
        tmp_data2[4] = act_com_pos(0);
        tmp_data2[5] = act_com_pos(1);
        tmp_data2[6] = act_com_pos(2);

        tmp_data2[29] = com_vel(0);
        tmp_data2[30] = com_vel(1);
        tmp_data2[31] = com_vel(2);
        tmp_data2[32] = act_com_vel(0);
        tmp_data2[33] = act_com_vel(1);
        tmp_data2[34] = act_com_vel(2);

    //    //    tmp_data2[41] = tmp_com_vel(0);
    //    //    tmp_data2[42] = tmp_com_vel(1);
    //    //    tmp_data2[43] = tmp_com_vel(2);
    //
    //    tmp_data2[44] = actual_EP_vel[0];
    //    tmp_data2[45] = actual_EP_vel[3];
    //    tmp_data2[46] = actual_EP_vel[6];
    //    tmp_data2[47] = actual_EP_vel[9];
    //
        tmp_data2[7] = base_ori(0) * R2D; // [deg]
        tmp_data2[8] = base_ori(1) * R2D;
        tmp_data2[9] = base_ori(2) * R2D;
        tmp_data2[10] = act_base_ori(0) * R2D; // [deg]
        tmp_data2[11] = act_base_ori(1) * R2D;
        tmp_data2[12] = act_base_ori(2) * R2D;

        tmp_data2[35] = base_ori_dot(0) * R2D; // [deg/s]
        tmp_data2[36] = base_ori_dot(1) * R2D;
        tmp_data2[37] = base_ori_dot(2) * R2D;
        tmp_data2[38] = act_base_ori_dot(0) * R2D; // [deg/s]
        tmp_data2[39] = act_base_ori_dot(1) * R2D;
        tmp_data2[40] = act_base_ori_dot(2) * R2D;
    //
    //    tmp_data2[13] = RL_foot_pos(0); //tar_RL_foot_pos_local(0); //RL_foot_pos(0);
    //    tmp_data2[14] = RL_foot_pos(1); //tar_RL_foot_pos_local(1); //RL_foot_pos(1);
    //    tmp_data2[15] = RL_foot_pos(2); //tar_RL_foot_pos_local(2); //RL_foot_pos(2);
    //    tmp_data2[16] = RR_foot_pos(0); //tar_RR_foot_pos_local(0); //act_RL_foot_pos(0);
    //    tmp_data2[17] = RR_foot_pos(1); //tar_RR_foot_pos_local(1); //act_RL_foot_pos(1);
    //    tmp_data2[18] = RR_foot_pos(2); //tar_RR_foot_pos_local(2); //act_RL_foot_pos(2);
    //    tmp_data2[19] = FL_foot_pos(0); //tar_RL_foot_pos_local(0); //RL_foot_pos(0);
    //    tmp_data2[20] = FL_foot_pos(1); //tar_RL_foot_pos_local(1); //RL_foot_pos(1);
    //    tmp_data2[21] = FL_foot_pos(2); //tar_RL_foot_pos_local(2); //RL_foot_pos(2);
    //    tmp_data2[22] = FR_foot_pos(0); //tar_RR_foot_pos_local(0); //act_RL_foot_pos(0);
    //    tmp_data2[23] = FR_foot_pos(1); //tar_RR_foot_pos_local(1); //act_RL_foot_pos(1);
    //    tmp_data2[24] = FR_foot_pos(2); //tar_RR_foot_pos_local(2); //act_RL_foot_pos(2);
    //
    //    tmp_data2[25] = act_RL_foot_pos(2); //joint[0].torque;
    //    tmp_data2[26] = act_RR_foot_pos(2); //joint[1].torque;
    //    tmp_data2[27] = act_FL_foot_pos(2); //joint[2].torque;
    //    tmp_data2[28] = act_FR_foot_pos(2); //ft_phase * 0.1;
    //
    //    tmp_data2[48] = com_acc(0);
    //    tmp_data2[49] = com_acc(1);
    //    tmp_data2[50] = com_acc(2);
    //
    //    tmp_data2[51] = act_com_acc(0);
    //    tmp_data2[52] = act_com_acc(1);
    //    tmp_data2[53] = act_com_acc(2);
    //
    //    tmp_data2[54] = Kp_q[2];
    //
    //    tmp_data2[55] = RL_foot_pos(2) - act_RL_foot_pos(2);
    //    tmp_data2[56] = RR_foot_pos(2) - act_RR_foot_pos(2);
    //    tmp_data2[57] = FL_foot_pos(2) - act_FL_foot_pos(2);
    //    tmp_data2[58] = FR_foot_pos(2) - act_FR_foot_pos(2);
    //
    //    tmp_data2[59] = IMURoll * R2D;
    //    tmp_data2[60] = IMUPitch * R2D;


        //    cout << "===========================" << endl;


     */
}

void CRobot::Pronk_Jump(void) {
    // =============== Pronk Initialize =============== //

    const double jump_ready_height = 0.45;
    const double jump_ready_time = 2.0;
    const int jump_ready_cnt = 2000;
    const double jump_stance_time = 0.300;
    const int jump_stance_cnt = 300;
    const double jump_flight_time = 0.15;
    const int jump_flight_cnt = 150;
    const double jump_landing_time = 0.3;
    const int jump_landing_cnt = 300;

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

    // =============== Pronk Initialize END =============== //

    if (pr_cnt < tmp_t1) {
        //        FC_PHASE = STOP;
        _c << 1, 1, 1, 1;
        contact_num = 4;
        JUMP_PHASE = 1;

        if (pr_cnt == 0) {
            first_jump_flag = true;
            //            moving_done_flag = false;
            //            global_jump_flight_cnt = jump_flight_cnt;
        }
    } else if (pr_cnt < tmp_t2) {
        _c << 1, 1, 1, 1;
        contact_num = 4;
        //        FC_PHASE = STOP;
        JUMP_PHASE = 2;
    } else if (pr_cnt < tmp_t3) {
        //        FC_PHASE = ZERO;
        _c << 0, 0, 0, 0;
        contact_num = 0;
        JUMP_PHASE = 3;
    } else if (pr_cnt < tmp_t4) {
        //        FC_PHASE = STOP;
        _c << 1, 1, 1, 1;
        contact_num = 4;
        JUMP_PHASE = 4;
    } else if (pr_cnt < tmp_t5) {
        //        FC_PHASE = STOP;
        _c << 1, 1, 1, 1;
        contact_num = 4;
        JUMP_PHASE = 5;
    } else {
        //        FC_PHASE = STOP;
        _c << 1, 1, 1, 1;
        contact_num = 4;
        JUMP_PHASE = 6;



        if (pr_cnt == tmp_t5) {
            moving_done_flag = true;

            com_pos[2] = tar_init_com_pos[2];
            com_vel[2] = 0;
            com_acc[2] = 0;
        }
    }


    switch (JUMP_PHASE) {
        case 1: // Jump Ready
            tmp_jump_cnt = pr_cnt;
            tmp_jump_time = (double) tmp_jump_cnt*dt;
            com_acc[2] = 0;

            if (tmp_jump_cnt == 0) {

                com_pos = tar_init_com_pos;
                com_vel = tar_init_com_vel;

                RL_foot_pos = tar_init_RL_foot_pos;
                RR_foot_pos = tar_init_RR_foot_pos;
                FL_foot_pos = tar_init_FL_foot_pos;
                FR_foot_pos = tar_init_FR_foot_pos;

                RL_foot_vel = tar_init_RL_foot_vel;
                RR_foot_vel = tar_init_RR_foot_vel;
                FL_foot_vel = tar_init_FL_foot_vel;
                FR_foot_vel = tar_init_FR_foot_vel;

                x_moving_speed = 0;
                y_moving_speed = 0;

                tmp_t[0] = jump_stance_time;
                tmp_t[1] = jump_flight_time; //jump_flight_time;
                tmp_t[2] = tmp_flight_time;
                tmp_t[3] = jump_landing_time;

                Jump_COM_Z_Traj_Gen(jump_ready_height, tmp_t);
            } else {
                com_pos[2] = tar_init_com_pos[2] + (jump_ready_height - tar_init_com_pos[2]) / 2.0 * (1 - cos(PI2 / (jump_ready_time * 2) * tmp_jump_time));
                com_vel[2] = (jump_ready_height - tar_init_com_pos[2]) / 2.0 * PI2 / (jump_ready_time * 2)*(sin(PI2 / (jump_ready_time * 2) * tmp_jump_time));
            }
            break;

        case 2: // Take-off
            tmp_jump_cnt = pr_cnt - tmp_t1;
            tmp_jump_time = (double) tmp_jump_cnt*dt;

            com_pos = tar_init_com_pos;
            com_vel = tar_init_com_vel;

            RL_foot_pos = tar_init_RL_foot_pos;
            RR_foot_pos = tar_init_RR_foot_pos;
            FL_foot_pos = tar_init_FL_foot_pos;
            FR_foot_pos = tar_init_FR_foot_pos;

            RL_foot_vel = tar_init_RL_foot_vel;
            RR_foot_vel = tar_init_RR_foot_vel;
            FL_foot_vel = tar_init_FL_foot_vel;
            FR_foot_vel = tar_init_FR_foot_vel;


            if (first_jump_flag == true) {

                com_pos[2] = fifth_order_poly(jump_z1, tmp_jump_time);
                com_vel[2] = fifth_order_poly_dot(jump_z1, tmp_jump_time);
                com_acc[2] = fifth_order_poly_2dot(jump_z1, tmp_jump_time);


                if (tmp_jump_cnt == jump_stance_cnt - 1) {
                    first_jump_flag = false;
                }

            } else {
                com_pos[2] = fifth_order_poly(jump_z1, tmp_jump_time);
                com_vel[2] = fifth_order_poly_dot(jump_z1, tmp_jump_time);
                com_acc[2] = fifth_order_poly_2dot(jump_z1, tmp_jump_time);
            }

            break;

        case 3: // Flight
            tmp_jump_cnt = pr_cnt - tmp_t2;
            tmp_jump_time = (double) tmp_jump_cnt*dt;

            com_pos = tar_init_com_pos;
            com_vel = tar_init_com_vel;

            RL_foot_pos = tar_init_RL_foot_pos;
            RR_foot_pos = tar_init_RR_foot_pos;
            FL_foot_pos = tar_init_FL_foot_pos;
            FR_foot_pos = tar_init_FR_foot_pos;

            RL_foot_vel = tar_init_RL_foot_vel;
            RR_foot_vel = tar_init_RR_foot_vel;
            FL_foot_vel = tar_init_FL_foot_vel;
            FR_foot_vel = tar_init_FR_foot_vel;

            if (tmp_jump_cnt < tmp_flight_cnt) {
                com_pos[2] = fifth_order_poly(jump_z2, tmp_jump_time);
                com_vel[2] = fifth_order_poly_dot(jump_z2, tmp_jump_time);
                com_acc[2] = fifth_order_poly_2dot(jump_z2, tmp_jump_time);

                //            com_pos[2] = jump_z2[5] * pow(tmp_jump_time, 5) + jump_z2[4] * pow(tmp_jump_time, 4) + jump_z2[3] * pow(tmp_jump_time, 3) + jump_z2[2] * pow(tmp_jump_time, 2) + jump_z2[1] * pow(tmp_jump_time, 1) + jump_z2[0];
            } else {
                com_pos[2] = fifth_order_poly(jump_z5, tmp_jump_time - tmp_flight_time);
                com_vel[2] = fifth_order_poly_dot(jump_z5, tmp_jump_time - tmp_flight_time);
                com_acc[2] = fifth_order_poly_2dot(jump_z5, tmp_jump_time - tmp_flight_time);

                //            com_pos[2] = jump_z5[5] * pow(tmp_jump_time - tmp_flight_time, 5) + jump_z5[4] * pow(tmp_jump_time - tmp_flight_time, 4) + jump_z5[3] * pow(tmp_jump_time - tmp_flight_time, 3) + jump_z5[2] * pow(tmp_jump_time - tmp_flight_time, 2) + jump_z5[1] * pow(tmp_jump_time - tmp_flight_time, 1) + jump_z5[0];
            }

            //        com_acc[2] = 0;
            break;

        case 4: // Landing
            tmp_jump_cnt = pr_cnt - tmp_t3;
            tmp_jump_time = (double) tmp_jump_cnt*dt;

            com_pos = tar_init_com_pos;
            com_vel = tar_init_com_vel;

            RL_foot_pos = tar_init_RL_foot_pos;
            RR_foot_pos = tar_init_RR_foot_pos;
            FL_foot_pos = tar_init_FL_foot_pos;
            FR_foot_pos = tar_init_FR_foot_pos;

            RL_foot_vel = tar_init_RL_foot_vel;
            RR_foot_vel = tar_init_RR_foot_vel;
            FL_foot_vel = tar_init_FL_foot_vel;
            FR_foot_vel = tar_init_FR_foot_vel;

            //        com_pos = init_com_pos;

            com_pos[2] = fifth_order_poly(jump_z4, tmp_jump_time);
            com_vel[2] = fifth_order_poly_dot(jump_z4, tmp_jump_time);
            com_acc[2] = fifth_order_poly_2dot(jump_z4, tmp_jump_time);

            //        com_pos[2] = jump_z4[5] * pow(tmp_jump_time, 5) + jump_z4[4] * pow(tmp_jump_time, 4) + jump_z4[3] * pow(tmp_jump_time, 3) + jump_z4[2] * pow(tmp_jump_time, 2) + jump_z4[1] * pow(tmp_jump_time, 1) + jump_z4[0];
            //        com_acc[2] = 20 * jump_z4[5] * pow(tmp_jump_time, 3) + 12 * jump_z4[4] * pow(tmp_jump_time, 2) + 6 * jump_z4[3] * pow(tmp_jump_time, 1) + 2 * jump_z4[2];
            break;


        case 5: // Walk Ready
            tmp_jump_cnt = pr_cnt - tmp_t4;
            tmp_jump_time = (double) tmp_jump_cnt*dt;

            com_pos = tar_init_com_pos;
            com_vel = tar_init_com_vel;

            RL_foot_pos = tar_init_RL_foot_pos;
            RR_foot_pos = tar_init_RR_foot_pos;
            FL_foot_pos = tar_init_FL_foot_pos;
            FR_foot_pos = tar_init_FR_foot_pos;

            RL_foot_vel = tar_init_RL_foot_vel;
            RR_foot_vel = tar_init_RR_foot_vel;
            FL_foot_vel = tar_init_FL_foot_vel;
            FR_foot_vel = tar_init_FR_foot_vel;

            //        com_pos = init_com_pos;

            com_pos[2] = jump_ready_height + (tar_init_com_pos[2] - jump_ready_height) / 2.0 * (1 - cos(PI2 / (jump_ready_time * 2)*(double) (tmp_jump_cnt) * dt));
            com_vel[2] = (tar_init_com_pos[2] - jump_ready_height) / 2.0 * PI2 / (jump_ready_time * 2)*(sin(PI2 / (jump_ready_time * 2)*(double) (tmp_jump_cnt) * dt));
            com_acc[2] = 0;

            break;
    }
    pr_cnt++;

    if (pr_cnt == tmp_t4) {
        if (move_stop_flag == false) { //jump_stop_flag

            pr_cnt = tmp_t1;

            jump_num++;

            printf("jump_num = %d\n", jump_num);

            //            if (T_RL == true || T_RR == true || T_FL == true || T_FR == true) {
            //                pr_cnt = tmp_t1;
            //
            //                jump_num++;
            //
            //                printf("jump_num = %d\n", jump_num);
            //            }
            //            else {
            //                printf("Not land\n");
            //                pr_cnt--;
            //            }
        }
    }

    // ============================ target_EP ========================== //

    if (Slope_con_onoff_flag == true) Slope_compensation_con();

    base_pos = com_pos + tmp_com_pos + base_offset;
}

void CRobot::Jump_COM_Z_Traj_Gen(double h0, double t[4]) {

    const double jump_h_0 = h0;
    const double jump_v_0 = 0;
    const double jump_a_0 = 0;

    const double jump_v_1 = 0.3;
    const double jump_a_1 = -GRAVITY;

    const double jump_h_2 = jump_h_0;
    const double jump_v_2 = -0.05; //-0.05;
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

void CRobot::QP_Con_Init(void) {
    cout << "QP Init Start !! " << endl;
    // Selection matrix
    S_mat.block<6, 19>(0, 0) = MatrixNd::Zero(6, 19);
    S_mat.block<13, 6>(6, 0) = MatrixNd::Zero(13, 6);
    S_mat.block<13, 13>(6, 6) = MatrixNd::Identity(13, 13);

    des_x_2dot << 0, 0, 0;
    des_w_dot << 0, 0, 0;

    //    _m = 46;//43;//53; //48.5; //kg
    _m = 53;
    //    _I_g << 1.7214, -0.0038, -0.1540,
    //            -0.0038, 4.9502, -0.0006,
    //            -0.1540, -0.0006, 5.1152;

    //    _I_g << 1.7214, 0, 0,
    //            0, 4.9502, 0,
    //            0, 0, 5.1152;

    //    _I_g << 1.5, 0, 0,
    //            0, 4.0, 0,
    //            0, 0, 2.0;
    //     _I_g << 1.5, 0, 0,
    //            0, 5.0, 0,
    //            0, 0, 2.0;

    _I_g << 1.5, 0, 0,
            0, 5.0, 0,
            0, 0, 2.0;

    //    _I_g << 0.7, 0, 0,
    //           0, 4.0, 0,
    //           0, 0, 2.0;

    _A.block<3, 3>(0, 0) = MatrixNd::Identity(3, 3);
    _A.block<3, 3>(0, 3) = MatrixNd::Identity(3, 3);
    _A.block<3, 3>(0, 6) = MatrixNd::Identity(3, 3);
    _A.block<3, 3>(0, 9) = MatrixNd::Identity(3, 3);

    tar_RL_foot_pos_local = tar_init_RL_foot_pos - init_base_pos;
    tar_RR_foot_pos_local = tar_init_RR_foot_pos - init_base_pos;
    tar_FL_foot_pos_local = tar_init_FL_foot_pos - init_base_pos;
    tar_FR_foot_pos_local = tar_init_FR_foot_pos - init_base_pos;

    cout << "tar_RL_foot_pos_local = " << tar_RL_foot_pos_local.transpose() << endl;
    cout << "tar_RR_foot_pos_local = " << tar_RR_foot_pos_local.transpose() << endl;
    cout << "tar_FL_foot_pos_local = " << tar_FL_foot_pos_local.transpose() << endl;
    cout << "tar_FR_foot_pos_local = " << tar_FR_foot_pos_local.transpose() << endl;

    p_com_oross_pro << 0, -tar_RL_foot_pos_local(2), tar_RL_foot_pos_local(1), 0, -tar_RR_foot_pos_local(2), tar_RR_foot_pos_local(1), 0, -tar_FL_foot_pos_local(2), tar_FL_foot_pos_local(1), 0, -tar_FR_foot_pos_local(2), tar_FR_foot_pos_local(1),
            tar_RL_foot_pos_local(2), 0, -tar_RL_foot_pos_local(0), tar_RR_foot_pos_local(2), 0, -tar_RR_foot_pos_local(0), tar_FL_foot_pos_local(2), 0, -tar_FL_foot_pos_local(0), tar_FR_foot_pos_local(2), 0, -tar_FR_foot_pos_local(0),
            -tar_RL_foot_pos_local(1), tar_RL_foot_pos_local(0), 0, -tar_RR_foot_pos_local(1), tar_RR_foot_pos_local(0), 0, -tar_FL_foot_pos_local(1), tar_FL_foot_pos_local(0), 0, -tar_FR_foot_pos_local(1), tar_FR_foot_pos_local(0), 0;

    _A.block<3, 12>(3, 0) = p_com_oross_pro;

    _g << 0, 0, 9.81;

    _b << _m * (des_x_2dot + _g),
            _I_g*des_w_dot;

    cout << "_b = " << _b << endl;
    //    fx_max = 50;
    //    fx_min = -50;
    //    fy_max = 50;
    //    fy_min = -50;

    _P = _A.transpose() * _S * _A + _alpha*_W;
    _q = -_A.transpose() * _S*_b;

    // ===================== OSQP  ====================== //


    int jj = 0;
    int kk = 0;
    int max_jj = 0;

    // ===================== P_x ====================== //
    for (unsigned int i = 0; i < P_nnz; ++i) {
        P_x[i] = _P(jj, kk);
        jj = jj + 1;

        if (jj > max_jj) {
            jj = 0;
            kk = kk + 1;
            max_jj = max_jj + 1;
        }
        //        cout << "i = " << i << ", P_x = " << P_x[i] << endl;
    }

    // ===================== P_i ====================== //
    jj = 0;
    max_jj = 0;

    for (unsigned int i = 0; i < P_nnz; ++i) {
        P_i[i] = jj;
        jj = jj + 1;

        if (jj > max_jj) {
            jj = 0;
            max_jj = max_jj + 1;
        }
        //        cout << "i = " << i << ", P_i = " << P_i[i] << endl;
    }

    // ===================== P_p ====================== //
    P_p[0] = 0;
    for (unsigned int i = 1; i < A_nnz + 1; ++i) {
        P_p[i] = P_p[i - 1] + i;

        //        cout << "i = " << i-1 << ", P_p = " << P_p[i-1] << endl;
    }
    //    cout << "i = " << A_nnz << ", P_p = " << P_p[A_nnz] << endl;

    // ===================== A_x ====================== //

    for (unsigned int i = 0; i < A_nnz; ++i) {
        A_x[i] = 1;

        //        cout << "i = " << i << ", A_x = " << A_x[i] << endl;
    }

    // ===================== A_i ====================== //

    for (unsigned int i = 0; i < A_nnz; ++i) {
        A_i[i] = i;

        //        cout << "i = " << i << ", A_i = " << A_i[i] << endl;
    }

    // ===================== A_p ====================== //
    jj = 0;
    for (unsigned int i = 0; i < A_nnz + 1; ++i) {
        A_p[i] = jj;

        jj = jj + 1;

        //        cout << "i = " << i << ", A_p = " << A_p[i] << endl;
    }
    //        cout << "i = " << A_nnz << ", A_p = " << A_p[A_nnz] << endl;
    // ===================== G_l & G_u ====================== //

    //    fz_max = 500;
    //    fz_min = 0;

    _c << 1, 1, 1, 1;

    if (_c(0) == 0) {
        fz_RL_max = 0;
        fz_RL_min = 0;
    } else {
        fz_RL_max = max_Fext_z;
        fz_RL_min = 0;
    }

    if (_c(1) == 0) {
        fz_RR_max = 0;
        fz_RR_min = 0;
    } else {
        fz_RR_max = max_Fext_z;
        fz_RR_min = 0;
    }

    if (_c(2) == 0) {
        fz_FL_max = 0;
        fz_FL_min = 0;
    } else {
        fz_FL_max = max_Fext_z;
        fz_FL_min = 0;
    }

    if (_c(3) == 0) {
        fz_FR_max = 0;
        fz_FR_min = 0;
    } else {
        fz_FR_max = max_Fext_z;
        fz_FR_min = 0;
    }

    Fc << 0, 0, 0, 0, 0, 0, 0, 0, 0, 140, 0, 0, 140, 0, 0, 140, 0, 0, 140;


    _d_u << mu * abs(Fc(2 + 7)), mu * abs(Fc(2 + 7)), fz_RL_max, mu * abs(Fc(5 + 7)), mu * abs(Fc(5 + 7)), fz_RR_max, mu * abs(Fc(8 + 7)), mu * abs(Fc(8 + 7)), fz_FL_max, mu * abs(Fc(11 + 7)), mu * abs(Fc(11 + 7)), fz_FR_max;
    _d_l << -mu * abs(Fc(2 + 7)), -mu * abs(Fc(2 + 7)), fz_RL_min, -mu * abs(Fc(5 + 7)), -mu * abs(Fc(5 + 7)), fz_RR_min, -mu * abs(Fc(8 + 7)), -mu * abs(Fc(8 + 7)), fz_FL_min, -mu * abs(Fc(11 + 7)), -mu * abs(Fc(11 + 7)), fz_FR_min;


    //    c_vec << _c(0),_c(0),_c(0),_c(1),_c(1),_c(1),_c(2),_c(2),_c(2),_c(3),_c(3),_c(3);

    //    cout << "c_vec = " << c_vec.transpose() << endl;

    for (unsigned int i = 0; i < A_nnz; ++i) {
        l[i] = _d_l(i);
        u[i] = _d_u(i);

        //        cout << "i = " << i << ", G_l = " << G_l[i] << endl;
        //        cout << "i = " << i << ", G_u = " << G_u[i] << endl;
        //        cout << "==============================" << endl;
    }

    for (unsigned int i = 0; i < A_nnz; ++i) {
        q[i] = _q(i);
    }

    // Populate data
    if (QP_data) {
        QP_data->n = n;
        QP_data->m = m;
        QP_data->P = csc_matrix(QP_data->n, QP_data->n, P_nnz, P_x, P_i, P_p);
        QP_data->q = q;
        QP_data->A = csc_matrix(QP_data->m, QP_data->n, A_nnz, A_x, A_i, A_p);
        QP_data->l = l;
        QP_data->u = u;
    }

    // Define solver settings as default
    if (QP_settings) {
        osqp_set_default_settings(QP_settings);
        QP_settings->alpha = 1; // Change alpha parameter
    }

    // Setup workspace
    QP_exitflag = osqp_setup(&QP_work, QP_data, QP_settings);

    // Solve Problem
    osqp_solve(QP_work);

    cout << "[RL] x = " << QP_work->solution->x[0] << ", y = " << QP_work->solution->x[1] << ", z = " << QP_work->solution->x[2] << endl;
    cout << "[RR] x = " << QP_work->solution->x[3] << ", y = " << QP_work->solution->x[4] << ", z = " << QP_work->solution->x[5] << endl;
    cout << "[FL] x = " << QP_work->solution->x[6] << ", y = " << QP_work->solution->x[7] << ", z = " << QP_work->solution->x[8] << endl;
    cout << "[FR] x = " << QP_work->solution->x[9] << ", y = " << QP_work->solution->x[10] << ", z = " << QP_work->solution->x[11] << endl;

    cout << "QP Init Done !! " << endl;
    //    cout << "solution = " << work->solution->x[1] << endl;
    //    cout << "solution = " << work->solution->x[2] << endl;

    //    // Cleanup
    //    if (data) {
    //        if (data->A) c_free(data->A);
    //        if (data->P) c_free(data->P);
    //        c_free(data);
    //    }
    //    if (settings) c_free(settings);

    // ==================== OSQP TEST END =================== //

}

void CRobot::QP_process(void) //Get_Opt_F
{
    p_com_oross_pro << 0, -tar_RL_foot_pos_local(2), tar_RL_foot_pos_local(1), 0, -tar_RR_foot_pos_local(2), tar_RR_foot_pos_local(1), 0, -tar_FL_foot_pos_local(2), tar_FL_foot_pos_local(1), 0, -tar_FR_foot_pos_local(2), tar_FR_foot_pos_local(1),
            tar_RL_foot_pos_local(2), 0, -tar_RL_foot_pos_local(0), tar_RR_foot_pos_local(2), 0, -tar_RR_foot_pos_local(0), tar_FL_foot_pos_local(2), 0, -tar_FL_foot_pos_local(0), tar_FR_foot_pos_local(2), 0, -tar_FR_foot_pos_local(0),
            -tar_RL_foot_pos_local(1), tar_RL_foot_pos_local(0), 0, -tar_RR_foot_pos_local(1), tar_RR_foot_pos_local(0), 0, -tar_FL_foot_pos_local(1), tar_FL_foot_pos_local(0), 0, -tar_FR_foot_pos_local(1), tar_FR_foot_pos_local(0), 0;

    //    p_com_oross_pro << 0, -act_RL_foot_pos_local(2), act_RL_foot_pos_local(1), 0, -act_RR_foot_pos_local(2), act_RR_foot_pos_local(1), 0, -act_FL_foot_pos_local(2), act_FL_foot_pos_local(1), 0, -act_FR_foot_pos_local(2), act_FR_foot_pos_local(1),
    //            act_RL_foot_pos_local(2), 0, -act_RL_foot_pos_local(0), act_RR_foot_pos_local(2), 0, -act_RR_foot_pos_local(0), act_FL_foot_pos_local(2), 0, -act_FL_foot_pos_local(0), act_FR_foot_pos_local(2), 0, -act_FR_foot_pos_local(0),
    //            -act_RL_foot_pos_local(1), act_RL_foot_pos_local(0), 0, -act_RR_foot_pos_local(1), act_RR_foot_pos_local(0), 0, -act_FL_foot_pos_local(1), act_FL_foot_pos_local(0), 0, -act_FR_foot_pos_local(1), act_FR_foot_pos_local(0), 0;

    _A.block<3, 12>(3, 0) = p_com_oross_pro;

    des_x_2dot = com_acc * 1.0 + Kp_x * (com_pos - act_com_pos) + Kd_x * (com_vel - act_com_vel);
    des_w_dot = Kp_w * (base_ori - act_base_ori) + Kd_w * (base_ori_dot - act_base_ori_dot);

    _b << _m * (des_x_2dot + _g),
            _I_g*des_w_dot;

    _P = _A.transpose() * _S * _A + _alpha*_W;
    _q = -_A.transpose() * _S*_b;

    // ===================== OSQP  ====================== //

    static int jj = 0;
    static int kk = 0;
    static int max_jj = 0;

    jj = 0;
    kk = 0;
    max_jj = 0;

    // ===================== P_x ====================== //
    for (unsigned int i = 0; i < P_nnz; ++i) {
        P_x[i] = _P(jj, kk);
        jj = jj + 1;

        if (jj > max_jj) {
            jj = 0;
            kk = kk + 1;
            max_jj = max_jj + 1;
        }
    }

    for (unsigned int i = 0; i < A_nnz; ++i) {
        q[i] = _q(i);
    }

    // ===================== Constraints ====================== //
    if (_c(0) == 0) {
        fz_RL_max = 0;
        fz_RL_min = 0;
    } else {
        fz_RL_max = max_Fext_z;
        fz_RL_min = 0;
    }

    if (_c(1) == 0) {
        fz_RR_max = 0;
        fz_RR_min = 0;
    } else {
        fz_RR_max = max_Fext_z;
        fz_RR_min = 0;
    }

    if (_c(2) == 0) {
        fz_FL_max = 0;
        fz_FL_min = 0;
    } else {
        fz_FL_max = max_Fext_z;
        fz_FL_min = 0;
    }

    if (_c(3) == 0) {
        fz_FR_max = 0;
        fz_FR_min = 0;
    } else {
        fz_FR_max = max_Fext_z;
        fz_FR_min = 0;
    }

    _d_u << mu * abs(Fc(2 + 7)), mu * abs(Fc(2 + 7)), fz_RL_max, mu * abs(Fc(5 + 7)), mu * abs(Fc(5 + 7)), fz_RR_max, mu * abs(Fc(8 + 7)), mu * abs(Fc(8 + 7)), fz_FL_max, mu * abs(Fc(11 + 7)), mu * abs(Fc(11 + 7)), fz_FR_max;
    _d_l << -mu * abs(Fc(2 + 7)), -mu * abs(Fc(2 + 7)), fz_RL_min, -mu * abs(Fc(5 + 7)), -mu * abs(Fc(5 + 7)), fz_RR_min, -mu * abs(Fc(8 + 7)), -mu * abs(Fc(8 + 7)), fz_FL_min, -mu * abs(Fc(11 + 7)), -mu * abs(Fc(11 + 7)), fz_FR_min;

    for (unsigned int i = 0; i < A_nnz; ++i) {
        l[i] = _d_l(i);
        u[i] = _d_u(i);
    }

    // Update problem
    osqp_update_P(QP_work, P_x, OSQP_NULL, 78);
    osqp_update_lin_cost(QP_work, q);
    osqp_update_bounds(QP_work, l, u);

    // Solve updated problem
    osqp_solve(QP_work);

    for (unsigned int i = 0; i < A_nnz; ++i) {
        Fc(i + 7) = QP_work->solution->x[i]; //fc_weight * QP_work->solution->x[i];
    }

    //    ROT_Y << cos(base_ori(1)*1), 0, sin(base_ori(1)*1),
    //            0, 1, 0,
    //            -sin(base_ori(1)*1), 0, cos(base_ori(1)*1);
    //
    //    Fc2.block(7, 0, 3, 1) << ROT_Y*Fc.block(7, 0, 3, 1);
    //    Fc2.block(10, 0, 3, 1) << ROT_Y*Fc.block(10, 0, 3, 1);
    //    Fc2.block(13, 0, 3, 1) << ROT_Y*Fc.block(13, 0, 3, 1);
    //    Fc2.block(16, 0, 3, 1) << ROT_Y*Fc.block(16, 0, 3, 1);
    //
    //
    //    cout << "Fc = " << Fc.transpose() << endl;
    //    cout << "Fc2 = " << Fc2.transpose() << endl;


}

void CRobot::WalkReady_Pos_Traj(void) {
    if (wr_cnt == 0) {
        moving_done_flag = false;
        _c << 1, 1, 1, 1;
        contact_num = 4;

        init_RL_foot_pos = act_RL_foot_pos;
        init_RR_foot_pos = act_RR_foot_pos;
        init_FL_foot_pos = act_FL_foot_pos;
        init_FR_foot_pos = act_FR_foot_pos;

        com_pos = init_com_pos;
        com_vel = init_com_vel;

        RL_foot_pos = init_RL_foot_pos;
        RR_foot_pos = init_RR_foot_pos;
        FL_foot_pos = init_FL_foot_pos;
        FR_foot_pos = init_FR_foot_pos;

        RL_foot_vel = tar_init_RL_foot_vel;
        RR_foot_vel = tar_init_RR_foot_vel;
        FL_foot_vel = tar_init_FL_foot_vel;
        FR_foot_vel = tar_init_FR_foot_vel;

        //        init_base_ori = base_ori;

        base_ori << 0, 0, 0;
        base_ori_dot << 0, 0, 0;
        tmp_com_pos << 0, 0, 0;

        //        BOC_sum_roll_err = 0.;
        //        BOC_sum_pitch_err = 0.;
        //        Slope_sum_pitch_err = 0.;
        lpf_tar_pitch_ang = 0;

        // waist
        init_pos[6] = actual_pos[6];
        target_pos[6] = actual_pos[6];

        //        init_IMUYaw = IMUYaw;

        //        if (Mode == MODE_SIMULATION) {
        //            fc_weight = 1;
        //        }
        //        else {
        //            fc_weight = 0;
        //        }
        fc_weight = 0;


        // for actual pos & vel
        pos_alpha = 1;
        vel_alpha = 1;

        wr_cnt++;
    } else if (wr_cnt <= walk_ready_cnt) {
        com_pos = init_com_pos;
        com_vel = init_com_vel;

        RL_foot_pos = init_RL_foot_pos + (tar_init_RL_foot_pos - init_RL_foot_pos) / 2.0 * (1 - cos(PI2 / (walk_ready_time * 2)*(double) (wr_cnt) * dt));
        RR_foot_pos = init_RR_foot_pos + (tar_init_RR_foot_pos - init_RR_foot_pos) / 2.0 * (1 - cos(PI2 / (walk_ready_time * 2)*(double) (wr_cnt) * dt));
        FL_foot_pos = init_FL_foot_pos + (tar_init_FL_foot_pos - init_FL_foot_pos) / 2.0 * (1 - cos(PI2 / (walk_ready_time * 2)*(double) (wr_cnt) * dt));
        FR_foot_pos = init_FR_foot_pos + (tar_init_FR_foot_pos - init_FR_foot_pos) / 2.0 * (1 - cos(PI2 / (walk_ready_time * 2)*(double) (wr_cnt) * dt));

        RL_foot_vel = (tar_init_RL_foot_pos - init_RL_foot_pos)*(PI2 / (walk_ready_time * 2)) / 2.0 * (sin(PI2 / (walk_ready_time * 2)*(double) (wr_cnt) * dt));
        RR_foot_vel = (tar_init_RR_foot_pos - init_RR_foot_pos)*(PI2 / (walk_ready_time * 2)) / 2.0 * (sin(PI2 / (walk_ready_time * 2)*(double) (wr_cnt) * dt));
        FL_foot_vel = (tar_init_FL_foot_pos - init_FL_foot_pos)*(PI2 / (walk_ready_time * 2)) / 2.0 * (sin(PI2 / (walk_ready_time * 2)*(double) (wr_cnt) * dt));
        FR_foot_vel = (tar_init_FR_foot_pos - init_FR_foot_pos)*(PI2 / (walk_ready_time * 2)) / 2.0 * (sin(PI2 / (walk_ready_time * 2)*(double) (wr_cnt) * dt));

        // tmp
        //        base_ori[1] = init_base_ori[1] + (-15*D2R - init_base_ori[1]) / 2.0 * (1 - cos(PI2 / (walk_ready_time * 2)*(double) (wr_cnt) * dt));
        // waist
        target_pos[6] = init_pos[6] + (0 - init_pos[6]) / 2.0 * (1 - cos(PI2 / (walk_ready_time * 2)*(double) (wr_cnt) * dt));

        //        if (Mode == MODE_SIMULATION) {
        //            fc_weight = 1;
        //        }
        //        else {
        //            fc_weight = 1 / 2.0 * (1 - cos(PI2 / (walk_ready_time * 2)*(double) (wr_cnt) * dt));
        //        }

        fc_weight = 1 / 2.0 * (1 - cos(PI2 / (walk_ready_time * 2)*(double) (wr_cnt) * dt));

        if (wr_cnt == 1) {
            pos_alpha = 0.02;
            vel_alpha = 0.02;
        }

        //        cout << "fc_weight = " << fc_weight << endl;
        wr_cnt++;
    } else if (wr_cnt <= walk_ready_cnt * 2) {
        com_pos = init_com_pos + (tar_init_com_pos - init_com_pos) / 2.0 * (1 - cos(PI2 / (walk_ready_time * 2)*(double) (wr_cnt - walk_ready_cnt) * dt));
        com_vel = (tar_init_com_pos - init_com_pos)*(PI2 / (walk_ready_time * 2)) / 2.0 * (sin(PI2 / (walk_ready_time * 2)*(double) (wr_cnt - walk_ready_cnt) * dt));

        RL_foot_pos = tar_init_RL_foot_pos;
        RR_foot_pos = tar_init_RR_foot_pos;
        FL_foot_pos = tar_init_FL_foot_pos;
        FR_foot_pos = tar_init_FR_foot_pos;

        RL_foot_vel = tar_init_RL_foot_vel;
        RR_foot_vel = tar_init_RR_foot_vel;
        FL_foot_vel = tar_init_FL_foot_vel;
        FR_foot_vel = tar_init_FR_foot_vel;

        // waist
        target_pos[6] = 0;

        fc_weight = 1;

        wr_cnt++;

        if (wr_cnt == walk_ready_cnt * 2) {
            walk_ready_moving_done_flag = true;
            cout << "!! Walk Ready Done !!" << endl;

            moving_done_flag = true;
        }
    } else {
        com_pos = tar_init_com_pos;
        com_vel = tar_init_com_vel;

        RL_foot_pos = tar_init_RL_foot_pos;
        RR_foot_pos = tar_init_RR_foot_pos;
        FL_foot_pos = tar_init_FL_foot_pos;
        FR_foot_pos = tar_init_FR_foot_pos;

        RL_foot_vel = tar_init_RL_foot_vel;
        RR_foot_vel = tar_init_RR_foot_vel;
        FL_foot_vel = tar_init_FL_foot_vel;
        FR_foot_vel = tar_init_FR_foot_vel;

        // waist
        target_pos[6] = 0;
    }

    //    base_pos = com_pos + base_offset;

    base_pos = com_pos + base_offset;


    //    cout << "walk_ready_moving_done_flag = " << walk_ready_moving_done_flag << endl;
    //    cout << "moving_done_flag = " << moving_done_flag << endl;
    //    cout << " ============ " << endl;
}

//void CRobot::MPC_Con_Init(void)
//{
//    // ================ MPC INIT ================== //
//
////    QP_exitflag = osqp_setup(&QP_work, QP_data, QP_settings);
//
//
//
//    _c << 1, 1, 1, 1;
//
//    cout << "<< MPC INIT Start >>" << endl;
//
//    I_g << 1.7214, 0, 0,
//            0, 4.9502, 0,
//            0, 0, 5.1152;
//
////    I_g << 1.0, 0, 0,
////            0, 4.0, 0,
////            0, 0, 2.0;
//
//    inv_I_g = I_g.inverse();
//
//    PHI <<  1, 0, 0, 0, 0, 0, Ts, 0, 0, 0, 0, 0, 0,
//            0, 1, 0, 0, 0, 0, 0, Ts, 0, 0, 0, 0, 0,
//            0, 0, 1, 0, 0, 0, 0, 0, Ts, 0, 0, 0, 0,
//            0, 0, 0, 1, 0, 0, 0, 0, 0, Ts, 0, 0, 0,
//            0, 0, 0, 0, 1, 0, 0, 0, 0, 0, Ts, 0, 0,
//            0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, Ts, -Ts * Ts / 2,
//            0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0,
//            0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0,
//            0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0,
//            0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0,
//            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0,
//            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, -Ts,
//            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1;
//
//    //cout << "PHI = " << endl << PHI << endl;
//
//    GAM << -(Ts * Ts * (RL_foot_pos(1) * inv_I_g(0, 2) - RL_foot_pos(2) * inv_I_g(0, 1))) / 2, (Ts * Ts * (RL_foot_pos(0) * inv_I_g(0, 2) - RL_foot_pos(2) * inv_I_g(0, 0))) / 2, -(Ts * Ts * (RL_foot_pos(0) * inv_I_g(0, 1) - RL_foot_pos(1) * inv_I_g(0, 0))) / 2, -(Ts * Ts * (RR_foot_pos(1) * inv_I_g(0, 2) - RR_foot_pos(2) * inv_I_g(0, 1))) / 2, (Ts * Ts * (RR_foot_pos(0) * inv_I_g(0, 2) - RR_foot_pos(2) * inv_I_g(0, 0))) / 2, -(Ts * Ts * (RR_foot_pos(0) * inv_I_g(0, 1) - RR_foot_pos(1) * inv_I_g(0, 0))) / 2, -(Ts * Ts * (FL_foot_pos(1) * inv_I_g(0, 2) - FL_foot_pos(2) * inv_I_g(0, 1))) / 2, (Ts * Ts * (FL_foot_pos(0) * inv_I_g(0, 2) - FL_foot_pos(2) * inv_I_g(0, 0))) / 2, -(Ts * Ts * (FL_foot_pos(0) * inv_I_g(0, 1) - FL_foot_pos(1) * inv_I_g(0, 0))) / 2, -(Ts * Ts * (FR_foot_pos(1) * inv_I_g(0, 2) - FR_foot_pos(2) * inv_I_g(0, 1))) / 2, (Ts * Ts * (FR_foot_pos(0) * inv_I_g(0, 2) - FR_foot_pos(2) * inv_I_g(0, 0))) / 2, -(Ts * Ts * (FR_foot_pos(0) * inv_I_g(0, 1) - FR_foot_pos(1) * inv_I_g(0, 0))) / 2,
//            -(Ts * Ts * (RL_foot_pos(1) * inv_I_g(1, 2) - RL_foot_pos(2) * inv_I_g(1, 1))) / 2, (Ts * Ts * (RL_foot_pos(0) * inv_I_g(1, 2) - RL_foot_pos(2) * inv_I_g(1, 0))) / 2, -(Ts * Ts * (RL_foot_pos(0) * inv_I_g(1, 1) - RL_foot_pos(1) * inv_I_g(1, 0))) / 2, -(Ts * Ts * (RR_foot_pos(1) * inv_I_g(1, 2) - RR_foot_pos(2) * inv_I_g(1, 1))) / 2, (Ts * Ts * (RR_foot_pos(0) * inv_I_g(1, 2) - RR_foot_pos(2) * inv_I_g(1, 0))) / 2, -(Ts * Ts * (RR_foot_pos(0) * inv_I_g(1, 1) - RR_foot_pos(1) * inv_I_g(1, 0))) / 2, -(Ts * Ts * (FL_foot_pos(1) * inv_I_g(1, 2) - FL_foot_pos(2) * inv_I_g(1, 1))) / 2, (Ts * Ts * (FL_foot_pos(0) * inv_I_g(1, 2) - FL_foot_pos(2) * inv_I_g(1, 0))) / 2, -(Ts * Ts * (FL_foot_pos(0) * inv_I_g(1, 1) - FL_foot_pos(1) * inv_I_g(1, 0))) / 2, -(Ts * Ts * (FR_foot_pos(1) * inv_I_g(1, 2) - FR_foot_pos(2) * inv_I_g(1, 1))) / 2, (Ts * Ts * (FR_foot_pos(0) * inv_I_g(1, 2) - FR_foot_pos(2) * inv_I_g(1, 0))) / 2, -(Ts * Ts * (FR_foot_pos(0) * inv_I_g(1, 1) - FR_foot_pos(1) * inv_I_g(1, 0))) / 2,
//            -(Ts * Ts * (RL_foot_pos(1) * inv_I_g(2, 2) - RL_foot_pos(2) * inv_I_g(2, 1))) / 2, (Ts * Ts * (RL_foot_pos(0) * inv_I_g(2, 2) - RL_foot_pos(2) * inv_I_g(2, 0))) / 2, -(Ts * Ts * (RL_foot_pos(0) * inv_I_g(2, 1) - RL_foot_pos(1) * inv_I_g(2, 0))) / 2, -(Ts * Ts * (RR_foot_pos(1) * inv_I_g(2, 2) - RR_foot_pos(2) * inv_I_g(2, 1))) / 2, (Ts * Ts * (RR_foot_pos(0) * inv_I_g(2, 2) - RR_foot_pos(2) * inv_I_g(2, 0))) / 2, -(Ts * Ts * (RR_foot_pos(0) * inv_I_g(2, 1) - RR_foot_pos(1) * inv_I_g(2, 0))) / 2, -(Ts * Ts * (FL_foot_pos(1) * inv_I_g(2, 2) - FL_foot_pos(2) * inv_I_g(2, 1))) / 2, (Ts * Ts * (FL_foot_pos(0) * inv_I_g(2, 2) - FL_foot_pos(2) * inv_I_g(2, 0))) / 2, -(Ts * Ts * (FL_foot_pos(0) * inv_I_g(2, 1) - FL_foot_pos(1) * inv_I_g(2, 0))) / 2, -(Ts * Ts * (FR_foot_pos(1) * inv_I_g(2, 2) - FR_foot_pos(2) * inv_I_g(2, 1))) / 2, (Ts * Ts * (FR_foot_pos(0) * inv_I_g(2, 2) - FR_foot_pos(2) * inv_I_g(2, 0))) / 2, -(Ts * Ts * (FR_foot_pos(0) * inv_I_g(2, 1) - FR_foot_pos(1) * inv_I_g(2, 0))) / 2,
//            Ts * Ts / (2 * mass), 0, 0, Ts * Ts / (2 * mass), 0, 0, Ts * Ts / (2 * mass), 0, 0, Ts * Ts / (2 * mass), 0, 0,
//            0, Ts * Ts / (2 * mass), 0, 0, Ts * Ts / (2 * mass), 0, 0, Ts * Ts / (2 * mass), 0, 0, Ts * Ts / (2 * mass), 0,
//            0, 0, Ts * Ts / (2 * mass), 0, 0, Ts * Ts / (2 * mass), 0, 0, Ts * Ts / (2 * mass), 0, 0, Ts * Ts / (2 * mass),
//            -Ts * (RL_foot_pos(1) * inv_I_g(0, 2) - RL_foot_pos(2) * inv_I_g(0, 1)), Ts * (RL_foot_pos(0) * inv_I_g(0, 2) - RL_foot_pos(2) * inv_I_g(0, 0)), -Ts * (RL_foot_pos(0) * inv_I_g(0, 1) - RL_foot_pos(1) * inv_I_g(0, 0)), -Ts * (RR_foot_pos(1) * inv_I_g(0, 2) - RR_foot_pos(2) * inv_I_g(0, 1)), Ts * (RR_foot_pos(0) * inv_I_g(0, 2) - RR_foot_pos(2) * inv_I_g(0, 0)), -Ts * (RR_foot_pos(0) * inv_I_g(0, 1) - RR_foot_pos(1) * inv_I_g(0, 0)), -Ts * (FL_foot_pos(1) * inv_I_g(0, 2) - FL_foot_pos(2) * inv_I_g(0, 1)), Ts * (FL_foot_pos(0) * inv_I_g(0, 2) - FL_foot_pos(2) * inv_I_g(0, 0)), -Ts * (FL_foot_pos(0) * inv_I_g(0, 1) - FL_foot_pos(1) * inv_I_g(0, 0)), -Ts * (FR_foot_pos(1) * inv_I_g(0, 2) - FR_foot_pos(2) * inv_I_g(0, 1)), Ts * (FR_foot_pos(0) * inv_I_g(0, 2) - FR_foot_pos(2) * inv_I_g(0, 0)), -Ts * (FR_foot_pos(0) * inv_I_g(0, 1) - FR_foot_pos(1) * inv_I_g(0, 0)),
//            -Ts * (RL_foot_pos(1) * inv_I_g(1, 2) - RL_foot_pos(2) * inv_I_g(1, 1)), Ts * (RL_foot_pos(0) * inv_I_g(1, 2) - RL_foot_pos(2) * inv_I_g(1, 0)), -Ts * (RL_foot_pos(0) * inv_I_g(1, 1) - RL_foot_pos(1) * inv_I_g(1, 0)), -Ts * (RR_foot_pos(1) * inv_I_g(1, 2) - RR_foot_pos(2) * inv_I_g(1, 1)), Ts * (RR_foot_pos(0) * inv_I_g(1, 2) - RR_foot_pos(2) * inv_I_g(1, 0)), -Ts * (RR_foot_pos(0) * inv_I_g(1, 1) - RR_foot_pos(1) * inv_I_g(1, 0)), -Ts * (FL_foot_pos(1) * inv_I_g(1, 2) - FL_foot_pos(2) * inv_I_g(1, 1)), Ts * (FL_foot_pos(0) * inv_I_g(1, 2) - FL_foot_pos(2) * inv_I_g(1, 0)), -Ts * (FL_foot_pos(0) * inv_I_g(1, 1) - FL_foot_pos(1) * inv_I_g(1, 0)), -Ts * (FR_foot_pos(1) * inv_I_g(1, 2) - FR_foot_pos(2) * inv_I_g(1, 1)), Ts * (FR_foot_pos(0) * inv_I_g(1, 2) - FR_foot_pos(2) * inv_I_g(1, 0)), -Ts * (FR_foot_pos(0) * inv_I_g(1, 1) - FR_foot_pos(1) * inv_I_g(1, 0)),
//            -Ts * (RL_foot_pos(1) * inv_I_g(2, 2) - RL_foot_pos(2) * inv_I_g(2, 1)), Ts * (RL_foot_pos(0) * inv_I_g(2, 2) - RL_foot_pos(2) * inv_I_g(2, 0)), -Ts * (RL_foot_pos(0) * inv_I_g(2, 1) - RL_foot_pos(1) * inv_I_g(2, 0)), -Ts * (RR_foot_pos(1) * inv_I_g(2, 2) - RR_foot_pos(2) * inv_I_g(2, 1)), Ts * (RR_foot_pos(0) * inv_I_g(2, 2) - RR_foot_pos(2) * inv_I_g(2, 0)), -Ts * (RR_foot_pos(0) * inv_I_g(2, 1) - RR_foot_pos(1) * inv_I_g(2, 0)), -Ts * (FL_foot_pos(1) * inv_I_g(2, 2) - FL_foot_pos(2) * inv_I_g(2, 1)), Ts * (FL_foot_pos(0) * inv_I_g(2, 2) - FL_foot_pos(2) * inv_I_g(2, 0)), -Ts * (FL_foot_pos(0) * inv_I_g(2, 1) - FL_foot_pos(1) * inv_I_g(2, 0)), -Ts * (FR_foot_pos(1) * inv_I_g(2, 2) - FR_foot_pos(2) * inv_I_g(2, 1)), Ts * (FR_foot_pos(0) * inv_I_g(2, 2) - FR_foot_pos(2) * inv_I_g(2, 0)), -Ts * (FR_foot_pos(0) * inv_I_g(2, 1) - FR_foot_pos(1) * inv_I_g(2, 0)),
//            Ts / mass, 0, 0, Ts / mass, 0, 0, Ts / mass, 0, 0, Ts / mass, 0, 0,
//            0, Ts / mass, 0, 0, Ts / mass, 0, 0, Ts / mass, 0, 0, Ts / mass, 0,
//            0, 0, Ts / mass, 0, 0, Ts / mass, 0, 0, Ts / mass, 0, 0, Ts / mass,
//            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
//
//    //cout << "GAM = " << endl << GAM << endl;
//
////    _Q_vec << 10, 10, 0, // body ori
////            1, 1, 1, // com pos
////            0.1, 0.1, 0, // body ori dot
////            0.01, 0.01, 0.01, 0; // com vel
//
//    _Q_vec << 10, 10, 0, // body ori
//              1, 1, 1000, // com pos
//              0.1, 0.1, 0, // body ori dot
//              0.01, 0.01, 1, 0; // com vel
//
//    for (unsigned int i = 0; i < output_num; ++i) {
//        _Q(i, i) = _Q_vec(i);
//    }
//
//    //cout << "_Q = " << _Q << endl;
//    //cout << "_R = " << _R << endl;
//
//    Q_tilda = Kron(MatrixNd::Identity(Np, Np), _Q);
//    //cout << "Q_tilda = " << Q_tilda << endl;
//
//
//    R_tilda = Kron(MatrixNd::Identity(Nc, Nc),_R);
////    cout << "R_tilda = " << R_tilda << endl;
//
////    ref_y << 0, 0, 0, // body ori
////            0.0, 0, 0.45, // com pos
////            0, 0, 0, // body ori dot
////            0, 0, 0, g; // com vel
//
//    ref_y << init_base_ori, // body ori
//             init_com_pos, // com pos
//             init_base_ori_dot, // body ori dot
//             init_com_vel, g; // com vel
//
////    cout <<
//    cout << "ref_y = " << ref_y.transpose() << endl;
//
//    ref_tilda = Kron(VectorNd::Ones(Np,1), ref_y);
//
//    for (int i = 0; i < Np; ++i) {
//        tmp_Ax = pow_mat(PHI, i);
//
//        Ax_tilda.block(i*state_num, 0, state_num, state_num) = tmp_Ax;
//    }
//
//    //    cout << "Ax_tilda = " << Ax_tilda << endl;
//
//    for (int i = 0; i < Np - 1; ++i) {
//        Bx_element = pow_mat(PHI, i) * GAM;
//
//        for (int j = 0; j < Np - i - 1; ++j) {
//            Bx_tilda.block(state_num*(i+j+1),input_num*(j),state_num, input_num) = Bx_element;
//        }
//    }
//
//    C_tilda = Kron(MatrixNd::Identity(Np, Np), C);
//
//    H = Bx_tilda.transpose() * C_tilda.transpose() * Q_tilda * C_tilda * Bx_tilda + R_tilda;
//
//    H = (H + H.transpose()) / 2;
//    //    cout << "new H = " << endl << H << endl;
//
////    cout << "H.cols = " << H.cols() << ", H.rows = " << H.rows() << endl;
//
//    // ================= OSQP Setting ================= //
//
//    int jj = 0;
//    int kk = 0;
//    int max_jj = 0;
//
//    // ===================== H_x ====================== //
//    for (unsigned int i = 0; i < H_nnz; ++i) {
//        H_x[i] = H(jj, kk);
//        jj = jj + 1;
//
//        if (jj > max_jj) {
//            jj = 0;
//            kk = kk + 1;
//            max_jj = max_jj + 1;
//        }
//        //        cout << "i = " << i << ", H_x = " << H_x[i] << endl;
//    }
//
//    // ===================== H_i ====================== //
//    jj = 0;
//    max_jj = 0;
//
//    for (unsigned int i = 0; i < H_nnz; ++i) {
//        H_i[i] = jj;
//        jj = jj + 1;
//
//        if (jj > max_jj) {
//            jj = 0;
//            max_jj = max_jj + 1;
//        }
//
//        //        cout << "i = " << i << ", H_i = " << H_i[i] << endl;
//    }
//
//    // ===================== H_p ====================== //
//    H_p[0] = 0;
//    for (unsigned int i = 1; i < Nc * input_num + 1; ++i) {
//        H_p[i] = H_p[i - 1] + i;
//
//        //        cout << "i = " << i-1 << ", H_p = " << H_p[i-1] << endl;
//    }
//
//    // ===================== G_x ====================== //
//
//    for (unsigned int i = 0; i < Nc * input_num; ++i) {
//        G_x[i] = 1;
//        //        cout << "i = " << i << ", G_x = " << G_x[i] << endl;
//    }
//
//    // ===================== G_i ====================== //
//
//    for (unsigned int i = 0; i < Nc * input_num; ++i) {
//        G_i[i] = i;
//        //        cout << "i = " << i << ", G_i = " << G_i[i] << endl;
//    }
//
//    // ===================== G_p ====================== //
//    jj = 0;
//    for (unsigned int i = 0; i < Nc * input_num + 1; ++i) {
//        G_p[i] = jj;
//
//        jj = jj + 1;
//        //        cout << "i = " << i << ", G_p = " << G_p[i] << endl;
//    }
//
//    // ===================== G_l & G_u ====================== //
//
//    // ===================== Constraints ====================== //
//    if (_c(0) == 0) {
//        fz_RL_max = 0;
//        fz_RL_min = 0;
//    }
//    else {
//        fz_RL_max = max_Fext_z;
//        fz_RL_min = 0;
//    }
//
//    if (_c(1) == 0) {
//        fz_RR_max = 0;
//        fz_RR_min = 0;
//    }
//    else {
//        fz_RR_max = max_Fext_z;
//        fz_RR_min = 0;
//    }
//
//    if (_c(2) == 0) {
//        fz_FL_max = 0;
//        fz_FL_min = 0;
//    }
//    else {
//        fz_FL_max = max_Fext_z;
//        fz_FL_min = 0;
//    }
//
//    if (_c(3) == 0) {
//        fz_FR_max = 0;
//        fz_FR_min = 0;
//    }
//    else {
//        fz_FR_max = max_Fext_z;
//        fz_FR_min = 0;
//    }
//
//    Fc << 0, 0, 0, 0, 0, 0, 0, 0, 0, 140, 0, 0, 140, 0, 0, 140, 0, 0, 140;
//
//    _d_u << mu * abs(Fc(2 + 7)), mu * abs(Fc(2 + 7)), fz_RL_max, mu * abs(Fc(5 + 7)), mu * abs(Fc(5 + 7)), fz_RR_max, mu * abs(Fc(8 + 7)), mu * abs(Fc(8 + 7)), fz_FL_max, mu * abs(Fc(11 + 7)), mu * abs(Fc(11 + 7)), fz_FR_max;
//    _d_l << -mu * abs(Fc(2 + 7)), -mu * abs(Fc(2 + 7)), fz_RL_min, -mu * abs(Fc(5 + 7)), -mu * abs(Fc(5 + 7)), fz_RR_min, -mu * abs(Fc(8 + 7)), -mu * abs(Fc(8 + 7)), fz_FL_min, -mu * abs(Fc(11 + 7)), -mu * abs(Fc(11 + 7)), fz_FR_min;
//
//
//    jj = 0;
//    for (unsigned int i = 0; i < Nc * input_num; ++i) {
//        G_l[i] = _d_l[jj];//-input_const(jj);
//        G_u[i] = _d_u[jj];//input_const(jj);
//
//        jj++;
//        if (jj == input_num) {
//            jj = 0;
//        }
//    }
//
////    act_state_x << act_base_ori(0), act_base_ori(1), 0,
////            act_com_pos(0), act_com_pos(1), act_com_pos(2),
////            act_base_ori_dot(0), act_base_ori_dot(1), 0,
////            act_com_vel(0), act_com_vel(1), act_com_vel(2), g;
//
//    act_state_x = ref_y;
//
//    cout << "act_state_x = " << act_state_x.transpose() << endl;
//
//    f = (act_state_x.transpose() * Ax_tilda.transpose() *C_tilda.transpose()  - ref_tilda.transpose()) * Q_tilda * C_tilda * Bx_tilda;//.transpose();
//
//
////    cout << "f = " << f.transpose() << endl;
//
////    cout << "f.rows = " << f.rows() << ", f.cols = " << f.cols() << endl;
//
////    cout << "ff = " ;
//    for (unsigned int i = 0; i < Nc * input_num; ++i) {
//        ff[i] = f(i);
////        cout << ff[i] << " ";
//    }
////    cout << endl;
//    //    cout << "ff = " << ff[0] << endl;
//    // ==================== OSQP ======================= //
//
//    // Exitflag
//    MPC_exitflag = 0;
//
//    cout << "[1]" << endl;
//    // Populate data
//    if (MPC_data) {
//        MPC_data->n = G_n;
//        MPC_data->m = G_m;
//        MPC_data->P = csc_matrix(MPC_data->n, MPC_data->n, H_nnz, H_x, H_i, H_p);
//        MPC_data->q = ff;
//        MPC_data->A = csc_matrix(MPC_data->m, MPC_data->n, G_nnz, G_x, G_i, G_p);
//        MPC_data->l = G_l;
//        MPC_data->u = G_u;
//
//        //        cout << "G_n = " << G_n << ", G_m = " << G_m << endl;
//
//    }
//
//    cout << "[2]" << endl;
//    // Define solver settings as default
//    if (MPC_settings) {
//        osqp_set_default_settings(MPC_settings);
//        MPC_settings->alpha = 1; // Change alpha parameter
//        cout << "[2-1]" << endl;
//    }
//
//    cout << "[3]" << endl;
//
//    // Setup workspace
//    MPC_exitflag = osqp_setup(&MPC_work, MPC_data, MPC_settings);
//
//    cout << "[4]" << endl;
//
//    // Solve Problem
//    osqp_solve(MPC_work);
//
//    for (unsigned int i = 0; i < input_num; ++i) {
//        Fc(i + 7) = MPC_work->solution->x[i];
//    }
//
//    cout << "[RL] x = " << MPC_work->solution->x[0] << ", y = " << MPC_work->solution->x[1] << ", z = " << MPC_work->solution->x[2] << endl;
//    cout << "[RR] x = " << MPC_work->solution->x[3] << ", y = " << MPC_work->solution->x[4] << ", z = " << MPC_work->solution->x[5] << endl;
//    cout << "[FL] x = " << MPC_work->solution->x[6] << ", y = " << MPC_work->solution->x[7] << ", z = " << MPC_work->solution->x[8] << endl;
//    cout << "[FR] x = " << MPC_work->solution->x[9] << ", y = " << MPC_work->solution->x[10] << ", z = " << MPC_work->solution->x[11] << endl;
//
//
//    cout << "[5] MPC Init Done !!!" << endl;
//}

//void CRobot::MPC_process(void)
//{
////    _c << 1, 1, 1, 1;
//
////	cout << "[1]" << endl;
//
//    GAM << -(Ts * Ts * (RL_foot_pos(1) * inv_I_g(0, 2) - RL_foot_pos(2) * inv_I_g(0, 1))) / 2, (Ts * Ts * (RL_foot_pos(0) * inv_I_g(0, 2) - RL_foot_pos(2) * inv_I_g(0, 0))) / 2, -(Ts * Ts * (RL_foot_pos(0) * inv_I_g(0, 1) - RL_foot_pos(1) * inv_I_g(0, 0))) / 2, -(Ts * Ts * (RR_foot_pos(1) * inv_I_g(0, 2) - RR_foot_pos(2) * inv_I_g(0, 1))) / 2, (Ts * Ts * (RR_foot_pos(0) * inv_I_g(0, 2) - RR_foot_pos(2) * inv_I_g(0, 0))) / 2, -(Ts * Ts * (RR_foot_pos(0) * inv_I_g(0, 1) - RR_foot_pos(1) * inv_I_g(0, 0))) / 2, -(Ts * Ts * (FL_foot_pos(1) * inv_I_g(0, 2) - FL_foot_pos(2) * inv_I_g(0, 1))) / 2, (Ts * Ts * (FL_foot_pos(0) * inv_I_g(0, 2) - FL_foot_pos(2) * inv_I_g(0, 0))) / 2, -(Ts * Ts * (FL_foot_pos(0) * inv_I_g(0, 1) - FL_foot_pos(1) * inv_I_g(0, 0))) / 2, -(Ts * Ts * (FR_foot_pos(1) * inv_I_g(0, 2) - FR_foot_pos(2) * inv_I_g(0, 1))) / 2, (Ts * Ts * (FR_foot_pos(0) * inv_I_g(0, 2) - FR_foot_pos(2) * inv_I_g(0, 0))) / 2, -(Ts * Ts * (FR_foot_pos(0) * inv_I_g(0, 1) - FR_foot_pos(1) * inv_I_g(0, 0))) / 2,
//            -(Ts * Ts * (RL_foot_pos(1) * inv_I_g(1, 2) - RL_foot_pos(2) * inv_I_g(1, 1))) / 2, (Ts * Ts * (RL_foot_pos(0) * inv_I_g(1, 2) - RL_foot_pos(2) * inv_I_g(1, 0))) / 2, -(Ts * Ts * (RL_foot_pos(0) * inv_I_g(1, 1) - RL_foot_pos(1) * inv_I_g(1, 0))) / 2, -(Ts * Ts * (RR_foot_pos(1) * inv_I_g(1, 2) - RR_foot_pos(2) * inv_I_g(1, 1))) / 2, (Ts * Ts * (RR_foot_pos(0) * inv_I_g(1, 2) - RR_foot_pos(2) * inv_I_g(1, 0))) / 2, -(Ts * Ts * (RR_foot_pos(0) * inv_I_g(1, 1) - RR_foot_pos(1) * inv_I_g(1, 0))) / 2, -(Ts * Ts * (FL_foot_pos(1) * inv_I_g(1, 2) - FL_foot_pos(2) * inv_I_g(1, 1))) / 2, (Ts * Ts * (FL_foot_pos(0) * inv_I_g(1, 2) - FL_foot_pos(2) * inv_I_g(1, 0))) / 2, -(Ts * Ts * (FL_foot_pos(0) * inv_I_g(1, 1) - FL_foot_pos(1) * inv_I_g(1, 0))) / 2, -(Ts * Ts * (FR_foot_pos(1) * inv_I_g(1, 2) - FR_foot_pos(2) * inv_I_g(1, 1))) / 2, (Ts * Ts * (FR_foot_pos(0) * inv_I_g(1, 2) - FR_foot_pos(2) * inv_I_g(1, 0))) / 2, -(Ts * Ts * (FR_foot_pos(0) * inv_I_g(1, 1) - FR_foot_pos(1) * inv_I_g(1, 0))) / 2,
//            -(Ts * Ts * (RL_foot_pos(1) * inv_I_g(2, 2) - RL_foot_pos(2) * inv_I_g(2, 1))) / 2, (Ts * Ts * (RL_foot_pos(0) * inv_I_g(2, 2) - RL_foot_pos(2) * inv_I_g(2, 0))) / 2, -(Ts * Ts * (RL_foot_pos(0) * inv_I_g(2, 1) - RL_foot_pos(1) * inv_I_g(2, 0))) / 2, -(Ts * Ts * (RR_foot_pos(1) * inv_I_g(2, 2) - RR_foot_pos(2) * inv_I_g(2, 1))) / 2, (Ts * Ts * (RR_foot_pos(0) * inv_I_g(2, 2) - RR_foot_pos(2) * inv_I_g(2, 0))) / 2, -(Ts * Ts * (RR_foot_pos(0) * inv_I_g(2, 1) - RR_foot_pos(1) * inv_I_g(2, 0))) / 2, -(Ts * Ts * (FL_foot_pos(1) * inv_I_g(2, 2) - FL_foot_pos(2) * inv_I_g(2, 1))) / 2, (Ts * Ts * (FL_foot_pos(0) * inv_I_g(2, 2) - FL_foot_pos(2) * inv_I_g(2, 0))) / 2, -(Ts * Ts * (FL_foot_pos(0) * inv_I_g(2, 1) - FL_foot_pos(1) * inv_I_g(2, 0))) / 2, -(Ts * Ts * (FR_foot_pos(1) * inv_I_g(2, 2) - FR_foot_pos(2) * inv_I_g(2, 1))) / 2, (Ts * Ts * (FR_foot_pos(0) * inv_I_g(2, 2) - FR_foot_pos(2) * inv_I_g(2, 0))) / 2, -(Ts * Ts * (FR_foot_pos(0) * inv_I_g(2, 1) - FR_foot_pos(1) * inv_I_g(2, 0))) / 2,
//            Ts * Ts / (2 * mass), 0, 0, Ts * Ts / (2 * mass), 0, 0, Ts * Ts / (2 * mass), 0, 0, Ts * Ts / (2 * mass), 0, 0,
//            0, Ts * Ts / (2 * mass), 0, 0, Ts * Ts / (2 * mass), 0, 0, Ts * Ts / (2 * mass), 0, 0, Ts * Ts / (2 * mass), 0,
//            0, 0, Ts * Ts / (2 * mass), 0, 0, Ts * Ts / (2 * mass), 0, 0, Ts * Ts / (2 * mass), 0, 0, Ts * Ts / (2 * mass),
//            -Ts * (RL_foot_pos(1) * inv_I_g(0, 2) - RL_foot_pos(2) * inv_I_g(0, 1)), Ts * (RL_foot_pos(0) * inv_I_g(0, 2) - RL_foot_pos(2) * inv_I_g(0, 0)), -Ts * (RL_foot_pos(0) * inv_I_g(0, 1) - RL_foot_pos(1) * inv_I_g(0, 0)), -Ts * (RR_foot_pos(1) * inv_I_g(0, 2) - RR_foot_pos(2) * inv_I_g(0, 1)), Ts * (RR_foot_pos(0) * inv_I_g(0, 2) - RR_foot_pos(2) * inv_I_g(0, 0)), -Ts * (RR_foot_pos(0) * inv_I_g(0, 1) - RR_foot_pos(1) * inv_I_g(0, 0)), -Ts * (FL_foot_pos(1) * inv_I_g(0, 2) - FL_foot_pos(2) * inv_I_g(0, 1)), Ts * (FL_foot_pos(0) * inv_I_g(0, 2) - FL_foot_pos(2) * inv_I_g(0, 0)), -Ts * (FL_foot_pos(0) * inv_I_g(0, 1) - FL_foot_pos(1) * inv_I_g(0, 0)), -Ts * (FR_foot_pos(1) * inv_I_g(0, 2) - FR_foot_pos(2) * inv_I_g(0, 1)), Ts * (FR_foot_pos(0) * inv_I_g(0, 2) - FR_foot_pos(2) * inv_I_g(0, 0)), -Ts * (FR_foot_pos(0) * inv_I_g(0, 1) - FR_foot_pos(1) * inv_I_g(0, 0)),
//            -Ts * (RL_foot_pos(1) * inv_I_g(1, 2) - RL_foot_pos(2) * inv_I_g(1, 1)), Ts * (RL_foot_pos(0) * inv_I_g(1, 2) - RL_foot_pos(2) * inv_I_g(1, 0)), -Ts * (RL_foot_pos(0) * inv_I_g(1, 1) - RL_foot_pos(1) * inv_I_g(1, 0)), -Ts * (RR_foot_pos(1) * inv_I_g(1, 2) - RR_foot_pos(2) * inv_I_g(1, 1)), Ts * (RR_foot_pos(0) * inv_I_g(1, 2) - RR_foot_pos(2) * inv_I_g(1, 0)), -Ts * (RR_foot_pos(0) * inv_I_g(1, 1) - RR_foot_pos(1) * inv_I_g(1, 0)), -Ts * (FL_foot_pos(1) * inv_I_g(1, 2) - FL_foot_pos(2) * inv_I_g(1, 1)), Ts * (FL_foot_pos(0) * inv_I_g(1, 2) - FL_foot_pos(2) * inv_I_g(1, 0)), -Ts * (FL_foot_pos(0) * inv_I_g(1, 1) - FL_foot_pos(1) * inv_I_g(1, 0)), -Ts * (FR_foot_pos(1) * inv_I_g(1, 2) - FR_foot_pos(2) * inv_I_g(1, 1)), Ts * (FR_foot_pos(0) * inv_I_g(1, 2) - FR_foot_pos(2) * inv_I_g(1, 0)), -Ts * (FR_foot_pos(0) * inv_I_g(1, 1) - FR_foot_pos(1) * inv_I_g(1, 0)),
//            -Ts * (RL_foot_pos(1) * inv_I_g(2, 2) - RL_foot_pos(2) * inv_I_g(2, 1)), Ts * (RL_foot_pos(0) * inv_I_g(2, 2) - RL_foot_pos(2) * inv_I_g(2, 0)), -Ts * (RL_foot_pos(0) * inv_I_g(2, 1) - RL_foot_pos(1) * inv_I_g(2, 0)), -Ts * (RR_foot_pos(1) * inv_I_g(2, 2) - RR_foot_pos(2) * inv_I_g(2, 1)), Ts * (RR_foot_pos(0) * inv_I_g(2, 2) - RR_foot_pos(2) * inv_I_g(2, 0)), -Ts * (RR_foot_pos(0) * inv_I_g(2, 1) - RR_foot_pos(1) * inv_I_g(2, 0)), -Ts * (FL_foot_pos(1) * inv_I_g(2, 2) - FL_foot_pos(2) * inv_I_g(2, 1)), Ts * (FL_foot_pos(0) * inv_I_g(2, 2) - FL_foot_pos(2) * inv_I_g(2, 0)), -Ts * (FL_foot_pos(0) * inv_I_g(2, 1) - FL_foot_pos(1) * inv_I_g(2, 0)), -Ts * (FR_foot_pos(1) * inv_I_g(2, 2) - FR_foot_pos(2) * inv_I_g(2, 1)), Ts * (FR_foot_pos(0) * inv_I_g(2, 2) - FR_foot_pos(2) * inv_I_g(2, 0)), -Ts * (FR_foot_pos(0) * inv_I_g(2, 1) - FR_foot_pos(1) * inv_I_g(2, 0)),
//            Ts / mass, 0, 0, Ts / mass, 0, 0, Ts / mass, 0, 0, Ts / mass, 0, 0,
//            0, Ts / mass, 0, 0, Ts / mass, 0, 0, Ts / mass, 0, 0, Ts / mass, 0,
//            0, 0, Ts / mass, 0, 0, Ts / mass, 0, 0, Ts / mass, 0, 0, Ts / mass,
//            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
//
//    for (int i = 0; i < Np - 1; ++i) {
//        Bx_element = pow_mat(PHI, i) * GAM;
//
//        for (int j = 0; j < Np - i - 1; ++j) {
//            Bx_tilda.block(state_num*(i+j+1),input_num*(j),state_num, input_num) = Bx_element;
//        }
//    }
//
////    cout << "[2]" << endl;
//
//    H = Bx_tilda.transpose() * C_tilda.transpose() * Q_tilda * C_tilda  * Bx_tilda + R_tilda;
//    H = (H + H.transpose()) / 2;
//
////    cout << "[3]" << endl;
//    static int jjj = 0;
//    static int kkk = 0;
//    static int max_jjj = 0;
//
//    jjj = 0;
//    kkk = 0;
//    max_jjj = 0;
//
//    // ===================== H_x ====================== //
//    for (unsigned int i = 0; i < H_nnz; ++i) {
//        H_x[i] = H(jjj, kkk);
//        jjj = jjj + 1;
//
//        if (jjj > max_jjj) {
//            jjj = 0;
//            kkk = kkk + 1;
//            max_jjj = max_jjj + 1;
//        }
//        //        cout << "i = " << i << ", H_x = " << H_x[i] << endl;
//    }
//
//    // ===================== G_l & G_u ====================== //
//
//    if (_c(0) == 0) {
//        fz_RL_max = 0;
//        fz_RL_min = 0;
//    }
//    else {
//        fz_RL_max = max_Fext_z;
//        fz_RL_min = 0;
//    }
//
//    if (_c(1) == 0) {
//        fz_RR_max = 0;
//        fz_RR_min = 0;
//    }
//    else {
//        fz_RR_max = max_Fext_z;
//        fz_RR_min = 0;
//    }
//
//    if (_c(2) == 0) {
//        fz_FL_max = 0;
//        fz_FL_min = 0;
//    }
//    else {
//        fz_FL_max = max_Fext_z;
//        fz_FL_min = 0;
//    }
//
//    if (_c(3) == 0) {
//        fz_FR_max = 0;
//        fz_FR_min = 0;
//    }
//    else {
//        fz_FR_max = max_Fext_z;
//        fz_FR_min = 0;
//    }
//
//    _d_u << mu * abs(Fc(2 + 7)), mu * abs(Fc(2 + 7)), fz_RL_max, mu * abs(Fc(5 + 7)), mu * abs(Fc(5 + 7)), fz_RR_max, mu * abs(Fc(8 + 7)), mu * abs(Fc(8 + 7)), fz_FL_max, mu * abs(Fc(11 + 7)), mu * abs(Fc(11 + 7)), fz_FR_max;
//    _d_l << -mu * abs(Fc(2 + 7)), -mu * abs(Fc(2 + 7)), fz_RL_min, -mu * abs(Fc(5 + 7)), -mu * abs(Fc(5 + 7)), fz_RR_min, -mu * abs(Fc(8 + 7)), -mu * abs(Fc(8 + 7)), fz_FL_min, -mu * abs(Fc(11 + 7)), -mu * abs(Fc(11 + 7)), fz_FR_min;
//
//    jjj = 0;
//    for (unsigned int i = 0; i < Nc * input_num; ++i) {
//        G_l[i] = _d_l[jjj];
//        G_u[i] = _d_u[jjj];
//
//        jjj++;
//        if (jjj == input_num) {
//            jjj = 0;
//        }
//    }
//
////    cout << "[4]" << endl;
//
//    act_state_x << act_base_ori(0), act_base_ori(1), 0,
//            act_com_pos(0), act_com_pos(1), act_com_pos(2),
//            act_base_ori_dot(0), act_base_ori_dot(1), 0,
//            act_com_vel(0), act_com_vel(1), act_com_vel(2), g;
//
//
////    act_state_x = ref_y;
//
//    ref_y << base_ori, // body ori
//             com_pos, // com pos
//             base_ori_dot, // body ori dot
//             com_vel, g; // com vel
//
//    ref_tilda = Kron(VectorNd::Ones(Np,1), ref_y);
//
//    f = (act_state_x.transpose() * Ax_tilda.transpose() * C_tilda.transpose()  - ref_tilda.transpose()) * Q_tilda * C_tilda * Bx_tilda;//.transpose();
//
////    cout << "f = " << f(0) << ", " << f(1) << ", " << f(2) << ", " << f(3) << ", " << f(4) << ", " << f(5) << endl;
////    cout << "f = " << f.transpose() << endl;
//
//    for (unsigned int i = 0; i < Nc * input_num; ++i) {
//        ff[i] = f(i);
//    }
//
//    // Update problem
//    osqp_update_P(MPC_work, H_x, OSQP_NULL, 7260);
//    osqp_update_lin_cost(MPC_work, ff);
//    osqp_update_bounds(MPC_work, G_l, G_u);
//
//    // Solve updated problem
//    osqp_solve(MPC_work);
//
//    for (unsigned int i = 0; i < input_num; ++i) {
//        Fc(i + 7) = MPC_work->solution->x[i];
//    }
//
////    cout << "[RL] x = " << MPC_work->solution->x[0] << ", y = " << MPC_work->solution->x[1] << ", z = " << MPC_work->solution->x[2] << endl;
////    cout << "[RR] x = " << MPC_work->solution->x[3] << ", y = " << MPC_work->solution->x[4] << ", z = " << MPC_work->solution->x[5] << endl;
////    cout << "[FL] x = " << MPC_work->solution->x[6] << ", y = " << MPC_work->solution->x[7] << ", z = " << MPC_work->solution->x[8] << endl;
////    cout << "[FR] x = " << MPC_work->solution->x[9] << ", y = " << MPC_work->solution->x[10] << ", z = " << MPC_work->solution->x[11] << endl<< endl;
//}

void CRobot::Test_Function(void) {
    const double up_down_time = 1;
    const int up_down_cnt = 1000;

    if (test_cnt == 0) {
        // ============ Initialize ============ //
        //            cout << "[0]" << endl;
        //        test_phase = 1;
        //        moving_done_flag = false;
        _c << 1, 1, 1, 1;
        contact_num = 4;

        com_pos = tar_init_com_pos;
        com_vel = tar_init_com_vel;

        RL_foot_pos = tar_init_RL_foot_pos;
        RR_foot_pos = tar_init_RR_foot_pos;
        FL_foot_pos = tar_init_FL_foot_pos;
        FR_foot_pos = tar_init_FR_foot_pos;

        RL_foot_vel = tar_init_RL_foot_vel;
        RR_foot_vel = tar_init_RR_foot_vel;
        FL_foot_vel = tar_init_FL_foot_vel;
        FR_foot_vel = tar_init_FR_foot_vel;

        test_cnt++;
    } else if (test_cnt <= up_down_cnt) {
        //        cout << "[test_cnt] " << test_cnt << endl;
        com_pos = tar_init_com_pos;
        com_vel = tar_init_com_vel;

        com_pos[2] = tar_init_com_pos[2] + (-0.10) / 2.0 * (1 - cos(PI2 / (up_down_time)*(double) test_cnt * dt));
        com_vel[2] = (-0.10) / 2.0 * PI2 / (up_down_time)*(sin(PI2 / (up_down_time)*(double) test_cnt * dt));

        RL_foot_pos = tar_init_RL_foot_pos;
        RR_foot_pos = tar_init_RR_foot_pos;
        FL_foot_pos = tar_init_FL_foot_pos;
        FR_foot_pos = tar_init_FR_foot_pos;

        RL_foot_vel = tar_init_RL_foot_vel;
        RR_foot_vel = tar_init_RR_foot_vel;
        FL_foot_vel = tar_init_FL_foot_vel;
        FR_foot_vel = tar_init_FR_foot_vel;

        if (test_cnt == up_down_cnt) {
            if (move_stop_flag == true) {
                moving_done_flag = true;
            } else {
                test_cnt = 0;
            }
        }

        test_cnt++;
    } else {
        com_pos = tar_init_com_pos;
        com_vel = tar_init_com_vel;

        RL_foot_pos = tar_init_RL_foot_pos;
        RR_foot_pos = tar_init_RR_foot_pos;
        FL_foot_pos = tar_init_FL_foot_pos;
        FR_foot_pos = tar_init_FR_foot_pos;

        RL_foot_vel = tar_init_RL_foot_vel;
        RR_foot_vel = tar_init_RR_foot_vel;
        FL_foot_vel = tar_init_FL_foot_vel;
        FR_foot_vel = tar_init_FR_foot_vel;
    }

    if (Slope_con_onoff_flag == true) Slope_compensation_con();
    base_pos = com_pos + tmp_com_pos + base_offset;

}

//void CRobot::Test_Function(void)
//{
//    if (test_phase == 0) {
//        // ============ Initialize ============ //
//        //            cout << "[0]" << endl;
//        test_phase = 1;
//        //        moving_done_flag = false;
//        _c << 1, 1, 1, 1;
//        contact_num = 4;
//
//        com_pos = tar_init_com_pos;
//        com_vel = tar_init_com_vel;
//
//        RL_foot_pos = tar_init_RL_foot_pos;
//        RR_foot_pos = tar_init_RR_foot_pos;
//        FL_foot_pos = tar_init_FL_foot_pos;
//        FR_foot_pos = tar_init_FR_foot_pos;
//
//        RL_foot_vel = tar_init_RL_foot_vel;
//        RR_foot_vel = tar_init_RR_foot_vel;
//        FL_foot_vel = tar_init_FL_foot_vel;
//        FR_foot_vel = tar_init_FR_foot_vel;
//
//        //        for(int i=0;i<6;++i){
//        //            c_state_x1[i] = 0;
//        //        }
//        //
//        //        tar_Fc_y = 0;
//
//    }
//    else {
//        //        test_phase = 1;
//        //        moving_done_flag = false;
//        _c << 1, 1, 1, 1;
//        contact_num = 4;
//
//        RL_foot_pos = tar_init_RL_foot_pos;
//        RR_foot_pos = tar_init_RR_foot_pos;
//        FL_foot_pos = tar_init_FL_foot_pos;
//        FR_foot_pos = tar_init_FR_foot_pos;
//
//        RL_foot_vel = tar_init_RL_foot_vel;
//        RR_foot_vel = tar_init_RR_foot_vel;
//        FL_foot_vel = tar_init_FL_foot_vel;
//        FR_foot_vel = tar_init_FR_foot_vel;
//
//        if (Mode == MODE_SIMULATION) {
//            MPC_process();
//        }
//
//        if (move_stop_flag == true) {
//            moving_done_flag = true;
//            _c << 1, 1, 1, 1;
//            contact_num = 4;
//        }
//
//        test_cnt++;
//    }
//
//    if(Slope_con_onoff_flag == true) Slope_compensation_con();
////    base_pos = com_pos + base_offset;
//    base_pos = com_pos + tmp_com_pos + base_offset;
//
//    //    tmp_data1[33] = input_u*0.01;
//    //    tmp_data1[34] = act_output_y(0);
//    //    tmp_data1[35] = (double)(test_cnt % Ts_cnt)/Ts_cnt*0.01;
//    //    tmp_data1[36] = tar_output_y(0);
//    //    tmp_data1[37] = tar_Fc_y;
//
//    //    // waist
//    //    target_pos[6] = 0;
//}

void CRobot::FK2(void) {
    //    RL_foot_pos = CalBodyToBaseCoordinates(*m_pModel, RobotState, RobotStatedot, ddqZero, RL.ID, EP_OFFSET_RL, true););

    act_RL_foot_pos = CalcBodyToBaseCoordinates(*m_pModel, RobotState, RL.ID, EP_OFFSET_RL, true);
    act_RR_foot_pos = CalcBodyToBaseCoordinates(*m_pModel, RobotState, RR.ID, EP_OFFSET_RR, true);
    act_FL_foot_pos = CalcBodyToBaseCoordinates(*m_pModel, RobotState, FL.ID, EP_OFFSET_FL, true);
    act_FR_foot_pos = CalcBodyToBaseCoordinates(*m_pModel, RobotState, FR.ID, EP_OFFSET_FR, true);

    act_RL_foot_pos_local = act_RL_foot_pos - CalcBodyToBaseCoordinates(*m_pModel, RobotState, base.ID, Originbase, true);
    act_RR_foot_pos_local = act_RR_foot_pos - CalcBodyToBaseCoordinates(*m_pModel, RobotState, base.ID, Originbase, true);
    act_FL_foot_pos_local = act_FL_foot_pos - CalcBodyToBaseCoordinates(*m_pModel, RobotState, base.ID, Originbase, true);
    act_FR_foot_pos_local = act_FR_foot_pos - CalcBodyToBaseCoordinates(*m_pModel, RobotState, base.ID, Originbase, true);


}

//VectorNd CRobot::IK1(VectorNd EP)
//{
//    const double L1 = 0.105;
//    const double L2 = 0.305;
//    const double L3 = 0.309; //0.305;
//
//    static double x = 0;
//    static double y = 0;
//    static double z = 0;
//
//    //    ROT_Y << cos(base_ori(1)*1), 0, sin(base_ori(1)*1),
//    //            0, 1, 0,
//    //            -sin(base_ori(1)*1), 0, cos(base_ori(1)*1);
//
//    //    ROT_Y << 1, 0, 0,
//    //    		 0, 1, 0,
//    //    		 0, 0, 1;
//
//    tmp_foot_pos << -(EP[0] - RL_base2hip_pos(0)),
//            EP[1] - RL_base2hip_pos(1),
//            EP[2] - RL_base2hip_pos(2);
//
//    //    tmp_foot_pos2 = ROT_Y*tmp_foot_pos;
//
//    x = tmp_foot_pos(0); // + 0.4*tan(base_ori(1));//tmp_foot_pos2(0);
//    y = tmp_foot_pos(1);
//    z = tmp_foot_pos(2); ///cos(base_ori(1));
//
//    //    cout << "tmp_foot_pos = " << tmp_foot_pos.transpose() << endl;
//    //    cout << "tmp_foot_pos2 = " << tmp_foot_pos2.transpose() << endl;
//    //    x = -(EP[0] - RL_base2hip_pos(0));
//    //    y = EP[1] - RL_base2hip_pos(1);
//    //    z = EP[2] - RL_base2hip_pos(2);
//
//    if (z < (-0.6)) z = -0.6;
//
//    //    cout << "[RL1] x = " << x << ", y = " << y << ", z = " << z << endl;
//    //    cout << "[RL2] x = " << x << ", y = " << y << ", z = " << z << endl;
//
//    target_pos[0] = atan(y / abs(z)) - PI / 2 + acos(L1 / (sqrt(pow(y, 2) + pow(z, 2))));
//    target_pos[1] = -(-atan(x / sqrt(abs(-pow(L1, 2) + pow(y, 2) + pow(z, 2)))) - acos((-pow(L1, 2) + pow(L2, 2) - pow(L3, 2) + pow(x, 2) + pow(y, 2) + pow(z, 2)) / (2 * L2 * sqrt(-pow(L1, 2) + pow(x, 2) + pow(y, 2) + pow(z, 2)))));
//    target_pos[2] = -(PI - acos((pow(L1, 2) + pow(L2, 2) + pow(L3, 2) - pow(x, 2) - pow(y, 2) - pow(z, 2)) / (2 * L2 * L3)));
//
////	cout << "[RL] q[0] = " << target_pos[0]*R2D << ", q[1] = " << target_pos[1]*R2D << ", q[2] = " << target_pos[2]*R2D << endl;
//
//    tmp_foot_pos << -(EP[3] - RR_base2hip_pos(0)),
//            EP[4] - RR_base2hip_pos(1),
//            EP[5] - RR_base2hip_pos(2);
//
//    //    tmp_foot_pos2 = ROT_Y*tmp_foot_pos;
//
//    x = tmp_foot_pos(0); // + 0.4*tan(base_ori(1));//tmp_foot_pos2(0);
//    y = tmp_foot_pos(1);
//    z = tmp_foot_pos(2); ///cos(base_ori(1));
//
//    //    x = tmp_foot_pos2(0);
//    //    y = tmp_foot_pos2(1);
//    //    z = tmp_foot_pos2(2);
//
//    //    x = -(EP[3] - RR_base2hip_pos(0));
//    //    y = EP[4] - RR_base2hip_pos(1);
//    //    z = EP[5] - RR_base2hip_pos(2);
//
//    if (z < (-0.6)) z = -0.6;
//
//    target_pos[3] = PI / 2 + atan(y / abs(z)) - acos(L1 / sqrt(pow(y, 2) + pow(z, 2))); //PI/2 - acos(L1/sqrt(pow(y,2) + pow(z,2))) - atan(abs(y)/abs(z)); //-((atan(y/abs(z)) - PI/2 + acos(L1/(sqrt(pow(y,2) + pow(z,2))))));
//    target_pos[4] = -(-atan(x / sqrt(abs(-pow(L1, 2) + pow(y, 2) + pow(z, 2)))) - acos((-pow(L1, 2) + pow(L2, 2) - pow(L3, 2) + pow(x, 2) + pow(y, 2) + pow(z, 2)) / (2 * L2 * sqrt(-pow(L1, 2) + pow(x, 2) + pow(y, 2) + pow(z, 2)))));
//    target_pos[5] = -(PI - acos((pow(L1, 2) + pow(L2, 2) + pow(L3, 2) - pow(x, 2) - pow(y, 2) - pow(z, 2)) / (2 * L2 * L3)));
//
//    tmp_foot_pos << -(EP[6] - FL_base2hip_pos(0)),
//            EP[7] - FL_base2hip_pos(1),
//            EP[8] - FL_base2hip_pos(2);
//
//    //    tmp_foot_pos2 = ROT_Y*tmp_foot_pos;
//
//    x = tmp_foot_pos(0); // + 0.4*tan(base_ori(1));//tmp_foot_pos2(0);
//    y = tmp_foot_pos(1);
//    z = tmp_foot_pos(2); ///cos(base_ori(1));
//    //    x = tmp_foot_pos2(0);
//    //    y = tmp_foot_pos2(1);
//    //    z = tmp_foot_pos2(2);
//
//    //    x = -(EP[6] - FL_base2hip_pos(0));
//    //    y = EP[7] - FL_base2hip_pos(1);
//    //    z = EP[8] - FL_base2hip_pos(2);
//
//    if (z < (-0.6)) z = -0.6;
//
//    target_pos[7] = atan(y / abs(z)) - PI / 2 + acos(L1 / (sqrt(pow(y, 2) + pow(z, 2))));
//    target_pos[8] = -(-atan(x / sqrt(abs(-pow(L1, 2) + pow(y, 2) + pow(z, 2)))) - acos((-pow(L1, 2) + pow(L2, 2) - pow(L3, 2) + pow(x, 2) + pow(y, 2) + pow(z, 2)) / (2 * L2 * sqrt(-pow(L1, 2) + pow(x, 2) + pow(y, 2) + pow(z, 2)))));
//    target_pos[9] = -(PI - acos((pow(L1, 2) + pow(L2, 2) + pow(L3, 2) - pow(x, 2) - pow(y, 2) - pow(z, 2)) / (2 * L2 * L3)));
//
//    tmp_foot_pos << -(EP[9] - FR_base2hip_pos(0)),
//            EP[10] - FR_base2hip_pos(1),
//            EP[11] - FR_base2hip_pos(2);
//
//    //    tmp_foot_pos2 = ROT_Y*tmp_foot_pos;
//
//    x = tmp_foot_pos(0); // + 0.4*tan(base_ori(1));//tmp_foot_pos2(0);
//    y = tmp_foot_pos(1);
//    z = tmp_foot_pos(2); ///cos(base_ori(1));
//    //    x = tmp_foot_pos2(0);
//    //    y = tmp_foot_pos2(1);
//    //    z = tmp_foot_pos2(2);
//
//    //    x = -(EP[9] - FR_base2hip_pos(0));
//    //    y = EP[10] - FR_base2hip_pos(1);
//    //    z = EP[11] - FR_base2hip_pos(2);
//
//    if (z < (-0.6)) z = -0.6;
//
//    target_pos[10] = PI / 2 + atan(y / abs(z)) - acos(L1 / sqrt(pow(y, 2) + pow(z, 2))); //PI/2 - acos(L1/sqrt(pow(y,2) + pow(z,2))) - atan(abs(y)/abs(z)); //-((atan(y/abs(z)) - PI/2 + acos(L1/(sqrt(pow(y,2) + pow(z,2))))));
//    target_pos[11] = -(-atan(x / sqrt(abs(-pow(L1, 2) + pow(y, 2) + pow(z, 2)))) - acos((-pow(L1, 2) + pow(L2, 2) - pow(L3, 2) + pow(x, 2) + pow(y, 2) + pow(z, 2)) / (2 * L2 * sqrt(-pow(L1, 2) + pow(x, 2) + pow(y, 2) + pow(z, 2)))));
//    target_pos[12] = -(PI - acos((pow(L1, 2) + pow(L2, 2) + pow(L3, 2) - pow(x, 2) - pow(y, 2) - pow(z, 2)) / (2 * L2 * L3)));
//
//    //    target_pos[6] = 0;
//    //    cout << "tar_pos = " << target_pos.transpose()*R2D << endl;
//
//
//
//    return target_pos;
//}

VectorNd CRobot::IK1(VectorNd EP) {
    const double L1 = 0.105;
    const double L2 = 0.305;
    const double L3 = 0.309; //0.305;

    static double x = 0;
    static double y = 0;
    static double z = 0;

    ROT_Y << cos(base_ori(1)*1), 0, sin(base_ori(1)*1),
            0, 1, 0,
            -sin(base_ori(1)*1), 0, cos(base_ori(1)*1);

    //    ROT_Y << 1, 0, 0,
    //    		 0, 1, 0,
    //    		 0, 0, 1;

    tmp_foot_pos << -(EP[0] - RL_base2hip_pos(0)),
            EP[1] - RL_base2hip_pos(1),
            EP[2] - RL_base2hip_pos(2);

    tmp_foot_pos2 = ROT_Y*tmp_foot_pos;

    //    x = tmp_foot_pos(0); // + 0.4*tan(base_ori(1));//tmp_foot_pos2(0);
    //    y = tmp_foot_pos(1);
    //    z = tmp_foot_pos(2); ///cos(base_ori(1));

    x = tmp_foot_pos2(0);
    y = tmp_foot_pos2(1);
    z = tmp_foot_pos2(2);

    //    cout << "tmp_foot_pos = " << tmp_foot_pos.transpose() << endl;
    //    cout << "tmp_foot_pos2 = " << tmp_foot_pos2.transpose() << endl;
    //    x = -(EP[0] - RL_base2hip_pos(0));
    //    y = EP[1] - RL_base2hip_pos(1);
    //    z = EP[2] - RL_base2hip_pos(2);

    if (z < (-0.6)) z = -0.6;

    //    cout << "[RL1] x = " << x << ", y = " << y << ", z = " << z << endl;
    //    cout << "[RL2] x = " << x << ", y = " << y << ", z = " << z << endl;

    target_pos[0] = atan(y / abs(z)) - PI / 2 + acos(L1 / (sqrt(pow(y, 2) + pow(z, 2))));
    target_pos[1] = -(-atan(x / sqrt(abs(-pow(L1, 2) + pow(y, 2) + pow(z, 2)))) - acos((-pow(L1, 2) + pow(L2, 2) - pow(L3, 2) + pow(x, 2) + pow(y, 2) + pow(z, 2)) / (2 * L2 * sqrt(-pow(L1, 2) + pow(x, 2) + pow(y, 2) + pow(z, 2)))));
    target_pos[2] = -(PI - acos((pow(L1, 2) + pow(L2, 2) + pow(L3, 2) - pow(x, 2) - pow(y, 2) - pow(z, 2)) / (2 * L2 * L3)));

    //	cout << "[RL] q[0] = " << target_pos[0]*R2D << ", q[1] = " << target_pos[1]*R2D << ", q[2] = " << target_pos[2]*R2D << endl;

    tmp_foot_pos << -(EP[3] - RR_base2hip_pos(0)),
            EP[4] - RR_base2hip_pos(1),
            EP[5] - RR_base2hip_pos(2);

    tmp_foot_pos2 = ROT_Y*tmp_foot_pos;

    x = tmp_foot_pos2(0); // + 0.4*tan(base_ori(1));//tmp_foot_pos2(0);
    y = tmp_foot_pos2(1);
    z = tmp_foot_pos2(2); ///cos(base_ori(1));

    //    x = tmp_foot_pos2(0);
    //    y = tmp_foot_pos2(1);
    //    z = tmp_foot_pos2(2);

    //    x = -(EP[3] - RR_base2hip_pos(0));
    //    y = EP[4] - RR_base2hip_pos(1);
    //    z = EP[5] - RR_base2hip_pos(2);

    if (z < (-0.6)) z = -0.6;

    target_pos[3] = PI / 2 + atan(y / abs(z)) - acos(L1 / sqrt(pow(y, 2) + pow(z, 2))); //PI/2 - acos(L1/sqrt(pow(y,2) + pow(z,2))) - atan(abs(y)/abs(z)); //-((atan(y/abs(z)) - PI/2 + acos(L1/(sqrt(pow(y,2) + pow(z,2))))));
    target_pos[4] = -(-atan(x / sqrt(abs(-pow(L1, 2) + pow(y, 2) + pow(z, 2)))) - acos((-pow(L1, 2) + pow(L2, 2) - pow(L3, 2) + pow(x, 2) + pow(y, 2) + pow(z, 2)) / (2 * L2 * sqrt(-pow(L1, 2) + pow(x, 2) + pow(y, 2) + pow(z, 2)))));
    target_pos[5] = -(PI - acos((pow(L1, 2) + pow(L2, 2) + pow(L3, 2) - pow(x, 2) - pow(y, 2) - pow(z, 2)) / (2 * L2 * L3)));

    tmp_foot_pos << -(EP[6] - FL_base2hip_pos(0)),
            EP[7] - FL_base2hip_pos(1),
            EP[8] - FL_base2hip_pos(2);

    tmp_foot_pos2 = ROT_Y*tmp_foot_pos;

    x = tmp_foot_pos2(0); // + 0.4*tan(base_ori(1));//tmp_foot_pos2(0);
    y = tmp_foot_pos2(1);
    z = tmp_foot_pos2(2); ///cos(base_ori(1));
    //    x = tmp_foot_pos2(0);
    //    y = tmp_foot_pos2(1);
    //    z = tmp_foot_pos2(2);

    //    x = -(EP[6] - FL_base2hip_pos(0));
    //    y = EP[7] - FL_base2hip_pos(1);
    //    z = EP[8] - FL_base2hip_pos(2);

    if (z < (-0.6)) z = -0.6;

    target_pos[7] = atan(y / abs(z)) - PI / 2 + acos(L1 / (sqrt(pow(y, 2) + pow(z, 2))));
    target_pos[8] = -(-atan(x / sqrt(abs(-pow(L1, 2) + pow(y, 2) + pow(z, 2)))) - acos((-pow(L1, 2) + pow(L2, 2) - pow(L3, 2) + pow(x, 2) + pow(y, 2) + pow(z, 2)) / (2 * L2 * sqrt(-pow(L1, 2) + pow(x, 2) + pow(y, 2) + pow(z, 2)))));
    target_pos[9] = -(PI - acos((pow(L1, 2) + pow(L2, 2) + pow(L3, 2) - pow(x, 2) - pow(y, 2) - pow(z, 2)) / (2 * L2 * L3)));

    tmp_foot_pos << -(EP[9] - FR_base2hip_pos(0)),
            EP[10] - FR_base2hip_pos(1),
            EP[11] - FR_base2hip_pos(2);

    tmp_foot_pos2 = ROT_Y*tmp_foot_pos;

    x = tmp_foot_pos2(0); // + 0.4*tan(base_ori(1));//tmp_foot_pos2(0);
    y = tmp_foot_pos2(1);
    z = tmp_foot_pos2(2); ///cos(base_ori(1));
    //    x = tmp_foot_pos2(0);
    //    y = tmp_foot_pos2(1);
    //    z = tmp_foot_pos2(2);


    if (z < (-0.6)) z = -0.6;

    target_pos[10] = PI / 2 + atan(y / abs(z)) - acos(L1 / sqrt(pow(y, 2) + pow(z, 2))); //PI/2 - acos(L1/sqrt(pow(y,2) + pow(z,2))) - atan(abs(y)/abs(z)); //-((atan(y/abs(z)) - PI/2 + acos(L1/(sqrt(pow(y,2) + pow(z,2))))));
    target_pos[11] = -(-atan(x / sqrt(abs(-pow(L1, 2) + pow(y, 2) + pow(z, 2)))) - acos((-pow(L1, 2) + pow(L2, 2) - pow(L3, 2) + pow(x, 2) + pow(y, 2) + pow(z, 2)) / (2 * L2 * sqrt(-pow(L1, 2) + pow(x, 2) + pow(y, 2) + pow(z, 2)))));
    target_pos[12] = -(PI - acos((pow(L1, 2) + pow(L2, 2) + pow(L3, 2) - pow(x, 2) - pow(y, 2) - pow(z, 2)) / (2 * L2 * L3)));

    //    target_pos[6] = 0;
    //    cout << "tar_pos = " << target_pos.transpose()*R2D << endl;



    return target_pos;
}

void CRobot::Init_Pos_Traj(void) {
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

void CRobot::Get_CP(void) {
    //    const double w_n = sqrt(com_height / GRAVITY)*1.0;
    //    const double cp_l = com_height;
    //    static double tmp_cp_y = 0;
    //    const double cp_alpha = 0.03;
    //
    //    Rot_Mat_XYZ << 1, 0, sin(act_base_ori(1)),
    //            0, cos(act_base_ori(0)), -sin(act_base_ori(0)) * cos(act_base_ori(1)),
    //            0, sin(act_base_ori(0)), cos(act_base_ori(0)) * cos(act_base_ori(1));
    //
    //    act_base_ori_dot_w = Rot_Mat_XYZ*act_base_ori_dot;
    //    act_com_vel2(0) = cp_l * act_base_ori_dot_w(1);
    //    act_com_vel2(1) = -cp_l * act_base_ori_dot_w(0);
    //    act_com_pos2(1) = -cp_l * act_base_ori(0);
    //
    //    //    lpf_act_com_vel(0) = (1 - alpha_act_com_x) * lpf_act_com_vel(0) + alpha_act_com_x*act_com_vel(0);
    //    //    lpf_act_com_vel(1) = (1 - alpha_act_com_y) * lpf_act_com_vel(1) + alpha_act_com_y*act_com_vel(1);
    //
    //
    //    //    tmp_cp_y =  w_n*(act_com_vel2(1) - 0)*weight_cp_y;
    //
    //    //    tmp_cp_y = ((act_com_pos(1) - com_pos(1)) + w_n * (act_com_vel(1) - 0)) * w_cp_y1;
    //    //    tmp_cp_y = ((act_com_pos2(1) - com_pos(1)) + w_n * (act_com_vel2(1) - com_vel(1))) * w_cp_y1;
    ////    tmp_cp_y = (act_com_pos2(1) + w_n * act_com_vel2(1)) * w_cp_y1;
    ////
    ////    cp_y = (1 - cp_alpha) * cp_y + cp_alpha*tmp_cp_y;
    //    //    tmp_data1[27] = cp_y;
    //
    //    //    if(CP_con_onoff_flag == false) cp_y = 0;
    //
    //
    //    //        if(cp_y <= cp_y_min && cp_y >= -cp_y_min){
    //    //            tar_cp_y = 0;
    //    //        }
    //    //        else if(cp_y > cp_y_max){
    //    //            tar_cp_y = cp_y_max;
    //    //        }
    //    //        else if(cp_y < -cp_y_max){
    //    //            tar_cp_y = -cp_y_max;
    //    //        }
    //    //        else{
    //    //            tar_cp_y = cp_y;
    //    //        }
    //    //
    //    //        tar_cp_x = tar_cp_x/2;
    //    //        tar_cp_y = tar_cp_y/2;
    //    //
    //    //        cout << "cp_x = " << cp_x << ", tar_cp_x = " << tar_cp_x << endl;
    //    //        cout << "cp_y = " << cp_y << ", tar_cp_y = " << tar_cp_y << endl;
    //    //        cout << "================================================" << endl;
    //
    //    //    cp_x =  w_n*(act_com_vel(0) - com_vel(0))*weight_cp_x;
    //    //    cp_y =  w_n*(act_com_vel(1) - com_vel(1))*weight_cp_y;
    //
    //    //    cout << "err = " << act_base_ori_dot_w(0) - act_base_ori_dot(0) << endl;
    //
    //    //    tmp_data1[28] = act_com_vel2(1);//act_base_ori_dot(0);
    //    //    tmp_data1[29] = com_vel(1);//act_base_ori_dot_w(0);
}














// ====================== Stair walking trajectory generation ===================== //

void CRobot::Stair_Walking(void) {
    tw_time = (double) tw_cnt*dt;
    base_ori(2) = tmp_base_ori(2);

    //    cout << "tar yaw = " << base_ori(2)<< endl;

    if (tw_cnt == 0) {
        // ============ Initialize ============ //
        //        cout << "[0]" << endl;
        walking_phase = TW_PHASE_INIT;
        //        moving_done_flag = false;
        _c << 1, 1, 1, 1;
        contact_num = 4;

        com_pos = tar_init_com_pos;
        com_vel = tar_init_com_vel;
        com_acc = tar_init_com_acc;

        RL_foot_pos = tar_init_RL_foot_pos;
        RR_foot_pos = tar_init_RR_foot_pos;
        FL_foot_pos = tar_init_FL_foot_pos;
        FR_foot_pos = tar_init_FR_foot_pos;

        RL_foot_vel = tar_init_RL_foot_vel;
        RR_foot_vel = tar_init_RR_foot_vel;
        FL_foot_vel = tar_init_FL_foot_vel;
        FR_foot_vel = tar_init_FR_foot_vel;

        x_moving_speed = 0;
        y_moving_speed = 0;

        //        TW_SF_Z_Traj_Gen();

        //        pre_RR_foot_pos(2) = act_RR_foot_pos(2);
        //        pre_FL_foot_pos(2) = act_FL_foot_pos(2);
    } else if (tw_cnt < dsp_cnt) {
        // ============ First Step (STANCE_RLFR) ============= //
        //        cout << "[1]" << endl;
        walking_phase = TW_DSP_RLFR_FIRST;
        tmp_t = tw_time;

        _c << 1, 0, 0, 1;
        contact_num = 2;

        com_pos = tar_init_com_pos;
        com_vel = tar_init_com_vel;

        RL_foot_pos = tar_init_RL_foot_pos;
        RR_foot_pos = tar_init_RR_foot_pos;
        FL_foot_pos = tar_init_FL_foot_pos;
        FR_foot_pos = tar_init_FR_foot_pos;

        RL_foot_vel = tar_init_RL_foot_vel;
        RR_foot_vel = tar_init_RR_foot_vel;
        FL_foot_vel = tar_init_FL_foot_vel;
        FR_foot_vel = tar_init_FR_foot_vel;

        RR_foot_pos[2] = tar_init_RR_foot_pos[2] + foot_height / 2.0 * (1 - cos(PI2 / (dsp_time) * tmp_t));
        FL_foot_pos[2] = tar_init_FL_foot_pos[2] + foot_height / 2.0 * (1 - cos(PI2 / (dsp_time) * tmp_t));
        RR_foot_vel[2] = foot_height / 2.0 * PI2 / (dsp_time)*(sin(PI2 / (dsp_time) * tmp_t));

        FL_foot_vel = RR_foot_vel;

        //        if (tw_cnt == dsp_cnt - 1) {
        //            pre_com_pos = com_pos;
        //            pre_RL_foot_pos = RL_foot_pos;
        //            pre_RR_foot_pos = RR_foot_pos;
        //            pre_FL_foot_pos = FL_foot_pos;
        //            pre_FR_foot_pos = FR_foot_pos;
        //        }


    } else if (tw_cnt < step_cnt) {
        // ============ First Step (FSP) ============= //
        //        cout << "[2]" << endl;
        walking_phase = TW_FSP;
        tmp_t = tw_time - dsp_time;

        _c << 1, 1, 1, 1;
        contact_num = 4;

        com_pos = tar_init_com_pos;
        com_vel = tar_init_com_vel;

        RL_foot_pos = tar_init_RL_foot_pos;
        RR_foot_pos = tar_init_RR_foot_pos;
        FL_foot_pos = tar_init_FL_foot_pos;
        FR_foot_pos = tar_init_FR_foot_pos;

        RL_foot_vel = tar_init_RL_foot_vel;
        RR_foot_vel = tar_init_RR_foot_vel;
        FL_foot_vel = tar_init_FL_foot_vel;
        FR_foot_vel = tar_init_FR_foot_vel;

        if (tw_cnt == step_cnt - 1) {

            pre_x_moving_speed = x_moving_speed;
            pre_y_moving_speed = y_moving_speed;
            x_moving_speed = tmp_x_moving_speed;
            y_moving_speed = tmp_y_moving_speed;

            pre_com_pos = com_pos;
            pre_RL_foot_pos = RL_foot_pos;
            pre_RR_foot_pos = RR_foot_pos;
            pre_FL_foot_pos = FL_foot_pos;
            pre_FR_foot_pos = FR_foot_pos;

            TW_COM_Traj_Gen();
            //            TW_SF_Traj_Gen();

            foot_x_length1 = pre_x_moving_speed * step_time + x_moving_speed * step_time;
            foot_x_length2 = x_moving_speed * step_time * 2;

            foot_y_length1 = pre_y_moving_speed * step_time + y_moving_speed * step_time;
            foot_y_length2 = y_moving_speed * step_time * 2;


        }
    } else if (tw_cnt < step_cnt + dsp_cnt) {
        // ============ Second Step (STANCE_RRFL) ============ //
        // ============ Continuous Walking Start ============= //
        //        cout << "[3]" << endl;
        walking_phase = TW_DSP_RRFL;
        tmp_t = tw_time - step_time;
        tmp_cnt = tw_cnt - step_cnt;

        _c << 0, 1, 1, 0;
        contact_num = 2;

        com_pos = tar_init_com_pos;
        com_vel = tar_init_com_vel;

        RL_foot_pos = pre_RL_foot_pos;
        RR_foot_pos = pre_RR_foot_pos;
        FL_foot_pos = pre_FL_foot_pos;
        FR_foot_pos = pre_FR_foot_pos;

        RL_foot_vel = tar_init_RL_foot_vel;
        RR_foot_vel = tar_init_RR_foot_vel;
        FL_foot_vel = tar_init_FL_foot_vel;
        FR_foot_vel = tar_init_FR_foot_vel;

        com_pos[0] = fifth_order_poly(c_com_x1, tmp_t);
        com_pos[1] = fifth_order_poly(c_com_y1, tmp_t);
        com_vel[0] = fifth_order_poly_dot(c_com_x1, tmp_t);
        com_vel[1] = fifth_order_poly_dot(c_com_y1, tmp_t);

        RL_foot_pos[0] = pre_RL_foot_pos[0] + foot_x_length1 / 2.0 * (1 - cos(PI2 / (dsp_time * 2) * tmp_t));
        RL_foot_pos[1] = pre_RL_foot_pos[1] + foot_y_length1 / 2.0 * (1 - cos(PI2 / (dsp_time * 2) * tmp_t));
        FR_foot_pos[0] = pre_FR_foot_pos[0] + foot_x_length1 / 2.0 * (1 - cos(PI2 / (dsp_time * 2) * tmp_t));
        FR_foot_pos[1] = pre_FR_foot_pos[1] + foot_y_length1 / 2.0 * (1 - cos(PI2 / (dsp_time * 2) * tmp_t));

        RL_foot_vel[0] = foot_x_length1 / 2.0 * PI2 / (dsp_time * 2)*(sin(PI2 / (dsp_time * 2) * tmp_t));
        RL_foot_vel[1] = foot_y_length1 / 2.0 * PI2 / (dsp_time * 2)*(sin(PI2 / (dsp_time * 2) * tmp_t));

        RL_foot_pos[2] = tar_init_RL_foot_pos[2] + foot_height / 2.0 * (1 - cos(PI2 / (dsp_time) * tmp_t));
        FR_foot_pos[2] = tar_init_FR_foot_pos[2] + foot_height / 2.0 * (1 - cos(PI2 / (dsp_time) * tmp_t));
        RL_foot_vel[2] = foot_height / 2.0 * PI2 / (dsp_time)*(sin(PI2 / (dsp_time) * tmp_t));

        FR_foot_vel[0] = RL_foot_vel[0];
        FR_foot_vel[1] = RL_foot_vel[1];
        FR_foot_vel[2] = foot_height / 2.0 * PI2 / dsp_time * (sin(PI2 / dsp_time * tmp_t));

        if (tmp_cnt == dsp_cnt - 1) {
            pre_com_pos(0) = pre_com_pos[0] + pre_x_moving_speed * (dsp_time) / 2.0 + x_moving_speed * (dsp_time) / 2.0;
            pre_com_pos(1) = pre_com_pos[1] + pre_y_moving_speed * (dsp_time) / 2.0 + y_moving_speed * (dsp_time) / 2.0;
            pre_com_pos(2) = com_pos(2);

            pre_RL_foot_pos = RL_foot_pos;
            pre_RR_foot_pos = RR_foot_pos;
            pre_FL_foot_pos = FL_foot_pos;
            pre_FR_foot_pos = FR_foot_pos;

        }
    } else if (tw_cnt < step_cnt * 2) {
        // ============ Second Step (FSP) ============= //
        //        cout << "[4]" << endl;
        walking_phase = TW_FSP;
        //        cout << "[walk4]" << endl;
        _c << 1, 1, 1, 1;
        contact_num = 4;

        tmp_t = tw_time - step_time - dsp_time;

        com_pos[0] = pre_com_pos(0) + x_moving_speed*tmp_t;
        com_pos[1] = pre_com_pos(1) + y_moving_speed*tmp_t;
        com_pos[2] = pre_com_pos(2);

        //        com_vel = tar_init_com_vel;

        com_vel[0] = x_moving_speed;
        com_vel[1] = y_moving_speed;
        com_vel[2] = tar_init_com_vel[2];

        RL_foot_pos = pre_RL_foot_pos;
        RR_foot_pos = pre_RR_foot_pos;
        FL_foot_pos = pre_FL_foot_pos;
        FR_foot_pos = pre_FR_foot_pos;

        RL_foot_vel = tar_init_RL_foot_vel;
        RR_foot_vel = tar_init_RR_foot_vel;
        FL_foot_vel = tar_init_FL_foot_vel;
        FR_foot_vel = tar_init_FR_foot_vel;

        if (tw_cnt == step_cnt * 2 - 1) {

            pre_com_pos(0) = pre_com_pos(0) + x_moving_speed*fsp_time;
            pre_com_pos(1) = pre_com_pos(1) + y_moving_speed*fsp_time;
            pre_com_pos(2) = com_pos(2);

            pre_RL_foot_pos = RL_foot_pos;
            pre_RR_foot_pos = RR_foot_pos;
            pre_FL_foot_pos = FL_foot_pos;
            pre_FR_foot_pos = FR_foot_pos;

        }
    } else if (tw_cnt < step_cnt * 2 + dsp_cnt) {
        // ============ Third Step (STANCE_RLFR) ============= //
        //        cout << "[5]" << endl;
        walking_phase = TW_DSP_RLFR;
        tmp_t = tw_time - step_time * 2;
        tmp_cnt = tw_cnt - step_cnt * 2;

        _c << 1, 0, 0, 1;
        contact_num = 2;

        com_pos = pre_com_pos;

        com_vel[0] = x_moving_speed;
        com_vel[1] = y_moving_speed;
        com_vel[2] = tar_init_com_vel[2];

        RL_foot_pos = pre_RL_foot_pos;
        RR_foot_pos = pre_RR_foot_pos;
        FL_foot_pos = pre_FL_foot_pos;
        FR_foot_pos = pre_FR_foot_pos;

        RL_foot_vel = tar_init_RL_foot_vel;
        RR_foot_vel = tar_init_RR_foot_vel;
        FL_foot_vel = tar_init_FL_foot_vel;
        FR_foot_vel = tar_init_FR_foot_vel;


        com_pos[0] = pre_com_pos(0) + x_moving_speed*tmp_t;
        com_pos[1] = pre_com_pos(1) + y_moving_speed*tmp_t;
        com_vel[0] = x_moving_speed;
        com_vel[1] = y_moving_speed;

        RR_foot_pos[0] = pre_RR_foot_pos[0] + foot_x_length2 / 2.0 * (1 - cos(PI2 / (dsp_time * 2) * tmp_t));
        RR_foot_pos[1] = pre_RR_foot_pos[1] + foot_y_length2 / 2.0 * (1 - cos(PI2 / (dsp_time * 2) * tmp_t));
        FL_foot_pos[0] = pre_FL_foot_pos[0] + foot_x_length2 / 2.0 * (1 - cos(PI2 / (dsp_time * 2) * tmp_t));
        FL_foot_pos[1] = pre_FL_foot_pos[1] + foot_y_length2 / 2.0 * (1 - cos(PI2 / (dsp_time * 2) * tmp_t));

        RR_foot_vel[0] = foot_x_length2 / 2.0 * PI2 / (dsp_time * 2)*(sin(PI2 / (dsp_time * 2) * tmp_t));
        RR_foot_vel[1] = foot_y_length2 / 2.0 * PI2 / (dsp_time * 2)*(sin(PI2 / (dsp_time * 2) * tmp_t));

        RR_foot_pos[2] = tar_init_RR_foot_pos[2] + foot_height / 2.0 * (1 - cos(PI2 / (dsp_time) * tmp_t));
        FL_foot_pos[2] = tar_init_FL_foot_pos[2] + foot_height / 2.0 * (1 - cos(PI2 / (dsp_time) * tmp_t));
        RR_foot_vel[2] = foot_height / 2.0 * PI2 / (dsp_time)*(sin(PI2 / (dsp_time) * tmp_t));

        FL_foot_vel[0] = RR_foot_vel[0];
        FL_foot_vel[1] = RR_foot_vel[1];
        FL_foot_vel[2] = foot_height / 2.0 * PI2 / dsp_time * (sin(PI2 / dsp_time * tmp_t));

        if (tmp_cnt == dsp_cnt - 1) {
            pre_com_pos(0) = pre_com_pos(0) + x_moving_speed*dsp_time;
            pre_com_pos(1) = pre_com_pos(1) + y_moving_speed*dsp_time;
            pre_com_pos(2) = com_pos(2);
            pre_RL_foot_pos = RL_foot_pos;
            pre_RR_foot_pos = RR_foot_pos;
            pre_FL_foot_pos = FL_foot_pos;
            pre_FR_foot_pos = FR_foot_pos;
        }
    } else if (tw_cnt < step_cnt * 3) {
        // ============ Third Step (FSP) ============= //
        //        cout << "[6]" << endl;
        walking_phase = TW_FSP;
        _c << 1, 1, 1, 1;
        contact_num = 4;

        tmp_t = tw_time - step_time * 2 - dsp_time;

        com_pos[0] = pre_com_pos(0) + x_moving_speed*tmp_t;
        com_pos[1] = pre_com_pos(1) + y_moving_speed*tmp_t;
        com_pos[2] = pre_com_pos(2);

        com_vel[0] = x_moving_speed;
        com_vel[1] = y_moving_speed;
        com_vel[2] = tar_init_com_vel[2];

        RL_foot_pos = pre_RL_foot_pos;
        RR_foot_pos = pre_RR_foot_pos;
        FL_foot_pos = pre_FL_foot_pos;
        FR_foot_pos = pre_FR_foot_pos;

        RL_foot_vel = tar_init_RL_foot_vel;
        RR_foot_vel = tar_init_RR_foot_vel;
        FL_foot_vel = tar_init_FL_foot_vel;
        FR_foot_vel = tar_init_FR_foot_vel;

        if (tw_cnt == step_cnt * 3 - 1) {

            pre_com_pos(0) = pre_com_pos(0) + x_moving_speed*fsp_time;
            pre_com_pos(1) = pre_com_pos(1) + y_moving_speed*fsp_time;
            pre_com_pos(2) = com_pos(2);

            pre_RL_foot_pos = RL_foot_pos;
            pre_RR_foot_pos = RR_foot_pos;
            pre_FL_foot_pos = FL_foot_pos;
            pre_FR_foot_pos = FR_foot_pos;

            pre_x_moving_speed = x_moving_speed;
            pre_y_moving_speed = y_moving_speed;
            x_moving_speed = tmp_x_moving_speed;
            y_moving_speed = tmp_y_moving_speed;

            TW_COM_Traj_Gen();
            //            TW_SF_Traj_Gen();

            foot_x_length1 = pre_x_moving_speed * step_time + x_moving_speed * step_time;
            foot_x_length2 = x_moving_speed * step_time * 2;

            foot_y_length1 = pre_y_moving_speed * step_time + y_moving_speed * step_time;
            foot_y_length2 = y_moving_speed * step_time * 2;


            if (move_stop_flag == false) {
                tw_cnt = step_cnt - 1;
            }
        }

    } else {
        //        cout << "[7] end" << endl;
        _c << 1, 1, 1, 1;
        contact_num = 4;

        com_pos = pre_com_pos;
        com_vel = tar_init_com_vel;

        RL_foot_pos = pre_RL_foot_pos;
        RR_foot_pos = pre_RR_foot_pos;
        FL_foot_pos = pre_FL_foot_pos;
        FR_foot_pos = pre_FR_foot_pos;

        RL_foot_vel = tar_init_RL_foot_vel;
        RR_foot_vel = tar_init_RR_foot_vel;
        FL_foot_vel = tar_init_FL_foot_vel;
        FR_foot_vel = tar_init_FR_foot_vel;

        if (tw_cnt == step_cnt * 3) {
            moving_done_flag = true;
        }
    }

    if (Slope_con_onoff_flag == true) Slope_compensation_con();

    if (gain_scheduling_flag == true) {
        //        gain_scheduling(tmp_t);
        if (walking_phase == TW_DSP_RLFR_FIRST || walking_phase == TW_DSP_RRFL || walking_phase == TW_DSP_RLFR) {
            gain_scheduling(tmp_t);
        }
    }
    //    com_pos(0) = com_pos(0) + tmp_com_x_pos;

    base_pos = com_pos + tmp_com_pos + base_offset;

    if (tw_cnt < step_cnt * 4) {
        TW_Turning_Traj_Gen();
        tw_cnt++;
    }

    //    tmp_data1[49] = walking_phase * 0.1;

}

void CRobot::Slope_compensation_con(void) {
    // ============== Orientation Compensation ================ //
    // ----------------------- Pitch --------------------- //
    static double Pz_rear = 0, Pz_front = 0;
    static double tar_pitch_ang = 0; //, lpf_tar_pitch_ang = 0;
    const double max_tar_pitch_ang = 40 * D2R;
    const double dist_front_rear_hip2hip = 0.7;

    Pz_rear = (act_RL_foot_pos_local(2) + act_RR_foot_pos_local(2)) / 2;
    Pz_front = (act_FL_foot_pos_local(2) + act_FR_foot_pos_local(2)) / 2;
    tar_pitch_ang = -atan((Pz_front - Pz_rear) / dist_front_rear_hip2hip) * 1.0;

    if (tar_pitch_ang < 3 * D2R && tar_pitch_ang > -3 * D2R) {
        tar_pitch_ang = 0;
    }

    lpf_tar_pitch_ang = (1 - 0.002) * lpf_tar_pitch_ang + 0.002 * tar_pitch_ang;

    base_ori(1) = lpf_tar_pitch_ang;

    if (base_ori(1) > max_tar_pitch_ang) {
        base_ori(1) = max_tar_pitch_ang;
    } else if (base_ori(1) < -max_tar_pitch_ang) {
        base_ori(1) = -max_tar_pitch_ang;
    }

    //    cout << "base_ori(1) = " << base_ori(1) * R2D << ", lpf_tar_pitch_ang = " << lpf_tar_pitch_ang * R2D << endl;

    // ------------ get COM x,z pos ------------- //

    //    static double tmp_xx = 0; //, tmp_yy = 0;
    //
    //    tmp_xx = com_height * tan(base_ori(1));

    tmp_com_pos << 0, //-tmp_xx * cos(base_ori(1))*0.8, //*0.6,
            0, //tmp_yy*cos(base_ori(0))*1.0,
            0.35 * abs(sin(base_ori(1)))*0.6; //tmp_xx*sin(base_ori(1))*0.5;

    const double max_com_pos_z = 0.12;

    //    if (tmp_com_pos(0) > max_com_pos_x) {
    //        tmp_com_pos(0) = max_com_pos_x;
    //    }
    //    else if (tmp_com_pos(0) < -max_com_pos_x) {
    //        tmp_com_pos(0) = -max_com_pos_x;
    //    }

    if (tmp_com_pos(2) > max_com_pos_z) {
        tmp_com_pos(2) = max_com_pos_z;
    } else if (tmp_com_pos(2) < -max_com_pos_z) {
        tmp_com_pos(2) = -max_com_pos_z;
    }

    //    cout << "[slope_comp] tmp_com_pos = " << tmp_com_pos.transpose() << endl << endl;
    //    cout << "===================================" << endl << endl;
    // ============== Orientation Compensation END ================ //

}

// ====================== Normal trot walking trajectory generation ===================== //

void CRobot::Trot_Walking4(void) {
    tw_time = (double) tw_cnt*dt;
    base_ori(2) = tmp_base_ori(2);

    if (tw_cnt == 0) {
        // ============ Initialize ============ //
        walking_phase = TW_PHASE_INIT;
        //        moving_done_flag = false;
        _c << 1, 1, 1, 1;
        contact_num = 4;

        com_pos = tar_init_com_pos;
        com_vel = tar_init_com_vel;

        RL_foot_pos = tar_init_RL_foot_pos;
        RR_foot_pos = tar_init_RR_foot_pos;
        FL_foot_pos = tar_init_FL_foot_pos;
        FR_foot_pos = tar_init_FR_foot_pos;

        RL_foot_vel = tar_init_RL_foot_vel;
        RR_foot_vel = tar_init_RR_foot_vel;
        FL_foot_vel = tar_init_FL_foot_vel;
        FR_foot_vel = tar_init_FR_foot_vel;

        x_moving_speed = 0;
        y_moving_speed = 0;

        TW_SF_Z_Traj_Gen();
    } else if (tw_cnt < dsp_cnt) {
        // ============ First Step (STANCE_RLFR) ============= //
        walking_phase = TW_DSP_RLFR_FIRST;
        tmp_t = tw_time;

        _c << 1, 0, 0, 1;
        contact_num = 2;

        com_pos = tar_init_com_pos;
        com_vel = tar_init_com_vel;

        RL_foot_pos = tar_init_RL_foot_pos;
        RR_foot_pos = tar_init_RR_foot_pos;
        FL_foot_pos = tar_init_FL_foot_pos;
        FR_foot_pos = tar_init_FR_foot_pos;

        RL_foot_vel = tar_init_RL_foot_vel;
        RR_foot_vel = tar_init_RR_foot_vel;
        FL_foot_vel = tar_init_FL_foot_vel;
        FR_foot_vel = tar_init_FR_foot_vel;

        RR_foot_pos[2] = tar_init_RR_foot_pos[2] + foot_height / 2.0 * (1 - cos(PI2 / dsp_time * tmp_t)); //fifth_order_poly(c_sf_z1, tmp_t);
        FL_foot_pos[2] = tar_init_FL_foot_pos[2] + foot_height / 2.0 * (1 - cos(PI2 / dsp_time * tmp_t)); //fifth_order_poly(c_sf_z1, tmp_t);
        RR_foot_vel[2] = foot_height / 2.0 * PI2 / dsp_time * (sin(PI2 / dsp_time * tmp_t)); //fifth_order_poly_dot(c_sf_z1, tmp_t);
        FL_foot_vel[2] = foot_height / 2.0 * PI2 / dsp_time * (sin(PI2 / dsp_time * tmp_t)); //fRR_foot_vel[2];//fifth_order_poly_dot(c_sf_z1, tmp_t);
    } else if (tw_cnt < step_cnt) {
        // ============ First Step (FSP) ============= //
        walking_phase = TW_FSP;
        tmp_t = tw_time - dsp_time;

        _c << 1, 1, 1, 1;
        contact_num = 4;

        com_pos = tar_init_com_pos;
        com_vel = tar_init_com_vel;

        RL_foot_pos = tar_init_RL_foot_pos;
        RR_foot_pos = tar_init_RR_foot_pos;
        FL_foot_pos = tar_init_FL_foot_pos;
        FR_foot_pos = tar_init_FR_foot_pos;

        RL_foot_vel = tar_init_RL_foot_vel;
        RR_foot_vel = tar_init_RR_foot_vel;
        FL_foot_vel = tar_init_FL_foot_vel;
        FR_foot_vel = tar_init_FR_foot_vel;

        if (tw_cnt == step_cnt - 1) {

            pre_x_moving_speed = x_moving_speed;
            pre_y_moving_speed = y_moving_speed;
            x_moving_speed = tmp_x_moving_speed;
            y_moving_speed = tmp_y_moving_speed;

            pre_com_pos = com_pos;
            pre_RL_foot_pos = RL_foot_pos;
            pre_RR_foot_pos = RR_foot_pos;
            pre_FL_foot_pos = FL_foot_pos;
            pre_FR_foot_pos = FR_foot_pos;

            TW_COM_Traj_Gen();


            foot_x_length1 = pre_x_moving_speed * step_time + x_moving_speed * step_time;
            foot_x_length2 = x_moving_speed * step_time * 2;

            foot_y_length1 = pre_y_moving_speed * step_time + y_moving_speed * step_time;
            foot_y_length2 = y_moving_speed * step_time * 2;
        }
    } else if (tw_cnt < step_cnt + dsp_cnt) {
        // ============ Second Step (STANCE_RRFL) ============ //
        // ============ Continuous Walking Start ============= //
        walking_phase = TW_DSP_RRFL;
        tmp_t = tw_time - step_time;
        tmp_cnt = tw_cnt - step_cnt;

        _c << 0, 1, 1, 0;
        contact_num = 2;

        com_pos = tar_init_com_pos;
        com_vel = tar_init_com_vel;

        RL_foot_pos = pre_RL_foot_pos;
        RR_foot_pos = pre_RR_foot_pos;
        FL_foot_pos = pre_FL_foot_pos;
        FR_foot_pos = pre_FR_foot_pos;

        RL_foot_vel = tar_init_RL_foot_vel;
        RR_foot_vel = tar_init_RR_foot_vel;
        FL_foot_vel = tar_init_FL_foot_vel;
        FR_foot_vel = tar_init_FR_foot_vel;

        com_pos[0] = fifth_order_poly(c_com_x1, tmp_t);
        com_pos[1] = fifth_order_poly(c_com_y1, tmp_t);
        com_vel[0] = fifth_order_poly_dot(c_com_x1, tmp_t);
        com_vel[1] = fifth_order_poly_dot(c_com_y1, tmp_t);

        RL_foot_pos[0] = pre_RL_foot_pos[0] + foot_x_length1 / 2.0 * (1 - cos(PI2 / (dsp_time * 2) * tmp_t)); //fifth_order_poly(c_sf_x1, tmp_t);
        RL_foot_pos[1] = pre_RL_foot_pos[1] + foot_y_length1 / 2.0 * (1 - cos(PI2 / (dsp_time * 2) * tmp_t)); //fifth_order_poly(c_sf_y1, tmp_t);
        FR_foot_pos[0] = pre_FR_foot_pos[0] + foot_x_length1 / 2.0 * (1 - cos(PI2 / (dsp_time * 2) * tmp_t)); //fifth_order_poly(c_sf_x1, tmp_t);
        FR_foot_pos[1] = pre_FR_foot_pos[1] + foot_y_length1 / 2.0 * (1 - cos(PI2 / (dsp_time * 2) * tmp_t)); //(c_sf_y1, tmp_t);

        RL_foot_vel[0] = foot_x_length1 / 2.0 * PI2 / (dsp_time * 2)*(sin(PI2 / (dsp_time * 2) * tmp_t)); //fifth_order_poly_dot(c_sf_x1, tmp_t);
        RL_foot_vel[1] = foot_y_length1 / 2.0 * PI2 / (dsp_time * 2)*(sin(PI2 / (dsp_time * 2) * tmp_t)); //fifth_order_poly_dot(c_sf_y1, tmp_t);

        RL_foot_pos[2] = tar_init_RL_foot_pos[2] + foot_height / 2.0 * (1 - cos(PI2 / dsp_time * tmp_t)); //fifth_order_poly(c_sf_z1, tmp_t);
        FR_foot_pos[2] = tar_init_FR_foot_pos[2] + foot_height / 2.0 * (1 - cos(PI2 / dsp_time * tmp_t)); //fifth_order_poly(c_sf_z1, tmp_t);
        RL_foot_vel[2] = foot_height / 2.0 * PI2 / dsp_time * (sin(PI2 / dsp_time * tmp_t)); //fifth_order_poly_dot(c_sf_z1, tmp_t);

        FR_foot_vel[0] = RL_foot_vel[0];
        FR_foot_vel[1] = RL_foot_vel[1];
        FR_foot_vel[2] = foot_height / 2.0 * PI2 / dsp_time * (sin(PI2 / dsp_time * tmp_t));

        if (tmp_cnt == dsp_cnt - 1) {
            pre_com_pos(0) = pre_com_pos[0] + pre_x_moving_speed * (dsp_time) / 2.0 + x_moving_speed * (dsp_time) / 2.0;
            pre_com_pos(1) = pre_com_pos[1] + pre_y_moving_speed * (dsp_time) / 2.0 + y_moving_speed * (dsp_time) / 2.0; // + tar_cp_y
            pre_com_pos(2) = com_pos(2);

            pre_RL_foot_pos = RL_foot_pos;
            pre_RR_foot_pos = RR_foot_pos;
            pre_FL_foot_pos = FL_foot_pos;
            pre_FR_foot_pos = FR_foot_pos;
        }
    } else if (tw_cnt < step_cnt * 2) {
        // ============ Second Step (FSP) ============= //
        walking_phase = TW_FSP;
        _c << 1, 1, 1, 1;
        contact_num = 4;

        tmp_t = tw_time - step_time - dsp_time;

        com_pos[0] = pre_com_pos(0) + x_moving_speed*tmp_t;
        com_pos[1] = pre_com_pos(1) + y_moving_speed*tmp_t;
        com_pos[2] = pre_com_pos(2);

        com_vel[0] = x_moving_speed;
        com_vel[1] = y_moving_speed;
        com_vel[2] = tar_init_com_vel[2];

        RL_foot_pos = pre_RL_foot_pos;
        RR_foot_pos = pre_RR_foot_pos;
        FL_foot_pos = pre_FL_foot_pos;
        FR_foot_pos = pre_FR_foot_pos;

        RL_foot_vel = tar_init_RL_foot_vel;
        RR_foot_vel = tar_init_RR_foot_vel;
        FL_foot_vel = tar_init_FL_foot_vel;
        FR_foot_vel = tar_init_FR_foot_vel;

        if (tw_cnt == step_cnt * 2 - 1) {

            pre_com_pos(0) = pre_com_pos(0) + x_moving_speed*fsp_time;
            pre_com_pos(1) = pre_com_pos(1) + y_moving_speed*fsp_time;
            pre_com_pos(2) = com_pos(2);

            pre_RL_foot_pos = RL_foot_pos;
            pre_RR_foot_pos = RR_foot_pos;
            pre_FL_foot_pos = FL_foot_pos;
            pre_FR_foot_pos = FR_foot_pos;
        }
    } else if (tw_cnt < step_cnt * 2 + dsp_cnt) {
        // ============ Third Step (STANCE_RLFR) ============= //
        walking_phase = TW_DSP_RLFR;
        tmp_t = tw_time - step_time * 2;
        tmp_cnt = tw_cnt - step_cnt * 2;

        _c << 1, 0, 0, 1;
        contact_num = 2;

        com_pos = pre_com_pos;

        com_vel[0] = x_moving_speed;
        com_vel[1] = y_moving_speed;
        com_vel[2] = tar_init_com_vel[2];

        RL_foot_pos = pre_RL_foot_pos;
        RR_foot_pos = pre_RR_foot_pos;
        FL_foot_pos = pre_FL_foot_pos;
        FR_foot_pos = pre_FR_foot_pos;

        RL_foot_vel = tar_init_RL_foot_vel;
        RR_foot_vel = tar_init_RR_foot_vel;
        FL_foot_vel = tar_init_FL_foot_vel;
        FR_foot_vel = tar_init_FR_foot_vel;

        com_pos[0] = pre_com_pos(0) + x_moving_speed*tmp_t;
        com_pos[1] = pre_com_pos(1) + y_moving_speed*tmp_t;
        com_vel[0] = x_moving_speed;
        com_vel[1] = y_moving_speed;

        RR_foot_pos[0] = pre_RR_foot_pos[0] + foot_x_length2 / 2.0 * (1 - cos(PI2 / (dsp_time * 2) * tmp_t));
        RR_foot_pos[1] = pre_RR_foot_pos[1] + foot_y_length2 / 2.0 * (1 - cos(PI2 / (dsp_time * 2) * tmp_t));
        FL_foot_pos[0] = pre_FL_foot_pos[0] + foot_x_length2 / 2.0 * (1 - cos(PI2 / (dsp_time * 2) * tmp_t));
        FL_foot_pos[1] = pre_FL_foot_pos[1] + foot_y_length2 / 2.0 * (1 - cos(PI2 / (dsp_time * 2) * tmp_t));

        RR_foot_vel[0] = foot_x_length2 / 2.0 * PI2 / (dsp_time * 2)*(sin(PI2 / (dsp_time * 2) * tmp_t));
        RR_foot_vel[1] = foot_y_length2 / 2.0 * PI2 / (dsp_time * 2)*(sin(PI2 / (dsp_time * 2) * tmp_t));

        RR_foot_pos[2] = tar_init_RR_foot_pos[2] + foot_height / 2.0 * (1 - cos(PI2 / dsp_time * tmp_t));
        FL_foot_pos[2] = tar_init_FL_foot_pos[2] + foot_height / 2.0 * (1 - cos(PI2 / dsp_time * tmp_t));
        RR_foot_vel[2] = foot_height / 2.0 * PI2 / dsp_time * (sin(PI2 / dsp_time * tmp_t));

        FL_foot_vel[0] = RR_foot_vel[0];
        FL_foot_vel[1] = RR_foot_vel[1];
        FL_foot_vel[2] = foot_height / 2.0 * PI2 / dsp_time * (sin(PI2 / dsp_time * tmp_t));

        if (tmp_cnt == dsp_cnt - 1) {
            pre_com_pos(0) = pre_com_pos(0) + x_moving_speed*dsp_time;
            pre_com_pos(1) = pre_com_pos(1) + y_moving_speed*dsp_time;
            pre_com_pos(2) = com_pos(2);
            pre_RL_foot_pos = RL_foot_pos;
            pre_RR_foot_pos = RR_foot_pos;
            pre_FL_foot_pos = FL_foot_pos;
            pre_FR_foot_pos = FR_foot_pos;
        }
    } else if (tw_cnt < step_cnt * 3) {
        // ============ Third Step (FSP) ============= //
        walking_phase = TW_FSP;
        _c << 1, 1, 1, 1;
        contact_num = 4;

        tmp_t = tw_time - step_time * 2 - dsp_time;

        com_pos[0] = pre_com_pos(0) + x_moving_speed*tmp_t;
        com_pos[1] = pre_com_pos(1) + y_moving_speed*tmp_t;
        com_pos[2] = pre_com_pos(2);

        com_vel[0] = x_moving_speed;
        com_vel[1] = y_moving_speed;
        com_vel[2] = tar_init_com_vel[2];

        RL_foot_pos = pre_RL_foot_pos;
        RR_foot_pos = pre_RR_foot_pos;
        FL_foot_pos = pre_FL_foot_pos;
        FR_foot_pos = pre_FR_foot_pos;

        RL_foot_vel = tar_init_RL_foot_vel;
        RR_foot_vel = tar_init_RR_foot_vel;
        FL_foot_vel = tar_init_FL_foot_vel;
        FR_foot_vel = tar_init_FR_foot_vel;

        if (tw_cnt == step_cnt * 3 - 1) {

            pre_com_pos(0) = pre_com_pos(0) + x_moving_speed*fsp_time;
            pre_com_pos(1) = pre_com_pos(1) + y_moving_speed*fsp_time;
            pre_com_pos(2) = com_pos(2);

            pre_RL_foot_pos = RL_foot_pos;
            pre_RR_foot_pos = RR_foot_pos;
            pre_FL_foot_pos = FL_foot_pos;
            pre_FR_foot_pos = FR_foot_pos;

            pre_x_moving_speed = x_moving_speed;
            pre_y_moving_speed = y_moving_speed;
            x_moving_speed = tmp_x_moving_speed;
            y_moving_speed = tmp_y_moving_speed;

            TW_COM_Traj_Gen();

            foot_x_length1 = pre_x_moving_speed * step_time + x_moving_speed * step_time;
            foot_x_length2 = x_moving_speed * step_time * 2;

            foot_y_length1 = pre_y_moving_speed * step_time + y_moving_speed * step_time; // + tar_cp_y;
            foot_y_length2 = y_moving_speed * step_time * 2; // + tar_cp_y;

            if (move_stop_flag == false) {
                tw_cnt = step_cnt - 1;
            } else {
                moving_done_flag = true;
            }
        }
    } else {
        if (tw_cnt == step_cnt * 3) {
            cout << "Trot walking Done !!!!!! " << endl;
            walking_phase = TW_FSP_FINAL;

            com_pos = pre_com_pos;
            com_vel = tar_init_com_vel;

            RL_foot_pos = pre_RL_foot_pos;
            RR_foot_pos = pre_RR_foot_pos;
            FL_foot_pos = pre_FL_foot_pos;
            FR_foot_pos = pre_FR_foot_pos;

            RL_foot_vel = tar_init_RL_foot_vel;
            RR_foot_vel = tar_init_RR_foot_vel;
            FL_foot_vel = tar_init_FL_foot_vel;
            FR_foot_vel = tar_init_FR_foot_vel;
        }

        //    	cout << "tw_cnt = " << tw_cnt << endl;
    }

    if (Slope_con_onoff_flag == true) Slope_compensation_con();
    if (gain_scheduling_flag == true) {
        if (walking_phase == TW_DSP_RLFR_FIRST || walking_phase == TW_DSP_RRFL || walking_phase == TW_DSP_RLFR) {
            gain_scheduling(tmp_t);
        }
    }

    base_pos = com_pos + tmp_com_pos + base_offset;

    if (tw_cnt < step_cnt * 4) {
        TW_Turning_Traj_Gen();
        tw_cnt++;
    }

}

void CRobot::gain_scheduling(double _t) {
    //    const double t1 = 0.1;

    for (unsigned int i = 0; i < 4; ++i) {
        if (_c[i] == CONTACT_OFF) {
            // gain  UP and DOWN during DSP time
            tmp_weight = 1 / 2.0 * (1 - cos(PI2 / (dsp_time) * _t));

            if (i < 2) {
                for (unsigned int j = 0; j < 3; ++j) {
                    Kp_q[i * 3 + j] = tar_Kp_q_low[i * 3 + j] + (tar_Kp_q[i * 3 + j] - tar_Kp_q_low[i * 3 + j]) * tmp_weight;
                    Kd_q[i * 3 + j] = tar_Kd_q_low[i * 3 + j] + (tar_Kd_q[i * 3 + j] - tar_Kd_q_low[i * 3 + j]) * tmp_weight;
                    Kp_t[i * 3 + j] = tar_Kp_t_low[i * 3 + j] + (tar_Kp_t[i * 3 + j] - tar_Kp_t_low[i * 3 + j]) * tmp_weight;
                    Kd_t[i * 3 + j] = tar_Kd_t_low[i * 3 + j] + (tar_Kd_t[i * 3 + j] - tar_Kd_t_low[i * 3 + j]) * tmp_weight;
                }
            } else {
                for (unsigned int j = 0; j < 3; ++j) {
                    Kp_q[i * 3 + j + 1] = tar_Kp_q_low[i * 3 + j + 1] + (tar_Kp_q[i * 3 + j + 1] - tar_Kp_q_low[i * 3 + j + 1]) * tmp_weight;
                    Kd_q[i * 3 + j + 1] = tar_Kd_q_low[i * 3 + j + 1] + (tar_Kd_q[i * 3 + j + 1] - tar_Kd_q_low[i * 3 + j + 1]) * tmp_weight;
                    Kp_t[i * 3 + j] = tar_Kp_t_low[i * 3 + j] + (tar_Kp_t[i * 3 + j] - tar_Kp_t_low[i * 3 + j]) * tmp_weight;
                    Kd_t[i * 3 + j] = tar_Kd_t_low[i * 3 + j] + (tar_Kd_t[i * 3 + j] - tar_Kd_t_low[i * 3 + j]) * tmp_weight;
                }
            }
        }
    }
}

void CRobot::TW_COM_Traj_Gen(void) {
    // ============ COM Pos. ========== //
    init_x[0] = pre_com_pos[0];
    init_x[1] = pre_x_moving_speed;
    init_x[2] = 0;

    final_x[0] = pre_com_pos[0] + pre_x_moving_speed * (dsp_time) / 2.0 + x_moving_speed * (dsp_time) / 2.0;
    final_x[1] = x_moving_speed;
    final_x[2] = 0;

    coefficient_5thPoly(init_x, final_x, dsp_time, c_com_x1);

    init_x[0] = pre_com_pos[1];
    init_x[1] = pre_y_moving_speed;
    init_x[2] = 0;

    final_x[0] = pre_com_pos[1] + pre_y_moving_speed * (dsp_time) / 2.0 + y_moving_speed * (dsp_time) / 2.0;
    final_x[1] = y_moving_speed;
    final_x[2] = 0;

    coefficient_5thPoly(init_x, final_x, dsp_time, c_com_y1);
}

void CRobot::TW_SF_Traj_Gen(void) {
    //    // ============ Swing Foot Pos. ============ //
    //    //    const double Kd_com = 0.1;
    //    //    static double tmp_foot_x_length = 0;
    //    //    const double max_foot_x_length = 0.05;
    //    //
    //    //
    //    //    tmp_foot_x_length = Kd_com * (com_vel[0]);// - tmp_com_vel[0]);
    //    //
    //    //    if(tmp_foot_x_length > max_foot_x_length){
    //    //        tmp_foot_x_length = max_foot_x_length;
    //    //    }
    //    //    else if(tmp_foot_x_length < -max_foot_x_length){
    //    //        tmp_foot_x_length = -max_foot_x_length;
    //    //    }
    //    //
    //    //    cout << "com vel err = " << com_vel[0] - tmp_com_vel[0] << endl;
    //    //    cout << "tmp foot length = " << tmp_foot_x_length << endl;
    //
    //    // Left (First)
    //    init_x[0] = 0;
    //    init_x[1] = 0;
    //    init_x[2] = 0;
    //
    //
    //    //        des_x_2dot = com_acc * 1.0 + Kp_x * (com_pos - act_com_pos) + Kd_x * (com_vel - act_com_vel);
    //
    //    final_x[0] = 0 + pre_x_moving_speed * step_time + x_moving_speed * step_time; // + tmp_foot_x_length;
    //    final_x[1] = 0;
    //    final_x[2] = 0;
    //
    //    coefficient_5thPoly(init_x, final_x, dsp_time, c_sf_x1);
    //
    //    // Right (First)
    //    init_x[0] = 0;
    //    init_x[1] = 0;
    //    init_x[2] = 0;
    //
    //    final_x[0] = 0 + x_moving_speed * step_time * 2; // + tmp_foot_x_length;
    //    final_x[1] = 0;
    //    final_x[2] = 0;
    //
    //    coefficient_5thPoly(init_x, final_x, dsp_time, c_sf_x2);
    //
    //
    //
    //    //    // Left (Second)
    //    //    init_x[0] = 0 + pre_x_moving_speed * step_time + x_moving_speed * step_time;
    //    //    init_x[1] = 0;
    //    //    init_x[2] = 0;
    //    //
    //    //    final_x[0] = 0 + pre_x_moving_speed * step_time + x_moving_speed * step_time * 3;
    //    //    final_x[1] = 0;
    //    //    final_x[2] = 0;
    //    //
    //    //    coefficient_5thPoly(init_x, final_x, dsp_time, c_sf_x3);
    //    //
    //    //    // Right (Second)
    //    //    init_x[0] = 0 + x_moving_speed * step_time * 2;
    //    //    init_x[1] = 0;
    //    //    init_x[2] = 0;
    //    //
    //    //    final_x[0] = 0 + x_moving_speed * step_time * 4;
    //    //    final_x[1] = 0;
    //    //    final_x[2] = 0;
    //    //
    //    //    coefficient_5thPoly(init_x, final_x, dsp_time, c_sf_x4);
    //
    //    // ================= Y axis ================== //
    //
    //    // Left (First)
    //    init_x[0] = 0;
    //    init_x[1] = 0;
    //    init_x[2] = 0;
    //
    //    final_x[0] = 0 + pre_y_moving_speed * step_time + y_moving_speed * step_time;
    //    final_x[1] = 0;
    //    final_x[2] = 0;
    //
    //    coefficient_5thPoly(init_x, final_x, dsp_time, c_sf_y1);
    //
    //    // Right (First)
    //    init_x[0] = 0;
    //    init_x[1] = 0;
    //    init_x[2] = 0;
    //
    //    final_x[0] = 0 + y_moving_speed * step_time * 2;
    //    final_x[1] = 0;
    //    final_x[2] = 0;
    //
    //    coefficient_5thPoly(init_x, final_x, dsp_time, c_sf_y2);
    //
    //    //    // Left (Second)
    //    //    init_x[0] = 0 + pre_y_moving_speed * step_time + y_moving_speed * step_time;
    //    //    init_x[1] = 0;
    //    //    init_x[2] = 0;
    //    //
    //    //    final_x[0] = 0 + pre_y_moving_speed * step_time + y_moving_speed * step_time * 3;
    //    //    final_x[1] = 0;
    //    //    final_x[2] = 0;
    //    //
    //    //    coefficient_5thPoly(init_x, final_x, dsp_time, c_sf_y3);
    //    //
    //    //    // Right (Second)
    //    //    init_x[0] = 0 + y_moving_speed * step_time * 2;
    //    //    init_x[1] = 0;
    //    //    init_x[2] = 0;
    //    //
    //    //    final_x[0] = 0 + y_moving_speed * step_time * 4;
    //    //    final_x[1] = 0;
    //    //    final_x[2] = 0;
    //    //
    //    //    coefficient_5thPoly(init_x, final_x, dsp_time, c_sf_y4);
}

void CRobot::TW_Turning_Traj_Gen(void) {
    // ======= Turning trajectory generation ======= //
    //    printf("================= [TW] Turning trajectory generation ============== \n");

    const double turn_l = 0.4134; // sqrt(0.35^2 + (0.115 + 0.105)^2)
    const double x1 = 0.35, y1 = 0.22, turn_theta1 = 57.8477 * D2R;
    static double x2 = 0.35, y2 = 0.22, turn_theta2 = 0;
    static double del_x = 0, del_y = 0;
    static double target_theta = turn_theta1;
    static double tmp_target_theta = 0;

    // turn left
    if ((base_ori(2) > 0.01) && (tw_cnt == 2 * step_cnt)) {
        if (turn_mode == 0) {
            turn_start_flag = true;
            turn_mode = 1;

            target_theta = -base_ori(2) * 2.0; // [rad]

            //            printf("[left] target_theta = %f\n", target_theta);
        }
    } else if ((base_ori(2) < -0.01) && (tw_cnt == step_cnt)) {
        if (turn_mode == 0) {
            turn_start_flag = true;
            turn_mode = 3;

            target_theta = base_ori(2) * 2.0; // [rad]

            //            printf("[right] target_theta = %f\n", target_theta);
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
            } else {
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

            if (turn_cnt == step_cnt - 1) {
                turn_mode = 2;
                turn_cnt = 0;
            }
        } else if (turn_mode == 2) {
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
            } else {
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
        } else if (turn_mode == 3) {
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
            } else {
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
            //            if (FC_PHASE == STANCE_RLFR) {
            if (turn_cnt == step_cnt - 1) {
                turn_mode = 4;
                turn_cnt = 0;
            }
        } else if (turn_mode == 4) {
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
            } else {
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
    } else {
        turn_mode = 0;
        turn_cnt = 0;

        turn_xl_EP = 0;
        turn_yl_EP = 0;
        turn_xr_EP = 0;
        turn_yr_EP = 0;
    }

    RL_foot_pos_local_offset << -turn_xr_EP, -turn_yr_EP, 0;
    RR_foot_pos_local_offset << -turn_xl_EP, -turn_yl_EP, 0;
    FL_foot_pos_local_offset << +turn_xl_EP, +turn_yl_EP, 0;
    FR_foot_pos_local_offset << +turn_xr_EP, +turn_yr_EP, 0;

    //    cout << "RL_foot_pos_local_offset = " << RL_foot_pos_local_offset.transpose() << endl;
    //    cout << "RR_foot_pos_local_offset = " << RR_foot_pos_local_offset.transpose() << endl;
    //    cout << "FL_foot_pos_local_offset = " << FL_foot_pos_local_offset.transpose() << endl;
    //    cout << "FR_foot_pos_local_offset = " << FR_foot_pos_local_offset.transpose() << endl;

    // ======= Turning trajectory generation  END ======= //
}

void CRobot::TW_COM_SF_X_Traj_Gen(void) {
    //    // ============ COM Pos. ========== //
    //    init_x[0] = pre_com_pos[0];
    //    init_x[1] = pre_x_moving_speed;
    //    init_x[2] = 0;
    //
    //    final_x[0] = pre_com_pos[0] + pre_x_moving_speed * (dsp_time) / 2.0 + x_moving_speed * (dsp_time) / 2.0;
    //    final_x[1] = x_moving_speed;
    //    final_x[2] = 0;
    //
    //    coefficient_5thPoly(init_x, final_x, dsp_time, c_com_x1);
    //
    //    // ============ Swing Foot Pos. ============ //
    //    // Left (First)
    //    init_x[0] = 0;
    //    init_x[1] = 0;
    //    init_x[2] = 0;
    //
    //    final_x[0] = 0 + pre_x_moving_speed * step_time + x_moving_speed * step_time;
    //    final_x[1] = 0;
    //    final_x[2] = 0;
    //
    //    coefficient_5thPoly(init_x, final_x, dsp_time, c_sf_x1);
    //
    //    // Right (First)
    //    init_x[0] = 0;
    //    init_x[1] = 0;
    //    init_x[2] = 0;
    //
    //    final_x[0] = 0 + x_moving_speed * step_time * 2;
    //    final_x[1] = 0;
    //    final_x[2] = 0;
    //
    //    coefficient_5thPoly(init_x, final_x, dsp_time, c_sf_x2);
    //
    //    // Left (Second)
    //    init_x[0] = 0 + pre_x_moving_speed * step_time + x_moving_speed * step_time;
    //    init_x[1] = 0;
    //    init_x[2] = 0;
    //
    //    final_x[0] = 0 + pre_x_moving_speed * step_time + x_moving_speed * step_time * 3;
    //    final_x[1] = 0;
    //    final_x[2] = 0;
    //
    //    coefficient_5thPoly(init_x, final_x, dsp_time, c_sf_x3);
    //
    //    // Right (Second)
    //    init_x[0] = 0 + x_moving_speed * step_time * 2;
    //    init_x[1] = 0;
    //    init_x[2] = 0;
    //
    //    final_x[0] = 0 + x_moving_speed * step_time * 4;
    //    final_x[1] = 0;
    //    final_x[2] = 0;
    //
    //    coefficient_5thPoly(init_x, final_x, dsp_time, c_sf_x4);
}

void CRobot::TW_SF_Z_Traj_Gen(void) {
    init_x[0] = 0;
    init_x[1] = 0;
    init_x[2] = 0;

    final_x[0] = foot_height;
    final_x[1] = 0;
    final_x[2] = 0;

    coefficient_5thPoly(init_x, final_x, dsp_time / 2.0, c_sf_z1);

    init_x[0] = foot_height;
    init_x[1] = 0;
    init_x[2] = 0;

    final_x[0] = 0;
    final_x[1] = 0;
    final_x[2] = 0;

    coefficient_5thPoly(init_x, final_x, dsp_time / 2.0, c_sf_z2);
}

void CRobot::Base_Ori_Con2(void) {
    //    //    static double sum_roll_err = 0.;//, sum_pitch_err = 0.;
    //    static double del_L_left = 0, del_L_right = 0, del_L_front = 0, del_L_rear = 0; //, del_L_rl = 0, del_L_rr = 0;
    //    const double limit_foot_z = 0.05;
    //    const double IMURoll_alpha = 0.01;
    //    static double lpf_IMURoll = 0, lpf_IMUPitch = 0;
    //    static double tmp_IMURoll = 0;
    //
    //    // ==================== Slop Compensation Control ==================== //
    //
    //    if (IMURoll * R2D < 2.0 && IMURoll * R2D > -2.0) {
    //        tmp_IMURoll = 0;
    //    }
    //    else {
    //        tmp_IMURoll = IMURoll;
    //    }
    //
    //    // roll
    //    lpf_IMURoll = (1 - IMURoll_alpha) * lpf_IMURoll + IMURoll_alpha*tmp_IMURoll;
    //
    //    //    lpf_IMURoll = IMURoll;
    //
    //    if (del_L_left < limit_foot_z && del_L_left > -limit_foot_z) {
    //        BOC_sum_roll_err = BOC_sum_roll_err + (0 - lpf_IMURoll) * dt;
    //    }
    //
    //    del_L_left = BOC_Kp_roll * (0 - lpf_IMURoll) + BOC_Ki_roll*BOC_sum_roll_err;
    //    del_L_right = -BOC_Kp_roll * (0 - lpf_IMURoll) - BOC_Ki_roll*BOC_sum_roll_err;
    //
    //    //    tmp_data2[61] = -del_L_left;
    //    //    tmp_data2[62] = -del_L_right;
    //
    //
    //    //    // pitch
    //    //    //    lpf_IMUPitch = (1 - IMUPitch_alpha) * lpf_IMUPitch + IMUPitch_alpha*IMUPitch;
    //    //    lpf_IMUPitch = IMUPitch;
    //    //
    //    //    if (del_L_front < limit_foot_z && del_L_front > -limit_foot_z) {
    //    //        BOC_sum_pitch_err = BOC_sum_pitch_err + (base_ori(1) - lpf_IMUPitch) * dt;
    //    //    }
    //    //
    //    //    del_L_front = BOC_Kp_pitch * (base_ori(1) - lpf_IMUPitch) + BOC_Ki_pitch*BOC_sum_pitch_err;
    //    //    del_L_rear = -BOC_Kp_pitch * (base_ori(1) - lpf_IMUPitch) - BOC_Ki_pitch*BOC_sum_pitch_err;
    //    //
    //    //
    //    //    tmp_data2[61] =del_L_front;
    //    //    tmp_data2[62] = del_L_rear;
    //    ////    cout << "base_ori(1) = " << base_ori(1)*R2D << ", lpf_IMUPitch = " << lpf_IMUPitch*R2D << endl;
    //    ////    cout << "del_L_front = " << del_L_front << ", del_L_rear = " << del_L_rear << endl;
    //    ////    cout << "======================" << endl << endl;
    //    //    //    printf("lpf_IMUPitch = %f, del_L_front = %f\n",lpf_IMUPitch,del_L_front);
    //    //
    //
    //    //    del_L_rear =   0.35*sin(base_ori(1));
    //    //    del_L_front = -0.35*sin(base_ori(1));
    //
    //    //    del_L_rear = 0.35 * sin(base_ori(1))*0.8;
    //    //    del_L_front = -0.35 * sin(base_ori(1))*0.8;
    //
    //    //    RL_foot_pos_local_offset(2) = -del_L_left + del_L_rear;
    //    //    RR_foot_pos_local_offset(2) = -del_L_right + del_L_rear;
    //    //    FL_foot_pos_local_offset(2) = -del_L_left + del_L_front;
    //    //    FR_foot_pos_local_offset(2) = -del_L_right + del_L_front;
    //
    //
    //    RL_foot_pos_local_offset(2) = -del_L_left;
    //    RR_foot_pos_local_offset(2) = -del_L_right;
    //    FL_foot_pos_local_offset(2) = -del_L_left;
    //    FR_foot_pos_local_offset(2) = -del_L_right;
    //
    //    cout << "lpf_IMURoll = " << lpf_IMURoll << endl;
    //    cout << "del_L_left = " << -del_L_left << ", del_L_right = " << -del_L_right << endl;
    //
    //
    //    if (RL_foot_pos_local_offset(2) > limit_foot_z) {
    //        RL_foot_pos_local_offset(2) = limit_foot_z;
    //    }
    //    else if (RL_foot_pos_local_offset(2) < -limit_foot_z) {
    //        RL_foot_pos_local_offset(2) = -limit_foot_z;
    //    }
    //
    //    if (RR_foot_pos_local_offset(2) > limit_foot_z) {
    //        RR_foot_pos_local_offset(2) = limit_foot_z;
    //    }
    //    else if (RR_foot_pos_local_offset(2) < -limit_foot_z) {
    //        RR_foot_pos_local_offset(2) = -limit_foot_z;
    //    }
    //
    //    if (FL_foot_pos_local_offset(2) > limit_foot_z) {
    //        FL_foot_pos_local_offset(2) = limit_foot_z;
    //    }
    //    else if (FL_foot_pos_local_offset(2) < -limit_foot_z) {
    //        FL_foot_pos_local_offset(2) = -limit_foot_z;
    //    }
    //
    //    if (FR_foot_pos_local_offset(2) > limit_foot_z) {
    //        FR_foot_pos_local_offset(2) = limit_foot_z;
    //    }
    //    else if (FR_foot_pos_local_offset(2) < -limit_foot_z) {
    //        FR_foot_pos_local_offset(2) = -limit_foot_z;
    //    }
    //
    //    //    cout << "RL = " << RL_foot_pos_local_offset(2) << ", RR = " << RR_foot_pos_local_offset(2) << ",FL = " << FL_foot_pos_local_offset(2) << ",FR = " << FR_foot_pos_local_offset(2) << endl;
    //
    //    //        printf("RL_offset(2) = %f,RR_offset(2) = %f\n",RL_foot_pos_local_offset(2),RR_foot_pos_local_offset(2));
}

void CRobot::coefficient_5thPoly(double *init_x, double *final_x, double tf, double *output) {
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

void CRobot::Torque_off(void) {
    for (int i = 0; i < nDOF; ++i) {
        joint[i].torque = 0;
        target_vel[i] = 0;
        target_acc[i] = 0;
    }
}

void CRobot::Robot_para_init(void) {
    _c << 1, 1, 1, 1;
    contact_num = 4;

    com_pos = tar_init_com_pos;
    com_vel = tar_init_com_vel;

    RL_foot_pos = tar_init_RL_foot_pos;
    RR_foot_pos = tar_init_RR_foot_pos;
    FL_foot_pos = tar_init_FL_foot_pos;
    FR_foot_pos = tar_init_FR_foot_pos;

    RL_foot_vel = tar_init_RL_foot_vel;
    RR_foot_vel = tar_init_RR_foot_vel;
    FL_foot_vel = tar_init_FL_foot_vel;
    FR_foot_vel = tar_init_FR_foot_vel;

    x_moving_speed = 0;
    y_moving_speed = 0;

    base_pos = com_pos + tmp_com_pos + base_offset;

    //    cout << "base_pos = " << base_pos.transpose() << endl;
    //    cout << "com_pos = " << com_pos.transpose() << endl;
    //    cout << "tmp_com_pos = " << tmp_com_pos.transpose() << endl;
    //    cout << "base_offset = " << base_offset.transpose() << endl;
    //    cout << "================================================== " << endl << endl;
    //
}

void CRobot::Get_act_com(void) {
    // ============== Get COM Position & Orientation ============ //

    if (contact_num != 0) {
        tmp_act_base_pos(0) = (_c(0) * (RL_foot_pos[0] - act_RL_foot_pos_local[0]) + _c(1) * (RR_foot_pos[0] - act_RR_foot_pos_local[0]) + _c(2) * (FL_foot_pos[0] - act_FL_foot_pos_local[0]) + _c(3) * (FR_foot_pos[0] - act_FR_foot_pos_local[0])) / contact_num;
        tmp_act_base_pos(1) = (_c(0) * (RL_foot_pos[1] - act_RL_foot_pos_local[1]) + _c(1) * (RR_foot_pos[1] - act_RR_foot_pos_local[1]) + _c(2) * (FL_foot_pos[1] - act_FL_foot_pos_local[1]) + _c(3) * (FR_foot_pos[1] - act_FR_foot_pos_local[1])) / contact_num;
        tmp_act_base_pos(2) = (_c(0) * (RL_foot_pos[2] - act_RL_foot_pos_local[2]) + _c(1) * (RR_foot_pos[2] - act_RR_foot_pos_local[2]) + _c(2) * (FL_foot_pos[2] - act_FL_foot_pos_local[2]) + _c(3) * (FR_foot_pos[2] - act_FR_foot_pos_local[2])) / contact_num;

        tmp_act_base_vel(0) = -(_c(0) * actual_EP_vel[0] + _c(1) * actual_EP_vel[3] + _c(2) * actual_EP_vel[6] + _c(3) * actual_EP_vel[9]) / contact_num;
        tmp_act_base_vel(1) = -(_c(0) * actual_EP_vel[1] + _c(1) * actual_EP_vel[4] + _c(2) * actual_EP_vel[7] + _c(3) * actual_EP_vel[10]) / contact_num;
        tmp_act_base_vel(2) = -(_c(0) * actual_EP_vel[2] + _c(1) * actual_EP_vel[5] + _c(2) * actual_EP_vel[8] + _c(3) * actual_EP_vel[11]) / contact_num;

    } else {
        tmp_act_base_pos(0) = ((RL_foot_pos[0] - act_RL_foot_pos_local[0]) + (RR_foot_pos[0] - act_RR_foot_pos_local[0]) + (FL_foot_pos[0] - act_FL_foot_pos_local[0]) + (FR_foot_pos[0] - act_FR_foot_pos_local[0])) / 4;
        tmp_act_base_pos(1) = ((RL_foot_pos[1] - act_RL_foot_pos_local[1]) + (RR_foot_pos[1] - act_RR_foot_pos_local[1]) + (FL_foot_pos[1] - act_FL_foot_pos_local[1]) + (FR_foot_pos[1] - act_FR_foot_pos_local[1])) / 4;
        tmp_act_base_pos(2) = ((RL_foot_pos[2] - act_RL_foot_pos_local[2]) + (RR_foot_pos[2] - act_RR_foot_pos_local[2]) + (FL_foot_pos[2] - act_FL_foot_pos_local[2]) + (FR_foot_pos[2] - act_FR_foot_pos_local[2])) / 4;

        tmp_act_base_vel(0) = -(actual_EP_vel[0] + actual_EP_vel[3] + actual_EP_vel[6] + actual_EP_vel[9]) / 4;
        tmp_act_base_vel(1) = -(actual_EP_vel[1] + actual_EP_vel[4] + actual_EP_vel[7] + actual_EP_vel[10]) / 4;
        tmp_act_base_vel(2) = -(actual_EP_vel[2] + actual_EP_vel[5] + actual_EP_vel[8] + actual_EP_vel[11]) / 4;
    }


    //    cout << "[1] tmp_act_base_pos = " << tmp_act_base_pos.transpose() << endl;
    //    cout << "[1] tmp_act_base_vel = " << tmp_act_base_vel.transpose() << endl;
    //

    //    if(ControlMode == CTRLMODE_NONE){
    //        act_base_pos = (1 - pos_alpha) * act_base_pos + pos_alpha*tmp_act_base_pos;
    //        act_base_vel = (1 - vel_alpha) * act_base_vel + vel_alpha*tmp_act_base_vel;
    //    }
    //    else{
    //        act_base_pos = tmp_act_base_pos;
    //        act_base_vel = tmp_act_base_vel;
    //    }

    if (move_cnt == 0 && CommandFlag != GOTO_WALK_READY_POS) {
        act_base_pos = tmp_act_base_pos;
        act_base_vel = tmp_act_base_vel;
        pre_act_com_vel = act_base_vel;

        //        move_cnt++;

        //        cout << "=================== move_cnt = 0 !!!! ====================" << endl<< endl;
    }

    act_base_pos = (1 - pos_alpha) * act_base_pos + pos_alpha*tmp_act_base_pos;
    act_base_vel = (1 - vel_alpha) * act_base_vel + vel_alpha*tmp_act_base_vel;

    //    cout << "act_base_pos = " << act_base_pos.transpose() << endl;
    //    cout << "base_pos = " << base_pos.transpose() << endl;
    //
    //    cout << "act_base_vel = " << act_base_vel.transpose() << endl;
    //    cout << "base_vel = " << base_vel.transpose() << endl;


    //    cout << "act_base_pos(0) = " << act_base_pos(0) << ", act_base_pos(1) = " << act_base_pos(1) << ", act_base_pos(2) = " << act_base_pos(2) << endl;
    //    cout << "act_base_pos2(0) = " << act_base_pos2(0) << ", act_base_pos2(1) = " << act_base_pos2(1) << ", act_base_pos2(2) = " << act_base_pos2(2) << endl;
    //    cout << "act_RL_foot_pos_local[2] = " << act_RL_foot_pos_local[2] << endl;
    //    cout << "===============================================================" << endl;

    act_com_pos = act_base_pos - tmp_com_pos - base_offset;
    act_com_vel = act_base_vel; //(0.90*lpf_base_alpha)*pre_act_com_vel + ((1 - 0.90)*lpf_base_alpha)*act_base_vel;

    //    cout <<
    //    static double lpf_com_x_vel = 0;

    //    tmp_com_vel[0] = (1 - 0.001)*tmp_com_vel[0] + 0.001*act_com_vel[0];
    //
    //    tmp_data2[41] = tmp_com_vel[0];//tmp_com_vel(0);


    tmp_act_com_acc = (act_com_vel - pre_act_com_vel) / dt;
    pre_act_com_vel = act_com_vel;

    act_com_acc = (1 - 0.02) * act_com_acc + 0.02 * tmp_act_com_acc;

    //            cout << "IMURoll = " << IMURoll*R2D << endl;
    tmp_act_base_ori << IMURoll, IMUPitch, IMUYaw - init_IMUYaw;
    tmp_act_base_ori_dot << IMURoll_dot, IMUPitch_dot, IMUYaw_dot;

    // low pass filter
    const double tmp_base_alpha = 1;
    act_base_ori = (1 - tmp_base_alpha) * act_base_ori + tmp_base_alpha*tmp_act_base_ori;
    act_base_ori_dot = (1 - tmp_base_alpha) * act_base_ori_dot + tmp_base_alpha*tmp_act_base_ori_dot;


    //    cout << "act_base_ori = " << act_base_ori.transpose()*R2D << endl;
    //    cout << "base_ori = " << base_ori.transpose()*R2D << endl;
    //
    //    cout << "act_base_ori_dot = " << act_base_ori_dot.transpose()*R2D << endl;
    //    cout << "base_ori_dot = " << base_ori_dot.transpose()*R2D << endl;

    //    tmp_act_base_ori_dot << IMURoll_dot, IMUPitch_dot, IMUYaw_dot;
    //
    //    act_base_ori_dot = (0.90*lpf_base_alpha)*pre_act_base_ori_dot + ((1 - 0.90)*lpf_base_alpha)*tmp_act_base_ori_dot;
    //
    //    pre_act_base_ori_dot = act_base_ori_dot;
    if (act_base_ori(0) > 40 * D2R) {
        act_base_ori(0) = 40 * D2R;
    } else if (act_base_ori(0) < -40 * D2R) {
        act_base_ori(0) = -40 * D2R;
    }

    if (act_base_ori(1) > 40 * D2R) {
        act_base_ori(1) = 40 * D2R;
    } else if (act_base_ori(1) < -40 * D2R) {
        act_base_ori(1) = -40 * D2R;
    }

    //    if (act_base_ori(2) > 20 * D2R) {
    //        act_base_ori(2) = 20 * D2R;
    //    }
    //    else if (act_base_ori(2) < -20 * D2R) {
    //        act_base_ori(2) = -20 * D2R;
    //    }
}

// ====================== flying trot trajectory generation ===================== //

void CRobot::Flying_Trot_Running3(void) {
    static double com_t = 0;
    static double sf_rl_t = 0, sf_rl_t2 = 0;
    static double sf_rr_t = 0, sf_rr_t2 = 0;
    static double tar_rl_foot_pos = 0, tar_rr_foot_pos = 0;
    static bool init_flag = false;

    ft_time = (double) ft_cnt*dt;
    base_ori(2) = tmp_base_ori(2);

    //    if(tmp_x_moving_speed < 2.8){
    //        tmp_x_moving_speed = tmp_x_moving_speed + 0.0002;
    //    }
    //    else{
    //        tmp_x_moving_speed = 2.8;
    //    }

    if (ft_cnt == 0) {
        //        cout << "=========== ft_phase = 0 ===========" << endl;
        ft_phase = 0;
        //        moving_done_flag = false;
        _c << 1, 1, 1, 1;
        contact_num = 4;

        com_t = 0;
        sf_rl_t = 0;
        sf_rl_t2 = 0;
        sf_rr_t = 0;
        sf_rr_t2 = 0;
        tar_rl_foot_pos = 0;
        tar_rr_foot_pos = 0;
        init_flag = false;

        com_pos = tar_init_com_pos;
        com_vel = tar_init_com_vel;
        com_acc = tar_init_com_acc;

        RL_foot_pos = tar_init_RL_foot_pos;
        RR_foot_pos = tar_init_RR_foot_pos;
        FL_foot_pos = tar_init_FL_foot_pos;
        FR_foot_pos = tar_init_FR_foot_pos;

        RL_foot_vel = tar_init_RL_foot_vel;
        RR_foot_vel = tar_init_RR_foot_vel;
        FL_foot_vel = tar_init_FL_foot_vel;
        FR_foot_vel = tar_init_FR_foot_vel;

        pre_com_pos = com_pos;
        pre_com_vel = com_vel;

        pre_RL_foot_pos = RL_foot_pos;
        pre_RR_foot_pos = RR_foot_pos;
        pre_FL_foot_pos = FL_foot_pos;
        pre_FR_foot_pos = FR_foot_pos;


        x_moving_speed = 0;

        FT_COM_SF_Z_Traj_Gen();
    } else if (ft_cnt < ts_cnt) {
        //        cout << "=========== ft_phase = 1 ===========" << endl;
        ft_phase = 1;
        _c << 1, 0, 0, 1;
        contact_num = 2;
        com_t = ft_time;
        sf_rr_t = sf_rr_t + dt;

        com_pos[2] = fifth_order_poly(c_com_z1, com_t);
        com_vel[2] = fifth_order_poly_dot(c_com_z1, com_t);
        com_acc[2] = fifth_order_poly_2dot(c_com_z1, com_t);

        RL_foot_pos[2] = tar_init_RL_foot_pos[2];
        FR_foot_pos[2] = tar_init_FR_foot_pos[2];
        RL_foot_vel[2] = tar_init_RL_foot_vel[2];

        if (sf_rr_t < (ts + tf) / 2.0) {
            RR_foot_pos[2] = tar_init_RR_foot_pos[2] + fifth_order_poly(c_sf_z1, sf_rr_t);
            FL_foot_pos[2] = tar_init_FL_foot_pos[2] + fifth_order_poly(c_sf_z1, sf_rr_t);
            RR_foot_vel[2] = fifth_order_poly_dot(c_sf_z1, sf_rr_t);

        } else {
            sf_rr_t2 = sf_rr_t - (ts + tf) / 2.0;
            RR_foot_pos[2] = tar_init_RR_foot_pos[2] + fifth_order_poly(c_sf_z2, sf_rr_t2);
            FL_foot_pos[2] = tar_init_FL_foot_pos[2] + fifth_order_poly(c_sf_z2, sf_rr_t2);
            RR_foot_vel[2] = fifth_order_poly_dot(c_sf_z2, sf_rr_t2);
        }

        FL_foot_vel = RR_foot_vel;
        FR_foot_vel = RL_foot_vel;

        if (ft_cnt == ts_cnt - 1) {
            pre_x_moving_speed = x_moving_speed;
            x_moving_speed = tmp_x_moving_speed;
            //            tar_rl_foot_pos = pre_x_moving_speed*(ts + tf) + x_moving_speed*(ts + tf);
            tar_rl_foot_pos = pre_com_pos[0] - 0.35 + pre_x_moving_speed * tf + (pre_x_moving_speed + x_moving_speed) * ts / 2.0 + x_moving_speed * tf + x_moving_speed * ts / 2.0;

            init_flag = true;
            tar_rr_foot_pos = pre_RR_foot_pos[0];
            sf_rl_t = 0;
        }
    } else if (ft_cnt < ft_step_cnt) {
        //        cout << "=========== ft_phase = 2 ===========" << endl;
        ft_phase = 2;
        _c << 0, 0, 0, 0;
        contact_num = 0;
        //        _c << 1, 0, 0, 1;
        //		contact_num = 2;
        com_t = ft_time - ts;
        sf_rl_t = sf_rl_t + dt;
        sf_rr_t = sf_rr_t + dt;

        com_pos[0] = pre_com_pos[0] + pre_x_moving_speed*com_t;
        com_pos[2] = fifth_order_poly(c_com_z2, com_t);

        com_vel[0] = pre_x_moving_speed;
        com_vel[2] = fifth_order_poly_dot(c_com_z2, com_t);

        com_acc[2] = fifth_order_poly_2dot(c_com_z2, com_t);

        RL_foot_pos[0] = pre_RL_foot_pos[0] + (tar_rl_foot_pos - pre_RL_foot_pos[0]) / 2.0 * (1 - cos(PI2 / (2 * (ft_step_time + tf)) * sf_rl_t));
        FR_foot_pos[0] = pre_FR_foot_pos[0] + (tar_rl_foot_pos - pre_RL_foot_pos[0]) / 2.0 * (1 - cos(PI2 / (2 * (ft_step_time + tf)) * sf_rl_t));
        RL_foot_vel[0] = (tar_rl_foot_pos - pre_RL_foot_pos[0]) / 2.0 * PI2 / (2 * (ft_step_time + tf))*(sin(PI2 / (2 * (ft_step_time + tf)) * sf_rl_t));

        RL_foot_pos[2] = tar_init_RL_foot_pos[2] + fifth_order_poly(c_sf_z3, sf_rl_t);
        FR_foot_pos[2] = tar_init_FR_foot_pos[2] + fifth_order_poly(c_sf_z3, sf_rl_t);
        RL_foot_vel[2] = fifth_order_poly_dot(c_sf_z3, sf_rl_t);

        if (init_flag != true) {
            RR_foot_pos[0] = pre_RR_foot_pos[0] + (tar_rr_foot_pos - pre_RR_foot_pos[0]) / 2.0 * (1 - cos(PI2 / (2 * (ft_step_time + tf)) * sf_rr_t));
            FL_foot_pos[0] = pre_FL_foot_pos[0] + (tar_rr_foot_pos - pre_RR_foot_pos[0]) / 2.0 * (1 - cos(PI2 / (2 * (ft_step_time + tf)) * sf_rr_t));
            RR_foot_vel[0] = (tar_rr_foot_pos - pre_RR_foot_pos[0]) / 2.0 * PI2 / (2 * (ft_step_time + tf))*(sin(PI2 / (2 * (ft_step_time + tf)) * sf_rr_t));

            sf_rr_t2 = sf_rr_t - (ft_step_time + tf) / 2.0;
            RR_foot_pos[2] = tar_init_RR_foot_pos[2] + fifth_order_poly(c_sf_z4, sf_rr_t2);
            FL_foot_pos[2] = tar_init_FL_foot_pos[2] + fifth_order_poly(c_sf_z4, sf_rr_t2);
            RR_foot_vel[2] = fifth_order_poly_dot(c_sf_z4, sf_rr_t2);

        } else {
            RR_foot_pos[0] = tar_init_RR_foot_pos[0];
            FL_foot_pos[0] = tar_init_FL_foot_pos[0];
            RR_foot_vel[0] = tar_init_RR_foot_vel[0];

            sf_rr_t2 = sf_rr_t - (ts + tf) / 2.0;
            RR_foot_pos[2] = tar_init_RR_foot_pos[2] + fifth_order_poly(c_sf_z2, sf_rr_t2);
            FL_foot_pos[2] = tar_init_FL_foot_pos[2] + fifth_order_poly(c_sf_z2, sf_rr_t2);
            RR_foot_vel[2] = fifth_order_poly_dot(c_sf_z2, sf_rr_t2);

            if (ft_cnt == ft_step_cnt - 1) {
                init_flag = false;
            }
        }

        FL_foot_vel = RR_foot_vel;
        FR_foot_vel = RL_foot_vel;


        if (ft_cnt == ft_step_cnt - 1) {

            pre_com_pos[0] = pre_com_pos[0] + pre_x_moving_speed*tf;
            pre_com_pos[1] = com_pos[1];
            pre_com_pos[2] = h_2;

            pre_com_vel[0] = com_vel[0];
            pre_com_vel[1] = com_vel[1];
            pre_com_vel[2] = v_2;

            pre_FL_foot_pos[0] = pre_FL_foot_pos[0] + (tar_rr_foot_pos - pre_RR_foot_pos[0]); // + tar_rr_foot_pos;
            pre_RR_foot_pos[0] = pre_RR_foot_pos[0] + (tar_rr_foot_pos - pre_RR_foot_pos[0]); //pre_RR_foot_pos[0] + tar_rr_foot_pos;

            FT_COM_X_Traj_Gen();
        }
    } else if (ft_cnt < ft_step_cnt + ts_cnt) {
        //        cout << "=========== ft_phase = 3 ===========" << endl;
        ft_phase = 3;
        _c << 0, 1, 1, 0;
        contact_num = 2;
        com_t = ft_time - ft_step_time;
        sf_rl_t = sf_rl_t + dt;

        com_pos[0] = fifth_order_poly(c_com_x1, com_t);
        com_pos[1] = pre_com_pos(1);
        com_pos[2] = fifth_order_poly(c_com_z3, com_t);

        com_vel[0] = fifth_order_poly_dot(c_com_x1, com_t);
        com_vel[1] = pre_com_vel[1];
        com_vel[2] = fifth_order_poly_dot(c_com_z3, com_t);

        com_acc[2] = fifth_order_poly_2dot(c_com_z3, com_t);

        RL_foot_pos[0] = pre_RL_foot_pos[0] + (tar_rl_foot_pos - pre_RL_foot_pos[0]) / 2.0 * (1 - cos(PI2 / (2 * (ft_step_time + tf)) * sf_rl_t));
        FR_foot_pos[0] = pre_FR_foot_pos[0] + (tar_rl_foot_pos - pre_RL_foot_pos[0]) / 2.0 * (1 - cos(PI2 / (2 * (ft_step_time + tf)) * sf_rl_t));
        RL_foot_vel[0] = (tar_rl_foot_pos - pre_RL_foot_pos[0]) / 2.0 * PI2 / (2 * (ft_step_time + tf))*(sin(PI2 / (2 * (ft_step_time + tf)) * sf_rl_t));

        RR_foot_pos[0] = pre_RR_foot_pos[0];
        FL_foot_pos[0] = pre_FL_foot_pos[0];
        RR_foot_vel[0] = tar_init_RR_foot_vel[0];

        RR_foot_pos[2] = tar_init_RR_foot_pos[2];
        FL_foot_pos[2] = tar_init_FL_foot_pos[2];
        RR_foot_vel[2] = tar_init_RR_foot_vel[2];

        if (sf_rl_t < (ft_step_time + tf) / 2.0) {
            RL_foot_pos[2] = tar_init_RL_foot_pos[2] + fifth_order_poly(c_sf_z3, sf_rl_t);
            FR_foot_pos[2] = tar_init_FR_foot_pos[2] + fifth_order_poly(c_sf_z3, sf_rl_t);
            RL_foot_vel[2] = fifth_order_poly_dot(c_sf_z3, sf_rl_t);

        } else {
            sf_rl_t2 = sf_rl_t - (ft_step_time + tf) / 2.0;
            RL_foot_pos[2] = tar_init_RL_foot_pos[2] + fifth_order_poly(c_sf_z4, sf_rl_t2);
            FR_foot_pos[2] = tar_init_FR_foot_pos[2] + fifth_order_poly(c_sf_z4, sf_rl_t2);
            RL_foot_vel[2] = fifth_order_poly_dot(c_sf_z4, sf_rl_t2);
        }

        FL_foot_vel = RR_foot_vel;
        FR_foot_vel = RL_foot_vel;


        if (ft_cnt == ft_step_cnt + ts_cnt - 1) {
            pre_com_pos[0] = pre_com_pos[0] + pre_x_moving_speed * (ts) / 2.0 + x_moving_speed * (ts) / 2.0;
            pre_com_pos[1] = com_pos(1);
            pre_com_pos[2] = h_1;

            pre_com_vel[0] = x_moving_speed;
            pre_com_vel[1] = com_vel[1];
            pre_com_vel[2] = v_1;

            sf_rr_t = 0;

            pre_x_moving_speed = x_moving_speed;
            x_moving_speed = tmp_x_moving_speed;

            //            tar_rr_foot_pos = pre_x_moving_speed*(ts + tf) + x_moving_speed*(ts + tf);
            tar_rr_foot_pos = pre_com_pos[0] - 0.35 + pre_x_moving_speed * tf + (pre_x_moving_speed + x_moving_speed) * ts / 2.0 + x_moving_speed * tf + x_moving_speed * ts / 2.0;
        }
    } else if (ft_cnt < 2 * ft_step_cnt) {
        //        cout << "=========== ft_phase = 4 ===========" << endl;
        ft_phase = 4;
        //        _c << 0, 1, 1, 0;
        //		contact_num = 2;
        _c << 0, 0, 0, 0;
        contact_num = 0;
        com_t = ft_time - ft_step_time - ts;
        sf_rl_t = sf_rl_t + dt;
        sf_rr_t = sf_rr_t + dt;

        com_pos[0] = pre_com_pos(0) + pre_x_moving_speed*com_t;
        com_pos[1] = pre_com_pos(1);
        com_pos[2] = fifth_order_poly(c_com_z2, com_t);

        com_vel[0] = pre_x_moving_speed;
        com_vel[1] = pre_com_vel[1];
        com_vel[2] = fifth_order_poly_dot(c_com_z2, com_t);

        com_acc[2] = fifth_order_poly_2dot(c_com_z2, com_t);

        RL_foot_pos[0] = pre_RL_foot_pos[0] + (tar_rl_foot_pos - pre_RL_foot_pos[0]) / 2.0 * (1 - cos(PI2 / (2 * (ft_step_time + tf)) * sf_rl_t));
        FR_foot_pos[0] = pre_FR_foot_pos[0] + (tar_rl_foot_pos - pre_RL_foot_pos[0]) / 2.0 * (1 - cos(PI2 / (2 * (ft_step_time + tf)) * sf_rl_t));
        RL_foot_vel[0] = (tar_rl_foot_pos - pre_RL_foot_pos[0]) / 2.0 * PI2 / (2 * (ft_step_time + tf))*(sin(PI2 / (2 * (ft_step_time + tf)) * sf_rl_t));

        sf_rl_t2 = sf_rl_t - (ft_step_time + tf) / 2.0;
        RL_foot_pos[2] = tar_init_RL_foot_pos[2] + fifth_order_poly(c_sf_z4, sf_rl_t2);
        FR_foot_pos[2] = tar_init_FR_foot_pos[2] + fifth_order_poly(c_sf_z4, sf_rl_t2);
        RL_foot_vel[2] = fifth_order_poly_dot(c_sf_z4, sf_rl_t2);

        RR_foot_pos[0] = pre_RR_foot_pos[0] + (tar_rr_foot_pos - pre_RR_foot_pos[0]) / 2.0 * (1 - cos(PI2 / (2 * (ft_step_time + tf)) * sf_rr_t));
        FL_foot_pos[0] = pre_FL_foot_pos[0] + (tar_rr_foot_pos - pre_RR_foot_pos[0]) / 2.0 * (1 - cos(PI2 / (2 * (ft_step_time + tf)) * sf_rr_t));
        RR_foot_vel[0] = (tar_rr_foot_pos - pre_RR_foot_pos[0]) / 2.0 * PI2 / (2 * (ft_step_time + tf))*(sin(PI2 / (2 * (ft_step_time + tf)) * sf_rr_t));

        RR_foot_pos[2] = tar_init_RR_foot_pos[2] + fifth_order_poly(c_sf_z3, sf_rr_t);
        FL_foot_pos[2] = tar_init_FL_foot_pos[2] + fifth_order_poly(c_sf_z3, sf_rr_t);
        RR_foot_vel[2] = fifth_order_poly_dot(c_sf_z3, sf_rr_t);

        FL_foot_vel = RR_foot_vel;
        FR_foot_vel = RL_foot_vel;

        if (ft_cnt == 2 * ft_step_cnt - 1) {

            pre_com_pos(0) = pre_com_pos(0) + pre_x_moving_speed*tf;
            pre_com_pos(1) = com_pos(1);
            pre_com_pos(2) = h_2;

            pre_com_vel[0] = pre_x_moving_speed;
            pre_com_vel[1] = com_vel[1];
            pre_com_vel[2] = v_2;

            pre_FR_foot_pos[0] = pre_FR_foot_pos[0] + (tar_rl_foot_pos - pre_RL_foot_pos[0]); //tar_rl_foot_pos;
            pre_RL_foot_pos[0] = pre_RL_foot_pos[0] + (tar_rl_foot_pos - pre_RL_foot_pos[0]); //pre_RL_foot_pos[0] + tar_rl_foot_pos;

            FT_COM_X_Traj_Gen();
        }
    } else if (ft_cnt < 2 * ft_step_cnt + ts_cnt) {
        //        cout << "=========== ft_phase = 5 ===========" << endl;
        ft_phase = 5;
        _c << 1, 0, 0, 1;
        contact_num = 2;
        com_t = ft_time - 2 * ft_step_time;
        sf_rr_t = sf_rr_t + dt;

        com_pos[0] = fifth_order_poly(c_com_x1, com_t);
        com_pos[1] = pre_com_pos(1);
        com_pos[2] = fifth_order_poly(c_com_z3, com_t);

        com_vel[0] = fifth_order_poly_dot(c_com_x1, com_t);
        com_vel[1] = pre_com_vel[1];
        com_vel[2] = fifth_order_poly_dot(c_com_z3, com_t);

        com_acc[2] = fifth_order_poly_2dot(c_com_z3, com_t);

        RL_foot_pos[0] = pre_RL_foot_pos[0];
        FR_foot_pos[0] = pre_FR_foot_pos[0];
        RL_foot_vel[0] = tar_init_RL_foot_vel[0];

        RL_foot_pos[2] = tar_init_RL_foot_pos[2];
        FR_foot_pos[2] = tar_init_RL_foot_pos[2];
        RL_foot_vel[2] = tar_init_RL_foot_vel[2];

        RR_foot_pos[0] = pre_RR_foot_pos[0] + (tar_rr_foot_pos - pre_RR_foot_pos[0]) / 2.0 * (1 - cos(PI2 / (2 * (ft_step_time + tf)) * sf_rr_t));
        FL_foot_pos[0] = pre_FL_foot_pos[0] + (tar_rr_foot_pos - pre_RR_foot_pos[0]) / 2.0 * (1 - cos(PI2 / (2 * (ft_step_time + tf)) * sf_rr_t));
        RR_foot_vel[0] = (tar_rr_foot_pos - pre_RR_foot_pos[0]) / 2.0 * PI2 / (2 * (ft_step_time + tf))*(sin(PI2 / (2 * (ft_step_time + tf)) * sf_rr_t));

        if (sf_rr_t < (ft_step_time + tf) / 2.0) {
            RR_foot_pos[2] = tar_init_RR_foot_pos[2] + fifth_order_poly(c_sf_z3, sf_rr_t);
            FL_foot_pos[2] = tar_init_FL_foot_pos[2] + fifth_order_poly(c_sf_z3, sf_rr_t);
            RR_foot_vel[2] = fifth_order_poly_dot(c_sf_z3, sf_rr_t);

        } else {
            sf_rr_t2 = sf_rr_t - (ft_step_time + tf) / 2.0;
            RR_foot_pos[2] = tar_init_RR_foot_pos[2] + fifth_order_poly(c_sf_z4, sf_rr_t2);
            FL_foot_pos[2] = tar_init_FL_foot_pos[2] + fifth_order_poly(c_sf_z4, sf_rr_t2);
            RR_foot_vel[2] = fifth_order_poly_dot(c_sf_z4, sf_rr_t2);
        }

        FL_foot_vel = RR_foot_vel;
        FR_foot_vel = RL_foot_vel;

        if (ft_cnt == 2 * ft_step_cnt + ts_cnt - 1) {
            //            cout << "test!!!!!!!!!!!!!!!!!!!!!!!!! " << endl;

            pre_com_pos[0] = pre_com_pos[0] + pre_x_moving_speed * ts / 2.0 + x_moving_speed * ts / 2.0;
            pre_com_pos[1] = com_pos(1);
            pre_com_pos[2] = h_1;

            pre_com_vel[0] = x_moving_speed;
            pre_com_vel[1] = com_vel[1];
            pre_com_vel[2] = v_1;

            sf_rl_t = 0;

            if (move_stop_flag == false) {
                ft_cnt = ts_cnt - 1;

                pre_x_moving_speed = x_moving_speed;
                x_moving_speed = tmp_x_moving_speed;

            } else {
                pre_x_moving_speed = x_moving_speed;
                x_moving_speed = 0;
            }

            tar_rl_foot_pos = pre_com_pos[0] - 0.35 + pre_x_moving_speed * tf + (pre_x_moving_speed + x_moving_speed) * ts / 2.0 + x_moving_speed * tf + x_moving_speed * ts / 2.0;
        }
    } else if (ft_cnt < 3 * ft_step_cnt) {
        //        cout << "=========== ft_phase = 6 (Final flight phase) ===========" << endl;
        ft_phase = 6;
        //        _c << 1, 0, 0, 1;
        //		contact_num = 2;
        _c << 0, 0, 0, 0;
        contact_num = 0;
        com_t = ft_time - 2 * ft_step_time - ts;
        sf_rl_t = sf_rl_t + dt;
        sf_rr_t = sf_rr_t + dt;

        com_pos[0] = pre_com_pos[0] + pre_x_moving_speed*com_t;
        com_pos[2] = fifth_order_poly(c_com_z2, com_t);

        com_vel[0] = pre_x_moving_speed;
        com_vel[2] = fifth_order_poly_dot(c_com_z2, com_t);

        com_acc[2] = fifth_order_poly_2dot(c_com_z2, com_t);

        RL_foot_pos[0] = pre_RL_foot_pos[0] + (tar_rl_foot_pos - pre_RL_foot_pos[0]) / 2.0 * (1 - cos(PI2 / (2 * (ft_step_time)) * sf_rl_t));
        FR_foot_pos[0] = pre_FR_foot_pos[0] + (tar_rl_foot_pos - pre_RL_foot_pos[0]) / 2.0 * (1 - cos(PI2 / (2 * (ft_step_time)) * sf_rl_t));
        RL_foot_vel[0] = (tar_rl_foot_pos - pre_RL_foot_pos[0]) / 2.0 * PI2 / (2 * (ft_step_time))*(sin(PI2 / (2 * (ft_step_time)) * sf_rl_t));

        RL_foot_pos[2] = tar_init_RL_foot_pos[2] + fifth_order_poly(c_sf_z1, sf_rl_t);
        FR_foot_pos[2] = tar_init_FR_foot_pos[2] + fifth_order_poly(c_sf_z1, sf_rl_t);
        RL_foot_vel[2] = fifth_order_poly_dot(c_sf_z1, sf_rl_t);

        RR_foot_pos[0] = pre_RR_foot_pos[0] + (tar_rr_foot_pos - pre_RR_foot_pos[0]) / 2.0 * (1 - cos(PI2 / (2 * (ft_step_time + tf)) * sf_rr_t));
        FL_foot_pos[0] = pre_FL_foot_pos[0] + (tar_rr_foot_pos - pre_RR_foot_pos[0]) / 2.0 * (1 - cos(PI2 / (2 * (ft_step_time + tf)) * sf_rr_t));
        RR_foot_vel[0] = (tar_rr_foot_pos - pre_RR_foot_pos[0]) / 2.0 * PI2 / (2 * (ft_step_time + tf))*(sin(PI2 / (2 * (ft_step_time + tf)) * sf_rr_t));

        sf_rr_t2 = sf_rr_t - (ft_step_time + tf) / 2.0;
        RR_foot_pos[2] = tar_init_RR_foot_pos[2] + fifth_order_poly(c_sf_z4, sf_rr_t2);
        FL_foot_pos[2] = tar_init_FL_foot_pos[2] + fifth_order_poly(c_sf_z4, sf_rr_t2);
        RR_foot_vel[2] = fifth_order_poly_dot(c_sf_z4, sf_rr_t2);

        FL_foot_vel = RR_foot_vel;
        FR_foot_vel = RL_foot_vel;

        if (ft_cnt == 3 * ft_step_cnt - 1) {

            pre_com_pos[0] = pre_com_pos[0] + pre_x_moving_speed*tf;
            pre_com_pos[1] = com_pos[1];
            pre_com_pos[2] = h_2;

            pre_com_vel[0] = com_vel[0];
            pre_com_vel[1] = com_vel[1];
            pre_com_vel[2] = v_2;

            pre_FL_foot_pos[0] = pre_FL_foot_pos[0] + (tar_rr_foot_pos - pre_RR_foot_pos[0]);
            pre_RR_foot_pos[0] = pre_RR_foot_pos[0] + (tar_rr_foot_pos - pre_RR_foot_pos[0]);

            FT_COM_X_Traj_Gen();
        }
    } else if (ft_cnt < 3 * ft_step_cnt + ts_cnt) {
        //        cout << "=========== ft_phase = 7 (Final stance phase) ===========" << endl;
        ft_phase = 7;
        _c << 0, 1, 1, 0;
        contact_num = 2;
        com_t = ft_time - 3 * ft_step_time;
        sf_rl_t = sf_rl_t + dt;

        com_pos[0] = fifth_order_poly(c_com_x1, com_t);
        com_pos[1] = pre_com_pos(1);
        com_pos[2] = fifth_order_poly(c_com_z4, com_t);

        com_vel[0] = fifth_order_poly_dot(c_com_x1, com_t);
        com_vel[1] = pre_com_vel[1];
        com_vel[2] = fifth_order_poly_dot(c_com_z4, com_t);

        com_acc[2] = fifth_order_poly_2dot(c_com_z4, com_t);

        RL_foot_pos[0] = pre_RL_foot_pos[0] + (tar_rl_foot_pos - pre_RL_foot_pos[0]) / 2.0 * (1 - cos(PI2 / (2 * (ft_step_time)) * sf_rl_t));
        FR_foot_pos[0] = pre_FR_foot_pos[0] + (tar_rl_foot_pos - pre_RL_foot_pos[0]) / 2.0 * (1 - cos(PI2 / (2 * (ft_step_time)) * sf_rl_t));
        RL_foot_vel[0] = (tar_rl_foot_pos - pre_RL_foot_pos[0]) / 2.0 * PI2 / (2 * (ft_step_time))*(sin(PI2 / (2 * (ft_step_time)) * sf_rl_t));

        if (sf_rl_t < (ft_step_time) / 2.0) {
            RL_foot_pos[2] = tar_init_RL_foot_pos[2] + fifth_order_poly(c_sf_z1, sf_rl_t);
            FR_foot_pos[2] = tar_init_FR_foot_pos[2] + fifth_order_poly(c_sf_z1, sf_rl_t);
            RL_foot_vel[2] = fifth_order_poly_dot(c_sf_z1, sf_rl_t);

        } else {
            sf_rl_t2 = sf_rl_t - (ft_step_time) / 2.0;
            RL_foot_pos[2] = tar_init_RL_foot_pos[2] + fifth_order_poly(c_sf_z2, sf_rl_t2);
            FR_foot_pos[2] = tar_init_FR_foot_pos[2] + fifth_order_poly(c_sf_z2, sf_rl_t2);
            RL_foot_vel[2] = fifth_order_poly_dot(c_sf_z2, sf_rl_t2);
        }

        RR_foot_pos[0] = pre_RR_foot_pos[0];
        FL_foot_pos[0] = pre_FL_foot_pos[0];
        RR_foot_vel[0] = tar_init_RR_foot_vel[0];

        RR_foot_pos[2] = tar_init_RR_foot_pos[2];
        FL_foot_pos[2] = tar_init_FL_foot_pos[2];
        RR_foot_vel[2] = tar_init_RR_foot_vel[2];

        FL_foot_vel = RR_foot_vel;
        FR_foot_vel = RL_foot_vel;

        if (ft_cnt == 3 * ft_step_cnt + ts_cnt - 1) {
            pre_com_pos[0] = pre_com_pos[0] + pre_x_moving_speed * (ts) / 2.0 + x_moving_speed * (ts) / 2.0; //pre_com_pos[0] + x_moving_speed * (ts) / 2.0;
            pre_com_pos[1] = com_pos(1);
            pre_com_pos[2] = h_0;

            pre_com_vel[0] = 0;
            pre_com_vel[1] = com_vel[1];
            pre_com_vel[2] = v_0;

            pre_FR_foot_pos[0] = pre_FR_foot_pos[0] + (tar_rl_foot_pos - pre_RL_foot_pos[0]);
            pre_RL_foot_pos[0] = pre_RL_foot_pos[0] + (tar_rl_foot_pos - pre_RL_foot_pos[0]);

            cout << "com_pos = " << pre_com_pos.transpose() << endl;
            cout << "com_vel = " << pre_com_vel.transpose() << endl;
            cout << "RL_foot_pos = " << pre_RL_foot_pos.transpose() << endl;
            cout << "RR_foot_pos = " << pre_RR_foot_pos.transpose() << endl;
            cout << "FL_foot_pos = " << pre_FL_foot_pos.transpose() << endl;
            cout << "FR_foot_pos = " << pre_FR_foot_pos.transpose() << endl;
        }
    } else {
        //        cout << "=========== ft_phase = 8 (Motion Done !!) ===========" << endl;
        ft_phase = 8;
        _c << 1, 1, 1, 1;
        contact_num = 4;
        moving_done_flag = true;

        com_pos = pre_com_pos;
        com_vel = tar_init_com_vel;
        com_acc = tar_init_com_acc;

        RL_foot_pos = pre_RL_foot_pos;
        RR_foot_pos = pre_RR_foot_pos;
        FL_foot_pos = pre_FL_foot_pos;
        FR_foot_pos = pre_FR_foot_pos;

        RL_foot_vel = tar_init_RL_foot_vel;
        RR_foot_vel = tar_init_RR_foot_vel;
        FL_foot_vel = tar_init_FL_foot_vel;
        FR_foot_vel = tar_init_FR_foot_vel;
    }

    FT_Turning_Traj_Gen2();

    //    base_pos = com_pos + base_offset;
    //    if (Slope_con_onoff_flag == true) Slope_compensation_con();

    base_pos = com_pos + tmp_com_pos + base_offset;
    target_pos[6] = 0;
    ft_cnt++;
}

void CRobot::FT_COM_X_Traj_Gen() {
    // =============== COM =============== //
    init_x[0] = pre_com_pos[0];
    init_x[1] = pre_x_moving_speed;
    init_x[2] = 0;

    final_x[0] = pre_com_pos[0] + pre_x_moving_speed * (ts) / 2.0 + x_moving_speed * (ts) / 2.0;
    final_x[1] = x_moving_speed;
    final_x[2] = 0;

    coefficient_5thPoly(init_x, final_x, ts, c_com_x1);
}

//void CRobot::FT_COM_SF_X_Traj_Gen()
//{
//    // =============== COM =============== //
//    init_x[0] = pre_com_pos[0];
//    init_x[1] = pre_x_moving_speed;
//    init_x[2] = 0;
//
//    final_x[0] = pre_com_pos[0] + pre_x_moving_speed * (ts) / 2.0 + x_moving_speed * (ts) / 2.0;
//    final_x[1] = x_moving_speed;
//    final_x[2] = 0;
//
//    coefficient_5thPoly(init_x, final_x, ts, c_com_x1);
//
//    // ============ Swing Foot Pos. ============ //
//    // Left (First)
//    init_x[0] = 0;
//    init_x[1] = 0;
//    init_x[2] = 0;
//
//    final_x[0] = 0 + pre_x_moving_speed * ft_step_time + x_moving_speed * ft_step_time;
//    final_x[1] = 0;
//    final_x[2] = 0;
//
//    coefficient_5thPoly(init_x, final_x, ts + tf, c_sf_x1);
//
//    // Right (First)
//    init_x[0] = 0;
//    init_x[1] = 0;
//    init_x[2] = 0;
//
//    final_x[0] = 0 + x_moving_speed * ft_step_time * 2;
//    final_x[1] = 0;
//    final_x[2] = 0;
//
//    coefficient_5thPoly(init_x, final_x, ts + tf, c_sf_x2);
//
//    // Left (Second)
//    init_x[0] = 0 + pre_x_moving_speed * ft_step_time + x_moving_speed * ft_step_time;
//    init_x[1] = 0;
//    init_x[2] = 0;
//
//    final_x[0] = 0 + pre_x_moving_speed * ft_step_time + x_moving_speed * ft_step_time * 3;
//    final_x[1] = 0;
//    final_x[2] = 0;
//
//    coefficient_5thPoly(init_x, final_x, ts, c_sf_x3);
//
//    // Right (Second)
//    init_x[0] = 0 + x_moving_speed * ft_step_time * 2;
//    init_x[1] = 0;
//    init_x[2] = 0;
//
//    final_x[0] = 0 + x_moving_speed * ft_step_time * 4;
//    final_x[1] = 0;
//    final_x[2] = 0;
//
//    coefficient_5thPoly(init_x, final_x, ts, c_sf_x4);
//
//}

void CRobot::FT_COM_SF_Z_Traj_Gen() {
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

    coefficient_5thPoly(init_x, final_x, (ts + tf) / 2.0, c_sf_z1);

    init_x[0] = swing_foot_height;
    init_x[1] = 0;
    init_x[2] = 0;

    final_x[0] = 0;
    final_x[1] = 0;
    final_x[2] = 0;

    coefficient_5thPoly(init_x, final_x, (ts + tf) / 2.0, c_sf_z2);

    init_x[0] = 0;
    init_x[1] = 0;
    init_x[2] = 0;

    final_x[0] = swing_foot_height;
    final_x[1] = 0;
    final_x[2] = 0;

    coefficient_5thPoly(init_x, final_x, (ts + 2 * tf) / 2.0, c_sf_z3);

    init_x[0] = swing_foot_height;
    init_x[1] = 0;
    init_x[2] = 0;

    final_x[0] = 0;
    final_x[1] = 0;
    final_x[2] = 0;

    coefficient_5thPoly(init_x, final_x, (ts + 2 * tf) / 2.0, c_sf_z4);

}

void CRobot::FT_Turning_Traj_Gen2(void) {
    const double turn_l = 0.4134; // sqrt(0.35^2 + (0.115 + 0.105)^2)
    const double x1 = 0.35, y1 = 0.22, turn_theta1 = 57.8477 * D2R;
    static double x2 = 0.35, y2 = 0.22, turn_theta2 = 0;
    static double del_x = 0, del_y = 0;
    static double target_theta = turn_theta1;
    static double tmp_target_theta = 0;

    // turn left
    if ((base_ori(2) > 0.01) && (ft_cnt == 2 * ft_step_cnt)) {
        if (turn_mode == 0) {
            turn_start_flag = true;
            turn_mode = 1;

            target_theta = -base_ori(2) * 2.0; // [rad]

            printf("[left] target_theta = %f\n", target_theta);
        }
    }// turn right
    else if ((base_ori(2) < -0.01) && (ft_cnt == ft_step_cnt)) {
        if (turn_mode == 0) {
            turn_start_flag = true;
            turn_mode = 3;

            target_theta = base_ori(2) * 2.0; // [rad]

            printf("[right] target_theta = %f\n", target_theta);
        }
    }

    if (turn_start_flag == true) {
        if (turn_mode == 1) { // turn left
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
            } else {
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

            if (turn_cnt == ft_step_cnt - 1) {
                turn_mode = 2;
                turn_cnt = 0;
            }
        } else if (turn_mode == 2) {
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
            } else {
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
        } else if (turn_mode == 3) { // turn right
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
            } else {
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

            if (turn_cnt == ft_step_cnt - 1) {
                turn_mode = 4;
                turn_cnt = 0;
            }
        } else if (turn_mode == 4) {
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
            } else {
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

            if (turn_cnt == ft_step_cnt - 1) { // FC_PHASE == STOP
                // Initialization
                turn_mode = 0;
                turn_cnt = 0;
                turn_start_flag = false;
            }
        }
    } else {
        turn_mode = 0;
        turn_cnt = 0;

        turn_xl_EP = 0;
        turn_yl_EP = 0;
        turn_xr_EP = 0;
        turn_yr_EP = 0;
    }

    RL_foot_pos_local_offset << -turn_xr_EP, -turn_yr_EP, 0;
    RR_foot_pos_local_offset << -turn_xl_EP, -turn_yl_EP, 0;
    FL_foot_pos_local_offset << +turn_xl_EP, +turn_yl_EP, 0;
    FR_foot_pos_local_offset << +turn_xr_EP, +turn_yr_EP, 0;

    // ======= Turning trajectory generation  END ======= //
}

double CRobot::fifth_order_poly(double c[], double t) {
    static double y = 0;
    y = c[5] * pow(t, 5) + c[4] * pow(t, 4) + c[3] * pow(t, 3) + c[2] * pow(t, 2) + c[1] * pow(t, 1) + c[0];
    return y;
}

double CRobot::fifth_order_poly_dot(double c[], double t) {
    static double y = 0;
    y = 5 * c[5] * pow(t, 4) + 4 * c[4] * pow(t, 3) + 3 * c[3] * pow(t, 2) + 2 * c[2] * pow(t, 1) + 1 * c[1];
    return y;
}

double CRobot::fifth_order_poly_2dot(double c[], double t) {
    static double y = 0;
    y = 20 * c[5] * pow(t, 3) + 12 * c[4] * pow(t, 2) + 6 * c[3] * pow(t, 1) + 2 * c[2];
    return y;
}

VectorNd CRobot::Get_COM(VectorNd base, VectorNd q) {
    const double m_body = 24.333;
    const double m_hp = 1.4;
    const double m_thigh = 3.209;
    const double m_calf = 0.634;
    const double m_leg = m_hp + m_thigh + m_calf;
    const double m_robot = m_leg * 4 + m_body;

    // Transformation & Rotation matrix (Base)
    R_w2base_R << 1, 0, 0, 0,
            0, cos(base(3)), -sin(base(3)), 0,
            0, sin(base(3)), cos(base(3)), 0,
            0, 0, 0, 1;
    R_w2base_P << cos(base(4)), 0, sin(base(4)), 0,
            0, 1, 0, 0,
            -sin(base(4)), 0, cos(base(4)), 0,
            0, 0, 0, 1;
    R_w2base_Y << cos(base(5)), -sin(base(5)), 0, 0,
            sin(base(5)), cos(base(5)), 0, 0,
            0, 0, 1, 0,
            0, 0, 0, 1;

    R_w2base = R_w2base_Y * R_w2base_P*R_w2base_R;
    T_w2base << 1, 0, 0, base(0),
            0, 1, 0, base(1),
            0, 0, 1, base(2),
            0, 0, 0, 1;

    // Transformation & Rotation matrix (Leg)
    // RL
    TR_RL_base2hp << 1, 0, 0, -0.35,
            0, cos(q(0)), -sin(q(0)), 0.115,
            0, sin(q(0)), cos(q(0)), -0.053,
            0, 0, 0, 1;

    TR_RL_hp2thigh << cos(q(1)), 0, sin(q(1)), 0.0,
            0, 1, 0, 0.105,
            -sin(q(1)), 0, cos(q(1)), 0.0,
            0, 0, 0, 1;

    TR_RL_thigh2calf << cos(q(2)), 0, sin(q(2)), 0.0,
            0, 1, 0, 0.0,
            -sin(q(2)), 0, cos(q(2)), -0.305,
            0, 0, 0, 1;

    p_RL_base2hp_com = TR_RL_base2hp*p_RL_hp_com;
    p_RL_base2thigh_com = TR_RL_base2hp * TR_RL_hp2thigh*p_RL_thigh_com;
    p_RL_base2calf_com = TR_RL_base2hp * TR_RL_hp2thigh * TR_RL_thigh2calf*p_RL_calf_com;

    p_RL_com = (m_hp * p_RL_base2hp_com + m_thigh * p_RL_base2thigh_com + m_calf * p_RL_base2calf_com) / (m_leg);

    //    cout << "p_RL_com = " << p_RL_com << endl;

    // RR
    TR_RR_base2hp << 1, 0, 0, -0.35,
            0, cos(q(3)), -sin(q(3)), -0.115,
            0, sin(q(3)), cos(q(3)), -0.053,
            0, 0, 0, 1;

    TR_RR_hp2thigh << cos(q(4)), 0, sin(q(4)), 0.0,
            0, 1, 0, -0.105,
            -sin(q(4)), 0, cos(q(4)), 0.0,
            0, 0, 0, 1;

    TR_RR_thigh2calf << cos(q(5)), 0, sin(q(5)), 0.0,
            0, 1, 0, 0.0,
            -sin(q(5)), 0, cos(q(5)), -0.305,
            0, 0, 0, 1;

    p_RR_base2hp_com = TR_RR_base2hp*p_RR_hp_com;
    p_RR_base2thigh_com = TR_RR_base2hp * TR_RR_hp2thigh*p_RR_thigh_com;
    p_RR_base2calf_com = TR_RR_base2hp * TR_RR_hp2thigh * TR_RR_thigh2calf*p_RR_calf_com;

    p_RR_com = (m_hp * p_RR_base2hp_com + m_thigh * p_RR_base2thigh_com + m_calf * p_RR_base2calf_com) / (m_leg);

    //    cout << "p_RR_com = " << p_RR_com << endl;

    // FL
    TR_FL_base2hp << 1, 0, 0, 0.35,
            0, cos(q(0)), -sin(q(0)), 0.115,
            0, sin(q(0)), cos(q(0)), -0.053,
            0, 0, 0, 1;

    TR_FL_hp2thigh << cos(q(1)), 0, sin(q(1)), 0.0,
            0, 1, 0, 0.105,
            -sin(q(1)), 0, cos(q(1)), 0.0,
            0, 0, 0, 1;

    TR_FL_thigh2calf << cos(q(2)), 0, sin(q(2)), 0.0,
            0, 1, 0, 0.0,
            -sin(q(2)), 0, cos(q(2)), -0.305,
            0, 0, 0, 1;


    p_FL_base2hp_com = TR_FL_base2hp*p_FL_hp_com;
    p_FL_base2thigh_com = TR_FL_base2hp * TR_FL_hp2thigh*p_FL_thigh_com;
    p_FL_base2calf_com = TR_FL_base2hp * TR_FL_hp2thigh * TR_FL_thigh2calf*p_FL_calf_com;

    p_FL_com = (m_hp * p_FL_base2hp_com + m_thigh * p_FL_base2thigh_com + m_calf * p_FL_base2calf_com) / (m_leg);

    //    cout << "p_FL_com = " << p_FL_com << endl;

    // FR
    TR_FR_base2hp << 1, 0, 0, 0.35,
            0, cos(q(3)), -sin(q(3)), -0.115,
            0, sin(q(3)), cos(q(3)), -0.053,
            0, 0, 0, 1;

    TR_FR_hp2thigh << cos(q(4)), 0, sin(q(4)), 0.0,
            0, 1, 0, -0.105,
            -sin(q(4)), 0, cos(q(4)), 0.0,
            0, 0, 0, 1;

    TR_FR_thigh2calf << cos(q(5)), 0, sin(q(5)), 0.0,
            0, 1, 0, 0.0,
            -sin(q(5)), 0, cos(q(5)), -0.305,
            0, 0, 0, 1;

    p_FR_base2hp_com = TR_FR_base2hp*p_FR_hp_com;
    p_FR_base2thigh_com = TR_FR_base2hp * TR_FR_hp2thigh*p_FR_thigh_com;
    p_FR_base2calf_com = TR_FR_base2hp * TR_FR_hp2thigh * TR_FR_thigh2calf*p_FR_calf_com;

    p_FR_com = (m_hp * p_FR_base2hp_com + m_thigh * p_FR_base2thigh_com + m_calf * p_FR_base2calf_com) / (m_leg);

    //    cout << "p_FR_com = " << p_FR_com << endl;

    // COM from base
    p_robot_com_from_base = (m_body * p_base2body_com + m_leg * (p_RL_com + p_RR_com + p_FL_com + p_FR_com)) / (m_robot);
    p_robot_com_from_w = T_w2base * R_w2base*p_robot_com_from_base;
    p_robot_com = p_robot_com_from_w.block<3, 1>(0, 0);

    cout << "p_robot_com = " << p_robot_com << endl;

    return p_robot_com;
}

void CRobot::FTsensorTransformation() {
    C_I_roll << 1, 0, 0, 0, cos(base.currentRoll), -sin(base.currentRoll), 0, sin(base.currentRoll), cos(base.currentRoll);

    C_I_pitch << cos(base.currentPitch), 0, sin(base.currentPitch), 0, 1, 0, -sin(base.currentPitch), 0, cos(base.currentPitch);

    RL_C_I_HP << 1, 0, 0, 0, cos(actual_pos[0]), -sin(actual_pos[0]), 0, sin(actual_pos[0]), cos(actual_pos[0]);
    RL_C_HP_HR << cos(actual_pos[1]), 0, sin(actual_pos[1]), 0, 1, 0, -sin(actual_pos[1]), 0, cos(actual_pos[1]);
    RL_C_HR_KN << cos(actual_pos[2]), 0, sin(actual_pos[2]), 0, 1, 0, -sin(actual_pos[2]), 0, cos(actual_pos[2]);
    RL_C_KN_TIP << cos(45 * D2R), 0, sin(45 * D2R), 0, 1, 0, -sin(45 * D2R), 0, cos(45 * D2R);
    RL.T_matrix = C_I_roll * C_I_pitch * RL_C_I_HP * RL_C_HP_HR * RL_C_HR_KN * RL_C_KN_TIP;

    RR_C_I_HP << 1, 0, 0, 0, cos(actual_pos[3]), -sin(actual_pos[3]), 0, sin(actual_pos[3]), cos(actual_pos[3]);
    RR_C_HP_HR << cos(actual_pos[4]), 0, sin(actual_pos[4]), 0, 1, 0, -sin(actual_pos[4]), 0, cos(actual_pos[4]);
    RR_C_HR_KN << cos(actual_pos[5]), 0, sin(actual_pos[5]), 0, 1, 0, -sin(actual_pos[5]), 0, cos(actual_pos[5]);
    RR_C_KN_TIP << cos(45 * D2R), 0, sin(45 * D2R), 0, 1, 0, -sin(45 * D2R), 0, cos(45 * D2R);
    RR.T_matrix = C_I_roll * C_I_pitch * RR_C_I_HP * RR_C_HP_HR * RR_C_HR_KN * RR_C_KN_TIP;

    FL_C_I_HP << 1, 0, 0, 0, cos(actual_pos[7]), -sin(actual_pos[7]), 0, sin(actual_pos[7]), cos(actual_pos[7]);
    FL_C_HP_HR << cos(actual_pos[8]), 0, sin(actual_pos[8]), 0, 1, 0, -sin(actual_pos[8]), 0, cos(actual_pos[8]);
    FL_C_HR_KN << cos(actual_pos[9]), 0, sin(actual_pos[9]), 0, 1, 0, -sin(actual_pos[9]), 0, cos(actual_pos[9]);
    FL_C_KN_TIP << cos(45 * D2R), 0, sin(45 * D2R), 0, 1, 0, -sin(45 * D2R), 0, cos(45 * D2R);
    FL.T_matrix = C_I_roll * C_I_pitch * FL_C_I_HP * FL_C_HP_HR * FL_C_HR_KN * FL_C_KN_TIP;

    FR_C_I_HP << 1, 0, 0, 0, cos(actual_pos[10]), -sin(actual_pos[10]), 0, sin(actual_pos[10]), cos(actual_pos[10]);
    FR_C_HP_HR << cos(actual_pos[11]), 0, sin(actual_pos[11]), 0, 1, 0, -sin(actual_pos[11]), 0, cos(actual_pos[11]);
    FR_C_HR_KN << cos(actual_pos[12]), 0, sin(actual_pos[12]), 0, 1, 0, -sin(actual_pos[12]), 0, cos(actual_pos[12]);
    FR_C_KN_TIP << cos(45 * D2R), 0, sin(45 * D2R), 0, 1, 0, -sin(45 * D2R), 0, cos(45 * D2R);
    FR.T_matrix = C_I_roll * C_I_pitch * FR_C_I_HP * FR_C_HP_HR * FR_C_HR_KN * FR_C_KN_TIP;

}

MatrixNd CRobot::pow_mat(MatrixNd mat, int x) {

    MatrixNd Out = MatrixNd::Zero(mat.rows(), mat.cols());

    //    cout << "[1] Out = " << Out << endl;

    if (x == 0) {
        Out = MatrixNd::Identity(mat.rows(), mat.cols());
    } else {
        for (int i = 0; i < x; ++i) {
            if (i == 0) {
                Out = mat;
            } else {
                Out = Out*mat;
            }
        }
    }

    //    cout << "[2] Out = " << Out << endl;

    return Out;
}

MatrixNd CRobot::Kron(MatrixNd AA, MatrixNd BB) {

    MatrixNd Out = MatrixNd::Zero(AA.rows() * BB.rows(), AA.cols() * BB.cols());
    //    cout << "AA = " << AA << endl;
    //    cout << "BB = " << BB << endl;
    //    cout << "Out(0,0) = " << Out(0,0) << endl;

    for (int i = 0; i < AA.rows(); i++) {
        for (int j = 0; j < AA.cols(); j++) {
            for (int k = 0; k < BB.rows(); k++) {
                for (int l = 0; l < BB.cols(); l++) {
                    Out(i * BB.rows() + k, j * BB.cols() + l) = AA(i, j) * BB(k, l);
                    //                    cout << "Out index = " << i+k << j+l << endl;
                    //                    cout << i << j<< k << l << endl;
                    //                    Out[0,0] = A[0,0] * B[0,0];
                }
            }
        }
    }

    //    cout << "Out = " << Out << endl;

    return Out;
}

//VectorNd CRobot::knee_out2in(double _q4)
//{
//	if(_q4 >= -0.01){
//		_q4 = -0.01;
//	}
//	return_q2q3(0) = -(PI - acos((pow(A0A,2) + pow(A0B0,2) - 2*cos(-_q4)*A0B0*B0B - pow(AB,2) + pow(B0B,2))/(2*A0A*sqrt(pow(A0B0,2) - 2*cos(-_q4)*A0B0*B0B + pow(B0B,2)))) - acos((A0B0 - B0B*cos(-_q4))/sqrt(pow(A0B0,2) - 2*cos(-_q4)*A0B0*B0B + pow(B0B,2))));
//	return_q2q3(1) = -(PI - acos((pow(A0A,2) + pow(A0B0,2) - 2*cos(-_q4)*A0B0*B0B - pow(AB,2) + pow(B0B,2))/(2*A0A*sqrt(pow(A0B0,2) - 2*cos(-_q4)*A0B0*B0B + pow(B0B,2)))) - acos((pow(A0A,2) - pow(A0B0,2) + 2*cos(-_q4)*A0B0*B0B + pow(AB,2) - pow(B0B,2))/(2*A0A*AB)) - acos((A0B0 - B0B*cos(-_q4))/sqrt(pow(A0B0,2) - 2*cos(-_q4)*A0B0*B0B + pow(B0B,2))));
//
//	return return_q2q3;
//}
//
//VectorNd CRobot::knee_in2out(double _q2)
//{
//	if(_q2 >= -1.15){
//		_q2 = -1.15;
//	}
//
//	return_q3q4(0) = -(acos((pow(A0A,2) + 2*cos(-_q2)*A0A*A0B0 + pow(A0B0,2) - pow(AB,2) + pow(B0B,2))/(2*B0B*sqrt(pow(A0A,2) + 2*cos(-_q2)*A0A*A0B0 + pow(A0B0,2)))) - acos((pow(A0A,2) + 2*cos(-_q2)*A0A*A0B0 + pow(A0B0,2) - pow(AB,2) - pow(B0B,2))/(2*AB*B0B)) + acos((A0B0 + A0A*cos(-_q2))/sqrt(pow(A0A,2) + 2*cos(-_q2)*A0A*A0B0 + pow(A0B0,2))));
//	return_q3q4(1) = -(acos((pow(A0A,2) + 2*cos(-_q2)*A0A*A0B0 + pow(A0B0,2) - pow(AB,2) + pow(B0B,2))/(2*B0B*sqrt(pow(A0A,2) + 2*cos(-_q2)*A0A*A0B0 + pow(A0B0,2)))) + acos((A0B0 + A0A*cos(-_q2))/sqrt(pow(A0A,2) + 2*cos(-_q2)*A0A*A0B0 + pow(A0B0,2))));
//
//	return return_q3q4;
//}
//
//double CRobot::q2_dot2q4_dot(double _q2_dot, double _q2, double _q3, double _q4)
//{
//	static double _q4_dot = 0;
//	static double V_A = 0, V_B = 0;
//	static double q_BA  = 0, q_A = 0, q_B = 0;
//
//	V_A = A0A*(-_q2_dot);
//
//	q_BA = PI/2 - (PI/2 + _q2 - _q4);
//	q_A = PI/2 - (PI/2 + _q4 - _q3);
//	q_B = PI - q_A - q_BA;
//
//	V_B = V_A/sin(q_A)*sin(q_B);
//
//	_q4_dot = -V_B/B0B;
//
//	return _q4_dot;
//}



//Hyun's Code

void CRobot::Global_Transform_Z_HS(void) {
    VectorNd target_EP_local_hip_HS(12);
    VectorNd actual_base_pos_local_HS(3);
    VectorNd tmp_actual_base_pos_global_HS(3);
    VectorNd tmp_target_EP_HS(12);
    VectorNd standard_leg(4);

    //offset_B2C << -0.04, 0.0, -0.0;
    offset_B2C << -0.05, 0.0, -0.0;

    actual_pos_HS = actual_pos;
    actual_EP_local_hip_HS = FK_HS(actual_pos_HS);
    actual_EP_local_HS = Localization_Hip2Base_Pos_HS(actual_EP_local_hip_HS);

    actual_base_ori_local_HS << IMURoll, IMUPitch, IMUYaw;
    actual_base_ori_vel_local_HS << IMURoll_dot, IMUPitch_dot, IMUYaw_dot;

    Base_Rotation_Matrix_HS();

    target_C_WB_YPR_12d_HS.block(0, 0, 3, 3) = target_C_WB_YPR_HS;
    target_C_WB_YPR_12d_HS.block(3, 3, 3, 3) = target_C_WB_YPR_HS;
    target_C_WB_YPR_12d_HS.block(6, 6, 3, 3) = target_C_WB_YPR_HS;
    target_C_WB_YPR_12d_HS.block(9, 9, 3, 3) = target_C_WB_YPR_HS;

    target_C_WB_PR_12d_HS.block(0, 0, 3, 3) = target_C_WB_PR_HS;
    target_C_WB_PR_12d_HS.block(3, 3, 3, 3) = target_C_WB_PR_HS;
    target_C_WB_PR_12d_HS.block(6, 6, 3, 3) = target_C_WB_PR_HS;
    target_C_WB_PR_12d_HS.block(9, 9, 3, 3) = target_C_WB_PR_HS;

    target_C_WB_Y_12d_HS.block(0, 0, 3, 3) = target_C_WB_Y_HS;
    target_C_WB_Y_12d_HS.block(3, 3, 3, 3) = target_C_WB_Y_HS;
    target_C_WB_Y_12d_HS.block(6, 6, 3, 3) = target_C_WB_Y_HS;
    target_C_WB_Y_12d_HS.block(9, 9, 3, 3) = target_C_WB_Y_HS;

    actual_C_WB_YPR_12d_HS.block(0, 0, 3, 3) = actual_C_WB_YPR_HS;
    actual_C_WB_YPR_12d_HS.block(3, 3, 3, 3) = actual_C_WB_YPR_HS;
    actual_C_WB_YPR_12d_HS.block(6, 6, 3, 3) = actual_C_WB_YPR_HS;
    actual_C_WB_YPR_12d_HS.block(9, 9, 3, 3) = actual_C_WB_YPR_HS;


    measure_z = -(Contact_Info_HS(0) * actual_EP_local_HS(2) / cos(abs(actual_base_ori_local_HS(1))) + Contact_Info_HS(1) * actual_EP_local_HS(5) / cos(abs(actual_base_ori_local_HS(1))) + Contact_Info_HS(2) * actual_EP_local_HS(8) / cos(abs(actual_base_ori_local_HS(1))) + Contact_Info_HS(3) * actual_EP_local_HS(11) / cos(abs(actual_base_ori_local_HS(1)))) / (Contact_Info_HS(0) + Contact_Info_HS(1) + Contact_Info_HS(2) + Contact_Info_HS(3));

    Base_Estimation_Test();
    //actual_base_pos_HS << target_base_pos_HS(0), target_base_pos_HS(1), x_hat(0);
    actual_base_pos_HS << target_base_pos_HS(0), target_base_pos_HS(1), measure_z;


    actual_EP_HS << actual_base_pos_HS + actual_C_WB_YPR_HS * actual_EP_local_HS.segment(0, 3), actual_base_pos_HS + actual_C_WB_YPR_HS * actual_EP_local_HS.segment(3, 3), actual_base_pos_HS + actual_C_WB_YPR_HS * actual_EP_local_HS.segment(6, 3), actual_base_pos_HS + actual_C_WB_YPR_HS * actual_EP_local_HS.segment(9, 3);
    //actual_EP_HS << actual_base_pos_HS + target_C_WB_YPR_HS * actual_EP_local_HS.segment(0, 3), actual_base_pos_HS + target_C_WB_YPR_HS * actual_EP_local_HS.segment(3, 3), actual_base_pos_HS + target_C_WB_YPR_HS * actual_EP_local_HS.segment(6, 3), actual_base_pos_HS + target_C_WB_YPR_HS * actual_EP_local_HS.segment(9, 3);
    //actual_EP_HS << target_base_pos_HS + target_C_WB_YPR_HS * actual_EP_local_HS.segment(0, 3), target_base_pos_HS + target_C_WB_YPR_HS * actual_EP_local_HS.segment(3, 3), target_base_pos_HS + target_C_WB_YPR_HS * actual_EP_local_HS.segment(6, 3), target_base_pos_HS + target_C_WB_YPR_HS * actual_EP_local_HS.segment(9, 3);
    if (initial_flag_HS == true) {
        pre_actual_EP_HS = actual_EP_HS;
    }
    actual_EP_vel_HS = (actual_EP_HS - pre_actual_EP_HS) / dt;
    pre_actual_EP_HS = actual_EP_HS;

    actual_EP_acc_HS = (actual_EP_vel_HS - pre_actual_EP_vel_HS) / dt;
    pre_actual_EP_vel_HS = actual_EP_vel_HS;

    if (initial_flag_HS == true) {
        pre_actual_base_pos_HS = actual_base_pos_HS;
    }

    actual_base_vel_HS = (actual_base_pos_HS - pre_actual_base_pos_HS) / dt;
    pre_actual_base_pos_HS = actual_base_pos_HS;

    //    if (Contact_Info_HS(0) == 1) {
    //        tmp_actual_EP_local_HS2(2) = actual_EP_local_HS(2);
    //    } else {
    //        tmp_actual_EP_local_HS2(2) = static_actual_EP_local_HS(2);
    //    }
    //
    //    if (Contact_Info_HS(1) == 1) {
    //        tmp_actual_EP_local_HS2(5) = actual_EP_local_HS(5);
    //    } else {
    //        tmp_actual_EP_local_HS2(5) = static_actual_EP_local_HS(5);
    //    }
    //
    //    if (Contact_Info_HS(2) == 1) {
    //        tmp_actual_EP_local_HS2(8) = actual_EP_local_HS(8);
    //    } else {
    //        tmp_actual_EP_local_HS2(8) = static_actual_EP_local_HS(8);
    //    }
    //
    //    if (Contact_Info_HS(3) == 1) {
    //        tmp_actual_EP_local_HS2(11) = actual_EP_local_HS(11);
    //    } else {
    //        tmp_actual_EP_local_HS2(11) = static_actual_EP_local_HS(11);
    //    }
    //    

    //  tmp_actual_EP_local_HS2 = actual_EP_local_HS;
    //standard_leg = Base_Estimation2(tmp_actual_EP_local_HS2);
    //    if (base_hold_flag == true) {
    // measure_z = -((standard_leg(0) * (tmp_actual_EP_local_HS2(2)) + standard_leg(1) * (tmp_actual_EP_local_HS2(5)) + standard_leg(2) * (tmp_actual_EP_local_HS2(8)) + standard_leg(3) * (tmp_actual_EP_local_HS2(11)))) / cos(abs(target_base_ori_local_HS(1)));
    //  } else {
    //measure_z = -(Contact_Info_HS(0) * (actual_EP_local_HS(2) / cos(abs(target_base_ori_local_HS(1)))) + Contact_Info_HS(1) * (actual_EP_local_HS(5) / cos(abs(target_base_ori_local_HS(1)))) + Contact_Info_HS(2) * (actual_EP_local_HS(8) / cos(abs(target_base_ori_local_HS(1)))) + Contact_Info_HS(3) * (actual_EP_local_HS(11) / cos(abs(target_base_ori_local_HS(1))))) / (Contact_Info_HS(0) + Contact_Info_HS(1) + Contact_Info_HS(2) + Contact_Info_HS(3));
    //    measure_z = -(Contact_Info_HS(0) * (actual_EP_local_HS(2) / cos(abs(actual_base_ori_local_HS(1)))) + Contact_Info_HS(1) * (actual_EP_local_HS(5) / cos(abs(actual_base_ori_local_HS(1)))) + Contact_Info_HS(2) * (actual_EP_local_HS(8) / cos(abs(actual_base_ori_local_HS(1)))) + Contact_Info_HS(3) * (actual_EP_local_HS(11) / cos(abs(actual_base_ori_local_HS(1))))) / (Contact_Info_HS(0) + Contact_Info_HS(1) + Contact_Info_HS(2) + Contact_Info_HS(3));
    //   // }
    //    
    //    Base_Estimation_Test();
    //    actual_base_pos_HS << target_base_pos_HS(0), target_base_pos_HS(1), x_hat(0);
    //actual_base_pos_HS << target_base_pos_HS(0), target_base_pos_HS(1), measure_z;


}

void CRobot::Base_Rotation_Matrix_HS(void) {
    target_C_WB_Y_HS << cos(target_yaw_HS), -sin(target_yaw_HS), 0\
, sin(target_yaw_HS), cos(target_yaw_HS), 0\
, 0, 0, 1;

    target_C_WB_P_HS << cos(target_base_ori_local_HS(1)), 0, sin(target_base_ori_local_HS(1))\
            , 0, 1, 0\
, -sin(target_base_ori_local_HS(1)), 0, cos(target_base_ori_local_HS(1));


    target_C_WB_R_HS << 1, 0, 0\
, 0, cos(target_base_ori_local_HS(0)), -sin(target_base_ori_local_HS(0))\
           , 0, sin(target_base_ori_local_HS(0)), cos(target_base_ori_local_HS(0));


    target_C_WB_Y_2d_HS = target_C_WB_Y_HS.block(0, 0, 2, 2);
    target_C_WB_PR_HS = target_C_WB_P_HS * target_C_WB_R_HS;
    target_C_WB_YPR_HS = target_C_WB_Y_HS*target_C_WB_PR_HS;

    actual_C_WB_P_HS << cos(actual_base_ori_local_HS(1)), 0, sin(actual_base_ori_local_HS(1))\
            , 0, 1, 0\
, -sin(actual_base_ori_local_HS(1)), 0, cos(actual_base_ori_local_HS(1));


    actual_C_WB_R_HS << 1, 0, 0\
, 0, cos(actual_base_ori_local_HS(0)), -sin(actual_base_ori_local_HS(0))\
           , 0, sin(actual_base_ori_local_HS(0)), cos(actual_base_ori_local_HS(0));
    
    actual_C_WB_PR_HS = actual_C_WB_P_HS*actual_C_WB_R_HS;
    actual_C_WB_YPR_HS = target_C_WB_Y_HS * actual_C_WB_PR_HS;

}

VectorNd CRobot::FK_HS(VectorNd joint_pos_HS) {
    VectorNd EP_pos_HS(12);

    const double L1 = 0.1045;
    const double L2 = 0.305;
    const double L3 = 0.309;

    double q1 = 0;
    double q2 = 0;
    double q3 = 0;

    //RL_EP
    q1 = joint_pos_HS[0];
    q2 = joint_pos_HS[1];
    q3 = joint_pos_HS[2];
    EP_pos_HS[0] = -L2 * sin(q2) - L3 * (cos(q2) * sin(q3) + cos(q3) * sin(q2));
    EP_pos_HS[1] = L1 * cos(q1) + L2 * cos(q2) * sin(q1) - L3 * (sin(q1) * sin(q2) * sin(q3) - cos(q2) * cos(q3) * sin(q1));
    EP_pos_HS[2] = L1 * sin(q1) - L2 * cos(q1) * cos(q2) - L3 * (cos(q1) * cos(q2) * cos(q3) - cos(q1) * sin(q2) * sin(q3));

    //RR_EP
    q1 = joint_pos_HS[3];
    q2 = joint_pos_HS[4];
    q3 = joint_pos_HS[5];
    EP_pos_HS[3] = -L2 * sin(q2) - L3 * (cos(q2) * sin(q3) + cos(q3) * sin(q2));
    EP_pos_HS[4] = -L1 * cos(q1) + L2 * cos(q2) * sin(q1) - L3 * (sin(q1) * sin(q2) * sin(q3) - cos(q2) * cos(q3) * sin(q1));
    EP_pos_HS[5] = -L1 * sin(q1) - L2 * cos(q1) * cos(q2) - L3 * (cos(q1) * cos(q2) * cos(q3) - cos(q1) * sin(q2) * sin(q3));

    //FL_EP
    q1 = joint_pos_HS[7];
    q2 = joint_pos_HS[8];
    q3 = joint_pos_HS[9];
    EP_pos_HS[6] = -L2 * sin(q2) - L3 * (cos(q2) * sin(q3) + cos(q3) * sin(q2));
    EP_pos_HS[7] = L1 * cos(q1) + L2 * cos(q2) * sin(q1) - L3 * (sin(q1) * sin(q2) * sin(q3) - cos(q2) * cos(q3) * sin(q1));
    EP_pos_HS[8] = L1 * sin(q1) - L2 * cos(q1) * cos(q2) - L3 * (cos(q1) * cos(q2) * cos(q3) - cos(q1) * sin(q2) * sin(q3));

    //FR_EP
    q1 = joint_pos_HS[10];
    q2 = joint_pos_HS[11];
    q3 = joint_pos_HS[12];
    EP_pos_HS[9] = -L2 * sin(q2) - L3 * (cos(q2) * sin(q3) + cos(q3) * sin(q2));
    EP_pos_HS[10] = -L1 * cos(q1) + L2 * cos(q2) * sin(q1) - L3 * (sin(q1) * sin(q2) * sin(q3) - cos(q2) * cos(q3) * sin(q1)); //revised by HSKIM
    EP_pos_HS[11] = -L1 * sin(q1) - L2 * cos(q1) * cos(q2) - L3 * (cos(q1) * cos(q2) * cos(q3) - cos(q1) * sin(q2) * sin(q3));

    return EP_pos_HS;
}

VectorNd CRobot::Localization_Hip2Base_Pos_HS(VectorNd EP_pos_local_hip) {
    VectorNd EP_pos_local(12);

    EP_pos_local(0) = EP_pos_local_hip(0) + RL_base2hip_pos(0);
    EP_pos_local(1) = EP_pos_local_hip(1) + RL_base2hip_pos(1);
    EP_pos_local(2) = EP_pos_local_hip(2) + RL_base2hip_pos(2);

    EP_pos_local(3) = EP_pos_local_hip(3) + RR_base2hip_pos(0);
    EP_pos_local(4) = EP_pos_local_hip(4) + RR_base2hip_pos(1);
    EP_pos_local(5) = EP_pos_local_hip(5) + RR_base2hip_pos(2);

    EP_pos_local(6) = EP_pos_local_hip(6) + FL_base2hip_pos(0);
    EP_pos_local(7) = EP_pos_local_hip(7) + FL_base2hip_pos(1);
    EP_pos_local(8) = EP_pos_local_hip(8) + FL_base2hip_pos(2);

    EP_pos_local(9) = EP_pos_local_hip(9) + FR_base2hip_pos(0);
    EP_pos_local(10) = EP_pos_local_hip(10) + FR_base2hip_pos(1);
    EP_pos_local(11) = EP_pos_local_hip(11) + FR_base2hip_pos(2);

    return EP_pos_local;
}

void CRobot::WalkReady_Pos_Traj_HS(void) {
    step_time_HS = 2.0;
    VectorNd init_base_pos_HS(3);
    VectorNd init_base_pos_12d_HS(12);
    // if (WalkReady_flag_HS == true) {
    if (cnt_HS == 0) {
        init_base_pos_HS = -(actual_EP_local_HS.segment(0, 3) + actual_EP_local_HS.segment(3, 3) + actual_EP_local_HS.segment(6, 3) + actual_EP_local_HS.segment(9, 3)) / 4.0;
        init_com_pos_HS = Get_COM_pos_HS2(init_base_pos_HS);
        init_base_pos_12d_HS << init_base_pos_HS, init_base_pos_HS, init_base_pos_HS, init_base_pos_HS;
        init_EP_HS = init_base_pos_12d_HS + target_C_WB_YPR_12d_HS * (actual_EP_local_HS);

        goal_com_pos_HS << 0.0, 0.0, com_height_HS;
        goal_EP_HS << tar_init_RL_foot_pos, tar_init_RR_foot_pos, tar_init_FL_foot_pos, tar_init_FR_foot_pos;

        target_com_pos_HS = init_com_pos_HS;
        target_EP_HS = init_EP_HS;
        target_com_vel_HS = VectorNd::Zero(3);
        target_com_acc_HS = VectorNd::Zero(3);
        target_EP_vel_HS = VectorNd::Zero(12);
        cnt_HS++;
    } else if (cnt_HS < step_time_HS / dt) {
        target_com_pos_HS = init_com_pos_HS + (goal_com_pos_HS - init_com_pos_HS) / 2.0 * (1 - cos(PI / step_time_HS * cnt_HS * dt));
        target_EP_HS = init_EP_HS + (goal_EP_HS - init_EP_HS) / 2.0 * (1 - cos(PI / step_time_HS * cnt_HS * dt));
        target_EP_vel_HS = PI / step_time_HS * (goal_EP_HS - init_EP_HS) / 2.0 * sin(PI / step_time_HS * cnt_HS * dt);

        cnt_HS++;
    } else {
        target_com_pos_HS = goal_com_pos_HS;
        target_com_vel_HS = VectorNd::Zero(3);
        target_com_acc_HS = VectorNd::Zero(3);
        target_EP_HS = goal_EP_HS;
        target_EP_vel_HS = VectorNd::Zero(12);
        cnt_HS = 0;
        WalkReady_flag_HS = false;
    }
    //}
    //tmp_actual_EP_local_HS = actual_EP_local_HS;
    target_base_pos_HS = Get_Base_pos_HS2(target_com_pos_HS);
    //std::cout << cnt_HS << std::endl;
}

VectorNd CRobot::Get_COM_pos_HS2(VectorNd _Base_Pos) {
    VectorNd Com_Pos(3);

    Com_Pos = _Base_Pos + target_C_WB_YPR_HS*offset_B2C;
    return Com_Pos;
}

VectorNd CRobot::Get_Base_pos_HS2(VectorNd _Com_Pos) {
    VectorNd Base_Pos(3);

    Base_Pos = _Com_Pos - target_C_WB_YPR_HS*offset_B2C;
    return Base_Pos;
}

void CRobot::Walking_Gait_Traj_HS(void) {
    // std::cout << "cnt_HS:" << cnt_HS << std::endl;
    //std::cout << "_------------------------" << std::endl;
    step_time_HS = tsp_time_HS + fsp_time_HS;

    if (cnt_HS < preview_cnt_HS) {
        Walking_Traj_First_HS(cnt_HS);
        cnt_HS++;
    } else if (cnt_HS < (preview_cnt_HS + step_cnt_HS * 4.0)) {

        Walking_Traj_COM_VER_HS5(cnt_HS - preview_cnt_HS);

        Fly_Leg_Gain_Controller4(cnt_HS - preview_cnt_HS);

        if (cnt_HS == preview_cnt_HS + step_cnt_HS * 4.0 - 1) {
            cnt_HS = preview_cnt_HS - 1;
        }

        cnt_HS++;
    }
    COM_XY_Traj_Gen_COM_VER_HS3(init_com_pos_HS, goal_com_pos_HS);

    Break_leg();
}

void CRobot::Walking_Traj_First_HS(unsigned int _i) {
    if (_i == 0) {
        //*************** Foot Step Trajectory***************//
        init_com_pos_HS = target_com_pos_HS;

        pre_init_yaw_HS = target_yaw_HS;
        init_yaw_HS = pre_init_yaw_HS;

        pre_init_EP_HS = target_EP_HS;
        init_EP_HS = pre_init_EP_HS;

        now_vel_HS << 0.0, 0.0, 0.0;
        tar_vel_HS << 0.0, 0.0, 0.0;

        FootStepPlanning_HS3(init_com_pos_HS, init_yaw_HS, init_EP_HS, tar_vel_HS);

        goal_EP_HS(2) = 0.0;
        goal_EP_HS(5) = 0.0;
        goal_EP_HS(8) = 0.0;
        goal_EP_HS(11) = 0.0;

    }
    //******************** Com Trajectory************************//
    target_com_pos_HS(2) = goal_com_pos_HS(2);
    target_base_pos_HS = Get_Base_pos_HS2(target_com_pos_HS);
    Contact_Info_HS << 1, 1, 1, 1;
}

void CRobot::FootStepPlanning_HS3(VectorNd _now_com_pos, double _now_yaw, VectorNd _now_EP_pos, VectorNd _tar_vel) {

    static double increment_com_x, increment_com_y, increment_base_yaw;
    VectorNd goal_EP_rot(12);
    MatrixNd R_Rot(3, 3);
    MatrixNd R_goal(2, 2);

    VectorNd r_RL_now(2), r_RR_now(2), r_FL_now(2), r_FR_now(2);
    VectorNd tmp_RL_goal_delx(2), tmp_RR_goal_delx(2), tmp_FL_goal_delx(2), tmp_FR_goal_delx(2);
    VectorNd tmp_RL_goal_dely(2), tmp_RR_goal_dely(2), tmp_FL_goal_dely(2), tmp_FR_goal_dely(2);

    VectorNd r_del_local(2);
    VectorNd r_del_local_x(2);
    VectorNd r_del_local_y(2);
    VectorNd n_x_local(2);
    VectorNd n_x_global(2);
    VectorNd n_y_local(2);
    VectorNd n_y_global(2);
    MatrixNd R_now_HS(2, 2);

    increment_com_x = 4.0 * step_time_HS * _tar_vel(0) * cos(target_base_ori_local_HS(1));
    increment_com_y = 4.0 * step_time_HS * _tar_vel(1);
    increment_base_yaw = 4.0 * step_time_HS * _tar_vel(2);

    goal_yaw_HS = _now_yaw + increment_base_yaw;

    R_Rot << cos(increment_base_yaw), -sin(increment_base_yaw), 0\
, sin(increment_base_yaw), cos(increment_base_yaw), 0\
, 0, 0, 1;

    goal_EP_rot.segment(0, 3) = R_Rot * (_now_EP_pos.segment(0, 3) - _now_com_pos) + _now_com_pos;
    goal_EP_rot.segment(3, 3) = R_Rot * (_now_EP_pos.segment(3, 3) - _now_com_pos) + _now_com_pos;
    goal_EP_rot.segment(6, 3) = R_Rot * (_now_EP_pos.segment(6, 3) - _now_com_pos) + _now_com_pos;
    goal_EP_rot.segment(9, 3) = R_Rot * (_now_EP_pos.segment(9, 3) - _now_com_pos) + _now_com_pos;

    r_RL_now << goal_EP_rot(0), goal_EP_rot(1);
    r_RR_now << goal_EP_rot(3), goal_EP_rot(4);
    r_FL_now << goal_EP_rot(6), goal_EP_rot(7);
    r_FR_now << goal_EP_rot(9), goal_EP_rot(10);

    R_now_HS << cos(_now_yaw), -sin(_now_yaw)\
           , sin(_now_yaw), cos(_now_yaw);

    R_goal << cos(goal_yaw_HS), -sin(goal_yaw_HS)\
           , sin(goal_yaw_HS), cos(goal_yaw_HS);

    r_del_local << increment_com_x, increment_com_y;
    r_del_local_x << increment_com_x, 0.0;
    r_del_local_y << 0.0, increment_com_y;

    n_x_local << 1, 0;
    n_y_local << 0, 1;

    n_x_global = R_goal*n_x_local;
    n_y_global = R_goal*n_y_local;

    tmp_RL_goal_delx = r_RL_now + R_goal*r_del_local_x;
    tmp_FR_goal_delx = r_FR_now + R_goal*r_del_local_x;
    tmp_RR_goal_delx = r_RR_now + ((r_RL_now(0) - r_RR_now(0)) * n_x_global(0)+(r_RL_now(1) - r_RR_now(1)) * n_x_global(1))*(n_x_global) + R_goal * r_del_local_x / 2.0;
    tmp_FL_goal_delx = r_FL_now + ((r_FR_now(0) - r_FL_now(0)) * n_x_global(0)+(r_FR_now(1) - r_FL_now(1)) * n_x_global(1))*(n_x_global) + R_goal * r_del_local_x / 2.0;

    tmp_FR_goal_dely = tmp_FR_goal_delx + R_goal*r_del_local_y;
    tmp_FL_goal_dely = tmp_FL_goal_delx + R_goal*r_del_local_y;
    tmp_RR_goal_dely = tmp_RR_goal_delx + ((tmp_FR_goal_delx(0) - tmp_RR_goal_delx(0)) * n_y_global(0)+(tmp_FR_goal_delx(1) - tmp_RR_goal_delx(1)) * n_y_global(1))*(n_y_global) + R_goal * r_del_local_y / 2.0;
    tmp_RL_goal_dely = tmp_RL_goal_delx + ((tmp_FL_goal_delx(0) - tmp_RL_goal_delx(0)) * n_y_global(0)+(tmp_FL_goal_delx(1) - tmp_RL_goal_delx(1)) * n_y_global(1))*(n_y_global) + R_goal * r_del_local_y / 2.0;

    goal_EP_HS(0) = tmp_RL_goal_dely(0);
    goal_EP_HS(1) = tmp_RL_goal_dely(1);
    goal_EP_HS(3) = tmp_RR_goal_dely(0);
    goal_EP_HS(4) = tmp_RR_goal_dely(1);
    goal_EP_HS(6) = tmp_FL_goal_dely(0);
    goal_EP_HS(7) = tmp_FL_goal_dely(1);
    goal_EP_HS(9) = tmp_FR_goal_dely(0);
    goal_EP_HS(10) = tmp_FR_goal_dely(1);

    goal_com_pos_HS(0) = (goal_EP_HS(0) + goal_EP_HS(3) + goal_EP_HS(6) + goal_EP_HS(9)) / 4.0;
    goal_com_pos_HS(1) = (goal_EP_HS(1) + goal_EP_HS(4) + goal_EP_HS(7) + goal_EP_HS(10)) / 4.0;
}

void CRobot::Walking_Traj_COM_VER_HS5(unsigned int _i) { //RR/FL/RL/FR
    VectorNd tmp_bezi_val(6);
    walk_time = _i*dt;

    if (_i == 0) {

        if (move_stop_flag == true) {
            if (com_stop_flag_HS == true) {
                walk_stop_flag_HS = true;
                pre_sub_ctrl_flag = move_stop_flag;
            }
            if (speed_stop_flag_HS == true) {
                com_stop_flag_HS = true;
            }
            speed_stop_flag_HS = true;
            speed_x = 0.0;
            speed_y = 0.0;
            speed_yaw = 0.0;
        }
        if (move_stop_flag == false) {
            if (com_stop_flag_HS == false) {
                walk_stop_flag_HS = false;
                pre_sub_ctrl_flag = move_stop_flag;
            }
            com_stop_flag_HS = false;
            speed_stop_flag_HS = false;
        }

        tar_vel_HS << speed_x, speed_y, speed_yaw;

        FootStepPlanning_HS3(init_com_pos_HS, init_yaw_HS, init_EP_HS, tar_vel_HS);

    }//************** Foot step Run *****************//
    if (walk_stop_flag_HS != true) {
        if (_i < tsp_cnt_HS) { //cnt:1400~1649 (250)
            if (adaptive_flag_HS == true) {
                //actual_EP_HS << target_base_pos_HS + target_C_WB_YPR_HS * actual_EP_local_HS.segment(0, 3), target_base_pos_HS + target_C_WB_YPR_HS * actual_EP_local_HS.segment(3, 3), target_base_pos_HS + target_C_WB_YPR_HS * actual_EP_local_HS.segment(6, 3), target_base_pos_HS + target_C_WB_YPR_HS * actual_EP_local_HS.segment(9, 3);
                //                actual_EP_HS << actual_base_pos_HS + target_C_WB_YPR_HS * actual_EP_local_HS.segment(0, 3), actual_base_pos_HS + target_C_WB_YPR_HS * actual_EP_local_HS.segment(3, 3), actual_base_pos_HS + target_C_WB_YPR_HS * actual_EP_local_HS.segment(6, 3), actual_base_pos_HS + target_C_WB_YPR_HS * actual_EP_local_HS.segment(9, 3);
                //actual_EP_HS << actual_base_pos_HS + actual_C_WB_YPR_HS * actual_EP_local_HS.segment(0, 3), actual_base_pos_HS + actual_C_WB_YPR_HS * actual_EP_local_HS.segment(3, 3), actual_base_pos_HS + actual_C_WB_YPR_HS * actual_EP_local_HS.segment(6, 3), actual_base_pos_HS + actual_C_WB_YPR_HS * actual_EP_local_HS.segment(9, 3);

                target_EP_HS(2) = actual_EP_HS(2);
                //target_EP_HS(5) = actual_EP_HS(5);
                target_EP_HS(8) = actual_EP_HS(8);
                target_EP_HS(11) = actual_EP_HS(11);
            }
            if (_i == 0) {
                if (adaptive_flag_HS == true) {
                    target_EP_HS(2) = actual_EP_HS(2);
                    target_EP_HS(5) = actual_EP_HS(5);
                    target_EP_HS(8) = actual_EP_HS(8);
                    target_EP_HS(11) = actual_EP_HS(11);
                }
                pre_init_EP_HS(2) = target_EP_HS(2);
                pre_init_EP_HS(5) = target_EP_HS(5);
                pre_init_EP_HS(8) = target_EP_HS(8);
                pre_init_EP_HS(11) = target_EP_HS(11);

                init_EP_HS(2) = pre_init_EP_HS(2);
                init_EP_HS(5) = pre_init_EP_HS(5);
                init_EP_HS(8) = pre_init_EP_HS(8);
                init_EP_HS(11) = pre_init_EP_HS(11);

                if (foot_height_HS == 0.0) {
                    Contact_Info_HS << 1, 1, 1, 1;
                } else {
                    Contact_Info_HS << 1, 0, 1, 1;
                    Leg_state(0) = 2;
                }
                tmp_actual_EP_local_HS = actual_EP_local_HS;
                static_actual_EP_local_HS = actual_EP_local_HS;
            }

            tmp_bezi_val = Bezier_Curve_trajectory3(Contact_Info_HS, pre_init_EP_HS.segment(3, 3), init_EP_HS.segment(3, 3), tmp_actual_EP_local_HS.segment(3, 3));
            //tmp_bezi_val = Bezier_Curve_trajectory4(Contact_Info_HS, pre_init_EP_HS.segment(3, 3), init_EP_HS.segment(3, 3), tmp_actual_EP_local_HS.segment(3, 3));
            target_EP_HS.segment(3, 3) = tmp_bezi_val.segment(0, 3);
            target_EP_vel_HS.segment(3, 3) = tmp_bezi_val.segment(3, 3);

            target_EP_HS(0) = pre_init_EP_HS(0);
            target_EP_HS(1) = pre_init_EP_HS(1);
            target_EP_HS(2) = target_EP_HS(2);
            target_EP_vel_HS(0) = 0.0;
            target_EP_vel_HS(1) = 0.0;
            target_EP_vel_HS(2) = 0.0;

            target_EP_HS(6) = pre_init_EP_HS(6);
            target_EP_HS(7) = pre_init_EP_HS(7);
            target_EP_HS(8) = target_EP_HS(8);
            target_EP_vel_HS(6) = 0.0;
            target_EP_vel_HS(7) = 0.0;
            target_EP_vel_HS(8) = 0.0;

            target_EP_HS(9) = pre_init_EP_HS(9);
            target_EP_HS(10) = pre_init_EP_HS(10);
            target_EP_HS(11) = target_EP_HS(11);
            target_EP_vel_HS(9) = 0.0;
            target_EP_vel_HS(10) = 0.0;
            target_EP_vel_HS(11) = 0.0;

        } else if (_i < step_cnt_HS) { //cnt:1650~1749
            target_EP_HS(0) = pre_init_EP_HS(0);
            target_EP_HS(1) = pre_init_EP_HS(1);

            target_EP_HS(3) = init_EP_HS(3);
            target_EP_HS(4) = init_EP_HS(4);

            target_EP_HS(6) = pre_init_EP_HS(6);
            target_EP_HS(7) = pre_init_EP_HS(7);

            target_EP_HS(9) = pre_init_EP_HS(9);
            target_EP_HS(10) = pre_init_EP_HS(10);

            // Z direction
            if (adaptive_flag_HS == true) {
                //actual_EP_HS << target_base_pos_HS + target_C_WB_YPR_HS * actual_EP_local_HS.segment(0, 3), target_base_pos_HS + target_C_WB_YPR_HS * actual_EP_local_HS.segment(3, 3), target_base_pos_HS + target_C_WB_YPR_HS * actual_EP_local_HS.segment(6, 3), target_base_pos_HS + target_C_WB_YPR_HS * actual_EP_local_HS.segment(9, 3);
                //                actual_EP_HS << actual_base_pos_HS + target_C_WB_YPR_HS * actual_EP_local_HS.segment(0, 3), actual_base_pos_HS + target_C_WB_YPR_HS * actual_EP_local_HS.segment(3, 3), actual_base_pos_HS + target_C_WB_YPR_HS * actual_EP_local_HS.segment(6, 3), actual_base_pos_HS + target_C_WB_YPR_HS * actual_EP_local_HS.segment(9, 3);
                //actual_EP_HS << actual_base_pos_HS + actual_C_WB_YPR_HS * actual_EP_local_HS.segment(0, 3), actual_base_pos_HS + actual_C_WB_YPR_HS * actual_EP_local_HS.segment(3, 3), actual_base_pos_HS + actual_C_WB_YPR_HS * actual_EP_local_HS.segment(6, 3), actual_base_pos_HS + actual_C_WB_YPR_HS * actual_EP_local_HS.segment(9, 3);
                target_EP_HS(2) = actual_EP_HS(2);
                target_EP_HS(5) = actual_EP_HS(5);
                target_EP_HS(8) = actual_EP_HS(8);
                target_EP_HS(11) = actual_EP_HS(11);
            } else {
                target_EP_HS(2) = target_EP_HS(2);
                target_EP_HS(5) = target_EP_HS(5);
                target_EP_HS(8) = target_EP_HS(8);
                target_EP_HS(11) = target_EP_HS(11);
            }

            target_EP_vel_HS = VectorNd::Zero(12);

            Contact_Info_HS << 1, 1, 1, 1;
        } else if (_i < step_cnt_HS + tsp_cnt_HS) { //cnt:1750~1999
            if (adaptive_flag_HS == true) {
                //actual_EP_HS << target_base_pos_HS + target_C_WB_YPR_HS * actual_EP_local_HS.segment(0, 3), target_base_pos_HS + target_C_WB_YPR_HS * actual_EP_local_HS.segment(3, 3), target_base_pos_HS + target_C_WB_YPR_HS * actual_EP_local_HS.segment(6, 3), target_base_pos_HS + target_C_WB_YPR_HS * actual_EP_local_HS.segment(9, 3);
                //actual_EP_HS << actual_base_pos_HS + target_C_WB_YPR_HS * actual_EP_local_HS.segment(0, 3), actual_base_pos_HS + target_C_WB_YPR_HS * actual_EP_local_HS.segment(3, 3), actual_base_pos_HS + target_C_WB_YPR_HS * actual_EP_local_HS.segment(6, 3), actual_base_pos_HS + target_C_WB_YPR_HS * actual_EP_local_HS.segment(9, 3);
                //actual_EP_HS << actual_base_pos_HS + actual_C_WB_YPR_HS * actual_EP_local_HS.segment(0, 3), actual_base_pos_HS + actual_C_WB_YPR_HS * actual_EP_local_HS.segment(3, 3), actual_base_pos_HS + actual_C_WB_YPR_HS * actual_EP_local_HS.segment(6, 3), actual_base_pos_HS + actual_C_WB_YPR_HS * actual_EP_local_HS.segment(9, 3);
                //target_EP_HS(2) = actual_EP_HS(2);
                target_EP_HS(5) = actual_EP_HS(5);
                target_EP_HS(8) = actual_EP_HS(8);
                target_EP_HS(11) = actual_EP_HS(11);
            }
            if (_i == (step_cnt_HS)) {
                if (adaptive_flag_HS == true) {
                    target_EP_HS(2) = actual_EP_HS(2);
                    target_EP_HS(5) = actual_EP_HS(5);
                    target_EP_HS(8) = actual_EP_HS(8);
                    target_EP_HS(11) = actual_EP_HS(11);
                }
                pre_init_EP_HS(2) = target_EP_HS(2);
                pre_init_EP_HS(5) = target_EP_HS(5);
                pre_init_EP_HS(8) = target_EP_HS(8);
                pre_init_EP_HS(11) = target_EP_HS(11);

                init_EP_HS(2) = pre_init_EP_HS(2);
                init_EP_HS(5) = pre_init_EP_HS(5);
                init_EP_HS(8) = pre_init_EP_HS(8);
                init_EP_HS(11) = pre_init_EP_HS(11);

                if (foot_height_HS == 0.0) {
                    Contact_Info_HS << 1, 1, 1, 1;
                } else {
                    Contact_Info_HS << 0, 1, 1, 1;
                    Leg_state(0) = 1;
                }
                tmp_actual_EP_local_HS = actual_EP_local_HS;
                static_actual_EP_local_HS = actual_EP_local_HS;
            }

            tmp_bezi_val = Bezier_Curve_trajectory3(Contact_Info_HS, pre_init_EP_HS.segment(0, 3), init_EP_HS.segment(0, 3), tmp_actual_EP_local_HS.segment(0, 3));
            //            tmp_bezi_val = Bezier_Curve_trajectory4(Contact_Info_HS, pre_init_EP_HS.segment(0, 3), init_EP_HS.segment(0, 3), tmp_actual_EP_local_HS.segment(0, 3));
            target_EP_HS.segment(0, 3) = tmp_bezi_val.segment(0, 3);
            target_EP_vel_HS.segment(0, 3) = tmp_bezi_val.segment(3, 3);

            target_EP_HS(3) = init_EP_HS(3);
            target_EP_HS(4) = init_EP_HS(4);
            target_EP_HS(5) = target_EP_HS(5);
            target_EP_vel_HS(3) = 0.0;
            target_EP_vel_HS(4) = 0.0;
            target_EP_vel_HS(5) = 0.0;

            target_EP_HS(6) = pre_init_EP_HS(6);
            target_EP_HS(7) = pre_init_EP_HS(7);
            target_EP_HS(8) = target_EP_HS(8);
            target_EP_vel_HS(6) = 0.0;
            target_EP_vel_HS(7) = 0.0;
            target_EP_vel_HS(8) = 0.0;

            target_EP_HS(9) = pre_init_EP_HS(9);
            target_EP_HS(10) = pre_init_EP_HS(10);
            target_EP_HS(11) = target_EP_HS(11);
            target_EP_vel_HS(9) = 0.0;
            target_EP_vel_HS(10) = 0.0;
            target_EP_vel_HS(11) = 0.0;

        } else if (_i < step_cnt_HS * 2) { //cnt:2000~2099
            target_EP_HS(0) = init_EP_HS(0);
            target_EP_HS(1) = init_EP_HS(1);

            target_EP_HS(3) = init_EP_HS(3);
            target_EP_HS(4) = init_EP_HS(4);

            target_EP_HS(6) = pre_init_EP_HS(6);
            target_EP_HS(7) = pre_init_EP_HS(7);

            target_EP_HS(9) = pre_init_EP_HS(9);
            target_EP_HS(10) = pre_init_EP_HS(10);

            if (adaptive_flag_HS == true) {
                //actual_EP_HS << target_base_pos_HS + target_C_WB_YPR_HS * actual_EP_local_HS.segment(0, 3), target_base_pos_HS + target_C_WB_YPR_HS * actual_EP_local_HS.segment(3, 3), target_base_pos_HS + target_C_WB_YPR_HS * actual_EP_local_HS.segment(6, 3), target_base_pos_HS + target_C_WB_YPR_HS * actual_EP_local_HS.segment(9, 3);
                //actual_EP_HS << actual_base_pos_HS + target_C_WB_YPR_HS * actual_EP_local_HS.segment(0, 3), actual_base_pos_HS + target_C_WB_YPR_HS * actual_EP_local_HS.segment(3, 3), actual_base_pos_HS + target_C_WB_YPR_HS * actual_EP_local_HS.segment(6, 3), actual_base_pos_HS + target_C_WB_YPR_HS * actual_EP_local_HS.segment(9, 3);
                //actual_EP_HS << actual_base_pos_HS + actual_C_WB_YPR_HS * actual_EP_local_HS.segment(0, 3), actual_base_pos_HS + actual_C_WB_YPR_HS * actual_EP_local_HS.segment(3, 3), actual_base_pos_HS + actual_C_WB_YPR_HS * actual_EP_local_HS.segment(6, 3), actual_base_pos_HS + actual_C_WB_YPR_HS * actual_EP_local_HS.segment(9, 3);
                target_EP_HS(2) = actual_EP_HS(2);
                target_EP_HS(5) = actual_EP_HS(5);
                target_EP_HS(8) = actual_EP_HS(8);
                target_EP_HS(11) = actual_EP_HS(11);
            } else {
                target_EP_HS(2) = target_EP_HS(2);
                target_EP_HS(5) = target_EP_HS(5);
                target_EP_HS(8) = target_EP_HS(8);
                target_EP_HS(11) = target_EP_HS(11);
            }

            target_EP_vel_HS = VectorNd::Zero(12);

            Contact_Info_HS << 1, 1, 1, 1;

        } else if (_i < step_cnt_HS * 2 + tsp_cnt_HS) { //cnt:2100~2349
            if (adaptive_flag_HS == true) {
                //actual_EP_HS << target_base_pos_HS + target_C_WB_YPR_HS * actual_EP_local_HS.segment(0, 3), target_base_pos_HS + target_C_WB_YPR_HS * actual_EP_local_HS.segment(3, 3), target_base_pos_HS + target_C_WB_YPR_HS * actual_EP_local_HS.segment(6, 3), target_base_pos_HS + target_C_WB_YPR_HS * actual_EP_local_HS.segment(9, 3);
                //actual_EP_HS << actual_base_pos_HS + target_C_WB_YPR_HS * actual_EP_local_HS.segment(0, 3), actual_base_pos_HS + target_C_WB_YPR_HS * actual_EP_local_HS.segment(3, 3), actual_base_pos_HS + target_C_WB_YPR_HS * actual_EP_local_HS.segment(6, 3), actual_base_pos_HS + target_C_WB_YPR_HS * actual_EP_local_HS.segment(9, 3);
                //actual_EP_HS << actual_base_pos_HS + actual_C_WB_YPR_HS * actual_EP_local_HS.segment(0, 3), actual_base_pos_HS + actual_C_WB_YPR_HS * actual_EP_local_HS.segment(3, 3), actual_base_pos_HS + actual_C_WB_YPR_HS * actual_EP_local_HS.segment(6, 3), actual_base_pos_HS + actual_C_WB_YPR_HS * actual_EP_local_HS.segment(9, 3);
                target_EP_HS(2) = actual_EP_HS(2);
                target_EP_HS(5) = actual_EP_HS(5);
                //target_EP_HS(8) = actual_EP_HS(8);
                target_EP_HS(11) = actual_EP_HS(11);
            }
            if (_i == (step_cnt_HS * 2)) {
                if (adaptive_flag_HS == true) {
                    target_EP_HS(2) = actual_EP_HS(2);
                    target_EP_HS(5) = actual_EP_HS(5);
                    target_EP_HS(8) = actual_EP_HS(8);
                    target_EP_HS(11) = actual_EP_HS(11);
                }
                pre_init_EP_HS(2) = target_EP_HS(2);
                pre_init_EP_HS(5) = target_EP_HS(5);
                pre_init_EP_HS(8) = target_EP_HS(8);
                pre_init_EP_HS(11) = target_EP_HS(11);

                init_EP_HS(2) = pre_init_EP_HS(2);
                init_EP_HS(5) = pre_init_EP_HS(5);
                init_EP_HS(8) = pre_init_EP_HS(8);
                init_EP_HS(11) = pre_init_EP_HS(11);

                if (foot_height_HS == 0.0) {
                    Contact_Info_HS << 1, 1, 1, 1;
                } else {
                    Contact_Info_HS << 1, 1, 0, 1;
                    Leg_state(0) = 3;
                }
                tmp_actual_EP_local_HS = actual_EP_local_HS;
                static_actual_EP_local_HS = actual_EP_local_HS;
            }

            tmp_bezi_val = Bezier_Curve_trajectory3(Contact_Info_HS, pre_init_EP_HS.segment(6, 3), init_EP_HS.segment(6, 3), tmp_actual_EP_local_HS.segment(6, 3));
            //tmp_bezi_val = Bezier_Curve_trajectory4(Contact_Info_HS, pre_init_EP_HS.segment(6, 3), init_EP_HS.segment(6, 3), tmp_actual_EP_local_HS.segment(6, 3));
            target_EP_HS.segment(6, 3) = tmp_bezi_val.segment(0, 3);
            target_EP_vel_HS.segment(6, 3) = tmp_bezi_val.segment(3, 3);

            target_EP_HS(0) = init_EP_HS(0);
            target_EP_HS(1) = init_EP_HS(1);
            target_EP_HS(2) = target_EP_HS(2);
            target_EP_vel_HS(0) = 0.0;
            target_EP_vel_HS(1) = 0.0;
            target_EP_vel_HS(2) = 0.0;

            target_EP_HS(3) = init_EP_HS(3);
            target_EP_HS(4) = init_EP_HS(4);
            target_EP_HS(5) = target_EP_HS(5);
            target_EP_vel_HS(3) = 0.0;
            target_EP_vel_HS(4) = 0.0;
            target_EP_vel_HS(5) = 0.0;

            target_EP_HS(9) = pre_init_EP_HS(9);
            target_EP_HS(10) = pre_init_EP_HS(10);
            target_EP_HS(11) = target_EP_HS(11);
            target_EP_vel_HS(9) = 0.0;
            target_EP_vel_HS(10) = 0.0;
            target_EP_vel_HS(11) = 0.0;

        } else if (_i < step_cnt_HS * 3) { //cnt:2350~2449
            target_EP_HS(0) = init_EP_HS(0);
            target_EP_HS(1) = init_EP_HS(1);
            target_EP_HS(3) = init_EP_HS(3);
            target_EP_HS(4) = init_EP_HS(4);
            target_EP_HS(6) = init_EP_HS(6);
            target_EP_HS(7) = init_EP_HS(7);
            target_EP_HS(9) = pre_init_EP_HS(9);
            target_EP_HS(10) = pre_init_EP_HS(10);

            if (adaptive_flag_HS == true) {
                //actual_EP_HS << target_base_pos_HS + target_C_WB_YPR_HS * actual_EP_local_HS.segment(0, 3), target_base_pos_HS + target_C_WB_YPR_HS * actual_EP_local_HS.segment(3, 3), target_base_pos_HS + target_C_WB_YPR_HS * actual_EP_local_HS.segment(6, 3), target_base_pos_HS + target_C_WB_YPR_HS * actual_EP_local_HS.segment(9, 3);
                //actual_EP_HS << actual_base_pos_HS + target_C_WB_YPR_HS * actual_EP_local_HS.segment(0, 3), actual_base_pos_HS + target_C_WB_YPR_HS * actual_EP_local_HS.segment(3, 3), actual_base_pos_HS + target_C_WB_YPR_HS * actual_EP_local_HS.segment(6, 3), actual_base_pos_HS + target_C_WB_YPR_HS * actual_EP_local_HS.segment(9, 3);
                //actual_EP_HS << actual_base_pos_HS + actual_C_WB_YPR_HS * actual_EP_local_HS.segment(0, 3), actual_base_pos_HS + actual_C_WB_YPR_HS * actual_EP_local_HS.segment(3, 3), actual_base_pos_HS + actual_C_WB_YPR_HS * actual_EP_local_HS.segment(6, 3), actual_base_pos_HS + actual_C_WB_YPR_HS * actual_EP_local_HS.segment(9, 3);
                target_EP_HS(2) = actual_EP_HS(2);
                target_EP_HS(5) = actual_EP_HS(5);
                target_EP_HS(8) = actual_EP_HS(8);
                target_EP_HS(11) = actual_EP_HS(11);
            } else {
                target_EP_HS(2) = target_EP_HS(2);
                target_EP_HS(5) = target_EP_HS(5);
                target_EP_HS(8) = target_EP_HS(8);
                target_EP_HS(11) = target_EP_HS(11);
            }

            target_EP_vel_HS = VectorNd::Zero(12);

            Contact_Info_HS << 1, 1, 1, 1;
        } else if (_i < step_cnt_HS * 3 + tsp_cnt_HS) { //cnt:2450~2699
            if (adaptive_flag_HS == true) {
                //actual_EP_HS << target_base_pos_HS + target_C_WB_YPR_HS * actual_EP_local_HS.segment(0, 3), target_base_pos_HS + target_C_WB_YPR_HS * actual_EP_local_HS.segment(3, 3), target_base_pos_HS + target_C_WB_YPR_HS * actual_EP_local_HS.segment(6, 3), target_base_pos_HS + target_C_WB_YPR_HS * actual_EP_local_HS.segment(9, 3);
                //actual_EP_HS << actual_base_pos_HS + target_C_WB_YPR_HS * actual_EP_local_HS.segment(0, 3), actual_base_pos_HS + target_C_WB_YPR_HS * actual_EP_local_HS.segment(3, 3), actual_base_pos_HS + target_C_WB_YPR_HS * actual_EP_local_HS.segment(6, 3), actual_base_pos_HS + target_C_WB_YPR_HS * actual_EP_local_HS.segment(9, 3);
                //actual_EP_HS << actual_base_pos_HS + actual_C_WB_YPR_HS * actual_EP_local_HS.segment(0, 3), actual_base_pos_HS + actual_C_WB_YPR_HS * actual_EP_local_HS.segment(3, 3), actual_base_pos_HS + actual_C_WB_YPR_HS * actual_EP_local_HS.segment(6, 3), actual_base_pos_HS + actual_C_WB_YPR_HS * actual_EP_local_HS.segment(9, 3);
                target_EP_HS(2) = actual_EP_HS(2);
                target_EP_HS(5) = actual_EP_HS(5);
                target_EP_HS(8) = actual_EP_HS(8);
                //target_EP_HS(11) = actual_EP_HS(11);
            }
            if (_i == (step_cnt_HS * 3)) {
                if (adaptive_flag_HS == true) {
                    target_EP_HS(2) = actual_EP_HS(2);
                    target_EP_HS(5) = actual_EP_HS(5);
                    target_EP_HS(8) = actual_EP_HS(8);
                    target_EP_HS(11) = actual_EP_HS(11);
                }
                pre_init_EP_HS(2) = target_EP_HS(2);
                pre_init_EP_HS(5) = target_EP_HS(5);
                pre_init_EP_HS(8) = target_EP_HS(8);
                pre_init_EP_HS(11) = target_EP_HS(11);

                init_EP_HS(2) = pre_init_EP_HS(2);
                init_EP_HS(5) = pre_init_EP_HS(5);
                init_EP_HS(8) = pre_init_EP_HS(8);
                init_EP_HS(11) = pre_init_EP_HS(11);

                if (foot_height_HS == 0.0) {
                    Contact_Info_HS << 1, 1, 1, 1;
                } else {
                    Contact_Info_HS << 1, 1, 1, 0;
                    Leg_state(0) = 4;
                }
                tmp_actual_EP_local_HS = actual_EP_local_HS;
                static_actual_EP_local_HS = actual_EP_local_HS;
            }

            tmp_bezi_val = Bezier_Curve_trajectory3(Contact_Info_HS, pre_init_EP_HS.segment(9, 3), init_EP_HS.segment(9, 3), tmp_actual_EP_local_HS.segment(9, 3));
            //tmp_bezi_val = Bezier_Curve_trajectory4(Contact_Info_HS, pre_init_EP_HS.segment(9, 3), init_EP_HS.segment(9, 3), tmp_actual_EP_local_HS.segment(9, 3));
            target_EP_HS.segment(9, 3) = tmp_bezi_val.segment(0, 3);
            target_EP_vel_HS.segment(9, 3) = tmp_bezi_val.segment(3, 3);

            target_EP_HS(0) = init_EP_HS(0);
            target_EP_HS(1) = init_EP_HS(1);
            target_EP_HS(2) = target_EP_HS(2);
            target_EP_vel_HS(0) = 0.0;
            target_EP_vel_HS(1) = 0.0;
            target_EP_vel_HS(2) = 0.0;

            target_EP_HS(3) = init_EP_HS(3);
            target_EP_HS(4) = init_EP_HS(4);
            target_EP_HS(5) = target_EP_HS(5);
            target_EP_vel_HS(3) = 0.0;
            target_EP_vel_HS(4) = 0.0;
            target_EP_vel_HS(5) = 0.0;

            target_EP_HS(6) = init_EP_HS(6);
            target_EP_HS(7) = init_EP_HS(7);
            target_EP_HS(8) = target_EP_HS(8);
            target_EP_vel_HS(6) = 0.0;
            target_EP_vel_HS(7) = 0.0;
            target_EP_vel_HS(8) = 0.0;

        } else { //cnt:2700~2799
            target_EP_HS(0) = init_EP_HS(0);
            target_EP_HS(1) = init_EP_HS(1);
            target_EP_HS(3) = init_EP_HS(3);
            target_EP_HS(4) = init_EP_HS(4);
            target_EP_HS(6) = init_EP_HS(6);
            target_EP_HS(7) = init_EP_HS(7);
            target_EP_HS(9) = init_EP_HS(9);
            target_EP_HS(10) = init_EP_HS(10);

            target_EP_vel_HS = VectorNd::Zero(12);

            if (adaptive_flag_HS == true) {
                //actual_EP_HS << target_base_pos_HS + target_C_WB_YPR_HS * actual_EP_local_HS.segment(0, 3), target_base_pos_HS + target_C_WB_YPR_HS * actual_EP_local_HS.segment(3, 3), target_base_pos_HS + target_C_WB_YPR_HS * actual_EP_local_HS.segment(6, 3), target_base_pos_HS + target_C_WB_YPR_HS * actual_EP_local_HS.segment(9, 3);
                //actual_EP_HS << actual_base_pos_HS + target_C_WB_YPR_HS * actual_EP_local_HS.segment(0, 3), actual_base_pos_HS + target_C_WB_YPR_HS * actual_EP_local_HS.segment(3, 3), actual_base_pos_HS + target_C_WB_YPR_HS * actual_EP_local_HS.segment(6, 3), actual_base_pos_HS + target_C_WB_YPR_HS * actual_EP_local_HS.segment(9, 3);
                //actual_EP_HS << actual_base_pos_HS + actual_C_WB_YPR_HS * actual_EP_local_HS.segment(0, 3), actual_base_pos_HS + actual_C_WB_YPR_HS * actual_EP_local_HS.segment(3, 3), actual_base_pos_HS + actual_C_WB_YPR_HS * actual_EP_local_HS.segment(6, 3), actual_base_pos_HS + actual_C_WB_YPR_HS * actual_EP_local_HS.segment(9, 3);
                target_EP_HS(2) = actual_EP_HS(2);
                target_EP_HS(5) = actual_EP_HS(5);
                target_EP_HS(8) = actual_EP_HS(8);
                target_EP_HS(11) = actual_EP_HS(11);
            } else {
                target_EP_HS(2) = target_EP_HS(2);
                target_EP_HS(5) = target_EP_HS(5);
                target_EP_HS(8) = target_EP_HS(8);
                target_EP_HS(11) = target_EP_HS(11);
            }

            Contact_Info_HS << 1, 1, 1, 1;
        }

        target_yaw_HS = pre_init_yaw_HS + (init_yaw_HS - pre_init_yaw_HS) / 2.0 * (1 - cos(PI / (step_time_HS * 4.0) * walk_time));

        //* update
        //        if (_i == (step_cnt_HS * 4 - 1)) {
        if (_i == (step_cnt_HS * 4 - 1)) {
            now_vel_HS = tar_vel_HS;

            init_com_pos_HS(0) = goal_com_pos_HS(0); //
            init_com_pos_HS(1) = goal_com_pos_HS(1); //

            pre_init_yaw_HS = init_yaw_HS;
            init_yaw_HS = goal_yaw_HS;

            pre_init_EP_HS(0) = init_EP_HS(0); //
            pre_init_EP_HS(1) = init_EP_HS(1); //
            pre_init_EP_HS(3) = init_EP_HS(3); //
            pre_init_EP_HS(4) = init_EP_HS(4); //
            pre_init_EP_HS(6) = init_EP_HS(6); //
            pre_init_EP_HS(7) = init_EP_HS(7); //
            pre_init_EP_HS(9) = init_EP_HS(9); //
            pre_init_EP_HS(10) = init_EP_HS(10); //

            init_EP_HS(0) = goal_EP_HS(0);
            init_EP_HS(1) = goal_EP_HS(1);
            init_EP_HS(3) = goal_EP_HS(3);
            init_EP_HS(4) = goal_EP_HS(4);
            init_EP_HS(6) = goal_EP_HS(6);
            init_EP_HS(7) = goal_EP_HS(7);
            init_EP_HS(9) = goal_EP_HS(9);
            init_EP_HS(10) = goal_EP_HS(10);
        }
    }
}

VectorNd CRobot::Bezier_Curve_trajectory3(VectorNd Contact_info, VectorNd init_EP_pos_3d, VectorNd goal_EP_pos_3d, VectorNd tmp_act_EP_local) {
    double b0, b1, b2, b3, b4, b5, b6;
    double b0_dot, b1_dot, b2_dot, b3_dot, b4_dot, b5_dot, b6_dot;

    double t_bez;
    double alpha = 1.2;
    double tmp_h_foot;
    double h_foot;
    double tmp_EP_z_size;

    double tmp_tsp_time_HS = tsp_time_HS;

    VectorNd bezi_pos(3);
    VectorNd bezi_vel(3);
    VectorNd bezi_val(6);

    VectorNd Dist_2d_global(2);
    VectorNd now_vel_global(2);

    //Dist = now_vel_HS(0) * step_time_HS * 4.0;

    now_vel_global = target_C_WB_Y_2d_HS * now_vel_HS.segment(0, 2);
    Dist_2d_global = now_vel_global * step_time_HS * 4.0;

    contact_num = Contact_info(0) + Contact_info(1) + Contact_info(2) + Contact_info(3);

    if (contact_num == 3) {
        if (cnt_bez_HS == 0) {
            tmp_EP_z_size = abs(tmp_act_EP_local(2)) / cos(abs(target_base_ori_local_HS(1)));

//            if (now_vel_HS(2) > 0) {
//                if (Contact_info(0) == 0 || Contact_info(2) == 0) {
//                    tmp_h_foot = foot_height_HS / 1.2;
//                }
//                if (Contact_info(1) == 0 || Contact_info(3) == 0) {
//                    tmp_h_foot = foot_height_HS + 0.05;
//                }
//            } else if (now_vel_HS(2) < 0) {
//                if (Contact_info(0) == 0 || Contact_info(2) == 0) {
//                    tmp_h_foot = foot_height_HS + 0.05;
//                }
//                if (Contact_info(1) == 0 || Contact_info(3) == 0) {
//                    tmp_h_foot = foot_height_HS / 1.2;
//                }
//            } else {
                tmp_h_foot = foot_height_HS + 0.05;
          //  }

            if (tmp_EP_z_size > 0.4) {
                h_foot = tmp_h_foot;
            } else {
                h_foot = tmp_EP_z_size / 3.0;
            }

            //                h_foot = tmp_h_foot;

            P0 << init_EP_pos_3d(0), init_EP_pos_3d(1), init_EP_pos_3d(2);
            P1 << init_EP_pos_3d(0), init_EP_pos_3d(1), init_EP_pos_3d(2);


            P2 << init_EP_pos_3d(0) - Dist_2d_global(0) / 2.0, init_EP_pos_3d(1) - Dist_2d_global(1) / 2.0, init_EP_pos_3d(2) + h_foot*alpha;
            P3 << (init_EP_pos_3d(0) + (goal_EP_pos_3d(0) - init_EP_pos_3d(0))*2.0 / 4.0), (init_EP_pos_3d(1) + (goal_EP_pos_3d(1) - init_EP_pos_3d(1))*2.0 / 4.0), init_EP_pos_3d(2) + h_foot;
            //P4 << goal_EP_pos_3d(0) + Dist_2d_global(0) / 2.0, goal_EP_pos_3d(1) + Dist_2d_global(1) / 2.0, init_EP_pos_3d(2) + h_foot*alpha;
            P4 << goal_EP_pos_3d(0), goal_EP_pos_3d(1), init_EP_pos_3d(2) + h_foot*alpha;

            //            P5 << goal_EP_pos_3d(0), goal_EP_pos_3d(1), goal_EP_pos_3d(2);
            //            P6 << goal_EP_pos_3d(0), goal_EP_pos_3d(1), goal_EP_pos_3d(2);
            //            P5 << goal_EP_pos_3d(0), goal_EP_pos_3d(1), init_EP_pos_3d(2) + Dist_2d_global(0) * tan(-target_base_ori_local_HS(1));
            //            P6 << goal_EP_pos_3d(0), goal_EP_pos_3d(1), init_EP_pos_3d(2) + Dist_2d_global(0) * tan(-target_base_ori_local_HS(1));
            //            P5 << goal_EP_pos_3d(0), goal_EP_pos_3d(1), init_EP_pos_3d(2) + Dist_2d_global(0) * tan(-actual_base_ori_local_HS(1));
            //            P6 << goal_EP_pos_3d(0), goal_EP_pos_3d(1), init_EP_pos_3d(2) + Dist_2d_global(0) * tan(-actual_base_ori_local_HS(1));
            P5 << goal_EP_pos_3d(0), goal_EP_pos_3d(1), init_EP_pos_3d(2) - 2 * Dist_2d_global(0) * tan(abs(target_base_ori_local_HS(1)));
            P6 << goal_EP_pos_3d(0), goal_EP_pos_3d(1), init_EP_pos_3d(2) - 2 * Dist_2d_global(0) * tan(abs(target_base_ori_local_HS(1)));

            bezi_pos = P0;
            bezi_vel << 0.0, 0.0, 0.0;
            cnt_bez_HS++;
        } else if (cnt_bez_HS < tsp_cnt_HS) {
            t_bez = (cnt_bez_HS) * dt;

            if (cnt_bez_HS < tsp_cnt_HS / 2.0) {
                Leg_state(Leg_state(0)) = 1;
            } else {
                Leg_state(Leg_state(0)) = -1;
            }

            b0 = pow((1 - t_bez / (tmp_tsp_time_HS)), 6);
            b1 = 6 * pow((1 - t_bez / (tmp_tsp_time_HS)), 5) * pow((t_bez / (tmp_tsp_time_HS)), 1);
            b2 = 15 * pow((1 - t_bez / (tmp_tsp_time_HS)), 4) * pow((t_bez / (tmp_tsp_time_HS)), 2);
            b3 = 20 * pow((1 - t_bez / (tmp_tsp_time_HS)), 3) * pow((t_bez / (tmp_tsp_time_HS)), 3);
            b4 = 15 * pow((1 - t_bez / (tmp_tsp_time_HS)), 2) * pow((t_bez / (tmp_tsp_time_HS)), 4);
            b5 = 6 * pow((1 - t_bez / (tmp_tsp_time_HS)), 1) * pow((t_bez / (tmp_tsp_time_HS)), 5);
            b6 = pow((t_bez / (tmp_tsp_time_HS)), 6);

            b0_dot = 6 * pow((1 - t_bez / (tmp_tsp_time_HS)), 5)*(-1 / (tmp_tsp_time_HS));
            b1_dot = 30 * pow((1 - t_bez / (tmp_tsp_time_HS)), 4)*(-1 / (tmp_tsp_time_HS)) * pow((t_bez / (tmp_tsp_time_HS)), 1) + 6 * pow((1 - t_bez / (tmp_tsp_time_HS)), 5)*(1 / (tmp_tsp_time_HS));
            b2_dot = 60 * pow((1 - t_bez / (tmp_tsp_time_HS)), 3)*(-1 / (tmp_tsp_time_HS)) * pow((t_bez / (tmp_tsp_time_HS)), 2) + 30 * pow((1 - t_bez / (tmp_tsp_time_HS)), 4) * pow((t_bez / (tmp_tsp_time_HS)), 1)*(1 / (tmp_tsp_time_HS));
            b3_dot = 60 * pow((1 - t_bez / (tmp_tsp_time_HS)), 2)*(-1 / (tmp_tsp_time_HS)) * pow((t_bez / (tmp_tsp_time_HS)), 3) + 60 * pow((1 - t_bez / (tmp_tsp_time_HS)), 3) * pow((t_bez / (tmp_tsp_time_HS)), 2)*(1 / (tmp_tsp_time_HS));
            b4_dot = 30 * pow((1 - t_bez / (tmp_tsp_time_HS)), 1)*(-1 / (tmp_tsp_time_HS)) * pow((t_bez / (tmp_tsp_time_HS)), 4) + 60 * pow((1 - t_bez / (tmp_tsp_time_HS)), 2) * pow((t_bez / (tmp_tsp_time_HS)), 3)*(1 / (tmp_tsp_time_HS));
            b5_dot = 6 * (-1 / (tmp_tsp_time_HS)) * pow((t_bez / (tmp_tsp_time_HS)), 5) + 30 * pow((1 - t_bez / (tmp_tsp_time_HS)), 1) * pow((t_bez / (tmp_tsp_time_HS)), 4)*(1 / (tmp_tsp_time_HS));
            b6_dot = 6 * pow((t_bez / (tmp_tsp_time_HS)), 5)*(1 / (tmp_tsp_time_HS));

            bezi_pos = P0 * b0 + P1 * b1 + P2 * b2 + P3 * b3 + P4 * b4 + P5 * b5 + P6*b6;
            bezi_vel = P0 * b0_dot + P1 * b1_dot + P2 * b2_dot + P3 * b3_dot + P4 * b4_dot + P5 * b5_dot + P6*b6_dot;
            cnt_bez_HS++;


            if (cnt_bez_HS == tsp_cnt_HS) {
                cnt_bez_HS = 0;
                //bezi_pos = P4;
                bezi_vel << 0.0, 0.0, 0.0;
                Leg_state(Leg_state(0)) = 0;
            }
        }
        bezi_val << bezi_pos, bezi_vel;
    } else {
        P0 << init_EP_pos_3d(0), init_EP_pos_3d(1), init_EP_pos_3d(2);
        bezi_pos = P0;
        bezi_vel << 0, 0, 0;
        bezi_val << bezi_pos, bezi_vel;
    }
    return bezi_val;
}

void CRobot::Fly_Leg_Gain_Controller4(unsigned int _i) {

    VectorNd init_kp_EP_global(3);
    VectorNd init_kd_EP_global(3);
    VectorNd goal_kp_EP_global(3);
    VectorNd goal_kd_EP_global(3);

    VectorNd tmp_init_kp_EP_local(3);
    VectorNd tmp_init_kd_EP_local(3);
    VectorNd tmp_goal_kp_EP_local(3);
    VectorNd tmp_goal_kd_EP_local(3);

    VectorNd init_kp_EP_local(3);
    VectorNd init_kd_EP_local(3);
    VectorNd goal_kp_EP_local(3);
    VectorNd goal_kd_EP_local(3);

    MatrixNd tmp_R_pitch(3, 3);

    unsigned int tmp_fly_cnt;
    unsigned int interval1 = tsp_cnt_HS * 0.5;
    unsigned int interval2 = interval1 + tsp_cnt_HS * 0.5;

    init_kp_EP_global << goal_kp_EP_HS(0), goal_kp_EP_HS(1), goal_kp_EP_HS(2);
    init_kd_EP_global << goal_kd_EP_HS(0), goal_kd_EP_HS(1), goal_kd_EP_HS(2);
    goal_kp_EP_global << goal_kp_EP_HS(0), goal_kp_EP_HS(1), 0;
    goal_kd_EP_global << goal_kd_EP_HS(0), goal_kd_EP_HS(1), goal_kd_EP_HS(2);

    //    tmp_R_pitch << cos(abs(target_base_ori_local_HS(1))), 0, sin(abs(target_base_ori_local_HS(1)))\
//                 , 0, 1, 0\
//, sin(abs(target_base_ori_local_HS(1))), 0, cos(abs(target_base_ori_local_HS(1)));


    //    tmp_init_kp_EP_local = target_C_WB_P_HS*init_kp_EP_global;
    //    tmp_init_kd_EP_local = target_C_WB_P_HS*init_kd_EP_global;
    //    tmp_goal_kp_EP_local = target_C_WB_P_HS*goal_kp_EP_global;
    //    tmp_goal_kd_EP_local = target_C_WB_P_HS*goal_kd_EP_global;
    //
    //    init_kp_EP_local(0) = abs(tmp_init_kp_EP_local(0));
    //    init_kp_EP_local(1) = abs(tmp_init_kp_EP_local(1));
    //    init_kp_EP_local(2) = abs(tmp_init_kp_EP_local(2));
    //
    //    init_kd_EP_local(0) = abs(tmp_init_kd_EP_local(0));
    //    init_kd_EP_local(1) = abs(tmp_init_kd_EP_local(1));
    //    init_kd_EP_local(2) = abs(tmp_init_kd_EP_local(2));
    //
    //    goal_kp_EP_local(0) = abs(tmp_goal_kp_EP_local(0));
    //    goal_kp_EP_local(1) = abs(tmp_goal_kp_EP_local(1));
    //    goal_kp_EP_local(2) = abs(tmp_goal_kp_EP_local(2));
    //
    //    goal_kd_EP_local(0) = abs(tmp_goal_kd_EP_local(0));
    //    goal_kd_EP_local(1) = abs(tmp_goal_kd_EP_local(1));
    //    goal_kd_EP_local(2) = abs(tmp_goal_kd_EP_local(2));

    init_kp_EP_local = init_kp_EP_global;
    init_kd_EP_local = init_kd_EP_global;
    goal_kp_EP_local = goal_kp_EP_global;
    goal_kd_EP_local = goal_kd_EP_global;

    if (walk_stop_flag_HS == false) {
        if (_i < tsp_cnt_HS) {
            if (_i == 0) {
                z4_gain_flag = false;
                z2_gain_flag = true;
            }
            if (_i < tsp_cnt_HS * (0.5)) {
                if (_i == tsp_cnt_HS * (0.5) - 1) {

                }
            } else {


            }
        } else if (_i < step_cnt_HS) {


        } else if (_i < step_cnt_HS + tsp_cnt_HS) {
            if (_i == step_cnt_HS * 1) {
                z2_gain_flag = false;
                z1_gain_flag = true;
            }
            if (_i < step_cnt_HS + tsp_cnt_HS * (0.5)) {
                if (_i == step_cnt_HS + tsp_cnt_HS * (0.5) - 1) {

                }
            } else {

            }
        } else if (_i < step_cnt_HS * 2) {


        } else if (_i < step_cnt_HS * 2 + tsp_cnt_HS) {
            if (_i == step_cnt_HS * 2) {
                z1_gain_flag = false;
                z3_gain_flag = true;
            }

            if (_i < step_cnt_HS * 2 + tsp_cnt_HS * (0.5)) {
                if (_i == step_cnt_HS * 2 + tsp_cnt_HS * (0.5) - 1) {

                }
            } else {

            }
        } else if (_i < step_cnt_HS * 3) {

        } else if (_i < step_cnt_HS * 3 + tsp_cnt_HS) {
            if (_i == step_cnt_HS * 3) {
                z3_gain_flag = false;
                z4_gain_flag = true;
            }
            if (_i < step_cnt_HS * 3 + tsp_cnt_HS * (0.5)) {
                if (_i == step_cnt_HS * 3 + tsp_cnt_HS * (0.5) - 1) {

                }
            } else {

            }
        } else {

        }

        if (z1_gain_flag == true) {
            if (fly_cnt_HS1 == 0) {
                kp_EP_HS.segment(0, 3) = init_kp_EP_local;
                kd_EP_HS.segment(0, 3) = init_kd_EP_local;
            } else if (fly_cnt_HS1 < interval1) {
                tmp_fly_cnt = fly_cnt_HS1;
                kp_EP_HS.segment(0, 3) = init_kp_EP_local;
                kd_EP_HS.segment(0, 3) = init_kd_EP_local;
            } else if (fly_cnt_HS1 < interval2) {
                tmp_fly_cnt = fly_cnt_HS1 - interval1;
                kp_EP_HS.segment(0, 3) = init_kp_EP_local + (goal_kp_EP_local - init_kp_EP_local) / 2.0 * (1 - cos(PI / (interval1) * tmp_fly_cnt));
                kd_EP_HS.segment(0, 3) = init_kd_EP_local + (goal_kd_EP_local - init_kd_EP_local) / 2.0 * (1 - cos(PI / (interval1) * tmp_fly_cnt));
            } else {
                kp_EP_HS.segment(0, 3) = goal_kp_EP_local;
                kd_EP_HS.segment(0, 3) = goal_kd_EP_local;
            }
            fly_cnt_HS1++;
        } else {
            kp_EP_HS.segment(0, 3) = goal_kp_EP_local;
            kd_EP_HS.segment(0, 3) = init_kd_EP_local;
            fly_cnt_HS1 = 0;
        }

        if (z2_gain_flag == true) {
            if (fly_cnt_HS2 == 0) {
                kp_EP_HS.segment(3, 3) = init_kp_EP_local;
                kd_EP_HS.segment(3, 3) = init_kd_EP_local;
            } else if (fly_cnt_HS2 < interval1) {
                tmp_fly_cnt = fly_cnt_HS2;
                kp_EP_HS.segment(3, 3) = init_kp_EP_local;
                kd_EP_HS.segment(3, 3) = init_kd_EP_local;
            } else if (fly_cnt_HS2 < interval2) {
                tmp_fly_cnt = fly_cnt_HS2 - interval1;
                kp_EP_HS.segment(3, 3) = init_kp_EP_local + (goal_kp_EP_local - init_kp_EP_local) / 2.0 * (1 - cos(PI / (interval1) * tmp_fly_cnt));
                kd_EP_HS.segment(3, 3) = init_kd_EP_local + (goal_kd_EP_local - init_kd_EP_local) / 2.0 * (1 - cos(PI / (interval1) * tmp_fly_cnt));
            } else {
                kp_EP_HS.segment(3, 3) = goal_kp_EP_local;
                kd_EP_HS.segment(3, 3) = goal_kd_EP_local;
            }
            fly_cnt_HS2++;
        } else {
            kp_EP_HS.segment(3, 3) = goal_kp_EP_local;
            kd_EP_HS.segment(3, 3) = init_kd_EP_local;
            fly_cnt_HS2 = 0;
        }

        if (z3_gain_flag == true) {
            if (fly_cnt_HS3 == 0) {
                kp_EP_HS.segment(6, 3) = init_kp_EP_local;
                kd_EP_HS.segment(6, 3) = init_kd_EP_local;
            } else if (fly_cnt_HS3 < interval1) {
                tmp_fly_cnt = fly_cnt_HS3;
                kp_EP_HS.segment(6, 3) = init_kp_EP_local;
                kd_EP_HS.segment(6, 3) = init_kd_EP_local;
            } else if (fly_cnt_HS3 < interval2) {
                tmp_fly_cnt = fly_cnt_HS3 - interval1;
                kp_EP_HS.segment(6, 3) = init_kp_EP_local + (goal_kp_EP_local - init_kp_EP_local) / 2.0 * (1 - cos(PI / (interval1) * tmp_fly_cnt));
                kd_EP_HS.segment(6, 3) = init_kd_EP_local + (goal_kd_EP_local - init_kd_EP_local) / 2.0 * (1 - cos(PI / (interval1) * tmp_fly_cnt));
            } else {
                kp_EP_HS.segment(6, 3) = goal_kp_EP_local;
                kd_EP_HS.segment(6, 3) = goal_kd_EP_local;
            }
            fly_cnt_HS3++;
        } else {
            kp_EP_HS.segment(6, 3) = goal_kp_EP_local;
            kd_EP_HS.segment(6, 3) = init_kd_EP_local;
            fly_cnt_HS3 = 0;
        }

        if (z4_gain_flag == true) {
            if (fly_cnt_HS4 == 0) {
                kp_EP_HS.segment(9, 3) = init_kp_EP_local;
                kd_EP_HS.segment(9, 3) = init_kd_EP_local;
            } else if (fly_cnt_HS4 < interval1) {
                tmp_fly_cnt = fly_cnt_HS4;
                kp_EP_HS.segment(9, 3) = init_kp_EP_local;
                kd_EP_HS.segment(9, 3) = init_kd_EP_local;
            } else if (fly_cnt_HS4 < interval2) {
                tmp_fly_cnt = fly_cnt_HS4 - interval1;
                kp_EP_HS.segment(9, 3) = init_kp_EP_local + (goal_kp_EP_local - init_kp_EP_local) / 2.0 * (1 - cos(PI / (interval1) * tmp_fly_cnt));
                kd_EP_HS.segment(9, 3) = init_kd_EP_local + (goal_kd_EP_local - init_kd_EP_local) / 2.0 * (1 - cos(PI / (interval1) * tmp_fly_cnt));
            } else {
                kp_EP_HS.segment(9, 3) = goal_kp_EP_local;
                kd_EP_HS.segment(9, 3) = goal_kd_EP_local;
            }
            fly_cnt_HS4++;
        } else {
            kp_EP_HS.segment(9, 3) = goal_kp_EP_local;
            kd_EP_HS.segment(9, 3) = init_kd_EP_local;
            fly_cnt_HS4 = 0;
        }
    } else {
        kp_EP_HS << init_kp_EP_local, init_kp_EP_local, init_kp_EP_local, init_kp_EP_local;
        kd_EP_HS << init_kd_EP_local, init_kd_EP_local, init_kd_EP_local, init_kd_EP_local;
    }
}

void CRobot::COM_XY_Traj_Gen_COM_VER_HS3(VectorNd _init_com_pos, VectorNd _goal_com_pos) {

    double tmp_zmp_x_ref;
    double tmp_zmp_y_ref;

    VectorNd del_com_pos_2d(2);
    VectorNd tmp_zmp_ref(2);
    VectorNd init_com_pos_2d(2);
    VectorNd goal_com_pos_2d(2);
    VectorNd swing_com_x_2d_global(2);
    VectorNd swing_com_y_2d_global(2);

    double tmp_target_yaw = 0.0;
    if (cnt_preview_HS == 0) {
        if (com_stop_flag_HS == true) {
            swing_dist_x = 0.0;
            swing_dist_y = 0.0;
        } else {
            //            swing_dist_x = 0.05 * cos(target_base_ori_local_HS(1));
            swing_dist_x = 0.05;
            //swing_dist_x = 0.0;
            swing_dist_y = 0.08;
            //swing_dist_y = 0.08;
        }
    }

    //    init_com_pos_2d << _init_com_pos(0), _init_com_pos(1);
    //    goal_com_pos_2d << _goal_com_pos(0), _goal_com_pos(1);
    tmp_target_yaw = init_yaw_HS + (goal_yaw_HS - init_yaw_HS) / 2.0 * (1 - cos(PI / (step_cnt_HS * 4) * cnt_preview_HS));

    swing_com_x_2d_global << swing_dist_x * cos(tmp_target_yaw), swing_dist_x * sin(tmp_target_yaw);
    swing_com_y_2d_global << -swing_dist_y * sin(tmp_target_yaw), swing_dist_y * cos(tmp_target_yaw);

    if (cnt_preview_HS <= tsp_cnt_HS) {//RR Swing
        //        del_com_pos_2d = (goal_com_pos_2d - init_com_pos_2d)*0.0 / 8.0 + swing_com_x_2d_global + swing_com_y_2d_global;
        //        tmp_zmp_ref = init_com_pos_2d + del_com_pos_2d;
        //tmp_zmp_ref = (init_EP_HS.segment(0,2)+init_EP_HS.segment(6,2)+init_EP_HS.segment(9,2))/3.0;
        tmp_zmp_ref = (init_EP_HS.segment(0, 2) + init_EP_HS.segment(9, 2)) / 2.0 + swing_com_x_2d_global + swing_com_y_2d_global;

        tmp_zmp_x_ref = tmp_zmp_ref(0);
        tmp_zmp_y_ref = tmp_zmp_ref(1);

    } else if (cnt_preview_HS <= step_cnt_HS) {//RR Stance
        //        del_com_pos_2d = (goal_com_pos_2d - init_com_pos_2d)*2.0 / 8.0 + swing_com_x_2d_global - swing_com_y_2d_global;
        //        tmp_zmp_ref = init_com_pos_2d + del_com_pos_2d;
        //tmp_zmp_ref = (goal_EP_HS.segment(3,2)+init_EP_HS.segment(6,2)+init_EP_HS.segment(9,2))/3.0;
        tmp_zmp_ref = (goal_EP_HS.segment(3, 2) + init_EP_HS.segment(6, 2)) / 2.0 + swing_com_x_2d_global - swing_com_y_2d_global;

        tmp_zmp_x_ref = tmp_zmp_ref(0);
        tmp_zmp_y_ref = tmp_zmp_ref(1);

    } else if (cnt_preview_HS <= step_cnt_HS + tsp_cnt_HS) {//RL Swing
        //        del_com_pos_2d = (goal_com_pos_2d - init_com_pos_2d)*2.0 / 8.0 + swing_com_x_2d_global - swing_com_y_2d_global;
        //        tmp_zmp_ref = init_com_pos_2d + del_com_pos_2d;
        //tmp_zmp_ref = (goal_EP_HS.segment(3,2)+init_EP_HS.segment(6,2)+init_EP_HS.segment(9,2))/3.0;
        tmp_zmp_ref = (goal_EP_HS.segment(3, 2) + init_EP_HS.segment(6, 2)) / 2.0 + swing_com_x_2d_global - swing_com_y_2d_global;

        tmp_zmp_x_ref = tmp_zmp_ref(0);
        tmp_zmp_y_ref = tmp_zmp_ref(1);

    } else if (cnt_preview_HS <= step_cnt_HS * 2) {//RL Stance
        //        del_com_pos_2d = (goal_com_pos_2d - init_com_pos_2d)*4.0 / 8.0 - swing_com_x_2d_global - swing_com_y_2d_global;
        //        tmp_zmp_ref = init_com_pos_2d + del_com_pos_2d;
        //tmp_zmp_ref = (goal_EP_HS.segment(0,2)+goal_EP_HS.segment(3,2)+init_EP_HS.segment(9,2))/3.0;
        tmp_zmp_ref = (goal_EP_HS.segment(0, 2) + init_EP_HS.segment(9, 2)) / 2.0 - swing_com_x_2d_global - swing_com_y_2d_global;

        tmp_zmp_x_ref = tmp_zmp_ref(0);
        tmp_zmp_y_ref = tmp_zmp_ref(1);

    } else if (cnt_preview_HS <= step_cnt_HS * 2 + tsp_cnt_HS) {//FL Swing
        //        del_com_pos_2d = (goal_com_pos_2d - init_com_pos_2d)*4.0 / 8.0 - swing_com_x_2d_global - swing_com_y_2d_global;
        //        tmp_zmp_ref = init_com_pos_2d + del_com_pos_2d;
        //tmp_zmp_ref = (goal_EP_HS.segment(0,2)+goal_EP_HS.segment(3,2)+init_EP_HS.segment(9,2))/3.0;
        tmp_zmp_ref = (goal_EP_HS.segment(0, 2) + init_EP_HS.segment(9, 2)) / 2.0 - swing_com_x_2d_global - swing_com_y_2d_global;

        tmp_zmp_x_ref = tmp_zmp_ref(0);
        tmp_zmp_y_ref = tmp_zmp_ref(1);

    } else if (cnt_preview_HS <= step_cnt_HS * 3) {// FL Stance
        //        del_com_pos_2d = (goal_com_pos_2d - init_com_pos_2d)*6.0 / 8.0 - swing_com_x_2d_global + swing_com_y_2d_global;
        //        tmp_zmp_ref = init_com_pos_2d + del_com_pos_2d;
        //tmp_zmp_ref = (goal_EP_HS.segment(0,2)+goal_EP_HS.segment(3,2)+goal_EP_HS.segment(6,2))/3.0;
        tmp_zmp_ref = (goal_EP_HS.segment(3, 2) + goal_EP_HS.segment(6, 2)) / 2.0 - swing_com_x_2d_global + swing_com_y_2d_global;

        tmp_zmp_x_ref = tmp_zmp_ref(0);
        tmp_zmp_y_ref = tmp_zmp_ref(1);

    } else if (cnt_preview_HS <= step_cnt_HS * 3 + tsp_cnt_HS) {//FR Swing
        //        del_com_pos_2d = (goal_com_pos_2d - init_com_pos_2d)*6.0 / 8.0 - swing_com_x_2d_global + swing_com_y_2d_global;
        //        tmp_zmp_ref = init_com_pos_2d + del_com_pos_2d;
        //tmp_zmp_ref = (goal_EP_HS.segment(0,2)+goal_EP_HS.segment(3,2)+goal_EP_HS.segment(6,2))/3.0;
        tmp_zmp_ref = (goal_EP_HS.segment(3, 2) + goal_EP_HS.segment(6, 2)) / 2.0 - swing_com_x_2d_global + swing_com_y_2d_global;

        tmp_zmp_x_ref = tmp_zmp_ref(0);
        tmp_zmp_y_ref = tmp_zmp_ref(1);

    } else if (cnt_preview_HS <= step_cnt_HS * 4) {//FR Stance
        //        del_com_pos_2d = (goal_com_pos_2d - init_com_pos_2d)*8.0 / 8.0 + swing_com_x_2d_global + swing_com_y_2d_global;
        //        tmp_zmp_ref = init_com_pos_2d + del_com_pos_2d;
        //tmp_zmp_ref = (goal_EP_HS.segment(0,2)+goal_EP_HS.segment(6,2)+goal_EP_HS.segment(9,2))/3.0;
        tmp_zmp_ref = (goal_EP_HS.segment(0, 2) + goal_EP_HS.segment(9, 2)) / 2.0 + swing_com_x_2d_global + swing_com_y_2d_global;

        tmp_zmp_x_ref = tmp_zmp_ref(0);
        tmp_zmp_y_ref = tmp_zmp_ref(1);

    } else {
        //        del_com_pos_2d = (goal_com_pos_2d - init_com_pos_2d)*8.0 / 8.0 + swing_com_x_2d_global + swing_com_y_2d_global;
        //        tmp_zmp_ref = init_com_pos_2d + del_com_pos_2d;
        //tmp_zmp_ref = (goal_EP_HS.segment(0,2)+goal_EP_HS.segment(6,2)+goal_EP_HS.segment(9,2))/3.0;
        tmp_zmp_ref = (goal_EP_HS.segment(0, 2) + goal_EP_HS.segment(9, 2)) / 2.0 + swing_com_x_2d_global + swing_com_y_2d_global;

        tmp_zmp_x_ref = tmp_zmp_ref(0);
        tmp_zmp_y_ref = tmp_zmp_ref(1);
    }

    for (unsigned int j = 0; j <= preview_cnt_HS - 2; ++j) {
        zmp_ref_x_array_HS(j) = zmp_ref_x_array_HS(j + 1);
        zmp_ref_y_array_HS(j) = zmp_ref_y_array_HS(j + 1);
    }

    zmp_ref_x_array_HS(preview_cnt_HS - 1) = tmp_zmp_x_ref;
    zmp_ref_y_array_HS(preview_cnt_HS - 1) = tmp_zmp_y_ref;

    Preview_con_HS(); //This returns COM traj.

    target_com_pos_HS(0) = X_new_x_HS(0);
    target_com_pos_HS(1) = X_new_y_HS(0);

    target_com_vel_HS(0) = X_new_x_HS(1);
    target_com_vel_HS(1) = X_new_y_HS(1);

    target_com_acc_HS(0) = X_new_x_HS(2);
    target_com_acc_HS(1) = X_new_y_HS(2);

    target_base_pos_HS = Get_Base_pos_HS2(target_com_pos_HS);

    cnt_preview_HS++;

    if (cnt_preview_HS == step_cnt_HS * 4.0) {
        cnt_preview_HS = 0;
    }
}

void CRobot::Preview_con_HS() {
    static double err_x, err_y, err_force;
    static double sum_p_x, sum_p_y, sum_p_force;

    static double u_x, u_y, u_force;
    static double zmp_ref_x_old, zmp_ref_y_old, force_ref_old;

    zmp_ref_x_old = CC_HS.transpose() * X_new_x_HS;
    zmp_ref_y_old = CC_HS.transpose() * X_new_y_HS;

    err_x = zmp_ref_x_old - zmp_ref_x_array_HS(0);
    err_y = zmp_ref_y_old - zmp_ref_y_array_HS(0);

    sum_e_x_HS = sum_e_x_HS + err_x;
    sum_e_y_HS = sum_e_y_HS + err_y;

    sum_p_x = 0.0;
    sum_p_y = 0.0;

    for (unsigned int j = 0; j < preview_cnt_HS - 1; ++j) {
        sum_p_x = sum_p_x + Gp_HS(j) * zmp_ref_x_array_HS(j);
        sum_p_y = sum_p_y + Gp_HS(j) * zmp_ref_y_array_HS(j);
    }

    u_x = -Gi_HS * sum_e_x_HS - Gx_HS.transpose() * X_new_x_HS - sum_p_x;
    u_y = -Gi_HS * sum_e_y_HS - Gx_HS.transpose() * X_new_y_HS - sum_p_y;

    X_new_x_HS = AA_HS * X_new_x_HS + BB_HS*u_x;
    X_new_y_HS = AA_HS * X_new_y_HS + BB_HS*u_y;
}

void CRobot::Get_gain_HS(void) {
    double z_c = com_height_HS;
    int nCount = 0;
    double temp_Gp_gain, temp_Gx_gain, temp_Gi_gain;

    AA_HS << 1, dt, dt * dt / 2.0f,
            0, 1, dt,
            0, 0, 1;

    BB_HS << dt * dt * dt / 6.0f, dt * dt / 2.0f, dt;

    CC_HS << 1, 0, -z_c / GRAVITY;

    FILE *fp1;
    FILE *fp2;
    FILE *fp3;

    fp1 = fopen("/home/hyunseok/catkin_ws/src/RcLab-PongBotQ/src/gain/WC3S/Gp.txt", "r");
    if (fp1 == NULL)printf("CAN NOT OPEN Gp TEXT FILE \n");
    while (fscanf(fp1, "%lf", &temp_Gp_gain) == 1) {
        pv_Gp_HS[nCount] = temp_Gp_gain;
        nCount++;
    }
    fclose(fp1);
    nCount = 0;

    fp2 = fopen("/home/hyunseok/catkin_ws/src/RcLab-PongBotQ/src/gain/WC3S/Gx.txt", "r");
    if (fp2 == NULL)printf("CAN NOT OPEN Gx TEXT FILE \n");
    while (fscanf(fp2, "%lf", &temp_Gx_gain) == 1) {
        pv_Gx_HS[nCount] = temp_Gx_gain;
        nCount++;
    }
    fclose(fp2);
    nCount = 0;

    fp3 = fopen("/home/hyunseok/catkin_ws/src/RcLab-PongBotQ/src/gain/WC3S/Gi.txt", "r");
    if (fp3 == NULL)printf("CAN NOT OPEN Gi TEXT FILE \n");
    while (fscanf(fp3, "%lf", &temp_Gi_gain) == 1) {
        pv_Gi_HS[nCount] = temp_Gi_gain;
        nCount++;
    }
    fclose(fp3);

    Gi_HS = pv_Gi_HS[0];

    Gx_HS(0) = pv_Gx_HS[0];
    Gx_HS(1) = pv_Gx_HS[1];
    Gx_HS(2) = pv_Gx_HS[2];

    for (unsigned int i = 0; i < preview_cnt_HS - 1; ++i) {
        Gp_HS(i) = pv_Gp_HS[i];
    }
}

void CRobot::ComputeTorqueControl_HS(void) {
    VectorNd tmp_target_EP_vel = VectorNd::Zero(19);
    VectorNd tmp_target_joint_vel = VectorNd::Zero(19);
    VectorNd tmp_actual_vel = VectorNd::Zero(19);
    VectorNd tmp_actual_EP_vel = VectorNd::Zero(19);

    CalcPointJacobian6D(*m_pModel, RobotState, base.ID, Originbase, J_BASE, true);
    CalcPointJacobian6D(*m_pModel, RobotState, front_body.ID, Originbase, J_FRONT_BODY, true);
    CalcPointJacobian(*m_pModel, RobotState, RL.ID, EP_OFFSET_RL, J_RL, true);
    CalcPointJacobian(*m_pModel, RobotState, RR.ID, EP_OFFSET_RR, J_RR, true);
    CalcPointJacobian(*m_pModel, RobotState, FL.ID, EP_OFFSET_FL, J_FL, true);
    CalcPointJacobian(*m_pModel, RobotState, FR.ID, EP_OFFSET_FR, J_FR, true);

    J_A.block(0, 0, 6, 19) = J_BASE;
    J_A.block(6, 0, 1, 19) = J_FRONT_BODY.block(2, 0, 1, 19); // only yaw
    J_A.block(7, 0, 3, 19) = J_RL;
    J_A.block(10, 0, 3, 19) = J_RR;
    J_A.block(13, 0, 3, 19) = J_FL;
    J_A.block(16, 0, 3, 19) = J_FR;

    //*********** Actual value*************//
    tmp_actual_vel << 0, 0, 0, 0, 0, 0, actual_vel[6], actual_vel[0], actual_vel[1], actual_vel[2], actual_vel[3], actual_vel[4], actual_vel[5], actual_vel[6], actual_vel[7], actual_vel[8], actual_vel[9], actual_vel[10], actual_vel[11];
    tmp_actual_EP_vel = J_A*tmp_actual_vel;
    //****************Not use**********************//

    CompositeRigidBodyAlgorithm(*m_pModel, RobotState, M_term, true);
    NonlinearEffects(*m_pModel, RobotState, RobotStatedot, hatNonLinearEffects);
    NonlinearEffects(*m_pModel, RobotState, VectorNd::Zero(m_pModel->dof_count), G_term);
    C_term = hatNonLinearEffects - G_term;

    //Task_Space_Controller();
    Task_Space_Controller3();

    //Get_Opt_F_HS();
    //Get_Opt_F_HS2();
    Get_Opt_F_HS3();
    CTC_Torque = C_term + G_term + J_A.transpose() * Task_Control_value_HS - J_A.transpose() * OSQP_Control_value_HS;
    

     //QuadPP_SOLVE_TEST();
    //CTC_Torque = C_term + G_term + J_A.transpose() * Task_Control_value_HS - J_A.transpose() * QUADPROGPP_Control_value_HS;

    for (int nJoint = 0; nJoint < nDOF; nJoint++) {
        joint[nJoint].torque = CTC_Torque(6 + nJoint);
    }
    initial_flag_HS = false;
    std::cout << "----End-----" << std::endl;
}

void CRobot::Task_Space_Controller(void) {
    VectorNd target_base_pos_12d_HS(12);
    VectorNd target_base_vel_12d_HS(12);
    VectorNd EP_Control_value(12);
    VectorNd tmp_F_task_HS(12);
    Task_Gain_Setting_HS();

    target_base_pos_12d_HS << target_base_pos_HS, target_base_pos_HS, target_base_pos_HS, target_base_pos_HS;
    target_EP_local_HS = target_C_WB_YPR_12d_HS.transpose()*(target_EP_HS - target_base_pos_12d_HS);

    target_base_vel_HS = target_com_vel_HS;
    target_base_vel_12d_HS << target_base_vel_HS, target_base_vel_HS, target_base_vel_HS, target_base_vel_HS;
    target_EP_vel_local_HS = target_C_WB_YPR_12d_HS.transpose()*(target_EP_vel_HS - target_base_vel_12d_HS);

    J_A.block(0, 0, 6, 19) = J_BASE;
    J_A.block(6, 0, 1, 19) = J_FRONT_BODY.block(2, 0, 1, 19); // only yaw
    J_A.block(7, 0, 3, 19) = J_RL;
    J_A.block(10, 0, 3, 19) = J_RR;
    J_A.block(13, 0, 3, 19) = J_FL;
    J_A.block(16, 0, 3, 19) = J_FR;

    J_RL2 << J_RL.block(0, 6, 3, 3);
    J_RR2 << J_RR.block(0, 9, 3, 3);
    J_FL2 << J_FL.block(0, 13, 3, 3);
    J_FR2 << J_FR.block(0, 16, 3, 3);

    act_RL_q_dot << actual_vel[0], actual_vel[1], actual_vel[2];
    act_RR_q_dot << actual_vel[3], actual_vel[4], actual_vel[5];
    act_FL_q_dot << actual_vel[7], actual_vel[8], actual_vel[9];
    act_FR_q_dot << actual_vel[10], actual_vel[11], actual_vel[12];

    act_RL_foot_vel = J_RL2*act_RL_q_dot;
    act_RR_foot_vel = J_RR2*act_RR_q_dot;
    act_FL_foot_vel = J_FL2*act_FL_q_dot;
    act_FR_foot_vel = J_FR2*act_FR_q_dot;

    actual_EP_vel_local_HS << act_RL_foot_vel, act_RR_foot_vel, act_FL_foot_vel, act_FR_foot_vel;

    EP_pos_local_err_HS = target_EP_local_HS - actual_EP_local_HS;
    EP_vel_local_err_HS = target_EP_vel_local_HS - actual_EP_vel_local_HS;

    if (Leg_state(0) != 0) {
        std::cout << "kp_EP=" << kp_EP_HS((Leg_state(0) - 1)*3 + 0) << "," << kp_EP_HS((Leg_state(0) - 1)*3 + 1) << "," << kp_EP_HS((Leg_state(0) - 1)*3 + 2) << std::endl;
        std::cout << "kd_EP=" << kd_EP_HS((Leg_state(0) - 1)*3 + 0) << "," << kd_EP_HS((Leg_state(0) - 1)*3 + 1) << "," << kd_EP_HS((Leg_state(0) - 1)*3 + 2) << std::endl;
        //        std::cout << "kp_Base=" << kp_Base_HS((Leg_state(0) - 1)*3 + 0) << "," << kp_Base_HS((Leg_state(0) - 1)*3 + 1) << "," << kp_Base_HS((Leg_state(0) - 1)*3 + 2) << std::endl;
        //        std::cout << "kd_Base=" << kd_Base_HS((Leg_state(0) - 1)*3 + 0) << "," << kd_Base_HS((Leg_state(0) - 1)*3 + 1) << "," << kd_Base_HS((Leg_state(0) - 1)*3 + 2) << std::endl;
        std::cout << "---" << std::endl;
        std::cout << "EP(d)=" << target_EP_local_HS((Leg_state(0) - 1)*3 + 0) << "," << target_EP_local_HS((Leg_state(0) - 1)*3 + 1) << "," << target_EP_local_HS((Leg_state(0) - 1)*3 + 2) << std::endl;
        std::cout << "EP(a)=" << actual_EP_local_HS((Leg_state(0) - 1)*3 + 0) << "," << actual_EP_local_HS((Leg_state(0) - 1)*3 + 1) << "," << actual_EP_local_HS((Leg_state(0) - 1)*3 + 2) << std::endl;
        std::cout << "EP_vel(d)=" << target_EP_vel_local_HS((Leg_state(0) - 1)*3 + 0) << "," << target_EP_vel_local_HS((Leg_state(0) - 1)*3 + 1) << "," << target_EP_vel_local_HS((Leg_state(0) - 1)*3 + 2) << std::endl;
        std::cout << "EP_vel(a)=" << actual_EP_vel_local_HS((Leg_state(0) - 1)*3 + 0) << "," << actual_EP_vel_local_HS((Leg_state(0) - 1)*3 + 1) << "," << actual_EP_vel_local_HS((Leg_state(0) - 1)*3 + 2) << std::endl;
        //        std::cout << "Base_d=" << target_base_pos_HS.transpose() << std::endl;
        //        std::cout << "Base_a=" << actual_base_pos_HS.transpose() << std::endl;
        std::cout << "---" << std::endl;
        std::cout << "EP_pos_err=" << EP_pos_local_err_HS((Leg_state(0) - 1)*3 + 0) << "," << EP_pos_local_err_HS((Leg_state(0) - 1)*3 + 1) << "," << EP_pos_local_err_HS((Leg_state(0) - 1)*3 + 2) << std::endl;
        std::cout << "EP_vel_err=" << EP_vel_local_err_HS((Leg_state(0) - 1)*3 + 0) << "," << EP_vel_local_err_HS((Leg_state(0) - 1)*3 + 1) << "," << EP_vel_local_err_HS((Leg_state(0) - 1)*3 + 2) << std::endl;
        //        std::cout << "base_pos_err=" << Base_pos_err((Leg_state(0) - 1)*3 + 0) << "," << Base_pos_err((Leg_state(0) - 1)*3 + 1) << "," << Base_pos_err((Leg_state(0) - 1)*3 + 2) << std::endl;
        //        std::cout << "base_vel_err=" << Base_vel_err((Leg_state(0) - 1)*3 + 0) << "," << Base_vel_err((Leg_state(0) - 1)*3 + 1) << "," << Base_vel_err((Leg_state(0) - 1)*3 + 2) << std::endl;
    }

    for (int i = 0; i < 12; ++i) {
        EP_Control_value[i] = kp_EP_HS[i]*(EP_pos_local_err_HS[i]) + kd_EP_HS[i]*(EP_vel_local_err_HS[i]);
    }

    tmp_F_task_HS = target_C_WB_PR_12d_HS*EP_Control_value;
    if (Leg_state(0) != 0) {
        std::cout << "F(motion)=" << tmp_F_task_HS((Leg_state(0) - 1)*3 + 0) << "," << tmp_F_task_HS((Leg_state(0) - 1)*3 + 1) << "," << tmp_F_task_HS((Leg_state(0) - 1)*3 + 2) << std::endl;
    }


    Task_Control_value_HS << 0, 0, 0, 0, 0, 0, 0, EP_Control_value[0], EP_Control_value[1], EP_Control_value[2], EP_Control_value[3], EP_Control_value[4], EP_Control_value[5], EP_Control_value[6], EP_Control_value[7], EP_Control_value[8], EP_Control_value[9], EP_Control_value[10], EP_Control_value[11];

    //    Get_Opt_F_HS();
    Get_Opt_F_HS2();


    std::cout << "----End-----" << std::endl;
}

void CRobot::Task_Space_Controller2(void) {

    VectorNd EP_Control_value(12);
    VectorNd tmp_F_task_global_HS(12);
    Task_Gain_Setting_HS();

    actual_base_pos_12d_HS << actual_base_pos_HS, actual_base_pos_HS, actual_base_pos_HS, actual_base_pos_HS;
    target_base_pos_12d_HS << target_base_pos_HS, target_base_pos_HS, target_base_pos_HS, target_base_pos_HS;

    actual_base_vel_12d_HS << actual_base_vel_HS, actual_base_vel_HS, actual_base_vel_HS, actual_base_vel_HS;

    target_base_vel_HS = target_com_vel_HS;
    target_base_vel_12d_HS << target_base_vel_HS, target_base_vel_HS, target_base_vel_HS, target_base_vel_HS;

    EP_pos_err = target_EP_HS - actual_EP_HS;
    //    std::cout << "EP_d=" << target_EP_HS.transpose() << std::endl;
    //    std::cout << "EP_a=" << actual_EP_HS.transpose() << std::endl;
    //    std::cout << "------------" << std::endl;
    //    
    //    std::cout << "Base_d=" << target_base_pos_HS.transpose() << std::endl;
    //    std::cout << "Base_a=" << actual_base_pos_HS.transpose() << std::endl;
    //    std::cout << "------------" << std::endl;


    EP_vel_err = target_EP_vel_HS - actual_EP_vel_HS;

    Base_pos_err = actual_base_pos_12d_HS - target_base_pos_12d_HS;
    Base_vel_err = actual_base_vel_12d_HS - target_base_vel_12d_HS;


    for (int i = 0; i < 12; i++) {
        F_task_global_HS(i) = kp_EP_HS(i) * (EP_pos_err(i)) + kd_EP_HS(i) * (EP_vel_err(i)) + kp_Base_HS(i) * (Base_pos_err(i)) + kd_Base_HS(i) * (Base_vel_err(i));
    }

    tmp_F_task_global_HS << F_task_global_HS(0), F_task_global_HS(1), (1 - Contact_Info_HS(0)) * F_task_global_HS(2), F_task_global_HS(3), F_task_global_HS(4), (1 - Contact_Info_HS(1)) * F_task_global_HS(5), F_task_global_HS(6), F_task_global_HS(7), (1 - Contact_Info_HS(2)) * F_task_global_HS(8), F_task_global_HS(9), F_task_global_HS(10), (1 - Contact_Info_HS(3)) * F_task_global_HS(11);

    if (Leg_state(0) != 0) {
        std::cout << "kp_EP=" << kp_EP_HS((Leg_state(0) - 1)*3 + 0) << "," << kp_EP_HS((Leg_state(0) - 1)*3 + 1) << "," << kp_EP_HS((Leg_state(0) - 1)*3 + 2) << std::endl;
        std::cout << "kd_EP=" << kd_EP_HS((Leg_state(0) - 1)*3 + 0) << "," << kd_EP_HS((Leg_state(0) - 1)*3 + 1) << "," << kd_EP_HS((Leg_state(0) - 1)*3 + 2) << std::endl;
        std::cout << "kp_Base=" << kp_Base_HS((Leg_state(0) - 1)*3 + 0) << "," << kp_Base_HS((Leg_state(0) - 1)*3 + 1) << "," << kp_Base_HS((Leg_state(0) - 1)*3 + 2) << std::endl;
        std::cout << "kd_Base=" << kd_Base_HS((Leg_state(0) - 1)*3 + 0) << "," << kd_Base_HS((Leg_state(0) - 1)*3 + 1) << "," << kd_Base_HS((Leg_state(0) - 1)*3 + 2) << std::endl;
        std::cout << "---" << std::endl;
        std::cout << "EP_d=" << target_EP_HS((Leg_state(0) - 1)*3 + 0) << "," << target_EP_HS((Leg_state(0) - 1)*3 + 1) << "," << target_EP_HS((Leg_state(0) - 1)*3 + 2) << std::endl;
        std::cout << "EP_a=" << actual_EP_HS((Leg_state(0) - 1)*3 + 0) << "," << actual_EP_HS((Leg_state(0) - 1)*3 + 1) << "," << actual_EP_HS((Leg_state(0) - 1)*3 + 2) << std::endl;
        std::cout << "Base_d=" << target_base_pos_HS.transpose() << std::endl;
        std::cout << "Base_a=" << actual_base_pos_HS.transpose() << std::endl;
        std::cout << "---" << std::endl;
        std::cout << "EP_pos_err=" << EP_pos_err((Leg_state(0) - 1)*3 + 0) << "," << EP_pos_err((Leg_state(0) - 1)*3 + 1) << "," << EP_pos_err((Leg_state(0) - 1)*3 + 2) << std::endl;
        std::cout << "EP_vel_err=" << EP_vel_err((Leg_state(0) - 1)*3 + 0) << "," << EP_vel_err((Leg_state(0) - 1)*3 + 1) << "," << EP_vel_err((Leg_state(0) - 1)*3 + 2) << std::endl;
        std::cout << "base_pos_err=" << Base_pos_err((Leg_state(0) - 1)*3 + 0) << "," << Base_pos_err((Leg_state(0) - 1)*3 + 1) << "," << Base_pos_err((Leg_state(0) - 1)*3 + 2) << std::endl;
        std::cout << "base_vel_err=" << Base_vel_err((Leg_state(0) - 1)*3 + 0) << "," << Base_vel_err((Leg_state(0) - 1)*3 + 1) << "," << Base_vel_err((Leg_state(0) - 1)*3 + 2) << std::endl;
    }

    F_task_local_HS = target_C_WB_YPR_12d_HS.transpose() * tmp_F_task_global_HS;

    for (int i = 0; i < 12; ++i) {
        EP_Control_value[i] = F_task_local_HS[i];
    }
    if (Leg_state(0) != 0) {
        std::cout << "F(motion)=" << F_task_local_HS((Leg_state(0) - 1)*3 + 0) << "," << F_task_local_HS((Leg_state(0) - 1)*3 + 1) << "," << F_task_local_HS((Leg_state(0) - 1)*3 + 2) << std::endl;
    }
    std::cout << "---" << std::endl;
    Task_Control_value_HS << 0, 0, 0, 0, 0, 0, 0, EP_Control_value[0], EP_Control_value[1], EP_Control_value[2], EP_Control_value[3], EP_Control_value[4], EP_Control_value[5], EP_Control_value[6], EP_Control_value[7], EP_Control_value[8], EP_Control_value[9], EP_Control_value[10], EP_Control_value[11];

    //    Get_Opt_F_HS();
    Get_Opt_F_HS2();

}

void CRobot::Task_Space_Controller3(void) {

    VectorNd target_base_pos_12d_HS(12);
    VectorNd target_base_vel_12d_HS(12);
    VectorNd EP_Control_value(12);
    VectorNd tmp_F_task_HS(12);

    VectorNd global_target_EP_local_HS(12);
    VectorNd global_target_EP_vel_local_HS(12);
    VectorNd global_actual_EP_local_HS(12);
    VectorNd global_actual_EP_vel_local_HS(12);
    VectorNd global_F_task_local_HS(12);

    Task_Gain_Setting_HS();

    actual_base_pos_12d_HS << actual_base_pos_HS, actual_base_pos_HS, actual_base_pos_HS, actual_base_pos_HS;
    target_base_pos_12d_HS << target_base_pos_HS, target_base_pos_HS, target_base_pos_HS, target_base_pos_HS;

    actual_base_vel_12d_HS << actual_base_vel_HS, actual_base_vel_HS, actual_base_vel_HS, actual_base_vel_HS;
    target_base_vel_HS = target_com_vel_HS;
    target_base_vel_12d_HS << target_base_vel_HS, target_base_vel_HS, target_base_vel_HS, target_base_vel_HS;

    global_target_EP_local_HS = target_EP_HS - target_base_pos_12d_HS;
    global_target_EP_vel_local_HS = target_EP_vel_HS - target_base_vel_12d_HS;

    J_A.block(0, 0, 6, 19) = J_BASE;
    J_A.block(6, 0, 1, 19) = J_FRONT_BODY.block(2, 0, 1, 19); // only yaw
    J_A.block(7, 0, 3, 19) = J_RL;
    J_A.block(10, 0, 3, 19) = J_RR;
    J_A.block(13, 0, 3, 19) = J_FL;
    J_A.block(16, 0, 3, 19) = J_FR;

    J_RL2 << J_RL.block(0, 6, 3, 3);
    J_RR2 << J_RR.block(0, 9, 3, 3);
    J_FL2 << J_FL.block(0, 13, 3, 3);
    J_FR2 << J_FR.block(0, 16, 3, 3);

    act_RL_q_dot << actual_vel[0], actual_vel[1], actual_vel[2];
    act_RR_q_dot << actual_vel[3], actual_vel[4], actual_vel[5];
    act_FL_q_dot << actual_vel[7], actual_vel[8], actual_vel[9];
    act_FR_q_dot << actual_vel[10], actual_vel[11], actual_vel[12];

    actual_EP_vel_local_HS.segment(0, 3) = J_RL2*act_RL_q_dot;
    actual_EP_vel_local_HS.segment(3, 3) = J_RR2*act_RR_q_dot;
    actual_EP_vel_local_HS.segment(6, 3) = J_FL2*act_FL_q_dot;
    actual_EP_vel_local_HS.segment(9, 3) = J_FR2*act_FR_q_dot;

    global_actual_EP_local_HS = actual_C_WB_YPR_12d_HS * actual_EP_local_HS;
    global_actual_EP_vel_local_HS = actual_C_WB_YPR_12d_HS * actual_EP_vel_local_HS;


    for (int i = 0; i < 12; i++) {
        global_F_task_local_HS(i) = kp_EP_HS(i)*(global_target_EP_local_HS(i) - global_actual_EP_local_HS(i)) + kd_EP_HS(i)*(global_target_EP_vel_local_HS(i) - global_actual_EP_vel_local_HS(i));
    }
    F_task_local_HS = target_C_WB_YPR_12d_HS.transpose() * global_F_task_local_HS;


    //    if (Leg_state(0) != 0) {
    //        std::cout << "kp_EP=" << kp_EP_HS((Leg_state(0) - 1)*3 + 0) << "," << kp_EP_HS((Leg_state(0) - 1)*3 + 1) << "," << kp_EP_HS((Leg_state(0) - 1)*3 + 2) << std::endl;
    //        std::cout << "kd_EP=" << kd_EP_HS((Leg_state(0) - 1)*3 + 0) << "," << kd_EP_HS((Leg_state(0) - 1)*3 + 1) << "," << kd_EP_HS((Leg_state(0) - 1)*3 + 2) << std::endl;
    //        //        std::cout << "kp_Base=" << kp_Base_HS((Leg_state(0) - 1)*3 + 0) << "," << kp_Base_HS((Leg_state(0) - 1)*3 + 1) << "," << kp_Base_HS((Leg_state(0) - 1)*3 + 2) << std::endl;
    //        //        std::cout << "kd_Base=" << kd_Base_HS((Leg_state(0) - 1)*3 + 0) << "," << kd_Base_HS((Leg_state(0) - 1)*3 + 1) << "," << kd_Base_HS((Leg_state(0) - 1)*3 + 2) << std::endl;
    //        std::cout << "---" << std::endl;
    //        std::cout << "EP(d)=" << target_EP_local_HS((Leg_state(0) - 1)*3 + 0) << "," << target_EP_local_HS((Leg_state(0) - 1)*3 + 1) << "," << target_EP_local_HS((Leg_state(0) - 1)*3 + 2) << std::endl;
    //        std::cout << "EP(a)=" << actual_EP_local_HS((Leg_state(0) - 1)*3 + 0) << "," << actual_EP_local_HS((Leg_state(0) - 1)*3 + 1) << "," << actual_EP_local_HS((Leg_state(0) - 1)*3 + 2) << std::endl;
    //        std::cout << "EP_vel(d)=" << target_EP_vel_local_HS((Leg_state(0) - 1)*3 + 0) << "," << target_EP_vel_local_HS((Leg_state(0) - 1)*3 + 1) << "," << target_EP_vel_local_HS((Leg_state(0) - 1)*3 + 2) << std::endl;
    //        std::cout << "EP_vel(a)=" << actual_EP_vel_local_HS((Leg_state(0) - 1)*3 + 0) << "," << actual_EP_vel_local_HS((Leg_state(0) - 1)*3 + 1) << "," << actual_EP_vel_local_HS((Leg_state(0) - 1)*3 + 2) << std::endl;
    //        //        std::cout << "Base_d=" << target_base_pos_HS.transpose() << std::endl;
    //        //        std::cout << "Base_a=" << actual_base_pos_HS.transpose() << std::endl;
    //        std::cout << "---" << std::endl;
    //        std::cout << "EP_pos_err=" << EP_pos_local_err_HS((Leg_state(0) - 1)*3 + 0) << "," << EP_pos_local_err_HS((Leg_state(0) - 1)*3 + 1) << "," << EP_pos_local_err_HS((Leg_state(0) - 1)*3 + 2) << std::endl;
    //        std::cout << "EP_vel_err=" << EP_vel_local_err_HS((Leg_state(0) - 1)*3 + 0) << "," << EP_vel_local_err_HS((Leg_state(0) - 1)*3 + 1) << "," << EP_vel_local_err_HS((Leg_state(0) - 1)*3 + 2) << std::endl;
    //        //        std::cout << "base_pos_err=" << Base_pos_err((Leg_state(0) - 1)*3 + 0) << "," << Base_pos_err((Leg_state(0) - 1)*3 + 1) << "," << Base_pos_err((Leg_state(0) - 1)*3 + 2) << std::endl;
    //        //        std::cout << "base_vel_err=" << Base_vel_err((Leg_state(0) - 1)*3 + 0) << "," << Base_vel_err((Leg_state(0) - 1)*3 + 1) << "," << Base_vel_err((Leg_state(0) - 1)*3 + 2) << std::endl;
    //    }

    for (int i = 0; i < 12; ++i) {
        EP_Control_value[i] = F_task_local_HS[i];
    }

    tmp_F_task_HS = target_C_WB_PR_12d_HS*EP_Control_value;
    //    if (Leg_state(0) != 0) {
    //        std::cout << "F(motion)=" << tmp_F_task_HS((Leg_state(0) - 1)*3 + 0) << "," << tmp_F_task_HS((Leg_state(0) - 1)*3 + 1) << "," << tmp_F_task_HS((Leg_state(0) - 1)*3 + 2) << std::endl;
    //    }


    Task_Control_value_HS << 0, 0, 0, 0, 0, 0, 0, EP_Control_value[0], EP_Control_value[1], EP_Control_value[2], EP_Control_value[3], EP_Control_value[4], EP_Control_value[5], EP_Control_value[6], EP_Control_value[7], EP_Control_value[8], EP_Control_value[9], EP_Control_value[10], EP_Control_value[11];

}

void CRobot::Task_Gain_Setting_HS(void) {
    double goal_time_force = 2.0;

    if (init_Force_flag_HS == true) {

        if (cnt_force_HS == 0) {
            init_kp_EP_HS = Kp_t;
            init_kd_EP_HS = Kd_t;
            //            goal_kp_EP_HS  << 6000, 6000, 3000, 6000, 6000, 3000, 6000, 6000, 3000, 6000, 6000, 3000;
            //            goal_kd_EP_HS << 150, 150, 100, 150, 150, 100, 150, 150, 100, 150, 150, 100; //Designed Gain

            //            goal_kp_EP_HS << 3000, 3000, 1500, 3000, 3000, 1500, 3000, 3000, 1500, 3000, 3000, 1500;
            //            goal_kd_EP_HS << 80, 80, 40, 80, 80, 40, 80, 80, 40, 80, 80, 40; //Designed Gain
            goal_kp_EP_HS << 2000, 2000, 1000, 2000, 2000, 1000, 2000, 2000, 1000, 2000, 2000, 1000;
            goal_kd_EP_HS << 70, 70, 50, 70, 70, 50, 70, 70, 50, 70, 70, 50; //Designed Gain

            target_kp_EP_HS = init_kp_EP_HS;
            target_kd_EP_HS = init_kd_EP_HS;

            init_kp_Base_HS = init_kp_EP_HS;
            init_kd_Base_HS = init_kd_EP_HS;
            goal_kp_Base_HS = goal_kp_EP_HS;
            goal_kd_Base_HS = goal_kd_EP_HS;
            target_kp_EP_HS = init_kp_Base_HS;
            target_kd_EP_HS = init_kd_Base_HS;

            cnt_force_HS++;

        } else if (cnt_force_HS < goal_time_force / dt) {
            target_kp_EP_HS = init_kp_EP_HS + (goal_kp_EP_HS - init_kp_EP_HS) / 2.0 * (1 - cos(PI / goal_time_force * cnt_force_HS * dt));
            target_kd_EP_HS = init_kd_EP_HS + (goal_kd_EP_HS - init_kd_EP_HS) / 2.0 * (1 - cos(PI / goal_time_force * cnt_force_HS * dt));

            target_kp_Base_HS = init_kp_Base_HS + (goal_kp_Base_HS - init_kp_Base_HS) / 2.0 * (1 - cos(PI / goal_time_force * cnt_force_HS * dt));
            target_kd_Base_HS = init_kd_Base_HS + (goal_kd_Base_HS - init_kd_Base_HS) / 2.0 * (1 - cos(PI / goal_time_force * cnt_force_HS * dt));


            cnt_force_HS++;
        } else {
            target_kp_EP_HS = goal_kp_EP_HS;
            target_kd_EP_HS = goal_kd_EP_HS;

            target_kp_Base_HS = goal_kp_Base_HS;
            target_kd_Base_HS = goal_kd_Base_HS;

            init_Force_flag_HS = false;
        }
        kp_EP_HS = target_kp_EP_HS;
        kd_EP_HS = target_kd_EP_HS;
        kp_Base_HS = target_kp_Base_HS;
        kd_Base_HS = target_kd_Base_HS;
    }
}

//void CRobot::Get_Opt_F_HS(void) {
//    double Fz_RL_max, Fz_RL_min;
//    double Fz_RR_max, Fz_RR_min;
//    double Fz_FL_max, Fz_FL_min;
//    double Fz_FR_max, Fz_FR_min;
//
//    double Rear_body_mass = 13.826;
//    double Front_body_mass = 10.507;
//    double One_leg_mass = 5.295;
//    double Robot_mass = Rear_body_mass + Front_body_mass + One_leg_mass * 4;
//    double Robot_Weight = Robot_mass*GRAVITY;
//
//    double one_leg_weight = 120;
//
//    MatrixNd A_BC(6, 12);
//    MatrixNd tmp_A_BC(12, 6);
//    VectorNd b_BC(6);
//    MatrixNd W(12, 12);
//
//    VectorNd r_RL(3);
//    VectorNd r_RR(3);
//    VectorNd r_FL(3);
//    VectorNd r_FR(3);
//    VectorNd r_COM(3);
//
//    VectorNd k_p_QP(6);
//    VectorNd k_d_QP(6);
//    double b1, b2, b3, b4, b5, b6;
//    //VectorNd semi_F_QP_global(12);
//    VectorNd F_QP_global(12);
//    VectorNd F_QP_local(12);
//
//    //double alpha = 1.0;
//    double alpha = 0.001;
//    double beta = 1.0;
//    //double beta = 2.0;
//    //    double tmp_mu = 0.8;
//    double tmp_mu = 1.0;
//    W = MatrixNd::Identity(12, 12);
//
//    VectorNd target_base_pos_rel(3);
//    VectorNd actual_base_pos_rel(3);
//    VectorNd target_com_pos_rel(3);
//    VectorNd actual_com_pos_rel(3);
//
//    VectorNd target_base_vel_rel(3);
//    VectorNd actual_base_vel_rel(3);
//
//    VectorNd Err_com_pos(3);
//    VectorNd Err_com_vel(3);
//
//    VectorNd standard_info_HS(4);
//    double standard_leg_height_HS;
//
//    VectorNd tmp_actual_plane_dist_vel(12);
//    double actual_plane_dist_vel;
//
//    TF_Global2Semi();
//
//    // k_p_QP << 1000, 1000, 5000, 20000, 20000, 0;
//    // k_d_QP << 10, 10, 30, 1000, 1000, 1000;
//    //    k_p_QP << 10, 10, 3000, 1000, 1000, 0;
//    //    k_d_QP << 0.1, 0.1, 30, 10, 10, 0;
//
//    k_p_QP << 10, 10, 4000, 3000, 3000, 0;
//    k_d_QP << 0.1, 0.1, 40, 30, 30, 10;
//
//    //k_p_QP << 1, 50, 1, 1000, 1000, 0;
//    //k_d_QP << 0.01, 5, 0.1, 10, 10, 0;
//
//    r_RL << semi_target_EP_HS_yaw(0), semi_target_EP_HS_yaw(1), semi_target_EP_HS_yaw(2);
//    r_RR << semi_target_EP_HS_yaw(3), semi_target_EP_HS_yaw(4), semi_target_EP_HS_yaw(5);
//    r_FL << semi_target_EP_HS_yaw(6), semi_target_EP_HS_yaw(7), semi_target_EP_HS_yaw(8);
//    r_FR << semi_target_EP_HS_yaw(9), semi_target_EP_HS_yaw(10), semi_target_EP_HS_yaw(11);
//
//    r_COM=semi_target_com_pos_HS_yaw;
//
//    A_BC << Contact_Info_HS(0), 0, 0, Contact_Info_HS(1), 0, 0, Contact_Info_HS(2), 0, 0, Contact_Info_HS(3), 0, 0\
//, 0, Contact_Info_HS(0), 0, 0, Contact_Info_HS(1), 0, 0, Contact_Info_HS(2), 0, 0, Contact_Info_HS(3), 0\
//, 0, 0, Contact_Info_HS(0), 0, 0, Contact_Info_HS(1), 0, 0, Contact_Info_HS(2), 0, 0, Contact_Info_HS(3)\
//          , 0, -Contact_Info_HS(0) * (r_RL(2) - r_COM(2)), Contact_Info_HS(0) * (r_RL(1) - r_COM(1)), 0, -Contact_Info_HS(1) * (r_RR(2) - r_COM(2)), Contact_Info_HS(1) * (r_RR(1) - r_COM(1)), 0, -Contact_Info_HS(2) * (r_FL(2) - r_COM(2)), Contact_Info_HS(2) * (r_FL(1) - r_COM(1)), 0, -Contact_Info_HS(3) * (r_FR(2) - r_COM(2)), Contact_Info_HS(3) * (r_FR(1) - r_COM(1))\
//          , Contact_Info_HS(0) * (r_RL(2) - r_COM(2)), 0, -Contact_Info_HS(0)*(r_RL(0) - r_COM(0)), Contact_Info_HS(1) * (r_RR(2) - r_COM(2)), 0, -Contact_Info_HS(1)*(r_RR(0) - r_COM(0)), Contact_Info_HS(2) * (r_FL(2) - r_COM(2)), 0, -Contact_Info_HS(2)*(r_FL(0) - r_COM(0)), Contact_Info_HS(3) * (r_FR(2) - r_COM(2)), 0, -Contact_Info_HS(3)*(r_FR(0) - r_COM(0))\
//          , -Contact_Info_HS(0) * (r_RL(1) - r_COM(1)), Contact_Info_HS(0) * (r_RL(0) - r_COM(0)), 0, -Contact_Info_HS(1) * (r_RR(1) - r_COM(1)), Contact_Info_HS(1)*(r_RR(0) - r_COM(0)), 0, -Contact_Info_HS(2) * (r_FL(1) - r_COM(1)), Contact_Info_HS(2)*(r_FL(0) - r_COM(0)), 0, -Contact_Info_HS(3) * (r_FR(1) - r_COM(1)), Contact_Info_HS(3)*(r_FR(0) - r_COM(0)), 0;
//
//    tmp_A_BC = A_BC.transpose();
//
//    target_base_pos_rel = -(Contact_Info_HS(0) * target_EP_local_HS.segment(0, 3) + Contact_Info_HS(1) * target_EP_local_HS.segment(3, 3) + Contact_Info_HS(2) * target_EP_local_HS.segment(6, 3) + Contact_Info_HS(3) * target_EP_local_HS.segment(9, 3)) / (Contact_Info_HS(0) + Contact_Info_HS(1) + Contact_Info_HS(2) + Contact_Info_HS(3));
//    actual_base_pos_rel = -(Contact_Info_HS(0) * actual_EP_local_HS.segment(0, 3) + Contact_Info_HS(1) * actual_EP_local_HS.segment(3, 3) + Contact_Info_HS(2) * actual_EP_local_HS.segment(6, 3) + Contact_Info_HS(3) * actual_EP_local_HS.segment(9, 3)) / (Contact_Info_HS(0) + Contact_Info_HS(1) + Contact_Info_HS(2) + Contact_Info_HS(3));
//
//    target_base_vel_rel = -(Contact_Info_HS(0) * target_EP_vel_local_HS.segment(0, 3) + Contact_Info_HS(1) * target_EP_vel_local_HS.segment(3, 3) + Contact_Info_HS(2) * target_EP_vel_local_HS.segment(6, 3) + Contact_Info_HS(3) * target_EP_vel_local_HS.segment(9, 3)) / (Contact_Info_HS(0) + Contact_Info_HS(1) + Contact_Info_HS(2) + Contact_Info_HS(3));
//    actual_base_vel_rel = -(Contact_Info_HS(0) * actual_EP_vel_local_HS.segment(0, 3) + Contact_Info_HS(1) * actual_EP_vel_local_HS.segment(3, 3) + Contact_Info_HS(2) * actual_EP_vel_local_HS.segment(6, 3) + Contact_Info_HS(3) * actual_EP_vel_local_HS.segment(9, 3)) / (Contact_Info_HS(0) + Contact_Info_HS(1) + Contact_Info_HS(2) + Contact_Info_HS(3));
//
//    target_com_pos_rel = Get_COM_pos_HS2(target_base_pos_rel);
//    actual_com_pos_rel = Get_COM_pos_HS2(actual_base_pos_rel);
//
//    Err_com_pos = target_C_WB_RP_HS * (target_com_pos_rel - actual_com_pos_rel);
//    Err_com_vel = target_C_WB_RP_HS * (target_base_vel_rel - actual_base_vel_rel);
//
//    target_plane_dist_HS = com_height_HS / cos(target_base_ori_HS(1));
//
//    standard_leg_height_HS = -((Contact_Info_HS(0) * (actual_EP_local_HS(2)) + Contact_Info_HS(1) * (actual_EP_local_HS(5)) + Contact_Info_HS(2) * (actual_EP_local_HS(8)) + Contact_Info_HS(3) * (actual_EP_local_HS(11)))) / (Contact_Info_HS(0) + Contact_Info_HS(1) + Contact_Info_HS(2) + Contact_Info_HS(3));
////    if (WalkReady_flag_HS == true && cnt_HS == 0) {
////        tmp_standard_leg_height_HS << standard_leg_height_HS, standard_leg_height_HS;
////    }
////    tmp_standard_leg_height_HS=Low_pass_Filter_HS(standard_leg_height_HS,tmp_standard_leg_height_HS(0),0.99);
//    actual_plane_dist_HS = (standard_leg_height_HS + offset_B2C(2)) / cos(target_base_ori_HS(1));
//    //actual_plane_dist_HS = (standard_leg_height_HS + offset_B2C(2)) / cos(actual_base_ori_local_HS(1));
//
//    //actual_plane_dist_HS = (actual_plane_dist + offset_B2C(2)) / cos(target_base_ori_local_HS(1));
//
//    tmp_actual_plane_dist_vel << -target_C_WB_RP_HS * actual_EP_vel_local_HS.segment(0, 3), -target_C_WB_RP_HS * actual_EP_vel_local_HS.segment(3, 3), -target_C_WB_RP_HS * actual_EP_vel_local_HS.segment(6, 3), -target_C_WB_RP_HS * actual_EP_vel_local_HS.segment(9, 3);
//    actual_plane_dist_vel = (-Contact_Info_HS(0) * (tmp_actual_plane_dist_vel(2)) - Contact_Info_HS(1) * (tmp_actual_plane_dist_vel(5)) - Contact_Info_HS(2) * (tmp_actual_plane_dist_vel(8)) - Contact_Info_HS(3) * (tmp_actual_plane_dist_vel(11))) / (Contact_Info_HS(0) + Contact_Info_HS(1) + Contact_Info_HS(2) + Contact_Info_HS(3));
//
//    b1 = Robot_mass * (semi_target_com_acc_HS_yaw(0)) + k_p_QP(0) * Err_com_pos(0) + k_d_QP(0) * Err_com_vel(0);
//    b2 = Robot_mass * (semi_target_com_acc_HS_yaw(1)) + k_p_QP(1) * Err_com_pos(1) + k_d_QP(1) * Err_com_vel(1);
//    //b3 = Robot_mass * (semi_target_com_acc_HS_yaw(2)) + k_p_QP(2)*(target_plane_dist_HS - actual_plane_dist_HS) + k_d_QP(2)*(0.0 - actual_plane_dist_vel) + Robot_Weight;
//    b3 = Robot_mass * (semi_target_com_acc_HS_yaw(2)) + k_p_QP(2)*(target_plane_dist_HS - actual_plane_dist_HS) + Robot_Weight;
//    std::cout<<"acc="<<semi_target_com_acc_HS_yaw.transpose()<<std::endl;
//    b4 = k_p_QP(3)*(target_base_ori_local_HS(0) - actual_base_ori_local_HS(0)) + k_d_QP(3)*(target_base_ori_vel_local_HS(0) - actual_base_ori_vel_local_HS(0));
//    b5 = k_p_QP(4)*(target_base_ori_local_HS(1) - actual_base_ori_local_HS(1)) + k_d_QP(4)*(target_base_ori_vel_local_HS(1) - actual_base_ori_vel_local_HS(1));
//    b6 = k_d_QP(5)*(target_base_ori_vel_local_HS(2) - actual_base_ori_vel_local_HS(2));
//
//
//    b_BC << b1, b2, b3, b4, b5, b6;
//
//    _P = (tmp_A_BC * A_BC + alpha * W);
//    _q = -(tmp_A_BC * b_BC);
//
//    // ===================== OSQP  ====================== //
//    int jj = 0;
//    int kk = 0;
//    int max_jj = 0;
//
//    // ===================== P_x ====================== //
//    for (unsigned int i = 0; i < P_nnz; ++i) {
//        P_x[i] = _P(jj, kk);
//        jj = jj + 1;
//
//        if (jj > max_jj) {
//            jj = 0;
//            kk = kk + 1;
//            max_jj = max_jj + 1;
//        }
//    }
//
//    for (unsigned int i = 0; i < A_nnz; ++i) {
//        q[i] = _q(i);
//    }
//
//    tmp_Friction1 = tmp_mu * Robot_Weight / 4.0 * cos(target_base_ori_local_HS(1));
//    tmp_Friction2 = tmp_mu * Robot_Weight / 4.0;
//
//    // ===================== Constraints ====================== //
//    if (Contact_Info_HS(0) == 0) {
//        _d_u(0) = 0.0;
//        _d_l(0) = 0.0;
//        _d_u(1) = 0.0;
//        _d_l(1) = 0.0;
//        _d_u(2) = 0.0;
//        _d_l(2) = 0.0;
//    } else {
//        //_d_u(2) = Robot_Weight * beta + tmp_Friction1 * sin(target_base_ori_local_HS(1));
//        _d_u(2) = Robot_Weight * beta;
//        //_d_l(2) = -tmp_Friction1 * sin(abs(target_base_ori_local_HS(1)));
//        _d_l(2) = 0.0;
//        if (target_base_ori_local_HS(1) < 0.0) {
//            _d_u(0) = 0.0;
//            //            _d_l(0) = -tmp_Friction1 * cos(target_base_ori_local_HS(1));
//            _d_l(0) = -tmp_Friction1;
//        } else if (target_base_ori_local_HS(1) > 0.0) {
//            //            _d_u(0) = tmp_Friction1 * cos(target_base_ori_local_HS(1));
//            _d_u(0) = tmp_Friction1;
//            _d_l(0) = 0.0;
//        } else {
//            _d_u(0) = tmp_Friction1 * cos(target_base_ori_local_HS(1));
//            _d_l(0) = -tmp_Friction1 * cos(target_base_ori_local_HS(1));
//        }
//
//        _d_u(1) = tmp_Friction2;
//        _d_l(1) = -tmp_Friction2;
//
//    }
//
//    if (Contact_Info_HS(1) == 0) {
//        _d_u(3) = 0.0;
//        _d_l(3) = 0.0;
//        _d_u(4) = 0.0;
//        _d_l(4) = 0.0;
//        _d_u(5) = 0.0;
//        _d_l(5) = 0.0;
//    } else {
//        //_d_u(5) = Robot_Weight * beta + tmp_Friction1 * sin(target_base_ori_local_HS(1));
//        _d_u(5) = Robot_Weight * beta;
//        //_d_l(5) = -tmp_Friction1 * sin(abs(target_base_ori_local_HS(1)));
//        _d_l(5) = 0.0;
//
//        if (target_base_ori_local_HS(1) < 0.0) {
//            _d_u(3) = 0.0;
//            //            _d_l(3) = -tmp_Friction1 * cos(target_base_ori_local_HS(1));
//            _d_l(3) = -tmp_Friction1;
//        } else if (target_base_ori_local_HS(1) > 0.0) {
//            //            _d_u(3) = tmp_Friction1 * cos(target_base_ori_local_HS(1));
//            _d_u(3) = tmp_Friction1;
//            _d_l(3) = 0.0;
//        } else {
//            _d_u(3) = tmp_Friction1 * cos(target_base_ori_local_HS(1));
//            _d_l(3) = -tmp_Friction1 * cos(target_base_ori_local_HS(1));
//
//        }
//
//        _d_u(4) = tmp_Friction2;
//        _d_l(4) = -tmp_Friction2;
//    }
//
//    if (Contact_Info_HS(2) == 0) {
//        _d_u(6) = 0.0;
//        _d_l(6) = 0.0;
//        _d_u(7) = 0.0;
//        _d_l(7) = 0.0;
//        _d_u(8) = 0.0;
//        _d_l(8) = 0.0;
//    } else {
//        //        _d_u(8) = Robot_Weight * beta + tmp_Friction1 * sin(target_base_ori_local_HS(1));
//        _d_u(8) = Robot_Weight * beta;
//        //_d_l(8) = -tmp_Friction1 * sin(abs(target_base_ori_local_HS(1)));
//        _d_l(8) = 0.0;
//
//        if (target_base_ori_local_HS(1) < 0.0) {
//            _d_u(6) = 0.0;
//            //_d_l(6) = -tmp_Friction1 * cos(target_base_ori_local_HS(1));
//            _d_l(6) = -tmp_Friction1;
//        } else if (target_base_ori_local_HS(1) > 0.0) {
//            //_d_u(6) = tmp_Friction1 * cos(target_base_ori_local_HS(1));
//            _d_u(6) = tmp_Friction1;
//            _d_l(6) = 0.0;
//        } else {
//            _d_u(6) = tmp_Friction1 * cos(target_base_ori_local_HS(1));
//            _d_l(6) = -tmp_Friction1 * cos(target_base_ori_local_HS(1));
//        }
//        _d_u(7) = tmp_Friction2;
//        _d_l(7) = -tmp_Friction2;
//    }
//
//
//    if (Contact_Info_HS(3) == 0) {
//        _d_u(9) = 0.0;
//        _d_l(9) = 0.0;
//        _d_u(10) = 0.0;
//        _d_l(10) = 0.0;
//        _d_u(11) = 0.0;
//        _d_l(11) = 0.0;
//    } else {
//        //_d_u(11) = Robot_Weight * beta + tmp_Friction1 * sin(target_base_ori_local_HS(1));
//        _d_u(11) = Robot_Weight * beta;
//        //_d_l(11) = -tmp_Friction1 * sin(abs(target_base_ori_local_HS(1)));
//        _d_l(11) = 0.0;
//
//        if (target_base_ori_local_HS(1) < 0.0) {
//            _d_u(9) = 0.0;
//            //_d_l(9) = -tmp_Friction1 * cos(target_base_ori_local_HS(1));
//            _d_l(9) = -tmp_Friction1;
//        } else if (target_base_ori_local_HS(1) > 0.0) {
//            //            _d_u(9) = tmp_Friction1 * cos(target_base_ori_local_HS(1));
//            _d_u(9) = tmp_Friction1;
//            _d_l(9) = 0.0;
//        } else {
//            _d_u(9) = tmp_Friction1 * cos(target_base_ori_local_HS(1));
//            _d_l(9) = -tmp_Friction1 * cos(target_base_ori_local_HS(1));
//        }
//        _d_u(10) = tmp_Friction2;
//        _d_l(10) = -tmp_Friction2;
//    }
//
//
//    for (unsigned int i = 0; i < A_nnz; ++i) {
//        l[i] = _d_l(i);
//        u[i] = _d_u(i);
//    }
//
//    osqp_update_P(QP_work, P_x, OSQP_NULL, 78);
//    osqp_update_lin_cost(QP_work, q);
//    osqp_update_bounds(QP_work, l, u);
//
//    // Solve updated problem
//    osqp_solve(QP_work);
//
//    for (unsigned int i = 0; i < A_nnz; ++i) {
//
//        semi_F_QP_global_yaw(i) = QP_work->solution->x[i];
//    }
//
//    target_C_WB_Y_12d_HS.block(0, 0, 3, 3) = target_C_WB_Y_HS;
//    target_C_WB_Y_12d_HS.block(3, 3, 3, 3) = target_C_WB_Y_HS;
//    target_C_WB_Y_12d_HS.block(6, 6, 3, 3) = target_C_WB_Y_HS;
//    target_C_WB_Y_12d_HS.block(9, 9, 3, 3) = target_C_WB_Y_HS;
//
//    F_QP_global = target_C_WB_Y_12d_HS * semi_F_QP_global_yaw;
//    F_QP_local = target_C_WB_12d_HS.transpose() * F_QP_global;
//    OSQP_Control_value_HS << 0, 0, 0, 0, 0, 0, 0, F_QP_local;
//    // std::cout <<"Force(N)="<< F_QP_local(2) + F_QP_local(5) + F_QP_local(8) + F_QP_local(11) << std::endl;
//    //std::cout<<"-------------------"<<std::endl;
//}

void CRobot::Get_Opt_F_HS2(void) {
    double Fz_RL_max, Fz_RL_min;
    double Fz_RR_max, Fz_RR_min;
    double Fz_FL_max, Fz_FL_min;
    double Fz_FR_max, Fz_FR_min;

    double Rear_body_mass = 13.826;
    double Front_body_mass = 10.507;
    double One_leg_mass = 5.295;
    double Robot_mass = Rear_body_mass + Front_body_mass + One_leg_mass * 4;
    double Robot_Weight = Robot_mass*GRAVITY;

    double one_leg_weight = 120;

    MatrixNd A_BC(6, 12);
    MatrixNd tmp_A_BC(12, 6);
    VectorNd b_BC(6);
    MatrixNd W(12, 12);

    VectorNd r_RL(3);
    VectorNd r_RR(3);
    VectorNd r_FL(3);
    VectorNd r_FR(3);
    VectorNd r_COM(3);

    VectorNd k_p_QP(6);
    VectorNd k_d_QP(6);
    double b1, b2, b3, b4, b5, b6;

    VectorNd F_QP_global(12);
    VectorNd F_QP_local(12);

    double alpha = 0.001;
    double beta = 1.0;
    double tmp_mu = 1.0;
    W = MatrixNd::Identity(12, 12);

    VectorNd target_base_pos_rel(3);
    VectorNd actual_base_pos_rel(3);
    VectorNd target_com_pos_rel(3);
    VectorNd actual_com_pos_rel(3);

    VectorNd target_base_vel_rel(3);
    VectorNd actual_base_vel_rel(3);

    VectorNd Err_com_pos(3);
    VectorNd Err_com_vel(3);

    VectorNd standard_info_HS(4);
    double standard_leg_height_HS;

    double N;

    VectorNd r_com_EP_local(12);
    VectorNd r_com_EP_semi_global(12);
    VectorNd r_com_RL(3);
    VectorNd r_com_RR(3);
    VectorNd r_com_FL(3);
    VectorNd r_com_FR(3);

    semi_target_com_acc_HS_yaw << target_C_WB_Y_2d_HS.transpose() * target_com_acc_HS.segment(0, 2), target_com_acc_HS(2);

    k_p_QP << 10, 10, 4000, 3000, 3000, 0;
    k_d_QP << 0.1, 0.1, 40, 30, 30, 10;

    r_com_EP_local << actual_EP_local_HS.segment(0, 3) - offset_B2C, actual_EP_local_HS.segment(3, 3) - offset_B2C, actual_EP_local_HS.segment(6, 3) - offset_B2C, actual_EP_local_HS.segment(9, 3) - offset_B2C;
    r_com_EP_semi_global = target_C_WB_PR_12d_HS*r_com_EP_local;

    r_com_RL << r_com_EP_semi_global(0), r_com_EP_semi_global(1), r_com_EP_semi_global(2);
    r_com_RR << r_com_EP_semi_global(3), r_com_EP_semi_global(4), r_com_EP_semi_global(5);
    r_com_FL << r_com_EP_semi_global(6), r_com_EP_semi_global(7), r_com_EP_semi_global(8);
    r_com_FR << r_com_EP_semi_global(9), r_com_EP_semi_global(10), r_com_EP_semi_global(11);


    A_BC << Contact_Info_HS(0), 0, 0, Contact_Info_HS(1), 0, 0, Contact_Info_HS(2), 0, 0, Contact_Info_HS(3), 0, 0\
, 0, Contact_Info_HS(0), 0, 0, Contact_Info_HS(1), 0, 0, Contact_Info_HS(2), 0, 0, Contact_Info_HS(3), 0\
, 0, 0, Contact_Info_HS(0), 0, 0, Contact_Info_HS(1), 0, 0, Contact_Info_HS(2), 0, 0, Contact_Info_HS(3)\
          , 0, -Contact_Info_HS(0) * (r_com_RL(2)), Contact_Info_HS(0) * (r_com_RL(1)), 0, -Contact_Info_HS(1) * (r_com_RR(2)), Contact_Info_HS(1) * (r_com_RR(1)), 0, -Contact_Info_HS(2) * (r_com_FL(2)), Contact_Info_HS(2) * (r_com_FL(1)), 0, -Contact_Info_HS(3) * (r_com_FR(2)), Contact_Info_HS(3) * (r_com_FR(1))\
          , Contact_Info_HS(0) * (r_com_RL(2)), 0, -Contact_Info_HS(0)*(r_com_RL(0)), Contact_Info_HS(1) * (r_com_RR(2)), 0, -Contact_Info_HS(1)*(r_com_RR(0)), Contact_Info_HS(2) * (r_com_FL(2)), 0, -Contact_Info_HS(2)*(r_com_FL(0)), Contact_Info_HS(3) * (r_com_FR(2)), 0, -Contact_Info_HS(3)*(r_com_FR(0))\
          , -Contact_Info_HS(0) * (r_com_RL(1)), Contact_Info_HS(0) * (r_com_RL(0)), 0, -Contact_Info_HS(1) * (r_com_RR(1)), Contact_Info_HS(1)*(r_com_RR(0)), 0, -Contact_Info_HS(2) * (r_com_FL(1)), Contact_Info_HS(2)*(r_com_FL(0)), 0, -Contact_Info_HS(3) * (r_com_FR(1)), Contact_Info_HS(3)*(r_com_FR(0)), 0;

    tmp_A_BC = A_BC.transpose();

    target_base_pos_rel = -(Contact_Info_HS(0) * target_EP_local_HS.segment(0, 3) + Contact_Info_HS(1) * target_EP_local_HS.segment(3, 3) + Contact_Info_HS(2) * target_EP_local_HS.segment(6, 3) + Contact_Info_HS(3) * target_EP_local_HS.segment(9, 3)) / (Contact_Info_HS(0) + Contact_Info_HS(1) + Contact_Info_HS(2) + Contact_Info_HS(3));
    actual_base_pos_rel = -(Contact_Info_HS(0) * actual_EP_local_HS.segment(0, 3) + Contact_Info_HS(1) * actual_EP_local_HS.segment(3, 3) + Contact_Info_HS(2) * actual_EP_local_HS.segment(6, 3) + Contact_Info_HS(3) * actual_EP_local_HS.segment(9, 3)) / (Contact_Info_HS(0) + Contact_Info_HS(1) + Contact_Info_HS(2) + Contact_Info_HS(3));

    target_base_vel_rel = -(Contact_Info_HS(0) * target_EP_vel_local_HS.segment(0, 3) + Contact_Info_HS(1) * target_EP_vel_local_HS.segment(3, 3) + Contact_Info_HS(2) * target_EP_vel_local_HS.segment(6, 3) + Contact_Info_HS(3) * target_EP_vel_local_HS.segment(9, 3)) / (Contact_Info_HS(0) + Contact_Info_HS(1) + Contact_Info_HS(2) + Contact_Info_HS(3));
    actual_base_vel_rel = -(Contact_Info_HS(0) * actual_EP_vel_local_HS.segment(0, 3) + Contact_Info_HS(1) * actual_EP_vel_local_HS.segment(3, 3) + Contact_Info_HS(2) * actual_EP_vel_local_HS.segment(6, 3) + Contact_Info_HS(3) * actual_EP_vel_local_HS.segment(9, 3)) / (Contact_Info_HS(0) + Contact_Info_HS(1) + Contact_Info_HS(2) + Contact_Info_HS(3));

    target_com_pos_rel = Get_COM_pos_HS2(target_base_pos_rel);
    actual_com_pos_rel = Get_COM_pos_HS2(actual_base_pos_rel);

    Err_com_pos = target_C_WB_PR_HS * (target_com_pos_rel - actual_com_pos_rel);
    Err_com_vel = target_C_WB_PR_HS * (target_base_vel_rel - actual_base_vel_rel);

    target_plane_dist_HS = com_height_HS / cos(abs(target_base_ori_local_HS(1)));

    //standard_leg_height_HS = -((Contact_Info_HS(0) * (actual_EP_local_HS(2)) + Contact_Info_HS(1) * (actual_EP_local_HS(5)) + Contact_Info_HS(2) * (actual_EP_local_HS(8)) + Contact_Info_HS(3) * (actual_EP_local_HS(11)))) / (Contact_Info_HS(0) + Contact_Info_HS(1) + Contact_Info_HS(2) + Contact_Info_HS(3));

    //    actual_plane_dist_HS = (standard_leg_height_HS + offset_B2C(2)) / cos(target_base_ori_HS(1));

    actual_plane_dist_HS = actual_base_pos_HS(2) + offset_B2C(2) / cos(abs(actual_base_ori_local_HS(1)));

    if (initial_flag_HS == true) {
        pre_actual_plane_dist_HS = actual_plane_dist_HS;
    }
    actual_plane_dist_vel_HS = (actual_plane_dist_HS - pre_actual_plane_dist_HS) / dt;
    pre_actual_plane_dist_HS = actual_plane_dist_HS;

    b1 = Robot_mass * (semi_target_com_acc_HS_yaw(0)) + k_p_QP(0) * Err_com_pos(0) + k_d_QP(0) * Err_com_vel(0);
    b2 = Robot_mass * (semi_target_com_acc_HS_yaw(1)) + k_p_QP(1) * Err_com_pos(1) + k_d_QP(1) * Err_com_vel(1);
    b3 = Robot_mass * (semi_target_com_acc_HS_yaw(2)) + k_p_QP(2)*(target_plane_dist_HS - actual_plane_dist_HS) + k_d_QP(2)*(0.0 - actual_plane_dist_vel_HS) + Robot_Weight;

    b4 = k_p_QP(3)*(target_base_ori_local_HS(0) - actual_base_ori_local_HS(0)) + k_d_QP(3)*(target_base_ori_vel_local_HS(0) - actual_base_ori_vel_local_HS(0));
    b5 = k_p_QP(4)*(target_base_ori_local_HS(1) - actual_base_ori_local_HS(1)) + k_d_QP(4)*(target_base_ori_vel_local_HS(1) - actual_base_ori_vel_local_HS(1));
    b6 = k_d_QP(5)*(target_base_ori_vel_local_HS(2) - actual_base_ori_vel_local_HS(2));


    b_BC << b1, b2, b3, b4, b5, b6;

    _P = (tmp_A_BC * A_BC + alpha * W);
    _q = -(tmp_A_BC * b_BC);

    // ===================== OSQP  ====================== //
    int jj = 0;
    int kk = 0;
    int max_jj = 0;

    // ===================== P_x ====================== //
    for (unsigned int i = 0; i < P_nnz; ++i) {
        P_x[i] = _P(jj, kk);
        jj = jj + 1;

        if (jj > max_jj) {
            jj = 0;
            kk = kk + 1;
            max_jj = max_jj + 1;
        }
    }

    for (unsigned int i = 0; i < A_nnz; ++i) {
        q[i] = _q(i);
    }
    //    N = Robot_Weight / 4.0 * cos(target_base_ori_local_HS(1));
    N = Robot_Weight / 4.0 * cos(actual_base_ori_local_HS(1));
    tmp_Friction1 = tmp_mu * N;
    tmp_Friction2 = tmp_mu * Robot_Weight / 4.0;

    // ===================== Constraints ====================== //
    //    if (Contact_Info_HS(0) == 0) {
    //        _d_u(0) = 0.0;
    //        _d_l(0) = 0.0;
    //        _d_u(1) = 0.0;
    //        _d_l(1) = 0.0;
    //        _d_u(2) = 0.0;
    //        _d_l(2) = 0.0;
    //    } else {
    //        _d_u(2) = Robot_Weight * beta + tmp_Friction1 * sin(target_base_ori_local_HS(1));
    //        _d_l(2) = 0.0;
    //
    //        if (target_base_ori_local_HS(1) < 0.0) {
    //            _d_u(0) = 0.0;
    //            _d_l(0) = -(tmp_Friction1 * cos(target_base_ori_local_HS(1)) - N * sin(target_base_ori_local_HS(1)));
    //        } else if (target_base_ori_local_HS(1) > 0.0) {
    //            _d_u(0) = (tmp_Friction1 * cos(target_base_ori_local_HS(1)) - N * sin(target_base_ori_local_HS(1)));
    //            _d_l(0) = 0.0;
    //        } else {
    //            _d_u(0) = tmp_Friction1 * cos(target_base_ori_local_HS(1));
    //            _d_l(0) = -tmp_Friction1 * cos(target_base_ori_local_HS(1));
    //        }
    //        _d_u(1) = tmp_Friction2;
    //        _d_l(1) = -tmp_Friction2;
    //
    //    }
    //
    //    if (Contact_Info_HS(1) == 0) {
    //        _d_u(3) = 0.0;
    //        _d_l(3) = 0.0;
    //        _d_u(4) = 0.0;
    //        _d_l(4) = 0.0;
    //        _d_u(5) = 0.0;
    //        _d_l(5) = 0.0;
    //    } else {
    //        _d_u(5) = Robot_Weight * beta + tmp_Friction1 * sin(target_base_ori_local_HS(1));
    //        _d_l(5) = 0.0;
    //
    //        if (target_base_ori_local_HS(1) < 0.0) {
    //            _d_u(3) = 0.0;
    //            _d_l(3) = -(tmp_Friction1 * cos(target_base_ori_local_HS(1)) - N * sin(target_base_ori_local_HS(1)));
    //
    //        } else if (target_base_ori_local_HS(1) > 0.0) {
    //            _d_u(3) = (tmp_Friction1 * cos(target_base_ori_local_HS(1)) - N * sin(target_base_ori_local_HS(1)));
    //            _d_l(3) = 0.0;
    //        } else {
    //            _d_u(3) = tmp_Friction1 * cos(target_base_ori_local_HS(1));
    //            _d_l(3) = -tmp_Friction1 * cos(target_base_ori_local_HS(1));
    //        }
    //
    //        _d_u(4) = tmp_Friction2;
    //        _d_l(4) = -tmp_Friction2;
    //    }
    //
    //    if (Contact_Info_HS(2) == 0) {
    //        _d_u(6) = 0.0;
    //        _d_l(6) = 0.0;
    //        _d_u(7) = 0.0;
    //        _d_l(7) = 0.0;
    //        _d_u(8) = 0.0;
    //        _d_l(8) = 0.0;
    //    } else {
    //        _d_u(8) = Robot_Weight * beta + tmp_Friction1 * sin(target_base_ori_local_HS(1));
    //        _d_l(8) = 0.0;
    //
    //        if (target_base_ori_local_HS(1) < 0.0) {
    //            _d_u(6) = 0.0;
    //            _d_l(6) = -(tmp_Friction1 * cos(target_base_ori_local_HS(1)) - N * sin(target_base_ori_local_HS(1)));
    //
    //        } else if (target_base_ori_local_HS(1) > 0.0) {
    //            _d_u(6) = (tmp_Friction1 * cos(target_base_ori_local_HS(1)) - N * sin(target_base_ori_local_HS(1)));
    //            _d_l(6) = 0.0;
    //        } else {
    //            _d_u(6) = tmp_Friction1 * cos(target_base_ori_local_HS(1));
    //            _d_l(6) = -tmp_Friction1 * cos(target_base_ori_local_HS(1));
    //        }
    //        _d_u(7) = tmp_Friction2;
    //        _d_l(7) = -tmp_Friction2;
    //    }
    //
    //
    //    if (Contact_Info_HS(3) == 0) {
    //        _d_u(9) = 0.0;
    //        _d_l(9) = 0.0;
    //        _d_u(10) = 0.0;
    //        _d_l(10) = 0.0;
    //        _d_u(11) = 0.0;
    //        _d_l(11) = 0.0;
    //    } else {
    //        _d_u(11) = Robot_Weight * beta + tmp_Friction1 * sin(target_base_ori_local_HS(1));
    //        _d_l(11) = 0.0;
    //
    //        if (target_base_ori_local_HS(1) < 0.0) {
    //            _d_u(9) = 0.0;
    //            _d_l(9) = -(tmp_Friction1 * cos(target_base_ori_local_HS(1)) - N * sin(target_base_ori_local_HS(1)));
    //
    //        } else if (target_base_ori_local_HS(1) > 0.0) {
    //            _d_u(9) = (tmp_Friction1 * cos(target_base_ori_local_HS(1)) - N * sin(target_base_ori_local_HS(1)));
    //            _d_l(9) = 0.0;
    //        } else {
    //            _d_u(9) = tmp_Friction1 * cos(target_base_ori_local_HS(1));
    //            _d_l(9) = -tmp_Friction1 * cos(target_base_ori_local_HS(1));
    //        }
    //        _d_u(10) = tmp_Friction2;
    //        _d_l(10) = -tmp_Friction2;
    //    }
    if (Contact_Info_HS(0) == 0) {
        _d_u(0) = 0.0;
        _d_l(0) = 0.0;
        _d_u(1) = 0.0;
        _d_l(1) = 0.0;
        _d_u(2) = 0.0;
        _d_l(2) = 0.0;
    } else {
        _d_u(2) = Robot_Weight * beta + tmp_Friction1 * sin(actual_base_ori_local_HS(1));
        _d_l(2) = 0.0;

        if (target_base_ori_local_HS(1) < 0.0) {
            _d_u(0) = 0.0;
            _d_l(0) = -(tmp_Friction1 * cos(actual_base_ori_local_HS(1)) - N * sin(actual_base_ori_local_HS(1)));
        } else if (target_base_ori_local_HS(1) > 0.0) {
            _d_u(0) = (tmp_Friction1 * cos(actual_base_ori_local_HS(1)) - N * sin(actual_base_ori_local_HS(1)));
            _d_l(0) = 0.0;
        } else {
            _d_u(0) = tmp_Friction1 * cos(actual_base_ori_local_HS(1));
            _d_l(0) = -tmp_Friction1 * cos(actual_base_ori_local_HS(1));
        }
        _d_u(1) = tmp_Friction2;
        _d_l(1) = -tmp_Friction2;

    }

    if (Contact_Info_HS(1) == 0) {
        _d_u(3) = 0.0;
        _d_l(3) = 0.0;
        _d_u(4) = 0.0;
        _d_l(4) = 0.0;
        _d_u(5) = 0.0;
        _d_l(5) = 0.0;
    } else {
        _d_u(5) = Robot_Weight * beta + tmp_Friction1 * sin(actual_base_ori_local_HS(1));
        _d_l(5) = 0.0;

        if (target_base_ori_local_HS(1) < 0.0) {
            _d_u(3) = 0.0;
            _d_l(3) = -(tmp_Friction1 * cos(actual_base_ori_local_HS(1)) - N * sin(actual_base_ori_local_HS(1)));

        } else if (target_base_ori_local_HS(1) > 0.0) {
            _d_u(3) = (tmp_Friction1 * cos(actual_base_ori_local_HS(1)) - N * sin(actual_base_ori_local_HS(1)));
            _d_l(3) = 0.0;
        } else {
            _d_u(3) = tmp_Friction1 * cos(actual_base_ori_local_HS(1));
            _d_l(3) = -tmp_Friction1 * cos(actual_base_ori_local_HS(1));
        }

        _d_u(4) = tmp_Friction2;
        _d_l(4) = -tmp_Friction2;
    }

    if (Contact_Info_HS(2) == 0) {
        _d_u(6) = 0.0;
        _d_l(6) = 0.0;
        _d_u(7) = 0.0;
        _d_l(7) = 0.0;
        _d_u(8) = 0.0;
        _d_l(8) = 0.0;
    } else {
        _d_u(8) = Robot_Weight * beta + tmp_Friction1 * sin(actual_base_ori_local_HS(1));
        _d_l(8) = 0.0;

        if (target_base_ori_local_HS(1) < 0.0) {
            _d_u(6) = 0.0;
            _d_l(6) = -(tmp_Friction1 * cos(actual_base_ori_local_HS(1)) - N * sin(actual_base_ori_local_HS(1)));

        } else if (target_base_ori_local_HS(1) > 0.0) {
            _d_u(6) = (tmp_Friction1 * cos(actual_base_ori_local_HS(1)) - N * sin(actual_base_ori_local_HS(1)));
            _d_l(6) = 0.0;
        } else {
            _d_u(6) = tmp_Friction1 * cos(actual_base_ori_local_HS(1));
            _d_l(6) = -tmp_Friction1 * cos(actual_base_ori_local_HS(1));
        }
        _d_u(7) = tmp_Friction2;
        _d_l(7) = -tmp_Friction2;
    }


    if (Contact_Info_HS(3) == 0) {
        _d_u(9) = 0.0;
        _d_l(9) = 0.0;
        _d_u(10) = 0.0;
        _d_l(10) = 0.0;
        _d_u(11) = 0.0;
        _d_l(11) = 0.0;
    } else {
        _d_u(11) = Robot_Weight * beta + tmp_Friction1 * sin(actual_base_ori_local_HS(1));
        _d_l(11) = 0.0;

        if (target_base_ori_local_HS(1) < 0.0) {
            _d_u(9) = 0.0;
            _d_l(9) = -(tmp_Friction1 * cos(actual_base_ori_local_HS(1)) - N * sin(actual_base_ori_local_HS(1)));

        } else if (target_base_ori_local_HS(1) > 0.0) {
            _d_u(9) = (tmp_Friction1 * cos(actual_base_ori_local_HS(1)) - N * sin(actual_base_ori_local_HS(1)));
            _d_l(9) = 0.0;
        } else {
            _d_u(9) = tmp_Friction1 * cos(actual_base_ori_local_HS(1));
            _d_l(9) = -tmp_Friction1 * cos(actual_base_ori_local_HS(1));
        }
        _d_u(10) = tmp_Friction2;
        _d_l(10) = -tmp_Friction2;
    }


    for (unsigned int i = 0; i < A_nnz; ++i) {
        l[i] = _d_l(i);
        u[i] = _d_u(i);
    }

    osqp_update_P(QP_work, P_x, OSQP_NULL, 78);
    osqp_update_lin_cost(QP_work, q);
    osqp_update_bounds(QP_work, l, u);

    // Solve updated problem
    osqp_solve(QP_work);

    for (unsigned int i = 0; i < A_nnz; ++i) {

        semi_F_QP_global_yaw(i) = QP_work->solution->x[i];
    }

    //    if (Leg_state(0) != 0) {
    //        std::cout << "F(QP)=" << semi_F_QP_global_yaw((Leg_state(0) - 1)*3 + 0) << "," << semi_F_QP_global_yaw((Leg_state(0) - 1)*3 + 1) << "," << semi_F_QP_global_yaw((Leg_state(0) - 1)*3 + 2) << std::endl;
    //    }


    F_QP_local = target_C_WB_PR_12d_HS.transpose() * semi_F_QP_global_yaw;
    OSQP_Control_value_HS << 0, 0, 0, 0, 0, 0, 0, F_QP_local;
    //cout<<"osqp="<<semi_F_QP_global_yaw.transpose()<<endl;
   // cout << "Fz=" << semi_F_QP_global_yaw(2) + semi_F_QP_global_yaw(5) + semi_F_QP_global_yaw(8) + semi_F_QP_global_yaw(11) << endl;
}

void CRobot::TF_Global2Semi(void) {

    //    semi_target_EP_HS_yaw << target_C_WB_Y_2d_HS.transpose() * target_EP_HS.segment(0, 2), target_EP_HS(2), target_C_WB_Y_2d_HS.transpose() * target_EP_HS.segment(3, 2), target_EP_HS(5), target_C_WB_Y_2d_HS.transpose() * target_EP_HS.segment(6, 2), target_EP_HS(8), target_C_WB_Y_2d_HS.transpose() * target_EP_HS.segment(9, 2), target_EP_HS(11);
    //    semi_target_base_ori_HS_yaw << target_C_WB_Y_2d_HS.transpose() * target_base_ori_HS.segment(0, 2), target_base_ori_HS(2);
    //    semi_target_base_ori_vel_HS_yaw << target_C_WB_Y_2d_HS.transpose() * target_base_ori_vel_HS.segment(0, 2), target_base_ori_vel_HS(2);
    //    semi_target_com_pos_HS_yaw << target_C_WB_Y_2d_HS.transpose() * target_com_pos_HS.segment(0, 2), target_com_pos_HS(2);
    semi_target_com_acc_HS_yaw << target_C_WB_Y_2d_HS.transpose() * target_com_acc_HS.segment(0, 2), target_com_acc_HS(2);
}

void CRobot::Slope_Controller3(void) {
    unsigned int tmp_slope_cnt_HS = 0.0;
    unsigned int tmp_slope_cnt_HS2 = 0.0;

    Angle_Estimation2();

    if (slope_traj_gen_flag_HS == false) {
        goal_base_ori_local_HS(1) = filtered_Pitch_local_vec(1);
    }
    if (slope_traj_gen_flag_HS == true) {
        if (slope_cnt_HS == 0) {
            init_base_ori_local_HS(1) = target_base_ori_local_HS(1);
            target_base_ori_local_HS(1) = init_base_ori_local_HS(1);
            target_base_ori_vel_local_HS(1) = 0;

            slope_cnt_HS++;
        } else if (slope_cnt_HS < (step_cnt_HS + tsp_cnt_HS)) {
            tmp_slope_cnt_HS = slope_cnt_HS;
            target_base_ori_local_HS(1) = init_base_ori_local_HS(1)+(goal_base_ori_local_HS(1) - init_base_ori_local_HS(1)) / 2.0 * (1 - cos(PI / (step_cnt_HS + tsp_cnt_HS) * tmp_slope_cnt_HS));
            slope_cnt_HS++;
            if (slope_cnt_HS == (step_cnt_HS + tsp_cnt_HS)) {
                slope_cnt_HS = 0;
                slope_traj_gen_flag_HS = false;
            }
        }
    }
}

//void CRobot::Angle_Estimation(void) {
//    Vector3d b_n, w_n;
//    Vector3d b_r_RL, b_r_RR, b_r_FL, b_r_FR;
//    Vector3d b_r_RL_RR, b_r_RR_FR, b_r_FR_FL, b_r_FL_RL;
//    Vector3d n_plane1, n_plane2, n_plane3, n_plane4;
//    VectorNd Pitch_local_vec(3);
//
//    double C_RL, C_RR, C_FL, C_FR;
//    double estimated_angle_size;
//    VectorNd Angle_vec_global(2);
//    VectorNd Angle_vec_local(2);
//    VectorNd Angle_vec_global_3d(3);
//    VectorNd Angle_vec_local_3d(3);
//    double alpha = 0.999;
//    double thres_roll = 5 * D2R;
//
//    if (Contact_Info_HS(0) + Contact_Info_HS(1) + Contact_Info_HS(2) + Contact_Info_HS(3) == 3) {
//
//        tmp_Contact_Info_HS = Contact_Info_HS;
//    }
//
//    C_RL = tmp_Contact_Info_HS(0);
//    C_RR = tmp_Contact_Info_HS(1);
//    C_FL = tmp_Contact_Info_HS(2);
//    C_FR = tmp_Contact_Info_HS(3);
//
//    b_r_RL = actual_EP_local_HS.segment(0, 3);
//    b_r_RR = actual_EP_local_HS.segment(3, 3);
//    b_r_FL = actual_EP_local_HS.segment(6, 3);
//    b_r_FR = actual_EP_local_HS.segment(9, 3);
//
//    b_r_RL_RR = b_r_RR - b_r_RL;
//    b_r_RR_FR = b_r_FR - b_r_RR;
//    b_r_FR_FL = b_r_FL - b_r_FR;
//    b_r_FL_RL = b_r_RL - b_r_FL;
//
//
//    b_n = C_RL * C_RR * C_FR * b_r_RL_RR.cross(b_r_RR_FR) + C_RR * C_FR * C_FL * b_r_RR_FR.cross(b_r_FR_FL) + C_FR * C_FL * C_RL * b_r_FR_FL.cross(b_r_FL_RL) + C_FL * C_RL * C_RR * b_r_FL_RL.cross(b_r_RL_RR);
//    /* global*/
//    w_n = target_C_WB_YPR_HS*b_n;
//    estimated_angle_size = acos(w_n(2) / sqrt(pow(w_n(0), 2) + pow(w_n(1), 2) + pow(w_n(2), 2)));
//    Angle_vec_global = w_n.segment(0, 2) / sqrt(pow(w_n(0), 2) + pow(w_n(1), 2)) * estimated_angle_size;
//    Angle_vec_local = target_C_WB_Y_HS.transpose().block(0, 0, 2, 2) * Angle_vec_global;
//    Plane_Angle << -Angle_vec_local(1), Angle_vec_local(0);
//    //Plane_Angle << -Angle_vec_local_3d(1), Angle_vec_local_3d(0);
//
//    Roll_set = Low_pass_Filter_HS(Plane_Angle(0), Roll_set(0), 0.999);
//    Pitch_set = Low_pass_Filter_HS(Plane_Angle(1), Pitch_set(0), 0.999);
//
////    if(abs(Plane_Angle(0))<thres_roll){
////      filtered_Pitch_local_vec << 0.0, Pitch_set(0), 0.0;
////    }
////    else{
////      filtered_Pitch_local_vec << 0.0, 0.0, 0.0;
////    }
//}

void CRobot::Angle_Estimation2(void) {
    //Vector3d semi_w_n;
    //Vector3d semi_r_RL, semi_r_RR, semi_r_FL, semi_r_FR;
    MatrixNd A_angle(4, 3);
    VectorNd b_angle(4);
    MatrixNd tmp_A_angle(3, 3);
    VectorNd tmp_angle_coeff(3);
    double estimated_angle_size;
    VectorNd Angle_vec_semi_global(2);
    double thres_roll = 5 * D2R;

    if (Contact_Info_HS(0) + Contact_Info_HS(1) + Contact_Info_HS(2) + Contact_Info_HS(3) == 4) {
        //        tmp_Contact_Info_HS = Contact_Info_HS;
        semi_r_RL = target_C_WB_PR_HS * actual_EP_local_HS.segment(0, 3);
        semi_r_RR = target_C_WB_PR_HS * actual_EP_local_HS.segment(3, 3);
        semi_r_FL = target_C_WB_PR_HS * actual_EP_local_HS.segment(6, 3);
        semi_r_FR = target_C_WB_PR_HS * actual_EP_local_HS.segment(9, 3);

        A_angle << 1, semi_r_RL(0), semi_r_RL(1)\
            , 1, semi_r_RR(0), semi_r_RR(1)\
            , 1, semi_r_FL(0), semi_r_FL(1)\
            , 1, semi_r_FR(0), semi_r_FR(1);

        b_angle << semi_r_RL(2), semi_r_RR(2), semi_r_FL(2), semi_r_FR(2);

        tmp_A_angle = A_angle.transpose() * A_angle;
        tmp_angle_coeff = tmp_A_angle.inverse() * A_angle.transpose() * b_angle;

        semi_w_n << -tmp_angle_coeff(1), -tmp_angle_coeff(2), 1;

        /* global*/
        estimated_angle_size = acos(semi_w_n(2) / sqrt(pow(semi_w_n(0), 2) + pow(semi_w_n(1), 2) + pow(semi_w_n(2), 2)));

        Angle_vec_semi_global = semi_w_n.segment(0, 2) / sqrt(pow(semi_w_n(0), 2) + pow(semi_w_n(1), 2)) * estimated_angle_size;
        Plane_Angle << Angle_vec_semi_global(1), Angle_vec_semi_global(0);
        //cout<<"Roll="<<Angle_vec_semi_global(1)*R2D<<endl;
        //cout<<"Pitch="<<Angle_vec_semi_global(0)*R2D<<endl;
    } else {
        slope_traj_gen_flag_HS = true;
    }

    Roll_set = Low_pass_Filter_HS(Plane_Angle(0), Roll_set(0), 0.999);
    Pitch_set = Low_pass_Filter_HS(Plane_Angle(1), Pitch_set(0), 0.999);

    //    std::cout << abs(Plane_Angle(0)) << std::endl;
    //    std::cout << "----------------------------" << std::endl;
    //    
    //if (abs(Plane_Angle(0)) < thres_roll) {
    //filtered_Pitch_local_vec << 0.0, Pitch_set(0), 0.0;
    //if (slope_traj_gen_flag_HS == true) {
    filtered_Pitch_local_vec << 0.0, Plane_Angle(1), 0.0;
    base_hold_flag = false;
    //  }
    //} else {
    //  if (slope_traj_gen_flag_HS == true) {
    //     filtered_Pitch_local_vec << 0.0, 0.0, 0.0;
    //    base_hold_flag = true;
    // }
    ///}
}

VectorNd CRobot::Low_pass_Filter_HS(double _updated_value, double _pre_estimated_value, double _alpha) {
    double estimated_value;
    VectorNd value_set(2);
    double alpha;

    if (isnan(_updated_value)) {
        _updated_value = 0;
        alpha = 1;
    }

    estimated_value = _alpha * _pre_estimated_value + (1.0 - _alpha) * _updated_value;
    value_set << estimated_value, _pre_estimated_value;

    return value_set;
}

void CRobot::print_HS(void) {

    std::cout << "cnt_HS=" << cnt_HS << std::endl;
    std::cout << "CONTACT_info=" << Contact_Info_HS.transpose() << std::endl;
    std::cout << "--------------------------" << std::endl;
    std::cout << "kp_EP=" << kp_EP_HS.transpose() << std::endl;
    std::cout << "kd_EP=" << kp_EP_HS.transpose() << std::endl;
    std::cout << "--------------------------" << std::endl;
    std::cout << "Task=" << Task_Control_value_HS.transpose() << std::endl;
    std::cout << "QP=" << OSQP_Control_value_HS.transpose() << std::endl;
    // std::cout << "tar_base_pos(G) : " << target_base_pos_HS(0) << "/" << target_base_pos_HS(1) << "/" << target_base_pos_HS(2) << std::endl;
    //std::cout << "target_com_pos : " << target_com_pos_HS(0) << "/" << target_com_pos_HS(1) << "/" << target_com_pos_HS(2) << std::endl;

    //std::cout << "target_EP(L) : " << "(" << "(" << target_EP_local_HS(0) << "/" << target_EP_local_HS(1) << "/" << target_EP_local_HS(2) << ")" << "," << "(" << target_EP_local_HS(3) << "/" << target_EP_local_HS(4) << "/" << target_EP_local_HS(5) << ")" << "," << "(" << target_EP_local_HS(6) << "/" << target_EP_local_HS(7) << "/" << target_EP_local_HS(8) << ")" << "," << "(" << target_EP_local_HS(9) << "/" << target_EP_local_HS(10) << "/" << target_EP_local_HS(11) << ")" << std::endl;
    //std::cout<<"now_speed="<<now_vel_HS(0)<<"/"<<now_vel_HS(1)<<"/"<<now_vel_HS(2)<<std::endl;
    //std::cout<<"speed="<<tar_vel_HS(0)<<"/"<<tar_vel_HS(1)<<"/"<<tar_vel_HS(2)<<std::endl;

    //std::cout << "init_EP(G) : " << "(" << init_EP_HS(0) << "/" << init_EP_HS(1) << "/" << init_EP_HS(2) << ")" << "," << "(" << init_EP_HS(3) << "/" << init_EP_HS(4) << "/" << init_EP_HS(5) << ")" << "," << "(" << init_EP_HS(6) << "/" << init_EP_HS(7) << "/" << init_EP_HS(8) << ")" << "," << "(" << init_EP_HS(9) << "/" << init_EP_HS(10) << "/" << init_EP_HS(11) << ")" << std::endl;
    // std::cout << "goal_EP(G) : " << "(" << goal_EP_HS(0) << "/" << goal_EP_HS(1) << "/" << goal_EP_HS(2) << ")" << "," << "(" << goal_EP_HS(3) << "/" << goal_EP_HS(4) << "/" << goal_EP_HS(5) << ")" << "," << "(" << goal_EP_HS(6) << "/" << goal_EP_HS(7) << "/" << goal_EP_HS(8) << ")" << "," << "(" << goal_EP_HS(9) << "/" << goal_EP_HS(10) << "/" << goal_EP_HS(11) << ")" << std::endl;
    //std::cout << "target_EP(G) : " << "(" << target_EP_HS(0) << "/" << target_EP_HS(1) << "/" << target_EP_HS(2) << ")" << "," << "(" << target_EP_HS(3) << "/" << target_EP_HS(4) << "/" << target_EP_HS(5) << ")" << "," << "(" << target_EP_HS(6) << "/" << target_EP_HS(7) << "/" << target_EP_HS(8) << ")" << "," << "(" << target_EP_HS(9) << "/" << target_EP_HS(10) << "/" << target_EP_HS(11) << ")" << std::endl;
    //std::cout << "tmp_goal_EP(semiG) : " << "(" << semi_tmp_goal_EP_HS(0) << "/" << semi_tmp_goal_EP_HS(1) << "/" << semi_tmp_goal_EP_HS(2) << ")" << "," << "(" << semi_tmp_goal_EP_HS(3) << "/" << semi_tmp_goal_EP_HS(4) << "/" << semi_tmp_goal_EP_HS(5) << ")" << "," << "(" << semi_tmp_goal_EP_HS(6) << "/" << semi_tmp_goal_EP_HS(7) << "/" << semi_tmp_goal_EP_HS(8) << ")" << "," << "(" << semi_tmp_goal_EP_HS(9) << "/" << semi_tmp_goal_EP_HS(10) << "/" << semi_tmp_goal_EP_HS(11) << ")" << std::endl;


    //std::cout << "pre_init_EP(semiG) : " << "(" << semi_pre_init_EP_HS(0) << "/" << semi_pre_init_EP_HS(1) << "/" << semi_pre_init_EP_HS(2) << ")" << "," << "(" << semi_pre_init_EP_HS(3) << "/" << semi_pre_init_EP_HS(4) << "/" << semi_pre_init_EP_HS(5) << ")" << "," << "(" << semi_pre_init_EP_HS(6) << "/" << semi_pre_init_EP_HS(7) << "/" << semi_pre_init_EP_HS(8) << ")" << "," << "(" << semi_pre_init_EP_HS(9) << "/" << semi_pre_init_EP_HS(10) << "/" << semi_pre_init_EP_HS(11) << ")" << std::endl;
    //std::cout << "init_EP(semiG) : " << "(" << semi_init_EP_HS(0) << "/" << semi_init_EP_HS(1) << "/" << semi_init_EP_HS(2) << ")" << "," << "(" << semi_init_EP_HS(3) << "/" << semi_init_EP_HS(4) << "/" << semi_init_EP_HS(5) << ")" << "," << "(" << semi_init_EP_HS(6) << "/" << semi_init_EP_HS(7) << "/" << semi_init_EP_HS(8) << ")" << "," << "(" << semi_init_EP_HS(9) << "/" << semi_init_EP_HS(10) << "/" << semi_init_EP_HS(11) << ")" << std::endl;
    //std::cout << "goal_EP(semiG) : " << "(" << semi_goal_EP_HS(0) << "/" << semi_goal_EP_HS(1) << "/" << semi_goal_EP_HS(2) << ")" << "," << "(" << semi_goal_EP_HS(3) << "/" << semi_goal_EP_HS(4) << "/" << semi_goal_EP_HS(5) << ")" << "," << "(" << semi_goal_EP_HS(6) << "/" << semi_goal_EP_HS(7) << "/" << semi_goal_EP_HS(8) << ")" << "," << "(" << semi_goal_EP_HS(9) << "/" << semi_goal_EP_HS(10) << "/" << semi_goal_EP_HS(11) << ")" << std::endl;
    // std::cout << "target_EP(semiG) : " << "(" << semi_target_EP_HS(0) << "/" << semi_target_EP_HS(1) << "/" << semi_target_EP_HS(2) << ")" << "," << "(" << semi_target_EP_HS(3) << "/" << semi_target_EP_HS(4) << "/" << semi_target_EP_HS(5) << ")" << "," << "(" << semi_target_EP_HS(6) << "/" << semi_target_EP_HS(7) << "/" << semi_target_EP_HS(8) << ")" << "," << "(" << semi_target_EP_HS(9) << "/" << semi_target_EP_HS(10) << "/" << semi_target_EP_HS(11) << ")" << std::endl;
    //td::cout << "target_com_pos(semiG) : " << "(" << semi_target_com_pos_HS(0) << "/" << semi_target_com_pos_HS(1) << "/" << semi_target_com_pos_HS(2) << ")" << std::endl;
    //  std::cout << "target_com_acc(semiG) : " << "(" << semi_target_com_acc_HS(0) << "/" << semi_target_com_acc_HS(1) << "/" << semi_target_com_acc_HS(2) << ")" << std::endl;
    //   std::cout << "target_Base_ori(semiG) : " << "(" << semi_target_base_ori_HS(0) * R2D << "/" << semi_target_base_ori_HS(1) * R2D << "/" << semi_target_base_ori_HS(2) * R2D << ")" << std::endl;
    //    std::cout << "target_Base_ori_vel(semiG) : " << "(" << semi_target_base_ori_vel_HS(0) << "/" << semi_target_base_ori_vel_HS(1) << "/" << semi_target_base_ori_vel_HS(2) << ")" << std::endl;
    //std::cout << "target_Yaw" << target_C_WB_Y_2d_HS << std::endl;
    std::cout << "__________________________________________________________" << std::endl;
}

void CRobot::Base_Estimation_Test(void) {

    VectorNd k_gain(2);
    double tmp_mat;
    VectorNd f(2);
    MatrixNd A(2, 2);
    VectorNd H(2);
    MatrixNd Q(2, 2);
    double R;
    double a_z;
    double z_measure;
    double g = 9.81;
    double theta;

    Q << 0.001, 0\
, 0, 0.001;

    R=30;
        //R=50;
    //R = 500;

    // System matrix
    A << 1, dt\
, 0, 1;

    H << 1, 0;

    //theta=target_base_ori_local_HS(1);
    theta = actual_base_ori_local_HS(1);
    // Sensor
    
    actual_base_acc_HS = actual_C_WB_PR_HS*actual_base_acc_local_HS;
    //a_z = sin(theta) * linear_acc_x + cos(theta) * linear_acc_z;
    //z_measure = actual_plane_dist_HS;
    z_measure = measure_z;

    if (initial_flag_HS == true) {
        x_hat(0) = z_measure;
        x_hat(1) = 0.0;
        //        p_hat(0, 0) = 0.1;
        //p_hat(1, 1) = 0.01;
        p_hat(0, 0) = 0.0;
        p_hat(1, 1) = 0.0;
        pre_x_hat = x_hat;
    }
    //Prediciton
    x_bar(0) = x_hat(0) + dt * x_hat(1) + 1 / 2 * (actual_base_acc_HS(2) - g) * dt*dt;
    x_bar(1) = x_hat(1)+(actual_base_acc_HS(2) - g) * dt;
    p_bar = A * p_hat * A.transpose() + Q;

    //Kalman gain
    k_gain = p_bar * H / (H.transpose() * p_bar * H + R);

    //Correction
    x_hat = x_bar + k_gain * (z_measure - H.transpose() * x_bar);
    p_hat = p_bar - k_gain * H.transpose() * p_bar.transpose();

}

VectorNd CRobot::Base_Estimation2(VectorNd actual_EP_pos_local) {

    double z_RL, z_RR, z_FL, z_FR;
    VectorNd tmp_Standard_leg = VectorNd::Zero(4);
    VectorNd tmp_actual_EP_pos_global;

    //tmp_actual_EP_pos_global=target_C_WB_PR_12d_HS*actual_EP_pos_local;
    z_RL = abs(actual_EP_pos_local(2) / cos(abs(target_base_ori_local_HS(1))));
    z_RR = abs(actual_EP_pos_local(5) / cos(abs(target_base_ori_local_HS(1))));
    z_FL = abs(actual_EP_pos_local(8) / cos(abs(target_base_ori_local_HS(1))));
    z_FR = abs(actual_EP_pos_local(11) / cos(abs(target_base_ori_local_HS(1))));

    std::cout << z_RL << "," << z_RR << "," << z_FL << "," << z_FR << std::endl;
    std::cout << "-----------------------" << std::endl;

    if (z_RL >= z_RR) {
        if (z_RL >= z_FL) {
            if (z_RL >= z_FR) {
                tmp_Standard_leg << 1, 0, 0, 0; //RL
            } else {
                tmp_Standard_leg << 0, 0, 0, 1; //FR
            }
        } else {
            if (z_FL >= z_FR) {
                tmp_Standard_leg << 0, 0, 1, 0; //FL
            } else {
                tmp_Standard_leg << 0, 0, 0, 1; //FR
            }
        }
    } else {
        if (z_RR >= z_FL) {
            if (z_RR >= z_FR) {
                tmp_Standard_leg << 0, 1, 0, 0; //RR
            } else {
                tmp_Standard_leg << 0, 0, 0, 1; //FR
            }
        } else {
            if (z_FL >= z_FR) {
                tmp_Standard_leg << 0, 0, 1, 0; //RR
            } else {
                tmp_Standard_leg << 0, 0, 0, 1; //FR
            }
        }

    }


    return tmp_Standard_leg;
}

void CRobot::Break_leg(void) {
    if (Leg_state(Leg_state(0)) == -1) {
        if (actual_EP_acc_HS((Leg_state(0) - 1) * 3 + 2) > 100) {
            Early_Contact_flag_HS = true;
        }
        if (Early_Contact_flag_HS == true) {
            target_EP_vel_HS((Leg_state(0) - 1)*3 + 0) = 0.0;
            target_EP_vel_HS((Leg_state(0) - 1)*3 + 1) = 0.0;
            target_EP_vel_HS((Leg_state(0) - 1)*3 + 2) = 0.0;
        }
    } else {
        Early_Contact_flag_HS = false;
    }
}

void CRobot::QuadPP_SOLVE_TEST(void) {
    quadprogpp::Matrix<double> G, CE, CI;
    quadprogpp::Vector<double> g0, ce0, ci0, x;
    MatrixNd tmp_G(12, 12);
    VectorNd tmp_g0(12);

    int n, m, p;

    MatrixNd A_BC(6, 12);
    MatrixNd tmp_A_BC(12, 6);
    VectorNd b_BC(6);
    MatrixNd W(12, 12);

    double Fz_RL_max, Fz_RL_min;
    double Fz_RR_max, Fz_RR_min;
    double Fz_FL_max, Fz_FL_min;
    double Fz_FR_max, Fz_FR_min;

    double Rear_body_mass = 13.826;
    double Front_body_mass = 10.507;
    double One_leg_mass = 5.295;
    double Robot_mass = Rear_body_mass + Front_body_mass + One_leg_mass * 4;
    double Robot_Weight = Robot_mass*GRAVITY;

    double one_leg_weight = 120;

    VectorNd r_RL(3);
    VectorNd r_RR(3);
    VectorNd r_FL(3);
    VectorNd r_FR(3);
    VectorNd r_COM(3);

    VectorNd k_p_QP(6);
    VectorNd k_d_QP(6);
    double b1, b2, b3, b4, b5, b6;

    VectorNd F_QP_global(12);
    VectorNd F_QP_local(12);

    double alpha = 0.001;
    double beta = 1.0;
    double tmp_mu = 1.0;
    W = MatrixNd::Identity(12, 12);

    VectorNd target_base_pos_rel(3);
    VectorNd actual_base_pos_rel(3);
    VectorNd target_com_pos_rel(3);
    VectorNd actual_com_pos_rel(3);

    VectorNd target_base_vel_rel(3);
    VectorNd actual_base_vel_rel(3);

    VectorNd Err_com_pos(3);
    VectorNd Err_com_vel(3);

    VectorNd standard_info_HS(4);
    double standard_leg_height_HS;

    double N;

    VectorNd r_com_EP_local(12);
    VectorNd r_com_EP_semi_global(12);
    VectorNd r_com_RL(3);
    VectorNd r_com_RR(3);
    VectorNd r_com_FL(3);
    VectorNd r_com_FR(3);

    VectorNd tmp_t = VectorNd::Zero(3);
    VectorNd tmp_b = VectorNd::Zero(3);


    semi_target_com_acc_HS_yaw << target_C_WB_Y_2d_HS.transpose() * target_com_acc_HS.segment(0, 2), target_com_acc_HS(2);

    VectorNd n_vec_HS = VectorNd(3);
    VectorNd b_vec_HS = VectorNd(3);
    VectorNd t_vec_HS = VectorNd(3);

    k_p_QP << 10, 10, 4000, 3000, 3000, 0;
    k_d_QP << 0.1, 0.1, 40, 30, 30, 10;

    r_com_EP_local << actual_EP_local_HS.segment(0, 3) - offset_B2C, actual_EP_local_HS.segment(3, 3) - offset_B2C, actual_EP_local_HS.segment(6, 3) - offset_B2C, actual_EP_local_HS.segment(9, 3) - offset_B2C;
    r_com_EP_semi_global = target_C_WB_PR_12d_HS*r_com_EP_local;

    r_com_RL << r_com_EP_semi_global(0), r_com_EP_semi_global(1), r_com_EP_semi_global(2);
    r_com_RR << r_com_EP_semi_global(3), r_com_EP_semi_global(4), r_com_EP_semi_global(5);
    r_com_FL << r_com_EP_semi_global(6), r_com_EP_semi_global(7), r_com_EP_semi_global(8);
    r_com_FR << r_com_EP_semi_global(9), r_com_EP_semi_global(10), r_com_EP_semi_global(11);


    A_BC << Contact_Info_HS(0), 0, 0, Contact_Info_HS(1), 0, 0, Contact_Info_HS(2), 0, 0, Contact_Info_HS(3), 0, 0\
, 0, Contact_Info_HS(0), 0, 0, Contact_Info_HS(1), 0, 0, Contact_Info_HS(2), 0, 0, Contact_Info_HS(3), 0\
, 0, 0, Contact_Info_HS(0), 0, 0, Contact_Info_HS(1), 0, 0, Contact_Info_HS(2), 0, 0, Contact_Info_HS(3)\
          , 0, -Contact_Info_HS(0) * (r_com_RL(2)), Contact_Info_HS(0) * (r_com_RL(1)), 0, -Contact_Info_HS(1) * (r_com_RR(2)), Contact_Info_HS(1) * (r_com_RR(1)), 0, -Contact_Info_HS(2) * (r_com_FL(2)), Contact_Info_HS(2) * (r_com_FL(1)), 0, -Contact_Info_HS(3) * (r_com_FR(2)), Contact_Info_HS(3) * (r_com_FR(1))\
          , Contact_Info_HS(0) * (r_com_RL(2)), 0, -Contact_Info_HS(0)*(r_com_RL(0)), Contact_Info_HS(1) * (r_com_RR(2)), 0, -Contact_Info_HS(1)*(r_com_RR(0)), Contact_Info_HS(2) * (r_com_FL(2)), 0, -Contact_Info_HS(2)*(r_com_FL(0)), Contact_Info_HS(3) * (r_com_FR(2)), 0, -Contact_Info_HS(3)*(r_com_FR(0))\
          , -Contact_Info_HS(0) * (r_com_RL(1)), Contact_Info_HS(0) * (r_com_RL(0)), 0, -Contact_Info_HS(1) * (r_com_RR(1)), Contact_Info_HS(1)*(r_com_RR(0)), 0, -Contact_Info_HS(2) * (r_com_FL(1)), Contact_Info_HS(2)*(r_com_FL(0)), 0, -Contact_Info_HS(3) * (r_com_FR(1)), Contact_Info_HS(3)*(r_com_FR(0)), 0;

    tmp_A_BC = A_BC.transpose();

    target_base_pos_rel = -(Contact_Info_HS(0) * target_EP_local_HS.segment(0, 3) + Contact_Info_HS(1) * target_EP_local_HS.segment(3, 3) + Contact_Info_HS(2) * target_EP_local_HS.segment(6, 3) + Contact_Info_HS(3) * target_EP_local_HS.segment(9, 3)) / (Contact_Info_HS(0) + Contact_Info_HS(1) + Contact_Info_HS(2) + Contact_Info_HS(3));
    actual_base_pos_rel = -(Contact_Info_HS(0) * actual_EP_local_HS.segment(0, 3) + Contact_Info_HS(1) * actual_EP_local_HS.segment(3, 3) + Contact_Info_HS(2) * actual_EP_local_HS.segment(6, 3) + Contact_Info_HS(3) * actual_EP_local_HS.segment(9, 3)) / (Contact_Info_HS(0) + Contact_Info_HS(1) + Contact_Info_HS(2) + Contact_Info_HS(3));

    target_base_vel_rel = -(Contact_Info_HS(0) * target_EP_vel_local_HS.segment(0, 3) + Contact_Info_HS(1) * target_EP_vel_local_HS.segment(3, 3) + Contact_Info_HS(2) * target_EP_vel_local_HS.segment(6, 3) + Contact_Info_HS(3) * target_EP_vel_local_HS.segment(9, 3)) / (Contact_Info_HS(0) + Contact_Info_HS(1) + Contact_Info_HS(2) + Contact_Info_HS(3));
    actual_base_vel_rel = -(Contact_Info_HS(0) * actual_EP_vel_local_HS.segment(0, 3) + Contact_Info_HS(1) * actual_EP_vel_local_HS.segment(3, 3) + Contact_Info_HS(2) * actual_EP_vel_local_HS.segment(6, 3) + Contact_Info_HS(3) * actual_EP_vel_local_HS.segment(9, 3)) / (Contact_Info_HS(0) + Contact_Info_HS(1) + Contact_Info_HS(2) + Contact_Info_HS(3));

    target_com_pos_rel = Get_COM_pos_HS2(target_base_pos_rel);
    actual_com_pos_rel = Get_COM_pos_HS2(actual_base_pos_rel);

    Err_com_pos = target_C_WB_PR_HS * (target_com_pos_rel - actual_com_pos_rel);
    Err_com_vel = target_C_WB_PR_HS * (target_base_vel_rel - actual_base_vel_rel);

    target_plane_dist_HS = com_height_HS / cos(abs(target_base_ori_local_HS(1)));
    actual_plane_dist_HS = actual_base_pos_HS(2) + offset_B2C(2) / cos(abs(actual_base_ori_local_HS(1)));

    if (initial_flag_HS == true) {
        pre_actual_plane_dist_HS = actual_plane_dist_HS;
    }
    actual_plane_dist_vel_HS = (actual_plane_dist_HS - pre_actual_plane_dist_HS) / dt;
    pre_actual_plane_dist_HS = actual_plane_dist_HS;

    b1 = Robot_mass * (semi_target_com_acc_HS_yaw(0)) + k_p_QP(0) * Err_com_pos(0) + k_d_QP(0) * Err_com_vel(0);
    b2 = Robot_mass * (semi_target_com_acc_HS_yaw(1)) + k_p_QP(1) * Err_com_pos(1) + k_d_QP(1) * Err_com_vel(1);
    b3 = Robot_mass * (semi_target_com_acc_HS_yaw(2)) + k_p_QP(2)*(target_plane_dist_HS - actual_plane_dist_HS) + k_d_QP(2)*(0.0 - actual_plane_dist_vel_HS) + Robot_Weight;

    b4 = k_p_QP(3)*(target_base_ori_local_HS(0) - actual_base_ori_local_HS(0)) + k_d_QP(3)*(target_base_ori_vel_local_HS(0) - actual_base_ori_vel_local_HS(0));
    b5 = k_p_QP(4)*(target_base_ori_local_HS(1) - actual_base_ori_local_HS(1)) + k_d_QP(4)*(target_base_ori_vel_local_HS(1) - actual_base_ori_vel_local_HS(1));
    b6 = k_d_QP(5)*(target_base_ori_vel_local_HS(2) - actual_base_ori_vel_local_HS(2));


    b_BC << b1, b2, b3, b4, b5, b6;

    // QP algorithm
    n = 12;
    m = 0;
    p = 24;
    //lamda = 120 * 1; //[N]

    tmp_G = 2 * (tmp_A_BC * A_BC + alpha * W);
    tmp_g0 = -2 * (tmp_A_BC * b_BC);

    G.resize(n, n);
    {
        for (int i = 0; i < n; i++) {
            for (int j = 0; j < n; j++) {
                G[i][j] = tmp_G(i, j);
            }
        }
    }
    //std::cout << "G: " << G << std::endl;
    g0.resize(n);
    {
        for (int i = 0; i < n; i++) {
            g0[i] = tmp_g0[i];
        }
    }
    //std::cout << "g0: " << g0 << std::endl;

    CE.resize(n, m);
    {
        for (int i = 0; i < n; i++) {
            for (int j = 0; j < m; j++) {
                CE[i][j] = 0;
            }
        }
    }
    //std::cout << "CE: " << CE << std::endl;
    ce0.resize(m);
    {
        for (int j = 0; j < m; j++) {
            //            ce0[j] = -b_BC[j];
            ce0[j] = 0;
        }
    }

    N = Robot_Weight / 4.0 * cos(actual_base_ori_local_HS(1));
    tmp_Friction1 = tmp_mu * N;
    tmp_Friction2 = tmp_mu * Robot_Weight / 4.0;

    MatrixNd tmp_CI = MatrixNd::Zero(p, n);

    CI.resize(n, p);
    {
        for (int i = 0; i < n; i++) {
            for (int j = 0; j < p; j++) {
                CI[i][j] = 0;
            }
        }
    }

    n_vec_HS = semi_w_n / sqrt(semi_w_n(0) * semi_w_n(0) + semi_w_n(1) * semi_w_n(1) + semi_w_n(2) * semi_w_n(2));
    tmp_b << 0, 1, 0;
    tmp_t << 1, 0, 0;

    b_vec_HS = target_C_WB_PR_HS*tmp_b;
    t_vec_HS = target_C_WB_PR_HS*tmp_t;


    // std::cout << nx << "," << ny << "," << nz << std::endl;
    // std::cout<<"---"<<std::endl;
    tmp_mu = 1.0 / sqrt(2);
    //tmp_mu = 0.8/sqrt(2);

    tmp_CI.block(0, 0, 6, 3) << Contact_Info_HS(0) * n_vec_HS(0), Contact_Info_HS(0) * n_vec_HS(1), Contact_Info_HS(0) * n_vec_HS(2)\
, Contact_Info_HS(0)*(-t_vec_HS(0) + tmp_mu * n_vec_HS(0)), Contact_Info_HS(0)*(-t_vec_HS(1) + tmp_mu * n_vec_HS(1)), Contact_Info_HS(0)*(-t_vec_HS(2) + tmp_mu * n_vec_HS(2))\
, Contact_Info_HS(0)*(t_vec_HS(0) + tmp_mu * n_vec_HS(0)), Contact_Info_HS(0)*(t_vec_HS(1) + tmp_mu * n_vec_HS(1)), Contact_Info_HS(0)*(t_vec_HS(2) + tmp_mu * n_vec_HS(2))\
, Contact_Info_HS(0)*(-b_vec_HS(0) + tmp_mu * n_vec_HS(0)), Contact_Info_HS(0)*(-b_vec_HS(1) + tmp_mu * n_vec_HS(1)), Contact_Info_HS(0)*(-b_vec_HS(2) + tmp_mu * n_vec_HS(2))\
, Contact_Info_HS(0)*(b_vec_HS(0) + tmp_mu * n_vec_HS(0)), Contact_Info_HS(0)*(b_vec_HS(1) + tmp_mu * n_vec_HS(1)), Contact_Info_HS(0)*(b_vec_HS(2) + tmp_mu * n_vec_HS(2))\
, 0, 0, -Contact_Info_HS(0);

    tmp_CI.block(6, 3, 6, 3) << Contact_Info_HS(1) * n_vec_HS(0), Contact_Info_HS(1) * n_vec_HS(1), Contact_Info_HS(1) * n_vec_HS(2)\
, Contact_Info_HS(1)*(-t_vec_HS(0) + tmp_mu * n_vec_HS(0)), Contact_Info_HS(1)*(-t_vec_HS(1) + tmp_mu * n_vec_HS(1)), Contact_Info_HS(1)*(-t_vec_HS(2) + tmp_mu * n_vec_HS(2))\
, Contact_Info_HS(1)*(t_vec_HS(0) + tmp_mu * n_vec_HS(0)), Contact_Info_HS(1)*(t_vec_HS(1) + tmp_mu * n_vec_HS(1)), Contact_Info_HS(1)*(t_vec_HS(2) + tmp_mu * n_vec_HS(2))\
, Contact_Info_HS(1)*(-b_vec_HS(0) + tmp_mu * n_vec_HS(0)), Contact_Info_HS(1)*(-b_vec_HS(1) + tmp_mu * n_vec_HS(1)), Contact_Info_HS(1)*(-b_vec_HS(2) + tmp_mu * n_vec_HS(2))\
, Contact_Info_HS(1)*(b_vec_HS(0) + tmp_mu * n_vec_HS(0)), Contact_Info_HS(1)*(b_vec_HS(1) + tmp_mu * n_vec_HS(1)), Contact_Info_HS(1)*(b_vec_HS(2) + tmp_mu * n_vec_HS(2))\
, 0, 0, -Contact_Info_HS(1);

    tmp_CI.block(12, 6, 6, 3) << Contact_Info_HS(2) * n_vec_HS(0), Contact_Info_HS(2) * n_vec_HS(1), Contact_Info_HS(2) * n_vec_HS(2)\
, Contact_Info_HS(2)*(-t_vec_HS(0) + tmp_mu * n_vec_HS(0)), Contact_Info_HS(2)*(-t_vec_HS(1) + tmp_mu * n_vec_HS(1)), Contact_Info_HS(2)*(-t_vec_HS(2) + tmp_mu * n_vec_HS(2))\
, Contact_Info_HS(2)*(t_vec_HS(0) + tmp_mu * n_vec_HS(0)), Contact_Info_HS(2)*(t_vec_HS(1) + tmp_mu * n_vec_HS(1)), Contact_Info_HS(2)*(t_vec_HS(2) + tmp_mu * n_vec_HS(2))\
, Contact_Info_HS(2)*(-b_vec_HS(0) + tmp_mu * n_vec_HS(0)), Contact_Info_HS(2)*(-b_vec_HS(1) + tmp_mu * n_vec_HS(1)), Contact_Info_HS(2)*(-b_vec_HS(2) + tmp_mu * n_vec_HS(2))\
, Contact_Info_HS(2)*(b_vec_HS(0) + tmp_mu * n_vec_HS(0)), Contact_Info_HS(2)*(b_vec_HS(1) + tmp_mu * n_vec_HS(1)), Contact_Info_HS(2)*(b_vec_HS(2) + tmp_mu * n_vec_HS(2))\
, 0, 0, -Contact_Info_HS(2);

    tmp_CI.block(18, 9, 6, 3) << Contact_Info_HS(3) * n_vec_HS(0), Contact_Info_HS(3) * n_vec_HS(1), Contact_Info_HS(3) * n_vec_HS(2)\
, Contact_Info_HS(3)*(-t_vec_HS(0) + tmp_mu * n_vec_HS(0)), Contact_Info_HS(3)*(-t_vec_HS(1) + tmp_mu * n_vec_HS(1)), Contact_Info_HS(3)*(-t_vec_HS(2) + tmp_mu * n_vec_HS(2))\
, Contact_Info_HS(3)*(t_vec_HS(0) + tmp_mu * n_vec_HS(0)), Contact_Info_HS(3)*(t_vec_HS(1) + tmp_mu * n_vec_HS(1)), Contact_Info_HS(3)*(t_vec_HS(2) + tmp_mu * n_vec_HS(2))\
, Contact_Info_HS(3)*(-b_vec_HS(0) + tmp_mu * n_vec_HS(0)), Contact_Info_HS(3)*(-b_vec_HS(1) + tmp_mu * n_vec_HS(1)), Contact_Info_HS(3)*(-b_vec_HS(2) + tmp_mu * n_vec_HS(2))\
, Contact_Info_HS(3)*(b_vec_HS(0) + tmp_mu * n_vec_HS(0)), Contact_Info_HS(3)*(b_vec_HS(1) + tmp_mu * n_vec_HS(1)), Contact_Info_HS(3)*(b_vec_HS(2) + tmp_mu * n_vec_HS(2))\
, 0, 0, -Contact_Info_HS(3);
    //    tmp_CI.block(0, 0, 3, 3) << Contact_Info_HS(0) * nx, Contact_Info_HS(0) * ny, Contact_Info_HS(0) * nz\
//, Contact_Info_HS(0)*(1 + tmp_mu * nx - nx * (nx + ny + nz)), Contact_Info_HS(0)*(1 + tmp_mu * ny - ny * (nx + ny + nz)), Contact_Info_HS(0)*(1 + tmp_mu * nz - nz * (nx + ny + nz))\
//, -Contact_Info_HS(0)*(1 - tmp_mu * nx - nx * (nx + ny + nz)), -Contact_Info_HS(0)*(1 - tmp_mu * ny - ny * (nx + ny + nz)), -Contact_Info_HS(0)*(1 - tmp_mu * nz - nz * (nx + ny + nz));
    //    
    //    tmp_CI.block(3, 3, 3, 3) << Contact_Info_HS(1) * nx, Contact_Info_HS(1) * ny, Contact_Info_HS(1) * nz\
//, Contact_Info_HS(1)*(1 + tmp_mu * nx - nx * (nx + ny + nz)), Contact_Info_HS(1)*(1 + tmp_mu * ny - ny * (nx + ny + nz)), Contact_Info_HS(1)*(1 + tmp_mu * nz - nz * (nx + ny + nz))\
//, -Contact_Info_HS(1)*(1 - tmp_mu * nx - nx * (nx + ny + nz)), -Contact_Info_HS(1)*(1 - tmp_mu * ny - ny * (nx + ny + nz)), -Contact_Info_HS(1)*(1 - tmp_mu * nz - nz * (nx + ny + nz));
    //    
    //    tmp_CI.block(6, 6, 3, 3) << Contact_Info_HS(2) * nx, Contact_Info_HS(2) * ny, Contact_Info_HS(2) * nz\
//, Contact_Info_HS(2)*(1 + tmp_mu * nx - nx * (nx + ny + nz)), Contact_Info_HS(2)*(1 + tmp_mu * ny - ny * (nx + ny + nz)), Contact_Info_HS(2)*(1 + tmp_mu * nz - nz * (nx + ny + nz))\
//, -Contact_Info_HS(2)*(1 - tmp_mu * nx - nx * (nx + ny + nz)), -Contact_Info_HS(2)*(1 - tmp_mu * ny - ny * (nx + ny + nz)), -Contact_Info_HS(2)*(1 - tmp_mu * nz - nz * (nx + ny + nz));
    //
    //    tmp_CI.block(9, 9, 3, 3) << Contact_Info_HS(3) * nx, Contact_Info_HS(3) * ny, Contact_Info_HS(3) * nz\
//, Contact_Info_HS(3)*(1 + tmp_mu * nx - nx * (nx + ny + nz)), Contact_Info_HS(3)*(1 + tmp_mu * ny - ny * (nx + ny + nz)), Contact_Info_HS(3)*(1 + tmp_mu * nz - nz * (nx + ny + nz))\
//, -Contact_Info_HS(3)*(1 - tmp_mu * nx - nx * (nx + ny + nz)), -Contact_Info_HS(3)*(1 - tmp_mu * ny - ny * (nx + ny + nz)), -Contact_Info_HS(3)*(1 - tmp_mu * nz - nz * (nx + ny + nz));

    //std::cout<<"1="  << tmp_CI << std::endl;
    //std::cout << "-----" << std::endl;
    for (int i = 0; i < n; i++) {
        for (int j = 0; j < p; j++) {
            CI[i][j] = tmp_CI(j, i);
        }
    }
    //std::cout<<"=" << CI << std::endl;
    //std::cout << "-----" << std::endl;
    ci0.resize(p);
    {
        for (int j = 0; j < p; j++) {
            ci0[j] = 0;
        }
    }

    ci0[5] = Robot_Weight;
    ci0[11] = Robot_Weight;
    ci0[17] = Robot_Weight;
    ci0[23] = Robot_Weight;

    //    ci0[0] = 0;
    //    ci0[1] = 0;
    //    ci0[2] = 0;
    //    
    //    ci0[3] = 0;
    //    ci0[4] = 0;
    //    ci0[5] = 0;
    //
    //    ci0[6] = 0;
    //    ci0[7] = 0;
    //    ci0[8] = 0;
    //    
    //    ci0[9] = lamda * target_Contact_Info(1);
    //    ci0[10] = Robot_Weight * target_Contact_Info(1) * beta;
    //    ci0[11] = 0;

    x.resize(n);

    if (isfinite(solve_quadprog(G, g0, CE, ce0, CI, ci0, x))) {
        for (int j = 0; j < n; ++j) {
            x_saved(j) = x[j];
        }
        //std::cout<<"A"<<std::endl;
    } else {
        for (int j = 0; j < n; ++j) {
            x[j] = x_saved(j);
        }
        //std::cout<<"B"<<std::endl;
    }
    //    
    //std::cout << x_saved.transpose() << std::endl;
    //std::cout << "----------" << std::endl;

    for (int j = 0; j < n; ++j) {
        semi_F_QP_global_yaw[j] = x[j];
    }

    F_QP_local = target_C_WB_PR_12d_HS.transpose() * semi_F_QP_global_yaw;
    
    cout<<"Quad="<<semi_F_QP_global_yaw.transpose()<<endl;
    
    QUADPROGPP_Control_value_HS << 0, 0, 0, 0, 0, 0, 0, F_QP_local;

};

void CRobot::set_osqp_HS(void) {
    double Rear_body_mass = 13.826;
    double Front_body_mass = 10.507;
    double One_leg_mass = 5.295;
    double Robot_mass = Rear_body_mass + Front_body_mass + One_leg_mass * 4;
    double Robot_Weight = Robot_mass*GRAVITY;
    double one_leg_weight = 120;

    MatrixNd A_BC(6, 12);
    MatrixNd tmp_A_BC(12, 6);
    VectorNd b_BC(6);
    MatrixNd W(12, 12);
    double b1, b2, b3, b4, b5, b6;

    VectorNd r_RL_local(3);
    VectorNd r_RR_local(3);
    VectorNd r_FL_local(3);
    VectorNd r_FR_local(3);

    VectorNd r_RL(3);
    VectorNd r_RR(3);
    VectorNd r_FL(3);
    VectorNd r_FR(3);

    double alpha = 0.001;
    double beta = 1.0;
    double tmp_mu_HS = 1.0;
    W = MatrixNd::Identity(12, 12);

    r_RL_local << -0.35, 0.22, -com_height_HS;
    r_RR_local << -0.35, -0.22, -com_height_HS;
    r_FL_local << 0.35, 0.22, -com_height_HS;
    r_FR_local << 0.35, -0.22, -com_height_HS;

    r_RL = r_RL_local - offset_B2C;
    r_RR = r_RL_local - offset_B2C;
    r_FL = r_RL_local - offset_B2C;
    r_FR = r_RL_local - offset_B2C;

    A_BC << 1, 0, 0, 1, 0, 0, 1, 0, 0, 1, 0, 0\
, 0, 1, 0, 0, 1, 0, 0, 1, 0, 0, 1, 0\
, 0, 0, 1, 0, 0, 1, 0, 0, 1, 0, 0, 1\
, 0, -1 * (r_RL(2)), 1 * (r_RL(1)), 0, -1 * (r_RR(2)), 1 * (r_RR(1)), 0, -1 * (r_FL(2)), 1 * (r_FL(1)), 0, -1 * (r_FR(2)), 1 * (r_FR(1))\
, 1 * (r_RL(2)), 0, -1 * (r_RL(0)), 1 * (r_RR(2)), 0, -1 * (r_RR(0)), 1 * (r_FL(2)), 0, -1 * (r_FL(0)), 1 * (r_FR(2)), 0, -1 * (r_FR(0))\
, -1 * (r_RL(1)), 1 * (r_RL(0)), 0, -1 * (r_RR(1)), 1 * (r_RR(0)), 0, -1 * (r_FL(1)), 1 * (r_FL(0)), 0, -1 * (r_FR(1)), 1 * (r_FR(0)), 0;

    tmp_A_BC = A_BC.transpose();

    b1 = 0.0;
    b2 = 0.0;
    b3 = Robot_Weight;
    b4 = 0.0;
    b5 = 0.0;
    b6 = 0.0;

    b_BC << b1, b2, b3, b4, b5, b6;

    _P_HS = (tmp_A_BC * A_BC + alpha * W);
    _q_HS = -(tmp_A_BC * b_BC);

    // **********************Objective Function**************************//
    // ==================== OSQP  ====================== //
    int jj = 0;
    int kk = 0;
    int max_jj = 0;

    // ===================== P_x ====================== //
    for (unsigned int i = 0; i < P_nnz_HS; ++i) {
        P_x_HS[i] = _P(jj, kk);
        jj = jj + 1;

        if (jj > max_jj) {
            jj = 0;
            kk = kk + 1;
            max_jj = max_jj + 1;
        }
           //cout << "i = " << i << ", P_x = " << P_x[i] << endl;
    }

    // ===================== P_i ====================== // position
    jj = 0;
    max_jj = 0;

    for (unsigned int i = 0; i < P_nnz_HS; ++i) {
        P_i_HS[i] = jj;
        jj = jj + 1;

        if (jj > max_jj) {
            jj = 0;
            max_jj = max_jj + 1;
        }
              // cout << "i = " << i << ", P_i = " << P_i[i] << endl;
    }

    // ===================== P_p ====================== //  //cumulative value
    P_p_HS[0] = 0;
    for (unsigned int i = 1; i < n_HS + 1; ++i) {
        P_p_HS[i] = P_p_HS[i - 1] + i;

              // cout << "i = " << i << ", P_p = " << P_p[i] << endl;
    }
    // cout << "i = " << A_nnz << ", P_p = " << P_p[A_nnz] << endl;
    
    // ===================== q ====================== //
    for (unsigned int i = 0; i < n_HS; ++i) {
        q_HS[i] = _q_HS(i);
     //   cout << "i = " << i << ", q_HS = " << q_HS[i] << endl;
    }

    // **********************Constraints**************************//
    n_vec_HS << 0, 0, 1;
    b_vec_HS << 0, 1, 0;
    t_vec_HS << 1, 0, 0;

    H_RL_HS << Contact_Info_HS(0) * n_vec_HS(0), Contact_Info_HS(0) * n_vec_HS(1), Contact_Info_HS(0) * n_vec_HS(2)\
, Contact_Info_HS(0)*(-t_vec_HS(0) + tmp_mu_HS * n_vec_HS(0)), Contact_Info_HS(0)*(-t_vec_HS(1) + tmp_mu_HS * n_vec_HS(1)), Contact_Info_HS(0)*(-t_vec_HS(2) + tmp_mu_HS * n_vec_HS(2))\
, Contact_Info_HS(0)*(t_vec_HS(0) + tmp_mu_HS * n_vec_HS(0)), Contact_Info_HS(0)*(t_vec_HS(1) + tmp_mu_HS * n_vec_HS(1)), Contact_Info_HS(0)*(t_vec_HS(2) + tmp_mu_HS * n_vec_HS(2))\
, Contact_Info_HS(0)*(-b_vec_HS(0) + tmp_mu_HS * n_vec_HS(0)), Contact_Info_HS(0)*(-b_vec_HS(1) + tmp_mu_HS * n_vec_HS(1)), Contact_Info_HS(0)*(-b_vec_HS(2) + tmp_mu_HS * n_vec_HS(2))\
, Contact_Info_HS(0)*(b_vec_HS(0) + tmp_mu_HS * n_vec_HS(0)), Contact_Info_HS(0)*(b_vec_HS(1) + tmp_mu_HS * n_vec_HS(1)), Contact_Info_HS(0)*(b_vec_HS(2) + tmp_mu_HS * n_vec_HS(2))\
, 0, 0, -Contact_Info_HS(0);    

    H_RR_HS << Contact_Info_HS(1) * n_vec_HS(0), Contact_Info_HS(1) * n_vec_HS(1), Contact_Info_HS(1) * n_vec_HS(2)\
, Contact_Info_HS(1)*(-t_vec_HS(0) + tmp_mu_HS * n_vec_HS(0)), Contact_Info_HS(1)*(-t_vec_HS(1) + tmp_mu_HS * n_vec_HS(1)), Contact_Info_HS(1)*(-t_vec_HS(2) + tmp_mu_HS * n_vec_HS(2))\
, Contact_Info_HS(1)*(t_vec_HS(0) + tmp_mu_HS * n_vec_HS(0)), Contact_Info_HS(1)*(t_vec_HS(1) + tmp_mu_HS * n_vec_HS(1)), Contact_Info_HS(1)*(t_vec_HS(2) + tmp_mu_HS * n_vec_HS(2))\
, Contact_Info_HS(1)*(-b_vec_HS(0) + tmp_mu_HS * n_vec_HS(0)), Contact_Info_HS(1)*(-b_vec_HS(1) + tmp_mu_HS * n_vec_HS(1)), Contact_Info_HS(1)*(-b_vec_HS(2) + tmp_mu_HS * n_vec_HS(2))\
, Contact_Info_HS(1)*(b_vec_HS(0) + tmp_mu_HS * n_vec_HS(0)), Contact_Info_HS(1)*(b_vec_HS(1) + tmp_mu_HS * n_vec_HS(1)), Contact_Info_HS(1)*(b_vec_HS(2) + tmp_mu_HS * n_vec_HS(2))\
, 0, 0, -Contact_Info_HS(1);

    H_FL_HS << Contact_Info_HS(2) * n_vec_HS(0), Contact_Info_HS(2) * n_vec_HS(1), Contact_Info_HS(2) * n_vec_HS(2)\
, Contact_Info_HS(2)*(-t_vec_HS(0) + tmp_mu_HS * n_vec_HS(0)), Contact_Info_HS(2)*(-t_vec_HS(1) + tmp_mu_HS * n_vec_HS(1)), Contact_Info_HS(2)*(-t_vec_HS(2) + tmp_mu_HS * n_vec_HS(2))\
, Contact_Info_HS(2)*(t_vec_HS(0) + tmp_mu_HS * n_vec_HS(0)), Contact_Info_HS(2)*(t_vec_HS(1) + tmp_mu_HS * n_vec_HS(1)), Contact_Info_HS(2)*(t_vec_HS(2) + tmp_mu_HS * n_vec_HS(2))\
, Contact_Info_HS(2)*(-b_vec_HS(0) + tmp_mu_HS * n_vec_HS(0)), Contact_Info_HS(2)*(-b_vec_HS(1) + tmp_mu_HS * n_vec_HS(1)), Contact_Info_HS(2)*(-b_vec_HS(2) + tmp_mu_HS * n_vec_HS(2))\
, Contact_Info_HS(2)*(b_vec_HS(0) + tmp_mu_HS * n_vec_HS(0)), Contact_Info_HS(2)*(b_vec_HS(1) + tmp_mu_HS * n_vec_HS(1)), Contact_Info_HS(2)*(b_vec_HS(2) + tmp_mu_HS * n_vec_HS(2))\
, 0, 0, -Contact_Info_HS(2);

    H_FR_HS << Contact_Info_HS(3) * n_vec_HS(0), Contact_Info_HS(3) * n_vec_HS(1), Contact_Info_HS(3) * n_vec_HS(2)\
, Contact_Info_HS(3)*(-t_vec_HS(0) + tmp_mu_HS * n_vec_HS(0)), Contact_Info_HS(3)*(-t_vec_HS(1) + tmp_mu_HS * n_vec_HS(1)), Contact_Info_HS(3)*(-t_vec_HS(2) + tmp_mu_HS * n_vec_HS(2))\
, Contact_Info_HS(3)*(t_vec_HS(0) + tmp_mu_HS * n_vec_HS(0)), Contact_Info_HS(3)*(t_vec_HS(1) + tmp_mu_HS * n_vec_HS(1)), Contact_Info_HS(3)*(t_vec_HS(2) + tmp_mu_HS * n_vec_HS(2))\
, Contact_Info_HS(3)*(-b_vec_HS(0) + tmp_mu_HS * n_vec_HS(0)), Contact_Info_HS(3)*(-b_vec_HS(1) + tmp_mu_HS * n_vec_HS(1)), Contact_Info_HS(3)*(-b_vec_HS(2) + tmp_mu_HS * n_vec_HS(2))\
, Contact_Info_HS(3)*(b_vec_HS(0) + tmp_mu_HS * n_vec_HS(0)), Contact_Info_HS(3)*(b_vec_HS(1) + tmp_mu_HS * n_vec_HS(1)), Contact_Info_HS(3)*(b_vec_HS(2) + tmp_mu_HS * n_vec_HS(2))\
, 0, 0, -Contact_Info_HS(3);
   // std::cout << "H_RL=" << endl << H_RL << std::endl;
    // ===================== A_x ====================== //

    jj = 0;
    kk = 0;

    for (unsigned int i = 0; i < A_nnz_HS; ++i) { //A_nnz_HS=72(18*4)
        if (i <= 18) {
            if (i <= 17) {
                A_x_HS[i] = H_RL_HS(jj, kk);
                jj = jj + 1;
                if (jj == 6) {
                    jj = 0;
                    kk = kk + 1;
                }
                if (kk == 3) {
                    kk = 0;
                    jj = 0;
                }
            }
            if (i == 18) {
                A_x_HS[i] = -Contact_Info_HS(0);
            }
        } else if (i <= 37) {
            if (i <= 36) {
                A_x_HS[i] = H_RR_HS(jj, kk);
                jj = jj + 1;
                if (jj == 6) {
                    jj = 0;
                    kk = kk + 1;
                }
                if (kk == 3) {
                    kk = 0;
                    jj = 0;
                }
            }
            if (i == 37) {
                A_x_HS[i] = -Contact_Info_HS(1);
            }   
        } else if (i <= 56) {
            if (i <= 55) {
                A_x_HS[i] = H_FL_HS(jj, kk);
                jj = jj + 1;
                if (jj == 6) {
                    jj = 0;
                    kk = kk + 1;
                }
                if (kk == 3) {
                    kk = 0;
                    jj = 0;
                }
            }
            if (i == 56) {
                A_x_HS[i] = -Contact_Info_HS(2);
            }   
        } else {
            if (i <= 74) {
                A_x_HS[i] = H_FR_HS(jj, kk);
                jj = jj + 1;
                if (jj == 6) {
                    jj = 0;
                    kk = kk + 1;
                }
                if (kk == 3) {
                    kk = 0;
                    jj = 0;
                }
            }
            if (i == 75) {
                A_x_HS[i] = -Contact_Info_HS(3);
            }
        } 
        cout << "i = " << i << ", A_x_HS = " << A_x_HS[i] << endl;
    }

    // ===================== A_i ====================== //
    jj = 0;
    for (unsigned int i = 0; i < A_nnz_HS; ++i) {
        if (i <= 18) {
            if (i <= 17) {
                A_i_HS[i] = jj;
                jj = jj + 1;
                if (jj == 6) {
                    jj = 0;
                }
            }
            if (i == 18) {
                A_i_HS[i] = 24;
            }
        } else if (i <= 37) {
            if (i <= 36) {
                A_i_HS[i] = jj + 6;
                jj = jj + 1;
                if (jj == 6) {
                    jj = 0;
                }
            }
            if (i == 37) {
                A_i_HS[i] = 24;
            }
        } else if (i <= 56) {
            if (i <= 55) {
                A_i_HS[i] = jj + 12;
                jj = jj + 1;
                if (jj == 6) {
                    jj = 0;
                }
            }
            if (i == 56) {
                A_i_HS[i] = 24;
            }
        } else {
            if (i <= 74) {
                A_i_HS[i] = jj + 18;
                jj = jj + 1;
                if (jj == 6) {
                    jj = 0;
                }
            }
            if (i == 75) {
                A_i_HS[i] = 24;
            }
        }
        //cout << "i = " << i << ", A_i = " << A_i_HS[i] << endl;
    }

    // ===================== A_p ====================== //

    for (unsigned int i = 0; i <= n_HS; ++i) {
        A_p_HS[i] = num_HS;
        
        if (i == 2 || i == 5 || i == 8 || i == 11) {
            num_HS = num_HS + 7;
        } else {
            num_HS = num_HS + 6;
        }
        
        if (i == 12) {
            num_HS = 0;
        }
       // cout << "i = " << i << ", A_p_HS = " << A_p_HS[i] << endl;
    }
    //        cout << "i = " << A_nnz << ", A_p = " << A_p[A_nnz] << endl;

    // ===================== G_l & G_u ====================== //
    for (unsigned int i = 0; i < m_HS; ++i) {
        if (i == 5 || i == 11 || i == 17 || i == 23) {
            l_HS[i] = -Robot_Weight * 1.0;
        } else if (i == 24) {
            l_HS[i] = -Robot_Weight * 1.5;
        } else {
            l_HS[i] = 0.0;
        }

        if (i == 5 || i == 11 || i == 17 || i == 23) {
            u_HS[i] = 0.0;
        } else if (i == 24) {
            u_HS[i] = -Robot_Weight * 1.0;
        } else {
            u_HS[i] = 1000;
        }

       // cout << "i = " << i << ", G_l = " << l_HS[i] << endl;
       // cout << "i = " << i << ", G_u = " << u_HS[i] << endl;
        // cout << "==============================" << endl;
    }

    // Load problem data
    // Exitflag
    c_int exitflag = 0;

    // Populate data
    if (data_HS) {
        data_HS->n = n_HS; //12
        data_HS->m = m_HS; //25
        data_HS->P = csc_matrix(data_HS->n, data_HS->n, P_nnz_HS, P_x_HS, P_i_HS, P_p_HS);
        data_HS->q = q_HS;
        data_HS->A = csc_matrix(data_HS->m, data_HS->n, A_nnz_HS, A_x_HS, A_i_HS, A_p_HS);
        data_HS->l = l_HS;
        data_HS->u = u_HS;
    }

    // Define solver settings as default
    if (settings_HS) {
        osqp_set_default_settings(settings_HS);
        settings_HS->alpha = 1.0; // Change alpha parameter
    }

    // Setup workspace
    exitflag = osqp_setup(&work_HS, data_HS, settings_HS);

    // Solve Problem
    osqp_solve(work_HS);
    cout << "[RL] x = " << work_HS->solution->x[0] << ", y = " << work_HS->solution->x[1] << ", z = " << work_HS->solution->x[2] << endl;
    cout << "[RR] x = " << work_HS->solution->x[3] << ", y = " << work_HS->solution->x[4] << ", z = " << work_HS->solution->x[5] << endl;
    cout << "[FL] x = " << work_HS->solution->x[6] << ", y = " << work_HS->solution->x[7] << ", z = " << work_HS->solution->x[8] << endl;
    cout << "[FR] x = " << work_HS->solution->x[9] << ", y = " << work_HS->solution->x[10] << ", z = " << work_HS->solution->x[11] << endl;
    // Cleanup
    if (data_HS) {
        if (data_HS->A) c_free(data_HS->A);
        if (data_HS->P) c_free(data_HS->P);
        c_free(data_HS);
    }
    if (settings_HS) c_free(settings_HS);
}

void CRobot::Get_Opt_F_HS3(void) {
    double Fz_RL_max, Fz_RL_min;
    double Fz_RR_max, Fz_RR_min;
    double Fz_FL_max, Fz_FL_min;
    double Fz_FR_max, Fz_FR_min;

    double Rear_body_mass = 13.826;
    double Front_body_mass = 10.507;
    double One_leg_mass = 5.295;
    double Robot_mass = Rear_body_mass + Front_body_mass + One_leg_mass * 4;
    double Robot_Weight = Robot_mass*GRAVITY;

    double one_leg_weight = 120;

    MatrixNd A_BC(6, 12);
    MatrixNd tmp_A_BC(12, 6);
    VectorNd b_BC(6);
    MatrixNd W(12, 12);

    VectorNd r_RL(3);
    VectorNd r_RR(3);
    VectorNd r_FL(3);
    VectorNd r_FR(3);
    VectorNd r_COM(3);

    VectorNd k_p_QP(6);
    VectorNd k_d_QP(6);
    double b1, b2, b3, b4, b5, b6;

    VectorNd F_QP_global(12);
    VectorNd F_QP_local(12);

    double alpha = 0.001;
    double beta = 1.0;
    double tmp_mu_HS = 0.0;
    W = MatrixNd::Identity(12, 12);

    VectorNd target_base_pos_rel(3);
    VectorNd actual_base_pos_rel(3);
    VectorNd target_com_pos_rel(3);
    VectorNd actual_com_pos_rel(3);

    VectorNd target_base_vel_rel(3);
    VectorNd actual_base_vel_rel(3);

    VectorNd Err_com_pos(3);
    VectorNd Err_com_vel(3);

    VectorNd standard_info_HS(4);
    double standard_leg_height_HS;
   
    VectorNd r_com_EP_local(12);
    VectorNd r_com_EP_semi_global(12);
    VectorNd r_com_RL(3);
    VectorNd r_com_RR(3);
    VectorNd r_com_FL(3);
    VectorNd r_com_FR(3);

    VectorNd tmp_b = VectorNd::Zero(3);
    VectorNd tmp_t = VectorNd::Zero(3);
    
    semi_target_com_acc_HS_yaw << target_C_WB_Y_2d_HS.transpose() * target_com_acc_HS.segment(0, 2), target_com_acc_HS(2);

    k_p_QP << 10, 10, 4000, 3000, 3000, 0;
    k_d_QP << 0.1, 0.1, 40, 30, 30, 10;

    r_com_EP_local << actual_EP_local_HS.segment(0, 3) - offset_B2C, actual_EP_local_HS.segment(3, 3) - offset_B2C, actual_EP_local_HS.segment(6, 3) - offset_B2C, actual_EP_local_HS.segment(9, 3) - offset_B2C;
    r_com_EP_semi_global = target_C_WB_PR_12d_HS*r_com_EP_local;

    r_com_RL << r_com_EP_semi_global(0), r_com_EP_semi_global(1), r_com_EP_semi_global(2);
    r_com_RR << r_com_EP_semi_global(3), r_com_EP_semi_global(4), r_com_EP_semi_global(5);
    r_com_FL << r_com_EP_semi_global(6), r_com_EP_semi_global(7), r_com_EP_semi_global(8);
    r_com_FR << r_com_EP_semi_global(9), r_com_EP_semi_global(10), r_com_EP_semi_global(11);

    A_BC << Contact_Info_HS(0), 0, 0, Contact_Info_HS(1), 0, 0, Contact_Info_HS(2), 0, 0, Contact_Info_HS(3), 0, 0\
, 0, Contact_Info_HS(0), 0, 0, Contact_Info_HS(1), 0, 0, Contact_Info_HS(2), 0, 0, Contact_Info_HS(3), 0\
, 0, 0, Contact_Info_HS(0), 0, 0, Contact_Info_HS(1), 0, 0, Contact_Info_HS(2), 0, 0, Contact_Info_HS(3)\
          , 0, -Contact_Info_HS(0) * (r_com_RL(2)), Contact_Info_HS(0) * (r_com_RL(1)), 0, -Contact_Info_HS(1) * (r_com_RR(2)), Contact_Info_HS(1) * (r_com_RR(1)), 0, -Contact_Info_HS(2) * (r_com_FL(2)), Contact_Info_HS(2) * (r_com_FL(1)), 0, -Contact_Info_HS(3) * (r_com_FR(2)), Contact_Info_HS(3) * (r_com_FR(1))\
          , Contact_Info_HS(0) * (r_com_RL(2)), 0, -Contact_Info_HS(0)*(r_com_RL(0)), Contact_Info_HS(1) * (r_com_RR(2)), 0, -Contact_Info_HS(1)*(r_com_RR(0)), Contact_Info_HS(2) * (r_com_FL(2)), 0, -Contact_Info_HS(2)*(r_com_FL(0)), Contact_Info_HS(3) * (r_com_FR(2)), 0, -Contact_Info_HS(3)*(r_com_FR(0))\
          , -Contact_Info_HS(0) * (r_com_RL(1)), Contact_Info_HS(0) * (r_com_RL(0)), 0, -Contact_Info_HS(1) * (r_com_RR(1)), Contact_Info_HS(1)*(r_com_RR(0)), 0, -Contact_Info_HS(2) * (r_com_FL(1)), Contact_Info_HS(2)*(r_com_FL(0)), 0, -Contact_Info_HS(3) * (r_com_FR(1)), Contact_Info_HS(3)*(r_com_FR(0)), 0;

    tmp_A_BC = A_BC.transpose();

    target_base_pos_rel = -(Contact_Info_HS(0) * target_EP_local_HS.segment(0, 3) + Contact_Info_HS(1) * target_EP_local_HS.segment(3, 3) + Contact_Info_HS(2) * target_EP_local_HS.segment(6, 3) + Contact_Info_HS(3) * target_EP_local_HS.segment(9, 3)) / (Contact_Info_HS(0) + Contact_Info_HS(1) + Contact_Info_HS(2) + Contact_Info_HS(3));
    actual_base_pos_rel = -(Contact_Info_HS(0) * actual_EP_local_HS.segment(0, 3) + Contact_Info_HS(1) * actual_EP_local_HS.segment(3, 3) + Contact_Info_HS(2) * actual_EP_local_HS.segment(6, 3) + Contact_Info_HS(3) * actual_EP_local_HS.segment(9, 3)) / (Contact_Info_HS(0) + Contact_Info_HS(1) + Contact_Info_HS(2) + Contact_Info_HS(3));

    target_base_vel_rel = -(Contact_Info_HS(0) * target_EP_vel_local_HS.segment(0, 3) + Contact_Info_HS(1) * target_EP_vel_local_HS.segment(3, 3) + Contact_Info_HS(2) * target_EP_vel_local_HS.segment(6, 3) + Contact_Info_HS(3) * target_EP_vel_local_HS.segment(9, 3)) / (Contact_Info_HS(0) + Contact_Info_HS(1) + Contact_Info_HS(2) + Contact_Info_HS(3));
    actual_base_vel_rel = -(Contact_Info_HS(0) * actual_EP_vel_local_HS.segment(0, 3) + Contact_Info_HS(1) * actual_EP_vel_local_HS.segment(3, 3) + Contact_Info_HS(2) * actual_EP_vel_local_HS.segment(6, 3) + Contact_Info_HS(3) * actual_EP_vel_local_HS.segment(9, 3)) / (Contact_Info_HS(0) + Contact_Info_HS(1) + Contact_Info_HS(2) + Contact_Info_HS(3));

    target_com_pos_rel = Get_COM_pos_HS2(target_base_pos_rel);
    actual_com_pos_rel = Get_COM_pos_HS2(actual_base_pos_rel);

    Err_com_pos = target_C_WB_PR_HS * (target_com_pos_rel - actual_com_pos_rel);
    Err_com_vel = target_C_WB_PR_HS * (target_base_vel_rel - actual_base_vel_rel);

    target_plane_dist_HS = com_height_HS;
    actual_plane_dist_HS = actual_base_pos_HS(2) + offset_B2C(2) / cos(abs(actual_base_ori_local_HS(1)));

    if (initial_flag_HS == true) {
        pre_actual_plane_dist_HS = actual_plane_dist_HS;
    }
    actual_plane_dist_vel_HS = (actual_plane_dist_HS - pre_actual_plane_dist_HS) / dt;
    pre_actual_plane_dist_HS = actual_plane_dist_HS;

    b1 = Robot_mass * (semi_target_com_acc_HS_yaw(0)) + k_p_QP(0) * Err_com_pos(0) + k_d_QP(0) * Err_com_vel(0);
    b2 = Robot_mass * (semi_target_com_acc_HS_yaw(1)) + k_p_QP(1) * Err_com_pos(1) + k_d_QP(1) * Err_com_vel(1);
    b3 = Robot_mass * (semi_target_com_acc_HS_yaw(2)) + k_p_QP(2)*(target_plane_dist_HS - actual_plane_dist_HS) + k_d_QP(2)*(0.0 - actual_plane_dist_vel_HS) + Robot_Weight;

    b4 = k_p_QP(3)*(target_base_ori_local_HS(0) - actual_base_ori_local_HS(0)) + k_d_QP(3)*(target_base_ori_vel_local_HS(0) - actual_base_ori_vel_local_HS(0));
    b5 = k_p_QP(4)*(target_base_ori_local_HS(1) - actual_base_ori_local_HS(1)) + k_d_QP(4)*(target_base_ori_vel_local_HS(1) - actual_base_ori_vel_local_HS(1));
    b6 = k_d_QP(5)*(target_base_ori_vel_local_HS(2) - actual_base_ori_vel_local_HS(2));

    b_BC << b1, b2, b3, b4, b5, b6;

    _P = (tmp_A_BC * A_BC + alpha * W);
    _q = -(tmp_A_BC * b_BC);

    
    // ===================== OSQP  ====================== //
    int jj = 0;
    int kk = 0;
    int max_jj = 0;

    for (unsigned int i = 0; i < P_nnz_HS; ++i) {
        P_x_HS[i] = _P(jj, kk);
        jj = jj + 1;

        if (jj > max_jj) {
            jj = 0;
            kk = kk + 1;
            max_jj = max_jj + 1;
        }
           //cout << "i = " << i << ", P_x = " << P_x[i] << endl;
    }

     // ===================== q ====================== //
    for (unsigned int i = 0; i < n_HS; ++i) {
        q_HS[i] = _q(i);
     //   cout << "i = " << i << ", q_HS = " << q_HS[i] << endl;
    }
    
    // ===================== A_x ====================== //
    n_vec_HS = semi_w_n / sqrt(semi_w_n(0) * semi_w_n(0) + semi_w_n(1) * semi_w_n(1) + semi_w_n(2) * semi_w_n(2));
    tmp_b << 0, 1, 0;
    tmp_t << 1, 0, 0;

    b_vec_HS = target_C_WB_PR_HS*tmp_b;
    t_vec_HS = target_C_WB_PR_HS*tmp_t;
    tmp_mu_HS = 1.0 / sqrt(2);

    H_RL_HS << Contact_Info_HS(0) * n_vec_HS(0), Contact_Info_HS(0) * n_vec_HS(1), Contact_Info_HS(0) * n_vec_HS(2)\
, Contact_Info_HS(0)*(-t_vec_HS(0) + tmp_mu_HS * n_vec_HS(0)), Contact_Info_HS(0)*(-t_vec_HS(1) + tmp_mu_HS * n_vec_HS(1)), Contact_Info_HS(0)*(-t_vec_HS(2) + tmp_mu_HS * n_vec_HS(2))\
, Contact_Info_HS(0)*(t_vec_HS(0) + tmp_mu_HS * n_vec_HS(0)), Contact_Info_HS(0)*(t_vec_HS(1) + tmp_mu_HS * n_vec_HS(1)), Contact_Info_HS(0)*(t_vec_HS(2) + tmp_mu_HS * n_vec_HS(2))\
, Contact_Info_HS(0)*(-b_vec_HS(0) + tmp_mu_HS * n_vec_HS(0)), Contact_Info_HS(0)*(-b_vec_HS(1) + tmp_mu_HS * n_vec_HS(1)), Contact_Info_HS(0)*(-b_vec_HS(2) + tmp_mu_HS * n_vec_HS(2))\
, Contact_Info_HS(0)*(b_vec_HS(0) + tmp_mu_HS * n_vec_HS(0)), Contact_Info_HS(0)*(b_vec_HS(1) + tmp_mu_HS * n_vec_HS(1)), Contact_Info_HS(0)*(b_vec_HS(2) + tmp_mu_HS * n_vec_HS(2))\
, 0, 0, -Contact_Info_HS(0);

    H_RR_HS << Contact_Info_HS(1) * n_vec_HS(0), Contact_Info_HS(1) * n_vec_HS(1), Contact_Info_HS(1) * n_vec_HS(2)\
, Contact_Info_HS(1)*(-t_vec_HS(0) + tmp_mu_HS * n_vec_HS(0)), Contact_Info_HS(1)*(-t_vec_HS(1) + tmp_mu_HS * n_vec_HS(1)), Contact_Info_HS(1)*(-t_vec_HS(2) + tmp_mu_HS * n_vec_HS(2))\
, Contact_Info_HS(1)*(t_vec_HS(0) + tmp_mu_HS * n_vec_HS(0)), Contact_Info_HS(1)*(t_vec_HS(1) + tmp_mu_HS * n_vec_HS(1)), Contact_Info_HS(1)*(t_vec_HS(2) + tmp_mu_HS * n_vec_HS(2))\
, Contact_Info_HS(1)*(-b_vec_HS(0) + tmp_mu_HS * n_vec_HS(0)), Contact_Info_HS(1)*(-b_vec_HS(1) + tmp_mu_HS * n_vec_HS(1)), Contact_Info_HS(1)*(-b_vec_HS(2) + tmp_mu_HS * n_vec_HS(2))\
, Contact_Info_HS(1)*(b_vec_HS(0) + tmp_mu_HS * n_vec_HS(0)), Contact_Info_HS(1)*(b_vec_HS(1) + tmp_mu_HS * n_vec_HS(1)), Contact_Info_HS(1)*(b_vec_HS(2) + tmp_mu_HS * n_vec_HS(2))\
, 0, 0, -Contact_Info_HS(1);

    H_FL_HS << Contact_Info_HS(2) * n_vec_HS(0), Contact_Info_HS(2) * n_vec_HS(1), Contact_Info_HS(2) * n_vec_HS(2)\
, Contact_Info_HS(2)*(-t_vec_HS(0) + tmp_mu_HS * n_vec_HS(0)), Contact_Info_HS(2)*(-t_vec_HS(1) + tmp_mu_HS * n_vec_HS(1)), Contact_Info_HS(2)*(-t_vec_HS(2) + tmp_mu_HS * n_vec_HS(2))\
, Contact_Info_HS(2)*(t_vec_HS(0) + tmp_mu_HS * n_vec_HS(0)), Contact_Info_HS(2)*(t_vec_HS(1) + tmp_mu_HS * n_vec_HS(1)), Contact_Info_HS(2)*(t_vec_HS(2) + tmp_mu_HS * n_vec_HS(2))\
, Contact_Info_HS(2)*(-b_vec_HS(0) + tmp_mu_HS * n_vec_HS(0)), Contact_Info_HS(2)*(-b_vec_HS(1) + tmp_mu_HS * n_vec_HS(1)), Contact_Info_HS(2)*(-b_vec_HS(2) + tmp_mu_HS * n_vec_HS(2))\
, Contact_Info_HS(2)*(b_vec_HS(0) + tmp_mu_HS * n_vec_HS(0)), Contact_Info_HS(2)*(b_vec_HS(1) + tmp_mu_HS * n_vec_HS(1)), Contact_Info_HS(2)*(b_vec_HS(2) + tmp_mu_HS * n_vec_HS(2))\
, 0, 0, -Contact_Info_HS(2);

    H_FR_HS << Contact_Info_HS(3) * n_vec_HS(0), Contact_Info_HS(3) * n_vec_HS(1), Contact_Info_HS(3) * n_vec_HS(2)\
, Contact_Info_HS(3)*(-t_vec_HS(0) + tmp_mu_HS * n_vec_HS(0)), Contact_Info_HS(3)*(-t_vec_HS(1) + tmp_mu_HS * n_vec_HS(1)), Contact_Info_HS(3)*(-t_vec_HS(2) + tmp_mu_HS * n_vec_HS(2))\
, Contact_Info_HS(3)*(t_vec_HS(0) + tmp_mu_HS * n_vec_HS(0)), Contact_Info_HS(3)*(t_vec_HS(1) + tmp_mu_HS * n_vec_HS(1)), Contact_Info_HS(3)*(t_vec_HS(2) + tmp_mu_HS * n_vec_HS(2))\
, Contact_Info_HS(3)*(-b_vec_HS(0) + tmp_mu_HS * n_vec_HS(0)), Contact_Info_HS(3)*(-b_vec_HS(1) + tmp_mu_HS * n_vec_HS(1)), Contact_Info_HS(3)*(-b_vec_HS(2) + tmp_mu_HS * n_vec_HS(2))\
, Contact_Info_HS(3)*(b_vec_HS(0) + tmp_mu_HS * n_vec_HS(0)), Contact_Info_HS(3)*(b_vec_HS(1) + tmp_mu_HS * n_vec_HS(1)), Contact_Info_HS(3)*(b_vec_HS(2) + tmp_mu_HS * n_vec_HS(2))\
, 0, 0, -Contact_Info_HS(3);
    
    jj = 0;
    kk = 0;

    for (unsigned int i = 0; i < A_nnz_HS; ++i) { //A_nnz_HS=72(18*4)
        if (i <= 18) {
            if (i <= 17) {
                A_x_HS[i] = H_RL_HS(jj, kk);
                jj = jj + 1;
                if (jj == 6) {
                    jj = 0;
                    kk = kk + 1;
                }
                if (kk == 3) {
                    kk = 0;
                    jj = 0;
                }
            }
            if (i == 18) {
                A_x_HS[i] = -Contact_Info_HS(0);
            }
        } else if (i <= 37) {
            if (i <= 36) {
                A_x_HS[i] = H_RR_HS(jj, kk);
                jj = jj + 1;
                if (jj == 6) {
                    jj = 0;
                    kk = kk + 1;
                }
                if (kk == 3) {
                    kk = 0;
                    jj = 0;
                }
            }
            if (i == 37) {
                A_x_HS[i] = -Contact_Info_HS(1);
            }   
        } else if (i <= 56) {
            if (i <= 55) {
                A_x_HS[i] = H_FL_HS(jj, kk);
                jj = jj + 1;
                if (jj == 6) {
                    jj = 0;
                    kk = kk + 1;
                }
                if (kk == 3) {
                    kk = 0;
                    jj = 0;
                }
            }
            if (i == 56) {
                A_x_HS[i] = -Contact_Info_HS(2);
            }   
        } else {
            if (i <= 74) {
                A_x_HS[i] = H_FR_HS(jj, kk);
                jj = jj + 1;
                if (jj == 6) {
                    jj = 0;
                    kk = kk + 1;
                }
                if (kk == 3) {
                    kk = 0;
                    jj = 0;
                }
            }
            if (i == 75) {
                A_x_HS[i] = -Contact_Info_HS(3);
            }
        } 
    }
    
     for (unsigned int i = 0; i < m_HS; ++i) {
        if (i == 5 || i == 11 || i == 17 || i == 23) {
            l_HS[i] = -Robot_Weight * 1.0;
        } else if (i == 24) {
            l_HS[i] = -Robot_Weight * 1.5;
        } else {
            l_HS[i] = 0.0;
        }

        if (i == 5 || i == 11 || i == 17 || i == 23) {
            u_HS[i] = 0.0;
        } else if (i == 24) {
            //u_HS[i] = -Robot_Weight * 1.0;
            u_HS[i] = -Robot_Weight * 0.8;
        } else {
            u_HS[i] = 1000;
        }
    }

    osqp_update_P(work_HS, P_x_HS, OSQP_NULL, 78);
    osqp_update_lin_cost(work_HS, q_HS);
    osqp_update_A(work_HS, A_x_HS, OSQP_NULL, 76);
    osqp_update_bounds(work_HS, l_HS, u_HS);

    // Solve updated problem
    osqp_solve(work_HS);

    for (unsigned int i = 0; i < n_HS; ++i) {

        semi_F_QP_global_yaw(i) = work_HS->solution->x[i];
    }

    F_QP_local = target_C_WB_PR_12d_HS.transpose() * semi_F_QP_global_yaw;
    //cout<<"osqp="<<semi_F_QP_global_yaw.transpose()<<endl;
    //cout << "Fz=" << semi_F_QP_global_yaw(2) + semi_F_QP_global_yaw(5) + semi_F_QP_global_yaw(8) + semi_F_QP_global_yaw(11) << endl;
    OSQP_Control_value_HS << 0, 0, 0, 0, 0, 0, 0, F_QP_local;
}
