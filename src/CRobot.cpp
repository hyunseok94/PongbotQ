
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
//    Mode = MODE_ACTUAL_ROBOT;

    // ============== Controller OnOff ================ //
    Base_Ori_Con_onoff_flag = false; //false; //true; //true;
    CP_con_onoff_flag = false; //true; //false;//true;
    // ============== Controller OnOff End ================ //

    RL_base2hip_pos << -0.350, 0.115, -0.053;
    RR_base2hip_pos << -0.350, -0.115, -0.053;
    FL_base2hip_pos << 0.350, 0.115, -0.053;
    FR_base2hip_pos << 0.350, -0.115, -0.053;

    base2hip_pos << RL_base2hip_pos, RR_base2hip_pos, FL_base2hip_pos, FR_base2hip_pos;

    if (Mode == MODE_SIMULATION) {
        com_height = 0.40;
        foot_height = 0.06;
        swing_foot_height = 0.06;

        // global init foot position
        tar_init_RL_foot_pos << RL_base2hip_pos(0), RL_base2hip_pos(1) + 0.105 - 0.0, 0.0;
        tar_init_RR_foot_pos << RR_base2hip_pos(0), RR_base2hip_pos(1) - 0.105 + 0.0, 0.0;
        tar_init_FL_foot_pos << FL_base2hip_pos(0), FL_base2hip_pos(1) + 0.105 - 0.0, 0.0;
        tar_init_FR_foot_pos << FR_base2hip_pos(0), FR_base2hip_pos(1) - 0.105 + 0.0, 0.0;

        Kp_q << 200, 250, 250, 200, 250, 250, 15000, 200, 250, 250, 200, 250, 250;
        Kd_q << 4, 5, 5, 4, 5, 5, 400, 4, 5, 5, 4, 5, 5;

        //        FT_Kp_q << 400, 500, 500, 400, 500, 500, 5000, 400, 500, 500, 400, 500, 500;
        //        FT_Kd_q << 10, 12, 12, 10, 12, 12, 100, 10, 12, 12, 10, 12, 12;

        // 2020.04.20
        //        Kp_t << 2500, 2500, 4000, 2500, 2500, 4000, 2500, 2500, 4000, 2500, 2500, 4000;
        //        Kd_t << 100, 100, 150, 100, 100, 150, 100, 100, 150, 100, 100, 150;
        //
        //        Kp_x << 100, 0, 0, 0, 100, 0, 0, 0, 50;
        //        Kd_x << 1.0, 0, 0, 0, 1.0, 0, 0, 0, 0.5;
        //        Kp_w << 400, 0, 0, 0, 200, 0, 0, 0, 0;
        //        Kd_w << 6, 0, 0, 0, 1.0, 0, 0, 0, 0;

        // =========== Walking ============ //
        //        Kp_t << 2000, 2000, 4000,
        //                2000, 2000, 4000,
        //                2000, 2000, 4000,
        //                2000, 2000, 4000;
        //        Kd_t << 100, 100, 150,
        //                100, 100, 150,
        //                100, 100, 150,
        //                100, 100, 150;
        //        
        //        Kp_x << 10, 0, 0,
        //                0, 10, 0,
        //                0, 0, 10;
        //        Kd_x << 0.1, 0, 0,
        //                0, 0.1, 0,
        //                0, 0, 0.1;
        //        Kp_w << 400, 0, 0,
        //                0, 400, 0,
        //                0, 0, 0;
        //        Kd_w << 2, 0, 0,
        //                0, 2.0, 0,
        //                0, 0, 0;



        // =========== Walking ============ // 
        // 10000/472
        //  2020.06.17
        Kp_t << 4000, 4000, 4000,
                4000, 4000, 4000,
                4000, 4000, 4000,
                4000, 4000, 4000;
        Kd_t << 300, 300, 300,
                300, 300, 300,
                300, 300, 300,
                300, 300, 300;

        Kp_x << 0, 0, 0,
                0, 0, 0,
                0, 0, 0;
        Kd_x << 0, 0, 0,
                0, 0, 0,
                0, 0, 0;
        Kp_w << 0, 0, 0,
                0, 0, 0,
                0, 0, 0;
        Kd_w << 0, 0, 0,
                0, 0, 0,
                0, 0, 0;
        
//        Kp_x << 1, 0, 0,
//                0, 1, 0,
//                0, 0, 1;
//        Kd_x << 0.01, 0, 0,
//                0, 0.01, 0,
//                0, 0, 0.01;
//        Kp_w << 1000, 0, 0,
//                0, 100, 0,
//                0, 0, 0;
//        Kd_w << 5, 0, 0,
//                0, 1, 0,
//                0, 0, 0;



        //        alpha_act_com_x = 0.1, alpha_act_com_y = 0.1;

        //        weight_cp_x = 0.0, weight_cp_y = 1.0;
        w_cp_y1 = 1.0;
        w_cp_y2 = 0.8;
        cp_y_max = 0.08, cp_y_min = 0.03;

        BOC_Kp_roll = 0; //0.0010;
        BOC_Ki_roll = 0; //0.10; //0.003
        BOC_Kp_pitch = 0; //0.0010;
        BOC_Ki_pitch = 0; //0.10;

    }
    else if (Mode == MODE_ACTUAL_ROBOT) {
        com_height = 0.40;
        foot_height = 0.045;
        swing_foot_height = 0.07;

        // global init foot position
        //        tar_init_RL_foot_pos << RL_base2hip_pos(0), RL_base2hip_pos(1) + 0.105 + 0.03, 0.0;
        //        tar_init_RR_foot_pos << RR_base2hip_pos(0), RR_base2hip_pos(1) - 0.105 - 0.01, 0.0;
        tar_init_RL_foot_pos << RL_base2hip_pos(0), RL_base2hip_pos(1) + 0.105 + 0.0, -0.0;
        tar_init_RR_foot_pos << RR_base2hip_pos(0), RR_base2hip_pos(1) - 0.105 - 0.0, -0.0;
        tar_init_FL_foot_pos << FL_base2hip_pos(0), FL_base2hip_pos(1) + 0.105, -0.0;
        tar_init_FR_foot_pos << FR_base2hip_pos(0), FR_base2hip_pos(1) - 0.105, -0.0;

        Kp_q << 200, 400, 400, 200, 400, 400, 15000, 200, 400, 400, 200, 400, 400;
        Kd_q << 3, 10, 10, 3, 10, 10, 300, 3, 10, 10, 3, 10, 10;

        //      FT_Kp_q << 300, 500, 500, 300, 500, 500, 5000, 300, 500, 500, 300, 500, 500;
        //      FT_Kd_q << 5, 12, 12, 5, 12, 12, 100, 5, 12, 12, 5, 12, 12;

        //      Kp_t << 1500, 1500, 2500, 1500, 1500, 2500, 1500, 1500, 2500, 1500, 1500, 2500;
        //      Kd_t << 50, 50, 100, 50, 50, 100, 50, 50, 100, 50, 50, 100;

        //	    Kp_t << 2500, 2500, 5000, 2500, 2500, 5000, 2500, 2500, 5000, 2500, 2500, 5000;
        //		Kd_t << 100, 100, 200, 100, 100, 200, 100, 100, 200, 100, 100, 200;

        //        Kp_t << 2500, 2500, 4000, 2500, 2500, 4000, 2500, 2500, 4000, 2500, 2500, 4000;
        //        Kd_t << 100, 100, 150, 100, 100, 150, 100, 100, 150, 100, 100, 150;

        //        Kp_t << 2500, 2500, 2500, 2500, 2500, 2500, 2500, 2500, 2500, 2500, 2500, 4000;
        //        Kd_t << 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 150;

        //        Kp_t << 1500, 1500, 3500, 1500, 1500, 3500, 1500, 1500, 3500, 1500, 1500, 5500;
        //        Kd_t << 50, 50, 130, 50, 50, 130, 50, 50, 130, 50, 50, 150;

        //        Kp_t << 2000, 2000, 4000,
        //        		2000, 2000, 4000,
        //        		2000, 2000, 4000,
        //        		2000, 2000, 5200;
        //        Kd_t << 80, 80, 140,
        //                80, 80, 140,
        //                80, 80, 140,
        //                80, 80, 150;


        //        Kp_t << 2000, 2500, 4000,
        //				2000, 2500, 4000,
        //				2000, 2500, 4000,
        //				2000, 2500, 4000;
        //		Kd_t << 80, 90, 140,
        //				80, 90, 140,
        //				80, 90, 140,
        //				80, 90, 140;

        // 2020.06.16
        //        Kp_t << 2000, 2500, 3500,
        //                2000, 2500, 3500,
        //                2000, 2500, 3500,
        //                2000, 2500, 4500;
        //        Kd_t << 80, 100, 120,
        //                80, 100, 120,
        //                80, 100, 120,
        //                80, 100, 150;

        //        Kp_x << 1, 0, 0,
        //                0, 1, 0,
        //                0, 0, 1;
        //        Kd_x << 0.01, 0, 0,
        //                0, 0.02, 0,
        //                0, 0, 0.01;
        //        Kp_w << 300, 0, 0,
        //                0, 100, 0,
        //                0, 0, 0;
        //        Kd_w << 0.30, 0, 0,
        //                0, 0.10, 0,
        //                0, 0, 0;

        // 2020.06.22
//        Kp_t << 4000, 4000, 4500,
//                4000, 4000, 4500,
//                4000, 4000, 4500,
//                5000, 4000, 5500;
//        Kd_t << 150, 150, 150,
//                150, 150, 150,
//                150, 150, 150,
//                170, 150, 200;
//        
        Kp_x << 1, 0, 0,
                0, 1, 0,
                0, 0, 100;
        Kd_x << 0.1, 0, 0,
                0, 0.1, 0,
                0, 0, 1.0;
        Kp_w << 800, 0, 0,
                0, 800, 0,
                0, 0, 0;
        Kd_w << 1.0, 0, 0,
                0, 1, 0,
                0, 0, 0;


//        Kp_t << 5000, 4000, 9500,
//                5000, 4000, 9500,
//                5000, 4000, 9500,
//                5000, 4000, 9500;
//        Kd_t << 100, 80, 300,
//                100, 80, 300,
//                100, 80, 300,
//                100, 80, 300;


//        Kp_t << 5000, 5000, 4000,
//        		5000, 5000, 3000,
//        		5000, 5000, 3000,
//        		5000, 5000, 4000;
//        Kd_t << 100, 100, 80,
//                100, 100, 50,
//                100, 100, 50,
//                100, 100, 80;

        Kp_t << 5000, 5000, 2000,
        		5000, 5000, 2000,
        		5000, 5000, 2000,
        		5000, 5000, 2000;
        Kd_t << 100, 100, 20,
                100, 100, 20,
                100, 100, 20,
                100, 100, 20;

//        Kp_x << 0, 0, 0,
//                0, 0, 0,
//                0, 0, 0;
//        Kd_x << 0, 0, 0,
//                0, 0, 0,
//                0, 0, 0;
//        Kp_w << 0, 0, 0,
//                0, 0, 0,
//                0, 0, 0;
//        Kd_w << 0, 0, 0,
//                0, 0, 0,
//                0, 0, 0;

        //        alpha_act_com_x = 0.01, alpha_act_com_y = 0.01;
        //        weight_cp_x = 0.5, weight_cp_y = 0.9;
        //        weight_cp_x = 0.0, weight_cp_y = 1.2;
        w_cp_y1 = 1.0;
        w_cp_y2 = 1.2;
        cp_y_max = 0.06, cp_y_min = 0.02;

        BOC_Kp_roll = 0; //0.00010;
        BOC_Ki_roll = 0; //0.010; //0.003
        BOC_Kp_pitch = 0; //0.00010;
        BOC_Ki_pitch = 0; //0.010;

    }

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
        com_offset << -0.015, 0, 0;
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

    // =============== Flying trot parameters initialize =============== //

    ts = 0.20; //0.22; //0.25;
    tf = 0.08; //0.07;
    ft_step_time = ts + tf;

    ts_cnt = 200; //220;
    tf_cnt = 80;
    ft_step_cnt = ts_cnt + tf_cnt;

    h_0 = tar_init_com_pos(2);
    v_0 = 0;
    a_0 = 0;

    v_1 = 0.2; //0.10; //0.15;
    a_1 = -GRAVITY;

    h_2 = tar_init_com_pos(2);
    v_2 = -0.0; //-0.05; // -0.3
    a_2 = -GRAVITY;

    h_3 = tar_init_com_pos(2);
    v_3 = 0;
    a_3 = 0;

    h_1 = 0.5 * GRAVITY * tf * tf - v_1 * tf + h_2;

    //    // =============== Flying trot parameters initialize END =============== //
    moving_done_flag = true;
    walk_ready_moving_done_flag = false;

    for (unsigned int i = 0; i < 6; ++i) {
        pd_con_joint[i] = 0;
        pd_con_task[i] = 0;
    }

    pd_con_task[6] = 0;
    tmp_CTC_Torque = CTC_Torque;

    osqp_init();
    
    
    // ================ MPC INIT ================== //
    
    cout << "<< MPC INIT >>" << endl;
        
    PHI << 1, Ts, Ts*Ts/2,
           0,  1,   Ts,
           0,  0,    1;
   
    GAM << Ts*Ts*Ts/6, Ts*Ts/2, Ts;  
   
    C << 1, 0, -h_com/g;
    
    Q_tilda = Kron(MatrixNd::Identity(Np,Np),_Q);
    R_tilda = Kron(MatrixNd::Identity(Nc,Nc),_R);
 
    y_ini = C*x_ini;
    
    for(int i=0;i<Np;++i){
        tmp_Ax = pow_mat(PHI,i);
        
        Ax_tilda.block(i*state_num,0,state_num,state_num) = tmp_Ax;
    }
    
    for(int i=0;i<Np-1;++i){
        Bx_element = pow_mat(PHI,i)*GAM;
        
        MatrixNd tmp_Bx = MatrixNd::Zero((Np - i+1)*state_num, (Np - i+1)*input_num);
        tmp_Bx = Kron(MatrixNd::Identity(Nc-(i+1),Nc-(i+1)),Bx_element);

        tmp_Bx2 << MatrixNd::Zero(state_num*(i+1),input_num*Nc),
                    tmp_Bx, MatrixNd::Zero(state_num*(Np - (i+1)),input_num*(i+1));
        
        Bx_tilda = Bx_tilda + tmp_Bx2;    
    }

    C_tilda = Kron(MatrixNd::Identity(Np,Np),C);

    Q_bar = C_tilda.transpose()*Q_tilda*C_tilda;
    
    H = Bx_tilda.transpose()*Q_bar*Bx_tilda + R_tilda;
    
    H = (H + H.transpose())/2;
//    cout << "new H = " << endl << H << endl;
 
    // ================= Constraints ================= //
    const double output_constraint = 0.1;
    const double input_constraint = 30;

//    Gy_tmp = Kron(MatrixNd::Identity(Np,Np),MatrixNd::Identity(output_num,output_num));
    Gy_tmp = Kron(MatrixNd::Identity(Np,Np),MatrixNd::Zero(output_num,output_num));
    Gy <<  Gy_tmp, -Gy_tmp;
    gy << output_constraint*Kron(VectorNd::Ones(Np,1),VectorNd::Ones(output_num,1)), output_constraint*Kron(VectorNd::Ones(Np,1),VectorNd::Ones(output_num,1));
        
    Gy_new = Gy*C_tilda*Bx_tilda;
    
    gy_new = gy - Gy*C_tilda*Ax_tilda*state_x;
    
    Gu_tmp = Kron(MatrixNd::Identity(Nc,Nc),MatrixNd::Identity(input_num,input_num));
    Gu <<  Gu_tmp, -Gu_tmp;
    gu << input_constraint*Kron(VectorNd::Ones(Nc,1),VectorNd::Ones(input_num,1)), input_constraint*Kron(VectorNd::Ones(Nc,1),VectorNd::Ones(input_num,1));

    G << Gy_new , Gu;
    bk << gy_new , gu;
    
//    cout << "G = " << endl << G << endl;
//    cout << "bk = " << endl << bk << endl;
    
    // ================= Constraints END ================= //
    
    
    c_float H_x[55] = {H(0,0),
                       H(0,1),H(1,1),
                       H(0,2),H(1,2),H(2,2),
                       H(0,3),H(1,3),H(2,3),H(3,3),
                       H(0,4),H(1,4),H(2,4),H(3,4),H(4,4),
                       H(0,5),H(1,5),H(2,5),H(3,5),H(4,5),H(5,5),
                       H(0,6),H(1,6),H(2,6),H(3,6),H(4,6),H(5,6),H(6,6),
                       H(0,7),H(1,7),H(2,7),H(3,7),H(4,7),H(5,7),H(6,7),H(7,7),
                       H(0,8),H(1,8),H(2,8),H(3,8),H(4,8),H(5,8),H(6,8),H(7,8),H(8,8),
                       H(0,9),H(1,9),H(2,9),H(3,9),H(4,9),H(5,9),H(6,9),H(7,9),H(8,9),H(9,9)};
    
    c_int H_nnz = 55;
    c_int H_i[55] = {0,
                     0, 1,
                     0, 1, 2,
                     0, 1, 2, 3,
                     0, 1, 2, 3, 4,
                     0, 1, 2, 3, 4, 5,
                     0, 1, 2, 3, 4, 5, 6,
                     0, 1, 2, 3, 4, 5, 6, 7,
                     0, 1, 2, 3, 4, 5, 6, 7, 8,
                     0, 1, 2, 3, 4, 5, 6, 7, 8, 9};
    
    c_int H_p[11] = {0, 1, 3, 6, 10, 15, 21, 28, 36, 45, 55};
    
    c_float ff[10] = {f(0), f(1), f(2), f(3), f(4), f(5), f(6), f(7), f(8), f(9)};
    
    // ===================== Constraint ===================== //
//    c_float G_x[12] = {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1};
//    c_int G_nnz = 12;
//    c_int G_i[12] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11};
//    c_int G_p[13] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12};
//    c_float G_l[12]; // = {_d_l(0), _d_l(1), _d_l(2), _d_l(3), _d_l(4), _d_l(5), _d_l(6), _d_l(7), _d_l(8), _d_l(9), _d_l(10), _d_l(11)};
//    c_float G_u[12]; // = {_d_u(0), _d_u(1), _d_u(2), _d_u(3), _d_u(4), _d_u(5), _d_u(6), _d_u(7), _d_u(8), _d_u(9), _d_u(10), _d_u(11)};
//    c_int G_n = 12;
//    c_int G_m = 12;
    
    
}



MatrixNd CRobot::pow_mat(MatrixNd mat, int x){
    
    MatrixNd Out = MatrixNd::Zero(mat.rows(), mat.cols());
    
//    cout << "[1] Out = " << Out << endl;
    
    if(x == 0){
        Out = MatrixNd::Identity(mat.rows(), mat.cols());
    }
    else{
        for(int i = 0; i< x ; ++i){
            if(i == 0){
                Out = mat;
            }
            else{
                Out = Out*mat;
            }
        }
    }
    
//    cout << "[2] Out = " << Out << endl;
    
    return Out;
}

MatrixNd CRobot::Kron(MatrixNd AA, MatrixNd BB){

    MatrixNd Out = MatrixNd::Zero(AA.rows()*BB.rows(), AA.cols()*BB.cols());
//    cout << "AA = " << AA << endl;
//    cout << "BB = " << BB << endl;
//    cout << "Out(0,0) = " << Out(0,0) << endl;
    
    for(int i = 0; i<AA.rows(); i++){
        for(int j = 0; j<AA.cols(); j++){
            for(int k = 0; k<BB.rows(); k++){
                for(int l = 0; l<BB.cols(); l++){
                    Out(i*BB.rows()+k,j*BB.cols()+l) = AA(i,j) * BB(k,l);
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

// ============================================================================================ //

void CRobot::StateUpdate(void)
{
    RobotState(AXIS_X) = base_pos(0); //base.currentX;
    RobotState(AXIS_Y) = base_pos(1); //base.currentY;
    RobotState(AXIS_Z) = base_pos(2); //base.currentZ;
    RobotState(AXIS_Roll) = base_ori(0); //base.currentRoll;
    RobotState(AXIS_Pitch) = base_ori(1); //base.currentPitch;
    RobotState(AXIS_Yaw) = base_ori(2); //base.currentYaw;
    RobotStatedot(AXIS_X) = base_vel(0); //base.currentXvel;
    RobotStatedot(AXIS_Y) = base_vel(1); //base.currentYvel;
    RobotStatedot(AXIS_Z) = base_vel(2); //base.currentZvel;
    RobotStatedot(AXIS_Roll) = base_ori_dot(0); //base.currentRollvel;
    RobotStatedot(AXIS_Pitch) = base_ori_dot(1); //base.currentPitchvel;
    RobotStatedot(AXIS_Yaw) = base_ori_dot(2); //base.currentYawvel;

    for (int nJoint = 0; nJoint < nDOF; nJoint++) {
//    	tar_RobotState(6 + nJoint) = target_pos[nJoint];
        RobotState(6 + nJoint) = actual_pos[nJoint];
        RobotStatedot(6 + nJoint) = actual_vel[nJoint];
        RobotState2dot(6 + nJoint) = actual_acc[nJoint];
        //        RobotState(6 + nJoint) = target_pos[nJoint]; //actual_pos[nJoint];
        //        RobotStatedot(6 + nJoint) = target_vel[nJoint]; //actual_vel[nJoint];
        //        RobotState2dot(6 + nJoint) = target_acc[nJoint]; //actual_vel[nJoint];
    }

    base_ori_quat = Math::Quaternion::fromXYZAngles(base_ori);
    Math::Quaternion QQ(base_ori_quat);
    m_pModel->SetQuaternion(base.ID, QQ, RobotState);

    FK2();

    //    Get_act_com();
}

void CRobot::ComputeTorqueControl()
{
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

    J_RL2 << J_RL.block(0,6,3,3);
    J_RR2 << J_RR.block(0,9,3,3);
    J_FL2 << J_FL.block(0,13,3,3);
    J_FR2 << J_FR.block(0,16,3,3);
    
    act_RL_q_dot << actual_vel[0],actual_vel[1],actual_vel[2];
    act_RR_q_dot << actual_vel[3],actual_vel[4],actual_vel[5];
    act_FL_q_dot << actual_vel[7],actual_vel[8],actual_vel[9];
    act_FR_q_dot << actual_vel[10],actual_vel[11],actual_vel[12];
    
    act_RL_foot_vel = J_RL2*act_RL_q_dot;
    act_RR_foot_vel = J_RR2*act_RR_q_dot;
    act_FL_foot_vel = J_FL2*act_FL_q_dot;
    act_FR_foot_vel = J_FR2*act_FR_q_dot;
    
    actual_EP_vel << act_RL_foot_vel, act_RR_foot_vel, act_FL_foot_vel, act_FR_foot_vel;
//    act_RR_foot_vel = J_RR2*tar_RR_foot_vel_local;
//    act_FL_foot_vel = J_FL2*tar_FL_foot_vel_local;
//    act_FR_foot_vel = J_FR2*tar_FR_foot_vel_local;

//    cout << J_RL2 << endl;

    // ================= Cal Jacobian END ================= //

//    x_dot = J_A*RobotStatedot;
//    actual_EP_vel = x_dot.block(7, 0, 12, 1);

    tar_RL_foot_pos_local = (RL_foot_pos + RL_cp_foot_pos - base_pos) + RL_foot_pos_local_offset;
    tar_RR_foot_pos_local = (RR_foot_pos + RR_cp_foot_pos - base_pos) + RR_foot_pos_local_offset;
    tar_FL_foot_pos_local = (FL_foot_pos + FL_cp_foot_pos - base_pos) + FL_foot_pos_local_offset;
    tar_FR_foot_pos_local = (FR_foot_pos + FR_cp_foot_pos - base_pos) + FR_foot_pos_local_offset;
    
//    cout << "target_RL_foot_pos_z = " << tar_RL_foot_pos_local[2] << endl;
    
    base_vel = com_vel;
    
    tar_RL_foot_vel_local = RL_foot_vel + RL_cp_foot_vel - base_vel;
    tar_RR_foot_vel_local = RR_foot_vel + RR_cp_foot_vel - base_vel;
    tar_FL_foot_vel_local = FL_foot_vel + FL_cp_foot_vel - base_vel;
    tar_FR_foot_vel_local = FR_foot_vel + FR_cp_foot_vel - base_vel;
    
//    cout << "RL_foot_vel[2] = " << RL_foot_vel[2] << endl;
//    cout << "RL_cp_foot_vel[2] = " << RL_cp_foot_vel[2] << endl;
//    cout << "base_vel[2] = " << base_vel[2] << endl;
//    cout << "target_RL_foot_vel_z = " << tar_RL_foot_vel_local[2] << endl;
//    
//    cout << "==========================" << endl;

    target_EP << tar_RL_foot_pos_local, tar_RR_foot_pos_local, tar_FL_foot_pos_local, tar_FR_foot_pos_local;
    actual_EP << act_RL_foot_pos_local, act_RR_foot_pos_local, act_FL_foot_pos_local, act_FR_foot_pos_local;

    target_EP_vel << tar_RL_foot_vel_local, tar_RR_foot_vel_local, tar_FL_foot_vel_local, tar_FR_foot_vel_local;
    
    target_pos = IK1(target_EP);

    tar_RL_q_dot = J_RL2.inverse()*tar_RL_foot_vel_local;
    tar_RR_q_dot = J_RR2.inverse()*tar_RR_foot_vel_local;
    tar_FL_q_dot = J_FL2.inverse()*tar_FL_foot_vel_local;
    tar_FR_q_dot = J_FR2.inverse()*tar_FR_foot_vel_local;

    target_vel << tar_RL_q_dot, tar_RR_q_dot, 0 ,tar_FL_q_dot, tar_FR_q_dot;

    for (unsigned int i = 0; i < 12; ++i) {

        pd_con_task[i + 7] = Kp_t[i]*(target_EP[i] - actual_EP[i]) + Kd_t[i]*(target_EP_vel[i] - actual_EP_vel[i]); // + target_EP_offset[i];

        tmp_data1[i] = Fc(i + 7);
    }

    CompositeRigidBodyAlgorithm(*m_pModel, RobotState, M_term, true);
    NonlinearEffects(*m_pModel, RobotState, RobotStatedot, hatNonLinearEffects);
    NonlinearEffects(*m_pModel, RobotState, VectorNd::Zero(m_pModel->dof_count), G_term);
    C_term = hatNonLinearEffects - G_term;

    //	Fc << 0,0,0,0,0,0,0, 0,0,110, 0,0,110, 0,0,110, 0,0,110;

    CTC_Torque = fc_weight * (C_term + G_term - J_A.transpose() * (Fc - pd_con_task));
    for (int nJoint = 0; nJoint < nDOF; nJoint++) {
        joint[nJoint].torque = CTC_Torque(6 + nJoint);
    }

    static int tmp_cnt5 = 0;
    tmp_data2[0] = (double) tmp_cnt5*dt;

    tmp_cnt5++;
    
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
    tmp_data2[41] = tmp_com_vel(0);
    tmp_data2[42] = tmp_com_vel(1);
    tmp_data2[43] = tmp_com_vel(2);
    
    tmp_data2[44] = actual_EP_vel[0];
    tmp_data2[45] = actual_EP_vel[3];
    tmp_data2[46] = actual_EP_vel[6];
    tmp_data2[47] = actual_EP_vel[9];

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

    tmp_data2[13] = RL_foot_pos(0);//tar_RL_foot_pos_local(0); //RL_foot_pos(0);
    tmp_data2[14] = RL_foot_pos(1);//tar_RL_foot_pos_local(1); //RL_foot_pos(1);
    tmp_data2[15] = RL_foot_pos(2);//tar_RL_foot_pos_local(2); //RL_foot_pos(2);
    tmp_data2[16] = RR_foot_pos(0);//tar_RR_foot_pos_local(0); //act_RL_foot_pos(0);
    tmp_data2[17] = RR_foot_pos(1);//tar_RR_foot_pos_local(1); //act_RL_foot_pos(1);
    tmp_data2[18] = RR_foot_pos(2);//tar_RR_foot_pos_local(2); //act_RL_foot_pos(2);
    tmp_data2[19] = FL_foot_pos(0);//tar_RL_foot_pos_local(0); //RL_foot_pos(0);
    tmp_data2[20] = FL_foot_pos(1);//tar_RL_foot_pos_local(1); //RL_foot_pos(1);
    tmp_data2[21] = FL_foot_pos(2);//tar_RL_foot_pos_local(2); //RL_foot_pos(2);
    tmp_data2[22] = FR_foot_pos(0);//tar_RR_foot_pos_local(0); //act_RL_foot_pos(0);
    tmp_data2[23] = FR_foot_pos(1);//tar_RR_foot_pos_local(1); //act_RL_foot_pos(1);
    tmp_data2[24] = FR_foot_pos(2);//tar_RR_foot_pos_local(2); //act_RL_foot_pos(2);

    tmp_data2[25] = joint[0].torque;
    tmp_data2[26] = joint[1].torque;
    tmp_data2[27] = joint[2].torque;
    
    tmp_data2[28] = ft_phase*0.1;

    



//    tmp_data2[28] = IMURoll * R2D;
//    tmp_data2[29] = IMUPitch * R2D;

//    tmp_data1[12] = tar_FL_foot_pos_local(0);
//    tmp_data1[13] = tar_FL_foot_pos_local(1);
//    tmp_data1[14] = tar_FL_foot_pos_local(2);
//    tmp_data1[15] = tar_FR_foot_pos_local(0);
//    tmp_data1[16] = tar_FR_foot_pos_local(1);
//    tmp_data1[17] = tar_FR_foot_pos_local(2);

//    tmp_data1[18] = target_EP_vel[0];
//    tmp_data1[19] = target_EP_vel[1];
//    tmp_data1[20] = target_EP_vel[2];

    tmp_data1[21] = RL_foot_vel[0];//base_pos[0];
    tmp_data1[22] = RL_foot_vel[1];
    tmp_data1[23] = RL_foot_vel[2];
    tmp_data1[24] = RR_foot_vel[0];//base_pos[0];
    tmp_data1[25] = RR_foot_vel[1];
    tmp_data1[26] = RR_foot_vel[2];
    tmp_data1[27] = FL_foot_vel[0];//base_pos[0];
    tmp_data1[28] = FL_foot_vel[1];
    tmp_data1[29] = FL_foot_vel[2];
    tmp_data1[30] = FR_foot_vel[0];//base_pos[0];
    tmp_data1[31] = FR_foot_vel[1];
    tmp_data1[32] = FR_foot_vel[2];
    
    
    //    tmp_data1[22] = target_EP_vel[2];

    //    tmp_data1[12] = base_ori_dot(0);
    //    tmp_data1[13] = base_ori_dot(1);
    //    tmp_data1[14] = base_ori_dot(2);
    //    tmp_data1[15] = act_base_ori_dot(0);
    //    tmp_data1[16] = act_base_ori_dot(1);
    //    tmp_data1[17] = act_base_ori_dot(2);

    //    tmp_data1[18] = CP_x;
    //    tmp_data1[19] = CP_y;

    //    tmp_data2[22] = RR_foot_pos(0);
    //    tmp_data2[23] = RR_foot_pos(1);
    //    tmp_data2[24] = RR_foot_pos(2);
    //    tmp_data2[25] = act_RR_foot_pos(0);
    //    tmp_data2[26] = act_RR_foot_pos(1);
    //    tmp_data2[27] = act_RR_foot_pos(2);

    //        tmp_data2[6] = base_ori(0);
    //        tmp_data2[7] = base_ori(1);
    //        tmp_data2[8] = base_ori(2);
    //        tmp_data2[9] = act_base_ori(0);
    //        tmp_data2[10] = act_base_ori(1);
    //        tmp_data2[11] = act_base_ori(2);

    //    //x
    //    tmp_data2[12] = Fc(7);
    //    tmp_data2[13] = Fc(10);
    //    tmp_data2[14] = Fc(13);
    //    tmp_data2[15] = Fc(16);
    //    //y
    //    tmp_data2[16] = Fc(8);
    //    tmp_data2[17] = Fc(11);
    //    tmp_data2[18] = Fc(14);
    //    tmp_data2[19] = Fc(17);
    //    //z
    //    tmp_data2[22] = Fc(9);
    //    tmp_data2[23] = Fc(12);
    //    tmp_data2[24] = Fc(15);
    //    tmp_data2[25] = Fc(18);

    //    tmp_data2[25] = RL_foot_pos_local_offset(0);
    //    tmp_data2[26] = RL_foot_pos_local_offset(1);
    //    tmp_data2[27] = RR_foot_pos_local_offset(0);
    //    tmp_data2[28] = RR_foot_pos_local_offset(1);

    //    tmp_data2[6] = test1(8);
    //    tmp_data2[7] =  test2(8);
    //    tmp_data2[8] =  (double)test_cnt*dt;
    //
    //    tmp_data2[9] =  com_pos(2) - act_com_pos(2);
    //
    //    tmp_data2[10] =  com_ori(0);
    //    tmp_data2[11] =  act_com_ori(0);

    //    cout << "===========================" << endl;
}

void CRobot::osqp_init(void)
{
    // Selection matrix
    S_mat.block<6, 19>(0, 0) = MatrixNd::Zero(6, 19);
    S_mat.block<13, 6>(6, 0) = MatrixNd::Zero(13, 6);
    S_mat.block<13, 13>(6, 6) = MatrixNd::Identity(13, 13);

    des_x_2dot << 0, 0, 0;
    des_w_dot << 0, 0, 0;

    _m = 48.5; //kg
    //    _I_g << 1.7214, -0.0038, -0.1540,
    //            -0.0038, 4.9502, -0.0006,
    //            -0.1540, -0.0006, 5.1152;

    _I_g << 1.7214, 0, 0,
            0, 4.9502, 0,
            0, 0, 5.1152;

    _A.block<3, 3>(0, 0) = MatrixNd::Identity(3, 3);
    _A.block<3, 3>(0, 3) = MatrixNd::Identity(3, 3);
    _A.block<3, 3>(0, 6) = MatrixNd::Identity(3, 3);
    _A.block<3, 3>(0, 9) = MatrixNd::Identity(3, 3);

    tar_RL_foot_pos_local = tar_init_RL_foot_pos - init_base_pos;
    tar_RR_foot_pos_local = tar_init_RR_foot_pos - init_base_pos;
    tar_FL_foot_pos_local = tar_init_FL_foot_pos - init_base_pos;
    tar_FR_foot_pos_local = tar_init_FR_foot_pos - init_base_pos;

    p_com_oross_pro << 0, -tar_RL_foot_pos_local(2), tar_RL_foot_pos_local(1), 0, -tar_RR_foot_pos_local(2), tar_RR_foot_pos_local(1), 0, -tar_FL_foot_pos_local(2), tar_FL_foot_pos_local(1), 0, -tar_FR_foot_pos_local(2), tar_FR_foot_pos_local(1),
            tar_RL_foot_pos_local(2), 0, -tar_RL_foot_pos_local(0), tar_RR_foot_pos_local(2), 0, -tar_RR_foot_pos_local(0), tar_FL_foot_pos_local(2), 0, -tar_FL_foot_pos_local(0), tar_FR_foot_pos_local(2), 0, -tar_FR_foot_pos_local(0),
            -tar_RL_foot_pos_local(1), tar_RL_foot_pos_local(0), 0, -tar_RR_foot_pos_local(1), tar_RR_foot_pos_local(0), 0, -tar_FL_foot_pos_local(1), tar_FL_foot_pos_local(0), 0, -tar_FR_foot_pos_local(1), tar_FR_foot_pos_local(0), 0;

    _A.block<3, 12>(3, 0) = p_com_oross_pro;

    _g << 0, 0, 9.81;

    _b << _m * (des_x_2dot + _g),
            _I_g*des_w_dot;

    fx_max = 100;
    fx_min = -100;
    fy_max = 100;
    fy_min = -100;
    fz_max = 1000;
    fz_min = 0;

    _d_u << fx_max, fy_max, fz_max, fx_max, fy_max, fz_max, fx_max, fy_max, fz_max, fx_max, fy_max, fz_max;
    _d_l << fx_min, fy_min, fz_min, fx_min, fy_min, fz_min, fx_min, fy_min, fz_min, fx_min, fy_min, fz_min;

    _P = _A.transpose() * _S * _A + _alpha*_W;
    _q = -_A.transpose() * _S*_b;

    // ===================== OSQP  ====================== //

    c_float P_x[78] = {_P(0, 0),
        _P(0, 1), _P(1, 1),
        _P(0, 2), _P(1, 2), _P(2, 2),
        _P(0, 3), _P(1, 3), _P(2, 3), _P(3, 3),
        _P(0, 4), _P(1, 4), _P(2, 4), _P(3, 4), _P(4, 4),
        _P(0, 5), _P(1, 5), _P(2, 5), _P(3, 5), _P(4, 5), _P(5, 5),
        _P(0, 6), _P(1, 6), _P(2, 6), _P(3, 6), _P(4, 6), _P(5, 6), _P(6, 6),
        _P(0, 7), _P(1, 7), _P(2, 7), _P(3, 7), _P(4, 7), _P(5, 7), _P(6, 7), _P(7, 7),
        _P(0, 8), _P(1, 8), _P(2, 8), _P(3, 8), _P(4, 8), _P(5, 8), _P(6, 8), _P(7, 8), _P(8, 8),
        _P(0, 9), _P(1, 9), _P(2, 9), _P(3, 9), _P(4, 9), _P(5, 9), _P(6, 9), _P(7, 9), _P(8, 9), _P(9, 9),
        _P(0, 10), _P(1, 10), _P(2, 10), _P(3, 10), _P(4, 10), _P(5, 10), _P(6, 10), _P(7, 10), _P(8, 10), _P(9, 10), _P(10, 10),
        _P(0, 11), _P(1, 11), _P(2, 11), _P(3, 11), _P(4, 11), _P(5, 11), _P(6, 11), _P(7, 11), _P(8, 11), _P(9, 11), _P(10, 11), _P(11, 11)};

    c_float q[12] = {_q(0), _q(1), _q(2), _q(3), _q(4), _q(5), _q(6), _q(7), _q(8), _q(9), _q(10), _q(11)};

    c_float l[12] = {_c(0) * _d_l(0), _c(0) * _d_l(1), _c(0) * _d_l(2), _c(1) * _d_l(3), _c(1) * _d_l(4), _c(1) * _d_l(5), _c(2) * _d_l(6), _c(2) * _d_l(7), _c(2) * _d_l(8), _c(3) * _d_l(9), _c(3) * _d_l(10), _c(3) * _d_l(11)};
    c_float u[12] = {_c(0) * _d_u(0), _c(0) * _d_u(1), _c(0) * _d_u(2), _c(1) * _d_u(3), _c(1) * _d_u(4), _c(1) * _d_u(5), _c(2) * _d_u(6), _c(2) * _d_u(7), _c(2) * _d_u(8), _c(3) * _d_u(9), _c(3) * _d_u(10), _c(3) * _d_u(11)};

    //    // Workspace structures
    //    OSQPWorkspace *work;
    //    OSQPSettings *settings = (OSQPSettings *) c_malloc(sizeof (OSQPSettings));
    //    OSQPData *data = (OSQPData *) c_malloc(sizeof (OSQPData));

    // Populate data
    if (data) {
        data->n = n;
        data->m = m;
        data->P = csc_matrix(data->n, data->n, P_nnz, P_x, P_i, P_p);
        data->q = q;
        data->A = csc_matrix(data->m, data->n, A_nnz, A_x, A_i, A_p);
        data->l = l;
        data->u = u;
    }

    // Define solver settings as default
    if (settings) {
        osqp_set_default_settings(settings);
        settings->alpha = 0.1; // Change alpha parameter
    }

    // Setup workspace
    exitflag = osqp_setup(&work, data, settings);

    // Solve Problem
    osqp_solve(work);

    cout << "[RL] x = " << work->solution->x[0] << ", y = " << work->solution->x[1] << ", z = " << work->solution->x[2] << endl;
    cout << "[RR] x = " << work->solution->x[3] << ", y = " << work->solution->x[4] << ", z = " << work->solution->x[5] << endl;
    cout << "[FL] x = " << work->solution->x[6] << ", y = " << work->solution->x[7] << ", z = " << work->solution->x[8] << endl;
    cout << "[FR] x = " << work->solution->x[9] << ", y = " << work->solution->x[10] << ", z = " << work->solution->x[11] << endl;

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

VectorNd CRobot::Get_COM(VectorNd base, VectorNd q)
{
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

void CRobot::Get_Opt_F(void)
{
    p_com_oross_pro << 0, -tar_RL_foot_pos_local(2), tar_RL_foot_pos_local(1), 0, -tar_RR_foot_pos_local(2), tar_RR_foot_pos_local(1), 0, -tar_FL_foot_pos_local(2), tar_FL_foot_pos_local(1), 0, -tar_FR_foot_pos_local(2), tar_FR_foot_pos_local(1),
            tar_RL_foot_pos_local(2), 0, -tar_RL_foot_pos_local(0), tar_RR_foot_pos_local(2), 0, -tar_RR_foot_pos_local(0), tar_FL_foot_pos_local(2), 0, -tar_FL_foot_pos_local(0), tar_FR_foot_pos_local(2), 0, -tar_FR_foot_pos_local(0),
            -tar_RL_foot_pos_local(1), tar_RL_foot_pos_local(0), 0, -tar_RR_foot_pos_local(1), tar_RR_foot_pos_local(0), 0, -tar_FL_foot_pos_local(1), tar_FL_foot_pos_local(0), 0, -tar_FR_foot_pos_local(1), tar_FR_foot_pos_local(0), 0;

//        p_com_oross_pro << 0, -act_RL_foot_pos_local(2), act_RL_foot_pos_local(1), 0, -act_RR_foot_pos_local(2), act_RR_foot_pos_local(1), 0, -act_FL_foot_pos_local(2), act_FL_foot_pos_local(1), 0, -act_FR_foot_pos_local(2), act_FR_foot_pos_local(1),
//                act_RL_foot_pos_local(2), 0, -act_RL_foot_pos_local(0), act_RR_foot_pos_local(2), 0, -act_RR_foot_pos_local(0), act_FL_foot_pos_local(2), 0, -act_FL_foot_pos_local(0), act_FR_foot_pos_local(2), 0, -act_FR_foot_pos_local(0),
//                -act_RL_foot_pos_local(1), act_RL_foot_pos_local(0), 0, -act_RR_foot_pos_local(1), act_RR_foot_pos_local(0), 0, -act_FL_foot_pos_local(1), act_FL_foot_pos_local(0), 0, -act_FR_foot_pos_local(1), act_FR_foot_pos_local(0), 0;


    _A.block<3, 12>(3, 0) = p_com_oross_pro;

    Get_act_com();

    des_x_2dot = com_acc*2.0 + Kp_x * (com_pos - act_com_pos) + Kd_x * (com_vel - act_com_vel);
//    des_x_2dot = Kp_x * (com_pos - act_com_pos) + Kd_x * (com_vel - act_com_vel);
    des_w_dot = Kp_w * (base_ori - act_base_ori) + Kd_w * (base_ori_dot - act_base_ori_dot);

    //    des_x_2dot << 0,0,0;
    //    des_w_dot << 0,0,0;

    _b << _m * (des_x_2dot + _g),
            _I_g*des_w_dot;

    _P = _A.transpose() * _S * _A + _alpha*_W;
    _q = -_A.transpose() * _S*_b;

    // ===================== OSQP  ====================== //

    tmp_P_x << _P(0, 0),
            _P(0, 1), _P(1, 1),
            _P(0, 2), _P(1, 2), _P(2, 2),
            _P(0, 3), _P(1, 3), _P(2, 3), _P(3, 3),
            _P(0, 4), _P(1, 4), _P(2, 4), _P(3, 4), _P(4, 4),
            _P(0, 5), _P(1, 5), _P(2, 5), _P(3, 5), _P(4, 5), _P(5, 5),
            _P(0, 6), _P(1, 6), _P(2, 6), _P(3, 6), _P(4, 6), _P(5, 6), _P(6, 6),
            _P(0, 7), _P(1, 7), _P(2, 7), _P(3, 7), _P(4, 7), _P(5, 7), _P(6, 7), _P(7, 7),
            _P(0, 8), _P(1, 8), _P(2, 8), _P(3, 8), _P(4, 8), _P(5, 8), _P(6, 8), _P(7, 8), _P(8, 8),
            _P(0, 9), _P(1, 9), _P(2, 9), _P(3, 9), _P(4, 9), _P(5, 9), _P(6, 9), _P(7, 9), _P(8, 9), _P(9, 9),
            _P(0, 10), _P(1, 10), _P(2, 10), _P(3, 10), _P(4, 10), _P(5, 10), _P(6, 10), _P(7, 10), _P(8, 10), _P(9, 10), _P(10, 10),
            _P(0, 11), _P(1, 11), _P(2, 11), _P(3, 11), _P(4, 11), _P(5, 11), _P(6, 11), _P(7, 11), _P(8, 11), _P(9, 11), _P(10, 11), _P(11, 11);

    for (unsigned int i = 0; i < 78; ++i) {
        P_x[i] = tmp_P_x(i);
    }

    new_c << _c(0), _c(0), _c(0), _c(1), _c(1), _c(1), _c(2), _c(2), _c(2), _c(3), _c(3), _c(3);

    for (unsigned int i = 0; i < 12; ++i) {
        q[i] = _q(i);
        l[i] = new_c(i) * _d_l(i);
        u[i] = new_c(i) * _d_u(i);
    }

    // Update problem
    osqp_update_P(work, P_x, OSQP_NULL, 78);
    osqp_update_lin_cost(work, q);
    osqp_update_bounds(work, l, u);

    // Solve updated problem
    osqp_solve(work);

    for (unsigned int i = 0; i < 12; ++i) {
        Fc(i + 7) = fc_weight * work->solution->x[i];
        //        Fc(i + 7) = work->solution->x[i];
    }
}

void CRobot::WalkReady_Pos_Traj(void)
{
    
//    if(wr_cnt == 1){
//        pos_alpha = 1; 
//        vel_alpha = 1; 
//    }
//    else if(){
//        pos_alpha = 0.02; 
//        vel_alpha = 0.02; 
//    }
    
    if (wr_cnt == 0) {
        moving_done_flag = false;
        _c << 1, 1, 1, 1;
        contact_num = 4;

        // Global
        
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
        

        // waist
        init_pos[6] = actual_pos[6];
        target_pos[6] = actual_pos[6];

        init_IMUYaw = IMUYaw;

        if (Mode == MODE_SIMULATION) {
            fc_weight = 1;
        }
        else {
            fc_weight = 0;
        }

        
        // for actual pos & vel
        pos_alpha = 1; 
        vel_alpha = 1; 

        wr_cnt++;
    }

    else if (wr_cnt <= walk_ready_cnt) {
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
        
        // waist
        target_pos[6] = init_pos[6] + (0 - init_pos[6]) / 2.0 * (1 - cos(PI2 / (walk_ready_time * 2)*(double) (wr_cnt) * dt));

        if (Mode == MODE_SIMULATION) {
            fc_weight = 1;
        }
        else {
            fc_weight = 1 / 2.0 * (1 - cos(PI2 / (walk_ready_time * 2)*(double) (wr_cnt) * dt));
        }

        if(wr_cnt == 1){
            pos_alpha = 0.02; 
            vel_alpha = 0.02; 
        }
        

        wr_cnt++;
    }
    else if (wr_cnt <= walk_ready_cnt * 2) {
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
    }
    else {
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

    base_pos = com_pos + base_offset;

    if (Base_Ori_Con_onoff_flag == true) {
        Base_Ori_Con2();
    }
}

void CRobot::MPC_Init(void)
{
//    int state_num = 3;
//    int output_num = 1;
//    int input_num = 1;
//
//    double Ts = 0.15;
//    int Nc = 10;
//    int Np = 10 + 0;
//    
//    MatrixNd PHI = MatrixNd::Zero(3, 3);
//
//    
//    PHI << 1, Ts, Ts*Ts/2,
//           0,  1,   Ts,
//           0,  0,    1;
//    
//    cout << "PHI = " << PHI << endl;
}

void CRobot::Test_Function(void)
{
        if (test_cnt == 0) {
    
            // ============ Initialize ============ //
//            cout << "[0]" << endl;
            test_phase = 0;
            moving_done_flag = false;
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
            
        }
    
//        else if (test_cnt <= (unsigned int) (0.5 * 3 / dt)) {
//    
//            com_pos(2) = tar_init_com_pos(2) + (0.03) * (sin(PI2 / (0.5 * 1)*(double) (test_cnt) * dt));
//            com_vel(2) = (0.03) * (cos(PI2 / (0.5 * 1)*(double) (test_cnt) * dt));
//    
//            base_pos = com_pos + base_offset;
//            base_vel = com_vel;
//    
//            RL_foot_pos = tar_init_RL_foot_pos;
//            RR_foot_pos = tar_init_RR_foot_pos;
//            FL_foot_pos = tar_init_FL_foot_pos;
//            FR_foot_pos = tar_init_FR_foot_pos;
//    
//            test_cnt++;
//    
//            if (test_cnt == (unsigned int) (0.5 * 3 / dt)) {
//                cout << "!! Up & Down Done !!" << endl;
//                moving_done_flag = true;
//            }
//        }
    
        else {
//            cout << "[1]" << endl;
            test_phase = 1;
            moving_done_flag = false;
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
            
            state_x << act_com_pos(1), act_com_vel(1), act_com_acc(1);
            
            output_y = C*state_x;
            
            //========== reference generation ========== //
            ref_tilda << ref_tilda.block(1,0,Np-1,1), new_ref;  // ref : ZMP
            
            f = (state_x.transpose()*Ax_tilda.transpose()*Q_bar*Bx_tilda - ref_tilda.transpose()*Q_tilda*C_tilda*Bx_tilda).transpose();
            
            gy_new = gy - Gy*C_tilda*Ax_tilda*state_x;
            bk << gy_new , gu;
            
            
            
            
//            cout << "f = " << f.transpose() << endl;
            
            
        }
        
        base_pos = com_pos + base_offset;
        
        test_cnt++;
        
    
        //    // waist
        //    target_pos[6] = 0;
}

void CRobot::FTsensorTransformation()
{
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
    const double L3 = 0.309; //0.305;

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

void CRobot::FK2(void)
{
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

VectorNd CRobot::IK1(VectorNd EP)
{
    const double L1 = 0.105;
    const double L2 = 0.305;
    const double L3 = 0.309; //0.305;

    static double x = 0;
    static double y = 0;
    static double z = 0;

    x = -(EP[0] - RL_base2hip_pos(0));
    y = EP[1] - RL_base2hip_pos(1);
    z = EP[2] - RL_base2hip_pos(2);

    //    cout << "[RL1] x = " << -EP[0] << ", y = " << EP[1] << ", z = " << EP[2] << endl;
    //    cout << "[RL2] x = " << x << ", y = " << y << ", z = " << z << endl;

    target_pos[0] = atan(y / abs(z)) - PI / 2 + acos(L1 / (sqrt(pow(y, 2) + pow(z, 2))));
    target_pos[1] = -(-atan(x / sqrt(abs(-pow(L1, 2) + pow(y, 2) + pow(z, 2)))) - acos((-pow(L1, 2) + pow(L2, 2) - pow(L3, 2) + pow(x, 2) + pow(y, 2) + pow(z, 2)) / (2 * L2 * sqrt(-pow(L1, 2) + pow(x, 2) + pow(y, 2) + pow(z, 2)))));
    target_pos[2] = -(PI - acos((pow(L1, 2) + pow(L2, 2) + pow(L3, 2) - pow(x, 2) - pow(y, 2) - pow(z, 2)) / (2 * L2 * L3)));

    //    cout << "[RL] q[0] = " << target_pos[0] << ", q[1] = " << target_pos[0] << ", q[2] = " << target_pos[0] << endl;

    x = -(EP[3] - RR_base2hip_pos(0));
    y = EP[4] - RR_base2hip_pos(1);
    z = EP[5] - RR_base2hip_pos(2);

    target_pos[3] = PI / 2 + atan(y / abs(z)) - acos(L1 / sqrt(pow(y, 2) + pow(z, 2))); //PI/2 - acos(L1/sqrt(pow(y,2) + pow(z,2))) - atan(abs(y)/abs(z)); //-((atan(y/abs(z)) - PI/2 + acos(L1/(sqrt(pow(y,2) + pow(z,2))))));
    target_pos[4] = -(-atan(x / sqrt(abs(-pow(L1, 2) + pow(y, 2) + pow(z, 2)))) - acos((-pow(L1, 2) + pow(L2, 2) - pow(L3, 2) + pow(x, 2) + pow(y, 2) + pow(z, 2)) / (2 * L2 * sqrt(-pow(L1, 2) + pow(x, 2) + pow(y, 2) + pow(z, 2)))));
    target_pos[5] = -(PI - acos((pow(L1, 2) + pow(L2, 2) + pow(L3, 2) - pow(x, 2) - pow(y, 2) - pow(z, 2)) / (2 * L2 * L3)));

    x = -(EP[6] - FL_base2hip_pos(0));
    y = EP[7] - FL_base2hip_pos(1);
    z = EP[8] - FL_base2hip_pos(2);

    target_pos[7] = atan(y / abs(z)) - PI / 2 + acos(L1 / (sqrt(pow(y, 2) + pow(z, 2))));
    target_pos[8] = -(-atan(x / sqrt(abs(-pow(L1, 2) + pow(y, 2) + pow(z, 2)))) - acos((-pow(L1, 2) + pow(L2, 2) - pow(L3, 2) + pow(x, 2) + pow(y, 2) + pow(z, 2)) / (2 * L2 * sqrt(-pow(L1, 2) + pow(x, 2) + pow(y, 2) + pow(z, 2)))));
    target_pos[9] = -(PI - acos((pow(L1, 2) + pow(L2, 2) + pow(L3, 2) - pow(x, 2) - pow(y, 2) - pow(z, 2)) / (2 * L2 * L3)));

    x = -(EP[9] - FR_base2hip_pos(0));
    y = EP[10] - FR_base2hip_pos(1);
    z = EP[11] - FR_base2hip_pos(2);

    target_pos[10] = PI / 2 + atan(y / abs(z)) - acos(L1 / sqrt(pow(y, 2) + pow(z, 2))); //PI/2 - acos(L1/sqrt(pow(y,2) + pow(z,2))) - atan(abs(y)/abs(z)); //-((atan(y/abs(z)) - PI/2 + acos(L1/(sqrt(pow(y,2) + pow(z,2))))));
    target_pos[11] = -(-atan(x / sqrt(abs(-pow(L1, 2) + pow(y, 2) + pow(z, 2)))) - acos((-pow(L1, 2) + pow(L2, 2) - pow(L3, 2) + pow(x, 2) + pow(y, 2) + pow(z, 2)) / (2 * L2 * sqrt(-pow(L1, 2) + pow(x, 2) + pow(y, 2) + pow(z, 2)))));
    target_pos[12] = -(PI - acos((pow(L1, 2) + pow(L2, 2) + pow(L3, 2) - pow(x, 2) - pow(y, 2) - pow(z, 2)) / (2 * L2 * L3)));

    target_pos[6] = 0;
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

void CRobot::Get_CP(void)
{
    const double w_n = sqrt(com_height / GRAVITY)*1.0;
    const double cp_l = com_height;
    static double tmp_cp_y = 0;
    const double cp_alpha = 0.03;

    Rot_Mat_XYZ << 1, 0, sin(act_base_ori(1)),
            0, cos(act_base_ori(0)), -sin(act_base_ori(0)) * cos(act_base_ori(1)),
            0, sin(act_base_ori(0)), cos(act_base_ori(0)) * cos(act_base_ori(1));

    act_base_ori_dot_w = Rot_Mat_XYZ*act_base_ori_dot;
    act_com_vel2(0) = cp_l * act_base_ori_dot_w(1);
    act_com_vel2(1) = -cp_l * act_base_ori_dot_w(0);
    act_com_pos2(1) = -cp_l * act_base_ori(0);

    //    lpf_act_com_vel(0) = (1 - alpha_act_com_x) * lpf_act_com_vel(0) + alpha_act_com_x*act_com_vel(0);
    //    lpf_act_com_vel(1) = (1 - alpha_act_com_y) * lpf_act_com_vel(1) + alpha_act_com_y*act_com_vel(1);


    //    tmp_cp_y =  w_n*(act_com_vel2(1) - 0)*weight_cp_y;

    //    tmp_cp_y = ((act_com_pos(1) - com_pos(1)) + w_n * (act_com_vel(1) - 0)) * w_cp_y1;
    //    tmp_cp_y = ((act_com_pos2(1) - com_pos(1)) + w_n * (act_com_vel2(1) - com_vel(1))) * w_cp_y1;
    tmp_cp_y = (act_com_pos2(1) + w_n * act_com_vel2(1)) * w_cp_y1;

    cp_y = (1 - cp_alpha) * cp_y + cp_alpha*tmp_cp_y;
    tmp_data1[27] = cp_y;

    //    if(CP_con_onoff_flag == false) cp_y = 0;


    //        if(cp_y <= cp_y_min && cp_y >= -cp_y_min){
    //            tar_cp_y = 0;
    //        }
    //        else if(cp_y > cp_y_max){
    //            tar_cp_y = cp_y_max;
    //        }
    //        else if(cp_y < -cp_y_max){
    //            tar_cp_y = -cp_y_max;
    //        }
    //        else{
    //            tar_cp_y = cp_y;
    //        }
    //
    //        tar_cp_x = tar_cp_x/2;
    //        tar_cp_y = tar_cp_y/2;
    //
    //        cout << "cp_x = " << cp_x << ", tar_cp_x = " << tar_cp_x << endl;
    //        cout << "cp_y = " << cp_y << ", tar_cp_y = " << tar_cp_y << endl;
    //        cout << "================================================" << endl;

    //    cp_x =  w_n*(act_com_vel(0) - com_vel(0))*weight_cp_x;
    //    cp_y =  w_n*(act_com_vel(1) - com_vel(1))*weight_cp_y;

    //    cout << "err = " << act_base_ori_dot_w(0) - act_base_ori_dot(0) << endl;

    //    tmp_data1[28] = act_com_vel2(1);//act_base_ori_dot(0);
    //    tmp_data1[29] = com_vel(1);//act_base_ori_dot_w(0);
}

// ====================== Normal trot walking trajectory generation ===================== //

void CRobot::Trot_Walking4(void)
{
    static double tar_cp_x = 0, tar_cp_x2 = 0, tar_cp_y = 0, tar_cp_y2 = 0, cp_x1 = 0, cp_x2 = 0, cp_y1 = 0, cp_y2 = 0;
    //    static int walking_phase = 0;
    //    static int cp_phase = 0;
    //    static int cp_step_num = 0;

    //    Get_CP();
    if (CP_con_onoff_flag == true) Get_CP();


    tw_time = (double) tw_cnt*dt;
    base_ori(2) = tmp_base_ori(2);

    if (tw_cnt == 0) {
        // ============ Initialize ============ //
        walking_phase = 0;
        moving_done_flag = false;
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
    }
    else if (tw_cnt < dsp_cnt) {
        // ============ First Step (STANCE_RLFR) ============= //
        walking_phase = 1;
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

        if (tmp_t <= dsp_time / 2) {
            RR_foot_pos[2] = tar_init_RR_foot_pos[2] + c_sf_z1[5] * pow(tmp_t, 5) + c_sf_z1[4] * pow(tmp_t, 4) + c_sf_z1[3] * pow(tmp_t, 3) + c_sf_z1[2] * pow(tmp_t, 2) + c_sf_z1[1] * pow(tmp_t, 1) + c_sf_z1[0];
            FL_foot_pos[2] = tar_init_FL_foot_pos[2] + c_sf_z1[5] * pow(tmp_t, 5) + c_sf_z1[4] * pow(tmp_t, 4) + c_sf_z1[3] * pow(tmp_t, 3) + c_sf_z1[2] * pow(tmp_t, 2) + c_sf_z1[1] * pow(tmp_t, 1) + c_sf_z1[0];
            RR_foot_vel[2] = 5*c_sf_z1[5] * pow(tmp_t, 4) + 4*c_sf_z1[4] * pow(tmp_t, 3) + 3*c_sf_z1[3] * pow(tmp_t, 2) + 2*c_sf_z1[2] * pow(tmp_t, 1) + 1*c_sf_z1[1];
            FL_foot_vel[2] = 5*c_sf_z1[5] * pow(tmp_t, 4) + 4*c_sf_z1[4] * pow(tmp_t, 3) + 3*c_sf_z1[3] * pow(tmp_t, 2) + 2*c_sf_z1[2] * pow(tmp_t, 1) + 1*c_sf_z1[1];
        }
        else {
            tmp_t2 = tmp_t - dsp_time / 2.0;
            RR_foot_pos[2] = tar_init_RR_foot_pos[2] + c_sf_z2[5] * pow(tmp_t2, 5) + c_sf_z2[4] * pow(tmp_t2, 4) + c_sf_z2[3] * pow(tmp_t2, 3) + c_sf_z2[2] * pow(tmp_t2, 2) + c_sf_z2[1] * pow(tmp_t2, 1) + c_sf_z2[0];
            FL_foot_pos[2] = tar_init_FL_foot_pos[2] + c_sf_z2[5] * pow(tmp_t2, 5) + c_sf_z2[4] * pow(tmp_t2, 4) + c_sf_z2[3] * pow(tmp_t2, 3) + c_sf_z2[2] * pow(tmp_t2, 2) + c_sf_z2[1] * pow(tmp_t2, 1) + c_sf_z2[0];
            RR_foot_vel[2] = 5*c_sf_z2[5] * pow(tmp_t2, 4) + 4*c_sf_z2[4] * pow(tmp_t2, 3) + 3*c_sf_z2[3] * pow(tmp_t2, 2) + 2*c_sf_z2[2] * pow(tmp_t2, 1) + 1*c_sf_z2[1];
            FL_foot_vel[2] = 5*c_sf_z2[5] * pow(tmp_t2, 4) + 4*c_sf_z2[4] * pow(tmp_t2, 3) + 3*c_sf_z2[3] * pow(tmp_t2, 2) + 2*c_sf_z2[2] * pow(tmp_t2, 1) + 1*c_sf_z2[1];
        }
    }
    else if (tw_cnt < step_cnt) {
        // ============ First Step (FSP) ============= //
        walking_phase = 2;
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
            TW_SF_Traj_Gen();

            tar_cp_y = 0;
            tar_cp_y2 = 0;
        }
    }
    else if (tw_cnt < step_cnt + dsp_cnt) {
        // ============ Second Step (STANCE_RRFL) ============ //
        // ============ Continuous Walking Start ============= //
        walking_phase = 3;
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

        // =============== CP =============== //
        if (cp_phase == 0) {
            tar_cp_y = 0;
            cp_y1 = -tar_cp_y2 + (tar_cp_y - (-tar_cp_y2)) / 2.0 * (1 - cos(PI2 / (2 * (dsp_time)) * tmp_t));
            cp_y2 = tar_cp_y2 + (-tar_cp_y - tar_cp_y2) / 2.0 * (1 - cos(PI2 / (2 * (dsp_time)) * tmp_t));

            if (cp_y >= cp_y_min || cp_y <= -cp_y_min) {
                cp_phase = 1;
                tmp_t3 = tmp_t;

                tar_cp_y2 = -cp_y1;

                cout << "============== dsp cp con on !! ==============" << endl;
            }
        }
        else if (cp_phase == 1) { // DSP
            //            cout << "[1] DSP" << endl;

            if (cp_y > cp_y_max) {
                tar_cp_y = cp_y_max;
            }
            else if (cp_y < -cp_y_max) {
                tar_cp_y = -cp_y_max;
            }
            else {
                tar_cp_y = cp_y / 1.0;
            }
            //            tar_cp_y = cp_y/1.0;
            tmp_t2 = tmp_t - tmp_t3;
            cp_y1 = -tar_cp_y2 + (tar_cp_y - (-tar_cp_y2)) / 2.0 * (1 - cos(PI2 / (2 * (dsp_time - tmp_t3)) * tmp_t2));
            cp_y2 = tar_cp_y2 + (-tar_cp_y - tar_cp_y2) / 2.0 * (1 - cos(PI2 / (2 * (dsp_time - tmp_t3)) * tmp_t2));

        }

        // =============== CP END =============== //
        //        RL_cp_foot_pos(0) = cp_x1;
        //        RR_cp_foot_pos(0) = cp_x2;
        //        FL_cp_foot_pos(0) = cp_x2;
        //        FR_cp_foot_pos(0) = cp_x1;

        RL_cp_foot_pos(1) = cp_y1;
        RR_cp_foot_pos(1) = cp_y2;
        FL_cp_foot_pos(1) = cp_y2;
        FR_cp_foot_pos(1) = cp_y1;

        com_pos[0] = c_com_x1[5] * pow(tmp_t, 5) + c_com_x1[4] * pow(tmp_t, 4) + c_com_x1[3] * pow(tmp_t, 3) + c_com_x1[2] * pow(tmp_t, 2) + c_com_x1[1] * pow(tmp_t, 1) + c_com_x1[0];
        com_pos[1] = c_com_y1[5] * pow(tmp_t, 5) + c_com_y1[4] * pow(tmp_t, 4) + c_com_y1[3] * pow(tmp_t, 3) + c_com_y1[2] * pow(tmp_t, 2) + c_com_y1[1] * pow(tmp_t, 1) + c_com_y1[0];
        com_vel[0] = 5 * c_com_x1[5] * pow(tmp_t, 4) + 4 * c_com_x1[4] * pow(tmp_t, 3) + 3 * c_com_x1[3] * pow(tmp_t, 2) + 2 * c_com_x1[2] * pow(tmp_t, 1) + c_com_x1[1];
        com_vel[1] = 5 * c_com_y1[5] * pow(tmp_t, 4) + 4 * c_com_y1[4] * pow(tmp_t, 3) + 3 * c_com_y1[3] * pow(tmp_t, 2) + 2 * c_com_y1[2] * pow(tmp_t, 1) + c_com_y1[1];
        
        RL_foot_pos[0] = pre_RL_foot_pos[0] + c_sf_x1[5] * pow(tmp_t, 5) + c_sf_x1[4] * pow(tmp_t, 4) + c_sf_x1[3] * pow(tmp_t, 3) + c_sf_x1[2] * pow(tmp_t, 2) + c_sf_x1[1] * pow(tmp_t, 1) + c_sf_x1[0];
        RL_foot_pos[1] = pre_RL_foot_pos[1] + c_sf_y1[5] * pow(tmp_t, 5) + c_sf_y1[4] * pow(tmp_t, 4) + c_sf_y1[3] * pow(tmp_t, 3) + c_sf_y1[2] * pow(tmp_t, 2) + c_sf_y1[1] * pow(tmp_t, 1) + c_sf_y1[0];
        FR_foot_pos[0] = pre_FR_foot_pos[0] + c_sf_x1[5] * pow(tmp_t, 5) + c_sf_x1[4] * pow(tmp_t, 4) + c_sf_x1[3] * pow(tmp_t, 3) + c_sf_x1[2] * pow(tmp_t, 2) + c_sf_x1[1] * pow(tmp_t, 1) + c_sf_x1[0];
        FR_foot_pos[1] = pre_FR_foot_pos[1] + c_sf_y1[5] * pow(tmp_t, 5) + c_sf_y1[4] * pow(tmp_t, 4) + c_sf_y1[3] * pow(tmp_t, 3) + c_sf_y1[2] * pow(tmp_t, 2) + c_sf_y1[1] * pow(tmp_t, 1) + c_sf_y1[0];
        
        RL_foot_vel[0] = 5*c_sf_x1[5] * pow(tmp_t, 4) + 4*c_sf_x1[4] * pow(tmp_t, 3) + 3*c_sf_x1[3] * pow(tmp_t, 2) + 2*c_sf_x1[2] * pow(tmp_t, 1) + 1*c_sf_x1[1];
        RL_foot_vel[1] = 5*c_sf_y1[5] * pow(tmp_t, 4) + 4*c_sf_y1[4] * pow(tmp_t, 3) + 3*c_sf_y1[3] * pow(tmp_t, 2) + 2*c_sf_y1[2] * pow(tmp_t, 1) + 1*c_sf_y1[1];
        FR_foot_vel[0] = 5*c_sf_x1[5] * pow(tmp_t, 4) + 4*c_sf_x1[4] * pow(tmp_t, 3) + 3*c_sf_x1[3] * pow(tmp_t, 2) + 2*c_sf_x1[2] * pow(tmp_t, 1) + 1*c_sf_x1[1];
        FR_foot_vel[1] = 5*c_sf_y1[5] * pow(tmp_t, 4) + 4*c_sf_y1[4] * pow(tmp_t, 3) + 3*c_sf_y1[3] * pow(tmp_t, 2) + 2*c_sf_y1[2] * pow(tmp_t, 1) + 1*c_sf_y1[1];
        
        if (tmp_cnt <= dsp_cnt / 2) {
            RL_foot_pos[2] = tar_init_RL_foot_pos[2] + c_sf_z1[5] * pow(tmp_t, 5) + c_sf_z1[4] * pow(tmp_t, 4) + c_sf_z1[3] * pow(tmp_t, 3) + c_sf_z1[2] * pow(tmp_t, 2) + c_sf_z1[1] * pow(tmp_t, 1) + c_sf_z1[0];
            FR_foot_pos[2] = tar_init_FR_foot_pos[2] + c_sf_z1[5] * pow(tmp_t, 5) + c_sf_z1[4] * pow(tmp_t, 4) + c_sf_z1[3] * pow(tmp_t, 3) + c_sf_z1[2] * pow(tmp_t, 2) + c_sf_z1[1] * pow(tmp_t, 1) + c_sf_z1[0];
            RL_foot_vel[2] = 5*c_sf_z1[5] * pow(tmp_t, 4) + 4*c_sf_z1[4] * pow(tmp_t, 3) + 3*c_sf_z1[3] * pow(tmp_t, 2) + 2*c_sf_z1[2] * pow(tmp_t, 1) + 1*c_sf_z1[1];
            FR_foot_vel[2] = 5*c_sf_z1[5] * pow(tmp_t, 4) + 4*c_sf_z1[4] * pow(tmp_t, 3) + 3*c_sf_z1[3] * pow(tmp_t, 2) + 2*c_sf_z1[2] * pow(tmp_t, 1) + 1*c_sf_z1[1];
        }
        else {
            tmp_t2 = tmp_t - dsp_time / 2.0;
            RL_foot_pos[2] = tar_init_RL_foot_pos[2] + c_sf_z2[5] * pow(tmp_t2, 5) + c_sf_z2[4] * pow(tmp_t2, 4) + c_sf_z2[3] * pow(tmp_t2, 3) + c_sf_z2[2] * pow(tmp_t2, 2) + c_sf_z2[1] * pow(tmp_t2, 1) + c_sf_z2[0];
            FR_foot_pos[2] = tar_init_FR_foot_pos[2] + c_sf_z2[5] * pow(tmp_t2, 5) + c_sf_z2[4] * pow(tmp_t2, 4) + c_sf_z2[3] * pow(tmp_t2, 3) + c_sf_z2[2] * pow(tmp_t2, 2) + c_sf_z2[1] * pow(tmp_t2, 1) + c_sf_z2[0];
            RL_foot_vel[2] = 5*c_sf_z2[5] * pow(tmp_t2, 4) + 4*c_sf_z2[4] * pow(tmp_t2, 3) + 3*c_sf_z2[3] * pow(tmp_t2, 2) + 2*c_sf_z2[2] * pow(tmp_t2, 1) + 1*c_sf_z2[1];
            FR_foot_vel[2] = 5*c_sf_z2[5] * pow(tmp_t2, 4) + 4*c_sf_z2[4] * pow(tmp_t2, 3) + 3*c_sf_z2[3] * pow(tmp_t2, 2) + 2*c_sf_z2[2] * pow(tmp_t2, 1) + 1*c_sf_z2[1];
            
            if (tmp_cnt == dsp_cnt - 1) {
                pre_com_pos(0) = pre_com_pos[0] + pre_x_moving_speed * (dsp_time) / 2.0 + x_moving_speed * (dsp_time) / 2.0;
                pre_com_pos(1) = pre_com_pos[1] + pre_y_moving_speed * (dsp_time) / 2.0 + y_moving_speed * (dsp_time) / 2.0;
                pre_com_pos(2) = com_pos(2);

                pre_RL_foot_pos = RL_foot_pos;
                pre_RR_foot_pos = RR_foot_pos;
                pre_FL_foot_pos = FL_foot_pos;
                pre_FR_foot_pos = FR_foot_pos;

            }
        }
    }
    else if (tw_cnt < step_cnt * 2) {
        // ============ Second Step (FSP) ============= //
        walking_phase = 4;
        //        cout << "[walk4]" << endl;
        _c << 1, 1, 1, 1;
        contact_num = 4;

        tmp_t = tw_time - step_time - dsp_time;

        //        RL_cp_foot_pos(0) = tar_cp_x;
        //        RR_cp_foot_pos(0) = -tar_cp_x;
        //        FL_cp_foot_pos(0) = -tar_cp_x;
        //        FR_cp_foot_pos(0) = tar_cp_x;

        RL_cp_foot_pos(1) = tar_cp_y;
        RR_cp_foot_pos(1) = -tar_cp_y;
        FL_cp_foot_pos(1) = -tar_cp_y;
        FR_cp_foot_pos(1) = tar_cp_y;

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

            cp_phase = 0;
        }

        //        // ============ CP (first) ============= //
        //        if(tmp_t > 0.01 && (cp_y >= cp_y_min || cp_y <= -cp_y_min)){
        //            cp_phase = 1;
        //            tmp_t3 = 0;
        //            
        //            pre_com_pos(0) = pre_com_pos(0) + x_moving_speed*(tmp_t+dt);
        //            pre_com_pos(1) = pre_com_pos(1) + y_moving_speed*(tmp_t+dt);
        //            pre_com_pos(2) = com_pos(2);
        //            pre_RL_foot_pos = RL_foot_pos;
        //            pre_RR_foot_pos = RR_foot_pos;
        //            pre_FL_foot_pos = FL_foot_pos;
        //            pre_FR_foot_pos = FR_foot_pos;
        //            
        //            tw_cnt = step_cnt * 2 - 1;
        //            
        ////            tar_cp_y2 = cp_y/1.0;
        //            cout << "tmp_t = " << tmp_t << endl;
        //            cout << "<< cp control on (1)" << ", init_cp_y = " << tar_cp_y << ", cp_y = " << cp_y << endl;
        //            cout << "====================" << endl;
        //        }
    }
    else if (tw_cnt < step_cnt * 2 + dsp_cnt) {
        // ============ Third Step (STANCE_RLFR) ============= //
        walking_phase = 5;
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

        if (cp_phase == 0) {
            tar_cp_y2 = 0;
            cp_y1 = tar_cp_y + (-tar_cp_y2 - (tar_cp_y)) / 2.0 * (1 - cos(PI2 / (2 * (dsp_time)) * tmp_t));
            cp_y2 = -tar_cp_y + (tar_cp_y2 - (-tar_cp_y)) / 2.0 * (1 - cos(PI2 / (2 * (dsp_time)) * tmp_t));

            if (cp_y >= cp_y_min || cp_y <= -cp_y_min) {
                cp_phase = 1;
                tmp_t3 = tmp_t;

                tar_cp_y = cp_y1;

                cout << "============== dsp cp con on !! ==============" << endl;
            }
        }
        else if (cp_phase == 1) { // DSP
            //            cout << "[1] DSP" << endl;
            if (cp_y > cp_y_max) {
                tar_cp_y2 = cp_y_max;
            }
            else if (cp_y < -cp_y_max) {
                tar_cp_y2 = -cp_y_max;
            }
            else {
                tar_cp_y2 = cp_y / 1.0;
            }
            //            tar_cp_y = cp_y/1.0;
            //            tar_cp_y2 = cp_y/1.0;
            tmp_t2 = tmp_t - tmp_t3;
            cp_y1 = tar_cp_y + (-tar_cp_y2 - (tar_cp_y)) / 2.0 * (1 - cos(PI2 / (2 * (dsp_time - tmp_t3)) * tmp_t2));
            cp_y2 = -tar_cp_y + (tar_cp_y2 - (-tar_cp_y)) / 2.0 * (1 - cos(PI2 / (2 * (dsp_time - tmp_t3)) * tmp_t2));
        }

        RL_cp_foot_pos(1) = cp_y1;
        RR_cp_foot_pos(1) = cp_y2;
        FL_cp_foot_pos(1) = cp_y2;
        FR_cp_foot_pos(1) = cp_y1;

        com_pos[0] = pre_com_pos(0) + x_moving_speed*tmp_t;
        com_pos[1] = pre_com_pos(1) + y_moving_speed*tmp_t;
        com_vel[0] = x_moving_speed;
        com_vel[1] = y_moving_speed;
        
        RR_foot_pos[0] = pre_RR_foot_pos[0] + c_sf_x2[5] * pow(tmp_t, 5) + c_sf_x2[4] * pow(tmp_t, 4) + c_sf_x2[3] * pow(tmp_t, 3) + c_sf_x2[2] * pow(tmp_t, 2) + c_sf_x2[1] * pow(tmp_t, 1) + c_sf_x2[0];
        RR_foot_pos[1] = pre_RR_foot_pos[1] + c_sf_y2[5] * pow(tmp_t, 5) + c_sf_y2[4] * pow(tmp_t, 4) + c_sf_y2[3] * pow(tmp_t, 3) + c_sf_y2[2] * pow(tmp_t, 2) + c_sf_y2[1] * pow(tmp_t, 1) + c_sf_y2[0];
        FL_foot_pos[0] = pre_FL_foot_pos[0] + c_sf_x2[5] * pow(tmp_t, 5) + c_sf_x2[4] * pow(tmp_t, 4) + c_sf_x2[3] * pow(tmp_t, 3) + c_sf_x2[2] * pow(tmp_t, 2) + c_sf_x2[1] * pow(tmp_t, 1) + c_sf_x2[0];
        FL_foot_pos[1] = pre_FL_foot_pos[1] + c_sf_y2[5] * pow(tmp_t, 5) + c_sf_y2[4] * pow(tmp_t, 4) + c_sf_y2[3] * pow(tmp_t, 3) + c_sf_y2[2] * pow(tmp_t, 2) + c_sf_y2[1] * pow(tmp_t, 1) + c_sf_y2[0];

        RR_foot_vel[0] = 5*c_sf_x2[5] * pow(tmp_t, 4) + 4*c_sf_x2[4] * pow(tmp_t, 3) + 3*c_sf_x2[3] * pow(tmp_t, 2) + 2*c_sf_x2[2] * pow(tmp_t, 1) + 1*c_sf_x2[1];
        RR_foot_vel[1] = 5*c_sf_y2[5] * pow(tmp_t, 4) + 4*c_sf_y2[4] * pow(tmp_t, 3) + 3*c_sf_y2[3] * pow(tmp_t, 2) + 2*c_sf_y2[2] * pow(tmp_t, 1) + 1*c_sf_y2[1];
        FL_foot_vel[0] = 5*c_sf_x2[5] * pow(tmp_t, 4) + 4*c_sf_x2[4] * pow(tmp_t, 3) + 3*c_sf_x2[3] * pow(tmp_t, 2) + 2*c_sf_x2[2] * pow(tmp_t, 1) + 1*c_sf_x2[1];
        FL_foot_vel[1] = 5*c_sf_y2[5] * pow(tmp_t, 4) + 4*c_sf_y2[4] * pow(tmp_t, 3) + 3*c_sf_y2[3] * pow(tmp_t, 2) + 2*c_sf_y2[2] * pow(tmp_t, 1) + 1*c_sf_y2[1];
        
        if (tmp_cnt <= dsp_cnt / 2) {
            RR_foot_pos[2] = tar_init_RR_foot_pos[2] + c_sf_z1[5] * pow(tmp_t, 5) + c_sf_z1[4] * pow(tmp_t, 4) + c_sf_z1[3] * pow(tmp_t, 3) + c_sf_z1[2] * pow(tmp_t, 2) + c_sf_z1[1] * pow(tmp_t, 1) + c_sf_z1[0];
            FL_foot_pos[2] = tar_init_FL_foot_pos[2] + c_sf_z1[5] * pow(tmp_t, 5) + c_sf_z1[4] * pow(tmp_t, 4) + c_sf_z1[3] * pow(tmp_t, 3) + c_sf_z1[2] * pow(tmp_t, 2) + c_sf_z1[1] * pow(tmp_t, 1) + c_sf_z1[0];
            RR_foot_vel[2] = 5*c_sf_z1[5] * pow(tmp_t, 4) + 4*c_sf_z1[4] * pow(tmp_t, 3) + 3*c_sf_z1[3] * pow(tmp_t, 2) + 2*c_sf_z1[2] * pow(tmp_t, 1) + 1*c_sf_z1[1];
            FL_foot_vel[2] = 5*c_sf_z1[5] * pow(tmp_t, 4) + 4*c_sf_z1[4] * pow(tmp_t, 3) + 3*c_sf_z1[3] * pow(tmp_t, 2) + 2*c_sf_z1[2] * pow(tmp_t, 1) + 1*c_sf_z1[1];
        }
        else {
            tmp_t2 = tmp_t - dsp_time / 2.0;
            RR_foot_pos[2] = tar_init_RR_foot_pos[2] + c_sf_z2[5] * pow(tmp_t2, 5) + c_sf_z2[4] * pow(tmp_t2, 4) + c_sf_z2[3] * pow(tmp_t2, 3) + c_sf_z2[2] * pow(tmp_t2, 2) + c_sf_z2[1] * pow(tmp_t2, 1) + c_sf_z2[0];
            FL_foot_pos[2] = tar_init_FL_foot_pos[2] + c_sf_z2[5] * pow(tmp_t2, 5) + c_sf_z2[4] * pow(tmp_t2, 4) + c_sf_z2[3] * pow(tmp_t2, 3) + c_sf_z2[2] * pow(tmp_t2, 2) + c_sf_z2[1] * pow(tmp_t2, 1) + c_sf_z2[0];
            RR_foot_vel[2] = 5*c_sf_z2[5] * pow(tmp_t2, 4) + 4*c_sf_z2[4] * pow(tmp_t2, 3) + 3*c_sf_z2[3] * pow(tmp_t2, 2) + 2*c_sf_z2[2] * pow(tmp_t2, 1) + 1*c_sf_z2[1];
            FL_foot_vel[2] = 5*c_sf_z2[5] * pow(tmp_t2, 4) + 4*c_sf_z2[4] * pow(tmp_t2, 3) + 3*c_sf_z2[3] * pow(tmp_t2, 2) + 2*c_sf_z2[2] * pow(tmp_t2, 1) + 1*c_sf_z2[1];
            
            if (tmp_cnt == dsp_cnt - 1) {
                pre_com_pos(0) = pre_com_pos(0) + x_moving_speed*dsp_time;
                pre_com_pos(1) = pre_com_pos(1) + y_moving_speed*dsp_time;
                pre_com_pos(2) = com_pos(2);
                pre_RL_foot_pos = RL_foot_pos;
                pre_RR_foot_pos = RR_foot_pos;
                pre_FL_foot_pos = FL_foot_pos;
                pre_FR_foot_pos = FR_foot_pos;
            }
        }
    }
    else if (tw_cnt < step_cnt * 3) {
        // ============ Third Step (FSP) ============= //
        walking_phase = 6;
        _c << 1, 1, 1, 1;
        contact_num = 4;

        tmp_t = tw_time - step_time * 2 - dsp_time;

        RL_cp_foot_pos(1) = -tar_cp_y2;
        RR_cp_foot_pos(1) = tar_cp_y2;
        FL_cp_foot_pos(1) = tar_cp_y2;
        FR_cp_foot_pos(1) = -tar_cp_y2;

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
            TW_SF_Traj_Gen();

            tar_cp_y = 0;
            cp_phase = 0;

            if (sub_ctrl_flag == false) {
                tw_cnt = step_cnt - 1;
            }
        }

        //        // ============ CP (second) ============= //
        //        if(tmp_t > 0.01 && (cp_y >= cp_y_min || cp_y <= -cp_y_min)){
        //        
        //            cp_phase = 1;
        //            tmp_t3 = 0;
        //            
        //            pre_com_pos(0) = pre_com_pos(0) + x_moving_speed*(tmp_t+dt);
        //            pre_com_pos(1) = pre_com_pos(1) + y_moving_speed*(tmp_t+dt);
        //            pre_com_pos(2) = com_pos(2);
        //            pre_RL_foot_pos = RL_foot_pos;
        //            pre_RR_foot_pos = RR_foot_pos;
        //            pre_FL_foot_pos = FL_foot_pos;
        //            pre_FR_foot_pos = FR_foot_pos;
        //            
        //            pre_x_moving_speed = x_moving_speed;
        //            pre_y_moving_speed = y_moving_speed;
        //            x_moving_speed = tmp_x_moving_speed;
        //            y_moving_speed = tmp_y_moving_speed;
        //
        //            TW_COM_Traj_Gen();
        //            TW_SF_Traj_Gen();
        //            
        //            cout << "tmp_t = " << tmp_t << endl;
        //            cout << "<< cp control on (2)" << ", init_cp_y = " << tar_cp_y2 << ", target_cp_y = " << tar_cp_y << endl;
        //
        //            cout << "====================" << endl;
        //            
        //            if (sub_ctrl_flag == false) {
        //                tw_cnt = step_cnt - 1;
        //            }
        //        }
        // ============ Continuous Walking End ============== //
    }
    else {
    	_c << 1, 1, 1, 1;
		contact_num = 4;

        com_vel = tar_init_com_vel;
                
        RL_foot_vel = tar_init_RL_foot_vel;
        RR_foot_vel = tar_init_RR_foot_vel;
        FL_foot_vel = tar_init_FL_foot_vel;
        FR_foot_vel = tar_init_FR_foot_vel;
        
        if (tw_cnt == step_cnt * 3) {
            moving_done_flag = true;
            _c << 1, 1, 1, 1;
            contact_num = 4;
        }
    }


    RL_cp_foot_pos(1) = w_cp_y2 * RL_cp_foot_pos(1);
    RR_cp_foot_pos(1) = w_cp_y2 * RR_cp_foot_pos(1);
    FL_cp_foot_pos(1) = w_cp_y2 * FL_cp_foot_pos(1);
    FR_cp_foot_pos(1) = w_cp_y2 * FR_cp_foot_pos(1);


    //    tmp_data1[18] = RL_cp_foot_pos(0);
    //    tmp_data1[19] = RR_cp_foot_pos(0);
    //    tmp_data1[20] = RL_cp_foot_pos(0);
    //    tmp_data1[21] = RR_cp_foot_pos(0);
    //    tmp_data1[22] = cp_x;

//    tmp_data1[23] = RL_cp_foot_pos(1);
//    tmp_data1[24] = RR_cp_foot_pos(1);
//    tmp_data1[25] = FL_cp_foot_pos(1);
//    tmp_data1[26] = FR_cp_foot_pos(1);
    //    tmp_data1[27] = cp_y;
    tmp_data1[49] = walking_phase * 0.1;


    base_pos = com_pos + base_offset;

    TW_Turning_Traj_Gen();

    if (Base_Ori_Con_onoff_flag == true) {
        Base_Ori_Con2();
    }
    target_pos[6] = 0; //goal_pos[6];

    tw_cnt++;
}

void CRobot::TW_COM_Traj_Gen(void)
{
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

void CRobot::TW_SF_Traj_Gen(void)
{
    // ============ Swing Foot Pos. ============ //
    //    static double tmp_cp_x = 0, tmp_cp_y = 0;
    //    static double tmp_cp_x2 = 0, tmp_cp_y2 = 0;
    //    double w_n = sqrt(com_height/GRAVITY)*1.0;
    //    const double alpha_act_com_x = 0.5;
    //    
    //    lpf_act_com_vel(0) = (1 - alpha_act_com_x) * lpf_act_com_vel(0) + alpha_act_com_x*act_com_vel(0);
    //    
    //    tmp_cp_x2 =  w_n*(lpf_act_com_vel(0) - com_vel(0))*2;
    //    tmp_cp_y =  0;//w_n*(act_com_vel(1) - com_vel(1));
    //    
    //    if(tmp_cp_x2 < 0.01 && tmp_cp_x2 > -0.01){
    //        tmp_cp_x = 0;
    //    }
    //    else if(tmp_cp_x2 > 0.04){
    //        tmp_cp_x = 0.04;
    //    }
    //    else if(tmp_cp_x2 < -0.04){
    //        tmp_cp_x = -0.04;
    //    }
    //    else{
    //        tmp_cp_x = tmp_cp_x2;
    //        cout << "tmp_cp_x = " << tmp_cp_x << endl;
    //    }
    // ================= X axis ================== //
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

    // ================= Y axis ================== //

    // Left (First)
    init_x[0] = 0;
    init_x[1] = 0;
    init_x[2] = 0;

    final_x[0] = 0 + pre_y_moving_speed * step_time + y_moving_speed * step_time;
    final_x[1] = 0;
    final_x[2] = 0;

    coefficient_5thPoly(init_x, final_x, dsp_time, c_sf_y1);

    // Right (First)
    init_x[0] = 0;
    init_x[1] = 0;
    init_x[2] = 0;

    final_x[0] = 0 + y_moving_speed * step_time * 2;
    final_x[1] = 0;
    final_x[2] = 0;

    coefficient_5thPoly(init_x, final_x, dsp_time, c_sf_y2);

    // Left (Second)
    init_x[0] = 0 + pre_y_moving_speed * step_time + y_moving_speed * step_time;
    init_x[1] = 0;
    init_x[2] = 0;

    final_x[0] = 0 + pre_y_moving_speed * step_time + y_moving_speed * step_time * 3;
    final_x[1] = 0;
    final_x[2] = 0;

    coefficient_5thPoly(init_x, final_x, dsp_time, c_sf_y3);

    // Right (Second)
    init_x[0] = 0 + y_moving_speed * step_time * 2;
    init_x[1] = 0;
    init_x[2] = 0;

    final_x[0] = 0 + y_moving_speed * step_time * 4;
    final_x[1] = 0;
    final_x[2] = 0;

    coefficient_5thPoly(init_x, final_x, dsp_time, c_sf_y4);
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
    if ((base_ori(2) > 0.01) && (tw_cnt == 2 * step_cnt)) {
        if (turn_mode == 0) {
            turn_start_flag = true;
            turn_mode = 1;

            target_theta = -base_ori(2) * 2.0; // [rad]

            printf("[left] target_theta = %f\n", target_theta);
        }
    }
    else if ((base_ori(2) < -0.01) && (tw_cnt == step_cnt)) {
        if (turn_mode == 0) {
            turn_start_flag = true;
            turn_mode = 3;

            target_theta = base_ori(2) * 2.0; // [rad]

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

            if (turn_cnt == step_cnt - 1) {
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
        else if (turn_mode == 3) {
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
            //            if (FC_PHASE == STANCE_RLFR) {
            if (turn_cnt == step_cnt - 1) {
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

    RL_foot_pos_local_offset << -turn_xr_EP, -turn_yr_EP, 0;
    RR_foot_pos_local_offset << -turn_xl_EP, -turn_yl_EP, 0;
    FL_foot_pos_local_offset << +turn_xl_EP, +turn_yl_EP, 0;
    FR_foot_pos_local_offset << +turn_xr_EP, +turn_yr_EP, 0;

    // ======= Turning trajectory generation  END ======= //
}

void CRobot::TW_COM_SF_X_Traj_Gen(void)
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

void CRobot::TW_SF_Z_Traj_Gen(void)
{
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

void CRobot::Base_Ori_Con2(void)
{
    static double sum_roll_err = 0., sum_pitch_err = 0.;
    static double del_L_left = 0, del_L_right = 0, del_L_front = 0, del_L_rear = 0; //, del_L_rl = 0, del_L_rr = 0;
    const double limit_foot_z = 0.04;
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
        sum_pitch_err = sum_pitch_err + (0 - lpf_IMUPitch) * dt;
    }

    del_L_front = BOC_Kp_pitch * (0 - lpf_IMUPitch) + BOC_Ki_pitch*sum_pitch_err;
    del_L_rear = -BOC_Kp_pitch * (0 - lpf_IMUPitch) - BOC_Ki_pitch*sum_pitch_err;

    //    cout << "del_L_front = " << del_L_front << ", del_L_rear = " << del_L_rear << endl;

    //    printf("lpf_IMUPitch = %f, del_L_front = %f\n",lpf_IMUPitch,del_L_front);

    RL_foot_pos_local_offset(2) = -del_L_left + del_L_rear;
    RR_foot_pos_local_offset(2) = -del_L_right + del_L_rear;
    FL_foot_pos_local_offset(2) = -del_L_left + del_L_front;
    FR_foot_pos_local_offset(2) = -del_L_right + del_L_front;

    if (RL_foot_pos_local_offset(2) > limit_foot_z) {
        RL_foot_pos_local_offset(2) = limit_foot_z;
    }
    else if (RL_foot_pos_local_offset(2) < -limit_foot_z) {
        RL_foot_pos_local_offset(2) = -limit_foot_z;
    }

    if (RR_foot_pos_local_offset(2) > limit_foot_z) {
        RR_foot_pos_local_offset(2) = limit_foot_z;
    }
    else if (RR_foot_pos_local_offset(2) < -limit_foot_z) {
        RR_foot_pos_local_offset(2) = -limit_foot_z;
    }

    if (FL_foot_pos_local_offset(2) > limit_foot_z) {
        FL_foot_pos_local_offset(2) = limit_foot_z;
    }
    else if (FL_foot_pos_local_offset(2) < -limit_foot_z) {
        FL_foot_pos_local_offset(2) = -limit_foot_z;
    }

    if (FR_foot_pos_local_offset(2) > limit_foot_z) {
        FR_foot_pos_local_offset(2) = limit_foot_z;
    }
    else if (FR_foot_pos_local_offset(2) < -limit_foot_z) {
        FR_foot_pos_local_offset(2) = -limit_foot_z;
    }

    //    cout << "RL = " << RL_foot_pos_local_offset(2) << ", RR = " << RR_foot_pos_local_offset(2) << ",FL = " << FL_foot_pos_local_offset(2) << ",FR = " << FR_foot_pos_local_offset(2) << endl;

    //        printf("RL_offset(2) = %f,RR_offset(2) = %f\n",RL_foot_pos_local_offset(2),RR_foot_pos_local_offset(2));
}

//void CRobot::SF_X_Traj_Gen_Final(void)
//{
//    init_x[0] = pre_foot_l_2d(0, 0);
//    init_x[1] = 0;
//    init_x[2] = 0;
//
//    final_x[0] = pre_foot_l_2d(1, 0);
//    final_x[1] = 0;
//    final_x[2] = 0;
//
//    coefficient_5thPoly(init_x, final_x, dsp_time, x5);
//
//}

//void CRobot::SF_X_Traj_Gen(void)
//{
//    init_x[0] = pre_foot_l_2d(0, 0);
//    init_x[1] = 0;
//    init_x[2] = 0;
//
//    final_x[0] = pre_foot_l_2d(1, 0);
//    final_x[1] = 0;
//    final_x[2] = 0;
//
//    coefficient_5thPoly(init_x, final_x, dsp_time, x1);
//
//    init_x[0] = pre_foot_r_2d(1, 0);
//    init_x[1] = 0;
//    init_x[2] = 0;
//
//    final_x[0] = pre_foot_r_2d(2, 0);
//    final_x[1] = 0;
//    final_x[2] = 0;
//
//    coefficient_5thPoly(init_x, final_x, dsp_time, x2);
//
//    init_x[0] = pre_foot_l_2d(2, 0);
//    init_x[1] = 0;
//    init_x[2] = 0;
//
//    final_x[0] = pre_foot_l_2d(3, 0);
//    final_x[1] = 0;
//    final_x[2] = 0;
//
//    coefficient_5thPoly(init_x, final_x, dsp_time, x3);
//
//    init_x[0] = pre_foot_r_2d(3, 0);
//    init_x[1] = 0;
//    init_x[2] = 0;
//
//    final_x[0] = pre_foot_r_2d(4, 0);
//    final_x[1] = 0;
//    final_x[2] = 0;
//
//    coefficient_5thPoly(init_x, final_x, dsp_time, x4);
//}
//
//void CRobot::SF_Z_Traj_Gen(void)
//{
//    dsp_t1 = dsp_time / 2.0f;
//    dsp_t2 = dsp_time - dsp_t1;
//
//    init_x[0] = 0;
//    init_x[1] = 0;
//    init_x[2] = 0;
//
//    final_x[0] = foot_height;
//    final_x[1] = 0;
//    final_x[2] = 0;
//
//    coefficient_5thPoly(init_x, final_x, dsp_t1, z1);
//
//    init_x[0] = foot_height;
//    init_x[1] = 0;
//    init_x[2] = 0;
//
//    final_x[0] = 0;
//    final_x[1] = 0;
//    final_x[2] = 0;
//
//    coefficient_5thPoly(init_x, final_x, dsp_t2, z2);
//}

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

void CRobot::Torque_off(void)
{
    for (int i = 0; i < nDOF; ++i) {
        joint[i].torque = 0;
        target_vel[i] = 0;
        target_acc[i] = 0;
    }
}

void CRobot::Get_act_com(void)
{
    // ============== Get COM Position & Orientation ============ //

    //x
    //            act_com_pos[0] = com_pos[0] + com_height * IMUPitch * PI / 180;
    //            act_com_vel[0] = com_height * IMUPitch_dot * PI / 180;

    //    VectorNd act_base_pos2 = VectorNd::Zero(3);
    //    
    //    global_foot_center(0) = (_c(0) * RL_foot_pos[0] + _c(1) * RR_foot_pos[0] + _c(2) * FL_foot_pos[0] + _c(3) * FR_foot_pos[0]) / contact_num;
    //    global_foot_center(1) = (_c(0) * RL_foot_pos[1] + _c(1) * RR_foot_pos[1] + _c(2) * FL_foot_pos[1] + _c(3) * FR_foot_pos[1]) / contact_num;
    //
    //    act_base_pos[0] = global_foot_center(0) -(_c(0) * act_RL_foot_pos_local[0] + _c(1) * act_RR_foot_pos_local[0] + _c(2) * act_FL_foot_pos_local[0] + _c(3) * act_FR_foot_pos_local[0]) / contact_num;
    //    act_base_vel[0] = -(_c(0) * actual_EP_vel[0] + _c(1) * actual_EP_vel[3] + _c(2) * actual_EP_vel[6] + _c(3) * actual_EP_vel[9]) / contact_num;
    //
    //    // y
    //    //            act_com_pos[1] = com_pos[1] - com_height * IMURoll * PI / 180 * 1.0;
    //    //            act_com_vel[1] = -com_height * IMURoll_dot * PI / 180 * 1.0;
    //    act_base_pos[1] = global_foot_center(1) -(_c(0) * act_RL_foot_pos_local[1] + _c(1) * act_RR_foot_pos_local[1] + _c(2) * act_FL_foot_pos_local[1] + _c(3) * act_FR_foot_pos_local[1]) / contact_num;
    //    act_base_vel[1] = -(_c(0) * actual_EP_vel[1] + _c(1) * actual_EP_vel[4] + _c(2) * actual_EP_vel[7] + _c(3) * actual_EP_vel[10]) / contact_num;
    //
    //    //z
    //    act_base_pos[2] = -(_c(0) * act_RL_foot_pos_local[2] + _c(1) * act_RR_foot_pos_local[2] + _c(2) * act_FL_foot_pos_local[2] + _c(3) * act_FR_foot_pos_local[2]) / contact_num;
    //    act_base_vel[2] = -(_c(0) * actual_EP_vel[2] + _c(1) * actual_EP_vel[5] + _c(2) * actual_EP_vel[8] + _c(3) * actual_EP_vel[11]) / contact_num;


//    static double pos_alpha = 0.02; 
//    static double vel_alpha = 0.02; 
    
    
    
    if(contact_num != 0){
        tmp_act_base_pos(0) = (_c(0) * (RL_foot_pos[0] - act_RL_foot_pos_local[0]) + _c(1) * (RR_foot_pos[0] - act_RR_foot_pos_local[0]) + _c(2) * (FL_foot_pos[0] - act_FL_foot_pos_local[0]) + _c(3) * (FR_foot_pos[0] - act_FR_foot_pos_local[0])) / contact_num;
        tmp_act_base_pos(1) = (_c(0) * (RL_foot_pos[1] - act_RL_foot_pos_local[1]) + _c(1) * (RR_foot_pos[1] - act_RR_foot_pos_local[1]) + _c(2) * (FL_foot_pos[1] - act_FL_foot_pos_local[1]) + _c(3) * (FR_foot_pos[1] - act_FR_foot_pos_local[1])) / contact_num;
        tmp_act_base_pos(2) = (_c(0) * (RL_foot_pos[2] - act_RL_foot_pos_local[2]) + _c(1) * (RR_foot_pos[2] - act_RR_foot_pos_local[2]) + _c(2) * (FL_foot_pos[2] - act_FL_foot_pos_local[2]) + _c(3) * (FR_foot_pos[2] - act_FR_foot_pos_local[2])) / contact_num;

        tmp_act_base_vel(0) = -(_c(0) * actual_EP_vel[0] + _c(1) * actual_EP_vel[3] + _c(2) * actual_EP_vel[6] + _c(3) * actual_EP_vel[9]) / contact_num;
        tmp_act_base_vel(1) = -(_c(0) * actual_EP_vel[1] + _c(1) * actual_EP_vel[4] + _c(2) * actual_EP_vel[7] + _c(3) * actual_EP_vel[10]) / contact_num;
        tmp_act_base_vel(2) = -(_c(0) * actual_EP_vel[2] + _c(1) * actual_EP_vel[5] + _c(2) * actual_EP_vel[8] + _c(3) * actual_EP_vel[11]) / contact_num;
        
    }
    else{
        tmp_act_base_pos(0) = ((RL_foot_pos[0] - act_RL_foot_pos_local[0]) + (RR_foot_pos[0] - act_RR_foot_pos_local[0]) + (FL_foot_pos[0] - act_FL_foot_pos_local[0]) + (FR_foot_pos[0] - act_FR_foot_pos_local[0])) / 4;
        tmp_act_base_pos(1) = ((RL_foot_pos[1] - act_RL_foot_pos_local[1]) + (RR_foot_pos[1] - act_RR_foot_pos_local[1]) + (FL_foot_pos[1] - act_FL_foot_pos_local[1]) + (FR_foot_pos[1] - act_FR_foot_pos_local[1])) / 4;
        tmp_act_base_pos(2) = ((RL_foot_pos[2] - act_RL_foot_pos_local[2]) + (RR_foot_pos[2] - act_RR_foot_pos_local[2]) + (FL_foot_pos[2] - act_FL_foot_pos_local[2]) + (FR_foot_pos[2] - act_FR_foot_pos_local[2])) / 4;

        tmp_act_base_vel(0) = -(actual_EP_vel[0] + actual_EP_vel[3] + actual_EP_vel[6] + actual_EP_vel[9]) / 4;
        tmp_act_base_vel(1) = -(actual_EP_vel[1] + actual_EP_vel[4] + actual_EP_vel[7] + actual_EP_vel[10]) / 4;
        tmp_act_base_vel(2) = -(actual_EP_vel[2] + actual_EP_vel[5] + actual_EP_vel[8] + actual_EP_vel[11]) / 4;

    }
    
    act_base_pos = (1 - vel_alpha)*act_base_pos + vel_alpha*tmp_act_base_pos;
    act_base_vel = (1 - vel_alpha)*act_base_vel + vel_alpha*tmp_act_base_vel;
    

    //    cout << "act_base_pos(0) = " << act_base_pos(0) << ", act_base_pos(1) = " << act_base_pos(1) << ", act_base_pos(2) = " << act_base_pos(2) << endl;
    //    cout << "act_base_pos2(0) = " << act_base_pos2(0) << ", act_base_pos2(1) = " << act_base_pos2(1) << ", act_base_pos2(2) = " << act_base_pos2(2) << endl;
    //    cout << "act_RL_foot_pos_local[2] = " << act_RL_foot_pos_local[2] << endl;
    //    cout << "===============================================================" << endl;



    act_com_pos = act_base_pos - base_offset;
    act_com_vel = act_base_vel; //(0.90*lpf_base_alpha)*pre_act_com_vel + ((1 - 0.90)*lpf_base_alpha)*act_base_vel;

    act_com_acc = (act_com_vel - pre_act_com_vel)/dt;
    pre_act_com_vel = act_com_vel;

    //            cout << "IMURoll = " << IMURoll*R2D << endl;
    act_base_ori << IMURoll, IMUPitch, IMUYaw - init_IMUYaw;
    act_base_ori_dot << IMURoll_dot, IMUPitch_dot, IMUYaw_dot;
    //    tmp_act_base_ori_dot << IMURoll_dot, IMUPitch_dot, IMUYaw_dot;
    //
    //    act_base_ori_dot = (0.90*lpf_base_alpha)*pre_act_base_ori_dot + ((1 - 0.90)*lpf_base_alpha)*tmp_act_base_ori_dot;
    //
    //    pre_act_base_ori_dot = act_base_ori_dot;
    if (act_base_ori(0) > 20 * D2R) {
        act_base_ori(0) = 20 * D2R;
    }
    else if (act_base_ori(0) < -20 * D2R) {
        act_base_ori(0) = -20 * D2R;
    }

    if (act_base_ori(1) > 20 * D2R) {
        act_base_ori(1) = 20 * D2R;
    }
    else if (act_base_ori(1) < -20 * D2R) {
        act_base_ori(1) = -20 * D2R;
    }

    //    if (act_base_ori(2) > 20 * D2R) {
    //        act_base_ori(2) = 20 * D2R;
    //    }
    //    else if (act_base_ori(2) < -20 * D2R) {
    //        act_base_ori(2) = -20 * D2R;
    //    }
}





















// ====================== flying trot trajectory generation ===================== //
void CRobot::Flying_Trot_Running3(void)
{
    static double com_t = 0;
    static double sf_rl_t = 0, sf_rl_t2 = 0;
    static double sf_rr_t = 0, sf_rr_t2 = 0;
    static double tar_rl_foot_pos = 0, tar_rr_foot_pos = 0;
    static bool init_flag = false;
    
    ft_time = (double) ft_cnt*dt;
    base_ori(2) = tmp_base_ori(2);

//    if (CP_con_onoff_flag == true) Get_CP();

    if (ft_cnt == 0) {
        //        cout << "=========== ft_phase = 0 ===========" << endl;
        ft_phase = 0;
        moving_done_flag = false;
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
    }
    else if (ft_cnt < ts_cnt) {
        //        cout << "=========== ft_phase = 1 ===========" << endl;
        ft_phase = 1;
        _c << 1, 0, 0, 1;
        contact_num = 2;
        com_t = ft_time;
        sf_rr_t = sf_rr_t + dt;

        com_pos[2] = fifth_order_poly(c_com_z1,com_t);
        com_vel[2] = fifth_order_poly_dot(c_com_z1,com_t);
        com_acc[2] = fifth_order_poly_2dot(c_com_z1,com_t);

        RL_foot_pos[2] = tar_init_RL_foot_pos[2];
        FR_foot_pos[2] = tar_init_FR_foot_pos[2];
        RL_foot_vel[2] = tar_init_RL_foot_vel[2];
                
        if(sf_rr_t < (ts+tf)/2.0){
            RR_foot_pos[2] = tar_init_RR_foot_pos[2] + fifth_order_poly(c_sf_z1,sf_rr_t);
            FL_foot_pos[2] = tar_init_FL_foot_pos[2] + fifth_order_poly(c_sf_z1,sf_rr_t);
            RR_foot_vel[2] = fifth_order_poly_dot(c_sf_z1,sf_rr_t);
 
        }
        else{
            sf_rr_t2 = sf_rr_t - (ts+tf)/2.0;
            RR_foot_pos[2] = tar_init_RR_foot_pos[2] + fifth_order_poly(c_sf_z2,sf_rr_t2);
            FL_foot_pos[2] = tar_init_FL_foot_pos[2] + fifth_order_poly(c_sf_z2,sf_rr_t2);
            RR_foot_vel[2] = fifth_order_poly_dot(c_sf_z2,sf_rr_t2);
        }
        
        FL_foot_vel = RR_foot_vel;
        FR_foot_vel = RL_foot_vel;
        
        if(ft_cnt == ts_cnt - 1){
            pre_x_moving_speed = x_moving_speed;
            x_moving_speed = tmp_x_moving_speed;
//            tar_rl_foot_pos = pre_x_moving_speed*(ts + tf) + x_moving_speed*(ts + tf);
            tar_rl_foot_pos = pre_com_pos[0] - 0.35 + pre_x_moving_speed*tf + (pre_x_moving_speed + x_moving_speed)*ts/2.0 + x_moving_speed*tf + x_moving_speed*ts/2.0;
            
            init_flag = true;
            tar_rr_foot_pos = pre_RR_foot_pos[0];
            sf_rl_t = 0;
        }
    }
    else if (ft_cnt < ft_step_cnt) {
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
        com_pos[2] = fifth_order_poly(c_com_z2,com_t);

        com_vel[0] = pre_x_moving_speed;
        com_vel[2] = fifth_order_poly_dot(c_com_z2,com_t);
        
        com_acc[2] = fifth_order_poly_2dot(c_com_z2,com_t);

        RL_foot_pos[0] = pre_RL_foot_pos[0] + (tar_rl_foot_pos - pre_RL_foot_pos[0])/2.0*(1-cos(PI2/(2*(ft_step_time + tf))*sf_rl_t));
        FR_foot_pos[0] = pre_FR_foot_pos[0] + (tar_rl_foot_pos - pre_RL_foot_pos[0])/2.0*(1-cos(PI2/(2*(ft_step_time + tf))*sf_rl_t));
        RL_foot_vel[0] = (tar_rl_foot_pos - pre_RL_foot_pos[0])/2.0*PI2/(2*(ft_step_time + tf))*(sin(PI2/(2*(ft_step_time + tf))*sf_rl_t));
        
        RL_foot_pos[2] = tar_init_RL_foot_pos[2] + fifth_order_poly(c_sf_z3,sf_rl_t);
        FR_foot_pos[2] = tar_init_FR_foot_pos[2] + fifth_order_poly(c_sf_z3,sf_rl_t);
        RL_foot_vel[2] = fifth_order_poly_dot(c_sf_z3,sf_rl_t);
        
        if(init_flag != true){
            RR_foot_pos[0] = pre_RR_foot_pos[0] + (tar_rr_foot_pos - pre_RR_foot_pos[0])/2.0*(1-cos(PI2/(2*(ft_step_time + tf))*sf_rr_t));
            FL_foot_pos[0] = pre_FL_foot_pos[0] + (tar_rr_foot_pos - pre_RR_foot_pos[0])/2.0*(1-cos(PI2/(2*(ft_step_time + tf))*sf_rr_t));
            RR_foot_vel[0] = (tar_rr_foot_pos - pre_RR_foot_pos[0])/2.0*PI2/(2*(ft_step_time + tf))*(sin(PI2/(2*(ft_step_time + tf))*sf_rr_t));

            sf_rr_t2 = sf_rr_t -(ft_step_time + tf)/2.0;
            RR_foot_pos[2] = tar_init_RR_foot_pos[2] + fifth_order_poly(c_sf_z4,sf_rr_t2);
            FL_foot_pos[2] = tar_init_FL_foot_pos[2] + fifth_order_poly(c_sf_z4,sf_rr_t2);
            RR_foot_vel[2] = fifth_order_poly_dot(c_sf_z4,sf_rr_t2);
        
        }
        else{
            RR_foot_pos[0] = tar_init_RR_foot_pos[0];
            FL_foot_pos[0] = tar_init_FL_foot_pos[0];
            RR_foot_vel[0] = tar_init_RR_foot_vel[0];

            sf_rr_t2 = sf_rr_t - (ts+tf)/2.0;
            RR_foot_pos[2] = tar_init_RR_foot_pos[2] + fifth_order_poly(c_sf_z2,sf_rr_t2);
            FL_foot_pos[2] = tar_init_FL_foot_pos[2] + fifth_order_poly(c_sf_z2,sf_rr_t2);
            RR_foot_vel[2] = fifth_order_poly_dot(c_sf_z2,sf_rr_t2);
            
            if(ft_cnt == ft_step_cnt - 1){
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

            pre_FL_foot_pos[0] = pre_FL_foot_pos[0] + (tar_rr_foot_pos - pre_RR_foot_pos[0]);// + tar_rr_foot_pos;
            pre_RR_foot_pos[0] = pre_RR_foot_pos[0] + (tar_rr_foot_pos - pre_RR_foot_pos[0]);//pre_RR_foot_pos[0] + tar_rr_foot_pos;
                    
            FT_COM_X_Traj_Gen();
        }
    }
    else if (ft_cnt < ft_step_cnt + ts_cnt) {
        //        cout << "=========== ft_phase = 3 ===========" << endl;
        ft_phase = 3;
        _c << 0, 1, 1, 0;
        contact_num = 2;
        com_t = ft_time - ft_step_time;
        sf_rl_t = sf_rl_t + dt;

        com_pos[0] = fifth_order_poly(c_com_x1,com_t);
        com_pos[1] = pre_com_pos(1);
        com_pos[2] = fifth_order_poly(c_com_z3,com_t);

        com_vel[0] = fifth_order_poly_dot(c_com_x1,com_t);
        com_vel[1] = pre_com_vel[1];
        com_vel[2] = fifth_order_poly_dot(c_com_z3,com_t);
        
        com_acc[2] = fifth_order_poly_2dot(c_com_z3,com_t);
        
        RL_foot_pos[0] = pre_RL_foot_pos[0] + (tar_rl_foot_pos - pre_RL_foot_pos[0])/2.0*(1-cos(PI2/(2*(ft_step_time + tf))*sf_rl_t));
        FR_foot_pos[0] = pre_FR_foot_pos[0] + (tar_rl_foot_pos - pre_RL_foot_pos[0])/2.0*(1-cos(PI2/(2*(ft_step_time + tf))*sf_rl_t));
        RL_foot_vel[0] = (tar_rl_foot_pos - pre_RL_foot_pos[0])/2.0*PI2/(2*(ft_step_time + tf))*(sin(PI2/(2*(ft_step_time + tf))*sf_rl_t));
        
        RR_foot_pos[0] = pre_RR_foot_pos[0];
        FL_foot_pos[0] = pre_FL_foot_pos[0];
        RR_foot_vel[0] = tar_init_RR_foot_vel[0];
        
        RR_foot_pos[2] = tar_init_RR_foot_pos[2];
        FL_foot_pos[2] = tar_init_FL_foot_pos[2];
        RR_foot_vel[2] = tar_init_RR_foot_vel[2];
        
        if(sf_rl_t < (ft_step_time + tf)/2.0){
            RL_foot_pos[2] = tar_init_RL_foot_pos[2] + fifth_order_poly(c_sf_z3,sf_rl_t);
            FR_foot_pos[2] = tar_init_FR_foot_pos[2] + fifth_order_poly(c_sf_z3,sf_rl_t);
            RL_foot_vel[2] = fifth_order_poly_dot(c_sf_z3,sf_rl_t);
 
        }
        else{
            sf_rl_t2 = sf_rl_t - (ft_step_time + tf)/2.0;
            RL_foot_pos[2] = tar_init_RL_foot_pos[2] + fifth_order_poly(c_sf_z4,sf_rl_t2);
            FR_foot_pos[2] = tar_init_FR_foot_pos[2] + fifth_order_poly(c_sf_z4,sf_rl_t2);
            RL_foot_vel[2] = fifth_order_poly_dot(c_sf_z4,sf_rl_t2);
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
            tar_rr_foot_pos = pre_com_pos[0] - 0.35 + pre_x_moving_speed*tf + (pre_x_moving_speed + x_moving_speed)*ts/2.0 + x_moving_speed*tf + x_moving_speed*ts/2.0;
        }
    }
    else if (ft_cnt < 2 * ft_step_cnt) {
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
        com_pos[2] = fifth_order_poly(c_com_z2,com_t);

        com_vel[0] = pre_x_moving_speed;
        com_vel[1] = pre_com_vel[1];
        com_vel[2] = fifth_order_poly_dot(c_com_z2,com_t);
        
        com_acc[2] = fifth_order_poly_2dot(c_com_z2,com_t);

        RL_foot_pos[0] = pre_RL_foot_pos[0] + (tar_rl_foot_pos - pre_RL_foot_pos[0])/2.0*(1-cos(PI2/(2*(ft_step_time + tf))*sf_rl_t));
        FR_foot_pos[0] = pre_FR_foot_pos[0] + (tar_rl_foot_pos - pre_RL_foot_pos[0])/2.0*(1-cos(PI2/(2*(ft_step_time + tf))*sf_rl_t));
        RL_foot_vel[0] = (tar_rl_foot_pos - pre_RL_foot_pos[0])/2.0*PI2/(2*(ft_step_time + tf))*(sin(PI2/(2*(ft_step_time + tf))*sf_rl_t));
        
        sf_rl_t2 = sf_rl_t - (ft_step_time + tf)/2.0;
        RL_foot_pos[2] = tar_init_RL_foot_pos[2] + fifth_order_poly(c_sf_z4,sf_rl_t2);
        FR_foot_pos[2] = tar_init_FR_foot_pos[2] + fifth_order_poly(c_sf_z4,sf_rl_t2);
        RL_foot_vel[2] = fifth_order_poly_dot(c_sf_z4,sf_rl_t2);

        RR_foot_pos[0] = pre_RR_foot_pos[0] + (tar_rr_foot_pos - pre_RR_foot_pos[0])/2.0*(1-cos(PI2/(2*(ft_step_time + tf))*sf_rr_t));
        FL_foot_pos[0] = pre_FL_foot_pos[0] + (tar_rr_foot_pos - pre_RR_foot_pos[0])/2.0*(1-cos(PI2/(2*(ft_step_time + tf))*sf_rr_t));
        RR_foot_vel[0] = (tar_rr_foot_pos - pre_RR_foot_pos[0])/2.0*PI2/(2*(ft_step_time + tf))*(sin(PI2/(2*(ft_step_time + tf))*sf_rr_t));
        
        RR_foot_pos[2] = tar_init_RR_foot_pos[2] + fifth_order_poly(c_sf_z3,sf_rr_t);
        FL_foot_pos[2] = tar_init_FL_foot_pos[2] + fifth_order_poly(c_sf_z3,sf_rr_t);
        RR_foot_vel[2] = fifth_order_poly_dot(c_sf_z3,sf_rr_t);

        FL_foot_vel = RR_foot_vel;
        FR_foot_vel = RL_foot_vel;
        
        if (ft_cnt == 2 * ft_step_cnt - 1) {

            pre_com_pos(0) = pre_com_pos(0) + pre_x_moving_speed*tf;
            pre_com_pos(1) = com_pos(1);
            pre_com_pos(2) = h_2; 

            pre_com_vel[0] = pre_x_moving_speed;
            pre_com_vel[1] = com_vel[1];
            pre_com_vel[2] = v_2;

            pre_FR_foot_pos[0] = pre_FR_foot_pos[0] + (tar_rl_foot_pos - pre_RL_foot_pos[0]);//tar_rl_foot_pos;
            pre_RL_foot_pos[0] = pre_RL_foot_pos[0] + (tar_rl_foot_pos - pre_RL_foot_pos[0]);//pre_RL_foot_pos[0] + tar_rl_foot_pos;
            
            FT_COM_X_Traj_Gen();
        }
    }
    else if (ft_cnt < 2 * ft_step_cnt + ts_cnt) {
        //        cout << "=========== ft_phase = 5 ===========" << endl;
        ft_phase = 5;
        _c << 1, 0, 0, 1;
        contact_num = 2;
        com_t = ft_time - 2 * ft_step_time;
        sf_rr_t = sf_rr_t + dt;

        com_pos[0] = fifth_order_poly(c_com_x1,com_t);
        com_pos[1] = pre_com_pos(1);
        com_pos[2] = fifth_order_poly(c_com_z3,com_t);

        com_vel[0] = fifth_order_poly_dot(c_com_x1,com_t);
        com_vel[1] = pre_com_vel[1];
        com_vel[2] = fifth_order_poly_dot(c_com_z3,com_t);

        com_acc[2] = fifth_order_poly_2dot(c_com_z3,com_t);
        
        RL_foot_pos[0] = pre_RL_foot_pos[0];
        FR_foot_pos[0] = pre_FR_foot_pos[0];
        RL_foot_vel[0] = tar_init_RL_foot_vel[0];
        
        RL_foot_pos[2] = tar_init_RL_foot_pos[2];
        FR_foot_pos[2] = tar_init_RL_foot_pos[2];
        RL_foot_vel[2] = tar_init_RL_foot_vel[2];
        
        RR_foot_pos[0] = pre_RR_foot_pos[0] + (tar_rr_foot_pos - pre_RR_foot_pos[0])/2.0*(1-cos(PI2/(2*(ft_step_time + tf))*sf_rr_t));
        FL_foot_pos[0] = pre_FL_foot_pos[0] + (tar_rr_foot_pos - pre_RR_foot_pos[0])/2.0*(1-cos(PI2/(2*(ft_step_time + tf))*sf_rr_t));
        RR_foot_vel[0] = (tar_rr_foot_pos - pre_RR_foot_pos[0])/2.0*PI2/(2*(ft_step_time + tf))*(sin(PI2/(2*(ft_step_time + tf))*sf_rr_t));
        
        if(sf_rr_t < (ft_step_time + tf)/2.0){
            RR_foot_pos[2] = tar_init_RR_foot_pos[2] + fifth_order_poly(c_sf_z3,sf_rr_t);
            FL_foot_pos[2] = tar_init_FL_foot_pos[2] + fifth_order_poly(c_sf_z3,sf_rr_t);
            RR_foot_vel[2] = fifth_order_poly_dot(c_sf_z3,sf_rr_t);
 
        }
        else{
            sf_rr_t2 = sf_rr_t - (ft_step_time + tf)/2.0;
            RR_foot_pos[2] = tar_init_RR_foot_pos[2] + fifth_order_poly(c_sf_z4,sf_rr_t2);
            FL_foot_pos[2] = tar_init_FL_foot_pos[2] + fifth_order_poly(c_sf_z4,sf_rr_t2);
            RR_foot_vel[2] = fifth_order_poly_dot(c_sf_z4,sf_rr_t2);
        }
        
        FL_foot_vel = RR_foot_vel;
        FR_foot_vel = RL_foot_vel;
        
        if (ft_cnt == 2 * ft_step_cnt + ts_cnt - 1) {
            //            cout << "test!!!!!!!!!!!!!!!!!!!!!!!!! " << endl;

            pre_com_pos[0] = pre_com_pos[0] + pre_x_moving_speed*ts/2.0 + x_moving_speed*ts/2.0;
            pre_com_pos[1] = com_pos(1);
            pre_com_pos[2] = h_1; 

            pre_com_vel[0] = x_moving_speed;
            pre_com_vel[1] = com_vel[1];
            pre_com_vel[2] = v_1;
                   
            sf_rl_t = 0;
            
            if (sub_ctrl_flag == false) {
                ft_cnt = ts_cnt - 1;
                
                pre_x_moving_speed = x_moving_speed;
                x_moving_speed = tmp_x_moving_speed;

            }
            else{
                pre_x_moving_speed = x_moving_speed;
                x_moving_speed = 0;    
            }
            
            tar_rl_foot_pos = pre_com_pos[0] - 0.35 + pre_x_moving_speed*tf + (pre_x_moving_speed + x_moving_speed)*ts/2.0 + x_moving_speed*tf + x_moving_speed*ts/2.0;
        }
    }
    else if (ft_cnt < 3 * ft_step_cnt) {
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
        com_pos[2] = fifth_order_poly(c_com_z2,com_t);

        com_vel[0] = pre_x_moving_speed;
        com_vel[2] = fifth_order_poly_dot(c_com_z2,com_t);

        com_acc[2] = fifth_order_poly_2dot(c_com_z2,com_t);
        
        RL_foot_pos[0] = pre_RL_foot_pos[0] + (tar_rl_foot_pos - pre_RL_foot_pos[0])/2.0*(1-cos(PI2/(2*(ft_step_time))*sf_rl_t));
        FR_foot_pos[0] = pre_FR_foot_pos[0] + (tar_rl_foot_pos - pre_RL_foot_pos[0])/2.0*(1-cos(PI2/(2*(ft_step_time))*sf_rl_t));
        RL_foot_vel[0] = (tar_rl_foot_pos - pre_RL_foot_pos[0])/2.0*PI2/(2*(ft_step_time))*(sin(PI2/(2*(ft_step_time))*sf_rl_t));
        
        RL_foot_pos[2] = tar_init_RL_foot_pos[2] + fifth_order_poly(c_sf_z1,sf_rl_t);
        FR_foot_pos[2] = tar_init_FR_foot_pos[2] + fifth_order_poly(c_sf_z1,sf_rl_t);
        RL_foot_vel[2] = fifth_order_poly_dot(c_sf_z1,sf_rl_t);

        RR_foot_pos[0] = pre_RR_foot_pos[0] + (tar_rr_foot_pos - pre_RR_foot_pos[0])/2.0*(1-cos(PI2/(2*(ft_step_time + tf))*sf_rr_t));
        FL_foot_pos[0] = pre_FL_foot_pos[0] + (tar_rr_foot_pos - pre_RR_foot_pos[0])/2.0*(1-cos(PI2/(2*(ft_step_time + tf))*sf_rr_t));
        RR_foot_vel[0] = (tar_rr_foot_pos - pre_RR_foot_pos[0])/2.0*PI2/(2*(ft_step_time + tf))*(sin(PI2/(2*(ft_step_time + tf))*sf_rr_t));

        sf_rr_t2 = sf_rr_t -(ft_step_time + tf)/2.0;
        RR_foot_pos[2] = tar_init_RR_foot_pos[2] + fifth_order_poly(c_sf_z4,sf_rr_t2);
        FL_foot_pos[2] = tar_init_FL_foot_pos[2] + fifth_order_poly(c_sf_z4,sf_rr_t2);
        RR_foot_vel[2] = fifth_order_poly_dot(c_sf_z4,sf_rr_t2);

        FL_foot_vel = RR_foot_vel;
        FR_foot_vel = RL_foot_vel;
        
        if (ft_cnt == 3*ft_step_cnt - 1) {
            
            pre_com_pos[0] = pre_com_pos[0] + pre_x_moving_speed*tf;
            pre_com_pos[1] = com_pos[1];
            pre_com_pos[2] = h_2;

            pre_com_vel[0] = com_vel[0];
            pre_com_vel[1] = com_vel[1];
            pre_com_vel[2] = v_2;

            pre_FL_foot_pos[0] = pre_FL_foot_pos[0] + (tar_rr_foot_pos - pre_RR_foot_pos[0]);
            pre_RR_foot_pos[0] = pre_RR_foot_pos[0] + (tar_rr_foot_pos - pre_RR_foot_pos[0]);
                    
            FT_COM_X_Traj_Gen();
//            // =============== COM =============== //                
//            init_x[0] = pre_com_pos[0];
//            init_x[1] = x_moving_speed;
//            init_x[2] = 0;
//
//            final_x[0] = pre_com_pos[0] + x_moving_speed * (ts) / 2.0;
//            final_x[1] = 0;
//            final_x[2] = 0;
//
//            coefficient_5thPoly(init_x, final_x, ts, c_com_x2);            
        }        
    }
    else if (ft_cnt < 3*ft_step_cnt + ts_cnt) {
        //        cout << "=========== ft_phase = 7 (Final stance phase) ===========" << endl;
        ft_phase = 7;
        _c << 0, 1, 1, 0;
        contact_num = 2;
        com_t = ft_time - 3*ft_step_time;
        sf_rl_t = sf_rl_t + dt;

        com_pos[0] = fifth_order_poly(c_com_x1,com_t);
        com_pos[1] = pre_com_pos(1);
        com_pos[2] = fifth_order_poly(c_com_z4,com_t);

        com_vel[0] = fifth_order_poly_dot(c_com_x1,com_t);
        com_vel[1] = pre_com_vel[1];
        com_vel[2] = fifth_order_poly_dot(c_com_z4,com_t);
        
        com_acc[2] = fifth_order_poly_2dot(c_com_z4,com_t);
        
        RL_foot_pos[0] = pre_RL_foot_pos[0] + (tar_rl_foot_pos - pre_RL_foot_pos[0])/2.0*(1-cos(PI2/(2*(ft_step_time))*sf_rl_t));
        FR_foot_pos[0] = pre_FR_foot_pos[0] + (tar_rl_foot_pos - pre_RL_foot_pos[0])/2.0*(1-cos(PI2/(2*(ft_step_time))*sf_rl_t));
        RL_foot_vel[0] = (tar_rl_foot_pos - pre_RL_foot_pos[0])/2.0*PI2/(2*(ft_step_time))*(sin(PI2/(2*(ft_step_time))*sf_rl_t));
        
        if(sf_rl_t < (ft_step_time)/2.0){
            RL_foot_pos[2] = tar_init_RL_foot_pos[2] + fifth_order_poly(c_sf_z1,sf_rl_t);
            FR_foot_pos[2] = tar_init_FR_foot_pos[2] + fifth_order_poly(c_sf_z1,sf_rl_t);
            RL_foot_vel[2] = fifth_order_poly_dot(c_sf_z1,sf_rl_t);
 
        }
        else{
            sf_rl_t2 = sf_rl_t - (ft_step_time)/2.0;
            RL_foot_pos[2] = tar_init_RL_foot_pos[2] + fifth_order_poly(c_sf_z2,sf_rl_t2);
            FR_foot_pos[2] = tar_init_FR_foot_pos[2] + fifth_order_poly(c_sf_z2,sf_rl_t2);
            RL_foot_vel[2] = fifth_order_poly_dot(c_sf_z2,sf_rl_t2);
        }

        RR_foot_pos[0] = pre_RR_foot_pos[0];
        FL_foot_pos[0] = pre_FL_foot_pos[0];
        RR_foot_vel[0] = tar_init_RR_foot_vel[0];
        
        RR_foot_pos[2] = tar_init_RR_foot_pos[2];
        FL_foot_pos[2] = tar_init_FL_foot_pos[2];
        RR_foot_vel[2] = tar_init_RR_foot_vel[2];
        
        FL_foot_vel = RR_foot_vel;
        FR_foot_vel = RL_foot_vel;
        
        if (ft_cnt == 3*ft_step_cnt + ts_cnt - 1) {
            pre_com_pos[0] = pre_com_pos[0] + pre_x_moving_speed * (ts) / 2.0 + x_moving_speed * (ts) / 2.0;//pre_com_pos[0] + x_moving_speed * (ts) / 2.0;
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
//        if (ft_cnt == ft_step_cnt + ts_cnt - 1) {
//            pre_com_pos[0] = pre_com_pos[0] + pre_x_moving_speed * (ts) / 2.0 + x_moving_speed * (ts) / 2.0;
//            pre_com_pos[1] = com_pos(1);
//            pre_com_pos[2] = h_1;
//
//            pre_com_vel[0] = x_moving_speed;
//            pre_com_vel[1] = com_vel[1];
//            pre_com_vel[2] = v_1;
//            
//            sf_rr_t = 0;
//            
//            pre_x_moving_speed = x_moving_speed;
//            x_moving_speed = tmp_x_moving_speed;
//            
//            tar_rr_foot_pos = pre_x_moving_speed*(ts + tf) + x_moving_speed*(ts + tf);
//        }
    }
    else {
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

    base_pos = com_pos + base_offset;
    target_pos[6] = 0;
    ft_cnt++;
}
//void CRobot::Flying_Trot_Running2(void)
//{
//    ft_time = (double) ft_cnt*dt;
//    base_ori(2) = tmp_base_ori(2);
//
//    if (CP_con_onoff_flag == true) Get_CP();
//
//    if (ft_cnt == 0) {
//        //        cout << "=========== ft_phase = 0 ===========" << endl;
//        ft_phase = 0;
//        moving_done_flag = false;
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
//        x_moving_speed = 0;
//        y_moving_speed = 0;
//
//        FT_COM_SF_Z_Traj_Gen();
//    }
//    else if (ft_cnt < ts_cnt) {
//        //        cout << "=========== ft_phase = 1 ===========" << endl;
//        ft_phase = 1;
//        _c << 1, 0, 0, 1;
//        contact_num = 2;
//        tmp_t = ft_time;
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
//        com_pos[2] = c_com_z1[5] * pow(tmp_t, 5) + c_com_z1[4] * pow(tmp_t, 4) + c_com_z1[3] * pow(tmp_t, 3) + c_com_z1[2] * pow(tmp_t, 2) + c_com_z1[1] * pow(tmp_t, 1) + c_com_z1[0];
//        com_vel[2] = 5 * c_com_z1[5] * pow(tmp_t, 4) + 4 * c_com_z1[4] * pow(tmp_t, 3) + 3 * c_com_z1[3] * pow(tmp_t, 2) + 2 * c_com_z1[2] * pow(tmp_t, 1) + 1 * c_com_z1[1];
//
//        RR_foot_pos[2] = tar_init_RR_foot_pos[2] + c_sf_z1[5] * pow(tmp_t, 5) + c_sf_z1[4] * pow(tmp_t, 4) + c_sf_z1[3] * pow(tmp_t, 3) + c_sf_z1[2] * pow(tmp_t, 2) + c_sf_z1[1] * pow(tmp_t, 1) + c_sf_z1[0];
//        FL_foot_pos[2] = tar_init_FL_foot_pos[2] + c_sf_z1[5] * pow(tmp_t, 5) + c_sf_z1[4] * pow(tmp_t, 4) + c_sf_z1[3] * pow(tmp_t, 3) + c_sf_z1[2] * pow(tmp_t, 2) + c_sf_z1[1] * pow(tmp_t, 1) + c_sf_z1[0];
//        RR_foot_vel[2] = 5*c_sf_z1[5] * pow(tmp_t, 4) + 4*c_sf_z1[4] * pow(tmp_t, 3) + 3*c_sf_z1[3] * pow(tmp_t, 2) + 2*c_sf_z1[2] * pow(tmp_t, 1) + 1*c_sf_z1[1];
//        FL_foot_vel[2] = 5*c_sf_z1[5] * pow(tmp_t, 4) + 4*c_sf_z1[4] * pow(tmp_t, 3) + 3*c_sf_z1[3] * pow(tmp_t, 2) + 2*c_sf_z1[2] * pow(tmp_t, 1) + 1*c_sf_z1[1];
//
//    }
//    else if (ft_cnt < ft_step_cnt) {
//        //        cout << "=========== ft_phase = 2 ===========" << endl;
//        ft_phase = 2;
//        _c << 0, 0, 0, 0;
//        contact_num = 0;
//        tmp_t = ft_time - ts;
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
//        com_pos[2] = c_com_z2[5] * pow(tmp_t, 5) + c_com_z2[4] * pow(tmp_t, 4) + c_com_z2[3] * pow(tmp_t, 3) + c_com_z2[2] * pow(tmp_t, 2) + c_com_z2[1] * pow(tmp_t, 1) + c_com_z2[0];
//        com_vel[2] = 5 * c_com_z2[5] * pow(tmp_t, 4) + 4 * c_com_z2[4] * pow(tmp_t, 3) + 3 * c_com_z2[3] * pow(tmp_t, 2) + 2 * c_com_z2[2] * pow(tmp_t, 1) + 1 * c_com_z2[1];
//
//        RL_foot_pos[2] = tar_init_RL_foot_pos[2] + c_sf_z2[5] * pow(tmp_t, 5) + c_sf_z2[4] * pow(tmp_t, 4) + c_sf_z2[3] * pow(tmp_t, 3) + c_sf_z2[2] * pow(tmp_t, 2) + c_sf_z2[1] * pow(tmp_t, 1) + c_sf_z2[0];
//        RR_foot_pos[2] = tar_init_RR_foot_pos[2] + c_sf_z3[5] * pow(tmp_t, 5) + c_sf_z3[4] * pow(tmp_t, 4) + c_sf_z3[3] * pow(tmp_t, 3) + c_sf_z3[2] * pow(tmp_t, 2) + c_sf_z3[1] * pow(tmp_t, 1) + c_sf_z3[0];
//        FL_foot_pos[2] = tar_init_FL_foot_pos[2] + c_sf_z3[5] * pow(tmp_t, 5) + c_sf_z3[4] * pow(tmp_t, 4) + c_sf_z3[3] * pow(tmp_t, 3) + c_sf_z3[2] * pow(tmp_t, 2) + c_sf_z3[1] * pow(tmp_t, 1) + c_sf_z3[0];
//        FR_foot_pos[2] = tar_init_FR_foot_pos[2] + c_sf_z2[5] * pow(tmp_t, 5) + c_sf_z2[4] * pow(tmp_t, 4) + c_sf_z2[3] * pow(tmp_t, 3) + c_sf_z2[2] * pow(tmp_t, 2) + c_sf_z2[1] * pow(tmp_t, 1) + c_sf_z2[0];
//        
//        RL_foot_vel[2] = 5*c_sf_z2[5] * pow(tmp_t, 4) + 4*c_sf_z2[4] * pow(tmp_t, 3) + 3*c_sf_z2[3] * pow(tmp_t, 2) + 2*c_sf_z2[2] * pow(tmp_t, 1) + 1*c_sf_z2[1];
//        RR_foot_vel[2] = 5*c_sf_z3[5] * pow(tmp_t, 4) + 4*c_sf_z3[4] * pow(tmp_t, 3) + 3*c_sf_z3[3] * pow(tmp_t, 2) + 2*c_sf_z3[2] * pow(tmp_t, 1) + 1*c_sf_z3[1];
//        FL_foot_vel[2] = 5*c_sf_z3[5] * pow(tmp_t, 4) + 4*c_sf_z3[4] * pow(tmp_t, 3) + 3*c_sf_z3[3] * pow(tmp_t, 2) + 2*c_sf_z3[2] * pow(tmp_t, 1) + 1*c_sf_z3[1];
//        FR_foot_vel[2] = 5*c_sf_z2[5] * pow(tmp_t, 4) + 4*c_sf_z2[4] * pow(tmp_t, 3) + 3*c_sf_z2[3] * pow(tmp_t, 2) + 2*c_sf_z2[2] * pow(tmp_t, 1) + 1*c_sf_z2[1];
//
//        if (ft_cnt == ft_step_cnt - 1) {
//            pre_x_moving_speed = x_moving_speed;
//            pre_y_moving_speed = y_moving_speed;
//            x_moving_speed = tmp_x_moving_speed;
//            y_moving_speed = tmp_y_moving_speed;
//
//            pre_com_pos[0] = com_pos[0];
//            pre_com_pos[1] = com_pos[1];
//            pre_com_pos[2] = h_2;
//
//            pre_com_vel[0] = com_vel[0];
//            pre_com_vel[1] = com_vel[1];
//            pre_com_vel[2] = v_2;
//
//            pre_RL_foot_pos[0] = RL_foot_pos[0];
//            pre_RL_foot_pos[1] = RL_foot_pos[1];
//            pre_RL_foot_pos[2] = tar_init_RL_foot_pos[2] + swing_foot_height;
//
//            pre_RR_foot_pos[0] = RR_foot_pos[0];
//            pre_RR_foot_pos[1] = RR_foot_pos[1];
//            pre_RR_foot_pos[2] = tar_init_RR_foot_pos[2];
//
//            pre_FL_foot_pos[0] = FL_foot_pos[0];
//            pre_FL_foot_pos[1] = FL_foot_pos[1];
//            pre_FL_foot_pos[2] = tar_init_FL_foot_pos[2];
//
//            pre_FR_foot_pos[0] = FR_foot_pos[0];
//            pre_FR_foot_pos[1] = FR_foot_pos[1];
//            pre_FR_foot_pos[2] = tar_init_FR_foot_pos[2] + swing_foot_height;
//
//            FT_COM_SF_X_Traj_Gen();
//        }
//    }
//    else if (ft_cnt < ft_step_cnt + ts_cnt) {
//        //        cout << "=========== ft_phase = 3 ===========" << endl;
//        ft_phase = 3;
//        _c << 0, 1, 1, 0;
//        contact_num = 2;
//        tmp_t = ft_time - ft_step_time;
//        tmp_t2 = ft_time - ft_step_time;
//
//        com_pos[0] = c_com_x1[5] * pow(tmp_t, 5) + c_com_x1[4] * pow(tmp_t, 4) + c_com_x1[3] * pow(tmp_t, 3) + c_com_x1[2] * pow(tmp_t, 2) + c_com_x1[1] * pow(tmp_t, 1) + c_com_x1[0]; //init_com_pos(0) + x_moving_speed * (ts / 2 + tf + t2);
//        com_pos[1] = pre_com_pos(1);
//        com_pos[2] = c_com_z3[5] * pow(tmp_t, 5) + c_com_z3[4] * pow(tmp_t, 4) + c_com_z3[3] * pow(tmp_t, 3) + c_com_z3[2] * pow(tmp_t, 2) + c_com_z3[1] * pow(tmp_t, 1) + c_com_z3[0];
//
//        com_vel[0] = 5 * c_com_x1[5] * pow(tmp_t, 4) + 4 * c_com_x1[4] * pow(tmp_t, 3) + 3 * c_com_x1[3] * pow(tmp_t, 2) + 2 * c_com_x1[2] * pow(tmp_t, 1) + 1 * c_com_x1[1]; //init_com_pos(0) + x_moving_speed * (ts / 2 + tf + t2);
//        com_vel[1] = pre_com_vel[1];
//        com_vel[2] = 5 * c_com_z3[5] * pow(tmp_t, 4) + 4 * c_com_z3[4] * pow(tmp_t, 3) + 3 * c_com_z3[3] * pow(tmp_t, 2) + 2 * c_com_z3[2] * pow(tmp_t, 1) + 1 * c_com_z3[1];
//
//        RL_foot_pos[0] = pre_RL_foot_pos[0] + c_sf_x1[5] * pow(tmp_t2, 5) + c_sf_x1[4] * pow(tmp_t2, 4) + c_sf_x1[3] * pow(tmp_t2, 3) + c_sf_x1[2] * pow(tmp_t2, 2) + c_sf_x1[1] * pow(tmp_t2, 1) + c_sf_x1[0];
//        RL_foot_pos[1] = pre_RL_foot_pos[1];
//        RL_foot_pos[2] = pre_RL_foot_pos[2];
//
//        RR_foot_pos[0] = pre_RR_foot_pos[0];
//        RR_foot_pos[1] = pre_RR_foot_pos[1];
//        RR_foot_pos[2] = pre_RR_foot_pos[2];
//
//        FL_foot_pos[0] = pre_FL_foot_pos[0];
//        FL_foot_pos[1] = pre_FL_foot_pos[1];
//        FL_foot_pos[2] = pre_FL_foot_pos[2];
//
//        FR_foot_pos[0] = pre_FR_foot_pos[0] + c_sf_x1[5] * pow(tmp_t2, 5) + c_sf_x1[4] * pow(tmp_t2, 4) + c_sf_x1[3] * pow(tmp_t2, 3) + c_sf_x1[2] * pow(tmp_t2, 2) + c_sf_x1[1] * pow(tmp_t2, 1) + c_sf_x1[0];
//        FR_foot_pos[1] = pre_FR_foot_pos[1];
//        FR_foot_pos[2] = pre_FR_foot_pos[2];
//        
//        RL_foot_vel = tar_init_RL_foot_vel;
//        RR_foot_vel = tar_init_RR_foot_vel;
//        FL_foot_vel = tar_init_FL_foot_vel;
//        FR_foot_vel = tar_init_FR_foot_vel;
//        
//        RL_foot_vel[0] = 5*c_sf_x1[5] * pow(tmp_t2, 4) + 4*c_sf_x1[4] * pow(tmp_t2, 3) + 3*c_sf_x1[3] * pow(tmp_t2, 2) + 2*c_sf_x1[2] * pow(tmp_t2, 1) + 1*c_sf_x1[1];
//        FR_foot_vel[0] = 5*c_sf_x1[5] * pow(tmp_t2, 4) + 4*c_sf_x1[4] * pow(tmp_t2, 3) + 3*c_sf_x1[3] * pow(tmp_t2, 2) + 2*c_sf_x1[2] * pow(tmp_t2, 1) + 1*c_sf_x1[1];     
//
//        if (ft_cnt == ft_step_cnt + ts_cnt - 1) {
//            pre_com_pos[0] = pre_com_pos[0] + pre_x_moving_speed * (ts) / 2.0 + x_moving_speed * (ts) / 2.0;
//            pre_com_pos[1] = com_pos(1);
//            pre_com_pos[2] = h_1; //com_pos(2);
//
//            pre_com_vel[0] = x_moving_speed;
//            pre_com_vel[1] = com_vel[1];
//            pre_com_vel[2] = v_1;
//
//            //            pre_RL_foot_pos[0] = RL_foot_pos[0];//pre_RL_foot_pos[0] + pre_x_moving_speed * ft_step_time + x_moving_speed * ft_step_time;//RL_foot_pos[0];//x_moving_speed * (ft_step_time);
//            pre_RL_foot_pos[1] = RL_foot_pos[1];
//            pre_RL_foot_pos[2] = RL_foot_pos[2];
//
//            pre_RR_foot_pos[0] = RR_foot_pos[0];
//            pre_RR_foot_pos[1] = RR_foot_pos[1];
//            pre_RR_foot_pos[2] = RR_foot_pos[2];
//
//            pre_FL_foot_pos[0] = FL_foot_pos[0];
//            pre_FL_foot_pos[1] = FL_foot_pos[1];
//            pre_FL_foot_pos[2] = FL_foot_pos[2];
//
//            //            pre_FR_foot_pos[0] = FR_foot_pos[0];//pre_FR_foot_pos[0] + pre_x_moving_speed * ft_step_time + x_moving_speed * ft_step_time;//FR_foot_pos[0];
//            pre_FR_foot_pos[1] = FR_foot_pos[1];
//            pre_FR_foot_pos[2] = FR_foot_pos[2];
//
//            //            cout << "pre_RL_foot_pos[0] = " << pre_RL_foot_pos[0] << endl;
//            //            printf("[final] FR_foot_pos[0] = %f, pre_FR_foot_pos[0] = %f\n",FR_foot_pos[0],pre_FR_foot_pos[0]);
//        }
//    }
//    else if (ft_cnt < 2 * ft_step_cnt) {
//        //        cout << "=========== ft_phase = 4 ===========" << endl;
//        ft_phase = 4;
//        _c << 0, 0, 0, 0;
//        contact_num = 0;
//        tmp_t = ft_time - ft_step_time - ts;
//        tmp_t2 = ft_time - ft_step_time;
//
//        com_pos[0] = pre_com_pos(0) + x_moving_speed*tmp_t;
//        com_pos[1] = pre_com_pos(1);
//        com_pos[2] = c_com_z2[5] * pow(tmp_t, 5) + c_com_z2[4] * pow(tmp_t, 4) + c_com_z2[3] * pow(tmp_t, 3) + c_com_z2[2] * pow(tmp_t, 2) + c_com_z2[1] * pow(tmp_t, 1) + c_com_z2[0];
//
//        com_vel[0] = x_moving_speed;
//        com_vel[1] = pre_com_vel[1];
//        com_vel[2] = 5 * c_com_z2[5] * pow(tmp_t, 4) + 4 * c_com_z2[4] * pow(tmp_t, 3) + 3 * c_com_z2[3] * pow(tmp_t, 2) + 2 * c_com_z2[2] * pow(tmp_t, 1) + 1 * c_com_z2[1];
//
//        RL_foot_pos[0] = pre_RL_foot_pos[0] + c_sf_x1[5] * pow(tmp_t2, 5) + c_sf_x1[4] * pow(tmp_t2, 4) + c_sf_x1[3] * pow(tmp_t2, 3) + c_sf_x1[2] * pow(tmp_t2, 2) + c_sf_x1[1] * pow(tmp_t2, 1) + c_sf_x1[0];
//        RL_foot_pos[1] = pre_RL_foot_pos[1];
//        RL_foot_pos[2] = tar_init_RL_foot_pos[2] + c_sf_z3[5] * pow(tmp_t, 5) + c_sf_z3[4] * pow(tmp_t, 4) + c_sf_z3[3] * pow(tmp_t, 3) + c_sf_z3[2] * pow(tmp_t, 2) + c_sf_z3[1] * pow(tmp_t, 1) + c_sf_z3[0];
//
//        RR_foot_pos[0] = pre_RR_foot_pos[0];
//        RR_foot_pos[1] = pre_RR_foot_pos[1];
//        RR_foot_pos[2] = tar_init_RR_foot_pos[2] + c_sf_z2[5] * pow(tmp_t, 5) + c_sf_z2[4] * pow(tmp_t, 4) + c_sf_z2[3] * pow(tmp_t, 3) + c_sf_z2[2] * pow(tmp_t, 2) + c_sf_z2[1] * pow(tmp_t, 1) + c_sf_z2[0];
//        //        RR_foot_pos[2] = init_RR_foot_pos[2] + swing_foot_height/2.0*(1-cos(PI2/(2*tf)*tmp_t));
//
//        FL_foot_pos[0] = pre_FL_foot_pos[0];
//        FL_foot_pos[1] = pre_FL_foot_pos[1];
//        FL_foot_pos[2] = tar_init_FL_foot_pos[2] + c_sf_z2[5] * pow(tmp_t, 5) + c_sf_z2[4] * pow(tmp_t, 4) + c_sf_z2[3] * pow(tmp_t, 3) + c_sf_z2[2] * pow(tmp_t, 2) + c_sf_z2[1] * pow(tmp_t, 1) + c_sf_z2[0];
//        //        FL_foot_pos[2] = init_FL_foot_pos[2] + swing_foot_height/2.0*(1-cos(PI2/(2*tf)*tmp_t));
//
//        FR_foot_pos[0] = pre_FR_foot_pos[0] + c_sf_x1[5] * pow(tmp_t2, 5) + c_sf_x1[4] * pow(tmp_t2, 4) + c_sf_x1[3] * pow(tmp_t2, 3) + c_sf_x1[2] * pow(tmp_t2, 2) + c_sf_x1[1] * pow(tmp_t2, 1) + c_sf_x1[0];
//        FR_foot_pos[1] = pre_FR_foot_pos[1];
//        FR_foot_pos[2] = tar_init_FR_foot_pos[2] + c_sf_z3[5] * pow(tmp_t, 5) + c_sf_z3[4] * pow(tmp_t, 4) + c_sf_z3[3] * pow(tmp_t, 3) + c_sf_z3[2] * pow(tmp_t, 2) + c_sf_z3[1] * pow(tmp_t, 1) + c_sf_z3[0];
//
//        // ---------------------------------------------------------------- //
//        RL_foot_vel[0] = 5*c_sf_x1[5] * pow(tmp_t2, 4) + 4*c_sf_x1[4] * pow(tmp_t2, 3) + 3*c_sf_x1[3] * pow(tmp_t2, 2) + 2*c_sf_x1[2] * pow(tmp_t2, 1) + 1*c_sf_x1[1];
//        RL_foot_vel[1] = tar_init_RL_foot_vel[1];
//        RL_foot_vel[2] = 5*c_sf_z3[5] * pow(tmp_t, 4) + 4*c_sf_z3[4] * pow(tmp_t, 3) + 3*c_sf_z3[3] * pow(tmp_t, 2) + 2*c_sf_z3[2] * pow(tmp_t, 1) + 1*c_sf_z3[1];
//
//        RR_foot_vel[0] = tar_init_RR_foot_vel[0];
//        RR_foot_vel[1] = tar_init_RR_foot_vel[1];
//        RR_foot_vel[2] = 5*c_sf_z2[5] * pow(tmp_t, 4) + 4*c_sf_z2[4] * pow(tmp_t, 3) + 3*c_sf_z2[3] * pow(tmp_t, 2) + 2*c_sf_z2[2] * pow(tmp_t, 1) + 1*c_sf_z2[1];
//
//        FL_foot_vel[0] = tar_init_FL_foot_vel[0];
//        FL_foot_vel[1] = tar_init_FL_foot_vel[1];
//        FL_foot_vel[2] = RR_foot_vel[2];
//
//        FR_foot_vel[0] = RL_foot_vel[0];
//        FR_foot_vel[1] = tar_init_FR_foot_vel[1];
//        FR_foot_vel[2] = RL_foot_vel[2];
//
//        
//        if (ft_cnt == 2 * ft_step_cnt - 1) {
//            tmp_t2 = ts + tf;
//
//            pre_com_pos(0) = pre_com_pos(0) + x_moving_speed*tf;
//            pre_com_pos(1) = com_pos(1);
//            pre_com_pos(2) = h_2; //com_pos(2);
//
//            pre_com_vel[0] = x_moving_speed;
//            pre_com_vel[1] = com_vel[1];
//            pre_com_vel[2] = v_2;
//
//            pre_RL_foot_pos[0] = pre_RL_foot_pos[0] + c_sf_x1[5] * pow(tmp_t2, 5) + c_sf_x1[4] * pow(tmp_t2, 4) + c_sf_x1[3] * pow(tmp_t2, 3) + c_sf_x1[2] * pow(tmp_t2, 2) + c_sf_x1[1] * pow(tmp_t2, 1) + c_sf_x1[0]; //RL_foot_pos[0];//pre_RL_foot_pos(0) + pre_x_moving_speed * step_time + x_moving_speed * step_time;//RL_foot_pos[0];//x_moving_speed * (ft_step_time);
//            pre_RL_foot_pos[1] = RL_foot_pos[1];
//            pre_RL_foot_pos[2] = tar_init_RL_foot_pos[2];
//
//            pre_RR_foot_pos[0] = RR_foot_pos[0];
//            pre_RR_foot_pos[1] = RR_foot_pos[1];
//            pre_RR_foot_pos[2] = tar_init_RR_foot_pos[2] + swing_foot_height;
//
//            pre_FL_foot_pos[0] = FL_foot_pos[0];
//            pre_FL_foot_pos[1] = FL_foot_pos[1];
//            pre_FL_foot_pos[2] = tar_init_FL_foot_pos[2] + swing_foot_height;
//
//            pre_FR_foot_pos[0] = pre_FR_foot_pos[0] + c_sf_x1[5] * pow(tmp_t2, 5) + c_sf_x1[4] * pow(tmp_t2, 4) + c_sf_x1[3] * pow(tmp_t2, 3) + c_sf_x1[2] * pow(tmp_t2, 2) + c_sf_x1[1] * pow(tmp_t2, 1) + c_sf_x1[0];
//            pre_FR_foot_pos[1] = FR_foot_pos[1];
//            pre_FR_foot_pos[2] = tar_init_FR_foot_pos[2];
//        }
//    }
//    else if (ft_cnt < 2 * ft_step_cnt + ts_cnt) {
//        //        cout << "=========== ft_phase = 5 ===========" << endl;
//        ft_phase = 5;
//        _c << 1, 0, 0, 1;
//        contact_num = 2;
//        tmp_t = ft_time - 2 * ft_step_time;
//        tmp_t2 = ft_time - 2 * ft_step_time;
//
//        com_pos[0] = pre_com_pos(0) + x_moving_speed*tmp_t;
//        com_pos[1] = pre_com_pos(1);
//        com_pos[2] = c_com_z3[5] * pow(tmp_t, 5) + c_com_z3[4] * pow(tmp_t, 4) + c_com_z3[3] * pow(tmp_t, 3) + c_com_z3[2] * pow(tmp_t, 2) + c_com_z3[1] * pow(tmp_t, 1) + c_com_z3[0];
//
//        com_vel[0] = x_moving_speed;
//        com_vel[1] = pre_com_vel[1];
//        com_vel[2] = 5 * c_com_z3[5] * pow(tmp_t, 4) + 4 * c_com_z3[4] * pow(tmp_t, 3) + 3 * c_com_z3[3] * pow(tmp_t, 2) + 2 * c_com_z3[2] * pow(tmp_t, 1) + 1 * c_com_z3[1];
//
//        RL_foot_pos[0] = pre_RL_foot_pos[0];
//        RL_foot_pos[1] = pre_RL_foot_pos[1];
//        RL_foot_pos[2] = pre_RL_foot_pos[2];
//
//        RR_foot_pos[0] = pre_RR_foot_pos[0] + c_sf_x2[5] * pow(tmp_t2, 5) + c_sf_x2[4] * pow(tmp_t2, 4) + c_sf_x2[3] * pow(tmp_t2, 3) + c_sf_x2[2] * pow(tmp_t2, 2) + c_sf_x2[1] * pow(tmp_t2, 1) + c_sf_x2[0];
//        RR_foot_pos[1] = pre_RR_foot_pos[1];
//        RR_foot_pos[2] = pre_RR_foot_pos[2];
//
//        FL_foot_pos[0] = pre_FL_foot_pos[0] + c_sf_x2[5] * pow(tmp_t2, 5) + c_sf_x2[4] * pow(tmp_t2, 4) + c_sf_x2[3] * pow(tmp_t2, 3) + c_sf_x2[2] * pow(tmp_t2, 2) + c_sf_x2[1] * pow(tmp_t2, 1) + c_sf_x2[0];
//        FL_foot_pos[1] = pre_FL_foot_pos[1];
//        FL_foot_pos[2] = pre_FL_foot_pos[2];
//
//        FR_foot_pos[0] = pre_FR_foot_pos[0];
//        FR_foot_pos[1] = pre_FR_foot_pos[1];
//        FR_foot_pos[2] = pre_FR_foot_pos[2];
//        
//        RL_foot_vel = tar_init_RL_foot_vel;
//        RR_foot_vel = tar_init_RR_foot_vel;
//        FL_foot_vel = tar_init_FL_foot_vel;
//        FR_foot_vel = tar_init_FR_foot_vel;
//        
//        RR_foot_vel[0] = 5*c_sf_x2[5] * pow(tmp_t2, 4) + 4*c_sf_x2[4] * pow(tmp_t2, 3) + 3*c_sf_x2[3] * pow(tmp_t2, 2) + 2*c_sf_x2[2] * pow(tmp_t2, 1) + 1*c_sf_x2[1];
//        FL_foot_vel[0] = RR_foot_vel[0];
//
//
//        if (ft_cnt == 2 * ft_step_cnt + ts_cnt - 1) {
//            //            cout << "test!!!!!!!!!!!!!!!!!!!!!!!!! " << endl;
//
//            pre_com_pos[0] = pre_com_pos[0] + x_moving_speed * (ts);
//            pre_com_pos[1] = com_pos(1);
//            pre_com_pos[2] = h_1; //com_pos(2);
//
//            pre_com_vel[0] = x_moving_speed;
//            pre_com_vel[1] = com_vel[1];
//            pre_com_vel[2] = v_1;
//
//            pre_RL_foot_pos[0] = RL_foot_pos[0];
//            pre_RL_foot_pos[1] = RL_foot_pos[1];
//            pre_RL_foot_pos[2] = RL_foot_pos[2];
//
//            //            pre_RR_foot_pos[0] = pre_RR_foot_pos(0) + x_moving_speed * ft_step_time*2;//RR_foot_pos[0];
//            pre_RR_foot_pos[1] = RR_foot_pos[1];
//            pre_RR_foot_pos[2] = RR_foot_pos[2];
//
//            //            pre_FL_foot_pos[0] = pre_FL_foot_pos(0) + x_moving_speed * ft_step_time*2;//FL_foot_pos[0];
//            pre_FL_foot_pos[1] = FL_foot_pos[1];
//            pre_FL_foot_pos[2] = FL_foot_pos[2];
//
//            pre_FR_foot_pos[0] = FR_foot_pos[0];
//            pre_FR_foot_pos[1] = FR_foot_pos[1];
//            pre_FR_foot_pos[2] = FR_foot_pos[2];
//        }
//    }
//    else if (ft_cnt < 3 * ft_step_cnt) {
//        //        cout << "=========== ft_phase = 6 ===========" << endl;
//        ft_phase = 6;
//        _c << 0, 0, 0, 0;
//        contact_num = 0;
//        tmp_t = ft_time - 2 * ft_step_time - ts;
//        tmp_t2 = ft_time - 2 * ft_step_time;
//
//        com_pos[0] = pre_com_pos(0) + x_moving_speed*tmp_t;
//        com_pos[1] = pre_com_pos(1);
//        com_pos[2] = c_com_z2[5] * pow(tmp_t, 5) + c_com_z2[4] * pow(tmp_t, 4) + c_com_z2[3] * pow(tmp_t, 3) + c_com_z2[2] * pow(tmp_t, 2) + c_com_z2[1] * pow(tmp_t, 1) + c_com_z2[0];
//
//        com_vel[0] = x_moving_speed;
//        com_vel[1] = pre_com_vel[1];
//        com_vel[2] = 5 * c_com_z2[5] * pow(tmp_t, 4) + 4 * c_com_z2[4] * pow(tmp_t, 3) + 3 * c_com_z2[3] * pow(tmp_t, 2) + 2 * c_com_z2[2] * pow(tmp_t, 1) + 1 * c_com_z2[1];
//
//        RL_foot_pos[0] = pre_RL_foot_pos[0];
//        RL_foot_pos[1] = pre_RL_foot_pos[1];
//        RL_foot_pos[2] = tar_init_RL_foot_pos[2] + c_sf_z2[5] * pow(tmp_t, 5) + c_sf_z2[4] * pow(tmp_t, 4) + c_sf_z2[3] * pow(tmp_t, 3) + c_sf_z2[2] * pow(tmp_t, 2) + c_sf_z2[1] * pow(tmp_t, 1) + c_sf_z2[0];
//
//        RR_foot_pos[0] = pre_RR_foot_pos[0] + c_sf_x2[5] * pow(tmp_t2, 5) + c_sf_x2[4] * pow(tmp_t2, 4) + c_sf_x2[3] * pow(tmp_t2, 3) + c_sf_x2[2] * pow(tmp_t2, 2) + c_sf_x2[1] * pow(tmp_t2, 1) + c_sf_x2[0];
//        RR_foot_pos[1] = pre_RR_foot_pos[1];
//        RR_foot_pos[2] = tar_init_RR_foot_pos[2] + c_sf_z3[5] * pow(tmp_t, 5) + c_sf_z3[4] * pow(tmp_t, 4) + c_sf_z3[3] * pow(tmp_t, 3) + c_sf_z3[2] * pow(tmp_t, 2) + c_sf_z3[1] * pow(tmp_t, 1) + c_sf_z3[0];
//
//        FL_foot_pos[0] = pre_FL_foot_pos[0] + c_sf_x2[5] * pow(tmp_t2, 5) + c_sf_x2[4] * pow(tmp_t2, 4) + c_sf_x2[3] * pow(tmp_t2, 3) + c_sf_x2[2] * pow(tmp_t2, 2) + c_sf_x2[1] * pow(tmp_t2, 1) + c_sf_x2[0];
//        FL_foot_pos[1] = pre_FL_foot_pos[1];
//        FL_foot_pos[2] = tar_init_FL_foot_pos[2] + c_sf_z3[5] * pow(tmp_t, 5) + c_sf_z3[4] * pow(tmp_t, 4) + c_sf_z3[3] * pow(tmp_t, 3) + c_sf_z3[2] * pow(tmp_t, 2) + c_sf_z3[1] * pow(tmp_t, 1) + c_sf_z3[0];
//
//        FR_foot_pos[0] = pre_FR_foot_pos[0];
//        FR_foot_pos[1] = pre_FR_foot_pos[1];
//        FR_foot_pos[2] = tar_init_FR_foot_pos[2] + c_sf_z2[5] * pow(tmp_t, 5) + c_sf_z2[4] * pow(tmp_t, 4) + c_sf_z2[3] * pow(tmp_t, 3) + c_sf_z2[2] * pow(tmp_t, 2) + c_sf_z2[1] * pow(tmp_t, 1) + c_sf_z2[0];
//
//        // ---------------------------------------------------------------- //
////        RL_foot_vel[0] = 5*c_sf_x1[5] * pow(tmp_t2, 4) + 4*c_sf_x1[4] * pow(tmp_t2, 3) + 3*c_sf_x1[3] * pow(tmp_t2, 2) + 2*c_sf_x1[2] * pow(tmp_t2, 1) + 1*c_sf_x1[1];
//        RL_foot_vel[0] = tar_init_RL_foot_vel[0];
//        RL_foot_vel[1] = tar_init_RL_foot_vel[1];
//        RL_foot_vel[2] = 5*c_sf_z2[5] * pow(tmp_t, 4) + 4*c_sf_z2[4] * pow(tmp_t, 3) + 3*c_sf_z2[3] * pow(tmp_t, 2) + 2*c_sf_z2[2] * pow(tmp_t, 1) + 1*c_sf_z2[1];
//
//        RR_foot_vel[0] = 5*c_sf_x2[5] * pow(tmp_t2, 4) + 4*c_sf_x2[4] * pow(tmp_t2, 3) + 3*c_sf_x2[3] * pow(tmp_t2, 2) + 2*c_sf_x2[2] * pow(tmp_t2, 1) + 1*c_sf_x2[1];
//        RR_foot_vel[1] = tar_init_RR_foot_vel[1];
//        RR_foot_vel[2] = 5*c_sf_z3[5] * pow(tmp_t, 4) + 4*c_sf_z3[4] * pow(tmp_t, 3) + 3*c_sf_z3[3] * pow(tmp_t, 2) + 2*c_sf_z3[2] * pow(tmp_t, 1) + 1*c_sf_z3[1];
//
//        FL_foot_vel[0] = RR_foot_vel[0];
//        FL_foot_vel[1] = tar_init_FL_foot_vel[1];
//        FL_foot_vel[2] = RR_foot_vel[2];
//
//        FR_foot_vel[0] = tar_init_FR_foot_vel[0];
//        FR_foot_vel[1] = tar_init_FR_foot_vel[1];
//        FR_foot_vel[2] = RL_foot_vel[2];
//        
//        if (ft_cnt == 3 * ft_step_cnt - 1) {
//
//            pre_com_pos(0) = pre_com_pos[0] + x_moving_speed * tf;
//            pre_com_pos(1) = pre_com_pos[1];
//            pre_com_pos(2) = h_2;
//
//            pre_com_vel[0] = x_moving_speed;
//            pre_com_vel[1] = com_vel[1];
//            pre_com_vel[2] = v_2;
//
//            pre_RL_foot_pos[0] = RL_foot_pos[0];
//            pre_RL_foot_pos[1] = RL_foot_pos[1];
//            pre_RL_foot_pos[2] = tar_init_RL_foot_pos[2] + swing_foot_height;
//
//            pre_RR_foot_pos[0] = RR_foot_pos[0];
//            pre_RR_foot_pos[1] = RR_foot_pos[1];
//            pre_RR_foot_pos[2] = tar_init_RR_foot_pos[2];
//
//            pre_FL_foot_pos[0] = FL_foot_pos[0];
//            pre_FL_foot_pos[1] = FL_foot_pos[1];
//            pre_FL_foot_pos[2] = tar_init_FL_foot_pos[2];
//
//            pre_FR_foot_pos[0] = FR_foot_pos[0];
//            pre_FR_foot_pos[1] = FR_foot_pos[1];
//            pre_FR_foot_pos[2] = tar_init_FR_foot_pos[2] + swing_foot_height;
//
//
//            if (sub_ctrl_flag == false) {
//                pre_x_moving_speed = x_moving_speed;
//                x_moving_speed = tmp_x_moving_speed;
//
//                FT_COM_SF_X_Traj_Gen();
//                ft_cnt = ft_step_cnt - 1;
//            }
//            else {
//                // =============== COM =============== //                
//                init_x[0] = pre_com_pos[0];
//                init_x[1] = x_moving_speed;
//                init_x[2] = 0;
//
//                final_x[0] = pre_com_pos[0] + x_moving_speed * (ts) / 2.0;
//                final_x[1] = 0;
//                final_x[2] = 0;
//
//                coefficient_5thPoly(init_x, final_x, ts, c_com_x2);
//
//                // =============== Swing Foot =============== //
//
//                // Left (Second)
//                init_x[0] = 0;
//                init_x[1] = 0;
//                init_x[2] = 0;
//
//                final_x[0] = 0 + x_moving_speed * ft_step_time;
//                final_x[1] = 0;
//                final_x[2] = 0;
//
//                coefficient_5thPoly(init_x, final_x, ts, c_sf_x5);
//            }
//        }
//    }
//    else if (ft_cnt < 3 * ft_step_cnt + ts_cnt) {
//        //       cout << "=========== ft_phase = 7 (FINAL) ===========" << endl;
//        ft_phase = 7;
//        _c << 0, 1, 1, 0;
//        contact_num = 2;
//        tmp_t = ft_time - 3 * ft_step_time;
//
//        com_pos[0] = c_com_x2[5] * pow(tmp_t, 5) + c_com_x2[4] * pow(tmp_t, 4) + c_com_x2[3] * pow(tmp_t, 3) + c_com_x2[2] * pow(tmp_t, 2) + c_com_x2[1] * pow(tmp_t, 1) + c_com_x2[0];
//        com_pos[1] = init_com_pos(1);
//        com_pos[2] = c_com_z4[5] * pow(tmp_t, 5) + c_com_z4[4] * pow(tmp_t, 4) + c_com_z4[3] * pow(tmp_t, 3) + c_com_z4[2] * pow(tmp_t, 2) + c_com_z4[1] * pow(tmp_t, 1) + c_com_z4[0];
//
//        com_vel[0] = 5 * c_com_x2[5] * pow(tmp_t, 4) + 4 * c_com_x2[4] * pow(tmp_t, 3) + 3 * c_com_x2[3] * pow(tmp_t, 2) + 2 * c_com_x2[2] * pow(tmp_t, 1) + 1 * c_com_x2[1];
//        com_vel[1] = pre_com_vel[1];
//        com_vel[2] = 5 * c_com_z4[5] * pow(tmp_t, 4) + 4 * c_com_z4[4] * pow(tmp_t, 3) + 3 * c_com_z4[3] * pow(tmp_t, 2) + 2 * c_com_z4[2] * pow(tmp_t, 1) + 1 * c_com_z4[1];
//
//        RL_foot_pos[0] = pre_RL_foot_pos[0] + c_sf_x5[5] * pow(tmp_t, 5) + c_sf_x5[4] * pow(tmp_t, 4) + c_sf_x5[3] * pow(tmp_t, 3) + c_sf_x5[2] * pow(tmp_t, 2) + c_sf_x5[1] * pow(tmp_t, 1) + c_sf_x5[0];
//        RL_foot_pos[1] = pre_RL_foot_pos[1];
//        RL_foot_pos[2] = tar_init_RL_foot_pos[2] + c_sf_z4[5] * pow(tmp_t, 5) + c_sf_z4[4] * pow(tmp_t, 4) + c_sf_z4[3] * pow(tmp_t, 3) + c_sf_z4[2] * pow(tmp_t, 2) + c_sf_z4[1] * pow(tmp_t, 1) + c_sf_z4[0];
//
//        RR_foot_pos[0] = pre_RR_foot_pos[0];
//        RR_foot_pos[1] = pre_RR_foot_pos[1];
//        RR_foot_pos[2] = pre_RR_foot_pos[2];
//
//        FL_foot_pos[0] = pre_FL_foot_pos[0];
//        FL_foot_pos[1] = pre_FL_foot_pos[1];
//        FL_foot_pos[2] = pre_FL_foot_pos[2];
//
//        FR_foot_pos[0] = pre_FR_foot_pos[0] + c_sf_x5[5] * pow(tmp_t, 5) + c_sf_x5[4] * pow(tmp_t, 4) + c_sf_x5[3] * pow(tmp_t, 3) + c_sf_x5[2] * pow(tmp_t, 2) + c_sf_x5[1] * pow(tmp_t, 1) + c_sf_x5[0];
//        FR_foot_pos[1] = pre_FR_foot_pos[1];
//        FR_foot_pos[2] = tar_init_FR_foot_pos[2] + c_sf_z4[5] * pow(tmp_t, 5) + c_sf_z4[4] * pow(tmp_t, 4) + c_sf_z4[3] * pow(tmp_t, 3) + c_sf_z4[2] * pow(tmp_t, 2) + c_sf_z4[1] * pow(tmp_t, 1) + c_sf_z4[0];
//
//        // ----------------------- TOMMOROW ------------------------- //
//        RL_foot_vel[0] = 5*c_sf_x5[5] * pow(tmp_t, 4) + 4*c_sf_x5[4] * pow(tmp_t, 3) + 3*c_sf_x5[3] * pow(tmp_t, 2) + 2*c_sf_x5[2] * pow(tmp_t, 1) + 1*c_sf_x5[1];
//        RL_foot_vel[1] = tar_init_RL_foot_vel[1];
//        RL_foot_vel[2] = 5*c_sf_z4[5] * pow(tmp_t, 4) + 4*c_sf_z4[4] * pow(tmp_t, 3) + 3*c_sf_z4[3] * pow(tmp_t, 2) + 2*c_sf_z4[2] * pow(tmp_t, 1) + 1*c_sf_z4[1];
//
//        RR_foot_vel[0] = tar_init_RR_foot_vel[0];
//        RR_foot_vel[1] = tar_init_RR_foot_vel[1];
//        RR_foot_vel[2] = tar_init_RR_foot_vel[2];
//
//        FL_foot_vel[0] = tar_init_FL_foot_vel[0];
//        FL_foot_vel[1] = tar_init_FL_foot_vel[1];
//        FL_foot_vel[2] = tar_init_FL_foot_vel[2];
//
//        FR_foot_vel[0] = RL_foot_vel[0];
//        FR_foot_vel[1] = tar_init_FR_foot_vel[1];
//        FR_foot_vel[2] = RL_foot_vel[2];
//        
//        if (ft_cnt == 3 * ft_step_cnt + ts_cnt - 1) {
//            pre_com_pos[0] = pre_com_pos[0] + x_moving_speed * (ts) / 2.0;
//            pre_com_pos[1] = com_pos(1);
//            pre_com_pos[2] = h_0; //com_pos(2);
//
//            pre_RL_foot_pos[0] = pre_RL_foot_pos(0) + x_moving_speed * ft_step_time; //init_RL_foot_pos[0];// + pre_x_moving_speed * step_time + x_moving_speed * step_time;//RL_foot_pos[0];//x_moving_speed * (ft_step_time);
//            pre_RL_foot_pos[1] = pre_RL_foot_pos[1];
//            pre_RL_foot_pos[2] = tar_init_RL_foot_pos[2];
//
//            pre_RR_foot_pos[0] = pre_RR_foot_pos[0];
//            pre_RR_foot_pos[1] = pre_RR_foot_pos[1];
//            pre_RR_foot_pos[2] = pre_RR_foot_pos[2];
//
//            pre_FL_foot_pos[0] = pre_FL_foot_pos[0];
//            pre_FL_foot_pos[1] = pre_FL_foot_pos[1];
//            pre_FL_foot_pos[2] = pre_FL_foot_pos[2];
//
//            pre_FR_foot_pos[0] = pre_FR_foot_pos(0) + x_moving_speed * ft_step_time; //init_FR_foot_pos[0];// + pre_x_moving_speed * step_time + x_moving_speed * step_time;//FR_foot_pos[0];
//            pre_FR_foot_pos[1] = pre_FR_foot_pos[1];
//            pre_FR_foot_pos[2] = tar_init_FR_foot_pos[2];
//        }
//    }
//
//    else {
//        //        cout << "=========== ft_phase = 8 ===========" << endl;
//        ft_phase = 8;
//        _c << 1, 1, 1, 1;
//        contact_num = 4;
//        moving_done_flag = true;
//
//        com_pos = pre_com_pos;
//        com_vel = tar_init_com_pos;
//        
//        RL_foot_pos = pre_RL_foot_pos;
//        RR_foot_pos = pre_RR_foot_pos;
//        FL_foot_pos = pre_FL_foot_pos;
//        FR_foot_pos = pre_FR_foot_pos;
//        
//        RL_foot_vel = tar_init_RL_foot_vel;
//        RR_foot_vel = tar_init_RR_foot_vel;
//        FL_foot_vel = tar_init_FL_foot_vel;
//        FR_foot_vel = tar_init_FR_foot_vel;
//    }
//
//    FT_Turning_Traj_Gen2();
//
//    base_pos = com_pos + base_offset;
//    target_pos[6] = 0;
//    ft_cnt++;
//}

void CRobot::FT_COM_X_Traj_Gen()
{
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

void CRobot::FT_COM_SF_Z_Traj_Gen()
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

    coefficient_5thPoly(init_x, final_x, (ts + tf)/2.0, c_sf_z1);

    init_x[0] = swing_foot_height;
    init_x[1] = 0;
    init_x[2] = 0;

    final_x[0] = 0;
    final_x[1] = 0;
    final_x[2] = 0;

    coefficient_5thPoly(init_x, final_x, (ts + tf)/2.0, c_sf_z2);

    init_x[0] = 0;
    init_x[1] = 0;
    init_x[2] = 0;

    final_x[0] = swing_foot_height;
    final_x[1] = 0;
    final_x[2] = 0;

    coefficient_5thPoly(init_x, final_x, (ts + 2*tf)/2.0, c_sf_z3);

    init_x[0] = swing_foot_height;
    init_x[1] = 0;
    init_x[2] = 0;

    final_x[0] = 0;
    final_x[1] = 0;
    final_x[2] = 0;

    coefficient_5thPoly(init_x, final_x, (ts + 2*tf)/2.0, c_sf_z4);

}

void CRobot::FT_Turning_Traj_Gen2(void)
{
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
    } // turn right
    else if ((base_ori(2) < -0.01) && (ft_cnt == ft_step_cnt)) {
        if (turn_mode == 0) {
            turn_start_flag = true;
            turn_mode = 3;

            target_theta = base_ori(2) * 2.0; // [rad]

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

            if (turn_cnt == ft_step_cnt - 1) {
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
        else if (turn_mode == 3) {
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

            if (turn_cnt == step_cnt - 1) {
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

    RL_foot_pos_local_offset << -turn_xr_EP, -turn_yr_EP, 0;
    RR_foot_pos_local_offset << -turn_xl_EP, -turn_yl_EP, 0;
    FL_foot_pos_local_offset << +turn_xl_EP, +turn_yl_EP, 0;
    FR_foot_pos_local_offset << +turn_xr_EP, +turn_yr_EP, 0;

    // ======= Turning trajectory generation  END ======= //    
}

double CRobot::fifth_order_poly(double c[], double t)
{
    static double y = 0;
    y = c[5]*pow(t, 5) + c[4]*pow(t, 4) + c[3]*pow(t, 3) + c[2]*pow(t, 2) + c[1]*pow(t, 1) + c[0];
    return y;
}

double CRobot::fifth_order_poly_dot(double c[], double t)
{
    static double y = 0;
    y = 5*c[5]*pow(t, 4) + 4*c[4]*pow(t, 3) + 3*c[3]*pow(t, 2) + 2*c[2]*pow(t, 1) + 1*c[1];
    return y;
}

double CRobot::fifth_order_poly_2dot(double c[], double t)
{
    static double y = 0;
    y = 20*c[5]*pow(t, 3) + 12*c[4]*pow(t, 2) + 6*c[3]*pow(t, 1) + 2*c[2];
    return y;
}
