#ifndef CROBOT_H
#define CROBOT_H

#include "rbdl/rbdl.h"
#include <rbdl/addons/urdfreader/urdfreader.h>

#include "osqp.h"

#define PI  3.14159265359
#define PI2 6.28318530718
#define GRAVITY 9.81

#define onemsec 0.001
#define onesec  1
#define tasktime 0.001
#define onesecSize 1000 // 0.001 x 1000 = 1sec

#define POSITIONCONTROLMODE 1
#define TORQUECONTROLMODE   2

#define AXIS_X     0
#define AXIS_Y     1
#define AXIS_Z     2
#define AXIS_Roll  3
#define AXIS_Pitch 4
#define AXIS_Yaw   5

#define R2D 57.295779513
#define D2R 0.0174532925

using namespace std;
using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;

typedef enum {
    MODE_SIMULATION,
    MODE_ACTUAL_ROBOT
} _MODE;

typedef enum {
    CTRLMODE_NONE,
    CTRLMODE_INITIALIZE,
    CTRLMODE_HOME_POS,
    CTRLMODE_WALK_READY,
    CTRLMODE_TROT_WALKING,
    CTRLMODE_FLYING_TROT,
    CTRLMODE_PRONK_JUMP,
    CTRLMODE_TEST,
    CTRLMODE_SLOW_WALK_HS
} _CONTROL_MODE;

typedef enum {
    NO_ACT,
    GOTO_HOME_POS,
    GOTO_WALK_READY_POS,
    NOMAL_TROT_WALKING,
    FLYING_TROT_RUNNING,
    TORQUE_OFF,
    PRONK_JUMP,
    TEST_FLAG,
    NO_ACT_WITH_CTC,
    GOTO_SLOW_WALK_POS_HS
} _COMMAND_FLAG;

typedef struct Base //coordinate of Base
{
    //* Current information
    double currentX, currentXvel, currentXacc;
    double currentY, currentYvel, currentYacc;
    double currentZ, currentZvel, currentZacc;
    double currentRoll, currentRollvel, currentRollacc;
    double currentPitch, currentPitchvel, currenPitchacc;
    double currentYaw, currentYawvel, currenYawacc;

    //* Reference information
    double refX, refXvel, refXacc;
    double refY, refYvel, refYacc;
    double refZ, refZvel, refZacc;
    double refRoll, refRollvel, refRollacc;
    double refPitch, refPitchvel, refPitchacc;
    double refYaw, refYawvel, refYawacc;

    RigidBodyDynamics::Math::VectorNd current; //* Current values
    RigidBodyDynamics::Math::VectorNd pre; //* Pre values
    RigidBodyDynamics::Math::VectorNd vel; //* vel values
    RigidBodyDynamics::Math::VectorNd prevel;
    RigidBodyDynamics::Math::VectorNd acc;
    RigidBodyDynamics::Math::VectorNd preacc;

    RigidBodyDynamics::Math::VectorNd refCoM; //* Current values
    RigidBodyDynamics::Math::VectorNd prerefCoM; //* Pre values
    RigidBodyDynamics::Math::VectorNd refvel;
    RigidBodyDynamics::Math::VectorNd prerefvel; //* Pre values
    RigidBodyDynamics::Math::VectorNd refacc;

    int ID;

} BASE;

typedef struct Joint {
    //* Current information
    double currentAngle;
    double currentVel;
    double currentAcc;
    double torque;

    //* Reference information
    double refAngle;
    double refVel;
    double refAcc;

    //* Control P I D gain
    double gain_P;
    double gain_I;
    double gain_D;
} JOINT;

typedef struct FTSennsor {
    double Fx, Fy, Fz;
    double Mx, My, Mz;
} FTS;

typedef struct EndPoint {
    //RigidBodyDynamics::Math::VectorNd pos;

    FTS ftSensor;

    RigidBodyDynamics::Math::VectorNd current; //* Current values
    RigidBodyDynamics::Math::VectorNd pre; //* Pre values
    RigidBodyDynamics::Math::VectorNd vel; //* vel values
    RigidBodyDynamics::Math::VectorNd prevel;
    RigidBodyDynamics::Math::VectorNd acc;
    RigidBodyDynamics::Math::VectorNd preacc;

    RigidBodyDynamics::Math::VectorNd refpos; //* Current values
    RigidBodyDynamics::Math::VectorNd prerefpos; //* Pre values
    RigidBodyDynamics::Math::VectorNd refvel;
    RigidBodyDynamics::Math::VectorNd prerefvel; //* Pre values
    RigidBodyDynamics::Math::VectorNd refacc;

    RigidBodyDynamics::Math::VectorNd Target;

    RigidBodyDynamics::Math::Matrix3d T_matrix;

    int ID;
} ENDPOINT;

class CRobot {
public:
    //Functions
    CRobot();
    CRobot(const CRobot& orig);
    virtual ~CRobot();
    void setRobotModel(Model* getModel); //* get Robot Model
    void getCurrentJoint(VectorNd Angle, VectorNd Vel); // get Current Joint
    void getRobotState(VectorNd BasePosOri, VectorNd BaseVel, VectorNd jointAngle, VectorNd jointVel);
    void ComputeTorqueControl();
    void FTsensorTransformation();
    VectorNd FK1(VectorNd q);
    void FK2(void);
    VectorNd IK1(VectorNd EP);
    void Init_Pos_Traj(void);
    void WalkReady_Pos_Traj(void);
    void coefficient_5thPoly(double *init_x, double *final_x, double tf, double *output);
    void Cal_Fc(void);
    void Cal_Fc2(void);
    void Flying_Trot_Traj(void);
    //    void ballistics(double flight_time, double landing_height, double take_off_speed);
    void Torque_off(void);
    void Cal_CP(void);
    void Cal_CP2(void);
    void Trot_Walking(void);
    void Trot_Walking2(void);
    void Trot_Walking3(void);
    void Trot_Walking4(void);
    void Get_gain(void);
    void Trot_Walking_Traj_First(unsigned int i);
    void Trot_Walking_Traj(unsigned int i);
    void Trot_Walking_Traj_Final(unsigned int i);
    void Foot_step_planner(VectorNd init_foot_l_2d, VectorNd init_foot_r_2d);
    void Foot_step_planner_first(VectorNd init_foot_l_2d, VectorNd init_foot_r_2d);
    void COM_X_Traj_Gen(unsigned int i);
    //    void Preview_con(void);
    //    void Preview_con_2d(void);
    void SF_Z_Traj_Gen(void);
    void SF_X_Traj_Gen(void);
    void SF_X_Traj_Gen_Final(void);
    void get_zmp(void);
    //    void Flying_Trot_Running2(void);
    void Flying_Trot_Running3(void);
    void Flying_Trot_Running_Traj(unsigned int i);
    void Flying_Trot_Running_Traj_Final(unsigned int i);
    void SF_Flying_Trot_Z_Traj_Gen(void);
    void COM_Flying_Trot_Z_Traj_Gen(void);
    void CP_Con(void);
    //    void CP_foot_step_planner(VectorNd init_foot_l_3d, VectorNd init_foot_r_3d);
    void CP_foot_traj_gen();
    void check_CP(void);
    void CP_Con_TW(void);
    void Body_Ori_Con(void);
    void Base_Ori_Con2(void);
    void One_Step_Standing_Jump(void);
    void Jump_COM_Z_Traj_Gen(double h0, double t[4]);
    void Get_act_com(void);
    void Damping_con(void);
    void Damping_con2(void);
    void COM_FT_X_Traj_Gen(void);
    void COM_FT_Z_Traj_Gen(void);
    void SF_FT_X_Traj_Gen(void);
    void SF_FT_Z_Traj_Gen(void);
    //    void Pronk_Jump(void);
    void Test_Function(void);
    void Turning_Traj_Gen(void);
    void FT_Turning_Traj_Gen(void);
    void FT_Turning_Traj_Gen2(void);
    void TW_COM_SF_X_Traj_Gen(void);
    void TW_SF_Z_Traj_Gen(void);
    void TW_Turning_Traj_Gen(void);
    void FT_COM_SF_X_Traj_Gen(void);
    void FT_COM_SF_Z_Traj_Gen(void);
    void CP_Con_FT(void);
    void check_CP_FT(void);
    void FT_Traj_Gen(void);
    void Cal_JFc(void);
    void StateUpdate(void);
    void Get_Opt_F(void);
    VectorNd Get_COM(VectorNd base, VectorNd q);
    void QP_Con_Init(void);
    void TW_COM_Traj_Gen(void);
    void TW_SF_Traj_Gen(void);
    void Get_CP(void);
    void MPC_Con_Init(void);
    void FT_COM_X_Traj_Gen(void);
    double fifth_order_poly(double c[], double t);
    double fifth_order_poly_dot(double c[], double t);
    double fifth_order_poly_2dot(double c[], double t);
    MatrixNd Kron(MatrixNd AA, MatrixNd BB);
    MatrixNd pow_mat(MatrixNd mat, int x);
    void MPC_process(void);

    //    void Get_CP(void);

    //VectorNd FK1(VectorNd q);

    enum Fc_Phase {
        INIT_Fc = 0,
        STOP,
        STANCE_RLFR,
        STANCE_RRFL,
        JUMP_Fc,
        ZERO
    };

    enum Contact_Phase {
        Four_feet = 0,
        RLFR,
        RRFL,
        Zero
    };

    enum Trot_Phase {
        INIT_FORWARD = 0, // 0
        INIT_STANCE_RRFL, // 1
        INIT_STANCE_FOUR_AFTER_RRFL, // 2
        TROT_STANCE_RLFR, // 3
        TROT_STANCE_RLFR2, // 4
        TROT_STANCE_RRFL, // 5
        TROT_STANCE_FOUR_AFTER_RLFR, // 6
        TROT_STANCE_FOUR_AFTER_RLFR2, // 7
        TROT_STANCE_FOUR_AFTER_RRFL, // 8
        FINAL_STANCE_FOUR, // 9
        FINAL_STANCE_RLFR, // 10
        FINAL_STANCE_RRFL // 11
    };

    enum Fc_Phase FC_PHASE;
    enum Contact_Phase C_PHASE;
    enum Trot_Phase TROT_PHASE;

    //Variables
    BASE base; //* coordinate of Body
    JOINT* joint; //* joints of the robot
    ENDPOINT FR, FL, RR, RL, front_body;
    int nDOF; //* number of DOFs of a robot

    int ControlMode;
    int CommandFlag;
    int sub_ctrl_flag;
    int Mode;

    RigidBodyDynamics::Model* m_pModel; //* URDF Model
    //    RigidBodyDynamics::Math::VectorNd tar_RobotState;
    RigidBodyDynamics::Math::VectorNd RobotState;
    RigidBodyDynamics::Math::VectorNd RobotStatedot;
    RigidBodyDynamics::Math::VectorNd RobotState2dot;
    RigidBodyDynamics::Math::VectorNd BasePosOri;
    RigidBodyDynamics::Math::VectorNd BaseVel;
    RigidBodyDynamics::Math::VectorNd JointAngle;
    RigidBodyDynamics::Math::VectorNd JointVel;

    VectorNd pd_con_joint = VectorNd::Zero(19);
    VectorNd pd_con_task = VectorNd::Zero(19);

    MatrixNd M_term = MatrixNd::Zero(19, 19);
    VectorNd hatNonLinearEffects = VectorNd::Zero(19);
    VectorNd G_term = VectorNd::Zero(19);
    VectorNd C_term = VectorNd::Zero(19);
    VectorNd CTC_Torque = VectorNd::Zero(19);
    VectorNd JFc = VectorNd::Zero(19);
    VectorNd tmp_CTC_Torque = VectorNd::Zero(19);

    //    VectorNd EP_RL = Vector3d(0, 0, 0);
    //    VectorNd EP_RR = Vector3d(0, 0, 0);
    //    VectorNd EP_FL = Vector3d(0, 0, 0);
    //    VectorNd EP_FR = Vector3d(0, 0, 0);

    double L3_x = 0; //0.025516;
    double L3_y = 0; //0.0;
    double L3_z = 0.309; //0.304515;

    VectorNd EP_OFFSET_RL = Vector3d(-L3_x, L3_y, -L3_z);
    VectorNd EP_OFFSET_RR = Vector3d(-L3_x, L3_y, -L3_z);
    VectorNd EP_OFFSET_FL = Vector3d(-L3_x, L3_y, -L3_z);
    VectorNd EP_OFFSET_FR = Vector3d(-L3_x, L3_y, -L3_z);
    VectorNd Originbase = Vector3d(0, 0, 0);

    VectorNd actual_EP = VectorNd::Zero(12);
    VectorNd actual_EP_vel = VectorNd::Zero(12);
    VectorNd actual_EP_acc = VectorNd::Zero(12);

    VectorNd abs_pos = VectorNd::Zero(13);
    VectorNd inc_pos = VectorNd::Zero(13);
    VectorNd inc_offset_pos = VectorNd::Zero(13);
    VectorNd actual_pos = VectorNd::Zero(13);
    VectorNd actual_vel = VectorNd::Zero(13);
    VectorNd lpf_actual_vel = VectorNd::Zero(13);
    VectorNd actual_acc = VectorNd::Zero(13);

    VectorNd pre_actual_pos = VectorNd::Zero(13);
    VectorNd init_pos = VectorNd::Zero(13);

    VectorNd target_pos = VectorNd::Zero(13);
    VectorNd target_pos_with_con = VectorNd::Zero(13);
    VectorNd target_pos_offset = VectorNd::Zero(13);
    VectorNd target_vel = VectorNd::Zero(13);
    VectorNd target_acc = VectorNd::Zero(13);
    VectorNd pre_target_pos = VectorNd::Zero(13);
    VectorNd pre_target_vel = VectorNd::Zero(13);

    //    VectorNd target_q2 = VectorNd::Zero(13);
    VectorNd base_pos_ori = VectorNd::Zero(6);


    VectorNd target_tor = VectorNd::Zero(13);

    MatrixNd J_RL = MatrixNd::Zero(3, 19);
    MatrixNd J_RR = MatrixNd::Zero(3, 19);
    MatrixNd J_FL = MatrixNd::Zero(3, 19);
    MatrixNd J_FR = MatrixNd::Zero(3, 19);

    MatrixNd J_RL2 = MatrixNd::Zero(3, 3);
    MatrixNd J_RR2 = MatrixNd::Zero(3, 3);
    MatrixNd J_FL2 = MatrixNd::Zero(3, 3);
    MatrixNd J_FR2 = MatrixNd::Zero(3, 3);

    MatrixNd J_FRONT_BODY = MatrixNd::Zero(6, 19);
    MatrixNd J_BASE = MatrixNd::Zero(6, 19);

    MatrixNd J_A = MatrixNd::Zero(19, 19);
    MatrixNd S_mat = MatrixNd::Zero(19, 19);

    VectorNd x_dot = VectorNd::Zero(19);
    VectorNd ddqZero = VectorNd::Zero(19);

    VectorNd RL_dJdQ = Vector3d(0, 0, 0);
    VectorNd RR_dJdQ = Vector3d(0, 0, 0);
    VectorNd FL_dJdQ = Vector3d(0, 0, 0);
    VectorNd FR_dJdQ = Vector3d(0, 0, 0);
    VectorNd FRONT_BODY_dJdQ = VectorNd::Zero(6);
    VectorNd base_dJdQ = VectorNd::Zero(6);
    VectorNd dJdQ = VectorNd::Zero(19);

    VectorNd Fc = VectorNd::Zero(19);
    VectorNd Fc2 = VectorNd::Zero(19);
    VectorNd x_2dot_cp = VectorNd::Zero(19);


    VectorNd Kp_EP = VectorNd::Zero(12); //(100,100,100,100,100,100,100,100,100,100,100,100);
    VectorNd Kd_EP = VectorNd::Zero(12); //(1,1,1,1,1,1,1,1,1,1,1,1);
    VectorNd target_EP = VectorNd::Zero(12);
    VectorNd tmp_target_EP = VectorNd::Zero(12);
    VectorNd target_EP_offset = VectorNd::Zero(12);
    VectorNd pre_target_EP = VectorNd::Zero(12);
    VectorNd target_EP_vel = VectorNd::Zero(12);
    VectorNd target_EP_acc = VectorNd::Zero(12);
    VectorNd goal_EP = VectorNd::Zero(12); //(0,0.218,-0.45,0,-0.218,-0.45,0.7,0.218,-0.45,0.7,-0.218,-0.45);
    VectorNd init_EP = VectorNd::Zero(12);
    VectorNd init_goal_EP = VectorNd::Zero(12);
    VectorNd trot_goal_EP = VectorNd::Zero(12);
    double Fc_RL_z = 0, Fc_RR_z = 0, Fc_FL_z = 0, Fc_FR_z = 0;
    double Fc_RL_x = 0, Fc_RR_x = 0, Fc_FL_x = 0, Fc_FR_x = 0;
    double Fc_RL_y = 0, Fc_RR_y = 0, Fc_FL_y = 0, Fc_FR_y = 0;

    VectorNd angle_err = VectorNd::Zero(13);
    VectorNd EP_err = VectorNd::Zero(12);
    VectorNd EP_vel_err = VectorNd::Zero(12);

    VectorNd init_target_pos = VectorNd::Zero(13);

    VectorNd RL_base2hip_pos = VectorNd::Zero(3);
    VectorNd RR_base2hip_pos = VectorNd::Zero(3);
    VectorNd FL_base2hip_pos = VectorNd::Zero(3);
    VectorNd FR_base2hip_pos = VectorNd::Zero(3);

    VectorNd tar_init_RL_foot_pos = VectorNd::Zero(3);
    VectorNd tar_init_RR_foot_pos = VectorNd::Zero(3);
    VectorNd tar_init_FL_foot_pos = VectorNd::Zero(3);
    VectorNd tar_init_FR_foot_pos = VectorNd::Zero(3);

    VectorNd base2hip_pos = VectorNd::Zero(12);
    VectorNd init_RL_foot_pos_local = VectorNd::Zero(3);
    VectorNd init_RR_foot_pos_local = VectorNd::Zero(3);
    VectorNd init_FL_foot_pos_local = VectorNd::Zero(3);
    VectorNd init_FR_foot_pos_local = VectorNd::Zero(3);

    VectorNd tar_RL_foot_pos_local = VectorNd::Zero(3);
    VectorNd tar_RR_foot_pos_local = VectorNd::Zero(3);
    VectorNd tar_FL_foot_pos_local = VectorNd::Zero(3);
    VectorNd tar_FR_foot_pos_local = VectorNd::Zero(3);

    VectorNd tar_RL_foot_vel_local = VectorNd::Zero(3);
    VectorNd tar_RR_foot_vel_local = VectorNd::Zero(3);
    VectorNd tar_FL_foot_vel_local = VectorNd::Zero(3);
    VectorNd tar_FR_foot_vel_local = VectorNd::Zero(3);

    VectorNd act_RL_q_dot = VectorNd::Zero(3);
    VectorNd act_RR_q_dot = VectorNd::Zero(3);
    VectorNd act_FL_q_dot = VectorNd::Zero(3);
    VectorNd act_FR_q_dot = VectorNd::Zero(3);

    VectorNd tar_RL_q_dot = VectorNd::Zero(3);
    VectorNd tar_RR_q_dot = VectorNd::Zero(3);
    VectorNd tar_FL_q_dot = VectorNd::Zero(3);
    VectorNd tar_FR_q_dot = VectorNd::Zero(3);



    VectorNd act_RL_foot_pos_local = VectorNd::Zero(3);
    VectorNd act_RR_foot_pos_local = VectorNd::Zero(3);
    VectorNd act_FL_foot_pos_local = VectorNd::Zero(3);
    VectorNd act_FR_foot_pos_local = VectorNd::Zero(3);

    VectorNd act_RL_foot_pos = VectorNd::Zero(3);
    VectorNd act_RR_foot_pos = VectorNd::Zero(3);
    VectorNd act_FL_foot_pos = VectorNd::Zero(3);
    VectorNd act_FR_foot_pos = VectorNd::Zero(3);

    //    VectorNd goal_RL_foot_pos = VectorNd::Zero(3);
    //    VectorNd goal_RR_foot_pos = VectorNd::Zero(3);
    //    VectorNd goal_FL_foot_pos = VectorNd::Zero(3);
    //    VectorNd goal_FR_foot_pos = VectorNd::Zero(3);


    VectorNd des_x_2dot = VectorNd::Zero(3);
    VectorNd des_w_dot = VectorNd::Zero(3);

    double _m;
    MatrixNd _I_g = MatrixNd::Zero(3, 3);
    MatrixNd _A = MatrixNd::Zero(6, 12);
    MatrixNd p_com_oross_pro = MatrixNd::Zero(3, 12);
    VectorNd _g = VectorNd::Zero(3);
    VectorNd _b = VectorNd::Zero(6);
    MatrixNd _C = MatrixNd::Identity(12, 12);
    VectorNd _d_u = VectorNd::Zero(12);
    VectorNd _d_l = VectorNd::Zero(12);
    VectorNd _c = VectorNd::Zero(4);
    VectorNd c_vec = VectorNd::Zero(12);
    int contact_num;

    // weight
    MatrixNd _S = MatrixNd::Identity(6, 6);
    MatrixNd _W = MatrixNd::Identity(12, 12);
    double _alpha = 0.001; //0.000001; // 0.00001

    MatrixNd _P = MatrixNd::Zero(12, 12);
    VectorNd _q = VectorNd::Zero(12);


    //    double fx_max, fx_min;
    //    double fy_max, fy_min;
    //    double fz_max, fz_min;

    double fz_RL_max = 0;
    double fz_RL_min = 0;
    double fz_RR_max = 0;
    double fz_RR_min = 0;
    double fz_FL_max = 0;
    double fz_FL_min = 0;
    double fz_FR_max = 0;
    double fz_FR_min = 0;


    // =============== Flag Define ================ //
    bool moving_done_flag;
    bool home_init_flag = true;
    bool trot_init_flag = true;
    bool forward_init_flag = true;
    bool up_down_init_flag = true; //up down flag added by HSKIM
    bool raise_leg_init_flag = true; //raise leg flag added by HSKIM
    bool jump_init_flag = true; //jump flag added by HSKIM
    bool target_init_flag = true;
    bool flying_trot_final_flag;
    bool flying_trot_init_flag;
    bool walk_ready_moving_done_flag;
    bool CP_moving_flag;
    bool CP_init_flag;
    bool CP_moving_start_flag;
    int CP_con_onoff_flag;
    bool get_CP_flag, CP_check_flag;
    bool get_cp_done_flag;
    bool CP_move_done_flag;
    int Base_Ori_Con_onoff_flag;
    bool turn_start_flag;
    bool first_jump_flag;
    bool T_RL_on_flag, T_RR_on_flag, T_FL_on_flag, T_FR_on_flag;
    bool ft_ready_flag, ft_finish_flag;
    bool cp_con_on_flag;


    //    unsigned int ctc_cnt = 0, ctc_cnt2 = 0;
    unsigned int wr_cnt = 0;
    double dt = 0.001;

    VectorNd Kp_q = VectorNd::Zero(13);
    VectorNd Kd_q = VectorNd::Zero(13);
    VectorNd init_Kp_q = VectorNd::Zero(13);
    VectorNd init_Kd_q = VectorNd::Zero(13);
    VectorNd goal_Kp_q = VectorNd::Zero(13);
    VectorNd goal_Kd_q = VectorNd::Zero(13);
    VectorNd Kp_f = VectorNd::Zero(12);
    VectorNd Kd_f = VectorNd::Zero(12);
    VectorNd FT_Kp_q = VectorNd::Zero(13);
    VectorNd FT_Kd_q = VectorNd::Zero(13);
    VectorNd Kp_t = VectorNd::Zero(12);
    VectorNd Kd_t = VectorNd::Zero(12);

    int tmp_cnt = 0, tmp_cnt2 = 0;


    // =============== Time ================ //
    // 3Hz
    //    double dsp_time = 0.25, fsp_time = 0.1;
    //    double step_time = dsp_time + fsp_time;
    //    int dsp_cnt = 250, fsp_cnt = 100;
    //    int step_cnt = dsp_cnt + fsp_cnt;

    // 4Hz
    //    double dsp_time = 0.20, fsp_time = 0.05;
    //    double step_time = dsp_time + fsp_time;
    //    int dsp_cnt = 200, fsp_cnt = 50;
    //    int step_cnt = dsp_cnt + fsp_cnt;

    double dsp_time = 0.20, fsp_time = 0.07; //0.07
    double step_time = dsp_time + fsp_time;
    int dsp_cnt = 200, fsp_cnt = 70; //70;
    int step_cnt = dsp_cnt + fsp_cnt;

    //    int stride_cnt = step_cnt * 2;

    // =============== Trajectory ================ //
    double tmp_time = 0, tmp_time2 = 0;
    double z_f1[6], z_f2[6], z_f3[6], z_s1[6], z_s2[6], z_s3[6], z_final1[6];
    double x_init1[6], x_init2[6], x_final1[6], x_final2[6], x_trot_dsp1[6], x_trot_dsp2[6], x_trot_fsp1[6], x_trot_fsp2[6];
    double temp_t1 = 0, temp_t2 = 0;
    double to_height = 0;
    double c_xl1[6], c_xl2[6], c_xl3[6], c_xl4[6], c_xr1[6], c_xr2[6], c_xr3[6], c_xr4[6], c_xl_f[6], c_xr_f[6];
    double xl_f[7], xr_f[7], xl1[7], xl2[7], xl3[7], xl4[7], xr1[7], xr2[7], xr3[7], xr4[7];
    double c_z1[6], c_z2[6];
    double init_x[3] = {0, 0, 0};
    double final_x[3] = {0, 0, 0};
    double _t = 0;
    double _out[6] = {0, 0, 0, 0, 0, 0};

    double x_moving_speed, y_moving_speed, pre_x_moving_speed, pre_y_moving_speed; // [m/s]
    VectorNd act_com_pos = VectorNd::Zero(3);
    VectorNd pre_act_com_pos = VectorNd::Zero(3);

    VectorNd tmp_com_pos = VectorNd::Zero(3);
    VectorNd tmp_pre_com_pos = VectorNd::Zero(3);
    VectorNd tmp_com_vel = VectorNd::Zero(3);

    VectorNd tmp_base_ori = VectorNd::Zero(3);
    double tmp_x_moving_speed, tmp_y_moving_speed;
    VectorNd act_com_vel = VectorNd::Zero(3);
    VectorNd act_com_acc = VectorNd::Zero(3);
    VectorNd tmp_act_com_acc = VectorNd::Zero(3);
    VectorNd act_com_vel2 = VectorNd::Zero(2);
    VectorNd act_com_pos2 = VectorNd::Zero(2);
    VectorNd lpf_act_com_vel = VectorNd::Zero(3);
    double x_step = 0; //step_time*moving_speed/2.0;
    double x_fsp = 0; //fsp_time*moving_speed/2.0;
    double pre_foot_l[3]; // = [ -x_step - x_fsp,-moving_speed,0];
    double pre_foot_r[3]; // = [  x_step - x_fsp,-moving_speed,0];

    VectorNd R = VectorNd::Zero(6);
    VectorNd P = VectorNd::Zero(6);
    MatrixNd A = MatrixNd::Zero(6, 6);

    VectorNd tmp_data1 = VectorNd::Zero(80);
    VectorNd tmp_data2 = VectorNd::Zero(80);
    VectorNd computed_tor = VectorNd::Zero(13);
    VectorNd Fc_vsd = VectorNd::Zero(12);
    VectorNd Kp_vsd = VectorNd::Zero(12);
    VectorNd Kd_vsd = VectorNd::Zero(12);

    Quaternion QQ;

    // =============== IMU ================ //
    double IMURoll, IMUPitch, IMUYaw;
    double IMURoll_dot, IMUPitch_dot, IMUYaw_dot;
    double init_IMUYaw;

    // =============== CP ================ //
    double COM_Height, natural_freq;
    double CP_y, COM_y, COM_y_dot, lpf_COM_y, lpf_COM_y_dot, init_CP_y;
    double CP_x, COM_x, COM_x_dot, lpf_COM_x, lpf_COM_x_dot;
    double F_fd_r, F_fd_l, F_fd_f, F_fd_b;

    double w1, w2;

    double foot_height;
    double com_height;
    double z1[6], z2[6], z3[6];
    double jump_z1[6], jump_z2[6], jump_z3[6], jump_z4[6], jump_z5[6];
    double x1[6], x2[6], x3[6], x4[6], x5[6];
    double y1[6], y2[6], y3[6], y4[6], y5[6];
    double walk_time, dsp_t1, dsp_t2;

    // ============== Preview ============= //
    //    unsigned int preview_cnt = 1000;
    //
    //    double Gi = 0;
    //    VectorNd Gx = VectorNd::Zero(3);
    //    VectorNd Gp = VectorNd::Zero(1000);
    //
    //    MatrixNd AA = MatrixNd::Zero(3, 3);
    //    VectorNd BB = VectorNd::Zero(3);
    //    VectorNd CC = VectorNd::Zero(3);
    //
    //    double pv_Gp[1000], pv_Gx[3], pv_Gi[1];

    //    VectorNd com_x = VectorNd::Zero(3);   // x, x_dot, x_2dot
    VectorNd com_pos = VectorNd::Zero(3); // x,y,z
    VectorNd pre_com_pos = VectorNd::Zero(3); // x,y,z
    VectorNd pre_com_vel = VectorNd::Zero(3);
    VectorNd com_vel = VectorNd::Zero(3); // x,y,z
    //    VectorNd com_ori = VectorNd::Zero(3); // roll,pitch,yaw
    //    VectorNd pre_com_ori = VectorNd::Zero(3); // roll,pitch,yaw
    //    VectorNd com_ori_dot = VectorNd::Zero(3); // roll,pitch,yaw
    //    VectorNd act_com_ori = VectorNd::Zero(3); // roll,pitch,yaw
    //    VectorNd act_com_ori_dot = VectorNd::Zero(3); // roll,pitch,yaw
    VectorNd base_ori_quat = VectorNd::Zero(4); // roll,pitch,yaw
    VectorNd old_com_pos = VectorNd::Zero(3); // x,y,z
    VectorNd old_com_vel = VectorNd::Zero(3); // x,y,z
    VectorNd tar_init_com_vel = VectorNd::Zero(3); // x,y,z
    VectorNd tar_init_com_acc = VectorNd::Zero(3); // x,y,z

    VectorNd base_pos = VectorNd::Zero(3); // x,y,z
    VectorNd tmp_act_base_pos = VectorNd::Zero(3);
    VectorNd act_base_pos = VectorNd::Zero(3);
    VectorNd base_vel = VectorNd::Zero(3);
    VectorNd act_base_vel = VectorNd::Zero(3);
    VectorNd tmp_act_base_vel = VectorNd::Zero(3);
    VectorNd pre_act_com_vel = VectorNd::Zero(3);
    VectorNd base_ori = VectorNd::Zero(3);
    VectorNd pre_base_ori = VectorNd::Zero(3);
    VectorNd act_base_ori = VectorNd::Zero(3);
    VectorNd base_ori_dot = VectorNd::Zero(3);
    VectorNd act_base_ori_dot = VectorNd::Zero(3);
    VectorNd act_base_ori_dot_w = VectorNd::Zero(3);
    VectorNd pre_act_base_ori_dot = VectorNd::Zero(3);
    VectorNd tmp_act_base_ori_dot = VectorNd::Zero(3);



    MatrixNd lpf_base_alpha = MatrixNd::Identity(3, 3);

    //    VectorNd pre_base_ori = VectorNd::Zero(3);
    //    VectorNd actual_com_pos = VectorNd::Zero(3); // x,y,z
    //    VectorNd actual_com_vel = VectorNd::Zero(3); // x,y,z
    //    VectorNd pre_com_pos = VectorNd::Zero(3); // x,y,z

    VectorNd local_RL_foot_pos = VectorNd::Zero(3);
    VectorNd local_RR_foot_pos = VectorNd::Zero(3);
    VectorNd local_FL_foot_pos = VectorNd::Zero(3);
    VectorNd local_FR_foot_pos = VectorNd::Zero(3);

    MatrixNd foot_l_2d = MatrixNd::Zero(5, 2);
    MatrixNd foot_r_2d = MatrixNd::Zero(5, 2);
    MatrixNd pre_foot_l_2d = MatrixNd::Zero(5, 2);
    MatrixNd pre_foot_r_2d = MatrixNd::Zero(5, 2);
    VectorNd final_foot_l_2d = VectorNd::Zero(2);
    VectorNd final_foot_r_2d = VectorNd::Zero(2);
    //    MatrixNd zmp_ref_array = MatrixNd::Zero(preview_cnt, 2);
    VectorNd XX = VectorNd::Zero(3);
    VectorNd X_new = VectorNd::Zero(3);
    VectorNd Y_new = VectorNd::Zero(3);
    VectorNd zmp_ref_old = VectorNd::Zero(2, 1);

    VectorNd cp_RL_foot_pos = VectorNd::Zero(3);
    VectorNd cp_RR_foot_pos = VectorNd::Zero(3);
    VectorNd cp_FL_foot_pos = VectorNd::Zero(3);
    VectorNd cp_FR_foot_pos = VectorNd::Zero(3);

    VectorNd init_cp_RL_foot_pos = VectorNd::Zero(3);
    VectorNd init_cp_RR_foot_pos = VectorNd::Zero(3);
    VectorNd init_cp_FL_foot_pos = VectorNd::Zero(3);
    VectorNd init_cp_FR_foot_pos = VectorNd::Zero(3);

    VectorNd tmp_cp_RL_foot_pos = VectorNd::Zero(3);
    VectorNd tmp_cp_RR_foot_pos = VectorNd::Zero(3);
    VectorNd tmp_cp_FL_foot_pos = VectorNd::Zero(3);
    VectorNd tmp_cp_FR_foot_pos = VectorNd::Zero(3);

    //    MatrixNd com_x_array = MatrixNd::Zero(preview_cnt, 3);
    //    VectorNd zmp_x_array = VectorNd::Zero(preview_cnt);
    double zmp_x, zmp_y, lpf_zmp_x, old_lpf_zmp_x;
    //    VectorNd sum_e = VectorNd::Zero(2, 1);
    //    VectorNd pre_err = VectorNd::Zero(2, 1);
    //    VectorNd pre_sum_p = VectorNd::Zero(2, 1);
    //    VectorNd zmp_ref = VectorNd::Zero(2, 1);

    VectorNd init_com_pos = VectorNd::Zero(3);
    VectorNd init_com_vel = VectorNd::Zero(3);
    VectorNd goal_com_pos = VectorNd::Zero(3);

    VectorNd init_base_pos = VectorNd::Zero(3);
    VectorNd init_base_ori = VectorNd::Zero(3);


    VectorNd init_RL_foot_pos = VectorNd::Zero(3);
    VectorNd init_RR_foot_pos = VectorNd::Zero(3);
    VectorNd init_FL_foot_pos = VectorNd::Zero(3);
    VectorNd init_FR_foot_pos = VectorNd::Zero(3);

    VectorNd tar_init_RL_foot_vel = VectorNd::Zero(3);
    VectorNd tar_init_RR_foot_vel = VectorNd::Zero(3);
    VectorNd tar_init_FL_foot_vel = VectorNd::Zero(3);
    VectorNd tar_init_FR_foot_vel = VectorNd::Zero(3);

    VectorNd RL_foot_pos = VectorNd::Zero(3);
    VectorNd RR_foot_pos = VectorNd::Zero(3);
    VectorNd FL_foot_pos = VectorNd::Zero(3);
    VectorNd FR_foot_pos = VectorNd::Zero(3);

    VectorNd RL_cp_foot_pos = VectorNd::Zero(3);
    VectorNd RR_cp_foot_pos = VectorNd::Zero(3);
    VectorNd FL_cp_foot_pos = VectorNd::Zero(3);
    VectorNd FR_cp_foot_pos = VectorNd::Zero(3);

    VectorNd RL_cp_foot_vel = VectorNd::Zero(3);
    VectorNd RR_cp_foot_vel = VectorNd::Zero(3);
    VectorNd FL_cp_foot_vel = VectorNd::Zero(3);
    VectorNd FR_cp_foot_vel = VectorNd::Zero(3);

    VectorNd RL_foot_pos_local_offset = VectorNd::Zero(3);
    VectorNd RR_foot_pos_local_offset = VectorNd::Zero(3);
    VectorNd FL_foot_pos_local_offset = VectorNd::Zero(3);
    VectorNd FR_foot_pos_local_offset = VectorNd::Zero(3);

    //    VectorNd RL_foot_pos_local_offset2 = VectorNd::Zero(3);
    //    VectorNd RR_foot_pos_local_offset2 = VectorNd::Zero(3);
    //    VectorNd FL_foot_pos_local_offset2 = VectorNd::Zero(3);
    //    VectorNd FR_foot_pos_local_offset2 = VectorNd::Zero(3);

    VectorNd pre_RL_foot_pos = VectorNd::Zero(3);
    VectorNd pre_RR_foot_pos = VectorNd::Zero(3);
    VectorNd pre_FL_foot_pos = VectorNd::Zero(3);
    VectorNd pre_FR_foot_pos = VectorNd::Zero(3);

    VectorNd RL_foot_vel = VectorNd::Zero(3);
    VectorNd RR_foot_vel = VectorNd::Zero(3);
    VectorNd FL_foot_vel = VectorNd::Zero(3);
    VectorNd FR_foot_vel = VectorNd::Zero(3);

    VectorNd act_RL_foot_vel = VectorNd::Zero(3);
    VectorNd act_RR_foot_vel = VectorNd::Zero(3);
    VectorNd act_FL_foot_vel = VectorNd::Zero(3);
    VectorNd act_FR_foot_vel = VectorNd::Zero(3);


    bool stop_flag = false;
    bool traj_stop_flag = true;
    int step_num = 0;

    double cp_foot_pos_y, tmp_cp_foot_pos_y, target_cp_foot_pos_y;

    // =============== flying trot parameters ================= //
    double ts, tf;
    int ts_cnt, tf_cnt;
    double h_0, h_1, h_2, h_3, v_0, v_1, v_2, v_3, a_0, a_1, a_2, a_3;
    double swing_foot_height;
    double c_com_z1[6], c_com_z2[6], c_com_z3[6], c_com_z4[6];
    double c_com_x1[6], c_com_x2[6], c_com_x3[6], c_com_x4[6], c_com_x5[6];
    double c_com_y1[6], c_com_y2[6], c_com_y3[6], c_com_y4[6], c_com_y5[6];
    double c_state_x1[6];
    double c_sf_z1[6], c_sf_z2[6], c_sf_z3[6], c_sf_z4[6];
    double c_sf_x1[6], c_sf_x2[6], c_sf_x3[6], c_sf_x4[6], c_sf_x5[6];
    double c_sf_y1[6], c_sf_y2[6], c_sf_y3[6], c_sf_y4[6], c_sf_y5[6];


    // ============================ //

    MatrixNd C_I_roll = MatrixNd::Zero(3, 3);
    MatrixNd C_I_pitch = MatrixNd::Zero(3, 3);

    MatrixNd RL_C_I_HP = MatrixNd::Zero(3, 3);
    MatrixNd RL_C_HP_HR = MatrixNd::Zero(3, 3);
    MatrixNd RL_C_HR_KN = MatrixNd::Zero(3, 3);
    MatrixNd RL_C_KN_TIP = MatrixNd::Zero(3, 3);

    MatrixNd RR_C_I_HP = MatrixNd::Zero(3, 3);
    MatrixNd RR_C_HP_HR = MatrixNd::Zero(3, 3);
    MatrixNd RR_C_HR_KN = MatrixNd::Zero(3, 3);
    MatrixNd RR_C_KN_TIP = MatrixNd::Zero(3, 3);

    MatrixNd FL_C_I_HP = MatrixNd::Zero(3, 3);
    MatrixNd FL_C_HP_HR = MatrixNd::Zero(3, 3);
    MatrixNd FL_C_HR_KN = MatrixNd::Zero(3, 3);
    MatrixNd FL_C_KN_TIP = MatrixNd::Zero(3, 3);

    MatrixNd FR_C_I_HP = MatrixNd::Zero(3, 3);
    MatrixNd FR_C_HP_HR = MatrixNd::Zero(3, 3);
    MatrixNd FR_C_HR_KN = MatrixNd::Zero(3, 3);
    MatrixNd FR_C_KN_TIP = MatrixNd::Zero(3, 3);

    MatrixNd Rot_x = MatrixNd::Zero(3, 3);
    MatrixNd Rot_y = MatrixNd::Zero(3, 3);
    MatrixNd Rot_z = MatrixNd::Zero(3, 3);

    MatrixNd Kp_x = MatrixNd::Zero(3, 3);
    MatrixNd Kd_x = MatrixNd::Zero(3, 3);
    MatrixNd Kp_w = MatrixNd::Zero(3, 3);
    MatrixNd Kd_w = MatrixNd::Zero(3, 3);

    VectorNd p_base2body_com = VectorNd::Zero(4, 1);
    VectorNd p_RL_hp_com = VectorNd::Zero(4, 1);
    VectorNd p_RL_thigh_com = VectorNd::Zero(4, 1);
    VectorNd p_RL_calf_com = VectorNd::Zero(4, 1);
    VectorNd p_RR_hp_com = VectorNd::Zero(4, 1);
    VectorNd p_RR_thigh_com = VectorNd::Zero(4, 1);
    VectorNd p_RR_calf_com = VectorNd::Zero(4, 1);
    VectorNd p_FL_hp_com = VectorNd::Zero(4, 1);
    VectorNd p_FL_thigh_com = VectorNd::Zero(4, 1);
    VectorNd p_FL_calf_com = VectorNd::Zero(4, 1);
    VectorNd p_FR_hp_com = VectorNd::Zero(4, 1);
    VectorNd p_FR_thigh_com = VectorNd::Zero(4, 1);
    VectorNd p_FR_calf_com = VectorNd::Zero(4, 1);

    MatrixNd R_w2base_R = MatrixNd::Zero(4, 4);
    MatrixNd R_w2base_P = MatrixNd::Zero(4, 4);
    MatrixNd R_w2base_Y = MatrixNd::Zero(4, 4);
    MatrixNd R_w2base = MatrixNd::Zero(4, 4);
    MatrixNd T_w2base = MatrixNd::Zero(4, 4);
    MatrixNd TR_RL_base2hp = MatrixNd::Zero(4, 4);
    MatrixNd TR_RL_hp2thigh = MatrixNd::Zero(4, 4);
    MatrixNd TR_RL_thigh2calf = MatrixNd::Zero(4, 4);
    MatrixNd TR_RR_base2hp = MatrixNd::Zero(4, 4);
    MatrixNd TR_RR_hp2thigh = MatrixNd::Zero(4, 4);
    MatrixNd TR_RR_thigh2calf = MatrixNd::Zero(4, 4);
    MatrixNd TR_FL_base2hp = MatrixNd::Zero(4, 4);
    MatrixNd TR_FL_hp2thigh = MatrixNd::Zero(4, 4);
    MatrixNd TR_FL_thigh2calf = MatrixNd::Zero(4, 4);
    MatrixNd TR_FR_base2hp = MatrixNd::Zero(4, 4);
    MatrixNd TR_FR_hp2thigh = MatrixNd::Zero(4, 4);
    MatrixNd TR_FR_thigh2calf = MatrixNd::Zero(4, 4);

    VectorNd p_RL_base2hp_com = VectorNd::Zero(4, 1);
    VectorNd p_RL_base2thigh_com = VectorNd::Zero(4, 1);
    VectorNd p_RL_base2calf_com = VectorNd::Zero(4, 1);
    VectorNd p_RL_com = VectorNd::Zero(4, 1);
    VectorNd p_RR_base2hp_com = VectorNd::Zero(4, 1);
    VectorNd p_RR_base2thigh_com = VectorNd::Zero(4, 1);
    VectorNd p_RR_base2calf_com = VectorNd::Zero(4, 1);
    VectorNd p_RR_com = VectorNd::Zero(4, 1);
    VectorNd p_FL_base2hp_com = VectorNd::Zero(4, 1);
    VectorNd p_FL_base2thigh_com = VectorNd::Zero(4, 1);
    VectorNd p_FL_base2calf_com = VectorNd::Zero(4, 1);
    VectorNd p_FL_com = VectorNd::Zero(4, 1);
    VectorNd p_FR_base2hp_com = VectorNd::Zero(4, 1);
    VectorNd p_FR_base2thigh_com = VectorNd::Zero(4, 1);
    VectorNd p_FR_base2calf_com = VectorNd::Zero(4, 1);
    VectorNd p_FR_com = VectorNd::Zero(4, 1);

    VectorNd p_robot_com_from_base = VectorNd::Zero(4, 1);
    VectorNd p_robot_com_from_w = VectorNd::Zero(4, 1);
    VectorNd p_robot_com = VectorNd::Zero(3, 1);
    VectorNd base_offset = VectorNd::Zero(3, 1);
    VectorNd tar_init_com_pos = VectorNd::Zero(3, 1);

    int CP_PHASE;
    double tmp_t, tmp_t2, tmp_t3;
    int CP_move_step;
    double cp_y_limit;
    int moving_cnt;
    int JUMP_PHASE;

    double tar_Fc_RL, tar_Fc_RR, tar_Fc_FL, tar_Fc_FR;

    double kp_roll, kd_roll;
    double target_kp_roll, target_kd_roll;

    double BOC_Kp_roll, BOC_Ki_roll, BOC_Kp_pitch, BOC_Ki_pitch;
    double cp_foot_offset_y;
    double Kp_cp;
    double ft_time, ft_step_time;
    double ft_time2;
    int ft_cnt, ft_step_cnt, ft_ready_cnt, ft_finish_cnt;
    int ft_cnt2;
    //    double des_theta;
    int turn_mode, turn_cnt;
    double turn_xl_EP, turn_yl_EP, turn_xr_EP, turn_yr_EP;

    int js_x_vel;

    VectorNd tmp_zmp_ref = VectorNd::Zero(2);
    double init_zmp_ref_y, pre_zmp_ref_y, tmp_final_zmp_ref_y;
    double x_dist, y_dist;
    int jump_num;
    bool T_RL, T_RR, T_FL, T_FR;
    double RL_init_z, RR_init_z, FL_init_z, FR_init_z;
    double tmp_test_time;
    int tmp_phase;
    int RL_landing_cnt, RR_landing_cnt, FL_landing_cnt, FR_landing_cnt;
    int RL_landing_cnt2, RR_landing_cnt2, FL_landing_cnt2, FR_landing_cnt2;
    VectorNd com_acc = VectorNd::Zero(3);
    int test_phase = 0;
    double tmp_para[10];
    double global_jump_flight_cnt;

    int tw_cnt;
    double tw_time;

    //    double des_pitch_deg;

    double fc_weight = 0;
    int test_cnt = 0;

    unsigned int walk_ready_cnt = 2000;
    double walk_ready_time = 2;

    double alpha_act_com_x, alpha_act_com_y;
    //    double weight_cp_x, weight_cp_y;
    double w_cp_y1, w_cp_y2;
    double cp_x_max, cp_x_min, cp_y_max, cp_y_min;
    double cp_x, cp_y;

    MatrixNd Rot_Mat_XYZ = MatrixNd::Zero(3, 3);

    double kp_dc, kd_dc;

    int cp_phase = 0;
    int walking_phase = 0;
    int ft_phase = 0;

    double pos_alpha = 0;
    double vel_alpha = 0;

    //    int cp_x_case, cp_y_case;
    //    double next_cp_x, next_cp_y;

    //    VectorNd global_foot_center   = VectorNd::Zero(3,1);


    // Exitflag

    // =============== for QP based balance controller ================= //

    // Workspace structures
    OSQPWorkspace *QP_work;
    OSQPSettings *QP_settings = (OSQPSettings *) c_malloc(sizeof (OSQPSettings));
    OSQPData *QP_data = (OSQPData *) c_malloc(sizeof (OSQPData));

    c_int QP_exitflag = 0;

    c_int P_nnz = 78;
    c_float P_x[78];
    c_int P_i[78];
    c_int P_p[13];
    c_float q[12];

    c_float A_x[12]; // = {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1};
    c_int A_nnz = 12;
    c_int A_i[12]; // = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11};
    c_int A_p[13]; // = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12};
    c_float l[12]; // = {_d_l(0), _d_l(1), _d_l(2), _d_l(3), _d_l(4), _d_l(5), _d_l(6), _d_l(7), _d_l(8), _d_l(9), _d_l(10), _d_l(11)};
    c_float u[12]; // = {_d_u(0), _d_u(1), _d_u(2), _d_u(3), _d_u(4), _d_u(5), _d_u(6), _d_u(7), _d_u(8), _d_u(9), _d_u(10), _d_u(11)};
    c_int n = 12;
    c_int m = 12;





    //    VectorNd tmp_P_x = VectorNd::Zero(78);
    //    VectorNd new_c = VectorNd::Zero(12);

    // =============== for QP based balance controller END ================= //

    // ============== MPC (preview control)============== //

    OSQPWorkspace *MPC_work;
    OSQPSettings *MPC_settings = (OSQPSettings *) c_malloc(sizeof (OSQPSettings));
    OSQPData *MPC_data = (OSQPData *) c_malloc(sizeof (OSQPData));

    c_int MPC_exitflag = 0;

    int state_num = 13;
    int output_num = 13;
    int input_num = 12;

    //    double Ts = 0.15;//0.15;//0.005;//0.15;//0.05;
    //    int Ts_cnt = 150;//150;//5;//150;//50;
    //    int Nc = 10;//10;//300;//10;//30;
    //    int Np = Nc;//10;//300;//10;//30 + 0;

    double Ts = 0.05;
    int Ts_cnt = 5;
    int Nc = 10;
    int Np = Nc;

    double mass = 46;
    //    double h_com = 0.4;
    double g = 9.81;
    //    double new_ref = 0;
    MatrixNd I_g = MatrixNd::Zero(3, 3);
    MatrixNd inv_I_g = MatrixNd::Zero(3, 3);

    MatrixNd PHI = MatrixNd::Zero(13, 13); // state_num x state_num
    MatrixNd GAM = MatrixNd::Zero(13, 12); // state_num x input_num
    MatrixNd C = MatrixNd::Identity(13, 13); // output_num x state_num

    MatrixNd _Q = MatrixNd::Identity(output_num, output_num);
    VectorNd _Q_vec = VectorNd::Zero(output_num);
    MatrixNd _R = MatrixNd::Identity(input_num, input_num) * pow(10, -6);

    MatrixNd Q_tilda = MatrixNd::Zero(output_num*Np, output_num*Np);
    MatrixNd R_tilda = MatrixNd::Zero(input_num*Nc, input_num*Nc);

    VectorNd ref_y = VectorNd::Zero(output_num);
    VectorNd ref_tilda = VectorNd::Zero(output_num*Np);

    //    MatrixNd _Q = MatrixNd::Identity(1, 1);
    //    MatrixNd _R = MatrixNd::Identity(1, 1)*pow(10,-6)*1;
    //    VectorNd x_ini = VectorNd::Zero(3);
    //    VectorNd y_ini = VectorNd::Zero(1);
    //
    MatrixNd Ax_tilda = MatrixNd::Zero(state_num*Np, state_num);
    MatrixNd tmp_Ax = MatrixNd::Zero(state_num, state_num);
    MatrixNd Bx_tilda = MatrixNd::Zero(state_num*Np, input_num*Nc);
    MatrixNd tmp_Bx2 = MatrixNd::Zero(state_num*Np, input_num*Nc);
    MatrixNd Bx_element = MatrixNd::Zero(state_num, input_num);
    MatrixNd C_tilda = MatrixNd::Zero(output_num*Np, state_num*Np);
    MatrixNd Q_bar = MatrixNd::Zero(output_num*Np, output_num*Np);
    MatrixNd H = MatrixNd::Zero(input_num*Nc, input_num*Nc);

    VectorNd f = VectorNd::Zero(Nc*input_num);
    //    MatrixNd Gy_tmp = MatrixNd::Zero(Np*output_num, Np*output_num);
    //    MatrixNd Gy = MatrixNd::Zero(Np*output_num*2, Np*output_num);
    //    MatrixNd gy = MatrixNd::Zero(Np*output_num*2, output_num);
    //    MatrixNd Gy_new = MatrixNd::Zero(Np*output_num*2, input_num*Nc);
    //    MatrixNd gy_new = MatrixNd::Zero(Np*output_num*2, 1);
    //
    //    MatrixNd Gu_tmp = MatrixNd::Zero(Nc*input_num, Nc*input_num);
    //    MatrixNd Gu = MatrixNd::Zero(Nc*input_num*2, Nc*input_num);
    //    MatrixNd gu = MatrixNd::Zero(Nc*input_num*2, 1);
    //
    //    MatrixNd G = MatrixNd::Zero(Np*output_num*2 + Nc*input_num*2, input_num*Nc);
    //    MatrixNd bk = MatrixNd::Zero(Np*output_num*2 + Nc*input_num*2, 1);
    //
    //    double input_u = 0;
    ////    VectorNd input_u = VectorNd::Zero(1);
    //    VectorNd tar_state_x = VectorNd::Zero(3);
    //    VectorNd tar_output_y = VectorNd::Zero(3);
    //    VectorNd next_state_x = VectorNd::Zero(3);
    //    VectorNd pre_state_x = VectorNd::Zero(3);
    VectorNd act_state_x = VectorNd::Zero(13);
    //    VectorNd act_output_y = VectorNd::Zero(1);
    //    VectorNd next_output_y = VectorNd::Zero(3);
    ////    double act_output_y = 0;
    //
    ////    double output_constraint = 0.1;
    //    double input_constraint = 10000.0;
    //
    VectorNd input_const = VectorNd::Zero(12);
    c_int H_nnz = 7260;
    c_float H_x[7260];
    c_int H_i[7260];
    c_int H_p[121];
    c_float G_x[120];
    c_int G_nnz = 120;
    c_int G_i[120];
    c_int G_p[121];
    c_float G_l[120];
    c_float G_u[120];
    c_int G_n = 120;
    c_int G_m = 120;
    c_float ff[120];

    double mu = 0.6; // static friction coefficient
    double max_Fext_z = 1000;

    //    c_int H_nnz = 45150;
    //    c_float H_x[45150];
    //    c_int H_i[45150];
    //    c_int H_p[301];
    //    c_float G_x[300];
    //    c_int G_nnz = 300;
    //    c_int G_i[300];
    //    c_int G_p[301];
    //    c_float G_l[300];
    //    c_float G_u[300];
    //    c_int G_n = 300;
    //    c_int G_m = 300;
    //    c_float ff[300];


    // ============== MPC (preview control) END ============== //


    //    // ============== MPC (preview control)============== //
    //
    //    OSQPWorkspace *MPC_work;
    //    OSQPSettings *MPC_settings = (OSQPSettings *) c_malloc(sizeof (OSQPSettings));
    //    OSQPData *MPC_data = (OSQPData *) c_malloc(sizeof (OSQPData));
    //
    //    c_int MPC_exitflag = 0;
    //
    //    int state_num = 3;
    //    int output_num = 1;
    //    int input_num = 1;
    //
    ////    double Ts = 0.15;//0.15;//0.005;//0.15;//0.05;
    ////    int Ts_cnt = 150;//150;//5;//150;//50;
    ////    int Nc = 10;//10;//300;//10;//30;
    ////    int Np = Nc;//10;//300;//10;//30 + 0;
    //
    //    double Ts = 0.005;
    //    int Ts_cnt = 5;
    //    int Nc = 300;
    //    int Np = Nc;
    //
    ////    double Ts = 0.001;
    ////    int Ts_cnt = 1;
    ////    int Nc = 500;
    ////    int Np = Nc;
    //
    //    // it's not worked.
    ////    double Ts = 0.001;
    ////    int Ts_cnt = 1;
    ////    int Nc = 1000;
    ////    int Np = Nc;
    //
    //    double h_com = 0.4;
    //    double g = 9.81;
    //    double new_ref = 0;
    //
    //    MatrixNd PHI = MatrixNd::Zero(3, 3);
    //    VectorNd GAM = VectorNd::Zero(3);
    //    MatrixNd C = MatrixNd::Zero(1,3);
    //
    //    MatrixNd Q_tilda = MatrixNd::Zero(Nc, Nc);
    //    MatrixNd R_tilda = MatrixNd::Zero(Nc, Nc);
    //    MatrixNd _Q = MatrixNd::Identity(1, 1);
    //    MatrixNd _R = MatrixNd::Identity(1, 1)*pow(10,-6)*1;
    //    VectorNd x_ini = VectorNd::Zero(3);
    //    VectorNd y_ini = VectorNd::Zero(1);
    //
    //    MatrixNd Ax_tilda = MatrixNd::Zero(state_num*Np, state_num);
    //    MatrixNd tmp_Ax = MatrixNd::Zero(state_num, state_num);
    //    MatrixNd Bx_tilda = MatrixNd::Zero(state_num*Np, input_num*Nc);
    //    MatrixNd tmp_Bx2 = MatrixNd::Zero(state_num*Np, input_num*Nc);
    //    MatrixNd Bx_element = MatrixNd::Zero(state_num, input_num);
    //    MatrixNd C_tilda = MatrixNd::Zero(output_num*Np, state_num*Np);
    //    MatrixNd Q_bar = MatrixNd::Zero(state_num*Np, state_num*Np);
    //    MatrixNd H = MatrixNd::Zero(Nc, Nc);
    //    MatrixNd ref_tilda = MatrixNd::Zero(Np, 1);
    //    MatrixNd f = MatrixNd::Zero(Np, 1);
    //    MatrixNd Gy_tmp = MatrixNd::Zero(Np*output_num, Np*output_num);
    //    MatrixNd Gy = MatrixNd::Zero(Np*output_num*2, Np*output_num);
    //    MatrixNd gy = MatrixNd::Zero(Np*output_num*2, output_num);
    //    MatrixNd Gy_new = MatrixNd::Zero(Np*output_num*2, input_num*Nc);
    //    MatrixNd gy_new = MatrixNd::Zero(Np*output_num*2, 1);
    //
    //    MatrixNd Gu_tmp = MatrixNd::Zero(Nc*input_num, Nc*input_num);
    //    MatrixNd Gu = MatrixNd::Zero(Nc*input_num*2, Nc*input_num);
    //    MatrixNd gu = MatrixNd::Zero(Nc*input_num*2, 1);
    //
    //    MatrixNd G = MatrixNd::Zero(Np*output_num*2 + Nc*input_num*2, input_num*Nc);
    //    MatrixNd bk = MatrixNd::Zero(Np*output_num*2 + Nc*input_num*2, 1);
    //
    //    double input_u = 0;
    ////    VectorNd input_u = VectorNd::Zero(1);
    //    VectorNd tar_state_x = VectorNd::Zero(3);
    //    VectorNd tar_output_y = VectorNd::Zero(3);
    //    VectorNd next_state_x = VectorNd::Zero(3);
    //    VectorNd pre_state_x = VectorNd::Zero(3);
    //    VectorNd act_state_x = VectorNd::Zero(3);
    //    VectorNd act_output_y = VectorNd::Zero(1);
    //    VectorNd next_output_y = VectorNd::Zero(3);
    ////    double act_output_y = 0;
    //
    ////    double output_constraint = 0.1;
    //    double input_constraint = 10000.0;
    //
    ////    c_int H_nnz = 55;//45150; //55
    ////    c_float H_x[55];
    ////    c_int H_i[55];
    ////    c_int H_p[11];// = {0, 1, 3, 6, 10, 15, 21, 28, 36, 45, 55};
    ////    c_float G_x[10];
    ////    c_int G_nnz = 10;
    ////    c_int G_i[10];
    ////    c_int G_p[11];
    ////    c_float G_l[10];
    ////    c_float G_u[10];
    ////    c_int G_n = 10;
    ////    c_int G_m = 10;
    ////    c_float ff[10];
    //
    //    c_int H_nnz = 45150;
    //    c_float H_x[45150];
    //    c_int H_i[45150];
    //    c_int H_p[301];
    //    c_float G_x[300];
    //    c_int G_nnz = 300;
    //    c_int G_i[300];
    //    c_int G_p[301];
    //    c_float G_l[300];
    //    c_float G_u[300];
    //    c_int G_n = 300;
    //    c_int G_m = 300;
    //    c_float ff[300];
    //
    //    // 500
    ////    c_int H_nnz = 125250;
    ////    c_float H_x[125250];
    ////    c_int H_i[125250];
    ////    c_int H_p[501];
    ////    c_float G_x[500];
    ////    c_int G_nnz = 500;
    ////    c_int G_i[500];
    ////    c_int G_p[501];
    ////    c_float G_l[500];
    ////    c_float G_u[500];
    ////    c_int G_n = 500;
    ////    c_int G_m = 500;
    ////    c_float ff[500];
    //
    //    // N = 1000, but not works
    ////    c_int H_nnz = 500500; //55
    ////    c_float H_x[500500];
    ////    c_int H_i[500500];
    ////    c_int H_p[1001];// = {0, 1, 3, 6, 10, 15, 21, 28, 36, 45, 55};
    ////    c_float G_x[1000];// = {1, 1, 1, 1, 1, 1, 1, 1, 1, 1};//{0, 0, 0, 0, 0, 0, 0, 0, 0, 0};//{1, 1, 1, 1, 1, 1, 1, 1, 1, 1};
    ////    c_int G_nnz = 1000;
    ////    c_int G_i[1000];// = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9};
    ////    c_int G_p[1001];// = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10};
    ////    c_float G_l[1000];// = {-input_constraint, -input_constraint, -input_constraint, -input_constraint, -input_constraint, -input_constraint, -input_constraint, -input_constraint, -input_constraint, -input_constraint};
    ////    c_float G_u[1000];// = { input_constraint,  input_constraint,  input_constraint,  input_constraint,  input_constraint,  input_constraint,  input_constraint,  input_constraint,  input_constraint,  input_constraint};
    ////    c_int G_n = 1000;
    ////    c_int G_m = 1000;
    ////    c_float ff[1000];
    //
    //    double tar_Fc_y;
    //    VectorNd MPC_Fc = VectorNd::Zero(19);
    //
    //    // ============== MPC (preview control) END ============== //

    //Hyunseok Functions
    
    
    VectorNd Get_Base_pos_HS(VectorNd COM_pos, VectorNd q);
    VectorNd Get_COM_pos_HS(VectorNd Base_pos, VectorNd q);
    VectorNd Localization_Base2Hip_Pos_HS(VectorNd EP_pos_local);
    VectorNd Localization_Hip2Base_Pos_HS(VectorNd EP_pos_local);
    VectorNd Transform_G2L(VectorNd Base_pos, VectorNd EP_ori, VectorNd EP_pos);
    
    VectorNd Get_Actual_Base_pos_HS(VectorNd _COM_pos, VectorNd q);
    
    VectorNd IK_HS(VectorNd EP_pos_HS);
    VectorNd FK_HS(VectorNd joint_pos_HS);
    MatrixNd Base_Rotation_Matrix_HS(VectorNd _Base_ori);  
    
    void COM_XY_Traj_Gen_COM_VER_HS(unsigned int _i, VectorNd _init_com_pos, VectorNd _goal_com_pos);
    void WalkReady_Pos_Traj_HS(void);
    void Walking_Gait_Traj_HS(void);
    void Walking_Traj_First_HS(unsigned int _i);
    void Walking_Traj_COM_VER_HS(unsigned int _i);
    void FootStepPlanning_HS(VectorNd _now_com_pos, VectorNd _now_base_ori, VectorNd _now_EP_pos, VectorNd _now_vel, VectorNd _tar_vel);
    void SF_EP_Traj_Gen_HS(double _travel_time, VectorNd _init_EP_pos, VectorNd _goal_EP_pos);
    void Get_gain_HS(void);
    void Preview_con_HS();
    void Diff_COM_HS(void);
    
    void Task_Gain_Setting_HS(void);
    void ComputeTorqueControl_HS(void);
    void Joint_Space_Controller(void);
    void Task_Space_Controller(void);
    void QP_Task_Space_Controller(void);

    void Transform_DH2HS(void);
    void Transform_HS2DH(void);
    
    VectorNd Base_Estimation(VectorNd actual_EP_local);
    
    void print_HS(void);
    
    
    unsigned int cnt_HS = 0;
    int tsp_cnt_HS = 250, fsp_cnt_HS = 100;
    int step_cnt_HS = tsp_cnt_HS + fsp_cnt_HS;
    double tsp_time_HS = tsp_cnt_HS*dt;
    double fsp_time_HS = fsp_cnt_HS*dt;
    double step_time_HS;
    unsigned int preview_cnt_HS = 1400;
    double preview_time_HS = preview_cnt_HS*dt;
    //double com_height_HS = 0.45;
    double com_height_HS = 0.4;
    double foot_height_HS=0.10;
    double tmp_foot_height_HS=foot_height_HS;
    
    VectorNd Contact_Info_HS = VectorNd::Ones(4);
    VectorNd now_vel_HS = VectorNd::Zero(3);
    VectorNd tar_vel_HS = VectorNd::Zero(3);

    bool init_Transform_flag_HS = true;
    bool init_Transform_flag_HS2 = true;
    
    VectorNd target_pos_HS = VectorNd::Zero(13);
    VectorNd target_vel_HS = VectorNd::Zero(13);
    VectorNd actual_pos_HS = VectorNd::Zero(13);
    VectorNd actual_vel_HS = VectorNd::Zero(13);

    VectorNd init_EP_HS = VectorNd::Zero(12);
    VectorNd goal_EP_HS = VectorNd::Zero(12);
    VectorNd actual_EP_HS = VectorNd::Zero(12);
    VectorNd actual_EP_vel_HS = VectorNd::Zero(12);
    VectorNd target_EP_HS = VectorNd::Zero(12);
    VectorNd target_EP_vel_HS = VectorNd::Zero(12);
    VectorNd target_EP_ori_HS=VectorNd::Zero(12);
    VectorNd pre_init_EP_HS = VectorNd::Zero(12);
    VectorNd pre_actual_EP_HS=VectorNd::Zero(12);
    VectorNd EP_err_HS = VectorNd::Zero(12);
    VectorNd EP_vel_err_HS = VectorNd::Zero(12);

    VectorNd init_com_pos_HS = VectorNd::Zero(3);
    VectorNd goal_com_pos_HS = VectorNd::Zero(3);
    VectorNd actual_com_pos_HS = VectorNd::Zero(3);
    VectorNd actual_com_vel_HS = VectorNd::Zero(3);
    VectorNd target_com_pos_HS = VectorNd::Zero(3);
    VectorNd target_com_vel_HS = VectorNd::Zero(3);
    VectorNd target_com_acc_HS=VectorNd::Zero(3);
    VectorNd com_pos_err_HS = VectorNd::Zero(12);
    VectorNd com_vel_err_HS = VectorNd::Zero(12);
    VectorNd pre_actual_com_pos_HS=VectorNd::Zero(3);
    VectorNd pre_target_com_pos_HS=VectorNd::Zero(3);
    VectorNd pre_target_com_vel_HS=VectorNd::Zero(3);
    
    VectorNd init_base_ori_HS = VectorNd::Zero(3);
    VectorNd goal_base_ori_HS = VectorNd::Zero(3);
    VectorNd actual_base_ori_HS = VectorNd::Zero(3);
    VectorNd actual_base_ori_vel_HS = VectorNd::Zero(3);
    VectorNd actual_base_pos_HS=VectorNd::Zero(3);
    VectorNd target_base_ori_HS = VectorNd::Zero(3);
    VectorNd target_base_ori_vel_HS = VectorNd::Zero(3);
    VectorNd target_base_pos_HS=VectorNd::Zero(3);

    MatrixNd target_C_WB_HS = MatrixNd::Zero(3, 3);
    MatrixNd target_C_WB_12d_HS = MatrixNd::Zero(12, 12);

    VectorNd zmp_ref_x_array_HS = VectorNd::Zero(preview_cnt_HS);
    VectorNd zmp_ref_y_array_HS = VectorNd::Zero(preview_cnt_HS);

    // Preview Variables
    double pv_Gp_HS[1400], pv_Gx_HS[3], pv_Gi_HS[1];
    double Gi_HS = 0;
    VectorNd Gx_HS = VectorNd::Zero(3);
    VectorNd Gp_HS = VectorNd::Zero(preview_cnt_HS);
    VectorNd X_new_x_HS = VectorNd::Zero(3);
    VectorNd X_new_y_HS = VectorNd::Zero(3);
    MatrixNd AA_HS = MatrixNd::Zero(3, 3);
    VectorNd BB_HS = VectorNd::Zero(3);
    VectorNd CC_HS = VectorNd::Zero(3);
    double sum_e_x_HS = 0.0;
    double sum_e_y_HS = 0.0;

    double z1_up[6], z2_up[6], z3_up[6], z4_up[6];
    double z1_down[6], z2_down[6], z3_down[6], z4_down[6];
    VectorNd tmp_moving_speed_HS = VectorNd::Zero(3);
    double speed_x = 0.0;
    double speed_y = 0.0;
    double speed_yaw = 0.0;

    VectorNd Joint_Control_value_HS = VectorNd::Zero(19);
    VectorNd Task_Control_value_HS = VectorNd::Zero(19);
    VectorNd QP_Control_value_HS = VectorNd::Zero(19);

    VectorNd kp_joint_HS = VectorNd::Zero(13);
    VectorNd kd_joint_HS = VectorNd::Zero(13);
    VectorNd kp_EP_HS = VectorNd::Zero(12);
    VectorNd kd_EP_HS = VectorNd::Zero(12);
    VectorNd kp_com_HS = VectorNd::Zero(12);
    VectorNd kd_com_HS = VectorNd::Zero(12);

    bool init_Force_flag_HS = true;
    VectorNd init_kp_EP_HS = VectorNd::Zero(12);
    VectorNd init_kd_EP_HS = VectorNd::Zero(12);
    VectorNd init_kp_com_HS = VectorNd::Zero(12);
    VectorNd init_kd_com_HS = VectorNd::Zero(12);
    VectorNd goal_kp_EP_HS = VectorNd::Zero(12);
    VectorNd goal_kd_EP_HS = VectorNd::Zero(12);
    VectorNd goal_kp_com_HS = VectorNd::Zero(12);
    VectorNd goal_kd_com_HS = VectorNd::Zero(12);
    VectorNd target_kp_EP_HS = VectorNd::Zero(12);
    VectorNd target_kd_EP_HS = VectorNd::Zero(12);
    VectorNd target_kp_com_HS = VectorNd::Zero(12);
    VectorNd target_kd_com_HS = VectorNd::Zero(12);
    unsigned int cnt_force=0;
    
    VectorNd x_saved = VectorNd::Zero(12);
    VectorNd Standard_leg=VectorNd::Zero(4);   
    bool WalkReady_flag_HS=true;
    bool tmp_sub_ctrl_flag_HS=false;
    unsigned int sub_cnt_HS=0;
    //VectorNd target_EP_local_HS=VectorNd(12);
    
private:
};

#endif /* CROBOT_H */
