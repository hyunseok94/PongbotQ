#ifndef CROBOT_H
#define CROBOT_H


#include "rbdl/rbdl.h"
#include <rbdl/addons/urdfreader/urdfreader.h>

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
    CTRLMODE_TROT,
    CTRLMODE_FLYING_TROT,
    CTRLMODE_PRONK_JUMP,
    CTRLMODE_TEST
} _CONTROL_MODE;

typedef enum {
    NO_ACT,
    EXIT_PROGRAM,
    SET_MOTOR_GAIN,
    SET_CURRENT_GAIN,
    LOAD_PARAMETER,
    SAVE_PARAMETER,
    SET_JOINT_PARAMETER,
    GET_JOINT_PARAMETER,
    SET_BOARD_PARAMETER,
    GET_BOARD_PARAMETER,
    PRINT_JOINT_PARAMETER,
    CHECK_DEVICE,
    GAIN_SETTING,
    ENABLE_FET,
    ENABLE_FET_EACH,
    DISABLE_FET,
    DISABLE_FET_EACH,
    RUN_CMD,
    RUN_CMD_EACH,
    STOP_CMD,
    STOP_CMD_EACH,
    GOTO_LIMIT_POS,
    GOTO_LIMIT_POS_UPPER_ALL,
    GOTO_LIMIT_POS_LOWER_ALL,
    GOTO_LIMIT_POS_ALL,
    ENCODER_ZERO,
    ENCODER_ZERO_EACH,
    SAVE_ZMP_INIT_POS,
    SET_ENCODER_RESOLUTION,
    SET_DEADZONE,
    SET_JAMPWM_FAULT,
    SET_MAX_VEL_ACC,
    SET_CONTROL_MODE,
    SET_HOME_SEARCH_PARAMETER,
    SET_HOME_MAX_VEL_ACC,
    SET_POSITION_LIMIT,
    SET_ERROR_BOUND,
    REQUEST_PARAMETER,
    POSITION_LIMIT_ONOFF,
    BEEP,
    JOINT_REF_SET_RELATIVE,
    JOINT_REF_SET_ABS,
    SET_FT_PARAMETER,
    GET_FT_PARAMETER,
    NULL_FT_SENSOR,
    NULL_WRIST_FT_SENSOR,
    NULL_FOOT_ANGLE_SENSOR,
    NULL_IMU_SENSOR,
    SET_IMU_OFFSET,
    PRINT_FT_PARAMETER,
    SET_IMU_PARAMETER,
    GET_IMU_PARAMETER,
    PRINT_IMU_PARAMETER,
    SET_DAMPING_GAIN,
    SET_DSP_GAIN,
    GOTO_WALK_READY_POS,
    GOTO_HOME_POS,
    START_ZMP_INITIALIZATION,
    STOP_ZMP_INITIALIZATION,
    GOTO_FORWARD,
    STOP_WALKING,
    SET_MOCAP,
    C_CONTROL_MODE,
    P_CONTROL_MODE,
    GRIP_ON,
    GRIP_OFF,
    GRIP_STOP,
    DEMO_FLAG,
    TEST_FUNCTION,
    DEMO_GRASP, // jungho77
    SET_PREDEF_WALK, // jungho77
    INIT_WB_MOCAP, // by Inhyeok
    DEMO_CONTROL_OFF_POS,
    DEMO_CONTROL_ON_POS,
    RBT_ON_MODE, //CDI
    RBT_OFF_MODE, //CDI
    CCTM_ON,
    CCTM_OFF,
    JUMP_ONESTEP, // BKCho
    NOMAL_TROT_WALKING,
    FLYING_TROT_RUNNING,
    TORQUE_OFF,
    PRONK_JUMP,
    TEST_FLAG,
    NO_ACT_WITH_CTC

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

    //ORI orientation;
    //POS pos;
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
    void Get_gain(void);
    void Trot_Walking_Traj_First(unsigned int i);
    void Trot_Walking_Traj(unsigned int i);
    void Trot_Walking_Traj_Final(unsigned int i);
    void Foot_step_planner(VectorNd init_foot_l_2d, VectorNd init_foot_r_2d);
    void Foot_step_planner_first(VectorNd init_foot_l_2d, VectorNd init_foot_r_2d);
    void COM_X_Traj_Gen(unsigned int i);
    void Preview_con(void);
    void Preview_con_2d(void);
    void SF_Z_Traj_Gen(void);
    void SF_X_Traj_Gen(void);
    void SF_X_Traj_Gen_Final(void);
    void get_zmp(void);
    void Flying_Trot_Running(void);
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
    void Body_Ori_Con2(void);
    void One_Step_Standing_Jump(void);
    void Jump_COM_Z_Traj_Gen(double h0, double t[4]);
    void Get_act_com(void);
    void Damping_con(void);
    void COM_FT_X_Traj_Gen(void);
    void COM_FT_Z_Traj_Gen(void);
    void SF_FT_X_Traj_Gen(void);
    void SF_FT_Z_Traj_Gen(void);
    void Pronk_Jump(void);
    void Test_Function(void);
    void Turning_Traj_Gen(void);
    void FT_Turning_Traj_Gen(void);
    void COM_SF_TW_X_Traj_Gen(void);
    void SF_TW_Z_Traj_Gen(void);
    void TW_Turning_Traj_Gen(void);
    void COM_SF_FT_X_Traj_Gen(void);
    void COM_SF_FT_Z_Traj_Gen(void);
    void CP_Con_FT(void);
    void check_CP_FT(void);
    void FT_Traj_Gen(void);
    void Cal_JFc(void);
    
    

    enum Fc_Phase {
        INIT_Fc = 0,
        STOP,
        STANCE_RLFR,
        STANCE_RRFL,
        JUMP_Fc,
        ZERO
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

    VectorNd EP_RL = Vector3d(0, 0, 0);
    VectorNd EP_RR = Vector3d(0, 0, 0);
    VectorNd EP_FL = Vector3d(0, 0, 0);
    VectorNd EP_FR = Vector3d(0, 0, 0);

    double L3_x = 0.025516;
    double L3_y = 0.0;
    double L3_z = 0.304515;

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

    VectorNd target_tor = VectorNd::Zero(13);

    MatrixNd J_RL = MatrixNd::Zero(3, 19);
    MatrixNd J_RR = MatrixNd::Zero(3, 19);
    MatrixNd J_FL = MatrixNd::Zero(3, 19);
    MatrixNd J_FR = MatrixNd::Zero(3, 19);
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
    int Body_Ori_Con_onoff_flag;
    bool turn_start_flag;
    bool first_jump_flag;
    bool T_RL_on_flag, T_RR_on_flag, T_FL_on_flag, T_FR_on_flag;
    bool ft_ready_flag, ft_finish_flag;


    unsigned int ctc_cnt = 0, ctc_cnt2 = 0, ctc_cnt3 = 0, ctc_cnt4 = 0, ctc_cnt5 = 0; //Counts added by HSKIM
    double home_pos_time, init_pos_time;
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

    double dsp_time = 0.22, fsp_time = 0.05;
	double step_time = dsp_time + fsp_time;
	int dsp_cnt = 220, fsp_cnt = 50;
	int step_cnt = dsp_cnt + fsp_cnt;
    
    int stride_cnt = step_cnt*2;
    
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
    VectorNd actual_com_position=VectorNd::Zero(3);
    VectorNd pre_actual_com_position=VectorNd::Zero(3);
    VectorNd actual_com_speed=VectorNd::Zero(3);
    double tmp_x_moving_speed, tmp_y_moving_speed;
    double x_step = 0; //step_time*moving_speed/2.0;
    double x_fsp = 0; //fsp_time*moving_speed/2.0;
    double pre_foot_l[3]; // = [ -x_step - x_fsp,-moving_speed,0];
    double pre_foot_r[3]; // = [  x_step - x_fsp,-moving_speed,0];

    VectorNd R = VectorNd::Zero(6);
    VectorNd P = VectorNd::Zero(6);
    MatrixNd A = MatrixNd::Zero(6, 6);

    VectorNd tmp_data1 = VectorNd::Zero(30);
    VectorNd tmp_data2 = VectorNd::Zero(30);
    VectorNd computed_tor = VectorNd::Zero(13);
    VectorNd Fc_vsd = VectorNd::Zero(12);
    VectorNd Kp_vsd = VectorNd::Zero(12);
    VectorNd Kd_vsd = VectorNd::Zero(12);

    Quaternion QQ;


    // =============== IMU ================ //
    double IMURoll, IMUPitch, IMUYaw;
    double IMURoll_dot, IMUPitch_dot, IMUYaw_dot;

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
    unsigned int preview_cnt = 1000;
    
    double Gi = 0;
    VectorNd Gx = VectorNd::Zero(3);   
    VectorNd Gp = VectorNd::Zero(1000);
    
    MatrixNd AA = MatrixNd::Zero(3, 3);
    VectorNd BB = VectorNd::Zero(3);
    VectorNd CC = VectorNd::Zero(3);
    
    double pv_Gp[1000],pv_Gx[3],pv_Gi[1];
    
    VectorNd com_x = VectorNd::Zero(3);   // x, x_dot, x_2dot
    VectorNd com_pos = VectorNd::Zero(3); // x,y,z
    VectorNd old_com_pos = VectorNd::Zero(3); // x,y,z
    VectorNd old_com_vel = VectorNd::Zero(3); // x,y,z
    VectorNd target_com_vel = VectorNd::Zero(3); // x,y,z
    VectorNd target_com_acc = VectorNd::Zero(3); // x,y,z
    VectorNd actual_com_pos = VectorNd::Zero(3); // x,y,z
    VectorNd actual_com_vel = VectorNd::Zero(3); // x,y,z
    VectorNd pre_com_pos = VectorNd::Zero(3); // x,y,z
    
    VectorNd local_RL_foot_pos = VectorNd::Zero(3);
    VectorNd local_RR_foot_pos = VectorNd::Zero(3);
    VectorNd local_FL_foot_pos = VectorNd::Zero(3);
    VectorNd local_FR_foot_pos = VectorNd::Zero(3);
    
    MatrixNd foot_l_2d = MatrixNd::Zero(5,2);
    MatrixNd foot_r_2d = MatrixNd::Zero(5,2);
    MatrixNd pre_foot_l_2d = MatrixNd::Zero(5,2);
    MatrixNd pre_foot_r_2d = MatrixNd::Zero(5,2);
    VectorNd final_foot_l_2d = VectorNd::Zero(2);
    VectorNd final_foot_r_2d = VectorNd::Zero(2);
    MatrixNd zmp_ref_array = MatrixNd::Zero(preview_cnt,2);
    VectorNd XX = VectorNd::Zero(3);
    VectorNd X_new = VectorNd::Zero(3);
    VectorNd Y_new = VectorNd::Zero(3);
    VectorNd zmp_ref_old = VectorNd::Zero(2,1);
        
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
    
    MatrixNd com_x_array = MatrixNd::Zero(preview_cnt,3);
    VectorNd zmp_x_array = VectorNd::Zero(preview_cnt);
    double zmp_x, zmp_y, lpf_zmp_x, old_lpf_zmp_x;
    VectorNd sum_e = VectorNd::Zero(2,1);
    VectorNd pre_err = VectorNd::Zero(2,1);
    VectorNd pre_sum_p = VectorNd::Zero(2,1);
    VectorNd zmp_ref = VectorNd::Zero(2,1);

    VectorNd init_com_pos = VectorNd::Zero(3);
    
    VectorNd init_RL_foot_pos = VectorNd::Zero(3);
    VectorNd init_RR_foot_pos = VectorNd::Zero(3);
    VectorNd init_FL_foot_pos = VectorNd::Zero(3);
    VectorNd init_FR_foot_pos = VectorNd::Zero(3);
    
    VectorNd RL_foot_pos = VectorNd::Zero(3);
    VectorNd RR_foot_pos = VectorNd::Zero(3);
    VectorNd FL_foot_pos = VectorNd::Zero(3);
    VectorNd FR_foot_pos = VectorNd::Zero(3);
    
    VectorNd pre_RL_foot_pos = VectorNd::Zero(3);
    VectorNd pre_RR_foot_pos = VectorNd::Zero(3);
    VectorNd pre_FL_foot_pos = VectorNd::Zero(3);
    VectorNd pre_FR_foot_pos = VectorNd::Zero(3);
    
    VectorNd RL_foot_vel = VectorNd::Zero(3);
    VectorNd RR_foot_vel = VectorNd::Zero(3);
    VectorNd FL_foot_vel = VectorNd::Zero(3);
    VectorNd FR_foot_vel = VectorNd::Zero(3);
    
    
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
    double c_sf_z1[6], c_sf_z2[6], c_sf_z3[6], c_sf_z4[6];
    double c_sf_x1[6], c_sf_x2[6], c_sf_x3[6], c_sf_x4[6], c_sf_x5[6];
    
    
    // ============================ //
    
    MatrixNd C_I_roll=MatrixNd::Zero(3,3);
    MatrixNd C_I_pitch=MatrixNd::Zero(3,3);
    
    MatrixNd RL_C_I_HP=MatrixNd::Zero(3,3);
    MatrixNd RL_C_HP_HR=MatrixNd::Zero(3,3);
    MatrixNd RL_C_HR_KN=MatrixNd::Zero(3,3);
    MatrixNd RL_C_KN_TIP=MatrixNd::Zero(3,3);
    
    MatrixNd RR_C_I_HP=MatrixNd::Zero(3,3);
    MatrixNd RR_C_HP_HR=MatrixNd::Zero(3,3);
    MatrixNd RR_C_HR_KN=MatrixNd::Zero(3,3);
    MatrixNd RR_C_KN_TIP=MatrixNd::Zero(3,3);
    
    MatrixNd FL_C_I_HP=MatrixNd::Zero(3,3);
    MatrixNd FL_C_HP_HR=MatrixNd::Zero(3,3);
    MatrixNd FL_C_HR_KN=MatrixNd::Zero(3,3);
    MatrixNd FL_C_KN_TIP=MatrixNd::Zero(3,3);
    
    MatrixNd FR_C_I_HP=MatrixNd::Zero(3,3);
    MatrixNd FR_C_HP_HR=MatrixNd::Zero(3,3);
    MatrixNd FR_C_HR_KN=MatrixNd::Zero(3,3);
    MatrixNd FR_C_KN_TIP=MatrixNd::Zero(3,3);
    
    
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
    int  ft_cnt, ft_step_cnt, ft_ready_cnt, ft_finish_cnt;
    int  ft_cnt2;
    double des_theta;
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
    int RL_landing_cnt, RR_landing_cnt, FL_landing_cnt ,FR_landing_cnt;
    int RL_landing_cnt2, RR_landing_cnt2, FL_landing_cnt2 ,FR_landing_cnt2;
    VectorNd com_acc = VectorNd::Zero(3);
    int test_phase = 0;
    double tmp_para[10];
    double global_jump_flight_cnt;
    
    int tw_cnt;
    double tw_time;
    
    double des_pitch_deg;
    
private:
};

#endif /* CROBOT_H */

