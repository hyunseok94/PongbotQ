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
    CTRLMODE_NONE,
    CTRLMODE_INITIALIZE,
    CTRLMODE_HOME_POS,
    CTRLMODE_WALK_READY,
    CTRLMODE_TROT
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
    WALKING

} _COMMAND_FLAG;

//    enum{
//        IDLE = 0,
//        INITIALIZE,
//        HOME_POS,
//        POS_INIT,
//        TROT,
//        FORWARD,
//        TURN_RIGHT,
//        TURN_LEFT,
//        JUMPING,
//        FLYING_TROT,
//        UP_DOWN, //Up down mode added by HSKIM
//        RAISE_LEG, //Raise leg mode added by HSKIM
//        JUMP,       //Jump mode added by HSKIM
//        TEST        //Test mode added by HSKIM
//    };
    
//enum ControlMode CONTROL_MODE;

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
    VectorNd FK1(VectorNd jointAngle);
    VectorNd IK1(VectorNd EP);
    void Init_Pos_Traj(void);
    void Home_Pos_Traj(void);
    void TROT_Traj(void);
    void Forward_Traj(void);
    void Up_Down_Traj(void); //Up Down trajectory added by HSKIM
    void Raise_Leg_Traj(void); //Raise leg trajectory added by HSKIM
    void Jump_Traj(void); //Jump trajectory added by HSKIM
    void TEST_Traj(void); //Test trajectory added by HSKIM
    void Traj_gen(void);
    void coefficient_5thPoly(double *init_x, double *final_x, double tf, double *output);
    void Cal_Fc(void);
    void Flying_Trot_Traj(void);
    void ballistics(double flight_time, double landing_height, double take_off_speed);
    
    
    enum{
        IDLE = 0,
        INITIALIZE,
        HOME_POS,
        POS_INIT,
        TROT,
        FORWARD,
        TURN_RIGHT,
        TURN_LEFT,
        JUMPING,
        FLYING_TROT,
        UP_DOWN, //Up down mode added by HSKIM
        RAISE_LEG, //Raise leg mode added by HSKIM
        JUMP,       //Jump mode added by HSKIM
        TEST        //Test mode added by HSKIM
    };
    
    enum Phase {
        INIT_Fc = 0,
        STOP,
        STANCE_RLFR,
        STANCE_RRFL,
        STANCE_FOUR_LEGS_AFTER_RLFR,
        STANCE_FOUR_LEGS_AFTER_RRFL,
        JUMP_Fc,
        UP,       //Phase of Up Down mode added by HSKIM
        DOWN,     //Phase of Up Down mode added by HSKIM
        COM_MOVE, //Phase of Raise Leg mode added by HSKIM
        LEG_UP,   //Phase of Raise Leg mode added by HSKIM
        JUMP_READY, //Phase of Jump mode added by HSKIM
        JUMPPING,     //Phase of Jump mode added by HSKIM
        TEST_JUMPPING
    //    REAR_L
    };
    
    enum Forward_Phase {
        INIT_FORWARD = 0,
        INIT_STANCE_RRFL,
        INIT_STANCE_FOUR_AFTER_RRFL,
        TROT_STANCE_RLFR,
        TROT_STANCE_RRFL,
        TROT_STANCE_FOUR_AFTER_RLFR,
        TROT_STANCE_FOUR_AFTER_RRFL,
        FINAL_STANCE_FOUR,
        FINAL_STANCE_RLFR,
        FINAL_STANCE_RRFL
    };
    
//    enum ControlMode
//        {
//            INIT_POS,
//            HOME_POS,
//            POS_INIT,
//            TROT
//        };
//        
    enum Phase TROT_PHASE;
    enum Forward_Phase FORWARD_PHASE;
    enum Phase UP_DOWN_PHASE;    //UP DOWN mode's Phase added by HSKIM
    enum Phase RAISE_LEG_PHASE;  //RAISE LEG mode's Phase added by HSKIM
    enum Phase JUMP_PHASE;  //Jump mode's Phase added by HSKIM
    enum Phase TEST_PHASE;  //TEST mode's Phase added by HSKIM
    
    //Variables
    BASE base; //* coordinate of Body
    JOINT* joint; //* joints of the robot
    ENDPOINT FR, FL, RR, RL, front_body;
    int nDOF; //* number of DOFs of a robot
    
 //enum ControlMode CONTROL_MODE;
//    CONTROL_MODE CTR_MODE;
    
//    int ControlMode;
    
    unsigned int ControlMode;
    unsigned int CommandFlag;

    RigidBodyDynamics::Model* m_pModel; //* URDF Model
    RigidBodyDynamics::Math::VectorNd RobotState;
    RigidBodyDynamics::Math::VectorNd RobotStatedot;
    RigidBodyDynamics::Math::VectorNd RobotState2dot;
    RigidBodyDynamics::Math::VectorNd BasePosOri;
    RigidBodyDynamics::Math::VectorNd BaseVel;
    RigidBodyDynamics::Math::VectorNd JointAngle;
    RigidBodyDynamics::Math::VectorNd JointVel;

    MatrixNd M_term = MatrixNd::Zero(19, 19);
    VectorNd hatNonLinearEffects = VectorNd::Zero(19);
    VectorNd G_term = VectorNd::Zero(19);
    VectorNd C_term = VectorNd::Zero(19);
    VectorNd CTC_Torque = VectorNd::Zero(19);

    VectorNd EP_RL = Vector3d(0,0,0);
    VectorNd EP_RR = Vector3d(0,0,0);
    VectorNd EP_FL = Vector3d(0,0,0);
    VectorNd EP_FR = Vector3d(0,0,0);

    double L3_x = 0.025516;
    double L3_y = 0.0;
    double L3_z = 0.304515;

    VectorNd EP_OFFSET_RL = Vector3d( L3_x, L3_y, -L3_z);
    VectorNd EP_OFFSET_RR = Vector3d( L3_x, L3_y, -L3_z);
    VectorNd EP_OFFSET_FL = Vector3d(-L3_x, L3_y, -L3_z);
    VectorNd EP_OFFSET_FR = Vector3d(-L3_x, L3_y, -L3_z);
    VectorNd Originbase = Vector3d(0, 0, 0);

    VectorNd actual_EP = VectorNd::Zero(12);
    VectorNd actual_EP_vel = VectorNd::Zero(12);
    VectorNd actual_EP_acc = VectorNd::Zero(12);
    
    VectorNd actual_pos = VectorNd::Zero(13);
    VectorNd actual_vel = VectorNd::Zero(13);
    VectorNd actual_acc = VectorNd::Zero(13);
    
    VectorNd pre_actual_pos = VectorNd::Zero(13);
    VectorNd init_pos = VectorNd::Zero(13);

    VectorNd target_pos = VectorNd::Zero(13);
    VectorNd target_vel = VectorNd::Zero(13);
    VectorNd target_acc = VectorNd::Zero(13);
    VectorNd pre_target_pos = VectorNd::Zero(13);
    VectorNd pre_target_vel = VectorNd::Zero(13);

    VectorNd target_tor = VectorNd::Zero(13);
    
    MatrixNd J_RL = MatrixNd::Zero(3,19);
    MatrixNd J_RR = MatrixNd::Zero(3,19);
    MatrixNd J_FL = MatrixNd::Zero(3,19);
    MatrixNd J_FR = MatrixNd::Zero(3,19);
    MatrixNd J_FRONT_BODY = MatrixNd::Zero(6,19);
    MatrixNd J_BASE = MatrixNd::Zero(6,19);

    MatrixNd J_A = MatrixNd::Zero(19,19);

    VectorNd x_dot = VectorNd::Zero(19);
    VectorNd ddqZero = VectorNd::Zero(19);

    VectorNd RL_dJdQ = Vector3d(0,0,0);
    VectorNd RR_dJdQ = Vector3d(0,0,0);
    VectorNd FL_dJdQ = Vector3d(0,0,0);
    VectorNd FR_dJdQ = Vector3d(0,0,0);
    VectorNd FRONT_BODY_dJdQ = VectorNd::Zero(6);
    VectorNd base_dJdQ = VectorNd::Zero(6);
    VectorNd dJdQ = VectorNd::Zero(19);

    VectorNd Fc = VectorNd::Zero(19);
    VectorNd x_2dot_cp = VectorNd::Zero(19);

    VectorNd Kp_EP = VectorNd::Zero(12);//(100,100,100,100,100,100,100,100,100,100,100,100);
    VectorNd Kd_EP = VectorNd::Zero(12);//(1,1,1,1,1,1,1,1,1,1,1,1);
    VectorNd target_EP = VectorNd::Zero(12);
    VectorNd target_EP_vel = VectorNd::Zero(12);
    VectorNd target_EP_acc = VectorNd::Zero(12);
    VectorNd goal_EP = VectorNd::Zero(12);//(0,0.218,-0.45,0,-0.218,-0.45,0.7,0.218,-0.45,0.7,-0.218,-0.45);
    VectorNd init_EP = VectorNd::Zero(12);
    VectorNd init_goal_EP = VectorNd::Zero(12);
    VectorNd trot_goal_EP = VectorNd::Zero(12);
    double Fc_RL_z = 0, Fc_RR_z = 0, Fc_FL_z = 0, Fc_FR_z = 0;
    double Fc_RL_x = 0, Fc_RR_x = 0, Fc_FL_x = 0, Fc_FR_x = 0;
    double Fc_RL_y = 0, Fc_RR_y = 0, Fc_FL_y = 0, Fc_FR_y = 0;
    
    VectorNd angle_err = VectorNd::Zero(13);
    VectorNd EP_err=VectorNd::Zero(12);
    VectorNd EP_vel_err=VectorNd::Zero(12);
    
    VectorNd init_target_pos = VectorNd::Zero(13);
    
    bool home_init_flag = true;
    bool trot_init_flag = true;
    bool forward_init_flag = true;
    bool flying_trot_init_flag = true;
    bool up_down_init_flag=true;   //up down flag added by HSKIM
    bool raise_leg_init_flag=true;   //raise leg flag added by HSKIM
    bool jump_init_flag=true;   //jump flag added by HSKIM
    bool target_init_flag=true;
    
    unsigned int ctc_cnt = 0, ctc_cnt2 = 0, ctc_cnt3=0, ctc_cnt4=0, ctc_cnt5=0; //Counts added by HSKIM
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
    
    int tmp_cnt = 0, tmp_cnt2 = 0;
    
    double dsp_time = 0.30, fsp_time = 0.1;
    double step_time = dsp_time + fsp_time;
    unsigned int step_cnt = 0;
    unsigned int step_num = 0;
    double tmp_time = 0, tmp_time2 = 0;

    double z_f1[6], z_f2[6], z_f3[6], z_s1[6], z_s2[6], z_s3[6], z_final1[6];
    double x_init1[6], x_init2[6], x_final1[6], x_final2[6], x_trot_dsp1[6], x_trot_dsp2[6], x_trot_fsp1[6], x_trot_fsp2[6];
    double temp_t1 = 0, temp_t2 = 0;
    double to_height = 0;
    double ts, tf;

//    MatrixNd R = (6,1), A(6,6), P(6,1);
    
    VectorNd R = VectorNd::Zero(6);
    VectorNd P = VectorNd::Zero(6);
    MatrixNd A = MatrixNd::Zero(6,6);
    
    Quaternion QQ;
private:
};

#endif /* CROBOT_H */
