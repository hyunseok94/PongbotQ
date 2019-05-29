/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   CRobot.h
 * Author: BK Cho
 *
 * Created on August 1, 2018, 3:56 AM
 * 
 * Editor : Yunho Han, 2019, 01, 09
 * 
 */

#ifndef CROBOT_H
#define CROBOT_H


#include "rbdl/rbdl.h"
//#include "rbdl/urdfreader.h"
#include <rbdl/addons/urdfreader/urdfreader.h>



#define PI  3.14159265359
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



using namespace std;
using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;

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

typedef struct Position {
    double F_refX, refX, X;
    double F_refY, refY, Y;
    double F_refZ, refZ, Z;


} POS;

typedef struct Orientation {
    double F_refRoll, refRoll, Roll;
    double F_refPitch, refPithc, Pitch;
    double F_refYaw, refYaw, Yaw;

} ORI;

typedef struct EndPoint {
    //RigidBodyDynamics::Math::VectorNd pos;

    ORI orientation;
    POS pos;
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

    int ID;
} ENDPOINT;

typedef struct LowerbodyKinematic //coordinate of Body 
{
    //Method Kinematics
    double LEG_SIDE_OFFSET = 450.0; //mm 
    double THIGH_LENGTH = 845.99; //300.0; 
    double CALF_LENGTH = 770; //mm 
    double ANKLE_LENGTH = 343.75; //mm 
    double LEG_LENGTH = THIGH_LENGTH + CALF_LENGTH + ANKLE_LENGTH; //mm (THIGH_LENGTH + CALF_LENGTH + ANKLE_LENGTH) 
    //Method Kinematics

    double foot_x_length = 900;
    double foot_y_length = 500;

} LBK;

typedef struct XSystemID //systemID for x-axis 
{
    RigidBodyDynamics::Math::MatrixNd T1; //SystemID for ZMPobserver
    RigidBodyDynamics::Math::MatrixNd T2; //SystemID for ZMPobserver
    RigidBodyDynamics::Math::MatrixNd T1_DSP; //SystemID for ZMPobserver
    RigidBodyDynamics::Math::MatrixNd T2_DSP; //SystemID for ZMPobserver
    RigidBodyDynamics::Math::MatrixNd T1_SSP; //SystemID for ZMPobserver
    RigidBodyDynamics::Math::MatrixNd T2_SSP; //SystemID for ZMPobserver
    double T; //SystemID for ZMPobserver

    double Spring_K; //SystemID for ZMPobserver
    double Damping_C; //SystemID for ZMPobserver

    RigidBodyDynamics::Math::MatrixNd A; //SystemID for ZMPobserver
    RigidBodyDynamics::Math::MatrixNd B; //SystemID for ZMPobserver
    RigidBodyDynamics::Math::MatrixNd C; //SystemID for ZMPobserver
    double D; //SystemID for ZMPobserver

} XID;

typedef struct YSystemID // systemID for y-axis 
{
    RigidBodyDynamics::Math::MatrixNd T1; //SystemID for ZMPobserver
    RigidBodyDynamics::Math::MatrixNd T2; //SystemID for ZMPobserver
    RigidBodyDynamics::Math::MatrixNd T1_DSP; //SystemID for ZMPobserver
    RigidBodyDynamics::Math::MatrixNd T2_DSP; //KSystemID for ZMPobserver
    RigidBodyDynamics::Math::MatrixNd T1_SSP; //SystemID for ZMPobserver
    RigidBodyDynamics::Math::MatrixNd T2_SSP; //KSystemID for ZMPobserver
    double T; //SystemID for ZMPobserver

    double Spring_K; //SystemID for ZMPobserver
    double Damping_C; //SystemID for ZMPobserver

    RigidBodyDynamics::Math::MatrixNd A; //SystemID for ZMPobserver
    RigidBodyDynamics::Math::MatrixNd B; //SystemID for ZMPobserver
    RigidBodyDynamics::Math::MatrixNd C; //SystemID for ZMPobserver
    double D; //SystemID for ZMPobserver

} YID;

typedef struct SystemID //SystemID 
{
    double ComHeight; //mm /
    double robotM; //kg // 1438.28 - 2 X 113 Robot total m - robot foots m
    double robotI; //Moment of Inertia 
    double robotH; //now Robot CoM height    

    XID X;
    YID Y;

} ID;

typedef struct XPreviewControl // PreviewControl for x-axis
{
    MatrixNd ref;
    double m_ref;

    double old_zmp;
    MatrixNd state; //3X1 matrix 
    MatrixNd new_state; //3X1 matrix  
    double E;
    double sum_E;
    double sum_P;
    double U;
    double CoM;

} XPRE;

typedef struct YPreviewControl // PreviewControl for x-axis
{
    MatrixNd ref;
    double m_ref;

    double old_zmp;
    MatrixNd state; //3X1 matrix 
    MatrixNd new_state; //3X1 matrix  
    double E;
    double sum_E;
    double sum_P;
    double U;
    double CoM;

} YPRE;

typedef struct PreviewControl //
{
    double RefTotalTrajSize;
    double RefTotalTime;
    double PreviewTimeSize;
    double PreviewTime;

    RigidBodyDynamics::Math::MatrixNd A;

    RigidBodyDynamics::Math::MatrixNd B;

    RigidBodyDynamics::Math::MatrixNd C;

    RigidBodyDynamics::Math::MatrixNd B_Tilde;

    RigidBodyDynamics::Math::MatrixNd I_Tilde;

    RigidBodyDynamics::Math::MatrixNd F_Tilde;

    RigidBodyDynamics::Math::MatrixNd Q_Tilde;

    RigidBodyDynamics::Math::MatrixNd R;

    RigidBodyDynamics::Math::MatrixNd A_Tilde;

    RigidBodyDynamics::Math::MatrixNd K_Tilde;

    RigidBodyDynamics::Math::MatrixNd G_I;

    RigidBodyDynamics::Math::MatrixNd G_X;

    RigidBodyDynamics::Math::MatrixNd G_P;

    RigidBodyDynamics::Math::MatrixNd A_Tilde_c;

    RigidBodyDynamics::Math::MatrixNd X_Tilde;


    XPRE X;
    YPRE Y;

    double swap_Y_gain;
    double swap_X_gain;

    int count;

} PREVIEWCONTROL;

typedef struct XObserver // Observer for x-axis
{
    RigidBodyDynamics::Math::MatrixNd hat_state; //ZMPobserver
    RigidBodyDynamics::Math::MatrixNd hat_new_state; //ZMPobserver
    RigidBodyDynamics::Math::MatrixNd hat_d_state; //ZMPobserver
    double m_hat_state; //ZMPobserver
    double hat_ZMP; //ZMPobserver
    double m_hat_ZMP; //ZMPobserver
    RigidBodyDynamics::Math::MatrixNd L; //ZMPobserver gain L

    RigidBodyDynamics::Math::MatrixNd L_DSP; //ZMPobserver gain L
    RigidBodyDynamics::Math::MatrixNd L_SSP; //ZMPobserver gain L

} XOB;

typedef struct YObserver // Observer for y-axis
{
    RigidBodyDynamics::Math::MatrixNd hat_state; //ZMPobserver
    RigidBodyDynamics::Math::MatrixNd hat_new_state; //ZMPobserver
    RigidBodyDynamics::Math::MatrixNd hat_d_state; //KZMPobserver
    double m_hat_state; //ZMPobserver
    double hat_ZMP; //ZMPobserver
    double m_hat_ZMP; //ZMPobserver
    RigidBodyDynamics::Math::MatrixNd L; //ZMPobserver gain L

    RigidBodyDynamics::Math::MatrixNd L_DSP; //ZMPobserver gain L
    RigidBodyDynamics::Math::MatrixNd L_SSP; //ZMPobserver gain L

} YOB;

typedef struct Observer //
{
    XOB X;
    YOB Y;

} OBSERVER;

typedef struct XControl // Observer for x-axis
{
    double U; //ZMPControl Input U
    double m_U; //ZMPControl Input U
    double U_Limit; //ZMPControl Input U

    RigidBodyDynamics::Math::MatrixNd K; //ZMPControl gain K
    RigidBodyDynamics::Math::MatrixNd K_DSP; //ZMPControl gain K
    RigidBodyDynamics::Math::MatrixNd K_SSP; //KZMPControl gain K

} XCONTROL;

typedef struct YControl // Observer for y-axis
{
    double U; //ZMPControl Input U
    double m_U; //ZMPControl Input U
    double U_Limit; //ZMPControl Input U

    RigidBodyDynamics::Math::MatrixNd K; //ZMPControl gain K
    RigidBodyDynamics::Math::MatrixNd K_DSP; //ZMPControl gain K
    RigidBodyDynamics::Math::MatrixNd K_SSP; //KZMPControl gain K

} YCONTROL;

typedef struct Controller //
{
    XCONTROL X;
    YCONTROL Y;
    bool dsp2ssp;
    bool ssp2dsp;
    double gainChangeTime;
    double gainChange_t;

    double dsp_gain;
    double ssp_gain;

} CONTROL;

typedef struct zmpx //
{
    double sensor;
    double global;
    double d_global;
    double m_global;
    double offset;
    double Fmargin;
    double Bmargin;

} XZMP;

typedef struct zmpy //
{
    double sensor;
    double global;
    double d_global;
    double m_global;
    double offset;
    double Lmargin;
    double Rmargin;

} YZMP;

typedef struct ZeroMomentPoint //ZMP 
{
    PREVIEWCONTROL previewControl;
    OBSERVER observer;
    CONTROL controller;

    XZMP X;
    YZMP Y;

} ZMP;

typedef struct Offset //Offset 
{
    double x = 50;
    double y;
    double z = 110;
    double m_x;
    double m_y;
    double m_z;

} OFFSET;

typedef struct Step //Step
{
    int current;
    int checking;
    double total;

    double FBstepSize;
    double footHeight;

    enum {
        S_L_F = 1, //start foot L 
        S_R_F = 2 //start foot R
    };
    int start_foot; //start foot

    enum {
        forwardWalking = 1, //walk forward 
        backwardWalking = -1 // back forward
    };
    int walkig_f_or_b; //walkig f or b? 

} STEP;

typedef struct WalkingTime //Step
{
    double sec;
    double periodTime;
    double DSP_ratio;
    double SSP_ratio;
    double SSP_start_time;
    double SSP_end_time;
    double SSP_time;
    double ready;

} WALKINGTIME;

typedef struct Walking //
{
    OFFSET offset;
    STEP step;
    WALKINGTIME time;

    double Y_SWAP = 315;

    enum {
        LSSP = 1,
        RSSP = -1,
        DSP = 0,
        UP = 2
    };
    int m_SP;
    int pre_SP;

    bool Ready;
    bool State;

} WALK;

class CRobot {
public:
    CRobot();
    CRobot(const CRobot& orig);
    virtual ~CRobot();

    void setRobotModel(Model* getModel); //* get Robot Model

    JOINT* joint; //* joints of the robot
    int nDOF; //* number of DOFs of a robot


private:

    double init_t; //* Time for Initialize

    RigidBodyDynamics::Model* m_pModel; //* URDF Model

    /*
    void getRobotState(VectorNd BasePosOri, VectorNd BaseVel, VectorNd JointAngle, VectorNd JointVel); //* get Robot State

    void initializeJoint(double initializetime); //* set all joint position 0, initializetime initialize  
    void setTargetJointPIDgain(double Pgain, double Igain, double Dgain); //* Set a Target PID gain
    void getCurrentJoint(VectorNd Angle, VectorNd Vel); //* Get Current Joint Angle, Vel

    void setTargetRFootPos(double posX, double posY, double posZ); //* Set a Target Position of the RightFoot
    void setTargetJointAngle(double* value);

    void setWalkingTime(double periodTime, double DSPratio);
    void setWalkingInitPostion(double x_Offset, double y_Offset, double z_Offset);
    void setWalkingStep(double steps, int Startfoot, int f_or_b, double FBsize, double footHeight);

    void initializeFTSensorValue(void); //* initialize FTSensorValue
    void getFTSensorValue(VectorNd LFT, VectorNd RFT); //* get FTSensor Information

    void initializeSystemID(void); //* Initialize SystemID
    void setZmpPreview(double previewTime); //* set about ZMP preview control
    void setZmpPreviewTime(double previewTime); //* set about ZMP preview time
    void zmpPreviewControl(void); //* ZMP preview Control
    void initializeZmpControl(void); //* initialize preview Control
    void zmpObserverControl(void); //* ZMP observer Control 
    void zmpCalculation(void); //* Get ZMP Calculation
    void globalZmpUpdate(void); //* Update Glabal ZMP

    void setZmpSwapPlan(void); //* Planning ZMP Reference for swap
    void setZmpOneStepPlan(void); //* Planning ZMP Reference for swap
    void setZmpFBPlan(void); //* Planning ZMP Reference for Forward or Back

    void walkingPlan(void); //* Planning Foot X,Z Reference for Walking 
    void walkingReady(double readytime); //* Walking Ready

    void ComputeTorqueControl(void); //* ComputedTorqueControl


    BASE base; //* coordinate of Body    
    JOINT* joint; //* joints of the robot
    ENDPOINT RFoot, LFoot; //* Variable of Foot
    WALK walking; //* Variable of Walking
    ZMP zmp; //* About ZMP, ZMP preview control, ZMP Observer and Controller

    int mode; //* Position Control or Torque Control
    bool initposition; //* InitPosition
    double cosWave(double Amp, double period, double time, double int_pos); //* CosWave generation
    bool computeIK(double *out, double x, double y, double z, double roll, double pitch, double yaw); //* Robot Method's Lower body IK
    bool computeFK(double *currentFoot, VectorNd jointstate); //* Robot Method's Lower body FK

    double CXK, CXD, OXK, OXD;
    double CYK, CYD, OYK, OYD;
    double CZK, CZD, OZK, OZD;

private:

    double init_t; //* Time for Initialize

    RigidBodyDynamics::Model* m_pModel; //* URDF Model
    int nDOF; //* number of DOFs of a robot
    int nElementofEnd = 12; //* number of End Effect data of a robot

    MatrixNd setTransform(VectorNd pos, VectorNd orientation);

    LBK lowerbody; //* LowerBody Kinematics

    double FTsensorFxLPF; //* FTsensor Low Pass Filter
    double FTsensorMxLPF; //* FTsensor Low Pass Filter
    double FTsensorFyLPF; //* FTsensor Low Pass Filter
    double FTsensorMyLPF; //* FTsensor Low Pass Filter
    double FTsensorFzLPF; //* FTsensor Low Pass Filter
    double FTsensorMzLPF; //* FTsensor Low Pass Filter



    RigidBodyDynamics::Math::MatrixNd ZMP_DARE(RigidBodyDynamics::Math::MatrixNd& A, RigidBodyDynamics::Math::MatrixNd& B,
            RigidBodyDynamics::Math::MatrixNd& Q, RigidBodyDynamics::Math::MatrixNd& R);
    Eigen::EigenSolver<RigidBodyDynamics::Math::MatrixNd> eZMPSolver;

    ID systemID; //* System ID about the robot

    void getSystemID(void); //* Get SystemID
    void zmpObserver(void); //* ZMP observer
    void zmpController(void); //* ZMP Controller
    void zmpControlGainChange(void); //* ZMP GainChange

    RigidBodyDynamics::Math::VectorNd RobotState; //* Robotstate
    RigidBodyDynamics::Math::VectorNd RobotStatedot; //* Robotstatedot

    RigidBodyDynamics::Math::MatrixNd PseudoInverse(RigidBodyDynamics::Math::MatrixNd mat);

    VectorNd outJF;

    MatrixNd J_Rfoot; // = MatrixNd::Zero(6, m_pModel->dof_count);
    MatrixNd J_Lfoot; // = MatrixNd::Zero(6, m_pModel->dof_count);
    MatrixNd J_Base; //  = MatrixNd::Zero(6, m_pModel->dof_count);
    MatrixNd preJ_Rfoot; // = MatrixNd::Zero(6, m_pModel->dof_count);
    MatrixNd preJ_Lfoot; // = MatrixNd::Zero(6, m_pModel->dof_count);
    MatrixNd preJ_Base; //  = MatrixNd::Zero(6, m_pModel->dof_count);
    MatrixNd dotJ_Rfoot; // = MatrixNd::Zero(6, m_pModel->dof_count);
    MatrixNd dotJ_Lfoot; // = MatrixNd::Zero(6, m_pModel->dof_count);
    MatrixNd dotJ_Base; //  = MatrixNd::Zero(6, m_pModel->dof_count);
     * */

};

#endif /* CROBOT_H */

