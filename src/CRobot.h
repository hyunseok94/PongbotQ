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

    //Variables
    BASE base; //* coordinate of Body
    JOINT* joint; //* joints of the robot
    ENDPOINT FR, FL, RR, RL, front_body;
    int nDOF; //* number of DOFs of a robot

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
//    VectorNd init_goal_EP = VectorNd::Zero(12);
    VectorNd trot_goal_EP = VectorNd::Zero(12);
    double Fc_RL = 0, Fc_RR = 0, Fc_FL = 0, Fc_FR = 0;

    Quaternion QQ;
private:
};

#endif /* CROBOT_H */
