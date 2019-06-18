#ifndef CROBOT_H
#define CROBOT_H


#include "rbdl/rbdl.h"
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
    RigidBodyDynamics::Math::VectorNd BasePosOri;
    RigidBodyDynamics::Math::VectorNd BaseVel;
    RigidBodyDynamics::Math::VectorNd JointAngle;
    RigidBodyDynamics::Math::VectorNd JointVel;
    
private:
};

#endif /* CROBOT_H */

