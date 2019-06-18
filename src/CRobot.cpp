
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
    Quaternion QQ(0, 0, 0, 1);

    m_pModel = getModel;
    m_pModel->gravity = Vector3d(0., 0., -9.81);
    nDOF = m_pModel->dof_count - 6; //* get Degree of freedom, Except x,y,z,roll,pitch,yaw of the robot
    joint = new JOINT[nDOF]; //* only joint of the robot excepting x,y,z,roll,pitch,yaw of the robot 
    RobotState = VectorNd::Zero(m_pModel->dof_count);
    RobotStatedot = VectorNd::Zero(m_pModel->dof_count);
    BasePosOri = VectorNd::Zero(6);
    BaseVel = VectorNd::Zero(6);
    JointAngle = VectorNd::Zero(nDOF);
    JointVel = VectorNd::Zero(nDOF);

    base.ID = m_pModel->GetBodyId("REAR_BODY");
    FR.ID = m_pModel->GetBodyId("FR_CALF");
    FL.ID = m_pModel->GetBodyId("FL_CALF");
    RR.ID = m_pModel->GetBodyId("RR_CALF");
    RL.ID = m_pModel->GetBodyId("RR_CALF");
    m_pModel->SetQuaternion(base.ID, QQ, RobotState);
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
        RobotState(6 + nJoint) = joint[nJoint].currentAngle;
        RobotStatedot(6 + nJoint) = joint[nJoint].currentVel;
    }

    MatrixNd M_term = MatrixNd::Zero(m_pModel->dof_count, m_pModel->dof_count);
    CompositeRigidBodyAlgorithm(*m_pModel, RobotState, M_term, true);

    VectorNd hatNonLinearEffects = VectorNd::Zero(m_pModel->dof_count);
    NonlinearEffects(*m_pModel, RobotState, RobotStatedot, hatNonLinearEffects);

    VectorNd G_term = VectorNd::Zero(m_pModel->dof_count);
    NonlinearEffects(*m_pModel, RobotState, VectorNd::Zero(m_pModel->dof_count), G_term);

    VectorNd C_term = VectorNd::Zero(m_pModel->dof_count);
    C_term = hatNonLinearEffects - G_term;

    for (int nJoint = 0; nJoint < nDOF; nJoint++) {
        joint[nJoint].torque = G_term(6 + nJoint);
    }
}

void CRobot::FTsensorTransformation()
{
    RL.T_matrix = CalcBodyWorldOrientation(*m_pModel, RobotState, RL.ID, true);
    RR.T_matrix = CalcBodyWorldOrientation(*m_pModel, RobotState, RR.ID, true);
    FL.T_matrix = CalcBodyWorldOrientation(*m_pModel, RobotState, FL.ID, true);
    FR.T_matrix = CalcBodyWorldOrientation(*m_pModel, RobotState, FR.ID, true);
}