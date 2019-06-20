
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
    m_pModel = getModel;
    m_pModel->gravity = Vector3d(0., 0., -9.81);
    nDOF = m_pModel->dof_count - 6; //* get Degree of freedom, Except x,y,z,roll,pitch,yaw of the robot
    joint = new JOINT[nDOF]; //* only joint of the robot excepting x,y,z,roll,pitch,yaw of the robot 
    RobotState = VectorNd::Zero(m_pModel->q_size);
    RobotStatedot = VectorNd::Zero(m_pModel->dof_count);
    RobotState2dot = VectorNd::Zero(m_pModel->dof_count);
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
    
    QQ << 0,0,0,1;
    m_pModel->SetQuaternion(base.ID, QQ, RobotState);
    
    goal_EP << 0,0.218,-0.45,  0,-0.218,-0.45,  0.7,0.218,-0.45,  0.7,-0.218,-0.45;
//    Kp_EP << 4000,4000,14000,4000,4000,14000,4000,4000,14000,4000,4000,14000;//2,3,10, 2,3,10, 2,3,10, 2,3,10;
//    Kd_EP << 100,100,400,100,100,400,100,100,400,100,100,400;
  
    Kp_EP << 5000,5000,15000,5000,5000,15000,5000,5000,15000,5000,5000,15000;//2,3,10, 2,3,10, 2,3,10, 2,3,10;
    Kd_EP << 200,200,600,200,200,600,200,200,600,200,200,600;
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
    RobotState(AXIS_Roll)  = base.currentRoll;
    RobotState(AXIS_Pitch) = base.currentPitch;
    RobotState(AXIS_Yaw)   = base.currentYaw;
    RobotStatedot(AXIS_X) = base.currentXvel;
    RobotStatedot(AXIS_Y) = base.currentYvel;
    RobotStatedot(AXIS_Z) = base.currentZvel;
    RobotStatedot(AXIS_Roll)  = base.currentRollvel;
    RobotStatedot(AXIS_Pitch) = base.currentPitchvel;
    RobotStatedot(AXIS_Yaw)   = base.currentYawvel;

    for (int nJoint = 0; nJoint < nDOF; nJoint++) {
        RobotState(6 + nJoint) = joint[nJoint].currentAngle;
        RobotStatedot(6 + nJoint) = joint[nJoint].currentVel;
    }
    
    EP_RL = CalcBodyToBaseCoordinates(*m_pModel, RobotState, RL.ID, EP_OFFSET_RL, true);
    EP_RR = CalcBodyToBaseCoordinates(*m_pModel, RobotState, RR.ID, EP_OFFSET_RR, true);
    EP_FL = CalcBodyToBaseCoordinates(*m_pModel, RobotState, FL.ID, EP_OFFSET_FL, true);
    EP_FR = CalcBodyToBaseCoordinates(*m_pModel, RobotState, FR.ID, EP_OFFSET_FR, true);
    
    actual_EP.block<3,1>(0,0) = EP_RL;
    actual_EP.block<3,1>(3,0) = EP_RR;
    actual_EP.block<3,1>(6,0) = EP_FL;
    actual_EP.block<3,1>(9,0) = EP_FR;

    CalcPointJacobian(*m_pModel, RobotState, RL.ID,     EP_OFFSET_RL,   J_RL, true);   
    CalcPointJacobian(*m_pModel, RobotState, RR.ID,     EP_OFFSET_RR,   J_RR, true);
    CalcPointJacobian(*m_pModel, RobotState, FL.ID,     EP_OFFSET_FL,   J_FL, true);
    CalcPointJacobian(*m_pModel, RobotState, FR.ID,     EP_OFFSET_FR,   J_FR, true);
    CalcPointJacobian6D(*m_pModel, RobotState, front_body.ID, Originbase,   J_FRONT_BODY, true);
    CalcPointJacobian6D(*m_pModel, RobotState, base.ID,   Originbase, J_BASE,  true);
 
    J_A.block<6,19>(0,0) =  J_BASE;
    J_A.block<1,19>(6,0) =  J_FRONT_BODY.block<1,19>(2,0); // only yaw
    J_A.block<3,19>(7,0) =  J_RL;
    J_A.block<3,19>(10,0) = J_RR;
    J_A.block<3,19>(13,0) = J_FL;
    J_A.block<3,19>(16,0) = J_FR;
    
    x_dot = J_A*RobotStatedot;
    
    actual_EP_vel = x_dot.block<12,1>(7,0);
    
    // ====================== Get dJdQ for CTC ===================== //
 
    base_dJdQ = CalcPointAcceleration6D(*m_pModel,RobotState,RobotStatedot,ddqZero,base.ID,Originbase,true);
    FRONT_BODY_dJdQ = CalcPointAcceleration6D(*m_pModel,RobotState,RobotStatedot,ddqZero,front_body.ID,Originbase,true);
    RL_dJdQ = CalcPointAcceleration(*m_pModel,RobotState,RobotStatedot,ddqZero,RL.ID,EP_OFFSET_RL,true);
    RR_dJdQ = CalcPointAcceleration(*m_pModel,RobotState,RobotStatedot,ddqZero,RR.ID,EP_OFFSET_RR,true);
    FL_dJdQ = CalcPointAcceleration(*m_pModel,RobotState,RobotStatedot,ddqZero,FL.ID,EP_OFFSET_FL,true);
    FR_dJdQ = CalcPointAcceleration(*m_pModel,RobotState,RobotStatedot,ddqZero,FR.ID,EP_OFFSET_FR,true);
 
    dJdQ.block<6,1>(0,0) = base_dJdQ;
    dJdQ.block<1,1>(6,0) = FRONT_BODY_dJdQ.block<1,1>(2,0);
    dJdQ.block<3,1>(7,0) = RL_dJdQ;
    dJdQ.block<3,1>(10,0) = RR_dJdQ;
    dJdQ.block<3,1>(13,0) = FL_dJdQ;
    dJdQ.block<3,1>(16,0) = FR_dJdQ;
    
    Fc << 0, 0, 0, 0, 0, 0, 0 ,0, 0, Fc_RL,  0, 0, Fc_RR, 0, 0, Fc_FL, 0, 0, Fc_FR;
    
    
    for(unsigned int i=0;i<7;++i){
        x_2dot_cp[i] = 0;
        
        if(i == 6){ // waist
            x_2dot_cp[i] = 0 + 10000*(0 - RobotState[12]) + 100*(0 - RobotStatedot[12]);
//            cout << "x_2dot_cp[6] = "<< x_2dot_cp[6] << endl;
        }
    }
    
    target_EP.block<12,1>(0,0) = goal_EP;     
    
    for(unsigned int i=0;i<12;++i){
        x_2dot_cp[i+7] = target_EP_acc[i] + Kp_EP[i]*(target_EP[i] - actual_EP[i]) + Kd_EP[i]*(target_EP_vel[i] - actual_EP_vel[i]);
        
    }
     
    RobotState2dot = J_A.inverse()*(x_2dot_cp - dJdQ);
    
   
    CompositeRigidBodyAlgorithm(*m_pModel, RobotState, M_term, true);

    NonlinearEffects(*m_pModel, RobotState, RobotStatedot, hatNonLinearEffects);

    NonlinearEffects(*m_pModel, RobotState, VectorNd::Zero(m_pModel->dof_count), G_term);

    C_term = hatNonLinearEffects - G_term;
    
    CTC_Torque = M_term*RobotState2dot + C_term + G_term + J_A.transpose()*Fc;
   

    for (int nJoint = 0; nJoint < nDOF; nJoint++) {
        joint[nJoint].torque = CTC_Torque(6 + nJoint);
    }
}

void CRobot::FTsensorTransformation()
{
    RL.T_matrix = CalcBodyWorldOrientation(*m_pModel, RobotState, RL.ID, true);
    RR.T_matrix = CalcBodyWorldOrientation(*m_pModel, RobotState, RR.ID, true);
    FL.T_matrix = CalcBodyWorldOrientation(*m_pModel, RobotState, FL.ID, true);
    FR.T_matrix = CalcBodyWorldOrientation(*m_pModel, RobotState, FR.ID, true);
}