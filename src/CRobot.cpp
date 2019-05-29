/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   CRobot.cpp
 * Author: BK Cho
 * 
 * Created on August 1, 2018, 3:56 AM
 * 
 * Editor : Yunho Han, 2019, 01, 09
 * 
 */


#include "CRobot.h"


CRobot::CRobot() {
    /*
    this->LFoot.current = RigidBodyDynamics::Math::VectorNd::Zero(6);
    this->LFoot.pre     = RigidBodyDynamics::Math::VectorNd::Zero(6);
    this->LFoot.vel     = RigidBodyDynamics::Math::VectorNd::Zero(6);
    this->LFoot.prevel     = RigidBodyDynamics::Math::VectorNd::Zero(6);
    this->LFoot.acc     = RigidBodyDynamics::Math::VectorNd::Zero(6);
    this->LFoot.preacc     = RigidBodyDynamics::Math::VectorNd::Zero(6);
    
    this->LFoot.refpos     = RigidBodyDynamics::Math::VectorNd::Zero(6);
    this->LFoot.prerefpos     = RigidBodyDynamics::Math::VectorNd::Zero(6);
    this->LFoot.refvel     = RigidBodyDynamics::Math::VectorNd::Zero(6);
    this->LFoot.prerefvel     = RigidBodyDynamics::Math::VectorNd::Zero(6);
    this->LFoot.refacc     = RigidBodyDynamics::Math::VectorNd::Zero(6);
    
    this->RFoot.current = RigidBodyDynamics::Math::VectorNd::Zero(6);
    this->RFoot.pre     = RigidBodyDynamics::Math::VectorNd::Zero(6);
    this->RFoot.vel     = RigidBodyDynamics::Math::VectorNd::Zero(6);
    this->RFoot.prevel     = RigidBodyDynamics::Math::VectorNd::Zero(6);
    this->RFoot.acc     = RigidBodyDynamics::Math::VectorNd::Zero(6);
    this->RFoot.preacc     = RigidBodyDynamics::Math::VectorNd::Zero(6);
    
    this->RFoot.refpos     = RigidBodyDynamics::Math::VectorNd::Zero(6);
    this->RFoot.prerefpos     = RigidBodyDynamics::Math::VectorNd::Zero(6);
    this->RFoot.refvel     = RigidBodyDynamics::Math::VectorNd::Zero(6);
    this->RFoot.prerefvel     = RigidBodyDynamics::Math::VectorNd::Zero(6);
    this->RFoot.refacc     = RigidBodyDynamics::Math::VectorNd::Zero(6);
    
    LFoot.pos.F_refX = 0; 
    LFoot.pos.F_refY = 0;
    LFoot.pos.F_refZ = 0;
    LFoot.orientation.F_refRoll = 0; 
    LFoot.orientation.F_refPitch = 0;
    LFoot.orientation.F_refYaw = 0;
    
    RFoot.pos.F_refX = 0; 
    RFoot.pos.F_refY = 0;
    RFoot.pos.F_refZ = 0;
    RFoot.orientation.F_refRoll = 0; 
    RFoot.orientation.F_refPitch = 0;
    RFoot.orientation.F_refYaw = 0;

    initposition = false;
    walking.Ready = false;
    walking.State = false;
    init_t = 0;
    walking.time.ready = 0;
    walking.step.walkig_f_or_b = walking.step.backwardWalking; 
    mode = POSITIONCONTROLMODE;
    */
    
}

CRobot::CRobot(const CRobot& orig) {
}

CRobot::~CRobot() {
}

void CRobot::setRobotModel(Model* getModel) {

    m_pModel = getModel;

    m_pModel->gravity = Vector3d(0., 0., -9.81);
    nDOF = m_pModel->dof_count - 6; //* get Degree of freedom, Except x,y,z,roll,pitch,yaw of the robot
    joint = new JOINT[nDOF]; //* only joint of the robot excepting x,y,z,roll,pitch,yaw of the robot 

    /*
    RobotState    = VectorNd::Zero(m_pModel->dof_count);
    RobotStatedot = VectorNd::Zero(m_pModel->dof_count);

    
    outJF = VectorNd::Zero(m_pModel->dof_count);
    
    J_Rfoot = MatrixNd::Zero(6, m_pModel->dof_count);
    J_Lfoot = MatrixNd::Zero(6, m_pModel->dof_count);
    J_Base  = MatrixNd::Zero(6, m_pModel->dof_count);
    preJ_Rfoot = MatrixNd::Zero(6, m_pModel->dof_count);
    preJ_Lfoot = MatrixNd::Zero(6, m_pModel->dof_count);
    preJ_Base  = MatrixNd::Zero(6, m_pModel->dof_count);
    dotJ_Rfoot = MatrixNd::Zero(6, m_pModel->dof_count);
    dotJ_Lfoot = MatrixNd::Zero(6, m_pModel->dof_count);
    dotJ_Base  = MatrixNd::Zero(6, m_pModel->dof_count);
    */
}
/*
void CRobot::getRobotState(VectorNd BasePosOri, VectorNd BaseVel, VectorNd JointAngle, VectorNd JointVel) {
    
    base.currentX     = BasePosOri(AXIS_X);
    base.currentY     = BasePosOri(AXIS_Y);
    base.currentZ     = BasePosOri(AXIS_Z);    
    base.currentRoll  = BasePosOri(AXIS_Roll);
    base.currentPitch = BasePosOri(AXIS_Pitch);
    base.currentYaw   = BasePosOri(AXIS_Yaw);
    
    base.currentXvel     = BaseVel(AXIS_X);
    base.currentYvel     = BaseVel(AXIS_Y);
    base.currentZvel     = BaseVel(AXIS_Z);    
    base.currentRollvel  = BaseVel(AXIS_Roll);
    base.currentPitchvel = BaseVel(AXIS_Pitch);
    base.currentYawvel   = BaseVel(AXIS_Yaw);
    
    getCurrentJoint(JointAngle,JointVel);
    

}

void CRobot::initializeJoint(double initializetime) {
    
        
        if (init_t < initializetime) 
        {
            for (int nJoint = 0; nJoint < nDOF; nJoint++) {
                joint[nJoint].refAngle = cosWave(-joint[nJoint].currentAngle, initializetime, init_t, joint[nJoint].currentAngle);
            }
            init_t += tasktime;
        } 
        
        else 
        {
            init_t = 0;
            initposition = true;
        }
}

void CRobot::setTargetJointPIDgain(double Pgain, double Igain, double Dgain) {
    for (int nJoint = 0; nJoint < nDOF; nJoint++) {
        joint[nJoint].gain_P = Pgain;
        joint[nJoint].gain_I = Igain;
        joint[nJoint].gain_D = Dgain;
    }
}//Setting is your thinking

void CRobot::getCurrentJoint(VectorNd Angle, VectorNd Vel) {
    for (int nJoint = 0; nJoint < nDOF; nJoint++) {
        joint[nJoint].currentAngle = Angle(nJoint);
        joint[nJoint].currentVel   = Vel(nJoint);
    }   
}

void CRobot::setTargetRFootPos(double posX, double posY, double posZ) {
    RFoot.Target(0) = posX;
    RFoot.Target(1) = posY;
    RFoot.Target(2) = posZ;
}

void CRobot::setTargetJointAngle(double *value) {
    joint[0].refAngle = *value;
}

void CRobot::setWalkingTime(double periodTime, double DSPratio) {
    walking.time.periodTime = periodTime;
    walking.time.DSP_ratio = DSPratio;
    walking.time.SSP_ratio = 1-walking.time.DSP_ratio;
    
    walking.time.SSP_start_time = walking.time.periodTime*walking.time.DSP_ratio/2;
    walking.time.SSP_time = walking.time.periodTime*walking.time.SSP_ratio;
    walking.time.SSP_end_time = walking.time.SSP_start_time+walking.time.SSP_time;
}

void CRobot::setWalkingInitPostion(double x_Offset, double y_Offset, double z_Offset) {
    walking.offset.x = x_Offset;
    walking.offset.y = y_Offset;
    walking.offset.z = z_Offset;
}

void CRobot::setWalkingStep(double steps, int Startfoot, int f_or_b, double FBsize, double footHeight)
{
    walking.step.total = steps;
    walking.step.start_foot = Startfoot;
    
    walking.step.footHeight = footHeight;
    LFoot.pos.refZ = walking.step.footHeight;
    RFoot.pos.refZ = walking.step.footHeight;
    
    walking.step.walkig_f_or_b *= -f_or_b;
    walking.step.FBstepSize = walking.step.walkig_f_or_b*FBsize;
    
    LFoot.pos.refX = walking.step.FBstepSize;
    RFoot.pos.refX = walking.step.FBstepSize;
}
void CRobot::initializeFTSensorValue(void) {
    LFoot.ftSensor.Fx = 0;
    LFoot.ftSensor.Fy = 0;    
    LFoot.ftSensor.Fz = 0;    

    LFoot.ftSensor.Mx = 0;
    LFoot.ftSensor.My = 0;    
    LFoot.ftSensor.Mz = 0;
    
    RFoot.ftSensor.Fx = 0;
    RFoot.ftSensor.Fy = 0;    
    RFoot.ftSensor.Fz = 0;
    
    RFoot.ftSensor.Mx = 0;
    RFoot.ftSensor.My = 0;    
    RFoot.ftSensor.Mz = 0;

    FTsensorFxLPF = 0.9;
    FTsensorMxLPF = 0.9;
    FTsensorFyLPF = 0.9;
    FTsensorMyLPF = 0.9;
    FTsensorFzLPF = 0.9;
    FTsensorMzLPF = 0.9;
}


void CRobot::getFTSensorValue(VectorNd LFT,VectorNd RFT) {
    LFoot.ftSensor.Fx = FTsensorFxLPF*LFoot.ftSensor.Fx + (1-FTsensorFxLPF)*LFT(0);
    LFoot.ftSensor.Fy = FTsensorFyLPF*LFoot.ftSensor.Fy + (1-FTsensorFyLPF)*LFT(1);    
    LFoot.ftSensor.Fz = FTsensorFzLPF*LFoot.ftSensor.Fz + (1-FTsensorFzLPF)*LFT(2);    

    LFoot.ftSensor.Mx = FTsensorMxLPF*LFoot.ftSensor.Mx + (1-FTsensorMxLPF)*LFT(3);
    LFoot.ftSensor.My = FTsensorMyLPF*LFoot.ftSensor.My + (1-FTsensorMyLPF)*LFT(4);    
    LFoot.ftSensor.Mz = FTsensorMzLPF*LFoot.ftSensor.Mz + (1-FTsensorMzLPF)*LFT(5);
    
    RFoot.ftSensor.Fx = FTsensorFxLPF*RFoot.ftSensor.Fx + (1-FTsensorFxLPF)*RFT(0);
    RFoot.ftSensor.Fy = FTsensorFyLPF*RFoot.ftSensor.Fy + (1-FTsensorFyLPF)*RFT(1);    
    RFoot.ftSensor.Fz = FTsensorFzLPF*RFoot.ftSensor.Fz + (1-FTsensorFzLPF)*RFT(2);
    
    RFoot.ftSensor.Mx = FTsensorMxLPF*RFoot.ftSensor.Mx + (1-FTsensorMxLPF)*RFT(3);
    RFoot.ftSensor.My = FTsensorMyLPF*RFoot.ftSensor.My + (1-FTsensorMyLPF)*RFT(4);    
    RFoot.ftSensor.Mz = FTsensorMzLPF*RFoot.ftSensor.Mz + (1-FTsensorMzLPF)*RFT(5); 

//    printf("FX %lf\t %lf\n",LFoot.ftSensor.Fx,RFoot.ftSensor.Fx);
//    printf("FY %lf\t %lf\n",LFoot.ftSensor.Fy,RFoot.ftSensor.Fy);
//    printf("FZ %lf\t %lf\n",LFoot.ftSensor.Fz,RFoot.ftSensor.Fz);
//    printf("MX %lf\t %lf\n",LFoot.ftSensor.Mx,RFoot.ftSensor.Mx);
//    printf("MY %lf\t %lf\n",LFoot.ftSensor.My,RFoot.ftSensor.My);
//    printf("MZ %lf\t %lf\n",LFoot.ftSensor.Mz,RFoot.ftSensor.Mz);    
}

void CRobot::initializeSystemID() {
    
    systemID.ComHeight = 1730;
    systemID.robotH = (systemID.ComHeight-walking.offset.z)*0.001;  //mm->m
    systemID.robotM = 1416.28;
    systemID.robotI = systemID.robotM*(systemID.robotH)*(systemID.robotH);
    
    systemID.X.T1 = MatrixNd::Zero(1, 2); //Kookmin.Univ SystemID for ZMPobserver
    systemID.X.T2 = MatrixNd::Zero(1, 2); //Kookmin.Univ SystemID for ZMPobserver
    
    systemID.Y.T1 = MatrixNd::Zero(1, 2); //Kookmin.Univ SystemID for ZMPobserver
    systemID.Y.T2 = MatrixNd::Zero(1, 2); //Kookmin.Univ SystemID for ZMPobserver
    
    systemID.X.T1_DSP = MatrixNd::Zero(1, 2); //Kookmin.Univ SystemID for ZMPobserver
    systemID.X.T2_DSP = MatrixNd::Zero(1, 2); //Kookmin.Univ SystemID for ZMPobserver
    
    systemID.Y.T1_DSP = MatrixNd::Zero(1, 2); //Kookmin.Univ SystemID for ZMPobserver
    systemID.Y.T2_DSP = MatrixNd::Zero(1, 2); //Kookmin.Univ SystemID for ZMPobserver
    
    systemID.X.T1_SSP = MatrixNd::Zero(1, 2); //Kookmin.Univ SystemID for ZMPobserver
    systemID.X.T2_SSP = MatrixNd::Zero(1, 2); //Kookmin.Univ SystemID for ZMPobserver
    
    systemID.Y.T1_SSP = MatrixNd::Zero(1, 2); //Kookmin.Univ SystemID for ZMPobserver
    systemID.Y.T2_SSP = MatrixNd::Zero(1, 2); //Kookmin.Univ SystemID for ZMPobserver
    
    systemID.X.T1_DSP << 59.8, -393.60;//DSP_X
    systemID.X.T2_DSP << 59.98, -378.10;//DSP_X
    
    systemID.Y.T1_DSP << 24.3, -247.1;//DSP_Y
    systemID.Y.T2_DSP << 24.45,-201.8;//DSP_Y
    
    systemID.X.T1_SSP << 47.65, -394.1;//SSP_X
    systemID.X.T2_SSP << 48.02, -295.2;//SSP_X
       
    systemID.Y.T1_SSP << 44.41, -44.54;//SSP_Y
    systemID.Y.T2_SSP << 45.01, -32.83;//SSP_Y
    
    
    
    systemID.X.A = MatrixNd::Zero(2, 2);
    systemID.X.B = MatrixNd::Zero(2, 1);
    systemID.X.C = MatrixNd::Zero(1, 2);
    systemID.X.D = 0;
    
    systemID.Y.A = MatrixNd::Zero(2, 2);
    systemID.Y.B = MatrixNd::Zero(2, 1);
    systemID.Y.C = MatrixNd::Zero(1, 2);
    systemID.Y.D = 0;
    
    systemID.X.T1 = systemID.X.T1_DSP;//DSP_X
    systemID.X.T2 = systemID.X.T2_DSP;//DSP_X
        
    systemID.Y.T1 = systemID.Y.T1_DSP;//DSP_Y
    systemID.Y.T2 = systemID.Y.T2_DSP;//DSP_Y
}

void CRobot::setZmpPreview(double previewTime){
    
    setZmpPreviewTime(previewTime);
    
    double T = (double)tasktime;

    zmp.previewControl.A = MatrixNd::Zero(3, 3);
    zmp.previewControl.A << 1, T, T*T/2.0\
                           ,0, 1, T\
                           ,0, 0, 1;


    zmp.previewControl.B = MatrixNd::Zero(3, 1);
    zmp.previewControl.B << T*T*T/6.0\
                           ,T*T/2.0\
                           ,T;

    zmp.previewControl.C = MatrixNd::Zero(1, 3);
    zmp.previewControl.C << 1 ,0 ,-(systemID.robotH)/9.81;
    
    zmp.previewControl.B_Tilde = MatrixNd::Zero(4, 1);
    zmp.previewControl.B_Tilde.block(0, 0, 1, 1) = zmp.previewControl.C* zmp.previewControl.B;
    zmp.previewControl.B_Tilde.block(1, 0, 3, 1) = zmp.previewControl.B;
    
    zmp.previewControl.I_Tilde = MatrixNd::Zero(4, 1);
    zmp.previewControl.I_Tilde << 1, 0, 0, 0;
    
    zmp.previewControl.F_Tilde = MatrixNd::Zero(4, 3);
    zmp.previewControl.F_Tilde.block(0, 0, 1, 3) = zmp.previewControl.C * zmp.previewControl.A;
    zmp.previewControl.F_Tilde.block(1, 0, 3, 3) = zmp.previewControl.A;

    
    zmp.previewControl.Q_Tilde = MatrixNd::Zero(4, 4);

    zmp.previewControl.Q_Tilde.block(0, 0, 1, 1) = MatrixNd::Identity(1, 1);
    zmp.previewControl.Q_Tilde.block(1, 1, 3, 3) = MatrixNd::Zero(3, 3);
    
    zmp.previewControl.R = MatrixNd::Identity(1, 1)*0.000001;

    zmp.previewControl.A_Tilde = MatrixNd::Zero(4, 4);
    zmp.previewControl.A_Tilde.block(0, 0, 4, 1) = zmp.previewControl.I_Tilde;
    zmp.previewControl.A_Tilde.block(0, 1, 4, 3) = zmp.previewControl.F_Tilde;
    
    zmp.previewControl.K_Tilde = MatrixNd::Zero(4, 4);
    zmp.previewControl.K_Tilde = ZMP_DARE(zmp.previewControl.A_Tilde, zmp.previewControl.B_Tilde, zmp.previewControl.Q_Tilde, zmp.previewControl.R);
    
    zmp.previewControl.G_I = MatrixNd::Zero(1, 1);
    zmp.previewControl.G_I = (zmp.previewControl.R + zmp.previewControl.B_Tilde.transpose()*zmp.previewControl.K_Tilde*zmp.previewControl.B_Tilde).inverse()*zmp.previewControl.B_Tilde.transpose()*zmp.previewControl.K_Tilde*zmp.previewControl.I_Tilde;
    
    zmp.previewControl.G_X = MatrixNd::Zero(1, 3);
    zmp.previewControl.G_X = (zmp.previewControl.R + zmp.previewControl.B_Tilde.transpose()*zmp.previewControl.K_Tilde*zmp.previewControl.B_Tilde).inverse()*zmp.previewControl.B_Tilde.transpose()*zmp.previewControl.K_Tilde*zmp.previewControl.F_Tilde;   

    zmp.previewControl.A_Tilde_c = MatrixNd::Zero(4, 4);
    zmp.previewControl.A_Tilde_c = zmp.previewControl.A_Tilde - zmp.previewControl.B_Tilde*(zmp.previewControl.R + zmp.previewControl.B_Tilde.transpose()*zmp.previewControl.K_Tilde*zmp.previewControl.B_Tilde).inverse()*zmp.previewControl.B_Tilde.transpose()*zmp.previewControl.K_Tilde*zmp.previewControl.A_Tilde; 
    
    zmp.previewControl.X_Tilde = MatrixNd::Zero(4, 1);
    
    zmp.previewControl.Y.ref = MatrixNd::Zero(1, zmp.previewControl.RefTotalTrajSize);
    zmp.previewControl.Y.m_ref = 0;
    zmp.previewControl.swap_Y_gain = lowerbody.LEG_SIDE_OFFSET/315.0;
    zmp.previewControl.Y.state = MatrixNd::Zero(3, 1);	//3X1 matrix
    zmp.previewControl.Y.new_state = MatrixNd::Zero(3, 1);    
    zmp.previewControl.Y.E = 0;
    zmp.previewControl.Y.sum_E = 0;
    zmp.previewControl.Y.sum_P = 0;
    zmp.previewControl.Y.old_zmp = 0;
    zmp.previewControl.Y.U = 0;
    zmp.previewControl.Y.CoM = 0;
    
    zmp.previewControl.X.ref = MatrixNd::Zero(1, zmp.previewControl.RefTotalTrajSize);
    zmp.previewControl.X.m_ref = 0;

    zmp.previewControl.X.state = MatrixNd::Zero(3, 1);	//3X1 matrix
    zmp.previewControl.X.new_state = MatrixNd::Zero(3, 1);    
    zmp.previewControl.X.E = 0;
    zmp.previewControl.X.sum_E = 0;
    zmp.previewControl.X.sum_P = 0;
    zmp.previewControl.X.old_zmp = 0;
    zmp.previewControl.X.U = 0;
    zmp.previewControl.X.CoM = 0;
    
    zmp.previewControl.count = 0;

}

void CRobot::setZmpPreviewTime(double previewTime){
    
    zmp.previewControl.PreviewTime = previewTime;
    
    zmp.previewControl.PreviewTimeSize = zmp.previewControl.PreviewTime*(double)onesecSize;
    
    zmp.previewControl.RefTotalTrajSize = (zmp.previewControl.PreviewTime\
                                           + walking.step.total * walking.time.periodTime\
                                           + 3*zmp.previewControl.PreviewTime)*(double)onesecSize;
    
    zmp.previewControl.RefTotalTime = zmp.previewControl.RefTotalTrajSize/(double)onesecSize;
}

void CRobot::zmpPreviewControl(void) {
    
    if(zmp.previewControl.count < zmp.previewControl.RefTotalTrajSize - zmp.previewControl.PreviewTimeSize)
    {
        zmp.previewControl.Y.state = zmp.previewControl.Y.new_state;
        zmp.previewControl.X.state = zmp.previewControl.X.new_state;
        
        zmp.previewControl.Y.E = zmp.previewControl.Y.old_zmp - zmp.previewControl.Y.ref(0,zmp.previewControl.count);
        zmp.previewControl.X.E = zmp.previewControl.X.old_zmp - zmp.previewControl.X.ref(0,zmp.previewControl.count);
        
        zmp.previewControl.Y.m_ref = zmp.previewControl.Y.ref(0,zmp.previewControl.count)/zmp.previewControl.swap_Y_gain; 
        zmp.previewControl.X.m_ref = zmp.previewControl.X.ref(0,zmp.previewControl.count);
        
        zmp.previewControl.Y.sum_E = zmp.previewControl.Y.sum_E + zmp.previewControl.Y.E;
        zmp.previewControl.X.sum_E = zmp.previewControl.X.sum_E + zmp.previewControl.X.E;
        
        for(int pre_cnt = 0; pre_cnt < zmp.previewControl.PreviewTimeSize; pre_cnt++)
        {
            if(pre_cnt == 0)
            {
                zmp.previewControl.X_Tilde = -zmp.previewControl.A_Tilde_c.transpose()*zmp.previewControl.K_Tilde*zmp.previewControl.I_Tilde;
                zmp.previewControl.G_P = -zmp.previewControl.G_I;
            }
            else
            {
                zmp.previewControl.X_Tilde = zmp.previewControl.A_Tilde_c.transpose()*zmp.previewControl.X_Tilde;
                zmp.previewControl.G_P = (zmp.previewControl.R + zmp.previewControl.B_Tilde.transpose()*zmp.previewControl.K_Tilde*zmp.previewControl.B_Tilde).inverse()*zmp.previewControl.B_Tilde.transpose()*zmp.previewControl.X_Tilde;
            }
           
            zmp.previewControl.Y.sum_P = zmp.previewControl.Y.sum_P + zmp.previewControl.G_P(0,0)*zmp.previewControl.Y.ref(0,zmp.previewControl.count+pre_cnt);
            zmp.previewControl.X.sum_P = zmp.previewControl.X.sum_P + zmp.previewControl.G_P(0,0)*zmp.previewControl.X.ref(0,zmp.previewControl.count+pre_cnt);
        }
        
        zmp.previewControl.Y.U = -zmp.previewControl.G_I(0,0)*zmp.previewControl.Y.sum_E - (zmp.previewControl.G_X*zmp.previewControl.Y.state)(0,0) - zmp.previewControl.Y.sum_P;
        zmp.previewControl.X.U = -zmp.previewControl.G_I(0,0)*zmp.previewControl.X.sum_E - (zmp.previewControl.G_X*zmp.previewControl.X.state)(0,0) - zmp.previewControl.X.sum_P;
        
        zmp.previewControl.Y.new_state = zmp.previewControl.A*zmp.previewControl.Y.state + zmp.previewControl.B*zmp.previewControl.Y.U;
        zmp.previewControl.X.new_state = zmp.previewControl.A*zmp.previewControl.X.state + zmp.previewControl.B*zmp.previewControl.X.U;
        
        zmp.previewControl.Y.old_zmp = (zmp.previewControl.C*zmp.previewControl.Y.state)(0,0);
        zmp.previewControl.X.old_zmp = (zmp.previewControl.C*zmp.previewControl.X.state)(0,0);
        
        zmp.previewControl.Y.CoM = zmp.previewControl.Y.new_state(0,0);
        zmp.previewControl.X.CoM = zmp.previewControl.X.new_state(0,0);
        
        
        zmp.previewControl.Y.sum_P = 0;
        zmp.previewControl.X.sum_P = 0;

    }
    else
    {

    }
    
}

void CRobot::initializeZmpControl() {
    
    zmp.observer.X.hat_state     = MatrixNd::Zero(2, 1);
    zmp.observer.X.hat_new_state = MatrixNd::Zero(2, 1);
    zmp.observer.X.hat_d_state   = MatrixNd::Zero(2, 1);
    zmp.observer.X.hat_ZMP       = 0;
    
    zmp.observer.Y.hat_state     = MatrixNd::Zero(2, 1);
    zmp.observer.Y.hat_new_state = MatrixNd::Zero(2, 1);
    zmp.observer.Y.hat_d_state   = MatrixNd::Zero(2, 1);
    zmp.observer.Y.hat_ZMP       = 0;
    
    zmp.observer.X.L    = MatrixNd::Zero(2, 1); //Kookmin.Univ ZMPobserver gain L
    zmp.observer.Y.L    = MatrixNd::Zero(2, 1); //Kookmin.Univ ZMPobserver gain L
    
    zmp.controller.X.K  = MatrixNd::Zero(1, 2); //Kookmin.Univ ZMPControl gain K
    zmp.controller.Y.K  = MatrixNd::Zero(1, 2); //Kookmin.Univ ZMPControl gain K
    
    zmp.controller.X.U      = 0; //Kookmin.Univ ZMPControl Input U
    zmp.controller.Y.U       = 0; //Kookmin.Univ ZMPControl Input U
    zmp.controller.X.m_U     = 0; //Kookmin.Univ ZMPControl Input U
    zmp.controller.Y.m_U     = 0; //Kookmin.Univ ZMPControl Input U
    zmp.controller.X.U_Limit = lowerbody.foot_x_length/3; //Kookmin.Univ ZMPControl Input U
    zmp.controller.Y.U_Limit = lowerbody.foot_y_length/3; //Kookmin.Univ ZMPControl Input U
    
    
    zmp.observer.X.L_DSP  = MatrixNd::Zero(2, 1); //Kookmin.Univ ZMPobserver gain L
    zmp.observer.Y.L_DSP  = MatrixNd::Zero(2, 1); //Kookmin.Univ ZMPobserver gain L
    zmp.observer.X.L_SSP  = MatrixNd::Zero(2, 1); //Kookmin.Univ ZMPobserver gain L
    zmp.observer.Y.L_SSP  = MatrixNd::Zero(2, 1); //Kookmin.Univ ZMPobserver gain L
    
    zmp.observer.X.L_DSP << 0.8916\
                           ,65.0228;//Observer gain L pole -150 +-0.1i
        
    zmp.observer.Y.L_DSP << 0.5636\
                           ,44.0376;//Observer gain L pole -150 +-0.1ii
    
    zmp.observer.X.L_SSP << 1.2260\
                           ,33.2347;//Observer gain L pole -50 +-0.1i
        
    zmp.observer.Y.L_SSP << 0.9028\
                           ,3.6686;//Observer gain L pole -15 +-0.1i
    
    
    zmp.controller.X.K_DSP  = MatrixNd::Zero(1, 2); //Kookmin.Univ ZMPControl gain K
    zmp.controller.Y.K_DSP  = MatrixNd::Zero(1, 2); //Kookmin.Univ ZMPControl gain K
    zmp.controller.X.K_SSP  = MatrixNd::Zero(1, 2); //Kookmin.Univ ZMPControl gain K
    zmp.controller.Y.K_SSP  = MatrixNd::Zero(1, 2); //Kookmin.Univ ZMPControl gain K
    
    zmp.controller.X.K_DSP << -1.5643, 0.0153; //Kookmin.Univ ZMPControl gain K pole -6 +-0.1i
    zmp.controller.Y.K_DSP << -1.5914, 0.0067; //Kookmin.Univ ZMPControl gain K pole -5 +-0.1i
    
    zmp.controller.X.K_SSP << -1.4787, 0.0420; //Kookmin.Univ ZMPControl gain K pole -4 +-0.1i
    zmp.controller.Y.K_SSP << -1.1856, 0.1256; //Kookmin.Univ ZMPControl gain K pole -5 +-0.1i
    
    zmp.controller.gainChangeTime = 0.1;
    zmp.controller.dsp2ssp = false;
    zmp.controller.ssp2dsp = false;
    
    //ZMPobserver and ZMPcontroller init
        
    zmp.observer.X.L   = zmp.observer.X.L_DSP;//Observer gain L pole -150 +-0.1i
    zmp.observer.Y.L   = zmp.observer.Y.L_DSP;//Observer gain L pole -150 +-0.1i    
    
    zmp.controller.X.K   = zmp.controller.X.K_DSP;//ZMPControl gain K pole -4 +-0.1i
    zmp.controller.Y.K   = zmp.controller.Y.K_DSP;//ZMPControl gain K pole -5 +-0.1i
}

void CRobot::zmpObserverControl() {
    getSystemID();
    zmpObserver();
    zmpController();
}

void CRobot::getSystemID() {
    //Get system Spring and Damping constant
    
    systemID.X.T         = systemID.X.T2(0,0)-systemID.X.T1(0,0);
    systemID.X.Damping_C = 2*systemID.robotI*log(systemID.X.T1(0,1)/systemID.X.T2(0,1))/systemID.X.T; //Kookmin.Univ Damping_C_X for ZMPobserver
    systemID.X.Spring_K  = (pow(2*PI/systemID.X.T,2)+pow(systemID.X.Damping_C/(2*systemID.robotI),2))*systemID.robotI+systemID.robotM*GRAVITY*systemID.robotH;//Kookmin.Univ Spring_K_X for ZMPobserver
    
    systemID.Y.T         = systemID.Y.T2(0,0)-systemID.Y.T1(0,0);
    systemID.Y.Damping_C = 2*systemID.robotI*log(systemID.Y.T1(0,1)/systemID.Y.T2(0,1))/systemID.Y.T; //Kookmin.Univ Damping_C_Y for ZMPobserver
    systemID.Y.Spring_K  = (pow(2*PI/systemID.Y.T,2)+pow(systemID.Y.Damping_C/(2*systemID.robotI),2))*systemID.robotI+systemID.robotM*GRAVITY*systemID.robotH;//Kookmin.Univ Spring_K_Y for ZMPobserver    
    //system dynamics
    //dx = Ax + Bu
    //y = Cx + Du
    
    //Get System ID
    
    systemID.X.A << 0, 1\
                   ,-(systemID.X.Spring_K-systemID.robotM*GRAVITY*systemID.robotH)/(systemID.robotM*systemID.robotH*systemID.robotH), -systemID.X.Damping_C/(systemID.robotM*systemID.robotH*systemID.robotH);   
    systemID.X.B << 0\
                   ,systemID.X.Spring_K/(systemID.robotM*pow(systemID.robotH,3));
    
    systemID.X.C << systemID.X.Spring_K/(systemID.robotM*GRAVITY), systemID.X.Damping_C/(systemID.robotM*GRAVITY);
    
    systemID.X.D = -systemID.X.Spring_K/(systemID.robotM*GRAVITY*systemID.robotH);
    
    
    
    systemID.Y.A << 0, 1\
                   ,-(systemID.Y.Spring_K-systemID.robotM*GRAVITY*systemID.robotH)/(systemID.robotM*systemID.robotH*systemID.robotH), -systemID.Y.Damping_C/(systemID.robotM*systemID.robotH*systemID.robotH);   
    systemID.Y.B << 0\
                   ,systemID.Y.Spring_K/(systemID.robotM*pow(systemID.robotH,3));
    
    systemID.Y.C << systemID.Y.Spring_K/(systemID.robotM*GRAVITY), systemID.Y.Damping_C/(systemID.robotM*GRAVITY);
    
    systemID.Y.D = -systemID.Y.Spring_K/(systemID.robotM*GRAVITY*systemID.robotH);
}

void CRobot::zmpCalculation() {
    
    ////Checking SSP/DSP=??
    if(LFoot.ftSensor.Fz < 100)
    {
       walking.m_SP = walking.RSSP;
    }
    else if(RFoot.ftSensor.Fz < 100)
    {
        walking.m_SP = walking.LSSP;
    }
    else if(LFoot.ftSensor.Fz < 100 && RFoot.ftSensor.Fz < 100)
    {
        walking.m_SP = walking.UP;
    }
    else
    {
        walking.m_SP = walking.DSP;
    }
    
    zmpControlGainChange();
    
    ///ZmpCalculation
    if(walking.m_SP == walking.RSSP)
    {
        zmp.X.global = zmp.previewControl.X.m_ref + zmp.X.m_global;
        
        zmp.Y.sensor = ((RFoot.ftSensor.Mx)*1000)/(RFoot.ftSensor.Fz) - lowerbody.LEG_SIDE_OFFSET;
        zmp.X.sensor = ((-RFoot.ftSensor.My)*1000)/(RFoot.ftSensor.Fz) + zmp.X.global;
        
        zmp.Y.offset = -walking.Y_SWAP;
        
        zmp.Y.Lmargin = -lowerbody.LEG_SIDE_OFFSET + lowerbody.foot_y_length/2;
        zmp.Y.Rmargin = -lowerbody.LEG_SIDE_OFFSET - lowerbody.foot_y_length/2;   
        
        zmp.X.Fmargin = zmp.X.global + lowerbody.foot_x_length/2;
        zmp.X.Bmargin = zmp.X.global - lowerbody.foot_x_length/2;
        
    }
    else if(walking.m_SP == walking.LSSP)
    {
        zmp.X.global = zmp.previewControl.X.m_ref + zmp.X.m_global;
        
        zmp.Y.sensor = ((LFoot.ftSensor.Mx)*1000)/(LFoot.ftSensor.Fz) + lowerbody.LEG_SIDE_OFFSET;
        zmp.X.sensor = ((-LFoot.ftSensor.My)*1000)/(LFoot.ftSensor.Fz) + zmp.X.global;
        
        zmp.Y.offset = walking.Y_SWAP;
        
        zmp.Y.Lmargin = lowerbody.LEG_SIDE_OFFSET + lowerbody.foot_y_length/2;
        zmp.Y.Rmargin = lowerbody.LEG_SIDE_OFFSET - lowerbody.foot_y_length/2;   
        
        zmp.X.Fmargin = zmp.X.global + lowerbody.foot_x_length/2;
        zmp.X.Bmargin = zmp.X.global - lowerbody.foot_x_length/2;
        
    }
    else if(walking.m_SP == walking.UP)
    {
        
    }
    else//DSP
    {
        
        
        zmp.Y.sensor = ((LFoot.ftSensor.Mx + RFoot.ftSensor.Mx)*1000)/(LFoot.ftSensor.Fz + RFoot.ftSensor.Fz)+(LFoot.ftSensor.Fz)*(lowerbody.LEG_SIDE_OFFSET)/(LFoot.ftSensor.Fz + RFoot.ftSensor.Fz)-(RFoot.ftSensor.Fz)*(lowerbody.LEG_SIDE_OFFSET)/(LFoot.ftSensor.Fz + RFoot.ftSensor.Fz);
        
        
        if(walking.step.checking == 0 || walking.step.checking == walking.step.total)
        {
            zmp.X.global = zmp.previewControl.X.m_ref + zmp.X.m_global;
        
            zmp.X.sensor = ((-LFoot.ftSensor.My - RFoot.ftSensor.My)*1000)/(LFoot.ftSensor.Fz + RFoot.ftSensor.Fz)+zmp.X.global;
        
            zmp.X.Fmargin = zmp.X.global+lowerbody.foot_x_length/2;
            zmp.X.Bmargin = zmp.X.global-lowerbody.foot_x_length/2;
        }
        else
        {
            zmp.X.global = (walking.step.FBstepSize)/2 + walking.step.FBstepSize*(walking.step.checking-1) + zmp.X.m_global;
        
            zmp.X.sensor = ((-LFoot.ftSensor.My - RFoot.ftSensor.My)*1000)/(LFoot.ftSensor.Fz + RFoot.ftSensor.Fz)+(LFoot.ftSensor.Fz)*(-zmp.previewControl.X.m_ref + LFoot.pos.X)/(LFoot.ftSensor.Fz + RFoot.ftSensor.Fz)+(RFoot.ftSensor.Fz)*(-zmp.previewControl.X.m_ref + RFoot.pos.X)/(LFoot.ftSensor.Fz + RFoot.ftSensor.Fz)+zmp.X.global;
        
            zmp.X.Fmargin = zmp.X.global+(abs)(walking.step.FBstepSize)/2+lowerbody.foot_x_length/2;
            zmp.X.Bmargin = zmp.X.global-(abs)(walking.step.FBstepSize)/2-lowerbody.foot_x_length/2;
        }
        
        
        zmp.Y.offset = 0;
        
        zmp.Y.Lmargin = lowerbody.LEG_SIDE_OFFSET+lowerbody.foot_y_length/2;
        zmp.Y.Rmargin = -lowerbody.LEG_SIDE_OFFSET-lowerbody.foot_y_length/2; 
          

    }

    walking.pre_SP = walking.m_SP;
}

void CRobot::zmpObserver() {
    
    zmp.observer.X.hat_d_state = systemID.X.A*zmp.observer.X.hat_state + systemID.X.B*zmp.controller.X.U + zmp.observer.X.L*(zmp.X.sensor-(zmp.X.global)-zmp.observer.X.hat_ZMP);
    zmp.observer.X.hat_ZMP     = (systemID.X.C*zmp.observer.X.hat_state)(0,0) + systemID.X.D*zmp.controller.X.U;
    
    zmp.observer.X.hat_new_state = zmp.observer.X.hat_state + zmp.observer.X.hat_d_state*(double)tasktime;
    zmp.observer.X.hat_state     = zmp.observer.X.hat_new_state;
    zmp.observer.X.m_hat_state   = zmp.observer.X.hat_new_state(0,0);
    zmp.observer.X.m_hat_ZMP     = zmp.observer.X.hat_ZMP;
    
    
    zmp.observer.Y.hat_d_state = systemID.Y.A*zmp.observer.Y.hat_state + systemID.Y.B*zmp.controller.Y.U + zmp.observer.Y.L*(zmp.Y.sensor-(zmp.Y.offset)-zmp.observer.Y.hat_ZMP);
    zmp.observer.Y.hat_ZMP     = (systemID.Y.C*zmp.observer.Y.hat_state)(0,0) + systemID.Y.D*zmp.controller.Y.U;
    
    zmp.observer.Y.hat_new_state = zmp.observer.Y.hat_state + zmp.observer.Y.hat_d_state*(double)tasktime;
    zmp.observer.Y.hat_state     = zmp.observer.Y.hat_new_state;
    zmp.observer.Y.m_hat_state   = zmp.observer.Y.hat_new_state(0,0);
    zmp.observer.Y.m_hat_ZMP     = zmp.observer.Y.hat_ZMP;
    
    
}
void CRobot::zmpController() {
    zmp.controller.X.U = (-zmp.controller.X.K*zmp.observer.X.hat_new_state)(0,0);
    zmp.controller.Y.U = (-zmp.controller.Y.K*zmp.observer.Y.hat_new_state)(0,0);
    
    if(zmp.controller.X.U > zmp.controller.X.U_Limit)
        zmp.controller.X.U = zmp.controller.X.U_Limit;
    else if(zmp.controller.X.U < -zmp.controller.X.U_Limit)
        zmp.controller.X.U = -zmp.controller.X.U_Limit;
    
    if(zmp.controller.Y.U > zmp.controller.Y.U_Limit)
        zmp.controller.Y.U = zmp.controller.Y.U_Limit;
    else if(zmp.controller.Y.U < -zmp.controller.Y.U_Limit)
        zmp.controller.Y.U = -zmp.controller.Y.U_Limit;
    
    if(isnan(zmp.controller.X.U) == 1)
        zmp.controller.X.U = zmp.controller.X.m_U;
    if(isnan(zmp.controller.Y.U) == 1)
        zmp.controller.Y.U = zmp.controller.Y.m_U;
    
    zmp.controller.X.m_U = zmp.controller.X.U;
    zmp.controller.Y.m_U = zmp.controller.Y.U;
    
}
void CRobot::zmpControlGainChange() {
    if((walking.pre_SP == walking.LSSP || walking.pre_SP == walking.RSSP) && walking.m_SP == walking.DSP)
    {
        zmp.controller.gainChange_t = 0;
       
        //ZMPobserver and ZMPcontroller
        systemID.X.T1 = systemID.X.T1_DSP;//DSP_X
        systemID.X.T2 = systemID.X.T2_DSP;//DSP_X
        
        systemID.Y.T1 = systemID.Y.T1_DSP;//DSP_Y
        systemID.Y.T2 = systemID.Y.T2_DSP;//DSP_Y
        
        zmp.observer.X.L   = zmp.observer.X.L_DSP;
        zmp.observer.Y.L   = zmp.observer.Y.L_DSP;
        
        zmp.controller.ssp2dsp = true;     
    }
    else if(walking.pre_SP == walking.DSP && (walking.m_SP == walking.LSSP || walking.m_SP == walking.RSSP))
    {
        zmp.controller.gainChange_t = 0;
       
        //ZMPobserver and ZMPcontroller
        systemID.X.T1 = systemID.X.T1_SSP;//SSP_X
        systemID.X.T2 = systemID.X.T2_SSP;//SSP_X
        
        systemID.Y.T1 = systemID.Y.T1_SSP;//SSP_Y
        systemID.Y.T2 = systemID.Y.T2_SSP;//SSP_Y
        
        zmp.observer.X.L   = zmp.observer.X.L_SSP;
        zmp.observer.Y.L   = zmp.observer.Y.L_SSP;
        
        zmp.controller.dsp2ssp = true;
    }
    
    
    if(zmp.controller.ssp2dsp == true)
    {
        if(zmp.controller.gainChange_t < zmp.controller.gainChangeTime)
        {
            systemID.X.T1               = cosWave(1,zmp.controller.gainChangeTime,zmp.controller.gainChange_t,0)*systemID.X.T1_DSP               + cosWave(-1,zmp.controller.gainChangeTime,zmp.controller.gainChange_t,1)*systemID.X.T1_SSP;
            systemID.X.T2               = cosWave(1,zmp.controller.gainChangeTime,zmp.controller.gainChange_t,0)*systemID.X.T2_DSP               + cosWave(-1,zmp.controller.gainChangeTime,zmp.controller.gainChange_t,1)*systemID.X.T2_SSP;
            systemID.Y.T1               = cosWave(1,zmp.controller.gainChangeTime,zmp.controller.gainChange_t,0)*systemID.Y.T1_DSP               + cosWave(-1,zmp.controller.gainChangeTime,zmp.controller.gainChange_t,1)*systemID.Y.T1_SSP;
            systemID.Y.T2               = cosWave(1,zmp.controller.gainChangeTime,zmp.controller.gainChange_t,0)*systemID.Y.T2_DSP               + cosWave(-1,zmp.controller.gainChangeTime,zmp.controller.gainChange_t,1)*systemID.Y.T2_SSP;
            zmp.observer.X.L            = cosWave(1,zmp.controller.gainChangeTime,zmp.controller.gainChange_t,0)*zmp.observer.X.L_DSP            + cosWave(-1,zmp.controller.gainChangeTime,zmp.controller.gainChange_t,1)*zmp.observer.X.L_SSP ;
            zmp.observer.Y.L            = cosWave(1,zmp.controller.gainChangeTime,zmp.controller.gainChange_t,0)*zmp.observer.Y.L_DSP            + cosWave(-1,zmp.controller.gainChangeTime,zmp.controller.gainChange_t,1)*zmp.observer.Y.L_SSP ;
            zmp.controller.X.K          = cosWave(1,zmp.controller.gainChangeTime,zmp.controller.gainChange_t,0)*zmp.controller.X.K_DSP          + cosWave(-1,zmp.controller.gainChangeTime,zmp.controller.gainChange_t,1)*zmp.controller.X.K_SSP ;
            zmp.controller.Y.K          = cosWave(1,zmp.controller.gainChangeTime,zmp.controller.gainChange_t,0)*zmp.controller.Y.K_DSP          + cosWave(-1,zmp.controller.gainChangeTime,zmp.controller.gainChange_t,1)*zmp.controller.Y.K_SSP ;
            zmp.controller.dsp_gain     = cosWave(1,zmp.controller.gainChangeTime,zmp.controller.gainChange_t,0);
            zmp.controller.ssp_gain     = cosWave(-1,zmp.controller.gainChangeTime,zmp.controller.gainChange_t,1);      
            zmp.controller.gainChange_t += (double)tasktime;
        }
        else
            zmp.controller.ssp2dsp = false;         
    }
    else if(zmp.controller.dsp2ssp == true)
    {
        if(zmp.controller.gainChange_t < zmp.controller.gainChangeTime)
        {
            systemID.X.T1               = cosWave(-1,zmp.controller.gainChangeTime,zmp.controller.gainChange_t,1)*systemID.X.T1_DSP               + cosWave(1,zmp.controller.gainChangeTime,zmp.controller.gainChange_t,0)*systemID.X.T1_SSP;
            systemID.X.T2               = cosWave(-1,zmp.controller.gainChangeTime,zmp.controller.gainChange_t,1)*systemID.X.T2_DSP               + cosWave(1,zmp.controller.gainChangeTime,zmp.controller.gainChange_t,0)*systemID.X.T2_SSP;
            systemID.Y.T1               = cosWave(-1,zmp.controller.gainChangeTime,zmp.controller.gainChange_t,1)*systemID.Y.T1_DSP               + cosWave(1,zmp.controller.gainChangeTime,zmp.controller.gainChange_t,0)*systemID.Y.T1_SSP;
            systemID.Y.T2               = cosWave(-1,zmp.controller.gainChangeTime,zmp.controller.gainChange_t,1)*systemID.Y.T2_DSP               + cosWave(1,zmp.controller.gainChangeTime,zmp.controller.gainChange_t,0)*systemID.Y.T2_SSP;
            zmp.observer.X.L            = cosWave(-1,zmp.controller.gainChangeTime,zmp.controller.gainChange_t,1)*zmp.observer.X.L_DSP            + cosWave(1,zmp.controller.gainChangeTime,zmp.controller.gainChange_t,0)*zmp.observer.X.L_SSP ;
            zmp.observer.Y.L            = cosWave(-1,zmp.controller.gainChangeTime,zmp.controller.gainChange_t,1)*zmp.observer.Y.L_DSP            + cosWave(1,zmp.controller.gainChangeTime,zmp.controller.gainChange_t,0)*zmp.observer.Y.L_SSP ;
            zmp.controller.X.K          = cosWave(-1,zmp.controller.gainChangeTime,zmp.controller.gainChange_t,1)*zmp.controller.X.K_DSP          + cosWave(1,zmp.controller.gainChangeTime,zmp.controller.gainChange_t,0)*zmp.controller.X.K_SSP ;
            zmp.controller.Y.K          = cosWave(-1,zmp.controller.gainChangeTime,zmp.controller.gainChange_t,1)*zmp.controller.Y.K_DSP          + cosWave(1,zmp.controller.gainChangeTime,zmp.controller.gainChange_t,0)*zmp.controller.Y.K_SSP ;
            zmp.controller.dsp_gain     = cosWave(-1,zmp.controller.gainChangeTime,zmp.controller.gainChange_t,1);
            zmp.controller.ssp_gain     = cosWave(1,zmp.controller.gainChangeTime,zmp.controller.gainChange_t,0);      
            zmp.controller.gainChange_t += (double)tasktime;
        }
        else
            zmp.controller.dsp2ssp = false; 
    }
}

void CRobot::globalZmpUpdate() {
    
    //global ZMP update, Kookmin.Univ
    zmp.X.m_global   += zmp.X.d_global-walking.step.FBstepSize;
    walking.step.current      = 0;
    walking.step.checking = 0;
    zmp.X.d_global    = 0;
    
    LFoot.pos.X=0;
    RFoot.pos.X=0;
    //global ZMP update    
}

void CRobot::setZmpSwapPlan() {
    int sign;
    if(walking.step.start_foot == walking.step.S_L_F)
    {
       sign = 1;
    }
    else if(walking.step.start_foot == walking.step.S_R_F)
    {
       sign = -1;        
    }
    for(int m_step = 0; m_step < walking.step.total; m_step++)
    {
        if((m_step+1)%2 == 1)
        {
            for(int pre_cnt = zmp.previewControl.RefTotalTrajSize*(zmp.previewControl.PreviewTime + walking.time.SSP_start_time + (m_step)*walking.time.periodTime)/zmp.previewControl.RefTotalTime; \
                    pre_cnt < zmp.previewControl.RefTotalTrajSize*(zmp.previewControl.PreviewTime + walking.time.SSP_end_time + (m_step)*walking.time.periodTime)/zmp.previewControl.RefTotalTime; \
                    pre_cnt++)
            {
                zmp.previewControl.Y.ref(0,pre_cnt) = -sign*walking.Y_SWAP*zmp.previewControl.swap_Y_gain;
                //printf("%lf\n",zmp.previewControl.Y.ref(0,pre_cnt));
            }              
        }
        else if((m_step+1)%2 == 0)
        {
            for(int pre_cnt = zmp.previewControl.RefTotalTrajSize*(zmp.previewControl.PreviewTime + walking.time.SSP_start_time + (m_step)*walking.time.periodTime)/zmp.previewControl.RefTotalTime; \
                    pre_cnt < zmp.previewControl.RefTotalTrajSize*(zmp.previewControl.PreviewTime + walking.time.SSP_end_time + (m_step)*walking.time.periodTime)/zmp.previewControl.RefTotalTime; \
                    pre_cnt++)
            {
                zmp.previewControl.Y.ref(0,pre_cnt) = sign*walking.Y_SWAP*zmp.previewControl.swap_Y_gain;
                //printf("%lf\n",zmp.previewControl.Y.ref(0,pre_cnt));
            }          
        }
       
    }
}


void CRobot::setZmpFBPlan() {
    
    int sign;
    if(walking.step.walkig_f_or_b == walking.step.forwardWalking)
    {
       sign = 1;
    }
    else if(walking.step.walkig_f_or_b == walking.step.backwardWalking)
    {
       sign = -1;        
    }
    for(int pre_cnt = 0; \
            pre_cnt < zmp.previewControl.RefTotalTrajSize*(zmp.previewControl.PreviewTime + 1*walking.time.periodTime)/zmp.previewControl.RefTotalTime; \
            pre_cnt++)
    {
        zmp.previewControl.X.ref(0,pre_cnt) = 0;
    }
    for(int m_step = 1; m_step < walking.step.total; m_step++)
    {
        for(int pre_cnt = zmp.previewControl.RefTotalTrajSize*(zmp.previewControl.PreviewTime + (m_step)*walking.time.periodTime)/zmp.previewControl.RefTotalTime; \
                pre_cnt < zmp.previewControl.RefTotalTrajSize*(zmp.previewControl.PreviewTime + (m_step+1)*walking.time.periodTime)/zmp.previewControl.RefTotalTime; \
                pre_cnt++)
        {
            zmp.previewControl.X.ref(0,pre_cnt) = sign*(abs)(walking.step.FBstepSize)*m_step;
        }         

    }
    for(int pre_cnt = zmp.previewControl.RefTotalTrajSize*(zmp.previewControl.PreviewTime + (walking.step.total)*walking.time.periodTime)/zmp.previewControl.RefTotalTime; \
            pre_cnt < zmp.previewControl.RefTotalTrajSize; \
            pre_cnt++)
    {
        zmp.previewControl.X.ref(0,pre_cnt) = sign*(abs)(walking.step.FBstepSize)*(walking.step.total-1);
    }
}

void CRobot::setZmpOneStepPlan() {
    int sign;
    if(walking.step.start_foot == walking.step.S_L_F)
    {
       sign = 1;
    }
    else if(walking.step.start_foot == walking.step.S_R_F)
    {
       sign = -1;        
    }
    for(int pre_cnt = 0; \
            pre_cnt < zmp.previewControl.RefTotalTrajSize*(zmp.previewControl.PreviewTime + walking.time.SSP_start_time + (1)*walking.time.periodTime)/zmp.previewControl.RefTotalTime; \
            pre_cnt++)
    {

        zmp.previewControl.Y.ref(0,pre_cnt) = 0;
       
    }
    for(int pre_cnt = zmp.previewControl.RefTotalTrajSize*(zmp.previewControl.PreviewTime + walking.time.SSP_start_time + (1)*walking.time.periodTime)/zmp.previewControl.RefTotalTime; \
            pre_cnt < zmp.previewControl.RefTotalTrajSize; \
            pre_cnt++)
    {

        zmp.previewControl.Y.ref(0,pre_cnt) = -sign*walking.Y_SWAP*zmp.previewControl.swap_Y_gain;
       
    }
}

void CRobot::walkingReady(double readytime) {

    if(walking.time.ready<readytime)
    {
        walking.offset.m_x = cosWave(walking.offset.x,readytime,walking.time.ready,0);
        walking.offset.m_z = cosWave(walking.offset.z,readytime,walking.time.ready,0);
  
        walking.time.ready += tasktime;
    }
    else
     {
        walking.time.ready = 0;
        walking.Ready = true;
    }
    
}

void CRobot::walkingPlan() {
    
    zmpPreviewControl();
    
    if(zmp.previewControl.count < zmp.previewControl.RefTotalTrajSize*(zmp.previewControl.PreviewTime+(0)*walking.time.periodTime)/zmp.previewControl.RefTotalTime)
    {
        walking.time.sec = 0;
    }
    else if(zmp.previewControl.count < zmp.previewControl.RefTotalTrajSize*(zmp.previewControl.PreviewTime+(walking.step.total)*walking.time.periodTime)/zmp.previewControl.RefTotalTime)
    {
       //Kookmin.Univ start foot checking
        int Checking_S_L_F; //Kookmin.Univ start foot checking 
        int Checking_S_R_F; //Kookmin.Univ start foot checking
        if(walking.step.start_foot == walking.step.S_L_F)
        {
            Checking_S_L_F = 1;
            Checking_S_R_F = 0;
        }
        else if(walking.step.start_foot == walking.step.S_R_F)
        {
            Checking_S_L_F = 0;
            Checking_S_R_F = 1;               
        }
        //Kookmin.Univ start foot checking
            
            
        //Kookmin.Univ Walking
        if(walking.time.sec < walking.time.SSP_start_time)
        {
            LFoot.pos.Z  = 0;
            RFoot.pos.Z  = 0;
        }
        else if(walking.time.sec < walking.time.SSP_end_time)
        {
            LFoot.pos.Z  = Checking_S_L_F*cosWave(LFoot.pos.refZ, walking.time.SSP_time/2, walking.time.sec-walking.time.SSP_start_time, 0);
            RFoot.pos.Z  = Checking_S_R_F*cosWave(RFoot.pos.refZ, walking.time.SSP_time/2, walking.time.sec-walking.time.SSP_start_time, 0);           
            
            if(walking.step.current == 0)
            {
                LFoot.pos.X  = Checking_S_L_F*cosWave(LFoot.pos.refX, walking.time.SSP_time, walking.time.sec-walking.time.SSP_start_time, 0) + zmp.X.d_global;
                RFoot.pos.X  = Checking_S_R_F*cosWave(RFoot.pos.refX, walking.time.SSP_time, walking.time.sec-walking.time.SSP_start_time, 0) + zmp.X.d_global;
            }
            else if(walking.step.current == walking.step.total-1)
            {
                LFoot.pos.X  = Checking_S_L_F*cosWave(LFoot.pos.refX, walking.time.SSP_time,walking.time.sec-walking.time.SSP_start_time ,0) + zmp.X.d_global-walking.step.FBstepSize;
                RFoot.pos.X  = Checking_S_R_F*cosWave(RFoot.pos.refX, walking.time.SSP_time,walking.time.sec-walking.time.SSP_start_time, 0) + zmp.X.d_global-walking.step.FBstepSize;
            }
            else
            {
                LFoot.pos.X  = Checking_S_L_F*cosWave(2*LFoot.pos.refX, walking.time.SSP_time, walking.time.sec-walking.time.SSP_start_time,0) + zmp.X.d_global-walking.step.FBstepSize;
                RFoot.pos.X  = Checking_S_R_F*cosWave(2*RFoot.pos.refX, walking.time.SSP_time, walking.time.sec-walking.time.SSP_start_time,0) + zmp.X.d_global-walking.step.FBstepSize;
            }
            
            if(walking.step.start_foot == walking.step.S_L_F && walking.step.current != 0)
            {
                RFoot.pos.X  = zmp.X.d_global;
            }
            else if(walking.step.start_foot == walking.step.S_R_F && walking.step.current != 0)
            {
                LFoot.pos.X  = zmp.X.d_global;
            }
        }
        else if(walking.time.sec < walking.time.periodTime+walking.time.SSP_start_time)
        {
            LFoot.pos.Z  = 0;
            RFoot.pos.Z  = 0;
        }
        else if(walking.time.sec < walking.time.periodTime+walking.time.SSP_end_time)
        {
            LFoot.pos.Z  = Checking_S_R_F*cosWave(LFoot.pos.refZ, walking.time.SSP_time/2, walking.time.sec-(walking.time.periodTime+walking.time.SSP_start_time), 0);
            RFoot.pos.Z  = Checking_S_L_F*cosWave(RFoot.pos.refZ, walking.time.SSP_time/2, walking.time.sec-(walking.time.periodTime+walking.time.SSP_start_time), 0);            
            
            if(walking.step.current == walking.step.total-1)
            {
                LFoot.pos.X  = Checking_S_R_F*cosWave(LFoot.pos.refX, walking.time.SSP_time, walking.time.sec-(walking.time.periodTime+walking.time.SSP_start_time), 0) + zmp.X.d_global-walking.step.FBstepSize;
                RFoot.pos.X  = Checking_S_L_F*cosWave(RFoot.pos.refX, walking.time.SSP_time, walking.time.sec-(walking.time.periodTime+walking.time.SSP_start_time), 0) + zmp.X.d_global-walking.step.FBstepSize;
            }
            else
            {
                LFoot.pos.X  = Checking_S_R_F*cosWave(2*LFoot.pos.refX, walking.time.SSP_time, walking.time.sec-(walking.time.periodTime+walking.time.SSP_start_time),0) + zmp.X.d_global-walking.step.FBstepSize;
                RFoot.pos.X  = Checking_S_L_F*cosWave(2*RFoot.pos.refX, walking.time.SSP_time, walking.time.sec-(walking.time.periodTime+walking.time.SSP_start_time),0) + zmp.X.d_global-walking.step.FBstepSize;
            }
            if(walking.step.start_foot == walking.step.S_L_F)
            {
                LFoot.pos.X  = zmp.X.d_global;
            }
            else if(walking.step.start_foot == walking.step.S_R_F)
            {
                RFoot.pos.X  = zmp.X.d_global;
            }
            
        }
        else
        {
            LFoot.pos.Z  = 0;
            RFoot.pos.Z  = 0;
            
        }
        //Kookmin.Univ Walking
               
        //Kookmin.Univ Walking Time
        if(walking.time.sec < (2*walking.time.periodTime))
        {
            walking.time.sec += tasktime;
            
//            //onesteptest
//            if(walking.time.sec > (walking.time.periodTime)/2)
//            {
//                walking.time.sec = (walking.time.periodTime)/2;
//            }
//            //onesteptest
            
            
            if(walking.time.sec >= (walking.time.periodTime)/2-tasktime && walking.time.sec < (walking.time.periodTime)/2)
            {
                walking.step.checking ++;
            }          
            else if(walking.time.sec >= walking.time.periodTime-tasktime && walking.time.sec < walking.time.periodTime)
            {
                walking.step.current ++;
                zmp.X.d_global += walking.step.FBstepSize;
            }
            else if(walking.time.sec >= (walking.time.periodTime)*3/2-tasktime && walking.time.sec < (walking.time.periodTime)*3/2)
            {
                walking.step.checking ++;
            }
            else if(walking.time.sec >= 2*walking.time.periodTime-tasktime && walking.time.sec < 2*walking.time.periodTime)
            {
                walking.step.current ++;
                zmp.X.d_global += walking.step.FBstepSize;
            }
            
            
            if(walking.time.sec > (2*walking.time.periodTime)-tasktime)
            {
                walking.time.sec = 0;            
            }
        }
             
    }
    else
    {
        walking.time.sec = 0;
        if(zmp.previewControl.count > zmp.previewControl.RefTotalTrajSize)
        {
            walking.State           = false;           
        }        
    }
    
    zmp.previewControl.count++;
    
}

void CRobot::ComputeTorqueControl() {    
    
    // ↓↓↓↓↓↓↓ state update
    RobotState(AXIS_X)     = base.currentX;
    RobotState(AXIS_Y)     = base.currentY;
    RobotState(AXIS_Z)     = base.currentZ;
    RobotState(AXIS_Roll)  = base.currentRoll;
    RobotState(AXIS_Pitch) = base.currentPitch;
    RobotState(AXIS_Yaw)   = base.currentYaw;
    RobotStatedot(AXIS_X)     = base.currentXvel;
    RobotStatedot(AXIS_Y)     = base.currentYvel;
    RobotStatedot(AXIS_Z)     = base.currentZvel;
    RobotStatedot(AXIS_Roll)  = base.currentRollvel;
    RobotStatedot(AXIS_Pitch) = base.currentPitchvel;
    RobotStatedot(AXIS_Yaw)   = base.currentYawvel;
    
    for (int nJoint = 0; nJoint < nDOF; nJoint++) {
        RobotState(6+nJoint)    = joint[nJoint].currentAngle;
        RobotStatedot(6+nJoint) = joint[nJoint].currentVel;
    }
        
    // state update
    
    
    // ↓↓↓↓↓↓↓ Jacobian
        
    VectorNd OriginRightFoot = Vector3d(0, 0, 0);
    VectorNd OriginLeftFoot  = Vector3d(0, 0, 0);
    VectorNd Originbase      = Vector3d(0, 0, 0);
    
    CalcPointJacobian6D(*m_pModel, RobotState, RFoot.ID , OriginRightFoot, J_Rfoot, true);
    CalcPointJacobian6D(*m_pModel, RobotState, LFoot.ID,  OriginLeftFoot,  J_Lfoot, true);
    CalcPointJacobian6D(*m_pModel, RobotState, base.ID,   Originbase,      J_Base,  true);
    
    MatrixNd dJ_Rfoot = MatrixNd::Zero(6, m_pModel->dof_count);
    MatrixNd dJ_Lfoot = MatrixNd::Zero(6, m_pModel->dof_count);
    MatrixNd dJ_Base  = MatrixNd::Zero(6, m_pModel->dof_count);
    
    dJ_Rfoot = J_Rfoot - preJ_Rfoot;
    dJ_Lfoot = J_Lfoot - preJ_Lfoot;
    dJ_Base  = J_Base  - preJ_Base;
    for(int i = 0; i < 6; i++)
    {
        for(int j = 0; j < m_pModel->dof_count; j++)
        {
            dotJ_Rfoot(i,j) = dJ_Rfoot(i,j)/(double)tasktime;
            dotJ_Lfoot(i,j) = dJ_Lfoot(i,j)/(double)tasktime;
            dotJ_Base(i,j)  = dJ_Base(i,j)/(double)tasktime;
        }
    }
    preJ_Rfoot = J_Rfoot;
    preJ_Lfoot = J_Lfoot;
    preJ_Base  = J_Base;
    
    //MatrixNd J_A = MatrixNd::Zero(18, 18);
    //MatrixNd dotJ_A = MatrixNd::Zero(18, 18);
    MatrixNd J_A = MatrixNd::Zero(12, 18);
    MatrixNd dotJ_A = MatrixNd::Zero(12, 18);
    
//    J_A.block<6, 18>(0,0) = J_Base;     // base foot y,p,r,x,y,z
//    J_A.block<6, 18>(6,0) = J_Lfoot;     // left foot y,p,r,x,y,z
//    J_A.block<6, 18>(12,0) = J_Rfoot;     // right foot y,p,r,x,y,z
//    
//    dotJ_A.block<6, 18>(0,0) = dotJ_Base;     // base foot y,p,r,x,y,z
//    dotJ_A.block<6, 18>(6,0) = dotJ_Lfoot;     // left foot y,p,r,x,y,z
//    dotJ_A.block<6, 18>(12,0) = dotJ_Rfoot;     // right foot y,p,r,x,y,z
    
    J_A.block<6, 18>(0,0) = J_Lfoot;     // left foot y,p,r,x,y,z
    J_A.block<6, 18>(6,0) = J_Rfoot;     // right foot y,p,r,x,y,z
    
    dotJ_A.block<6, 18>(0,0) = dotJ_Lfoot;     // left foot y,p,r,x,y,z
    dotJ_A.block<6, 18>(6,0) = dotJ_Rfoot;     // right foot y,p,r,x,y,z
    
    
//    s
    
    // Jacobian
    
    
    // ↓↓↓↓↓↓↓ External Cartesian forces
    
    VectorNd FTsensor_R = VectorNd::Zero(6);    
    VectorNd FTsensor_L = VectorNd::Zero(6); 
    
    FTsensor_R(0) = 0.7*RFoot.ftSensor.Mx;    
    FTsensor_R(1) = 0.7*RFoot.ftSensor.My;
    FTsensor_R(2) = 0.0*RFoot.ftSensor.Mz;
    FTsensor_R(3) = 0.0*RFoot.ftSensor.Fx;
    FTsensor_R(4) = 0.0*RFoot.ftSensor.Fy;
    FTsensor_R(5) = RFoot.ftSensor.Fz;//RFoot.ftSensor.Fz;//1418.28*GRAVITY/2;
    if(FTsensor_R(5) > systemID.robotM*GRAVITY)
    {
        FTsensor_R(5) = systemID.robotM*GRAVITY;
    }
    
    
    FTsensor_L(0) = 0.7*LFoot.ftSensor.Mx;    
    FTsensor_L(1) = 0.7*LFoot.ftSensor.My;
    FTsensor_L(2) = 0.0*LFoot.ftSensor.Mz;
    FTsensor_L(3) = 0.0*LFoot.ftSensor.Fx;
    FTsensor_L(4) = 0.0*LFoot.ftSensor.Fy;
    FTsensor_L(5) = LFoot.ftSensor.Fz;//LFoot.ftSensor.Fz;//1418.28*GRAVITY/2
    if(FTsensor_L(5) > systemID.robotM*GRAVITY)
    {
        FTsensor_L(5) = systemID.robotM*GRAVITY;
    }
    
    printf("FX %lf\t %lf\n",FTsensor_L(3),FTsensor_R(3));
    printf("FY %lf\t %lf\n",FTsensor_L(4),FTsensor_R(4));
    printf("FZ %lf\t %lf\n",FTsensor_L(5),FTsensor_R(5));
    printf("MX %lf\t %lf\n",FTsensor_L(0),FTsensor_R(0));
    printf("MY %lf\t %lf\n",FTsensor_L(1),FTsensor_R(1));
    printf("MZ %lf\t %lf\n",FTsensor_L(2),FTsensor_R(2));
    
    VectorNd JF = VectorNd::Zero(m_pModel->dof_count);
    
    JF = J_Rfoot.transpose()*FTsensor_R + J_Lfoot.transpose()*FTsensor_L;
    
    for(int nJoint = 0; nJoint < nDOF; nJoint++)
    {
        outJF(6+nJoint) = 0.0*outJF(6+nJoint) + 1.0*JF(6+nJoint);
    }
    
    // External Cartesian forces
    
    
    // ↓↓↓↓↓↓↓ GET Joint Space M C G 
    
    MatrixNd M = MatrixNd::Zero(m_pModel->dof_count, m_pModel->dof_count);
    CompositeRigidBodyAlgorithm(*m_pModel, RobotState,M);
            
    VectorNd hatNonLinearEffects = VectorNd::Zero(m_pModel->dof_count);
    NonlinearEffects(*m_pModel, RobotState, RobotStatedot, hatNonLinearEffects);//C(q,q')+G(q)
    
    VectorNd G = VectorNd::Zero(m_pModel->dof_count);
    NonlinearEffects(*m_pModel, RobotState, VectorNd::Zero(m_pModel->dof_count), G);//G(q)
    
    VectorNd C = VectorNd::Zero(m_pModel->dof_count);
    C = hatNonLinearEffects - G;//C(q)
    
    // GET Joint Space M C G
    
    
    // ↓↓↓↓↓↓↓ GET Task Space M C G
    
//    MatrixNd Task_M = MatrixNd::Zero(m_pModel->dof_count, m_pModel->dof_count);
//    MatrixNd Task_C = MatrixNd::Zero(m_pModel->dof_count, 1);
//    MatrixNd Task_G = MatrixNd::Zero(m_pModel->dof_count, 1);
    
    MatrixNd Task_M = MatrixNd::Zero(nElementofEnd, nElementofEnd);
    MatrixNd Task_C = MatrixNd::Zero(nElementofEnd, 1);
    MatrixNd Task_G = MatrixNd::Zero(nElementofEnd, 1);
    
    Task_M = (J_A*M.inverse()*J_A.transpose()).inverse();
    Task_C = Task_M*J_A*M.inverse()*C-Task_M*dotJ_A*RobotStatedot;
    Task_G = Task_M*J_A*M.inverse()*G;
    
    // GET Task Space M C G
   
    
    // ↓↓↓↓↓↓↓ GET Ref & Current of End Effector
    
    VectorNd LeftLegjointstate = VectorNd::Zero(6);
    VectorNd RightLegjointstate = VectorNd::Zero(6);
    
    for (int nJoint = 0; nJoint < 6; nJoint++) {
        LeftLegjointstate(nJoint)    = joint[nJoint].currentAngle;
        RightLegjointstate(nJoint)   = joint[6+nJoint].currentAngle;
    }
        
    double currentLF[6];
    double currentRF[6];
    if((computeFK(&currentLF[0],LeftLegjointstate) == 1 ) && (computeFK(&currentRF[0],RightLegjointstate) == 1))
    {
        for (int n = 0; n < nElementofEnd/2; n++) {
            LFoot.current(n)    = currentLF[n];
            RFoot.current(n)    = currentRF[n];
            LFoot.vel(n) = (LFoot.current(n)-LFoot.pre(n))/(double)tasktime;
            RFoot.vel(n) = (RFoot.current(n)-RFoot.pre(n))/(double)tasktime;
            if(n == 3 || n == 4 || n == 5)
            {
                LFoot.acc(n) = 0.99*LFoot.preacc(n)+0.01*(LFoot.vel(n)-LFoot.prevel(n))/(double)tasktime/1000.0;
                RFoot.acc(n) = 0.99*RFoot.preacc(n)+0.01*(RFoot.vel(n)-RFoot.prevel(n))/(double)tasktime/1000.0; 
            }
            else
            {
                LFoot.acc(n) = 0.99*LFoot.preacc(n)+0.01*(LFoot.vel(n)-LFoot.prevel(n))/(double)tasktime;
                RFoot.acc(n) = 0.99*RFoot.preacc(n)+0.01*(RFoot.vel(n)-RFoot.prevel(n))/(double)tasktime; 
            }
            LFoot.pre(n) = LFoot.current(n);
            RFoot.pre(n) = RFoot.current(n);
            LFoot.prevel(n) = LFoot.vel(n);
            RFoot.prevel(n) = RFoot.vel(n);
            LFoot.preacc(n) = LFoot.acc(n);
            RFoot.preacc(n) = RFoot.acc(n);      
        }
    }
    
    MatrixNd LFootMapping = MatrixNd::Zero(6, 6);
    MatrixNd RFootMapping = MatrixNd::Zero(6, 6);
    
    LFootMapping(0,0) = 0;
    LFootMapping(1,0) = 0;
    LFootMapping(2,0) = 1;
    RFootMapping(0,0) = 0;
    RFootMapping(1,0) = 0;
    RFootMapping(2,0) = 1;
    
    LFootMapping(0,1) = -sin(LFoot.current(0));
    LFootMapping(1,1) = cos(LFoot.current(0));
    LFootMapping(2,1) = 0;
    RFootMapping(0,1) = -sin(RFoot.current(0));
    RFootMapping(1,1) = cos(RFoot.current(0));
    RFootMapping(2,1) = 0;
    
    LFootMapping(0,2) = cos(LFoot.current(1))*cos(LFoot.current(0));
    LFootMapping(1,2) = cos(LFoot.current(1))*sin(LFoot.current(0));
    LFootMapping(2,2) = -sin(LFoot.current(1));
    RFootMapping(0,2) = cos(RFoot.current(1))*cos(RFoot.current(0));
    RFootMapping(1,2) = cos(RFoot.current(1))*sin(RFoot.current(0));
    RFootMapping(2,2) = -sin(RFoot.current(1));
    
    LFootMapping.block<3, 3>(3,3) = MatrixNd::Identity(3, 3);     //Identity(3, 3)
    RFootMapping.block<3, 3>(3,3) = MatrixNd::Identity(3, 3);     //Identity(3, 3)
    
     
    LFoot.refpos(3) = LFoot.pos.F_refX; 
    LFoot.refpos(4) = LFoot.pos.F_refY;
    LFoot.refpos(5) = LFoot.pos.F_refZ;
    LFoot.refpos(0) = LFoot.orientation.F_refYaw; 
    LFoot.refpos(1) = LFoot.orientation.F_refPitch;
    LFoot.refpos(2) = LFoot.orientation.F_refRoll;
        
    RFoot.refpos(3) = RFoot.pos.F_refX; 
    RFoot.refpos(4) = RFoot.pos.F_refY;
    RFoot.refpos(5) = RFoot.pos.F_refZ;
    RFoot.refpos(0) = RFoot.orientation.F_refYaw; 
    RFoot.refpos(1) = RFoot.orientation.F_refPitch;
    RFoot.refpos(2) = RFoot.orientation.F_refRoll;
    
    for (int n = 0; n < nElementofEnd/2; n++) {
        LFoot.refvel(n) = (LFoot.refpos(n)-LFoot.prerefpos(n))/(double)tasktime;
        RFoot.refvel(n) = (RFoot.refpos(n)-RFoot.prerefpos(n))/(double)tasktime;
             
        if(n == 3 || n == 4 || n == 5)
        {
            LFoot.refacc(n) = (LFoot.refvel(n)-LFoot.prerefvel(n))/(double)tasktime/1000.0;
            RFoot.refacc(n) = (RFoot.refvel(n)-RFoot.prerefvel(n))/(double)tasktime/1000.0;
        }
        else
        {
            LFoot.refacc(n) = (LFoot.refvel(n)-LFoot.prerefvel(n))/(double)tasktime;
            RFoot.refacc(n) = (RFoot.refvel(n)-RFoot.prerefvel(n))/(double)tasktime; 
        }
        
        LFoot.prerefpos(n) = LFoot.refpos(n);
        RFoot.prerefpos(n) = RFoot.refpos(n);
        LFoot.prerefvel(n) = LFoot.refvel(n);
        RFoot.prerefvel(n) = RFoot.refvel(n);
    }
    
    
    VectorNd LFootErr = VectorNd::Zero(6);
    VectorNd RFootErr = VectorNd::Zero(6);
    
    LFootErr = LFootMapping*(LFoot.refpos - LFoot.current);
    RFootErr = RFootMapping*(RFoot.refpos - RFoot.current);
    
    VectorNd LFootvelErr = VectorNd::Zero(6);
    VectorNd RFootvelErr = VectorNd::Zero(6);
    
    LFootvelErr = LFootMapping*(LFoot.refvel - LFoot.vel);
    RFootvelErr = RFootMapping*(RFoot.refvel - RFoot.vel);
    
    VectorNd PDLCatEul= VectorNd::Zero(6);
    VectorNd PDRCatEul= VectorNd::Zero(6);
    
    // GET Ref & Current of End Effector
    
    printf("current Rx = %lf\t current Ry = %lf\t current Rz = %lf\t",RFootErr [3],RFootErr [4],RFootErr [5]);
    printf("current Lx = %lf\t current Ly = %lf\t current Lz = %lf\n",LFootErr[3],LFootErr[4],LFootErr[5]);
    printf("ori Rx = %lf\t ori Ry = %lf\t ori Rz = %lf\t",RFootErr [0]*180.0/PI,RFootErr [1]*180.0/PI,RFootErr [2]*180.0/PI);
    printf("ori Lx = %lf\t ori Ly = %lf\t ori Lz = %lf\n",LFootErr[0]*180.0/PI,LFootErr[1]*180.0/PI,LFootErr[2]*180.0/PI);

    
    
    //↓↓↓↓↓↓↓ PD Control
    CXK=0.5;
    CXD=0.05;
    OXK=50000;
    OXD=110;
    CYK=0.65;
    CYD=0.0865;
    OYK=45000;
    OYD=90;
    CZK=1.0;
    CZD=0.105;
    OZK=15000;
    OZD=40;
//    
    PDLCatEul(0) = OXK*LFootErr(0)  + OXD*LFootvelErr(0);
    PDLCatEul(1) = OYK*LFootErr(1)  + OYD*LFootvelErr(1);
    PDLCatEul(2) = OZK*LFootErr(2)  + OZD*LFootvelErr(2);
    PDLCatEul(3) = CXK*LFootErr(3) + CXD*LFootvelErr(3);
    PDLCatEul(4) = CYK*LFootErr(4) + CYD*LFootvelErr(4);
    PDLCatEul(5) = CZK*LFootErr(5) + CZD*LFootvelErr(5);
    
    PDRCatEul(0) = OXK*RFootErr(0)  + OXD*RFootvelErr(0);
    PDRCatEul(1) = OYK*RFootErr(1) + OYD*RFootvelErr(1);
    PDRCatEul(2) = OZK*RFootErr(2)   + OZD*RFootvelErr(2);
    PDRCatEul(3) = CXK*RFootErr(3) + CXD*RFootvelErr(3);
    PDRCatEul(4) = CYK*RFootErr(4) + CYD*RFootvelErr(4);
    PDRCatEul(5) = CZK*RFootErr(5) + CZD*RFootvelErr(5);
    
    
    MatrixNd refLAccel = MatrixNd::Zero(6, 1);
    MatrixNd refRAccel = MatrixNd::Zero(6, 1);
    MatrixNd refBAccel = MatrixNd::Zero(6, 1);
    
    refLAccel = PDLCatEul + LFoot.refacc;
    refRAccel = PDRCatEul + RFoot.refacc;
    
    // PD Control
    
//    MatrixNd EndPointAccel = MatrixNd::Zero(18, 1);
// 
//    EndPointAccel.block<6, 1>(0,0) = refBAccel;     // pelvis x,y  
//    EndPointAccel.block<6, 1>(6,0) = refLAccel;     // left foot y,p,r,x,y,z 
//    EndPointAccel.block<6, 1>(12,0) = refRAccel;    // right foot y,p,r,x,y,z
    
    MatrixNd EndPointAccel = MatrixNd::Zero( nElementofEnd, 1);
 
    EndPointAccel.block<6, 1>(0,0) = refLAccel;     // left foot y,p,r,x,y,z 
    EndPointAccel.block<6, 1>(6,0) = refRAccel;    // right foot y,p,r,x,y,z
       
    VectorNd JPDCE = VectorNd::Zero(m_pModel->dof_count);
    
    
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//    JPDCE = J_A.transpose()*EndPointAccel + J_A.transpose()*Task_C + J_A.transpose()*Task_G; //Hanyunho.ver
    JPDCE = J_A.transpose()*Task_M*EndPointAccel + J_A.transpose()*Task_C + J_A.transpose()*Task_G; //original
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    
    for (int nJoint = 0; nJoint < nDOF; nJoint++) {
        joint[nJoint].torque    = (JPDCE)(6+nJoint)-outJF(6+nJoint);
    }
    for(int nJoint = 0; nJoint < nDOF/2; nJoint++)
    {
        printf("%d\t = L %lf\t R %lf\t FL %lf\t FR %lf\n",nJoint, joint[nJoint].torque, joint[nJoint+6].torque,outJF(6+nJoint),outJF(12+nJoint) );
    }

}


MatrixNd CRobot::ZMP_DARE(MatrixNd& A, MatrixNd& B, MatrixNd& Q, MatrixNd& R)//Kookmin.Univ Preview
{
    unsigned int nSize = A.rows();
    MatrixNd Z(nSize* 2, nSize* 2);
    
    Z.block(0, 0, nSize, nSize) = A+ B* R.inverse()* B.transpose()* (A.inverse()).transpose() * Q;
    Z.block(0, nSize, nSize, nSize) = -B* R.inverse()* B.transpose()* (A.inverse()).transpose();
    Z.block(nSize, 0, nSize, nSize) = -(A.inverse()).transpose()* Q;
    Z.block(nSize, nSize, nSize, nSize) = (A.inverse()).transpose();
    
     eZMPSolver.compute(Z, true);
     
    Eigen::MatrixXcd U(nSize* 2, nSize);
    unsigned int j=0;
    for (unsigned int i=0; i<nSize* 2; i++)
    {
        std::complex<double> eigenvalue = eZMPSolver.eigenvalues()[i];
        double dReal = eigenvalue.real();
        double dImag = eigenvalue.imag();

        if( std::sqrt((dReal* dReal) + (dImag* dImag)) < 1.0)
        {
            U.block(0, j, nSize* 2, 1) = eZMPSolver.eigenvectors().col(i);
            j++;
        }
    }
    if(j != nSize)
    {
        std::cout << "Warning! ******* Pelvis Planning *******" << std::endl;
    }

    Eigen::MatrixXcd U1 = U.block(0, 0, nSize, nSize);
    Eigen::MatrixXcd U2 = U.block(nSize, 0, nSize, nSize);

    Eigen::MatrixXcd X = U2 * U1.inverse();

    return X.real();
}


double CRobot::cosWave(double amp, double period, double time, double int_pos) {
    return (amp / 2)*(1 - cos(PI / period * time)) + int_pos;
}

bool CRobot::computeIK(double *out, double x, double y, double z, double roll, double pitch, double yaw) {
    VectorNd pos(3), ori(3), r(3);
    MatrixNd Tad, Tda, Tcd, Tdc, Tac, Tca, R123, R1, R2, R3, R4, R5, R6, D1, D2, D3, T;
    double _Rac, _Acos, _Atan, _alpha, _sign, _x, _y;

    ///////////////////////////////////////////////////////////////////
    pos << x, y, z - lowerbody.LEG_LENGTH;
    ori << roll, pitch, yaw;
    Tad = setTransform(pos, ori);

    pos << 0, 0, -lowerbody.ANKLE_LENGTH;
    ori << 0, 0, 0;
    Tcd = setTransform(pos, ori);
    Tdc = Tcd.inverse();
    Tac = Tad*Tdc;
    Tca = Tac.inverse();
    r(0) = Tca(0, 3);
    r(1) = Tca(1, 3);
    r(2) = Tca(2, 3);

    // Get Knee
    _Rac = r.norm();

    _Acos = acos((pow(_Rac, 2) - pow(lowerbody.THIGH_LENGTH, 2) - pow(lowerbody.CALF_LENGTH, 2)) / (2 * lowerbody.THIGH_LENGTH * lowerbody.CALF_LENGTH));
        if(isnan(_Acos) == 1)
            return false;
        *(out + 3) = _Acos;
        
        _alpha = asin((lowerbody.THIGH_LENGTH/_Rac)*sin(PI-*(out + 3)));


    // Get Ankle Roll 
    _Atan = atan2(r(1), r(2));
        if(isinf(_Atan) == 1)
           return false;
        if(_Atan > PI/2) 
            _Atan=_Atan-PI; 
        else if(_Atan < -PI/2) 
            _Atan=_Atan+PI;
        *(out + 5)=_Atan;
        
        
        if(r(2)>0)
            _sign = 1;
        else if(r(2)<0)
            _sign = -1;
        else
            _sign = 0;
        // Get Ankle Pitch        
        _Atan = -atan2(r(0),_sign*sqrt(pow(r(1),2)+pow(r(2),2))) -_alpha; 
        if(isinf(_Atan) == 1)
            return false;
    
        *(out + 4)=_Atan;
        
        
        //////////////////////////////////////////////////////////////////
        pos<<0,0,-lowerbody.THIGH_LENGTH;
        ori<<0,0,0;       
        D1 = setTransform(pos,ori);   
        
        pos<<0,0,0;
        ori<<0,*(out + 3),0;       
        R4 = setTransform(pos,ori);   
           
        pos<<0,0,-lowerbody.CALF_LENGTH;
        ori<<0,0,0;       
        D2 = setTransform(pos,ori);    
        
        pos<<0,0,0;
        ori<<0,*(out + 4),0;       
        R5 = setTransform(pos,ori); 
        
        
        pos<<0,0,0;
        ori<<*(out + 5),0,0;       
        R6 = setTransform(pos,ori);
        
        
        pos<<0,0,-lowerbody.ANKLE_LENGTH;
        ori<<0,0,0;       
        D3 = setTransform(pos,ori);  
        
        R123=Tad*D3.inverse()*R6.inverse()*R5.inverse()*D2.inverse()*R4.inverse()*D1.inverse();
        
        //////////////////////////////////////////////////////////////////////////////
        
        // Get Hip Pitch
        _x = R123(2,2);
        _y = R123(0,2);
        _Atan = atan2(_y,_x);
        if(isinf(_Atan) == 1)
            return false;
    
        *(out)=_Atan;
        
        // Get Hip Roll
        _x = R123(0,2)*sin(*(out))+R123(2,2)*cos(*(out));
        _y = -R123(1,2);
    
        _Atan = atan2(_y,_x);
        
        if(isinf(_Atan) == 1)
            return false;
        if(_Atan > PI/2) 
            _Atan=_Atan-PI; 
        else if(_Atan < -PI/2) 
            _Atan=_Atan+PI;
        *(out+1)=_Atan;
        
        
        // Get Hip Yaw
        _x = R123(1,1);
        _y = R123(1,0);
    
        _Atan = atan2(_y,_x);
        if(isinf(_Atan) == 1)
            return false;
        if(_Atan > PI/2) 
            _Atan=_Atan-PI; 
        else if(_Atan < -PI/2) 
            _Atan=_Atan+PI;
        *(out+2)=_Atan;

    return true;
    //    
}

bool CRobot::computeFK(double *currentFoot, VectorNd jointstate) {
    
    VectorNd pos(3), ori(3);
    MatrixNd R1, R2, R3, R4, R5, R6, D1, D2, D3;
    double _x, _y, _Atan;
    
    pos<<0,0,0;
    ori<<0,jointstate(0),0;       
    R1 = setTransform(pos,ori);
    //Hip pitch
    
    pos<<0,0,0;
    ori<<jointstate(1),0,0;       
    R2 = setTransform(pos,ori);
    //Hip Roll
    
    pos<<0,0,0;
    ori<<0,0,jointstate(2);       
    R3 = setTransform(pos,ori);
    //Hip Roll
    
    pos<<0,0,-lowerbody.THIGH_LENGTH;
    ori<<0,0,0;       
    D1 = setTransform(pos,ori);   
        
    pos<<0,0,0;
    ori<<0,jointstate(3),0;       
    R4 = setTransform(pos,ori);   
           
    pos<<0,0,-lowerbody.CALF_LENGTH;
    ori<<0,0,0;       
    D2 = setTransform(pos,ori);    
        
    pos<<0,0,0;
    ori<<0,jointstate(4),0;       
    R5 = setTransform(pos,ori); 
        
        
    pos<<0,0,0;
    ori<<jointstate(5),0,0;       
    R6 = setTransform(pos,ori);
        
        
    pos<<0,0,-lowerbody.ANKLE_LENGTH;
    ori<<0,0,0;       
    D3 = setTransform(pos,ori);
    
    MatrixNd EdFoot = MatrixNd::Identity(4, 4);
    
    EdFoot = R1*R2*R3*D1*R4*D2*R5*R6*D3;
    
    *(currentFoot+3) = EdFoot(0,3); // X
    *(currentFoot+4) = EdFoot(1,3); // Y
    *(currentFoot+5) = EdFoot(2,3)+lowerbody.LEG_LENGTH; // Z
    
    // Roll
    _x =  EdFoot(2,2);
    _y =  EdFoot(2,1);
    
    _Atan = atan2(_y,_x);
        
    if(isinf(_Atan) == 1)
            return false;
    if(_Atan > PI/2) 
        _Atan=_Atan-PI; 
    else if(_Atan < -PI/2) 
         _Atan=_Atan+PI;
    *(currentFoot+2) =_Atan;
    
    // Yaw
    _x =  EdFoot(0,0);
    _y =  EdFoot(1,0);
    
    _Atan = atan2(_y,_x);
        
    if(isinf(_Atan) == 1)
            return false;
    if(_Atan > PI/2) 
        _Atan=_Atan-PI; 
    else if(_Atan < -PI/2) 
         _Atan=_Atan+PI;
    *(currentFoot)=_Atan;
    
     // Pitch
    _x =  cos(*(currentFoot))*EdFoot(0,0) + sin(*(currentFoot))*EdFoot(1,0);
    _y =  -EdFoot(2,0);
    
    _Atan = atan2(_y,_x);
        
    if(isinf(_Atan) == 1)
            return false;
    if(_Atan > PI/2) 
        _Atan=_Atan-PI; 
    else if(_Atan < -PI/2) 
         _Atan=_Atan+PI;
    *(currentFoot+1)=_Atan;
    
    return true;
}


MatrixNd CRobot::setTransform(VectorNd pos, VectorNd orientation) {
    double Cx = cos(orientation(0));
    double Cy = cos(orientation(1));
    double Cz = cos(orientation(2));
    double Sx = sin(orientation(0));
    double Sy = sin(orientation(1));
    double Sz = sin(orientation(2));



    MatrixNd m = MatrixNd::Identity(4, 4);
    m(0, 0) = Cz * Cy;
    m(0, 1) = Cz * Sy * Sx - Sz * Cx;
    m(0, 2) = Cz * Sy * Cx + Sz * Sx;
    m(0, 3) = pos(0);
    m(1, 0) = Sz * Cy;
    m(1, 1) = Sz * Sy * Sx + Cz * Cx;
    m(1, 2) = Sz * Sy * Cx - Cz * Sx;
    m(1, 3) = pos(1);
    m(2, 0) = -Sy;
    m(2, 1) = Cy * Sx;
    m(2, 2) = Cy * Cx;
    m(2, 3) = pos(2);
    return m;
}

MatrixNd CRobot::PseudoInverse(MatrixNd mat)
{
    MatrixNd AtA = mat.transpose()* mat;
    MatrixNd Ret = AtA.inverse()* mat.transpose();
 
    return Ret;
}
 
 */