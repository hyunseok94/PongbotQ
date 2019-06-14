#include <gazebo/common/common.hh>
#include <gazebo/common/Plugin.hh>
#include <ros/ros.h>
#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/sensors/sensors.hh>
#include <stdio.h>
#include <iostream>
#include <std_msgs/Float64.h>
#include <functional>
#include <ignition/math/Vector3.hh>
#include "Eigen/Dense"
#include <rbdl/rbdl.h>
#include <rbdl/addons/urdfreader/urdfreader.h>

#include "CRobot.h" // by BKCho

//#define PI      3.141592
#define D2R     PI/180
#define R2D     180/PI

using Eigen::MatrixXd;
using Eigen::VectorXd;
using Eigen::Vector3d;

//using namespace RigidBodyDynamics;
//using namespace RigidBodyDynamics::Math;

//RBDL
//#define AXIS_X     0
//#define AXIS_Y     1
//#define AXIS_Z     2
//#define AXIS_Roll  3
//#define AXIS_Pitch 4
//#define AXIS_Yaw   5

BASE base;
JOINT* joint;

void getCurrentJoint(VectorNd Angle, VectorNd Vel);
void getRobotState(VectorNd BasePosOri, VectorNd BaseVel, VectorNd JointAngle, VectorNd JointVel);
void CTC(void);

Model* pongbot_q_model = new Model();

unsigned int m_nDOF;
unsigned int Joint_Num;

// Inverse Dynamics
RigidBodyDynamics::Math::VectorNd RobotState;
RigidBodyDynamics::Math::VectorNd RobotStatedot;

// DH PARAMETERS
RigidBodyDynamics::Math::VectorNd jointAngle;
RigidBodyDynamics::Math::VectorNd jointVel;
RigidBodyDynamics::Math::VectorNd basePosOri;
RigidBodyDynamics::Math::VectorNd baseVel;

namespace gazebo
{

    class PongBotQ_plugin : public ModelPlugin
    {
        physics::LinkPtr REAR_BODY;
        physics::LinkPtr FRONT_BODY;
        physics::LinkPtr FL_HIP;
        physics::LinkPtr FL_THIGH;
        physics::LinkPtr FL_CALF;
        physics::LinkPtr FL_TIP;
        physics::LinkPtr FR_HIP;
        physics::LinkPtr FR_THIGH;
        physics::LinkPtr FR_CALF;
        physics::LinkPtr FR_TIP;
        physics::LinkPtr RL_HIP;
        physics::LinkPtr RL_THIGH;
        physics::LinkPtr RL_CALF;
        physics::LinkPtr RL_TIP;
        physics::LinkPtr RR_HIP;
        physics::LinkPtr RR_THIGH;
        physics::LinkPtr RR_CALF;
        physics::LinkPtr RR_TIP;

        physics::JointPtr WAIST_JOINT;
        physics::JointPtr FL_HIP_JOINT;
        physics::JointPtr FL_THIGH_JOINT;
        physics::JointPtr FL_CALF_JOINT;
        physics::JointPtr FL_TIP_JOINT;
        physics::JointPtr FR_HIP_JOINT;
        physics::JointPtr FR_THIGH_JOINT;
        physics::JointPtr FR_CALF_JOINT;
        physics::JointPtr FR_TIP_JOINT;
        physics::JointPtr RL_HIP_JOINT;
        physics::JointPtr RL_THIGH_JOINT;
        physics::JointPtr RL_CALF_JOINT;
        physics::JointPtr RL_TIP_JOINT;
        physics::JointPtr RR_HIP_JOINT;
        physics::JointPtr RR_THIGH_JOINT;
        physics::JointPtr RR_CALF_JOINT;
        physics::JointPtr RR_TIP_JOINT;
        physics::ModelPtr model;

        //  PID
        common::PID pid_FL_HR, pid_FL_HP, pid_FL_KN;
        common::PID pid_FR_HR, pid_FR_HP, pid_FR_KN;
        common::PID pid_RL_HR, pid_RL_HP, pid_RL_KN;
        common::PID pid_RR_HR, pid_RR_HP, pid_RR_KN;
        common::PID pid_WAIST;

        //setting for getting <dt>(=derivative time)
        common::Time last_update_time;
        event::ConnectionPtr update_connection;
        double dt;
        double time = 0;

        //setting for IMU sensor
        sensors::SensorPtr Sensor;
        sensors::ImuSensorPtr IMU;
        double IMU_Update;
        double angular_velocity_x;
        double angular_velocity_y;
        double angular_velocity_z;
        double linear_acc_x = 0;
        double linear_acc_y = 0;
        double linear_acc_z = 0;

        //setting for FT sensor
        physics::JointWrench wrench;
        ignition::math::Vector3d torque;
        ignition::math::Vector3d force;
        double FR_force_x, FR_force_y, FR_force_z;
        double FR_torque_x, FR_torque_y, FR_torque_z;
        double FL_force_x, FL_force_y, FL_force_z;
        double FL_torque_x, FL_torque_y, FL_torque_z;
        double RR_force_x, RR_force_y, RR_force_z;
        double RR_torque_x, RR_torque_y, RR_torque_z;
        double RL_force_x, RL_force_y, RL_force_z;
        double RL_torque_x, RL_torque_y, RL_torque_z;

        //setting for rqt telecommunication
        ros::NodeHandle n;
        ros::Publisher P_Times;
        ros::Publisher P_angular_velocity_x;
        ros::Publisher P_angular_velocity_y;
        ros::Publisher P_angular_velocity_z;

        ros::Publisher P_FR_force_x;
        ros::Publisher P_FR_force_y;
        ros::Publisher P_FR_force_z;
        ros::Publisher P_FL_force_x;
        ros::Publisher P_FL_force_y;
        ros::Publisher P_FL_force_z;
        ros::Publisher P_RR_force_x;
        ros::Publisher P_RR_force_y;
        ros::Publisher P_RR_force_z;
        ros::Publisher P_RL_force_x;
        ros::Publisher P_RL_force_y;
        ros::Publisher P_RL_force_z;

        std_msgs::Float64 m_Times;
        std_msgs::Float64 m_angular_velocity_x;
        std_msgs::Float64 m_angular_velocity_y;
        std_msgs::Float64 m_angular_velocity_z;
        std_msgs::Float64 m_FR_force_x;
        std_msgs::Float64 m_FR_force_y;
        std_msgs::Float64 m_FR_force_z;
        std_msgs::Float64 m_FL_force_x;
        std_msgs::Float64 m_FL_force_y;
        std_msgs::Float64 m_FL_force_z;
        std_msgs::Float64 m_RR_force_x;
        std_msgs::Float64 m_RR_force_y;
        std_msgs::Float64 m_RR_force_z;
        std_msgs::Float64 m_RL_force_x;
        std_msgs::Float64 m_RL_force_y;
        std_msgs::Float64 m_RL_force_z;

        // DH PARA
        double tar_deg[13] = {0, 30, -60,
            -60, 30, 0,
            0, -30, 60,
            60, -30, 0,
            0};
        //double tar_deg[13] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

        VectorXd pre_Encoder = VectorXd::Zero(13);
        VectorXd angle_err = VectorXd::Zero(13);
        VectorXd angle_vel = VectorXd::Zero(13);
        VectorXd tau = VectorXd::Zero(13);
        VectorXd tau_G = VectorXd::Zero(13);

        // FT sensor Transformation
        VectorXd Encoder = VectorXd::Zero(17);
        VectorXd FR_Force_E = VectorXd::Zero(3);
        VectorXd FL_Force_E = VectorXd::Zero(3);
        VectorXd RR_Force_E = VectorXd::Zero(3);
        VectorXd RL_Force_E = VectorXd::Zero(3);
        VectorXd FR_Force_I = VectorXd::Zero(3);
        VectorXd FL_Force_I = VectorXd::Zero(3);
        VectorXd RR_Force_I = VectorXd::Zero(3);
        VectorXd RL_Force_I = VectorXd::Zero(3);
        VectorXd FR_Torque_E = VectorXd::Zero(3);
        VectorXd FL_Torque_E = VectorXd::Zero(3);
        VectorXd RR_Torque_E = VectorXd::Zero(3);
        VectorXd RL_Torque_E = VectorXd::Zero(3);
        VectorXd FR_Torque_I = VectorXd::Zero(3);
        VectorXd FL_Torque_I = VectorXd::Zero(3);
        VectorXd RR_Torque_I = VectorXd::Zero(3);
        VectorXd RL_Torque_I = VectorXd::Zero(3);
        MatrixXd FR_C_IE = MatrixXd::Zero(3, 3);
        MatrixXd FL_C_IE = MatrixXd::Zero(3, 3);
        MatrixXd RR_C_IE = MatrixXd::Zero(3, 3);
        MatrixXd RL_C_IE = MatrixXd::Zero(3, 3);

        // Adding CRobot Class by BKCho
        CRobot PongBotQ;

    public:
        //For model load
        void Load(physics::ModelPtr _model, sdf::ElementPtr /*_sdf*/);
        void UpdateAlgorithm();
    };
    GZ_REGISTER_MODEL_PLUGIN(PongBotQ_plugin);
}

MatrixXd getTransformI0()
{
    MatrixXd tmp_m(4, 4);
    tmp_m << 1, 0, 0, 0\
, 0, 1, 0, 0\
, 0, 0, 1, 0\
, 0, 0, 0, 1;
    return tmp_m;
}

MatrixXd F_jointToTransform01(VectorXd q)
{
    MatrixXd tmp_m(4, 4);
    double qq = q(12);
    tmp_m << cos(qq), -sin(qq), 0, 0.35\
, sin(qq), cos(qq), 0, 0\
, 0, 0, 1, 0.0352\
, 0, 0, 0, 1;
    return tmp_m;
}

MatrixXd FL_jointToTransform12(VectorXd q)
{
    MatrixXd tmp_m(4, 4);
    double qq = q(5);
    tmp_m << 1, 0, 0, 0.35\
, 0, cos(qq), -sin(qq), 0.115\
, 0, sin(qq), cos(qq), -0.0355\
, 0, 0, 0, 1;
    return tmp_m;
}

MatrixXd FL_jointToTransform23(VectorXd q)
{
    MatrixXd tmp_m(4, 4);
    double qq = q(4);
    tmp_m << cos(qq), 0, sin(qq), 0\
, 0, 1, 0, 0.105\
, -sin(qq), 0, cos(qq), 0\
, 0, 0, 0, 1;
    return tmp_m;
}

MatrixXd FL_jointToTransform34(VectorXd q)
{
    MatrixXd tmp_m(4, 4);
    double qq = q(3);
    tmp_m << cos(qq), 0, sin(qq), 0\
, 0, 1, 0, 0\
, -sin(qq), 0, cos(qq), -0.305\
, 0, 0, 0, 1;
    return tmp_m;
}

MatrixXd FL_jointToTransform4E(VectorXd q)
{
    MatrixXd tmp_m(4, 4);
    double qq = q(14);
    tmp_m << cos(qq), 0, sin(qq), -0.015\
, 0, 1, 0, 0\
, -sin(qq), 0, cos(qq), -0.288\
, 0, 0, 0, 1;
    return tmp_m;
}

MatrixXd FL_getTransformIE(VectorXd q)
{
    MatrixXd T_I0(4, 4), T_01(4, 4), T_12(4, 4), T_23(4, 4), T_34(4, 4), T_4E(4, 4);
    MatrixXd T_IE(4, 4);
    T_I0 = getTransformI0();
    T_01 = F_jointToTransform01(q);
    T_12 = FL_jointToTransform12(q);
    T_23 = FL_jointToTransform23(q);
    T_34 = FL_jointToTransform34(q);
    T_4E = FL_jointToTransform4E(q);
    T_IE = T_I0 * T_01 * T_12 * T_23 * T_34 * T_4E;
    return T_IE;
}

MatrixXd FL_jointToRotMat(VectorXd q)
{
    MatrixXd C_IE(3, 3);
    MatrixXd T_IE(4, 4);
    T_IE = FL_getTransformIE(q);
    C_IE = T_IE.block(0, 0, 3, 3);
    return C_IE;
}

MatrixXd FR_jointToTransform12(VectorXd q)
{
    MatrixXd tmp_m(4, 4);
    double qq = q(0);
    tmp_m << 1, 0, 0, 0.35\
, 0, cos(qq), -sin(qq), -0.115\
, 0, sin(qq), cos(qq), -0.0355\
, 0, 0, 0, 1;
    return tmp_m;
}

MatrixXd FR_jointToTransform23(VectorXd q)
{
    MatrixXd tmp_m(4, 4);
    double qq = q(1);
    tmp_m << cos(qq), 0, sin(qq), 0\
, 0, 1, 0, -0.105\
, -sin(qq), 0, cos(qq), 0\
, 0, 0, 0, 1;
    return tmp_m;
}

MatrixXd FR_jointToTransform34(VectorXd q)
{
    MatrixXd tmp_m(4, 4);
    double qq = q(2);
    tmp_m << cos(qq), 0, sin(qq), 0\
, 0, 1, 0, 0\
, -sin(qq), 0, cos(qq), -0.305\
, 0, 0, 0, 1;
    return tmp_m;
}

MatrixXd FR_jointToTransform4E(VectorXd q)
{
    MatrixXd tmp_m(4, 4);
    double qq = q(13);
    tmp_m << cos(qq), 0, sin(qq), -0.015\
, 0, 1, 0, 0\
, -sin(qq), 0, cos(qq), -0.288\
, 0, 0, 0, 1;
    return tmp_m;
}

MatrixXd FR_getTransformIE(VectorXd q)
{
    MatrixXd T_I0(4, 4), T_01(4, 4), T_12(4, 4), T_23(4, 4), T_34(4, 4), T_4E(4, 4);
    MatrixXd T_IE(4, 4);
    T_I0 = getTransformI0();
    T_01 = F_jointToTransform01(q);
    T_12 = FR_jointToTransform12(q);
    T_23 = FR_jointToTransform23(q);
    T_34 = FR_jointToTransform34(q);
    T_4E = FR_jointToTransform4E(q);
    T_IE = T_I0 * T_01 * T_12 * T_23 * T_34 * T_4E;
    return T_IE;
}

MatrixXd FR_jointToRotMat(VectorXd q)
{
    MatrixXd C_IE(3, 3);
    MatrixXd T_IE(4, 4);
    T_IE = FR_getTransformIE(q);
    C_IE = T_IE.block(0, 0, 3, 3);
    return C_IE;
}

MatrixXd RL_jointToTransform01(VectorXd q)
{
    MatrixXd tmp_m(4, 4);
    double qq = q(6);
    tmp_m << 1, 0, 0, 0\
, 0, cos(qq), -sin(qq), 0.115\
, 0, sin(qq), cos(qq), 0\
, 0, 0, 0, 1;
    return tmp_m;
}

MatrixXd RL_jointToTransform12(VectorXd q)
{
    MatrixXd tmp_m(4, 4);
    double qq = q(7);
    tmp_m << cos(qq), 0, sin(qq), 0\
, 0, 1, 0, 0.105\
, -sin(qq), 0, cos(qq), 0\
, 0, 0, 0, 1;
    return tmp_m;
}

MatrixXd RL_jointToTransform23(VectorXd q)
{
    MatrixXd tmp_m(4, 4);
    double qq = q(8);
    tmp_m << cos(qq), 0, sin(qq), 0\
, 0, 1, 0, 0\
, -sin(qq), 0, cos(qq), -0.305\
, 0, 0, 0, 1;
    return tmp_m;
}

MatrixXd RL_jointToTransform3E(VectorXd q)
{
    MatrixXd tmp_m(4, 4);
    double qq = q(16);
    tmp_m << cos(qq), 0, sin(qq), 0.015\
, 0, 1, 0, 0\
, -sin(qq), 0, cos(qq), -0.288\
, 0, 0, 0, 1;
    return tmp_m;
}

MatrixXd RL_getTransformIE(VectorXd q)
{
    MatrixXd T_I0(4, 4), T_01(4, 4), T_12(4, 4), T_23(4, 4), T_3E(4, 4);
    MatrixXd T_IE(4, 4);
    T_I0 = getTransformI0();
    T_01 = RL_jointToTransform01(q);
    T_12 = RL_jointToTransform12(q);
    T_23 = RL_jointToTransform23(q);
    T_3E = RL_jointToTransform3E(q);
    T_IE = T_I0 * T_01 * T_12 * T_23 * T_3E;
    return T_IE;
}

MatrixXd RL_jointToRotMat(VectorXd q)
{
    MatrixXd C_IE(3, 3);
    MatrixXd T_IE(4, 4);
    T_IE = RL_getTransformIE(q);
    C_IE = T_IE.block(0, 0, 3, 3);
    return C_IE;
}

MatrixXd RR_jointToTransform01(VectorXd q)
{
    MatrixXd tmp_m(4, 4);
    double qq = q(11);
    tmp_m << 1, 0, 0, 0\
, 0, cos(qq), -sin(qq), -0.115\
, 0, sin(qq), cos(qq), 0\
, 0, 0, 0, 1;
    return tmp_m;
}

MatrixXd RR_jointToTransform12(VectorXd q)
{
    MatrixXd tmp_m(4, 4);
    double qq = q(10);
    tmp_m << cos(qq), 0, sin(qq), 0\
, 0, 1, 0, -0.105\
, -sin(qq), 0, cos(qq), 0\
, 0, 0, 0, 1;
    return tmp_m;
}

MatrixXd RR_jointToTransform23(VectorXd q)
{
    MatrixXd tmp_m(4, 4);
    double qq = q(9);
    tmp_m << cos(qq), 0, sin(qq), 0\
, 0, 1, 0, 0\
, -sin(qq), 0, cos(qq), -0.305\
, 0, 0, 0, 1;
    return tmp_m;
}

MatrixXd RR_jointToTransform3E(VectorXd q)
{
    MatrixXd tmp_m(4, 4);
    double qq = q(15);
    tmp_m << cos(qq), 0, sin(qq), 0.015\
, 0, 1, 0, 0\
, -sin(qq), 0, cos(qq), -0.288\
, 0, 0, 0, 1;
    return tmp_m;
}

MatrixXd RR_getTransformIE(VectorXd q)
{
    MatrixXd T_I0(4, 4), T_01(4, 4), T_12(4, 4), T_23(4, 4), T_3E(4, 4);
    MatrixXd T_IE(4, 4);
    T_I0 = getTransformI0();
    T_01 = RR_jointToTransform01(q);
    T_12 = RR_jointToTransform12(q);
    T_23 = RR_jointToTransform23(q);
    T_3E = RR_jointToTransform3E(q);
    T_IE = T_I0 * T_01 * T_12 * T_23 * T_3E;
    return T_IE;
}

MatrixXd RR_jointToRotMat(VectorXd q)
{
    MatrixXd C_IE(3, 3);
    MatrixXd T_IE(4, 4);
    T_IE = RR_getTransformIE(q);
    C_IE = T_IE.block(0, 0, 3, 3);
    return C_IE;
}

void gazebo::PongBotQ_plugin::Load(physics::ModelPtr _model, sdf::ElementPtr /*_sdf*/)
{
    //RBDL
    // model = link + joint +sensor
    this->model = _model;

    rbdl_check_api_version(RBDL_API_VERSION);

    int version_test;
    version_test = rbdl_get_api_version();
    printf("rbdl api version = %d\n", version_test);

    //Model* pongbot_q_model = new Model();
    //Addons::URDFReadFromFile("/root/.gazebo/models/PONGBOT_Q_V2/urdf/PONGBOT_Q_V2.urdf", pongbot_q_model, true, true);
    Addons::URDFReadFromFile("/home/hyunseok/.gazebo/models/PONGBOT_Q_V2/urdf/PONGBOT_Q_V2.urdf", pongbot_q_model, true, true);
    pongbot_q_model->gravity = Math::Vector3d(0., 0., -9.81);

    m_nDOF = pongbot_q_model->dof_count;
    Joint_Num = m_nDOF - 6;

    RobotState = VectorNd::Zero(m_nDOF);
    RobotStatedot = VectorNd::Zero(m_nDOF);
    basePosOri = VectorNd::Zero(6);
    baseVel = VectorNd::Zero(6);
    jointAngle = VectorNd::Zero(Joint_Num);
    jointVel = VectorNd::Zero(Joint_Num);

    joint = new JOINT[Joint_Num];

    basePosOri << 0, 0, 0.7, 0, 0, 0;
    baseVel << 0, 0, 0, 0, 0, 0;
    jointAngle << 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
    jointVel << 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;

    getRobotState(basePosOri, baseVel, jointAngle, jointVel);
    CTC();

    PongBotQ.setRobotModel(pongbot_q_model);

    //LINK DEFINITION
    this->REAR_BODY = this->model->GetLink("REAR_BODY");
    this->FRONT_BODY = this->model->GetLink("FRONT_BODY");
    this->FR_HIP = this->model->GetLink("FR_HIP");
    this->FR_THIGH = this->model->GetLink("FR_THIGH");
    this->FR_CALF = this->model->GetLink("FR_CALF");
    this->FR_TIP = this->model->GetLink("FR_TIP");
    this->FL_HIP = this->model->GetLink("FL_HIP");
    this->FL_THIGH = this->model->GetLink("FL_THIGH");
    this->FL_CALF = this->model->GetLink("FL_CALF");
    this->FL_TIP = this->model->GetLink("FL_TIP");
    this->RR_HIP = this->model->GetLink("RR_HIP");
    this->RR_THIGH = this->model->GetLink("RR_THIGH");
    this->RR_CALF = this->model->GetLink("RR_CALF");
    this->RR_TIP = this->model->GetLink("RR_TIP");
    this->RL_HIP = this->model->GetLink("RL_HIP");
    this->RL_THIGH = this->model->GetLink("RL_THIGH");
    this->RL_CALF = this->model->GetLink("RL_CALF");
    this->RL_TIP = this->model->GetLink("RL_TIP");

    //JOINT DEFINITION
    this->WAIST_JOINT = this->model->GetJoint("WAIST_JOINT");
    this->FR_HIP_JOINT = this->model->GetJoint("FR_HIP_JOINT");
    this->FR_THIGH_JOINT = this->model->GetJoint("FR_THIGH_JOINT");
    this->FR_CALF_JOINT = this->model->GetJoint("FR_CALF_JOINT");
    this->FR_TIP_JOINT = this->model->GetJoint("FR_TIP_JOINT");
    this->FL_HIP_JOINT = this->model->GetJoint("FL_HIP_JOINT");
    this->FL_THIGH_JOINT = this->model->GetJoint("FL_THIGH_JOINT");
    this->FL_CALF_JOINT = this->model->GetJoint("FL_CALF_JOINT");
    this->FL_TIP_JOINT = this->model->GetJoint("FL_TIP_JOINT");
    this->RR_HIP_JOINT = this->model->GetJoint("RR_HIP_JOINT");
    this->RR_THIGH_JOINT = this->model->GetJoint("RR_THIGH_JOINT");
    this->RR_CALF_JOINT = this->model->GetJoint("RR_CALF_JOINT");
    this->RR_TIP_JOINT = this->model->GetJoint("RR_TIP_JOINT");
    this->RL_HIP_JOINT = this->model->GetJoint("RL_HIP_JOINT");
    this->RL_THIGH_JOINT = this->model->GetJoint("RL_THIGH_JOINT");
    this->RL_CALF_JOINT = this->model->GetJoint("RL_CALF_JOINT");
    this->RL_TIP_JOINT = this->model->GetJoint("RL_TIP_JOINT");

    //setting for getting dt
    this->last_update_time = this->model->GetWorld()->GetSimTime();
    this->update_connection = event::Events::ConnectWorldUpdateBegin(boost::bind(&PongBotQ_plugin::UpdateAlgorithm, this));

    //setting for IMU sensor
    this->Sensor = sensors::get_sensor("IMU");
    this->IMU = std::dynamic_pointer_cast<sensors::ImuSensor>(Sensor);

    //setting for communication
    P_Times = n.advertise<std_msgs::Float64>("times", 1);
    P_angular_velocity_x = n.advertise<std_msgs::Float64>("angular_velocity_x", 1);
    P_angular_velocity_y = n.advertise<std_msgs::Float64>("angular_velocity_y", 1);
    P_angular_velocity_z = n.advertise<std_msgs::Float64>("angular_velocity_z", 1);
    P_FR_force_x = n.advertise<std_msgs::Float64>("FR_force_x", 1);
    P_FR_force_y = n.advertise<std_msgs::Float64>("FR_force_y", 1);
    P_FR_force_z = n.advertise<std_msgs::Float64>("FR_force_z", 1);
    P_FL_force_x = n.advertise<std_msgs::Float64>("FL_force_x", 1);
    P_FL_force_y = n.advertise<std_msgs::Float64>("FL_force_y", 1);
    P_FL_force_z = n.advertise<std_msgs::Float64>("FL_force_z", 1);
    P_RR_force_x = n.advertise<std_msgs::Float64>("RR_force_x", 1);
    P_RR_force_y = n.advertise<std_msgs::Float64>("RR_force_y", 1);
    P_RR_force_z = n.advertise<std_msgs::Float64>("RR_force_z", 1);
    P_RL_force_x = n.advertise<std_msgs::Float64>("RL_force_x", 1);
    P_RL_force_y = n.advertise<std_msgs::Float64>("RL_force_y", 1);
    P_RL_force_z = n.advertise<std_msgs::Float64>("RL_force_z", 1);
    //ros::Rate loop_rate(1000);
}

void gazebo::PongBotQ_plugin::UpdateAlgorithm()
{ //* Writing realtime code here!!


    //setting for getting dt
    common::Time current_time = this->model->GetWorld()->GetSimTime();
    dt = current_time.Double() - this->last_update_time.Double();
    time = time + dt;

    //getting angular_velocity and linear acceleration.
    angular_velocity_x = this->IMU->AngularVelocity(false)[0];
    angular_velocity_y = this->IMU->AngularVelocity(false)[1];
    angular_velocity_z = this->IMU->AngularVelocity(false)[2];
    linear_acc_x = this->IMU->LinearAcceleration(false)[0];
    linear_acc_y = this->IMU->LinearAcceleration(false)[1];
    linear_acc_z = this->IMU->LinearAcceleration(false)[2];

    // Coordinate encoder angle[radian]
    Encoder[0] = this->FR_HIP_JOINT->GetAngle(0).Radian();
    Encoder[1] = this->FR_THIGH_JOINT->GetAngle(0).Radian();
    Encoder[2] = this->FR_CALF_JOINT->GetAngle(0).Radian();
    Encoder[13] = PI / 4; //FR_TIP
    Encoder[5] = this->FL_HIP_JOINT->GetAngle(0).Radian();
    Encoder[4] = this->FL_THIGH_JOINT->GetAngle(0).Radian();
    Encoder[3] = this->FL_CALF_JOINT->GetAngle(0).Radian();
    Encoder[14] = PI / 4; //FL_TIP
    Encoder[6] = this->RR_HIP_JOINT->GetAngle(0).Radian();
    Encoder[7] = this->RR_THIGH_JOINT->GetAngle(0).Radian();
    Encoder[8] = this->RR_CALF_JOINT->GetAngle(0).Radian();
    Encoder[16] = -PI / 4; //RR_TIP
    Encoder[11] = this->RL_HIP_JOINT->GetAngle(0).Radian();
    Encoder[10] = this->RL_THIGH_JOINT->GetAngle(0).Radian();
    Encoder[9] = this->RL_CALF_JOINT->GetAngle(0).Radian();
    Encoder[15] = -PI / 4; //RL_TIP
    Encoder[12] = this->WAIST_JOINT->GetAngle(0).Radian();

    ///getting Force and Torque of Front Right Leg
    wrench = this->FR_TIP_JOINT->GetForceTorque(0);
#if GAZEBO_MAJOR_VERSION >= 8
    force = wrench.body2Force;
    torque = wrench.body2Torque;
#else
    force = wrench.body2Force.Ign();
    torque = wrench.body2Torque.Ign();
#endif

    FR_Force_E[0] = force.X();
    FR_Force_E[1] = force.Y();
    FR_Force_E[2] = force.Z();
    FR_Torque_E[0] = torque.X();
    FR_Torque_E[1] = torque.Y();
    FR_Torque_E[2] = torque.Z();
    FR_C_IE = FR_jointToRotMat(Encoder);
    FR_Force_I = FR_C_IE*FR_Force_E;
    FR_Torque_I = FR_C_IE*FR_Torque_E;

    ///getting Force and Torque of Front Left Leg
    wrench = this->FL_TIP_JOINT->GetForceTorque(0);
#if GAZEBO_MAJOR_VERSION >= 8
    force = wrench.body2Force;
    torque = wrench.body2Torque;
#else
    force = wrench.body2Force.Ign();
    torque = wrench.body2Torque.Ign();
#endif
    FL_Force_E[0] = force.X();
    FL_Force_E[1] = force.Y();
    FL_Force_E[2] = force.Z();
    FL_Torque_E[0] = torque.X();
    FL_Torque_E[1] = torque.Y();
    FL_Torque_E[2] = torque.Z();
    FL_C_IE = FL_jointToRotMat(Encoder);
    FL_Force_I = FL_C_IE*FL_Force_E;
    FL_Torque_I = FL_C_IE*FL_Torque_E;

    ///getting Force and Torque of Rear Right Leg
    wrench = this->RR_TIP_JOINT->GetForceTorque(0);
#if GAZEBO_MAJOR_VERSION >= 8
    force = wrench.body2Force;
    torque = wrench.body2Torque;
#else
    force = wrench.body2Force.Ign();
    torque = wrench.body2Torque.Ign();
#endif
    RR_Force_E[0] = force.X();
    RR_Force_E[1] = force.Y();
    RR_Force_E[2] = force.Z();
    RR_Torque_E[0] = torque.X();
    RR_Torque_E[1] = torque.Y();
    RR_Torque_E[2] = torque.Z();
    RR_C_IE = RR_jointToRotMat(Encoder);
    RR_Force_I = RR_C_IE*RR_Force_E;
    RR_Torque_I = RR_C_IE*RR_Torque_E;

    ///getting Force and Torque of Rear Left Leg
    wrench = this->RL_TIP_JOINT->GetForceTorque(0);
#if GAZEBO_MAJOR_VERSION >= 8
    force = wrench.body2Force;
    torque = wrench.body2Torque;
#else
    force = wrench.body2Force.Ign();
    torque = wrench.body2Torque.Ign();
#endif
    RL_Force_E[0] = force.X();
    RL_Force_E[1] = force.Y();
    RL_Force_E[2] = force.Z();
    RL_Torque_E[0] = torque.X();
    RL_Torque_E[1] = torque.Y();
    RL_Torque_E[2] = torque.Z();
    RL_C_IE = RL_jointToRotMat(Encoder);
    RL_Force_I = RL_C_IE*RL_Force_E;
    RL_Torque_I = RL_C_IE*RL_Torque_E;


    //* calculating errors
    // Angle & Angular velocity of leg
    for (int i = 0; i < 13; i++) {
        angle_vel[i] = (Encoder[i] - pre_Encoder[i]) / dt;
        pre_Encoder[i] = Encoder[i];
        angle_err[i] = (tar_deg[i] * D2R) - Encoder[i];
    }

    //RBDL

    basePosOri << 0, 0, 0.7, 0, 0, 0;
    baseVel << 0, 0, 0, 0, 0, 0;
    //jointAngle << 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
    //jointVel << 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
    jointVel << angle_vel[11], angle_vel[10], angle_vel[9], angle_vel[6], angle_vel[7], angle_vel[8], angle_vel[12], angle_vel[5], angle_vel[4], angle_vel[3], angle_vel[0], angle_vel[1], angle_vel[2];
    jointAngle << Encoder[11], Encoder[10], Encoder[9], Encoder[6], Encoder[7], Encoder[8], Encoder[12], Encoder[5], Encoder[4], Encoder[3], Encoder[0], Encoder[1], Encoder[2];
    getRobotState(basePosOri, baseVel, jointAngle, jointVel);
    CTC();

    tau_G[0] = joint[10].torque; //FR_HIP
    tau_G[1] = joint[11].torque; //FR_THIGH
    tau_G[2] = joint[12].torque; //FR_CALF
    tau_G[5] = joint[7].torque; //FL_HIP
    tau_G[4] = joint[8].torque; //FL_THIGH
    tau_G[3] = joint[9].torque; //FL_CALF
    tau_G[6] = joint[3].torque; //RR_HIP
    tau_G[7] = joint[4].torque; //RR_THIGH
    tau_G[8] = joint[5].torque; //RR_CALF
    tau_G[11] = joint[0].torque; //RL_HIP
    tau_G[10] = joint[1].torque; //RL_THIGH
    tau_G[9] = joint[2].torque; //RL_CALF
    tau_G[12] = joint[6].torque; //Waist

    //* Control law of PD Control + Gravity Compensation
    tau[0] = 10 * (angle_err[0]) + 2 * (0 - angle_vel[0]) + tau_G[0]; //FR_HIP
    tau[1] = 20 * (angle_err[1]) + 5 * (0 - angle_vel[1]) + tau_G[1]; //FR_THIGH
    tau[2] = 20 * (angle_err[2]) + 5 * (0 - angle_vel[2]) + tau_G[2]; //FR_CALF
    tau[5] = 10 * (angle_err[5]) + 2 * (0 - angle_vel[5]) + tau_G[5]; //FL_HIP
    tau[4] = 20 * (angle_err[4]) + 5 * (0 - angle_vel[4]) + tau_G[4]; //FL_THIGH
    tau[3] = 20 * (angle_err[3]) + 5 * (0 - angle_vel[3]) + tau_G[3]; //FL_CALF
    tau[6] = 10 * (angle_err[6]) + 2 * (0 - angle_vel[6]) + tau_G[6]; //RR_HIP
    tau[7] = 20 * (angle_err[7]) + 5 * (0 - angle_vel[7]) + tau_G[7]; //RR_THIGH
    tau[8] = 20 * (angle_err[8]) + 5 * (0 - angle_vel[8]) + tau_G[8]; //RR_CALF
    tau[11] = 10 * (angle_err[11]) + 2 * (0 - angle_vel[11]) + tau_G[11]; //RL_HIP
    tau[10] = 20 * (angle_err[10]) + 5 * (0 - angle_vel[10]) + tau_G[10]; //RL_THIGH
    tau[9] = 20 * (angle_err[9]) + 5 * (0 - angle_vel[9]) + tau_G[9]; //RL_CALF
    tau[12] = 10 * (angle_err[12]) + 2 * (0 - angle_vel[12]) + tau_G[12]; //Waist
    
    //* Control law of PD Control
    //tau[0] = 200 * (angle_err[0]) + 10 * (0 - angle_vel[0]);//FR_HIP
    //tau[1] = 400 * (angle_err[1]) + 10 * (0 - angle_vel[1]); //FR_THIGH
    //tau[2] = 400 * (angle_err[2]) + 10 * (0 - angle_vel[2]); //FR_CALF
    //tau[5] = 200 * (angle_err[5]) + 10 * (0 - angle_vel[5]); //FL_HIP
    //tau[4] = 400 * (angle_err[4]) + 10 * (0 - angle_vel[4]); //FL_THIGH
    //tau[3] = 400 * (angle_err[3]) + 10 * (0 - angle_vel[3]); //FL_CALF
    //tau[6] = 200 * (angle_err[6]) + 10 * (0 - angle_vel[6]); //RR_HIP
    //tau[7] = 400 * (angle_err[7]) + 10 * (0 - angle_vel[7]); //RR_THIGH
    //tau[8] = 400 * (angle_err[8]) + 10 * (0 - angle_vel[8]); //RR_CALF
    //tau[11] = 200 * (angle_err[11]) + 10 * (0 - angle_vel[11]); //RL_HIP
    //tau[10] = 400 * (angle_err[10]) + 10 * (0 - angle_vel[10]); //RL_THIGH
    //tau[9] = 400 * (angle_err[9]) + 10 * (0 - angle_vel[9]); //RL_CALF
    //tau[12] = 200 * (angle_err[12]) + 10 * (0 - angle_vel[12]); //Waist

    //* Applying torques
    this->FR_HIP_JOINT->SetForce(1, tau[0]);
    this->FR_THIGH_JOINT->SetForce(1, tau[1]);
    this->FR_CALF_JOINT->SetForce(1, tau[2]);
    this->FL_HIP_JOINT->SetForce(1, tau[5]);
    this->FL_THIGH_JOINT->SetForce(1, tau[4]);
    this->FL_CALF_JOINT->SetForce(1, tau[3]);
    this->RR_HIP_JOINT->SetForce(1, tau[6]);
    this->RR_THIGH_JOINT->SetForce(1, tau[7]);
    this->RR_CALF_JOINT->SetForce(1, tau[8]);
    this->RL_HIP_JOINT->SetForce(1, tau[11]);
    this->RL_THIGH_JOINT->SetForce(1, tau[10]);
    this->RL_CALF_JOINT->SetForce(1, tau[9]);
    this->WAIST_JOINT->SetForce(1, tau[12]);

    /*
    if (this->pid_FR_HR.GetCmd() >= 1000 || this->pid_FR_HR.GetCmd() <= -1000) {
        printf("pid_FR_HR = %f\n", this->pid_FR_HR.GetCmd());
    }
    if (this->pid_FR_HP.GetCmd() >= 1000 || this->pid_FR_HP.GetCmd() <= -1000) {
        printf("pid_FR_HP = %f\n", this->pid_FR_HP.GetCmd());
    }
    if (this->pid_FR_KN.GetCmd() >= 1000 || this->pid_FR_KN.GetCmd() <= -1000) {
        printf("pid_FR_KN = %f\n", this->pid_FR_KN.GetCmd());
    }
    if (this->pid_FL_HR.GetCmd() >= 1000 || this->pid_FL_HR.GetCmd() <= -1000) {
        printf("pid_FL_HR = %f\n", this->pid_FL_HR.GetCmd());
    }
    if (this->pid_FL_HP.GetCmd() >= 1000 || this->pid_FL_HP.GetCmd() <= -1000) {
        printf("pid_FL_HP = %f\n", this->pid_FL_HP.GetCmd());
    }
    if (this->pid_FL_KN.GetCmd() >= 1000 || this->pid_FL_KN.GetCmd() <= -1000) {
        printf("pid_FL_KN = %f\n", this->pid_FL_KN.GetCmd());
    }
    if (this->pid_RL_HR.GetCmd() >= 1000 || this->pid_RL_HR.GetCmd() <= -1000) {
        printf("pid_RL_HR = %f\n", this->pid_RL_HR.GetCmd());
    }
    if (this->pid_RL_HP.GetCmd() >= 1000 || this->pid_RL_HP.GetCmd() <= -1000) {
        printf("pid_RL_HP = %f\n", this->pid_RL_HP.GetCmd());
    }
    if (this->pid_RL_KN.GetCmd() >= 1000 || this->pid_RL_KN.GetCmd() <= -1000) {
        printf("pid_RL_KN = %f\n", this->pid_RL_KN.GetCmd());
    }
    if (this->pid_RR_HR.GetCmd() >= 1000 || this->pid_RR_HR.GetCmd() <= -1000) {
        printf("pid_RR_HR = %f\n", this->pid_RR_HR.GetCmd());
    }
    if (this->pid_RR_HP.GetCmd() >= 1000 || this->pid_RR_HP.GetCmd() <= -1000) {
        printf("pid_RR_HP = %f\n", this->pid_RR_HP.GetCmd());
    }
    if (this->pid_RR_KN.GetCmd() >= 1000 || this->pid_RR_KN.GetCmd() <= -1000) {
        printf("pid_RR_KN = %f\n", this->pid_RR_KN.GetCmd());
    }
    if (this->pid_WAIST.GetCmd() >= 1000 || this->pid_WAIST.GetCmd() <= -1000) {
        printf("pid_WAIST = %f\n", this->pid_WAIST.GetCmd());
    }
     */

    //setting for getting dt
    this->last_update_time = current_time;

    //getting readable angular_velocity data
    m_Times.data = current_time.Double();
    m_angular_velocity_x.data = angular_velocity_x;
    m_angular_velocity_y.data = angular_velocity_y;
    m_angular_velocity_z.data = angular_velocity_z;

    //getting readable force data
    m_FR_force_x.data = FR_Force_I[0];
    m_FR_force_y.data = FR_Force_I[1];
    m_FR_force_z.data = FR_Force_I[2];
    m_FL_force_x.data = FL_Force_I[0];
    m_FL_force_y.data = FL_Force_I[1];
    m_FL_force_z.data = FL_Force_I[2];
    m_RR_force_x.data = RR_Force_I[0];
    m_RR_force_y.data = RR_Force_I[1];
    m_RR_force_z.data = RR_Force_I[2];
    m_RL_force_x.data = RL_Force_I[0];
    m_RL_force_y.data = RL_Force_I[1];
    m_RL_force_z.data = RL_Force_I[2];

    //publishing data
    P_Times.publish(m_Times);
    P_angular_velocity_x.publish(m_angular_velocity_x);
    P_angular_velocity_y.publish(m_angular_velocity_y);
    P_angular_velocity_z.publish(m_angular_velocity_z);
    P_FR_force_x.publish(m_FR_force_x);
    P_FR_force_y.publish(m_FR_force_y);
    P_FR_force_z.publish(m_FR_force_z);
    P_FL_force_x.publish(m_FL_force_x);
    P_FL_force_y.publish(m_FL_force_y);
    P_FL_force_z.publish(m_FL_force_z);
    P_RR_force_x.publish(m_RR_force_x);
    P_RR_force_y.publish(m_RR_force_y);
    P_RR_force_z.publish(m_RR_force_z);
    P_RL_force_x.publish(m_RL_force_x);
    P_RL_force_y.publish(m_RL_force_y);
    P_RL_force_z.publish(m_RL_force_z);
}

void getCurrentJoint(VectorNd Angle, VectorNd Vel)
{
    for (int nJoint = 0; nJoint < Joint_Num; nJoint++) {
        joint[nJoint].currentAngle = Angle(nJoint);
        joint[nJoint].currentVel = Vel(nJoint);
    }
}

void getRobotState(VectorNd BasePosOri, VectorNd BaseVel, VectorNd JointAngle, VectorNd JointVel)
{
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

    getCurrentJoint(JointAngle, JointVel);
}

void CTC(void)
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

    for (int nJoint = 0; nJoint < Joint_Num; nJoint++) {
        RobotState(6 + nJoint) = joint[nJoint].currentAngle;
        RobotStatedot(6 + nJoint) = joint[nJoint].currentVel;
        //std::cout << RobotStatedot(6 + nJoint) << std::endl;
    }

    MatrixNd M_term = MatrixNd::Zero(m_nDOF, m_nDOF);
    CompositeRigidBodyAlgorithm(*pongbot_q_model, RobotState, M_term);

    VectorNd hatNonLinearEffects = VectorNd::Zero(m_nDOF);
    NonlinearEffects(*pongbot_q_model, RobotState, RobotStatedot, hatNonLinearEffects);

    VectorNd G_term = VectorNd::Zero(m_nDOF);
    NonlinearEffects(*pongbot_q_model, RobotState, VectorNd::Zero(m_nDOF), G_term);

    VectorNd C_term = VectorNd::Zero(m_nDOF);
    C_term = hatNonLinearEffects - G_term;

    for (int nJoint = 0; nJoint < Joint_Num; nJoint++) {
        joint[nJoint].torque = G_term(6 + nJoint);
        //std::cout << joint[nJoint].torque << std::endl;
    }
    //std::cout << "/" << std::endl;
}
