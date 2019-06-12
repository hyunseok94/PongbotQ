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

using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;

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
        double angle_err[13];
        double encoder_angle[13];
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
    tmp_m <<  1, 0, 0, 0\
            , 0, 1, 0, 0\
            , 0, 0, 1, 0\
            , 0, 0, 0, 1;
    return tmp_m;
}

MatrixXd F_jointToTransform01(VectorXd q)
{
    MatrixXd tmp_m(4, 4);
    double qq = q(12);
    tmp_m <<  cos(qq), -sin(qq), 0,   0.35\
            , sin(qq),  cos(qq), 0,      0\
            ,       0,        0, 1, 0.0352\
            ,       0,        0, 0,     1;
    return tmp_m;
}

MatrixXd FL_jointToTransform12(VectorXd q)
{
    MatrixXd tmp_m(4, 4);
    double qq = q(5);
    tmp_m <<   1,       0,        0,    0.35\
            ,  0, cos(qq), -sin(qq),   0.115\
            ,  0, sin(qq),  cos(qq), -0.0355\
            ,  0,       0,        0,       1;
    return tmp_m;
}

MatrixXd FL_jointToTransform23(VectorXd q)
{
    MatrixXd tmp_m(4, 4);
    double qq = q(4);
    tmp_m <<   cos(qq), 0, sin(qq),      0\
            ,        0, 1,       0,  0.105\
            , -sin(qq), 0, cos(qq),      0\
            ,        0, 0,       0,      1;
    return tmp_m;
}

MatrixXd FL_jointToTransform34(VectorXd q)
{
    MatrixXd tmp_m(4, 4);
    double qq = q(3);
    tmp_m <<   cos(qq), 0, sin(qq),      0\
            ,        0, 1,       0,      0\
            , -sin(qq), 0, cos(qq), -0.305\
            ,        0, 0,       0,     1;
    return tmp_m;
}

MatrixXd FL_jointToTransform4E(VectorXd q)
{
    MatrixXd tmp_m(4, 4);
    double qq = q(14);
    tmp_m <<  cos(qq), 0, sin(qq), -0.015\
           ,        0, 1,       0,      0\
           , -sin(qq), 0, cos(qq), -0.288\
           ,        0, 0,       0,     1;
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
    tmp_m <<   1,       0,        0,    0.35\
            ,  0, cos(qq), -sin(qq),  -0.115\
            ,  0, sin(qq),  cos(qq), -0.0355\
            ,  0,       0,        0,       1;
    return tmp_m;
}

MatrixXd FR_jointToTransform23(VectorXd q)
{
    MatrixXd tmp_m(4, 4);
    double qq = q(1);
    tmp_m <<   cos(qq), 0, sin(qq),      0\
            ,        0, 1,       0, -0.105\
            , -sin(qq), 0, cos(qq),      0\
            ,        0, 0,       0,      1;
    return tmp_m;
}

MatrixXd FR_jointToTransform34(VectorXd q)
{
    MatrixXd tmp_m(4, 4);
    double qq = q(2);
    tmp_m <<   cos(qq), 0, sin(qq),      0\
            ,        0, 1,       0,      0\
            , -sin(qq), 0, cos(qq), -0.305\
            ,        0, 0,       0,     1;
    return tmp_m;
}

MatrixXd FR_jointToTransform4E(VectorXd q)
{
    MatrixXd tmp_m(4, 4);
    double qq = q(13);
    tmp_m <<  cos(qq), 0, sin(qq), -0.015\
           ,        0, 1,       0,      0\
           , -sin(qq), 0, cos(qq), -0.288\
           ,        0, 0,       0,     1;
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
    tmp_m <<   1,       0,        0,       0\
            ,  0, cos(qq), -sin(qq),   0.115\
            ,  0, sin(qq),  cos(qq),       0\
            ,  0,       0,        0,       1;
    return tmp_m;
}

MatrixXd RL_jointToTransform12(VectorXd q)
{
    MatrixXd tmp_m(4, 4);
    double qq = q(7);
    tmp_m <<   cos(qq), 0, sin(qq),      0\
            ,        0, 1,       0,  0.105\
            , -sin(qq), 0, cos(qq),      0\
            ,        0, 0,       0,      1;
    return tmp_m;
}

MatrixXd RL_jointToTransform23(VectorXd q)
{
    MatrixXd tmp_m(4, 4);
    double qq = q(8);
    tmp_m <<   cos(qq), 0, sin(qq),      0\
            ,        0, 1,       0,      0\
            , -sin(qq), 0, cos(qq), -0.305\
            ,        0, 0,       0,      1;
    return tmp_m;
}

MatrixXd RL_jointToTransform3E(VectorXd q)
{
    MatrixXd tmp_m(4, 4);
    double qq = q(16);
    tmp_m <<   cos(qq), 0, sin(qq),  0.015\
            ,        0, 1,       0,      0\
            , -sin(qq), 0, cos(qq), -0.288\
            ,        0, 0,       0,      1;
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
    tmp_m <<   1,       0,        0,        0\
            ,  0, cos(qq), -sin(qq),   -0.115\
            ,  0, sin(qq),  cos(qq),        0\
            ,  0,       0,        0,        1;
    return tmp_m;
}

MatrixXd RR_jointToTransform12(VectorXd q)
{
    MatrixXd tmp_m(4, 4);
    double qq = q(10);
    tmp_m <<   cos(qq), 0, sin(qq),       0\
            ,        0, 1,       0,  -0.105\
            , -sin(qq), 0, cos(qq),       0\
            ,        0, 0,       0,       1;
    return tmp_m;
}

MatrixXd RR_jointToTransform23(VectorXd q)
{
    MatrixXd tmp_m(4, 4);
    double qq = q(9);
    tmp_m <<   cos(qq), 0, sin(qq),      0\
            ,        0, 1,       0,      0\
            , -sin(qq), 0, cos(qq), -0.305\
            ,        0, 0,       0,      1;
    return tmp_m;
}

MatrixXd RR_jointToTransform3E(VectorXd q)
{
    MatrixXd tmp_m(4, 4);
    double qq = q(15);
    tmp_m <<   cos(qq), 0, sin(qq),  0.015\
            ,        0, 1,       0,      0\
            , -sin(qq), 0, cos(qq), -0.288\
            ,        0, 0,       0,      1;
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
    // model = link + joint +sensor
    this->model = _model;

    rbdl_check_api_version(RBDL_API_VERSION);

    int version_test;
    version_test = rbdl_get_api_version();
    printf("rbdl api version = %d\n", version_test);

    Model* pongbot_q_model = new Model();
    //Addons::URDFReadFromFile("/root/.gazebo/models/PONGBOT_Q_V2/urdf/PONGBOT_Q_V2.urdf", pongbot_q_model, true, true);
    Addons::URDFReadFromFile("/home/hyunseok/.gazebo/models/PONGBOT_Q_V2/urdf/PONGBOT_Q_V2.urdf", pongbot_q_model, true, true);

    PongBotQ.setRobotModel(pongbot_q_model);

    //LINK DEFINITION
    this->REAR_BODY = this->model->GetLink("REAR_BODY");
    this->FRONT_BODY = this->model->GetLink("FRONT_BODY");
    this->FL_HIP = this->model->GetLink("FL_HIP");
    this->FL_THIGH = this->model->GetLink("FL_THIGH");
    this->FL_CALF = this->model->GetLink("FL_CALF");
    this->FL_TIP = this->model->GetLink("FL_TIP");
    this->FR_HIP = this->model->GetLink("FR_HIP");
    this->FR_THIGH = this->model->GetLink("FR_THIGH");
    this->FR_CALF = this->model->GetLink("FR_CALF");
    this->FR_TIP = this->model->GetLink("FR_TIP");
    this->RL_HIP = this->model->GetLink("RL_HIP");
    this->RL_THIGH = this->model->GetLink("RL_THIGH");
    this->RL_CALF = this->model->GetLink("RL_CALF");
    this->RL_TIP = this->model->GetLink("RL_TIP");
    this->RR_HIP = this->model->GetLink("RR_HIP");
    this->RR_THIGH = this->model->GetLink("RR_THIGH");
    this->RR_CALF = this->model->GetLink("RR_CALF");
    this->RR_TIP = this->model->GetLink("RR_TIP");

    //JOINT DEFINITION
    this->WAIST_JOINT = this->model->GetJoint("WAIST_JOINT");
    this->FL_HIP_JOINT = this->model->GetJoint("FL_HIP_JOINT");
    this->FL_THIGH_JOINT = this->model->GetJoint("FL_THIGH_JOINT");
    this->FL_CALF_JOINT = this->model->GetJoint("FL_CALF_JOINT");
    this->FL_TIP_JOINT = this->model->GetJoint("FL_TIP_JOINT");
    this->FR_HIP_JOINT = this->model->GetJoint("FR_HIP_JOINT");
    this->FR_THIGH_JOINT = this->model->GetJoint("FR_THIGH_JOINT");
    this->FR_CALF_JOINT = this->model->GetJoint("FR_CALF_JOINT");
    this->FR_TIP_JOINT = this->model->GetJoint("FR_TIP_JOINT");
    this->RL_HIP_JOINT = this->model->GetJoint("RL_HIP_JOINT");
    this->RL_THIGH_JOINT = this->model->GetJoint("RL_THIGH_JOINT");
    this->RL_CALF_JOINT = this->model->GetJoint("RL_CALF_JOINT");
    this->RL_TIP_JOINT = this->model->GetJoint("RL_TIP_JOINT");
    this->RR_HIP_JOINT = this->model->GetJoint("RR_HIP_JOINT");
    this->RR_THIGH_JOINT = this->model->GetJoint("RR_THIGH_JOINT");
    this->RR_CALF_JOINT = this->model->GetJoint("RR_CALF_JOINT");
    this->RR_TIP_JOINT = this->model->GetJoint("RR_TIP_JOINT");

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
 
    // =================== PID GAIN TUNNING ==================== //

    //this->pid_FR_HR.Init(50, 0.1, 5, 200, -200, 1000, -1000);
    //this->pid_FR_HP.Init(50, 0.1, 5, 200, -200, 1000, -1000);
    //this->pid_FR_KN.Init(50, 0.1, 5, 200, -200, 1000, -1000);
    //this->pid_FL_HR.Init(50, 0.1, 5, 200, -200, 1000, -1000);
    //this->pid_FL_HP.Init(50, 0.1, 5, 200, -200, 1000, -1000);
    //this->pid_FL_KN.Init(50, 0.1, 5, 200, -200, 1000, -1000);
    
    this->pid_FR_HR.Init(200, 0.1, 10, 200, -200, 1000, -1000);
    this->pid_FR_HP.Init(400, 0.3, 10, 200, -200, 1000, -1000);
    this->pid_FR_KN.Init(400, 0.3, 10, 200, -200, 1000, -1000);
    this->pid_FL_HR.Init(200, 0.1, 10, 200, -200, 1000, -1000);
    this->pid_FL_HP.Init(400, 0.3, 10, 200, -200, 1000, -1000);
    this->pid_FL_KN.Init(400, 0.3, 10, 200, -200, 1000, -1000);
    this->pid_RL_HR.Init(200, 0.1, 10, 200, -200, 1000, -1000);
    this->pid_RL_HP.Init(400, 0.3, 10, 200, -200, 1000, -1000);
    this->pid_RL_KN.Init(400, 0.3, 10, 200, -200, 1000, -1000);
    this->pid_RR_HR.Init(200, 0.1, 10, 200, -200, 1000, -1000);
    this->pid_RR_HP.Init(400, 0.3, 10, 200, -200, 1000, -1000);
    this->pid_RR_KN.Init(400, 0.3, 10, 200, -200, 1000, -1000); 
    this->pid_WAIST.Init(200, 0.1, 10, 200, -200, 1000, -1000);


    //ros::Rate loop_rate(1000);
}

void gazebo::PongBotQ_plugin::UpdateAlgorithm()
{ //* Writing realtime code here!!
    VectorXd Encoder(17); 
    MatrixXd FR_C_IE(3,3), FL_C_IE(3,3), RR_C_IE(3,3), RL_C_IE(3,3);
    VectorXd FR_Force_E(3), FL_Force_E(3), RR_Force_E(3), RL_Force_E(3);
    VectorXd FR_Force_I(3), FL_Force_I(3), RR_Force_I(3), RL_Force_I(3);
    VectorXd FR_Torque_E(3), FL_Torque_E(3), RR_Torque_E(3), RL_Torque_E(3);
    VectorXd FR_Torque_I(3), FL_Torque_I(3), RR_Torque_I(3), RL_Torque_I(3);
    
    
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

     //coordinate encoder angle[radian)]
    Encoder[0]=this->FR_HIP_JOINT->GetAngle(0).Radian();
    Encoder[1]=this->FR_THIGH_JOINT->GetAngle(0).Radian();
    Encoder[2]=this->FR_CALF_JOINT->GetAngle(0).Radian();
    Encoder[13]=PI/4; //FR_TIP
    Encoder[5]=this->FL_HIP_JOINT->GetAngle(0).Radian();
    Encoder[4]=this->FL_THIGH_JOINT->GetAngle(0).Radian();
    Encoder[3]=this->FL_CALF_JOINT->GetAngle(0).Radian();
    Encoder[14]=PI/4; //FL_TIP
    Encoder[6]=this->RL_HIP_JOINT->GetAngle(0).Radian();
    Encoder[7]=this->RL_THIGH_JOINT->GetAngle(0).Radian();
    Encoder[8]=this->RL_CALF_JOINT->GetAngle(0).Radian();
    Encoder[16]=-PI/4; //RL_TIP
    Encoder[11]=this->RR_HIP_JOINT->GetAngle(0).Radian();
    Encoder[10]=this->RR_THIGH_JOINT->GetAngle(0).Radian();
    Encoder[9]=this->RR_CALF_JOINT->GetAngle(0).Radian();
    Encoder[15]=-PI/4; //RR_TIP
    Encoder[12]=this->WAIST_JOINT->GetAngle(0).Radian();
       
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
    FR_C_IE=FR_jointToRotMat(Encoder);
    FR_Force_I=FR_C_IE*FR_Force_E;
    FR_Torque_I=FR_C_IE*FR_Torque_E;
    //std::cout << "FR=" << std::endl << FR_Force_I << std::endl;
    
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
    FL_C_IE=FL_jointToRotMat(Encoder);
    FL_Force_I=FL_C_IE*FL_Force_E;
    FL_Torque_I=FL_C_IE*FL_Torque_E;
    //std::cout << "FL=" << std::endl << FL_Force_I << std::endl;
     
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
    RR_C_IE=RR_jointToRotMat(Encoder);
    RR_Force_I=RR_C_IE*RR_Force_E;
    RR_Torque_I=RR_C_IE*RR_Torque_E;
    //std::cout << "RR=" << std::endl << RR_Force_I << std::endl;
    
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
    RL_C_IE=RL_jointToRotMat(Encoder);
    RL_Force_I=RL_C_IE*RL_Force_E;
    RL_Torque_I=RL_C_IE*RL_Torque_E;
    //std::cout << "RL=" << std::endl << RL_Force_I << std::endl;
    
    //* calculating errors
    angle_err[0] = this->FR_HIP_JOINT->GetAngle(0).Radian() - (tar_deg[0] * D2R);
    angle_err[1] = this->FR_THIGH_JOINT->GetAngle(0).Radian() - (tar_deg[1] * D2R);
    angle_err[2] = this->FR_CALF_JOINT->GetAngle(0).Radian() - (tar_deg[2] * D2R);
    angle_err[5] = this->FL_HIP_JOINT->GetAngle(0).Radian() - (tar_deg[5] * D2R);
    angle_err[4] = this->FL_THIGH_JOINT->GetAngle(0).Radian() - (tar_deg[4] * D2R);
    angle_err[3] = this->FL_CALF_JOINT->GetAngle(0).Radian() - (tar_deg[3] * D2R);
    angle_err[6] = this->RL_HIP_JOINT->GetAngle(0).Radian() - (tar_deg[6] * D2R);
    angle_err[7] = this->RL_THIGH_JOINT->GetAngle(0).Radian() - (tar_deg[7] * D2R);
    angle_err[8] = this->RL_CALF_JOINT->GetAngle(0).Radian() - (tar_deg[8] * D2R);
    angle_err[11] = this->RR_HIP_JOINT->GetAngle(0).Radian() - (tar_deg[11] * D2R);
    angle_err[10] = this->RR_THIGH_JOINT->GetAngle(0).Radian() - (tar_deg[10] * D2R);
    angle_err[9] = this->RR_CALF_JOINT->GetAngle(0).Radian() - (tar_deg[9] * D2R);
    angle_err[12] = this->WAIST_JOINT->GetAngle(0).Radian() - (tar_deg[12] * D2R);

    //* Control law
    this->pid_FR_HR.Update(angle_err[0], dt);
    this->pid_FR_HP.Update(angle_err[1], dt);
    this->pid_FR_KN.Update(angle_err[2], dt);
    this->pid_FL_HR.Update(angle_err[5], dt);
    this->pid_FL_HP.Update(angle_err[4], dt);
    this->pid_FL_KN.Update(angle_err[3], dt);
    this->pid_RL_HR.Update(angle_err[6], dt);
    this->pid_RL_HP.Update(angle_err[7], dt);
    this->pid_RL_KN.Update(angle_err[8], dt);
    this->pid_RR_HR.Update(angle_err[11], dt);
    this->pid_RR_HP.Update(angle_err[10], dt);
    this->pid_RR_KN.Update(angle_err[9], dt);
    this->pid_WAIST.Update(angle_err[12], dt);

    //* Applying torques
    this->FR_HIP_JOINT->SetForce(1, this->pid_FR_HR.GetCmd());
    this->FR_THIGH_JOINT->SetForce(1, this->pid_FR_HP.GetCmd());
    this->FR_CALF_JOINT->SetForce(1, this->pid_FR_KN.GetCmd());
    this->FL_HIP_JOINT->SetForce(1, this->pid_FL_HR.GetCmd());
    this->FL_THIGH_JOINT->SetForce(1, this->pid_FL_HP.GetCmd());
    this->FL_CALF_JOINT->SetForce(1, this->pid_FL_KN.GetCmd());
    this->RL_HIP_JOINT->SetForce(1, this->pid_RL_HR.GetCmd());
    this->RL_THIGH_JOINT->SetForce(1, this->pid_RL_HP.GetCmd());
    this->RL_CALF_JOINT->SetForce(1, this->pid_RL_KN.GetCmd());
    this->RR_HIP_JOINT->SetForce(1, this->pid_RR_HR.GetCmd());
    this->RR_THIGH_JOINT->SetForce(1, this->pid_RR_HP.GetCmd());
    this->RR_CALF_JOINT->SetForce(1, this->pid_RR_KN.GetCmd());
    this->WAIST_JOINT->SetForce(1, this->pid_WAIST.GetCmd());

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

