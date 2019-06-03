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
        double FR_total_force, FL_total_force, RR_total_force, RL_total_force;
        double total_force;

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
        ros::Publisher P_FR_total_force;
        ros::Publisher P_FL_total_force;
        ros::Publisher P_RR_total_force;
        ros::Publisher P_RL_total_force;

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
        std_msgs::Float64 m_FR_total_force;
        std_msgs::Float64 m_FL_total_force;
        std_msgs::Float64 m_RR_total_force;
        std_msgs::Float64 m_RL_total_force;

        // DH PARA
        double tar_deg[13] = {0, 30, -60,
            -60, 30, 0,
            0, -30, 60,
            60, -30, 0,
            0};
        double angle_err[13];

        // Adding CRobot Class by BKCho
        CRobot PongBotQ;

    public:

        //For model load
        void Load(physics::ModelPtr _model, sdf::ElementPtr /*_sdf*/);
        void UpdateAlgorithm();
    };
    GZ_REGISTER_MODEL_PLUGIN(PongBotQ_plugin);
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
    P_FR_total_force = n.advertise<std_msgs::Float64>("FR_total_force", 1);
    P_FL_total_force = n.advertise<std_msgs::Float64>("FL_total_force", 1);
    P_RR_total_force = n.advertise<std_msgs::Float64>("RR_total_force", 1);
    P_RL_total_force = n.advertise<std_msgs::Float64>("RL_total_force", 1);

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

    //getting Force and Torque from FT sensor
    ///getting Force and Torque of Front Right Leg
    wrench = this->FR_TIP_JOINT->GetForceTorque(0);
#if GAZEBO_MAJOR_VERSION >= 8
    force = wrench.body2Force;
    torque = wrench.body2Torque;
#else              
    force = wrench.body2Force.Ign();
    torque = wrench.body2Torque.Ign();
#endif

    FR_force_x = force.X();
    FR_force_y = force.Y();
    FR_force_z = force.Z();
    FR_torque_x = torque.X();
    FR_torque_y = torque.Y();
    FR_torque_z = torque.Z();

    ///getting Force and Torque of Front Left Leg
    wrench = this->FL_TIP_JOINT->GetForceTorque(0);
#if GAZEBO_MAJOR_VERSION >= 8
    force = wrench.body2Force;
    torque = wrench.body2Torque;
#else              
    force = wrench.body2Force.Ign();
    torque = wrench.body2Torque.Ign();
#endif
    FL_force_x = force.X();
    FL_force_y = force.Y();
    FL_force_z = force.Z();
    FL_torque_x = torque.X();
    FL_torque_y = torque.Y();
    FL_torque_z = torque.Z();

    ///getting Force and Torque of Rear Right Leg
    wrench = this->RR_TIP_JOINT->GetForceTorque(0);
#if GAZEBO_MAJOR_VERSION >= 8
    force = wrench.body2Force;
    torque = wrench.body2Torque;
#else              
    force = wrench.body2Force.Ign();
    torque = wrench.body2Torque.Ign();
#endif
    RR_force_x = force.X();
    RR_force_y = force.Y();
    RR_force_z = force.Z();
    RR_torque_x = torque.X();
    RR_torque_y = torque.Y();
    RR_torque_z = torque.Z();

    ///getting Force and Torque of Rear Left Leg
    wrench = this->RL_TIP_JOINT->GetForceTorque(0);
#if GAZEBO_MAJOR_VERSION >= 8
    force = wrench.body2Force;
    torque = wrench.body2Torque;
#else              
    force = wrench.body2Force.Ign();
    torque = wrench.body2Torque.Ign();
#endif
    RL_force_x = force.X();
    RL_force_y = force.Y();
    RL_force_z = force.Z();
    RL_torque_x = torque.X();
    RL_torque_y = torque.Y();
    RL_torque_z = torque.Z();

    FR_total_force = sqrt(pow(FR_force_x, 2) + pow(FR_force_y, 2) + pow(FR_force_z, 2));
    FL_total_force = sqrt(pow(FL_force_x, 2) + pow(FL_force_y, 2) + pow(FL_force_z, 2));
    RR_total_force = sqrt(pow(RR_force_x, 2) + pow(RR_force_y, 2) + pow(RR_force_z, 2));
    RL_total_force = sqrt(pow(RL_force_x, 2) + pow(RL_force_y, 2) + pow(RL_force_z, 2));

    total_force = FR_total_force + FR_total_force + FR_total_force + FR_total_force;
    //std::cout << "total_force=" << std::endl << total_force << std::endl;

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
    m_FR_force_x.data = FR_force_x;
    m_FR_force_y.data = FR_force_y;
    m_FR_force_z.data = FR_force_z;
    m_FL_force_x.data = FL_force_x;
    m_FL_force_y.data = FL_force_y;
    m_FL_force_z.data = FL_force_z;
    m_RR_force_x.data = RR_force_x;
    m_RR_force_y.data = RR_force_y;
    m_RR_force_z.data = RR_force_z;
    m_RL_force_x.data = RL_force_x;
    m_RL_force_y.data = RL_force_y;
    m_RL_force_z.data = RL_force_z;
    m_FR_total_force.data = FR_total_force;
    m_FL_total_force.data = FL_total_force;
    m_RR_total_force.data = RR_total_force;
    m_RL_total_force.data = RL_total_force;

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
    P_FR_total_force.publish(m_FR_total_force);
    P_FL_total_force.publish(m_FL_total_force);
    P_RR_total_force.publish(m_RR_total_force);
    P_RL_total_force.publish(m_RL_total_force);
}
