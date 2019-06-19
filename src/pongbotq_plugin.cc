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
#include <sensor_msgs/JointState.h>             //for rviz
#include <tf/transform_broadcaster.h>           //for rviz

#include "CRobot.h" // by BKCho

//#define PI      3.141592
#define D2R     PI/180
#define R2D     180/PI

using Eigen::MatrixXd;
using Eigen::VectorXd;
using Eigen::Vector3d;

Model* pongbot_q_model = new Model();

using namespace RigidBodyDynamics::Math;

namespace gazebo
{

    class PongBotQ_plugin : public ModelPlugin
    {
        physics::LinkPtr REAR_BODY;
        physics::LinkPtr FRONT_BODY;
        physics::LinkPtr RL_HIP;
        physics::LinkPtr RL_THIGH;
        physics::LinkPtr RL_CALF;
        physics::LinkPtr RL_TIP;
        physics::LinkPtr RR_HIP;
        physics::LinkPtr RR_THIGH;
        physics::LinkPtr RR_CALF;
        physics::LinkPtr RR_TIP;
        physics::LinkPtr FL_HIP;
        physics::LinkPtr FL_THIGH;
        physics::LinkPtr FL_CALF;
        physics::LinkPtr FL_TIP;
        physics::LinkPtr FR_HIP;
        physics::LinkPtr FR_THIGH;
        physics::LinkPtr FR_CALF;
        physics::LinkPtr FR_TIP;

        physics::JointPtr RL_HIP_JOINT;
        physics::JointPtr RL_THIGH_JOINT;
        physics::JointPtr RL_CALF_JOINT;
        physics::JointPtr RL_TIP_JOINT;
        physics::JointPtr RR_HIP_JOINT;
        physics::JointPtr RR_THIGH_JOINT;
        physics::JointPtr RR_CALF_JOINT;
        physics::JointPtr RR_TIP_JOINT;
        physics::JointPtr WAIST_JOINT;
        physics::JointPtr FL_HIP_JOINT;
        physics::JointPtr FL_THIGH_JOINT;
        physics::JointPtr FL_CALF_JOINT;
        physics::JointPtr FL_TIP_JOINT;
        physics::JointPtr FR_HIP_JOINT;
        physics::JointPtr FR_THIGH_JOINT;
        physics::JointPtr FR_CALF_JOINT;
        physics::JointPtr FR_TIP_JOINT;

        physics::ModelPtr model;

        //  PID
        common::PID pid_RL_HR, pid_RL_HP, pid_RL_KN;
        common::PID pid_RR_HR, pid_RR_HP, pid_RR_KN;
        common::PID pid_WAIST;
        common::PID pid_FL_HR, pid_FL_HP, pid_FL_KN;
        common::PID pid_FR_HR, pid_FR_HP, pid_FR_KN;

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

        double accAngleX = 0;
        double accAngleY = 0;
        double gyroAngleX = 0;
        double gyroAngleY = 0;
        double pitch = 0;
        double roll = 0;
        double yaw = 0;

        //setting for FT sensor
        physics::JointWrench wrench;
        ignition::math::Vector3d torque;
        ignition::math::Vector3d force;

        double RL_force_x, RL_force_y, RL_force_z;
        double RL_torque_x, RL_torque_y, RL_torque_z;
        double RR_force_x, RR_force_y, RR_force_z;
        double RR_torque_x, RR_torque_y, RR_torque_z;
        double FL_force_x, FL_force_y, FL_force_z;
        double FL_torque_x, FL_torque_y, FL_torque_z;
        double FR_force_x, FR_force_y, FR_force_z;
        double FR_torque_x, FR_torque_y, FR_torque_z;

        //setting for rqt telecommunication
        ros::NodeHandle n;

        ros::Publisher P_Times;
        ros::Publisher P_angular_velocity_x;
        ros::Publisher P_angular_velocity_y;
        ros::Publisher P_angular_velocity_z;

        ros::Publisher P_RL_force_x;
        ros::Publisher P_RL_force_y;
        ros::Publisher P_RL_force_z;
        ros::Publisher P_RR_force_x;
        ros::Publisher P_RR_force_y;
        ros::Publisher P_RR_force_z;
        ros::Publisher P_FL_force_x;
        ros::Publisher P_FL_force_y;
        ros::Publisher P_FL_force_z;
        ros::Publisher P_FR_force_x;
        ros::Publisher P_FR_force_y;
        ros::Publisher P_FR_force_z;
        ros::Publisher P_pitch;
        ros::Publisher P_roll;
        ros::Publisher P_yaw;

        std_msgs::Float64 m_Times;
        std_msgs::Float64 m_angular_velocity_x;
        std_msgs::Float64 m_angular_velocity_y;
        std_msgs::Float64 m_angular_velocity_z;
        std_msgs::Float64 m_RL_force_x;
        std_msgs::Float64 m_RL_force_y;
        std_msgs::Float64 m_RL_force_z;
        std_msgs::Float64 m_RR_force_x;
        std_msgs::Float64 m_RR_force_y;
        std_msgs::Float64 m_RR_force_z;
        std_msgs::Float64 m_FL_force_x;
        std_msgs::Float64 m_FL_force_y;
        std_msgs::Float64 m_FL_force_z;
        std_msgs::Float64 m_FR_force_x;
        std_msgs::Float64 m_FR_force_y;
        std_msgs::Float64 m_FR_force_z;
        std_msgs::Float64 m_pitch;
        std_msgs::Float64 m_roll;
        std_msgs::Float64 m_yaw;

        // Rviz variable setting
        ros::Publisher P_joint_states;
        sensor_msgs::JointState m_joint_states;
        tf::TransformBroadcaster broadcaster;
        geometry_msgs::TransformStamped odom_trans;
        math::Pose pose;

        // DH PARA
        //double tar_deg[13] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
        double tar_deg[13] = {0, -30, 60, 0, -30, 60, 0, 0, 30, -60, 0, 30, -60};
        VectorXd tau = VectorXd::Zero(13);
        VectorXd pre_Encoder = VectorXd::Zero(13); // Only 13 Joints
        VectorXd angle_err = VectorXd::Zero(13);
        VectorXd angle_vel = VectorXd::Zero(13);

        //Gravity Compensation
        VectorXd basePosOri = VectorXd::Zero(6);
        VectorXd baseVel = VectorXd::Zero(6);
        VectorXd jointAngle = VectorXd::Zero(13);
        VectorXd jointVel = VectorXd::Zero(13);

        // FT sensor Transformation
        VectorXd Encoder = VectorXd::Zero(17); // 13 Joints + 4 Tips
        VectorXd RL_Force_E = VectorXd::Zero(3);
        VectorXd RL_Force_I = VectorXd::Zero(3);
        VectorXd RL_Torque_E = VectorXd::Zero(3);
        VectorXd RL_Torque_I = VectorXd::Zero(3);
        VectorXd RR_Force_E = VectorXd::Zero(3);
        VectorXd RR_Force_I = VectorXd::Zero(3);
        VectorXd RR_Torque_E = VectorXd::Zero(3);
        VectorXd RR_Torque_I = VectorXd::Zero(3);
        VectorXd FL_Force_E = VectorXd::Zero(3);
        VectorXd FL_Force_I = VectorXd::Zero(3);
        VectorXd FL_Torque_E = VectorXd::Zero(3);
        VectorXd FL_Torque_I = VectorXd::Zero(3);
        VectorXd FR_Force_E = VectorXd::Zero(3);
        VectorXd FR_Force_I = VectorXd::Zero(3);
        VectorXd FR_Torque_E = VectorXd::Zero(3);
        VectorXd FR_Torque_I = VectorXd::Zero(3);
        MatrixXd RL_C_IE = MatrixXd::Zero(3, 3);
        MatrixXd RL_C_EI = MatrixXd::Zero(3, 3);
        MatrixXd RR_C_IE = MatrixXd::Zero(3, 3);
        MatrixXd RR_C_EI = MatrixXd::Zero(3, 3);
        MatrixXd FR_C_IE = MatrixXd::Zero(3, 3);
        MatrixXd FR_C_EI = MatrixXd::Zero(3, 3);
        MatrixXd FL_C_IE = MatrixXd::Zero(3, 3);
        MatrixXd FL_C_EI = MatrixXd::Zero(3, 3);
        MatrixXd ROT_FORWARD_TIP = MatrixXd::Zero(3, 3);
        MatrixXd ROT_REAR_TIP = MatrixXd::Zero(3, 3);

        // Adding CRobot Class by BKCho
        CRobot PongBotQ;

        //TEST
        ros::Publisher P_World_angle_y;
        std_msgs::Float64 m_World_angle_y;
        VectorXd xyz_angle = VectorXd::Zero(3);
        VectorXd xyz_quat = VectorXd::Zero(4);
        math::Quaternion World_Quaternion;
        double World_angle_y;


    public:
        //For model load
        void Load(physics::ModelPtr _model, sdf::ElementPtr /*_sdf*/);
        void UpdateAlgorithm();
    };
    GZ_REGISTER_MODEL_PLUGIN(PongBotQ_plugin);
}

void gazebo::PongBotQ_plugin::Load(physics::ModelPtr _model, sdf::ElementPtr /*_sdf*/)
{
    //***************************RBDL setting**********************************/
    // model = link + joint +sensor
    this->model = _model;

    rbdl_check_api_version(RBDL_API_VERSION);

    int version_test;
    version_test = rbdl_get_api_version();
    printf("rbdl api version = %d\n", version_test);

    Addons::URDFReadFromFile("/root/.gazebo/models/PONGBOT_Q_V2/urdf/PONGBOT_Q_V2.urdf", pongbot_q_model, true, true);
//    Addons::URDFReadFromFile("/home/hyunseok/.gazebo/models/PONGBOT_Q_V2/urdf/PONGBOT_Q_V2.urdf", pongbot_q_model, true, true);
    PongBotQ.setRobotModel(pongbot_q_model);

    //************************Link & Joint Setting*********************************//
    //LINK DEFINITION
    this->RL_HIP = this->model->GetLink("RL_HIP");
    this->RL_THIGH = this->model->GetLink("RL_THIGH");
    this->RL_CALF = this->model->GetLink("RL_CALF");
    this->RL_TIP = this->model->GetLink("RL_TIP");

    this->RR_HIP = this->model->GetLink("RR_HIP");
    this->RR_THIGH = this->model->GetLink("RR_THIGH");
    this->RR_CALF = this->model->GetLink("RR_CALF");
    this->RR_TIP = this->model->GetLink("RR_TIP");

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


    //JOINT DEFINITION    
    this->RL_HIP_JOINT = this->model->GetJoint("RL_HIP_JOINT");
    this->RL_THIGH_JOINT = this->model->GetJoint("RL_THIGH_JOINT");
    this->RL_CALF_JOINT = this->model->GetJoint("RL_CALF_JOINT");
    this->RL_TIP_JOINT = this->model->GetJoint("RL_TIP_JOINT");

    this->RR_HIP_JOINT = this->model->GetJoint("RR_HIP_JOINT");
    this->RR_THIGH_JOINT = this->model->GetJoint("RR_THIGH_JOINT");
    this->RR_CALF_JOINT = this->model->GetJoint("RR_CALF_JOINT");
    this->RR_TIP_JOINT = this->model->GetJoint("RR_TIP_JOINT");

    this->WAIST_JOINT = this->model->GetJoint("WAIST_JOINT");

    this->FL_HIP_JOINT = this->model->GetJoint("FL_HIP_JOINT");
    this->FL_THIGH_JOINT = this->model->GetJoint("FL_THIGH_JOINT");
    this->FL_CALF_JOINT = this->model->GetJoint("FL_CALF_JOINT");
    this->FL_TIP_JOINT = this->model->GetJoint("FL_TIP_JOINT");

    this->FR_HIP_JOINT = this->model->GetJoint("FR_HIP_JOINT");
    this->FR_THIGH_JOINT = this->model->GetJoint("FR_THIGH_JOINT");
    this->FR_CALF_JOINT = this->model->GetJoint("FR_CALF_JOINT");
    this->FR_TIP_JOINT = this->model->GetJoint("FR_TIP_JOINT");

    //************************Time Setting*********************************//
    //setting for getting dt
    this->last_update_time = this->model->GetWorld()->GetSimTime();
    this->update_connection = event::Events::ConnectWorldUpdateBegin(boost::bind(&PongBotQ_plugin::UpdateAlgorithm, this));

    //************************Imu Setting*********************************//
    //setting for IMU sensor
    this->Sensor = sensors::get_sensor("IMU");
    this->IMU = std::dynamic_pointer_cast<sensors::ImuSensor>(Sensor);

    //************************FT Transformation Setting*********************************//
    //Rotation Matrix
    ROT_FORWARD_TIP << cos(PI / 4), 0, sin(PI / 4), 0, 1, 0, -sin(PI / 4), 0, cos(PI / 4);
    ROT_REAR_TIP << cos(-PI / 4), 0, sin(-PI / 4), 0, 1, 0, -sin(-PI / 4), 0, cos(-PI / 4);

    //************************ROS Msg Setting*********************************//
    //setting for communication
    P_Times = n.advertise<std_msgs::Float64>("times", 1);
    P_angular_velocity_x = n.advertise<std_msgs::Float64>("angular_velocity_x", 1);
    P_angular_velocity_y = n.advertise<std_msgs::Float64>("angular_velocity_y", 1);
    P_angular_velocity_z = n.advertise<std_msgs::Float64>("angular_velocity_z", 1);
    P_RL_force_x = n.advertise<std_msgs::Float64>("RL_force_x", 1);
    P_RL_force_y = n.advertise<std_msgs::Float64>("RL_force_y", 1);
    P_RL_force_z = n.advertise<std_msgs::Float64>("RL_force_z", 1);
    P_RR_force_x = n.advertise<std_msgs::Float64>("RR_force_x", 1);
    P_RR_force_y = n.advertise<std_msgs::Float64>("RR_force_y", 1);
    P_RR_force_z = n.advertise<std_msgs::Float64>("RR_force_z", 1);
    P_FL_force_x = n.advertise<std_msgs::Float64>("FL_force_x", 1);
    P_FL_force_y = n.advertise<std_msgs::Float64>("FL_force_y", 1);
    P_FL_force_z = n.advertise<std_msgs::Float64>("FL_force_z", 1);
    P_FR_force_x = n.advertise<std_msgs::Float64>("FR_force_x", 1);
    P_FR_force_y = n.advertise<std_msgs::Float64>("FR_force_y", 1);
    P_FR_force_z = n.advertise<std_msgs::Float64>("FR_force_z", 1);
    P_pitch = n.advertise<std_msgs::Float64>("pitch", 1);
    P_roll = n.advertise<std_msgs::Float64>("roll", 1);
    P_yaw = n.advertise<std_msgs::Float64>("yaw", 1);
    P_World_angle_y = n.advertise<std_msgs::Float64>("World_angle_y", 1);


    //**************************rviz inital setting*************************************//
    P_joint_states = n.advertise<sensor_msgs::JointState>("joint_states", 1);
    m_joint_states.name.resize(13);
    m_joint_states.position.resize(13);
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "REAR_BODY";
    //ros::Rate loop_rate(1000);
}

void gazebo::PongBotQ_plugin::UpdateAlgorithm()
{ //* Writing realtime code here!!
    //********************* Base Pose for rviz *****************************//
    pose = this->model->GetWorldPose();

    //************************** Time ********************************//
    common::Time current_time = this->model->GetWorld()->GetSimTime();
    dt = current_time.Double() - this->last_update_time.Double();
    time = time + dt;

    //*********************** Imu(Complementary filter) ******************************//
    angular_velocity_x = 0.50 * angular_velocity_x + 0.50 * (this->IMU->AngularVelocity(false)[0]);
    angular_velocity_y = 0.50 * angular_velocity_y + 0.50 * (this->IMU->AngularVelocity(false)[1]);
    angular_velocity_z = 0.50 * angular_velocity_z + 0.50 * (this->IMU->AngularVelocity(false)[2]);

    //angular_velocity_x = this->IMU->AngularVelocity(false)[0];
    //angular_velocity_y = this->IMU->AngularVelocity(false)[1];
    //angular_velocity_z = this->IMU->AngularVelocity(false)[2];

    //linear_acc_x = this->IMU->LinearAcceleration(false)[0];
    //linear_acc_y = this->IMU->LinearAcceleration(false)[1];
    //linear_acc_z = this->IMU->LinearAcceleration(false)[2];

    linear_acc_x = 0.999 * linear_acc_z + 0.001 * (this->IMU->LinearAcceleration(false)[0]);
    linear_acc_y = 0.999 * linear_acc_z + 0.001 * (this->IMU->LinearAcceleration(false)[1]);
    linear_acc_z = 0.999 * linear_acc_z + 0.001 * (this->IMU->LinearAcceleration(false)[2]);

    World_Quaternion = this->REAR_BODY->GetWorldPose().rot;
    World_angle_y = World_Quaternion.GetAsEuler().y;

    //Complementary filter
    accAngleX = (atan2(linear_acc_y, sqrt(pow(linear_acc_x, 2) + pow(linear_acc_z, 2))))*180 / PI;
    accAngleY = (atan2(-linear_acc_x, sqrt(pow(linear_acc_y, 2) + pow(linear_acc_z, 2))))*180 / PI;
    //gyroAngleX = (gyroAngleX + angular_velocity_x*dt)*180/PI;
    ///gyroAngleY = (gyroAngleY + angular_velocity_y*dt)*180/PI;
    gyroAngleX = gyroAngleX + angular_velocity_x * dt;
    gyroAngleY = gyroAngleY + angular_velocity_y * dt;

    //yaw = yaw + angular_velocity_z*dt;
    yaw = angular_velocity_z*dt;
    roll = 0.9 * gyroAngleX * 180 / PI + 0.1 * accAngleX;
    pitch = 0.9 * gyroAngleY * 180 / PI + 0.1 * accAngleY;


    //************************** Encoder ********************************//
    Encoder[0] = this->RL_HIP_JOINT->GetAngle(0).Radian();
    Encoder[1] = this->RL_THIGH_JOINT->GetAngle(0).Radian();
    Encoder[2] = this->RL_CALF_JOINT->GetAngle(0).Radian();

    Encoder[3] = this->RR_HIP_JOINT->GetAngle(0).Radian();
    Encoder[4] = this->RR_THIGH_JOINT->GetAngle(0).Radian();
    Encoder[5] = this->RR_CALF_JOINT->GetAngle(0).Radian();

    Encoder[6] = this->WAIST_JOINT->GetAngle(0).Radian();

    Encoder[7] = this->FL_HIP_JOINT->GetAngle(0).Radian();
    Encoder[8] = this->FL_THIGH_JOINT->GetAngle(0).Radian();
    Encoder[9] = this->FL_CALF_JOINT->GetAngle(0).Radian();

    Encoder[10] = this->FR_HIP_JOINT->GetAngle(0).Radian();
    Encoder[11] = this->FR_THIGH_JOINT->GetAngle(0).Radian();
    Encoder[12] = this->FR_CALF_JOINT->GetAngle(0).Radian();

    Encoder[13] = -PI / 4; //RL_TIP
    Encoder[14] = -PI / 4; //RR_TIP
    Encoder[15] = PI / 4; //FL_TIP
    Encoder[16] = PI / 4; //FR_TIP

    //* calculating errors
    // Angle & Angular velocity of leg
    for (int i = 0; i < 13; i++) {
        angle_vel[i] = (Encoder[i] - pre_Encoder[i]) / dt;
        pre_Encoder[i] = Encoder[i];
        angle_err[i] = (tar_deg[i] * D2R) - Encoder[i];
    }


    //***************************Set Torque********************************//

    //xyz_angle << 0, 0, PI / 2;
    //xyz_quat = Quaternion::fromXYZAngles(xyz_angle);
    //basePosOri << 0, 0, 0, xyz_quat[0], xyz_quat[1], xyz_quat[2];

    basePosOri << pose.pos.x, pose.pos.y, pose.pos.z, pose.rot.x, pose.pos.y, pose.pos.z;
    baseVel << 0, 0, 0, 0, 0, 0;
    jointAngle << Encoder[0], Encoder[1], Encoder[2], Encoder[3], Encoder[4], Encoder[5], Encoder[6], Encoder[7], Encoder[8], Encoder[9], Encoder[10], Encoder[11], Encoder[12];
    jointVel << angle_vel[0], angle_vel[1], angle_vel[2], angle_vel[3], angle_vel[4], angle_vel[5], angle_vel[6], angle_vel[7], angle_vel[8], angle_vel[9], angle_vel[10], angle_vel[11], angle_vel[12];

    PongBotQ.getRobotState(basePosOri, baseVel, jointAngle, jointVel);
    PongBotQ.ComputeTorqueControl();

    //* Control law of PD Control + Gravity Compensation
    tau[0] = 10 * (angle_err[0]) + 2 * (0 - angle_vel[0]) + PongBotQ.joint[0].torque; //RL_HIP
    tau[1] = 20 * (angle_err[1]) + 5 * (0 - angle_vel[1]) + PongBotQ.joint[1].torque; //RL_THIGH
    tau[2] = 20 * (angle_err[2]) + 5 * (0 - angle_vel[2]) + PongBotQ.joint[2].torque; //RL_CALF

    tau[3] = 10 * (angle_err[3]) + 2 * (0 - angle_vel[3]) + PongBotQ.joint[3].torque; //RR_HIP
    tau[4] = 20 * (angle_err[4]) + 5 * (0 - angle_vel[4]) + PongBotQ.joint[4].torque; //RR_THIGH
    tau[5] = 20 * (angle_err[5]) + 5 * (0 - angle_vel[5]) + PongBotQ.joint[5].torque; //RR_CALF

    tau[6] = 10 * (angle_err[6]) + 2 * (0 - angle_vel[6]) + PongBotQ.joint[6].torque; //WAIST

    tau[7] = 10 * (angle_err[7]) + 2 * (0 - angle_vel[7]) + PongBotQ.joint[7].torque; //FL_THIP
    tau[8] = 20 * (angle_err[8]) + 5 * (0 - angle_vel[8]) + PongBotQ.joint[8].torque; //FL_THIGH
    tau[9] = 20 * (angle_err[9]) + 5 * (0 - angle_vel[9]) + PongBotQ.joint[9].torque; //FL_CALF

    tau[10] = 10 * (angle_err[10]) + 2 * (0 - angle_vel[10]) + PongBotQ.joint[10].torque; //FR_HIP
    tau[11] = 20 * (angle_err[11]) + 5 * (0 - angle_vel[11]) + PongBotQ.joint[11].torque; //FR_THIGH
    tau[12] = 20 * (angle_err[12]) + 5 * (0 - angle_vel[12]) + PongBotQ.joint[12].torque; //FR_CALF

    //* Control law of PD Control
    //        tau[0] = 200 * (angle_err[0]) + 10 * (0 - angle_vel[0]); //RL_HIP
    //        tau[1] = 400 * (angle_err[1]) + 10 * (0 - angle_vel[1]); //RL_THIGH
    //        tau[2] = 400 * (angle_err[2]) + 10 * (0 - angle_vel[2]); //RL_CALF
    //        tau[3] = 200 * (angle_err[3]) + 10 * (0 - angle_vel[3]); //RR_HIP
    //        tau[4] = 400 * (angle_err[4]) + 10 * (0 - angle_vel[4]); //RR_THIGH
    //        tau[5] = 400 * (angle_err[5]) + 10 * (0 - angle_vel[5]); //RR_CALF
    //    
    //        tau[6] = 200 * (angle_err[6]) + 10 * (0 - angle_vel[6]); //WAIST
    //    
    //        tau[7] = 200 * (angle_err[7]) + 10 * (0 - angle_vel[7]); //FL_HIP
    //        tau[8] = 400 * (angle_err[8]) + 10 * (0 - angle_vel[8]); //FL_THIGH
    //        tau[9] = 400 * (angle_err[9]) + 10 * (0 - angle_vel[9]); //FL_CALF
    //        tau[10] = 200 * (angle_err[10]) + 10 * (0 - angle_vel[10]); //FR_HIP
    //        tau[11] = 400 * (angle_err[11]) + 10 * (0 - angle_vel[11]); //FR_THIGH
    //        tau[12] = 400 * (angle_err[12]) + 10 * (0 - angle_vel[12]); //FR_CALF

    //* Applying torques
    this->RL_HIP_JOINT->SetForce(1, tau[0]);
    this->RL_THIGH_JOINT->SetForce(1, tau[1]);
    this->RL_CALF_JOINT->SetForce(1, tau[2]);

    this->RR_HIP_JOINT->SetForce(1, tau[3]);
    this->RR_THIGH_JOINT->SetForce(1, tau[4]);
    this->RR_CALF_JOINT->SetForce(1, tau[5]);

    this->WAIST_JOINT->SetForce(1, tau[6]);

    this->FL_HIP_JOINT->SetForce(1, tau[7]);
    this->FL_THIGH_JOINT->SetForce(1, tau[8]);
    this->FL_CALF_JOINT->SetForce(1, tau[9]);

    this->FR_HIP_JOINT->SetForce(1, tau[10]);
    this->FR_THIGH_JOINT->SetForce(1, tau[11]);
    this->FR_CALF_JOINT->SetForce(1, tau[12]);

    //***************************FT sensor********************************//
    PongBotQ.FTsensorTransformation();

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
    RL_C_EI = ROT_REAR_TIP.transpose() * PongBotQ.RL.T_matrix;
    RL_C_IE = RL_C_EI.transpose();
    RL_Force_I = RL_C_IE*RL_Force_E;
    RL_Torque_I = RL_C_IE*RL_Torque_E;

    std::cout << "MATRIX" << PongBotQ.RL.T_matrix << std::endl;

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
    RR_C_EI = ROT_REAR_TIP.transpose() * PongBotQ.RR.T_matrix;
    RR_C_IE = RR_C_EI.transpose();
    RR_Force_I = RR_C_IE*RR_Force_E;
    RR_Torque_I = RR_C_IE*RR_Torque_E;

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
    FL_C_EI = ROT_FORWARD_TIP.transpose() * PongBotQ.FL.T_matrix;
    FL_C_IE = FL_C_EI.transpose();
    FL_Force_I = FL_C_IE*FL_Force_E;
    FL_Torque_I = FL_C_IE*FL_Torque_E;

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
    FR_C_EI = ROT_FORWARD_TIP.transpose() * PongBotQ.FR.T_matrix;
    FR_C_IE = FR_C_EI.transpose();
    FR_Force_I = FR_C_IE*FR_Force_E;
    FR_Torque_I = FR_C_IE*FR_Torque_E;

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

    //***************************RQT DATA********************************//
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
    m_pitch.data = pitch;
    m_roll.data = roll;
    m_yaw.data = yaw;
    m_World_angle_y.data = World_angle_y;

    //********************Rviz data**************************//
    m_joint_states.header.stamp = ros::Time::now();
    m_joint_states.name[0] = "RL_HIP_JOINT";
    m_joint_states.name[1] = "RL_THIGH_JOINT";
    m_joint_states.name[2] = "RL_CALF_JOINT";
    m_joint_states.name[3] = "RR_HIP_JOINT";
    m_joint_states.name[4] = "RR_THIGH_JOINT";
    m_joint_states.name[5] = "RR_CALF_JOINT";
    m_joint_states.name[6] = "WAIST_JOINT";
    m_joint_states.name[7] = "FL_HIP_JOINT";
    m_joint_states.name[8] = "FL_THIGH_JOINT";
    m_joint_states.name[9] = "FL_CALF_JOINT";
    m_joint_states.name[10] = "FR_HIP_JOINT";
    m_joint_states.name[11] = "FR_THIGH_JOINT";
    m_joint_states.name[12] = "FR_CALF_JOINT";

    m_joint_states.position[0] = Encoder[0]; //RR_HIP
    m_joint_states.position[1] = Encoder[1]; //RR_THIGH
    m_joint_states.position[2] = Encoder[2]; //RR_CALF
    m_joint_states.position[3] = Encoder[3]; //RL_HIP
    m_joint_states.position[4] = Encoder[4]; //RL_THIGH
    m_joint_states.position[5] = Encoder[5]; //RL_CALF
    m_joint_states.position[6] = Encoder[6]; //WAIST
    m_joint_states.position[7] = Encoder[7]; //FL_HIP
    m_joint_states.position[8] = Encoder[8]; //FL_THIGH 
    m_joint_states.position[9] = Encoder[9]; //FL_CALF
    m_joint_states.position[10] = Encoder[10]; //FR_HIP
    m_joint_states.position[11] = Encoder[11]; //FR_THIGH 
    m_joint_states.position[12] = Encoder[12]; //FR_CALF

    odom_trans.header.stamp = ros::Time::now();
    odom_trans.transform.translation.x = pose.pos.x;
    odom_trans.transform.translation.y = pose.pos.y;
    odom_trans.transform.translation.z = pose.pos.z;
    odom_trans.transform.rotation.x = pose.rot.x;
    odom_trans.transform.rotation.y = pose.rot.y;
    odom_trans.transform.rotation.z = pose.rot.z;
    odom_trans.transform.rotation.w = pose.rot.w;
    //odom_trans.transform.translation.x = 0;
    //odom_trans.transform.translation.y = 0;
    //odom_trans.transform.translation.z = 0.6;
    //odom_trans.transform.rotation = tf::createQuaternionMsgFromYaw(0);

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
    P_pitch.publish(m_pitch);
    P_roll.publish(m_roll);
    P_yaw.publish(m_yaw);
    broadcaster.sendTransform(odom_trans);

    P_World_angle_y.publish(m_World_angle_y);
    P_joint_states.publish(m_joint_states);
}
