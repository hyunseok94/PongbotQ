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
//#include <Eigen/Dense>
#include <rbdl/rbdl.h>
#include <rbdl/addons/urdfreader/urdfreader.h>
#include <sensor_msgs/JointState.h>             //for rviz
#include <tf/transform_broadcaster.h>           //for rviz
#include <geometry_msgs/WrenchStamped.h>        //for rviz
#include "CRobot.h" // by BKCho

//#define PI      3.141592
//#define D2R     PI/180
//#define R2D     180/PI
//#define gravity       9.81;

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using Eigen::Vector3d;

Model* pongbot_q_model = new Model();

//using namespace RigidBodyDynamics::Math;

MatrixXd g_H(4, 4), g_Q(4, 4), g_R(4, 4);
VectorXd g_x(4);
MatrixXd g_P(4, 4);
int g_firstRun;

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
        double angular_vel_x;
        double angular_vel_y;
        double angular_vel_z;
        double linear_acc_x;
        double linear_acc_y;
        double linear_acc_z;

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
        ros::Publisher P_angular_vel_x;
        ros::Publisher P_angular_vel_y;
        ros::Publisher P_angular_vel_z;
        ros::Publisher P_linear_acc_x;
        ros::Publisher P_linear_acc_y;
        ros::Publisher P_linear_acc_z;

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
        ros::Publisher P_kalman_pitch;
        ros::Publisher P_kalman_roll;
        ros::Publisher P_pitch_err;
        ros::Publisher P_roll_err;
        ros::Publisher P_roll_gyro;
        ros::Publisher P_pitch_gyro;
        ros::Publisher P_yaw_gyro;
        ros::Publisher P_roll_acc;
        ros::Publisher P_pitch_acc;
        ros::Publisher P_yaw_acc;
        ros::Publisher P_roll_comp;
        ros::Publisher P_pitch_comp;

        std_msgs::Float64 m_Times;
        std_msgs::Float64 m_angular_vel_x;
        std_msgs::Float64 m_angular_vel_y;
        std_msgs::Float64 m_angular_vel_z;
        std_msgs::Float64 m_linear_acc_x;
        std_msgs::Float64 m_linear_acc_y;
        std_msgs::Float64 m_linear_acc_z;

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
        std_msgs::Float64 m_kalman_pitch;
        std_msgs::Float64 m_kalman_roll;
        std_msgs::Float64 m_pitch_err;
        std_msgs::Float64 m_roll_err;
        std_msgs::Float64 m_roll_gyro;
        std_msgs::Float64 m_pitch_gyro;
        std_msgs::Float64 m_yaw_gyro;
        std_msgs::Float64 m_roll_acc;
        std_msgs::Float64 m_pitch_acc;
        std_msgs::Float64 m_yaw_acc;
        std_msgs::Float64 m_roll_comp;
        std_msgs::Float64 m_pitch_comp;

        // Rviz variable setting
        ros::Publisher P_joint_states;
        sensor_msgs::JointState m_joint_states;
        tf::TransformBroadcaster broadcaster;
        geometry_msgs::TransformStamped odom_trans;
        math::Pose pose;

        ros::Publisher P_RL_force;
        ros::Publisher P_RR_force;
        ros::Publisher P_FL_force;
        ros::Publisher P_FR_force;
        geometry_msgs::WrenchStamped m_RL_force;
        geometry_msgs::WrenchStamped m_RR_force;
        geometry_msgs::WrenchStamped m_FL_force;
        geometry_msgs::WrenchStamped m_FR_force;

        //ros::Publisher P_joint_force;
        // DH PARA
        //double tar_deg[13] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
        //double tar_deg[13] = {0, -30, 60, 0, -30, 60, 0, 0, 30, -60, 0, 30, -60};
        double tar_deg[13] = {0, -45, 90, 0, -45, 90, 0, 0, 45, -90, 0, 45, -90};
        VectorXd tar_torque = VectorXd::Zero(13);
        VectorXd pre_actual_q = VectorXd::Zero(13); // Only 13 Joints
        VectorXd q_err = VectorXd::Zero(13);
        VectorXd actual_q_dot = VectorXd::Zero(13);

        //Gravity Compensation
        VectorXd basePosOri = VectorXd::Zero(6);
        VectorXd baseVel = VectorXd::Zero(6);
        VectorXd jointAngle = VectorXd::Zero(13);
        VectorXd jointVel = VectorXd::Zero(13);

        // FT sensor Transformation
        VectorXd actual_q = VectorXd::Zero(17); // 13 Joints + 4 Tips
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

        //TEST_kalman
        VectorXd EulerGyro = VectorXd::Zero(3);
        VectorXd EulerAccel = VectorXd::Zero(3);
        VectorXd z = VectorXd::Zero(4);
        MatrixXd A = MatrixXd::Zero(4, 4);
        VectorXd EulerKalman = VectorXd::Zero(3);
        VectorXd Euler_err = VectorXd::Zero(3);


        //TEST Complementary Filter
        double pitch_gyro = 0;
        double roll_gyro = 0;
        double yaw_gyro = 0;
        double pitch_acc = 0;
        double roll_acc = 0;
        double yaw_acc = 0;
        double roll_comp = 0;
        double pitch_comp = 0;

        //double alpha_roll=0.9996;
        //double alpha_roll=0.996;
        //double alpha_roll=0.997;
        double alpha_roll = 0.998;
        double alpha_pitch = 0.999;

        // DH Parameters
        VectorXd xyz_angle = VectorXd::Zero(3);
        VectorXd xyz_quat = VectorXd::Zero(4);
        double init_pos_time = 1;
        double home_pos_time = 1;
        double trot_time = 1;

        enum ControlMode
        {
            IDLE = 0,
            INIT_POS,
            ENC_ZERO,
            HOME_POS,
            POS_INIT,
            TROT
        };

        enum Phase
        {
            INIT_Fc = 0,
            STOP,
            STANCE_RLFR,
            STANCE_RRFL,
            STANCE_FOUR_LEGS_AFTER_RLFR,
            STANCE_FOUR_LEGS_AFTER_RRFL
            //    REAR_L
        };

        enum Foot_Phase
        {
            FOUR = 0,
            ONE_RL,
            ONE_RR,
            ONE_FL,
            ONE_FR,
            TWO_RL_RR,
            TWO_RL_FL,
            TWO_RL_FR,
            TWO_RR_FL,
            TWO_RR_FR,
            TWO_FL_FR,
            THREE_RL_RR_FL,
            THREE_RL_RR_FR,
            THREE_RL_FL_FR,
            THREE_RR_FL_FR
        };

        enum ControlMode CONTROL_MODE;
        enum Phase TROT_PHASE;
        enum Foot_Phase FOOT_PHASE;

        double ctc_cnt = 0, ctc_cnt2 = 0;
        bool trot_init_flag = false;

    public:
        //For model load
        void Load(physics::ModelPtr _model, sdf::ElementPtr /*_sdf*/);
        void UpdateAlgorithm();

        void Init_Pos_Traj(void);
        void Home_Pos_Traj(void);
        void Pos_Init_Traj(void);
        void TROT_Traj(void);
        MatrixXd SystemMatrix(double wx, double wy, double wz, double dt);
        VectorXd GetEulerAccel(double ax, double ay, double az);
        VectorXd EulerToQuaternion(double roll, double pitch, double yaw);
        VectorXd GetEulerKalman(MatrixXd A, VectorXd z);
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

    //Addons::URDFReadFromFile("/root/.gazebo/models/PONGBOT_Q_V2/urdf/PONGBOT_Q_V2.urdf", pongbot_q_model, true, true);
    Addons::URDFReadFromFile("/home/hyunseok/.gazebo/models/PONGBOT_Q_V2/urdf/PONGBOT_Q_V2.urdf", pongbot_q_model, true, true);
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
    P_angular_vel_x = n.advertise<std_msgs::Float64>("angular_vel_x", 1);
    P_angular_vel_y = n.advertise<std_msgs::Float64>("angular_vel_y", 1);
    P_angular_vel_z = n.advertise<std_msgs::Float64>("angular_vel_z", 1);
    P_linear_acc_x = n.advertise<std_msgs::Float64>("linear_acc_x", 1);
    P_linear_acc_y = n.advertise<std_msgs::Float64>("linear_acc_y", 1);
    P_linear_acc_z = n.advertise<std_msgs::Float64>("linear_acc_z", 1);

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
    P_kalman_pitch = n.advertise<std_msgs::Float64>("kalman_pitch", 1);
    P_kalman_roll = n.advertise<std_msgs::Float64>("kalman_roll", 1);
    P_pitch_err = n.advertise<std_msgs::Float64>("pitch_err", 1);
    P_roll_err = n.advertise<std_msgs::Float64>("roll_err", 1);

    P_roll_gyro = n.advertise<std_msgs::Float64>("roll_gyro", 1);
    P_pitch_gyro = n.advertise<std_msgs::Float64>("pitch_gyro", 1);
    P_yaw_gyro = n.advertise<std_msgs::Float64>("yaw_gyro", 1);
    P_roll_acc = n.advertise<std_msgs::Float64>("roll_acc", 1);
    P_pitch_acc = n.advertise<std_msgs::Float64>("pitch_acc", 1);
    P_yaw_acc = n.advertise<std_msgs::Float64>("yaw_acc", 1);
    P_roll_comp = n.advertise<std_msgs::Float64>("roll_comp", 1);
    P_pitch_comp = n.advertise<std_msgs::Float64>("pitch_comp", 1);

    //**************************rviz inital setting*************************************//
    P_joint_states = n.advertise<sensor_msgs::JointState>("joint_states", 1);
    m_joint_states.name.resize(13);
    m_joint_states.position.resize(13);
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "REAR_BODY";

    P_RL_force = n.advertise<geometry_msgs::WrenchStamped>("RL_force", 1000);
    P_RR_force = n.advertise<geometry_msgs::WrenchStamped>("RR_force", 1000);
    P_FL_force = n.advertise<geometry_msgs::WrenchStamped>("FL_force", 1000);
    P_FR_force = n.advertise<geometry_msgs::WrenchStamped>("FR_force", 1000);
    m_RL_force.header.frame_id = "RL_TIP";
    m_RR_force.header.frame_id = "RR_TIP";
    m_FL_force.header.frame_id = "FL_TIP";
    m_FR_force.header.frame_id = "FR_TIP";
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

    //*********************** Imu(Kalman filter) ******************************//
    angular_vel_x = this->IMU->AngularVelocity(false)[0];
    angular_vel_y = this->IMU->AngularVelocity(false)[1];
    angular_vel_z = this->IMU->AngularVelocity(false)[2];

    linear_acc_x = this->IMU->LinearAcceleration(false)[0];
    linear_acc_y = this->IMU->LinearAcceleration(false)[1];
    linear_acc_z = this->IMU->LinearAcceleration(false)[2];

    pitch = pose.rot.GetPitch();
    roll = pose.rot.GetRoll();
    yaw = pose.rot.GetYaw();

    roll_gyro = roll_gyro + angular_vel_x*dt;
    pitch_gyro = pitch_gyro + angular_vel_y*dt;
    yaw_gyro = yaw_gyro + angular_vel_z*dt;

    roll_acc = atan(linear_acc_y / linear_acc_z);
    pitch_acc = atan(linear_acc_x / sqrt(linear_acc_y * linear_acc_y + linear_acc_z * linear_acc_z));
    yaw_acc = 0;

    if (isnan(roll_acc) || isnan(pitch_acc)) {
        roll_acc = 0;
        pitch_acc = 0;
    }

    //    roll_comp = alpha_roll * (angular_vel_x * dt + roll_comp)+(1 - alpha_roll) * roll_acc;
    //    pitch_comp = alpha_pitch * (angular_vel_y * dt + pitch_comp)+(1 - alpha_pitch) * pitch_acc;
    //
    //
    //    //TEST1(Kalman filter)
    //    A = SystemMatrix(angular_vel_x, angular_vel_y, angular_vel_z, dt);
    //    EulerAccel = GetEulerAccel(linear_acc_x, linear_acc_y, linear_acc_z);
    //    z = EulerToQuaternion(EulerAccel[0], EulerAccel[1], EulerAccel[2]); //z=(w,x,y,z)
    //    EulerKalman = GetEulerKalman(A, z);

    //************************** Encoder ********************************//
    actual_q[0] = this->RL_HIP_JOINT->GetAngle(0).Radian();
    actual_q[1] = this->RL_THIGH_JOINT->GetAngle(0).Radian();
    actual_q[2] = this->RL_CALF_JOINT->GetAngle(0).Radian();

    actual_q[3] = this->RR_HIP_JOINT->GetAngle(0).Radian();
    actual_q[4] = this->RR_THIGH_JOINT->GetAngle(0).Radian();
    actual_q[5] = this->RR_CALF_JOINT->GetAngle(0).Radian();

    actual_q[6] = this->WAIST_JOINT->GetAngle(0).Radian();

    actual_q[7] = this->FL_HIP_JOINT->GetAngle(0).Radian();
    actual_q[8] = this->FL_THIGH_JOINT->GetAngle(0).Radian();
    actual_q[9] = this->FL_CALF_JOINT->GetAngle(0).Radian();

    actual_q[10] = this->FR_HIP_JOINT->GetAngle(0).Radian();
    actual_q[11] = this->FR_THIGH_JOINT->GetAngle(0).Radian();
    actual_q[12] = this->FR_CALF_JOINT->GetAngle(0).Radian();

    actual_q[13] = -PI / 4; //RL_TIP
    actual_q[14] = -PI / 4; //RR_TIP
    actual_q[15] = PI / 4; //FL_TIP
    actual_q[16] = PI / 4; //FR_TIP

    //* calculating errors
    // Angle & Angular velocity of leg
    for (int i = 0; i < 13; i++) {
        actual_q_dot[i] = (actual_q[i] - pre_actual_q[i]) / dt;
        pre_actual_q[i] = actual_q[i];
        q_err[i] = (tar_deg[i] * D2R) - actual_q[i];
    }

    // ******************* DH : Trajectory Generation ****************** //

    xyz_angle << 0, 0, 0; //PI / 2;
    xyz_quat = Quaternion::fromXYZAngles(xyz_angle);
    PongBotQ.QQ << xyz_quat[0], xyz_quat[1], xyz_quat[2], xyz_quat[3];

    basePosOri << 0, 0, 0, xyz_quat[0], xyz_quat[1], xyz_quat[2]; //pose.rot.z;
    baseVel << 0, 0, 0, 0, 0, 0;
    jointAngle << actual_q[0], actual_q[1], actual_q[2], actual_q[3], actual_q[4], actual_q[5], actual_q[6], actual_q[7], actual_q[8], actual_q[9], actual_q[10], actual_q[11], actual_q[12];
    jointVel << actual_q_dot[0], actual_q_dot[1], actual_q_dot[2], actual_q_dot[3], actual_q_dot[4], actual_q_dot[5], actual_q_dot[6], actual_q_dot[7], actual_q_dot[8], actual_q_dot[9], actual_q_dot[10], actual_q_dot[11], actual_q_dot[12];

    //    cout << "jointAngle =" << jointAngle.transpose()*R2D << endl;

    pongbot_q_model->SetQuaternion(PongBotQ.base.ID, PongBotQ.QQ, PongBotQ.RobotState);
    PongBotQ.getRobotState(basePosOri, baseVel, jointAngle, jointVel);


    static unsigned int ctr_cnt = 0;

    // next time, I will get this value(CONTROL_MODE) from rqt.
    if (ctr_cnt < init_pos_time / dt) {
        //        cout << "ctr_cnt = " << ctr_cnt << endl;
        //        printf(".");
        CONTROL_MODE = INIT_POS;
    }
    else if (ctr_cnt < (init_pos_time + home_pos_time) / dt) {
        CONTROL_MODE = HOME_POS;
    }
    else {// if (ctr_cnt < (init_pos_time + home_pos_time + trot_time) / dt) {
        CONTROL_MODE = TROT;
    }

    ctr_cnt++;

    if (CONTROL_MODE == INIT_POS) {
        Init_Pos_Traj();
    }
    else if (CONTROL_MODE == HOME_POS) {
        Home_Pos_Traj();
    }
    else if (CONTROL_MODE == POS_INIT) {
        Pos_Init_Traj();
    }
    else if (CONTROL_MODE == TROT) {
        TROT_Traj();
    }

    static double tmp_Fc1 = 80;
    static double tmp_Fc2 = tmp_Fc1 * 2;


    if (TROT_PHASE == STOP) {
        PongBotQ.Fc_RL = -tmp_Fc1;
        PongBotQ.Fc_RR = -tmp_Fc1;
        PongBotQ.Fc_FL = -tmp_Fc1;
        PongBotQ.Fc_FR = -tmp_Fc1;

    }
    else if (TROT_PHASE == STANCE_RLFR) {
        PongBotQ.Fc_RL = -tmp_Fc2;
        PongBotQ.Fc_RR = 0;
        PongBotQ.Fc_FL = 0;
        PongBotQ.Fc_FR = -tmp_Fc2;

    }
    else if (TROT_PHASE == STANCE_RRFL) {
        PongBotQ.Fc_RL = 0;
        PongBotQ.Fc_RR = -tmp_Fc2;
        PongBotQ.Fc_FL = -tmp_Fc2;
        PongBotQ.Fc_FR = 0;
    }

    if (CONTROL_MODE != INIT_POS) {
        //        xyz_angle << 0, 0, 0; //PI / 2;
        //        xyz_quat = Quaternion::fromXYZAngles(xyz_angle);
        //        PongBotQ.QQ << xyz_quat[0], xyz_quat[1], xyz_quat[2], xyz_quat[3];
        //        //    cout<< "quat al = " << xyz_quat << endl;
        //        //basePosOri << 0, 0, 0, xyz_quat[0], xyz_quat[1], xyz_quat[2];
        //        //basePosOri << pose.pos.x, pose.pos.y, pose.pos.z, pose.rot.x, pose.rot.y, 45*PI/180;//pose.rot.z;
        //        basePosOri << 0, 0, 0, xyz_quat[0], xyz_quat[1], xyz_quat[2]; //pose.rot.z;
        //        baseVel << 0, 0, 0, 0, 0, 0;
        //        jointAngle << actual_q[0], actual_q[1], actual_q[2], actual_q[3], actual_q[4], actual_q[5], actual_q[6], actual_q[7], actual_q[8], actual_q[9], actual_q[10], actual_q[11], actual_q[12];
        //        jointVel << actual_q_dot[0], actual_q_dot[1], actual_q_dot[2], actual_q_dot[3], actual_q_dot[4], actual_q_dot[5], actual_q_dot[6], actual_q_dot[7], actual_q_dot[8], actual_q_dot[9], actual_q_dot[10], actual_q_dot[11], actual_q_dot[12];
        //
        //        //    cout << "jointAngle =" << jointAngle.transpose()*R2D << endl;
        //
        //        pongbot_q_model->SetQuaternion(PongBotQ.base.ID, PongBotQ.QQ, PongBotQ.RobotState);
        //        PongBotQ.getRobotState(basePosOri, baseVel, jointAngle, jointVel);
        PongBotQ.ComputeTorqueControl();

        //* Control law of PD Control + Gravity Compensation
        tar_torque[0] = PongBotQ.joint[0].torque; //10 * (q_err[0]) + 2 * (0 - actual_q_dot[0]) + PongBotQ.joint[0].torque; //RL_HIP
        tar_torque[1] = PongBotQ.joint[1].torque; //20 * (q_err[1]) + 5 * (0 - actual_q_dot[1]) + PongBotQ.joint[1].torque; //RL_THIGH
        tar_torque[2] = PongBotQ.joint[2].torque; //20 * (q_err[2]) + 5 * (0 - actual_q_dot[2]) + PongBotQ.joint[2].torque; //RL_CALF

        tar_torque[3] = PongBotQ.joint[3].torque; //10 * (q_err[3]) + 2 * (0 - actual_q_dot[3]) + PongBotQ.joint[3].torque; //RR_HIP
        tar_torque[4] = PongBotQ.joint[4].torque; //20 * (q_err[4]) + 5 * (0 - actual_q_dot[4]) + PongBotQ.joint[4].torque; //RR_THIGH
        tar_torque[5] = PongBotQ.joint[5].torque; //20 * (q_err[5]) + 5 * (0 - actual_q_dot[5]) + PongBotQ.joint[5].torque; //RR_CALF

        tar_torque[6] = PongBotQ.joint[6].torque; //10 * (q_err[6]) + 2 * (0 - actual_q_dot[6]) + PongBotQ.joint[6].torque; //WAIST

        tar_torque[7] = PongBotQ.joint[7].torque; //10 * (q_err[7]) + 2 * (0 - actual_q_dot[7]) + PongBotQ.joint[7].torque; //FL_THIP
        tar_torque[8] = PongBotQ.joint[8].torque; //20 * (q_err[8]) + 5 * (0 - actual_q_dot[8]) + PongBotQ.joint[8].torque; //FL_THIGH
        tar_torque[9] = PongBotQ.joint[9].torque; //20 * (q_err[9]) + 5 * (0 - actual_q_dot[9]) + PongBotQ.joint[9].torque; //FL_CALF

        tar_torque[10] = PongBotQ.joint[10].torque; //10 * (q_err[10]) + 2 * (0 - actual_q_dot[10]) + PongBotQ.joint[10].torque; //FR_HIP
        tar_torque[11] = PongBotQ.joint[11].torque; //20 * (q_err[11]) + 5 * (0 - actual_q_dot[11]) + PongBotQ.joint[11].torque; //FR_THIGH
        tar_torque[12] = PongBotQ.joint[12].torque; //20 * (q_err[12]) + 5 * (0 - actual_q_dot[12]) + PongBotQ.joint[12].torque; //FR_CALF

    }

    //***************************Set Torque********************************//

    //            PongBotQ.QQ << pose.rot.x, pose.rot.y, pose.rot.z, pose.rot.w;
    //            basePosOri << pose.pos.x, pose.pos.y, pose.pos.z, pose.rot.x, pose.rot.y, pose.rot.z;
    //            baseVel << 0, 0, 0, 0, 0, 0;
    //            jointAngle << actual_q[0], actual_q[1], actual_q[2], actual_q[3], actual_q[4], actual_q[5], actual_q[6], actual_q[7], actual_q[8], actual_q[9], actual_q[10], actual_q[11], actual_q[12];
    //            jointVel << actual_q_dot[0], actual_q_dot[1], actual_q_dot[2], actual_q_dot[3], actual_q_dot[4], actual_q_dot[5], actual_q_dot[6], actual_q_dot[7], actual_q_dot[8], actual_q_dot[9], actual_q_dot[10], actual_q_dot[11], actual_q_dot[12];
    //        
    //            pongbot_q_model->SetQuaternion(PongBotQ.base.ID, PongBotQ.QQ, PongBotQ.RobotState);
    //            PongBotQ.getRobotState(basePosOri, baseVel, jointAngle, jointVel);
    //            PongBotQ.ComputeTorqueControl();
    //        
    //            //* Control law of PD Control + Gravity Compensation
    //            //    tar_torque[0] = 10 * (q_err[0]) + 2 * (0 - actual_q_dot[0]) + PongBotQ.joint[0].torque; //RL_HIP
    //            //    tar_torque[1] = 20 * (q_err[1]) + 5 * (0 - actual_q_dot[1]) + PongBotQ.joint[1].torque; //RL_THIGH
    //            //    tar_torque[2] = 20 * (q_err[2]) + 5 * (0 - actual_q_dot[2]) + PongBotQ.joint[2].torque; //RL_CALF
    //            //
    //            //    tar_torque[3] = 10 * (q_err[3]) + 2 * (0 - actual_q_dot[3]) + PongBotQ.joint[3].torque; //RR_HIP
    //            //    tar_torque[4] = 20 * (q_err[4]) + 5 * (0 - actual_q_dot[4]) + PongBotQ.joint[4].torque; //RR_THIGH
    //            //    tar_torque[5] = 20 * (q_err[5]) + 5 * (0 - actual_q_dot[5]) + PongBotQ.joint[5].torque; //RR_CALF
    //            //
    //            //    tar_torque[6] = 10 * (q_err[6]) + 2 * (0 - actual_q_dot[6]) + PongBotQ.joint[6].torque; //WAIST
    //            //
    //            //    tar_torque[7] = 10 * (q_err[7]) + 2 * (0 - actual_q_dot[7]) + PongBotQ.joint[7].torque; //FL_THIP
    //            //    tar_torque[8] = 20 * (q_err[8]) + 5 * (0 - actual_q_dot[8]) + PongBotQ.joint[8].torque; //FL_THIGH
    //            //    tar_torque[9] = 20 * (q_err[9]) + 5 * (0 - actual_q_dot[9]) + PongBotQ.joint[9].torque; //FL_CALF
    //            //
    //            //    tar_torque[10] = 10 * (q_err[10]) + 2 * (0 - actual_q_dot[10]) + PongBotQ.joint[10].torque; //FR_HIP
    //            //    tar_torque[11] = 20 * (q_err[11]) + 5 * (0 - actual_q_dot[11]) + PongBotQ.joint[11].torque; //FR_THIGH
    //            //    tar_torque[12] = 20 * (q_err[12]) + 5 * (0 - actual_q_dot[12]) + PongBotQ.joint[12].torque; //FR_CALF
    //        
    //            //* Control law of PD Control
    //            tar_torque[0] = 200 * (q_err[0]) + 10 * (0 - actual_q_dot[0]); //RL_HIP
    //            tar_torque[1] = 400 * (q_err[1]) + 10 * (0 - actual_q_dot[1]); //RL_THIGH
    //            tar_torque[2] = 400 * (q_err[2]) + 10 * (0 - actual_q_dot[2]); //RL_CALF
    //            tar_torque[3] = 200 * (q_err[3]) + 10 * (0 - actual_q_dot[3]); //RR_HIP
    //            tar_torque[4] = 400 * (q_err[4]) + 10 * (0 - actual_q_dot[4]); //RR_THIGH
    //            tar_torque[5] = 400 * (q_err[5]) + 10 * (0 - actual_q_dot[5]); //RR_CALF
    //        
    //            tar_torque[6] = 200 * (q_err[6]) + 10 * (0 - actual_q_dot[6]); //WAIST
    //        
    //            tar_torque[7] = 200 * (q_err[7]) + 10 * (0 - actual_q_dot[7]); //FL_HIP
    //            tar_torque[8] = 400 * (q_err[8]) + 10 * (0 - actual_q_dot[8]); //FL_THIGH
    //            tar_torque[9] = 400 * (q_err[9]) + 10 * (0 - actual_q_dot[9]); //FL_CALF
    //            tar_torque[10] = 200 * (q_err[10]) + 10 * (0 - actual_q_dot[10]); //FR_HIP
    //            tar_torque[11] = 400 * (q_err[11]) + 10 * (0 - actual_q_dot[11]); //FR_THIGH
    //            tar_torque[12] = 400 * (q_err[12]) + 10 * (0 - actual_q_dot[12]); //FR_CALF

    //* Applying torques
    this->RL_HIP_JOINT->SetForce(1, tar_torque[0]);
    this->RL_THIGH_JOINT->SetForce(1, tar_torque[1]);
    this->RL_CALF_JOINT->SetForce(1, tar_torque[2]);

    this->RR_HIP_JOINT->SetForce(1, tar_torque[3]);
    this->RR_THIGH_JOINT->SetForce(1, tar_torque[4]);
    this->RR_CALF_JOINT->SetForce(1, tar_torque[5]);

    this->WAIST_JOINT->SetForce(1, tar_torque[6]);

    this->FL_HIP_JOINT->SetForce(1, tar_torque[7]);
    this->FL_THIGH_JOINT->SetForce(1, tar_torque[8]);
    this->FL_CALF_JOINT->SetForce(1, tar_torque[9]);

    this->FR_HIP_JOINT->SetForce(1, tar_torque[10]);
    this->FR_THIGH_JOINT->SetForce(1, tar_torque[11]);
    this->FR_CALF_JOINT->SetForce(1, tar_torque[12]);

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

    //setting for getting dt
    this->last_update_time = current_time;

    //***************************RQT DATA********************************//
    //getting readable angular_velocity data
    m_Times.data = current_time.Double();
    m_angular_vel_x.data = angular_vel_x;
    m_angular_vel_y.data = angular_vel_y;
    m_angular_vel_z.data = angular_vel_z;
    m_linear_acc_x.data = linear_acc_x;
    m_linear_acc_y.data = linear_acc_y;
    m_linear_acc_z.data = linear_acc_z;

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
    m_pitch.data = pitch * 180 / PI;
    m_roll.data = roll * 180 / PI;
    m_yaw.data = yaw * 180 / PI;
    m_kalman_pitch.data = EulerKalman[1];
    m_kalman_roll.data = EulerKalman[0];
    m_pitch_err.data = pitch - EulerKalman[1];
    m_roll_err.data = roll - EulerKalman[0];
    m_roll_gyro.data = roll_gyro;
    m_pitch_gyro.data = pitch_gyro;
    m_yaw_gyro.data = yaw_gyro;
    m_roll_acc.data = roll_acc;
    m_pitch_acc.data = pitch_acc;
    m_yaw_acc.data = yaw_acc;
    m_roll_comp.data = roll_comp * 180 / PI;
    m_pitch_comp.data = pitch_comp * 180 / PI;

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

    m_joint_states.position[0] = actual_q[0]; //RL_HIP
    m_joint_states.position[1] = actual_q[1]; //RL_THIGH
    m_joint_states.position[2] = actual_q[2]; //RL_CALF
    m_joint_states.position[3] = actual_q[3]; //RR_HIP
    m_joint_states.position[4] = actual_q[4]; //RR_THIGH
    m_joint_states.position[5] = actual_q[5]; //RR_CALF
    m_joint_states.position[6] = actual_q[6]; //WAIST
    m_joint_states.position[7] = actual_q[7]; //FL_HIP
    m_joint_states.position[8] = actual_q[8]; //FL_THIGH 
    m_joint_states.position[9] = actual_q[9]; //FL_CALF
    m_joint_states.position[10] = actual_q[10]; //FR_HIP
    m_joint_states.position[11] = actual_q[11]; //FR_THIGH 
    m_joint_states.position[12] = actual_q[12]; //FR_CALF


    odom_trans.header.stamp = ros::Time::now();
    odom_trans.transform.translation.x = pose.pos.x;
    odom_trans.transform.translation.y = pose.pos.y;
    odom_trans.transform.translation.z = pose.pos.z;
    odom_trans.transform.rotation.x = pose.rot.x;
    odom_trans.transform.rotation.y = pose.rot.y;
    odom_trans.transform.rotation.z = pose.rot.z;
    odom_trans.transform.rotation.w = pose.rot.w;

    m_RL_force.header.stamp = ros::Time::now();
    m_RL_force.wrench.force.x = RL_Force_I[0];
    m_RL_force.wrench.force.y = RL_Force_I[1];
    m_RL_force.wrench.force.z = RL_Force_I[2];
    m_RL_force.wrench.torque.x = RL_Torque_I[0];
    m_RL_force.wrench.torque.y = RL_Torque_I[1];
    m_RL_force.wrench.torque.z = RL_Torque_I[2];
    m_RR_force.header.stamp = ros::Time::now();
    m_RR_force.wrench.force.x = RR_Force_I[0];
    m_RR_force.wrench.force.y = RR_Force_I[1];
    m_RR_force.wrench.force.z = RR_Force_I[2];
    m_RR_force.wrench.torque.x = RR_Torque_I[0];
    m_RR_force.wrench.torque.y = RR_Torque_I[1];
    m_RR_force.wrench.torque.z = RR_Torque_I[2];
    m_FL_force.header.stamp = ros::Time::now();
    m_FL_force.wrench.force.x = FL_Force_I[0];
    m_FL_force.wrench.force.y = FL_Force_I[1];
    m_FL_force.wrench.force.z = FL_Force_I[2];
    m_FL_force.wrench.torque.x = FL_Torque_I[0];
    m_FL_force.wrench.torque.y = FL_Torque_I[1];
    m_FL_force.wrench.torque.z = FL_Torque_I[2];
    m_FR_force.header.stamp = ros::Time::now();
    m_FR_force.wrench.force.x = FR_Force_I[0];
    m_FR_force.wrench.force.y = FR_Force_I[1];
    m_FR_force.wrench.force.z = FR_Force_I[2];
    m_FR_force.wrench.torque.x = FR_Torque_I[0];
    m_FR_force.wrench.torque.y = FR_Torque_I[1];
    m_FR_force.wrench.torque.z = FR_Torque_I[2];

    //publishing data
    P_Times.publish(m_Times);
    P_angular_vel_x.publish(m_angular_vel_x);
    P_angular_vel_y.publish(m_angular_vel_y);
    P_angular_vel_z.publish(m_angular_vel_z);
    P_linear_acc_x.publish(m_linear_acc_x);
    P_linear_acc_y.publish(m_linear_acc_y);
    P_linear_acc_z.publish(m_linear_acc_z);

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

    broadcaster.sendTransform(odom_trans);
    P_joint_states.publish(m_joint_states);
    P_RL_force.publish(m_RL_force);
    P_RR_force.publish(m_RR_force);
    P_FL_force.publish(m_FL_force);
    P_FR_force.publish(m_FR_force);
    
    P_pitch.publish(m_pitch);
    P_roll.publish(m_roll);
    P_yaw.publish(m_yaw);
    P_kalman_pitch.publish(m_kalman_pitch);
    P_kalman_roll.publish(m_kalman_roll);
    P_pitch_err.publish(m_pitch_err);
    P_roll_err.publish(m_roll_err);
    P_roll_gyro.publish(m_roll_gyro);
    P_pitch_gyro.publish(m_pitch_gyro);
    P_yaw_gyro.publish(m_yaw_gyro);
    P_roll_acc.publish(m_roll_acc);
    P_pitch_acc.publish(m_pitch_acc);
    P_yaw_acc.publish(m_yaw_acc);
    P_roll_comp.publish(m_roll_comp);
    P_pitch_comp.publish(m_pitch_comp);
}

void gazebo::PongBotQ_plugin::Init_Pos_Traj(void)
{
    tar_torque[0] = 100 * (q_err[0]) + 10 * (0 - actual_q_dot[0]); //RL_HIP
    tar_torque[1] = 200 * (q_err[1]) + 10 * (0 - actual_q_dot[1]); //RL_THIGH
    tar_torque[2] = 200 * (q_err[2]) + 10 * (0 - actual_q_dot[2]); //RL_CALF
    tar_torque[3] = 100 * (q_err[3]) + 10 * (0 - actual_q_dot[3]); //RR_HIP
    tar_torque[4] = 200 * (q_err[4]) + 10 * (0 - actual_q_dot[4]); //RR_THIGH
    tar_torque[5] = 200 * (q_err[5]) + 10 * (0 - actual_q_dot[5]); //RR_CALF

    tar_torque[6] = 100 * (q_err[6]) + 10 * (0 - actual_q_dot[6]); //WAIST

    tar_torque[7] = 100 * (q_err[7]) + 10 * (0 - actual_q_dot[7]); //FL_HIP
    tar_torque[8] = 200 * (q_err[8]) + 10 * (0 - actual_q_dot[8]); //FL_THIGH
    tar_torque[9] = 200 * (q_err[9]) + 10 * (0 - actual_q_dot[9]); //FL_CALF
    tar_torque[10] = 100 * (q_err[10]) + 10 * (0 - actual_q_dot[10]); //FR_HIP
    tar_torque[11] = 200 * (q_err[11]) + 10 * (0 - actual_q_dot[11]); //FR_THIGH
    tar_torque[12] = 200 * (q_err[12]) + 10 * (0 - actual_q_dot[12]); //FR_CALF
}

void gazebo::PongBotQ_plugin::Home_Pos_Traj(void)
{
    //    PongBotQ.target_EP.block<12, 1>(0, 0) = PongBotQ.goal_EP;
    //    PongBotQ.target_EP_vel << 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
    //    PongBotQ.target_EP_acc << 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;

    TROT_PHASE = STOP;

    if (ctc_cnt == 0) {

        // get actual End point

        PongBotQ.RobotState(AXIS_X) = PongBotQ.base.currentX;
        PongBotQ.RobotState(AXIS_Y) = PongBotQ.base.currentY;
        PongBotQ.RobotState(AXIS_Z) = PongBotQ.base.currentZ;
        PongBotQ.RobotState(AXIS_Roll) = PongBotQ.base.currentRoll;
        PongBotQ.RobotState(AXIS_Pitch) = PongBotQ.base.currentPitch;
        PongBotQ.RobotState(AXIS_Yaw) = PongBotQ.base.currentYaw;
        PongBotQ.RobotStatedot(AXIS_X) = PongBotQ.base.currentXvel;
        PongBotQ.RobotStatedot(AXIS_Y) = PongBotQ.base.currentYvel;
        PongBotQ.RobotStatedot(AXIS_Z) = PongBotQ.base.currentZvel;
        PongBotQ.RobotStatedot(AXIS_Roll) = PongBotQ.base.currentRollvel;
        PongBotQ.RobotStatedot(AXIS_Pitch) = PongBotQ.base.currentPitchvel;
        PongBotQ.RobotStatedot(AXIS_Yaw) = PongBotQ.base.currentYawvel;

        for (int nJoint = 0; nJoint < PongBotQ.nDOF; nJoint++) {
            PongBotQ.RobotState(6 + nJoint) = PongBotQ.joint[nJoint].currentAngle;
            PongBotQ.RobotStatedot(6 + nJoint) = PongBotQ.joint[nJoint].currentVel;
        }

        PongBotQ.EP_RL = CalcBodyToBaseCoordinates(*pongbot_q_model, PongBotQ.RobotState, PongBotQ.RL.ID, PongBotQ.EP_OFFSET_RL, true);
        PongBotQ.EP_RR = CalcBodyToBaseCoordinates(*pongbot_q_model, PongBotQ.RobotState, PongBotQ.RR.ID, PongBotQ.EP_OFFSET_RR, true);
        PongBotQ.EP_FL = CalcBodyToBaseCoordinates(*pongbot_q_model, PongBotQ.RobotState, PongBotQ.FL.ID, PongBotQ.EP_OFFSET_FL, true);
        PongBotQ.EP_FR = CalcBodyToBaseCoordinates(*pongbot_q_model, PongBotQ.RobotState, PongBotQ.FR.ID, PongBotQ.EP_OFFSET_FR, true);

        PongBotQ.actual_EP.block<3, 1>(0, 0) = PongBotQ.EP_RL;
        PongBotQ.actual_EP.block<3, 1>(3, 0) = PongBotQ.EP_RR;
        PongBotQ.actual_EP.block<3, 1>(6, 0) = PongBotQ.EP_FL;
        PongBotQ.actual_EP.block<3, 1>(9, 0) = PongBotQ.EP_FR;


        for (unsigned int i = 0; i < 12; ++i) {
            PongBotQ.init_EP[i] = PongBotQ.actual_EP[i];
            PongBotQ.target_EP[i] = PongBotQ.actual_EP[i];
            PongBotQ.target_EP_vel[i] = 0;
            PongBotQ.target_EP_acc[i] = 0;
        }

        ctc_cnt++;

    }
    else if (ctc_cnt <= (unsigned int) (home_pos_time / dt)) {

        for (unsigned int i = 0; i < 12; ++i) {
            PongBotQ.target_EP[i] = PongBotQ.init_EP[i] + (PongBotQ.goal_EP[i] - PongBotQ.init_EP[i]) / 2.0 * (1 - cos(PI2 / (home_pos_time * 2)*(double) (ctc_cnt) * dt));
            PongBotQ.target_EP_vel[i] = (PongBotQ.goal_EP[i] - PongBotQ.init_EP[i]) / 2.0 * PI2 / (home_pos_time * 2)*(sin(PI2 / (home_pos_time * 2)*(double) (ctc_cnt) * dt));
            PongBotQ.target_EP_acc[i] = (PongBotQ.goal_EP[i] - PongBotQ.init_EP[i]) / 2.0 * PI2 / (home_pos_time * 2) * PI2 / (home_pos_time * 2)*(cos(PI2 / (home_pos_time * 2)*(double) (ctc_cnt) * dt));
        }

        ctc_cnt++;
    }
    else {
        for (unsigned int i = 0; i < 12; ++i) {
            PongBotQ.target_EP[i] = PongBotQ.goal_EP[i];
            PongBotQ.target_EP_vel[i] = 0;
            PongBotQ.target_EP_acc[i] = 0;
        }

        ctc_cnt2 = 0;
        trot_init_flag = true;
    }

    //    cout << "target_EP = " << PongBotQ.target_EP*R2D << endl;

}

void gazebo::PongBotQ_plugin::Pos_Init_Traj(void)
{
    //    TROT_PHASE = STOP;
    // 
    //    for(unsigned int i=0; i<12; ++i){
    // 
    //        RB_CON.target_EP[i] = RB_CON.goal_EP[i];//RB_CON.init_EP[i] + (RB_CON.trot_goal_EP[i] - RB_CON.init_EP[i])/2.0*(1-cos(PI2/(trot_time)*(double)(ctc_cnt2)*RB_CON.dt));
    //        RB_CON.target_EP_vel[i] = 0;
    //    }
    // 
    //    static double Ki = -0.001;
    //    static double sum_Roll_err = 0, sum_Pitch_err = 0;
    ////  static double F_RL_error = 0,F_RR_error = 0,F_FL_error = 0;//,F_FR_error = 0;
    ////  static double sum_F_RL_error = 0,sum_F_RR_error = 0,sum_F_FL_error = 0;//,sum_F_FR_error = 0;
    // 
    //    sum_Roll_err = sum_Roll_err + RB_CON.dt*(-1.0 - IMURoll);
    //    sum_Pitch_err = sum_Pitch_err + RB_CON.dt*(0 - IMUPitch);
    // 
    ////
    //    RB_CON.target_EP[5] = RB_CON.goal_EP[5]   + Ki*sum_Roll_err + Ki*sum_Pitch_err;
    //    RB_CON.target_EP[11] = RB_CON.goal_EP[11] + Ki*sum_Roll_err;
    // 
    //    RB_CON.target_EP[2] = RB_CON.goal_EP[2] + Ki*sum_Pitch_err;
    // 
    ////  cout << "Roll = " << IMURoll << "Pitch = " << IMUPitch << endl;

}

void gazebo::PongBotQ_plugin::TROT_Traj(void)
{
    //    PongBotQ.goal_EP << 0, 0.218, -0.35, 0, -0.218, -0.35, 0.7, 0.218, -0.35, 0.7, -0.218, -0.35;
    //    PongBotQ.target_EP.block<12, 1>(0, 0) = PongBotQ.goal_EP;
    //    PongBotQ.target_EP_vel << 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
    //    PongBotQ.target_EP_acc << 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;

    const double dsp_time = 0.25;
    const double fsp_time = 0.05;
    const double step_time = dsp_time + fsp_time;

    if (ctc_cnt2 == 0) {// && FOOT_PHASE == FOUR){
        TROT_PHASE = STOP;
        ctc_cnt2++;
    }
    else if (ctc_cnt2 <= (unsigned int) (step_time / dt)) {
        if (ctc_cnt2 <= (unsigned int) (dsp_time / dt)) {// && FOOT_PHASE == TWO_RR_FL){
            TROT_PHASE = STANCE_RRFL;
        }
        else {
            TROT_PHASE = STOP; //STANCE_FOUR_LEGS_AFTER_RRFL;
        }

        ctc_cnt2++;
    }
    else if (ctc_cnt2 <= (unsigned int) ((step_time * 2) / dt)) {
        if (ctc_cnt2 <= (unsigned int) ((step_time + dsp_time) / dt)) {// && FOOT_PHASE == TWO_RL_FR){
            TROT_PHASE = STANCE_RLFR;
        }
        else {
            TROT_PHASE = STOP;
        }

        ctc_cnt2++;
    }
    else {
        //        cout << "2.4[DONE]" << endl;
        TROT_PHASE = STOP;

        static unsigned int step_cnt = 0;
        static unsigned int step_num = 5;

        if (step_cnt < step_num - 1) {
            ctc_cnt2 = 1;
        }

        step_cnt++;
        ctc_cnt = 0;
    }


    switch (TROT_PHASE) {

    case STOP:

        if (trot_init_flag == true) {
            for (unsigned int i = 0; i < 12; ++i) {
                PongBotQ.init_EP[i] = PongBotQ.target_EP[i];
                PongBotQ.target_EP_vel[i] = 0;
                PongBotQ.target_EP_acc[i] = 0;
            }

            trot_init_flag = false;
        }

        for (unsigned int i = 0; i < 12; ++i) {
            if (i == 2 || i == 5 || i == 8 || i == 11) {
                PongBotQ.trot_goal_EP[i] = PongBotQ.init_EP[i] + 0.10;
            }
            else {
                PongBotQ.trot_goal_EP[i] = PongBotQ.init_EP[i];
            }
        }

        break;

    case STANCE_RRFL:

        for (unsigned int i = 0; i < 3; ++i) {

            //            if (i == 0) {
            //                PongBotQ.target_EP[i] = PongBotQ.init_EP[i];// + (PongBotQ.trot_goal_EP[i] - PongBotQ.init_EP[i]) / 2.0 * (1 - cos(PI2 / (dsp_time * 2)*(double) (ctc_cnt2) * dt));
            //                PongBotQ.target_EP_vel[i] = 0;//(PongBotQ.trot_goal_EP[i] - PongBotQ.init_EP[i]) / 2.0 * PI2 / (dsp_time * 2)*(sin(PI2 / (dsp_time * 2)*(double) (ctc_cnt2) * dt));
            //                PongBotQ.target_EP_acc[i] = 0;//(PongBotQ.trot_goal_EP[i] - PongBotQ.init_EP[i]) / 2.0 * PI2 / (dsp_time * 2) * PI2 / (dsp_time * 2)*(cos(PI2 / (dsp_time * 2)*(double) (ctc_cnt2) * dt));
            //            }
            //            else if (i == 1) {
            //                PongBotQ.target_EP[i] = PongBotQ.init_EP[i];//PongBotQ.init_EP[i] + (PongBotQ.trot_goal_EP[i] - PongBotQ.init_EP[i]) / 2.0 * (1 - cos(PI2 / (dsp_time * 2)*(double) (ctc_cnt2) * dt));
            //                PongBotQ.target_EP_vel[i] = 0;//(PongBotQ.trot_goal_EP[i] - PongBotQ.init_EP[i]) / 2.0 * PI2 / (dsp_time * 2)*(sin(PI2 / (dsp_time * 2)*(double) (ctc_cnt2) * dt));
            //                PongBotQ.target_EP_acc[i] = 0;//(PongBotQ.trot_goal_EP[i] - PongBotQ.init_EP[i]) / 2.0 * PI2 / (dsp_time * 2) * PI2 / (dsp_time * 2)*(cos(PI2 / (dsp_time * 2)*(double) (ctc_cnt2) * dt));
            //            }
            //            else { // z
            //                PongBotQ.target_EP[i] = PongBotQ.init_EP[i] + (PongBotQ.trot_goal_EP[i] - PongBotQ.init_EP[i]) / 2.0 * (1 - cos(PI2 / (dsp_time)*(double) (ctc_cnt2) * dt));
            //                PongBotQ.target_EP_vel[i] = (PongBotQ.trot_goal_EP[i] - PongBotQ.init_EP[i]) / 2.0 * PI2 / (dsp_time)*(sin(PI2 / (dsp_time)*(double) (ctc_cnt2) * dt));
            //                PongBotQ.target_EP_acc[i] = (PongBotQ.trot_goal_EP[i] - PongBotQ.init_EP[i]) / 2.0 * PI2 / (dsp_time) * PI2 / (dsp_time)*(cos(PI2 / (dsp_time)*(double) (ctc_cnt2) * dt));
            //            }

            PongBotQ.target_EP[i] = PongBotQ.init_EP[i] + (PongBotQ.trot_goal_EP[i] - PongBotQ.init_EP[i]) / 2.0 * (1 - cos(PI2 / (dsp_time)*(double) (ctc_cnt2) * dt));
            PongBotQ.target_EP_vel[i] = (PongBotQ.trot_goal_EP[i] - PongBotQ.init_EP[i]) / 2.0 * PI2 / (dsp_time)*(sin(PI2 / (dsp_time)*(double) (ctc_cnt2) * dt));
            PongBotQ.target_EP_acc[i] = (PongBotQ.trot_goal_EP[i] - PongBotQ.init_EP[i]) / 2.0 * PI2 / (dsp_time) * PI2 / (dsp_time)*(cos(PI2 / (dsp_time)*(double) (ctc_cnt2) * dt));
        }

        for (unsigned int i = 3; i < 6; ++i) {
            PongBotQ.target_EP[i] = PongBotQ.init_EP[i];
            PongBotQ.target_EP_vel[i] = 0;
            PongBotQ.target_EP_acc[i] = 0;
        }

        for (unsigned int i = 6; i < 9; ++i) {
            PongBotQ.target_EP[i] = PongBotQ.init_EP[i];
            PongBotQ.target_EP_vel[i] = 0;
            PongBotQ.target_EP_acc[i] = 0;
        }

        for (unsigned int i = 9; i < 12; ++i) {

            //            if (i == 9) {
            //                PongBotQ.target_EP[i] = PongBotQ.init_EP[i];//PongBotQ.init_EP[i] + (PongBotQ.trot_goal_EP[i] - PongBotQ.init_EP[i]) / 2.0 * (1 - cos(PI2 / (dsp_time * 2)*(double) (ctc_cnt2) * dt));
            //                PongBotQ.target_EP_vel[i] = 0;//(PongBotQ.trot_goal_EP[i] - PongBotQ.init_EP[i]) / 2.0 * PI2 / (dsp_time * 2)*(sin(PI2 / (dsp_time * 2)*(double) (ctc_cnt2) * dt));
            //                PongBotQ.target_EP_acc[i] = 0;//(PongBotQ.trot_goal_EP[i] - PongBotQ.init_EP[i]) / 2.0 * PI2 / (dsp_time * 2) * PI2 / (dsp_time * 2)*(cos(PI2 / (dsp_time * 2)*(double) (ctc_cnt2) * dt));
            //            }
            //            else if (i == 10) {
            //                PongBotQ.target_EP[i] = PongBotQ.init_EP[i];//PongBotQ.init_EP[i] + (PongBotQ.trot_goal_EP[i] - PongBotQ.init_EP[i]) / 2.0 * (1 - cos(PI2 / (dsp_time * 2)*(double) (ctc_cnt2) * dt));
            //                PongBotQ.target_EP_vel[i] = 0;//(PongBotQ.trot_goal_EP[i] - PongBotQ.init_EP[i]) / 2.0 * PI2 / (dsp_time * 2)*(sin(PI2 / (dsp_time * 2)*(double) (ctc_cnt2) * dt));
            //                PongBotQ.target_EP_acc[i] = 0;//(PongBotQ.trot_goal_EP[i] - PongBotQ.init_EP[i]) / 2.0 * PI2 / (dsp_time * 2) * PI2 / (dsp_time * 2)*(cos(PI2 / (dsp_time * 2)*(double) (ctc_cnt2) * dt));
            //            }
            //            else { // z
            //                PongBotQ.target_EP[i] = PongBotQ.init_EP[i] + (PongBotQ.trot_goal_EP[i] - PongBotQ.init_EP[i]) / 2.0 * (1 - cos(PI2 / (dsp_time)*(double) (ctc_cnt2) * dt));
            //                PongBotQ.target_EP_vel[i] = (PongBotQ.trot_goal_EP[i] - PongBotQ.init_EP[i]) / 2.0 * PI2 / (dsp_time)*(sin(PI2 / (dsp_time)*(double) (ctc_cnt2) * dt));
            //                PongBotQ.target_EP_acc[i] = (PongBotQ.trot_goal_EP[i] - PongBotQ.init_EP[i]) / 2.0 * PI2 / (dsp_time) * PI2 / (dsp_time)*(cos(PI2 / (dsp_time)*(double) (ctc_cnt2) * dt));
            //            }

            PongBotQ.target_EP[i] = PongBotQ.init_EP[i] + (PongBotQ.trot_goal_EP[i] - PongBotQ.init_EP[i]) / 2.0 * (1 - cos(PI2 / (dsp_time)*(double) (ctc_cnt2) * dt));
            PongBotQ.target_EP_vel[i] = (PongBotQ.trot_goal_EP[i] - PongBotQ.init_EP[i]) / 2.0 * PI2 / (dsp_time)*(sin(PI2 / (dsp_time)*(double) (ctc_cnt2) * dt));
            PongBotQ.target_EP_acc[i] = (PongBotQ.trot_goal_EP[i] - PongBotQ.init_EP[i]) / 2.0 * PI2 / (dsp_time) * PI2 / (dsp_time)*(cos(PI2 / (dsp_time)*(double) (ctc_cnt2) * dt));
        }

        //        cout << "[STANCE_RRFL]target_EP = " << PongBotQ.target_EP.transpose() << endl;

        break;


    case STANCE_RLFR:
        for (unsigned int i = 0; i < 3; ++i) {

            //            if (i == 0) {
            //                PongBotQ.target_EP[i] = PongBotQ.trot_goal_EP[i] + (PongBotQ.init_EP[i] - PongBotQ.trot_goal_EP[i]) / 2.0 * (1 - cos(PI2 / (dsp_time * 2)*((double) ((ctc_cnt2) * dt) - step_time)));
            //                PongBotQ.target_EP_vel[i] = (PongBotQ.init_EP[i] - PongBotQ.trot_goal_EP[i]) / 2.0 * PI2 / (dsp_time * 2)*(sin(PI2 / (dsp_time * 2)*((double) ((ctc_cnt2) * dt) - step_time)));
            //                PongBotQ.target_EP_acc[i] = (PongBotQ.init_EP[i] - PongBotQ.trot_goal_EP[i]) / 2.0 * PI2 / (dsp_time * 2) * PI2 / (dsp_time * 2)*(cos(PI2 / (dsp_time * 2)*((double) ((ctc_cnt2) * dt) - step_time)));
            //            }
            //            else if (i == 1) {
            //                PongBotQ.target_EP[i] = PongBotQ.trot_goal_EP[i] + (PongBotQ.init_EP[i] - PongBotQ.trot_goal_EP[i]) / 2.0 * (1 - cos(PI2 / (dsp_time * 2)*((double) ((ctc_cnt2) * dt) - step_time)));
            //                PongBotQ.target_EP_vel[i] = (PongBotQ.init_EP[i] - PongBotQ.trot_goal_EP[i]) / 2.0 * PI2 / (dsp_time * 2)*(sin(PI2 / (dsp_time * 2)*((double) ((ctc_cnt2) * dt) - step_time)));
            //                PongBotQ.target_EP_acc[i] = (PongBotQ.init_EP[i] - PongBotQ.trot_goal_EP[i]) / 2.0 * PI2 / (dsp_time * 2) * PI2 / (dsp_time * 2)*(cos(PI2 / (dsp_time * 2)*((double) ((ctc_cnt2) * dt) - step_time)));
            //            }
            //            else { // z
            //                PongBotQ.target_EP[i] = PongBotQ.init_EP[i];
            //                PongBotQ.target_EP_vel[i] = 0;
            //                PongBotQ.target_EP_acc[i] = 0;
            //            }

            PongBotQ.target_EP[i] = PongBotQ.init_EP[i];
            PongBotQ.target_EP_vel[i] = 0;
            PongBotQ.target_EP_acc[i] = 0;
        }

        for (unsigned int i = 3; i < 6; ++i) {
            PongBotQ.target_EP[i] = PongBotQ.init_EP[i] + (PongBotQ.trot_goal_EP[i] - PongBotQ.init_EP[i]) / 2.0 * (1 - cos(PI2 / (dsp_time)*((double) ((ctc_cnt2) * dt) - step_time)));
            PongBotQ.target_EP_vel[i] = (PongBotQ.trot_goal_EP[i] - PongBotQ.init_EP[i]) / 2.0 * PI2 / (dsp_time)*(sin(PI2 / (dsp_time)*((double) ((ctc_cnt2) * dt) - step_time)));
            PongBotQ.target_EP_acc[i] = (PongBotQ.trot_goal_EP[i] - PongBotQ.init_EP[i]) / 2.0 * PI2 / (dsp_time) * PI2 / (dsp_time)*(cos(PI2 / (dsp_time)*((double) ((ctc_cnt2) * dt) - step_time)));

        }

        for (unsigned int i = 6; i < 9; ++i) {
            PongBotQ.target_EP[i] = PongBotQ.init_EP[i] + (PongBotQ.trot_goal_EP[i] - PongBotQ.init_EP[i]) / 2.0 * (1 - cos(PI2 / (dsp_time)*((double) ((ctc_cnt2) * dt) - step_time)));
            PongBotQ.target_EP_vel[i] = (PongBotQ.trot_goal_EP[i] - PongBotQ.init_EP[i]) / 2.0 * PI2 / (dsp_time)*(sin(PI2 / (dsp_time)*((double) ((ctc_cnt2) * dt) - step_time)));
            PongBotQ.target_EP_acc[i] = (PongBotQ.trot_goal_EP[i] - PongBotQ.init_EP[i]) / 2.0 * PI2 / (dsp_time) * PI2 / (dsp_time)*(cos(PI2 / (dsp_time)*((double) ((ctc_cnt2) * dt) - step_time)));

        }

        for (unsigned int i = 9; i < 12; ++i) {


            //            if (i == 9) {
            //                PongBotQ.target_EP[i] = PongBotQ.trot_goal_EP[i] + (PongBotQ.init_EP[i] - PongBotQ.trot_goal_EP[i]) / 2.0 * (1 - cos(PI2 / (dsp_time * 2)*((double) ((ctc_cnt2) * dt) - step_time)));
            //                PongBotQ.target_EP_vel[i] = (PongBotQ.init_EP[i] - PongBotQ.trot_goal_EP[i]) / 2.0 * PI2 / (dsp_time * 2)*(sin(PI2 / (dsp_time * 2)*((double) ((ctc_cnt2) * dt) - step_time)));
            //                PongBotQ.target_EP_acc[i] = (PongBotQ.init_EP[i] - PongBotQ.trot_goal_EP[i]) / 2.0 * PI2 / (dsp_time * 2) * PI2 / (dsp_time * 2)*(cos(PI2 / (dsp_time * 2)*((double) ((ctc_cnt2) * dt) - step_time)));
            //            }
            //            else if (i == 10) {
            //                PongBotQ.target_EP[i] = PongBotQ.trot_goal_EP[i] + (PongBotQ.init_EP[i] - PongBotQ.trot_goal_EP[i]) / 2.0 * (1 - cos(PI2 / (dsp_time * 2)*((double) ((ctc_cnt2) * dt) - step_time)));
            //                PongBotQ.target_EP_vel[i] = (PongBotQ.init_EP[i] - PongBotQ.trot_goal_EP[i]) / 2.0 * PI2 / (dsp_time * 2)*(sin(PI2 / (dsp_time * 2)*((double) ((ctc_cnt2) * dt) - step_time)));
            //                PongBotQ.target_EP_acc[i] = (PongBotQ.init_EP[i] - PongBotQ.trot_goal_EP[i]) / 2.0 * PI2 / (dsp_time * 2) * PI2 / (dsp_time * 2)*(cos(PI2 / (dsp_time * 2)*((double) ((ctc_cnt2) * dt) - step_time)));
            //            }
            //            else { // z
            //                PongBotQ.target_EP[i] = PongBotQ.init_EP[i];
            //                PongBotQ.target_EP_vel[i] = 0;
            //                PongBotQ.target_EP_acc[i] = 0;
            //            }

            PongBotQ.target_EP[i] = PongBotQ.init_EP[i];
            PongBotQ.target_EP_vel[i] = 0;
            PongBotQ.target_EP_acc[i] = 0;
        }
        break;
    }
}

MatrixXd gazebo::PongBotQ_plugin::SystemMatrix(double wx, double wy, double wz, double dt)
{
    MatrixXd A(4, 4), tmp_matrix(4, 4);
    tmp_matrix << 0, -wx, -wy, -wz \
              , wx, 0, wz, -wy \
              , wy, -wz, 0, wx \
              , wz, wy, -wx, 0;

    A = MatrixXd::Identity(4, 4) + dt * 1 / 2 * tmp_matrix;
    return A;
}

VectorXd gazebo::PongBotQ_plugin::GetEulerAccel(double ax, double ay, double az)
{
    VectorXd EulerAccel(3);
    double theta, phi;

    phi = atan(ay / az);
    theta = atan(ax / (sqrt(ay * ay + az * az)));

    if (isnan(phi) || isnan(theta)) {
        EulerAccel(0) = 0; //Roll
        EulerAccel(1) = 0; //PITCH
        EulerAccel(2) = 0;
    }
    else {
        EulerAccel(0) = phi; //Roll
        EulerAccel(1) = theta; //PITCH
        EulerAccel(2) = 0;
    }


    return EulerAccel;
}

VectorXd gazebo::PongBotQ_plugin::EulerToQuaternion(double roll, double pitch, double yaw)
{
    VectorXd z(4);

    z(0) = cos(roll / 2) * cos(pitch / 2) * cos(yaw / 2) + sin(roll / 2) * sin(pitch / 2) * sin(yaw / 2);
    z(1) = sin(roll / 2) * cos(pitch / 2) * cos(yaw / 2) - cos(roll / 2) * sin(pitch / 2) * sin(yaw / 2);
    z(2) = cos(roll / 2) * sin(pitch / 2) * cos(yaw / 2) + sin(roll / 2) * cos(pitch / 2) * sin(yaw / 2);
    z(3) = cos(roll / 2) * cos(pitch / 2) * sin(yaw / 2) - sin(roll / 2) * sin(pitch / 2) * cos(yaw / 2);
    return z;
}

VectorXd gazebo::PongBotQ_plugin::GetEulerKalman(MatrixXd A, VectorXd z)
{
    VectorXd EulerKalman(3);
    VectorXd xp(4), x_new(4);
    MatrixXd Pp(4, 4), P_new(4, 4);
    MatrixXd tmp(4, 4), K(4, 4);

    double phi, theta, psi;

    if (g_firstRun == 0) {
        //g_Q = 0.00001 * MatrixXd::Identity(4, 4);
        //g_R = 100 * MatrixXd::Identity(4, 4);      
        //g_Q = 0.00001 * MatrixXd::Identity(4, 4);
        //g_R = 10 * MatrixXd::Identity(4, 4);  

        g_Q = 0.00001 * MatrixXd::Identity(4, 4);
        g_R = 100 * MatrixXd::Identity(4, 4);
        //        g_Q = MatrixXd::Zero(4, 4);
        //        g_R = MatrixXd::Zero(4, 4);

        g_H = MatrixXd::Identity(4, 4);
        g_P = MatrixXd::Identity(4, 4);
        g_x << 1, 0, 0, 0;
        g_firstRun = 1;

    }
    else {
        xp = A * g_x;
        Pp = A * g_P * A.transpose() + g_Q;
        tmp = g_H * Pp * g_H.transpose() + g_R;
        K = Pp * g_H.transpose() * tmp.inverse();
        x_new = xp + K * (z - g_H * xp);
        P_new = Pp - K * g_H*Pp;
        g_x = x_new;
        g_P = P_new;

        phi = atan2(2 * (x_new(2) * x_new(3) + x_new(0) * x_new(1)), (1 - 2 * (x_new(1) * x_new(1) + x_new(2) * x_new(2))));
        theta = -asin(2 * (x_new(1) * x_new(3) - x_new(0) * x_new(2)));
        psi = atan2(2 * (x_new(1) * x_new(2) + x_new(0) * x_new(3)), (1 - 2 * (x_new(2) * x_new(2) + x_new(3) * x_new(3))));

        EulerKalman(0) = phi * 180 / PI;
        EulerKalman(1) = theta * 180 / PI;
        EulerKalman(2) = psi * 180 / PI;
        return EulerKalman;
    }
}