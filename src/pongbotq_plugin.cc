#include <gazebo/common/common.hh>
#include <gazebo/common/Plugin.hh>
#include <ros/ros.h>
#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/sensors/sensors.hh>
#include <stdio.h>
#include <iostream>
#include <std_msgs/UInt16.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <functional>
#include <ignition/math/Vector3.hh>
// RBDL
#include "Eigen/Dense"
#include <rbdl/rbdl.h>
#include <rbdl/addons/urdfreader/urdfreader.h>
// Rviz
#include <sensor_msgs/JointState.h>             //for rviz
#include <tf/transform_broadcaster.h>           //for rviz
#include <geometry_msgs/WrenchStamped.h>        //for rviz
// by BKCho
#include "CRobot.h" 

using namespace std;

using Eigen::MatrixXd;
using Eigen::VectorXd;
using Eigen::Vector3d;

Model* pongbot_q_model = new Model();

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

        // Time
        common::Time last_update_time;
        event::ConnectionPtr update_connection;
        double dt = 0;
        double time = 0;

        //IMU sensor
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

        //FT sensor
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

        //rqt telecommunication
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
        ros::Publisher P_EP_err;    //Err message made by HSKIM

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
        std_msgs::Float64MultiArray m_EP_err; //Err message made by HSKIM

        // Rviz
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
        double TmpData[30] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
        ros::Publisher ros_pub2;
        std_msgs::Float64MultiArray ros_msg2;

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

        double alpha_roll = 0.998;
        double alpha_pitch = 0.999;
        
        unsigned int ctrl_cnt = 0;
        
        common::Time current_time;

//        ros::Subscriber S_mode;

        //ROS Rqt
//        ros::Subscriber S_ROSMODE;
        
        ros::Subscriber server_sub1;
        ros::Subscriber server_sub2;
        ros::Subscriber server_sub3;
        
        //ROS MODE
        int ROSMode_Flag = 0;
        
        VectorXd Err_EP = VectorXd::Zero(12);    //added by HSKIM
        VectorXd Err_pos = VectorXd::Zero(13);  //added by HSKIM
        
    public:
        //For model load
        void Load(physics::ModelPtr _model, sdf::ElementPtr /*_sdf*/);
        void UpdateAlgorithm();
        
        // Callback function
        void PongBot_Q_ROSmode(const std_msgs::Int32Ptr &msg);
        void Callback(const std_msgs::Int32Ptr &msg);
        void Callback2(const std_msgs::Float64Ptr &msg);
        void Callback3(const std_msgs::Int32Ptr &msg);
        void Print(void); //Print function added by HSKIM
        
        void RBDLSetting();
        void GetLinks();
        void GetJoints();
        void InitROSPubSetting();
        void InitRvizSetting();
        void SensorSetting();
        
        
        void IMUSensorRead();
        void FTSensorRead();
        void EncoderRead();
        
        void jointController();
        void ROSMsgPublish();
        
      
        
    };
    GZ_REGISTER_MODEL_PLUGIN(PongBotQ_plugin);
}

void gazebo::PongBotQ_plugin::Load(physics::ModelPtr _model, sdf::ElementPtr /*_sdf*/)
{
    printf("============= [Load] =============\n");

    this->model = _model;

    RBDLSetting();
    GetLinks();
    GetJoints();
    InitROSPubSetting();
    InitRvizSetting();
    SensorSetting();

    PongBotQ.ControlMode = CTRLMODE_HOME_POS;
    
    //************************Time Setting*********************************//
    this->last_update_time = this->model->GetWorld()->GetSimTime();
    this->update_connection = event::Events::ConnectWorldUpdateBegin(boost::bind(&PongBotQ_plugin::UpdateAlgorithm, this)); 
}

void gazebo::PongBotQ_plugin::UpdateAlgorithm()
{ //* Writing realtime code here!!
    //********************* Base Pose for rviz *****************************//
    pose = this->model->GetWorldPose();

    //************************** Time ********************************//
    current_time = this->model->GetWorld()->GetSimTime();
    dt = current_time.Double() - this->last_update_time.Double();
    time = time + dt;

    //* Read Sensors
    IMUSensorRead();
    FTSensorRead();
    EncoderRead();
    
    //* ControlMode
    switch (PongBotQ.ControlMode) {
        case CTRLMODE_NONE:
//          cout << "============= [CTRLMODE_NONE] ==========" << endl;
//          PongBotQ.CommandFlag = TORQUE_OFF;
//          PongBotQ.CommandFlag = NO_ACT;
            break;
            
        case CTRLMODE_INITIALIZE:
            cout << "============= [CTRLMODE_INITIALIZE] ==========" << endl;

            PongBotQ.ctc_cnt = 0;
            PongBotQ.ctc_cnt2 = 0;

            PongBotQ.CommandFlag = NO_ACT_WITH_CTC;
            PongBotQ.ControlMode = CTRLMODE_NONE;
            break;

        case CTRLMODE_HOME_POS:
            cout << "============= [CTRLMODE_HOME_POS] ==========" << endl;
            PongBotQ.CommandFlag = GOTO_HOME_POS;
            PongBotQ.ControlMode = CTRLMODE_NONE;
            break;

        case CTRLMODE_WALK_READY:
            cout << "============= [CTRLMODE_WALK_READY] ==========" << endl;

            PongBotQ.ctc_cnt = 0;

            for (unsigned int i = 0; i < 13; ++i) {
                    PongBotQ.pre_target_pos[i] = PongBotQ.init_target_pos[i] * D2R;
            }
            
            PongBotQ.actual_EP = PongBotQ.FK1(PongBotQ.actual_pos);
            for (unsigned int i = 0; i < 12; ++i) {
                    PongBotQ.target_EP[i] = PongBotQ.actual_EP[i];
            }

            PongBotQ.CommandFlag = GOTO_WALK_READY_POS;
            PongBotQ.ControlMode = CTRLMODE_NONE;
            break;

        case CTRLMODE_TROT:
            cout << "============= [CTRLMODE_TROT] ==========" << endl;

            PongBotQ.ctc_cnt2 = 0;

            PongBotQ.CommandFlag = NOMAL_TROT_WALKING;
            PongBotQ.ControlMode = CTRLMODE_NONE;
            break;

    }
    
    
    switch (PongBotQ.CommandFlag) {
         case NO_ACT:
            // No action
            break;

        case NO_ACT_WITH_CTC:
            PongBotQ.ComputeTorqueControl();
            break;
            
        case TORQUE_OFF:
            PongBotQ.Torque_off();
            break;

        case GOTO_HOME_POS:
            PongBotQ.Torque_off(); // Temp
            PongBotQ.Init_Pos_Traj(); // Only simulation
            break;

        case GOTO_WALK_READY_POS:
            PongBotQ.Home_Pos_Traj();
            PongBotQ.ComputeTorqueControl();
            break;

        case NOMAL_TROT_WALKING:
//            PongBotQ.moving_speed = rqt_moving_speed;
            PongBotQ.TROT_Traj();
            PongBotQ.ComputeTorqueControl();
            break;
             
    }
 
//    Print();
    
    jointController();
    
    ROSMsgPublish();
}

void gazebo::PongBotQ_plugin::Callback(const std_msgs::Int32Ptr &msg){
    PongBotQ.ControlMode = msg->data;
}

void gazebo::PongBotQ_plugin::Callback2(const std_msgs::Float64Ptr &msg) {
    PongBotQ.moving_speed = msg->data;
}

void gazebo::PongBotQ_plugin::Callback3(const std_msgs::Int32Ptr &msg) {
  PongBotQ.sub_ctrl_flag = (int)(msg->data);
}



void gazebo::PongBotQ_plugin::Print(void)     //Print function added by HSKIM
{
    for (unsigned int i = 0; i < 12; ++i) {
        Err_EP[i] = PongBotQ.target_EP[i] - PongBotQ.actual_EP[i];
    }
    for (unsigned int i = 0; i < 13; ++i) {
        Err_pos[i] = PongBotQ.target_pos[i] - PongBotQ.actual_pos[i];
    }
    //int CNT = CNT + 1;
    //if (CNT % 50 == 0) {
    
    
    
//    std::cout << "Now_EP:" << "RL=" << "(" << PongBotQ.actual_EP(0) << "," << PongBotQ.actual_EP(1) << "," << PongBotQ.actual_EP(2) << ")" << "RR=" << "(" << PongBotQ.actual_EP(3) << "," << PongBotQ.actual_EP(4) << "," << PongBotQ.actual_EP(5) << ")" << "FL=" << "(" << PongBotQ.actual_EP(6) << "," << PongBotQ.actual_EP(7) << "," << PongBotQ.actual_EP(8) << ")" << "FR=" << "(" << PongBotQ.actual_EP(9) << "," << PongBotQ.actual_EP(10) << "," << PongBotQ.actual_EP(11) << ")" << std::endl;
//    std::cout << "Now_q:" << "RL=" << "(" << PongBotQ.actual_pos(0)*R2D << "," << PongBotQ.actual_pos(1)*R2D << "," << PongBotQ.actual_pos(2)*R2D << ")" << "RR=" << "(" << PongBotQ.actual_pos(3)*R2D << "," << PongBotQ.actual_pos(4)*R2D << "," << PongBotQ.actual_pos(5)*R2D << ")" << "FL=" << "(" << PongBotQ.actual_pos(7)*R2D << "," << PongBotQ.actual_pos(8)*R2D << "," << PongBotQ.actual_pos(9)*R2D << ")" << "FR=" << "(" << PongBotQ.actual_pos(10)*R2D << "," << PongBotQ.actual_pos(11)*R2D << "," << PongBotQ.actual_pos(12)*R2D << ")" << std::endl;
//    //std::cout << "Init_EP : " << "RL=" << "(" << PongBotQ.init_EP[0] << "," << PongBotQ.init_EP[1] << "," << PongBotQ.init_EP[2] << ")" << "RR=" << "(" << PongBotQ.init_EP[3] << "," << PongBotQ.init_EP[4] << "," << PongBotQ.init_EP[5] << ")" << "FL=" << "(" << PongBotQ.init_EP[6] << "," << PongBotQ.init_EP[7] << "," << PongBotQ.init_EP[8] << ")" << "FR=" << "(" << PongBotQ.init_EP[9] << "," << PongBotQ.init_EP[10] << "," << PongBotQ.init_EP[11] << ")" << std::endl;
//    //std::cout << "Target_EP : " << "RL=" << "(" << PongBotQ.target_EP[0] << "," << PongBotQ.target_EP[1] << "," << PongBotQ.target_EP[2] << ")" << "RR=" << "(" << PongBotQ.target_EP[3] << "," << PongBotQ.target_EP[4] << "," << PongBotQ.target_EP[5] << ")" << "FL=" << "(" << PongBotQ.target_EP[6] << "," << PongBotQ.target_EP[7] << "," << PongBotQ.target_EP[8] << ")" << "FR=" << "(" << PongBotQ.target_EP[9] << "," << PongBotQ.target_EP[10] << "," << PongBotQ.target_EP[11] << ")" << std::endl;
//    //std::cout << "Goal_EP : " << "RL=" << "(" << PongBotQ.goal_EP[0] << "," << PongBotQ.goal_EP[1] << "," << PongBotQ.goal_EP[2] << ")" << "RR=" << "(" << PongBotQ.goal_EP[3] << "," << PongBotQ.goal_EP[4] << "," << PongBotQ.goal_EP[5] << ")" << "FL=" << "(" << PongBotQ.goal_EP[6] << "," << PongBotQ.goal_EP[7] << "," << PongBotQ.goal_EP[8] << ")" << "FR=" << "(" << PongBotQ.goal_EP[9] << "," << PongBotQ.goal_EP[10] << "," << PongBotQ.goal_EP[11] << ")" << std::endl;
//    //std::cout << "Err_EP:" << "(" << Err_EP[0] << "," << Err_EP[1] << "," << Err_EP[2] << ")" << "," << "(" << Err_EP[3] << "," << Err_EP[4] << "," << Err_EP[5] << ")" << "," << "(" << Err_EP[6] << "," << Err_EP[7] << "," << Err_EP[8] << ")" << "," << "(" << Err_EP[9] << "," << Err_EP[10] <<Err_EP[11]<< ")" << std::endl;
//    //std::cout << "Err_pos:" << "(" << Err_pos[0] * R2D << "," << Err_pos[1] * R2D << "," << Err_pos[2] * R2D << ")" << "," << "(" << Err_pos[3] * R2D << "," << Err_pos[4] * R2D << "," << Err_pos[5] * R2D << ")" << "," << "(" << Err_pos[6] * R2D << ")" << "," << "(" << Err_pos[7] * R2D << "," << Err_pos[8] * R2D << "," << Err_pos[9] * R2D << ")" << "," << "(" << Err_pos[10] * R2D << "," << Err_pos[11] * R2D << "," << Err_pos[12] * R2D << ")" << std::endl;
//    std::cout << "Force:" << "(" << PongBotQ.Fc[7] << "," << PongBotQ.Fc[8] << "," << PongBotQ.Fc[9] << ")" << "," << "(" << PongBotQ.Fc[10] << "," << PongBotQ.Fc[11] << "," << PongBotQ.Fc[12] << ")" << "," << "(" << PongBotQ.Fc[13] << "," << PongBotQ.Fc[14] << "," << PongBotQ.Fc[15] << ")" << "," << "(" << PongBotQ.Fc[16] << "," << PongBotQ.Fc[17] << "," << PongBotQ.Fc[18] << ")" << std::endl;
//    std::cout << "CNT2:" << PongBotQ.ctc_cnt2<<std::endl;
//    std::cout << "acutal_vel : " << "RL=" << "(" << PongBotQ.actual_EP_vel[0] << "," << PongBotQ.actual_EP_vel[1] << "," << PongBotQ.actual_EP_vel[2] << ")" << "RR=" << "(" << PongBotQ.actual_EP_vel[3] << "," << PongBotQ.actual_EP_vel[4] << "," << PongBotQ.actual_EP_vel[5] << ")" << "FL=" << "(" << PongBotQ.actual_EP_vel[6] << "," << PongBotQ.actual_EP_vel[7] << "," << PongBotQ.actual_EP_vel[8] << ")" << "FR=" << "(" << PongBotQ.actual_EP_vel[9] << "," << PongBotQ.actual_EP_vel[10] << "," << PongBotQ.actual_EP_vel[11] << ")" << std::endl;
//    std::cout << "target_vel : " << "RL=" << "(" << PongBotQ.target_EP_vel[0] << "," << PongBotQ.target_EP_vel[1] << "," << PongBotQ.target_EP_vel[2] << ")" << "RR=" << "(" << PongBotQ.target_EP_vel[3] << "," << PongBotQ.target_EP_vel[4] << "," << PongBotQ.target_EP_vel[5] << ")" << "FL=" << "(" << PongBotQ.target_EP_vel[6] << "," << PongBotQ.target_EP_vel[7] << "," << PongBotQ.target_EP_vel[8] << ")" << "FR=" << "(" << PongBotQ.target_EP_vel[9] << "," << PongBotQ.target_EP_vel[10] << "," << PongBotQ.target_EP_vel[11] << ")" << std::endl;
//    std::cout << "EP_vel_err : " << "RL=" << "(" << PongBotQ.EP_vel_err[0] << "," << PongBotQ.EP_vel_err[1] << "," << PongBotQ.EP_vel_err[2] << ")" << "RR=" << "(" << PongBotQ.EP_vel_err[3] << "," << PongBotQ.EP_vel_err[4] << "," << PongBotQ.EP_vel_err[5] << ")" << "FL=" << "(" << PongBotQ.EP_vel_err[6] << "," << PongBotQ.EP_vel_err[7] << "," << PongBotQ.EP_vel_err[8] << ")" << "FR=" << "(" << PongBotQ.EP_vel_err[9] << "," << PongBotQ.EP_vel_err[10] << "," << PongBotQ.EP_vel_err[11] << ")" << std::endl;
//    std::cout << "EP_err : " << "RL=" << "(" << PongBotQ.EP_err[0] << "," << PongBotQ.EP_err[1] << "," << PongBotQ.EP_err[2] << ")" << "RR=" << "(" << PongBotQ.EP_err[3] << "," << PongBotQ.EP_err[4] << "," << PongBotQ.EP_err[5] << ")" << "FL=" << "(" << PongBotQ.EP_err[6] << "," << PongBotQ.EP_err[7] << "," << PongBotQ.EP_err[8] << ")" << "FR=" << "(" << PongBotQ.EP_err[9] << "," << PongBotQ.EP_err[10] << "," << PongBotQ.EP_err[11] << ")" << std::endl;
//    std::cout << "_______________________________________________________________________________________________________________" << std::endl;

    
    
    //}
}     

void gazebo::PongBotQ_plugin::GetLinks() {

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
}

void gazebo::PongBotQ_plugin::GetJoints() {
    
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
    
}



void gazebo::PongBotQ_plugin::InitROSPubSetting() {
    
    //************************ROS Msg Setting*********************************//
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
    P_EP_err = n.advertise<std_msgs::Float64MultiArray>("EP_err", 1); //Err message added by HSKIM
    m_EP_err.data.resize(12);  //Err message added by HSKIM
    
    // DH Publisher
    ros_pub2 = n.advertise<std_msgs::Float64MultiArray>("/tmp_data/", 1000);
    ros_msg2.data.resize(30);
    
    P_RL_force = n.advertise<geometry_msgs::WrenchStamped>("RL_force", 1000);
    P_RR_force = n.advertise<geometry_msgs::WrenchStamped>("RR_force", 1000);
    P_FL_force = n.advertise<geometry_msgs::WrenchStamped>("FL_force", 1000);
    P_FR_force = n.advertise<geometry_msgs::WrenchStamped>("FR_force", 1000);
    m_RL_force.header.frame_id = "RL_TIP";
    m_RR_force.header.frame_id = "RR_TIP";
    m_FL_force.header.frame_id = "FL_TIP";
    m_FR_force.header.frame_id = "FR_TIP";
    //ros::Rate loop_rate(1000);
    server_sub1 = n.subscribe("ctrl_mode", 1, &gazebo::PongBotQ_plugin::Callback, this);    
    
//    S_ROSMODE = n.subscribe("ctrl_mode", 1, &gazebo::PongBotQ_plugin::PongBot_Q_ROSmode, this);
    
    server_sub2 = n.subscribe("rec_data1", 1, &gazebo::PongBotQ_plugin::Callback2, this);
    server_sub3 = n.subscribe("sub_ctrl_mode", 1, &gazebo::PongBotQ_plugin::Callback3, this);
    
}


void gazebo::PongBotQ_plugin::InitRvizSetting() {
//**************************rviz inital setting*************************************//
    P_joint_states = n.advertise<sensor_msgs::JointState>("joint_states", 1);
    m_joint_states.name.resize(13);
    m_joint_states.position.resize(13);
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "REAR_BODY";

}


void gazebo::PongBotQ_plugin::SensorSetting() {
    
//************************Imu Setting*********************************//
    //setting for IMU sensor
    this->Sensor = sensors::get_sensor("IMU");
    this->IMU = std::dynamic_pointer_cast<sensors::ImuSensor>(Sensor);

    //************************FT Transformation Setting*********************************//
    //Rotation Matrix
    ROT_FORWARD_TIP << cos(PI / 4), 0, sin(PI / 4), 0, 1, 0, -sin(PI / 4), 0, cos(PI / 4);
    ROT_REAR_TIP << cos(-PI / 4), 0, sin(-PI / 4), 0, 1, 0, -sin(-PI / 4), 0, cos(-PI / 4);

}


void gazebo::PongBotQ_plugin::RBDLSetting(){
    
    rbdl_check_api_version(RBDL_API_VERSION);

    int version_test;
    version_test = rbdl_get_api_version();
    printf("rbdl api version = %d\n", version_test);

    Addons::URDFReadFromFile("/root/.gazebo/models/PONGBOT_Q_V2/urdf/PONGBOT_Q_V2.urdf", pongbot_q_model, true, false);
    //Addons::URDFReadFromFile("/home/hyunseok/.gazebo/models/PONGBOT_Q_V2/urdf/PONGBOT_Q_V2.urdf", pongbot_q_model, true, true);
    PongBotQ.setRobotModel(pongbot_q_model);
    
}

void gazebo::PongBotQ_plugin::IMUSensorRead() {

}

void gazebo::PongBotQ_plugin::FTSensorRead() {
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
}

void gazebo::PongBotQ_plugin::EncoderRead() {
//************************** Encoder ********************************//
    PongBotQ.actual_pos[0] = this->RL_HIP_JOINT->GetAngle(0).Radian();
    PongBotQ.actual_pos[1] = this->RL_THIGH_JOINT->GetAngle(0).Radian();
    PongBotQ.actual_pos[2] = this->RL_CALF_JOINT->GetAngle(0).Radian();

    PongBotQ.actual_pos[3] = this->RR_HIP_JOINT->GetAngle(0).Radian();
    PongBotQ.actual_pos[4] = this->RR_THIGH_JOINT->GetAngle(0).Radian();
    PongBotQ.actual_pos[5] = this->RR_CALF_JOINT->GetAngle(0).Radian();

    PongBotQ.actual_pos[6] = this->WAIST_JOINT->GetAngle(0).Radian();

    PongBotQ.actual_pos[7] = this->FL_HIP_JOINT->GetAngle(0).Radian();
    PongBotQ.actual_pos[8] = this->FL_THIGH_JOINT->GetAngle(0).Radian();
    PongBotQ.actual_pos[9] = this->FL_CALF_JOINT->GetAngle(0).Radian();

    PongBotQ.actual_pos[10] = this->FR_HIP_JOINT->GetAngle(0).Radian();
    PongBotQ.actual_pos[11] = this->FR_THIGH_JOINT->GetAngle(0).Radian();
    PongBotQ.actual_pos[12] = this->FR_CALF_JOINT->GetAngle(0).Radian();

    //* calculating errors
    // Angle & Angular velocity of leg
    for (int i = 0; i < 13; i++) {
        PongBotQ.actual_vel[i] = (PongBotQ.actual_pos[i] - PongBotQ.pre_actual_pos[i]) / PongBotQ.dt;
        PongBotQ.pre_actual_pos[i] = PongBotQ.actual_pos[i];
    }
}


void gazebo::PongBotQ_plugin::jointController() {

    //***************************Set Torque********************************//

    //* Applying torques
    
    this->RL_HIP_JOINT->SetForce(1, PongBotQ.joint[0].torque);//PongBotQ.target_tor[0]);
    this->RL_THIGH_JOINT->SetForce(1, PongBotQ.joint[1].torque);//PongBotQ.target_tor[1]);
    this->RL_CALF_JOINT->SetForce(1, PongBotQ.joint[2].torque);//PongBotQ.target_tor[2]);

    this->RR_HIP_JOINT->SetForce(1, PongBotQ.joint[3].torque);//PongBotQ.target_tor[3]);
    this->RR_THIGH_JOINT->SetForce(1, PongBotQ.joint[4].torque);//PongBotQ.target_tor[4]);
    this->RR_CALF_JOINT->SetForce(1, PongBotQ.joint[5].torque);//PongBotQ.target_tor[5]);

    this->WAIST_JOINT->SetForce(1, PongBotQ.joint[6].torque);//PongBotQ.target_tor[6]);

    this->FL_HIP_JOINT->SetForce(1, PongBotQ.joint[7].torque);//PongBotQ.target_tor[7]);
    this->FL_THIGH_JOINT->SetForce(1, PongBotQ.joint[8].torque);//PongBotQ.target_tor[8]);
    this->FL_CALF_JOINT->SetForce(1, PongBotQ.joint[9].torque);//PongBotQ.target_tor[9]);

    this->FR_HIP_JOINT->SetForce(1, PongBotQ.joint[10].torque);//PongBotQ.target_tor[10]);
    this->FR_THIGH_JOINT->SetForce(1, PongBotQ.joint[11].torque);//PongBotQ.target_tor[11]);
    this->FR_CALF_JOINT->SetForce(1, PongBotQ.joint[12].torque);//PongBotQ.target_tor[12]);

}


void gazebo::PongBotQ_plugin::ROSMsgPublish() {
    
     //********************* DH : Data plot ***************************//

    TmpData[0] = PongBotQ.target_EP[0];
    TmpData[1] = PongBotQ.target_EP[1];
    TmpData[2] = PongBotQ.target_EP[2];
    TmpData[3] = PongBotQ.target_EP[3];
    TmpData[4] = PongBotQ.target_EP[4];
    TmpData[5] = PongBotQ.target_EP[5];
    TmpData[6] = PongBotQ.target_EP[6];
    TmpData[7] = PongBotQ.target_EP[7];
    TmpData[8] = PongBotQ.target_EP[8];
    TmpData[9] = PongBotQ.target_EP[9];
    TmpData[10] = PongBotQ.target_EP[10];
    TmpData[11] = PongBotQ.target_EP[11];

    TmpData[12] = PongBotQ.target_EP_vel[2];
    TmpData[13] = PongBotQ.target_EP_vel[5];
    TmpData[14] = PongBotQ.target_EP_vel[8];
    TmpData[15] = PongBotQ.target_EP_vel[11];

    TmpData[16] = 0;//PongBotQ.FORWARD_PHASE * (-0.1);

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

    m_EP_err.data[0] = PongBotQ.EP_err[0];
    m_EP_err.data[1] = PongBotQ.EP_err[1];
    m_EP_err.data[2] = PongBotQ.EP_err[2];
    m_EP_err.data[3] = PongBotQ.EP_err[3];
    m_EP_err.data[4] = PongBotQ.EP_err[4];
    m_EP_err.data[5] = PongBotQ.EP_err[5];
    m_EP_err.data[6] = PongBotQ.EP_err[6];
    m_EP_err.data[7] = PongBotQ.EP_err[7];
    m_EP_err.data[8] = PongBotQ.EP_err[8];
    m_EP_err.data[9] = PongBotQ.EP_err[9];
    m_EP_err.data[10] = PongBotQ.EP_err[10];
    m_EP_err.data[11] = PongBotQ.EP_err[11];
    
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

    m_joint_states.position[0] = PongBotQ.actual_pos[0]; //RL_HIP
    m_joint_states.position[1] = PongBotQ.actual_pos[1]; //RL_THIGH
    m_joint_states.position[2] = PongBotQ.actual_pos[2]; //RL_CALF
    m_joint_states.position[3] = PongBotQ.actual_pos[3]; //RR_HIP
    m_joint_states.position[4] = PongBotQ.actual_pos[4]; //RR_THIGH
    m_joint_states.position[5] = PongBotQ.actual_pos[5]; //RR_CALF
    m_joint_states.position[6] = PongBotQ.actual_pos[6]; //WAIST
    m_joint_states.position[7] = PongBotQ.actual_pos[7]; //FL_HIP
    m_joint_states.position[8] = PongBotQ.actual_pos[8]; //FL_THIGH 
    m_joint_states.position[9] = PongBotQ.actual_pos[9]; //FL_CALF
    m_joint_states.position[10] = PongBotQ.actual_pos[10]; //FR_HIP
    m_joint_states.position[11] = PongBotQ.actual_pos[11]; //FR_THIGH 
    m_joint_states.position[12] = PongBotQ.actual_pos[12]; //FR_CALF

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
    
    P_EP_err.publish(m_EP_err);

    for (unsigned int i = 0; i < 30; ++i) {
        ros_msg2.data[i] = TmpData[i];
    }
    
    ros_pub2.publish(ros_msg2);
   
//    ZmpDataPublish();
//    SimtimePublish();
//    RvizJointStatesPublish();
//    FTSensorPublish();
//    IMUAngularDataPublish();
//    ObserverDataPublish();
//    JointAngleErrPublish();
}