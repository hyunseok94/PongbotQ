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
#include <sensor_msgs/Joy.h>
// by BKCho
#include "CRobot.h" 

// DH Ahn
//#include <conio.h>
//#include <string>
#include <linux/input.h>

using namespace std;

using Eigen::MatrixXd;
using Eigen::VectorXd;
using Eigen::Vector3d;

Model* pongbot_q_model = new Model();

//#define KEY_UP 72
//#define KEY_DOWN 80
//#define KEY_LEFT 75
//#define KEY_RIGHT 77
//#define KEY_X 120

//struct input_event event;

#define SAVE_LENGTH 160
#define SAVE_COUNT  60000 //60000
//#define SAVE_COUNT  180000 //60000
#define NO_OF_JOINTS 13

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

        physics::JointPtr RL_HR_JOINT;
        physics::JointPtr RL_HP_JOINT;
        physics::JointPtr RL_KN_JOINT;
        physics::JointPtr RL_TIP_JOINT;
        physics::JointPtr RR_HR_JOINT;
        physics::JointPtr RR_HP_JOINT;
        physics::JointPtr RR_KN_JOINT;
        physics::JointPtr RR_TIP_JOINT;
        physics::JointPtr WAIST_JOINT;
        physics::JointPtr FL_HR_JOINT;
        physics::JointPtr FL_HP_JOINT;
        physics::JointPtr FL_KN_JOINT;
        physics::JointPtr FL_TIP_JOINT;
        physics::JointPtr FR_HR_JOINT;
        physics::JointPtr FR_HP_JOINT;
        physics::JointPtr FR_KN_JOINT;
        physics::JointPtr FR_TIP_JOINT;

        physics::ModelPtr model;
        //        physics::LinkPtr REAR_BODY;

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
        double time2 = 0;

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
        double Base_height=0.0;
        gazebo::math::Pose Base_pose;
        
        gazebo::math::Pose RL_TIP_pose;
        gazebo::math::Pose RR_TIP_pose;
        gazebo::math::Pose FL_TIP_pose;
        gazebo::math::Pose FR_TIP_pose;
        
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
        //        double TmpData1[30] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
        //        double TmpData2[30] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
        ros::Publisher ros_pub1;
        ros::Publisher ros_pub2;
        std_msgs::Float64MultiArray ros_msg1;
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
        unsigned int cnt_measure=0;
        common::Time current_time;

        //        ros::Subscriber S_mode;

        //ROS Rqt
        //        ros::Subscriber S_ROSMODE;

        ros::Subscriber server_sub1;
        ros::Subscriber server_sub2;
        ros::Subscriber server_sub3;
        ros::Subscriber server_sub4;
        ros::Subscriber server_sub5;
        ros::Subscriber server_sub6;
       

        bool moving_flag = false;
        //ROS MODE
        int ROSMode_Flag = 0;

        VectorXd Err_EP = VectorXd::Zero(12); //added by HSKIM
        VectorXd Err_pos = VectorXd::Zero(13); //added by HSKIM

    public:
        //For model load

        void Load(physics::ModelPtr _model, sdf::ElementPtr /*_sdf*/);
        void UpdateAlgorithm(void);
        // Callback function
        void PongBot_Q_ROSmode(const std_msgs::Int32Ptr &msg);
        //        void Callback(const std_msgs::Int32Ptr &msg);
        //        void Callback2(const std_msgs::Float64Ptr &msg);
        //        void Callback3(const std_msgs::Int32Ptr &msg);
        //        void Callback4(const std_msgs::Int32Ptr &msg);
        //        void Callback5(const std_msgs::Int32Ptr &msg);
        void Callback6(const sensor_msgs::Joy::ConstPtr &msg);
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
        void ROSMsgPublish1();
        void ROSMsgPublish2();

        void dh_save(void);
    };
    GZ_REGISTER_MODEL_PLUGIN(PongBotQ_plugin);
}

void gazebo::PongBotQ_plugin::Load(physics::ModelPtr _model, sdf::ElementPtr /*_sdf*/)
{
    printf("============= [Load] =============\n");

    this->model = _model;

    //    this->UPPER_BODY = this->model->GetLink("BODY");

    printf("[1]\n");
    RBDLSetting();
    printf("[2]\n");
    GetLinks();
    printf("[3]\n");
    GetJoints();
    printf("[4]\n");
    InitROSPubSetting();
    printf("[5]\n");
    InitRvizSetting();
    printf("[6]\n");
    SensorSetting();
    printf("[7]\n");

    PongBotQ.ControlMode = CTRLMODE_HOME_POS;

    //************************Time Setting*********************************//
    this->last_update_time = this->model->GetWorld()->GetSimTime();
    this->update_connection = event::Events::ConnectWorldUpdateBegin(boost::bind(&PongBotQ_plugin::UpdateAlgorithm, this));
    std::cout<<"Test Start"<<std::endl;
    PongBotQ.set_osqp_HS();
    std::cout<<"Test End"<<std::endl;
}

void gazebo::PongBotQ_plugin::UpdateAlgorithm(void)
{ //* Writing realtime code here!!
    //********************* Base Pose for rviz *****************************//

    pose = this->model->GetWorldPose();

    //************************** Time ********************************//
    current_time = this->model->GetWorld()->GetSimTime();
    dt = current_time.Double() - this->last_update_time.Double();
    time = time + dt;
    time2 = time2 + 0.001;

    //* get com pos
    
    static double sim_com_pos[3];
    static double sim_com_vel[3];
    static double sim_pre_com_pos[3];

    sim_com_pos[0] = this->REAR_BODY->GetWorldPose().pos.x;
    sim_com_pos[1] = this->REAR_BODY->GetWorldPose().pos.y;
    sim_com_pos[2] = this->REAR_BODY->GetWorldPose().pos.z;

    sim_com_vel[0] = (sim_com_pos[0] - sim_pre_com_pos[0]) / PongBotQ.dt;
    sim_com_vel[1] = (sim_com_pos[1] - sim_pre_com_pos[1]) / PongBotQ.dt;
    sim_com_vel[2] = (sim_com_pos[2] - sim_pre_com_pos[2]) / PongBotQ.dt;

    PongBotQ.tmp_data2[32] = sim_com_vel[0];
    PongBotQ.tmp_data2[33] = sim_com_vel[1];
    PongBotQ.tmp_data2[34] = sim_com_vel[2];
    
    sim_pre_com_pos[0] = sim_com_pos[0];
    sim_pre_com_pos[1] = sim_com_pos[1];
    sim_pre_com_pos[2] = sim_com_pos[2];
    

    //* Read Sensors
    IMUSensorRead();
//    FTSensorRead();
    EncoderRead();


    //    printf("PongBotQ.ControlMode = %d\n",PongBotQ.ControlMode);

    //* ControlMode
    switch (PongBotQ.ControlMode) {
    case CTRLMODE_NONE:
        //          cout << "============= [CTRLMODE_NONE] ==========" << endl;
        //          PongBotQ.CommandFlag = TORQUE_OFF;
        //          PongBotQ.CommandFlag = NO_ACT;
        break;


    case CTRLMODE_HOME_POS:
        cout << "============= [CTRLMODE_HOME_POS] ==========" << endl;
        PongBotQ.CommandFlag = GOTO_HOME_POS;
        PongBotQ.ControlMode = CTRLMODE_NONE;
        break;

    case CTRLMODE_WALK_READY:
        cout << "============= [CTRLMODE_WALK_READY] ==========" << endl;

//        PongBotQ.moving_done_flag = false;
        PongBotQ.wr_cnt = 0;
//        PongBotQ.mpc_cnt = 0;
        
        PongBotQ.com_pos = PongBotQ.init_com_pos;
        PongBotQ.com_vel << 0, 0, 0;
        PongBotQ.base_pos = PongBotQ.com_pos + PongBotQ.base_offset;
        PongBotQ.base_vel << 0, 0, 0;
        PongBotQ.base_ori << 0, 0, 0;
        PongBotQ.base_ori_dot << 0, 0, 0;
        PongBotQ.tmp_com_pos << 0,0,0;

        PongBotQ.CommandFlag = GOTO_WALK_READY_POS;
        PongBotQ.ControlMode = CTRLMODE_NONE;
        
//        PongBotQ.Robot_para_init();
                
        //        PongBotQ.target_pos = PongBotQ.actual_pos;
        break;

    case CTRLMODE_TROT_WALKING:
        cout << "============= [CTRLMODE_TROT_WALKING] ==========" << endl;

        if (PongBotQ.walk_ready_moving_done_flag == true) {
            PongBotQ.tw_cnt = 0;
//            PongBotQ.mpc_cnt = 0;
            PongBotQ.CommandFlag = NOMAL_TROT_WALKING;
            PongBotQ.move_cnt = 0;
            PongBotQ.moving_done_flag = false;
            PongBotQ.Robot_para_init();
        }
        else {
            cout << " ======== not yet walk ready ======== " << endl;
        }
        PongBotQ.ControlMode = CTRLMODE_NONE;

        break;


    case CTRLMODE_FLYING_TROT:
        cout << "============= [CTRLMODE_FLYING_TROT] ==========" << endl;

        if (PongBotQ.walk_ready_moving_done_flag == true) {
            PongBotQ.ft_cnt = 0;
            PongBotQ.CommandFlag = FLYING_TROT_RUNNING;
            PongBotQ.move_cnt = 0;
            PongBotQ.moving_done_flag = false;
            PongBotQ.Robot_para_init();
        }
        else {
            cout << " ======== not yet walk ready ======== " << endl;
        }
        PongBotQ.ControlMode = CTRLMODE_NONE;

        break;

    case CTRLMODE_PRONK_JUMP:
        cout << "============= [CTRLMODE_PRONK_JUMP] ==========" << endl;

        if (PongBotQ.walk_ready_moving_done_flag == true) {
            PongBotQ.pr_cnt = 0;
            //            PongBotQ.moving_cnt = 0;
            PongBotQ.JUMP_PHASE = 0;
            PongBotQ.jump_num = 0;
            PongBotQ.move_cnt = 0;
            PongBotQ.moving_done_flag = false;

            PongBotQ.CommandFlag = PRONK_JUMP;
            PongBotQ.Robot_para_init();
        }
        else {
            cout << " ======== not yet walk ready ======== " << endl;
        }
        PongBotQ.ControlMode = CTRLMODE_NONE;
        break;

    case CTRLMODE_TEST:
        cout << "============= [CTRLMODE_TEST] ==========" << endl;

        if (PongBotQ.walk_ready_moving_done_flag == true) {
            PongBotQ.test_cnt = 0;
            PongBotQ.move_cnt = 0;
            PongBotQ.moving_done_flag = false;

            PongBotQ.CommandFlag = TEST_FLAG;
            PongBotQ.Robot_para_init();
        }
        else {
            cout << " ======== not yet walk ready ======== " << endl;
        }

        PongBotQ.ControlMode = CTRLMODE_NONE;

        break;

    case CTRLMODE_TORQUE_OFF:
        cout << "============= [CTRLMODE_TORQUE_OFF] ==========" << endl;

        if (PongBotQ.walk_ready_moving_done_flag == true) {
            //            PongBotQ.test_cnt = 0;

            PongBotQ.CommandFlag = TORQUE_OFF;
        }
        else {
            cout << " ======== not yet walk ready ======== " << endl;
        }

        PongBotQ.ControlMode = CTRLMODE_NONE;

        break;

    case CTRLMODE_STAIR:
        cout << "============= [CTRLMODE_STAIR] ==========" << endl;

        if (PongBotQ.walk_ready_moving_done_flag == true) {
            PongBotQ.tw_cnt = 0;
            PongBotQ.move_cnt = 0;
            PongBotQ.CommandFlag = STAIR_WALKING;
            PongBotQ.moving_done_flag = false;
            PongBotQ.Robot_para_init();
        }
        else {
            cout << " ======== not yet walk ready ======== " << endl;
        }
        PongBotQ.ControlMode = CTRLMODE_NONE;

        break;

    case CTRLMODE_SLOW_WALK_HS:
            PongBotQ.cnt_HS = 0;

            cout << "============= [CTRLMODE_Walk] ==========" << endl;
            PongBotQ.now_vel_HS << 0.0, 0.0, 0.0;
            PongBotQ.tar_vel_HS << 0.0, 0.0, 0.0;

            PongBotQ.zmp_ref_x_array_HS = VectorNd::Zero(PongBotQ.preview_cnt_HS);
            PongBotQ.zmp_ref_y_array_HS = VectorNd::Zero(PongBotQ.preview_cnt_HS);

            PongBotQ.X_new_x_HS << 0.0, 0.0, 0.0;
            PongBotQ.X_new_y_HS << 0.0, 0.0, 0.0;

            PongBotQ.sum_e_x_HS = 0.0;
            PongBotQ.sum_e_y_HS = 0.0;

            PongBotQ.WalkReady_flag_HS = true;
            PongBotQ.move_stop_flag = false;
            PongBotQ.walk_stop_flag_HS = false;
            PongBotQ.com_stop_flag_HS = false;
            PongBotQ.foot_height_HS = PongBotQ.tmp_foot_height_HS;
            PongBotQ.init_Force_flag_HS = true;

            PongBotQ.target_base_ori_local_HS=VectorNd::Zero(3);
            
            PongBotQ.Contact_Info_HS << 1, 1, 1, 1;
            PongBotQ.CommandFlag = GOTO_SLOW_WALK_POS_HS;
            PongBotQ.ControlMode = CTRLMODE_NONE;

            break;
    }


    switch (PongBotQ.CommandFlag) {
    case NO_ACT:
        // No action
        break;

    case TORQUE_OFF:
        PongBotQ.Torque_off();
        break;

    case GOTO_HOME_POS:
        PongBotQ.Torque_off(); // Temp
        //        printf("[100]\n");
        PongBotQ.Init_Pos_Traj(); // Only simulation
        //        printf("[101]\n");
        break;

    case GOTO_WALK_READY_POS:

        PongBotQ.StateUpdate();
        
        if(PongBotQ.wr_cnt == 0){
            if (PongBotQ.Mode == MODE_SIMULATION) { 
                PongBotQ.set_simul_para();       
            }
            else if (PongBotQ.Mode == MODE_ACTUAL_ROBOT) {
                PongBotQ.set_act_robot_para();
            }
            
            PongBotQ.Kp_q = PongBotQ.tar_Kp_q_low;
            std::cout<<"kp_q1="<<PongBotQ.Kp_q<<std::endl;
            std::cout<<"kp_q1-2="<<PongBotQ.tar_Kp_q_low<<std::endl;
            
            PongBotQ.Kd_q = PongBotQ.tar_Kd_q_low;
            PongBotQ.Kp_t = PongBotQ.tar_Kp_t_low; 
            PongBotQ.Kd_t = PongBotQ.tar_Kd_t_low;
            PongBotQ.Kp_x = PongBotQ.tar_Kp_x;
            PongBotQ.Kd_x = PongBotQ.tar_Kd_x;
            PongBotQ.Kp_w = PongBotQ.tar_Kp_w;
            PongBotQ.Kd_w = PongBotQ.tar_Kd_w;
        }
        
        PongBotQ.WalkReady_Pos_Traj();
        std::cout<<"kp_q2="<<PongBotQ.Kp_q<<std::endl;
        std::cout<<"-------------"<<std::endl;
        //        PongBotQ.Fc << 0,0,0,0,0,0,0, 0,0,110, 0,0,110, 0,0,110, 0,0,110;
        
        if(PongBotQ.WH_Mode == QP_CON){
            PongBotQ.QP_process();
        }
//        else{
//            if(PongBotQ.mpc_cnt == PongBotQ.Ts_cnt){
//                PongBotQ.MPC_process();
//
//                PongBotQ.mpc_cnt = 0;
//            }
//            PongBotQ.mpc_cnt++;
//        }
        
        PongBotQ.ComputeTorqueControl();
        break;

    case NOMAL_TROT_WALKING:
        //        printf("===========================================\n");        

//        cout << "PongBotQ.move_cnt = " << PongBotQ.move_cnt << endl;
        PongBotQ.StateUpdate();
        if(PongBotQ.move_cnt == 0){
            
            if (PongBotQ.Mode == MODE_SIMULATION) { 
                PongBotQ.set_simul_para();       
            }
            else if (PongBotQ.Mode == MODE_ACTUAL_ROBOT) {
                PongBotQ.set_act_robot_para();
            }
            
            PongBotQ.init_Kp_q = PongBotQ.Kp_q;
            PongBotQ.init_Kd_q = PongBotQ.Kd_q;
            PongBotQ.init_Kp_t = PongBotQ.Kp_t; 
            PongBotQ.init_Kd_t = PongBotQ.Kd_t;
            PongBotQ.init_Kp_x = PongBotQ.Kp_x;
            PongBotQ.init_Kd_x = PongBotQ.Kd_x;
            PongBotQ.init_Kp_w = PongBotQ.Kp_w;
            PongBotQ.init_Kd_w = PongBotQ.Kd_w;
            
            PongBotQ.move_cnt++;
            
        }
        else if(PongBotQ.move_cnt < 1000){
            PongBotQ.tmp_weight = 1/2.0*(1-cos(PI2/(2)*(double)(PongBotQ.move_cnt)*PongBotQ.dt));
            PongBotQ.Kp_q = PongBotQ.init_Kp_q + (PongBotQ.tar_Kp_q_low - PongBotQ.init_Kp_q)*PongBotQ.tmp_weight;
            PongBotQ.Kd_q = PongBotQ.init_Kd_q + (PongBotQ.tar_Kd_q_low - PongBotQ.init_Kd_q)*PongBotQ.tmp_weight;
            PongBotQ.Kp_t = PongBotQ.init_Kp_t + (PongBotQ.tar_Kp_t_low - PongBotQ.init_Kp_t)*PongBotQ.tmp_weight;
            PongBotQ.Kd_t = PongBotQ.init_Kd_t + (PongBotQ.tar_Kd_t_low - PongBotQ.init_Kd_t)*PongBotQ.tmp_weight;
            PongBotQ.Kp_x = PongBotQ.init_Kp_x + (PongBotQ.tar_Kp_x - PongBotQ.init_Kp_x)*PongBotQ.tmp_weight;
            PongBotQ.Kd_x = PongBotQ.init_Kd_x + (PongBotQ.tar_Kd_x - PongBotQ.init_Kd_x)*PongBotQ.tmp_weight;
            PongBotQ.Kp_w = PongBotQ.init_Kp_w + (PongBotQ.tar_Kp_w - PongBotQ.init_Kp_w)*PongBotQ.tmp_weight;
            PongBotQ.Kd_w = PongBotQ.init_Kd_w + (PongBotQ.tar_Kd_w - PongBotQ.init_Kd_w)*PongBotQ.tmp_weight;
            
            PongBotQ.move_cnt++;
            
            PongBotQ.Robot_para_init();
        }
        else{
            PongBotQ.Trot_Walking4();
        }
//        PongBotQ.Trot_Walking4();
        
        if(PongBotQ.WH_Mode == QP_CON){
            PongBotQ.QP_process();
        }
//        else{
//            if(PongBotQ.mpc_cnt == PongBotQ.Ts_cnt){
//                PongBotQ.MPC_process();
//
//                PongBotQ.mpc_cnt = 0;
//            }
//            PongBotQ.mpc_cnt++;
//        }
        
//        if(PongBotQ.mpc_cnt == PongBotQ.Ts_cnt){
//            PongBotQ.MPC_process();
//            
//            PongBotQ.mpc_cnt = 0;
//        }
//        PongBotQ.mpc_cnt++;
//        PongBotQ.MPC_process();
//        PongBotQ.QP_process();
//        PongBotQ.Get_Opt_F();
        PongBotQ.ComputeTorqueControl();
        break;

    case FLYING_TROT_RUNNING:
        //        printf("===========================================\n");
        PongBotQ.StateUpdate();
        
        if(PongBotQ.move_cnt == 0){
//            PongBotQ.moving_done_flag = false;
            
            if (PongBotQ.Mode == MODE_SIMULATION) { 
                PongBotQ.set_simul_para();       
            }
            else if (PongBotQ.Mode == MODE_ACTUAL_ROBOT) {
                PongBotQ.set_act_robot_para();
            }
            
            PongBotQ.init_Kp_q = PongBotQ.Kp_q;
            PongBotQ.init_Kd_q = PongBotQ.Kd_q;
            PongBotQ.init_Kp_t = PongBotQ.Kp_t; 
            PongBotQ.init_Kd_t = PongBotQ.Kd_t;
            PongBotQ.init_Kp_x = PongBotQ.Kp_x;
            PongBotQ.init_Kd_x = PongBotQ.Kd_x;
            PongBotQ.init_Kp_w = PongBotQ.Kp_w;
            PongBotQ.init_Kd_w = PongBotQ.Kd_w;
            
            PongBotQ.move_cnt++;
            
        }
        else if(PongBotQ.move_cnt < 1000){
            PongBotQ.tmp_weight = 1/2.0*(1-cos(PI2/(2)*(double)(PongBotQ.move_cnt)*PongBotQ.dt));
            PongBotQ.Kp_q = PongBotQ.init_Kp_q + (PongBotQ.tar_Kp_q_low - PongBotQ.init_Kp_q)*PongBotQ.tmp_weight;
            PongBotQ.Kd_q = PongBotQ.init_Kd_q + (PongBotQ.tar_Kd_q_low - PongBotQ.init_Kd_q)*PongBotQ.tmp_weight;
            PongBotQ.Kp_t = PongBotQ.init_Kp_t + (PongBotQ.tar_Kp_t_low - PongBotQ.init_Kp_t)*PongBotQ.tmp_weight;
            PongBotQ.Kd_t = PongBotQ.init_Kd_t + (PongBotQ.tar_Kd_t_low - PongBotQ.init_Kd_t)*PongBotQ.tmp_weight;
            PongBotQ.Kp_x = PongBotQ.init_Kp_x + (PongBotQ.tar_Kp_x - PongBotQ.init_Kp_x)*PongBotQ.tmp_weight;
            PongBotQ.Kd_x = PongBotQ.init_Kd_x + (PongBotQ.tar_Kd_x - PongBotQ.init_Kd_x)*PongBotQ.tmp_weight;
            PongBotQ.Kp_w = PongBotQ.init_Kp_w + (PongBotQ.tar_Kp_w - PongBotQ.init_Kp_w)*PongBotQ.tmp_weight;
            PongBotQ.Kd_w = PongBotQ.init_Kd_w + (PongBotQ.tar_Kd_w - PongBotQ.init_Kd_w)*PongBotQ.tmp_weight;
            
            PongBotQ.move_cnt++;
        }
        else{
            PongBotQ.Flying_Trot_Running3();

            
//            PongBotQ.MPC_process();
//        PongBotQ.QP_process();
            
//            if (PongBotQ.ft_phase != 2 && PongBotQ.ft_phase != 4 && PongBotQ.ft_phase != 6) {
//                PongBotQ.Get_Opt_F();
//            }
//            else {
//                PongBotQ.Fc = VectorXd::Zero(19);
//                PongBotQ.Fc[9] = 0;
//                PongBotQ.Fc[12] = 0;
//                PongBotQ.Fc[15] = 0;
//                PongBotQ.Fc[18] = 0;
//            }

            //        PongBotQ.Get_Opt_F();

//            if (PongBotQ.ft_phase == 2 || PongBotQ.ft_phase == 4 || PongBotQ.ft_phase == 6) {
//                PongBotQ.Fc = VectorNd::Zero(19);
//                PongBotQ.Fc[9] = 0;
//                PongBotQ.Fc[12] = 0;
//                PongBotQ.Fc[15] = 0;
//                PongBotQ.Fc[18] = 0;
//            }
        }
        
        if(PongBotQ.WH_Mode == QP_CON){
            PongBotQ.QP_process();
        }
//        else{
//            if(PongBotQ.mpc_cnt == PongBotQ.Ts_cnt){
//                PongBotQ.MPC_process();
//
//                PongBotQ.mpc_cnt = 0;
//            }
//            PongBotQ.mpc_cnt++;
//        }

        PongBotQ.ComputeTorqueControl();
        break;

    case PRONK_JUMP:
        //        printf("===========================================\n");
        //        PongBotQ.Pronk_Jump();
        PongBotQ.StateUpdate();
        if(PongBotQ.move_cnt == 0){
//            PongBotQ.moving_done_flag = false;
            
            if (PongBotQ.Mode == MODE_SIMULATION) { 
                PongBotQ.set_simul_para();       
            }
            else if (PongBotQ.Mode == MODE_ACTUAL_ROBOT) {
                PongBotQ.set_act_robot_para();
            }
            
            PongBotQ.init_Kp_q = PongBotQ.Kp_q;
            PongBotQ.init_Kd_q = PongBotQ.Kd_q;
            PongBotQ.init_Kp_t = PongBotQ.Kp_t; 
            PongBotQ.init_Kd_t = PongBotQ.Kd_t;
            PongBotQ.init_Kp_x = PongBotQ.Kp_x;
            PongBotQ.init_Kd_x = PongBotQ.Kd_x;
            PongBotQ.init_Kp_w = PongBotQ.Kp_w;
            PongBotQ.init_Kd_w = PongBotQ.Kd_w;
            
            PongBotQ.move_cnt++;
            
        }
        else if(PongBotQ.move_cnt < 1000){
            PongBotQ.tmp_weight = 1/2.0*(1-cos(PI2/(2)*(double)(PongBotQ.move_cnt)*PongBotQ.dt));
            PongBotQ.Kp_q = PongBotQ.init_Kp_q + (PongBotQ.tar_Kp_q_low - PongBotQ.init_Kp_q)*PongBotQ.tmp_weight;
            PongBotQ.Kd_q = PongBotQ.init_Kd_q + (PongBotQ.tar_Kd_q_low - PongBotQ.init_Kd_q)*PongBotQ.tmp_weight;
            PongBotQ.Kp_t = PongBotQ.init_Kp_t + (PongBotQ.tar_Kp_t_low - PongBotQ.init_Kp_t)*PongBotQ.tmp_weight;
            PongBotQ.Kd_t = PongBotQ.init_Kd_t + (PongBotQ.tar_Kd_t_low - PongBotQ.init_Kd_t)*PongBotQ.tmp_weight;
            PongBotQ.Kp_x = PongBotQ.init_Kp_x + (PongBotQ.tar_Kp_x - PongBotQ.init_Kp_x)*PongBotQ.tmp_weight;
            PongBotQ.Kd_x = PongBotQ.init_Kd_x + (PongBotQ.tar_Kd_x - PongBotQ.init_Kd_x)*PongBotQ.tmp_weight;
            PongBotQ.Kp_w = PongBotQ.init_Kp_w + (PongBotQ.tar_Kp_w - PongBotQ.init_Kp_w)*PongBotQ.tmp_weight;
            PongBotQ.Kd_w = PongBotQ.init_Kd_w + (PongBotQ.tar_Kd_w - PongBotQ.init_Kd_w)*PongBotQ.tmp_weight;
            
            PongBotQ.move_cnt++;
        }
        else{
            PongBotQ.Pronk_Jump();
        }
        
        if(PongBotQ.WH_Mode == QP_CON){
            PongBotQ.QP_process();
        }
//        else{
//            if(PongBotQ.mpc_cnt == PongBotQ.Ts_cnt){
//                PongBotQ.MPC_process();
//
//                PongBotQ.mpc_cnt = 0;
//            }
//            PongBotQ.mpc_cnt++;
//        }
        
//        PongBotQ.MPC_process();
//        PongBotQ.QP_process();
//        PongBotQ.Get_Opt_F();

        PongBotQ.ComputeTorqueControl();
        break;

    case TEST_FLAG:
        //        printf("===========================================\n");
        
        PongBotQ.StateUpdate();
        
        if(PongBotQ.move_cnt == 0){
//            PongBotQ.moving_done_flag = false;
            
            if (PongBotQ.Mode == MODE_SIMULATION) { 
                PongBotQ.set_simul_para();       
            }
            else if (PongBotQ.Mode == MODE_ACTUAL_ROBOT) {
                PongBotQ.set_act_robot_para();
            }
            
            PongBotQ.init_Kp_q = PongBotQ.Kp_q;
            PongBotQ.init_Kd_q = PongBotQ.Kd_q;
            PongBotQ.init_Kp_t = PongBotQ.Kp_t; 
            PongBotQ.init_Kd_t = PongBotQ.Kd_t;
            PongBotQ.init_Kp_x = PongBotQ.Kp_x;
            PongBotQ.init_Kd_x = PongBotQ.Kd_x;
            PongBotQ.init_Kp_w = PongBotQ.Kp_w;
            PongBotQ.init_Kd_w = PongBotQ.Kd_w;
            
            PongBotQ.move_cnt++;
            
//            PongBotQ.Robot_para_init();
        }
        else if(PongBotQ.move_cnt < 1000){
            PongBotQ.tmp_weight = 1/2.0*(1-cos(PI2/(2)*(double)(PongBotQ.move_cnt)*PongBotQ.dt));
            PongBotQ.Kp_q = PongBotQ.init_Kp_q + (PongBotQ.tar_Kp_q_low - PongBotQ.init_Kp_q)*PongBotQ.tmp_weight;
            PongBotQ.Kd_q = PongBotQ.init_Kd_q + (PongBotQ.tar_Kd_q_low - PongBotQ.init_Kd_q)*PongBotQ.tmp_weight;
            PongBotQ.Kp_t = PongBotQ.init_Kp_t + (PongBotQ.tar_Kp_t_low - PongBotQ.init_Kp_t)*PongBotQ.tmp_weight;
            PongBotQ.Kd_t = PongBotQ.init_Kd_t + (PongBotQ.tar_Kd_t_low - PongBotQ.init_Kd_t)*PongBotQ.tmp_weight;
            PongBotQ.Kp_x = PongBotQ.init_Kp_x + (PongBotQ.tar_Kp_x - PongBotQ.init_Kp_x)*PongBotQ.tmp_weight;
            PongBotQ.Kd_x = PongBotQ.init_Kd_x + (PongBotQ.tar_Kd_x - PongBotQ.init_Kd_x)*PongBotQ.tmp_weight;
            PongBotQ.Kp_w = PongBotQ.init_Kp_w + (PongBotQ.tar_Kp_w - PongBotQ.init_Kp_w)*PongBotQ.tmp_weight;
            PongBotQ.Kd_w = PongBotQ.init_Kd_w + (PongBotQ.tar_Kd_w - PongBotQ.init_Kd_w)*PongBotQ.tmp_weight;
            
            PongBotQ.move_cnt++;
            
//            cout << "move_cnt = " << PongBotQ.move_cnt << endl;
            if(PongBotQ.move_cnt == 1000){
                cout << "gain switching done !!" << endl;
            }
        }
        else{
            PongBotQ.Test_Function();
        }
        
        if(PongBotQ.WH_Mode == QP_CON){
            PongBotQ.QP_process();
        }
//        else{
//            if(PongBotQ.mpc_cnt == PongBotQ.Ts_cnt){
//                PongBotQ.MPC_process();
//
//                PongBotQ.mpc_cnt = 0;
//            }
//            PongBotQ.mpc_cnt++;
//        }
//        PongBotQ.MPC_process();
//        PongBotQ.QP_process();
//        PongBotQ.Get_Opt_F();
        //        PongBotQ.Fc << 0,0,0,0,0,0,0, 0,0,110, 0,0,110, 0,0,110, 0,0,110;
        PongBotQ.ComputeTorqueControl();
        break;

    case STAIR_WALKING:
            //        printf("===========================================\n");        

            PongBotQ.StateUpdate();

            if (PongBotQ.move_cnt == 0) {
                //            PongBotQ.moving_done_flag = false;

                if (PongBotQ.Mode == MODE_SIMULATION) {
                    PongBotQ.set_simul_para();
                } else if (PongBotQ.Mode == MODE_ACTUAL_ROBOT) {
                    PongBotQ.set_act_robot_para();
                }

                PongBotQ.init_Kp_q = PongBotQ.Kp_q;
                PongBotQ.init_Kd_q = PongBotQ.Kd_q;
                PongBotQ.init_Kp_t = PongBotQ.Kp_t;
                PongBotQ.init_Kd_t = PongBotQ.Kd_t;
                PongBotQ.init_Kp_x = PongBotQ.Kp_x;
                PongBotQ.init_Kd_x = PongBotQ.Kd_x;
                PongBotQ.init_Kp_w = PongBotQ.Kp_w;
                PongBotQ.init_Kd_w = PongBotQ.Kd_w;

                PongBotQ.move_cnt++;


            } else if (PongBotQ.move_cnt < 1000) {
                PongBotQ.tmp_weight = 1 / 2.0 * (1 - cos(PI2 / (2)*(double) (PongBotQ.move_cnt) * PongBotQ.dt));
                PongBotQ.Kp_q = PongBotQ.init_Kp_q + (PongBotQ.tar_Kp_q_low - PongBotQ.init_Kp_q) * PongBotQ.tmp_weight;
                PongBotQ.Kd_q = PongBotQ.init_Kd_q + (PongBotQ.tar_Kd_q_low - PongBotQ.init_Kd_q) * PongBotQ.tmp_weight;
                PongBotQ.Kp_t = PongBotQ.init_Kp_t + (PongBotQ.tar_Kp_t_low - PongBotQ.init_Kp_t) * PongBotQ.tmp_weight;
                PongBotQ.Kd_t = PongBotQ.init_Kd_t + (PongBotQ.tar_Kd_t_low - PongBotQ.init_Kd_t) * PongBotQ.tmp_weight;
                PongBotQ.Kp_x = PongBotQ.init_Kp_x + (PongBotQ.tar_Kp_x - PongBotQ.init_Kp_x) * PongBotQ.tmp_weight;
                PongBotQ.Kd_x = PongBotQ.init_Kd_x + (PongBotQ.tar_Kd_x - PongBotQ.init_Kd_x) * PongBotQ.tmp_weight;
                PongBotQ.Kp_w = PongBotQ.init_Kp_w + (PongBotQ.tar_Kp_w - PongBotQ.init_Kp_w) * PongBotQ.tmp_weight;
                PongBotQ.Kd_w = PongBotQ.init_Kd_w + (PongBotQ.tar_Kd_w - PongBotQ.init_Kd_w) * PongBotQ.tmp_weight;

                PongBotQ.move_cnt++;
            } else {
                PongBotQ.Stair_Walking();
            }

            if (PongBotQ.WH_Mode == QP_CON) {
                PongBotQ.QP_process();
            }
            //        else{
            //            if(PongBotQ.mpc_cnt == PongBotQ.Ts_cnt){
            //                PongBotQ.MPC_process();
            //
            //                PongBotQ.mpc_cnt = 0;
            //            }
            //            PongBotQ.mpc_cnt++;
            //        }

            PongBotQ.ComputeTorqueControl();
            break;

        case GOTO_SLOW_WALK_POS_HS:
            PongBotQ.Kp_q[6] = 30000;
            PongBotQ.Kd_q[6] = 1000;
            PongBotQ.StateUpdate();
            PongBotQ.Global_Transform_Z_HS();
            PongBotQ.Slope_Controller3();
            if (PongBotQ.WalkReady_flag_HS == true) {
                PongBotQ.WalkReady_Pos_Traj_HS();
                
            } else {
                PongBotQ.Walking_Gait_Traj_HS();
                
                cnt_measure++;
            }
            PongBotQ.ComputeTorqueControl_HS();

            Base_pose=this->model->GetWorldPose();
            Base_height=Base_pose.pos.z;
            
            RL_TIP_pose=this->RL_TIP->GetWorldPose();
            RR_TIP_pose=this->RR_TIP->GetWorldPose();
            FL_TIP_pose=this->FL_TIP->GetWorldPose();
            FR_TIP_pose=this->FR_TIP->GetWorldPose();
            
            cout << "Base's (x,y,z)=" << Base_pose.pos.x << "," << Base_pose.pos.y << "," << Base_pose.pos.z << endl;
            cout << "RL's (x,y,z)=" << RL_TIP_pose.pos.x << "," << RL_TIP_pose.pos.y << "," << RL_TIP_pose.pos.z << endl;
            cout << "RR's (x,y,z)=" << RR_TIP_pose.pos.x << "," << RR_TIP_pose.pos.y << "," << RR_TIP_pose.pos.z << endl;
            cout << "FL's (x,y,z)=" << FL_TIP_pose.pos.x << "," << FL_TIP_pose.pos.y << "," << FL_TIP_pose.pos.z << endl;
            cout << "FR's (x,y,z)=" << FR_TIP_pose.pos.x << "," << FR_TIP_pose.pos.y << "," << FR_TIP_pose.pos.z << endl;
            cout<<"--------"<<endl;
            //Base_height=this->FRONT_BODY->GetWorldCoGPose().pos.z;
//            std::cout<<Base_height<<std::endl;
//            std::cout<<"--------"<<std::endl;
            //PongBotQ.print_HS();
            break;
            
    }

    jointController();

    ROSMsgPublish1();
    //ROSMsgPublish2();
}

//void gazebo::PongBotQ_plugin::Callback6(const sensor_msgs::Joy::ConstPtr &msg)
//{
//    const double max_x_vel = 0.8;//2.8;//0.8; //1.4;
//    const double max_y_vel = 0.3;
//    const double max_yaw_ori = 8 * D2R; // rad
//    //    const double max_x_acc = 3.0;
//    //    static double tmp_x_acc = 0.0, tmp_x_acc2 = 0.0;
//    //    static double pre_x_vel = 0, pre_x_vel2 = 0;
//    static double tmp_x_vel = 0, tmp_y_vel = 0;
//
//    if (msg->buttons[13] == true) {
//        // ========= [Walk Ready] ========== //
//        if (PongBotQ.moving_done_flag == true) {
//            PongBotQ.ControlMode = CTRLMODE_WALK_READY;
//        }
//    }
//    else if (msg->buttons[0] == true) {
//        // ========= [Trot Walk] ========== //
//        if (PongBotQ.moving_done_flag == true) {
//            PongBotQ.ControlMode = CTRLMODE_TROT_WALKING;
//            PongBotQ.move_stop_flag = false;
//        }
//    }
//    else if (msg->buttons[3] == true) {
//        // ========= [Pronk] ========== //
//        if (PongBotQ.moving_done_flag == true) {
//            PongBotQ.ControlMode = CTRLMODE_PRONK_JUMP;
//            PongBotQ.move_stop_flag = false;
//        }
//    }
//    else if (msg->buttons[5] == true) {
//        // ========= [Test] ========== //
//        //        cout << "1" << endl;
//        if (PongBotQ.moving_done_flag == true) {
//            //            cout << "2" << endl;
//            PongBotQ.ControlMode = CTRLMODE_TEST;
//            PongBotQ.move_stop_flag = false;
//        }
//    }
//    else if (msg->buttons[4] == true) {
//        // ========= [Flying Trot] ========== //
//        if (PongBotQ.moving_done_flag == true) {
//            PongBotQ.ControlMode = CTRLMODE_FLYING_TROT;
//            PongBotQ.move_stop_flag = false;
//        }
//    }
//    else if (msg->buttons[8] == true) {
//        // ========= [Flying Trot] ========== //
//        if (PongBotQ.moving_done_flag == true) {
//            PongBotQ.ControlMode = CTRLMODE_STAIR;
//            PongBotQ.move_stop_flag = false;
//        }
//    }
//     else if (msg->buttons[12] == true) {
//        if (PongBotQ.moving_done_flag == true) {
//            PongBotQ.ControlMode = CTRLMODE_SLOW_WALK_HS;
//            PongBotQ.move_stop_flag = false;
//        }
//    }
//    else {
//        PongBotQ.ControlMode = 0;
//    }
//
//    if (msg->buttons[2] == true) {
//        //        moving_flag = true;
////        PongBotQ.pre_sub_ctrl_flag = PongBotQ.move_stop_flag; // HS
//        PongBotQ.move_stop_flag = true;
//    }
//
//    if (msg->buttons[9] == true) {
//        // ========= [TORQUE OFF] ========== //
//        PongBotQ.ControlMode = CTRLMODE_TORQUE_OFF;
//        PongBotQ.move_stop_flag = false;
//    }
//
//    static double lpf_x_vel = 0;
//    static double pre_lpf_x_vel = 0;
//
//    tmp_x_vel = (msg->axes[1]) * max_x_vel;
//    tmp_y_vel = (msg->axes[0]) * max_y_vel;
//
//    lpf_x_vel = 0.995 * pre_lpf_x_vel + (1 - 0.995) * tmp_x_vel;
//
//    pre_lpf_x_vel = lpf_x_vel;
//
//    PongBotQ.tmp_x_moving_speed = lpf_x_vel;
//
//    if (tmp_y_vel < 0.1 && tmp_y_vel>-0.1) {
//        PongBotQ.tmp_y_moving_speed = 0;
//    }
//    else {
//        PongBotQ.tmp_y_moving_speed = tmp_y_vel;
//    }
//
////    cout << "msg->axes[2] = " << msg->axes[2] << endl;
//    PongBotQ.tmp_base_ori(2) = (msg->axes[2]) * max_yaw_ori;
//    
//    PongBotQ.speed_x = (msg->axes[1]) * 0.06;
//    PongBotQ.speed_y = (msg->axes[0]) * 0.04;
//    PongBotQ.speed_yaw = (msg->axes[3]) * 4.0 * (PI / 180);
//}

void gazebo::PongBotQ_plugin::Callback6(const sensor_msgs::Joy::ConstPtr &msg) { //Joystick Wire mode
    const double max_x_vel = 0.5; //1.4;
    const double max_y_vel = 0.5;
    const double max_yaw_ori = 5 * D2R; // rad
    static double tmp_x_vel = 0, tmp_y_vel = 0;

    if (msg->buttons[8] == true) {
        // ========= [Walk Ready] ========== //
        if (PongBotQ.moving_done_flag == true) {
            PongBotQ.ControlMode = 3;
        }
    } else if (msg->buttons[3] == true) {
        // ========= [Trot Walk] ========== //
        if (PongBotQ.moving_done_flag == true) {
            PongBotQ.ControlMode = 4;
            PongBotQ.ControlMode = 7;
            PongBotQ.move_stop_flag = 0;
        }
    } else if (msg->buttons[4] == true) {
        if (PongBotQ.moving_done_flag == true) {
            PongBotQ.ControlMode = 5;
            PongBotQ.move_stop_flag = 0;
            PongBotQ.move_stop_flag = 0;
        }
    } else if (msg->buttons[5] == true) {
        if (PongBotQ.moving_done_flag == true) {
            //            cout << "2" << endl;
        }
    } else if (msg->buttons[10] == true) {
        if (PongBotQ.moving_done_flag == true) {
            PongBotQ.ControlMode = CTRLMODE_SLOW_WALK_HS;
            PongBotQ.move_stop_flag = false;
        }
    } else {
        PongBotQ.ControlMode = 0;
    }

    if (msg->buttons[1] == true) {
        PongBotQ.moving_done_flag = true;
        PongBotQ.move_stop_flag = 1;
        if (PongBotQ.pre_sub_ctrl_flag == true) {
            PongBotQ.move_stop_flag = false;
        }
    }

    if (msg->buttons[9] == true) {
        PongBotQ.CommandFlag = TORQUE_OFF;
    }

    static double lpf_x_vel = 0;
    static double pre_lpf_x_vel = 0;

    tmp_x_vel = (msg->axes[1]) * max_x_vel;
    tmp_y_vel = (msg->axes[0]) * max_y_vel;

    lpf_x_vel = 0.995 * pre_lpf_x_vel + (1 - 0.995) * tmp_x_vel;

    pre_lpf_x_vel = lpf_x_vel;

    PongBotQ.tmp_x_moving_speed = lpf_x_vel;
    if (tmp_y_vel < 0.1 && tmp_y_vel>-0.1) {
        PongBotQ.tmp_y_moving_speed = 0;
    } else {
        PongBotQ.tmp_y_moving_speed = tmp_y_vel;
    }
    PongBotQ.tmp_base_ori(2) = (msg->axes[3]) * max_yaw_ori;
//    PongBotQ.speed_x = (msg->axes[1]) * 0.22;
//    PongBotQ.speed_y = (msg->axes[0]) * 0.15;
//    PongBotQ.speed_yaw = (msg->axes[3]) * 10.0 * (4.0 * PI / 180);
//    PongBotQ.speed_x = (msg->axes[1]) * 0.06;
//    PongBotQ.speed_x = (msg->axes[1]) * 0.1;
    PongBotQ.speed_x = (msg->axes[1]) * 0.05;
    PongBotQ.speed_y = (msg->axes[0]) * 0.04;
    PongBotQ.speed_yaw = (msg->axes[3]) * 4.0 * (PI / 180);
}


void gazebo::PongBotQ_plugin::Print(void) //Print function added by HSKIM
{
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
    //    std::cout << "_______________________________________________________________________________________________________________" << std::endl;

    //}
}

void gazebo::PongBotQ_plugin::GetLinks()
{
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

void gazebo::PongBotQ_plugin::GetJoints()
{
    //JOINT DEFINITION    
    this->RL_HR_JOINT = this->model->GetJoint("RL_HR_JOINT");
    this->RL_HP_JOINT = this->model->GetJoint("RL_HP_JOINT");
    this->RL_KN_JOINT = this->model->GetJoint("RL_KN_JOINT");
    this->RL_TIP_JOINT = this->model->GetJoint("RL_TIP_JOINT");

    this->RR_HR_JOINT = this->model->GetJoint("RR_HR_JOINT");
    this->RR_HP_JOINT = this->model->GetJoint("RR_HP_JOINT");
    this->RR_KN_JOINT = this->model->GetJoint("RR_KN_JOINT");
    this->RR_TIP_JOINT = this->model->GetJoint("RR_TIP_JOINT");

    this->WAIST_JOINT = this->model->GetJoint("WAIST_JOINT");

    this->FL_HR_JOINT = this->model->GetJoint("FL_HR_JOINT");
    this->FL_HP_JOINT = this->model->GetJoint("FL_HP_JOINT");
    this->FL_KN_JOINT = this->model->GetJoint("FL_KN_JOINT");
    this->FL_TIP_JOINT = this->model->GetJoint("FL_TIP_JOINT");

    this->FR_HR_JOINT = this->model->GetJoint("FR_HR_JOINT");
    this->FR_HP_JOINT = this->model->GetJoint("FR_HP_JOINT");
    this->FR_KN_JOINT = this->model->GetJoint("FR_KN_JOINT");
    this->FR_TIP_JOINT = this->model->GetJoint("FR_TIP_JOINT");

}

void gazebo::PongBotQ_plugin::InitROSPubSetting()
{
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

    // DH Publisher
    ros_pub1 = n.advertise<std_msgs::Float64MultiArray>("/tmp_data1/", 1000);
    ros_pub2 = n.advertise<std_msgs::Float64MultiArray>("/tmp_data2/", 1000);
//    ros_msg1.data.resize(80);
//    ros_msg2.data.resize(80);
    ros_msg1.data.resize(30);
    ros_msg2.data.resize(30);

    P_RL_force = n.advertise<geometry_msgs::WrenchStamped>("RL_force", 1000);
    P_RR_force = n.advertise<geometry_msgs::WrenchStamped>("RR_force", 1000);
    P_FL_force = n.advertise<geometry_msgs::WrenchStamped>("FL_force", 1000);
    P_FR_force = n.advertise<geometry_msgs::WrenchStamped>("FR_force", 1000);
    m_RL_force.header.frame_id = "RL_TIP";
    m_RR_force.header.frame_id = "RR_TIP";
    m_FL_force.header.frame_id = "FL_TIP";
    m_FR_force.header.frame_id = "FR_TIP";

    server_sub6 = n.subscribe("/joy", 1, &gazebo::PongBotQ_plugin::Callback6, this);

}

void gazebo::PongBotQ_plugin::InitRvizSetting()
{
    //**************************rviz inital setting*************************************//
    //    P_joint_states = n.advertise<sensor_msgs::JointState>("joint_states", 1);
    //    m_joint_states.name.resize(13);
    //    m_joint_states.position.resize(13);
    //    odom_trans.header.frame_id = "odom";
    //    odom_trans.child_frame_id = "REAR_BODY";

}

void gazebo::PongBotQ_plugin::SensorSetting()
{

    //************************Imu Setting*********************************//
    //setting for IMU sensor
    this->Sensor = sensors::get_sensor("IMU");
    this->IMU = std::dynamic_pointer_cast<sensors::ImuSensor>(Sensor);

    //************************FT Transformation Setting*********************************//
    //Rotation Matrix
    ROT_FORWARD_TIP << cos(PI / 4), 0, sin(PI / 4), 0, 1, 0, -sin(PI / 4), 0, cos(PI / 4);
    ROT_REAR_TIP << cos(-PI / 4), 0, sin(-PI / 4), 0, 1, 0, -sin(-PI / 4), 0, cos(-PI / 4);

}

void gazebo::PongBotQ_plugin::RBDLSetting()
{

    rbdl_check_api_version(RBDL_API_VERSION);

    int version_test;
    version_test = rbdl_get_api_version();
    printf("rbdl api version = %d\n", version_test);

    //Addons::URDFReadFromFile("/root/.gazebo/models/PONGBOT_Q_V4/urdf/PONGBOT_Q_V4.urdf", pongbot_q_model, true, false);
    Addons::URDFReadFromFile("/home/hyunseok/.gazebo/models/PONGBOT_Q_V4/urdf/PONGBOT_Q_V4.urdf", pongbot_q_model, true, false);
    //Addons::URDFReadFromFile("/home/hyunseok/.gazebo/models/PONGBOT_Q_V2/urdf/PONGBOT_Q_V2.urdf", pongbot_q_model, true, true);
    PongBotQ.setRobotModel(pongbot_q_model);

}

void gazebo::PongBotQ_plugin::IMUSensorRead()
{
    PongBotQ.IMURoll_dot = this->IMU->AngularVelocity(false)[0];
    PongBotQ.IMUPitch_dot = this->IMU->AngularVelocity(false)[1];
    PongBotQ.IMUYaw_dot = this->IMU->AngularVelocity(false)[2];

    PongBotQ.IMURoll = pose.rot.GetRoll(); //PongBotQ.IMURoll + PongBotQ.IMURoll_dot*dt;
    PongBotQ.IMUPitch = pose.rot.GetPitch(); //PongBotQ.IMUPitch + PongBotQ.IMUPitch_dot*dt;
    PongBotQ.IMUYaw = pose.rot.GetYaw(); //PongBotQ.IMUYaw + PongBotQ.IMUYaw_dot*dt;
    
//    PongBotQ.linear_acc_x =-(this->IMU->LinearAcceleration(false)[0]);
//    PongBotQ.linear_acc_y =-(this->IMU->LinearAcceleration(false)[1]);
    PongBotQ.linear_acc_x =(this->IMU->LinearAcceleration(false)[0]);
    PongBotQ.linear_acc_y =(this->IMU->LinearAcceleration(false)[1]);
    PongBotQ.linear_acc_z =(this->IMU->LinearAcceleration(false)[2]);
    
    PongBotQ.actual_base_acc_local_HS << PongBotQ.linear_acc_x, PongBotQ.linear_acc_y, PongBotQ.linear_acc_z;
//    cout << endl << "============================= " << endl;
//    cout << "[IMU] Pitch = " << PongBotQ.IMUPitch*R2D << endl;
//    cout << "[IMU] Pitch dot = " << PongBotQ.IMURoll_dot*R2D << endl;

    //    printf("angular_vel_x = %f [rad/s]\n",angular_vel_x);
}

void gazebo::PongBotQ_plugin::FTSensorRead()
{
    PongBotQ.FTsensorTransformation();

    wrench = this->RL_TIP_JOINT->GetForceTorque(0);

    force = wrench.body2Force.Ign();
    torque = wrench.body2Torque.Ign();

    RL_Force_E[0] = force.X();
    RL_Force_E[1] = force.Y();
    RL_Force_E[2] = force.Z();

    RL_Force_I = PongBotQ.RL.T_matrix*RL_Force_E;

    PongBotQ.RL.ftSensor.Fx = RL_Force_I[0];
    PongBotQ.RL.ftSensor.Fy = RL_Force_I[1]; //force.Y();
    PongBotQ.RL.ftSensor.Fz = RL_Force_I[2]; //force.Z();

    wrench = this->RR_TIP_JOINT->GetForceTorque(0);

    force = wrench.body2Force.Ign();
    torque = wrench.body2Torque.Ign();

    RR_Force_E[0] = force.X();
    RR_Force_E[1] = force.Y();
    RR_Force_E[2] = force.Z();

    RR_Force_I = PongBotQ.RR.T_matrix*RR_Force_E;

    PongBotQ.RR.ftSensor.Fx = RR_Force_I[0];
    PongBotQ.RR.ftSensor.Fy = RR_Force_I[1];
    PongBotQ.RR.ftSensor.Fz = RR_Force_I[2];

    wrench = this->FL_TIP_JOINT->GetForceTorque(0);

    force = wrench.body2Force.Ign();
    torque = wrench.body2Torque.Ign();

    FL_Force_E[0] = force.X();
    FL_Force_E[1] = force.Y();
    FL_Force_E[2] = force.Z();

    FL_Force_I = PongBotQ.FL.T_matrix*FL_Force_E;

    PongBotQ.FL.ftSensor.Fx = FL_Force_I[0];
    PongBotQ.FL.ftSensor.Fy = FL_Force_I[1];
    PongBotQ.FL.ftSensor.Fz = FL_Force_I[2];

    wrench = this->FR_TIP_JOINT->GetForceTorque(0);

    force = wrench.body2Force.Ign();
    torque = wrench.body2Torque.Ign();

    FR_Force_E[0] = force.X();
    FR_Force_E[1] = force.Y();
    FR_Force_E[2] = force.Z();

    FR_Force_I = PongBotQ.FR.T_matrix*FR_Force_E;

    PongBotQ.FR.ftSensor.Fx = FR_Force_I[0];
    PongBotQ.FR.ftSensor.Fy = FR_Force_I[1];
    PongBotQ.FR.ftSensor.Fz = FR_Force_I[2];

    //    cout << "RLz = " << RL_Force_E[2] << "RRz = " << RR_Force_E[2] << "FLz = " << FL_Force_E[2] << "FRz = " << FR_Force_E[2] << endl;

    const double thre = 10;
    const double thre_num = 4;
    static int rl_fz_cnt = 0, rr_fz_cnt = 0, fl_fz_cnt = 0, fr_fz_cnt = 0;

    if (PongBotQ.RL.ftSensor.Fz > thre) {
        if (rl_fz_cnt < thre_num) {
            rl_fz_cnt++;
        }
        else {
            PongBotQ.T_RL = true;
        }
    }
    else {
        PongBotQ.T_RL = false;
        rl_fz_cnt = 0;
    }

    if (PongBotQ.RR.ftSensor.Fz > thre) {
        if (rr_fz_cnt < thre_num) {
            rr_fz_cnt++;
        }
        else {
            PongBotQ.T_RR = true;
        }
    }
    else {
        PongBotQ.T_RR = false;
        rr_fz_cnt = 0;
    }

    if (PongBotQ.FL.ftSensor.Fz > thre) {
        if (fl_fz_cnt < thre_num) {
            fl_fz_cnt++;
        }
        else {
            PongBotQ.T_FL = true;
        }
    }
    else {
        PongBotQ.T_FL = false;
        fl_fz_cnt = 0;
    }

    if (PongBotQ.FR.ftSensor.Fz > thre) {
        if (fr_fz_cnt < thre_num) {
            fr_fz_cnt++;
        }
        else {
            PongBotQ.T_FR = true;
        }
    }
    else {
        PongBotQ.T_FR = false;
        fr_fz_cnt = 0;
    }

    //    printf("FZ_RL = %f, PongBotQ.T_RL = %d\n",PongBotQ.RL.ftSensor.Fz,PongBotQ.T_RL);    
}

void gazebo::PongBotQ_plugin::EncoderRead()
{
    //************************** Encoder ********************************//
    PongBotQ.actual_pos[0] = this->RL_HR_JOINT->GetAngle(0).Radian();
    PongBotQ.actual_pos[1] = this->RL_HP_JOINT->GetAngle(0).Radian();
    PongBotQ.actual_pos[2] = this->RL_KN_JOINT->GetAngle(0).Radian() - 0.36259;

    PongBotQ.actual_pos[3] = this->RR_HR_JOINT->GetAngle(0).Radian();
    PongBotQ.actual_pos[4] = this->RR_HP_JOINT->GetAngle(0).Radian();
    PongBotQ.actual_pos[5] = this->RR_KN_JOINT->GetAngle(0).Radian() - 0.36259;

    PongBotQ.actual_pos[6] = this->WAIST_JOINT->GetAngle(0).Radian();

    PongBotQ.actual_pos[7] = this->FL_HR_JOINT->GetAngle(0).Radian();
    PongBotQ.actual_pos[8] = this->FL_HP_JOINT->GetAngle(0).Radian();
    PongBotQ.actual_pos[9] = this->FL_KN_JOINT->GetAngle(0).Radian() - 0.36259;

    PongBotQ.actual_pos[10] = this->FR_HR_JOINT->GetAngle(0).Radian();
    PongBotQ.actual_pos[11] = this->FR_HP_JOINT->GetAngle(0).Radian();
    PongBotQ.actual_pos[12] = this->FR_KN_JOINT->GetAngle(0).Radian() - 0.36259;

//    cout << "actual_pos = " << PongBotQ.actual_pos.transpose()*R2D << endl;
    
    //* calculating errors
    // Angle & Angular velocity of leg

    static double act_vel_alpha = 0.04; //0.04;

    for (int i = 0; i < 13; i++) {
        PongBotQ.actual_vel[i] = (PongBotQ.actual_pos[i] - PongBotQ.pre_actual_pos[i]) / PongBotQ.dt;
        PongBotQ.pre_actual_pos[i] = PongBotQ.actual_pos[i];

        // filter
        PongBotQ.lpf_actual_vel[i] = (1 - act_vel_alpha) * PongBotQ.lpf_actual_vel[i] + act_vel_alpha * PongBotQ.actual_vel[i];
    }
}

void gazebo::PongBotQ_plugin::jointController()
{
    //***************************Set Torque********************************//

    PongBotQ.joint[6].torque = PongBotQ.Kp_q[6]*(PongBotQ.target_pos[6] - PongBotQ.actual_pos[6]) + PongBotQ.Kd_q[6]*(0 - PongBotQ.lpf_actual_vel[6]);

    //* Torque Limit
    for (unsigned int i = 0; i < 13; ++i) {
        if (PongBotQ.joint[i].torque >= 3000) {
            PongBotQ.joint[i].torque = 3000;
        }
        else if (PongBotQ.joint[i].torque <= -3000) {
            PongBotQ.joint[i].torque = -3000;
        }
    }

    //* Applying torques
    this->RL_HR_JOINT->SetForce(0, PongBotQ.joint[0].torque); //PongBotQ.target_tor[0]);
    this->RL_HP_JOINT->SetForce(1, PongBotQ.joint[1].torque); //PongBotQ.target_tor[1]);
    this->RL_KN_JOINT->SetForce(1, PongBotQ.joint[2].torque); //PongBotQ.target_tor[2]);

    this->RR_HR_JOINT->SetForce(0, PongBotQ.joint[3].torque); //PongBotQ.target_tor[3]);
    this->RR_HP_JOINT->SetForce(1, PongBotQ.joint[4].torque); //PongBotQ.target_tor[4]);
    this->RR_KN_JOINT->SetForce(1, PongBotQ.joint[5].torque); //PongBotQ.target_tor[5]);

    this->WAIST_JOINT->SetForce(2, PongBotQ.joint[6].torque); //PongBotQ.target_tor[6]);

    this->FL_HR_JOINT->SetForce(0, PongBotQ.joint[7].torque); //PongBotQ.target_tor[7]);
    this->FL_HP_JOINT->SetForce(1, PongBotQ.joint[8].torque); //PongBotQ.target_tor[8]);
    this->FL_KN_JOINT->SetForce(1, PongBotQ.joint[9].torque); //PongBotQ.target_tor[9]);

    this->FR_HR_JOINT->SetForce(0, PongBotQ.joint[10].torque); //PongBotQ.target_tor[10]);
    this->FR_HP_JOINT->SetForce(1, PongBotQ.joint[11].torque); //PongBotQ.target_tor[11]);
    this->FR_KN_JOINT->SetForce(1, PongBotQ.joint[12].torque); //PongBotQ.target_tor[12]);

}

void gazebo::PongBotQ_plugin::ROSMsgPublish1()
{
    //********************* DH : Data plot ***************************//

    PongBotQ.tmp_data1[0] = Base_height;
    PongBotQ.tmp_data1[1] = PongBotQ.measure_z;
    PongBotQ.tmp_data1[2] = PongBotQ.actual_base_pos_HS(2);
    PongBotQ.tmp_data1[3] = PongBotQ.base_hold_flag;
    PongBotQ.tmp_data1[4] = PongBotQ.Plane_Angle(0)*R2D;
    
    PongBotQ.tmp_data1[5] = -PongBotQ.Contact_Info_HS(0) * PongBotQ.actual_EP_local_HS(2) / cos(abs(PongBotQ.actual_base_ori_local_HS(1)));
    PongBotQ.tmp_data1[6] = -PongBotQ.Contact_Info_HS(1) * PongBotQ.actual_EP_local_HS(5) / cos(abs(PongBotQ.actual_base_ori_local_HS(1)));
    PongBotQ.tmp_data1[7] = -PongBotQ.Contact_Info_HS(2) * PongBotQ.actual_EP_local_HS(8) / cos(abs(PongBotQ.actual_base_ori_local_HS(1)));
    PongBotQ.tmp_data1[8] = -PongBotQ.Contact_Info_HS(3) * PongBotQ.actual_EP_local_HS(11) / cos(abs(PongBotQ.actual_base_ori_local_HS(1)));
    
//    PongBotQ.tmp_data1[8] = PongBotQ.Task_Control_value_HS(9);
//    PongBotQ.tmp_data1[9] = PongBotQ.Task_Control_value_HS(12);
//    PongBotQ.tmp_data1[10] = PongBotQ.Task_Control_value_HS[15];
//    PongBotQ.tmp_data1[11] = PongBotQ.Task_Control_value_HS(18);
    
    //PongBotQ.tmp_data1[8] = PongBotQ.target_plane_dist_HS;
    PongBotQ.tmp_data1[9] = PongBotQ.target_base_ori_local_HS(0)*R2D;
    PongBotQ.tmp_data1[10] = PongBotQ.target_base_ori_local_HS(1)*R2D;
    PongBotQ.tmp_data1[11] = PongBotQ.target_yaw_HS*R2D;
//
//    PongBotQ.tmp_data1[12] = PongBotQ.semi_F_QP_global_yaw(0);
//    PongBotQ.tmp_data1[13] = PongBotQ.semi_F_QP_global_yaw(3);
//    PongBotQ.tmp_data1[14] = PongBotQ.semi_F_QP_global_yaw[6];
//    PongBotQ.tmp_data1[15] = PongBotQ.semi_F_QP_global_yaw(9);

//    PongBotQ.tmp_data1[16] = PongBotQ.semi_F_QP_global_yaw[1];
//    PongBotQ.tmp_data1[17] = PongBotQ.semi_F_QP_global_yaw[4];
//    PongBotQ.tmp_data1[18] = PongBotQ.semi_F_QP_global_yaw[7];
//    PongBotQ.tmp_data1[19] = PongBotQ.semi_F_QP_global_yaw[10];
    PongBotQ.tmp_data1[12] = PongBotQ.actual_EP_HS[2];
    PongBotQ.tmp_data1[13] = PongBotQ.actual_EP_HS[5];
    PongBotQ.tmp_data1[14] = PongBotQ.actual_EP_HS[8];
    PongBotQ.tmp_data1[15] = PongBotQ.actual_EP_HS[11];
    
    PongBotQ.tmp_data1[16] = PongBotQ.target_EP_HS[2];
    PongBotQ.tmp_data1[17] = PongBotQ.target_EP_HS[5];
    PongBotQ.tmp_data1[18] = PongBotQ.target_EP_HS[8];
    PongBotQ.tmp_data1[19] = PongBotQ.target_EP_HS[11];
    
    
//    PongBotQ.tmp_data1[20] = PongBotQ.semi_F_QP_global_yaw[2];
//    PongBotQ.tmp_data1[21] = PongBotQ.semi_F_QP_global_yaw[5];
//    PongBotQ.tmp_data1[22] = PongBotQ.semi_F_QP_global_yaw[8];
//    PongBotQ.tmp_data1[23] = PongBotQ.semi_F_QP_global_yaw[11];
    PongBotQ.tmp_data1[20] = PongBotQ.actual_EP_vel_HS[2];
    PongBotQ.tmp_data1[21] = PongBotQ.actual_EP_vel_HS[5];
    PongBotQ.tmp_data1[22] = PongBotQ.actual_EP_vel_HS[8];
    PongBotQ.tmp_data1[23] = PongBotQ.actual_EP_vel_HS[11];
    
    PongBotQ.tmp_data1[24] = PongBotQ.target_EP_vel_HS(2);
    PongBotQ.tmp_data1[25] = PongBotQ.target_EP_vel_HS(5);
    PongBotQ.tmp_data1[26] = PongBotQ.target_EP_vel_HS(8);
    PongBotQ.tmp_data1[27] = PongBotQ.target_EP_vel_HS(11);
    PongBotQ.tmp_data1[28] = PongBotQ.Early_Contact_flag_HS;
    
//    for (unsigned int i = 0; i < 80; ++i) {
    for (unsigned int i = 0; i < 30; ++i) {
        ros_msg1.data[i] = PongBotQ.tmp_data1(i);
    }

    ros_pub1.publish(ros_msg1);
}

void gazebo::PongBotQ_plugin::ROSMsgPublish2()
{
    //********************* DH : Data plot ***************************//

    for (unsigned int i = 0; i < 80; ++i) {
        ros_msg2.data[i] = PongBotQ.tmp_data2[i];
    }

    ros_pub2.publish(ros_msg2);
}


