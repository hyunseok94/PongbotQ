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

#define PI      3.141592
#define D2R     PI/180
#define R2D     180/PI

using Eigen::MatrixXd;

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
    physics::LinkPtr FL_ROTATOR1;
    physics::LinkPtr FL_ROTATOR2;
    physics::LinkPtr FL_ROTATOR3;
    physics::LinkPtr FR_HIP;
    physics::LinkPtr FR_THIGH;
    physics::LinkPtr FR_ROTATOR1;
    physics::LinkPtr FR_ROTATOR2;
    physics::LinkPtr FR_ROTATOR3;
    physics::LinkPtr RL_HIP;
    physics::LinkPtr RL_THIGH;
    physics::LinkPtr RL_ROTATOR1;
    physics::LinkPtr RL_ROTATOR2;
    physics::LinkPtr RL_ROTATOR3;
    physics::LinkPtr RR_HIP;
    physics::LinkPtr RR_THIGH;
    physics::LinkPtr RR_ROTATOR1;
    physics::LinkPtr RR_ROTATOR2;
    physics::LinkPtr RR_ROTATOR3;

    physics::JointPtr WAIST_JOINT;                     
    physics::JointPtr FL_HIP_JOINT;
    physics::JointPtr FL_THIGH_JOINT;
    physics::JointPtr FL_ROTATOR1_JOINT;
    physics::JointPtr FL_ROTATOR2_JOINT;                     
    physics::JointPtr FL_ROTATOR3_JOINT;
    physics::JointPtr FR_HIP_JOINT;
    physics::JointPtr FR_THIGH_JOINT;
    physics::JointPtr FR_ROTATOR1_JOINT;
    physics::JointPtr FR_ROTATOR2_JOINT;                     
    physics::JointPtr FR_ROTATOR3_JOINT;
    physics::JointPtr RL_HIP_JOINT;
    physics::JointPtr RL_THIGH_JOINT;
    physics::JointPtr RL_ROTATOR1_JOINT;
    physics::JointPtr RL_ROTATOR2_JOINT;                     
    physics::JointPtr RL_ROTATOR3_JOINT;
    physics::JointPtr RR_HIP_JOINT;
    physics::JointPtr RR_THIGH_JOINT;
    physics::JointPtr RR_ROTATOR1_JOINT;
    physics::JointPtr RR_ROTATOR2_JOINT;                     
    physics::JointPtr RR_ROTATOR3_JOINT;
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
    double time=0;
    
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

    //setting for rqt telecommunication
    ros::NodeHandle n;
    ros::Publisher P_Times;
    ros::Publisher P_angular_velocity_x;
    ros::Publisher P_angular_velocity_y;
    ros::Publisher P_angular_velocity_z;
    std_msgs::Float64 m_Times;
    std_msgs::Float64 m_angular_velocity_x;
    std_msgs::Float64 m_angular_velocity_y;
    std_msgs::Float64 m_angular_velocity_z;
    
    // DH PARA
    double tar_deg[13] = {0,   30, -60,
                         -60,  30,   0,
                          0,  -30,  60,
                          60, -30,   0,
                          0}; 
    double angle_err[13];
    
    
    
    
    //For model load
    public: void Load(physics::ModelPtr _model, sdf::ElementPtr /*_sdf*/)
    {
        // model = link + joint +sensor
         this->model = _model;
         
         rbdl_check_api_version(RBDL_API_VERSION);
         
         int version_test;
         version_test = rbdl_get_api_version();
         printf("rbdl api version = %d\n", version_test);
         
         Model* pongbot_q_model = new Model();
         
//         if (!Addons::URDFReadFromFile ("./root/.gazebo/models/PONGBOT_Q_V1/model.urdf", pongbot_q_model, false)) {
//		std::cerr << "Error loading model ./root/.gazebo/models/PONGBOT_Q_V1/model.urdf" << std::endl;
//		abort();
//	}
        //LINK DEFINITION
         this->REAR_BODY = this->model->GetLink("REAR_BODY");
         this->FRONT_BODY = this->model->GetLink("FRONT_BODY");
         this->FL_HIP = this->model->GetLink("FL_HIP");
         this->FL_THIGH = this->model->GetLink("FL_THIGH");
         this->FL_ROTATOR1 = this->model->GetLink("FL_ROTATOR1");
         this->FL_ROTATOR2 = this->model->GetLink("FL_ROTATOR2");
         this->FL_ROTATOR3 = this->model->GetLink("FL_ROTATOR3");
         this->FR_HIP = this->model->GetLink("FR_HIP");
         this->FR_THIGH = this->model->GetLink("FR_THIGH");
         this->FR_ROTATOR1 = this->model->GetLink("FR_ROTATOR1");
         this->FR_ROTATOR2 = this->model->GetLink("FR_ROTATOR2");
         this->FR_ROTATOR3 = this->model->GetLink("FR_ROTATOR3");
         this->RL_HIP = this->model->GetLink("RL_HIP");
         this->RL_THIGH = this->model->GetLink("RL_THIGH");
         this->RL_ROTATOR1 = this->model->GetLink("RL_ROTATOR1");
         this->RL_ROTATOR2 = this->model->GetLink("RL_ROTATOR2");
         this->RL_ROTATOR3 = this->model->GetLink("RL_ROTATOR3");
         this->RR_HIP = this->model->GetLink("RR_HIP");
         this->RR_THIGH = this->model->GetLink("RR_THIGH");
         this->RR_ROTATOR1 = this->model->GetLink("RR_ROTATOR1");
         this->RR_ROTATOR2 = this->model->GetLink("RR_ROTATOR2");
         this->RR_ROTATOR3 = this->model->GetLink("RR_ROTATOR3");
         this->RR_HIP = this->model->GetLink("RR_HIP");
         this->RR_THIGH = this->model->GetLink("RR_THIGH");
         this->RR_ROTATOR1 = this->model->GetLink("RR_ROTATOR1");
         this->RR_ROTATOR2 = this->model->GetLink("RR_ROTATOR2");
         this->RR_ROTATOR3 = this->model->GetLink("RR_ROTATOR3");

        //JOINT DEFINITION
         this->WAIST_JOINT = this->model->GetJoint("WAIST_JOINT");
         this->FL_HIP_JOINT = this->model->GetJoint("FL_HIP_JOINT");
         this->FL_THIGH_JOINT = this->model->GetJoint("FL_THIGH_JOINT");
         this->FL_ROTATOR1_JOINT = this->model->GetJoint("FL_ROTATOR1_JOINT");
         this->FL_ROTATOR2_JOINT = this->model->GetJoint("FL_ROTATOR2_JOINT");
         this->FL_ROTATOR3_JOINT = this->model->GetJoint("FL_ROTATOR3_JOINT");
         this->FR_HIP_JOINT = this->model->GetJoint("FR_HIP_JOINT");
         this->FR_THIGH_JOINT = this->model->GetJoint("FR_THIGH_JOINT");
         this->FR_ROTATOR1_JOINT = this->model->GetJoint("FR_ROTATOR1_JOINT");
         this->FR_ROTATOR2_JOINT = this->model->GetJoint("FR_ROTATOR2_JOINT");
         this->FR_ROTATOR3_JOINT = this->model->GetJoint("FR_ROTATOR3_JOINT");
         this->RL_HIP_JOINT = this->model->GetJoint("RL_HIP_JOINT");
         this->RL_THIGH_JOINT = this->model->GetJoint("RL_THIGH_JOINT");
         this->RL_ROTATOR1_JOINT = this->model->GetJoint("RL_ROTATOR1_JOINT");
         this->RL_ROTATOR2_JOINT = this->model->GetJoint("RL_ROTATOR2_JOINT");
         this->RL_ROTATOR3_JOINT = this->model->GetJoint("RL_ROTATOR3_JOINT");
         this->RR_HIP_JOINT = this->model->GetJoint("RR_HIP_JOINT");
         this->RR_THIGH_JOINT = this->model->GetJoint("RR_THIGH_JOINT");
         this->RR_ROTATOR1_JOINT = this->model->GetJoint("RR_ROTATOR1_JOINT");
         this->RR_ROTATOR2_JOINT = this->model->GetJoint("RR_ROTATOR2_JOINT");
         this->RR_ROTATOR3_JOINT = this->model->GetJoint("RR_ROTATOR3_JOINT");

        //setting for getting dt
         this->last_update_time = this->model->GetWorld()->GetSimTime();
         this->update_connection = event::Events::ConnectWorldUpdateBegin(boost::bind(&PongBotQ_plugin::UpdateAlgorithm, this));

         //setting for IMU sensor
         this->Sensor = sensors::get_sensor("IMU");
         this->IMU = std::dynamic_pointer_cast<sensors::ImuSensor>(Sensor); 

        //setting for communication
         P_Times = n.advertise<std_msgs::Float64>("times",1);
         P_angular_velocity_x = n.advertise<std_msgs::Float64>("angular_velocity_x",1);
         P_angular_velocity_y = n.advertise<std_msgs::Float64>("angular_velocity_y",1);  
         P_angular_velocity_z = n.advertise<std_msgs::Float64>("angular_velocity_z",1);

         // =================== PID GAIN TUNNING ==================== //

         this->pid_FR_HR.Init(200, 0.1, 10, 200, -200, 1000, -1000);
         this->pid_FR_HP.Init(300, 0.2, 10, 200, -200, 1000, -1000);
         this->pid_FR_KN.Init(300, 0.2, 10, 200, -200, 1000, -1000);
         this->pid_FL_HR.Init(200, 0.1, 10, 200, -200, 1000, -1000);
         this->pid_FL_HP.Init(300, 0.2, 10, 200, -200, 1000, -1000);
         this->pid_FL_KN.Init(300, 0.2, 10, 200, -200, 1000, -1000);
         this->pid_RL_HR.Init(200, 0.1, 10, 200, -200, 1000, -1000);
         this->pid_RL_HP.Init(300, 0.2, 10, 200, -200, 1000, -1000);
         this->pid_RL_KN.Init(300, 0.2, 10, 200, -200, 1000, -1000);
         this->pid_RR_HR.Init(200, 0.1, 10, 200, -200, 1000, -1000);
         this->pid_RR_HP.Init(300, 0.2, 10, 200, -200, 1000, -1000);
         this->pid_RR_KN.Init(300, 0.2, 10, 200, -200, 1000, -1000);
         this->pid_WAIST.Init(200, 0.1, 10, 200, -200, 1000, -1000);


         ros::Rate loop_rate(1000);
    }
  
    void UpdateAlgorithm();
  
  };
    GZ_REGISTER_MODEL_PLUGIN(PongBotQ_plugin);
}


void gazebo::PongBotQ_plugin::UpdateAlgorithm()
{ //* Writing realtime code here!!
  
  //setting for getting dt  
  common::Time current_time = this->model->GetWorld()->GetSimTime();
  dt = current_time.Double() - this->last_update_time.Double();
  time=time+dt;

  //getting angular_velocity and linear acceleration.
  angular_velocity_x = this->IMU->AngularVelocity(false)[0];
  angular_velocity_y = this->IMU->AngularVelocity(false)[1];
  angular_velocity_z = this->IMU->AngularVelocity(false)[2];
  linear_acc_x = this->IMU->LinearAcceleration(false)[0];
  linear_acc_y = this->IMU->LinearAcceleration(false)[1];
  linear_acc_z = this->IMU->LinearAcceleration(false)[2];

 //Apply force to joint
//  this->FL_ROTATOR1_JOINT->SetForce(1,10);
  //this->FL_ROTATOR1_JOINT->SetForce(1,10);

    angle_err[0] = this->FR_HIP_JOINT->GetAngle(0).Radian() - (tar_deg[0]*D2R);
    angle_err[1] = this->FR_THIGH_JOINT->GetAngle(0).Radian() - (tar_deg[1]*D2R);
    angle_err[2] = this->FR_ROTATOR1_JOINT->GetAngle(0).Radian() - (tar_deg[2]*D2R);
    angle_err[5] = this->FL_HIP_JOINT->GetAngle(0).Radian() - (tar_deg[5]*D2R);
    angle_err[4] = this->FL_THIGH_JOINT->GetAngle(0).Radian() - (tar_deg[4]*D2R);
    angle_err[3] = this->FL_ROTATOR1_JOINT->GetAngle(0).Radian() - (tar_deg[3]*D2R);
    angle_err[6] = this->RL_HIP_JOINT->GetAngle(0).Radian() - (tar_deg[6]*D2R);
    angle_err[7] = this->RL_THIGH_JOINT->GetAngle(0).Radian() - (tar_deg[7]*D2R);
    angle_err[8] = this->RL_ROTATOR1_JOINT->GetAngle(0).Radian() - (tar_deg[8]*D2R);
    angle_err[11] = this->RR_HIP_JOINT->GetAngle(0).Radian() - (tar_deg[11]*D2R);
    angle_err[10] = this->RR_THIGH_JOINT->GetAngle(0).Radian() - (tar_deg[10]*D2R);
    angle_err[9]  = this->RR_ROTATOR1_JOINT->GetAngle(0).Radian() - (tar_deg[9]*D2R);
    angle_err[12] = this->WAIST_JOINT->GetAngle(0).Radian() - (tar_deg[12]*D2R);

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

    this->FR_HIP_JOINT->SetForce(1, this->pid_FR_HR.GetCmd());
    this->FR_THIGH_JOINT->SetForce(1, this->pid_FR_HP.GetCmd());
    this->FR_ROTATOR1_JOINT->SetForce(1, this->pid_FR_KN.GetCmd());
    this->FL_HIP_JOINT->SetForce(1, this->pid_FL_HR.GetCmd());
    this->FL_THIGH_JOINT->SetForce(1, this->pid_FL_HP.GetCmd());
    this->FL_ROTATOR1_JOINT->SetForce(1, this->pid_FL_KN.GetCmd());
    this->RL_HIP_JOINT->SetForce(1, this->pid_RL_HR.GetCmd());
    this->RL_THIGH_JOINT->SetForce(1, this->pid_RL_HP.GetCmd());
    this->RL_ROTATOR1_JOINT->SetForce(1, this->pid_RL_KN.GetCmd());
    this->RR_HIP_JOINT->SetForce(1, this->pid_RR_HR.GetCmd());
    this->RR_THIGH_JOINT->SetForce(1, this->pid_RR_HP.GetCmd());
    this->RR_ROTATOR1_JOINT->SetForce(1, this->pid_RR_KN.GetCmd());
    this->WAIST_JOINT->SetForce(1, this->pid_WAIST.GetCmd());

    if(this->pid_FR_HR.GetCmd() >= 1000 || this->pid_FR_HR.GetCmd() <= -1000){
      printf("pid_FR_HR = %f\n",this->pid_FR_HR.GetCmd());
    }
    if(this->pid_FR_HP.GetCmd() >= 1000 || this->pid_FR_HP.GetCmd() <= -1000){
      printf("pid_FR_HP = %f\n",this->pid_FR_HP.GetCmd());
    }
    if(this->pid_FR_KN.GetCmd() >= 1000 || this->pid_FR_KN.GetCmd() <= -1000){
      printf("pid_FR_KN = %f\n",this->pid_FR_KN.GetCmd());
    }
    if(this->pid_FL_HR.GetCmd() >= 1000 || this->pid_FL_HR.GetCmd() <= -1000){
      printf("pid_FL_HR = %f\n",this->pid_FL_HR.GetCmd());
    }
    if(this->pid_FL_HP.GetCmd() >= 1000 || this->pid_FL_HP.GetCmd() <= -1000){
      printf("pid_FL_HP = %f\n",this->pid_FL_HP.GetCmd());
    }
    if(this->pid_FL_KN.GetCmd() >= 1000 || this->pid_FL_KN.GetCmd() <= -1000){
      printf("pid_FL_KN = %f\n",this->pid_FL_KN.GetCmd());
    }
    if(this->pid_RL_HR.GetCmd() >= 1000 || this->pid_RL_HR.GetCmd() <= -1000){
      printf("pid_RL_HR = %f\n",this->pid_RL_HR.GetCmd());
    }
    if(this->pid_RL_HP.GetCmd() >= 1000 || this->pid_RL_HP.GetCmd() <= -1000){
      printf("pid_RL_HP = %f\n",this->pid_RL_HP.GetCmd());
    }
    if(this->pid_RL_KN.GetCmd() >= 1000 || this->pid_RL_KN.GetCmd() <= -1000){
      printf("pid_RL_KN = %f\n",this->pid_RL_KN.GetCmd());
    }
    if(this->pid_RR_HR.GetCmd() >= 1000 || this->pid_RR_HR.GetCmd() <= -1000){
      printf("pid_RR_HR = %f\n",this->pid_RR_HR.GetCmd());
    }
    if(this->pid_RR_HP.GetCmd() >= 1000 || this->pid_RR_HP.GetCmd() <= -1000){
      printf("pid_RR_HP = %f\n",this->pid_RR_HP.GetCmd());
    }
    if(this->pid_RR_KN.GetCmd() >= 1000 || this->pid_RR_KN.GetCmd() <= -1000){
      printf("pid_RR_KN = %f\n",this->pid_RR_KN.GetCmd());
    }
    if(this->pid_WAIST.GetCmd() >= 1000 || this->pid_WAIST.GetCmd() <= -1000){
      printf("pid_WAIST = %f\n",this->pid_WAIST.GetCmd());
    }

//setting for getting dt
  this->last_update_time = current_time;

//getting readable angular_velocity data
  m_Times.data = current_time.Double();
  m_angular_velocity_x.data = angular_velocity_x;
  m_angular_velocity_y.data = angular_velocity_y;
  m_angular_velocity_z.data = angular_velocity_z;
  P_Times.publish(m_Times);
  P_angular_velocity_x.publish(m_angular_velocity_x);
  P_angular_velocity_y.publish(m_angular_velocity_y);
  P_angular_velocity_z.publish(m_angular_velocity_z);
}