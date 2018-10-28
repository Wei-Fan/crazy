#include "ros/ros.h"

#include "easyfly/Swarm_Add.h"
#include "easyfly/LogBlock.h"
#include "easyfly/GenericLogData.h"
#include "easyfly/UpdateParams.h"
#include "easyfly/att_est.h"
#include "std_srvs/Empty.h"
#include "geometry_msgs/Twist.h"  
#include "type_methode.h"
#include "commons.h"
#include "std_msgs/Float32.h"
#include <easyfly/output.h>
#include <thread>
#include <mutex>
#include "Eigen/Eigen/Eigen"
#include "sensor_msgs/Imu.h"
#include "Eigen/Eigen/Geometry"
#include <crazyflie_cpp/Crazyflie.h>

#include <stdio.h> //sprintf, FILE* operations
#include <iostream>
#include <vector>
#include "string.h"
#include <sensor_msgs/Joy.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <easyfly/pos_ctrl_sp.h>
#include <easyfly/raw_ctrl_sp.h>
#include <easyfly/trj_ctrl_sp.h>
#include <easyfly/pos_est.h>
#include <easyfly/att_est.h>
#include <easyfly/Recording.h>
#include <vicon_bridge/Markers.h>
#include <vicon_bridge/Marker.h>
#include <easyfly/commands.h>
#include "control_modified.h"
#include <fstream>

using namespace std;
using namespace Eigen;

int g_joy_num=1;

class CrazyflieROS : private Controller//: private Attitude_estimator 
{
public:
  CrazyflieROS(
    const std::string& link_uri,
    const std::string& tf_prefix,
    int group_index,
    float roll_trim,
    float pitch_trim,
    bool enable_logging,
    bool enable_logging_imu,
    bool enable_parameters,
    std::vector<easyfly::LogBlock>& log_blocks,
    bool use_ros_time,
    bool enable_logging_att)
    : m_cf(link_uri)
    , m_uri(link_uri)
    , m_tf_prefix(tf_prefix)
    , m_isEmergency(false)
    , m_roll_trim(roll_trim)
    , m_pitch_trim(pitch_trim)
    , m_enableLogging(enable_logging)
    , m_enableParameters(enable_parameters)
    , m_logBlocks(log_blocks)
    , m_use_ros_time(use_ros_time)
    , m_enable_logging_att(enable_logging_att)
    , m_enable_logging_imu(enable_logging_imu)
    , m_serviceEmergency()
    , m_serviceUpdateParams()
    , m_pubRssi() //Received Signal Strength Indication
    //, m_attpub()
    //, m_pubImu()
    , m_sentSetpoint(false)
    , m_group_index(group_index)
    , isFirstPosEst(true)//controller
    , isFirstAccIMU(true)
    , isFirstposSp(true)
    , isFirstAttEst(true)
    , m_cmd()
    , m_ctrl()
    , m_pos_est()
    , m_sp_vecs()
    , m_est_vecs()
    , m_pubs()
    , m_subs()
    , m_RecPub()

  {
    m_times.m_IntePrevious = ros::Time::now();
    m_times.m_previousTime = ros::Time::now();
    m_times.m_latt_time = ros::Time::now();
    m_times.m_lposSp_time = ros::Time::now();
    m_times.m_lposEst_time = ros::Time::now();  
    v_posctrl_output.setZero();

    acc_IMU.setZero();
    m_uri = link_uri;
    ros::NodeHandle n("~");
    ros::NodeHandle nh;
    m_prevsT_integ_err = ros::Time::now();

    char msg_name[50];

    /*output*/
    // sprintf(msg_name,"/vehicle%d/output", m_group_index);
    // m_pubs.m_outputpub = nh.advertise<easyfly::output>(msg_name, 10);

    sprintf(msg_name,"/vehicle%d/pos_est",m_group_index);
    m_estsub = nh.subscribe<easyfly::pos_est>(msg_name,5,&CrazyflieROS::pos_estCallback, this);

    sprintf(msg_name,"/joygroup%d/joy",0);
    m_subs.m_joysub = nh.subscribe<sensor_msgs::Joy>(msg_name,5,&CrazyflieROS::joyCb, this);
    
    sprintf(msg_name,"/vehicle%d/Recording",m_group_index); //recording data
    m_RecPub = nh.advertise<easyfly::Recording>(msg_name, 5);
    
    //attitude estimation
    /*att_est*/
    // sprintf(msg_name,"/vehicle%d/att_est",m_group_index);
    // m_subs.m_yawsub = nh.subscribe<easyfly::att_est>(msg_name,5,&CrazyflieROS::att_estCallback, this);
    
    sprintf(msg_name,"/vehicle%d/raw_ctrl_sp",m_group_index);
    m_subs.m_rawsub = nh.subscribe<easyfly::raw_ctrl_sp>(msg_name,5,&CrazyflieROS::rawctrlCallback, this);

    /*Imu*/
    // sprintf(msg_name,"/vehicle%d/tf_prefix/Imu",m_group_index);
    // m_subs.cfIMUsub = nh.subscribe<sensor_msgs::Imu>(msg_name,5,&CrazyflieROS::cfIMUCallback, this);

    sprintf(msg_name,"/vehicle%d/pos_ctrl_sp",m_group_index);
    m_subs.m_possub = nh.subscribe<easyfly::pos_ctrl_sp>(msg_name,5,&CrazyflieROS::posctrlCallback, this);

    sprintf(msg_name,"/vehicle%d/trj_ctrl_sp",m_group_index);
    m_subs.m_trjsub = nh.subscribe<easyfly::trj_ctrl_sp>(msg_name,5,&CrazyflieROS::trjctrlCallback, this);
    
    m_subs.m_cmdsub = nh.subscribe<easyfly::commands>("/commands",5,&CrazyflieROS::cmdCallback, this);
    

    /*output*/
    // sprintf(msg_name,"/vehicle%d/output",m_group_index);
    // m_outputsub = nh.subscribe<easyfly::output>(msg_name,10,&CrazyflieROS::outputCallback, this);

    // if (m_enable_logging_att) {
    //   sprintf(msg_name,"/vehicle%d/att_est",m_group_index);
    //   m_attpub = n.advertise<easyfly::att_est>(msg_name, 10);
    // }
    // if (m_enable_logging_imu) {
    //   sprintf(msg_name,"/vehicle%d/tf_prefix/Imu",m_group_index);
    //   m_pubImu = n.advertise<sensor_msgs::Imu>(msg_name, 10);
    // }
    sprintf(msg_name,"/vehicle%d/rssi",m_group_index);
    m_pubRssi = n.advertise<std_msgs::Float32>(msg_name, 10);

    for (auto& logBlock : m_logBlocks)
    {
      m_pubLogDataGeneric.push_back(n.advertise<easyfly::GenericLogData>(tf_prefix + "/" + logBlock.topic_name, 10));
    }
    msg_att_est.uri = m_uri;
    msg_att_est.group_index = m_group_index;

    msg_att_est.att_est.x = m_roll_trim;
    msg_att_est.att_est.y = m_pitch_trim;
    msg_att_est.att_est.z = 0.0f;
    
    //test = true;
    std::thread t(&CrazyflieROS::run, this);
    t.detach();
  }

private:

  Vector3f acc_IMU;
  struct logStablizer {
    float roll;
    float pitch;
    float yaw;
    uint16_t throttle;
  } __attribute__((packed));
  
  struct logImu {
    float acc_x;
    float acc_y;
    float acc_z;
  } __attribute__((packed));

private:
  bool emergency(
    std_srvs::Empty::Request& req,
    std_srvs::Empty::Response& res)
  {
    ROS_FATAL("Emergency requested!");
    m_isEmergency = true;

    return true;
  }

  template<class T, class U>
  void updateParam(uint8_t id, const std::string& ros_param) {
      U value;
      ros::param::get(ros_param, value);
      m_cf.setParam<T>(id, (T)value);
  }

  bool updateParams(
    easyfly::UpdateParams::Request& req,
    easyfly::UpdateParams::Response& res)
  {
    ROS_INFO("Update parameters");
    for (auto&& p : req.params) {
      std::string ros_param = "/" + m_tf_prefix + "/" + p;
      size_t pos = p.find("/");
      std::string group(p.begin(), p.begin() + pos);
      std::string name(p.begin() + pos + 1, p.end());

      auto entry = m_cf.getParamTocEntry(group, name);
      if (entry)
      {
        switch (entry->type) {
          case Crazyflie::ParamTypeUint8:
            updateParam<uint8_t, int>(entry->id, ros_param);
            break;
          case Crazyflie::ParamTypeInt8:
            updateParam<int8_t, int>(entry->id, ros_param);
            break;
          case Crazyflie::ParamTypeUint16:
            updateParam<uint16_t, int>(entry->id, ros_param);
            break;
          case Crazyflie::ParamTypeInt16:
            updateParam<int16_t, int>(entry->id, ros_param);
            break;
          case Crazyflie::ParamTypeUint32:
            updateParam<uint32_t, int>(entry->id, ros_param);
            break;
          case Crazyflie::ParamTypeInt32:
            updateParam<int32_t, int>(entry->id, ros_param);
            break;
          case Crazyflie::ParamTypeFloat:
            updateParam<float, float>(entry->id, ros_param);
            break;
        }
      }
      else {
        ROS_ERROR("Could not find param %s/%s", group.c_str(), name.c_str());
      }
    }
    return true;
  }

  void run()
  {
    //printf("************start run\n");
    // m_cf.reboot();
    m_cf.logReset();
    std::thread::id this_id = std::this_thread::get_id();
    printf("thread number ########%d is running\n",this_id);
    fflush(stdout);
    float frequency = 50;
    std::function<void(float)> cb_lq = std::bind(&CrazyflieROS::onLinkQuality, this, std::placeholders::_1);
    m_cf.setLinkQualityCallback(cb_lq);
    auto start = std::chrono::system_clock::now();

    if (m_enableParameters)
    {
      ROS_INFO("Requesting parameters...");
      m_cf.requestParamToc();
      for (auto iter = m_cf.paramsBegin(); iter != m_cf.paramsEnd(); ++iter) {
        auto entry = *iter;
        std::string paramName = "/" + m_tf_prefix + "/" + entry.group + "/" + entry.name;
        switch (entry.type) {
          case Crazyflie::ParamTypeUint8:
            ros::param::set(paramName, m_cf.getParam<uint8_t>(entry.id));
            break;
          case Crazyflie::ParamTypeInt8:
            ros::param::set(paramName, m_cf.getParam<int8_t>(entry.id));
            break;
          case Crazyflie::ParamTypeUint16:
            ros::param::set(paramName, m_cf.getParam<uint16_t>(entry.id));
            break;
          case Crazyflie::ParamTypeInt16:
            ros::param::set(paramName, m_cf.getParam<int16_t>(entry.id));
            break;
          case Crazyflie::ParamTypeUint32:
            ros::param::set(paramName, (int)m_cf.getParam<uint32_t>(entry.id));
            break;
          case Crazyflie::ParamTypeInt32:
            ros::param::set(paramName, m_cf.getParam<int32_t>(entry.id));
            break;
          case Crazyflie::ParamTypeFloat:
            ros::param::set(paramName, m_cf.getParam<float>(entry.id));
            break;
        }
      }
    }

    std::unique_ptr<LogBlock<logStablizer> > logblockStablizer;
    std::unique_ptr<LogBlock<logImu> > logBlockImu;
    
    if (m_enableLogging) {

      std::function<void(const crtpPlatformRSSIAck*)> cb_ack = std::bind(&CrazyflieROS::onEmptyAck, this, std::placeholders::_1);
      m_cf.setEmptyAckCallback(cb_ack);

      ROS_INFO("Requesting Logging variables...");
      m_cf.requestLogToc();

      if(m_enable_logging_att){
        std::function<void(uint32_t, logStablizer*)> cb_stab = std::bind(&CrazyflieROS::onAttitude, this, std::placeholders::_1,std::placeholders::_2);

      logblockStablizer.reset(new LogBlock<logStablizer>(
          &m_cf,{
            {"stabilizer", "roll"},
            {"stabilizer", "pitch"},
            {"stabilizer", "yaw"},
            {"stabilizer", "thrust"},
          }, cb_stab));
        logblockStablizer->start(1); // 10ms
      }

      if (m_enable_logging_imu) {
        std::function<void(uint32_t, logImu*)> cb = std::bind(&CrazyflieROS::onImuData, this, std::placeholders::_1, std::placeholders::_2);

        logBlockImu.reset(new LogBlock<logImu>(
          &m_cf,{
            {"acc", "x"},
            {"acc", "y"},
            {"acc", "z"},
            }, cb));
        logBlockImu->start(1); // 10ms
        //printf("%f\n", cb.acc_x);
      }
      
    }
    ROS_INFO("Ready...");
    auto end = std::chrono::system_clock::now();
    std::chrono::duration<double> elapsedSeconds = end-start;
    ROS_INFO("Elapsed: %f s", elapsedSeconds.count());

    // Send 0 thrust initially for thrust-lock
    for (int i = 0; i < 100; ++i) {
       m_cf.sendSetpoint(0, 0, 0, 0);
    }

    while(!m_isEmergency) {
      // make sure we ping often enough to stream data out
      if (m_enableLogging && !m_sentSetpoint) {
        m_cf.sendPing();
      }
      m_sentSetpoint = false;
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    // Make sure we turn the engines off
    for (int i = 0; i < 100; ++i) {
       m_cf.sendSetpoint(0, 0, 0, 0);
    }

    ros::NodeHandle node;
    node.getParam("/flight_mode", m_flight_mode);
    ros::Timer timer = node.createTimer(ros::Duration(1.0/frequency), &CrazyflieROS::iteration, this);
    ros::spin();
  }

  void iteration(const ros::TimerEvent& e)
  {         
    
    static float time_elapse = 0;
    float dt = e.current_real.toSec() - e.last_real.toSec();
    time_elapse += dt;
    if(m_cmd.cut){
      m_output.att_sp.x = 0.0f;
      m_output.att_sp.y = 0.0f;
      m_output.att_sp.z = 0.0f;
      m_output.throttle = 0.0f;
     // m_pubs.m_outputpub.publish(m_output);
      m_cf.sendSetpoint(
          -m_output.att_sp.x * RAD2DEG,
          - m_output.att_sp.y * RAD2DEG,
          m_output.att_sp.z * RAD2DEG,
          m_output.throttle * 65000); 
      m_sentSetpoint = true;
    }
    else{     
      switch(m_flight_mode){
        case MODE_RAW:{

          m_output.att_sp.x = m_ctrl.raw.raw_att_sp.x;
          m_output.att_sp.y = m_ctrl.raw.raw_att_sp.y;
          m_output.att_sp.z = m_ctrl.raw.raw_att_sp.z;
          m_output.throttle = m_ctrl.raw.throttle;
          //m_pubs.m_outputpub.publish(m_output);
          m_cf.sendSetpoint(
              -m_output.att_sp.x * RAD2DEG,
              - m_output.att_sp.y * RAD2DEG,
              m_output.att_sp.z * RAD2DEG,
              m_output.throttle * 65000); 
          m_sentSetpoint = true;
        }//case MODE_RAW
        break;
        case MODE_POS:{
          if(m_cmd.flight_state!=Idle && !isFirstposSp && !isFirstPosEst){// && !isFirstAccIMU && !isFirstAttEst){
          //printf("%d    %d     %d     %d    %d!!\n",m_cmd.flight_state,isFirstposSp,isFirstPosEst,isFirstAccIMU,isFirstAttEst);
          //if(!isFirstAccIMU && !isFirstAttEst){ //static test
            control_nonLineaire(&m_recording, &m_pos_est, &m_sp_vecs.v_posctrl_posSp, &m_sp_vecs.v_posctrl_velFF, &m_sp_vecs.v_posctrl_acc_sp, &v_posctrl_output, &m_est_vecs.m_cfImuAcc, &m_est_vecs.m_att_est, dt);
            recordingFormatChanging(&m_recording);
            m_RecPub.publish(m_recordmsg);
            //attitude_control(&m_sp_vecs.v_posctrl_posSp, &v_posctrl_output, &m_est_vecs.m_att_est, dt);
            m_output.att_sp.x = v_posctrl_output(0);
            m_output.att_sp.y = v_posctrl_output(1);
            m_output.att_sp.z = v_posctrl_output(2);  
            m_output.throttle = v_posctrl_output(3);
            //printf("%f\n",m_sp_vecs.v_posctrl_acc_sp(0) );
            //printf("output give:  %f     %f\n", v_posctrl_output(0),v_posctrl_output(1));
            //m_pubs.m_outputpub.publish(m_output);
            m_cf.sendSetpoint(
                -m_output.att_sp.x * RAD2DEG,
                - m_output.att_sp.y * RAD2DEG,
                m_output.att_sp.z * RAD2DEG,
                m_output.throttle * 65000); 
            m_sentSetpoint = true;
          }
        //}//if flight_mode!=Idle
      }//case MODE_POS
        break;
        case MODE_TRJ:{
          
        }
        break;
        default:
        break;
      }//end switch flight mode
    }//end if cut
    
  }//iteration

  void joyCb(const sensor_msgs::Joy::ConstPtr& joy)
  {

    Pitch_Sp = -joy->axes[4] * 30 * DEG2RAD;//+-1
    Roll_Sp = -joy->axes[3] * 30 * DEG2RAD;
        
  }

  void recordingFormatChanging(M_recording* recording)
  {
    for (int i=0;i<3;i++){
      m_recordmsg.Rec_posEst.x = recording->Rec_posEst(0);
      m_recordmsg.Rec_posEst.y = recording->Rec_posEst(1);
      m_recordmsg.Rec_posEst.z = recording->Rec_posEst(2);
      m_recordmsg.Rec_velEst.x = recording->Rec_velEst(0);
      m_recordmsg.Rec_velEst.y = recording->Rec_velEst(1);
      m_recordmsg.Rec_velEst.z = recording->Rec_velEst(2);
      m_recordmsg.Rec_attEst.x = recording->Rec_attEst(0);
      m_recordmsg.Rec_attEst.y = recording->Rec_attEst(1);
      m_recordmsg.Rec_attEst.z = recording->Rec_attEst(2);
      m_recordmsg.Rec_posSp.x = recording->Rec_posSp(0);
      m_recordmsg.Rec_posSp.y = recording->Rec_posSp(1);
      m_recordmsg.Rec_posSp.z = recording->Rec_posSp(2);
      m_recordmsg.Rec_velSp.x = recording->Rec_velSp(0);
      m_recordmsg.Rec_velSp.y = recording->Rec_velSp(1);
      m_recordmsg.Rec_velSp.z = recording->Rec_velSp(2);
      m_recordmsg.Rec_accSp.x = recording->Rec_accSp(0);
      m_recordmsg.Rec_accSp.y = recording->Rec_accSp(1);
      m_recordmsg.Rec_accSp.z = recording->Rec_accSp(2);
      m_recordmsg.Rec_yaw_sp = recording->Rec_yaw_sp; 
      m_recordmsg.Rec_roll_sp = recording->Rec_roll_sp; 
      m_recordmsg.Rec_pitch_sp = recording->Rec_pitch_sp;
    }
  }

  void rawctrlCallback(const easyfly::raw_ctrl_sp::ConstPtr& ctrl)
  { 
    m_ctrl.raw.raw_att_sp.x = ctrl->raw_att_sp.x;
    m_ctrl.raw.raw_att_sp.y = ctrl->raw_att_sp.y;
    m_ctrl.raw.raw_att_sp.z = ctrl->raw_att_sp.z;
    m_ctrl.raw.throttle = ctrl->throttle;
  }

  void pos_estCallback(const easyfly::pos_est::ConstPtr& est)
  { 
    if(isFirstPosEst){
      m_group_index = est->vehicle_index;
      isFirstPosEst = false;
    }
    m_pos_est(0) = est->pos_est.x;
    m_pos_est(1) = est->pos_est.y;
    m_pos_est(2) = est->pos_est.z;
  }

  void posctrlCallback(const easyfly::pos_ctrl_sp::ConstPtr& ctrl)
  {
    //printf("posSp in controller: %f %f %f\n",m_sp_vecs.v_posctrl_posSp(0),m_sp_vecs.v_posctrl_posSp(1),m_sp_vecs.v_posctrl_posSp(2) );
    (m_sp_vecs.v_posctrl_posSp)(0) = ctrl->pos_sp.x;
    (m_sp_vecs.v_posctrl_posSp)(1) = ctrl->pos_sp.y;
    (m_sp_vecs.v_posctrl_posSp)(2) = ctrl->pos_sp.z;

    (m_sp_vecs.v_posctrl_velFF)(0) = ctrl->vel_ff.x;
    (m_sp_vecs.v_posctrl_velFF)(1) = ctrl->vel_ff.y;
    (m_sp_vecs.v_posctrl_velFF)(2) = ctrl->vel_ff.z;

    (m_sp_vecs.v_posctrl_acc_sp)(0) = ctrl->acc_sp.x;
    (m_sp_vecs.v_posctrl_acc_sp)(1) = ctrl->acc_sp.y;
    (m_sp_vecs.v_posctrl_acc_sp)(2) = ctrl->acc_sp.z;
    
    (m_sp_vecs.v_posctrl_posSp)(3) = ctrl->yaw_sp;
    if(isFirstposSp)
    {
      isFirstposSp = false; 
    }
    
  }
  void trjctrlCallback(const easyfly::trj_ctrl_sp::ConstPtr& ctrl)
  {}

  void cmdCallback(const easyfly::commands::ConstPtr& cmd)
  {
    m_cmd.flight_state = cmd->flight_state;
    m_cmd.l_flight_state = cmd->l_flight_state;
    m_cmd.cut = cmd->cut;
  }

  void onImuData(uint32_t time_in_ms, logImu* data) {
    if (m_enable_logging_imu) {
      sensor_msgs::Imu msg;
      if (m_use_ros_time) {
        msg.header.stamp = ros::Time::now();
      } else {
        msg.header.stamp = ros::Time(time_in_ms / 1000.0);
      }
      msg.header.frame_id = m_tf_prefix + "/base_link";
      msg.orientation_covariance[0] = -1;

      // measured in mG; need to convert to m/s^2
      msg.linear_acceleration.x = data->acc_x * -9.81;
      msg.linear_acceleration.y = data->acc_y * -9.81;
      msg.linear_acceleration.z = data->acc_z * -9.81;
      acc_IMU(0) = msg.linear_acceleration.x;
      acc_IMU(1) = msg.linear_acceleration.y;
      acc_IMU(2) = msg.linear_acceleration.z;
      
      //m_pubImu.publish(msg);

      (m_est_vecs.m_cfImuAcc)(0) = msg.linear_acceleration.x;
      (m_est_vecs.m_cfImuAcc)(1) = msg.linear_acceleration.y;
      (m_est_vecs.m_cfImuAcc)(2) = msg.linear_acceleration.z;
      
      if(isFirstAccIMU){
        //resetaccController(&m_est_vecs.m_cfImuAcc);
        isFirstAccIMU = false;
      }
    }//if m_enable_logging_imu
  }
  
  void onAttitude(uint32_t time_in_ms, logStablizer* data)
  {
      m_attest(0) = degToRad(data->roll);
      m_attest(1) = degToRad(data->pitch);
      m_attest(2) = degToRad(data->yaw);
      //m_attpub.publish(msg_att_est);
      //printf("Data from ###%s:  %f    %f    %f \n",m_uri.c_str(), m_attest(0), -m_attest(1), m_attest(2));
      (m_est_vecs.m_att_est)(0) = m_attest(0);
      (m_est_vecs.m_att_est)(1) = -m_attest(1);
      (m_est_vecs.m_att_est)(2) = m_attest(2);

      if(isFirstAttEst){
        isFirstAttEst = false;
      }
      char msg_name[50];
      sprintf(msg_name,"/vehicle%d/yaw_bias",m_group_index);
      ros::NodeHandle n;
      // ros::NodeHandle n;
      float yaw_bias=0;
      n.getParam(msg_name, yaw_bias);
      (m_est_vecs.m_att_est)(2) += yaw_bias;
      printf("YAW_GET_controller:  %f   %f \n",m_attest(2), (m_est_vecs.m_att_est)(2));
  }
  
  void onEmptyAck(const crtpPlatformRSSIAck* data) {
      std_msgs::Float32 msg;
      // dB
      msg.data = data->rssi;
      m_pubRssi.publish(msg);
  }

  void onLinkQuality(float linkQuality) {
      std::thread::id this_id = std::this_thread::get_id();
      if (linkQuality < 0.7) {
        ROS_WARN("Link Quality low (%f) for thread*****%d", linkQuality, this_id);
      }
  }


private:
  Crazyflie m_cf;
  std::string m_uri;
  std::string m_tf_prefix;
  bool m_isEmergency;
  float m_roll_trim;
  float m_pitch_trim;
  int m_group_index;

  bool m_enableLogging;
  bool m_enableParameters;
  std::vector<easyfly::LogBlock> m_logBlocks;
  bool m_use_ros_time;
  bool m_enable_logging_att;
  bool m_enable_logging_imu;
  ros::ServiceServer m_serviceEmergency;
  ros::ServiceServer m_serviceUpdateParams;
  //ros::Subscriber m_outputsub;
  
  //ros::Publisher m_attpub;
  ros::Publisher m_pubRssi;
  //ros::Publisher m_pubImu;
  ros::Time m_prevsT_integ_err;
  std::vector<ros::Publisher> m_pubLogDataGeneric;
  
  bool m_sentSetpoint;
  easyfly::att_est msg_att_est;
  Vector3f m_attest;//, m_init_North;
  //bool test;

  int m_flight_mode;
  bool isFirstPosEst;
  bool isFirstposSp;
  bool isFirstAccIMU;
  bool isFirstAttEst;

  struct M_Ctrl
  {
    easyfly::pos_ctrl_sp pos;
    easyfly::raw_ctrl_sp raw;
    easyfly::trj_ctrl_sp trj;
  };
  M_Ctrl m_ctrl;

  struct M_Pubs{
    ros::Publisher m_pos_estpub;
  };
  M_Pubs m_pubs;

  struct M_subs{
    ros::Subscriber m_viconsub, m_yawsub, m_joysub;
    ros::Subscriber m_rawsub, m_possub, m_trjsub, cfIMUsub, m_viconMarkersub;
    ros::Subscriber m_cmdsub, m_estsub;
  };
  ros::Subscriber m_estsub;
  M_subs m_subs;
  M_est_Vecs m_est_vecs;
  M_sp_Vecs m_sp_vecs;
  M_recording m_recording;
  struct M_times
  {
    ros::Time m_previousTime, m_latt_time, m_lposEst_time, m_lposSp_time, m_IntePrevious; 
  };
  M_times m_times;

  Vector4f v_posctrl_output;
  Vector3f m_pos_est;
  easyfly::commands m_cmd;
  easyfly::output m_output;
  easyfly::pos_est m_pos_estmsg;
  easyfly::Recording m_recordmsg;
  std::string tf_prefix;
  ros::Publisher m_RecPub;

};

void Controller::control_nonLineaire(M_recording* m_recording, const Vector3f* pos_est_Vicon, Vector4f* Sp, Vector3f* Vel_ff, Vector3f* acc_Sp, Vector4f* Output, Vector3f* acc_est_IMU, Vector3f* Euler, float dt)
{ 
  if(dt<0.1f)
  {
    //printf("%f   %f    %f \n", (*pos_est_Vicon)(0),(*pos_est_Vicon)(1),(*pos_est_Vicon)(2));
    _acc_Sp_W = *acc_Sp;
    euler2rotation(Euler,&R_est);
    body2earth(&R_est, acc_est_IMU, &acc_IMU_wd, 3);
    for(int i=0; i<3; i++){
      pos_Sp(i) = (*Sp)(i);
    }
    /**::position part::**/
    acc_IMU_wd(2) += GRAVITY/1000.0f;
    vel_estIMU += acc_IMU_wd*dt; 
    pos_estIMU += vel_estIMU*dt;
    pos_estIMU(2) = min(pos_estIMU(2),0.0f);

    float x_temp_est = (*pos_est_Vicon)(0);
    float y_temp_est = (*pos_est_Vicon)(1);
    float z_temp_est = (*pos_est_Vicon)(2);

    float x_sp = pos_Sp(0);
    float y_sp = pos_Sp(1);
    float z_sp = pos_Sp(2);

    vel_Sp = *Vel_ff;
    vel_estVicon(0) = ((*pos_est_Vicon)(0) - l_posVicon(0))/dt;
    vel_estVicon(1) = ((*pos_est_Vicon)(1) - l_posVicon(1))/dt;
    vel_estVicon(2) = ((*pos_est_Vicon)(2) - l_posVicon(2))/dt;
    //vec3f_derivative(&vel_estVicon, pos_est_Vicon, &l_posVicon, dt);
    l_posVicon(0) = x_temp_est;
    l_posVicon(1) = y_temp_est;
    l_posVicon(2) = z_temp_est;
    //l_possp = pos_Sp;

    float vx_temp_est = vel_estVicon(0);
    float vy_temp_est = vel_estVicon(1);
    float vz_temp_est = vel_estVicon(2);

    //easyfly::pos_est posestMsg;
    
    m_recording->Rec_posEst(0) = (*pos_est_Vicon)(0);
    m_recording->Rec_posEst(1) = (*pos_est_Vicon)(1);
    m_recording->Rec_posEst(2) = (*pos_est_Vicon)(2);

    m_recording->Rec_velEst(0) = vel_estVicon(0);
    m_recording->Rec_velEst(1) = vel_estVicon(1);
    m_recording->Rec_velEst(2) = vel_estVicon(2);
    //m_posEstPub.publish(posestMsg);

    float vx_sp = vel_Sp(0);
    float vy_sp = vel_Sp(1);
    float vz_sp = vel_Sp(2);
    
    vel_Sp(0) =  m_pidX.pp_update(x_temp_est , x_sp); //+ff
    vel_Sp(1) =  m_pidY.pp_update(y_temp_est , y_sp);
    vel_Sp(2) =  m_pidZ.pp_update(z_temp_est , z_sp);

    //easyfly::pos_ctrl_sp posSpmsg;
    m_recording->Rec_velSp(0) = vel_Sp(0);
    m_recording->Rec_velSp(1) = vel_Sp(1);
    m_recording->Rec_velSp(2) = vel_Sp(2);

    l_velsp = vel_Sp;
    //printf("%f\n",x_temp_est-x_sp );
    _acc_Sp_W(0) =  m_pidX.pid_update(vx_temp_est,vel_Sp(0),dt);
    _acc_Sp_W(1) =  m_pidY.pid_update(vy_temp_est,vel_Sp(1),dt);
    _acc_Sp_W(2) =  m_pidZ.pid_update(vz_temp_est,vel_Sp(2),dt);

    //_acc_Sp_W(2) =  m_pidVz.pid_update(z_temp_est,pos_Sp(2),dt);

    //vec3f_derivative(&(*acc_Sp), &vel_Sp, &l_velsp, dt);
    
    acc_Sp_net = _acc_Sp_W;

    _acc_Sp_W(2) = _acc_Sp_W(2) + GRAVITY/1000.0f * (float)VEHICLE_MASS;

    m_recording->Rec_accSp(0) = _acc_Sp_W(0);
    m_recording->Rec_accSp(1) = _acc_Sp_W(1);
    m_recording->Rec_accSp(2) = _acc_Sp_W(2);

    vec3f_passnorm(&_acc_Sp_W, &_Zb_des);

    for (int i=0; i<3; i++)
      _R_des(i,2) = _Zb_des(i);

    _Xc_des(0) = cos((*Sp)(3));
    _Xc_des(1) = sin((*Sp)(3));
    _Xc_des(2) = 0;
    
    /*float yaw_deriv = deriv_f((*Sp)(3),l_yawSp,dt);
    l_yawSp = (*Sp)(3);*/
    vec3f_cross(&_Zb_des, &_Xc_des, &_Yb_des);
    vec3f_normalize(&_Yb_des);
    vec3f_cross(&_Yb_des, &_Zb_des, &_Xb_des);

    for (int i=0; i<3; i++)
    {
      _R_des(i,0) = _Xb_des(i);
      _R_des(i,1) = _Yb_des(i);
    }

    rotation2euler(&_R_des,&RPY_des);

    m_recording->Rec_attEst(0) = (*Euler)(0);
    m_recording->Rec_attEst(1) = (*Euler)(1);
    m_recording->Rec_attEst(2) = (*Euler)(2);

    x_sp = RPY_des(0);
    y_sp = RPY_des(1);
    z_sp = RPY_des(2);

    for(int i=0;i<2;i++){
      (*Output)(i) = RPY_des(i);
    }
    (*Output)(2) = 0.0f;
    (*Output)(0) = -(*Output)(0);
    //(*Output)(2) = RPY_des(2);
    Vector3f temp;
    temp.setZero();
    for(int i=0;i<3;i++){
      temp(i) = _R_des(i,2);
    }

    //easyfly::att_est AttMsg;
    m_recording->Rec_roll_sp = RPY_des(0);
    m_recording->Rec_pitch_sp = RPY_des(1);
    m_recording->Rec_yaw_sp = RPY_des(2);
    //m_attSpPub.publish(AttMsg);

    float thrust_force = vec3f_dot(&_acc_Sp_W,&temp);

    thrust_force /= 440.0f;
    thrust_force = std::min(thrust_force,max_thrust);
    (*Output)(3) = thrust_force;    

  }
}

bool add_crazyflie(
  easyfly::Swarm_Add::Request  &req,
  easyfly::Swarm_Add::Response &res)
{
    ROS_INFO("Adding %s as %s with trim(%f, %f). Logging: %d, Parameters: %d, Use ROS time: %d, group_index: %d, g_vehicle_num: %d",
    req.uri.c_str(),
    req.tf_prefix.c_str(),
    req.roll_trim,
    req.pitch_trim,
    req.enable_logging,
    req.enable_parameters,
    req.use_ros_time,
    req.group_index,
    req.g_vehicle_num);

    /*crazyfly_bools.push_back(false);
    crazyfly_uris.push_back(req.uri);

    while(count != 0 && !crazyfly_bools[count-1])
    {
        ros::Duration(0.5).sleep();
    }
    printf("~~~~~~success~~~~~~~%s\n",req.uri.c_str());*/
    if (req.group_index != 0)
    {
      ros::Duration(2.0).sleep();
    }

    // Leak intentionally
    CrazyflieROS* cf = new CrazyflieROS(
    req.uri,
    req.tf_prefix,
    req.group_index,
    req.roll_trim,
    req.pitch_trim,
    req.enable_logging,
    req.enable_logging_imu,
    req.enable_parameters,
    req.log_blocks,
    req.use_ros_time,
    req.enable_logging_att);

    //crazyfly_bools[count] = true;
    //count++;

    return true;
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "swarm_Server_Decentralized");
  ros::NodeHandle n;
  char servicename[50];

  //use the absolute addresse for all cfs
  ros::ServiceServer service = n.advertiseService("/add_crazyflie", add_crazyflie);
  ros::spin();

  return 0;
}
