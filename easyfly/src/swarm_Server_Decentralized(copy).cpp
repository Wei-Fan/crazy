#include "ros/ros.h"
#include "../include/Controller_Decentrolized.h"
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
#include <stdio.h> //sprintf
#include <easyfly/output.h>
#include <thread>
#include <mutex>
#include "Eigen/Eigen/Eigen"
#include "sensor_msgs/Imu.h"
#include "Eigen/Eigen/Geometry"
#include <crazyflie_cpp/Crazyflie.h>
#include <map>
#include <boost/thread/thread.hpp> 
using namespace Eigen;
int g_vehicle_num = 4;
/*const float K_Pa = 0.5f;
const float K_Ia = 0.1f;
const float K_Pm = 0.3f;
const float K_Im = 0.1f;*/

//constexpr double pi() { return std::atan(1)*4; }

class CrazyflieROS 
{
    friend class Swarm_Controller;
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
    , m_attpub()
    // , m_pubImu()
    , m_sentSetpoint(false)
    , m_group_index(group_index)
    , controller(&m_cf,group_index,m_use_ros_time,m_tf_prefix)
  {
    acc_IMU.setZero();
    char msg_name[50];
    m_uri = link_uri;
    //ros::NodeHandle n("~");
    ros::NodeHandle nh;
    m_prevsT_integ_err = ros::Time::now();

    

    	sprintf(msg_name,"/vehicle%d/att_est",m_group_index);
      m_attpub = nh.advertise<easyfly::att_est>(msg_name, 10);

    for (auto& logBlock : m_logBlocks)
    {
      m_pubLogDataGeneric.push_back(nh.advertise<easyfly::GenericLogData>(tf_prefix + "/" + logBlock.topic_name, 10));
    }
    
    ResetAtt();
    boost::thread* m_thread = new boost::thread(boost::bind(&CrazyflieROS::run,this));
  }

  ~CrazyflieROS()
  {
  	printf("CrazyflieROS# %d Dead!!\n",m_group_index );
  }
private:
  
  Vector3f acc_IMU;

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
  void ResetAtt()
  {
  	msg_att_est.uri = m_uri;
    msg_att_est.group_index = m_group_index;

    msg_att_est.att_est.x = m_roll_trim;
    msg_att_est.att_est.y = m_pitch_trim;
    msg_att_est.att_est.z = 0.0f;
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
    //m_cf.reboot();
    
  	
  	//***OK if put reset into comments
    //printf("Server_id %d\n",m_cf.m_devId );
    std::function<void(float)> cb_lq = std::bind(&CrazyflieROS::onLinkQuality, this, std::placeholders::_1);
    m_cf.setLinkQualityCallback(cb_lq);
    auto start = std::chrono::system_clock::now();
    //OK***

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
      //ros::NodeHandle n;
      //m_serviceUpdateParams = n.advertiseService(m_tf_prefix + "/update_params", &CrazyflieROS::updateParams, this);
    }
    
    ROS_INFO("Ready...");
    Crazyflie* cf_ptr;
    Crazyflie *& cf_ref = cf_ptr;
    cf_ref = &m_cf;
    // controller = Swarm_Controller(cf_ref,m_group_index,m_use_ros_time,m_tf_prefix);
    controller.run(100);
    auto end = std::chrono::system_clock::now();
    std::chrono::duration<double> elapsedSeconds = end-start;
    printf("------------- hhh --------------------\n");
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
    //m_mutex.unlock();
    /*
    std::thread::id this_id = std::this_thread::get_id();
    printf("thread: %d is running!!!\n",this_id );
    std::this_thread::sleep_for(std::chrono::milliseconds(1));*/
    
  }//run()

  void onEmptyAck(const crtpPlatformRSSIAck* data) {
      std_msgs::Float32 msg;
      // dB
      msg.data = data->rssi;
      m_pubRssi.publish(msg);
  }

  void onLinkQuality(float linkQuality) {
      if (linkQuality < 0.7) {
      	//m_cf.logReset();
        ROS_WARN("Link Quality low (%f)", linkQuality);
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
  Swarm_Controller controller;

  bool m_enableLogging;
  bool m_enableParameters;
  std::vector<easyfly::LogBlock> m_logBlocks;
  bool m_use_ros_time;
  bool m_enable_logging_att;
  bool m_enable_logging_imu;
  ros::ServiceServer m_serviceEmergency;
  ros::ServiceServer m_serviceUpdateParams;
  ros::Subscriber m_outputsub;
  
  ros::Publisher m_attpub;
  ros::Publisher m_pubRssi;
  // ros::Publisher m_pubImu;
  ros::Time m_prevsT_integ_err;
  std::vector<ros::Publisher> m_pubLogDataGeneric;
  
  bool m_sentSetpoint;
  easyfly::att_est msg_att_est;
  
  std::mutex m_mutex;
  //std::thread m_thread;
};
static std::vector<std::string> crazy_uris;
static std::vector<bool> crazy_bools;
static std::map<std::string, CrazyflieROS*> crazyflies;
// static int count;

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

	// Ignore if uri is already in use
  	if (crazyflies.find(req.uri) != crazyflies.end()) {
    ROS_ERROR("Cannot add %s, already added.", req.uri.c_str());
    return false;
  }
  	crazy_uris.push_back(req.uri);
  	crazy_bools.push_back(false);
  	
  // Leak intentionally

  	if(req.group_index!=0)
  	{
  		ros::Duration(3.0).sleep(); //give time to the preceding cfs...
  	}

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
   	
   	/*crazy_bools[count] = true;
   	count++;
   	printf("---------------count: %d----------------\n",count);
  	fflush(stdout);*/
  	//crazyflies[req.uri] = cf;

  	return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "swarm_Server_Decentralized");
  ros::NodeHandle n;
  char servicename[50];
  //use the absolute addresse for all cfs
  ros::ServiceServer service = n.advertiseService("/add_crazyflie", add_crazyflie);
  // ros::MultiThreadedSpinner spinner(g_vehicle_num);
  ros::spin();
  return 0;
}
