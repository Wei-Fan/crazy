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
#include <stdio.h> //sprintf
#include <easyfly/output.h>
#include <thread>
#include <mutex>
#include "Eigen/Eigen/Eigen"
#include "sensor_msgs/Imu.h"
#include "Eigen/Eigen/Geometry"
#include <crazyflie_cpp/Crazyflie.h>

using namespace Eigen;

/*const float K_Pa = 0.5f;
const float K_Ia = 0.1f;
const float K_Pm = 0.3f;
const float K_Im = 0.1f;*/

//constexpr double pi() { return std::atan(1)*4; }


class CrazyflieROS //: private Attitude_estimator 
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
    , m_attpub()
    , m_pubImu()
    , m_sentSetpoint(false)
    , m_group_index(group_index)
  {
    acc_IMU.setZero();
    char msg_name[50];
    m_uri = link_uri;
    ros::NodeHandle n("~");
    ros::NodeHandle nh;
    m_prevsT_integ_err = ros::Time::now();

    //m_serviceEmergency = n.advertiseService(tf_prefix + "/emergency", &CrazyflieROS::emergency, this);
    //m_serviceUpdateParams = n.advertiseService(tf_prefix + "/update_params", &CrazyflieROS::updateParams, this);

    sprintf(msg_name,"/vehicle%d/output",m_group_index);
    m_outputsub = nh.subscribe<easyfly::output>(msg_name,10,&CrazyflieROS::outputCallback, this);

    if (m_enable_logging_att) {
    	sprintf(msg_name,"/vehicle%d/att_est",m_group_index);
      m_attpub = n.advertise<easyfly::att_est>(msg_name, 10);
    }
    if (m_enable_logging_imu) {
      sprintf(msg_name,"/vehicle%d/tf_prefix/Imu",m_group_index);
      m_pubImu = n.advertise<sensor_msgs::Imu>(msg_name, 10);
    }
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
	void outputCallback(const easyfly::output::ConstPtr& msg)
	{
		m_output.att_sp.x = msg->att_sp.x;
		m_output.att_sp.y = msg->att_sp.y;
		m_output.att_sp.z = msg->att_sp.z;
		m_output.throttle = msg->throttle;
        m_cf.sendSetpoint(
          -m_output.att_sp.x * RAD2DEG,
				  - m_output.att_sp.y * RAD2DEG,
				  m_output.att_sp.z * RAD2DEG,
				  m_output.throttle * 65000); 
          m_sentSetpoint = true;
	}	
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

  /*void run_test()
  {
    //test = !test;
    int count_test = 0;
    std::thread::id this_id = std::this_thread::get_id();
    while(count_test < 100&&ros::ok())//&&!test)
    {
      count_test++;
      printf("thread number *****%d : %d\n",this_id, count_test);
      ros::Duration(0.5).sleep();
    }
  }*/

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
      
      m_pubImu.publish(msg);
    }//if m_enable_logging_imu
  }
  
  void onAttitude(uint32_t time_in_ms, logStablizer* data)
  {
      msg_att_est.uri = m_uri;
      m_attest(0) = degToRad(data->roll);
      m_attest(1) = degToRad(data->pitch);
      m_attest(2) = degToRad(data->yaw);
      msg_att_est.group_index = m_group_index;
      msg_att_est.att_est.x = m_attest(0);
      msg_att_est.att_est.y = -m_attest(1);
      msg_att_est.att_est.z = m_attest(2);
      m_attpub.publish(msg_att_est);
      //printf("Data from ###%s:  %f    %f    %f \n",m_uri.c_str(), m_attest(0), -m_attest(1), m_attest(2));
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
  ros::Subscriber m_outputsub;
  
  ros::Publisher m_attpub;
  ros::Publisher m_pubRssi;
  ros::Publisher m_pubImu;
  ros::Time m_prevsT_integ_err;
  std::vector<ros::Publisher> m_pubLogDataGeneric;
  
  bool m_sentSetpoint;
  easyfly::output m_output;
  easyfly::att_est msg_att_est;
  Vector3f m_attest;//, m_init_North;
  //bool test;

};
//static std::vector<std::string> crazyfly_uris;
//static std::vector<bool> crazyfly_bools;
//static int count;
//.c_str()
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
  ros::init(argc, argv, "/vehicle0");
  ros::NodeHandle n;
  char servicename[50];

  //use the absolute addresse for all cfs
  ros::ServiceServer service = n.advertiseService("/add_crazyflie", add_crazyflie);
  ros::spin();

  return 0;
}
