#include "ros/ros.h"
#include <stdio.h> //sprintf, FILE* operations
#include <iostream>
#include <vector>
#include "string.h"
#include <sensor_msgs/Joy.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
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
#include "sensor_msgs/Imu.h"
#include "commons.h"
#include "control_modified.h"
#include <fstream>
#include <crazyflie_cpp/Crazyflie.h>
#include <map>
#include "easyfly/LogBlock.h"
#include "std_msgs/Float32.h"
#include "type_methode.cpp"


class Swarm_Controller: private Controller
{
    
private:
	int m_group_index;
	int m_vehicle_num;
	int m_flight_state, m_flight_mode;
	bool isFirstPosEst;
	bool isFirstposSp;
	bool isFirstAccIMU;
	bool isFirstAttEst;
	bool isStateMode;
	bool m_sentSetpoint;
	bool m_use_ros_time;
	
	// std::vector<easyfly::LogBlock>* m_logBlocks;
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
    float gyro_x;
    float gyro_y;
    float gyro_z;
  } __attribute__((packed));

	//std::vector<vicon_bridge::Marker> m_markers;
	char m_resFnameRoot[150];
	struct M_Ctrl
	{
		easyfly::pos_ctrl_sp pos;
		easyfly::raw_ctrl_sp raw;
		easyfly::trj_ctrl_sp trj;
	};
	M_Ctrl m_ctrl;

	struct M_Pubs{
		ros::Publisher m_outputpub, m_pos_estpub;
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
		ros::Time m_previousTime, m_latt_time, m_lposEst_time, m_lposSp_time, m_IntePrevious,t_entering_cb; 
	};
	M_times m_times;

	Vector4f v_posctrl_output;
	Vector3f m_pos_est;
	easyfly::commands m_cmd;
	easyfly::pos_est m_pos_estmsg;
	easyfly::Recording m_recordmsg;
	std::string tf_prefix;
	ros::Publisher m_RecPub;
	ros::Publisher m_pubRssi;
	ros::Publisher m_pubImu;
	std::string m_tf_prefix;


public:
    /*information from the crazyflies*/
	Crazyflie* m_cf;
    Swarm_Controller(
		  Crazyflie* cf
		// , std::vector<easyfly::LogBlock>* log_blocks
		, int group_index
		, bool use_ros_time
		, const std::string& tf_prefix);
    ~Swarm_Controller();
    void run(double frequency);
    void iteration(const ros::TimerEvent& e);
    void joyCb(const sensor_msgs::Joy::ConstPtr& joy);
    void recordingFormatChanging(M_recording* recording);
    void rawctrlCallback(const easyfly::raw_ctrl_sp::ConstPtr& ctrl);
    void pos_estCallback(const easyfly::pos_est::ConstPtr& est);
    void posctrlCallback(const easyfly::pos_ctrl_sp::ConstPtr& ctrl);
    void trjctrlCallback(const easyfly::trj_ctrl_sp::ConstPtr& ctrl);
    void cmdCallback(const easyfly::commands::ConstPtr& cmd);
    void onEmptyAck(const crtpPlatformRSSIAck* data);
    void onImuData(uint32_t time_in_ms, logImu* data);
    void onAttitude(uint32_t time_in_ms, logStablizer* data);
    void sendSp();



};