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
#include <easyfly/output.h>
#include "commons.h"
#include "control_modified.h"
#include <fstream>
using namespace Eigen;
using namespace std;
#define _USE_MATH_DEFINES //PI

int g_joy_num=1;

double get(
    const ros::NodeHandle& n,
    const std::string& name) {
	if (!n.hasParam(name)){
		ROS_INFO("No param named%s",name.c_str());
		return 0.0f;
	}
	else{
		double value;
    	n.getParam(name, value);
    	return value;
	}
    
}

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
public:
	
	Swarm_Controller(
		const ros::NodeHandle& n)
		: isFirstPosEst(true)
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
		, m_resFnameRoot()
		, m_RecPub()
	{
		m_times.m_IntePrevious = ros::Time::now();
		m_times.m_previousTime = ros::Time::now();
		m_times.m_latt_time = ros::Time::now();
		m_times.m_lposSp_time = ros::Time::now();
		m_times.m_lposEst_time = ros::Time::now();  
		v_posctrl_output.setZero();
		ros::NodeHandle nh("~");//~ means private param
		nh.getParam("group_index", m_group_index);

		sprintf(m_resFnameRoot,"/home/walt/catkin_ws/src/crazyflie_ros-first_trails/easyfly/resultat/vehicle%d/",m_group_index);

		/*char msg_name[50];
  		num_vehiclesub = nh.subscribe<crazyflie_driver::num_vehiclepub>("/num_vehiclepub",5,&Swarm_Controller::num_veh_Callback, this);*/
		char msg_name[50];
		sprintf(msg_name,"/vehicle%d/output", m_group_index);
		m_pubs.m_outputpub = nh.advertise<easyfly::output>(msg_name, 10);

		sprintf(msg_name,"/vehicle%d/pos_est",m_group_index);
		m_estsub = nh.subscribe<easyfly::pos_est>(msg_name,5,&Swarm_Controller::pos_estCallback, this);

		/*sprintf(msg_name,"/vehicle%d/pos_est", m_group_index); 
		m_pubs.m_pos_estpub = nh.advertise<easyfly::pos_est>(msg_name, 5);*/

		//sprintf(msg_name,"/vicon/crazyflie%d/whole",m_group_index);
		//m_subs.m_viconsub = nh.subscribe<geometry_msgs::TransformStamped>("/vicon/one_cf/cf",5,&Swarm_Controller::vicon_Callback, this);

		sprintf(msg_name,"/joygroup%d/joy",0);
		m_subs.m_joysub = nh.subscribe<sensor_msgs::Joy>(msg_name,5,&Swarm_Controller::joyCb, this);
		
		sprintf(msg_name,"/vehicle%d/Recording",m_group_index); //recording data
		m_RecPub = nh.advertise<easyfly::Recording>(msg_name, 5);

		/*sprintf(msg_name,"/vehicle%d/mpos_sp", m_group_index); 
		m_pos_sppub = nh.advertise<easyfly::pos_ctrl_sp>(msg_name, 5);

		sprintf(msg_name,"/vehicle%d/mpos_est", m_group_index); 
		m_posEstPub = nh.advertise<easyfly::pos_est>(msg_name, 5);

		sprintf(msg_name,"/vehicle%d/matt_sp", m_group_index); 
		m_attSpPub = nh.advertise<easyfly::att_est>(msg_name, 5);*/
		
		//attitude estimation
		sprintf(msg_name,"/vehicle%d/att_est",m_group_index);
		m_subs.m_yawsub = nh.subscribe<easyfly::att_est>(msg_name,5,&Swarm_Controller::att_estCallback, this);
		
		sprintf(msg_name,"/vehicle%d/raw_ctrl_sp",m_group_index);
		m_subs.m_rawsub = nh.subscribe<easyfly::raw_ctrl_sp>(msg_name,5,&Swarm_Controller::rawctrlCallback, this);

		sprintf(msg_name,"/vehicle%d/tf_prefix/Imu",m_group_index);
		m_subs.cfIMUsub = nh.subscribe<sensor_msgs::Imu>(msg_name,5,&Swarm_Controller::cfIMUCallback, this);

		sprintf(msg_name,"/vehicle%d/pos_ctrl_sp",m_group_index);
		m_subs.m_possub = nh.subscribe<easyfly::pos_ctrl_sp>(msg_name,5,&Swarm_Controller::posctrlCallback, this);

		sprintf(msg_name,"/vehicle%d/trj_ctrl_sp",m_group_index);
		m_subs.m_trjsub = nh.subscribe<easyfly::trj_ctrl_sp>(msg_name,5,&Swarm_Controller::trjctrlCallback, this);
		
		m_subs.m_cmdsub = nh.subscribe<easyfly::commands>("/commands",5,&Swarm_Controller::cmdCallback, this);
		
	}

	void run(double frequency)
	{
		ros::NodeHandle node;
		node.getParam("/flight_mode", m_flight_mode);
		//printf("hello!!!\n");

		ros::Timer timer = node.createTimer(ros::Duration(1.0/frequency), &Swarm_Controller::iteration, this);
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
			m_pubs.m_outputpub.publish(m_output);
		}
		else{			
			switch(m_flight_mode){
				case MODE_RAW:{

					m_output.att_sp.x = m_ctrl.raw.raw_att_sp.x;
					m_output.att_sp.y = m_ctrl.raw.raw_att_sp.y;
					m_output.att_sp.z = m_ctrl.raw.raw_att_sp.z;
					m_output.throttle = m_ctrl.raw.throttle;
					m_pubs.m_outputpub.publish(m_output);
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
						m_pubs.m_outputpub.publish(m_output);
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

	void cfIMUCallback(const sensor_msgs::Imu::ConstPtr& msg)
	{
		(m_est_vecs.m_cfImuAcc)(0) = msg->linear_acceleration.x;
		(m_est_vecs.m_cfImuAcc)(1) = msg->linear_acceleration.y;
		(m_est_vecs.m_cfImuAcc)(2) = msg->linear_acceleration.z;
		
		if(isFirstAccIMU){
			//resetaccController(&m_est_vecs.m_cfImuAcc);
			isFirstAccIMU = false;
		}
	}

	/*void vicon_Callback(const geometry_msgs::TransformStamped::ConstPtr& msg)
	{
		//printf("HELLO, VICON!!!\n");
		//if (strcmp(msg->marker_name.c_str(),"Unlabeled3".c_str())==0)
		//{
		//printf("unlabeled3!!!\n");
			m_pos_est(0) = msg->transform.translation.x;
			m_pos_est(1) = msg->transform.translation.y;
			m_pos_est(2) = msg->transform.translation.z;
			m_pos_estmsg.pos_est.x = msg->transform.translation.x;
			m_pos_estmsg.pos_est.y = msg->transform.translation.y;
			m_pos_estmsg.pos_est.z = msg->transform.translation.z;
			m_pubs.m_pos_estpub.publish(m_pos_estmsg);
			if(isFirstVicon)
			{
				//resetposController(&m_pos_est);
				isFirstVicon = false;
			}
		//}
		
	}*/

	void joyCb(const sensor_msgs::Joy::ConstPtr& joy)
	{

		Pitch_Sp = -joy->axes[4] * 30 * DEG2RAD;//+-1
		Roll_Sp = -joy->axes[3] * 30 * DEG2RAD;
				
	}
	void att_estCallback(const easyfly::att_est::ConstPtr& est)
	{	
		//printf("Att_est!!!  %f    %f    %f\n",(m_est_vecs.m_att_est)(0),(m_est_vecs.m_att_est)(1),(m_est_vecs.m_att_est)(2));
		(m_est_vecs.m_att_est)(0) = est->att_est.x;
		(m_est_vecs.m_att_est)(1) = est->att_est.y;
		(m_est_vecs.m_att_est)(2) = est->att_est.z;

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
		printf("YAW_GET_controller:  %f   %f \n",est->att_est.z, (m_est_vecs.m_att_est)(2));
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
	/*void vicon_markerCallback(const vicon_bridge::Markers::ConstPtr& msg)
	{
		printf("hello vicon_bridge%f\n",m_pos_est(0));
		m_markers = msg->markers;
		for (auto& Marker : m_markers)
    	{	
    		if(strcmp("Unlabeled0",Marker.marker_name.c_str()))
    		{	
    			m_pos_est(0) = Marker.translation.x/1000.0f;
				m_pos_est(1) = Marker.translation.y/1000.0f;
				m_pos_est(2) = Marker.translation.z/1000.0f;
				
				//printf("%f  %f  %f\n",m_pos_est(0),m_pos_est(1),m_pos_est(2));

				 m_pos_estmsg.pos_est.x = m_pos_est(0);
				 m_pos_estmsg.pos_est.y = m_pos_est(1);
				 m_pos_estmsg.pos_est.z = m_pos_est(2);
				 m_pubs.m_pos_estpub.publish(m_pos_estmsg);
				 if(isFirstVicon)
				 {
					resetposController(&m_pos_est);
					isFirstVicon = false;
				 }
    			//printf("%f\n",Marker.translation.z/1000.0f);
    		}
    		//printf("%s\n",Marker.marker_name.c_str());
    	}
	}*/

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
	{	

	}
	void cmdCallback(const easyfly::commands::ConstPtr& cmd)
	{
		m_cmd.flight_state = cmd->flight_state;
		m_cmd.l_flight_state = cmd->l_flight_state;
		m_cmd.cut = cmd->cut;
	}

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
int main(int argc, char **argv)
{
	char node_name[50];
	ros::init(argc, argv, "Swarm_Controller");
	ros::NodeHandle n("~");

	n.getParam("/joy_num", g_joy_num);
	Swarm_Controller Swarm_Controller(n);

	Swarm_Controller.run(50);
	return 0;
}