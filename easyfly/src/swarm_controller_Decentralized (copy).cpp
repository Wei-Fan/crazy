#include "../include/Controller_Decentrolized.h"

using namespace Eigen;
using namespace std;
#define _USE_MATH_DEFINES //PI

// int g_joy_num=1;

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
Swarm_Controller::Swarm_Controller(
	  Crazyflie* cf
	// , std::vector<easyfly::LogBlock>* log_blocks
	, int group_index
	, bool use_ros_time
	, const std::string& tf_prefix)
	: m_cf(cf)
	, m_group_index(group_index)
	, isFirstPosEst(true)
	, isFirstAccIMU(true)
	, isFirstposSp(true)
	, isFirstAttEst(true)
	, isStateMode(false)
	, m_sentSetpoint(false)
	, m_use_ros_time(use_ros_time)
	, m_cmd()
	, m_ctrl()
	, m_pos_est()
	, m_sp_vecs()
	, m_est_vecs()
	, m_pubs()
	, m_subs()
	, m_resFnameRoot()
	, m_RecPub()
	, m_recording()
	// , m_logBlocks(log_blocks)
	, m_pubRssi() //Received Signal Strength Indication
	, m_pubImu()
	, m_tf_prefix(tf_prefix)
{
	m_times.m_IntePrevious = ros::Time::now();
	m_times.m_previousTime = ros::Time::now();
	m_times.m_latt_time = ros::Time::now();
	m_times.t_entering_cb = ros::Time::now();
	m_times.m_lposSp_time = ros::Time::now();
	m_times.m_lposEst_time = ros::Time::now();  
	v_posctrl_output.setZero();
	ros::NodeHandle nh("~");//~ means private param
	
	char msg_name[50];
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
	
	sprintf(msg_name,"/vehicle%d/raw_ctrl_sp",m_group_index);
	m_subs.m_rawsub = nh.subscribe<easyfly::raw_ctrl_sp>(msg_name,5,&Swarm_Controller::rawctrlCallback, this);

	sprintf(msg_name,"/vehicle%d/pos_ctrl_sp",m_group_index);
	m_subs.m_possub = nh.subscribe<easyfly::pos_ctrl_sp>(msg_name,5,&Swarm_Controller::posctrlCallback, this);

	sprintf(msg_name,"/vehicle%d/trj_ctrl_sp",m_group_index);
	m_subs.m_trjsub = nh.subscribe<easyfly::trj_ctrl_sp>(msg_name,5,&Swarm_Controller::trjctrlCallback, this);
	
	m_subs.m_cmdsub = nh.subscribe<easyfly::commands>("/commands",5,&Swarm_Controller::cmdCallback, this);
	
	sprintf(msg_name,"/vehicle%d/rssi",m_group_index);
		m_pubRssi = nh.advertise<std_msgs::Float32>(msg_name, 10);
	
	sprintf(msg_name,"/vehicle%d/tf_prefix/Imu",m_group_index);
	m_pubImu = nh.advertise<sensor_msgs::Imu>(msg_name, 10);
}

void Swarm_Controller::run(double frequency)
{
	ros::NodeHandle node;
	node.getParam("/flight_mode", m_flight_mode);
	//printf("hello!!!\n");
	
	ros::Timer timer = node.createTimer(ros::Duration(1.0/frequency), &Swarm_Controller::iteration, this);
	ros::spin();
	printf("---------- hello controller! ------------");
		
}
Swarm_Controller::~Swarm_Controller()
{
	printf("Controller %d Dead!!!! \n",m_group_index );
}
void Swarm_Controller::iteration(const ros::TimerEvent& e)
{					
	std::unique_ptr<LogBlock<logStablizer> > logblockStablizer;
	std::unique_ptr<LogBlock<logImu> > logBlockImu;
	
	std::function<void (const crtpPlatformRSSIAck*)> cb_ack = std::bind(&Swarm_Controller::onEmptyAck, this, std::placeholders::_1);
	//auto cb_ack = std::bind(&Swarm_Controller::onEmptyAck, this, std::placeholders::_1);
	m_cf->setEmptyAckCallback(cb_ack);
	
	ROS_INFO("Requesting Logging variables...");
	m_cf->requestLogToc();
	
	std::function<void (uint32_t, logStablizer*)> cb_stab = std::bind(&Swarm_Controller::onAttitude, this, std::placeholders::_1,std::placeholders::_2);

	logblockStablizer.reset(new LogBlock<logStablizer>(
	m_cf,{
		{"stabilizer", "roll"},
		{"stabilizer", "pitch"},
		{"stabilizer", "yaw"},
		{"stabilizer", "thrust"},
	}, cb_stab));
	
	logblockStablizer->start(1); // 10ms

	std::function<void (uint32_t, logImu*)> cb = std::bind(&Swarm_Controller::onImuData, this, std::placeholders::_1, std::placeholders::_2);

	logBlockImu.reset(new LogBlock<logImu>(
	m_cf,{
		{"acc", "x"},
		{"acc", "y"},
		{"acc", "z"},
		{"gyro","x"},
		{"gyro","y"},
		{"gyro","z"},
		}, cb));
	logBlockImu->start(1); // 10ms
	
	static float time_elapse = 0;
	float dt = e.current_real.toSec() - e.last_real.toSec();
	time_elapse += dt;
	if(m_cmd.cut){
		v_posctrl_output(0) = 0.0f;
		v_posctrl_output(1) = 0.0f;
		v_posctrl_output(2) = 0.0f;
		v_posctrl_output(3) = 0.0f;
		sendSp();

	}
	else{			
		switch(m_flight_mode){
			case MODE_RAW:{

				v_posctrl_output(0) = m_ctrl.raw.raw_att_sp.x;
				v_posctrl_output(1) = m_ctrl.raw.raw_att_sp.y;
				v_posctrl_output(2) = m_ctrl.raw.raw_att_sp.z;
				v_posctrl_output(3) = m_ctrl.raw.throttle;
				sendSp();
			}//case MODE_RAW
			break;
			case MODE_POS:{
				if(m_cmd.flight_state!=Idle && !isFirstposSp && !isFirstPosEst && !isFirstAttEst){// && !isStateMode && !isFirstAccIMU){ 
					
					control_nonLineaire(&m_recording, &m_pos_est, &m_sp_vecs.v_posctrl_posSp, &m_sp_vecs.v_posctrl_velFF, &m_sp_vecs.v_posctrl_acc_sp, &v_posctrl_output, &m_est_vecs.m_cfImuAcc, &m_est_vecs.m_att_est, dt);
					recordingFormatChanging(&m_recording);
					m_RecPub.publish(m_recordmsg);
					sendSp();						
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

void Swarm_Controller::joyCb(const sensor_msgs::Joy::ConstPtr& joy)
{
	Pitch_Sp = -joy->axes[4] * 30 * DEG2RAD;//+-1
	Roll_Sp = -joy->axes[3] * 30 * DEG2RAD;
}

void Swarm_Controller::recordingFormatChanging(M_recording* recording)
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
void Swarm_Controller::rawctrlCallback(const easyfly::raw_ctrl_sp::ConstPtr& ctrl)
{	
	m_ctrl.raw.raw_att_sp.x = ctrl->raw_att_sp.x;
	m_ctrl.raw.raw_att_sp.y = ctrl->raw_att_sp.y;
	m_ctrl.raw.raw_att_sp.z = ctrl->raw_att_sp.z;
	m_ctrl.raw.throttle = ctrl->throttle;
}
void Swarm_Controller::pos_estCallback(const easyfly::pos_est::ConstPtr& est)
{	ros::Time t_cb = ros::Time::now();
	printf("------ time pos est: %f---------\n",(t_cb - m_times.t_entering_cb).toSec());
	printf("------ cf uri: %d -------------\n",m_group_index);
	m_times.t_entering_cb = t_cb;
	//printf("got pos estimation\n");
	if(isFirstPosEst){
		//m_group_index = est->vehicle_index;
		isFirstPosEst = false;
	}
	m_pos_est(0) = est->pos_est.x;
	m_pos_est(1) = est->pos_est.y;
	m_pos_est(2) = est->pos_est.z;
}

void Swarm_Controller::posctrlCallback(const easyfly::pos_ctrl_sp::ConstPtr& ctrl)
{
	//printf("posSp in controller: %f %f %f\n",m_sp_vecs.v_posctrl_posSp(0),m_sp_vecs.v_posctrl_posSp(1),m_sp_vecs.v_posctrl_posSp(2) );
	(m_sp_vecs.v_posctrl_posSp)(0) = ctrl->pos_sp.x;
	(m_sp_vecs.v_posctrl_posSp)(1) = ctrl->pos_sp.y;
	(m_sp_vecs.v_posctrl_posSp)(2) = ctrl->pos_sp.z;
	
	m_recording.Rec_posSp(0) = ctrl->pos_sp.x;
	m_recording.Rec_posSp(1) = ctrl->pos_sp.y;
	m_recording.Rec_posSp(2) = ctrl->pos_sp.z;

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
void Swarm_Controller::trjctrlCallback(const easyfly::trj_ctrl_sp::ConstPtr& ctrl)
{	
	
}
void Swarm_Controller::cmdCallback(const easyfly::commands::ConstPtr& cmd)
{
	m_cmd.flight_state = cmd->flight_state;
	m_cmd.l_flight_state = cmd->l_flight_state;
	m_cmd.cut = cmd->cut;
}

void Swarm_Controller::onEmptyAck(const crtpPlatformRSSIAck* data) {
	std_msgs::Float32 msg;
	// dB
	msg.data = data->rssi;
	m_pubRssi.publish(msg);
}
void Swarm_Controller::onImuData(uint32_t time_in_ms, logImu* data) {
	printf("------------- IMU data -------------\n");
	fflush(stdout);
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
	(m_est_vecs.m_cfImuAcc)(0) = msg.linear_acceleration.x;
	(m_est_vecs.m_cfImuAcc)(1) = msg.linear_acceleration.y;
	(m_est_vecs.m_cfImuAcc)(2) = msg.linear_acceleration.z;
	
	if(isFirstAccIMU){
		//resetaccController(&m_est_vecs.m_cfImuAcc);
		isFirstAccIMU = false;
	}

	msg.angular_velocity.x = degToRad(data->gyro_x);
	msg.angular_velocity.y = degToRad(data->gyro_y);
	msg.angular_velocity.z = degToRad(data->gyro_z);
	
	m_pubImu.publish(msg);
  }
void Swarm_Controller::onAttitude(uint32_t time_in_ms, logStablizer* data)
{
	(m_est_vecs.m_att_est)(0)   =  degToRad(data->roll);
	(m_est_vecs.m_att_est)(1) =  -degToRad(data->pitch);
	(m_est_vecs.m_att_est)(2) =  degToRad(data->yaw);
	printf("-------------att %f \n",(m_est_vecs.m_att_est)(0));
	if(isFirstAttEst){
		isFirstAttEst = false;
	}
}

void Swarm_Controller::sendSp()
{
	m_cf->sendSetpoint(   -v_posctrl_output(0) * RAD2DEG
						, - v_posctrl_output(1) * RAD2DEG
						, v_posctrl_output(2) * RAD2DEG
						, v_posctrl_output(3) * 65000); 
		m_sentSetpoint = true;
}

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

			m_recording->Rec_velEst(0) = vx_temp_est;
			m_recording->Rec_velEst(1) = vy_temp_est;
			m_recording->Rec_velEst(2) = vz_temp_est;

			//m_posEstPub.publish(posestMsg);
			vel_Sp(0) =  m_pidX.pp_update(x_temp_est , x_sp)*0.7f + (*Vel_ff)(0)*0.3f; //+ff
			vel_Sp(1) =  m_pidY.pp_update(y_temp_est , y_sp)*0.7f + (*Vel_ff)(1)*0.3f;
			vel_Sp(2) =  m_pidZ.pp_update(z_temp_est , z_sp)*0.7f + (*Vel_ff)(2)*0.3f;

			float vx_sp = vel_Sp(0);
			float vy_sp = vel_Sp(1);
			float vz_sp = vel_Sp(2);
			
			//easyfly::pos_ctrl_sp posSpmsg;
			m_recording->Rec_velSp(0) = vx_sp;
			m_recording->Rec_velSp(1) = vy_sp;
			m_recording->Rec_velSp(2) = vz_sp;

			l_velsp = vel_Sp;

			_acc_Sp_W(0) =  m_pidX.pid_update(vx_temp_est,vel_Sp(0),dt);
			_acc_Sp_W(1) =  m_pidY.pid_update(vy_temp_est,vel_Sp(1),dt);
			_acc_Sp_W(2) =  m_pidZ.pid_update(vz_temp_est,vel_Sp(2),dt);

			_acc_Sp_W(0) = 0.7f*_acc_Sp_W(0) + (*acc_Sp)(0)*0.3f;
			_acc_Sp_W(1) = 0.7f*_acc_Sp_W(1) + (*acc_Sp)(1)*0.3f;
			_acc_Sp_W(2) = 0.7f*_acc_Sp_W(2) + (*acc_Sp)(2)*0.3f;

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
			(*Output)(2) = 0;  //yaw rate
			(*Output)(0) = -(*Output)(0);
			//(*Output)(2) = RPY_des(2);
			Vector3f temp;
			temp.setZero();
			for(int i=0;i<3;i++){
				temp(i) = _R_des(i,2);
			}

			//easyfly::att_est AttMsg;
			m_recording->Rec_roll_sp = x_sp;
			m_recording->Rec_pitch_sp =y_sp;
			m_recording->Rec_yaw_sp = z_sp;
			//m_attSpPub.publish(AttMsg);

			float thrust_force = vec3f_dot(&_acc_Sp_W,&temp);

			thrust_force /= 470.0f;
			thrust_force = std::min(thrust_force,max_thrust);
			(*Output)(3) = thrust_force;		

		}
	}
/*int main(int argc, char **argv)
{
	char node_name[50];
	ros::init(argc, argv, "Swarm_Controller");
	ros::NodeHandle n("~");

	n.getParam("/joy_num", g_joy_num);
	Swarm_Controller Swarm_Controller(n);

	Swarm_Controller.run(100);
	return 0;
}*/