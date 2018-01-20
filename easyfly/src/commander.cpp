#include "ros/ros.h"
#include <stdio.h> //sprintf
#include <iostream>
#include <vector>
#include <sensor_msgs/Joy.h>
//#include <sensor_msgs/Imu.h>
//#include <sensor_msgs/MagneticField.h>
//#include <sensor_msgs/>
#include <tf/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <easyfly/commands.h>
#include <easyfly/pos_ctrl_sp.h>
#include <easyfly/raw_ctrl_sp.h>
#include <easyfly/trj_ctrl_sp.h>
#include <easyfly/pos_est.h>	
#include <easyfly/att_est.h>
#include <vicon_bridge/Markers.h>
#include <vicon_bridge/Marker.h>
#include "commons.h"
#include "Eigen/Eigen/Eigen"
#include "Eigen/Eigen/Geometry"
#include "type_methode.h"
//#include "gl_declair.h"
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>


int g_vehicle_num=2;
int g_joy_num=1;
using namespace Eigen;
using namespace std;
using namespace cv;
#define _USE_MATH_DEFINES //PI
/*************YE Xin gl*/
vector<float> x_init_pos;
vector<float> y_init_pos;
vector<int> index_sequence;
void give_index(int index)
{
	index_sequence.push_back(index);
}
void clear_index()
{
	index_sequence.clear();
}
/*************YE Xin gl*/
class Commander
{
private:
	std::vector<ros::Publisher> m_rawpub_v, m_pospub_v, m_trjpub_v, m_pos_est_v;
	ros::Publisher m_cmdpub;
	ros::Subscriber m_viconMarkersub;
	std::vector<ros::Subscriber> m_joysub_v, m_estsub_v, m_estyawsub_v;
	std::vector<vicon_bridge::Marker> m_markers;

	bool takeoff_switch;
	bool takeoff_condition;
	float takeoff_objective_height;
	float takeoff_safe_distance;
	float takeoff_low_rate;
	float takeoff_high_rate;
	float m_landspeed;
	float m_landsafe;
	std::vector<float> m_thetaPos, m_radius_Pos;
	float _dt_deriv;
	int m_flight_mode;
	bool reset_takeoff,isFirstVicon;
	float m_takeoff_switch_Auto;
	float m_takeoff_switch_Hover;
	float m_land_switch_idle;
	bool isGotAtt, isGotPos, isFirstPos, isFirsrAtt, isFirstCircling;

	//For sequence initialization
	Mat src = Mat(Size(1000,1000), CV_8UC3, Scalar(0));

	//MODE_RAW 0
	//MODE_POS 1
	//MODE_TRJ 2
	int m_flight_state;
	int m_hovering_numb;
//	float m_velff_xy_P, m_velff_z_P;
	struct M_Joy
	{
		bool curr_buttons[14];
		bool changed_buttons[14];
		float axes[4];
		int curr_arrow[2];
		bool changed_arrow[2];
	};
	std::vector<M_Joy> m_joy_v;
	struct M_Ctrl
	{
		easyfly::pos_ctrl_sp posctrl_msg;
		easyfly::raw_ctrl_sp rawctrl_msg;
		easyfly::trj_ctrl_sp trjctrl_msg;
	};
	std::vector<M_Ctrl> m_ctrl_v;
	std::vector<easyfly::pos_est> m_est_v;
	std::vector<Vector3f> m_att_est_v;
	std::vector<Vector3f> swarm_pos;
	easyfly::commands m_cmd_msg;
	std::vector<Vector3f> _takeoff_Pos;
	Vector3f m_vel_ff, m_acc_sp,  m_last_rateSp;
	easyfly::pos_est m_pos_estmsg;

public:
	Commander(ros::NodeHandle& nh)
	:m_rawpub_v(g_vehicle_num)
	,m_pospub_v(g_vehicle_num)
	,m_trjpub_v(g_vehicle_num)
	,m_joysub_v(g_joy_num)
	,m_estsub_v(g_vehicle_num)
	,m_estyawsub_v(g_vehicle_num)
	,m_joy_v(g_joy_num)
	,m_ctrl_v(g_vehicle_num)
	,m_est_v(g_vehicle_num)
	,m_att_est_v(g_vehicle_num)
	,m_pos_est_v(g_vehicle_num)
	,swarm_pos(0)
	,m_cmd_msg()
	,takeoff_switch(false)
	,takeoff_condition(true)
	,isGotAtt(false)
	,isFirstVicon(true)
	,isGotPos(false)
	,isFirstPos(true)
	,isFirsrAtt(true)
	,isFirstCircling(true)
	,takeoff_objective_height(1.2f)
	,takeoff_safe_distance(0.0f)
	,takeoff_low_rate(0.2f)
	,takeoff_high_rate(0.3f)
	,reset_takeoff(false)
	,m_landspeed(0.2f)
	,m_thetaPos(0)
	,m_land_switch_idle(0.1f)
	,m_takeoff_switch_Auto(0.2f)
	,m_takeoff_switch_Hover(0.2f)
	,m_hovering_numb(0)
	
	{
		m_vel_ff.setZero();
		m_acc_sp.setZero();
		//_takeoff_Pos.setZero();
		m_last_rateSp.setZero();
		m_flight_state = Idle;
//		m_velff_xy_P = 0.6;
//		m_velff_z_P = 0.5;
		char msg_name[50];
		m_cmd_msg.cut = 0;
		m_cmd_msg.flight_state = Idle;
		m_cmd_msg.l_flight_state = Idle;
		m_cmdpub = nh.advertise<easyfly::commands>("/commands",1);
		for(int i=0;i<g_vehicle_num;i++){
			sprintf(msg_name,"/vehicle%d/raw_ctrl_sp",i);
			m_rawpub_v[i] = nh.advertise<easyfly::raw_ctrl_sp>(msg_name, 1);
			sprintf(msg_name,"/vehicle%d/pos_ctrl_sp",i);
			m_pospub_v[i] = nh.advertise<easyfly::pos_ctrl_sp>(msg_name, 1);
			sprintf(msg_name,"/vehicle%d/trj_ctrl_sp",i);
			m_trjpub_v[i] = nh.advertise<easyfly::trj_ctrl_sp>(msg_name, 1);
			sprintf(msg_name,"/vehicle%d/att_est",i);
			m_estyawsub_v[i] = nh.subscribe<easyfly::att_est>(msg_name,5,boost::bind(&Commander::att_estCallback, this, _1, i));

			m_viconMarkersub = nh.subscribe<vicon_bridge::Markers>("/vicon/markers",5,&Commander::vicon_markerCallback, this);

			sprintf(msg_name,"/vehicle%d/pos_est", i); 
			m_pos_est_v[i] = nh.advertise<easyfly::pos_est>(msg_name, 5);
		}
		for(int i=0;i<g_joy_num;i++){
			sprintf(msg_name,"/joygroup%d/joy",i);
			m_joysub_v[i] = nh.subscribe<sensor_msgs::Joy>(msg_name,5,boost::bind(&Commander::joyCallback, this, _1, i));
		}
		
	}
	void run(double frequency)
	{
		ros::NodeHandle node;
		node.getParam("flight_mode", m_flight_mode);

		char msg_name[50];
		/*for(int i=0;i<g_vehicle_num;i++){
			
		}*/
		ros::Timer timer = node.createTimer(ros::Duration(1.0/frequency), &Commander::iteration, this);
		ros::spin();
	}
	void iteration(const ros::TimerEvent& e)
	{	

		static float time_elapse = 0;
		_dt_deriv = e.current_real.toSec() - e.last_real.toSec();
		time_elapse += _dt_deriv;

		//cut off
		if(m_joy_v[0].curr_buttons[4] == 1 && m_joy_v[0].curr_buttons[5] == 1){
			m_cmd_msg.cut = 1;
		}
		else{
			switch(m_flight_mode){
				case MODE_RAW:{
					
					for(int i=0; i<g_vehicle_num;i++){
						m_ctrl_v[i].rawctrl_msg.raw_att_sp.x = -m_joy_v[0].axes[0] * 30 * DEG2RAD;//+-1
						m_ctrl_v[i].rawctrl_msg.raw_att_sp.y = m_joy_v[0].axes[1] * 30 * DEG2RAD;
						m_ctrl_v[i].rawctrl_msg.raw_att_sp.z = m_joy_v[0].axes[2] * 20 * DEG2RAD;//rate
						m_ctrl_v[i].rawctrl_msg.throttle = m_joy_v[0].axes[3];//0-1
						if(m_ctrl_v[i].rawctrl_msg.throttle<0){
							m_ctrl_v[i].rawctrl_msg.throttle=0;
						}
						m_rawpub_v[i].publish(m_ctrl_v[i].rawctrl_msg);
					}
				}
				break;
				case MODE_POS:{

					if(m_joy_v[0].changed_arrow[1] == false){ 
						//faire rien..
					}

					if(m_joy_v[0].changed_arrow[1] == true && m_joy_v[0].curr_arrow[1] == 1){//take off
						m_joy_v[0].changed_arrow[1] = false;
						if(m_flight_state == Idle){
							for(int i=0;i<g_vehicle_num;i++){
								posspReset(i);
								yawspReset(i);
							}
							m_flight_state = TakingOff;
							//printf("hello!!!!!run\n");
						}
					}	
					else if(m_joy_v[0].changed_arrow[1] == true && m_joy_v[0].curr_arrow[1] == -1){//land
						printf("landing!!\n");
						m_joy_v[0].changed_arrow[1] = false;
						if(m_flight_state == Hovering)
							m_flight_state = Landing;
					}
					
					switch(m_flight_state){
						case Idle:{
							for(int i=0;i<g_vehicle_num;i++){
							}
							//all motors off
						}
						break;
						case Automatic:{
							for(int i=0;i<g_vehicle_num;i++){
								float pos_move_rate[3];
								pos_move_rate[0] = m_joy_v[i].axes[0] * 2.0;
								pos_move_rate[1] = m_joy_v[i].axes[1] * 2.0;
								pos_move_rate[2] = m_joy_v[i].axes[3] * 1.0;
								m_ctrl_v[i].posctrl_msg.pos_sp.x += pos_move_rate[0];
								m_ctrl_v[i].posctrl_msg.pos_sp.y += pos_move_rate[1];
								m_ctrl_v[i].posctrl_msg.pos_sp.z += pos_move_rate[2];
								m_ctrl_v[i].posctrl_msg.vel_ff.x = pos_move_rate[0];
								m_ctrl_v[i].posctrl_msg.vel_ff.y = pos_move_rate[1];
								m_ctrl_v[i].posctrl_msg.vel_ff.z = pos_move_rate[2];
								float yaw_move_rate = m_joy_v[i].axes[2] * 20 * DEG2RAD;
								m_ctrl_v[i].posctrl_msg.yaw_sp += yaw_move_rate;
								m_pospub_v[i].publish(m_ctrl_v[i].posctrl_msg);

							}
						}
						break;
						case TakingOff:{
							//printf("%d  %d\n",isGotPos, isGotAtt);
							
							if(isGotPos && isGotAtt){
								for(int i=0;i<g_vehicle_num;i++){
									command_takeoff(i,_dt_deriv);
									//printf("pos sp commander: %f  %f  %f\n",m_ctrl_v[i].posctrl_msg.pos_sp.x,m_ctrl_v[i].posctrl_msg.pos_sp.y,m_ctrl_v[i].posctrl_msg.pos_sp.z );
									m_pospub_v[i].publish(m_ctrl_v[i].posctrl_msg);
								//printf("%f\n", m_ctrl_v[i].posctrl_msg.acc_sp.z); 
							}
							//TODO judge if it is time to get into Automatic
						}
						}
						break;
						case Landing:{
							for(int i=0;i<g_vehicle_num;i++){
								control_landing(i, _dt_deriv);
								//m_pospub_v[i].publish(m_ctrl_v[i].posctrl_msg);
							}
							//TODO judge if it is time to get into Idle
						}
						break;
						case Hovering:{
							printf("hovering!!\n");
							for (int i=0;i<g_vehicle_num;i++)
							{
								if (m_thetaPos[i] >= 50*3.14)
								{	
									m_flight_state = Landing;
								}
							}
							
							if (m_flight_state != Landing)
							{
								for(int i=0;i<g_vehicle_num;i++){
									//posspReset(i);
									//yawspReset(i);
									if(m_hovering_numb<10000)
									{
										m_ctrl_v[i].posctrl_msg.pos_sp.z = takeoff_objective_height;
										m_hovering_numb+=1;
									}
									if(m_hovering_numb == 10000)
									{
										m_flight_state = Circling;
										m_hovering_numb == 0;
									}
									/*if (m_ctrl_v[i].posctrl_msg.pos_sp.x<0.5){
										m_ctrl_v[i].posctrl_msg.pos_sp.x += 0.2*_dt_deriv;
									}*/
									m_pospub_v[i].publish(m_ctrl_v[i].posctrl_msg);
								}
							}	
							
						}
						break;
						case Circling:{
							printf("begin circling!!\n");
							for(int i=0;i<g_vehicle_num;i++){
								command_circling(i,_dt_deriv);
								//m_pospub_v[i].publish(m_ctrl_v[i].posctrl_msg);
							}
						}
					}//end switch state
				}//end case posctrl mode
				break;
				case MODE_TRJ:{
					
				}
				break;
				default:
				break;
			}//end switch mode
		}//end of cut off case
		m_cmd_msg.flight_state = m_flight_state;
		m_cmdpub.publish(m_cmd_msg);
		m_cmd_msg.l_flight_state = m_flight_state;
	}

	void posspReset(int index)
	{
		m_ctrl_v[index].posctrl_msg.pos_sp.x = m_est_v[index].pos_est.x;
		m_ctrl_v[index].posctrl_msg.pos_sp.y = m_est_v[index].pos_est.y;
		m_ctrl_v[index].posctrl_msg.pos_sp.z = m_est_v[index].pos_est.z;

		m_ctrl_v[index].posctrl_msg.vel_ff.x = 0.0f;
		m_ctrl_v[index].posctrl_msg.vel_ff.y = 0.0f;
		m_ctrl_v[index].posctrl_msg.vel_ff.z = 0.0f;
		m_last_rateSp.setZero();

		m_ctrl_v[index].posctrl_msg.acc_sp.x = 0.0f;
		m_ctrl_v[index].posctrl_msg.acc_sp.y = 0.0f;
		m_ctrl_v[index].posctrl_msg.acc_sp.z = 0.0f;

	}
	void yawspReset(int index)
	{
		m_ctrl_v[index].posctrl_msg.roll_sp = (m_att_est_v[index])(0);
		m_ctrl_v[index].posctrl_msg.pitch_sp = (m_att_est_v[index])(1);
		m_ctrl_v[index].posctrl_msg.yaw_sp = (m_att_est_v[index])(2);
		//m_ctrl_v[index].posctrl_msg.pitch_sp = m_att_est_v[index].Pitch_est;
		//m_ctrl_v[index].posctrl_msg.yaw_sp = m_att_est_v[index].Yaw_est;

	}
	void joyCallback(const sensor_msgs::Joy::ConstPtr& joy, int joy_index)
	{
		//0 PosCtrl, 1 AttCtrl
		joy_index=0;
		#define MAX_JOYS 5
		static bool l_buttons[MAX_JOYS][14];//at most 5 joysticks
		static int l_arrow[MAX_JOYS][2];
		for(int i=0;i<14;i++){
			m_joy_v[joy_index].curr_buttons[i] = joy->buttons[i];
			if(m_joy_v[joy_index].curr_buttons[i] != l_buttons[joy_index][i])
				m_joy_v[joy_index].changed_buttons[i] = true;
			else
				;//changed_buttons cleared in iteration
		}
		for(int i=0;i<14;i++){
			l_buttons[joy_index][i] = m_joy_v[joy_index].curr_buttons[i];
		}
										
		m_joy_v[joy_index].axes[0] = joy->axes[3];//roll
		m_joy_v[joy_index].axes[1] = -joy->axes[4];//pitch
		m_joy_v[joy_index].axes[2] = -joy->axes[0];//yaw
		m_joy_v[joy_index].axes[3] = joy->axes[1];//thr
		if(joy->axes[6]>0.5)//left and right button belong to one axes
			m_joy_v[joy_index].curr_arrow[0] = 1;
		else if(joy->axes[6]<-0.5)
			m_joy_v[joy_index].curr_arrow[0] = -1;
		else
			m_joy_v[joy_index].curr_arrow[0] = 0;
		if(joy->axes[7]>0.5)//up and down button is belong to one axes
			m_joy_v[joy_index].curr_arrow[1] = 1;
		else if(joy->axes[7]<-0.5)
			m_joy_v[joy_index].curr_arrow[1] = -1;
		else
			m_joy_v[joy_index].curr_arrow[1] = 0;
		for(int i=0;i<2;i++){
			if(m_joy_v[joy_index].curr_arrow[i] != l_arrow[joy_index][i])
				m_joy_v[joy_index].changed_arrow[i]=true;
		}
		for(int i=0;i<2;i++){
			l_arrow[joy_index][i] = m_joy_v[joy_index].curr_arrow[i];
		}
	}

	//For sequence intialization
	void displayFunc()
	{
		namedWindow("vicon_test");	
		Point p1 = Point(50,50);
		Point p2 = Point(950,950);
		rectangle(src, p1, p2, CV_RGB(0, 0, 255), -1);

		for(int i=0;i<x_init_pos.size();i++){
	    	circle(src, Point(x_init_pos[i]*1000, y_init_pos[i]*1000), 2, Scalar(0, 255, 0));  
	    	printf("x%d: %f\n", i, x_init_pos[i]);
	    	printf("y%d: %f\n", i, y_init_pos[i]);
    	}	
		imshow("vicon_test", src);
	}

	/*void configDone()
	{

	}*/

	//For sequence intialization
	static void onMouse(int event, int x, int y, int, void* userInput)
	{
		if (event != EVENT_LBUTTONDOWN) return;
		Mat *img = (Mat*)userInput;
		circle(*img, Point(x, y), 10, Scalar(0, 0, 255));
		imshow("vicon_test", *img);

		float nearest_dist=-1.0f;
		int nearest_index=0;
		for(int i=0;i<x_init_pos.size();i++){
			float sq_dist=(x-x_init_pos[i]*1000)*(x-x_init_pos[i]*1000)+(y-y_init_pos[i]*1000)*(y-y_init_pos[i]*1000);
			if(sq_dist<nearest_dist||nearest_dist<0){
				nearest_dist=sq_dist;
				nearest_index=i;
			}
		}
		give_index(nearest_index);
	}

	void vicon_markerCallback(const vicon_bridge::Markers::ConstPtr& msg)
	{	
		m_markers = msg->markers;
		
		if(isFirstVicon && msg->markers.size()==g_vehicle_num)
		{	
			printf("received vicon_bridge\n");
			//printf("%d\n",msg->markers.size());
			for (auto& Marker : m_markers)
    		{		
    			printf("hhhh");
    			Vector3f pos;
    			pos(0) = Marker.translation.x/1000.0f;
    			pos(1) = Marker.translation.y/1000.0f;
    			pos(2) = Marker.translation.z/1000.0f;
    			swarm_pos.push_back(pos);
/*************YE Xin gl*/
    			x_init_pos.push_back(pos(0));
				y_init_pos.push_back(pos(1));
				printf("x pos: %f\n", pos(0));
		    	printf("y pos: %f\n", pos(1));
		    	fflush(stdout);
			//	printf("\nx_command%d: %f\n", x_init_pos[]);
    		//	printf("y_command%d: %f\n", y_init_pos[i]);
/*************YE Xin gl*/
    			//resetposController(&m_pos_est);
    		}
    		for(int i=0;i<x_init_pos.size();i++){
		    	printf("x init%d: %f\n", i, x_init_pos[i]);
		    	printf("y init%d: %f\n", i, y_init_pos[i]);
		    	fflush(stdout);
		    }
    		if(swarm_pos.size()!=g_vehicle_num)
    		{
    			if(swarm_pos.size()>g_vehicle_num)
    			{
    				printf("More vehicles than actually have!! Warning!!\n");
    			}

    			else if(swarm_pos.size()<g_vehicle_num)
    			{
    				printf("Less vehicles than actually have!! Warning!!\n");
    			}
   
    		}
    		else
    		{
    			for (int i=0;i<swarm_pos.size();i++)
    			{
    				_takeoff_Pos.push_back(swarm_pos[i]);
    			}
    			isFirstVicon = false;
    			/*glutInitDisplayMode(GLUT_RGB | GLUT_SINGLE);// 设置显示模式(RGB颜色|单缓冲，对应的还有索引颜色和双缓冲)  
    			glutInitWindowPosition(100, 100);// 设置窗口的默认显示位置  
    			glutInitWindowSize(400, 400);// 设置窗口的大小  
    			glutCreateWindow("Crazyflie Identifier");// 设置窗口的标题  
				glutMouseFunc(MouseFunc);
				glutDisplayFunc(&displayFunc);// 设置绘图函数  
				glutMainLoop();*/
				/*Sequence initialization*/
				bool sequenceIsOk = false;
				while(!sequenceIsOk)
				{
					displayFunc();
					setMouseCallback("vicon_test", onMouse, &src);
					waitKey();
					destroyWindow("vicon_test");
					//printf("%d\n", index_sequence.size());
					/*check the click times and exit the initialization*/
					if(index_sequence.size()==g_vehicle_num){
						sequenceIsOk = true;
					}else{
						printf("Initialization fails!! Please click again!!\n");
						clear_index();
					}
				}
				for (int i = 0; i < index_sequence.size(); ++i)
				{
					//printf("%d\n", 100);
					int tmp_index = index_sequence[i];
					printf("%d", tmp_index);
					Vector3f tmp;
					tmp = swarm_pos[index_sequence[i]];
					printf("%f %f\n", tmp(0), tmp(1));
					fflush(stdout);
				}
    		}
    		
    		
		}
		if(!isFirstVicon && msg->markers.size()==g_vehicle_num)
		{
			for (auto& Marker : m_markers)
    		{	
    			Vector3f pos;
    			pos(0) = Marker.translation.x/1000.0f;
    			pos(1) = Marker.translation.y/1000.0f;
    			pos(2) = Marker.translation.z/1000.0f;
    			//printf("%f   %f   %f\n", pos(0),pos(1),pos(2));
    			for (int i=0;i<g_vehicle_num;i++)
    			{
    				Vector3f v_difference = pos-swarm_pos[i];
    				float norm;
    				vec3f_norm(&v_difference, norm);
    				if(  norm < MAX_VELOCITY*MAX_VELOCITY*0.02*0.02) //max circle
    				{
    					swarm_pos[i] = pos;
    				}
    			}
    		}
    		for (int i=0;i<g_vehicle_num;i++)
    		{
    			Vector3f pos = swarm_pos[i];
    			m_pos_estmsg.pos_est.x = pos(0);
				m_pos_estmsg.pos_est.y = pos(1);
				m_pos_estmsg.pos_est.z = pos(2);
				m_pos_estmsg.vehicle_index = i;
/*************YE Xin gl*/
				m_pos_est_v[index_sequence[i]].publish(m_pos_estmsg);
/*************YE Xin gl*/
    		}	
    	}
    	
	}
		
	/*void pos_estCallback(const easyfly::pos_est::ConstPtr& est, int vehicle_index)
	{
		if(isFirstPos){
			isGotPos = true;
			_takeoff_Pos(0) = est->pos_est.x;
			_takeoff_Pos(1) = est->pos_est.y;
			_takeoff_Pos(2) = est->pos_est.z;
			isFirstPos = false;
		}
		
		m_est_v[vehicle_index].pos_est.x = est->pos_est.x;
		m_est_v[vehicle_index].pos_est.y = est->pos_est.y;
		m_est_v[vehicle_index].pos_est.z = est->pos_est.z;
		//printf("%f    %f    %f\n", est->pos_est.x, est->pos_est.y, est->pos_est.z);
		/*m_est_v[vehicle_index].vel_est.x = est->vel_est.x;
		m_est_v[vehicle_index].vel_est.y = est->vel_est.y;
		m_est_v[vehicle_index].vel_est.z = est->vel_est.z;
	}*/
	void att_estCallback(const easyfly::att_est::ConstPtr& est, int vehicle_index)
	{
		//m_att_est_v[vehicle_index].Roll_est = est->Roll_est;
		isGotAtt = true;
		(m_att_est_v[vehicle_index])(0) = est->att_est.x;
		(m_att_est_v[vehicle_index])(1) = est->att_est.y;
		(m_att_est_v[vehicle_index])(2) = est->att_est.z;
		//printf("ROLL_GET:  %f   %f \n",est->att_est.x, (m_att_est_v[vehicle_index])(0));

	}
	void command_takeoff(int i, float dt)
	{
		reset_takeoff = false;
//		g_statusFlight = statusTakingoff;		
		m_ctrl_v[i].posctrl_msg.pos_sp.x = _takeoff_Pos[i](0);
		m_ctrl_v[i].posctrl_msg.pos_sp.y = _takeoff_Pos[i](1);
		yawspReset(i);

		//printf("pos sp commander: %f  %f  %f\n",m_ctrl_v[i].posctrl_msg.pos_sp.x,m_ctrl_v[i].posctrl_msg.pos_sp.y,m_ctrl_v[i].posctrl_msg.pos_sp.z );
		if(m_ctrl_v[i].posctrl_msg.pos_sp.z >= _takeoff_Pos[i](2) + takeoff_objective_height)
		{
			takeoff_switch = false;
			m_ctrl_v[i].posctrl_msg.pos_sp.z = _takeoff_Pos[i](2) + takeoff_objective_height;
		}
		else
		{
	    	if(takeoff_condition) //in case of a sudden thrust at the beggining..
	    	{
	    		//_takeoff_Pos(2) = m_est_v[i].pos_est.z; //reset the takingoff height
	    		m_ctrl_v[i].posctrl_msg.pos_sp.z = _takeoff_Pos[i](2) - takeoff_safe_distance;
	    		takeoff_condition = false;
	    	}
	    	else
	    	{
	    		if(m_ctrl_v[i].posctrl_msg.pos_sp.z < _takeoff_Pos[i](2) + 0.2f)
	    		{
	    			m_ctrl_v[i].posctrl_msg.pos_sp.z += takeoff_low_rate * dt;

	    			m_ctrl_v[i].posctrl_msg.vel_ff.z = takeoff_low_rate;

	    			m_ctrl_v[i].posctrl_msg.acc_sp.z = deriv_f(takeoff_low_rate, m_last_rateSp(2), dt);
	    			m_last_rateSp(2) = takeoff_low_rate;
	    		}
	    		else
	    		{
	    			printf("hello,high speed!\n");
	    			m_ctrl_v[i].posctrl_msg.pos_sp.z += takeoff_high_rate * dt;

	    			m_ctrl_v[i].posctrl_msg.vel_ff.z = takeoff_high_rate;

	    			m_ctrl_v[i].posctrl_msg.acc_sp.z = deriv_f(takeoff_high_rate, m_last_rateSp(2), dt);
	    			m_last_rateSp(2) = takeoff_high_rate;
	    		}
	    		/*if(m_est_v[i].pos_est.z < takeoff_objective_height + m_takeoff_switch_Auto)
	    		{
	    			m_flight_state = Automatic;
	    		}*/
	    		printf("%f \n", m_est_v[i].pos_est.z);
	    		if(m_est_v[i].pos_est.z > takeoff_objective_height - m_takeoff_switch_Hover)// + m_takeoff_switch_Hover)
	    		{
	    			printf("HOVERING\n");
	    			m_flight_state = Hovering;
	    		}
	      	}
	    }
	}

	void command_circling(int i, float dt)
	{
		if (isFirstCircling)
		{
			for(int i=0;i<swarm_pos.size();i++)
			{	
				m_thetaPos[i] = atan2(_takeoff_Pos[i](1),_takeoff_Pos[i](0)); 
				m_radius_Pos[i] = sqrt(_takeoff_Pos[i](1)*_takeoff_Pos[i](1) + _takeoff_Pos[i](0)*_takeoff_Pos[i](0));
			}
			isFirstCircling = false;
		}
		else
		{
			bool isReadyToCircle = false;
			float circle_err = 0.1f;
			float stretch_step = 0.05f;
			for (int i=0;i<g_vehicle_num;i++)
			{
				if((m_radius_Pos[i]-CIRCLING_R)*(m_radius_Pos[i]-CIRCLING_R)>circle_err)
				{
					m_ctrl_v[i].posctrl_msg.pos_sp.x += stretch_step*cos(m_thetaPos[i]);
					m_ctrl_v[i].posctrl_msg.pos_sp.y += stretch_step*sin(m_thetaPos[i]);
					m_ctrl_v[i].posctrl_msg.pos_sp.z = takeoff_objective_height;	
				}
				else
				{	
					printf("ready to circling!!\n");
					isReadyToCircle = true;
				}		
			}
			if(isReadyToCircle)
			{
				for(int i=0;i<swarm_pos.size();i++)
				{
					m_thetaPos[i] += 3.1415926/200.0f;
					m_ctrl_v[i].posctrl_msg.pos_sp.x = (m_radius_Pos[i])*cos(m_thetaPos[i]);
					m_ctrl_v[i].posctrl_msg.pos_sp.y = (m_radius_Pos[i])*sin(m_thetaPos[i]);
					m_ctrl_v[i].posctrl_msg.pos_sp.z = takeoff_objective_height;
					printf("%f\n",m_thetaPos[i]);
				}
			}
			
		}			
		
	}
	void control_landing(int i, float dt)
	{
		reset_takeoff = false;
		m_ctrl_v[i].posctrl_msg.pos_sp.x = m_est_v[i].pos_est.x;
		m_ctrl_v[i].posctrl_msg.pos_sp.y = m_est_v[i].pos_est.y;
		m_ctrl_v[i].posctrl_msg.pos_sp.z -= m_landspeed * dt;
		if (m_est_v[i].pos_est.z<=_takeoff_Pos[i](2) + m_land_switch_idle)
		{
			m_flight_state = Idle;
		}
	}
};

int main(int argc, char **argv)
{
//  int ret = init_scan(argc, argv);
	//glutInit(&argc, argv);
	ros::init(argc, argv, "commander");
	ros::NodeHandle n;
	// ros::NodeHandle n;
	n.getParam("/g_vehicle_num", g_vehicle_num);
	n.getParam("/joy_num", g_joy_num);
//	n.getParam("/flight_mode", g_flight_mode);this has moved to function run
	Commander commander(n);
	commander.run(50);


  return 0;


}
