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
vector<float> x_marker_init;
vector<float> y_marker_init;
vector<int> index_sequence;
float amp_coeff;
float about_edge = 0.5;
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
	std::vector<Vector3f> swarm_pos_predict;//records prediction based on last time position, error, and velocity
	std::vector<Vector3f> swarm_pos_err;//records position error for correction
	std::vector<Vector3f> swarm_pos_step;//records step(only velocity) from last two positions.
	std::vector<Vector3f> m_swarm_pos;//the modifiable version of swarm_pos
	easyfly::commands m_cmd_msg;
	std::vector<Vector3f> _takeoff_Pos;
	Vector3f m_vel_ff, m_acc_sp,  m_last_rateSp;
	easyfly::pos_est m_pos_estmsg;

	float z_ground;

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
							
							if(!isFirstVicon && isGotAtt){
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
		float tmp_max = 0;
		for (int i = 0; i < x_init_pos.size(); ++i)
		{
			float tmp = sqrt(x_init_pos[i]*x_init_pos[i]+y_init_pos[i]*y_init_pos[i]);
			if (tmp_max < tmp)
				tmp_max = tmp;	
		}
		amp_coeff = 400.0f/tmp_max;                        
		printf("tmp_max : %f***********amp_coeff : %f\n", tmp_max, amp_coeff);   

		namedWindow("vicon_test");	
		Point p1 = Point(50,50);
		Point p2 = Point(950,950);
		rectangle(src, p1, p2, CV_RGB(0, 0, 255), -1);


		for(int i=0;i<x_marker_init.size();i++){
			circle(src, Point(500-y_marker_init[i]*amp_coeff, 500+x_marker_init[i]*amp_coeff), 2, Scalar(0, 255, 0));  
	    	printf("x1 %d: %f\n", i, x_marker_init[i]);
	    	printf("y1 %d: %f\n", i, y_marker_init[i]);
    	}


		for(int i=0;i<x_init_pos.size();i++){
			circle(src, Point(500-y_init_pos[i]*amp_coeff, 500+x_init_pos[i]*amp_coeff), 2, Scalar(0, 255, 0));  
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
			float sq_dist=(y-x_init_pos[i]*amp_coeff-500)*(y-x_init_pos[i]*amp_coeff-500)+(x+y_init_pos[i]*amp_coeff-500)*(x+y_init_pos[i]*amp_coeff-500);
			if(sq_dist<nearest_dist||nearest_dist<0){
				nearest_dist=sq_dist;
				nearest_index=i;
			}
		}
		give_index(nearest_index);
	}

		void unite()
	{
		vector<bool> all_union(x_marker_init.size(),0);
		for(int i=0;i<x_marker_init.size();++i)
		{
		    int within_circle = 0;
			for(int j=1;j<x_marker_init.size();++j)
			{
				if(sqrt((x_marker_init[i]-x_marker_init[j])*(x_marker_init[i]-x_marker_init[j])+(y_marker_init[i]-y_marker_init[j])*(y_marker_init[i]-y_marker_init[j]))<about_edge)
					++within_circle;
			}
			if(within_circle<3)
				all_union[i]=1;
		}
		for(int i=0;i<x_marker_init.size();++i)
		{
			if(all_union[i])continue;
			all_union[i]=1;
			vector<int> num_of_point(3,-1);
			vector<float> min_dstc(2,-1);
		    float temp_dstc;
		    for(int j=1;j<x_marker_init.size();++j)
		    {
		    	if(all_union[j])continue;
		    	temp_dstc=(x_marker_init[i]-x_marker_init[j])*(x_marker_init[i]-x_marker_init[j])+(y_marker_init[i]-y_marker_init[j])*(y_marker_init[i]-y_marker_init[j]);
		    	if(min_dstc[0]>temp_dstc||min_dstc[0]<0)
		    	{
		    		num_of_point[0]=j;
		    		min_dstc[0]=temp_dstc;
		    	}
		    }
		    all_union[num_of_point[0]]=1;
		    for(int j=1;j<x_marker_init.size();++j)
		    {
		    	if(all_union[j])continue;
		    	temp_dstc=(x_marker_init[i]-x_marker_init[j])*(x_marker_init[i]-x_marker_init[j])+(y_marker_init[i]-y_marker_init[j])*(y_marker_init[i]-y_marker_init[j]);
		    	if(min_dstc[1]>temp_dstc||min_dstc[1]<0)
		    	{
		    		num_of_point[1]=j;
		    		min_dstc[1]=temp_dstc;
		    	}
		    }
		    all_union[num_of_point[1]]=1;
		    temp_dstc=x_marker_init[num_of_point[0]]+x_marker_init[num_of_point[1]];
		    x_init_pos.push_back(temp_dstc/2);
		    temp_dstc=y_marker_init[num_of_point[0]]+y_marker_init[num_of_point[1]];
		    y_init_pos.push_back(temp_dstc/2);

		    float temp_cross_product=-9999;
		    for(int j=1;j<x_marker_init.size();++j)
		    {
		    	if(all_union[j])continue;
		    	float cross_product;
		    	cross_product=(x_marker_init[num_of_point[0]]-x_marker_init[j])*(x_marker_init[num_of_point[1]]-x_marker_init[j])+(y_marker_init[num_of_point[0]]-y_marker_init[j])*(y_marker_init[num_of_point[1]]-y_marker_init[j]);
		    	if(cross_product<temp_cross_product||temp_cross_product<0)
		    	{
		    		num_of_point[2]=j;
		    		temp_cross_product=cross_product;
		    	}
		    }
		    all_union[num_of_point[2]]=1;
		}
	}

	void vicon_markerCallback(const vicon_bridge::Markers::ConstPtr& msg)
	{	
		m_markers = msg->markers;
		
		if(isFirstVicon && msg->markers.size() != 0)
		{	
			for (auto& Marker : m_markers)
    		{		
    			//printf("hhhh");
    			Vector3f pos;
    			pos(0) = Marker.translation.x/1000.0f;
    			pos(1) = Marker.translation.y/1000.0f;
    			pos(2) = Marker.translation.z/1000.0f;

    			x_marker_init.push_back(pos(0));
				y_marker_init.push_back(pos(1));
				z_ground = 0.02;
			}
    			
    		unite();//identify crazyflies and get their position into swarm_pos

			bool sequenceIsOk = false;
			while(!sequenceIsOk)//use mouse to rearrange index of swarm_pos
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
			
			//printf("%d*****%d\n", index_sequence.size(), index_sequence[0]);

			for (int i=0; i<index_sequence.size();i++)
			{
				Vector3f tmp_pos;
				tmp_pos(0) = x_init_pos[index_sequence[i]];
				tmp_pos(1) = y_init_pos[index_sequence[i]];
				tmp_pos(2) = z_ground;
				swarm_pos.push_back(tmp_pos);
				m_swarm_pos.push_back(tmp_pos);
				_takeoff_Pos.push_back(tmp_pos);

	    		Vector3f tmp_zero;
	    		tmp_zero(0) = tmp_zero(1) = tmp_zero(2) = 0;
	    		swarm_pos_step.push_back(tmp_zero);
	    		swarm_pos_predict.push_back(tmp_zero);
	    		swarm_pos_err.push_back(tmp_zero); 
			}
			/*for (int i = 0; i < index_sequence.size(); ++i)//print index_sequence
			{
				//printf("%d\n", 100);
				int tmp_index = index_sequence[i];
				printf("%d", tmp_index);
				Vector3f tmp;
				tmp = swarm_pos[index_sequence[i]];
				printf("%f %f\n", tmp(0), tmp(1));
				fflush(stdout);
			}*/

    		if(swarm_pos.size()==g_vehicle_num)
    			isFirstVicon = false;
    	}
		else if(!isFirstVicon && msg->markers.size() != 0)
		{
			std::vector<Vector3f> consider_pos;//markers_pos;
			for (auto& Marker : m_markers)
	    	{	
	    		Vector3f pos;
	    		pos(0) = Marker.translation.x/1000.0f;
	    		pos(1) = Marker.translation.y/1000.0f;
	    		pos(2) = Marker.translation.z/1000.0f;
	    		consider_pos.push_back(pos);
	    	}

			/*grand wipe out*/
			/*std::vector<Vector3f> consider_pos;
			for (int i=0;i<markers_pos.size();i++)
	    	{	
	    		float norm;
	    		vec3f_norm(&markers_pos[i], &norm);
	    		bool isInside = true;
	    		for (int j=0;j<swarm_pos.size();j++)
	    		{
	    			float swarm_norm;
	    			vec3f_norm(&swarm_pos[j], &swarm_norm);
	    			
	    			if(fabs(norm-swarm_norm) > RADIUS_SQUARE) //max circle
						isInside = false;
	    		}
	    		if (isInside)
	    			consider_pos.push_back(markers_pos[i]);
	    	}*/
	    	printf("*******consider_pos size : %d\n", consider_pos.size());
			/*find vehicles*/
	    	for (int i = 0; i < g_vehicle_num; ++i)//for every vehicles
	    	{
	    		/*prediction*/
	    		swarm_pos_predict[i] = swarm_pos[i] + swarm_pos_step[i] + REVISE_WEIGHT*swarm_pos_err[i];
	    		/*small wipe out*/
	    		printf("swarm_pos_predict %d: %f %f %f\n",i, swarm_pos_predict[i](0), swarm_pos_predict[i](1), swarm_pos_predict[i](2));
	    		std::vector<Vector3f> close_points;
	    		for (int j = 0; j < consider_pos.size(); ++j)//for every considered points
	    		{
	    			Vector3f tmp_diff;
	    			float tmp_norm;	  
	    			tmp_diff(0) = consider_pos[j](0) - swarm_pos_predict[i](0);
	    			tmp_diff(1) = consider_pos[j](1) - swarm_pos_predict[i](1);
	    			tmp_diff(2) = consider_pos[j](2) - swarm_pos_predict[i](2);
	    			vec3f_norm(&tmp_diff, &tmp_norm);

	    			//printf("consider_pos: %f %f %f\n", consider_pos[j](0), consider_pos[j](1), consider_pos[j](2));
	    			//printf("consider_pos: %f %f %f\n", tmp_diff(0), tmp_diff(1), tmp_diff(2));
	    			//printf("*****inside radius : %f\n", tmp_norm);
	    			if (tmp_norm < VEHICLE_SIZE)
	    				close_points.push_back(consider_pos[j]);  			
	    		}//j
	    		//printf("**********close_points : %d\n", close_points.size());
	    		/*condition 1 to 4*/
	    		if (close_points.size() >= 4 && close_points.size() <= 4*g_vehicle_num) //condition 4
	    		{
	    			bool FoundVehicle_i = false;
					/*find vehicle center from close_points*/
		    		for (int j = 0; j < close_points.size(); ++j)
		    		{
		    			//printf("********** j : %d\n", j);
		    			//printf("FoundVehicle_i : %d\n", FoundVehicle_i);
		    			if (FoundVehicle_i)
		    				break;
		    			/*record all the vectors that based on points j*/
		    			std::vector<Vector3f> consider_vec;
		    			for (int k = 0; k < close_points.size(); ++k)
		    			{
		    				if (k != j)
		    				{
		    					Vector3f tmp_vec;
			    				tmp_vec(0) = close_points[k](0) - close_points[j](0);
			    				tmp_vec(1) = close_points[k](1) - close_points[j](1);
			    				tmp_vec(2) = close_points[k](2) - close_points[j](2);
			    				consider_vec.push_back(tmp_vec);
		    				}
		    			}
		    			/*count the number of right pairs*/
		    			for (int p = 0; p < consider_vec.size(); ++p)
		    			{
		    				//printf("********** p : %d\n", p);
		    				std::vector<Vector3f> swarm_pos_p;
		    				int count_p = 0;
		    				float len_p;
		    				vec3f_norm(&consider_vec[p], &len_p);
		    				for (int q = 0; q < consider_vec.size(); ++q)
		    				{
		    					if (q != p)
		    					{
		    						//printf("********** q : %d\n", q);
				    				float len_q;
				    				vec3f_norm(&consider_vec[q], &len_q);
				    				float ctheta = (consider_vec[p](0)*consider_vec[q](0)+consider_vec[p](1)*consider_vec[q](1)+consider_vec[p](2)*consider_vec[q](2))/(len_p*len_q);
			    					if (ctheta < 0.2 && len_q/len_p < 1.05 && len_q/len_p > 1.05)
			    					{
			    						//printf("condition 1\n");
			    						Vector3f tmp_pos;
			    						tmp_pos(0) = 0.5*(consider_vec[p](0) + consider_vec[q](0)) + close_points[j](0);
			    						tmp_pos(1) = 0.5*(consider_vec[p](1) + consider_vec[q](1)) + close_points[j](1);
			    						tmp_pos(2) = 0.5*(consider_vec[p](2) + consider_vec[q](2)) + close_points[j](2);
			    						swarm_pos_p.push_back(tmp_pos);
			    						count_p++;
			    					} else if (ctheta < 0.75 && ctheta > 0.65 && len_q/len_p < 1.45 && len_q/len_p > 1.35)
			    					{
			    						//printf("condition 2\n");
			    						Vector3f tmp_pos;
			    						tmp_pos(0) = 0.5*consider_vec[q](0) + close_points[j](0);
			    						tmp_pos(1) = 0.5*consider_vec[q](1) + close_points[j](1);
			    						tmp_pos(2) = 0.5*consider_vec[q](2) + close_points[j](2);
			    						swarm_pos_p.push_back(tmp_pos);
			    						count_p++;
			    					} else if (ctheta < 0.75 && ctheta > 0.65 && len_p/len_q < 1.45 && len_p/len_q > 1.35)
			    					{
			    						//printf("condition 3\n");
			    						Vector3f tmp_pos;
			    						tmp_pos(0) = 0.5*consider_vec[p](0) + close_points[j](0);
			    						tmp_pos(1) = 0.5*consider_vec[p](1) + close_points[j](1);
			    						tmp_pos(2) = 0.5*consider_vec[p](2) + close_points[j](2);
			    						swarm_pos_p.push_back(tmp_pos);
			    						count_p++;
			    					}
		    					}//if
		    				}//for q
		    				if (count_p == 2)
		    				{
		    					m_swarm_pos[i](0) = (swarm_pos_p[0](0) + swarm_pos_p[1](0))/2;
		    					m_swarm_pos[i](1) = (swarm_pos_p[0](1) + swarm_pos_p[1](1))/2;
		    					m_swarm_pos[i](2) = (swarm_pos_p[0](2) + swarm_pos_p[1](2))/2;
		    					FoundVehicle_i = true;
		    					break;
		    				} else if (count_p == 1)
		    				{
		    					m_swarm_pos[i] = swarm_pos_p[0];
		    					FoundVehicle_i = true;
		    					break;
		    				}		
		    			}//for p
		    		}//for j
		    		if (!FoundVehicle_i)
    				{
    				printf("*****condition 4 failed! failure number : 1\n");
    				}
	    		}else if (close_points.size() == 3) //condition 3
	    		{
	    			//printf("*****condition 3\n");
	    			Vector3f tmp_vec_1;
	    			float tmp_len_1;
	    			tmp_vec_1(0) = close_points[1](0) - close_points[0](0);
	    			tmp_vec_1(1) = close_points[1](1) - close_points[0](1);
	    			tmp_vec_1(2) = close_points[1](2) - close_points[0](2);
	    			vec3f_norm(&tmp_vec_1, &tmp_len_1);

	    			Vector3f tmp_vec_2;
	    			float tmp_len_2;
	    			tmp_vec_2(0) = close_points[2](0) - close_points[0](0);
	    			tmp_vec_2(1) = close_points[2](1) - close_points[0](1);
	    			tmp_vec_2(2) = close_points[2](2) - close_points[0](2);
	    			vec3f_norm(&tmp_vec_2, &tmp_len_2);
	    			
	    			float ctheta = (tmp_vec_1(0)*tmp_vec_2(0)+tmp_vec_1(1)*tmp_vec_2(1)+tmp_vec_1(2)*tmp_vec_2(2))/(tmp_len_1*tmp_len_2);
	    			if (ctheta < 0.75 && ctheta > 0.65)
	    			{
	    				if (tmp_len_2/tmp_len_1 < 1.45 && tmp_len_2/tmp_len_1 > 1.35)
	    				{
	    					Vector3f tmp;
			    			tmp(0) = 0.5*(close_points[2](0) + close_points[0](0));
			    			tmp(1) = 0.5*(close_points[2](1) + close_points[0](1));
			    			tmp(2) = 0.5*(close_points[2](2) + close_points[0](2));	
			    			m_swarm_pos[i] = tmp;
	    				} else if (tmp_len_1/tmp_len_2 < 1.45 && tmp_len_1/tmp_len_2 > 1.35)
	    				{
	    					Vector3f tmp;
			    			tmp(0) = 0.5*(close_points[1](0) + close_points[0](0));
			    			tmp(1) = 0.5*(close_points[1](1) + close_points[0](1));
			    			tmp(2) = 0.5*(close_points[1](2) + close_points[0](2));	
			    			m_swarm_pos[i] = tmp;
	    				} else{
	    					printf("*****condition 3 failed! failure number : 1\n");
	    				}
	    			} else if (ctheta < 0.1 && fabs(tmp_len_2-tmp_len_1)<0.2)
	    			{
	    				Vector3f tmp;
			    		tmp(0) = 0.5*(close_points[2](0) + close_points[1](0));
			    		tmp(1) = 0.5*(close_points[2](1) + close_points[1](1));
			    		tmp(2) = 0.5*(close_points[2](2) + close_points[1](2));	
			    		m_swarm_pos[i] = tmp;
	    			} else {
	    				printf("*****condition 3 failed! failure number : 2\n");
	    			}
	    		} else if (close_points.size() == 2) //condition 2
	    		{
	    			//printf("*****condition 2\n");
    				Vector3f tmp_vec;
    				float tmp_len;
		    		tmp_vec(0) = close_points[1](0) - close_points[0](0);
		    		tmp_vec(1) = close_points[1](1) - close_points[0](1);
		    		tmp_vec(2) = close_points[1](2) - close_points[0](2);	
		    		vec3f_norm(&tmp_vec, &tmp_len);

		    		if (tmp_len > VEHICLE_EDGE_THRESHOLD && tmp_len < VEHICLE_EDGE_THRESHOLD+0.02)
		    		{
		    			Vector3f tmp;
			    		tmp(0) = 0.5*(close_points[1](0) + close_points[0](0));
			    		tmp(1) = 0.5*(close_points[1](1) + close_points[0](1));
			    		tmp(2) = 0.5*(close_points[1](2) + close_points[0](2));	
			    		m_swarm_pos[i] = tmp;
		    		} else if (tmp_len < VEHICLE_EDGE_THRESHOLD && tmp_len > VEHICLE_EDGE_THRESHOLD-0.02)
		    		{
		    			Vector3f center_pos;
		    			center_pos(0) = 0.5*(close_points[1](0) + close_points[0](0));
			    		center_pos(1) = 0.5*(close_points[1](1) + close_points[0](1));
			    		center_pos(2) = 0.5*(close_points[1](2) + close_points[0](2));	
			    		Vector3f predict_diff;
			    		predict_diff(0) = swarm_pos_predict[i](0) - center_pos(0);
			    		predict_diff(1) = swarm_pos_predict[i](1) - center_pos(1);
			    		predict_diff(2) = swarm_pos_predict[i](2) - center_pos(2);
			    		float tmp_dist;
			    		vec3f_norm(&predict_diff, &tmp_dist);

			    		float ratio = 0.035/tmp_dist;
			    		Vector3f tmp;
			    		tmp(0) = (1-ratio)*center_pos(0) + ratio*swarm_pos_predict[0](0);
			    		tmp(1) = (1-ratio)*center_pos(1) + ratio*swarm_pos_predict[0](1);
			    		tmp(2) = (1-ratio)*center_pos(2) + ratio*swarm_pos_predict[0](2);	
			    		m_swarm_pos[i] = tmp;
		    		} else {
		    			printf("*****condition 2 failed! failure number : 0\n");
		    		}

	    		} else if (close_points.size() == 1)
	    		{
	    			//printf("*****condition 1\n");
	    			Vector3f predict_diff;
		    		predict_diff(0) = swarm_pos_predict[i](0) - close_points[0](0);
		    		predict_diff(1) = swarm_pos_predict[i](1) - close_points[0](1);
		    		predict_diff(2) = swarm_pos_predict[i](2) - close_points[0](2);
		    		float tmp_dist;
		    		vec3f_norm(&predict_diff, &tmp_dist);

		    		float ratio = 0.035/tmp_dist;
		    		Vector3f tmp;
		    		tmp(0) = (1-ratio)*close_points[0](0) + ratio*swarm_pos_predict[0](0);
		    		tmp(1) = (1-ratio)*close_points[0](1) + ratio*swarm_pos_predict[0](1);
		    		tmp(2) = (1-ratio)*close_points[0](2) + ratio*swarm_pos_predict[0](2);	
		    		m_swarm_pos[i] = tmp;
	    		} else {
	    			printf("*****Cannot find vehicle%d\n", i);
	    			continue;
	    		}

	    		/*renew error*/
	    		swarm_pos_err[i](0) = m_swarm_pos[i](0) - swarm_pos_predict[i](0);
	    		swarm_pos_err[i](1) = m_swarm_pos[i](1) - swarm_pos_predict[i](1);
	    		swarm_pos_err[i](2) = m_swarm_pos[i](2) - swarm_pos_predict[i](2);

	    	}//i

			/*renew and publish*/
	    	for (int i = 0; i < swarm_pos.size(); ++i)
	    	{
	    		swarm_pos_step[i](0) = m_swarm_pos[i](0) - swarm_pos[i](0);
	    		swarm_pos_step[i](1) = m_swarm_pos[i](1) - swarm_pos[i](1);
	    		swarm_pos_step[i](2) = m_swarm_pos[i](2) - swarm_pos[i](2);

	    		swarm_pos[i] = m_swarm_pos[i];
	    		m_pos_estmsg.pos_est.x = swarm_pos[i](0);
				m_pos_estmsg.pos_est.y = swarm_pos[i](1);
				m_pos_estmsg.pos_est.z = swarm_pos[i](2);
				m_pos_estmsg.vehicle_index = i;
	    		m_pos_est_v[i].publish(m_pos_estmsg);

	    		printf("*****vehicle%d: %f  %f  %f\n", i, swarm_pos[i](0), swarm_pos[i](1), swarm_pos[i](2));
	    	}
		}// else if  	
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
