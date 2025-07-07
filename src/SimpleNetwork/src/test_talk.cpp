#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/JointState.h>
#include "liancheng_socket/MotorOrder.h"
// #include "liancheng_socket/SwitchOrder.h"
#include <servo_wrist/SerControl.h>
#include <std_msgs/UInt8MultiArray.h>
#include "std_msgs/String.h"
#include <serial/serial.h>
#include <string.h>
#include <iostream>

using namespace std;

void arm_order( ros::Publisher p,liancheng_socket::MotorOrder m,uint8_t station_num,uint8_t form,int16_t vel,uint16_t vel_ac,uint16_t vel_de,bool pos_mode,int32_t pos,uint16_t pos_thr)
{
	m.header.stamp=ros::Time::now();
	m.station_num.push_back(station_num);
	m.form.push_back(form);
	m.vel.push_back(vel);
	m.vel_ac.push_back(vel_ac);
	m.vel_de.push_back(vel_de);
	m.pos_mode.push_back(pos_mode);
	m.pos.push_back(pos);
	m.pos_thr.push_back(pos_thr);

	p.publish(m);
}

void servo_order( ros::Publisher p,servo_wrist::SerControl m,int32_t servo_id,int32_t target_position,int32_t velocity,int32_t acceleration)
{
	m.servo_id=servo_id;
	m.target_position=target_position;
	m.velocity=velocity;
	m.acceleration=acceleration;
	p.publish(m);
}

void relay_order(ros::Publisher p,std_msgs::String m,string s) //msg(absorb 0/1, screw 0/1)
{
	m.data = s;
	p.publish(m);
}


int main (int argc, char** argv)
{
    ros::init( argc, argv, "talker" );
    ros::NodeHandle nh;
    ros::Rate rate(100);

    ros::Publisher pub=nh.advertise<liancheng_socket::MotorOrder>("Controller_motor_order",5);
	ros::Publisher pub_servo=nh.advertise<servo_wrist::SerControl>("servo_control_topic",5);
	ros::Publisher pub_relay= nh.advertise<std_msgs::String>("RelayOrder", 5);

    rate.sleep();

    liancheng_socket::MotorOrder msg;
	liancheng_socket::MotorOrder msg_pinch;
	servo_wrist::SerControl msg_servo;
	std_msgs::String msg_relay;


    string rec;
	while(1)
	{
		cout << "Ready!Please input 1-3 to show different pose." << endl;
		cin>>rec;

		// video1 demonstrate degrees of freedom 
		if(rec=="1")	
		{
				//servo move
				servo_order(pub_servo,msg_servo,1,2500,1000,50);
				servo_order(pub_servo,msg_servo,2,200,1000,50);
				servo_order(pub_servo,msg_servo,3,2400,500,50);
				servo_order(pub_servo,msg_servo,4,2900,500,50);
				servo_order(pub_servo,msg_servo,5,2900,500,50);
				cout << "Initializing..." << endl;
				sleep(5);	

				//pinch motor move
				arm_order(pub,msg_pinch,1,11,0,0,0,true,-25,30);
				sleep(5);
				cout<<"pinch motor move"<<endl;

				//pinch motor zero
				arm_order(pub,msg_pinch,1,11,0,0,0,true,0,30);                                                                                                                                                                                                                                
				sleep(5);
				cout<<"pinch motor zero"<<endl;

				//feed motor move
				arm_order(pub,msg,2,100,200,0,0,false,-2000,10);
				sleep(0.5);
				arm_order(pub,msg,2,0,200,0,0,false,-2000,10);
				sleep(0.5);
				arm_order(pub,msg,2,99,200,0,0,false,-2000,10);
				sleep(3);
				cout<<"feed motor move"<<endl;

				//feed motor zero
				arm_order(pub,msg,2,100,200,0,0,false,1900,10);	
				sleep(0.5);
				arm_order(pub,msg,2,0,200,0,0,true,300,10);
				sleep(0.5);
				arm_order(pub,msg,2,99,200,0,0,false,1900,10);
				sleep(3);
				arm_order(pub,msg,2,100,100,0,0,false,1900,10);	
				sleep(0.5);
				cout<<"feed motor zero"<<endl;

				servo_order(pub_servo,msg_servo,1, 4000, 1000, 50);
				cout << "ID1 pos = 2700" << endl;
				sleep(4);
				servo_order(pub_servo,msg_servo,2, 1000, 1000, 50);
				cout << "ID2 pos = 3000" << endl;
				sleep(4);
				servo_order(pub_servo,msg_servo,3, 3200, 500, 50);
				servo_order(pub_servo,msg_servo,4, 2800, 500, 50);
				servo_order(pub_servo,msg_servo,5, 2600, 500, 50);
				cout << "ID3 pos = 3200" << endl << "ID4 pos = 2800" << endl << "ID5 pos = 2600" << endl;
				sleep(5);
				cout << "Task finished." <<  endl;

				//servo zero
				servo_order(pub_servo,msg_servo,1,2500,1000,50);
				servo_order(pub_servo,msg_servo,2,200,1000,50);
				servo_order(pub_servo,msg_servo,3,2400,500,50);
				servo_order(pub_servo,msg_servo,4,2900,500,50);
				servo_order(pub_servo,msg_servo,5,2900,500,50);
				cout << "Initializing..." << endl;
				sleep(7);		

		}
		// video2 demonstrate normal posture
		else if(rec=="2")
		{
				//servo zero
				servo_order(pub_servo,msg_servo,1,2500,1000,50);
				servo_order(pub_servo,msg_servo,2,200,1000,50);
				servo_order(pub_servo,msg_servo,3,2400,500,50);
				servo_order(pub_servo,msg_servo,4,2900,500,50);
				servo_order(pub_servo,msg_servo,5,2900,500,50);
				cout << "Initializing..." << endl;
				sleep(5);	

				// //pinch motor move
				// arm_order(pub,msg_pinch,1,11,0,0,0,true,-25,30);
				// sleep(5);
				// cout<<"pinch motor move"<<endl;
				
				//servo move		
				servo_order(pub_servo,msg_servo,1, 2000, 1000, 50);
				cout << "ID1 pos = 2700" << endl;
				sleep(3);
				servo_order(pub_servo,msg_servo,2, 1500, 1000, 50);
				cout << "ID2 pos = 3000" << endl;
				sleep(3);
				servo_order(pub_servo,msg_servo,3, 2100, 500, 50);
				servo_order(pub_servo,msg_servo,4, 3300, 500, 50);
				servo_order(pub_servo,msg_servo,5, 3300, 500, 50);
				cout << "ID3 pos = 3200" << endl << "ID4 pos = 2800" << endl << "ID5 pos = 2600" << endl;
				sleep(3);
				cout << "Task finished." <<  endl;

				//feed motor move
				arm_order(pub,msg,2,100,100,0,0,false,-2000,10);
				sleep(0.5);
				arm_order(pub,msg,2,0,100,0,0,false,-2500,10);
				sleep(0.5);
				arm_order(pub,msg,2,99,100,0,0,false,-2000,10);
				sleep(13);
				cout<<"feed motor move"<<endl;

				// absorb start
				relay_order(pub_relay,msg_relay,"11");	


				// //pinch motor zero
				// arm_order(pub,msg_pinch,1,11,0,0,0,true,0,30);
				// sleep(5);
				// cout<<"pinch motor zero"<<endl;

		 		//feed motor zero
		 		arm_order(pub,msg,2,100,200,0,0,false,2000,10);	
				sleep(0.5);
		 		arm_order(pub,msg,2,0,200,0,0,true,300,10);
		 		sleep(0.5);
		 		arm_order(pub,msg,2,99,200,0,0,false,2000,10);
		 		sleep(3);
		 		arm_order(pub,msg,2,100,100,0,0,false,2000,10);	
		 		sleep(0.5);
		 		cout<<"feed motor zero"<<endl;
				
				//servo zero
				servo_order(pub_servo,msg_servo,1,2500,1000,50);
				servo_order(pub_servo,msg_servo,2,200,1000,50);
				servo_order(pub_servo,msg_servo,3,2400,500,50);
				servo_order(pub_servo,msg_servo,4,2900,500,50);
				servo_order(pub_servo,msg_servo,5,2900,500,50);
				cout << "Initializing..." << endl;
				sleep(5);	

				// absorb shut down
				relay_order(pub_relay,msg_relay,"00");
		}
		// video3 demonstrate large degree posture
		else if(rec=="3")
		{
				//pinch motor move
				arm_order(pub,msg_pinch,1,11,0,0,0,true,-25,30);
				sleep(5);
				cout<<"pinch motor move"<<endl;
		}
		else break;
		ros::spinOnce();
	}


	// test1
	// while(1)
	// {
    // cout << "please input:" << endl;
	// cin>>rec;
    // if (rec=="1")  // arm motor move
	// 		{
	// 			msg.header.stamp=ros::Time::now();
	// 			msg.station_num.push_back(1);
	// 			msg.form.push_back(100);
	// 			msg.vel.push_back(100);
	// 			msg.vel_ac.push_back(0);
	// 			msg.vel_de.push_back(0);
	// 			msg.pos_mode.push_back(false);
	// 			msg.pos.push_back(-1500);
	// 			msg.pos_thr.push_back(10);
	// 			pub.publish(msg);

	// 			msg.station_num.push_back(1);
	// 			msg.form.push_back(0);
	// 			msg.vel.push_back(100);
	// 			msg.vel_ac.push_back(0);
	// 			msg.vel_de.push_back(0);
	// 			msg.pos_mode.push_back(false);
	// 			msg.pos.push_back(-1500);
	// 			msg.pos_thr.push_back(10);
	// 			pub.publish(msg);

	// 			msg.station_num.push_back(1);
	// 			msg.form.push_back(99);
	// 			msg.vel.push_back(100);
	// 			msg.vel_ac.push_back(0);
	// 			msg.vel_de.push_back(0);
	// 			msg.pos_mode.push_back(false);
	// 			msg.pos.push_back(-1500);
	// 			msg.pos_thr.push_back(10);
	// 			pub.publish(msg);
	// 			sleep(6);

	// 			cout<<"arm motor moved"<<endl;
	// 		}
	// 		else if (rec=="2") // arm motor zero
	// 		{
	// 			msg.header.stamp=ros::Time::now();
	// 			msg.station_num.push_back(1);
	// 			// msg.form.push_back(100);
	// 			// msg.vel.push_back(100);
	// 			// msg.vel_ac.push_back(0);
	// 			// msg.vel_de.push_back(0);
	// 			// msg.pos_mode.push_back(false);
	// 			// msg.pos.push_back(0);
	// 			// msg.pos_thr.push_back(10);
	// 			// pub.publish(msg);

	// 			msg.station_num.push_back(1);
	// 			msg.form.push_back(0);
	// 			msg.vel.push_back(100);
	// 			msg.vel_ac.push_back(0);
	// 			msg.vel_de.push_back(0);
	// 			msg.pos_mode.push_back(false);
	// 			msg.pos.push_back(-1500);
	// 			msg.pos_thr.push_back(10);
	// 			pub.publish(msg);

	// 			msg.station_num.push_back(1);
	// 			msg.form.push_back(99);
	// 			msg.vel.push_back(100);
	// 			msg.vel_ac.push_back(0);
	// 			msg.vel_de.push_back(0);
	// 			msg.pos_mode.push_back(false);
	// 			msg.pos.push_back(-1500);
	// 			msg.pos_thr.push_back(10);
	// 			pub.publish(msg);
	// 			sleep(6);

	// 			cout<<"arm motor back"<<endl;
	// 		}
    //         else if(rec=="3") //pinch motor move
    //         {
	// 			msg_pinch.header.stamp=ros::Time::now();
	// 			// msg_pinch.station_num.push_back(0);
	// 			msg_pinch.station_num.push_back(1);
	// 			msg_pinch.form.push_back(11);
	// 			msg_pinch.pos.push_back(0);
	// 			msg_pinch.pos_thr.push_back(30);
	// 			msg_pinch.vel.push_back(0);
	// 			msg_pinch.vel_ac.push_back(0);
	// 			msg_pinch.vel_de.push_back(0);
	// 			msg_pinch.pos_mode.push_back(true);
				
	// 			pub.publish(msg_pinch);

	// 			cout<<"pinch motor move"<<endl;
			
	// 		}
	// 		else if (rec=="4") //pinch motor zero
	// 		{
	// 			msg_pinch.header.stamp=ros::Time::now();
	// 			// msg_pinch.station_num.push_back(0);
	// 			msg_pinch.station_num.push_back(1);
	// 			msg_pinch.form.push_back(11);
	// 			msg_pinch.pos.push_back(-25);
	// 			msg_pinch.pos_thr.push_back(30);
	// 			msg_pinch.vel.push_back(0);
	// 			msg_pinch.vel_ac.push_back(0);
	// 			msg_pinch.vel_de.push_back(0);
	// 			msg_pinch.pos_mode.push_back(true);

	// 			pub.publish(msg_pinch);

	// 			cout<<"pinch motor zero"<<endl;
	// 		}
	// 		else 
	// 		{
	// 			break;
	// 		}
			
	// cout<<endl;
	// ros::spinOnce();
	// }
    return 0;
}

