#ifndef RS485_Serial_H
#define RS485_Serial_H


#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/JointState.h>
#include <liancheng_socket/MotorOrder.h>
#include <liancheng_socket/SwitchOrder.h>
#include <liancheng_socket/ReadOutput.h>
#include <std_msgs/UInt8MultiArray.h>
#include <serial/serial.h>
#include <string.h>
#include <iostream>

using namespace std;

class serialpublisher{

public:
    serialpublisher(ros::NodeHandle & nh);
    ~serialpublisher();

    void prepare();
    void vel_form(uint8_t station_num,int16_t vel, uint16_t vel_ac ,uint16_t vel_de);
    void pos_form(uint8_t station_num,bool pos_mode,int32_t pos, uint16_t pos_thr ,uint16_t vel,uint16_t vel_ac);
    void pos_pinch(uint8_t station_num,uint16_t maxSpeed,int32_t angleControl);
    void enable_off(uint8_t station_num);
    void enable_on(uint8_t station_num);
    // void pos_x(uint8_t station_num,int16_t vel,int32_t pos);
    // void pos_y(uint8_t station_num,int16_t vel,int32_t pos);
    void switch_ch(uint8_t station_num, uint16_t switch_num, uint8_t case_num);



private:
    serial::Serial ser;
    ros::NodeHandle nh_;
    ros::Duration sleeptime=ros::Duration(3);
    ros::Duration sleeptime_code=ros::Duration(0.01);
    serial::Timeout to;
    uint32_t baud;
    ros::Subscriber sub;
    ros::Subscriber sub_sw;
    ros::Subscriber sub_read;
    ros::Publisher pub_read;
    std::string rl;
    std::string port;
    int8_t form=-1;
    char *p;

    vector<vector<uint8_t>> code_list{vector<vector<uint8_t>>(23)};
    vector<vector<uint8_t>>motor_list{vector<vector<uint8_t>>(2)};
    
    
    
    
    

    void callback(const liancheng_socket::MotorOrder & order);
    void callback_sw(const liancheng_socket::SwitchOrder & order);
    void callback_read(const liancheng_socket::SwitchOrder & order);
    vector<uint8_t> CRC16_MudBus(vector<uint8_t> puchMsg, uint8_t usDataLen,uint8_t station_num);
    vector<uint8_t> CRC16_MudBus_pinch(vector<uint8_t> puchMsg, uint8_t usDataLen,uint8_t station_num);
    bool set_form(uint8_t station_num,int8_t form_input);//mode select:0=pos;1=vel/模式选择:0=位移模式；1=速度模式。
};
#endif
