#include "RS485Serial.h"

serialpublisher::serialpublisher(ros::NodeHandle & nh){

        nh_=nh;
        to = serial::Timeout::simpleTimeout(3000);
        baud=115200;
        
        // 先尝试ttyUSB0
        port="/dev/ttyUSB0";
        try {
            ser.setPort(port);
            ser.setBaudrate(baud);
            ser.setTimeout(to);
            ser.open();
        } catch (serial::IOException& e) {
            // 如果ttyUSB0失败，尝试ttyUSB1
            port="/dev/ttyUSB1";
            try {
                ser.setPort(port);
                ser.setBaudrate(baud);
                ser.setTimeout(to);
                ser.open();
            } catch (serial::IOException& e1) {
                ROS_ERROR_STREAM("无法打开串口设备: " << port);
                throw;
            }
        }

	sub=nh_.subscribe("/Controller_motor_order",5,&serialpublisher::callback,this);
        sub_sw=nh_.subscribe("/Controller_switch_order",5,&serialpublisher::callback_sw,this);
        sub_read=nh_.subscribe("/Read_motor",5,&serialpublisher::callback_read,this);
        pub_read=nh_.advertise<liancheng_socket::ReadOutput>("Read_motor_output",5);


        /*
    code list:
    0->DI1=enable/DI1端口关联使能;
    1->enable on/使能开;
    2->enable off/使能关;
    3->control model:vel/控制模式：速度控制;
    4->control model:pos/控制模式：位移控制;
    5->vel order from:internal vel/速度指令来源：内部速度指令;
    6->DI2=pos enable/DI2端口关联位移运行使能;
    7->pos enable on/位移运行使能开;
    8->pos enable off/位移运行使能关;
    9->DO1=pos arrived/DO1端口关联定位到达;
    10->DO1 positive/定位到达正逻辑;
    11->pso order from:internal pos/位置指令来源：内部位移指令;
    12->pos from:once/多段位置运行方式：单次运行;
    13->pos num:1/位移指令段数:1段;
    14->absolute pos/绝对位移模式;
    15->relative pos/相对位移模式;
    16->read vel/读取转速;
    17->read pos/读取位置;
    18->read input/读取输入端口情况;
    19->read output/读取输出端口情况;
    20->read current/读取相电流;
    21->read voltage/读取母线电压;
    22->read temperature/读取模块温度;
    */
    // vector<vector<uint8_t>> code_list(23);
    code_list[0]={0x06,0x03,0x02,0x00,0x01};
    code_list[1]={0x06,0x03,0x03,0x00,0x01};
    code_list[2]={0x06,0x03,0x03,0x00,0x00};
    code_list[3]={0x06,0x02,0x00,0x00,0x00};
    code_list[4]={0x06,0x02,0x00,0x00,0x01};
    code_list[5]={0x06,0x06,0x02,0x00,0x00};
    code_list[6]={0x06,0x03,0x04,0x00,0x1c};
    code_list[7]={0x06,0x03,0x05,0x00,0x01};
    code_list[8]={0x06,0x03,0x05,0x00,0x00};
    code_list[9]={0x06,0x04,0x00,0x00,0x05};
    code_list[10]={0x06,0x04,0x01,0x00,0x00};
    code_list[11]={0x06,0x05,0x00,0x00,0x02};
    code_list[12]={0x06,0x11,0x00,0x00,0x00};
    code_list[13]={0x06,0x11,0x01,0x00,0x01};
    code_list[14]={0x06,0x11,0x04,0x00,0x01};
    code_list[15]={0x06,0x11,0x04,0x00,0x00};
    code_list[16]={0x03,0x0b,0x00,0x00,0x01};
    code_list[17]={0x03,0x0b,0x07,0x00,0x02};
    code_list[18]={0x03,0x0b,0x03,0x00,0x01};
    code_list[19]={0x03,0x0b,0x05,0x00,0x01};
    code_list[20]={0x03,0x0b,0x18,0x00,0x01};
    code_list[21]={0x03,0x0b,0x1a,0x00,0x01};
    code_list[22]={0x03,0x0b,0x1b,0x00,0x01};
 /*
 motor_list:
 0-> 电机停止命令；
 1-> 电机关闭命令；
 
 */
motor_list[0]={0x3E,0x01,0x08,0x81,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
motor_list[1]={0x3E,0x01,0x08,0x80,0x00,0x00,0x00,0x00,0x00,0x00,0x00};



    try
    {
    //set the property of the port and open it.
        ser.setPort(port);
        ser.setBaudrate(baud);
        ser.setTimeout(to);
        ser.open();
    }
    catch (serial::IOException &e)
    {
        ROS_ERROR_STREAM("Unable to open port ");       
        //return -1;
    }
    catch(serial::SerialException &e)
    {
      ROS_ERROR_STREAM("Serial port already open. ");       
    }



    }

serialpublisher::~serialpublisher(){}

void serialpublisher::prepare()
{
    while(!ser.isOpen())
    {
        port="/dev/ttyUSB1";
      try
    {
    //set the property of the port and open it.
        ser.setPort(port);
        ser.setBaudrate(baud);
        ser.setTimeout(to);
        ser.open();
    }
    catch (serial::IOException &e)
    {
        ROS_ERROR_STREAM("Unable to open port ");       
    }
     catch(serial::SerialException &e)
    {
      ROS_ERROR_STREAM("Serial port already open. ");       
    }


    sleeptime.sleep();
    port="/dev/ttyUSB0";
      try
    {
    //set the property of the port and open it.
        ser.setPort(port);
        ser.setBaudrate(baud);
        ser.setTimeout(to);
        ser.open();
    }
    catch (serial::IOException &e)
    {
        ROS_ERROR_STREAM("Unable to open port ");       
    }
     catch(serial::SerialException &e)
    {
      ROS_ERROR_STREAM("Serial port already open. ");       
    }

    }

    sleeptime.sleep();

    ROS_INFO("Connected !");

    //sub = nh_.subscribe("/Controller_joint_states", 5, &serialpublisher::callback,this);
    ROS_INFO("End preparation");

}

bool serialpublisher::set_form(uint8_t station_num,int8_t form_input)
{

    if (form==form_input)
    {
        return true;
    }
    int *code_order;
    int code_num;
    
    if (form_input==1)
    {
        code_num=6;
        code_order=new int[code_num]{3,0,2,5,6,8};
    }
    else if (form_input==0)
    {
        code_num=10;
        code_order=new int[code_num]{4,0,2,6,8,9,10,11,12,13};
    }
    else
    {
        cout<<"form input error"<<endl;
        return false;
    }

    for (int i=0;i<code_num;i++)
    {
        ser.write(CRC16_MudBus(code_list[code_order[i]],code_list[code_order[i]].size(),station_num));
        sleeptime_code.sleep();
    }

    form=form_input;
    return true;
}

void serialpublisher::vel_form(uint8_t station_num,int16_t vel, uint16_t vel_ac,uint16_t vel_de){

    if (form!=1)
    {
        set_form(station_num,1);
    }
    if (vel > 6000)
    {
        vel=6000;
    }
    if (vel < -6000)
    {
        vel=-6000;
    }
    vector<uint8_t> code5={0x06,0x06,0x03,uint8_t(vel>>8),uint8_t(vel&0x00ff)};
    vector<uint8_t> code6={0x06,0x06,0x05,uint8_t(vel_ac>>8),uint8_t(vel_ac&0x00ff)};
    vector<uint8_t> code7={0x06,0x06,0x06,uint8_t(vel_de>>8),uint8_t(vel_de&0x00ff)};
    
    ser.write(CRC16_MudBus(code5,code5.size(),station_num));
    sleeptime_code.sleep();
    ser.write(CRC16_MudBus(code6,code6.size(),station_num));
    sleeptime_code.sleep();
    ser.write(CRC16_MudBus(code7,code7.size(),station_num));
    sleeptime_code.sleep();
    cout<<"vel_form set successfully!"<<endl;
    cout<<"station num="<<station_num<<endl;
    cout<<"set vel="<<vel<<endl;
    cout<<"set vel_ac="<<vel_ac<<endl;
    cout<<"set vel_de="<<vel_de<<endl;

}

void serialpublisher::pos_form(uint8_t station_num,bool pos_mode,int32_t pos, uint16_t pos_thr ,uint16_t vel,uint16_t vel_ac){

    if (form!=0)
    {
        set_form(station_num,0);
    }
    if (pos>300)
    {
        pos=300;
    }
    if (pos<-3500)
    {
        pos=-3500;
    }
    vector<uint8_t> code1={0x06,0x05,0x15,uint8_t(pos_thr>>8),uint8_t(pos_thr&0x00ff)};
    int16_t pos_high=int16_t(pos/(256*256));
    int16_t pos_low=int16_t(pos%(256*256));
    if (pos_high==0 && pos<0)
    {
         pos_high=0xffff;
    }
    vector<uint8_t> code2={0x10,0x11,0x0c,0x00,0x04,0x08,uint8_t(pos_low>>8),uint8_t(pos_low&0x00ff),uint8_t(pos_high>>8),uint8_t(pos_high&0x00ff),uint8_t(vel>>8),uint8_t(vel&0x00ff),uint8_t(vel_ac>>8),uint8_t(vel_ac&0x00ff)};

    if (pos_mode)
    {
        ser.write(CRC16_MudBus(code_list[14],code_list[14].size(),station_num));
        sleeptime_code.sleep();
    }
    else
    {
        ser.write(CRC16_MudBus(code_list[15],code_list[15].size(),station_num));
        sleeptime_code.sleep();
    }
    
    ser.write(CRC16_MudBus(code1,code1.size(),station_num));
    sleeptime_code.sleep();
    ser.write(CRC16_MudBus(code2,code2.size(),station_num));
    sleeptime_code.sleep();
    // ser.write(CRC16_MudBus(code_list[7],code_list[7].size(),station_num));
    // sleeptime_code.sleep();
    cout<<"pos_form set successfully!"<<endl;
    cout<<"station num="<<station_num<<endl;
    if (pos_mode)
    {
        cout<<"set pos_mode=absolute pos"<<endl;
    }
    else
    {
        cout<<"set pos_mode=relative pos"<<endl;
    }
    cout<<"set pos="<<pos<<endl;
    cout<<"set pos_thr="<<pos_thr<<endl;
    cout<<"set vel="<<vel<<endl;
    cout<<"set vel_ac="<<vel_ac<<endl;
}

void serialpublisher::pos_pinch(uint8_t station_num,uint16_t maxSpeed,int32_t angleControl)
{
    // maxSpeed: 1dps/LSB; angleControl: 0.01 degree/LSB; station_num: offset 0x01;
    vector<uint8_t>motor_list1={0x3E,0x08,0xA4,0x00,uint8_t(maxSpeed),uint8_t(maxSpeed>>8),uint8_t(angleControl),uint8_t(angleControl>>8),uint8_t(angleControl>>16),uint8_t(angleControl>>24)};

    ser.write(CRC16_MudBus_pinch(motor_list1,motor_list1.size(),station_num));
    sleeptime_code.sleep();

    cout<<"pinch motor set_pos successfully!"<<endl;
    cout<<"station num="<<station_num<<endl;

    cout<<"set maxSpeed="<<maxSpeed<<endl;
    cout<<"set angleControl="<<angleControl<<endl;

}

void serialpublisher::switch_ch(uint8_t station_num, uint16_t switch_num, uint8_t case_num)
{
    vector<uint8_t> code1={0x06,uint8_t(switch_num>>8),uint8_t(switch_num&0x00ff),0,case_num};
    ser.write(CRC16_MudBus(code1,code1.size(),station_num));
    sleeptime_code.sleep();
}

void serialpublisher::callback_read(const liancheng_socket::SwitchOrder & order)
{

    uint8_t station_num=order.station_num;
    uint16_t switch_num=order.switch_num;
    uint8_t  case_num=order.case_num;
    int n=order.switch_num+order.case_num*3;
    vector<uint8_t> data_list;
    int num,data;
    
    vector<uint16_t> switch_list={0x0b07,1008,1028,0x0b05,1012,1032};
    vector<uint8_t> num_list={2,1,1,1,1,1};


    vector<uint8_t> code1={0x03,uint8_t(switch_list[n]>>8),uint8_t(switch_list[n]&0x00ff),0,num_list[n]};
    ser.write(CRC16_MudBus(code1,code1.size(),station_num));
    sleeptime_code.sleep();
    num=ser.available();
    ser.read(data_list,num);
    if (n<=2)
    {
        data=data_list[3]*256+data_list[4];
    }
    else if (n==3)
    {
        data=data_list[4]&0x04;
        if (data!=0)
        {
            data=1;
        }
    }
    else
    {
        data=data_list[4]&0x08;
        if (data!=0)
        {
            data=1;
        }
    }
    liancheng_socket::ReadOutput msg;
    msg.header.stamp=ros::Time::now();
    msg.data_form=n;
    msg.data=data;
    pub_read.publish(msg);
    
}

void serialpublisher::enable_off(uint8_t station_num)
{


    ser.write(CRC16_MudBus(code_list[2],code_list[2].size(),station_num));
    sleeptime_code.sleep();
    ser.write(CRC16_MudBus(code_list[8],code_list[8].size(),station_num));
    sleeptime_code.sleep();
}

void serialpublisher::enable_on(uint8_t station_num)
{
    ser.write(CRC16_MudBus(code_list[1],code_list[1].size(),station_num));
    sleeptime_code.sleep();
    ser.write(CRC16_MudBus(code_list[7],code_list[7].size(),station_num));
    sleeptime_code.sleep();
}

void serialpublisher::callback(const liancheng_socket::MotorOrder & order)
{
    int order_num=order.station_num.size();
    
    for (int i=0;i<order_num;i++)
    {
        uint8_t station_num=order.station_num[i];
        uint8_t form=order.form[i];
        int16_t vel=order.vel[i];
        uint16_t vel_ac=order.vel_ac[i];
        uint16_t vel_de=order.vel_de[i];
        bool pos_mode=order.pos_mode[i];
        int32_t pos=order.pos[i];
        uint16_t pos_thr=order.pos_thr[i];
        switch (form)
        {
            case 0:
            this->pos_form(station_num,pos_mode,pos, pos_thr,vel,vel_ac);
            continue;

            case 1:
            this->vel_form(station_num,vel,vel_ac,vel_de);
            continue;

	        case 11:
            this->pos_pinch(station_num,pos_thr*100,pos*10000);
            continue;

            // case 12:
            // this->pos_y(station_num,vel,int16_t(pos));
            // continue;

            case 99:
            this->enable_on(station_num);
            continue;

            case 100:
            this->enable_off(station_num);
            continue;
        }

    return;
}
}

void serialpublisher::callback_sw(const liancheng_socket::SwitchOrder & order)
{
    uint8_t station_num=order.station_num;
    uint16_t switch_num=order.switch_num;
    uint8_t  case_num=order.case_num;
    this->switch_ch(station_num,switch_num,case_num);
    return;
}


vector<uint8_t> serialpublisher::CRC16_MudBus(vector<uint8_t> puchMsg, uint8_t usDataLen,uint8_t station_num){
	
	uint16_t uCRC = 0xffff;

    vector<uint8_t> code_v=puchMsg;
    // cout<<int(station_num)<<endl;
    code_v.insert(code_v.begin(),station_num);
    
    // for(uint8_t num=0;num<usDataLen+1;num++)
    // {
    //     cout<<int(code_v[num])<<endl;
    // }
	
	for(uint8_t num=0;num<usDataLen+1;num++){
		uCRC = code_v[num]^uCRC;
		for(uint8_t x=0;x<8;x++){	
			if(uCRC&0x0001){	
				uCRC = uCRC>>1;	
				uCRC = uCRC^0xA001;	
			}else{	
				uCRC = uCRC>>1;	
			}
		}
	}
    code_v.push_back(uCRC&0x00ff);
    code_v.push_back(uCRC>>8);

    //  for(uint8_t num=0;num<usDataLen+3;num++)
    // {
    //     cout<<hex<<int(code_v[num])<<endl;
    //  }

    
	return code_v;
}


vector<uint8_t> serialpublisher::CRC16_MudBus_pinch(vector<uint8_t> puchMsg, uint8_t usDataLen,uint8_t station_num){
	
	uint16_t uCRC = 0xffff;

    vector<uint8_t> code_v=puchMsg;
    // cout<<int(station_num)<<endl;
    code_v.insert(code_v.begin()+1,station_num);
    
    // for(uint8_t num=0;num<usDataLen+1;num++)
    // {
    //     cout<<int(code_v[num])<<endl;
    // }
	
	for(uint8_t num=0;num<usDataLen+1;num++){
		uCRC = code_v[num]^uCRC;
		for(uint8_t x=0;x<8;x++){	
			if(uCRC&0x0001){	
				uCRC = uCRC>>1;	
				uCRC = uCRC^0xA001;	
			}else{	
				uCRC = uCRC>>1;	
			}
		}
	}
    code_v.push_back(uCRC&0x00ff);
    code_v.push_back(uCRC>>8);

     //for(uint8_t num=0;num<usDataLen+3;num++)
    // {
     //    cout<<hex<<int(code_v[num])<<endl;
     //}

    
	return code_v;
}
