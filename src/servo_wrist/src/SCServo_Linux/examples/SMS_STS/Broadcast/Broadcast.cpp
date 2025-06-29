/*
舵机出厂速度单位是0.0146rpm，速度改为V=2400
*/

#include <iostream>
#include "SCServo.h"

SMS_STS sm_st;

int main(int argc, char **argv)
{
	//检查是否传入串口设备路径。若未传入（argc<2），打印错误并退出
	if(argc<2){
        std::cout<<"argc error!"<<std::endl;
        return 0;
	}

	//初始化串口，波特率 115200，设备路径为 argv[1]（如 /dev/ttyUSB0）
	std::cout<<"serial:"<<argv[1]<<std::endl;
    if(!sm_st.begin(115200, argv[1])){
        std::cout<<"Failed to init sms/sts motor!"<<std::endl;
        return 0;
    }

	//WritePosEx(0xfe, pos, speed, acc) 广播ID，控制所有连接的舵机，目标位置，速度，加速度
	//0-4095对应0-360°
	while(1){
		sm_st.WritePosEx(0xfe, 4095, 2400, 50);//舵机(广播)以最高速度V=2400(步/秒)，加速度A=50(50*100步/秒^2)，运行至P1=4095位置
		std::cout<<"pos = "<<4095<<std::endl;
		usleep(2187*1000);//[(P1-P0)/V]*1000+[V/(A*100)]*1000
  
		sm_st.WritePosEx(0xfe, 0, 2400, 50);//舵机(广播)以最高速度V=2400(步/秒)，加速度A=50(50*100步/秒^2)，运行至P0=0位置
		std::cout<<"pos = "<<0<<std::endl;
		usleep(2187*1000);//[(P1-P0)/V]*1000+[V/(A*100)]*1000
	}
	sm_st.end();
	return 1;
}

