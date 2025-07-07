/*
同步读指令，回读ID1与ID2两个舵机的位置与速度信息
*/

#include <iostream>
#include "SCServo.h"

SMS_STS sm_st;
//定义变量 舵机id 数据数组 位置 速度
uint8_t ID[] = {1, 2};
uint8_t rxPacket[4];
int16_t Position;
int16_t Speed;

int main(int argc, char **argv)
{
	if(argc<2){
        std::cout<<"argc error!"<<std::endl;
        return 0;
	}
	std::cout<<"serial:"<<argv[1]<<std::endl;
    if(!sm_st.begin(115200, argv[1])){
        std::cout<<"Failed to init sms/sts motor!"<<std::endl;
        return 0;
    }

	//读取舵机ID数组的大小，接收缓冲区的大小
	sm_st.syncReadBegin(sizeof(ID), sizeof(rxPacket));
	while(1){
		//发送同步读取指令
		sm_st.syncReadPacketTx(ID, sizeof(ID), SMS_STS_PRESENT_POSITION_L, sizeof(rxPacket));//同步读指令包发送
		//遍历所有舵机ID
		for(uint8_t i=0; i<sizeof(ID); i++){
			//接收ID[i]同步读返回包
			if(!sm_st.syncReadPacketRx(ID[i], rxPacket)){
				std::cout<<"ID:"<<(int)ID[i]<<" sync read error!"<<std::endl;
				continue;//接收解码失败
			}
			Position = sm_st.syncReadRxPacketToWrod(15);//解码两个字节 bit15为方向位,参数=0表示无方向位
			Speed = sm_st.syncReadRxPacketToWrod(15);//解码两个字节 bit15为方向位,参数=0表示无方向位
			std::cout<<"ID:"<<int(ID[i])<<" Position:"<<Position<<" Speed:"<<Speed<<std::endl;
		}
		usleep(10*1000);
	}
	sm_st.syncReadEnd();
	sm_st.end();
	return 1;
}

