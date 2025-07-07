/*
舵机出厂速度单位是0.0146rpm，速度改为V=2400
*/

#include <iostream>
#include "SCServo.h"

SMS_STS sm_st;

int main(int argc, char **argv)
{
	if(argc<2){
        std::cout<<"argc error!"<<std::endl;
        return 0;
	}
	std::cout<<"serial:"<<argv[1]<<std::endl;
    if(!sm_st.begin(1000000, argv[1])){
        std::cout<<"Failed to init sms/sts motor!"<<std::endl;
        return 0;
    }
	// while(1){
	// 	sm_st.WritePosEx(3, 3000, 200, 50);//舵机(ID1)以最高速度V=2400(步/秒)，加速度A=50(50*100步/秒^2)，运行至P1=4095位置
	// 	std::cout<<"pos = "<<3000<<std::endl;
	// 	//std::cout<<sm_st.txBufLen<<std::endl;

	// 	usleep(5*1000*1000);//[(P1-P0)/V]*1000+[V/(A*100)]*1000
  
	// 	sm_st.WritePosEx(3, 2400, 200, 50);//舵机(ID1)以最高速度V=2400(步/秒)，加速度A=50(50*100步/秒^2)，运行至P0=0位置
	// 	std::cout<<"pos = "<<2400<<std::endl;
	// 	usleep(5*1000*1000);//[(P1-P0)/V]*1000+[V/(A*100)]*1000
	// }


	int x = 0;
	// video1
	int v1 = 1000, v2 = 1000, v3 = 500, v4 = 500, v5 = 500, v6 = 600;
	std::cout << "Initializing..." << std::endl;
	//分别设置6个舵机（ID为1到6）的目标位置、速度和加速度
	sm_st.WritePosEx(1, 3000, v1, 50);
	sm_st.WritePosEx(2, 4000, v2, 50);
	sm_st.WritePosEx(3, 2700, v3, 50);
	sm_st.WritePosEx(4, 2800, v4, 50);
	sm_st.WritePosEx(5, 2700, v5, 50);
	sm_st.WritePosEx(6, 1800, v6, 50);
	usleep(5*1000000);
	std::cout << "Initialized." << std::endl;

	//用户输入x值
	std::cout << "cin:";
	std::cin >> x;

	//分别设置6个舵机（ID为1到6）的目标位置、速度和加速度
	sm_st.WritePosEx(1, 2500, v1, 50);
	std::cout << "ID1 pos = 2500" << std::endl;
	usleep(1*1000000);
	sm_st.WritePosEx(2, 3700, v2, 50);
	std::cout << "ID2 pos = 3700" << std::endl;
	usleep(1*1000000);
	sm_st.WritePosEx(3, 2000, v3, 50);
	sm_st.WritePosEx(4, 3100, v4, 50);
	sm_st.WritePosEx(5, 3100, v5, 50);
	std::cout << "ID3 pos = 2000" << std::endl << "ID4 pos = 3100" << std::endl << "ID5 pos = 3100" << std::endl;
	usleep(2*1000000);
	sm_st.WritePosEx(6, 3600, v6, 50);
	std::cout << "ID6 pos = 3600" << std::endl;
	std::cout << "Task finished." << std::endl;

	// video2
	// int v1 = 1000, v2 = 1000, v3 = 500, v4 = 500, v5 = 500, v6 = 600;
	// std::cout << "Initializing..." << std::endl;
	// sm_st.WritePosEx(6, 1800, v6, 50);
	// usleep(2*1000000);
	// sm_st.WritePosEx(3, 2800, v3, 50);
	// sm_st.WritePosEx(4, 2800, v4, 50);
	// sm_st.WritePosEx(5, 2800, v5, 50);
	// usleep(2*1000000);
	// sm_st.WritePosEx(1, 500, v1, 50);
	// usleep(5*1000000);
	// sm_st.WritePosEx(2, 4800, v2, 50);
	// usleep(3*1000000);
	// std::cout << "Initialized." << std::endl;

	// std::cout << "cin:";
	// std::cin >> x;

	// sm_st.WritePosEx(2, 2750, v2, 50);
	// std::cout << "ID2 pos = 2900" << std::endl;
	// usleep(3*1000000);
	// sm_st.WritePosEx(1, 4000, v1, 50);
	// std::cout << "ID1 pos = 4000" << std::endl;
	// usleep(5*1000000);
	// sm_st.WritePosEx(3, 4000, v3, 50);
	// sm_st.WritePosEx(4, 2000, v4, 50);
	// sm_st.WritePosEx(5, 3600, v5, 50);
	// std::cout << "ID3 pos = 2000" << std::endl << "ID4 pos = 3100" << std::endl << "ID5 pos = 3100" << std::endl;
	// usleep(2*1000000);
	// sm_st.WritePosEx(6, 5000, v6, 50);
	// std::cout << "ID6 pos = 5000" << std::endl;
	// std::cout << "Task finished." << std::endl;
	// usleep(2*1000000);

	// sm_st.WritePosEx(3, 4400, 595, 0);
	// sm_st.WritePosEx(4, 2200, 175, 0);
	// sm_st.WritePosEx(5, 2200, 175, 0);
	// usleep(4*1000000);

	// sm_st.WritePosEx(3, 2200, 500, 0);
	// sm_st.WritePosEx(4, 4400, 500, 0);
	// usleep(5*1000000);
	
	// sm_st.WritePosEx(4, 2200, 500, 0);
	// sm_st.WritePosEx(5, 4400, 500, 0);
	// usleep(5*1000000);
	

	// sm_st.WritePosEx(3, 2700, 175, 0);
	// sm_st.WritePosEx(4, 2700, 175, 0);
	// sm_st.WritePosEx(5, 2700, 595, 0);
	sm_st.end();
	return 1;
}

