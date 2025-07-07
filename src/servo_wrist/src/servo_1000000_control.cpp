#include <iostream>
#include "SCServo_Linux/SCServo.h"

SMS_STS sm_st;

int main(int argc, char **argv)
{
    if(argc<2){
        std::cout<<"使用方法: "<<argv[0]<<" <串口设备>"<<std::endl;
        std::cout<<"例如: "<<argv[0]<<" /dev/ttyUSB0"<<std::endl;
        return 0;
    }
    std::cout<<"使用串口:"<<argv[1]<<std::endl;
    
    // 将波特率设置为1000000，适用于STS3020和STS3032舵机
    if(!sm_st.begin(1000000, argv[1])){
        std::cout<<"初始化SMS/STS电机失败!"<<std::endl;
        return 0;
    }
    
    // 示例：控制ID为1的舵机移动到位置2000，速度500，加速度50
    int servoID = 1;
    int position = 2000;
    int velocity = 500;
    int acceleration = 50;
    
    std::cout<<"控制舵机ID: "<<servoID<<" 移动到位置: "<<position<<std::endl;
    sm_st.WritePosEx(servoID, position, velocity, acceleration);
    
    // 等待舵机到达目标位置
    usleep(2*1000*1000);
    
    // 读取当前位置
    int feedback = sm_st.ReadPos(servoID);
    if(feedback != -1){
        std::cout<<"舵机当前位置: "<<feedback<<std::endl;
    }
    else{
        std::cout<<"读取舵机位置失败"<<std::endl;
    }
    
    // 关闭连接
    sm_st.end();
    return 1;
} 