#include <iostream>
#include "SCServo.h"

SMS_STS sm_st;

int main(int argc, char **argv)
{
	//检查是否传入串口设备路径。若未传入（argc<2），打印错误并退出。
	if(argc<2){
        std::cout<< "argc error!"<<std::endl;
        return 0;
	}
	//初始化串口，波特率 115200，设备路径为 argv[1]
	std::cout<<"serial:"<<argv[1]<<std::endl;
    if(!sm_st.begin(115200, argv[1])){
        std::cout<<"Failed to init sms/sts motor!"<<std::endl;
        return 0;
    }
    //对 ID=1 的舵机执行零点校准（机械位置归零）。
	//输出提示：打印校准完成信息。
	sm_st.CalibrationOfs(1);
	std::cout<<"Calibration Ofs"<<std::endl;
	sm_st.end();
	return 1;
}

