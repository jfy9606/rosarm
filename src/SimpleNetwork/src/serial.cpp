#include <iostream>
#include <signal.h>
#include "RS485Serial.h"





int main(int argc, char **argv){


   ROS_INFO("starting");
    ros::init( argc,argv, "serial" );
    ros::NodeHandle nh;
    serialpublisher serialpub(nh);
    // serialpub.prepare();
    // int station_num;
    // cout<<"input station num"<<endl;
    // cin>>station_num;

    // // int16_t vel;
    // // cout<<"input vel (-6000-6000)"<<endl;
    // // cin>>vel;
    // cout<<int(station_num)<<endl;
    // serialpub.pos_form(uint8_t(station_num),true,20000, 10,1000,0);
    ros::Rate rate(50.0);
    

    // while(1)
    // {
    //     ros::spinOnce();
    //     int a;
    //     cin>>a;
    //     if  (a==1)
    //     {
    //         serialpub.enable_on(uint8_t(station_num));
    //     }
    //     else if (a==0)
    //     {
    //         serialpub.enable_off(uint8_t(station_num));
    //     }
    //     else
    //     {
    //         serialpub.enable_off(uint8_t(station_num));
    //         return 0;
    //     }
        
    // }

    ros::spin();
    
    return 0;
}


