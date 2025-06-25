# SimpleNetwork

<pre><code>
$ ./server 5050 1 <br>
$ ./client 127.0.0.1 5050 message
</code></pre>


##################使用方法###############
进给电机:
1、roscore
2、rosrun liancheng_socket talk_test
3、rostopic echo /Controller_motor_order监控信息


俯仰电机:
1、roscore
2、rosrun liancheng_socket talk_test_wrist
3、rostopic echo /CANController_motor_order监控信息



