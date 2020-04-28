# WJZ_Chassis
概述：ROS机器人底盘驱动，PC端包，主要针对差分驱动机器人和阿克曼转向的无人驾驶汽车，如果发现BUG或者需要我更新新的算法,或者需要技术合作,可以联系我修改或者pull request，如果觉得这个项目好请给我颗星星或者branch，这个项目会不断更新，谢谢大家！</br>
Overview: ROS chassis pkg for Differential drive robot and Ackermann steering robot like autocar,If you find a bug,or need me to update the new algorithm,or need technical cooperation, you can contact me to modify or pull the request. If you think this project is good, please give me a star or branch. This project will be constantly updated, thank you!</br>
BLOG:https://blog.csdn.net/qq_38588806</br>
联系方式 (contact)：</br>
163：wujiazheng2020@163.com</br>
gmail: wujiazheng2020@gmail.com</br>
QQ群：710805413</br>
算法大概介绍见(algorithm introduction):/doc</br>
# Dependece
1.ros (based on ros)
# contents:
## 1.Diff Robot
具体可见/doc介绍，代码如下：
`now_pose.x  += v*dt*cos(now_pose.th);`
`now_pose.y  += v*dt*sin(now_pose.th);`
`now_pose.th += w*dt;`
需要机器人用USB或者通过在机器人上放一个MCU如stm32，通过stm32读取驱动器/编码器的信息传输给PC段/MPU端，而PC端收到后发送odom消息和相应tf，同时如果PC端收到上层算法的消息，则发送给机器人的MCU端，该程序是PC端程序，因为stm32那个很简单就不写了。
## 2.Ackermann Steering
具体可见/doc介绍，代码如下：
`now_pose.x  += v*dt*cos(now_pose.th);`
`now_pose.y  += v*dt*sin(now_pose.th);`
`now_pose.th += v/wheel_base*tan(steer)*dt;`
需要无人车支持CAN并通过USBCAN连接到PC端，然后通过其他程序转述，这个只是例子，为了展现其运动学模型。
