/**
 * @file main
 * @author sunbin
 * @brief 通信包，用来与stm32通信
 * @version 0.1
 *
 * @copyright Copyright (c) 2022
 *
 */

#include <ros/ros.h>
#include <serial/serial.h>
#include <iostream>
using namespace std;

#include "fake_mavros/write_to_stm32.h"
#include "fake_mavros/read_from_stm32.h"

#define REC_HEAD_LENGTH 2
#define REC_PAYLOAD_LENGTH 28
#define REC_BAG_LENGTH  (REC_HEAD_LENGTH + REC_PAYLOAD_LENGTH)
#define SEND_HEAD_LENGTH 2
#define SEND_PAYLOAD_LENGTH 24
#define SEND_BAG_LENGTH  (SEND_HEAD_LENGTH + SEND_PAYLOAD_LENGTH)

uint8_t REC_HEADER[2] = {0xAA, 0xBB};
uint8_t SEND_HEADER[2] = {0xBB, 0xAA};

/**
 * @brief 接受包，接受下位機反馈信息
 *
 */
typedef union
{
    uint8_t data[REC_BAG_LENGTH];
    struct
    {
        uint8_t header[REC_HEAD_LENGTH];
        union
        {
            uint8_t payload[REC_PAYLOAD_LENGTH];
            struct
            {
                float pos_x;
                float pos_y;
                float w_z;
                float xangle;
                float yangle;
                float zangle;
                float point;
            }__attribute__((packed));
        };
    }__attribute__((packed));
}__attribute__((packed)) RECV_Bag_u;

/**
 * @brief 发送包，发送控制信息
 *
 */
typedef union
{
    uint8_t data[SEND_BAG_LENGTH];
    struct
    {
        uint8_t header[SEND_HEAD_LENGTH];
        union
        {
            uint8_t payload[SEND_PAYLOAD_LENGTH];
            struct
            {
                float vx_set;
                float vy_set;
                float vw_set;
                float x_set;
                float y_set;
                float w_set;
            }__attribute__((packed));
        };
    }__attribute__((packed));
}__attribute__((packed)) SEND_Bag_u;

fake_mavros::read_from_stm32  read_from_stm32_data;

ros::Publisher stm32_pub;
serial::Serial serialPort;
ros::Duration d(0.01);
bool first_connect =true;
ros::Time start_time;
int last_point;

/**
 * @brief 初始化串口Sp
 * @param sp
 * @param portName 串口名，默认为"/dev/ttyUSB0"
 * @param baudrate 波特率，默认为115200
 * @param timeOut  传输延时，默认为2
 * @return 成功返回1,否则返回0
 */
int SerialPort_INIT(serial::Serial *sp, char* portName = "/dev/ttyACM0", int baudrate = 115200, int timeOut = 1000)
{
    serial::Timeout to = serial::Timeout::simpleTimeout(timeOut);
    sp->setPort(portName);
    sp->setBaudrate(baudrate);
    sp->setTimeout(to);
    try
    {
        sp->open();
    }
    catch(serial::IOException& e)
    {
        ROS_ERROR_STREAM("Unable to open port.");
        return 0;
    }

    if(sp->isOpen())
    {
        ROS_INFO_STREAM("Port is opened.");
    }
    else
    {
        return 0;
    }
    return 1;
}

/**
 * @brief 收到控制信号后，将其传给下位机
 * @param ctrl_msg
 */
void ctrlDataCallback(const fake_mavros::write_to_stm32::ConstPtr& write_msg)
{
    SEND_Bag_u tempBag;
    tempBag.header[0] = SEND_HEADER[0];
    tempBag.header[1] = SEND_HEADER[1];

    tempBag.vx_set = write_msg->vx_set;
    tempBag.vy_set = write_msg->vy_set;
    tempBag.vw_set= write_msg->vw_set;
    tempBag.x_set = write_msg->x_set;
    tempBag.y_set = write_msg->y_set;
    tempBag.w_set= write_msg->w_set;

    tempBag.vx_set = 3.1415;

    serialPort.write(tempBag.data, SEND_BAG_LENGTH);
    std::cout<<"I write: vx="<<tempBag.vx_set<<",vy="<<tempBag.vy_set<<",vw="<<tempBag.vw_set<<std::endl;
    std::cout<<tempBag.header[0]<<"------------"<<tempBag.header[1]<<std::endl;
    d.sleep();
}

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "motor_serial");
    ros::NodeHandle nh;
    ros::Rate loop_rate(100);
    SEND_Bag_u tempBag;

    ros::Subscriber serial_sub = nh.subscribe<fake_mavros::write_to_stm32>("/fake_mavros/speed_control", 10,  ctrlDataCallback);
    stm32_pub = nh.advertise<fake_mavros::read_from_stm32>("/fake_mavros/joy_control", 10);

    ros::AsyncSpinner spinner(0);
    spinner.start();


    if(!SerialPort_INIT(&serialPort))
    {
        return  -1;
    }

    while(ros::ok())
     {
        ros::Time right_now = ros::Time::now();
    	ros::Duration pass_time = right_now-start_time;
        //获取缓冲区内的字节数，直至其长度大于等于两个包的长度
        size_t recv_len = 0;
        for(recv_len = 0; recv_len < 2*REC_BAG_LENGTH; recv_len = serialPort.available());

        //读取缓存区中数据
        uint8_t RxBuffer[2*REC_BAG_LENGTH];
        recv_len = serialPort.read(RxBuffer, 2*REC_BAG_LENGTH);

        //将电机反馈值发送出去
        for(int i=0; i<2*REC_BAG_LENGTH; ++i)
        {
            RECV_Bag_u tempBag = *(RECV_Bag_u*) (void *) (&(RxBuffer[i]));
            if(tempBag.header[0] == REC_HEADER[0] && tempBag.header[1] == REC_HEADER[1])
            {
                if(last_point!=tempBag.point&&pass_time.toSec()>1)
                {
                    read_from_stm32_data.point = tempBag.point;
                    stm32_pub.publish(read_from_stm32_data);
                }
                else
                {
                    read_from_stm32_data.point = -1;
                    stm32_pub.publish(read_from_stm32_data);
                }
                last_point = tempBag.point;
                first_connect = false;

                // std::cout<<"point="<<tempBag.point<<std::endl;
                // std::cout<<"x="<<tempBag.pos_x<<std::endl;
                // std::cout<<"y="<<tempBag.pos_y<<std::endl;
                // stm32_pub.publish(read_from_stm32_data);
                break;
            }
        }
    }
    //关闭串口
    serialPort.close();
    return 0;
}