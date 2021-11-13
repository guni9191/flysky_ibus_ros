#include <ros/ros.h>
#include <stdio.h>
#include <time.h>
#include "ibus.h"
#include <geometry_msgs/Twist.h>

int main(int argc, char** argv)
{
    // other param
    uint32_t serial_baudrate = 115200;
    double ch3_updown = 0;
    double ch1_leftright = 0;
    
    // ros param
    std::string frame_id = std::string("base_link");
    std::string pub_topicname_cmdvel = std::string("cmd_vel");
    std::string usb_port = std::string("/dev/ttyUSB0");
    bool ang_reverse = false;
    
    ros::init(argc, argv, "gl_ros_driver_udp_node");
    ros::NodeHandle nh;
    ros::NodeHandle nh_priv("~");

    nh_priv.param("frame_id", frame_id, frame_id);
    nh_priv.param("usb_port", usb_port, usb_port);
    nh_priv.param("pub_topicname_cmdvel", pub_topicname_cmdvel, pub_topicname_cmdvel);
    nh_priv.param("ang_reverse", ang_reverse, ang_reverse);

    ros::Publisher cmdvel_pub = nh.advertise<geometry_msgs::Twist>(pub_topicname_cmdvel, 10);
    
    ibusdriver ibus(usb_port,serial_baudrate);
    
    // loop
    ros::Rate loop_rate(80);
    while(ros::ok())
    {
        ch3_updown = (double)((int)ibus.final_output[2]-1500)/500.0;    //raw up 2000 down 1000, middle 1500 ==> -1 to 1 convert
        ch1_leftright = (double)((int)ibus.final_output[0]-1500)/500.0; //raw left 1000 right 2000 ==> -1 to 1 convert
        if(ang_reverse == true) ch1_leftright = -ch1_leftright;

        geometry_msgs::Twist vel;
        vel.linear.x = ch3_updown;
        vel.angular.z = ch1_leftright;
        cmdvel_pub.publish(vel);

        // std::cout<<"leftright:"<<ch1_leftright<<",updown:"<<ch3_updown<<std::endl;
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
