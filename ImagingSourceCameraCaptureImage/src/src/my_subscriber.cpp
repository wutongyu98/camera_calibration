#include "ros/ros.h"
#include "camera_publish/my_msg.h" // here use the project pkg name "camera", not the folder name
#include <iostream>
using namespace std;
void clbk(const camera_publish::my_msg::ConstPtr& msg) {
    ROS_INFO("%d", msg->another_field);

    ROS_INFO("first point: q0=%.2f, q1=%.2f, q2=%.2f, q3=%.2f, q4=%.2f, q5=%.2f, q6=%.2f, q7=%.2f, q8=%.2f, q9=%.2f, q10=%.2f, q11=%.2f, q12=%.2f, q13=%.2f, q14=%.2f, q15=%.2f", msg->Q_array[0], msg->Q_array[1], msg->Q_array[2],msg->Q_array[3],msg->Q_array[4],msg->Q_array[5],msg->Q_array[6],msg->Q_array[7],msg->Q_array[8],msg->Q_array[9],msg->Q_array[10],msg->Q_array[11],msg->Q_array[12],msg->Q_array[13],msg->Q_array[14],msg->Q_array[15]);
    
    
    
 
}

int main(int argc, char **argv)
{
  // ROS objects
  ros::init(argc, argv, "my_subscriber");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("my_topic", 1, clbk);

  ros::spin();

}
