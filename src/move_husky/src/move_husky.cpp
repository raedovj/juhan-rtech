#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "mouse_reader/mouse_event_msg.h"
#include <time.h>       /* time */

bool move_r = false;

void mouseCallback(const mouse_reader::mouse_event_msg::ConstPtr& msg)
{
  ROS_INFO("I heard mouse: x %d , y %d", msg->Xval, msg->Yval);
  if (msg->mouseValue == 1) 
  {
    if (move_r)
      move_r = false;
    else
      move_r = true;
  }
}

int main(int argc, char **argv)
{
  
  ros::init(argc, argv, "move_husky");
  
  ros::NodeHandle n;
  
  ros::Publisher move_publisher = n.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
  
  ros::Subscriber sub = n.subscribe("mouse", 1000, mouseCallback);

  ros::Rate loop_rate(1000);
    
  srand (time(NULL));
  
  while (ros::ok())
  {
    ros::spinOnce();               // check for incoming messages
    
    geometry_msgs::Twist msg;  
    
    // Give robot linear x and angular z speed = will start going round in circles    
    msg.linear.x = -0.5; 
    msg.linear.y = 0.0;
    msg.linear.z = 0.0;
    
    msg.angular.x = 0.0;
    msg.angular.y = 0.0;
    msg.angular.z = 0.75;
    // if the robot is supposed to move, publish Twist on topic cmd_vel. 
    // If the robot isn't supposed to move, we could publish linear and angular (0,0,0)-s also
    // but the robot also stops, when there are no msg-s on cmd_vel
    if (move_r) 
    {
      move_publisher.publish(msg);
    
      ROS_INFO("Published: linear [%f %f %f]", msg.linear.x, msg.linear.y, msg.linear.z);
    }    

    ros::spinOnce();

    loop_rate.sleep();
  }


  return 0;
}