#include "ros/ros.h"
#include "geometry_msgs/PointStamped.h"
#include "mouse_reader/MouseMotion.h"
#include <stdlib.h>     /* srand, rand */
#include <time.h>       /* time */

float xStart = 0;
float yStart = 0;
float zStart = 0;

void mouseCallback(const mouse_reader::MouseMotion::ConstPtr& msg)
{
  ROS_INFO("I heard mouse: code %d , value %d", msg->motion_code, msg->value);
  if (msg->motion_code == 0) 
    xStart += float(msg->value)/100;
  else if (msg->motion_code == 1) 
    yStart += float(msg->value)/100;
}

int main(int argc, char **argv)
{
  
  ros::init(argc, argv, "position_publisher_mouse");
  
  ros::NodeHandle n;
  
  ros::Publisher position_pub = n.advertise<geometry_msgs::PointStamped>("XYZ_positon", 1000);
  
  ros::Subscriber sub = n.subscribe("mouse", 1000, mouseCallback);

  ros::Rate loop_rate(1000);

  int count = 0;
    
  srand (time(NULL));
  
  while (ros::ok())
  {
    ros::spinOnce();               // check for incoming messages
    
    geometry_msgs::PointStamped msg;    
    msg.header.seq = count;
    msg.header.frame_id = "odom";
    
    msg.point.x = xStart;
    msg.point.y = yStart;
    msg.point.z = zStart;

    position_pub.publish(msg);
    
    ROS_INFO("Published: seq: %d, Point [%f %f %f]", msg.header.seq, msg.point.x, msg.point.y, msg.point.z);

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }


  return 0;
}