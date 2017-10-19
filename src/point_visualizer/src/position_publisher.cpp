#include "ros/ros.h"
#include "geometry_msgs/PointStamped.h"
#include <stdlib.h>     /* srand, rand */
#include <time.h>       /* time */

/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv)
{
  
  ros::init(argc, argv, "position_publisher");
  
  ros::NodeHandle n;
  
  ros::Publisher position_pub = n.advertise<geometry_msgs::PointStamped>("XYZ_positon", 1000);

  ros::Rate loop_rate(1);

  int count = 0;
  
  srand (time(NULL));
  
  while (ros::ok())
  {
    /**
     * This is a message object. You stuff it with data, and then publish it.
     */
    
    geometry_msgs::PointStamped msg;    
    msg.header.seq = count;
    msg.header.frame_id = "odom";
    
    msg.point.x = (rand() % 1000)/1000.0;
    msg.point.y = (rand() % 1000)/1000.0;
    msg.point.z = (rand() % 1000)/1000.0;

    position_pub.publish(msg);
    
    ROS_INFO("Published: seq: %d, Point [%f %f %f]", msg.header.seq, msg.point.x, msg.point.y, msg.point.z);

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }


  return 0;
}