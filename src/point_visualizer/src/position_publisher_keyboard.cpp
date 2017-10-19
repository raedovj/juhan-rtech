#include "ros/ros.h"
#include "std_msgs/String.h"
#include "keyboard_reader/Key.h"
#include "geometry_msgs/PointStamped.h"


float x = 0;
float y = 0;
float z = 0;

void chatterCallback(const keyboard_reader::Key::ConstPtr& key_event)
{  
  if (key_event->key_pressed) 
  {
    ROS_INFO("Got key %d", key_event->key_code);
    if (key_event->key_pressed == 106) 
    { // right
      x = x + 0.1;
    }
    else if (key_event->key_pressed == 105) 
    { //left
      x = x - 0.1;
    }
    else if (key_event->key_pressed == 103) 
    { // up
      y = y + 0.1;
    }
    else if (key_event->key_pressed == 108) 
    { // down
      y = y - 0.1;
    }
  }
  
  /*geometry_msgs::PointStamped msg;    
  msg.header.seq = count;
  msg.header.frame_id = "odom";
    
  msg.point.x = x;
  msg.point.y = y;
  msg.point.z = z;

  position_pub.publish(msg);*/
    
}

int main(int argc, char **argv)
{
  
  ros::init(argc, argv, "position_publisher_keyboard");
  ros::NodeHandle n;
  //ros::Publisher position_pub = n.advertise<geometry_msgs::PointStamped>("XYZ_positon", 1000);
  
  ros::Subscriber sub = n.subscribe("keyboard", 1000, chatterCallback);

  ros::spin();

  return 0;
}