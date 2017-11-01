#include "ros/ros.h"
#include "mouse_reader/mouse_reader.h"
#include "mouse_reader/mouse_vel.h"

/** Main function and a ROS publisher */
int main(int argc, char *argv[]) {
  
  // ROS init
  ros::init(argc, argv, "mouse_event_publisher");
  // Use async spinner
  ros::AsyncSpinner spinner(1);
  spinner.start();

  // ROS node handle
  ros::NodeHandle nh;
  // Private node handle for optional ROS parameter _path
  ros::NodeHandle pnh("~");

  // Getting user-specified path from ROS parameter server
  std::string mouse_path;
  pnh.param<std::string>("path", mouse_path, "");
  
  if (mouse_path.empty())
  {
    ROS_INFO("No mouse specified, let's find one.");
  }

  // Create a Mouse object
  Mouse mouse(mouse_path);

  if(!mouse.isReadable())
  {
    ROS_INFO("Unable to locate mouse.");
    ROS_INFO("Try: %s [device]\n", argv[0]);
    return 1;
  }

  ros::Publisher pub_mouse = nh.advertise<mouse_reader::mouse_vel>("mouse", 1000);
  
  mouse_reader::mouse_vel motion_event;

  // Vector containing event data
  std::vector <int16_t> event;
  
  while(ros::ok())
  {
    ROS_INFO("Getting mouse event.");
    event = mouse.getMouseMotionEvent();	
    
    // Compose a publishable message
    motion_event.Xval = event[0];
    motion_event.Yval = event[1];
    if (event[0] != 0 || event[0] != 1)
    {
      ROS_INFO("Publishing xvel=%d,yvel=%d", event[0],event[1]);
      pub_mouse.publish(motion_event);
      
    }
    
  } // end while
  
  mouse.closeMouse();

  return 0;
} //end main
