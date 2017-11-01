/** @file mouse_reader.h
 * 
 *  @author karl.kruusamae(at)utexas.edu
 * 
 *  NOTE: If you get permission denied when starting this node. Use ' ls -l /dev/input/event* ' to learn which group can access the events.
 *        Then add your username to this group with ' sudo usermod -a -G group_name user_name '
 */

#include <linux/input.h>
#include <string.h>
#include <map>
#include <vector>
#include <errno.h>
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <glob.h>	// for counting files in a dir (needed for counting event files in /dev/input)
#include <sstream>

#ifndef MOUSE_READER_H
#define MOUSE_READER_H

class Mouse
{
public:
  // Constructor
  Mouse (std::string mouse_event_path)
  {
    if ( mouse_event_path.empty() )
    {
      descriptor_ = findMouse();
    }
    else
    {
      descriptor_ = openMouse( mouse_event_path.c_str() );
    }
  };
  
  int findMouse();
  int openMouse(const char *device_path);

  void closeMouse();
  bool isReadable();
  //std::vector <int16_t> processEvent( struct input_event *ev );
  std::vector <int16_t> getMouseMotionEvent();
  //std::string getKeyName( uint16_t key_code );


private:
  int descriptor_;

  std::vector<std::string> valid_substrings =
  {
    "Logitech",
    "HP"    
  };

}; // end class Mouse

#endif
