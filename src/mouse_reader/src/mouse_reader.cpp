#include "mouse_reader/mouse_reader.h"
#include <sstream>
#include <string>


int Mouse::findMouse()
{
  int i, r;

  // using glob() [see: http://linux.die.net/man/3/glob ] for getting event files in /dev/input/
  glob_t gl;
  int num_event_dev = 0;					// number of relevant event files found in /dev/input/
  
  for (i=0; i < valid_substrings.size(); i++)
  {
    std::stringstream ss;
    ss << "/dev/input/by-id/*" << valid_substrings[i] << "*";
    std::string s = ss.str();
    const char * c = s.c_str();
    if(glob(c, 0, NULL, &gl) == 0)		// looks for filenames that match the given pattern
    {
      num_event_dev = gl.gl_pathc;				// get the number of matching event files
      if (num_event_dev != 0)
	break;
    }
  }
  r = openMouse(gl.gl_pathv[0]);
  printf("Opening mouse");
  globfree(&gl);
  return r;
} 


int Mouse::openMouse( const char *device_path )
{
  printf("Opening device: %s \n", device_path);
  int fd = open(device_path, O_RDONLY);
  printf("Opening device return number: %d \n", fd);
  return fd;
} 


void Mouse::closeMouse()
{
  printf("Closing mouse device.\n");
  close(descriptor_);
  return;
}


bool Mouse::isReadable ()
{
  if (descriptor_ < 0) return false;
  return true;
}

std::vector <int16_t> Mouse::getMouseMotionEvent()
{
  int const BUFFER_SIZE = 10;
  
  struct input_event ibuffer[BUFFER_SIZE];				// see: https://www.kernel.org/doc/Documentation/input/input.txt
  int r, events, i;
  std::vector <int16_t> event_info;					// processed event  
  
  // read events to input_event buffer
  r = read(descriptor_, ibuffer, sizeof(struct input_event) * BUFFER_SIZE);
  
  // Use these to average mouse motion
  int16_t xSum = 0; 
  int16_t ySum = 0;
  int16_t validReadings = 0;
  
  if( r > 0 )
  {
      events = r / sizeof(struct input_event);				// getting the number of events
      for(i=0; i<events; i++)						// going through all the read events
      {
	struct input_event ev = ibuffer[i];
	if (ev.type == EV_REL) 
	{
	  if (ev.code  == 0 )
	  {
	    xSum += (ev.value);
	    validReadings++;
	  }
	  if (ev.code  == 1) 
	  {
	    ySum += (ev.value);
	    validReadings++;
	  }
	}
      } //end for
      if (validReadings > 0) 
      {
	event_info.push_back(xSum*10.0/validReadings);
	event_info.push_back(ySum*10.0/validReadings);
	return event_info;
      }
      return {0, 0};
  }
  else
  {
    fprintf(stderr, "read() failed: %s\n", strerror(errno));	// let user know that read() failed
    return {0, 0};
  }
  return {0, 0};
}
