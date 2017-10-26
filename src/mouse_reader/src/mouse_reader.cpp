
#include "mouse_reader/mouse_reader.h"


int Mouse::findMouse()
{
  int i, r;

  // using glob() [see: http://linux.die.net/man/3/glob ] for getting event files in /dev/input/
  glob_t gl;
  int num_event_dev = 0;					// number of relevant event files found in /dev/input/
  if(glob("/dev/input/by-id/*Logitech*", 0, NULL, &gl) == 0)		// looks for filenames that match the given pattern
  {
    num_event_dev = gl.gl_pathc;				// get the number of matching event files
  }
  
  printf("\x1b[1;32mHere is the list of likely candiates for your Mouse. This function always starts with the \x1b[4mfirst one on the list.\x1b[0m ");
  
  // print all the paths that were found by glob()
  for(i=0; i<num_event_dev; i++)
  {
    printf("[%d] %s \n",i, gl.gl_pathv[i]);
  }
  
  i = 0;
  r = openMouse(gl.gl_pathv[i]);
  printf("Opening mouse with i=: %d \n", i);
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


std::vector <int16_t> Mouse::processEvent(struct input_event *ev)
{

  std::vector <int16_t> event;			// output vector
  
  if (ev->type == EV_SYN)
  {
    //Do nothing
  }  
  else if (ev->type == EV_REL) 
  {
    printf("Mouse motion event %d : code :%d, value: %d \n", ev->type, ev->code, ev->value);
    event.push_back(ev->code);		// push event code to output vector
    event.push_back(ev->value);
    return event;
  }
  else 
  {
    printf("Other mouse event %d : code :%d, value: %d \n", ev->type, ev->code, ev->value);
  }

  return {-1, 0};
} // end processEvent

std::vector <int16_t> Mouse::getMouseMotionEvent()
{
  int const BUFFER_SIZE = 32;
  
  struct input_event ibuffer[BUFFER_SIZE];				// see: https://www.kernel.org/doc/Documentation/input/input.txt
  int r, events, i;
  std::vector <int16_t> event_info;					// processed event
  
  
  // read events to input_event buffer
  r = read(descriptor_, ibuffer, sizeof(struct input_event) * BUFFER_SIZE);
  
  if( r > 0 )
  {
      events = r / sizeof(struct input_event);				// getting the number of events
      //std::vector<std::vector<int16_t> > event_info_vectors(events);
      for(i=0; i<events; i++)						// going through all the read events
      {
	event_info = processEvent(&ibuffer[i]);				// call processEvent() for every read event
	return event_info;
	//event_info_vectors.push_back( event_info );			// return only the code for events different from 0; ie, only when key was pressed or depressed
      } //end for
  }
  else
  {
    fprintf(stderr, "read() failed: %s\n", strerror(errno));	// let user know that read() failed
    return {-1, 0};
  }
  return {-1, 0};
}
