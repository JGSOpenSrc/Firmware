#include "EAGControl.h"
#include <px4_log.h>
#include <cstring>
#include <cstdlib>

EAGControl *controller = NULL;

/**
 * Print the correct usage.
 */
static void usage(const char *reason);

static void usage(const char *reason)
{
	if (reason) {
		warnx("%s\n", reason);
	}

	warnx("usage: eag_control {start|stop|status} [-p <additional params>]\n\n");
}

static bool command_is_good(char* command);

static bool command_is_good(char* command)
{
  if(!strcmp(command,"start")) return true;
  if(!strcmp(command,"stop")) return true;
  if(!strcmp(command,"status")) return true;
  return false;
}

extern "C" __EXPORT int eag_control_main(int argc, char* argv[]);
int eag_control_main(int argc, char* argv[])
{
  if(argc < 2){
    usage("at least one argument required");
    return -1;
  }

  if(!command_is_good(argv[1])){
    usage("invalid command argument");
    return -1;
  }
  // if this is the first time this has been called,
  // allocate the controller
  if(NULL == controller){
    controller = new EAGControl();
  }

  if(!strcmp(argv[1], "start")){

    if(controller->is_running()){
      warnx("Controller is already running");
    }
    else controller->start();
  }

  else if(!strcmp(argv[1],"stop")){

    if(controller->is_running()){
      controller->stop();
    }
    else warnx("Controller is not running");
  }

  else if(!strcmp(argv[1],"status")){
    controller->is_running()  ? warnx("EAG Control is running")
                              : warnx("EAG Control is not running");
    controller->eag_publisher_alive() ? warnx("EAG Publisher is running")
                                      : warnx("EAG Publisher is not running");
    controller->eag_scrubber_alive()  ? warnx("EAG Scrubber is running")
                                      : warnx("EAG Scrubber is not running");
  }
  return 0;
}
