#include "EAGScrubber.h"
#include <px4_middleware.h>

#define SLEEP_INTERVAL_HZ 1

int EAGScrubber::job()
{
	px4::Rate *sleep_timer = new px4::Rate(SLEEP_INTERVAL_HZ);

  while(!this->should_exit()){
		/* working loop of this thread */
		sleep_timer->sleep();
	}

  this->thread_running = false;
	return 0;
}
