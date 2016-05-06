#include "EAGScrubber.h"
#include <px4_middleware.h>

#define SLEEP_INTERVAL_HZ 1

void EAGScrubber::start()
{
  SimpleTask::start();

  this->schedule(EAG_SCRUBBER_SCHED_PRIORITY,
                 EAG_SCRUBBER_SCHED);
}

void EAGScrubber::stop()
{
  SimpleTask::stop();
}

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
