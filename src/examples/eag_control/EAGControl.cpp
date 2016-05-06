#include "EAGControl.h"
#include <px4_middleware.h>

#define SLEEP_INTERVAL_HZ 1

void EAGControl::start()
{
	SimpleTask::start();
	eag_publisher.start();
	// eag_scrubber.start();
}

void EAGControl::stop()
{
	SimpleTask::stop();
	eag_publisher.stop();
	eag_scrubber.stop();
}

int EAGControl::job()
{
	this->thread_running = true;
	px4::Rate *sleep_timer = new px4::Rate(SLEEP_INTERVAL_HZ);
	while(!this->should_exit()){
		/* working loop of this thread */
		sleep_timer->sleep();
	}

	this->thread_running = false;
	return 0;
}
