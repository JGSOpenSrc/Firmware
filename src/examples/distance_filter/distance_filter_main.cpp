#include "simple_task/SimpleTask.h"
#include "uORB/uORB.h"
#include "uORB/topics/distance_sensor.h"
#include "uORB/topics/distance_sensor_filtered.h"

//#include "px4_log.h"
#include <px4_posix.h>
#include <cstring>
#include <cstdlib>

// Scheduler defines
#define DISTANCE_FILTER_STACK_SIZE      1000
#define DISTANCE_FILTER_SCHED           SCHED_DEFAULT
#define DISTANCE_FILTER_SCHED_PRIORITY  SCHED_PRIORITY_DEFAULT

// Inherits from SimpleTask posix threading implementation
class DistanceFilter : public SimpleTask
{
public:
  DistanceFilter() : SimpleTask("DistanceFilter",
                                DISTANCE_FILTER_SCHED_PRIORITY,
                                DISTANCE_FILTER_STACK_SIZE,
                                DISTANCE_FILTER_SCHED)
  {
  }

  ~DistanceFilter();
  int job();
};

#define FILTER_ORDER 7

int DistanceFilter::job(){
  this->thread_running = true;

  // declaration of file descriptors and uORB subscriptions / publications
  static orb_advert_t distance_sensor_filtered_pub;
  static struct distance_sensor_filtered_s sensor_filtered;
  distance_sensor_filtered_pub = orb_advertise(ORB_ID(distance_sensor_filtered),
                                                &sensor_filtered);

  static int distance_sensor_sub;
  static struct distance_sensor_s sensor;
  distance_sensor_sub = orb_subscribe(ORB_ID(distance_sensor));

  // declare and initialize filter memory
  float y[FILTER_ORDER+1];
  float x[FILTER_ORDER];
  int i;
  for(i = 0; i < FILTER_ORDER; i++){
    y[i] = 0.f;
    x[i] = 0.f;
  }
  // declare and initialize filter coefficients
  float a[FILTER_ORDER+1] = {1., -3.33308141, 5.51763428, -5.46718602, 3.37223579, -1.21801153, 0.20129298, 1.f};

  float b[FILTER_ORDER] = {0.00112578, 0.00675467, 0.01688667, 0.02251556, 0.01688667, 0.00675467, 0.00112578};

  // declare and initialize px4_poll struct
  px4_pollfd_struct_t pollfd;
  pollfd.fd = distance_sensor_sub;
  pollfd.events = POLLIN;

  while(!this->should_exit())
  {
    // Poll the distance sensor sub for 150 milliseconds (should update at 10Hz)
    int ret = px4_poll(&pollfd, 1, 150);

    if(ret < 0)
    {
      warnx("[DistanceFilter] poll of distance sensor returned %d", ret);
    }

    else if(ret == 0)
    {
      warnx("[DistanceFilter] no data received in 150ms");
    }

    else if(pollfd.revents & POLLIN)
    {
      // Pull the data out
      orb_copy(ORB_ID(distance_sensor), distance_sensor_sub, &sensor);

      if(sensor.type == sensor.MAV_DISTANCE_SENSOR_INFRARED){
        // Update the filter output
        for(i = 1; i < FILTER_ORDER; i++)
        {
          x[FILTER_ORDER-i] = x[FILTER_ORDER-i-1];
          y[FILTER_ORDER-i] = y[FILTER_ORDER-i-1];
        }

        x[0] = sensor.current_distance;
        y[0] = x[0]*b[0];
        for(i = 1; i < FILTER_ORDER; i++)
        {
          y[0] += b[i]*x[i];
        }

        for(i = 1; i < FILTER_ORDER+1; i++)
        {
          y[0] -= a[i]*y[i];
        }

        // Publish the filtered sensor value
        sensor_filtered.timestamp = sensor.timestamp;
        sensor_filtered.current_distance = y[0];
        sensor_filtered.covariance = sensor.covariance;


        orb_publish(ORB_ID(distance_sensor_filtered),
                          distance_sensor_filtered_pub,
                          &sensor_filtered);
      }
    }
  }
  return 0;
}

/**
 * Print the correct usage.
 */
static void usage(const char *reason);

static void usage(const char *reason)
{
	if (reason) {
		warnx("%s\n", reason);
	}

	warnx("usage: distance_filter {start|stop|status} [-p <additional params>]\n\n");
}

static bool command_is_good(char* command);

static bool command_is_good(char* command)
{
  if(!strcmp(command,"start")) return true;
  if(!strcmp(command,"stop")) return true;
  if(!strcmp(command,"status")) return true;
  return false;
}

DistanceFilter *filter = NULL;

extern "C" __EXPORT int distance_filter_main(int argc, char* argv[]);
int distance_filter_main(int argc, char* argv[])
{

  // Check the user input
  if(argc < 2)
  {
    usage("at least one input argument required");
    return -1;
  }

  if(!command_is_good(argv[1]))
  {
    usage("invalid command");
    return -1;
  }

  // if this is the first time this module has been run,
  // allocate the filter
  if(NULL == filter)
  {
    filter = new DistanceFilter();
  }

  // Handle user command
  if(!strcmp(argv[1], "start"))
  {
    if(filter->is_running())
    {
      warnx("Filter is already running");
    }
    else filter->start();
  }

  else if(!strcmp(argv[1], "stop"))
  {
    if(filter->is_running())
    {
      filter->stop();
    }
    else
    {
      warnx("Filter is not running");
    }
  }

  else if(!strcmp(argv[1], "status"))
  {
    if(filter->is_running()) {
      warnx("Distance Filter is running");
    }
    else {
      warnx("Distance Filter is not running");
    }
  }

  return 0;

}
