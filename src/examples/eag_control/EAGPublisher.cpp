#include "EAGPublisher.h"
#include <px4_posix.h>
#include <px4_log.h>

#define SLEEP_INTERVAL_HZ 1
#define READ_BUFFER_SIZE 10

int EAGPublisher::job()
{
  this->thread_running = true;

  // attempt to open the _eag_device
  int eag_fd = px4_open(_eag_device, O_RDONLY);

  if(0 >= eag_fd){
    this->thread_runtime_exception("exception occurred opening port");
  }

  // struct for polling device for new data
  px4_pollfd_struct_t pollfd;
  pollfd.fd = eag_fd;
  pollfd.events = POLLIN;

  uint8_t buffer[READ_BUFFER_SIZE];

  /* Working loop of this thread */
  while(!this->should_exit()){

    int ret = px4_poll(&pollfd, 1, 10000);

    // Unexpected behavior
    if(0 > ret){
      this->thread_runtime_exception("poll returned negative value");
      break;
    }

    // No data
    else if(0 == ret){
      PX4_WARN("[EAGPublisher] no data received in last ten seconds!");
    }

    // Got data
    else if(pollfd.revents & POLLIN){
      ret = px4_read(eag_fd, buffer, READ_BUFFER_SIZE);
      uint64_t time_stamp = hrt_absolute_time();

      // Publish the data
      int i;
      for(i = 0; i < ret; i++){
        _eag_raw.time_stamp = time_stamp;
        _eag_raw.raw_data = buffer[i];
        orb_publish(ORB_ID(eag_raw), _eag_raw_pub, &_eag_raw);
      }
    }
	}

  this->thread_running = false;
	return 0;
}
