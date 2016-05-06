// #include <px4_tasks.h>
#include "SimpleTask.h"
#include "uORB/topics/eag_raw.h"

#define EAG_PUBLISHER_STACK_SIZE      2000
#define EAG_PUBLISHER_SCHED           SCHED_DEFAULT
#define EAG_PUBLISHER_SCHED_PRIORITY  SCHED_PRIORITY_DEFAULT

class EAGPublisher : public SimpleTask
{
public:

  EAGPublisher(): SimpleTask("EAGPublisher",
                              EAG_PUBLISHER_SCHED_PRIORITY,
                              EAG_PUBLISHER_STACK_SIZE,
                              EAG_PUBLISHER_SCHED)
  {
    _eag_raw_pub = orb_advertise(ORB_ID(eag_raw), &_eag_raw);

    _eag_device = "/dev/ttyS6";
  }
  ~EAGPublisher();

  int job();

private:

  char const * _eag_device;

  // Topics published to
  orb_advert_t _eag_raw_pub;
  struct eag_raw_s _eag_raw;

};
