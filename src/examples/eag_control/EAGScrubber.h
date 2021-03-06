// #include <px4_tasks.h>
#include "simple_task/SimpleTask.h"

#include "uORB/topics/eag_raw.h"
#include "uORB/topics/eag_scrubbed.h"

#define EAG_SCRUBBER_STACK_SIZE      2000
#define EAG_SCRUBBER_SCHED           SCHED_DEFAULT
#define EAG_SCRUBBER_SCHED_PRIORITY  SCHED_PRIORITY_DEFAULT

class EAGScrubber : public SimpleTask
{
public:

  EAGScrubber(): SimpleTask("EAGScrubber",
                            EAG_SCRUBBER_SCHED_PRIORITY,
                            EAG_SCRUBBER_STACK_SIZE,
                            EAG_SCRUBBER_SCHED)
  {
    _eag_scrubbed_pub = orb_advertise(ORB_ID(eag_scrubbed), &_eag_scrubbed_pub);
    _eag_raw_sub = orb_subscribe(ORB_ID(eag_raw));
  }
  ~EAGScrubber();

  int job();

private:

  orb_advert_t _eag_scrubbed_pub;
  struct eag_scrubbed_s _eag_scrubbed;

  int _eag_raw_sub;
  struct eag_raw_s _eag_raw;
};
