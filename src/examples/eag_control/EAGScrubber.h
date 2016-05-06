// #include <px4_tasks.h>
#include "SimpleTask.h"

#define EAG_SCRUBBER_STACK_SIZE      1000
#define EAG_SCRUBBER_SCHED           SCHED_DEFAULT
#define EAG_SCRUBBER_SCHED_PRIORITY  SCHED_PRIORITY_DEFAULT

class EAGScrubber : public SimpleTask
{
public:

  EAGScrubber(): SimpleTask("EAGScrubber", EAG_SCRUBBER_STACK_SIZE) {}
  ~EAGScrubber();

  void start();

  void stop();

  int job();

private:

  orb_advert_t _eag_scrubbed_pub;

  int _eag_raw_sub;
};
