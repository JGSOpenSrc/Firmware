// #include <px4_tasks.h>
#include "SimpleTask.h"

#define EAG_PUBLISHER_STACK_SIZE      1000
#define EAG_PUBLISHER_SCHED           SCHED_DEFAULT
#define EAG_PUBLISHER_SCHED_PRIORITY  SCHED_PRIORITY_DEFAULT

class EAGPublisher : public SimpleTask
{
public:

  EAGPublisher(): SimpleTask("EAGPublisher", EAG_PUBLISHER_STACK_SIZE) {}
  ~EAGPublisher();

  void start();

  void stop();

  int job();

private:

  static const char* eag_device = "/dev/ttyS6";

  int pollfd;

  orb_advert_t eag_raw_pub;
};
