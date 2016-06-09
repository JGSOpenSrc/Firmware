// #include <px4_tasks.h>
#include "simple_task/SimpleTask.h"
#include "EAGPublisher.h"
#include "EAGScrubber.h"

#include <px4_posix.h>

#define EAG_CONTROL_STACK_SIZE     2000
#define EAG_CONTROL_SCHED          SCHED_DEFAULT
#define EAG_CONTROL_SCHED_PRIORITY SCHED_PRIORITY_DEFAULT

#define NUM_EAG_CONTROL_SUBS       3

class EAGControl : public SimpleTask
{

public:

  EAGControl() : SimpleTask("EAGControl",
                            EAG_CONTROL_SCHED_PRIORITY,
                            EAG_CONTROL_STACK_SIZE,
                            EAG_CONTROL_SCHED),
  eag_publisher(EAGPublisher()),
  eag_scrubber(EAGScrubber())
  {}

  ~EAGControl();

  bool eag_publisher_alive() { return this->eag_publisher.is_running(); }

  bool eag_scrubber_alive() { return this->eag_scrubber.is_running(); }

  void start();

  void stop();

  int job();


private:
  // Child threads
  EAGPublisher eag_publisher;     // Collects and publishes raw eag data
  EAGScrubber  eag_scrubber;      // Filters eag data and publishes output
};
