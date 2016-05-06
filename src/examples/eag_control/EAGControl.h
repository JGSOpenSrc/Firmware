// #include <px4_tasks.h>
#include "SimpleTask.h"
#include "EAGPublisher.h"
#include "EAGScrubber.h"

#include "uORB/uORB.h"
#include "uORB/topics/offboard_control_mode.h"
#include "uORB/topics/vehicle_force_setpoint.h"
#include "uORB/topics/position_setpoint_triplet.h"
#include "uORB/topics/vehicle_control_mode.h"
#include "uORB/topics/eag_scrubbed.h"
#include "uORB/topics/vehicle_attitude.h"
#include "uORB/topics/manual_control_setpoint.h"


#define EAG_CONTROL_STACK_SIZE     1000
#define EAG_CONTROL_SCHED          SCHED_DEFAULT
#define EAG_CONTROL_SCHED_PRIORITY SCHED_PRIORITY_DEFAULT


class EAGControl : public SimpleTask
{

public:

  EAGControl() : SimpleTask("EAGContol", EAG_CONTROL_STACK_SIZE),
  eag_publisher(EAGPublisher()),
  eag_scrubber(EAGScrubber())
  {
    _offboard_control_mode_pub = orb_advertise(ORB_ID(offboard_control_mode),
                                &_offboard_control);

    _force_sp_pub = orb_advertise(ORB_ID(vehicle_force_setpoint), &_force_sp);

    _force_sp_triplet_pub = orb_advertise(ORB_ID(position_setpoint_triplet),
                            &_position_sp);

    _control_mode_sub = orb_subscribe(ORB_ID(vehicle_control_mode));

    _eag_scrubbed_sub = orb_subscribe(ORB_ID(eag_scrubbed));

    _vehicle_att_sub = orb_subscribe(ORB_ID(vehicle_attitude));

    _manual_sp_sub = orb_subscribe(ORB_ID(manual_control_setpoint));
  }

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

  // Topics published to
  orb_advert_t _offboard_control_mode_pub;
  struct offboard_control_mode_s _offboard_control;

  orb_advert_t _force_sp_pub;
  struct vehicle_force_setpoint_s _force_sp;

  orb_advert_t _force_sp_triplet_pub;
  struct position_setpoint_triplet_s _position_sp;

  // Topics subscribed to
  int _control_mode_sub;
  struct vehicle_control_mode_s _control_mode;

  int _eag_scrubbed_sub;
  struct eag_scrubbed_s _eag_scrubbed;

  int _vehicle_att_sub;
  struct vehicle_attitude_s _vehicle_att;

  int _manual_sp_sub;
  struct manual_control_setpoint_s _manual_sp;
};
