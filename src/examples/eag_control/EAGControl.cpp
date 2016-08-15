#include "EAGControl.h"

#include <drivers/drv_hrt.h>
#include <px4_middleware.h>
#include <cstring>

#include "uORB/uORB.h"
#include "uORB/topics/offboard_control_mode.h"
#include "uORB/topics/vehicle_control_mode.h"
#include "uORB/topics/eag_scrubbed.h"
#include "uORB/topics/vehicle_attitude.h"
#include "uORB/topics/vehicle_attitude_setpoint.h"
#include "uORB/topics/vehicle_local_position.h"
#include "uORB/topics/vehicle_local_position_setpoint.h"
#include "uORB/topics/manual_control_setpoint.h"

#define SLEEP_INTERVAL_HZ 10

// declaration of file descriptors  // Topics published to
static orb_advert_t offboard_control_mode_pub;
static struct offboard_control_mode_s *offboard_control;

static orb_advert_t vehicle_local_pos_sp_pub;
static struct vehicle_local_position_setpoint_s *local_pos_sp;

static orb_advert_t vehicle_attitude_sp_pub;
static struct vehicle_attitude_setpoint_s *att_sp;

// Topics subscribed to
static int control_mode_sub;
static struct vehicle_control_mode_s *control_mode;

static int eag_scrubbed_sub;
static struct eag_scrubbed_s *eag_scrubbed;

static int vehicle_att_sub;
static struct vehicle_attitude_s *vehicle_att;

static int vehicle_local_pos_sub;
static struct vehicle_local_position_s *vehicle_local_pos;

int manual_sp_sub;
struct manual_control_setpoint_s *manual_sp;

void EAGControl::start()
{
	SimpleTask::start();
	// eag_publisher.start();
	// eag_scrubber.start();
}

void EAGControl::stop()
{
	SimpleTask::stop();
	// eag_publisher.stop();
	// eag_scrubber.stop();
}

void initialize_publications();
void initialize_subscriptions();
void copy_subscriptions();
void delete_publications();
void delete_subscriptions();
void decode_control_mode();

int EAGControl::job()
{
	PX4_INFO("RUNNING JOB");
	this->thread_running = true;

	static hrt_abstime this_time = 0;

	// Initialize orb publications and advertiser
	initialize_publications();

	// Initialize orb subscriptions
	initialize_subscriptions();

	copy_subscriptions();

	// get the initial attitude and save the yaw - I don't want the controller
	// to automatically compensate initial yaw in NED frame
	struct vehicle_attitude_s initial_att; 				memset(&initial_att, 0, sizeof(vehicle_attitude_s));
	struct vehicle_local_position_s initial_lpe; 	memset(&initial_lpe, 0, sizeof(vehicle_local_position_s));
	bool est_initialized = false;

	// TODO: Need to initialize attitude and local position variables before takeoff

	px4::Rate *sleep_timer = new px4::Rate(SLEEP_INTERVAL_HZ);
	while(!this->should_exit()){
		/* working loop of this thread */

		this_time = hrt_absolute_time();

		copy_subscriptions();

		// if offboard mode
		if(control_mode->flag_control_offboard_enabled)
		{
			// Get the initial local position and attitude estimates for calculating
			// relative control setpoints.
			if(!est_initialized)
			{
				memcpy(&initial_att, vehicle_att, sizeof(vehicle_attitude_s));
				memcpy(&initial_lpe, vehicle_local_pos, sizeof(vehicle_local_position_s));
				est_initialized = true;
			}

			float takeoff_thrust = 10.30f; // Newtons
			if(!vehicle_local_pos->dist_bottom_valid){
				PX4_INFO("TAKE OFF AT GROUND DISTANCE %f", static_cast<double>(100.0f * vehicle_local_pos->dist_bottom));
				// TODO incease altitude until the range sensor measurement is valid

				att_sp->roll_body 					= (float) 0;
				att_sp->pitch_body 					= (float)	0;
				att_sp->yaw_body 						= (float) initial_att.yaw;
				att_sp->yaw_sp_move_rate 		=	(float) 0;
				att_sp->R_valid							= (bool) false;
				att_sp->q_d_valid						= (bool) false;
				att_sp->q_e_valid						= (bool) false;
				att_sp->thrust							= takeoff_thrust;
				att_sp->roll_reset_integral			= false;
				att_sp->pitch_reset_integral 		= false;
				att_sp->yaw_reset_integral 			= false;
				att_sp->fw_control_yaw					= false;
				att_sp->disable_mc_yaw_control 	= false;
				att_sp->apply_flaps							= false;

				orb_publish(ORB_ID(vehicle_attitude_setpoint),
													vehicle_attitude_sp_pub,
													att_sp);
			}
			else if(!vehicle_local_pos->xy_valid){
				PX4_INFO("XY INVALID AT GROUND DISTANCE %f", static_cast<double>(100.0f * vehicle_local_pos->dist_bottom));
				// Hold attitude
				att_sp->roll_body 					= (float) 0;
				att_sp->pitch_body 					= (float)	0;
				att_sp->yaw_body 						= (float) initial_att.yaw;
				att_sp->yaw_sp_move_rate 		=	(float) 0;
				att_sp->R_valid							= (bool) false;
				att_sp->q_d_valid						= (bool) false;
				att_sp->q_e_valid						= (bool) false;
				att_sp->thrust							= (float) 0;
				att_sp->roll_reset_integral			= false;
				att_sp->pitch_reset_integral 		= false;
				att_sp->yaw_reset_integral 			= false;
				att_sp->fw_control_yaw					= false;
				att_sp->disable_mc_yaw_control 	= false;
				att_sp->apply_flaps							= false;

				orb_publish(ORB_ID(vehicle_attitude_setpoint),
													vehicle_attitude_sp_pub,
													att_sp);
			}
			else {
				// TODO hold altitude, stabilize attitude, and begin casting

				// ascend to z_setpoint and hold position
				float ground_distance_setpoint = 1.0f; // (meters)
				float z_setpoint =  vehicle_local_pos->z - (ground_distance_setpoint - vehicle_local_pos->dist_bottom);
				const float tolerance = .1f;

				if(vehicle_local_pos->dist_bottom < ground_distance_setpoint - tolerance){
					PX4_INFO("ASCEND AT GROUND DISTANCE %f", static_cast<double>(vehicle_local_pos->dist_bottom * 100.0f));
				}
				else if(vehicle_local_pos->dist_bottom > ground_distance_setpoint + tolerance){
					PX4_INFO("DESCEND AT GROUND DISTANCE %f", static_cast<double>(vehicle_local_pos->dist_bottom * 100.0f));
				}
				else{
					PX4_INFO("HOLD Z AT GROUND DISTANCE %f", static_cast<double>(vehicle_local_pos->dist_bottom * 100.0f));
				}

				local_pos_sp->x							= (float) initial_lpe.x;
				local_pos_sp->y							= (float) initial_lpe.y;
				local_pos_sp->z							= (float) z_setpoint;
				local_pos_sp->yaw						= (float) initial_att.yaw;
				local_pos_sp->vx						= (float) 0;
				local_pos_sp->vy						= (float) 0;
				local_pos_sp->vz						= (float) 0;
				local_pos_sp->acc_x					= (float) 0;
				local_pos_sp->acc_y					= (float) 0;
				local_pos_sp->acc_z					= (float) 0;

				orb_publish(ORB_ID(vehicle_local_position_setpoint),
													vehicle_local_pos_sp_pub,
													local_pos_sp);
			}
		}
		// send offboard control heartbeat_time
		if(control_mode->flag_control_offboard_enabled){
			offboard_control->timestamp = (uint64_t)this_time;
			// Are the polarity of these signals correct?
			offboard_control->ignore_thrust = true;
			offboard_control->ignore_attitude = true;
			offboard_control->ignore_bodyrate = true;
			offboard_control->ignore_position = true;
			offboard_control->ignore_velocity = true;
			offboard_control->ignore_acceleration_force = true;

			orb_publish(ORB_ID(offboard_control_mode), offboard_control_mode_pub,
									offboard_control);
		}
		else {
			offboard_control->timestamp = (uint64_t)this_time;
			// Are the polarity of these signals correct?
			offboard_control->ignore_thrust = false;
			offboard_control->ignore_attitude = false;
			offboard_control->ignore_bodyrate = false;
			offboard_control->ignore_position = false;
			offboard_control->ignore_velocity = false;
			offboard_control->ignore_acceleration_force = false;

			orb_publish(ORB_ID(offboard_control_mode), offboard_control_mode_pub,
									offboard_control);
		}
		// sleep
		sleep_timer->sleep();
	}

	this->thread_running = false;
	delete_publications();
	delete_subscriptions();
	return 0;
}

void initialize_publications()
{
	offboard_control = (offboard_control_mode_s *)malloc(sizeof(offboard_control_mode_s));
	offboard_control_mode_pub = orb_advertise(ORB_ID(offboard_control_mode), offboard_control);
	memset(offboard_control, 0, sizeof(offboard_control_mode_s));

	local_pos_sp = (vehicle_local_position_setpoint_s *) malloc(sizeof(vehicle_local_position_setpoint_s));
	vehicle_local_pos_sp_pub = orb_advertise(ORB_ID(vehicle_local_position_setpoint), local_pos_sp);
	memset(local_pos_sp, 0, sizeof(vehicle_local_position_setpoint_s));

	att_sp = (vehicle_attitude_setpoint_s *) malloc(sizeof(vehicle_attitude_setpoint_s));
	vehicle_attitude_sp_pub = orb_advertise(ORB_ID(vehicle_attitude_setpoint), vehicle_att);
	memset(vehicle_att, 0, sizeof(vehicle_attitude_setpoint_s));
}

void initialize_subscriptions()
{

	control_mode = (vehicle_control_mode_s *) malloc(sizeof(vehicle_control_mode_s));
	control_mode_sub = orb_subscribe(ORB_ID(vehicle_control_mode));
	memset(control_mode, 0, sizeof(vehicle_control_mode_s));

	eag_scrubbed = (eag_scrubbed_s *) malloc(sizeof(eag_scrubbed_s));
	eag_scrubbed_sub = orb_subscribe(ORB_ID(eag_scrubbed));
	memset(eag_scrubbed, 0, sizeof(eag_scrubbed_s));

	vehicle_att = (vehicle_attitude_s *) malloc(sizeof(vehicle_attitude_s));
	vehicle_att_sub = orb_subscribe(ORB_ID(vehicle_attitude));
	memset(vehicle_att, 0, sizeof(vehicle_attitude_s));

	vehicle_local_pos = (vehicle_local_position_s *) malloc(sizeof(vehicle_local_position_s));
	vehicle_local_pos_sub = orb_subscribe(ORB_ID(vehicle_local_position));
	memset(vehicle_local_pos, 0, sizeof(vehicle_local_position_s));
}

void copy_subscriptions()
{
	bool updated;

	// check control mode
	orb_check(control_mode_sub, &updated);
	if(updated){
		orb_copy(ORB_ID(vehicle_control_mode), control_mode_sub, control_mode);
	}
	// Check vehicle attitude
	orb_check(vehicle_att_sub, &updated);
	if(updated){
		orb_copy(ORB_ID(vehicle_attitude), vehicle_att_sub, vehicle_att);
	}
	// Check vehicle local position
	orb_check(vehicle_local_pos_sub, &updated);
	if(updated){
		orb_copy(ORB_ID(vehicle_local_position), vehicle_local_pos_sub, vehicle_local_pos);
	}
	// Check eag scrubbed
	orb_check(eag_scrubbed_sub, &updated);
	if(updated){
		orb_copy(ORB_ID(eag_scrubbed), eag_scrubbed_sub, eag_scrubbed);
	}
	// Check manual control setpoint
	orb_check(manual_sp_sub, &updated);
	if(updated){
		orb_copy(ORB_ID(manual_control_setpoint), manual_sp_sub, manual_sp);
	}
}

void delete_publications()
{
	free(offboard_control_mode_pub);
	free(offboard_control);

	free(vehicle_local_pos_sp_pub);
	free(local_pos_sp);

	free(vehicle_attitude_sp_pub);
	free(att_sp);
}

void delete_subscriptions()
{
	free(control_mode);

	free(eag_scrubbed);

	free(vehicle_att);

	free(vehicle_local_pos);
}

/* Used for debugging the control mode of the vehicle in the nuttx shell */
/*
void decode_control_mode(){
	if(control_mode->flag_control_manual_enabled){
		PX4_INFO("Manual control enabled.");
	}
	if(control_mode->flag_control_offboard_enabled){
		PX4_INFO("Offboard control enabled");
	}
	if(control_mode->flag_control_attitude_enabled){
		PX4_INFO("Attitude control enabled");
	}
}
*/
