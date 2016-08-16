/**
 * @file ir_range_sensor.cpp
 *
 * Infrared range sensor implementation, designed using the SHARP GP2Y0A02YK0F
 * proximity sensor. This code can be adapted for other analog output IR sensors
 * that have a similar input-output behavior.
 *
 * The GP2Y0A02YK0F is suitable for low-noise precise distance measurement in the range of 15-150cm.
 *
 * @author Joseph Sullivan
 */

#include <uORB/topics/distance_sensor.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/ir_calibration.h>
#include <uORB/uORB.h>

#include <commander/commander_helper.h>

#include <simple_task/SimpleTask.h>

#include <drivers/drv_hrt.h>
#include <drivers/drv_adc.h>

#include <DevMgr.hpp>

#include <px4_middleware.h>
#include <px4_defines.h>
#include <px4_posix.h>
#include <px4_log.h>
#include <px4_adc.h>

#include <mathlib/mathlib.h>

#include <cstring>
#include <cstdlib>
#include <errno.h>
#include <unistd.h>

using namespace DriverFramework;

#define IR_SENSOR_CHANNEL 14                                  // 3.3V ADC channel

#ifndef MAV_DISTANCE_SENSOR_INFRARED
#define MAV_DISTANCE_SENSOR_INFRARED 2
#endif

#ifndef MAV_SENSOR_ORIENTATION_YAW_180
#define MAV_SENSOR_ORIENTATION_YAW_180 4
#endif

// Schedule defines
#define IR_RANGE_SENSOR_STACK_SIZE  4096
#define IR_RANGE_SENSOR_SCHED       SCHED_DEFAULT
#define IR_RANGE_SENSOR_PRIORITY    SCHED_PRIORITY_DEFAULT

/*
  Estimator related constants and defines
*/

#define SAMPLE_SIZE 6

// Number of coefficients in output voltage estimator polynomial
#define COEFFICIENT_COUNT 5

// File on SD card used for storing calibration parameters
#define CALIBRATION_FILE "/fs/microsd/ir_range_params"

// Vertical distance of sensor PCB to FMU in NED frame
#define VERTICAL_OFFSET_CM  8

#define SENSOR_COVARIANCE   0.0225f

/*
  Estimator related function declarations
*/

// Computes variance of a sample of size N
float variance(int N, float *buff);

// Computes average of a sample of size N
float average(int N, float *buff);

/*
* Class definition for the IR sensor. Inherits from the SimpleTask threading implementation.
*/
class InfraredRangeSensor : public SimpleTask
{
public:
  InfraredRangeSensor() : SimpleTask("InfraredRangeSensor",
                                    IR_RANGE_SENSOR_PRIORITY,
                                    IR_RANGE_SENSOR_STACK_SIZE,
                                    IR_RANGE_SENSOR_SCHED),
  _h_adc()
  {
    memset(y_samples, 0.f, sizeof(y_samples));
    memset(sample_var, 0.f, sizeof(sample_var));
    memset(coeffs, 0.f, sizeof(coeffs));
  }
  ~InfraredRangeSensor();

  // Working thread of this task
  int job();

private:

  // Calibration routine
  int calibrate(int ir_cal_sub, orb_advert_t ir_cal_pub, struct ir_calibration_s* msg);

  void dump_coeffs(orb_advert_t ir_cal_pub, struct ir_calibration_s* msg);

  // Reads calibration coefficients from file
  int read_calibration_params(int fd);

  // Writes calibration coefficients from file
  int write_calibration_params(int fd);

  // Polls device handle for ADC measurements
  int adc_poll(float *voltage);

  // Maps distance in cm to volts via fourth order polynomial
  float f(float distance);

  // Maps voltage to distance in cm using newton's method
  float f_inverse(float voltage);

  // Maps distance in cm to volts/cm via third order polynomial
  float df_dx(float distance);

  // Statistical regression optimizes error of InfraredRangeSensor::f()
  // based on x_samples and y_samples
  int fourth_order_linear_regression();

  float variance_lookup(float distance);

  int adc_init();

  DevHandle _h_adc;

  // Values at which distance is sampled for calibration
  float x_samples[SAMPLE_SIZE] = {2.e1f,4.e1f,6.e1f,8.e1f,10.e1f,12.e1f};

  // Voltage output of the ir sensor for each distance
  float y_samples[SAMPLE_SIZE];

  float sample_var[SAMPLE_SIZE];

  // Coefficients of fourth order polynomial, as in
  // c[0]x^4+c[1]x^3+...+c[4]
  float coeffs[COEFFICIENT_COUNT];
};

/*
* adc_init() taken from modules/sensors/sensors.cpp
*/
int
InfraredRangeSensor::adc_init()
{

	DevMgr::getHandle(ADC0_DEVICE_PATH, _h_adc);

	if (!_h_adc.isValid()) {
		warnx("FATAL: no ADC found: %s (%d)", ADC0_DEVICE_PATH, _h_adc.getError());
		return PX4_ERROR;
	}

	return PX4_OK;
}

/*
* adc_poll() also partially taken from modules/sensors/sensors.cpp
*/
int
InfraredRangeSensor::adc_poll(float *voltage)
{
  // Allocate memory for 12 channels (ensures that the IR channel is read)
  struct adc_msg_s buf_adc[12];
  // Read all channels
  int ret = _h_adc.read(&buf_adc, sizeof(buf_adc));

  if (ret >= (int)sizeof(buf_adc[0]))
  {
    // Check to see if the IR channel was read
    for (unsigned i = 0; i < ret / sizeof(buf_adc[0]); i++)
    {
      if(i < 10)
      {
        if(buf_adc[i].am_channel == IR_SENSOR_CHANNEL)
        {
          *voltage = buf_adc[i].am_data / (4096.0f / 3.3f);
          return PX4_OK;
        }
      }
    }
  }
  return PX4_ERROR;
}

// Working thread
int InfraredRangeSensor::job()
{
  this->thread_running = true;

  // Local variables used in the working thread_running
  int cal_fd = 0;                                             // file descriptor for calibration coefficient file
  bool calibrated = false;                                    // flag indicates sensor is calibrated
  bool updated = false;                                       // flag indicates that ADC poll contained the infrared sensor channel

  // Initialize ir_calibration topic sub / pub
  struct ir_calibration_s ir_cal;
  memset(&ir_cal, 0, sizeof(ir_cal));
  int ir_cal_sub = orb_subscribe(ORB_ID(ir_calibration));
  orb_advert_t ir_cal_pub = orb_advertise(ORB_ID(ir_calibration), &ir_cal);

  // Initialize distance_sensor topic pub
  struct distance_sensor_s sensor;
  orb_advert_t distance_sensor_pub = orb_advertise(ORB_ID(distance_sensor), &sensor);
  sensor.min_distance = 0.2;
  sensor.max_distance = 1.5;
  sensor.current_distance = 0.;
  sensor.covariance = SENSOR_COVARIANCE;                      // unsure of what this should be
  sensor.type = MAV_DISTANCE_SENSOR_INFRARED;
  sensor.id = 42;                                             // unsure of what this should be either, using magic number
  sensor.orientation = MAV_SENSOR_ORIENTATION_YAW_180;

  // Initialize ADC
  if(adc_init() == PX4_ERROR) {
    PX4_ERR("[ir_range_sensor] failed to initialize adc.");
    goto task_exit;
  }

  // Look for calibration file
  cal_fd = open(CALIBRATION_FILE, O_RDONLY);

  if(cal_fd > 0)
  {
    // If the calibration file exists and is read successfully, then the sensor can
    // publish data.
    if(read_calibration_params(cal_fd) == PX4_OK)
    {
      calibrated = true;
      dump_coeffs(ir_cal_pub, &ir_cal);
    }
    else
    {
      PX4_ERR("[ir_range_sensor] read_calibration_params() failed.");
      goto task_exit;
    }
  }

  // Worker loop
  while(!this->should_exit())
  {
    // Check for calibration messages
    orb_check(ir_cal_sub, &updated);
    if(updated)
    {
      orb_copy(ORB_ID(ir_calibration), ir_cal_sub, &ir_cal);
      if(ir_cal.data_code == ir_cal.IR_CAL_CODE_START)
      {
        calibrated = false;
        calibrated = this->calibrate(ir_cal_sub, ir_cal_pub, &ir_cal) == PX4_OK;
        if(calibrated)
        {
          PX4_INFO("Calibration Successful.");
          if(write_calibration_params(cal_fd) != PX4_OK)
          {
              PX4_ERR("[ir_range_sensor] write_calibration_params() failed.");
              goto task_exit;
          }
        }
      }
    }

    // If the sensor is calibrated, poll the ADC and compute the distance
    if(calibrated)
    {
      float voltage = 0.f;
      if(adc_poll(&voltage) == PX4_OK)
      {
        static float last_distance_m = 0.f;
        float distance_cm = f_inverse(voltage);

        if(distance_cm > 0.0f && distance_cm <= x_samples[SAMPLE_SIZE-1] && distance_cm >= x_samples[0])
        {
          // Publish valid readings
          hrt_abstime t = hrt_absolute_time();
          last_distance_m = (distance_cm + VERTICAL_OFFSET_CM) / 100.f;
          sensor.timestamp = t;
          sensor.current_distance = last_distance_m;
          sensor.covariance = SENSOR_COVARIANCE;
          orb_publish(ORB_ID(distance_sensor), distance_sensor_pub, &sensor);
        }
        /*else
        {
          // For invalid readings, publish last measurement with zero covariance
          hrt_abstime t = hrt_absolute_time();
          sensor.timestamp = t;
          sensor.current_distance = last_distance_m;
          sensor.covariance = 0.f
        }*/
      }
    }
    usleep(50000);
  }
task_exit:
  this->thread_running = false;
  PX4_INFO("Task exiting!");
  return 0;
}

#define SAMPLE_BUFFER_SIZE  100
#define VAR_TOLERANCE       8e-2f

/*
* The calibrate() function samples the IR sensor channel, assuming that the FMU
* is at certain preset distances specified in x_sampes. Once the variance of a set of
* measurements corresponding to a given distance is satisfactory, the average of that set
* is computed, recorded in y_sample, and published to the ir_calibration topic. A fourth
* order polynomial is fitted to the data to use as an estimator of the input-output relationship
* of the sensor.
*/
int
InfraredRangeSensor::calibrate(int ir_cal_sub, orb_advert_t ir_cal_pub, struct ir_calibration_s* ir_cal)
{
  // Initialize distance parameter to 20cm
  unsigned distance_index = 0;

  // Initialize ADC output array (ring buffer)
  unsigned sample_count = 0;
  unsigned sample_index = 0;
  float buff[SAMPLE_BUFFER_SIZE];
  memset(buff,0.f, SAMPLE_BUFFER_SIZE*sizeof(float));

  // Declare sample varaince and average
  float var = 0.f;
  float avg = 0.f;

  // While there are still distances to sample
  PX4_INFO("[ir_range_sensor] calibrating...");
  tune_neutral(true);

  PX4_INFO("[ir_range_sensor] Place the drone 20cm above the ground");
  usleep(5000000);
  while(distance_index < SAMPLE_SIZE)
  {
    // Sample ADC and push value into ring buffer
    if (adc_poll(&buff[sample_index % SAMPLE_BUFFER_SIZE]) == PX4_OK)
    {
      sample_count++;
      sample_index++;
    }
    // If the buffer is full,
    if(sample_count >= SAMPLE_BUFFER_SIZE)
    {
      // Compute the sample variance and average
      var = variance(SAMPLE_BUFFER_SIZE, buff);
      avg = average(SAMPLE_BUFFER_SIZE, buff);

      if(var < VAR_TOLERANCE && var > -1.0f*VAR_TOLERANCE)
      {
        hrt_abstime t = hrt_absolute_time();
        // save the average to y_samples
        y_samples[distance_index] = avg;
        sample_var[distance_index] = var;
        distance_index++;
        sample_count = 0;
        sample_index = 0;

        // Send the IR calibration message containing the average measured
        ir_cal->timestamp = t;
        ir_cal->data_code = ir_cal->IR_CAL_CODE_SAMPLE;
        ir_cal->data = avg;
        orb_publish(ORB_ID(ir_calibration), ir_cal_pub, ir_cal);

        int print_y = (double)(avg * 4096.0f / 3.3f);
        PX4_INFO("Sample taken at %d", print_y);

        // Sleep 5 seconds before doing the next calibration step to
        // give the user time to move the drone.
        usleep(5000000);
      }
    }
    usleep(50000);
  }

  // Do the regression
  this->fourth_order_linear_regression();

  // Publish the coefficients
  dump_coeffs(ir_cal_pub, ir_cal);

  // Notify that calibration process is complete
  hrt_abstime t = hrt_absolute_time();
  ir_cal->timestamp = t;
  ir_cal->data_code = ir_cal->IR_CAL_CODE_FINISH;
  ir_cal->data = 0.f;
  orb_publish(ORB_ID(ir_calibration), ir_cal_pub, ir_cal);
  return PX4_OK;
}

/*
  Estimator related function definitions
*/

// Maps distance in cm to volts by a fourth order polynomial
float
InfraredRangeSensor::f(float distance)
{
  float y = 0.0f;
  float x_power = 1.0f;

  int i;
  for(i = COEFFICIENT_COUNT -1; i >= 0; i--)
  {
    y += coeffs[i] * x_power;
    x_power*=distance;
  }

  return y;
}

// Maps distance in cm to volts / cm by a third order polynomial
float
InfraredRangeSensor::df_dx(float distance)
{
  float dydx = 0.0f;
  float x_power = 1.0f;

  int i;
  for(i = COEFFICIENT_COUNT-2; i >= 0; i--)
  {
    dydx += (COEFFICIENT_COUNT-i-1) * coeffs[i] * x_power;
    x_power*=distance;
  }
  return dydx;
}

// Maps volts to distance in cm, uses Newton's method to minimize the error
float
InfraredRangeSensor::f_inverse(float volts)
{
  float x = 0.f;

  // Select an interval that contains the voltage
  float yi, yj, xi, xj;

  // Edge case 1
  if(volts <= y_samples[0])
  {
    yi = y_samples[0];
    yj = y_samples[1];
    xi = x_samples[0];
    xj = x_samples[1];
  }
  // Edge case 2
  else if(volts >= y_samples[SAMPLE_SIZE-1])
  {
    yi = y_samples[SAMPLE_SIZE-2];
    yj = y_samples[SAMPLE_SIZE-1];
    xi = x_samples[SAMPLE_SIZE-2];
    xj = x_samples[SAMPLE_SIZE-1];
  }
  // General case
  else
  {
      int i;
      for(i=0; i < SAMPLE_SIZE - 1; i++)
      {
        if(y_samples[i] < volts) break;
      }
      yi = y_samples[i];
      yj = y_samples[i+1];
      xi = x_samples[i];
      xj = x_samples[i+1];
  }

  // Calculate initial guess from linear interpolation
  x = (volts - yi)*(xj - xi)/(yj - yi) + xi;

  // Newton's method iteration
  float tol = 0.001f;
  float dy = 1E6;
  unsigned iterations = 0;
  while((dy < -1.0f*tol || dy > tol) && iterations < 100)
  {
    float y = f(x);
    float dydx = df_dx(x);
    dy = y - volts;
    x -= dy/dydx;
    iterations++;
  }
  if(iterations == 100)
  {
    // PX4_ERR("[ir_range_sensor] estimator did not converge");
    return -1.0f;
  }
  else return x;
}

/*
* Implements the ordinary least squares regression as Ac = b, solves for c
* using the Matrix class provided in mathlib.
*/
int
InfraredRangeSensor::fourth_order_linear_regression()
{
  const float* x = x_samples;
  float* y = y_samples;
  unsigned N = SAMPLE_SIZE;

  math::Matrix<5,5> A;
  math::Matrix<5,5> Ainv;
  math::Vector<5> b;
  math::Vector<5> c;

  // Initialize sum_x_power memory
  int i,j;
  float sum_x_power[2*COEFFICIENT_COUNT-1];
  for(i = 0 ; i < 2*COEFFICIENT_COUNT-1; i++)
  {
    sum_x_power[i] = 0.0f;
  }

  // Compute powers of X and b vector
  for(i = 0; i < N; i++)
  {
    float x_power = 1.e0f;
    for(j = 0; j < 2*COEFFICIENT_COUNT-1; j++)
    {
      sum_x_power[j] += x_power;
      if(j < COEFFICIENT_COUNT)
      {
        b.data[COEFFICIENT_COUNT-1-j] += y[i]*x_power;
      }
      x_power*=x[i];
    }
  }

  // Compute A coefficients
  for(i = 0; i < COEFFICIENT_COUNT; i++)
  {
    for(j = 0; j < COEFFICIENT_COUNT; j++)
    {
      A.data[i][j] = sum_x_power[2*(COEFFICIENT_COUNT-1)-j-i];
    }
  }

  // Compute solution
  Ainv = A.inversed();
  c = Ainv*b;
  // Copy the matrix data
  for(i = 0; i < COEFFICIENT_COUNT; i++)
  {
    coeffs[i] = c.data[i];
  }

  // Test to see if c is a zero vector (this would happen if A were singular)
  float sum = 0.f;
  for(i = 0; i < 5; i++)
  {
    sum+=coeffs[i];
  }
  if(sum < 1e-8f && sum > -1e-8f)
  {
    return -1;
  }
  else
  {
    return 0;
  }
}

float
InfraredRangeSensor::variance_lookup(float distance)
{
  // Edge case 1
  if(distance <= x_samples[0])
  {
    return sample_var[0];
  }
  // Edge case 2
  if(distance >= x_samples[SAMPLE_SIZE-1])
  {
    return sample_var[SAMPLE_SIZE-1];
  }
  // General case
  unsigned i;
  for(i = 0; i < SAMPLE_SIZE; i++)
  {
    if(x_samples[i] > distance)
    {
      break;
    }
  }

  // Choose the varaince corresponding to the closer sample
  if((distance - x_samples[i-1]) < (x_samples[i] - distance))
  {
    return sample_var[i-1];
  }
  else return sample_var[i];
}


/*
  Calibration related function definitions
*/

// Publishes all the coefficients to the ir_calibration topic, with a 1.5second
// delay to accomidate the update rate of the mavlink stream. The delay could be adjusted,
// depending on the stream rate.
void
InfraredRangeSensor::dump_coeffs(orb_advert_t ir_cal_pub, struct ir_calibration_s* ir_cal)
{
  for(unsigned i = 0; i < COEFFICIENT_COUNT; i++)
  {
    hrt_abstime t = hrt_absolute_time();
    ir_cal->data_code = ir_cal->IR_CAL_CODE_EST_COEFF;
    ir_cal->data = coeffs[i];
    ir_cal->timestamp  = t;
    orb_publish(ORB_ID(ir_calibration), ir_cal_pub, ir_cal);
    usleep(1500000);
  }
}

// Reads calibration parameters from a file on the SD card.
int
InfraredRangeSensor::read_calibration_params(int cal_fd)
{
  float buff[COEFFICIENT_COUNT + 2*SAMPLE_SIZE];
  size_t ret = read(cal_fd, buff, sizeof(buff));
  close(cal_fd);
  if(ret != sizeof(buff))
  {
    PX4_ERR("[ir_range_sensor] during read of coefficient file %d bytes were read, expected %d bytes.",
            ret, sizeof(buff));
    return PX4_ERROR;
  }
  else
  {
    // copy the data
    unsigned i;

    for(i = 0; i < COEFFICIENT_COUNT; i++)
    {
      coeffs[i] = buff[i];
      double print_val = (4096.0f / 3.3f) * coeffs[i];
      PX4_INFO("read coefficient %d", print_val);
    }

    for(; i < COEFFICIENT_COUNT + SAMPLE_SIZE; i++)
    {
      y_samples[i - COEFFICIENT_COUNT] = buff[i];
    }

    for(; i < COEFFICIENT_COUNT + 2*SAMPLE_SIZE; i++)
    {
      sample_var[i - COEFFICIENT_COUNT - SAMPLE_SIZE] = buff[i];
    }

    return PX4_OK;
  }
}

// Writes the calibration parameters in RAM to a file on the SD card.
int
InfraredRangeSensor::write_calibration_params(int cal_fd)
{
  cal_fd = open(CALIBRATION_FILE, O_WRONLY | O_CREAT);

  if(cal_fd <= 0) { return PX4_ERROR; }

  else
  {
    float buff[COEFFICIENT_COUNT + 2*SAMPLE_SIZE];

    unsigned i;
    for(i = 0; i < COEFFICIENT_COUNT; i++)
    {
      buff[i] = coeffs[i];
      double print_val = (4096.0f / 3.3f) * buff[i];
      PX4_INFO("writing coefficient %d", print_val);
    }

    for(; i < COEFFICIENT_COUNT + SAMPLE_SIZE; i++)
    {
      buff[i] = y_samples[i - COEFFICIENT_COUNT];
    }

    for(; i < COEFFICIENT_COUNT + 2*SAMPLE_SIZE; i++)
    {
      buff[i] = sample_var[i - COEFFICIENT_COUNT - SAMPLE_SIZE];
    }

    ssize_t ret = write(cal_fd, buff, sizeof(buff));
    close(cal_fd);

    if(ret != sizeof(buff))
    {
      PX4_ERR("[ir_range_sensor] during write of coefficient file %d bytes were written, expected %d bytes",
              ret, sizeof(buff));
      return PX4_ERROR;
    }
    else { return PX4_OK; }
  }

}

// Computes variance of floating point data using recurrance formula
float variance(int N, float *buff)
{
  float m = 0.f;
  float avg = 0.f;
  unsigned i;
  for (i = 1; i <= N; i++)
  {
    float delta = buff[i-1] - avg;
    avg += delta / (1.f*i);
    m += delta * (buff[i-1] - avg);
  }

  return m / (1.f*(N-1));
}

// Computes the sample average
float average(int N, float *buff)
{
  float avg = 0.f;
  for (unsigned i = 0; i < N; i++)
  {
    avg += buff[i];
  }

  return (avg / (float) N);
}

/*
--------------------------------------------------------------------------------
Nuttx related code for starting / stopping the task
--------------------------------------------------------------------------------
*/

/**
 * Print the correct usage.
 */
static void usage(const char *reason);

static bool command_is_good(char* command);

static void usage(const char *reason)
{
	if (reason) {
		warnx("%s\n", reason);
	}

	warnx("usage: ir_range_sensor {start|stop|status} [-p <additional params>]\n\n");
}

static bool command_is_good(char* command)
{
  if(!strcmp(command,"start")) return true;
  if(!strcmp(command,"stop")) return true;
  if(!strcmp(command,"status")) return true;
  return false;
}

InfraredRangeSensor *ir_sensor = NULL;

extern "C" __EXPORT int ir_range_sensor_main(int argc, char* argv[]);
int ir_range_sensor_main(int argc, char* argv[])
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
  if(NULL == ir_sensor)
  {
    ir_sensor = new InfraredRangeSensor();
  }

  // Handle user command
  if(!strcmp(argv[1], "start"))
  {
    if(ir_sensor->is_running())
    {
      warnx("Filter is already running");
    }
    else ir_sensor->start();
  }

  else if(!strcmp(argv[1], "stop"))
  {
    if(ir_sensor->is_running())
    {
      ir_sensor->stop();
    }
    else
    {
      warnx("Sensor is not running");
    }
  }

  else if(!strcmp(argv[1], "status"))
  {
    if(ir_sensor->is_running()) {
      warnx("Infrared Range Sensor is running");
    }
    else {
      warnx("Infrared Range Sensor is not running");
    }
  }

  return 0;

}
