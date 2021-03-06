#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <sys/time.h>
#include <unistd.h>


#include <libserialport.h>

#include <libsbp/sbp.h>
#include <libsbp/system.h>
#include <libsbp/navigation.h>

char *serial_port_name = NULL;
struct sp_port *piksi_port = NULL;

static sbp_msg_callbacks_node_t gps_time_node;
static sbp_msg_callbacks_node_t pos_llh_node;
// static sbp_msg_callbacks_node_t pos_llh_cov_node; //discontinued
static sbp_msg_callbacks_node_t vel_ned_node;
static sbp_msg_callbacks_node_t baseline_node;
static sbp_msg_callbacks_node_t heartbeat_node;

static sbp_msg_callbacks_node_t heartbeat_node_0;

struct piksi_msg {
  // GPS solution data
  double lat, lon, h; //latitude [deg], longitude [deg], altitude [m]
  float cov_n_n, cov_n_e, cov_n_d, cov_e_e, cov_e_d, cov_d_d; // error covariances
  float h_accuracy;    // horizontal position estimated standard deviation [mm]
  float v_accuracy;    // vertical position estimated standard deviation [mm]

  // RTK data
  s32 n, e, d; // baseline North, East, Down coordinates [mm]
  s32 v_n, v_e, v_d; // baseline North, East, Down velocities [mm/s]

  // GPS time
  u8 hr, min, sec; //
  u32 ns;

  // GPS solution data
  u8 sats; // number of satellites in the vicinity

  // RTK status flag
  // 0 Invalid
  // 1 Single Point Position (SPP)
  // 2 Differential GNSS (DGNSS)
  // 3 Float RTK
  // 4 Fixed RTK
  // 5 Dead Reckoning
  u8 flag;
};

struct piksi_msg piksi;

struct timeval stop, start;
bool flag_start = 0;


void usage(char *prog_name) {
  /* Help string for -h argument */
  fprintf(stderr, "usage: %s [-p path/to/serial/port][-b baud_rate]\n"
          "default values:\n"
          "\t-p = /dev/ttyUSB0\n"
          "\t-b = 115200\n", prog_name);
}


void heartbeat_callback_0(u16 sender_id, u8 len, u8 msg[], void *context)
{
  (void)sender_id, (void)len, (void)msg, (void)context;
   fprintf(stdout, "First heartbeat detected.\n\n");
  flag_start = 1;
}


void heartbeat_callback(u16 sender_id, u8 len, u8 msg[], void *context)
{
  (void)sender_id, (void)len, (void)msg, (void)context;
   fprintf(stdout, "%s\n", __FUNCTION__);
}


void baseline_callback(u16 sender_id, u8 len, u8 msg[], void *context)
{
  (void)sender_id, (void)len, (void)msg, (void)context;
  // fprintf(stdout, "%s\n", __FUNCTION__);

  msg_baseline_ned_t baseline = *(msg_baseline_ned_t *)msg;
  piksi.n = baseline.n;
  piksi.e = baseline.e;
  piksi.d = baseline.d;
  piksi.flag = baseline.flags;
  piksi.sats = baseline.n_sats;

  fprintf(stdout, "%02i%02i%02i.%02.0f, %f, %f, %f, %f, %f, %f, %f, %f, %f,%i\n",
    piksi.hr, piksi.min, piksi.sec, piksi.ns / 1e7,
    piksi.lat, piksi.lon, piksi.h,
    piksi.v_n / 1e3, piksi.v_e / 1e3, piksi.v_d / 1e3,
    piksi.n / 1e3, piksi.e / 1e3, piksi.d / 1e3,
    piksi.flag);
}


void pos_llh_callback(u16 sender_id, u8 len, u8 msg[], void *context)
{
  (void)sender_id, (void)len, (void)msg, (void)context;
  // fprintf(stdout, "%s\n", __FUNCTION__);

  msg_pos_llh_t pos_llh = *(msg_pos_llh_t *)msg;
  piksi.lat = pos_llh.lat;
  piksi.lon = pos_llh.lon;
  piksi.h = pos_llh.height;

  piksi.h_accuracy = pos_llh.h_accuracy;
  piksi.v_accuracy = pos_llh.v_accuracy;
}


void pos_llh_cov_callback(u16 sender_id, u8 len, u8 msg[], void *context)
{
  (void)sender_id, (void)len, (void)msg, (void)context;
  // fprintf(stdout, "%s\n", __FUNCTION__);

  msg_pos_llh_cov_t pos_llh_cov = *(msg_pos_llh_cov_t *)msg;
  piksi.lat = pos_llh_cov.lat;
  piksi.lon = pos_llh_cov.lon;
  piksi.h = pos_llh_cov.height;
  piksi.cov_n_n = pos_llh_cov.cov_n_n;
  piksi.cov_n_e = pos_llh_cov.cov_n_e;
  piksi.cov_n_d = pos_llh_cov.cov_n_d;
  piksi.cov_e_e = pos_llh_cov.cov_e_e;
  piksi.cov_e_d = pos_llh_cov.cov_e_d;
  piksi.cov_d_d = pos_llh_cov.cov_d_d;
}


void vel_ned_callback(u16 sender_id, u8 len, u8 msg[], void *context)
{
  (void)sender_id, (void)len, (void)msg, (void)context;
  // fprintf(stdout, "%s\n", __FUNCTION__);

  msg_vel_ned_t vel_ned = *(msg_vel_ned_t *)msg;
  piksi.v_n = vel_ned.n;
  piksi.v_e = vel_ned.e;
  piksi.v_d = vel_ned.d;
}


void gps_time_callback(u16 sender_id, u8 len, u8 msg[], void *context)
{
  (void)sender_id, (void)len, (void)msg, (void)context;
  // fprintf(stdout, "%s\n", __FUNCTION__);

  msg_utc_time_t gps_time = *(msg_utc_time_t *)msg;
  piksi.hr = gps_time.hours;
  piksi.min = gps_time.minutes;
  piksi.sec = gps_time.seconds;
  piksi.ns = gps_time.ns;
}


void setup_port(int baud)
{
  /* set the serial port options for the Piksi */
  printf("Attempting to configure the serial port...\n");

  int result;

  // set baud rate
  result = sp_set_baudrate(piksi_port, baud);
  if (result != SP_OK) {
    fprintf(stderr, "Cannot set port baud rate!\n");
    exit(EXIT_FAILURE);
  }
  printf("Configured the baud rate...\n");

  // set flow control
  result = sp_set_flowcontrol(piksi_port, SP_FLOWCONTROL_NONE);
  if (result != SP_OK) {
    fprintf(stderr, "Cannot set flow control!\n");
    exit(EXIT_FAILURE);
  }
  printf("Configured the flow control...\n");

  // set bit size
  result = sp_set_bits(piksi_port, 8);
  if (result != SP_OK) {
    fprintf(stderr, "Cannot set data bits!\n");
    exit(EXIT_FAILURE);
  }
  printf("Configured the number of data bits...\n");

  // set parity
  result = sp_set_parity(piksi_port, SP_PARITY_NONE);
  if (result != SP_OK) {
    fprintf(stderr, "Cannot set parity!\n");
    exit(EXIT_FAILURE);
  }
  printf("Configured the parity...\n");

  // set stop bits
  result = sp_set_stopbits(piksi_port, 1);
  if (result != SP_OK) {
    fprintf(stderr, "Cannot set stop bits!\n");
    exit(EXIT_FAILURE);
  }
  printf("Configured the number of stop bits... done.\n");
}


u32 piksi_port_read(u8 *buff, u32 n, void *context)
{
  (void)context;
  u32 result;

  result = sp_blocking_read(piksi_port, buff, n, 0);

  return result;
}


int main(int argc, char **argv)
{
  int opt;
  int result = 0;

  sbp_state_t s;
  sbp_state_t s0;

  // parse the args
  serial_port_name = "/dev/ttyUSB0";
  unsigned int baud = 115200;
  while ((opt = getopt(argc, argv, "p:b:h")) != -1) {
    switch (opt) {
      case 'p':
        serial_port_name = (char *)calloc(strlen(optarg) + 1, sizeof(char));
        if (!serial_port_name) {
          fprintf(stderr, "Cannot allocate memory!\n");
          exit(EXIT_FAILURE);
        }
        strcpy(serial_port_name, optarg);
        break;
      case 'b':
        baud = atoi(optarg);
        break;
      case 'h':
        usage(argv[0]);
        exit(EXIT_FAILURE);
    }
  }

  printf("Attempting to open %s with baud rate %i ...\n", serial_port_name,
    baud);

  // check for serial port
  if (!serial_port_name) {
    fprintf(stderr, "Check the serial port path of the Piksi!\n");
    exit(EXIT_FAILURE);
  }

  // check if Piksi can be detected
  result = sp_get_port_by_name(serial_port_name, &piksi_port);
  if (result != SP_OK) {
    fprintf(stderr, "Cannot find provided serial port!\n");
    exit(EXIT_FAILURE);
  }

  // open Piksi for readings
  result = sp_open(piksi_port, SP_MODE_READ);
  if (result != SP_OK) {
    fprintf(stderr, "Cannot open %s for reading!\n", serial_port_name);
    exit(EXIT_FAILURE);
  }

  // set baud rate
  setup_port(baud);

  // create a secondary sbp_state such that the first reading of the sbp
  // messege sequence can be detected
  sbp_state_init(&s0);
  sbp_register_callback(&s0, SBP_MSG_HEARTBEAT, &heartbeat_callback_0, NULL,
                        &heartbeat_node_0); // 65535

  // create the primary sbp_state and register all the required callbacks
  sbp_state_init(&s);
  sbp_register_callback(&s, SBP_MSG_UTC_TIME, &gps_time_callback, NULL,
                        &gps_time_node); // 252
  sbp_register_callback(&s, SBP_MSG_POS_LLH, &pos_llh_callback, NULL,
                        &pos_llh_node); // 522
  // sbp_register_callback(&s, SBP_MSG_POS_LLH_COV, &pos_llh_cov_callback, NULL,
  //                       &pos_llh_cov_node); // 529
  sbp_register_callback(&s, SBP_MSG_VEL_NED, &vel_ned_callback, NULL,
                        &vel_ned_node); // 526
  sbp_register_callback(&s, SBP_MSG_BASELINE_NED, &baseline_callback, NULL,
                        &baseline_node); // 524
  sbp_register_callback(&s, SBP_MSG_HEARTBEAT, &heartbeat_callback, NULL,
                        &heartbeat_node); // 65535

  // wait for the first heartbeat so that the beginning of the sbp messege can
  // be identified
  fprintf(stdout, "\nWaiting for the first heartbeat...\n");
  while (!flag_start) {
    sbp_process(&s0, &piksi_port_read);
  }
  fprintf(stdout, "Starting the main loop...\n");

  // start the reading process
  int ret = 0;
  while(1) {

    // gettimeofday(&start, NULL);
    ret = sbp_process(&s, &piksi_port_read);

    if (ret < 0)
      printf("sbp_process error: %d\n", (int)ret);

    // gettimeofday(&stop, NULL);
    // printf("loop time: %f\n", (stop.tv_usec - start.tv_usec) / 1e3);
    // usleep(100e3);
  }

  result = sp_close(piksi_port);
  if (result != SP_OK) {
    fprintf(stderr, "Cannot close %s properly!\n", serial_port_name);
  }
  else {
    printf("Serial at %s port closed.", serial_port_name);
  }

  // clean exit the serial ports
  sp_free_port(piksi_port);
  free(serial_port_name);

  return 0;
}
