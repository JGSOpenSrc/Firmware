/****************************************************************************
 *
 *   Copyright (c) 2012-2015 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file px4_eag_publish.c
 *
 * @author Joseph Sullivan <jgs.424112@gmail.com.com>
 */

#include <px4_config.h>
#include <px4_tasks.h>
#include <px4_posix.h>
#include <px4_log.h>
#include <unistd.h>
#include <stdio.h>
#include <poll.h>
#include <string.h>

#include <uORB/uORB.h>
#include <uORB/topics/eag_raw.h>

__EXPORT int px4_eag_publish_main(int argc, char *argv[]);

int px4_eag_publish_main(int argc, char *argv[])
{

	// Opend device for reading
	int eag_fd = px4_open("/dev/ttyS6", O_RDONLY | O_NONBLOCK);

	if(eag_fd <= 0){
		PX4_ERR("[px4_eag_publish] error opening /dev/ttyS6 for reading...");
		return -1;
	}

	// Create poll fd struct
	px4_pollfd_struct_t pollfd = {.fd = eag_fd,
																.events = POLLIN};

	// Advertise uORB topic
	struct eag_raw_s eag;
	orb_advert_t eag_pub = orb_advertise(ORB_ID(eag_raw), &eag);

	if(eag_pub == NULL){
		PX4_ERR("[px4_eag_publish] error advertising topic, orb_advertise returned NULL");
		return -1;
	}

	int pollret, error_count = 0;
	while(error_count < 10){
		// Timeout in 5 seconds
		pollret = px4_poll(&pollfd, 1, 5000);

		if(pollret > 0){
			//  Got data, publish it
			px4_read(eag_fd, &(eag.raw_data), sizeof(eag.raw_data));
			eag.time_stamp = hrt_absolute_time();
			orb_publish(ORB_ID(eag_raw), &eag_pub, &eag);
		}

		else if (pollret == 0){
			// No data received
			PX4_WARN("[px4_eag_publish] no data received in last 5 seconds");
			error_count++;
		}

		else if (pollret < 0){
			// Poll returned error code
			PX4_WARN("[px4_eag_publish] px4_poll returned %d", pollret);
			error_count++;
		}
	}
	return 0;
}
