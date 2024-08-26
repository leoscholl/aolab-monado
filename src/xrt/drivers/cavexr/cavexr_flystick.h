/*!
 * @file
 * @brief  DTrack Flystick Controller interface
 * @author Jeremy Auzou <jeremy.auzou@insa-rouen.fr>
 * @author Swan Remacle <swanremacle@hotmail.fr>
 * @author Jean-Marc Cherfils <jean-marc.cherfils@insa-rouen.fr>
 * @ingroup drv_cavexr
 */

#ifndef CAVEXR_FLYSTICK_H
#define CAVEXR_FLYSTICK_H

#include "os/os_time.h"
#include "xrt/xrt_device.h"
#include "util/u_debug.h"
#include "DTrackSDK.hpp"
#include "cavexr.h"
#include "cavexr_interface.h"

#include <os/os_threading.h>

enum cavexr_buttons_index
{
    DTRACK_1 = 1,
    DTRACK_2 = 2,
    DTRACK_3 = 3,
    DTRACK_4 = 4,
    DTRACK_TRIGGER = 5,
    DTRACK_THUMBSTICK_CLICK = 6,
    DTRACK_THUMBSTICK = 7,

    DTRACK_GRIP_POSE = 8,
    DTRACK_AIM_POSE = 9,
};

struct dtrack_flystick_controller
{
	struct xrt_device base;

	struct os_mutex mutex;

	struct xrt_pose pose;

	/* The system this controller belongs to / receives reports from */
	struct cavexr *sys;

	// Array of inputs
	int buttons[DTRACK_FLYSTICK_BUTTONS];

	uint64_t created_ns;
	uint64_t frame_count;


	uint64_t device_id;

	CaveXrDTrack *dtrack;
};

struct dtrack_flystick_controller *
dtrack_flystick_controller_create(struct cavexr *sys, CaveXrDTrack *dtrack);

#endif
