/*!
 * @file
 * @brief Interface for CAVE driver.
 *
 *
 * @author Jeremy Auzou <jeremy.auzou@insa-rouen.fr>
 * @author Swan Remacle <swanremacle@hotmail.fr>
 * @author Jean-Marc Cherfils <jean-marc.cherfils@insa-rouen.fr>
 * @ingroup drv_cavexr
 */


#pragma once

#include "xrt/xrt_device.h"
#include "util/u_debug.h"
#include "cavexr_interface.h"
#include "dtrack.hpp"

#include <os/os_threading.h>

typedef struct {
    bool tracking;
    bool head_visible;
    bool flystick_visible;
} tracking_status;

struct cavexr
{
    struct xrt_device base;
    struct xrt_pose pose;

    struct os_mutex mutex;

    uint64_t created_ns;
    uint64_t frame_count;
    bool enable_3d;
    bool invert_eyes;
    float ipd;

    struct xrt_vec3 dimensions;
    enum u_logging_level log_level;

    /* The Dtrack controller the CAVE has. */
    CaveXrDTrack* dtrack;
    pthread_t dtrack_thread;

    struct dtrack_flystick_controller *controller;

    tracking_status status;
    double head_rot[9];
};
