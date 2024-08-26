// Copyright 2020-2021, Collabora, Ltd.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Device driver for CAVE environments.
 *
 *
 * @author Jeremy Auzou <jeremy.auzou@insa-rouen.fr>
 * @author Swan Remacle <swanremacle@hotmail.fr>
 * @author Jean-Marc Cherfils <jean-marc.cherfils@insa-rouen.fr>
 * @ingroup drv_cavexr
 */

#include "cavexr.h"
#include "cavexr_debug.h"

#include "xrt/xrt_device.h"

#include "os/os_time.h"
#include "os/os_threading.h"

#include "math/m_api.h"
#include "math/m_mathinclude.h"

#include "util/u_var.h"
#include "util/u_misc.h"
#include "util/u_time.h"
#include "util/u_debug.h"
#include "util/u_device.h"
#include "util/u_logging.h"
#include "util/u_distortion_mesh.h"
#include "DTrackSDK.hpp"
#define DTRACK_PORT 1234

#include "cavexr_flystick.h"
#include "util/u_visibility_mask.h"

#include <pthread.h>

#include <stdio.h>
#include <os/os_threading.h>


/*
 *
 * Structs and defines.
 *
 */

/*!
 * A sample HMD device.
 *
 * @implements xrt_device
 */

/// Casting helper function
static inline struct cavexr *
cavexr(struct xrt_device *xdev)
{
	return (struct cavexr *)xdev;
}

DEBUG_GET_ONCE_LOG_OPTION(sample_log, "SAMPLE_LOG", U_LOGGING_WARN)

#define SH_TRACE(p, ...) U_LOG_XDEV_IFL_T(&sh->base, sh->log_level, __VA_ARGS__)
#define SH_DEBUG(p, ...) U_LOG_XDEV_IFL_D(&sh->base, sh->log_level, __VA_ARGS__)
#define SH_ERROR(p, ...) U_LOG_XDEV_IFL_E(&sh->base, sh->log_level, __VA_ARGS__)
pthread_t debugThread;

static void
cavexr_destroy(struct xrt_device *xdev)
{
	struct cavexr *sh = cavexr(xdev);

	// Remove the variable tracking.
	u_var_remove_root(sh);

	// Destroy the thread
	os_mutex_destroy(&sh->mutex);

	// Fermer proprement fenêtre d'information
	cavexr_close_debug_window();
	pthread_join(debugThread, NULL);

    // Stopper le dtrack
    cavexr_dtrack_stop();
    pthread_join(sh->dtrack_thread, NULL);

	u_device_free(&sh->base);
}

static void
cavexr_update_inputs(struct xrt_device *xdev)
{
	
}

static void
cavexr_get_tracked_pose(struct xrt_device *xdev,
                            enum xrt_input_name name,
                            uint64_t at_timestamp_ns,
                            struct xrt_space_relation *out_relation)
{
	struct cavexr *sh = cavexr(xdev);
    auto dt = sh->dtrack;

	if (name != XRT_INPUT_GENERIC_HEAD_POSE) {
		SH_ERROR(cave, "unknown input name");
		return;
	}

    auto new_pose = sh->pose;

    tracking_status status = {
            .tracking = dt->isTracking,
            .head_visible = dt->headVisible,
            .flystick_visible = dt->flystickVisible
    };

    if (dt->isTracking && dt->headVisible) {
        new_pose.position.x = (float) dt->headPos[0] * 0.001f;
        new_pose.position.y = (float) dt->headPos[1] * 0.001f;
        new_pose.position.z = (float) dt->headPos[2] * 0.001f;
        memcpy(sh->head_rot, dt->headRot, sizeof(double) * 9);

    	new_pose.orientation.x = (float) dt->headQuat.x;
    	new_pose.orientation.y = (float) dt->headQuat.y;
    	new_pose.orientation.z = (float) dt->headQuat.z;
    	new_pose.orientation.w = (float) dt->headQuat.w;

    }

    sh->pose = new_pose;
    sh->status = status;

	// Estimate pose at timestamp at_timestamp_ns!
	math_quat_normalize(&sh->pose.orientation);
	out_relation->pose = sh->pose;
	out_relation->relation_flags = (enum xrt_space_relation_flags)(XRT_SPACE_RELATION_ORIENTATION_VALID_BIT |
	                                                               XRT_SPACE_RELATION_POSITION_VALID_BIT |
	                                                               XRT_SPACE_RELATION_ORIENTATION_TRACKED_BIT |
                                                                   XRT_SPACE_RELATION_POSITION_TRACKED_BIT);


}

static xrt_quat
get_corrected_view_orientation(xrt_quat head_rot, xrt_quat view_rot) {
	xrt_quat head_inverse{};
	math_quat_invert(&head_rot, &head_inverse);

	xrt_quat out_quat{};
	math_quat_rotate(&head_inverse, &view_rot, &out_quat);

	return out_quat;
}

static void
cavexr_get_view_poses(struct xrt_device *xdev,
                          const struct xrt_vec3 *default_eye_relation,
                          uint64_t at_timestamp_ns,
                          uint32_t view_count,
                          struct xrt_space_relation *out_head_relation,
                          struct xrt_fov *out_fovs,
                          struct xrt_pose *out_poses)
{
	struct xrt_fov *int_fovs = static_cast<xrt_fov *>(malloc(view_count * sizeof(struct xrt_fov)));
	struct xrt_pose *int_poses = static_cast<xrt_pose *>(malloc(view_count * sizeof(struct xrt_pose)));

	u_device_get_view_poses(xdev, default_eye_relation, at_timestamp_ns, view_count, out_head_relation, int_fovs,
	                        int_poses);

	struct cavexr *cave = cavexr(xdev);

	// Position
	out_poses[1].position.x = out_poses[0].position.x = cave->pose.position.x;
	out_poses[1].position.y = out_poses[0].position.y = cave->pose.position.y;
	out_poses[1].position.z = out_poses[0].position.z = cave->pose.position.z;
	out_poses[3].position.x = out_poses[2].position.x = cave->pose.position.x;
	out_poses[3].position.y = out_poses[2].position.y = cave->pose.position.y;
	out_poses[3].position.z = out_poses[2].position.z = cave->pose.position.z;

    struct xrt_vec3 eye_trans;

    // enable 3d
    if (cave->enable_3d) {
        auto ipd = cave->ipd;
        if (cave->invert_eyes)
            ipd = -ipd;

        auto half_ipd = ipd * 0.5;

        eye_trans.x = cave->head_rot[0] * half_ipd;
        eye_trans.y = cave->head_rot[1] * half_ipd;
        eye_trans.z = cave->head_rot[2] * half_ipd;

        out_poses[0].position.x += eye_trans.x;
        out_poses[0].position.y += eye_trans.y;
        out_poses[0].position.z += eye_trans.z;

        out_poses[1].position.x -= eye_trans.x;
        out_poses[1].position.y -= eye_trans.y;
        out_poses[1].position.z -= eye_trans.z;

        out_poses[2].position.x += eye_trans.x;
        out_poses[2].position.y += eye_trans.y;
        out_poses[2].position.z += eye_trans.z;

        out_poses[3].position.x -= eye_trans.x;
        out_poses[3].position.y -= eye_trans.y;
        out_poses[3].position.z -= eye_trans.z;
    }

    for (int pose = 0; pose < 2; ++pose) {
		const float distanceLeftEdge = out_poses[pose].position.x + cave->dimensions.x / 2;
		const float distanceGround = out_poses[pose].position.y;
		const float distanceWall = out_poses[pose].position.z;

    	xrt_quat orientation{0, 0, 0, 1};

    	out_poses[pose].orientation = get_corrected_view_orientation(cave->pose.orientation, orientation);

        out_fovs[pose].angle_left = -atan2f(distanceLeftEdge, distanceWall);
        out_fovs[pose].angle_right = atan2f(cave->dimensions.x - distanceLeftEdge, distanceWall);
        out_fovs[pose].angle_up = atan2f(cave->dimensions.y - distanceGround, distanceWall);
        out_fovs[pose].angle_down = -atan2f(distanceGround, distanceWall);
    }

    for (int pose = 2; pose < 4; ++pose) {
		const float distanceLeftEdge = out_poses[pose].position.x + cave->dimensions.x / 2;
		const float distanceGround = out_poses[pose].position.y;
		const float distanceWall = out_poses[pose].position.z;

		xrt_quat orientation{-sqrtf(2)/2, 0, 0, sqrtf(2)/2};

		out_poses[pose].orientation = get_corrected_view_orientation(cave->pose.orientation, orientation);

		out_fovs[pose].angle_left = -atan2f(distanceLeftEdge, distanceGround);
		out_fovs[pose].angle_right = atan2f(cave->dimensions.x - distanceLeftEdge, distanceGround);
		out_fovs[pose].angle_up = atan2f(distanceWall, distanceGround);
		out_fovs[pose].angle_down = -atan2f(cave->dimensions.z - distanceWall, distanceGround);
    }

    for (int p = 0; p < 4; ++p) {
        out_poses[p].position.x -= cave->pose.position.x;
        out_poses[p].position.y -= cave->pose.position.y;
        out_poses[p].position.z -= cave->pose.position.z;
    }

    cave->frame_count++;
}

xrt_result_t
cavexr_get_visibility_mask(struct xrt_device *xdev,
                           enum xrt_visibility_mask_type type,
                           uint32_t view_index,
                           struct xrt_visibility_mask **out_mask)
{
    struct xrt_fov fov;

    auto half_pi = (float)(2 * atan(1)); // 90°

    fov.angle_left = -half_pi;
    fov.angle_down = -half_pi;
    fov.angle_up = half_pi;
    fov.angle_right = half_pi;

    u_visibility_mask_get_default(type, &fov, out_mask);
    return XRT_SUCCESS;
}

struct xrt_device *
cavexr_get_controller(struct xrt_device *dev) {

	struct cavexr *sh = cavexr(dev);

	return (struct xrt_device *) sh->controller;
}

struct xrt_device *
cavexr_system_get_controller(struct cavexr *sys)
{
	os_mutex_lock(&sys->mutex);
	sys->controller = dtrack_flystick_controller_create(sys, sys->dtrack);
	os_mutex_unlock(&sys->mutex);

	return (struct xrt_device *)sys->controller;
}

struct xrt_device *
cavexr_create(void)
{
	// This indicates you won't be using Monado's built-in tracking algorithms.
	enum u_device_alloc_flags flags = (enum u_device_alloc_flags)(U_DEVICE_ALLOC_HMD | U_DEVICE_ALLOC_TRACKING_NONE);

	struct cavexr *sh = U_DEVICE_ALLOCATE(struct cavexr, flags, 2, 0);

	os_mutex_init(&sh->mutex);

	// This list should be ordered, most preferred first.
	size_t idx = 0;
	sh->base.hmd->blend_modes[idx++] = XRT_BLEND_MODE_OPAQUE;
	sh->base.hmd->blend_mode_count = idx;
    sh->base.hmd->view_count = 4;

	sh->base.update_inputs = cavexr_update_inputs;
	sh->base.get_tracked_pose = cavexr_get_tracked_pose;
	sh->base.get_view_poses = cavexr_get_view_poses;
	sh->base.destroy = cavexr_destroy;
    sh->base.get_visibility_mask = cavexr_get_visibility_mask;
    sh->base.tracking_origin->type = XRT_TRACKING_TYPE_OTHER;

	sh->pose = XRT_POSE_IDENTITY;
	sh->log_level = debug_get_log_option_sample_log();

	// Print name.
	snprintf(sh->base.str, XRT_DEVICE_NAME_LEN, "CaveXR");
	snprintf(sh->base.serial, XRT_DEVICE_NAME_LEN, "0123456789abcdef");

	// Init DTrack
    sh->dtrack = new CaveXrDTrack(DTRACK_PORT);

	// Setup input.
	sh->base.name = XRT_DEVICE_GENERIC_HMD;
	sh->base.device_type = XRT_DEVICE_TYPE_HMD;

	sh->base.inputs[0].name = XRT_INPUT_GENERIC_HEAD_POSE;
	sh->controller = (dtrack_flystick_controller*) cavexr_system_get_controller(sh);

	sh->base.orientation_tracking_supported = true;
	sh->base.position_tracking_supported = true;

	sh->created_ns = os_monotonic_get_ns();

    sh->frame_count = 0;

    sh->enable_3d = true;
    sh->ipd = 0.062f;

    // Dimensions par défaut
    sh->dimensions.x = 4.07f; // Largeur
    sh->dimensions.y = 2.30f; // Hauteur
    sh->dimensions.z = 2.30f; // Profondeur

	// Set up display details
	// refresh rate
	sh->base.hmd->screens[0].nominal_frame_interval_ns = time_s_to_ns(1.0f / 60.0f);

	struct u_device_simple_info info;

	info.display.w_pixels = 2560;
	info.display.h_pixels = 1440;
	info.display.w_meters = 4.07f;
	info.display.h_meters = 2.30f;
	info.lens_vertical_position_meters = 0.07f / 2.0f;
	info.lens_horizontal_separation_meters = 0.13f;

	for (int i = 0; i < XRT_MAX_VIEWS; i++) {
		info.fov[i] = 85.0f * ((float)(M_PI) / 180.0f);
	}

	if (!u_device_setup_fullscreen(&sh->base, &info)) {
		SH_ERROR(sh, "Failed to setup basic device info");
		cavexr_destroy(&sh->base);
		return NULL;
	}

	for (int i = 0; i < XRT_MAX_VIEWS; ++i) {
		// viewport: Layout on a info.display-sized target
		sh->base.hmd->views[i].viewport.x_pixels = i >= 2 ? info.display.w_pixels/2 : 0;
		sh->base.hmd->views[i].viewport.y_pixels = 0;
		sh->base.hmd->views[i].viewport.w_pixels = info.display.w_pixels/2;
		sh->base.hmd->views[i].viewport.h_pixels = info.display.h_pixels;

		// display: View texture base resolution
		sh->base.hmd->views[i].display.w_pixels = info.display.w_pixels/2;
		sh->base.hmd->views[i].display.h_pixels = info.display.h_pixels;
	}



	// Distortion information, fills in xdev->compute_distortion().
	u_distortion_mesh_set_none(&sh->base);

	// Setup variable tracker: Optional but useful for debugging
	u_var_add_root(sh, "Cave Device", true);
	u_var_add_pose(sh, &sh->pose, "pose");
	u_var_add_log_level(sh, &sh->log_level, "log_level");

    // Start DTrack thread
    pthread_create(&sh->dtrack_thread, NULL, cavexr_dtrack_run, (void*)sh->dtrack);

    // Open info window
    pthread_create(&debugThread, NULL, (void *(*)(void *)) cavexr_debug_window, sh);

    sh->pose.position.x = 0.00f;
    sh->pose.position.y = 1.00f;
    sh->pose.position.z = 1.00f;


	return &sh->base;
}
