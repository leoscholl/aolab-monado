// Copyright 2019-2020, Collabora, Ltd.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Android window code.
 * @author Rylie Pavlik <rylie.pavlik@collabora.com>
 * @author Lubosz Sarnecki <lubosz.sarnecki@collabora.com>
 * @author Jakob Bornecrantz <jakob@collabora.com>
 * @ingroup comp_main
 */

#include "xrt/xrt_compiler.h"
#include "xrt/xrt_instance.h"
#include "xrt/xrt_android.h"

#include "util/u_misc.h"
#include "os/os_threading.h"

#include "android/android_globals.h"
#include "android/android_custom_surface.h"

#include "main/comp_window.h"

#include <android/native_window.h>

#include <poll.h>
#include <errno.h>
#include <stdlib.h>
#include <string.h>
#include <linux/input.h>

#define WINDOW_TITLE "Monado"

/*
 *
 * Private structs.
 *
 */

/*!
 * An Android window.
 *
 * @implements comp_target_swapchain
 */
struct comp_window_android
{
	struct comp_target_swapchain base;
	struct comp_target_create_images_info create_info;
	void (*real_create_images)(struct comp_target *ct, const struct comp_target_create_images_info *create_info);

	bool needs_create_images;

	ANativeWindow *native_window;
	struct os_mutex surface_mutex;

	struct android_custom_surface *custom_surface;
};


/*
 *
 * Functions.
 *
 */

static inline struct vk_bundle *
get_vk(struct comp_window_android *cwa)
{
	return &cwa->base.base.c->base.vk;
}

static bool
comp_window_android_init(struct comp_target *ct)
{
	(void)ct;

	return true;
}


static void
comp_window_android_update_window_title(struct comp_target *ct, const char *title)
{
	(void)ct;
}

static struct ANativeWindow *
_create_android_window(struct comp_window_android *cwa)
{
	// 0 means default display
	cwa->custom_surface = android_custom_surface_async_start( //
	    android_globals_get_vm(),                             // vm
	    android_globals_get_context(),                        // context
	    0,                                                    // display_id
	    WINDOW_TITLE,                                         // title in dumpsys
	    0);                                                   // preferred_display_mode_id

	if (cwa->custom_surface == NULL) {
		COMP_ERROR(cwa->base.base.c,
		           "comp_window_android_create_surface: could not "
		           "start asynchronous attachment of our custom surface");
		return NULL;
	}

	return android_custom_surface_wait_get_surface(cwa->custom_surface, 2000);
}

static VkResult
comp_window_android_create_surface(struct comp_window_android *cwa,
                                   struct ANativeWindow *window,
                                   VkSurfaceKHR *out_surface)
{
	struct vk_bundle *vk = get_vk(cwa);
	VkResult ret;

	VkAndroidSurfaceCreateInfoKHR surface_info = {
	    .sType = VK_STRUCTURE_TYPE_ANDROID_SURFACE_CREATE_INFO_KHR,
	    .flags = 0,
	    .window = window,
	};

	VkSurfaceKHR surface = VK_NULL_HANDLE;
	ret = vk->vkCreateAndroidSurfaceKHR( //
	    vk->instance,                    //
	    &surface_info,                   //
	    NULL,                            //
	    &surface);                       //
	if (ret != VK_SUCCESS) {
		COMP_ERROR(cwa->base.base.c, "vkCreateAndroidSurfaceKHR: %s", vk_result_string(ret));
		return ret;
	}

	VK_NAME_SURFACE(vk, surface, "comp_window_android surface");
	*out_surface = surface;

	return VK_SUCCESS;
}

static bool
comp_window_android_init_swapchain(struct comp_target *ct, uint32_t width, uint32_t height)
{
	struct comp_window_android *cwa = (struct comp_window_android *)ct;
	VkResult ret;

	cwa->create_info.extent.width = width;
	cwa->create_info.extent.height = height;

	struct ANativeWindow *window = NULL;

	if (android_globals_get_activity() != NULL) {
		/* In process: Creating surface from activity */
		COMP_INFO(cwa->base.base.c, "We have an activity, so assuming in-process.");
		window = _create_android_window(cwa);
	} else if (android_custom_surface_can_draw_overlays(android_globals_get_vm(), android_globals_get_context())) {
		/* Out of process: Create surface */
		window = _create_android_window(cwa);
	} else {
		COMP_INFO(cwa->base.base.c, "No activity, so assuming out-of-process.");
		/* Out of process: Getting cached surface.
		 * This loop polls for a surface created by Client.java in blockingConnect.
		 * TODO: change java code to callback native code to notify Session lifecycle progress, instead
		 * of polling here
		 */
		for (int i = 0; i < 100; i++) {
			window = (struct ANativeWindow *)android_globals_get_window();
			if (window)
				break;
			os_nanosleep(20 * U_TIME_1MS_IN_NS);
		}
	}


	if (window == NULL) {
		COMP_ERROR(cwa->base.base.c, "could not get ANativeWindow");
		return false;
	}

	ret = comp_window_android_create_surface(cwa, window, &cwa->base.surface.handle);
	if (ret != VK_SUCCESS) {
		COMP_ERROR(ct->c, "Failed to create surface '%s'!", vk_result_string(ret));
		return false;
	}

	return true;
}

static void
comp_window_android_flush(struct comp_target *ct)
{
	(void)ct;
}

static void
comp_window_android_create_images_stub(struct comp_target *ct, const struct comp_target_create_images_info *create_info)
{

	struct comp_window_android *cwa = (struct comp_window_android *)ct;
	if (cwa->native_window != NULL) {
		cwa->real_create_images(ct, create_info);
		return;
	}
	cwa->create_info = *create_info;
	cwa->needs_create_images = true;
}

static bool
comp_window_android_handle_surface_acquired(struct xrt_instance_android *xinst_android,
                                            struct _ANativeWindow *window,
                                            enum xrt_android_surface_event event,
                                            void *userdata)
{
	struct comp_window_android *cwa = (struct comp_window_android *)userdata;
	COMP_INFO(cwa->base.base.c, "comp_window_android_handle_surface_acquired: got a surface!");
	if (cwa->native_window == NULL) {
		cwa->native_window = (ANativeWindow *)window;
		VkResult ret = comp_window_android_create_surface(cwa, cwa->native_window, &cwa->base.surface.handle);
		if (ret != VK_SUCCESS) {
			COMP_ERROR(cwa->base.base.c, "Failed to create surface '%s'!", vk_result_string(ret));
			return true;
		}
		if (cwa->needs_create_images) {
			cwa->needs_create_images = false;
			cwa->real_create_images(&cwa->base.base, &cwa->create_info);
		}
	}
	return true;
}

static bool
comp_window_android_handle_surface_lost(struct xrt_instance_android *xinst_android,
                                        struct _ANativeWindow *window,
                                        enum xrt_android_surface_event event,
                                        void *userdata)
{
	struct comp_window_android *cwa = (struct comp_window_android *)userdata;
	COMP_INFO(cwa->base.base.c, "comp_window_android_handle_surface_lost: oh noes!");
	if (cwa->native_window == (ANativeWindow *)window) {
		// yeah, we're losing this surface.
		os_mutex_lock(&cwa->surface_mutex);

		comp_target_swapchain_cleanup(&cwa->base);
		cwa->native_window = NULL;

		os_mutex_unlock(&cwa->surface_mutex);
	}
	return true;
}

static void
comp_window_android_destroy(struct comp_target *ct)
{
	struct comp_window_android *cwa = (struct comp_window_android *)ct;

	struct xrt_instance *xinst = cwa->base.base.c->xinst;
	xinst->android_instance->remove_surface_callback(xinst->android_instance,
	                                                 comp_window_android_handle_surface_acquired,
	                                                 XRT_ANDROID_SURFACE_EVENT_ACQUIRED, (void *)cwa);
	xinst->android_instance->remove_surface_callback(xinst->android_instance,
	                                                 comp_window_android_handle_surface_lost,
	                                                 XRT_ANDROID_SURFACE_EVENT_LOST, (void *)cwa);

	os_mutex_destroy(&cwa->surface_mutex);
	comp_target_swapchain_cleanup(&cwa->base);

	android_custom_surface_destroy(&cwa->custom_surface);

	free(ct);
}

struct comp_target *
comp_window_android_create(struct comp_compositor *c)
{
	struct comp_window_android *w = U_TYPED_CALLOC(struct comp_window_android);

	// The display timing code hasn't been tested on Android and may be broken.
	comp_target_swapchain_init_and_set_fnptrs(&w->base, COMP_TARGET_FORCE_FAKE_DISPLAY_TIMING);

	w->base.base.name = "Android";
	w->base.base.destroy = comp_window_android_destroy;
	w->base.base.flush = comp_window_android_flush;
	w->base.base.init_pre_vulkan = comp_window_android_init;
	w->base.base.init_post_vulkan = comp_window_android_init_swapchain;
	w->base.base.set_title = comp_window_android_update_window_title;
	w->base.base.c = c;

	// Intercept this call
	w->real_create_images = w->base.base.create_images;
	w->base.base.create_images = comp_window_android_create_images_stub;

	os_mutex_init(&w->surface_mutex);

	struct xrt_instance *xinst = c->xinst;
	xinst->android_instance->register_surface_callback(xinst->android_instance,
	                                                   comp_window_android_handle_surface_acquired,
	                                                   XRT_ANDROID_SURFACE_EVENT_ACQUIRED, (void *)w);
	xinst->android_instance->register_surface_callback(xinst->android_instance,
	                                                   comp_window_android_handle_surface_lost,
	                                                   XRT_ANDROID_SURFACE_EVENT_LOST, (void *)w);

	return &w->base.base;
}


/*
 *
 * Factory
 *
 */

static const char *instance_extensions[] = {
    VK_KHR_ANDROID_SURFACE_EXTENSION_NAME,
};

static bool
detect(const struct comp_target_factory *ctf, struct comp_compositor *c)
{
	return false;
}

static bool
create_target(const struct comp_target_factory *ctf, struct comp_compositor *c, struct comp_target **out_ct)
{
	struct comp_target *ct = comp_window_android_create(c);
	if (ct == NULL) {
		return false;
	}

	*out_ct = ct;

	return true;
}

const struct comp_target_factory comp_target_factory_android = {
    .name = "Android",
    .identifier = "android",
    .requires_vulkan_for_create = false,
    .is_deferred = true,
    .required_instance_version = 0,
    .required_instance_extensions = instance_extensions,
    .required_instance_extension_count = ARRAY_SIZE(instance_extensions),
    .optional_device_extensions = NULL,
    .optional_device_extension_count = 0,
    .detect = detect,
    .create_target = create_target,
};
