// Copyright 2018-2020, Collabora, Ltd.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Holds Vulkan related functions.
 * @author Jakob Bornecrantz <jakob@collabora.com>
 * @ingroup oxr_main
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "util/u_misc.h"
#include "util/u_debug.h"

#include "xrt/xrt_gfx_vk.h"

#include "oxr_objects.h"
#include "oxr_logger.h"
#include "oxr_two_call.h"


#define GET_PROC(name) PFN_##name name = (PFN_##name)getProc(vkInstance, #name)

XrResult
oxr_vk_get_instance_exts(struct oxr_logger *log,
                         struct oxr_system *sys,
                         uint32_t namesCapacityInput,
                         uint32_t *namesCountOutput,
                         char *namesString)
{
	size_t length = strlen(xrt_gfx_vk_instance_extensions) + 1;

	OXR_TWO_CALL_HELPER(log, namesCapacityInput, namesCountOutput,
	                    namesString, length, xrt_gfx_vk_instance_extensions,
	                    XR_SUCCESS);
}

XrResult
oxr_vk_get_device_exts(struct oxr_logger *log,
                       struct oxr_system *sys,
                       uint32_t namesCapacityInput,
                       uint32_t *namesCountOutput,
                       char *namesString)
{
	size_t length = strlen(xrt_gfx_vk_device_extensions) + 1;

	OXR_TWO_CALL_HELPER(log, namesCapacityInput, namesCountOutput,
	                    namesString, length, xrt_gfx_vk_device_extensions,
	                    XR_SUCCESS);
}

XrResult
oxr_vk_get_requirements(struct oxr_logger *log,
                        struct oxr_system *sys,
                        XrGraphicsRequirementsVulkanKHR *graphicsRequirements)
{
	struct xrt_api_requirements ver;

	xrt_gfx_vk_get_versions(&ver);
	graphicsRequirements->minApiVersionSupported =
	    XR_MAKE_VERSION(ver.min_major, ver.min_minor, ver.min_patch);
	graphicsRequirements->maxApiVersionSupported =
	    XR_MAKE_VERSION(ver.max_major, ver.max_minor, ver.max_patch);

	sys->gotten_requirements = true;

	return XR_SUCCESS;
}

DEBUG_GET_ONCE_BOOL_OPTION(print_debug, "XRT_COMPOSITOR_PRINT_DEBUG", false)

XrResult
oxr_vk_get_physical_device(struct oxr_logger *log,
                           struct oxr_instance *inst,
                           struct oxr_system *sys,
                           VkInstance vkInstance,
                           PFN_vkGetInstanceProcAddr getProc,
                           VkPhysicalDevice *vkPhysicalDevice)
{
	GET_PROC(vkEnumeratePhysicalDevices);
	GET_PROC(vkGetPhysicalDeviceProperties2);
	VkResult vk_ret;
	uint32_t count;

	vk_ret = vkEnumeratePhysicalDevices(vkInstance, &count, NULL);
	if (vk_ret != VK_SUCCESS) {
		return oxr_error(
		    log, XR_ERROR_RUNTIME_FAILURE,
		    "Call to vkEnumeratePhysicalDevices returned %u", vk_ret);
	}
	if (count == 0) {
		return oxr_error(
		    log, XR_ERROR_RUNTIME_FAILURE,
		    "Call to vkEnumeratePhysicalDevices returned zero "
		    "VkPhysicalDevices");
	}

	VkPhysicalDevice *phys = U_TYPED_ARRAY_CALLOC(VkPhysicalDevice, count);
	vk_ret = vkEnumeratePhysicalDevices(vkInstance, &count, phys);
	if (vk_ret != VK_SUCCESS) {
		free(phys);
		return oxr_error(
		    log, XR_ERROR_RUNTIME_FAILURE,
		    "Call to vkEnumeratePhysicalDevices returned %u", vk_ret);
	}
	if (count == 0) {
		free(phys);
		return oxr_error(
		    log, XR_ERROR_RUNTIME_FAILURE,
		    "Call to vkEnumeratePhysicalDevices returned zero "
		    "VkPhysicalDevices");
	}

	char suggested_uuid_str[XRT_GPU_UUID_SIZE * 3 + 1] = {0};
	for (int i = 0; i < XRT_GPU_UUID_SIZE; i++) {
		sprintf(suggested_uuid_str + i * 3, "%02x ",
		        sys->xcn->base.info.client_vk_deviceUUID[i]);
	}

	bool print_debug = debug_get_bool_option_print_debug();
	int gpu_index = -1;
	for (uint32_t i = 0; i < count; i++) {
		VkPhysicalDeviceIDProperties pdidp = {
		    .sType = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_ID_PROPERTIES};

		VkPhysicalDeviceProperties2 pdp2 = {
		    .sType = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_PROPERTIES_2,
		    .pNext = &pdidp};

		vkGetPhysicalDeviceProperties2(phys[i], &pdp2);

		char uuid_str[XRT_GPU_UUID_SIZE * 3 + 1] = {0};
		if (print_debug) {
			for (int i = 0; i < XRT_GPU_UUID_SIZE; i++) {
				sprintf(uuid_str + i * 3, "%02x ",
				        pdidp.deviceUUID[i]);
			}
			oxr_log(log, "GPU %d: uuid %s", i, uuid_str);
		}

		if (memcmp(pdidp.deviceUUID,
		           sys->xcn->base.info.client_vk_deviceUUID,
		           XRT_GPU_UUID_SIZE) == 0) {
			gpu_index = i;
			if (print_debug) {
				oxr_log(log,
				        "Using GPU %d with uuid %s suggested "
				        "by runtime",
				        gpu_index, uuid_str);
			}
			break;
		}
	}

	if (gpu_index == -1) {
		oxr_warn(
		    log,
		    "Did not find runtime suggested GPU, fall back to GPU 0");
		gpu_index = 0;
	}

	*vkPhysicalDevice = phys[gpu_index];

	free(phys);

	return XR_SUCCESS;
}
