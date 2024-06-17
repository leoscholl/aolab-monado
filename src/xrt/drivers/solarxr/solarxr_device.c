// Copyright 2024, rcelyte
// SPDX-License-Identifier: BSL-1.0

#define solarxr_protocol_rpc_SaveFileNotification_file_extension                                                       \
	solarxr_protocol_rpc_SaveFileNotification_file_extension
#include "solarxr_device.h"
#include "websocket.h"
#include "solarxr_protocol.h"

#include "math/m_relation_history.h"
#include "math/m_vec3.h"
#include "os/os_threading.h"
#include "os/os_time.h"
#include "util/u_debug.h"
#include "util/u_device.h"

#include "xrt/xrt_config_build.h" // TODO: drop `#ifdef XRT_FEATURE_OPENXR_BODY_TRACKING_FULL_BODY_META` guards once !2206 is merged

#include <stdio.h>
#include <wchar.h>

#define BodyPart_COUNT 25 // must be kept in sync with schema
#define MAX_GENERIC_TRACKERS 32
static const uint16_t updateIntervalMs = 10;
typedef uint32_t solarxr_trackerid_t;

DEBUG_GET_ONCE_LOG_OPTION(solarxr_log, "SOLARXR_LOG", U_LOGGING_WARN)
DEBUG_GET_ONCE_BOOL_OPTION(solarxr_raw_trackers, "SOLARXR_RAW_TRACKERS", false)

struct solarxr_generic_tracker
{
	struct xrt_device base;
	struct os_mutex *mutex;
	struct m_relation_history *history;
	struct solarxr_generic_tracker **weakRef;
};

struct solarxr_device
{
	struct xrt_device base;
	struct xrt_device *hmd;
	struct os_thread thread;
	struct WebSocket socket;
	struct os_mutex mutex;
	int64_t timestamp;
	uint32_t generation;
	struct solarxr_device_bone
	{
		struct xrt_pose pose;
		float length;
	} bones[BodyPart_COUNT];
	solarxr_trackerid_t trackerIds[MAX_GENERIC_TRACKERS];
	struct solarxr_generic_tracker *trackers[MAX_GENERIC_TRACKERS];
};

static inline struct solarxr_device *
solarxr_device(struct xrt_device *const xdev)
{
	return (struct solarxr_device *)xdev;
}

static inline struct solarxr_generic_tracker *
solarxr_generic_tracker(struct xrt_device *const xdev)
{
	return (struct solarxr_generic_tracker *)xdev;
}

static void
solarxr_device_get_tracked_pose(struct xrt_device *const xdev,
                                const enum xrt_input_name name,
                                const int64_t at_timestamp_ns,
                                struct xrt_space_relation *const out_relation)
{
	struct xrt_device *const hmd = solarxr_device(xdev)->hmd;
	if (hmd != NULL) {
		xrt_device_get_tracked_pose(hmd, name, at_timestamp_ns, out_relation);
	} else {
		*out_relation = (struct xrt_space_relation){0};
	}
}

static void
solarxr_device_update_inputs(struct xrt_device *const xdev)
{
	struct solarxr_device *const device = solarxr_device(xdev);
	os_mutex_lock(&device->mutex);
	for (uint32_t i = 0; i < device->base.input_count; ++i) {
		device->base.inputs[i].timestamp = device->timestamp;
	}
	for (uint32_t i = 0; i < ARRAY_SIZE(device->trackers); ++i) {
		if (device->trackers[i] != NULL) {
			device->trackers[i]->base.inputs[0].timestamp = device->timestamp;
		}
	}
	os_mutex_unlock(&device->mutex);

	struct xrt_space_relation head = {0};
	solarxr_device_get_tracked_pose(xdev, XRT_INPUT_GENERIC_HEAD_POSE, os_monotonic_get_ns(), &head);
	if (head.relation_flags == 0) {
		return;
	}
	char feedback[0x200];
	// clang-format off
	unsigned feedback_len = (unsigned)snprintf(feedback, sizeof(feedback), "%s%f%s%f%s%f%s%f%s%f%s%f%s%f%s", "{"
		"\"type\":\"pos\","
		"\"tracker_id\":0,"
		"\"x\":", (double)head.pose.position.x, ","
		"\"y\":", (double)head.pose.position.y - .2f, "," // SlimeVR add 0.2 to the WebSocket HMD's Y position for some reason
		"\"z\":", (double)head.pose.position.z, ","
		"\"qw\":", (double)head.pose.orientation.w, ","
		"\"qx\":", (double)head.pose.orientation.x, ","
		"\"qy\":", (double)head.pose.orientation.y, ","
		"\"qz\":", (double)head.pose.orientation.z,
	"}");
	// clang-format on
	assert(feedback_len < sizeof(feedback));
	WebSocket_sendWithOpcode(&device->socket, (uint8_t *)feedback, feedback_len, 0x1);
}

static inline struct xrt_body_skeleton_joint_fb
offset_joint(const struct xrt_body_skeleton_joint_fb parent, const int32_t name, const struct xrt_vec3 offset)
{
	return (struct xrt_body_skeleton_joint_fb){
	    .pose =
	        {
	            .orientation = parent.pose.orientation,
	            .position = m_vec3_add(parent.pose.position, offset),
	        },
	    .joint = name,
	    .parent_joint = parent.joint,
	};
}

static xrt_result_t
solarxr_device_get_body_skeleton(struct xrt_device *const xdev,
                                 const enum xrt_input_name body_tracking_type,
                                 struct xrt_body_skeleton *const out_value)
{
	struct xrt_body_skeleton_joint_fb *joints;
	uint32_t joint_count;
	int32_t none;
	switch (body_tracking_type) {
	case XRT_INPUT_FB_BODY_TRACKING: {
		joints = out_value->body_skeleton_fb.joints;
		joint_count = ARRAY_SIZE(out_value->body_skeleton_fb.joints);
		none = XRT_BODY_JOINT_NONE_FB;
		break;
	}
#ifdef XRT_FEATURE_OPENXR_BODY_TRACKING_FULL_BODY_META
	case XRT_INPUT_META_FULL_BODY_TRACKING: {
		joints = out_value->full_body_skeleton_meta.joints;
		joint_count = ARRAY_SIZE(out_value->full_body_skeleton_meta.joints);
		none = XRT_FULL_BODY_JOINT_NONE_META;
		break;
	}
#endif
	default: return XRT_ERROR_NOT_IMPLEMENTED;
	}

	struct solarxr_device *const device = solarxr_device(xdev);
	for (uint32_t i = 0; i < joint_count; ++i) {
		joints[i] = (struct xrt_body_skeleton_joint_fb){XRT_POSE_IDENTITY, none, none};
	}

	// The spec doesn't define a particular layout for these joints beyond simply "a T-pose", so...
	// clang-format off
	joints[0] = (struct xrt_body_skeleton_joint_fb){XRT_POSE_IDENTITY, XRT_BODY_JOINT_HEAD_FB, XRT_BODY_JOINT_ROOT_FB};
	joints[1] = offset_joint(joints[0], XRT_BODY_JOINT_NECK_FB, (struct xrt_vec3){0.f, -device->bones[solarxr_protocol_datatypes_BodyPart_NECK].length, 0.f});
	joints[2] = offset_joint(joints[1], XRT_BODY_JOINT_CHEST_FB, (struct xrt_vec3){0.f, -device->bones[solarxr_protocol_datatypes_BodyPart_UPPER_CHEST].length, 0.f});
	joints[3] = offset_joint(joints[2], XRT_BODY_JOINT_SPINE_UPPER_FB, (struct xrt_vec3){0.f, -device->bones[solarxr_protocol_datatypes_BodyPart_CHEST].length, 0.f});
	joints[4] = offset_joint(joints[3], XRT_BODY_JOINT_SPINE_LOWER_FB, (struct xrt_vec3){0.f, -device->bones[solarxr_protocol_datatypes_BodyPart_WAIST].length, 0.f});
	joints[5] = offset_joint(joints[4], XRT_BODY_JOINT_HIPS_FB, (struct xrt_vec3){0.f, -device->bones[solarxr_protocol_datatypes_BodyPart_HIP].length, 0.f});
	joints[6] = offset_joint(joints[1], XRT_BODY_JOINT_LEFT_SHOULDER_FB, (struct xrt_vec3){-device->bones[solarxr_protocol_datatypes_BodyPart_LEFT_SHOULDER].length, 0.f, 0.f});
	joints[7] = offset_joint(joints[1], XRT_BODY_JOINT_RIGHT_SHOULDER_FB, (struct xrt_vec3){device->bones[solarxr_protocol_datatypes_BodyPart_RIGHT_SHOULDER].length, 0.f, 0.f});
	joints[8] = offset_joint(joints[6], XRT_BODY_JOINT_LEFT_ARM_UPPER_FB, (struct xrt_vec3){-device->bones[solarxr_protocol_datatypes_BodyPart_LEFT_UPPER_ARM].length, 0.f, 0.f});
	joints[9] = offset_joint(joints[7], XRT_BODY_JOINT_RIGHT_ARM_UPPER_FB, (struct xrt_vec3){device->bones[solarxr_protocol_datatypes_BodyPart_RIGHT_UPPER_ARM].length, 0.f, 0.f});
	joints[10] = offset_joint(joints[8], XRT_BODY_JOINT_LEFT_ARM_LOWER_FB, (struct xrt_vec3){-device->bones[solarxr_protocol_datatypes_BodyPart_LEFT_LOWER_ARM].length, 0.f, 0.f});
	joints[11] = offset_joint(joints[9], XRT_BODY_JOINT_RIGHT_ARM_LOWER_FB, (struct xrt_vec3){device->bones[solarxr_protocol_datatypes_BodyPart_RIGHT_LOWER_ARM].length, 0.f, 0.f});
	joints[12] = offset_joint(joints[10], XRT_BODY_JOINT_LEFT_HAND_WRIST_FB, (struct xrt_vec3){-device->bones[solarxr_protocol_datatypes_BodyPart_LEFT_HAND].length, 0.f, 0.f});
	joints[13] = offset_joint(joints[11], XRT_BODY_JOINT_RIGHT_HAND_WRIST_FB, (struct xrt_vec3){device->bones[solarxr_protocol_datatypes_BodyPart_RIGHT_HAND].length, 0.f, 0.f});
#ifdef XRT_FEATURE_OPENXR_BODY_TRACKING_FULL_BODY_META
	if (body_tracking_type != XRT_INPUT_META_FULL_BODY_TRACKING) {
		return XRT_SUCCESS;
	}
	joints[14] = offset_joint(joints[5], XRT_FULL_BODY_JOINT_LEFT_UPPER_LEG_META, (struct xrt_vec3){0.f, -device->bones[solarxr_protocol_datatypes_BodyPart_LEFT_UPPER_LEG].length, 0.f});
	joints[15] = offset_joint(joints[5], XRT_FULL_BODY_JOINT_RIGHT_UPPER_LEG_META, (struct xrt_vec3){0.f, -device->bones[solarxr_protocol_datatypes_BodyPart_RIGHT_UPPER_LEG].length, 0.f});
	joints[16] = offset_joint(joints[14], XRT_FULL_BODY_JOINT_LEFT_LOWER_LEG_META, (struct xrt_vec3){0.f, -device->bones[solarxr_protocol_datatypes_BodyPart_LEFT_LOWER_LEG].length, 0.f});
	joints[17] = offset_joint(joints[15], XRT_FULL_BODY_JOINT_RIGHT_LOWER_LEG_META, (struct xrt_vec3){0.f, -device->bones[solarxr_protocol_datatypes_BodyPart_RIGHT_LOWER_LEG].length, 0.f});
	joints[18] = offset_joint(joints[16], XRT_FULL_BODY_JOINT_LEFT_FOOT_TRANSVERSE_META, (struct xrt_vec3){0.f, 0.f, -device->bones[solarxr_protocol_datatypes_BodyPart_LEFT_FOOT].length});
	joints[19] = offset_joint(joints[17], XRT_FULL_BODY_JOINT_RIGHT_FOOT_TRANSVERSE_META, (struct xrt_vec3){0.f, 0.f, -device->bones[solarxr_protocol_datatypes_BodyPart_RIGHT_FOOT].length});
#endif
	// clang-format on
	return XRT_SUCCESS;
}

static xrt_result_t
solarxr_device_get_body_joints(struct xrt_device *const xdev,
                               const enum xrt_input_name body_tracking_type,
                               const int64_t desired_timestamp_ns,
                               struct xrt_body_joint_set *const out_value)
{
	static const uint32_t jointMap[BodyPart_COUNT] = {
	    [solarxr_protocol_datatypes_BodyPart_HEAD] = XRT_BODY_JOINT_HEAD_FB,
	    [solarxr_protocol_datatypes_BodyPart_NECK] = XRT_BODY_JOINT_NECK_FB,
	    [solarxr_protocol_datatypes_BodyPart_CHEST] = XRT_BODY_JOINT_SPINE_UPPER_FB,
	    [solarxr_protocol_datatypes_BodyPart_WAIST] = XRT_BODY_JOINT_SPINE_LOWER_FB,
	    [solarxr_protocol_datatypes_BodyPart_HIP] = XRT_BODY_JOINT_HIPS_FB,
#ifdef XRT_FEATURE_OPENXR_BODY_TRACKING_FULL_BODY_META
	    [solarxr_protocol_datatypes_BodyPart_LEFT_UPPER_LEG] = XRT_FULL_BODY_JOINT_LEFT_UPPER_LEG_META,
	    [solarxr_protocol_datatypes_BodyPart_RIGHT_UPPER_LEG] = XRT_FULL_BODY_JOINT_RIGHT_UPPER_LEG_META,
	    [solarxr_protocol_datatypes_BodyPart_LEFT_LOWER_LEG] = XRT_FULL_BODY_JOINT_LEFT_LOWER_LEG_META,
	    [solarxr_protocol_datatypes_BodyPart_RIGHT_LOWER_LEG] = XRT_FULL_BODY_JOINT_RIGHT_LOWER_LEG_META,
	    [solarxr_protocol_datatypes_BodyPart_LEFT_FOOT] = XRT_FULL_BODY_JOINT_LEFT_FOOT_TRANSVERSE_META,
	    [solarxr_protocol_datatypes_BodyPart_RIGHT_FOOT] = XRT_FULL_BODY_JOINT_RIGHT_FOOT_TRANSVERSE_META,
#endif
	    [solarxr_protocol_datatypes_BodyPart_LEFT_LOWER_ARM] = XRT_BODY_JOINT_LEFT_ARM_LOWER_FB,
	    [solarxr_protocol_datatypes_BodyPart_RIGHT_LOWER_ARM] = XRT_BODY_JOINT_RIGHT_ARM_LOWER_FB,
	    [solarxr_protocol_datatypes_BodyPart_LEFT_UPPER_ARM] = XRT_BODY_JOINT_LEFT_ARM_UPPER_FB,
	    [solarxr_protocol_datatypes_BodyPart_RIGHT_UPPER_ARM] = XRT_BODY_JOINT_RIGHT_ARM_UPPER_FB,
	    [solarxr_protocol_datatypes_BodyPart_LEFT_HAND] = XRT_BODY_JOINT_LEFT_HAND_WRIST_FB,
	    [solarxr_protocol_datatypes_BodyPart_RIGHT_HAND] = XRT_BODY_JOINT_RIGHT_HAND_WRIST_FB,
	    [solarxr_protocol_datatypes_BodyPart_LEFT_SHOULDER] = XRT_BODY_JOINT_LEFT_SHOULDER_FB,
	    [solarxr_protocol_datatypes_BodyPart_RIGHT_SHOULDER] = XRT_BODY_JOINT_RIGHT_SHOULDER_FB,
	    [solarxr_protocol_datatypes_BodyPart_UPPER_CHEST] = XRT_BODY_JOINT_CHEST_FB,
	    // LEFT_HIP
	    // RIGHT_HIP
	};
	struct xrt_body_joint_location_fb *joints;
	uint32_t joint_count;
	switch (body_tracking_type) {
	case XRT_INPUT_FB_BODY_TRACKING: {
		joints = out_value->body_joint_set_fb.joint_locations;
		joint_count = ARRAY_SIZE(out_value->body_joint_set_fb.joint_locations);
		break;
	}
#ifdef XRT_FEATURE_OPENXR_BODY_TRACKING_FULL_BODY_META
	case XRT_INPUT_META_FULL_BODY_TRACKING: {
		joints = out_value->full_body_joint_set_meta.joint_locations;
		joint_count = ARRAY_SIZE(out_value->full_body_joint_set_meta.joint_locations);
		break;
	}
#endif
	default: return XRT_ERROR_NOT_IMPLEMENTED;
	}

	struct solarxr_device *const device = solarxr_device(xdev);
	os_mutex_lock(&device->mutex);
	out_value->base_body_joint_set_meta.sample_time_ns = device->timestamp;
	out_value->base_body_joint_set_meta.confidence = 1.f; // N/A
	out_value->base_body_joint_set_meta.skeleton_changed_count = device->generation;
	out_value->base_body_joint_set_meta.is_active = true;
	for (uint32_t i = 0; i < joint_count; ++i) {
		joints[i].relation = (struct xrt_space_relation)XRT_SPACE_RELATION_ZERO;
	}
	for (solarxr_protocol_datatypes_BodyPart_enum_t part = 0; part < ARRAY_SIZE(device->bones); ++part) {
		const struct xrt_pose pose = device->bones[part].pose;
		const uint32_t index = jointMap[part];
		if (index == 0 || index >= joint_count ||
		    memcmp(&pose.orientation, &(struct xrt_quat){0}, sizeof(struct xrt_quat)) == 0) {
			continue;
		}
		joints[index].relation = (struct xrt_space_relation){
		    .relation_flags = XRT_SPACE_RELATION_ORIENTATION_VALID_BIT | XRT_SPACE_RELATION_POSITION_VALID_BIT |
		                      XRT_SPACE_RELATION_ORIENTATION_TRACKED_BIT |
		                      XRT_SPACE_RELATION_POSITION_TRACKED_BIT,
		    .pose = pose,
		};
	}
	out_value->body_pose = (struct xrt_space_relation){
	    .relation_flags = XRT_SPACE_RELATION_ORIENTATION_VALID_BIT | XRT_SPACE_RELATION_POSITION_VALID_BIT |
	                      XRT_SPACE_RELATION_ORIENTATION_TRACKED_BIT | XRT_SPACE_RELATION_POSITION_TRACKED_BIT,
	    .pose = XRT_POSE_IDENTITY,
	};
	os_mutex_unlock(&device->mutex);
	return XRT_SUCCESS;
}

static void
solarxr_device_handle_trackers(struct solarxr_device *const device,
                               const solarxr_protocol_data_feed_tracker_TrackerData_vec_t trackers)
{
	for (uint32_t j = 0, trackers_len = solarxr_protocol_data_feed_tracker_TrackerData_vec_len(trackers);
	     j < trackers_len; ++j) {
		const solarxr_protocol_data_feed_tracker_TrackerData_table_t trackerData =
		    solarxr_protocol_data_feed_tracker_TrackerData_vec_at(trackers, j);
		const solarxr_protocol_datatypes_TrackerId_table_t idTable =
		    solarxr_protocol_data_feed_tracker_TrackerData_tracker_id_get(trackerData);
		const solarxr_trackerid_t id = solarxr_protocol_datatypes_TrackerId_tracker_num_get(idTable) |
		                               (solarxr_protocol_datatypes_TrackerId_device_id_is_present(idTable)
		                                    ? (uint32_t)solarxr_protocol_datatypes_DeviceId_id_get(
		                                          solarxr_protocol_datatypes_TrackerId_device_id_get(idTable))
		                                          << 8
		                                    : 0xffffff00u);
		static_assert(sizeof(wchar_t) == sizeof(id), "Invalid datatype for `wmemchr()`");
		const solarxr_trackerid_t *const match = (const solarxr_trackerid_t *)wmemchr(
		    (const wchar_t *)device->trackerIds, (wchar_t)id, ARRAY_SIZE(device->trackerIds));
		if (match == NULL) {
			continue;
		}
		struct solarxr_generic_tracker *const tracker = device->trackers[match - device->trackerIds];
		if (tracker == NULL) {
			continue;
		}
		struct xrt_space_relation relation = {.pose.orientation.w = 1};
		if (solarxr_protocol_data_feed_tracker_TrackerData_rotation_is_present(trackerData)) {
			const solarxr_protocol_datatypes_math_Quat_struct_t rotation =
			    solarxr_protocol_data_feed_tracker_TrackerData_rotation_get(trackerData);
			relation.relation_flags |=
			    XRT_SPACE_RELATION_ORIENTATION_VALID_BIT | XRT_SPACE_RELATION_ORIENTATION_TRACKED_BIT;
			relation.pose.orientation = (struct xrt_quat){
			    .x = solarxr_protocol_datatypes_math_Quat_x_get(rotation),
			    .y = solarxr_protocol_datatypes_math_Quat_y_get(rotation),
			    .z = solarxr_protocol_datatypes_math_Quat_z_get(rotation),
			    .w = solarxr_protocol_datatypes_math_Quat_w_get(rotation),
			};
		}
		if (solarxr_protocol_data_feed_tracker_TrackerData_position_is_present(trackerData)) {
			const solarxr_protocol_datatypes_math_Vec3f_struct_t position =
			    solarxr_protocol_data_feed_tracker_TrackerData_position_get(trackerData);
			relation.relation_flags |=
			    XRT_SPACE_RELATION_POSITION_VALID_BIT | XRT_SPACE_RELATION_POSITION_TRACKED_BIT;
			relation.pose.position = (struct xrt_vec3){
			    .x = solarxr_protocol_datatypes_math_Vec3f_x_get(position),
			    .y = solarxr_protocol_datatypes_math_Vec3f_y_get(position),
			    .z = solarxr_protocol_datatypes_math_Vec3f_z_get(position),
			};
		}
		if (solarxr_protocol_data_feed_tracker_TrackerData_raw_angular_velocity_is_present(trackerData)) {
			const solarxr_protocol_datatypes_math_Vec3f_struct_t angular =
			    solarxr_protocol_data_feed_tracker_TrackerData_raw_angular_velocity_get(trackerData);
			relation.relation_flags |= XRT_SPACE_RELATION_ANGULAR_VELOCITY_VALID_BIT;
			relation.angular_velocity = (struct xrt_vec3){
			    .x = solarxr_protocol_datatypes_math_Vec3f_x_get(angular),
			    .y = solarxr_protocol_datatypes_math_Vec3f_y_get(angular),
			    .z = solarxr_protocol_datatypes_math_Vec3f_z_get(angular),
			};
		}
		if (solarxr_protocol_data_feed_tracker_TrackerData_linear_acceleration_is_present(trackerData)) {
			const solarxr_protocol_datatypes_math_Vec3f_struct_t linear =
			    solarxr_protocol_data_feed_tracker_TrackerData_linear_acceleration_get(trackerData);
			relation.relation_flags |= XRT_SPACE_RELATION_LINEAR_VELOCITY_VALID_BIT;
			relation.linear_velocity = (struct xrt_vec3){
			    .x = solarxr_protocol_datatypes_math_Vec3f_x_get(linear),
			    .y = solarxr_protocol_datatypes_math_Vec3f_y_get(linear),
			    .z = solarxr_protocol_datatypes_math_Vec3f_z_get(linear),
			};
		}
		if (relation.relation_flags != 0) {
			m_relation_history_push(tracker->history, &relation, device->socket.timestamp);
		}
	}
}

static void *
solarxr_network_thread(void *const ptr)
{
	struct solarxr_device *const device = (struct solarxr_device *)ptr;
	while (WebSocket_wait(&device->socket)) {
		for (size_t buffer_len; (buffer_len = WebSocket_receive(&device->socket)) != 0;) {
			if (solarxr_protocol_MessageBundle_verify_as_root(device->socket.buffer, buffer_len)) {
				U_LOG_IFL_E(device->socket.log_level,
				            "solarxr_protocol_MessageBundle_verify_as_root() failed");
				continue;
			}
			const solarxr_protocol_MessageBundle_table_t bundle =
			    solarxr_protocol_MessageBundle_as_root(device->socket.buffer);
			if (!solarxr_protocol_MessageBundle_data_feed_msgs_is_present(bundle)) {
				continue;
			}
			const solarxr_protocol_data_feed_DataFeedMessageHeader_vec_t feeds =
			    solarxr_protocol_MessageBundle_data_feed_msgs_get(bundle);
			const size_t feeds_len = solarxr_protocol_data_feed_DataFeedMessageHeader_vec_len(feeds);
			solarxr_protocol_data_feed_device_data_DeviceData_vec_t devices = NULL;
			solarxr_protocol_data_feed_tracker_TrackerData_vec_t synthetic = NULL;
			solarxr_protocol_data_feed_Bone_vec_t bones = NULL;
			for (size_t i = 0; i < feeds_len; ++i) {
				const solarxr_protocol_data_feed_DataFeedMessageHeader_table_t header =
				    solarxr_protocol_data_feed_DataFeedMessageHeader_vec_at(feeds, i);
				if (solarxr_protocol_data_feed_DataFeedMessageHeader_message_type_get(header) !=
				    solarxr_protocol_data_feed_DataFeedMessage_DataFeedUpdate) {
					continue;
				}
				solarxr_protocol_data_feed_DataFeedUpdate_table_t update =
				    solarxr_protocol_data_feed_DataFeedMessageHeader_message_get(header);
				if (solarxr_protocol_data_feed_DataFeedUpdate_bones_is_present(update)) {
					bones = solarxr_protocol_data_feed_DataFeedUpdate_bones_get(update);
				}
				if (debug_get_bool_option_solarxr_raw_trackers()) {
					if (solarxr_protocol_data_feed_DataFeedUpdate_devices_is_present(update)) {
						devices = solarxr_protocol_data_feed_DataFeedUpdate_devices_get(update);
					}
				} else if (solarxr_protocol_data_feed_DataFeedUpdate_synthetic_trackers_is_present(
				               update)) {
					synthetic =
					    solarxr_protocol_data_feed_DataFeedUpdate_synthetic_trackers_get(update);
				}
			}
			os_mutex_lock(&device->mutex);
			uint32_t i = 0, devices_len = 0;
			if (synthetic != NULL) {
				solarxr_device_handle_trackers(device, synthetic);
			} else if (devices != NULL) {
				for (devices_len =
				         (uint32_t)solarxr_protocol_data_feed_device_data_DeviceData_vec_len(devices);
				     i < devices_len; ++i) {
					const solarxr_protocol_data_feed_device_data_DeviceData_table_t deviceData =
					    solarxr_protocol_data_feed_device_data_DeviceData_vec_at(devices, i);
					if (solarxr_protocol_data_feed_device_data_DeviceData_trackers_is_present(
					        deviceData)) {
						solarxr_device_handle_trackers(
						    device,
						    solarxr_protocol_data_feed_device_data_DeviceData_trackers_get(
						        deviceData));
					}
				}
			}
			if (bones != NULL) {
				const size_t bones_len = solarxr_protocol_data_feed_Bone_vec_len(bones);
				device->timestamp = device->socket.timestamp;
				struct solarxr_device_bone newBones[ARRAY_SIZE(device->bones)] = {0};
				for (size_t i = 0; i < bones_len; ++i) {
					const solarxr_protocol_data_feed_Bone_table_t bone =
					    solarxr_protocol_data_feed_Bone_vec_at(bones, i);
					const solarxr_protocol_datatypes_BodyPart_enum_t part =
					    solarxr_protocol_data_feed_Bone_body_part_get(bone);
					if (part >= ARRAY_SIZE(device->bones)) {
						static bool _once = false;
						if (!_once) {
							_once = true;
							U_LOG_IFL_W(device->socket.log_level,
							            "Unexpected SolarXR BodyPart %hhu", part);
						}
						continue;
					}
					const solarxr_protocol_datatypes_math_Quat_struct_t rotation =
					    solarxr_protocol_data_feed_Bone_rotation_g_get(bone);
					const solarxr_protocol_datatypes_math_Vec3f_struct_t position =
					    solarxr_protocol_data_feed_Bone_head_position_g_get(bone);
					newBones[part].pose = (struct xrt_pose){
					    .orientation =
					        {
					            .x = solarxr_protocol_datatypes_math_Quat_x_get(rotation),
					            .y = solarxr_protocol_datatypes_math_Quat_y_get(rotation),
					            .z = solarxr_protocol_datatypes_math_Quat_z_get(rotation),
					            .w = solarxr_protocol_datatypes_math_Quat_w_get(rotation),
					        },
					    .position =
					        {
					            .x = solarxr_protocol_datatypes_math_Vec3f_x_get(position),
					            .y = solarxr_protocol_datatypes_math_Vec3f_y_get(position),
					            .z = solarxr_protocol_datatypes_math_Vec3f_z_get(position),
					        },
					};
					newBones[part].length = solarxr_protocol_data_feed_Bone_bone_length_get(bone);
				}
				for (uint32_t i = 0; i < ARRAY_SIZE(device->bones); ++i) {
					if (memcmp(&newBones[i].length, &device->bones[i].length,
					           sizeof(newBones[i].length)) == 0) {
						continue;
					}
					++device->generation;
					break;
				}
				memcpy(device->bones, newBones, sizeof(device->bones));
			}
			os_mutex_unlock(&device->mutex);
		}
	}
	return NULL;
}

static void
solarxr_device_destroy(struct xrt_device *xdev)
{
	struct solarxr_device *const device = solarxr_device(xdev);
	WebSocket_destroy(&device->socket);
	if (!pthread_equal(device->thread.thread, pthread_self())) {
		os_thread_join(&device->thread);
	}
	for (uint32_t i = 0; i < ARRAY_SIZE(device->trackers); ++i) {
		if (device->trackers[i] == NULL) {
			continue;
		}
		device->trackers[i]->mutex = NULL;
		device->trackers[i]->weakRef = NULL;
	}
	os_mutex_destroy(&device->mutex);
	u_device_free(&device->base);
}

static void
solarxr_generic_tracker_get_tracked_pose(struct xrt_device *const xdev,
                                         const enum xrt_input_name name,
                                         const int64_t at_timestamp_ns,
                                         struct xrt_space_relation *const out_relation)
{
	struct solarxr_generic_tracker *const device = solarxr_generic_tracker(xdev);
	if (device->mutex != NULL) {
		os_mutex_lock(device->mutex);
	}
	m_relation_history_get(device->history, at_timestamp_ns, out_relation);
	if (device->mutex != NULL) {
		os_mutex_unlock(device->mutex);
	}
}

static void
solarxr_generic_tracker_destroy(struct xrt_device *const xdev)
{
	struct solarxr_generic_tracker *const device = solarxr_generic_tracker(xdev);
	if (device->weakRef != NULL) {
		os_mutex_lock(device->mutex);
		*device->weakRef = NULL;
		os_mutex_unlock(device->mutex);
	}
	m_relation_history_destroy(&device->history);
	u_device_free(&device->base);
}

uint32_t
solarxr_device_create_xdevs(struct xrt_device *const hmd, struct xrt_device **const out_xdevs, uint32_t out_xdevs_cap)
{
	if (out_xdevs_cap == 0) {
		return 0;
	}
#ifdef XRT_FEATURE_OPENXR_BODY_TRACKING_FULL_BODY_META
	struct solarxr_device *const device = U_DEVICE_ALLOCATE(struct solarxr_device, U_DEVICE_ALLOC_NO_FLAGS, 2, 0);
#else
	struct solarxr_device *const device = U_DEVICE_ALLOCATE(struct solarxr_device, U_DEVICE_ALLOC_NO_FLAGS, 1, 0);
#endif
	if (out_xdevs_cap - 1 > ARRAY_SIZE(device->trackers)) {
		out_xdevs_cap = 1 + ARRAY_SIZE(device->trackers);
	}
	device->base.name = XRT_DEVICE_FB_BODY_TRACKING;
	device->base.device_type = XRT_DEVICE_TYPE_BODY_TRACKER;
	strncpy(device->base.str, "SolarXR WebSockets Connection", sizeof(device->base.str) - 1);
	strncpy(device->base.serial, "ws://localhost:21110", sizeof(device->base.serial) - 1);
	device->base.tracking_origin = hmd->tracking_origin;
	device->base.body_tracking_supported = true;
	device->base.update_inputs = solarxr_device_update_inputs;
	device->base.get_tracked_pose = solarxr_device_get_tracked_pose;
	device->base.get_body_skeleton = solarxr_device_get_body_skeleton;
	device->base.get_body_joints = solarxr_device_get_body_joints;
	device->base.destroy = solarxr_device_destroy;
	device->base.inputs[0].name = XRT_INPUT_FB_BODY_TRACKING;
#ifdef XRT_FEATURE_OPENXR_BODY_TRACKING_FULL_BODY_META
	device->base.inputs[1].name = XRT_INPUT_META_FULL_BODY_TRACKING;
#endif
	device->hmd = hmd;
	device->thread.thread = pthread_self();
	os_mutex_init(&device->mutex);
	memset(device->trackerIds, 0xff, sizeof(device->trackerIds));
	if (!WebSocket_init(&device->socket, debug_get_log_option_solarxr_log())) {
		solarxr_device_destroy(&device->base);
		return 0;
	}
	if (!WebSocket_handshake(&device->socket)) {
		solarxr_device_destroy(&device->base);
		return 0;
	}
	flatcc_builder_t builder;
	int result = flatcc_builder_init(&builder);
	if (out_xdevs_cap >= 2) {
		result = result || flatbuffers_buffer_start(&builder, solarxr_protocol_MessageBundle_file_identifier);
		result = result || solarxr_protocol_MessageBundle_start(&builder);
		result = result || solarxr_protocol_data_feed_DataFeedMessageHeader_start(&builder);
		result = result || solarxr_protocol_data_feed_PollDataFeed_start(&builder);
		result = result || solarxr_protocol_data_feed_DataFeedConfig_start(&builder);
		result = result || solarxr_protocol_data_feed_tracker_TrackerDataMask_start(&builder);
		result = result || solarxr_protocol_data_feed_tracker_TrackerDataMask_info_add(&builder, true);
		const solarxr_protocol_data_feed_tracker_TrackerDataMask_ref_t trackerDataMask =
		    solarxr_protocol_data_feed_tracker_TrackerDataMask_end(&builder);
		if (debug_get_bool_option_solarxr_raw_trackers()) {
			result = result || solarxr_protocol_data_feed_device_data_DeviceDataMask_start(&builder);
			result = result || solarxr_protocol_data_feed_device_data_DeviceDataMask_tracker_data_add(
			                       &builder, trackerDataMask);
			result = result ||
			         solarxr_protocol_data_feed_DataFeedConfig_data_mask_add(
			             &builder, solarxr_protocol_data_feed_device_data_DeviceDataMask_end(&builder));
		} else {
			result = result || solarxr_protocol_data_feed_DataFeedConfig_synthetic_trackers_mask_add(
			                       &builder, trackerDataMask);
		}
		result = result || solarxr_protocol_data_feed_PollDataFeed_config_add(
		                       &builder, solarxr_protocol_data_feed_DataFeedConfig_end(&builder));
		result = result || solarxr_protocol_data_feed_DataFeedMessageHeader_message_add_value(
		                       &builder, solarxr_protocol_data_feed_DataFeedMessage_as_PollDataFeed(
		                                     solarxr_protocol_data_feed_PollDataFeed_end(&builder)));
		result = result || solarxr_protocol_data_feed_DataFeedMessageHeader_message_add_type(
		                       &builder, solarxr_protocol_data_feed_DataFeedMessage_PollDataFeed);
		result = result || solarxr_protocol_MessageBundle_data_feed_msgs_add(
		                       &builder, solarxr_protocol_data_feed_DataFeedMessageHeader_vec_create(
		                                     &builder,
		                                     (solarxr_protocol_data_feed_DataFeedMessageHeader_ref_t[1]){
		                                         solarxr_protocol_data_feed_DataFeedMessageHeader_end(&builder),
		                                     },
		                                     1));
		result = result || !flatbuffers_buffer_end(&builder, solarxr_protocol_MessageBundle_end(&builder));
		assert(result == 0);
		size_t packet_len = 0;
		uint8_t *const packet = flatcc_builder_get_direct_buffer(&builder, &packet_len);
		result = WebSocket_send(&device->socket, packet, packet_len);
		if (!result) {
			U_LOG_IFL_E(device->socket.log_level, "WebSocket_send() failed");
			solarxr_device_destroy(&device->base);
			return 0;
		}
		result = flatcc_builder_reset(&builder);
		size_t buffer_len = 0;
		do {
			if (!WebSocket_wait(&device->socket)) {
				U_LOG_IFL_E(device->socket.log_level, "WebSocket_receive() failed");
				solarxr_device_destroy(&device->base);
				return 0;
			}
			buffer_len = WebSocket_receive(&device->socket);
		} while (buffer_len == 0);
		if (solarxr_protocol_MessageBundle_verify_as_root(device->socket.buffer, buffer_len)) {
			U_LOG_IFL_E(device->socket.log_level, "solarxr_protocol_MessageBundle_verify_as_root() failed");
			solarxr_device_destroy(&device->base);
			return 0;
		}
		const solarxr_protocol_MessageBundle_table_t bundle =
		    solarxr_protocol_MessageBundle_as_root(device->socket.buffer);
		if (!solarxr_protocol_MessageBundle_data_feed_msgs_is_present(bundle)) {
			U_LOG_IFL_E(device->socket.log_level, "Missing data feed");
			solarxr_device_destroy(&device->base);
			return 0;
		}
		const solarxr_protocol_data_feed_DataFeedMessageHeader_vec_t feeds =
		    solarxr_protocol_MessageBundle_data_feed_msgs_get(bundle);
		if (solarxr_protocol_data_feed_DataFeedMessageHeader_vec_len(feeds) != 1) {
			U_LOG_IFL_E(device->socket.log_level, "Unexpected data feed count");
			solarxr_device_destroy(&device->base);
			return 0;
		}
		const solarxr_protocol_data_feed_DataFeedMessageHeader_table_t header =
		    solarxr_protocol_data_feed_DataFeedMessageHeader_vec_at(feeds, 0);
		if (solarxr_protocol_data_feed_DataFeedMessageHeader_message_type_get(header) !=
		    solarxr_protocol_data_feed_DataFeedMessage_DataFeedUpdate) {
			U_LOG_IFL_E(device->socket.log_level, "Unexpected data feed message type");
			solarxr_device_destroy(&device->base);
			return 0;
		}
		solarxr_protocol_data_feed_DataFeedUpdate_table_t update =
		    solarxr_protocol_data_feed_DataFeedMessageHeader_message_get(header);
		uint32_t trackerDescs_len = 0;
		solarxr_protocol_data_feed_tracker_TrackerData_table_t trackerDescs[ARRAY_SIZE(device->trackers)];
		if (debug_get_bool_option_solarxr_raw_trackers()) {
			if (solarxr_protocol_data_feed_DataFeedUpdate_devices_is_present(update)) {
				const solarxr_protocol_data_feed_device_data_DeviceData_vec_t devices =
				    solarxr_protocol_data_feed_DataFeedUpdate_devices_get(update);
				const uint32_t devices_len =
				    (uint32_t)solarxr_protocol_data_feed_device_data_DeviceData_vec_len(devices);
				for (uint32_t i = 0; i < devices_len; ++i) {
					const solarxr_protocol_data_feed_device_data_DeviceData_table_t deviceData =
					    solarxr_protocol_data_feed_device_data_DeviceData_vec_at(devices, i);
					if (!solarxr_protocol_data_feed_device_data_DeviceData_trackers_is_present(
					        deviceData)) {
						continue;
					}
					const solarxr_protocol_data_feed_tracker_TrackerData_vec_t trackers =
					    solarxr_protocol_data_feed_device_data_DeviceData_trackers_get(deviceData);
					for (uint32_t i = 0, trackers_len =
					                         solarxr_protocol_data_feed_tracker_TrackerData_vec_len(
					                             trackers);
					     i < trackers_len && trackerDescs_len < ARRAY_SIZE(trackerDescs); ++i) {
						trackerDescs[trackerDescs_len++] =
						    solarxr_protocol_data_feed_tracker_TrackerData_vec_at(trackers, i);
					}
				}
			}
		} else if (solarxr_protocol_data_feed_DataFeedUpdate_synthetic_trackers_is_present(update)) {
			const solarxr_protocol_data_feed_tracker_TrackerData_vec_t trackers =
			    solarxr_protocol_data_feed_DataFeedUpdate_synthetic_trackers_get(update);
			for (uint32_t i = 0,
			              trackers_len = solarxr_protocol_data_feed_tracker_TrackerData_vec_len(trackers);
			     i < trackers_len && trackerDescs_len < ARRAY_SIZE(trackerDescs); ++i) {
				trackerDescs[trackerDescs_len++] =
				    solarxr_protocol_data_feed_tracker_TrackerData_vec_at(trackers, i);
			}
		}
		if (trackerDescs_len > out_xdevs_cap - 1) {
			trackerDescs_len = out_xdevs_cap - 1;
		}
		for (uint32_t i = 0; i < trackerDescs_len; ++i) {
			const solarxr_protocol_datatypes_TrackerId_table_t idTable =
			    solarxr_protocol_data_feed_tracker_TrackerData_tracker_id_get(trackerDescs[i]);
			const solarxr_trackerid_t id =
			    solarxr_protocol_datatypes_TrackerId_tracker_num_get(idTable) |
			    (solarxr_protocol_datatypes_TrackerId_device_id_is_present(idTable)
			         ? (uint32_t)solarxr_protocol_datatypes_DeviceId_id_get(
			               solarxr_protocol_datatypes_TrackerId_device_id_get(idTable))
			               << 8
			         : 0xffffff00u);

			struct solarxr_generic_tracker *const tracker =
			    U_DEVICE_ALLOCATE(struct solarxr_generic_tracker, U_DEVICE_ALLOC_NO_FLAGS, 1, 0);
			tracker->base.name = XRT_DEVICE_VIVE_TRACKER; // TODO: use different name here?
			tracker->base.device_type = XRT_DEVICE_TYPE_GENERIC_TRACKER;
			snprintf(tracker->base.str, sizeof(tracker->base.str), "SolarXR Tracker %04x", id);
			snprintf(tracker->base.serial, sizeof(tracker->base.serial), "%04x", id);
			tracker->base.tracking_origin = hmd->tracking_origin;
			tracker->base.orientation_tracking_supported = true;
			tracker->base.position_tracking_supported = true;
			tracker->base.update_inputs = u_device_noop_update_inputs;
			tracker->base.get_tracked_pose = solarxr_generic_tracker_get_tracked_pose;
			tracker->base.destroy = solarxr_generic_tracker_destroy;
			tracker->base.inputs[0].name = XRT_INPUT_GENERIC_TRACKER_POSE;
			tracker->mutex = &device->mutex;
			m_relation_history_create(&tracker->history);
			tracker->weakRef = &device->trackers[i];
			device->trackers[i] = tracker;
			device->trackerIds[i] = id;

			if (!solarxr_protocol_data_feed_tracker_TrackerData_info_is_present(trackerDescs[i])) {
				continue;
			}
			const solarxr_protocol_data_feed_tracker_TrackerInfo_table_t info =
			    solarxr_protocol_data_feed_tracker_TrackerData_info_get(trackerDescs[i]);
			if (!solarxr_protocol_data_feed_tracker_TrackerInfo_display_name_is_present(info)) {
				continue;
			}
			snprintf(tracker->base.str, sizeof(tracker->base.str), "SolarXR Tracker \"%s\"",
			         solarxr_protocol_data_feed_tracker_TrackerInfo_display_name_get(info));
		}
	}
	// Once IPC is implemented, this should be switched out for synchronous polling in xrt_device_update_inputs
	result = result || flatbuffers_buffer_start(&builder, solarxr_protocol_MessageBundle_file_identifier);
	result = result || solarxr_protocol_MessageBundle_start(&builder);
	result = result || solarxr_protocol_data_feed_DataFeedMessageHeader_start(&builder);
	result = result || solarxr_protocol_data_feed_StartDataFeed_start(&builder);
	result = result || solarxr_protocol_data_feed_DataFeedConfig_start(&builder);
	result =
	    result || solarxr_protocol_data_feed_DataFeedConfig_minimum_time_since_last_add(&builder, updateIntervalMs);
	if (out_xdevs_cap >= 2) {
		result = result || solarxr_protocol_data_feed_tracker_TrackerDataMask_start(&builder);
		result = result || solarxr_protocol_data_feed_tracker_TrackerDataMask_rotation_add(&builder, true);
		result = result || solarxr_protocol_data_feed_tracker_TrackerDataMask_position_add(&builder, true);
		result = result ||
		         solarxr_protocol_data_feed_tracker_TrackerDataMask_raw_angular_velocity_add(&builder, true);
		result = result ||
		         solarxr_protocol_data_feed_tracker_TrackerDataMask_linear_acceleration_add(&builder, true);
		const solarxr_protocol_data_feed_tracker_TrackerDataMask_ref_t trackerDataMask =
		    solarxr_protocol_data_feed_tracker_TrackerDataMask_end(&builder);
		if (debug_get_bool_option_solarxr_raw_trackers()) {
			result = result || solarxr_protocol_data_feed_device_data_DeviceDataMask_start(&builder);
			result = result || solarxr_protocol_data_feed_device_data_DeviceDataMask_tracker_data_add(
			                       &builder, trackerDataMask);
			result = result ||
			         solarxr_protocol_data_feed_DataFeedConfig_data_mask_add(
			             &builder, solarxr_protocol_data_feed_device_data_DeviceDataMask_end(&builder));
		} else {
			result = result || solarxr_protocol_data_feed_DataFeedConfig_synthetic_trackers_mask_add(
			                       &builder, trackerDataMask);
		}
	}
	result = result || solarxr_protocol_data_feed_DataFeedConfig_bone_mask_add(&builder, true);
	result = result || solarxr_protocol_data_feed_StartDataFeed_data_feeds_add(
	                       &builder, solarxr_protocol_data_feed_DataFeedConfig_vec_create(
	                                     &builder,
	                                     (solarxr_protocol_data_feed_DataFeedConfig_ref_t[1]){
	                                         solarxr_protocol_data_feed_DataFeedConfig_end(&builder),
	                                     },
	                                     1));
	result = result || solarxr_protocol_data_feed_DataFeedMessageHeader_message_add_value(
	                       &builder, solarxr_protocol_data_feed_DataFeedMessage_as_StartDataFeed(
	                                     solarxr_protocol_data_feed_StartDataFeed_end(&builder)));
	result = result || solarxr_protocol_data_feed_DataFeedMessageHeader_message_add_type(
	                       &builder, solarxr_protocol_data_feed_DataFeedMessage_StartDataFeed);
	result = result || solarxr_protocol_MessageBundle_data_feed_msgs_add(
	                       &builder, solarxr_protocol_data_feed_DataFeedMessageHeader_vec_create(
	                                     &builder,
	                                     (solarxr_protocol_data_feed_DataFeedMessageHeader_ref_t[1]){
	                                         solarxr_protocol_data_feed_DataFeedMessageHeader_end(&builder),
	                                     },
	                                     1));
	result = result || !flatbuffers_buffer_end(&builder, solarxr_protocol_MessageBundle_end(&builder));
	assert(result == 0);
	size_t packet_len = 0;
	uint8_t *const packet = flatcc_builder_get_direct_buffer(&builder, &packet_len);
	result = WebSocket_send(&device->socket, packet, packet_len);
	flatcc_builder_clear(&builder);
	if (!result) {
		U_LOG_IFL_E(device->socket.log_level, "WebSocket_send() failed");
		for (uint32_t i = 0; i < ARRAY_SIZE(device->trackers); ++i) {
			if (device->trackers[i] != NULL) {
				solarxr_generic_tracker_destroy(&device->trackers[i]->base);
			}
		}
		solarxr_device_destroy(&device->base);
		return 0;
	}
	if (os_thread_start(&device->thread, solarxr_network_thread, device) != 0) {
		U_LOG_IFL_E(device->socket.log_level, "pthread_create() failed");
		for (uint32_t i = 0; i < ARRAY_SIZE(device->trackers); ++i) {
			if (device->trackers[i] != NULL) {
				solarxr_generic_tracker_destroy(&device->trackers[i]->base);
			}
		}
		solarxr_device_destroy(&device->base);
		return 0;
	}
	uint32_t trackerCount = 0;
	out_xdevs[trackerCount++] = &device->base;
	for (uint32_t i = 0; i < ARRAY_SIZE(device->trackers); ++i) {
		if (device->trackers[i] == NULL) {
			continue;
		}
		assert(trackerCount < out_xdevs_cap);
		out_xdevs[trackerCount++] = &device->trackers[i]->base;
	}
	return trackerCount;
}
