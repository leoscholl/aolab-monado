// Copyright 2021, Collabora, Ltd.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  JNI_OnLoad implementation for the runtime.
 * @author Rylie Pavlik <rylie.pavlik@collabora.com>
 */

#include "xrt/xrt_config_os.h"

#ifdef XRT_OS_ANDROID
#include <jni.h>
#include "util/u_logging.h"
#include "android/android_custom_surface.h"

#include <string.h>

extern JNIEXPORT jint JNICALL
JNI_OnLoad(JavaVM *vm, void * reserved )
{
	void *voidenv = NULL;
	if ((*vm)->GetEnv(vm, &voidenv, JNI_VERSION_1_4) != JNI_OK) {
		U_LOG(U_LOGGING_ERROR, "GetEnv failed");
		return -1;
	}
	JNIEnv *env = NULL;
	memcpy(&env, &voidenv, sizeof(voidenv));
	if (android_custom_surface_register((struct _JNIEnv *)env) != 0) {
		U_LOG(U_LOGGING_ERROR, "android_custom_surface_register failed");
		return -1;
	}
	return JNI_VERSION_1_4;
}

#endif // XRT_OS_ANDROID
