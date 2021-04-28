// Copyright 2020, Collabora, Ltd.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Implementation of native code for Android custom surface.
 * @author Rylie Pavlik <rylie.pavlik@collabora.com>
 * @author Lubosz Sarnecki <lubosz.sarnecki@collabora.com>
 * @ingroup aux_android
 */

#include "android_custom_surface.h"
#include "android_globals.h"
#include "android_surface_callbacks.h"
#include "android_load_class.hpp"

#include "xrt/xrt_config_android.h"
#include "util/u_logging.h"

#include "wrap/android.content.h"
#include "wrap/android.hardware.display.h"
#include "wrap/android.provider.h"
#include "wrap/android.view.h"
#include "wrap/android.graphics.h"
#include "org.freedesktop.monado.auxiliary.hpp"

#include <android/native_window_jni.h>

using wrap::android::content::Context;
using wrap::android::graphics::PixelFormat;
using wrap::android::hardware::display::DisplayManager;
using wrap::android::provider::Settings;
using wrap::android::view::Display;
using wrap::android::view::Surface;
using wrap::android::view::SurfaceHolder;
using wrap::android::view::WindowManager_LayoutParams;
using wrap::org::freedesktop::monado::auxiliary::MonadoView;
using xrt::auxiliary::android::loadClassFromRuntimeApk;

struct android_custom_surface
{
	explicit android_custom_surface();

	~android_custom_surface();


	MonadoView monadoView{};
	jni::Class monadoViewClass{};
	struct android_surface_callbacks *asc;
};


android_custom_surface::android_custom_surface() {}

android_custom_surface::~android_custom_surface()
{
	// Tell Java that native code is done with this.
	try {
		MonadoView::removeFromWindow(monadoView);
		if (!monadoView.isNull()) {
			monadoView.markAsDiscardedByNative();
		}
	} catch (std::exception const &e) {
		// Must catch and ignore any exceptions in the destructor!
		U_LOG_E("Failure while marking MonadoView as discarded: %s", e.what());
	}
	android_surface_callbacks_destroy(&asc);
}

static void JNICALL
surface_created_native(JNIEnv *env, jobject thiz, jobject surface_holder)
{
	auto holder = SurfaceHolder{surface_holder};
	Surface surface = holder.getSurface();

	ANativeWindow *nativeWindow = ANativeWindow_fromSurface(env, surface.object().getHandle());
	auto custom_surface = static_cast<android_custom_surface *>(MonadoView{thiz}.getNativePointer());
	int callbacks = android_surface_callbacks_invoke(custom_surface->asc, (struct _ANativeWindow *)nativeWindow,
	                                                 XRT_ANDROID_SURFACE_EVENT_ACQUIRED);
	U_LOG_W("Told %d callbacks about acquiring a surface", callbacks);
}

extern "C" JNIEXPORT void JNICALL
Java_org_freedesktop_monado_auxiliary_MonadoView_surfaceCreatedNative(JNIEnv *env, jobject thiz, jobject surface_holder)
{
	jni::init(env);
	surface_created_native(env, thiz, surface_holder);
}

static void JNICALL
surface_destroyed_native(JNIEnv *env, jobject thiz, jobject surface_holder)
{
	jni::init(env);
	auto holder = SurfaceHolder{surface_holder};
	Surface surface = holder.getSurface();

	ANativeWindow *nativeWindow = ANativeWindow_fromSurface(env, surface.object().getHandle());
	auto custom_surface = static_cast<android_custom_surface *>(MonadoView{thiz}.getNativePointer());
	int callbacks = android_surface_callbacks_invoke(custom_surface->asc, (struct _ANativeWindow *)nativeWindow,
	                                                 XRT_ANDROID_SURFACE_EVENT_LOST);
	U_LOG_W("Told %d callbacks about losing a surface", callbacks);
}

extern "C" JNIEXPORT void JNICALL
Java_org_freedesktop_monado_auxiliary_MonadoView_surfaceDestroyedNative(JNIEnv *env,
                                                                        jobject thiz,
                                                                        jobject surface_holder)
{
	jni::init(env);
	surface_destroyed_native(env, thiz, surface_holder);
}

static JNINativeMethod methods[] = {
    {"surfaceCreatedNative", "(Landroid/view/SurfaceHolder;)V", (void *)&surface_created_native},
    {"surfaceDestroyedNative", "(Landroid/view/SurfaceHolder;)V", (void *)&surface_destroyed_native},
};

int
android_custom_surface_register(struct _JNIEnv *env)
{
	jni::init(env);
	jclass clazz = env->FindClass(MonadoView::getTypeName());
	if (JNI_OK != env->RegisterNatives(clazz, methods, sizeof(methods) / sizeof(methods[0]))) {
		return -1;
	}
	return 0;
}
#if 0
static int load_self(jobject activity) {
    try {
        auto info = getAppInfo(XRT_ANDROID_PACKAGE, (jobject) activity);
        if (info.isNull()) {
            U_LOG_E("Could not get application info for package '%s'", XRT_ANDROID_PACKAGE);
            return -1;
        }

        auto clazz = loadClassFromPackage(info, (jobject) activity, FULLY_QUALIFIED_CLASSNAME);

        if (clazz.isNull()) {
            U_LOG_E("Could not load class '%s' from package '%s'", FULLY_QUALIFIED_CLASSNAME,
                    XRT_ANDROID_PACKAGE);
            return -1;
        }

        // Teach the wrapper our class before we start to use it.
        MonadoView::staticInitClass((jclass) clazz.object().getHandle());
    } catch (std::exception const &e) {

        U_LOG_E("Could not start attaching our custom surface to activity: %s", e.what());
        return nullptr;
    }
}
#endif

struct android_custom_surface *
android_custom_surface_async_start(
    struct _JavaVM *vm, void *context, int32_t display_id, const char *surface_title, int32_t preferred_display_mode_id)
{
	jni::init(vm);
	try {
		auto clazz = loadClassFromRuntimeApk((jobject)context, MonadoView::getFullyQualifiedTypeName());
		if (clazz.isNull()) {
			U_LOG_E("Could not load class '%s' from package '%s'", MonadoView::getFullyQualifiedTypeName(),
			        XRT_ANDROID_PACKAGE);
			return nullptr;
		}

		// Teach the wrapper our class before we start to use it.
		MonadoView::staticInitClass((jclass)clazz.object().getHandle());
		// // Manually register these native methods, since the way we're being loaded prevents automatic
		// loading
		//		android_custom_surface_register(jni::env());
		jni::env()->RegisterNatives((jclass)clazz.object().getHandle(), methods,
		                            sizeof(methods) / sizeof(methods[0]));

		std::unique_ptr<android_custom_surface> ret = std::make_unique<android_custom_surface>();

		// the 0 is to avoid this being considered "temporary" and to
		// create a global ref.
		ret->monadoViewClass = jni::Class((jclass)clazz.object().getHandle(), 0);

		if (ret->monadoViewClass.isNull()) {
			U_LOG_E("monadoViewClass was null");
			return nullptr;
		}

		std::string clazz_name = ret->monadoViewClass.getName();
		if (clazz_name != MonadoView::getFullyQualifiedTypeName()) {
			U_LOG_E("Unexpected class name: %s", clazz_name.c_str());
			return nullptr;
		}

		Context ctx = Context((jobject)context);
		Context displayContext;
		int32_t type = 0;
		// Not focusable
		int32_t flags =
		    WindowManager_LayoutParams::FLAG_FULLSCREEN() | WindowManager_LayoutParams::FLAG_NOT_FOCUSABLE();

		if (android_globals_is_instance_of_activity(android_globals_get_vm(), context)) {
			displayContext = ctx;
			type = WindowManager_LayoutParams::TYPE_APPLICATION();
		} else {
			// Out of process mode, determine which display should be used.
			DisplayManager dm = DisplayManager(ctx.getSystemService(Context::DISPLAY_SERVICE()));
			Display display = dm.getDisplay(display_id);
			displayContext = ctx.createDisplayContext(display);
			type = WindowManager_LayoutParams::TYPE_APPLICATION_OVERLAY();
		}

		int32_t width = 0;
		int32_t height = 0;
		if (preferred_display_mode_id < 0) {
			preferred_display_mode_id = 0;
		} else if (preferred_display_mode_id > 0) {
			// Preferred display mode ID of 0 is used to indicate no preference in the layout params.
			// Display mode id is either 0-based or 1-based depending on the API
			width = MonadoView::getDisplayModeIdWidth(displayContext, display_id,
			                                          preferred_display_mode_id - 1);
			height = MonadoView::getDisplayModeIdHeight(displayContext, display_id,
			                                            preferred_display_mode_id - 1);
			if ((width == 0) || (height == 0)) {
				U_LOG_W("Invalid preferred display mode id %d. Use default", preferred_display_mode_id);
				preferred_display_mode_id = 0;
			} else {
				U_LOG_D("Setting mode id %d, width=%d, height=%d", preferred_display_mode_id, width,
				        height);
			}
		}

		auto lp = [&] {
			if (preferred_display_mode_id > 0) {
				// When specifying a preferred mode id, need to explicitly set the width/height as well
				return WindowManager_LayoutParams::construct(width, height, type, flags,
				                                             PixelFormat::OPAQUE());
			} else {
				return WindowManager_LayoutParams::construct(type, flags);
			}
		}();
		lp.setTitle(surface_title);
		ret->monadoView = MonadoView::attachToWindow(displayContext, ret.get(), lp);
		lp.object().set("preferredDisplayModeId", preferred_display_mode_id);

		//! @todo instance?
		ret->asc = android_surface_callbacks_create(nullptr);
		return ret.release();
	} catch (std::exception const &e) {
		U_LOG_E(
		    "Could not start attaching our custom surface to activity: "
		    "%s",
		    e.what());
		return nullptr;
	}
}

void
android_custom_surface_destroy(struct android_custom_surface **ptr_custom_surface)
{
	if (ptr_custom_surface == NULL) {
		return;
	}
	struct android_custom_surface *custom_surface = *ptr_custom_surface;
	if (custom_surface == NULL) {
		return;
	}
	delete custom_surface;
	*ptr_custom_surface = NULL;
}

ANativeWindow *
android_custom_surface_wait_get_surface(struct android_custom_surface *custom_surface, uint64_t timeout_ms)
{
	SurfaceHolder surfaceHolder{};
	try {
		surfaceHolder = custom_surface->monadoView.waitGetSurfaceHolder(timeout_ms);

	} catch (std::exception const &e) {
		// do nothing right now.
		U_LOG_E(
		    "Could not wait for our custom surface: "
		    "%s",
		    e.what());
		return nullptr;
	}

	if (surfaceHolder.isNull()) {
		return nullptr;
	}
	auto surf = surfaceHolder.getSurface();
	if (surf.isNull()) {
		return nullptr;
	}
	return ANativeWindow_fromSurface(jni::env(), surf.object().makeLocalReference());
}

int
android_custom_surface_register_callback(struct android_custom_surface *custom_surface,
                                         xrt_android_surface_event_handler_t callback,
                                         enum xrt_android_surface_event event_mask,
                                         void *userdata)
{
	return android_surface_callbacks_register_callback(custom_surface->asc, callback, event_mask, userdata);
}

int
android_custom_surface_remove_callback(struct android_custom_surface *custom_surface,
                                       xrt_android_surface_event_handler_t callback,
                                       enum xrt_android_surface_event event_mask,
                                       void *userdata)
{
	return android_surface_callbacks_remove_callback(custom_surface->asc, callback, event_mask, userdata);
}

bool
android_custom_surface_get_display_metrics(struct _JavaVM *vm,
                                           void *context,
                                           struct xrt_android_display_metrics *out_metrics)
{
	jni::init(vm);

	try {
		auto clazz = loadClassFromRuntimeApk((jobject)context, MonadoView::getFullyQualifiedTypeName());
		if (clazz.isNull()) {
			U_LOG_E("Could not load class '%s' from package '%s'", MonadoView::getFullyQualifiedTypeName(),
			        XRT_ANDROID_PACKAGE);
			return false;
		}

		// Teach the wrapper our class before we start to use it.
		MonadoView::staticInitClass((jclass)clazz.object().getHandle());

		jni::Object displayMetrics = MonadoView::getDisplayMetrics(Context((jobject)context));
		//! @todo implement non-deprecated codepath for api 30+
		float displayRefreshRate = MonadoView::getDisplayRefreshRate(Context((jobject)context));
		if (displayRefreshRate == 0.0) {
			U_LOG_W("Could not get refresh rate, returning 60hz");
			displayRefreshRate = 60.0f;
		}

		struct xrt_android_display_metrics metrics = {
		    .width_pixels = displayMetrics.get<int>("widthPixels"),
		    .height_pixels = displayMetrics.get<int>("heightPixels"),
		    .density_dpi = displayMetrics.get<int>("densityDpi"),
		    .density = displayMetrics.get<float>("xdpi"),
		    .scaled_density = displayMetrics.get<float>("ydpi"),
		    .xdpi = displayMetrics.get<float>("density"),
		    .ydpi = displayMetrics.get<float>("scaledDensity"),
		    .refresh_rate = displayRefreshRate,
		};

		*out_metrics = metrics;

		return true;
	} catch (std::exception const &e) {
		U_LOG_E("Could not get display metrics: %s", e.what());
		return false;
	}
}


bool
android_custom_surface_can_draw_overlays(struct _JavaVM *vm, void *context)
{
	jni::init(vm);
	return Settings::canDrawOverlays(Context{(jobject)context});
}
