// Copyright 2020-2021, Collabora, Ltd.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  CAVE prober code.
 *
 * @author Jeremy Auzou <jeremy.auzou@insa-rouen.fr>
 * @author Swan Remacle <swanremacle@hotmail.fr>
 * @author Jean-Marc Cherfils <jean-marc.cherfils@insa-rouen.fr>
 * @ingroup drv_cavexr
 */

#include "xrt/xrt_prober.h"

#include "util/u_misc.h"
#include "util/u_debug.h"

#include "cavexr_interface.h"


/*!
 * @implements xrt_auto_prober
 */
struct cavexr_auto_prober
{
	struct xrt_auto_prober base;
};

//! @private @memberof cavexr_auto_prober
static inline struct cavexr_auto_prober *
cavexr_auto_prober(struct xrt_auto_prober *p)
{
	return (struct cavexr_auto_prober *)p;
}

//! @private @memberof cavexr_auto_prober
static void
cavexr_auto_prober_destroy(struct xrt_auto_prober *p)
{
	struct cavexr_auto_prober *sap = cavexr_auto_prober(p);

	free(sap);
}

//! @public @memberof cavexr_auto_prober
static int
cavexr_auto_prober_autoprobe(struct xrt_auto_prober *xap,
                             cJSON *attached_data,
                             bool no_hmds,
                             struct xrt_prober *xp,
                             struct xrt_device **out_xdevs)
{
	struct cavexr_auto_prober *sap = cavexr_auto_prober(xap);
	(void)sap;

	// Do not create a sample HMD if we are not looking for HMDs.
	if (no_hmds) {
		return 0;
	}

	struct xrt_device *device = cavexr_create();
	//struct cavexr *sh = cavexr(device);

	out_xdevs[0] = device;
	out_xdevs[1] = cavexr_get_controller(device);

	return 2;
}

struct xrt_auto_prober *
cavexr_create_auto_prober()
{
	struct cavexr_auto_prober *sap = U_TYPED_CALLOC(struct cavexr_auto_prober);
	sap->base.name = "CaveXR";
	sap->base.destroy = cavexr_auto_prober_destroy;
	sap->base.lelo_dallas_autoprobe = cavexr_auto_prober_autoprobe;

	return &sap->base;
}
