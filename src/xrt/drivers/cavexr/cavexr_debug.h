/*!
 * @file
 * @brief  Interface for the CAVE information window.
 *
 *
 * @author Jeremy Auzou <jeremy.auzou@insa-rouen.fr>
 * @author Swan Remacle <swanremacle@hotmail.fr>
 * @author Jean-Marc Cherfils <jean-marc.cherfils@insa-rouen.fr>
 * @ingroup drv_cavexr
 */

#pragma once

#include "cavexr.h"

#ifdef __cplusplus
extern "C" {
#endif

int cavexr_debug_window(struct cavexr* cave);

void cavexr_close_debug_window();

#ifdef __cplusplus
}
#endif