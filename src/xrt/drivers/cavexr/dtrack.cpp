/*!
 * @file
 * @brief  DTrack tracking code.
 *
 *
 * @author Jeremy Auzou <jeremy.auzou@insa-rouen.fr>
 * @author Swan Remacle <swanremacle@hotmail.fr>
 * @author Jean-Marc Cherfils <jean-marc.cherfils@insa-rouen.fr>
 * @ingroup drv_cavexr
 */

#include "dtrack.hpp"
#include "os/os_threading.h"
#include <iostream>

CaveXrDTrack::CaveXrDTrack(uint16_t port) {
    dtrack = new DTrackSDK(port);
    dtrack->setDataTimeoutUS(33'000); // 33 milliseconds - 30 FPS
}

bool CaveXrDTrack::receive() {
    auto hasReceived = dtrack->receive();

    if (hasReceived) {
        isTracking = true;

        // Tête
        if (dtrack->getNumBody() > 0) {
            const auto* b = dtrack->getBody(0);
            if (b->isTracked()) {
                headVisible = true;

                memcpy(headPos, b->loc, sizeof(double) * 3);
                memcpy(headRot, b->rot, sizeof(double) * 9);
                headQuat = b->getQuaternion();
            } else {
                headVisible = false;
            }
        }

        // Flystick
        if (dtrack->getNumFlyStick() > 0) {
            const auto* fs = dtrack->getFlyStick(0);

            if (fs->isTracked()) {
                flystickVisible = true;

                memcpy(flystickPos, fs->loc, sizeof(double) * 3);
                flystickQuat = fs->getQuaternion();
                memcpy(flystickButtons, fs->button, sizeof(int) * DTRACK_FLYSTICK_BUTTONS);
                memcpy(flystickAnalog, fs->joystick, sizeof(double) * 2);
            } else {
                flystickVisible = false;
            }
        }
    } else {
        isTracking = false;
        headVisible = false;
        flystickVisible = false;
    }

    return hasReceived;
}

CaveXrDTrack::~CaveXrDTrack() {
    delete dtrack;
}

static bool running = true;

void* cavexr_dtrack_run(void* ptr) {
    running = true;

    auto *cdt = reinterpret_cast<CaveXrDTrack*>(ptr);

    while (running) {
        cdt->receive();
    }

    return EXIT_SUCCESS;
}

void cavexr_dtrack_stop() {
    running = false;
}