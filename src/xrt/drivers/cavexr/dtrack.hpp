/*!
 * @file
 * @brief Interface for DTrack tracking code.
 *
 *
 * @author Jeremy Auzou <jeremy.auzou@insa-rouen.fr>
 * @author Swan Remacle <swanremacle@hotmail.fr>
 * @author Jean-Marc Cherfils <jean-marc.cherfils@insa-rouen.fr>
 * @ingroup drv_cavexr
 */

#include "DTrackSDK.hpp"

#define DEFAULT_DTRACK_PORT 5000
#define DTRACK_FLYSTICK_BUTTONS 6

class CaveXrDTrack {
private:
    DTrackSDK* dtrack = nullptr;

public:
    uint32_t dtrackFrame = 0;

    // Visibilité
    bool isTracking = false;
    bool headVisible = false;
    bool flystickVisible = false;

    // Tête
    double headPos[3] = {0};
    double headRot[9] = {0};
    DTrackQuaternion headQuat = DTrackQuaternion();

    // Flystick
    double flystickPos[3] = {0};
    DTrackQuaternion flystickQuat = DTrackQuaternion();
    int flystickButtons[DTRACK_FLYSTICK_BUTTONS] = {0};
    double flystickAnalog[2] = {0};

    CaveXrDTrack() : CaveXrDTrack(DEFAULT_DTRACK_PORT){}
    explicit CaveXrDTrack(uint16_t port);

    ~CaveXrDTrack();

    bool receive();
};

void* cavexr_dtrack_run(void* ptr);
void cavexr_dtrack_stop();