/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/
#pragma once

#include "AP_OpenMV_config.h"

// #if AP_OpenMV_ENABLED

#include "AP_OpenMV_Backend.h"

#include <AP_HAL/AP_HAL.h>
#include <AP_SerialManager/AP_SerialManager.h>

class AP_OpenMV_Parameters;

class AP_OpenMV {

public:
    AP_OpenMV();

    ~AP_OpenMV();

    /* Do not allow copies */
    CLASS_NO_COPY(AP_OpenMV);

    // init - perform required initialisation
    void init(const AP_SerialManager &serial_manager);

    // update flight control mode. The control mode is vehicle type specific
    bool update(void);

    static AP_OpenMV *get_singleton(void) {
        return singleton;
    }

    // get next telemetry data for external consumers of SPort data
    static bool get_telem_data(AP_OpenMV_Backend::sport_packet_t* packet_array, uint8_t &packet_count, const uint8_t max_size);
#if HAL_WITH_FRSKY_TELEM_BIDIRECTIONAL
    // set telemetry data from external producer of SPort data
    static bool set_telem_data(const uint8_t frame,const uint16_t appid, const uint32_t data);
#endif

    void queue_message(MAV_SEVERITY severity, const char *text) {
        if (_backend == nullptr) {
            return;
        }
        return _backend->queue_text_message(severity, text);
    }

    uint8_t cx;
    uint8_t cy;

    uint32_t last_frame_ms;

private:

    AP_OpenMV_Backend *_backend;
    AP_OpenMV_Parameters* _frsky_parameters;

    // get next telemetry data for external consumers of SPort data (internal function)
    bool _get_telem_data(AP_OpenMV_Backend::sport_packet_t* packet_array, uint8_t &packet_count, const uint8_t max_size);
#if HAL_WITH_FRSKY_TELEM_BIDIRECTIONAL
    // set telemetry data from external producer of SPort data (internal function)
    bool _set_telem_data(const uint8_t frame, const uint16_t appid, const uint32_t data);
#endif
    static void try_create_singleton_for_external_data(void);
    static AP_OpenMV *singleton;

    AP_HAL::UARTDriver *port;

    uint8_t _step;

    uint8_t _cx_temp;
    uint8_t _cy_temp;

};

// namespace AP {
//     AP_OpenMV *frsky_telem();
// };

// #endif  // AP_OpenMV_ENABLED
