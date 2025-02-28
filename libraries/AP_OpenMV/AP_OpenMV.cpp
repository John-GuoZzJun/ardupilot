/*

   Inspired by work done here
   https://github.com/PX4/Firmware/tree/master/src/drivers/frsky_telemetry from Stefan Rado <px4@sradonia.net>
   https://github.com/opentx/opentx/tree/2.3/radio/src/telemetry from the OpenTX team

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

/* 
   FRSKY Telemetry library
*/

#define AP_SERIALMANAGER_OPEN_MV_BAUD         115200
#define AP_SERIALMANAGER_OPENMV_BUFSIZE_RX        64
#define AP_SERIALMANAGER_OPENMV_BUFSIZE_TX        64

#include "AP_OpenMV_config.h"

// #if AP_OpenMV_ENABLED

#include "AP_OpenMV.h"
#include "AP_OpenMV_Parameters.h"

#include <AP_SerialManager/AP_SerialManager.h>

#include <AP_Vehicle/AP_Vehicle.h>

#include "AP_OpenMV_D.h"


extern const AP_HAL::HAL& hal;

AP_OpenMV *AP_OpenMV::singleton;

AP_OpenMV::AP_OpenMV()
{
    port = NULL;
    _step = 0;

    singleton = this;
// #if HAL_WITH_FRSKY_TELEM_BIDIRECTIONAL
//     _frsky_parameters = &AP::vehicle()->frsky_parameters;
// #endif //HAL_WITH_FRSKY_TELEM_BIDIRECTIONAL
}


AP_OpenMV::~AP_OpenMV(void)
{
    singleton = nullptr;
}

/*
 * init - perform required initialisation
 */
void AP_OpenMV::init(const AP_SerialManager &serial_manager)
{
    // const AP_SerialManager &serial_manager = AP::serialmanager();

    if ((port = serial_manager.find_serial(AP_SerialManager::SerialProtocol_OPEN_MV, 0))) {
        port->set_flow_control(AP_HAL::UARTDriver::FLOW_CONTROL_DISABLE);
        // initialise uart
        port->begin(AP_SERIALMANAGER_OPEN_MV_BAUD, AP_SERIALMANAGER_OPENMV_BUFSIZE_RX, AP_SERIALMANAGER_OPENMV_BUFSIZE_TX);
    }
}
    // check for protocol configured for a serial port - only the first serial port with one of these protocols will then run (cannot have FrSky on multiple serial ports)
    // AP_HAL::UARTDriver *port;
//     if ((port = serial_manager.find_serial(AP_SerialManager::SerialProtocol_FrSky_D, 0))) {
// #if AP_OpenMV_D_TELEM_ENABLED
//         _backend = new AP_OpenMV_D(port);
// #endif
//     } else if ((port = serial_manager.find_serial(AP_SerialManager::SerialProtocol_FrSky_SPort, 0))) {
// #if AP_OpenMV_SPORT_TELEM_ENABLED
//         _backend = new AP_OpenMV_SPort(port);
// #endif
//     } else if (use_external_data || (port = serial_manager.find_serial(AP_SerialManager::SerialProtocol_FrSky_SPort_Passthrough, 0))) {
// #if AP_OpenMV_SPORT_PASSTHROUGH_ENABLED
//         _backend = new AP_OpenMV_SPort_Passthrough(port, use_external_data, _frsky_parameters);
// #endif
//     }

//     if (_backend == nullptr) {
//         return false;
//     }

//     if (!_backend->init()) {
//         delete _backend;
//         _backend = nullptr;
//         return false;
//     }

//     return true;
//}

bool AP_OpenMV::update()
{
    if(port == NULL)
        return false;

    int16_t numc = port->available();
    uint8_t data;
    uint8_t checksum = 0;

    for (int16_t i = 0; i < numc; i++) {
        data = port->read();

        switch(_step) {
        case 0:
            if(data == 0xA5)
                _step = 1;
            break;

        case 1:
            if(data == 0x5A)
                _step = 2;
            else
                _step = 0;
            break;

        case 2:
            _cx_temp = data;
            _step = 3;
            break;

        case 3:
            _cy_temp = data;
            _step = 4;
            break;

        case 4:
            _step = 0;
            checksum = _cx_temp + _cy_temp;
            if(checksum == data) {
                cx = _cx_temp;
                cy = _cy_temp;
                last_frame_ms = AP_HAL::millis();
                return true;
            }
            break;

        default:
            _step = 0;
        }
    }

    return false;
}

// bool AP_OpenMV::_get_telem_data(AP_OpenMV_Backend::sport_packet_t* packet_array, uint8_t &packet_count, const uint8_t max_size)
// {
//     if (_backend == nullptr) {
//         return false;
//     }
//     if (packet_array == nullptr) {
//         return false;
//     }
//     return _backend->get_telem_data(packet_array, packet_count, max_size);
// }

// #if HAL_WITH_FRSKY_TELEM_BIDIRECTIONAL
// bool AP_OpenMV::_set_telem_data(uint8_t frame, uint16_t appid, uint32_t data)
// {
//     if (_backend == nullptr) {
//         return false;
//     }
//     return _backend->set_telem_data(frame, appid, data);
// }
// #endif

// void AP_OpenMV::try_create_singleton_for_external_data()
// {
//     // try to allocate an AP_OpenMV object only if we are disarmed
//     if (!singleton && !hal.util->get_soft_armed()) {
//         new AP_OpenMV();
//         // initialize the passthrough scheduler
//         if (singleton) {
//             singleton->init(true);
//         }
//     }
// }

// /*
//   fetch Sport data for an external transport, such as FPort
//  */
// bool AP_OpenMV::get_telem_data(AP_OpenMV_Backend::sport_packet_t* packet_array, uint8_t &packet_count, const uint8_t max_size)
// {
//     try_create_singleton_for_external_data();
//     if (singleton == nullptr) {
//         return false;
//     }
//     return singleton->_get_telem_data(packet_array, packet_count, max_size);
// }

// #if HAL_WITH_FRSKY_TELEM_BIDIRECTIONAL
// /*
//   allow external transports (e.g. FPort), to supply telemetry data
//  */
// bool AP_OpenMV::set_telem_data(const uint8_t frame, const uint16_t appid, const uint32_t data)
// {
//     try_create_singleton_for_external_data();
//     if (singleton == nullptr) {
//         return false;
//     }
//     return singleton->_set_telem_data(frame, appid, data);
// }
// #endif

// namespace AP
// {
// AP_OpenMV *frsky_telem()
// {
//     return AP_OpenMV::get_singleton();
// }
// };

// #endif  // AP_OpenMV_ENABLED
