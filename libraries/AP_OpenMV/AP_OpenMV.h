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

#include <AP_HAL/AP_HAL.h>

#include <AP_SerialManager/AP_SerialManager.h>

class AP_Frsky_Parameters;

class AP_OpenMV {

public:
    AP_OpenMV();

    /* Do not allow copies */
    AP_OpenMV(const AP_OpenMV &other) = delete;
    AP_OpenMV &operator=(const AP_OpenMV&) = delete;
    
    // init - perform required initialisation
    bool init(bool use_external_data=false);
    // init - perform require initialisation including detecting which protocol to use
    void init(const AP_SerialManager& serial_manager);

    // update flight control mode. The control mode is vehicle type specific
    bool update(void);

    uint8_t cx; // x轴坐标
    uint8_t cy; // y轴坐标

    uint32_t last_frame_ms; // 最后一次收到OpenMV传过来的帧的时间（ms）
   

private:
    AP_HAL::UARTDriver *_port;  // UART used to send data to receiver OpenMV接到飞控的某个串口
    
    uint8_t _step; // 解析帧的步，走到哪一步

    uint8_t _cx_temp; // 存储接收到的x和y的坐标
    uint8_t _cy_temp;

};

