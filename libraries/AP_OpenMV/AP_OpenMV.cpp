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
   OpenMV library
*/

#define AP_SERIALMANAGER_OPEN_MV_BAUD         115200  // 定义串口的波特率为115200
#define AP_SERIALMANAGER_OPENMV_BUFSIZE_RX        64  // 串口接收的缓存的大小定义为64个字节
#define AP_SERIALMANAGER_OPENMV_BUFSIZE_TX        64  // 向OpenMV发送的缓存的大小也为64个字节


#include "AP_OpenMV.h"

#include <AP_SerialManager/AP_SerialManager.h>



extern const AP_HAL::HAL& hal;

//constructor 构造函数
AP_OpenMV::AP_OpenMV(void)
{
    _port = NULL;  // OpenMV对应的串口默认为空
    _step = 0;   // 默认解析帧的步骤是0
}

/*
 * init - perform required initialisation
 */
//bool AP_OpenMV::init(bool use_external_data)
void AP_OpenMV::init(const AP_SerialManager& serial_manager)
{
    // const AP_SerialManager &serial_manager = AP::serialmanager();

    // check for protocol configured for a serial port - only the first serial port with one of these protocols will then run (cannot have FrSky on multiple serial ports)
    // 查找飞控连接OpenMV的串口 → 关闭流控制 → 打开串口并开始运行
    if ((_port = serial_manager.find_serial(AP_SerialManager::SerialProtocol_OPEN_MV, 0))) { // 如果从串口管理器serial_manager去查找一个串口，该串口类别为OpenMV，查找到的串口赋值给_port；若_port被赋值完后不是空（说明找到了与OpenMV连接的串口）
        _port->set_flow_control(AP_HAL::UARTDriver::FLOW_CONTROL_DISABLE); // 把流控制关掉
        // initialise uart 打开串口（按三个定义的变量设置）
        _port->begin(AP_SERIALMANAGER_OPEN_MV_BAUD, AP_SERIALMANAGER_OPENMV_BUFSIZE_RX, AP_SERIALMANAGER_OPENMV_BUFSIZE_TX);
    } 

   // return true;
}

bool AP_OpenMV::update()  // 用于解析OpenMV的帧（定时调用）
{   // *从飞控的完善性来讲，必须先判断是否有这个驱动！！！如果没有立即返回！
    if(_port == NULL)  // 判断接口是否为空 （*必须加上，否则会有问题。上面的if没用到，即没找到串口，再执行下面的available()的时候程序会崩溃）
        return false;  // 若为空，立即返回错误

    int16_t numc = _port->available();  // 读取对应串口里面已经收到的（多少）字节的数据的个数，把个数赋给变量
    uint8_t data;  // 定义一个临时变量data，表示一个字符
    uint8_t checksum = 0;  // 校验和

    for (int16_t i = 0; i < numc; i++) { // 从一个个串口接收缓存里面读取这些数据，读取numc个数据
        data = _port->read();  // 挨个读取出缓存里的数据

        switch(_step) {  // 挨个读取步骤
        case 0: // 寻找帧头
            if(data == 0xA5)
                _step = 1; // 找到后进行下一步
            break;

        case 1:
            if(data == 0x5A)
                _step = 2;
            else
                _step = 0;  // 否则立即重新去找帧头1
            break;  // 记得一定加break

        case 2: // 接收的是x轴的坐标
            _cx_temp = data;    // 将数值赋给临时的变量
            _step = 3;
            break;

        case 3: // 接收y轴坐标
            _cy_temp = data;
            _step = 4;
            break;

        case 4:
            _step = 0;  // 不管校验是否通过，下一步一定是重新找帧头
            checksum = _cx_temp + _cy_temp; // 计算校验和
            if(checksum == data) { // data里面是接收的校验和。如果两者相等，说明解帧成功
                cx = _cx_temp;  // 把临时变量赋给最终变量
                cy = _cy_temp;
                last_frame_ms = AP_HAL::millis();   // 记录接收成功的这一帧的时间（从0开始记录的毫秒数）
                return true;    // 返回成功
            }
            break;  // 返回成功，函数结束了

        default:
            _step = 0;
        }
    }

    return false; //返回失败（没有收到一个完整的帧）
}

