#pragma once

#include "AP_OpenMV_Backend.h"

#if AP_OpenMV_D_TELEM_ENABLED

class AP_OpenMV_D : public AP_OpenMV_Backend
{

public:

    using AP_OpenMV_Backend::AP_OpenMV_Backend;

protected:

    void send() override;
    uint32_t initial_baud() const override
    {
        return 9600;
    }

private:

    // methods related to the nuts-and-bolts of sending data
    void send_byte(uint8_t value);
    void send_uint16(uint16_t id, uint16_t data);

    struct {
        uint32_t last_200ms_frame;
        uint32_t last_1000ms_frame;
    } _D;

};

#endif  // AP_OpenMV_D_TELEM_ENABLED
