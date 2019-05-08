#pragma once

#include <AP_HAL/I2CDevice.h>

#include "AP_HAL_Linux.h"

#include "SeeSaw_Registers.h"

#define SEESAW_PRIMARY_ADDRESS             0x49 


#define CRICKIT_DUTY_CYCLE_OFF 0
#define CRICKIT_DUTY_CYCLE_MAX 65535

/**
 * notes about implementation by merging Adafruit_seesaw, Adafruit_Crickit and RCOutput_(reference)
 *
 * init() --> SWreset() + reset all channels
 *
 * reset_all_channels() --> zero all periods and frequencies with atomic action
 * set_freq() --> setPWMFreq() + set local buffer 
 *  get_freq() --> returns stored local buffer
 * enable_ch() --> no return, simply not implemented.
 * disable_ch() --> zero period and freq. of a channel.
 * write() --> write local cache and if not corked forces push
 * cork() --> implemented as per RCOutput standard
 * push() --> transfers to the driver in one atomic action the period_us of all updated channels 
 * read() --> reads from a stored local buffer 
 * 
 * Private:
 * write8() --> same as in Adafruit_seesaw
 * writeI2C() --> queue in I2C driver a command frame
 */

namespace Linux {

class RCOutput_SEESAW : public AP_HAL::RCOutput {
public:
    RCOutput_SEESAW(AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev);

    ~RCOutput_SEESAW();
    void     init() override;
    void     SWreset();
    void     reset_all_channels();
    void     set_freq(uint32_t chmask, uint16_t freq_hz) override;
    uint16_t get_freq(uint8_t ch) override;
    void     enable_ch(uint8_t ch) override;
    void     disable_ch(uint8_t ch) override;
    void     write(uint8_t ch, uint16_t period_us) override;
    void     cork() override;
    void     push() override;
    uint16_t read(uint8_t ch) override;
    void     read(uint16_t* period_us, uint8_t len) override;

private:
    void 	 reset();
    void     write8(uint8_t regHigh, uint8_t regLow, uint8_t value);
	void 	 writeI2C(uint8_t regHigh, uint8_t regLow, uint8_t *buf, uint8_t num);

    AP_HAL::OwnPtr<AP_HAL::I2CDevice> _dev;
 
    bool _corking = false;
    uint32_t _pending_write_mask = 0;
    struct pwm_channel {

    	uint16_t freq_hz;
    	uint16_t period_us;
        pwm_channel() : freq_hz(0), period_us(0){} /* zero-filling constructor */

   	} *_pulses_buffer;

};

}