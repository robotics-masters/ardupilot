/*!
 * @file RCOutput_SeeSaw.cpp
 *
 * You may refer to: @file Adafruit_seesaw.cpp for original implementation on Adafruit
 *
 * This is part of Ardupilot's seesaw driver.
 *
 * These chips use I2C to communicate, 2 pins (SCL+SDA) are required
 * to interface with the board.
 *
 * The implementatio is based on original Adafruit's seesaw driver for the Arduino platform and 
 * Ardupilot RCOutput_PCA9685 driver for implementation reference
 *
 * Authors: Cian Byrne & Pietro Mincuzzi
 *
 */

#include "RCOutput_SeeSaw.h"

#include <cmath>
#include <dirent.h>
#include <fcntl.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>
#include <utility>

#include <AP_HAL/AP_HAL.h>

#include "GPIO.h"

using namespace Linux;
/* only for debug */

#define SEESAW_DEB 
#ifdef SEESAW_DEB 
#define SEESAW_DEBUG printf ("__PRETTY_FUNCTION__ = %s\n", __PRETTY_FUNCTION__); 
#define SEESAW_DEBUGP 
#else
#define SEESAW_DEBUG  
#define SEESAW_DEBUGP /##/
#endif

//the pwm pins
#define CRICKIT_NUM_PWM 12
static const uint8_t CRICKIT_pwms[CRICKIT_NUM_PWM] = {CRICKIT_SERVO4, CRICKIT_SERVO3, CRICKIT_SERVO2, CRICKIT_SERVO1,
						      CRICKIT_MOTOR_B1, CRICKIT_MOTOR_B2, CRICKIT_MOTOR_A1, CRICKIT_MOTOR_A2, 
						      CRICKIT_DRIVE4, CRICKIT_DRIVE3, CRICKIT_DRIVE2, CRICKIT_DRIVE1};
//the adc pin
#define CRICKIT_NUM_ADC 8
static const uint8_t CRICKIT_adc[CRICKIT_NUM_ADC] = { CRICKIT_SIGNAL1, CRICKIT_SIGNAL2, CRICKIT_SIGNAL3, CRICKIT_SIGNAL4,
						      CRICKIT_SIGNAL5, CRICKIT_SIGNAL6, CRICKIT_SIGNAL7, CRICKIT_SIGNAL8 };						      

#define PWM_CHAN_COUNT 8
						      
static const AP_HAL::HAL& hal = AP_HAL::get_HAL();

/**
 *****************************************************************************************
 *  @brief      Create a RCOutput_SEESAW object on a given I2C Device.
 *              The channel I2C device in use is specified as an AP_HAL::I2CDevice
 *              as installed in the linux /sys/class/i2c-dev/ path
 *
 *  @param      dev the I2C Device connected to the seesaw board
 ****************************************************************************************/

RCOutput_SEESAW::RCOutput_SEESAW(AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev) :
_dev(std::move(dev))
{
SEESAW_DEBUG
   _pulses_buffer = new pwm_channel[PWM_CHAN_COUNT];  /* note the zero-filling constructor */
}

/* standard destructor */

RCOutput_SEESAW::~RCOutput_SEESAW()
{
SEESAW_DEBUG
    delete [] _pulses_buffer;
}

/**
 *****************************************************************************************
 *  @brief      perform a software reset. This resets all seesaw registers to their default values.
 *				Then it resets all channel
 *
 *  			The software reste is taken from Adafruit_seesaw::SWReset()
 * 
 *
 *  @return     none
 ****************************************************************************************/

void RCOutput_SEESAW::init()
{
SEESAW_DEBUG
	this->write8(SEESAW_STATUS_BASE, SEESAW_STATUS_SWRST, 0xFF);
    /* Wait for the last pulse to end */
	reset_all_channels();
}

/**
 *****************************************************************************************
 *  @brief      Set all PWM channels to 0 frequency 0 value
 * 
 *
 *  @return     none
 ****************************************************************************************/
void RCOutput_SEESAW::reset_all_channels()
{
SEESAW_DEBUG

	uint32_t chmask = 1U;

	for (uint8_t ch =0; ch < PWM_CHAN_COUNT; ch++)
	{
		/* zero each channel period_us */
		
		// write(ch,0); /* not needed because already executed in set_freq() to finish all pulses */
		
		_pulses_buffer[ch].period_us = 0; /* simply zero the period in the local buffer */
		
		/* set as many lsb in the mask as there are channels */
		chmask<<=1;
		chmask++;
	}
	
	/* zeroes all channels period_us and frequency at once*/
	set_freq(chmask,0);

    /* Wait for the last pulse to end */
	hal.scheduler->delay(500);
	
}

/**
 *****************************************************************************************
 *  @brief      set the PWM frequency of a PWM channel. 
 *				The implementation assumes max 32 channels and each with independent frequency
 *				control. The function can set them all (PWM_CHAN_COUNT = 8)
 * 
 *  @param      chmask. 32 bit mask representing the channels to be affected
 *				where bit 0 -> channel 0 and bit 31 -> channel 31
 *	@param		freq the frequency to set (max freq = 0XFFFE)
 *
 *  @return     none
 *
 *  @Note		Functions fails silently if cannot get a semaphore from the device
 *
 ****************************************************************************************/

void RCOutput_SEESAW::set_freq(uint32_t chmask, uint16_t freq_hz)
{
SEESAW_DEBUG
    /* Correctly finish last pulses ( this is from original RCOutput_PCA9685 class) */
    
    cork();
    
    for (int i = 0; i < PWM_CHAN_COUNT; i++) {
        write(i, _pulses_buffer[i].freq_hz);
    }
	
	push();
	
	// limit max freq reserving value 0XFFFF for error handling
	if (freq_hz == 0XFFFF)
		freq_hz--;

	bool sem = false;
			
	for(int ch=0; ch< PWM_CHAN_COUNT; ch++){
		
		if(chmask&1){
		
			/* set semaphore if not yet so */
			if (sem == false)
			{
				if (!_dev->get_semaphore()->take(10)) {
					/* crash out if fails */
				    break;
				}
				sem = true;
			}

			/* set local copy */
			_pulses_buffer[ch].freq_hz = freq_hz;
			
			uint8_t cmd[] = {(uint8_t) ch, (uint8_t)(freq_hz >> 8), (uint8_t)freq_hz};
			writeI2C(SEESAW_TIMER_BASE, SEESAW_TIMER_FREQ, cmd, 3);
		
		}
		chmask>>=1;
	}
	
	/* release the semaphore if set */
	_dev->get_semaphore()->give();

}

/**
 *****************************************************************************************
 *  @brief      Get the PWM frequency of a PWM channel. 
 * 
 *  @param      ch. The channel to read
 *
 *  @return     a uint16_t representing the frequency read from the local buffer.
 *              returns 0XFFFF in case of error
 *
 ****************************************************************************************/

uint16_t RCOutput_SEESAW::get_freq(uint8_t ch)
{
SEESAW_DEBUG
	/* error handling */
	if (ch >= PWM_CHAN_COUNT)
		return 0xFFFF;	
		
    return _pulses_buffer[ch].freq_hz;
}

/**
 *****************************************************************************************
 *  @brief      Enable a channel. 
 * 
 *  @param      ch. The channel to enable
 *				the function is only provided for compatibility but has no implementation
 *				as all channels are always enabled
 *
 *  @return     none
 *
 ****************************************************************************************/
 
void RCOutput_SEESAW::enable_ch(uint8_t ch)
{
SEESAW_DEBUG
	/* error handling */
	if (ch >= PWM_CHAN_COUNT)
		return;
			
	/* not implemented. Channels always enabled */
}

/**
 *****************************************************************************************
 *  @brief      Disable a channel. 
 * 
 *  @param      ch. The channel to Disable
 *				Simply resets the channel
 *
 *  @return     none
 *
 ****************************************************************************************/
 void RCOutput_SEESAW::disable_ch(uint8_t ch)
{
SEESAW_DEBUG
    /* error handling */
	if (ch >= PWM_CHAN_COUNT)
		return;	
	
	_pulses_buffer[ch].period_us = 0; /* zero the period in the local buffer */
	
	uint32_t chmask = 1U;
	
	/* build channel mask with only one channel */
	for (uint8_t i=0;i<ch;i++)
		chmask<<=1;
		
	set_freq(chmask,0);
}

/**
 *****************************************************************************************
 *  @brief      set the PWM duty cycle. 
 * 				If corking then the value is only set in the local buffer and it will be 
 *				released when calling push()
 *  @param      ch. channel to be controlled
 *	@param		period_us. Value to be set. Valid value from 0 to 0XFFFE. 0XFFFF reserved for error
 *
 *  @return     none
 ****************************************************************************************/

void RCOutput_SEESAW::write(uint8_t ch, uint16_t period_us)
{
SEESAW_DEBUG
	/* error handling */
    if (ch >= PWM_CHAN_COUNT ) {
        return;
    }
    
    if(period_us == 0XFFFF)
    	period_us--;

    _pulses_buffer[ch].period_us = period_us;
    _pending_write_mask |= (1U << ch);
    
    if (!_corking) {
        _corking = true;
        push();
    }
}

/**
 *****************************************************************************************
 *  @brief      set the corking mode. 
 * 				If corking then the PWM values are only set in the local buffer by write()
 *              and they will be released when push() is called
 *
 *  @return     none
 ****************************************************************************************/

void RCOutput_SEESAW::cork()
{
SEESAW_DEBUG
    _corking = true;
}

/**
 *****************************************************************************************
 *  @brief      push all updated PWM values to the PWM channels. 
 *
 *  @return     none
 *
 *  @Note		Functions fails silently if cannot get a semaphore from the device
 *
 ****************************************************************************************/

void RCOutput_SEESAW::push()
{
SEESAW_DEBUG
	/* corking is set iether explicitly or within the write() function */
    if (!_corking) {
        return;
    }
    _corking = false;

    /* do nothing is no pending channel updates */
    if (_pending_write_mask == 0){
        return;
    }
	uint32_t channel_mask = 1U;

	bool sem = false;
			
	for(int ch=0; ch<PWM_CHAN_COUNT; ch++){
	
		if(_pending_write_mask & channel_mask)
		{
			/* set semaphore if not yet so */
			if (sem == false) {
				if (!_dev->get_semaphore()->take(10)) {
					/* crash out if fails */
				    break;
				}
				sem = true;
			}

			uint16_t period_us = _pulses_buffer[ch].period_us;
			uint8_t cmd[] = {(uint8_t) ch, (uint8_t)(period_us >> 8), (uint8_t)period_us};
			this->writeI2C(SEESAW_TIMER_BASE, SEESAW_TIMER_PWM, cmd, 3);
		}
		
		channel_mask <<=1;
	}

    _pending_write_mask = 0;

	/* release the semaphore if set */
	_dev->get_semaphore()->give();


}


/**
 *****************************************************************************************
 *  @brief      Get the PWM period of a PWM channel. 
 * 
 *  @param      ch. The channel to read
 *
 *  @return     a uint16_t representing the perio read from the local buffer.
 *              returns 0XFFFF in case of error
 *
 ****************************************************************************************/
uint16_t RCOutput_SEESAW::read(uint8_t ch)
{
SEESAW_DEBUG

	/* error handling */
	if (ch >= PWM_CHAN_COUNT)
		return 0XFFFF;	
		
    return _pulses_buffer[ch].period_us;
}

/**
 *****************************************************************************************
 *  @brief      Get the PWM period of a number of PWM channels from channel 0. 
 *				Reads as many chennels as requested within the maximum of exostong channels
 *				remaining values in the buffer if any are set of 0XFFFF
 *
 *  @param		period_us. Pointer to an array of uint16_t to receive the values
 *  @param      len. The numebr of channels to read
 *
 *  @return     none
 *
 ****************************************************************************************/
void RCOutput_SEESAW::read(uint16_t* period_us, uint8_t len)
{
SEESAW_DEBUG
    for (int i = 0; i < len; i++) {
    	if (i<PWM_CHAN_COUNT)
        	period_us[i] = _pulses_buffer[i].period_us;
    	else
    		period_us[i] = 0XFFFF;
    }
}

/**
 *****************************************************************************************
 *  @brief      Write 1 byte to the specified seesaw register.
 * 
 *  @param      regHigh the module address register (ex. SEESAW_NEOPIXEL_BASE)
 *	@param		regLow the function address register (ex. SEESAW_NEOPIXEL_PIN)
 *	@param		value the value between 0 and 255 to write
 *
 *  @Note		Functions fails silently if cannot get a semaphore from the device
 *
  ****************************************************************************************/
void RCOutput_SEESAW::write8(uint8_t regHigh, uint8_t regLow, uint8_t value)
{
SEESAW_DEBUG
	if (!_dev->get_semaphore()->take_nonblocking()) {
	    return;
	}

	writeI2C(regHigh, regLow, &value, 1);
	
    _dev->get_semaphore()->give();

}
/**
 *****************************************************************************************
 *  @brief      Write a specified number of bytes to the seesaw from the passed buffer 
 *              through the HAL I2C driver (asynchronous).
 *
 *				The function acts as a bridge through the Adafruit_seesaw driver that uses
 *              direct _i2cbus->write to the i2c channel and the HAL I2C driver that runs 
 *              asynchronously.
 * 
 *  @param      regHigh the module address register (ex. SEESAW_GPIO_BASE)
 *  @param		regLow the function address register (ex. SEESAW_GPIO_BULK_SET)
 *  @param		buf the buffer the the bytes from
 *  @param		num the number of bytes to write.
 *
 *  @return     none
 *
 *	@note		The functions does not hanlde the semaphore that must be handled by the calling function!
 *
 ****************************************************************************************/
void RCOutput_SEESAW::writeI2C(uint8_t regHigh, uint8_t regLow, uint8_t *buf, uint8_t num)
{
SEESAW_DEBUG

	size_t payload_size = 2+num;
	uint8_t I2C_values[payload_size];
	uint16_t p=0;
	
	I2C_values[p++]=(uint8_t)regHigh;
SEESAW_DEBUGP printf("frame: %02X ",(uint8_t)regHigh);
	I2C_values[p++]=(uint8_t)regLow;
SEESAW_DEBUGP printf("%02X ",(uint8_t)regLow);

	for (int i=0;i<num;i++,p++)
	{
		I2C_values[p++]=*buf++;
SEESAW_DEBUGP printf("%02X ",(uint8_t)I2C_values[p-1]);
	}
SEESAW_DEBUGP printf("\n");
    _dev->write_register(regHigh, regLow);
    _dev->transfer(buf, sizeof(buf), nullptr, 0);
    hal.scheduler->delay(10);

}
