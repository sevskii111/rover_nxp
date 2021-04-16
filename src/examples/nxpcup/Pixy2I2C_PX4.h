/****************************************************************************
 *
 * Copyright 2020 Charmed Labs.
 * Copyright 2020 NXP.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

//
// Arduino ICSP SPI link class

#ifndef _PIXY2_H
#define _PIXY2_H

#include "TPixy2.h"
#include <drivers/device/i2c.h>
#include "board_config.h"

#define IRLOCK_I2C_BUS 1
#define IRLOCK_I2C_ADDRESS 0x54 /** 7-bit address (non shifted) **/

#define IRLOCK0_DEVICE_PATH "/dev/Pixy2"

class PIXY2_I2C : public device::I2C
{
public:
	PIXY2_I2C() : I2C(0, "PIXY2_I2C", IRLOCK_I2C_BUS, IRLOCK_I2C_ADDRESS, 400000)
	{
		_external = true;
	}

	virtual ~PIXY2_I2C() = default;

	bool is_external()
	{
		return _external;
	};
	int init()
	{
		return I2C::init();
	};

	int8_t open(uint32_t arg)
	{
		return I2C::init();
		;
	}

	void close()
	{
		;
	}

	int16_t recv(uint8_t *buf, uint8_t len, uint16_t *cs = NULL)
	{
		uint8_t i;

		if (cs)
		{
			*cs = 0;
		}

		transfer(nullptr, 0, &buf[0], len);

		for (i = 0; i < len; i++)
		{

			if (cs)
			{
				*cs += buf[i];
			}
		}

		return len;
	}

	int16_t send(uint8_t *buf, uint8_t len)
	{

		transfer(&buf[0], len, nullptr, 0);
		//int ret_tran = transfer(&buf[0], len, nullptr, 0);
		//printf("ret_tran = %i\n", ret_tran);

		return len;
	}

private:
	bool _external;
};

typedef TPixy2<PIXY2_I2C> Pixy2;

#endif
