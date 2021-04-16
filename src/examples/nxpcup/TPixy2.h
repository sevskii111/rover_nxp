/****************************************************************************
 *
 * Copyright 2020 Charmed Labs.
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
// Main Pixy template class.  This class takes a link class and uses
// it to communicate with Pixy over I2C, SPI, UART or USB using the
// Pixy packet protocol.

#ifndef _TPIXY2_H
#define _TPIXY2_H

#include <px4_defines.h>

// uncomment to turn on debug prints to console
#define PIXY_DEBUG

#define PIXY_DEFAULT_ARGVAL 0x80000000
#define PIXY_BUFFERSIZE 0x104
#define PIXY_CHECKSUM_SYNC 0xc1af
#define PIXY_NO_CHECKSUM_SYNC 0xc1ae
#define PIXY_SEND_HEADER_SIZE 4
#define PIXY_MAX_PROGNAME 33

#define PIXY_TYPE_REQUEST_CHANGE_PROG 0x02
#define PIXY_TYPE_REQUEST_RESOLUTION 0x0c
#define PIXY_TYPE_RESPONSE_RESOLUTION 0x0d
#define PIXY_TYPE_REQUEST_VERSION 0x0e
#define PIXY_TYPE_RESPONSE_VERSION 0x0f
#define PIXY_TYPE_RESPONSE_RESULT 0x01
#define PIXY_TYPE_RESPONSE_ERROR 0x03
#define PIXY_TYPE_REQUEST_BRIGHTNESS 0x10
#define PIXY_TYPE_REQUEST_SERVO 0x12
#define PIXY_TYPE_REQUEST_LED 0x14
#define PIXY_TYPE_REQUEST_LAMP 0x16
#define PIXY_TYPE_REQUEST_FPS 0x18

#define PIXY_RESULT_OK 0
#define PIXY_RESULT_ERROR -1
#define PIXY_RESULT_BUSY -2
#define PIXY_RESULT_CHECKSUM_ERROR -3
#define PIXY_RESULT_TIMEOUT -4
#define PIXY_RESULT_BUTTON_OVERRIDE -5
#define PIXY_RESULT_PROG_CHANGING -6

// RC-servo values
#define PIXY_RCS_MIN_POS 0
#define PIXY_RCS_MAX_POS 1000L
#define PIXY_RCS_CENTER_POS ((PIXY_RCS_MAX_POS - PIXY_RCS_MIN_POS) / 2)

#include "Pixy2CCC.h"
#include "Pixy2Line.h"
#include "Pixy2Video.h"
#include <drivers/drv_hrt.h>
#include <nuttx/clock.h>

struct Version
{
	void print()
	{
		char buf[64];
		sprintf(buf, "hardware ver: 0x%x firmware ver: %d.%d.%d %s\n", hardware, firmwareMajor, firmwareMinor, firmwareBuild,
				firmwareType);
		printf(buf);
	}

	uint16_t hardware;
	uint8_t firmwareMajor;
	uint8_t firmwareMinor;
	uint16_t firmwareBuild;
	char firmwareType[10];
};

template <class LinkType>
class TPixy2
{
public:
	TPixy2();
	~TPixy2();

	int8_t init(uint32_t arg = PIXY_DEFAULT_ARGVAL);

	int8_t getVersion();
	int8_t changeProg(const char *prog);
	int8_t setServos(uint16_t s0, uint16_t s1);
	int8_t setCameraBrightness(uint8_t brightness);
	int8_t setLED(uint8_t r, uint8_t g, uint8_t b);
	int8_t setLamp(uint8_t upper, uint8_t lower);
	int8_t getResolution();
	int8_t getFPS();

	Version *version;
	uint16_t frameWidth;
	uint16_t frameHeight;

	// Color connected components, color codes
	Pixy2CCC<LinkType> ccc;
	friend class Pixy2CCC<LinkType>;

	// Line following
	Pixy2Line<LinkType> line;
	friend class Pixy2Line<LinkType>;

	// Video
	Pixy2Video<LinkType> video;
	friend class Pixy2Video<LinkType>;

	LinkType m_link;

private:
	int16_t getSync();
	int16_t recvPacket();
	int16_t sendPacket();

	uint8_t *m_buf;
	uint8_t *m_bufPayload;
	uint8_t m_type;
	uint8_t m_length;
	bool m_cs;
};

template <class LinkType>
TPixy2<LinkType>::TPixy2() : ccc(this), line(this), video(this)
{
	// allocate buffer space for send/receive
	m_buf = (uint8_t *)malloc(PIXY_BUFFERSIZE);
	// shifted buffer is used for sending, so we have space to write header information
	m_bufPayload = m_buf + PIXY_SEND_HEADER_SIZE;
	frameWidth = frameHeight = 0;
	version = NULL;
}

template <class LinkType>
TPixy2<LinkType>::~TPixy2()
{
	m_link.close();
	free(m_buf);
}

template <class LinkType>
int8_t TPixy2<LinkType>::init(uint32_t arg)
{
	// uint32_t t0;
	int8_t res;

	res = m_link.open(arg);

	if (res < 0)
	{
		return res;
	}

	// wait for pixy to be ready -- that is, Pixy takes a second or 2 boot up
	// getVersion is an effective "ping".  We timeout after 5s.
	// 	for (t0 = millis(); millis() - t0 < 5000;) {
	uint64_t start_time = hrt_absolute_time();

	while ((hrt_absolute_time() - start_time) < 5000000)
	{
		if (getVersion() >= 0)
		{					 // successful version get -> pixy is ready
			getResolution(); // get resolution so we have it
			return PIXY_RESULT_OK;
		}

		usleep(5000);
		// 		//delayMicroseconds(5000); // delay for sync
	}

	// timeout
	return PIXY_RESULT_TIMEOUT;
}

template <class LinkType>
int16_t TPixy2<LinkType>::getSync()
{
	uint8_t i, j, cprev;
	uint8_t c = 0;
	int16_t res;
	uint16_t start;

	// parse bytes until we find sync
	for (i = j = 0, cprev = 0; true; i++)
	{
		res = m_link.recv(&c, 1);

		if (res >= PIXY_RESULT_OK)
		{
			// since we're using little endian, previous byte is least significant byte
			start = cprev;
			// current byte is most significant byte
			start |= c << 8;
			cprev = c;

			if (start == PIXY_CHECKSUM_SYNC)
			{
				m_cs = true;
				return PIXY_RESULT_OK;
			}

			if (start == PIXY_NO_CHECKSUM_SYNC)
			{
				m_cs = false;
				return PIXY_RESULT_OK;
			}
		}

		// If we've read some bytes and no sync, then wait and try again.
		// And do that several more times before we give up.
		// Pixy guarantees to respond within 100us.
		if (i >= 4)
		{
			if (j >= 4)
			{
#ifdef PIXY_DEBUG
				PX4_ERR("error: no response");
#endif
				return PIXY_RESULT_ERROR;
			}

			//delayMicroseconds(25);
			usleep(25);
			j++;
			i = 0;
		}
	}
}

template <class LinkType>
int16_t TPixy2<LinkType>::recvPacket()
{
	uint16_t csCalc, csSerial;
	int16_t res;

	res = getSync();

	if (res < 0)
	{
		return res;
	}

	if (m_cs)
	{
		res = m_link.recv(m_buf, 4);

		if (res < 0)
		{
			return res;
		}

		m_type = m_buf[0];
		m_length = m_buf[1];

		csSerial = *(uint16_t *)&m_buf[2];

		res = m_link.recv(m_buf, m_length, &csCalc);

		if (res < 0)
		{
			return res;
		}

		if (csSerial != csCalc)
		{
#ifdef PIXY_DEBUG
			PX4_ERR("error: checksum");
#endif
			return PIXY_RESULT_CHECKSUM_ERROR;
		}
	}
	else
	{
		res = m_link.recv(m_buf, 2);

		if (res < 0)
		{
			return res;
		}

		m_type = m_buf[0];
		m_length = m_buf[1];

		res = m_link.recv(m_buf, m_length);

		if (res < 0)
		{
			return res;
		}
	}

	return PIXY_RESULT_OK;
}

template <class LinkType>
int16_t TPixy2<LinkType>::sendPacket()
{
	// write header info at beginnig of buffer
	m_buf[0] = PIXY_NO_CHECKSUM_SYNC & 0xff;
	m_buf[1] = PIXY_NO_CHECKSUM_SYNC >> 8;
	m_buf[2] = m_type;
	m_buf[3] = m_length;
	// send whole thing -- header and data in one call
	return m_link.send(m_buf, m_length + PIXY_SEND_HEADER_SIZE);
}

template <class LinkType>
int8_t TPixy2<LinkType>::changeProg(const char *prog)
{
	int32_t res;

	// poll for program to change
	while (1)
	{
		strncpy((char *)m_bufPayload, prog, PIXY_MAX_PROGNAME);
		m_length = PIXY_MAX_PROGNAME;
		m_type = PIXY_TYPE_REQUEST_CHANGE_PROG;
		sendPacket();

		if (recvPacket() == 0)
		{
			res = *(uint32_t *)m_buf;

			if (res > 0)
			{
				getResolution();	   // get resolution so we have it
				return PIXY_RESULT_OK; // success
			}
		}
		else
		{
			return PIXY_RESULT_ERROR; // some kind of bitstream error
		}

		//delayMicroseconds(1000);
		usleep(1000);
	}
}

template <class LinkType>
int8_t TPixy2<LinkType>::getVersion()
{
	m_length = 0;
	m_type = PIXY_TYPE_REQUEST_VERSION;
	sendPacket();

	if (recvPacket() == 0)
	{
		if (m_type == PIXY_TYPE_RESPONSE_VERSION)
		{
			version = (Version *)m_buf;
			return m_length;
		}
		else if (m_type == PIXY_TYPE_RESPONSE_ERROR)
		{
			return PIXY_RESULT_BUSY;
		}
	}

	return PIXY_RESULT_ERROR; // some kind of bitstream error
}

template <class LinkType>
int8_t TPixy2<LinkType>::getResolution()
{
	m_length = 1;
	m_bufPayload[0] = 0; // for future types of queries
	m_type = PIXY_TYPE_REQUEST_RESOLUTION;
	sendPacket();

	if (recvPacket() == 0)
	{
		if (m_type == PIXY_TYPE_RESPONSE_RESOLUTION)
		{
			frameWidth = *(uint16_t *)m_buf;
			frameHeight = *(uint16_t *)(m_buf + sizeof(uint16_t));
			return PIXY_RESULT_OK; // success
		}
		else
		{
			return PIXY_RESULT_ERROR;
		}
	}
	else
	{
		return PIXY_RESULT_ERROR; // some kind of bitstream error
	}
}

template <class LinkType>
int8_t TPixy2<LinkType>::setCameraBrightness(uint8_t brightness)
{
	uint32_t res;

	m_bufPayload[0] = brightness;
	m_length = 1;
	m_type = PIXY_TYPE_REQUEST_BRIGHTNESS;
	sendPacket();

	if (recvPacket() == 0)
	{ // && m_type==PIXY_TYPE_RESPONSE_RESULT && m_length==4)
		res = *(uint32_t *)m_buf;
		return (int8_t)res;
	}
	else
	{
		return PIXY_RESULT_ERROR; // some kind of bitstream error
	}
}

template <class LinkType>
int8_t TPixy2<LinkType>::setServos(uint16_t s0, uint16_t s1)
{
	uint32_t res;

	*(int16_t *)(m_bufPayload + 0) = s0;
	*(int16_t *)(m_bufPayload + 2) = s1;
	m_length = 4;
	m_type = PIXY_TYPE_REQUEST_SERVO;
	sendPacket();

	if (recvPacket() == 0 && m_type == PIXY_TYPE_RESPONSE_RESULT && m_length == 4)
	{
		res = *(uint32_t *)m_buf;
		return (int8_t)res;
	}
	else
	{
		return PIXY_RESULT_ERROR; // some kind of bitstream error
	}
}

template <class LinkType>
int8_t TPixy2<LinkType>::setLED(uint8_t r, uint8_t g, uint8_t b)
{
	uint32_t res;

	m_bufPayload[0] = r;
	m_bufPayload[1] = g;
	m_bufPayload[2] = b;
	m_length = 3;
	m_type = PIXY_TYPE_REQUEST_LED;
	sendPacket();

	if (recvPacket() == 0 && m_type == PIXY_TYPE_RESPONSE_RESULT && m_length == 4)
	{
		res = *(uint32_t *)m_buf;
		return (int8_t)res;
	}
	else
	{
		return PIXY_RESULT_ERROR; // some kind of bitstream error
	}
}

template <class LinkType>
int8_t TPixy2<LinkType>::setLamp(uint8_t upper, uint8_t lower)
{
	uint32_t res;

	m_bufPayload[0] = upper;
	m_bufPayload[1] = lower;
	m_length = 2;
	m_type = PIXY_TYPE_REQUEST_LAMP;
	sendPacket();

	if (recvPacket() == 0 && m_type == PIXY_TYPE_RESPONSE_RESULT && m_length == 4)
	{
		res = *(uint32_t *)m_buf;
		return (int8_t)res;
	}
	else
	{
		return PIXY_RESULT_ERROR; // some kind of bitstream error
	}
}

template <class LinkType>
int8_t TPixy2<LinkType>::getFPS()
{
	uint32_t res;

	m_length = 0; // no args
	m_type = PIXY_TYPE_REQUEST_FPS;
	sendPacket();

	if (recvPacket() == 0 && m_type == PIXY_TYPE_RESPONSE_RESULT && m_length == 4)
	{
		res = *(uint32_t *)m_buf;
		return (int8_t)res;
	}
	else
	{
		return PIXY_RESULT_ERROR; // some kind of bitstream error
	}
}

#endif
