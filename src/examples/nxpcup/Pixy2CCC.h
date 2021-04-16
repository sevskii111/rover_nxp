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
// This file is for defining the Block struct and the Pixy template class version 2.
// (TPixy2).  TPixy takes a communication link as a template parameter so that
// all communication modes (SPI, I2C and UART) can share the same code.
//

#ifndef _PIXY2CCC_H
#define _PIXY2CCC_H

#include <stdio.h>
#include <string.h>

#define CCC_MAX_SIGNATURE 7

#define CCC_RESPONSE_BLOCKS 0x21
#define CCC_REQUEST_BLOCKS 0x20

// Defines for sigmap:
// You can bitwise "or" these together to make a custom sigmap.
// For example if you're only interested in receiving blocks
// with signatures 1 and 5, you could use a sigmap of
// PIXY_SIG1 | PIXY_SIG5
#define CCC_SIG1 1
#define CCC_SIG2 2
#define CCC_SIG3 4
#define CCC_SIG4 8
#define CCC_SIG5 16
#define CCC_SIG6 32
#define CCC_SIG7 64
#define CCC_COLOR_CODES 128

#define CCC_SIG_ALL 0xff // all bits or'ed together

struct Block
{
	// print block structure!
	void print()
	{
		int i, j;
		char buf[128], sig[6], d;
		bool flag;

		if (m_signature > CCC_MAX_SIGNATURE)
		{ // color code! (CC)
			// convert signature number to an octal string
			for (i = 12, j = 0, flag = false; i >= 0; i -= 3)
			{
				d = (m_signature >> i) & 0x07;

				if (d > 0 && !flag)
				{
					flag = true;
				}

				if (flag)
				{
					sig[j++] = d + '0';
				}
			}

			sig[j] = '\0';
			sprintf(buf, "CC block sig: %s (%d decimal) x: %d y: %d width: %d height: %d angle: %d index: %d age: %d", sig,
					m_signature, m_x, m_y, m_width, m_height, m_angle, m_index, m_age);
		}
		else
		{ // regular block.  Note, angle is always zero, so no need to print
			sprintf(buf, "sig: %d x: %d y: %d width: %d height: %d index: %d age: %d", m_signature, m_x, m_y, m_width, m_height,
					m_index, m_age);
		}

		printf(buf);
		printf("\n");
	}

	uint16_t m_signature;
	uint16_t m_x;
	uint16_t m_y;
	uint16_t m_width;
	uint16_t m_height;
	int16_t m_angle;
	uint8_t m_index;
	uint8_t m_age;
};

template <class LinkType>
class TPixy2;

template <class LinkType>
class Pixy2CCC
{
public:
	Pixy2CCC(TPixy2<LinkType> *pixy)
	{
		m_pixy = pixy;
	}

	int8_t getBlocks(bool wait = true, uint8_t sigmap = CCC_SIG_ALL, uint8_t maxBlocks = 0xff);

	uint8_t numBlocks;
	Block *blocks;

private:
	TPixy2<LinkType> *m_pixy;
};

template <class LinkType>
int8_t Pixy2CCC<LinkType>::getBlocks(bool wait, uint8_t sigmap, uint8_t maxBlocks)
{
	blocks = NULL;
	numBlocks = 0;

	while (1)
	{
		// fill in request data
		m_pixy->m_bufPayload[0] = sigmap;
		m_pixy->m_bufPayload[1] = maxBlocks;
		m_pixy->m_length = 2;
		m_pixy->m_type = CCC_REQUEST_BLOCKS;

		// send request
		m_pixy->sendPacket();

		if (m_pixy->recvPacket() == 0)
		{
			if (m_pixy->m_type == CCC_RESPONSE_BLOCKS)
			{
				blocks = (Block *)m_pixy->m_buf;
				numBlocks = m_pixy->m_length / sizeof(Block);
				return numBlocks;
			}

			// deal with busy and program changing states from Pixy (we'll wait)
			else if (m_pixy->m_type == PIXY_TYPE_RESPONSE_ERROR)
			{
				if ((int8_t)m_pixy->m_buf[0] == PIXY_RESULT_BUSY)
				{
					if (!wait)
					{
						return PIXY_RESULT_BUSY; // new data not available yet
					}
				}
				else if ((int8_t)m_pixy->m_buf[0] != PIXY_RESULT_PROG_CHANGING)
				{
					return m_pixy->m_buf[0];
				}
			}
		}
		else
		{
			return PIXY_RESULT_ERROR; // some kind of bitstream error
		}

		// If we're waiting for frame data, don't thrash Pixy with requests.
		// We can give up half a millisecond of latency (worst case)
		//delayMicroseconds(500);
		usleep(500);
	}
}

#endif
