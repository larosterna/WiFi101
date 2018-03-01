/**
 *
 * \file
 *
 * \brief This module contains NMC1000 bus wrapper APIs implementation.
 *
 * Copyright (c) 2014 Atmel Corporation. All rights reserved.
 *
 * \asf_license_start
 *
 * \page License
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. The name of Atmel may not be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * 4. This software may only be redistributed and used in connection with an
 *    Atmel microcontroller product.
 *
 * THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
 * EXPRESSLY AND SPECIFICALLY DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * \asf_license_stop
 *
 */

#include <Arduino.h>
#include <SPI.h>

/*
 * Variants may define an alternative SPI instace to use for WiFi101.
 * If not defined the following defaults are used:
 *   WINC1501_SPI    - SPI
 */
#if !defined(WINC1501_SPI)
  #define WINC1501_SPI SPI
#endif

extern "C" {

#include "bsp/include/nm_bsp.h"
#include "bsp/include/nm_bsp_arduino.h"
#include "common/include/nm_common.h"
#include "bus_wrapper/include/nm_bus_wrapper.h"

}

// timing
#include "../../utility/dbtimer.h"

#define NM_BUS_MAX_TRX_SZ	512

tstrNmBusCapabilities egstrNmBusCapabilities =
{
	NM_BUS_MAX_TRX_SZ
};

#define wifi_SPISettings SPISettings(12000000L, MSBFIRST, SPI_MODE0)

// separate declaration for function attributes
static inline sint8 spi_rw(uint8* pu8Mosi, uint8* pu8Miso, uint16 u16Sz) __attribute__ ((hot));

static inline sint8 spi_rw(uint8* pu8Mosi, uint8* pu8Miso, uint16 u16Sz)
{
	uint8 u8Dummy[4] = {0,0,0,0};
	static uint8 initialized = 0;
	
	if (initialized == 0 or gi8Winc1501SpiFLags != 0)
		WINC1501_SPI.beginTransaction(wifi_SPISettings);
	digitalWrite(gi8Winc1501CsPin, LOW);

	// common small message sizes handled w/o call overhead
	uint8 *ptx = (pu8Mosi != 0) ? pu8Mosi : u8Dummy;
	uint8 *prx = (pu8Miso != 0) ? pu8Miso : u8Dummy;
	switch (u16Sz) {
		case 4:
			*prx++ = WINC1501_SPI.transfer(*ptx++);
		case 3:
			*prx++ = WINC1501_SPI.transfer(*ptx++);
		case 2:
			*prx++ = WINC1501_SPI.transfer(*ptx++);
		case 1:
			*prx = WINC1501_SPI.transfer(*ptx);
		case 0:
			break;
		default:
			WINC1501_SPI.transceive(pu8Mosi, pu8Miso, u16Sz);
	}

	digitalWrite(gi8Winc1501CsPin, HIGH);
	
	if (initialized == 0 or gi8Winc1501SpiFLags != 0) {
		WINC1501_SPI.endTransaction();
		initialized = 1;
	}

	return M2M_SUCCESS;
}

extern "C" {

/*
*	@fn		nm_bus_init
*	@brief	Initialize the bus wrapper
*	@return	M2M_SUCCESS in case of success and M2M_ERR_BUS_FAIL in case of failure
*	@author	M.S.M
*	@date	28 oct 2013
*	@version	1.0
*/
sint8 nm_bus_init(void * /* pvInitValue */)
{
	sint8 result = M2M_SUCCESS;

	/* Configure SPI peripheral. */
	WINC1501_SPI.begin();
	
	/* Configure CS PIN. */
	pinMode(gi8Winc1501CsPin, OUTPUT);
	digitalWrite(gi8Winc1501CsPin, HIGH);

	/* Reset WINC1500. */
	nm_bsp_reset();
	nm_bsp_sleep(1);

	return result;
}

/*
*	@fn		nm_bus_ioctl
*	@brief	send/receive from the bus
*	@param[IN]	u8Cmd
*					IOCTL command for the operation
*	@param[IN]	pvParameter
*					Arbitrary parameter depenging on IOCTL
*	@return	M2M_SUCCESS in case of success and M2M_ERR_BUS_FAIL in case of failure
*	@author	M.S.M
*	@date	28 oct 2013
*	@note	For SPI only, it's important to be able to send/receive at the same time
*	@version	1.0
*/
sint8 nm_bus_ioctl(uint8 u8Cmd, void* pvParameter)
{
	sint8 s8Ret = 0;
	switch(u8Cmd)
	{
		case NM_BUS_IOCTL_RW: {
			tstrNmSpiRw *pstrParam = (tstrNmSpiRw *)pvParameter;
			s8Ret = spi_rw(pstrParam->pu8InBuf, pstrParam->pu8OutBuf, pstrParam->u16Sz);
		}
		break;
		default:
			s8Ret = -1;
			M2M_ERR("invalide ioctl cmd\n");
			break;
	}

	return s8Ret;
}

/*
*	@fn		nm_bus_deinit
*	@brief	De-initialize the bus wrapper
*	@author	M.S.M
*	@date	28 oct 2013
*	@version	1.0
*/
sint8 nm_bus_deinit(void)
{
	WINC1501_SPI.end();
	return 0;
}

/*
*	@fn			nm_bus_reinit
*	@brief		re-initialize the bus wrapper
*	@param [in]	void *config
*					re-init configuration data
*	@return		M2M_SUCCESS in case of success and M2M_ERR_BUS_FAIL in case of failure
*	@author		Dina El Sissy
*	@date		19 Sept 2012
*	@version	1.0
*/
sint8 nm_bus_reinit(void* /* config */)
{
	return M2M_SUCCESS;
}

} // extern "C"

