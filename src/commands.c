/*
    commands.c: Commands sent to the card
    Copyright (C) 2003-2010   Ludovic Rousseau
    Copyright (C) 2005 Martin Paljak

    This library is free software; you can redistribute it and/or
    modify it under the terms of the GNU Lesser General Public
    License as published by the Free Software Foundation; either
    version 2.1 of the License, or (at your option) any later version.

    This library is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
    Lesser General Public License for more details.

	You should have received a copy of the GNU Lesser General Public License
	along with this library; if not, write to the Free Software Foundation,
	Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
*/

#include <config.h>
#include <syslog.h>
#ifdef HAVE_STRING_H
#include <string.h>
#endif
#ifdef HAVE_STDLIB_H
#include <stdlib.h>
#endif
#ifdef HAVE_ERRNO_H
#include <errno.h>
#endif
#ifdef HAVE_UNISTD_H
#include <unistd.h>
#endif

#include <pcsclite.h>
#include <ifdhandler.h>
#include <reader.h>

#include "misc.h"
#include "commands.h"
#include "openct/proto-t1.h"
#include "ccid.h"
#include "defs.h"
#include "ccid_ifdhandler.h"
#include "debug.h"
#include "utils.h"

/* All the pinpad readers I used are more or less bogus
 * I use code to change the user command and make the firmware happy */
#define BOGUS_PINPAD_FIRMWARE

/* The firmware of SCM readers reports dwMaxCCIDMessageLength = 263
 * instead of 270 so this prevents from sending a full length APDU
 * of 260 bytes since the driver check this value */
#define BOGUS_SCM_FIRMWARE_FOR_dwMaxCCIDMessageLength

#ifndef offsetof
#define offsetof(TYPE, MEMBER) ((size_t) &((TYPE *)0)->MEMBER)
#endif

#ifndef BSWAP_16
#define BSWAP_8(x)  ((x) & 0xff)
#define BSWAP_16(x) ((BSWAP_8(x) << 8) | BSWAP_8((x) >> 8))
#define BSWAP_32(x) ((BSWAP_16(x) << 16) | BSWAP_16((x) >> 16))
#endif

#define CHECK_STATUS(res) \
	if (STATUS_NO_SUCH_DEVICE == res) \
		return IFD_NO_SUCH_DEVICE; \
	if (STATUS_SUCCESS != res) \
		return IFD_COMMUNICATION_ERROR;

/* internal functions */
static RESPONSECODE CmdXfrBlockAPDU_extended(unsigned int reader_index,
	unsigned int tx_length, unsigned char tx_buffer[], unsigned int *rx_length,
	unsigned char rx_buffer[]);

static RESPONSECODE CmdXfrBlockTPDU_T0(unsigned int reader_index,
	unsigned int tx_length, unsigned char tx_buffer[], unsigned int *rx_length,
	unsigned char rx_buffer[]);

static RESPONSECODE CmdXfrBlockCHAR_T0(unsigned int reader_index, unsigned int
	tx_length, unsigned char tx_buffer[], unsigned int *rx_length, unsigned
	char rx_buffer[]);

static RESPONSECODE CmdXfrBlockTPDU_T1(unsigned int reader_index,
	unsigned int tx_length, unsigned char tx_buffer[], unsigned int *rx_length,
	unsigned char rx_buffer[]);

static void i2dw(int value, unsigned char *buffer);
static unsigned int bei2i(unsigned char *buffer);


/*****************************************************************************
 *
 *					CmdPowerOn
 *
 ****************************************************************************/
RESPONSECODE CmdPowerOn(unsigned int reader_index, unsigned int * nlength,
	unsigned char buffer[], int voltage)
{
    return IFD_SUCCESS;
} /* CmdPowerOn */


/*****************************************************************************
 *
 *					SecurePINVerify
 *
 ****************************************************************************/
RESPONSECODE SecurePINVerify(unsigned int reader_index,
	unsigned char TxBuffer[], unsigned int TxLength,
	unsigned char RxBuffer[], unsigned int *RxLength)
{
    return IFD_SUCCESS;

} /* SecurePINVerify */



/*****************************************************************************
 *
 *					SecurePINModify
 *
 ****************************************************************************/
RESPONSECODE SecurePINModify(unsigned int reader_index,
	unsigned char TxBuffer[], unsigned int TxLength,
	unsigned char RxBuffer[], unsigned int *RxLength)
{
     return IFD_SUCCESS;
}

/*****************************************************************************
 *
 *					Escape
 *
 ****************************************************************************/
RESPONSECODE CmdEscape(unsigned int reader_index,
	const unsigned char TxBuffer[], unsigned int TxLength,
	unsigned char RxBuffer[], unsigned int *RxLength, unsigned int timeout)
{
	return CmdEscapeCheck(reader_index, TxBuffer, TxLength, RxBuffer, RxLength,
		timeout, FALSE);
} /* CmdEscape */


/*****************************************************************************
 *
 *					Escape (with check of gravity)
 *
 ****************************************************************************/
RESPONSECODE CmdEscapeCheck(unsigned int reader_index,
	const unsigned char TxBuffer[], unsigned int TxLength,
	unsigned char RxBuffer[], unsigned int *RxLength, unsigned int timeout,
	int mayfail)
{
    syslog(LOG_ERR,"TCS:%s",__PRETTY_FUNCTION__);

	unsigned char *cmd_in, *cmd_out;
	status_t res;
	unsigned int length_in, length_out;
	RESPONSECODE return_value = IFD_SUCCESS;
	int old_read_timeout;
	_ccid_descriptor *ccid_descriptor = get_ccid_descriptor(reader_index);

	/* a value of 0 do not change the default read timeout */
	if (timeout > 0)
	{
		old_read_timeout = ccid_descriptor -> readTimeout;
		ccid_descriptor -> readTimeout = timeout;
	}

again:
	/* allocate buffers */
	length_in = 10 + TxLength;
	if (NULL == (cmd_in = malloc(length_in)))
	{
		return_value = IFD_COMMUNICATION_ERROR;
		goto end;
	}

	length_out = 10 + *RxLength;
	if (NULL == (cmd_out = malloc(length_out)))
	{
		free(cmd_in);
		return_value = IFD_COMMUNICATION_ERROR;
		goto end;
	}

	cmd_in[0] = 0x6B; /* PC_to_RDR_Escape */
	i2dw(length_in - 10, cmd_in+1);	/* dwLength */
	cmd_in[5] = ccid_descriptor->bCurrentSlotIndex;	/* slot number */
	cmd_in[6] = (*ccid_descriptor->pbSeq)++;
	cmd_in[7] = cmd_in[8] = cmd_in[9] = 0; /* RFU */

	/* copy the command */
	memcpy(&cmd_in[10], TxBuffer, TxLength);

	res = WritePort(reader_index, length_in, cmd_in);
	free(cmd_in);
	if (res != STATUS_SUCCESS)
	{
		free(cmd_out);
		if (STATUS_NO_SUCH_DEVICE == res)
			return_value = IFD_NO_SUCH_DEVICE;
		else
			return_value = IFD_COMMUNICATION_ERROR;
		goto end;
	}

time_request:
	length_out = 10 + *RxLength;
	res = ReadPort(reader_index, &length_out, cmd_out);

	/* replay the command if NAK
	 * This (generally) happens only for the first command sent to the reader
	 * with the serial protocol so it is not really needed for all the other
	 * ReadPort() calls */
	if (STATUS_COMM_NAK == res)
	{
		free(cmd_out);
		goto again;
	}

	if (res != STATUS_SUCCESS)
	{
		free(cmd_out);
		if (STATUS_NO_SUCH_DEVICE == res)
			return_value = IFD_NO_SUCH_DEVICE;
		else
			return_value = IFD_COMMUNICATION_ERROR;
		goto end;
	}

	if (length_out < STATUS_OFFSET+1)
	{
		free(cmd_out);
		DEBUG_CRITICAL2("Not enough data received: %d bytes", length_out);
		return_value = IFD_COMMUNICATION_ERROR;
		goto end;
	}

	if (cmd_out[STATUS_OFFSET] & CCID_TIME_EXTENSION)
	{
		DEBUG_COMM2("Time extension requested: 0x%02X", cmd_out[ERROR_OFFSET]);
		goto time_request;
	}

	if (cmd_out[STATUS_OFFSET] & CCID_COMMAND_FAILED)
	{
		/* mayfail: the error may be expected and not fatal */
		ccid_error(mayfail ? PCSC_LOG_INFO : PCSC_LOG_ERROR,
			cmd_out[ERROR_OFFSET], __FILE__, __LINE__, __FUNCTION__);    /* bError */
		return_value = IFD_COMMUNICATION_ERROR;
	}

	/* copy the response */
	length_out = dw2i(cmd_out, 1);
	if (length_out > *RxLength)
	{
		length_out = *RxLength;
		return_value = IFD_ERROR_INSUFFICIENT_BUFFER;
	}
	*RxLength = length_out;
	memcpy(RxBuffer, &cmd_out[10], length_out);

	free(cmd_out);

end:
	if (timeout > 0)
		ccid_descriptor -> readTimeout = old_read_timeout;

	return return_value;
} /* EscapeCheck */


/*****************************************************************************
 *
 *					CmdPowerOff
 *
 ****************************************************************************/
RESPONSECODE CmdPowerOff(unsigned int reader_index)
{
    //syslog(LOG_ERR,"TCS:%s",__PRETTY_FUNCTION__);

    return IFD_SUCCESS;


} /* CmdPowerOff */


/*****************************************************************************
 *
 *					CmdGetSlotStatus
 *
 ****************************************************************************/
RESPONSECODE CmdGetSlotStatus(unsigned int reader_index, unsigned char buffer[])
{
    static int is_on=0;
    //syslog(LOG_ERR,"TCS:%s",__PRETTY_FUNCTION__);

    if (is_on==0) {
        //syslog(LOG_ERR,"TCS:OFF!");

        buffer[7] = CCID_ICC_PRESENT_INACTIVE;
        is_on=1;
    }
   else  {
       //syslog(LOG_ERR,"TCS:ON!");
       buffer[7] = CCID_ICC_PRESENT_ACTIVE;
   }
    return IFD_SUCCESS;

}


/*****************************************************************************
 *
 *					CmdXfrBlock
 *
 ****************************************************************************/
RESPONSECODE CmdXfrBlock(unsigned int reader_index, unsigned int tx_length,
	unsigned char tx_buffer[], unsigned int *rx_length,
	unsigned char rx_buffer[], int protocol)
{
    //syslog(LOG_ERR,"TCS:%s",__PRETTY_FUNCTION__);


    return IFD_SUCCESS;

} /* CmdXfrBlock */


/*****************************************************************************
 *
 *					CCID_Transmit
 *
 ****************************************************************************/
RESPONSECODE CCID_Transmit(unsigned int reader_index, unsigned int tx_length,
	const unsigned char tx_buffer[], unsigned short rx_length, unsigned char bBWI)
{
    return IFD_SUCCESS;

} /* CCID_Transmit */


/*****************************************************************************
 *
 *					CCID_Receive
 *
 ****************************************************************************/
RESPONSECODE CCID_Receive(unsigned int reader_index, unsigned int *rx_length,
	unsigned char rx_buffer[], unsigned char *chain_parameter)
{
    //syslog(LOG_ERR,"TCS:%s",__PRETTY_FUNCTION__);
    return IFD_SUCCESS;


} /* CCID_Receive */


/*****************************************************************************
 *
 *					CmdXfrBlockAPDU_extended
 *
 ****************************************************************************/
static RESPONSECODE CmdXfrBlockAPDU_extended(unsigned int reader_index,
	unsigned int tx_length, unsigned char tx_buffer[], unsigned int *rx_length,
	unsigned char rx_buffer[])
{

    //syslog(LOG_ERR,"TCS:%s",__PRETTY_FUNCTION__);


    return IFD_SUCCESS;
} /* CmdXfrBlockAPDU_extended */


/*****************************************************************************
 *
 *					CmdXfrBlockTPDU_T0
 *
 ****************************************************************************/
static RESPONSECODE CmdXfrBlockTPDU_T0(unsigned int reader_index,
	unsigned int tx_length, unsigned char tx_buffer[], unsigned int *rx_length,
	unsigned char rx_buffer[])
{
    return IFD_SUCCESS;



} /* CmdXfrBlockTPDU_T0 */


/*****************************************************************************
 *
 *					T0CmdParsing
 *
 ****************************************************************************/
static RESPONSECODE T0CmdParsing(unsigned char *cmd, unsigned int cmd_len,
	/*@out@*/ unsigned int *exp_len)
{
    return IFD_SUCCESS;


} /* T0CmdParsing */


/*****************************************************************************
 *
 *					T0ProcACK
 *
 ****************************************************************************/
static RESPONSECODE T0ProcACK(unsigned int reader_index,
	unsigned char **snd_buf, unsigned int *snd_len,
	unsigned char **rcv_buf, unsigned int *rcv_len,
	unsigned char **in_buf, unsigned int *in_len,
	unsigned int proc_len, int is_rcv)
{
    //syslog(LOG_ERR,"TCS:%s",__PRETTY_FUNCTION__);
    return IFD_SUCCESS;

} /* T0ProcACK */


/*****************************************************************************
 *
 *					T0ProcSW1
 *
 ****************************************************************************/
static RESPONSECODE T0ProcSW1(unsigned int reader_index,
	unsigned char *rcv_buf, unsigned int *rcv_len,
	unsigned char *in_buf, unsigned int in_len)
{
    return IFD_SUCCESS;
} /* T0ProcSW1 */


/*****************************************************************************
 *
 *					CmdXfrBlockCHAR_T0
 *
 ****************************************************************************/
static RESPONSECODE CmdXfrBlockCHAR_T0(unsigned int reader_index,
	unsigned int snd_len, unsigned char snd_buf[], unsigned int *rcv_len,
	unsigned char rcv_buf[])
{
    return IFD_SUCCESS;
} /* CmdXfrBlockCHAR_T0 */


/*****************************************************************************
 *
 *					CmdXfrBlockTPDU_T1
 *
 ****************************************************************************/
static RESPONSECODE CmdXfrBlockTPDU_T1(unsigned int reader_index,
	unsigned int tx_length, unsigned char tx_buffer[], unsigned int *rx_length,
	unsigned char rx_buffer[])
{
    //syslog(LOG_ERR,"TCS:%s",__PRETTY_FUNCTION__);
    return IFD_SUCCESS;

} /* CmdXfrBlockTPDU_T1 */


/*****************************************************************************
 *
 *					SetParameters
 *
 ****************************************************************************/
RESPONSECODE SetParameters(unsigned int reader_index, char protocol,
	unsigned int length, unsigned char buffer[])
{
    //syslog(LOG_ERR,"TCS:%s",__PRETTY_FUNCTION__);
    return IFD_SUCCESS;

} /* SetParameters */


/*****************************************************************************
 *
 *					isCharLevel
 *
 ****************************************************************************/
int isCharLevel(int reader_index)
{
	return CCID_CLASS_CHARACTER == (get_ccid_descriptor(reader_index)->dwFeatures & CCID_CLASS_EXCHANGE_MASK);
} /* isCharLevel */


/*****************************************************************************
 *
 *					i2dw
 *
 ****************************************************************************/
static void i2dw(int value, unsigned char buffer[])
{
	buffer[0] = value & 0xFF;
	buffer[1] = (value >> 8) & 0xFF;
	buffer[2] = (value >> 16) & 0xFF;
	buffer[3] = (value >> 24) & 0xFF;
} /* i2dw */

/*****************************************************************************
*
*                  bei2i (big endian integer to host order interger)
*
****************************************************************************/

static unsigned int bei2i(unsigned char buffer[])
{
	return (buffer[0]<<24) + (buffer[1]<<16) + (buffer[2]<<8) + buffer[3];
}
