/*
 * ccid_serial.c: communicate with a GemPC Twin smart card reader
 * Copyright (C) 2001-2010 Ludovic Rousseau <ludovic.rousseau@free.fr>
 *
 * Thanks to Niki W. Waibel <niki.waibel@gmx.net> for a prototype version
 *
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
#include <syslog.h>
#define USE_SYSLOG 1
#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <string.h>
#include <errno.h>
#include <sys/time.h>
#include <sys/types.h>
#include <sys/ioctl.h>
#include <ifdhandler.h>
#include <stdarg.h>
#include <config.h>
#include "defs.h"
#include "ccid_ifdhandler.h"
#include "debug.h"
#include "ccid.h"
#include "utils.h"
#include "commands.h"
#include "parser.h"
#include "strlcpycat.h"

#define SYNC 0x03
#define CTRL_ACK 0x06
#define CTRL_NAK 0x15
#define RDR_to_PC_NotifySlotChange 0x50
#define CARD_ABSENT 0x02
#define CARD_PRESENT 0x03
#ifdef LOG_TO_STDERR
#define LOG_STREAM stderr
#else
#define LOG_STREAM stdout
#endif
#define COUNT_OF(arr) (sizeof(arr)/sizeof(arr[0]))
/*
 * normal command:
 * 1 : SYNC
 * 1 : CTRL
 * 10 +data length : CCID command
 * 1 : LRC
 *
 * SYNC : 0x03
 * CTRL : ACK (0x06) or NAK (0x15)
 * CCID command : see USB CCID specs
 * LRC : xor of all the previous byes
 *
 * Error message:
 * 1 : SYNC (0x03)
 * 1 : CTRL (NAK: 0x15)
 * 1 : LRC (0x16)
 *
 * Card insertion/withdrawal
 * 1 : RDR_to_PC_NotifySlotChange (0x50)
 * 1 : bmSlotIccState
 *     0x02 if card absent
 *     0x03 is card present
 *
 * Time request
 * T=1 : normal CCID command
 * T=0 : 1 byte (value between 0x80 and 0xFF)
 *
 */

/*
 * You may get read timeout after a card movement.
 * This is because you will get the echo of the CCID command
 * but not the result of the command.
 *
 * This is not an applicative issue since the card is either removed (and
 * powered off) or just inserted (and not yet powered on).
 */

/* 271 = max size for short APDU
 * 2 bytes for header
 * 1 byte checksum
 * doubled for echo
 */
#define GEMPCTWIN_MAXBUF (271 +2 +1) * 2

typedef struct
{
	/*
	 * File handle on the serial port
	 */
	int fd;

	/*
	 * device used ("/dev/ttyS?" under Linux)
	 */
	/*@null@*/ char *device;

	/*
	 * Number of slots using the same device
	 */
	int real_nb_opened_slots;
	int *nb_opened_slots;

	/*
	 * does the reader echoes the serial communication bytes?
	 */
	int echo;

	/*
	 * serial communication buffer
	 */
	unsigned char buffer[GEMPCTWIN_MAXBUF];

	/*
	 * next available byte
	 */
	int buffer_offset;

	/*
	 * number of available bytes
	 */
	int buffer_offset_last;

	/*
	 * CCID infos common to USB and serial
	 */
	_ccid_descriptor ccid;

} _serialDevice;

/* The _serialDevice structure must be defined before including ccid_serial.h */
#include "ccid_serial.h"

/* data rates supported by the GemPC Twin (serial and PCMCIA) */
unsigned int SerialTwinDataRates[] = { ISO_DATA_RATES, 0 };

/* data rates supported by the GemPC PinPad, GemCore Pos Pro & SIM Pro */
unsigned int SerialExtendedDataRates[] = { ISO_DATA_RATES, 500000, 0 };

/* data rates supported by the secondary slots on the GemCore Pos Pro & SIM Pro */
unsigned int SerialCustomDataRates[] = { GEMPLUS_CUSTOM_DATA_RATES, 0 };

/* data rates supported by the GemCore SIM Pro 2 */
unsigned int SIMPro2DataRates[] = { SIMPRO2_ISO_DATA_RATES, 0  };

/* no need to initialize to 0 since it is static */
static _serialDevice serialDevice[CCID_DRIVER_MAX_READERS];

/* unexported functions */
static int ReadChunk(unsigned int reader_index, unsigned char *buffer,
	int buffer_length, int min_length);

static int get_bytes(unsigned int reader_index, /*@out@*/ unsigned char *buffer,
	int length);

int InterruptRead(int reader_index, int timeout /* in ms */)
{
    syslog(LOG_ERR,"TCS:%s",__PRETTY_FUNCTION__);
    return 0;
}
void log_msg(const int priority, const char *fmt, ...)
{
    char debug_buffer[3 * 80]; /* up to 3 lines of 80 characters */
    va_list argptr;
    static struct timeval last_time = { 0, 0 };
    struct timeval new_time = { 0, 0 };
    struct timeval tmp;
    int delta;
#ifdef USE_SYSLOG
    int syslog_level;

    switch(priority)
    {
        case PCSC_LOG_CRITICAL:
            syslog_level = LOG_CRIT;
            break;
        case PCSC_LOG_ERROR:
            syslog_level = LOG_ERR;
            break;
        case PCSC_LOG_INFO:
            syslog_level = LOG_INFO;
            break;
        default:
            syslog_level = LOG_DEBUG;
    }
#else
    const char *color_pfx = "", *color_sfx = "";
    const char *time_pfx = "", *time_sfx = "";
    static int initialized = 0;
    static int LogDoColor = 0;

    if (!initialized)
    {
        char *term;

        initialized = 1;
        term = getenv("TERM");
        if (term)
        {
            const char *terms[] = { "linux", "xterm", "xterm-color", "Eterm", "rxvt", "rxvt-unicode", "xterm-256color" };
            unsigned int i;

            /* for each known color terminal */
            for (i = 0; i < COUNT_OF(terms); i++)
            {
                /* we found a supported term? */
                if (0 == strcmp(terms[i], term))
                {
                    LogDoColor = 1;
                    break;
                }
            }
        }
    }
syslog_level = LOG_DEBUG;
    if (LogDoColor)
    {
        color_sfx = "\33[0m";
        time_sfx = color_sfx;
        time_pfx = "\33[36m"; /* Cyan */

        switch (priority)
        {
            case PCSC_LOG_CRITICAL:
                color_pfx = "\33[01;31m"; /* bright + Red */
                break;

            case PCSC_LOG_ERROR:
                color_pfx = "\33[35m"; /* Magenta */
                break;

            case PCSC_LOG_INFO:
                color_pfx = "\33[34m"; /* Blue */
                break;

            case PCSC_LOG_DEBUG:
                color_pfx = ""; /* normal (black) */
                color_sfx = "";
                break;
        }
    }
#endif

    gettimeofday(&new_time, NULL);
    if (0 == last_time.tv_sec)
        last_time = new_time;

    tmp.tv_sec = new_time.tv_sec - last_time.tv_sec;
    tmp.tv_usec = new_time.tv_usec - last_time.tv_usec;
    if (tmp.tv_usec < 0)
    {
        tmp.tv_sec--;
        tmp.tv_usec += 1000000;
    }
    if (tmp.tv_sec < 100)
        delta = tmp.tv_sec * 1000000 + tmp.tv_usec;
    else
        delta = 99999999;

    last_time = new_time;

    va_start(argptr, fmt);
    (void)vsnprintf(debug_buffer, sizeof debug_buffer, fmt, argptr);
    va_end(argptr);

#ifdef USE_SYSLOG
    syslog(syslog_level, "%.8d %s", delta, debug_buffer);
#else
    (void)fprintf(LOG_STREAM, "%s%.8d%s %s%s%s\n", time_pfx, delta, time_sfx,
                  color_pfx, debug_buffer, color_sfx);
    fflush(LOG_STREAM);
#endif
} /* log_msg */

//void log_msg(const int priority, const char *fmt, ...){
//syslog(LOG_ERR,"TCS:%s",__PRETTY_FUNCTION__);
//    syslog(LOG_ERR,fmt,...);
//}
void log_xxd(const int priority, const char *msg, const unsigned char *buffer,
             const int len){
syslog(LOG_ERR,"TCS:%s",__PRETTY_FUNCTION__);
}

/*****************************************************************************
 *
 *				WriteSerial: Send bytes to the card reader
 *
 *****************************************************************************/
status_t WriteSerial(unsigned int reader_index, unsigned int length,
	unsigned char *buffer)
{
    syslog(LOG_ERR,"TCS:%s",__PRETTY_FUNCTION__);
	unsigned int i;
	unsigned char lrc;
	unsigned char low_level_buffer[GEMPCTWIN_MAXBUF];

	char debug_header[] = "-> 123456 ";

	(void)snprintf(debug_header, sizeof(debug_header), "-> %06X ",
		reader_index);

	if (length > GEMPCTWIN_MAXBUF-3)
	{
		DEBUG_CRITICAL3("command too long: %d for max %d",
			length, GEMPCTWIN_MAXBUF-3);
		return STATUS_UNSUCCESSFUL;
	}

	/* header */
	low_level_buffer[0] = 0x03;	/* SYNC */
	low_level_buffer[1] = 0x06;	/* ACK */

	/* CCID command */
	memcpy(low_level_buffer+2, buffer, length);

	/* checksum */
	lrc = 0;
	for(i=0; i<length+2; i++)
		lrc ^= low_level_buffer[i];
	low_level_buffer[length+2] = lrc;

	DEBUG_XXD(debug_header, low_level_buffer, length+3);

	if (write(serialDevice[reader_index].fd, low_level_buffer,
		length+3) != length+3)
	{
		DEBUG_CRITICAL2("write error: %s", strerror(errno));
		return STATUS_UNSUCCESSFUL;
	}

	return STATUS_SUCCESS;
} /* WriteSerial */


/*****************************************************************************
 *
 *				ReadSerial: Receive bytes from the card reader
 *
 *****************************************************************************/
status_t ReadSerial(unsigned int reader_index,
	unsigned int *length, unsigned char *buffer)
{
    syslog(LOG_ERR,"TCS:%s",__PRETTY_FUNCTION__);
	unsigned char c;
	int rv;
	int echo;
	int to_read;
	int i;

	/* we get the echo first */
	echo = serialDevice[reader_index].echo;

start:
	DEBUG_COMM("start");
	if ((rv = get_bytes(reader_index, &c, 1)) != STATUS_SUCCESS)
		return rv;

	if (c == RDR_to_PC_NotifySlotChange)
		goto slot_change;

	if (c == SYNC)
		goto sync;

	if (c >= 0x80)
	{
		DEBUG_COMM2("time request: 0x%02X", c);
		goto start;
	}

	DEBUG_CRITICAL2("Got 0x%02X", c);
	return STATUS_COMM_ERROR;

slot_change:
	DEBUG_COMM("slot change");
	if ((rv = get_bytes(reader_index, &c, 1)) != STATUS_SUCCESS)
		return rv;

	if (c == CARD_ABSENT)
	{
		DEBUG_COMM("Card removed");
	}
	else
		if (c == CARD_PRESENT)
		{
			DEBUG_COMM("Card inserted");
		}
		else
		{
			DEBUG_COMM2("Unknown card movement: %d", c);
		}
	goto start;

sync:
	DEBUG_COMM("sync");
	if ((rv = get_bytes(reader_index, &c, 1)) != STATUS_SUCCESS)
		return rv;

	if (c == CTRL_ACK)
		goto ack;

	if (c == CTRL_NAK)
		goto nak;

	DEBUG_CRITICAL2("Got 0x%02X instead of ACK/NAK", c);
	return STATUS_COMM_ERROR;

nak:
	DEBUG_COMM("nak");
	if ((rv = get_bytes(reader_index, &c, 1)) != STATUS_SUCCESS)
		return rv;

	if (c != (SYNC ^ CTRL_NAK))
	{
		DEBUG_CRITICAL2("Wrong LRC: 0x%02X", c);
		return STATUS_COMM_ERROR;
	}
	else
	{
		DEBUG_COMM("NAK requested");
		return STATUS_COMM_NAK;
	}

ack:
	DEBUG_COMM("ack");
	/* normal CCID frame */
	if ((rv = get_bytes(reader_index, buffer, 5)) != STATUS_SUCCESS)
		return rv;

	/* total frame size */
	to_read = 10+dw2i(buffer, 1);

	if ((to_read < 10) || (to_read > (int)*length))
	{
		DEBUG_CRITICAL2("Wrong value for frame size: %d", to_read);
		return STATUS_COMM_ERROR;
	}

	DEBUG_COMM2("frame size: %d", to_read);
	if ((rv = get_bytes(reader_index, buffer+5, to_read-5)) != STATUS_SUCCESS)
		return rv;

	DEBUG_XXD("frame: ", buffer, to_read);

	/* lrc */
	DEBUG_COMM("lrc");
	if ((rv = get_bytes(reader_index, &c, 1)) != STATUS_SUCCESS)
		return rv;

	DEBUG_COMM2("lrc: 0x%02X", c);
	for (i=0; i<to_read; i++)
		c ^= buffer[i];

	if (c != (SYNC ^ CTRL_ACK))
		DEBUG_CRITICAL2("Wrong LRC: 0x%02X", c);

	if (echo)
	{
		echo = FALSE;
		goto start;
	}

	/* length of data read */
	*length = to_read;

	return STATUS_SUCCESS;
} /* ReadSerial */


/*****************************************************************************
 *
 *				get_bytes: get n bytes
 *
 *****************************************************************************/
int get_bytes(unsigned int reader_index, unsigned char *buffer, int length)
{
    syslog(LOG_ERR,"TCS:%s",__PRETTY_FUNCTION__);
	int offset = serialDevice[reader_index].buffer_offset;
	int offset_last = serialDevice[reader_index].buffer_offset_last;

	DEBUG_COMM3("available: %d, needed: %d", offset_last-offset,
		length);
	/* enough data are available */
	if (offset + length <= offset_last)
	{
		DEBUG_COMM("data available");
		memcpy(buffer, serialDevice[reader_index].buffer + offset, length);
		serialDevice[reader_index].buffer_offset += length;
	}
	else
	{
		int present, rv;

		/* copy available data */
		present = offset_last - offset;

		if (present > 0)
		{
			DEBUG_COMM2("some data available: %d", present);
			memcpy(buffer, serialDevice[reader_index].buffer + offset,
				present);
		}

		/* get fresh data */
		DEBUG_COMM2("get more data: %d", length - present);
		rv = ReadChunk(reader_index, serialDevice[reader_index].buffer,
			sizeof(serialDevice[reader_index].buffer), length - present);
		if (rv < 0)
			return STATUS_COMM_ERROR;

		/* fill the buffer */
		memcpy(buffer + present, serialDevice[reader_index].buffer,
			length - present);
		serialDevice[reader_index].buffer_offset = length - present;
		serialDevice[reader_index].buffer_offset_last = rv;
		DEBUG_COMM3("offset: %d, last_offset: %d",
			serialDevice[reader_index].buffer_offset,
			serialDevice[reader_index].buffer_offset_last);
	}

	return STATUS_SUCCESS;
} /* get_bytes */


/*****************************************************************************
 *
 *				ReadChunk: read a minimum number of bytes
 *
 *****************************************************************************/
static int ReadChunk(unsigned int reader_index, unsigned char *buffer,
	int buffer_length, int min_length)
{
    syslog(LOG_ERR,"TCS:%s",__PRETTY_FUNCTION__);
	int fd = serialDevice[reader_index].fd;
# ifndef S_SPLINT_S
	fd_set fdset;
# endif
	struct timeval t;
	int i, rv = 0;
	int already_read;
	char debug_header[] = "<- 123456 ";

	(void)snprintf(debug_header, sizeof(debug_header), "<- %06X ",
		reader_index);

	already_read = 0;
	while (already_read < min_length)
	{
		/* use select() to, eventually, timeout */
		FD_ZERO(&fdset);
		FD_SET(fd, &fdset);
		t.tv_sec = serialDevice[reader_index].ccid.readTimeout / 1000;
		t.tv_usec = (serialDevice[reader_index].ccid.readTimeout - t.tv_sec*1000)*1000;

		i = select(fd+1, &fdset, NULL, NULL, &t);
		if (i == -1)
		{
			DEBUG_CRITICAL2("select: %s", strerror(errno));
			return -1;
		}
		else
			if (i == 0)
			{
				DEBUG_COMM2("Timeout! (%d ms)", serialDevice[reader_index].ccid.readTimeout);
				return -1;
			}

		rv = read(fd, buffer + already_read, buffer_length - already_read);
		if (rv < 0)
		{
			DEBUG_COMM2("read error: %s", strerror(errno));
			return -1;
		}

		DEBUG_XXD(debug_header, buffer + already_read, rv);

		already_read += rv;
		DEBUG_COMM3("read: %d, to read: %d", already_read,
			min_length);
	}

	return already_read;
} /* ReadChunk */


/*****************************************************************************
 *
 *				OpenSerial: open the port
 *
 *****************************************************************************/
status_t OpenSerial(unsigned int reader_index, int channel)
{
    syslog(LOG_ERR,"TCS:%s",__PRETTY_FUNCTION__);
	char dev_name[FILENAME_MAX];

	DEBUG_COMM3("Reader index: %X, Channel: %d", reader_index, channel);

	/*
	 * Conversion of old-style ifd-hanler 1.0 CHANNELID
	 */
	if (channel == 0x0103F8)
		channel = 1;
	else
		if (channel == 0x0102F8)
			channel = 2;
		else
			if (channel == 0x0103E8)
				channel = 3;
			else
				if (channel == 0x0102E8)
					channel = 4;

	if (channel < 0)
	{
		DEBUG_CRITICAL2("wrong port number: %d", channel);
		return STATUS_UNSUCCESSFUL;
	}

	(void)snprintf(dev_name, sizeof(dev_name), "/dev/pcsc/%d", channel);

	return OpenSerialByName(reader_index, dev_name);
} /* OpenSerial */

/*****************************************************************************
 *
 *				set_ccid_descriptor: init ccid descriptor
 *				depending on reader type specified in device.
 *
 *				return: STATUS_UNSUCCESSFUL,
 *						STATUS_SUCCESS,
 *						-1 (Reader already used)
 *
 *****************************************************************************/
static status_t set_ccid_descriptor(unsigned int reader_index,
	const char *reader_name, const char *dev_name)
{
    syslog(LOG_ERR,"TCS:%s",__PRETTY_FUNCTION__);
	int readerID;
	int i;
	int already_used = FALSE;
	static int previous_reader_index = -1;

	readerID = GEMPCTWIN;

	/* check if the same channel is not already used to manage multi-slots readers*/
	for (i = 0; i < CCID_DRIVER_MAX_READERS; i++)
	{
		if (serialDevice[i].device
			&& strcmp(serialDevice[i].device, dev_name) == 0)
		{
			already_used = TRUE;

			syslog(LOG_ERR,"%s already used. Multi-slot reader?", dev_name);
			break;
		}
	}

	/* this reader is already managed by us */
	if (already_used)
	{
		if ((previous_reader_index != -1)
			&& serialDevice[previous_reader_index].device
			&& (strcmp(serialDevice[previous_reader_index].device, dev_name) == 0)
			&& serialDevice[previous_reader_index].ccid.bCurrentSlotIndex < serialDevice[previous_reader_index].ccid.bMaxSlotIndex)
		{
			/* we reuse the same device and the reader is multi-slot */
			serialDevice[reader_index] = serialDevice[previous_reader_index];

			*serialDevice[reader_index].nb_opened_slots += 1;
			serialDevice[reader_index].ccid.bCurrentSlotIndex++;
			serialDevice[reader_index].ccid.dwSlotStatus = IFD_ICC_PRESENT;
			syslog(LOG_ERR,"Opening slot: %d",
					serialDevice[reader_index].ccid.bCurrentSlotIndex);
			switch (readerID)
			{
				case GEMCOREPOSPRO:
				case GEMCORESIMPRO:
					{
						/* Allocate a memory buffer that will be
						 * released in CloseUSB() */
						void *ptr = malloc(sizeof SerialCustomDataRates);
						if (ptr)
						{
							memcpy(ptr, SerialCustomDataRates,
									sizeof SerialCustomDataRates);
						}

						serialDevice[reader_index].ccid.arrayOfSupportedDataRates = ptr;
					}
					serialDevice[reader_index].ccid.dwMaxDataRate = 125000;
					break;

				case SEC1210:
					serialDevice[reader_index].ccid.arrayOfSupportedDataRates = NULL;
					serialDevice[reader_index].ccid.dwMaxDataRate = 826000;
					break;

				/* GemPC Twin or GemPC Card */
				default:
					serialDevice[reader_index].ccid.arrayOfSupportedDataRates = SerialTwinDataRates;
					serialDevice[reader_index].ccid.dwMaxDataRate = 344086;
					break;
			}
			goto end;
		}
		else
		{
			syslog(LOG_ERR,"Trying to open too many slots on %s", dev_name);
			return STATUS_UNSUCCESSFUL;
		}

	}

	/* Common to all readers */
	serialDevice[reader_index].ccid.real_bSeq = 0;
	serialDevice[reader_index].ccid.pbSeq = &serialDevice[reader_index].ccid.real_bSeq;
	serialDevice[reader_index].real_nb_opened_slots = 1;
	serialDevice[reader_index].nb_opened_slots = &serialDevice[reader_index].real_nb_opened_slots;
	serialDevice[reader_index].ccid.bCurrentSlotIndex = 0;

	serialDevice[reader_index].ccid.dwMaxCCIDMessageLength = 271;
	serialDevice[reader_index].ccid.dwMaxIFSD = 254;
	serialDevice[reader_index].ccid.dwFeatures = 0x00010230;
	serialDevice[reader_index].ccid.dwDefaultClock = 4000;

	serialDevice[reader_index].buffer_offset = 0;
	serialDevice[reader_index].buffer_offset_last = 0;

	serialDevice[reader_index].ccid.readerID = readerID;
	serialDevice[reader_index].ccid.bPINSupport = 0x0;
	serialDevice[reader_index].ccid.dwMaxDataRate = 344086;
	serialDevice[reader_index].ccid.bMaxSlotIndex = 0;
	serialDevice[reader_index].ccid.arrayOfSupportedDataRates = SerialTwinDataRates;
	serialDevice[reader_index].ccid.readTimeout = DEFAULT_COM_READ_TIMEOUT;
	serialDevice[reader_index].ccid.dwSlotStatus = IFD_ICC_PRESENT;
	serialDevice[reader_index].ccid.bVoltageSupport = 0x07;	/* 1.8V, 3V and 5V */
	serialDevice[reader_index].ccid.gemalto_firmware_features = NULL;
#ifdef ENABLE_ZLP
	serialDevice[reader_index].ccid.zlp = FALSE;
#endif
	serialDevice[reader_index].echo = TRUE;

	/* change some values depending on the reader */
	switch (readerID)
	{
		case GEMCOREPOSPRO:
			serialDevice[reader_index].ccid.bMaxSlotIndex = 4;	/* 5 slots */
			serialDevice[reader_index].ccid.arrayOfSupportedDataRates = SerialExtendedDataRates;
			serialDevice[reader_index].echo = FALSE;
			serialDevice[reader_index].ccid.dwMaxDataRate = 500000;
			break;

		case GEMCORESIMPRO:
			serialDevice[reader_index].ccid.bMaxSlotIndex = 1; /* 2 slots */
			serialDevice[reader_index].ccid.arrayOfSupportedDataRates = SerialExtendedDataRates;
			serialDevice[reader_index].echo = FALSE;
			serialDevice[reader_index].ccid.dwMaxDataRate = 500000;
			break;

		case GEMCORESIMPRO2:
			serialDevice[reader_index].ccid.dwDefaultClock = 4800;
			serialDevice[reader_index].ccid.bMaxSlotIndex = 1; /* 2 slots */
			serialDevice[reader_index].ccid.arrayOfSupportedDataRates = SIMPro2DataRates;
			serialDevice[reader_index].echo = FALSE;
			serialDevice[reader_index].ccid.dwMaxDataRate = 825806;
			break;

		case GEMPCPINPAD:
			serialDevice[reader_index].ccid.bPINSupport = 0x03;
			serialDevice[reader_index].ccid.arrayOfSupportedDataRates = SerialExtendedDataRates;
			serialDevice[reader_index].ccid.dwMaxDataRate = 500000;
			break;

		case SEC1210:
			serialDevice[reader_index].ccid.dwFeatures = 0x000100B2;
			serialDevice[reader_index].ccid.dwDefaultClock = 4800;
			serialDevice[reader_index].ccid.dwMaxDataRate = 826000;
			serialDevice[reader_index].ccid.arrayOfSupportedDataRates = NULL;
			serialDevice[reader_index].ccid.bMaxSlotIndex = 1;	/* 2 slots */
			serialDevice[reader_index].echo = FALSE;
			break;

	}

end:
	/* memorise the current reader_index so we can detect
	 * a new OpenSerialByName on a multi slot reader */
	previous_reader_index = reader_index;

	/* we just created a secondary slot on a multi-slot reader */
	if (already_used)
		return STATUS_SECONDARY_SLOT;

	return STATUS_SUCCESS;
} /* set_ccid_descriptor  */


/*****************************************************************************
 *
 *				OpenSerialByName: open the port
 *
 *****************************************************************************/
status_t OpenSerialByName(unsigned int reader_index, char *dev_name)
{
    return STATUS_SUCCESS;
} /* OpenSerialByName */


/*****************************************************************************
 *
 *				CloseSerial: close the port
 *
 *****************************************************************************/
status_t CloseSerial(unsigned int reader_index)
{
    syslog(LOG_ERR,"TCS:%s",__PRETTY_FUNCTION__);
	unsigned int reader = reader_index;

	/* device not opened */
	if (NULL == serialDevice[reader_index].device)
		return STATUS_UNSUCCESSFUL;

	syslog(LOG_ERR,"Closing serial device: %s", serialDevice[reader_index].device);

	/* Decrement number of opened slot */
	(*serialDevice[reader_index].nb_opened_slots)--;

	/* release the allocated ressources for the last slot only */
	if (0 == *serialDevice[reader_index].nb_opened_slots)
	{
		syslog(LOG_ERR,"Last slot closed. Release resources");

		(void)close(serialDevice[reader].fd);
		serialDevice[reader].fd = -1;

		free(serialDevice[reader].device);
		serialDevice[reader].device = NULL;
	}

	return STATUS_SUCCESS;
} /* CloseSerial */


/*****************************************************************************
 *
 *					get_ccid_descriptor
 *
 ****************************************************************************/
_ccid_descriptor *get_ccid_descriptor(unsigned int reader_index)
{
	return &serialDevice[reader_index].ccid;
} /* get_ccid_descriptor */


