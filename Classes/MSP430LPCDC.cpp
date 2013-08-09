/*
 *
 * @APPLE_LICENSE_HEADER_START@
 * 
 * Copyright (c) 1998-2003 Apple Computer, Inc.  All Rights Reserved.
 * 
 * This file contains Original Code and/or Modifications of Original Code
 * as defined in and that are subject to the Apple Public Source License
 * Version 2.0 (the 'License'). You may not use this file except in
 * compliance with the License. Please obtain a copy of the License at
 * http://www.opensource.apple.com/apsl/ and read it before using this
 * file.
 * 
 * The Original Code and all software distributed under the License are
 * distributed on an 'AS IS' basis, WITHOUT WARRANTY OF ANY KIND, EITHER
 * EXPRESS OR IMPLIED, AND APPLE HEREBY DISCLAIMS ALL SUCH WARRANTIES,
 * INCLUDING WITHOUT LIMITATION, ANY WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE, QUIET ENJOYMENT OR NON-INFRINGEMENT.
 * Please see the License for the specific language governing rights and
 * limitations under the License.
 * 
 * @APPLE_LICENSE_HEADER_END@
 */

    /* MSP430LPCDC.cpp - MacOSX implementation based on Apple's CDC/ACM of	*/
    /* Launch Pad USB Communication Device Class (CDC) Driver, ACM Data Interface.	*/

#include <machine/limits.h>			/* UINT_MAX */
#include <libkern/OSByteOrder.h>

#include <IOKit/assert.h>
#include <IOKit/IOLib.h>
#include <IOKit/IOService.h>
#include <IOKit/IOBufferMemoryDescriptor.h>
#include <IOKit/IOMessage.h>

#include <IOKit/pwr_mgt/RootDomain.h>

#include <IOKit/usb/IOUSBBus.h>
#include <IOKit/usb/IOUSBNub.h>
#include <IOKit/usb/IOUSBDevice.h>
#include <IOKit/usb/IOUSBLog.h>
#include <IOKit/usb/IOUSBPipe.h>
#include <IOKit/usb/USB.h>
#include <IOKit/usb/IOUSBInterface.h>

#include <IOKit/serial/IOSerialKeys.h>
#include <IOKit/serial/IOSerialDriverSync.h>
#include <IOKit/serial/IOModemSerialStreamSync.h>
#include <IOKit/serial/IORS232SerialStreamSync.h>

#include <UserNotification/KUNCUserNotifications.h>

#define DEBUG_NAME "MSP430LPCDC"
#define drvver "1.0.3b"
#include "AppleUSBCDCACM.h"
#include "MSP430LPCDC.h"

#define MIN_BAUD (50 << 1)

//MSP430LPCDC		*gControlDriver = NULL;			// Our Control driver

static const OSSymbol *gPMWakeOnRingSymbol = NULL;

static IOPMPowerState gOurPowerStates[kNumCDCStates] =
{
    {1,0,0,0,0,0,0,0,0,0,0,0},
    {1,IOPMDeviceUsable,IOPMPowerOn,IOPMPowerOn,0,0,0,0,0,0,0,0}
};

#define super IOSerialDriverSync

OSDefineMetaClassAndStructors(MSP430LPCDC, IOSerialDriverSync);


// Encode the 4 modem status bits (so we only make one call to setState)

#if 0
static UInt32 sMapModemStates[16] = 
{
    0 |              0 |              0 |              0, // 0000
    0 |              0 |              0 | PD_RS232_S_DCD, // 0001
    0 |              0 | PD_RS232_S_DSR |              0, // 0010
    0 |              0 | PD_RS232_S_DSR | PD_RS232_S_DCD, // 0011
    0 | PD_RS232_S_BRK |              0 |              0, // 0100
    0 | PD_RS232_S_BRK |              0 | PD_RS232_S_DCD, // 0101
    0 | PD_RS232_S_BRK | PD_RS232_S_DSR |              0, // 0110
    0 | PD_RS232_S_BRK | PD_RS232_S_DSR | PD_RS232_S_DCD, // 0111
	PD_RS232_S_RNG |              0 |              0 |              0, // 1000
	PD_RS232_S_RNG |              0 |              0 | PD_RS232_S_DCD, // 1001
	PD_RS232_S_RNG |              0 | PD_RS232_S_DSR |              0, // 1010
	PD_RS232_S_RNG |              0 | PD_RS232_S_DSR | PD_RS232_S_DCD, // 1011
	PD_RS232_S_RNG | PD_RS232_S_BRK |              0 |              0, // 1100
	PD_RS232_S_RNG | PD_RS232_S_BRK |              0 | PD_RS232_S_DCD, // 1101
	PD_RS232_S_RNG | PD_RS232_S_BRK | PD_RS232_S_DSR |              0, // 1110
	PD_RS232_S_RNG | PD_RS232_S_BRK | PD_RS232_S_DSR | PD_RS232_S_DCD, // 1111
};
#endif


/****************************************************************************************************/
//
//		Function:	findControlDriverAD
//
//		Inputs:		me - my address
//
//		Outputs:	MSP430LPCDC
//
//		Desc:		Finds our matching control driver
//
/****************************************************************************************************/

// Think can delete as now both are the same driver.

MSP430LPCDC *findControlDriverAD(void *me)
{
    Boolean      worked             = false;
    MSP430LPCDC	*tempDriver         = (MSP430LPCDC*)me;    
    XTRACE(me, 0, 0, "findControlDriverAD");
    worked = true;
	return tempDriver;
	
}
/* end findControlDriverAD */


#if LOG_DATA
#define dumplen		32		// Set this to the number of bytes to dump and the rest should work out correct

#define buflen		((dumplen*2)+dumplen)+3
#define Asciistart	(dumplen*2)+3

/****************************************************************************************************/
//
//		Function:	USBLogData
//
//		Inputs:		Dir - direction
//				Count - number of bytes
//				buf - the data
//
//		Outputs:	
//
//		Desc:		Puts the data in the log. 
//
/****************************************************************************************************/

void MSP430LPCDC::USBLogData(UInt8 Dir, SInt32 Count, char *buf)
{    
    SInt32	wlen;
    SInt32	llen, rlen;
    SInt16	i, Aspnt, Hxpnt;
    UInt8	wchr;
    UInt8	LocBuf[buflen+1];
    
    switch (Dir)
    {
        case kDataIn:
            Log( "MSP430LPCDC: USBLogData - Read Complete, address = %8p, size = %8d\n", (void *)buf, (UInt)Count );
            break;
        case kDataOut:
            Log( "MSP430LPCDC: USBLogData - Write, address = %8p, size = %8d\n", (void *)buf, (UInt)Count );
            break;
        case kDataOther:
            Log( "MSP430LPCDC: USBLogData - Other, address = %8p, size = %8d\n", (void *)buf, (UInt)Count );
            break;
    }

    if (Count > dumplen)
    {
        wlen = dumplen;
    } else {
        wlen = Count;
    }

    if (wlen == 0)
    {
        Log( "MSP430LPCDC: USBLogData - No data, Count=0\n" );
        return;
    }

    rlen = 0;
    do
    {
		memset(LocBuf, 0x20, buflen);
        
        if (wlen > dumplen)
        {
            llen = dumplen;
            wlen -= dumplen;
        } else {
            llen = wlen;
            wlen = 0;
        }
        Aspnt = Asciistart;
        Hxpnt = 0;
        for (i=1; i<=llen; i++)
        {
            wchr = buf[i-1];
            LocBuf[Hxpnt++] = Asciify(wchr >> 4);
            LocBuf[Hxpnt++] = Asciify(wchr);
            if ((wchr < 0x20) || (wchr > 0x7F)) 		// Non printable characters
            {
                LocBuf[Aspnt++] = 0x2E;				// Replace with a period
            } else {
                LocBuf[Aspnt++] = wchr;
            }
        }
        LocBuf[Aspnt] = 0x00;

        Log("%s\n", LocBuf);
#if USE_IOL
        IOSleep(Sleep_Time);					// Try and keep the log from overflowing
#endif       
        rlen += llen;
        buf = &buf[rlen];
    } while (wlen != 0);

}/* end USBLogData */

/****************************************************************************************************/
//
//		Function:	MSP430LPCDC::dumpData
//
//		Inputs:		Dir - direction
//					buf - the data
//					size - number of bytes
//
//		Outputs:	None
//
//		Desc:		Creates formatted data for the log
//
/****************************************************************************************************/

void MSP430LPCDC::dumpData(UInt8 Dir, char *buf, SInt32 Count)
{
    SInt32	curr, len, dlen;
	
	switch (Dir)
    {
        case kDataIn:
            Log( "MSP430LPCDC: dumpData - Read Complete, address = %8p, size = %8d\n", (void *)buf, (UInt)Count );
            break;
        case kDataOut:
            Log( "MSP430LPCDC: dumpData - Write, address = %8p, size = %8d\n", (void *)buf, (UInt)Count );
            break;
        case kDataOther:
            Log( "MSP430LPCDC: dumpData - Other, address = %8p, size = %8d\n", (void *)buf, (UInt)Count );
            break;
    }

    dlen = 0;
    len = Count;
    
    for (curr=0; curr<Count; curr+=dumplen)
    {
        if (len > dumplen)
        {
            dlen = dumplen;
        } else {
            dlen = len;
        }
        Log("%8p ", (void *)&buf[curr]);
        USBLogData(kDataNone, dlen, &buf[curr]);
        len -= dlen;
    }
   
}/* end dumpData */
#endif

/****************************************************************************************************/
//
//		Method:		AddBytetoQueue
//
//		Inputs:		Queue - the queue to be added to
//				Value - Byte to be added
//
//		Outputs:	Queue status - full or no error
//
//		Desc:		Add a byte to the circular queue.
//				Check to see if there is space by comparing the next pointer,
//				with the last, If they match we are either Empty or full, so
//				check InQueue for zero.
//
/****************************************************************************************************/

QueueStatus MSP430LPCDC::AddBytetoQueue(CirQueue *Queue, char Value)
{
    
    if ((Queue->NextChar == Queue->LastChar) && Queue->InQueue)
    {
        return queueFull;
    }

    *Queue->NextChar++ = Value;
    Queue->InQueue++;

        // Check to see if we need to wrap the pointer.
		
    if (Queue->NextChar >= Queue->End)
        Queue->NextChar =  Queue->Start;

    return queueNoError;
	
}/* end AddBytetoQueue */

/****************************************************************************************************/
//
//		Method:		GetBytetoQueue
//
//		Inputs:		Queue - the queue to be removed from
//
//		Outputs:	Value - where to put the byte
//				QueueStatus - empty or no error
//
//		Desc:		Remove a byte from the circular queue.
//
/****************************************************************************************************/

QueueStatus MSP430LPCDC::GetBytetoQueue(CirQueue *Queue, UInt8 *Value)
{
    
    if ((Queue->NextChar == Queue->LastChar) && !Queue->InQueue)
    {
        return queueEmpty;
    }

    *Value = *Queue->LastChar++;
    Queue->InQueue--;

        // Check to see if we need to wrap the pointer.
        
    if (Queue->LastChar >= Queue->End)
        Queue->LastChar =  Queue->Start;

    return queueNoError;
	
}/* end GetBytetoQueue */

/****************************************************************************************************/
//
//		Method:		InitQueue
//
//		Inputs:		Queue - the queue to be initialized
//				Buffer - the buffer
//				size - length of buffer
//
//		Outputs:	QueueStatus - queueNoError.
//
//		Desc:		Pass a buffer of memory and this routine will set up the internal data structures.
//
/****************************************************************************************************/

QueueStatus MSP430LPCDC::InitQueue(CirQueue *Queue, UInt8 *Buffer, size_t Size)
{
    Queue->Start	= Buffer;
    Queue->End		= (UInt8*)((size_t)Buffer + Size);
    Queue->Size		= Size;
    Queue->NextChar	= Buffer;
    Queue->LastChar	= Buffer;
    Queue->InQueue	= 0;

//    IOSleep(1);
	
    return queueNoError ;
	
}/* end InitQueue */

/****************************************************************************************************/
//
//		Method:		CloseQueue
//
//		Inputs:		Queue - the queue to be closed
//
//		Outputs:	QueueStatus - queueNoError.
//
//		Desc:		Clear out all of the data structures.
//
/****************************************************************************************************/

QueueStatus MSP430LPCDC::CloseQueue(CirQueue *Queue)
{

    Queue->Start	= 0;
    Queue->End		= 0;
    Queue->NextChar	= 0;
    Queue->LastChar	= 0;
    Queue->Size		= 0;

    return queueNoError;
	
}/* end CloseQueue */

/****************************************************************************************************/
//
//		Function:	MSP430LPCDC::AddtoRXQueue
//
//		Inputs:		Queue - the queue to be added to
//					buffs - data to add
//					Size - length of data
//
//		Outputs:	BytesWritten - Number of bytes actually put in the queue.
//
//		Desc:		Add an entire buffer to the queue.
//
/****************************************************************************************************/

size_t MSP430LPCDC::AddtoRXQueue(CirQueue *Queue, inPipeBuffers *buffs, size_t Size)
{
	UInt8	*Buffer = buffs->pipeBuffer;
    size_t	BytesWritten = 0;
	size_t	inQueue = 0;
	
	inQueue = FreeSpaceinQueue(Queue);
	if (inQueue < Size)
	{
		XTRACE(this, inQueue, Size, "AddtoRXQueue - Queue full, buffer will be held" );
		return 0;
	}
	
    while (FreeSpaceinQueue(Queue) && (Size > BytesWritten))
    {
        AddBytetoQueue(Queue, *Buffer++);
        BytesWritten++;
    }
	
	if (BytesWritten < Size)
	{
		ALERT(BytesWritten, Size, "AddtoRXQueue - Queue full, data has been dropped" );
	}
	
    return BytesWritten;
	
}/* end AddtoRXQueue */

/****************************************************************************************************/
//
//		Method:		AddtoQueue
//
//		Inputs:		Queue - the queue to be added to
//				Buffer - data to add
//				Size - length of data
//
//		Outputs:	BytesWritten - Number of bytes actually put in the queue.
//
//		Desc:		Add an entire buffer to the queue.
//
/****************************************************************************************************/

size_t MSP430LPCDC::AddtoQueue(CirQueue *Queue, UInt8 *Buffer, size_t Size)
{
    size_t	BytesWritten = 0;

    while (FreeSpaceinQueue(Queue) && (Size > BytesWritten))
    {
        AddBytetoQueue(Queue, *Buffer++);
        BytesWritten++;
    }

    return BytesWritten;
	
}/* end AddtoQueue */

/****************************************************************************************************/
//
//		Method:		RemovefromQueue
//
//		Inputs:		Queue - the queue to be removed from
//				Size - size of buffer
//
//		Outputs:	Buffer - Where to put the data
//				BytesReceived - Number of bytes actually put in Buffer.
//
//		Desc:		Get a buffers worth of data from the queue.
//
/****************************************************************************************************/

size_t MSP430LPCDC::RemovefromQueue(CirQueue *Queue, UInt8 *Buffer, size_t MaxSize)
{
    size_t	BytesReceived = 0;
    UInt8	Value;

    //  while((GetBytetoQueue(Queue, &Value) == queueNoError) && (MaxSize >= BytesReceived))
    while((MaxSize > BytesReceived) && (GetBytetoQueue(Queue, &Value) == queueNoError)) 
    {
        *Buffer++ = Value;
        BytesReceived++;
    }/* end while */

    return BytesReceived;
	
}/* end RemovefromQueue */

/****************************************************************************************************/
//
//		Method:		FreeSpaceinQueue
//
//		Inputs:		Queue - the queue to be queried
//
//		Outputs:	Return Value - Free space left
//
//		Desc:		Return the amount of free space left in this buffer.
//
/****************************************************************************************************/

size_t MSP430LPCDC::FreeSpaceinQueue(CirQueue *Queue)
{
    size_t	retVal = 0;
    
    retVal = Queue->Size - Queue->InQueue;

    return retVal;
	
}/* end FreeSpaceinQueue */

/****************************************************************************************************/
//
//		Method:		UsedSpaceinQueue
//
//		Inputs:		Queue - the queue to be queried
//
//		Outputs:	UsedSpace - Amount of data in buffer
//
//		Desc:		Return the amount of data in this buffer.
//
/****************************************************************************************************/

size_t MSP430LPCDC::UsedSpaceinQueue(CirQueue *Queue)
{
    return Queue->InQueue;
	
}/* end UsedSpaceinQueue */

/****************************************************************************************************/
//
//		Method:		GetQueueSize
//
//		Inputs:		Queue - the queue to be queried
//
//		Outputs:	QueueSize - The size of the queue.
//
//		Desc:		Return the total size of the queue.
//
/****************************************************************************************************/

size_t MSP430LPCDC::GetQueueSize(CirQueue *Queue)
{
    return Queue->Size;
	
}/* end GetQueueSize */

/****************************************************************************************************/
//
//		Method:		GetQueueStatus
//
//		Inputs:		Queue - the queue to be queried
//
//		Outputs:	Queue status - full, empty or no error
//
//		Desc:		Returns the status of the circular queue.
//
/****************************************************************************************************/

QueueStatus MSP430LPCDC::GetQueueStatus(CirQueue *Queue)
{
    if ((Queue->NextChar == Queue->LastChar) && Queue->InQueue)
        return queueFull;
    else if ((Queue->NextChar == Queue->LastChar) && !Queue->InQueue)
        return queueEmpty;
		
    return queueNoError ;
	
}/* end GetQueueStatus */

/****************************************************************************************************/
//
//		Method:		CheckQueues
//
//		Inputs:		
//
//		Outputs:	
//
//		Desc:		Checks the various queue's etc and manipulates the state(s) accordingly
//				Must be called from a gated method or completion routine.
//
/****************************************************************************************************/

void MSP430LPCDC::CheckQueues()
{
    UInt32	Used;
    UInt32	Free;
    UInt32	QueuingState;
    UInt32	DeltaState;

	// Initialise the QueueState with the current state.
        
    QueuingState = fPort.State;

        // Check to see if there is anything in the Transmit buffer.
        
    Used = UsedSpaceinQueue(&fPort.TX);
    Free = FreeSpaceinQueue(&fPort.TX);
    
    XTRACE(this, Free, Used, "CheckQueues");
    
    if (Free == 0)
    {
        QueuingState |=  PD_S_TXQ_FULL;
        QueuingState &= ~PD_S_TXQ_EMPTY;
    } else {
        if (Used == 0)
	{
            QueuingState &= ~PD_S_TXQ_FULL;
            QueuingState |=  PD_S_TXQ_EMPTY;
        } else {
            QueuingState &= ~PD_S_TXQ_FULL;
            QueuingState &= ~PD_S_TXQ_EMPTY;
        }
    }

    	// Check to see if we are below the low water mark.
        
    if (Used < fPort.TXStats.LowWater)
         QueuingState |=  PD_S_TXQ_LOW_WATER;
    else QueuingState &= ~PD_S_TXQ_LOW_WATER;

    if (Used > fPort.TXStats.HighWater)
         QueuingState |= PD_S_TXQ_HIGH_WATER;
    else QueuingState &= ~PD_S_TXQ_HIGH_WATER;


        // Check to see if there is anything in the Receive buffer.
        
    Used = UsedSpaceinQueue(&fPort.RX);
    Free = FreeSpaceinQueue(&fPort.RX);

    if (Free == 0)
    {
        QueuingState |= PD_S_RXQ_FULL;
        QueuingState &= ~PD_S_RXQ_EMPTY;
    } else {
        if (Used == 0)
	{
            QueuingState &= ~PD_S_RXQ_FULL;
            QueuingState |= PD_S_RXQ_EMPTY;
        } else {
            QueuingState &= ~PD_S_RXQ_FULL;
            QueuingState &= ~PD_S_RXQ_EMPTY;
        }
    }

        // Check to see if we are below the low water mark.
    
    if (Used < fPort.RXStats.LowWater)
         QueuingState |= PD_S_RXQ_LOW_WATER;
    else QueuingState &= ~PD_S_RXQ_LOW_WATER;

    if (Used > fPort.RXStats.HighWater)
         QueuingState |= PD_S_RXQ_HIGH_WATER;
    else QueuingState &= ~PD_S_RXQ_HIGH_WATER;

        // Figure out what has changed to get mask.
        
    DeltaState = QueuingState ^ fPort.State;
    setStateGated(&QueuingState, &DeltaState);
	
}/* end CheckQueues */

/****************************************************************************************************/
//
//		Method:		CheckHold
//
//		Inputs:		
//
//		Outputs:	
//
//		Desc:		Checks to see if there's any held buffers
//
/****************************************************************************************************/

void MSP430LPCDC::CheckHold()
{
	SInt32			size;
	inPipeBuffers	*buffs;
	IOReturn		ior = kIOReturnSuccess;
	
	XTRACE(this, fPort.holdQueueIndxIn, fPort.holdQueueIndxOut, "CheckHold");
	
	while (1)
	{
		if (fPort.holdQueue[fPort.holdQueueIndxOut] != 0)
		{
			buffs = fPort.holdQueue[fPort.holdQueueIndxOut];
			size = AddtoRXQueue(&fPort.RX, buffs, buffs->count);
			if (size == 0)
			{
				XTRACE(this, fPort.holdQueueIndxIn, fPort.holdQueueIndxOut, "CheckHold - Still holding");
				break;
			} else {
				buffs->count = 0;
				buffs->held = false;
				XTRACE(this, fPort.holdQueueIndxIn, fPort.holdQueueIndxOut, "CheckHold - Read issued");
				ior = fPort.InPipe->Read(buffs->pipeMDP, &buffs->completionInfo, NULL);
				if (ior != kIOReturnSuccess)
				{
					XTRACE(this, fPort.holdQueueIndxOut, ior, "CheckHold - Read io err");
					buffs->dead = true;
				}
				fPort.holdQueue[fPort.holdQueueIndxOut] = 0;
				fPort.holdQueueIndxOut++;
				if (fPort.holdQueueIndxOut >= kMaxInBufPool)
				{
					fPort.holdQueueIndxOut = 0;
				}
			}
		} else {
			break;
		}
	}
	
	CheckQueues();
	
	XTRACE(this, fPort.holdQueueIndxIn, fPort.holdQueueIndxOut, "CheckHold - Exit");
	
}/* end CheckHold */

/****************************************************************************************************/
//
//		Method:		MSP430LPCDC::dataReadComplete
//
//		Inputs:		obj - me
//				param - the buffer pool pointer
//				rc - return code
//				remaining - what's left
//
//		Outputs:	None
//
//		Desc:		BulkIn pipe read completion routine
//
/****************************************************************************************************/

void MSP430LPCDC::dataReadComplete(void *obj, void *param, IOReturn rc, UInt32 remaining)
{
    MSP430LPCDC	*me = (MSP430LPCDC*)obj;
    inPipeBuffers		*buffs = (inPipeBuffers *)param;
    IOReturn		ior;
    size_t			length;
	SInt32			putInQueue = 0;
    
    XTRACE(me, rc, 0, "dataReadComplete");
    
    if (me->fStopping)
        return;

    if (rc == kIOReturnSuccess)				// If operation returned ok
    {
        length = DATA_BUFF_SIZE - remaining;
        XTRACE(me, me->fPort.State, length, "dataReadComplete - data length");
		
		if (length > 0)
		{
			meLogData(kDataIn, length, buffs->pipeBuffer);
	
				// Move the incoming bytes to the ring buffer, if we can
            
//			me->AddtoQueue(&me->fPort.RX, buffs->pipeBuffer, length);
		
				// If the indices are not equal then there's something in the hold queue
		
			if (me->fPort.holdQueueIndxIn != me->fPort.holdQueueIndxOut)
			{
				putInQueue = 0;
			} else {
				putInQueue = me->AddtoRXQueue(&me->fPort.RX, buffs, length);
			}
			if (putInQueue == 0)
			{
				XTRACE(me, 0, me->fPort.holdQueueIndxIn, "dataReadComplete - Buffer held");
				buffs->held = true;
				buffs->count = length;
				me->fPort.holdQueue[me->fPort.holdQueueIndxIn++] = buffs;
				if (me->fPort.holdQueueIndxIn >= kMaxInBufPool)
				{
					me->fPort.holdQueueIndxIn = 0;
				}
			}
		}
    } else {
        XTRACE(me, 0, rc, "dataReadComplete - error");
		if (rc != kIOReturnAborted)
        {
			if (rc == kIOUSBPipeStalled)
			{
				rc = me->checkPipe(me->fPort.InPipe, true);
			} else {
				rc = me->checkPipe(me->fPort.InPipe, false);
			}
            if (rc != kIOReturnSuccess)
            {
                XTRACE(me, 0, rc, "dataReadComplete - clear stall failed (trying to continue)");
            }
        }
    }
    
        // Queue the read only if not aborted and the buffer is not held
	
    if (rc != kIOReturnAborted)
    {
		if (!buffs->held)
		{
			XTRACE(me, 0, me->fPort.holdQueueIndxIn, "dataReadComplete - Read issued");
			ior = me->fPort.InPipe->Read(buffs->pipeMDP, &buffs->completionInfo, NULL);
			if (ior != kIOReturnSuccess)
			{
				XTRACEP(me, buffs, ior, "dataReadComplete - Read io err");
				buffs->dead = true;
			} else {
				XTRACEP(me, buffs, me->fPort.InPipe, "dataReadComplete - Read posted");
			}
		}
        me->CheckQueues();
	} else {
		XTRACEP(me, buffs, me->fPort.InPipe, "dataReadComplete - Read aborted");
	}
	
}/* end dataReadComplete */

/****************************************************************************************************/
//
//		Method:		MSP430LPCDC::dataWriteComplete
//
//		Inputs:		obj - me
//				param - the buffer pool pointer
//				rc - return code
//				remaining - what's left
//
//		Outputs:	None
//
//		Desc:		BulkOut pipe write completion routine
//
/****************************************************************************************************/

void MSP430LPCDC::dataWriteComplete(void *obj, void *param, IOReturn rc, UInt32 remaining)
{
    MSP430LPCDC	*me = (MSP430LPCDC *)obj;
    outPipeBuffers		*buffs = (outPipeBuffers *)param;
    SInt32		dLen;
    UInt16		i;
    bool		busy = false;
	UInt32		state;
	UInt32		mask;
    
    XTRACE(me, rc, 0, "dataWriteComplete");
    
    if (me->fStopping)
        return;

    if (rc == kIOReturnSuccess)
    {	
        dLen = buffs->count - remaining;
        XTRACE(me, 0, dLen, "dataWriteComplete - data length");
        if (dLen > 0)						// Check if it was a zero length write
        {
            if ((dLen % me->fPort.OutPacketSize) == 0)		// If it was a multiple of max packet size then we need to do a zero length write
            {
                XTRACE(me, rc, dLen, "dataWriteComplete - writing zero length packet");
                buffs->count = 0;
                buffs->pipeMDP->setLength(0);
                
                me->fPort.OutPipe->Write(buffs->pipeMDP, &buffs->completionInfo);
                return;
            } else {
                buffs->avail = true;
            }
        } else {
            buffs->avail = true;
        }

        me->CheckQueues();
        
            // If any of the buffers are unavailable then we're still busy

        for (i=0; i<me->fOutBufPool; i++)
        {
            if (!me->fPort.outPool[i].avail)
            {
                busy = true;
                break;
            }
        }

        if (!busy)
        {
			state = 0;
			mask = PD_S_TX_BUSY;
            me->setStateGated(&state, &mask);	// Clear the busy state
        }

        me->setUpTransmit();						// just to keep it going??

    } else {
        XTRACE(me, 0, rc, "dataWriteComplete - io error");
		if (rc != kIOReturnAborted)
        {
			if (rc == kIOUSBPipeStalled)
			{
				rc = me->checkPipe(me->fPort.InPipe, true);
			} else {
				rc = me->checkPipe(me->fPort.InPipe, false);
			}
            if (rc != kIOReturnSuccess)
            {
                XTRACE(me, 0, rc, "dataWriteComplete - clear stall failed (trying to continue)");
            }
        }

        
        buffs->avail = true;
        
             // If any of the buffers are unavailable then we're still busy

        for (i=0; i<me->fOutBufPool; i++)
        {
            if (!me->fPort.outPool[i].avail)
            {
                busy = true;
                break;
            }
        }

        if (!busy)
        {
			state = 0;
			mask = PD_S_TX_BUSY;
            me->setStateGated(&state, &mask);
        }
    }
	
}/* end dataWriteComplete */

/****************************************************************************************************/
//
//		Method:		MSP430LPCDC::probe
//
//		Inputs:		provider - my provider
//
//		Outputs:	IOService - from super::probe, score - probe score
//
//		Desc:		Modify the probe score if necessary (we don't  at the moment)
//
/****************************************************************************************************/

IOService* MSP430LPCDC::probe( IOService *provider, SInt32 *score )
{ 
    IOService   *res;
    
    XTRACE(this,0,0, "Probing");
    
    fMSP430Interface = OSDynamicCast(IOUSBInterface, provider);
    if(!fMSP430Interface)
    {
        ALERT(0, 0, "start - provider invalid");
        return 0;
    }
    
    fProductID = fMSP430Interface->GetDevice()->GetProductID();
    fVendorID = fMSP430Interface->GetDevice()->GetVendorID();
 
   if ((fVendorID != 0x0451) || (fProductID != 0xf432)) {
        IOLog("%s : CDC /ACM found but not Launchpad / eZ430-RF2500\n",getName());
        return 0;
    }

    res = super::probe(provider, score);
    
    return res;
    
}/* end probe */

/****************************************************************************************************/
//
//		Method:		MSP430LPCDC::start
//
//		Inputs:		provider - my provider
//
//		Outputs:	Return code - true (it's me), false (sorry it probably was me, but I can't configure it)
//
//		Desc:		This is called once it has been determined I'm probably the best 
//				driver for this device.
//
/****************************************************************************************************/

bool MSP430LPCDC::start(IOService *provider)
{

    OSNumber	*bufNumber = NULL;
    UInt16		bufValue = 0;
	IOUSBDevice *usbDevice;
    int err;
    char devicename[0x40] = "";
    char deviceserialno[0x22] = "";
	
	XTRACE(this, 0, 0, "start");
    
    fSessions = 0;
    fTerminate = false;
	fSuppressWarning = false;
	fEnumOnWake = false;
    fStopping = false;
	fCDCDriver = NULL;
	fControlDriver = NULL;
	fWorkLoop = NULL;
	fPMRootDomain = NULL;
	fWoR = false;
	fWakeSettingControllerHandle = NULL;
	fWanDevice = kOSBooleanFalse;
    fThreadSleepCount = 0;
    fReady = false;
    
    initStructure();
    
    if(!super::start(provider))
    {
        ALERT(0, 0, "start - super failed");
        return false;
    }

	// Get my USB provider - the interface

    fMSP430Interface = OSDynamicCast(IOUSBInterface, provider);
    if(!fMSP430Interface)
    {
        ALERT(0, 0, "start - provider invalid");
        return false;
    }

    IOLog("----------------------------------------------------\n");
    IOLog("Texas Instrument LaunchPad / EZ430-RF2500 CDC/ACM Driver\n");
    IOLog("           Version %s - Terence Ang 2011 \n", drvver);
    IOLog("----------------------------------------------------\n");
    
    usbDevice = OSDynamicCast (IOUSBDevice, fMSP430Interface->GetDevice());
    
    err = usbDevice->GetStringDescriptor(usbDevice->GetProductStringIndex(), devicename, 0x40 , 0x409);
    err = usbDevice->GetStringDescriptor(usbDevice->GetSerialNumberStringIndex(), deviceserialno, 0x22 , 0x409);
    
    if(err){
        ALERT(0, 0, "Probing - Error Getting Device name and Serial no.");
        return NULL;
    }
    
    IOLog("%s : Device Name = %s\n", getName() , devicename);
    IOLog("%s : Serial Number = %s\n", getName(), deviceserialno);
    
    IOLog("----------------------------------------------------\n");
    

    fPort.DataInterfaceNumber = fMSP430Interface->GetInterfaceNumber();
    
      // get workloop
        
    fWorkLoop = getWorkLoop();
    if (!fWorkLoop)
    {
        ALERT(0, 0, "start - getWorkLoop failed");
        return false;
    }
    
    fCommandGate = IOCommandGate::commandGate(this);
    if (!fCommandGate)
    {
        ALERT(0, 0, "start - commandGate failed");
        return false;
    }
    
    if (fWorkLoop->addEventSource(fCommandGate) != kIOReturnSuccess)
    {
        ALERT(0, 0, "start - addEventSource(commandGate) failed");
        return false;
    }
	
		// Check for an input buffer pool override first
	
	fInBufPool = 0;
	fOutBufPool = 0;
		
	bufNumber = (OSNumber *)provider->getProperty(inputTag);
    if (bufNumber)
    {
		bufValue = bufNumber->unsigned16BitValue();
		XTRACE(this, 0, bufValue, "start - Number of input buffers override value");
        if (bufValue <= kMaxInBufPool)
        {
            fInBufPool = bufValue;
        } else {
            fInBufPool = kMaxInBufPool;
        }
	} else {
		fInBufPool = 0;
	}
    
		// Now set up the real input buffer pool values (only if not overridden)
    
	if (fInBufPool == 0)
	{
		bufNumber = NULL;
		bufNumber = (OSNumber *)getProperty(inputTag);
		if (bufNumber)
		{
			bufValue = bufNumber->unsigned16BitValue();
			XTRACE(this, 0, bufValue, "start - Number of input buffers requested");
			if (bufValue <= kMaxInBufPool)
			{
				fInBufPool = bufValue;
			} else {
				fInBufPool = kMaxInBufPool;
			}
		} else {
			fInBufPool = kInBufPool;
		}
    }
	
		// Check for an output buffer pool override
		
	bufNumber = NULL;
	bufNumber = (OSNumber *)provider->getProperty(outputTag);
    if (bufNumber)
    {
		bufValue = bufNumber->unsigned16BitValue();
		XTRACE(this, 0, bufValue, "start - Number of output buffers override value");
        if (bufValue <= kMaxOutBufPool)
        {
            fOutBufPool = bufValue;
        } else {
            fOutBufPool = kMaxOutBufPool;
        }
	} else {
		fOutBufPool = 0;
	}
    
        // Now set up the real output buffer pool values (only if not overridden)
    
	if (fOutBufPool == 0)
	{
		bufNumber = NULL;
		bufNumber = (OSNumber *)getProperty(outputTag);
		if (bufNumber)
		{
			bufValue = bufNumber->unsigned16BitValue();
			XTRACE(this, 0, bufValue, "start - Number of output buffers requested");
			if (bufValue <= kMaxOutBufPool)
			{
				fOutBufPool = bufValue;
			} else {
				fOutBufPool = kMaxOutBufPool;
			}
		} else {
			fOutBufPool = kOutBufPool;
		}
	}
    
    XTRACE(this, fInBufPool, fOutBufPool, "start - Buffer pools (input, output)");
	
		// Check Reset on Close
	
	OSBoolean *boolObj = OSDynamicCast(OSBoolean, provider->getProperty("ResetOnClose"));
    if (boolObj && boolObj->isTrue())
    {
		fResetOnClose = TRUE;
        XTRACE(this, 0, 0, "start - Reset on close is on");
    } else {
		fResetOnClose = FALSE;
	}
	
		// Check Suppress Warning
	
	OSBoolean *boolObj1 = OSDynamicCast(OSBoolean, provider->getProperty("SuppressWarning"));
    if (boolObj1 && boolObj1->isTrue())
    {
		fSuppressWarning = TRUE;
        XTRACE(this, 0, 0, "start - Suppress Warning is on");
    } else {
		fSuppressWarning = FALSE;
	}
	
		// Check Enumerate on wake
	
	OSBoolean *boolObj2 = OSDynamicCast(OSBoolean, provider->getProperty("EnumerateOnWake"));
    if (boolObj2 && boolObj2->isTrue())
    {
		fEnumOnWake = TRUE;
        XTRACE(this, 0, 0, "start - Enumerate on wake is on");
    } else {
		fEnumOnWake = FALSE;
	}
	
    if (!createSerialStream())					// Publish SerialStream services
    {
        ALERT(0, 0, "start - createSerialStream failed");
        return false;
    }
    
    if (!allocateDataResources()) 
    {
        ALERT(0, 0, "start - allocateCommResources data failed");
        releaseResources();
        return false;
    }
            
        // Looks like we're ok
    
    fMSP430Interface->retain();
    fWorkLoop->retain();
    fCommandGate->enable();

	gPMWakeOnRingSymbol = OSSymbol::withCString(kIOPMSettingWakeOnRingKey);
	
	if (fConfigAttributes & kUSBAtrRemoteWakeup)
    {
		XTRACE(this, 0, 0, "start - Remote wake up is supported");
		WakeonRing();
		setWakeFeature();
		if (!setupWakeOnRingPMCallback())
		{
			XTRACE(this, 0, 0, "start - Setting the Wake on Ring callback failed");
		}
	} else {
        XTRACE(this, 0, 0, "start - Remote wake up not supported");
    }
	
		// Save the ID's
    
    fVendorID = fMSP430Interface->GetDevice()->GetVendorID();
    fProductID = fMSP430Interface->GetDevice()->GetProductID();
    
    fReady = true;
    
    if (!getFunctionalDescriptors())
    {
        ALERT(0, 0, "start - configureACM failed");
        releaseResources();
        return false;
    }
    
    if (!allocateCommResources()) 
    {
		fMSP430Interface = NULL; //rs Since we did not retain it as yet.
        ALERT(0, 0, "start - allocateCommResources failed");
		releaseResources();
        return false;
    }

    
    if (!initForPM(provider))
    {
        ALERT(0, 0, "start - initForPM failed");
		releaseResources();
        return false;
    }
    
    fMSP430Interface->retain();
    
    registerService();
    
    XTRACE(this, 0, 0, "start - successful");
	
	IOLog("%s : Version number - %s, Input buffers %d, Output buffers %d\n", getName(), drvver, fInBufPool, fOutBufPool);
    
    return true;
    	
}/* end start */

/****************************************************************************************************/
//
//		Method:		MSP430LPCDC::stop
//
//		Inputs:		provider - my provider
//
//		Outputs:	None
//
//		Desc:		Stops the driver
//
/****************************************************************************************************/

void MSP430LPCDC::stop(IOService *provider)
{
    IOReturn	ret;
    
    IOLog("%s : Stop - Unload",getName());
    
    XTRACE(this, 0, 0, "stop");
    
    fStopping = true;
    fReady = false;
    
    retain();
    ret = fCommandGate->runAction(stopAction);
    release();
	
	if (fWakeSettingControllerHandle)
	{
		fWakeSettingControllerHandle->release();
	}
	
	if (fPMRootDomain)
	{
		fPMRootDomain->deRegisterInterestedDriver(this);
	}
        
    removeProperty((const char *)propertyTag);
    
	if (!fTerminate)				// If we're terminated this has already been done
	{
		releaseResources();
	}
    
	PMstop();
    
    super::stop(provider);

}/* end stop */

/****************************************************************************************************/
//
//		Method:		MSP430LPCDC::stopAction
//
//		Desc:		Dummy pass through for stopGated.
//
/****************************************************************************************************/

IOReturn MSP430LPCDC::stopAction(OSObject *owner, void *, void *, void *, void *)
{

    ((MSP430LPCDC *)owner)->stopGated();
    
    return kIOReturnSuccess;
    
}/* end stopAction */

/****************************************************************************************************/
//
//		Method:		MSP430LPCDC::stopGated
//
//		Inputs:		
//
//		Outputs:	
//
//		Desc:		Releases the resources 
//
/****************************************************************************************************/

void MSP430LPCDC::stopGated()
{
    
    XTRACE(this, 0, 0, "stopGated");
    
    releaseResources();
	
}/* end stopGated */

/****************************************************************************************************/
//
//		Method:		MSP430LPCDC::createSuffix
//
//		Inputs:		
//
//		Outputs:	return Code - true (suffix created), false (suffix not create)
//				sufKey - the key
//
//		Desc:		Creates the suffix key. It attempts to use the serial number string from the device
//				if it's reasonable i.e. less than 8 bytes ascii. Remember it's stored in unicode 
//				format. If it's not present or not reasonable it will generate the suffix based 
//				on the location property tag. At least this remains the same across boots if the
//				device is plugged into the same physical location. In the latter case trailing
//				zeros are removed.
//				The interface number is also added to make it unique for
//				multiple CDC configuration devices.
//
/****************************************************************************************************/

bool MSP430LPCDC::createSuffix(unsigned char *sufKey)
{
    
    UInt8	serBuf[25];	

    UInt8	indx;
    bool	keyOK = false;			
	
    XTRACE(this, 0, 0, "createSuffix");
	
    indx = fMSP430Interface->GetDevice()->GetSerialNumberStringIndex();	
    
    if (indx != 0)
    {	
            // Generate suffix key based on the serial number string

        fMSP430Interface->GetDevice()->GetStringDescriptor(indx, (char *)&serBuf, sizeof(serBuf));
        strlcpy((char *)sufKey, (const char *)&serBuf, (strlen((char *)&serBuf))+1);          
        keyOK = true;  
    }
        

    return keyOK;

}/* end createSuffix */





bool MSP430LPCDC::findSerialBSDClient (IOModemSerialStreamSync *nub) 
{
	IOReturn							resultCode = kIOReturnError;
	
	bsdClientState = 0;
	
	XTRACE (this, 0, 0,"findSerialBSDClient Adding notification with custom matching dictionary");
	bsdClientAddedNotifier = addMatchingNotification (gIOFirstPublishNotification,
													  serviceMatching("IOSerialBSDClient"),
													  (IOServiceMatchingNotificationHandler)&bsdClientPublished,
													  this,
													  nub);	
	
	resultCode = fCommandGate->runAction(waitForBSDClienAction);	
	XTRACE (this, 0, resultCode, "findSerialBSDClient Exiting....");	
	if (resultCode == kIOReturnSuccess)
		return TRUE;
	else
		return FALSE;	
}

/****************************************************************************************************/
//
//		Method:		MSP430LPCDC::waitForBSDClienAction
//
//		Desc:		Dummy pass through for sendDeviceRequestGated
//
/****************************************************************************************************/

IOReturn MSP430LPCDC::waitForBSDClienAction(OSObject *owner, void *, void *, void *, void *)
{
    return ((MSP430LPCDC *)owner)->waitForBSDClientGated();
}   // end sendDeviceRequestAction


/****************************************************************************************************/
//
//		Method:		MSP430LPCDC::waitForBSDClientGated
//
//		Inputs:		
//
//		Outputs:	return Code - true that the device object appeared
//
//		Desc:		wait for the BSDClient object to be ready 
//
/****************************************************************************************************/

IOReturn MSP430LPCDC::waitForBSDClientGated()
{
    IOReturn	result = kIOReturnSuccess;
	
	AbsoluteTime	when;
	uint64_t		now,offset,whentime;
    
#define AbsoluteTime_to_scalar(x)    (*(uint64_t *)(x))
    
#define CMP_ABSOLUTETIME(t1, t2)                \
(AbsoluteTime_to_scalar(t1) >                \
AbsoluteTime_to_scalar(t2)? (int)+1 :    \
(AbsoluteTime_to_scalar(t1) <                \
AbsoluteTime_to_scalar(t2)? (int)-1 : 0))
    
    /* t1 += t2 */
#define ADD_ABSOLUTETIME(t1, t2)                \
(AbsoluteTime_to_scalar(t1) +=                \
AbsoluteTime_to_scalar(t2))
    
    /* t1 -= t2 */
#define SUB_ABSOLUTETIME(t1, t2)                \
(AbsoluteTime_to_scalar(t1) -=                \
AbsoluteTime_to_scalar(t2))
    
#define ADD_ABSOLUTETIME_TICKS(t1, ticks)        \
(AbsoluteTime_to_scalar(t1) +=                \
(int32_t)(ticks))
		
    
	now = mach_absolute_time();
	nanoseconds_to_absolutetime (9000000000ULL, &offset);	//rcs We will wait for up to 9 Seconds before timing out..
	ADD_ABSOLUTETIME (&now, &offset);	//when we timeout
	nanoseconds_to_absolutetime (now, &whentime);	//rcs We will wait for up to 9 Seconds before timing out..
    
    when = * (AbsoluteTime*) &(whentime);
    
	if (bsdClientState == 1)
	{
        IOLog( "waitForBSDClientGated - bsdClientState is already 1 no need to wait...\n"); 
		XTRACE(this, 0, 0, "waitForBSDClientGated - bsdClientState is already 1 no need to wait..."); //Sometimes the match callback gets called before this...
		return result; //no Need it was already published....
	}
	
	result = fCommandGate->commandSleep((void *) &bsdClientState, when, THREAD_INTERRUPTIBLE);
	//	result = fCommandGate->commandSleep((void *) &bsdClientState);
	
	if (result == THREAD_TIMED_OUT)
	{
		result = kIOReturnTimeout;
		XTRACE(this, 0, 0, "waitForBSDClientGated - fCommandGate returned THREAD_TIMED_OUT");
		return result;
	}
	else if (result == THREAD_INTERRUPTED)
	{
		result = kIOReturnAborted;
		XTRACE(this, 0, 0, "waitForBSDClientGated - fCommandGate returned THREAD_INTERRUPTED");
		return result;
	}
	
	XTRACE(this, 0, 0, "waitForBSDClientGated - Exit");
    return result;
}

bool MSP430LPCDC::bsdClientPublished (MSP430LPCDC * target, void * ref, IOService * newService, IONotifier * notifier) 
{
	bool	resultCode = TRUE;
	
	resultCode = FALSE;	// Assume failure
	
	XTRACE(target, 0, 0, "bsdClientPublished");
	
	if (ref == newService->getProvider()) //is the bsdclient that was just published the one we created (since they can be multiple IOBSDClient objects on any given Sunday)
	{
		XTRACE (target, 0, 0, "bsdClientPublished - waking up command gate + removing Notifier");
		notifier->remove();
		resultCode = TRUE;
		target->bsdClientState = 1;
		target->fCommandGate->commandWakeup((void *) &target->bsdClientState);
	}
	return resultCode;
}


/****************************************************************************************************/
//
//		Method:		MSP430LPCDC::createSerialStream
//
//		Inputs:		
//
//		Outputs:	return Code - true (created and initialilzed ok), false (it failed)
//
//		Desc:		Creates and initializes the nub
//
/****************************************************************************************************/

bool MSP430LPCDC::createSerialStream()
{
    IOModemSerialStreamSync	*pNub = new IOModemSerialStreamSync;
    bool			ret;
    UInt8			indx;
    IOReturn			rc;
    unsigned char		rname[25];
	OSString			*portName;
    const char			*suffix = (const char *)&rname;
//	OSBoolean			*hideProp = NULL;
	
	OSString *portSuffixString = NULL;
	OSDictionary *fInfoCommands = NULL;
	OSDictionary *hiddenProperties = NULL;
	UInt32		ttyNameSize = 0;
	char		*ttyName = NULL;
	OSString	*ttyNameStr = NULL;
//    char    	*serBuf[20];
		
    XTRACEP(this, 0, pNub, "createSerialStream");
    
    if (!pNub)
    {
		XTRACEP(this, 0, pNub, "createSerialStream - Could not create serial stream");
        return false;
    }
		
    	// Either we attached and should get rid of our reference
    	// or we failed in which case we should get rid our reference as well.
        // This just makes sure the reference count is correct.
	
    ret = (pNub->init(0, 0) && pNub->attach(this));
	
    pNub->release();
    if (!ret)
    {
        XTRACE(this, ret, 0, "createSerialStream - Failed to attach to the nub");
        return false;
    }

	if (fWanDevice == kOSBooleanTrue)
		pNub->setProperty("WWAN", true);
	else
	{
		XTRACE(this, 0, 0, "createSerialStream - NON WAN CDC Device");
	}
	

		// Get the name from the InterfaceMapping dictionary.
		
	portName = getPortNameForInterface(fMSP430Interface->GetInterfaceNumber());
    
	if (portName != NULL)
	{
		pNub->setProperty(kIOTTYBaseNameKey, portName->getCStringNoCopy());
		if (!(portName->isEqualTo("wwan")))
		{
				pNub->setProperty((const char *)hiddenTag, true);
				pNub->setProperty((const char *)WWANTag, true);
		}
	} else {	
			// Report the base name to be used for generating device nodes
        
        pNub->setProperty(kIOTTYBaseNameKey, "uart-");
		XTRACE(this, 0, fMSP430Interface->GetInterfaceNumber(), "createSerialStream - using default naming and suffix...");
	
			// Create suffix key and set it
	
		if (createSuffix((unsigned char *)suffix))
		{	            
			pNub->setProperty(kIOTTYSuffixKey, suffix);
		}
	}
    
    pNub->registerService();
	
	XTRACE(this, 0, 0, "createSerialStream - wait for a sec...");
	if (!findSerialBSDClient(pNub))
	{
		XTRACE (this, 0, 0, "createSerialStream - findSerialBSDClient failed terminating nub");
		pNub->close(this);
		XTRACE (this, 0, 0, "createSerialStream - findSerialBSDClient returning false");
		return false;
	}
	
    fInfoCommands = (OSDictionary *) fMSP430Interface->GetDevice()->getProperty("InfoCommands");
	
	if ( (fInfoCommands != NULL) && (portName != NULL) )
	{
		hiddenProperties = (OSDictionary *) fInfoCommands->getObject("HiddenProperties");
		if (hiddenProperties)
		{
			portSuffixString = (OSSymbol *) pNub->copyProperty(kIOTTYSuffixKey);
			
			if (portSuffixString != NULL)
			{
				if ( (portSuffixString->getCStringNoCopy() != NULL) )
				{
						OSCollectionIterator *propertyIterator;		
						propertyIterator = OSCollectionIterator::withCollection( hiddenProperties);
						
						if ( propertyIterator != NULL )
						{
							OSString *key;
							propertyIterator->reset();
							
							while( (key = (OSString *)propertyIterator->getNextObject()))
							{
								OSString *value;
								value = (OSString *) hiddenProperties->getObject(key);
																
								if (value->isEqualTo(portName))
								{
									ttyNameSize = (portSuffixString->getLength() + portName->getLength() );						
									ttyName = (char *)IOMallocAligned(ttyNameSize+4, sizeof (char));
									bzero(ttyName,ttyNameSize+4);
									strncpy(ttyName, value->getCStringNoCopy(), value->getLength());
									strncat(ttyName, portSuffixString->getCStringNoCopy(), portSuffixString->getLength());
									
									ttyNameStr = OSString::withCString(ttyName);
									if ( ttyNameStr != NULL )
									{
										//OSString *foo;
										XTRACE(this, 0, 0, "createSerialStream - hiddenProperties: collision");
										fMSP430Interface->GetDevice()->setProperty(key->getCStringNoCopy(),ttyNameStr);
										//hiddenProperties->setObject(key->getCStringNoCopy(),ttyNameStr);
										
										//foo = (OSString *)hiddenProperties->getObject(key->getCStringNoCopy());
										//hiddenProperties->setObject(foo,ttyNameStr);
									}
									}
							} 		
							propertyIterator->release();
						}	else { XTRACE(this, 0, 0, "createSerialStream - propertyIterator is NULL...");}		
				} else { XTRACE(this, 0, 0, "createSerialStream - portSuffixString->getCStringNoCopy is NULL...");}
			} else { XTRACE(this, 0, 0, "createSerialStream - portSuffixString is NULL...");}
		} else { XTRACE(this, 0, 0, "createSerialStream - hiddenProperties is NULL...");}
	} else { XTRACE(this, 0, fMSP430Interface->GetInterfaceNumber(), "createSerialStream - fInfoCommands or portname is NULL...");}
	
    // Save the Product String (at least the first productNameLength's worth).

    indx = fMSP430Interface->GetDevice()->GetProductStringIndex();	
    
    if (indx != 0)
    {	
        rc = fMSP430Interface->GetDevice()->GetStringDescriptor(indx, (char *)&fProductName, sizeof(fProductName));
        if (!rc)
        {
            if (strlen((char *)fProductName) == 0)		// Believe it or not this sometimes happens - null string with an index defined???
            {
				strlcpy((char *)fProductName, defaultName, sizeof(defaultName));
//                strcpy((char *)fProductName, defaultName);
            }
            pNub->setProperty((const char *)propertyTag, (const char *)fProductName);
        }
    }

    return true;
	
}/* end createSerialStream */

/****************************************************************************************************/
//
//		Method:		MSP430LPCDC::acquirePort
//
//		Inputs:		sleep - true (wait for it), false (don't)
//				refCon - unused
//
//		Outputs:	Return Code - kIOReturnSuccess, kIOReturnExclusiveAccess, kIOReturnIOError and various others
//
//		Desc:		Set up for gated acquirePort call.
//
/****************************************************************************************************/

IOReturn MSP430LPCDC::acquirePort(bool sleep, void *refCon)
{
    IOReturn                ret;
    
    XTRACEP(this, refCon, 0, "acquirePort");
	
		// Check for being acquired after stop has been issued and before start
		
	if (fTerminate || fStopping)
	{
		XTRACE(this, 0, 0, "acquirePort - Offline");
		return kIOReturnOffline;
	}
	
		// Make sure we have a valid workloop
	
	if (!fWorkLoop)
    {
        XTRACE(this, 0, 0, "acquirePort - No workLoop");
        return kIOReturnOffline;
    }
    
        // Make sure start has finished (could be different threads)
    
    if (!fReady)
    {
        XTRACE(this, 0, 0, "acquirePort - Not ready");
		return kIOReturnNotReady;
    }
    
        // Find the matching control driver first (we're obviously not ready if he hasn't arrived)
        
    if (!fControlDriver)
    {
        fControlDriver = findControlDriverAD(this);
        if (fControlDriver == NULL)
        {
            XTRACE(this, 0, 0, "acquirePort - Cannot find control driver");
            return kIOReturnNotReady;
        }
    }

    retain();
    ret = fCommandGate->runAction(acquirePortAction, (void *)sleep);
    release();

    return ret;

}/* end acquirePort */

/****************************************************************************************************/
//
//		Method:		MSP430LPCDC::acquirePortAction
//
//		Desc:		Dummy pass through for acquirePortGated.
//
/****************************************************************************************************/

IOReturn MSP430LPCDC::acquirePortAction(OSObject *owner, void *arg0, void *, void *, void *)
{

    return ((MSP430LPCDC *)owner)->acquirePortGated((bool)arg0);
    
}/* end acquirePortAction */

/****************************************************************************************************/
//
//		Method:		MSP430LPCDC::acquirePortGated
//
//		Inputs:		sleep - true (wait for it), false (don't)
//
//		Outputs:	Return Code - kIOReturnSuccess, kIOReturnExclusiveAccess, kIOReturnIOError and various others
//
//		Desc:		acquirePort tests and sets the state of the port object.  If the port was
// 				available, then the state is set to busy, and kIOReturnSuccess is returned.
// 				If the port was already busy and sleep is YES, then the thread will sleep
// 				until the port is freed, then re-attempts the acquire.  If the port was
// 				already busy and sleep is NO, then kIOReturnExclusiveAccess is returned.
//
/****************************************************************************************************/

IOReturn MSP430LPCDC::acquirePortGated(bool sleep)
{
    UInt32 	busyState = 0;
    IOReturn 	rtn = kIOReturnSuccess;
    UInt16	i;
	UInt32	state;
	UInt32	mask;

    XTRACE(this, 0, sleep, "acquirePortGated");

    retain(); 							// Hold reference till releasePortGated, unless we fail to acquire
    while (true)
    {
        busyState = fPort.State & PD_S_ACQUIRED;
        if (!busyState)
        {		
                // Set busy bit (acquired), and clear everything else
                
			state = PD_S_ACQUIRED | DEFAULT_STATE;
			mask = STATE_ALL;
            setStateGated(&state, &mask);
            break;
        } else {
            if (!sleep)
            {
                XTRACE(this, 0, 0, "acquirePortGated - Busy exclusive access");
                release();
            	return kIOReturnExclusiveAccess;
            } else {
            	busyState = 0;
				mask = PD_S_ACQUIRED;
            	rtn = watchStateGated(&busyState, &mask);
            	if ((rtn == kIOReturnIOError) || (rtn == kIOReturnSuccess))
                {
                    continue;
            	} else {
                    XTRACE(this, 0, 0, "acquirePortGated - Interrupted!");
                    release();
                    return rtn;
                }
            }
        }
    }
    
    do
    {    	
        setStructureDefaults();				// Set the default values
        
            // Set up and read the data-in bulk pipe
        
        for (i=0; i<fInBufPool; i++)
        {
            if (fPort.inPool[i].pipeMDP)
            {
                fPort.inPool[i].completionInfo.target = this;
                fPort.inPool[i].completionInfo.action = dataReadComplete;
                fPort.inPool[i].completionInfo.parameter = (void *)&fPort.inPool[i];
                rtn = fPort.InPipe->Read(fPort.inPool[i].pipeMDP, &fPort.inPool[i].completionInfo, NULL);
                if (rtn != kIOReturnSuccess)
                {
                    XTRACE(this, i, rtn, "acquirePortGated - Read for bulk-in pipe failed");
					fPort.inPool[i].dead = true;
                    break;
                }
				XTRACEP(this, &fPort.inPool[i], fPort.InPipe, "acquirePortGated - Read posted");
            }
        }
        if (rtn == kIOReturnSuccess)
        {
        
                // Set up the data-out bulk pipe
		
            for (i=0; i<fOutBufPool; i++)
            {
                if (fPort.outPool[i].pipeMDP)
                {
                    fPort.outPool[i].completionInfo.target = this;
                    fPort.outPool[i].completionInfo.action = dataWriteComplete;
                    fPort.outPool[i].completionInfo.parameter = (void *)&fPort.outPool[i];
                }
            }
        } else {
            break;
        }

        fSessions++;					// Bump number of active sessions and turn on clear to send
		state = PD_RS232_S_CTS;
		mask = PD_RS232_S_CTS;
		setStateGated(&state, &mask);
        
            // Tell the Control driver we're good to go
        
		if (fControlDriver)
		{
			if (!fControlDriver->dataAcquired())
			{
				XTRACE(this, 0, 0, "acquirePortGated - dataAcquired to Control failed");
				break;
			}
		}
        
        return kIOReturnSuccess;
        
    } while (0);

    	// We failed for some reason

	state = 0;
	mask = STATE_ALL;
	setStateGated(&state, &mask);			// Clear the entire state

    release();
    
    return rtn;
	
}/* end acquirePortGated */

/****************************************************************************************************/
//
//		Method:		MSP430LPCDC::releasePort
//
//		Inputs:		refCon - unused
//
//		Outputs:	Return Code - kIOReturnSuccess or kIOReturnNotOpen
//
//		Desc:		Set up for gated releasePort call.
//
/****************************************************************************************************/

IOReturn MSP430LPCDC::releasePort(void *refCon)
{
    IOReturn	ret = kIOReturnSuccess;
    
    XTRACE(this, 0, 0, "releasePort");
	
        // Abort any outstanding I/O (only if we're not terminated)

	if (!fTerminate)
	{
		if (fPort.InPipe)
			fPort.InPipe->Abort();
		if (fPort.OutPipe)
			fPort.OutPipe->Abort();
	}

//	IOSleep(10);
	
    retain();
    ret = fCommandGate->runAction(releasePortAction);
    release();

        // Check the pipes before we leave (only if we're not terminated)
		// and Reset on Close is true. This resets the data toggle on both ends
   
    if (!fTerminate)
    {
		if (fResetOnClose)
		{
			if (fPort.InPipe)
				checkPipe(fPort.InPipe, true);
    
			if (fPort.OutPipe)
				checkPipe(fPort.OutPipe, true);

			if (fMSP430Interface)
			{
				ret = fMSP430Interface->GetDevice()->ResetDevice();
				if (ret != kIOReturnSuccess)
				{
					XTRACE(this, 0, ret, "releasePort - ResetDevice failed");
				}
			}
		}
    } else {
        clearSleepingThreads();
    }
        
    return ret;

}/* end releasePort */

/****************************************************************************************************/
//
//		Method:		MSP430LPCDC::releasePortAction
//
//		Desc:		Dummy pass through for releasePortGated.
//
/****************************************************************************************************/

IOReturn MSP430LPCDC::releasePortAction(OSObject *owner, void *, void *, void *, void *)
{

    return ((MSP430LPCDC *)owner)->releasePortGated();
    
}/* end releasePortAction */

/****************************************************************************************************/
//
//		Method:		MSP430LPCDC::releasePortGated
//
//		Inputs:		
//
//		Outputs:	Return Code - kIOReturnSuccess or kIOReturnNotOpen
//
//		Desc:		releasePort returns all the resources and does clean up.
//
/****************************************************************************************************/

IOReturn MSP430LPCDC::releasePortGated()
{
    UInt32 	busyState;
	UInt32	state;
	UInt32	mask;

    XTRACE(this, 0, 0, "releasePortGated");
    
    busyState = (fPort.State & PD_S_ACQUIRED);
    if (!busyState)
    {
        if (fTerminate || fStopping)
        {
            XTRACE(this, 0, 0, "releasePortGated - Offline");
            return kIOReturnOffline;
        }
        
        XTRACE(this, 0, 0, "releasePortGated - Not open");
        return kIOReturnNotOpen;
    }
	
    if (!fTerminate)
        setControlLineState(false, false);		// clear RTS and clear DTR only if not terminated
	
	state = 0;
	mask = STATE_ALL;
	setStateGated(&state, &mask);				// Clear the entire state word - which also deactivates the port

//#if 0    
        // Abort any outstanding I/O
        
    if (fPort.InPipe)
        fPort.InPipe->Abort();
    if (fPort.OutPipe)
        fPort.OutPipe->Abort();
//#endif
        
        // Tell the Control driver the port's been released, only when not terminated (control driver may already be gone)
		
	if (!fTerminate)
	{
		if (fControlDriver)
		{
			fControlDriver->dataReleased();
		}
	}
	    
    fSessions--;					// reduce number of active sessions
            
    release(); 						// Dispose of the self-reference we took in acquirePortGated()
    
    XTRACE(this, 0, 0, "releasePort - Exit");
    
    return kIOReturnSuccess;
	
}/* end releasePortGated */

/****************************************************************************************************/
//
//		Method:		MSP430LPCDC::getState
//
//		Inputs:		refCon - unused
//
//		Outputs:	Return value - port state
//
//		Desc:		Set up for gated getState call.
//
/****************************************************************************************************/

UInt32 MSP430LPCDC::getState(void *refCon)
{
    UInt32	currState;
    
    XTRACE(this, 0, 0, "getState");
    
    if (fTerminate || fStopping)
    {
        XTRACE(this, 0, kIOReturnOffline, "getState - Offline");
        return 0;
    }
    
    retain();
    currState = fCommandGate->runAction(getStateAction);
    release();
    
    return currState;
    
}/* end getState */

/****************************************************************************************************/
//
//		Method:		MSP430LPCDC::getStateAction
//
//		Desc:		Dummy pass through for getStateGated.
//
/****************************************************************************************************/

IOReturn MSP430LPCDC::getStateAction(OSObject *owner, void *, void *, void *, void *)
{
    UInt32	newState;

    newState = ((MSP430LPCDC *)owner)->getStateGated();
    
    return newState;
    
}/* end getStateAction */

/****************************************************************************************************/
//
//		Method:		MSP430LPCDC::getStateGated
//
//		Inputs:		port - unused
//
//		Outputs:	return value - port state
//
//		Desc:		Get the state for the port.
//
/****************************************************************************************************/

UInt32 MSP430LPCDC::getStateGated()
{
    UInt32 	state;
	
    XTRACE(this, 0, 0, "getStateGated");
    
    if (fTerminate || fStopping)
        return 0;
	
    CheckQueues();
	
    state = fPort.State & EXTERNAL_MASK;
	
    XTRACE(this, state, EXTERNAL_MASK, "getStateGated - Exit");
	
    return state;
	
}/* end getStateGated */

/****************************************************************************************************/
//
//		Method:		MSP430LPCDC::setState
//
//		Inputs:		state - the state
//				mask - the mask
//				refCon - unused
//
//		Outputs:	Return code - kIOReturnSuccess or kIOReturnBadArgument
//
//		Desc:		Set up for gated setState call.
//
/****************************************************************************************************/

IOReturn MSP430LPCDC::setState(UInt32 state, UInt32 mask, void *refCon)
{
    IOReturn	ret = kIOReturnSuccess;
    
    XTRACE(this, 0, 0, "setState");
    
    if (fTerminate || fStopping)
    {
        XTRACE(this, 0, kIOReturnOffline, "setState - Offline");
        return 0;
    }
    
        // Cannot acquire or activate via setState
    
    if (mask & (PD_S_ACQUIRED | PD_S_ACTIVE | (~EXTERNAL_MASK)))
    {
        ret = kIOReturnBadArgument;
    } else {
        
            // ignore any bits that are read-only
        
        mask &= (~fPort.FlowControl & PD_RS232_A_MASK) | PD_S_MASK;
        if (mask)
        {
            retain();
            ret = fCommandGate->runAction(setStateAction, (void *)&state, (void *)&mask);
            release();
        }
    }
    
    return ret;
    
}/* end setState */

/****************************************************************************************************/
//
//		Method:		MSP430LPCDC::setStateAction
//
//		Desc:		Dummy pass through for setStateGated.
//
/****************************************************************************************************/

IOReturn MSP430LPCDC::setStateAction(OSObject *owner, void *arg0, void *arg1, void *, void *)
{

    return ((MSP430LPCDC *)owner)->setStateGated((UInt32 *)arg0, (UInt32 *)arg1);
    
}/* end setStateAction */

/****************************************************************************************************/
//
//		Method:		MSP430LPCDC::setStateGated
//
//		Inputs:		state - state to set
//				mask - state mask
//
//		Outputs:	Return Code - kIOReturnSuccess or kIOReturnBadArgument
//
//		Desc:		Set the state for the port device.  The lower 16 bits are used to set the
//				state of various flow control bits (this can also be done by enqueueing a
//				PD_E_FLOW_CONTROL event).  If any of the flow control bits have been set
//				for automatic control, then they can't be changed by setState.  For flow
//				control bits set to manual (that are implemented in hardware), the lines
//				will be changed before this method returns.  The one weird case is if RXO
//				is set for manual, then an XON or XOFF character may be placed at the end
//				of the TXQ and transmitted later.
//
/****************************************************************************************************/

IOReturn MSP430LPCDC::setStateGated(UInt32 *pState, UInt32 *pMask)
{
	UInt32	state = *pState;
	UInt32	mask = *pMask;
    UInt32	delta;
	bool	controlUpdate = false;
	UInt32	DTRstate;
	UInt32	RTSstate;
	bool	DTRnew = false;
	bool	RTSnew = false;
	
    XTRACE(this, state, mask, "setStateGated");
    
    if (fStopping)
        return kIOReturnOffline;
    
        // Check if it's being acquired or already acquired

    if ((state & PD_S_ACQUIRED) || (fPort.State & PD_S_ACQUIRED))
    {
		XTRACE(this, state, mask, "setState - Requested state and mask");
		XTRACE(this, 0, fPort.State, "setState - Current state");
		DTRstate = fPort.State & PD_RS232_S_DTR;
		RTSstate = fPort.State & PD_RS232_S_RTS;
		XTRACE(this, DTRstate, RTSstate, "setState - DTRstate and RTSstate");
		
			// Set the new state based on the current setting
			
		if (fPort.State & PD_RS232_S_DTR)
		{
			DTRnew = true;
		}
		if (fPort.State & PD_RS232_S_RTS)
		{
			RTSnew = true;
		}
		XTRACE(this, DTRnew, RTSnew, "setState - DTRstate and RTSstate");
		
			// Handle DTR and RTS changes for the modem
		
        if (mask & PD_RS232_S_DTR)
        {
            if ((state & PD_RS232_S_DTR) != (fPort.State & PD_RS232_S_DTR))
            {
				controlUpdate = true;
                if (state & PD_RS232_S_DTR)
                {
                    XTRACE(this, 0, 0, "setState - Changing DTR to ON");
					DTRnew = true;
                } else {
					XTRACE(this, 0, 0, "setState - Changing DTR to OFF");
					DTRnew = false;
                }
            } else {
				XTRACE(this, 0, DTRstate, "setState - DTR state unchanged");
			}
        }
		if (mask & PD_RS232_S_RTS)
		{
			if ((state & PD_RS232_S_RTS) != (fPort.State & PD_RS232_S_RTS))
            {
				controlUpdate = true;
                if (state & PD_RS232_S_RTS)
                {
                    XTRACE(this, 0, 0, "setState - Changing RTS to ON");
					RTSnew = true;
                } else {
					XTRACE(this, 0, 0, "setState - Changing RTS to OFF");
					RTSnew = false;
                }
            } else {
				XTRACE(this, 0, RTSstate, "setState - RTS state unchanged");
			}
		}
		
		XTRACE(this, DTRnew, RTSnew, "setState - DTRnew and RTSnew");
		
		if ((!fTerminate) && (controlUpdate))
		{
			setControlLineState(RTSnew, DTRnew);
		}
        
        state = (fPort.State & ~mask) | (state & mask); 		// compute the new state
        delta = state ^ fPort.State;		    			// keep a copy of the diffs
        fPort.State = state;

	    // Wake up all threads asleep on WatchStateMask
		
        if (delta & fPort.WatchStateMask)
        {
            fCommandGate->commandWakeup((void *)&fPort.State);
        }
        
        return kIOReturnSuccess;

    } else {
        XTRACE(this, fPort.State, 0, "setStateGated - Not Acquired");
    }
    
    return kIOReturnNotOpen;
	
}/* end setStateGated */

/****************************************************************************************************/
//
//		Method:		MSP430LPCDC::watchState
//
//		Inputs:		state - state to watch for
//				mask - state mask bits
//				refCon - unused
//
//		Outputs:	Return Code - kIOReturnSuccess or value returned from ::watchState
//
//		Desc:		Set up for gated watchState call.
//
/****************************************************************************************************/

IOReturn MSP430LPCDC::watchState(UInt32 *state, UInt32 mask, void *refCon)
{
    IOReturn 	ret;

    XTRACE(this, *state, mask, "watchState");
    
    if (fTerminate || fStopping)
    {
        XTRACE(this, 0, kIOReturnOffline, "watchState - Offline");
        return kIOReturnOffline;
    }
    
    if (!state) 
        return kIOReturnBadArgument;
        
    if (!mask)
        return kIOReturnSuccess;

    retain();
    ret = fCommandGate->runAction(watchStateAction, (void *)state, (void *)&mask);
    release();
    
    return ret;

}/* end watchState */

/****************************************************************************************************/
//
//		Method:		MSP430LPCDC::watchStateAction
//
//		Desc:		Dummy pass through for watchStateGated.
//
/****************************************************************************************************/

IOReturn MSP430LPCDC::watchStateAction(OSObject *owner, void *arg0, void *arg1, void *, void *)
{

    return ((MSP430LPCDC *)owner)->watchStateGated((UInt32 *)arg0, (UInt32 *)arg1);
    
}/* end watchStateAction */

/****************************************************************************************************/
//
//		Method:		MSP430LPCDC::watchStateGated
//
//		Inputs:		state - state to watch for
//				mask - state mask bits
//
//		Outputs:	Return Code - kIOReturnSuccess or value returned from privateWatchState
//
//		Desc:		Wait for the at least one of the state bits defined in mask to be equal
//				to the value defined in state. Check on entry then sleep until necessary,
//				A return value of kIOReturnSuccess means that at least one of the port state
//				bits specified by mask is equal to the value passed in by state.  A return
//				value of kIOReturnIOError indicates that the port went inactive.  A return
//				value of kIOReturnIPCError indicates sleep was interrupted by a signal.
//
/****************************************************************************************************/

IOReturn MSP430LPCDC::watchStateGated(UInt32 *pState, UInt32 *pMask)
{
	UInt32		mask = *pMask;
    UInt32		watchState, foundStates;
    bool		autoActiveBit = false;
    IOReturn	ret = kIOReturnNotOpen;

    XTRACE(this, *pState, mask, "watchStateGated");
    
    if (fTerminate || fStopping)
        return kIOReturnOffline;
        
    if (fPort.State & PD_S_ACQUIRED)
    {
        ret = kIOReturnSuccess;
        mask &= EXTERNAL_MASK;
        
        watchState = *pState;
        if (!(mask & (PD_S_ACQUIRED | PD_S_ACTIVE)))
        {
            watchState &= ~PD_S_ACTIVE;				// Check for low PD_S_ACTIVE
            mask |=  PD_S_ACTIVE;				// Register interest in PD_S_ACTIVE bit
            autoActiveBit = true;
        }

        while (true)
        {
                // Check port state for any interesting bits with watchState value
                // NB. the '^ ~' is a XNOR and tests for equality of bits.
			
            foundStates = (watchState ^ ~fPort.State) & mask;

            if (foundStates)
            {
                *pState = fPort.State;
                if (autoActiveBit && (foundStates & PD_S_ACTIVE))
                {
                    ret = kIOReturnIOError;
                } else {
                    ret = kIOReturnSuccess;
                }
                break;
            }

                // Everytime we go around the loop we have to reset the watch mask.
                // This means any event that could affect the WatchStateMask must
                // wakeup all watch state threads.  The two events are an interrupt
                // or one of the bits in the WatchStateMask changing.
			
            fPort.WatchStateMask |= mask;
            
            XTRACE(this, fPort.State, fPort.WatchStateMask, "watchStateGated - Thread sleeping");
            
            retain();								// Just to make sure all threads are awake
            fCommandGate->retain();					// before we're released
        
            fThreadSleepCount++;
            
            ret = fCommandGate->commandSleep((void *)&fPort.State);
            
            fThreadSleepCount--;
        
            fCommandGate->release();
            
            XTRACE(this, fPort.State, ret, "watchStateGated - Thread restart");

            if (ret == THREAD_TIMED_OUT)
            {
                ret = kIOReturnTimeout;
				release();
                break;
            } else {
                if (ret == THREAD_INTERRUPTED)
                {
                    ret = kIOReturnAborted;
					release();
                    break;
                } else {
					if (fTerminate || fStopping)	// Make sure we not terminated or stopping
					{
						ret = kIOReturnOffline;
						release();
						break;
					}
				}
            }
            release();
        }       
        
            // As it is impossible to undo the masking used by this
            // thread, we clear down the watch state mask and wakeup
            // every sleeping thread to reinitialize the mask before exiting.
		
        fPort.WatchStateMask = 0;
        XTRACE(this, *pState, 0, "watchStateGated - Thread wakeing others");
        fCommandGate->commandWakeup((void *)&fPort.State);
 
        *pState &= EXTERNAL_MASK;
    }
	
    XTRACE(this, ret, 0, "watchState - Exit");
    
    return ret;
	
}/* end watchStateGated */

/****************************************************************************************************/
//
//		Method:		MSP430LPCDC::nextEvent
//
//		Inputs:		refCon - unused
//
//		Outputs:	Return Code - kIOReturnSuccess or kIOReturnOffline
//
//		Desc:		Not used by this driver.
//
/****************************************************************************************************/

UInt32 MSP430LPCDC::nextEvent(void *refCon)
{

    XTRACE(this, 0, 0, "nextEvent");
    
    if (fTerminate || fStopping)
        return kIOReturnOffline;
        
    if (getState(&fPort) & PD_S_ACTIVE)
    {
        return kIOReturnSuccess;
    }

    return kIOReturnNotOpen;
	
}/* end nextEvent */

/****************************************************************************************************/
//
//		Method:		MSP430LPCDC::executeEvent
//
//		Inputs:		event - The event
//				data - any data associated with the event
//				refCon - unused
//
//		Outputs:	Return Code - kIOReturnSuccess, kIOReturnNotOpen or kIOReturnBadArgument
//
//		Desc:		Set up for gated executeEvent call.
//
/****************************************************************************************************/

IOReturn MSP430LPCDC::executeEvent(UInt32 event, UInt32 data, void *refCon)
{
    IOReturn 	ret;
    
    XTRACE(this, data, event, "executeEvent");
    
    if (fTerminate || fStopping)
    {
        XTRACE(this, 0, kIOReturnOffline, "executeEvent - Offline");
        return kIOReturnOffline;
    }
    
    retain();
    ret = fCommandGate->runAction(executeEventAction, (void *)&event, (void *)&data);
    release();

    return ret;
    
}/* end executeEvent */

/****************************************************************************************************/
//
//		Method:		MSP430LPCDC::executeEventAction
//
//		Desc:		Dummy pass through for executeEventGated.
//
/****************************************************************************************************/

IOReturn MSP430LPCDC::executeEventAction(OSObject *owner, void *arg0, void *arg1, void *, void *)
{

    return ((MSP430LPCDC *)owner)->executeEventGated((UInt32 *)arg0, (UInt32 *)arg1);
    
}/* end executeEventAction */

/****************************************************************************************************/
//
//		Method:		MSP430LPCDC::executeEventGated
//
//		Inputs:		event - The event
//				data - any data associated with the event
//
//		Outputs:	Return Code - kIOReturnSuccess, kIOReturnNotOpen or kIOReturnBadArgument
//
//		Desc:		executeEvent causes the specified event to be processed immediately.
//				This is primarily used for channel control commands like START & STOP
//
/****************************************************************************************************/

IOReturn MSP430LPCDC::executeEventGated(UInt32 *pEvent, UInt32 *pData)
{
	UInt32		event = *pEvent;
	UInt32		data = *pData;
    IOReturn	ret = kIOReturnSuccess;
    UInt32		state, delta;
	UInt32		nState;
	UInt32		mask;
	
    if (fTerminate || fStopping)
        return kIOReturnOffline;
        
    delta = 0;
    state = fPort.State;	
    XTRACE(this, state, event, "executeEventGated");
	
    if ((state & PD_S_ACQUIRED) == 0)
        return kIOReturnNotOpen;

    switch (event)
    {
	case PD_RS232_E_XON_BYTE:
            XTRACE(this, data, event, "executeEventGated - PD_RS232_E_XON_BYTE");
            fPort.XONchar = data;
            break;
	case PD_RS232_E_XOFF_BYTE:
            XTRACE(this, data, event, "executeEventGated - PD_RS232_E_XOFF_BYTE");
            fPort.XOFFchar = data;
            break;
	case PD_E_SPECIAL_BYTE:
            XTRACE(this, data, event, "executeEventGated - PD_E_SPECIAL_BYTE");
            fPort.SWspecial[ data >> SPECIAL_SHIFT ] |= (1 << (data & SPECIAL_MASK));
            break;
	case PD_E_VALID_DATA_BYTE:
            XTRACE(this, data, event, "executeEventGated - PD_E_VALID_DATA_BYTE");
            fPort.SWspecial[ data >> SPECIAL_SHIFT ] &= ~(1 << (data & SPECIAL_MASK));
            break;
	case PD_E_FLOW_CONTROL:
            XTRACE(this, data, event, "executeEventGated - PD_E_FLOW_CONTROL");
            break;
	case PD_E_ACTIVE:
            XTRACE(this, data, event, "executeEventGated - PD_E_ACTIVE");
            if ((bool)data)
            {
                if (!(state & PD_S_ACTIVE))
                {
                    setStructureDefaults();
					nState = PD_S_ACTIVE;
					mask = PD_S_ACTIVE;
                    setStateGated(&nState, &mask); 			// activate port
					
					nState = PD_RS232_S_RTS;
					mask = PD_RS232_S_RTS;
					setStateGated(&nState, &mask);
					
					nState = PD_RS232_S_DTR;
					mask = PD_RS232_S_DTR;
					setStateGated(&nState, &mask);
					
 //                   setControlLineState(true, true);						// set RTS and set DTR
                }
            } else {
                if ((state & PD_S_ACTIVE))
                {
					nState = 0;
					mask = PD_S_ACTIVE;
                    setStateGated(&nState, &mask);					// deactivate port
				
                    setControlLineState(false, false);						// clear RTS and clear DTR
                }
            }
            break;
	case PD_E_DATA_LATENCY:
            XTRACE(this, data, event, "executeEventGated - PD_E_DATA_LATENCY");
            fPort.DataLatInterval = long2tval(data * 1000);
            break;
	case PD_RS232_E_MIN_LATENCY:
            XTRACE(this, data, event, "executeEventGated - PD_RS232_E_MIN_LATENCY");
            fPort.MinLatency = bool(data);
            break;
	case PD_E_DATA_INTEGRITY:
            XTRACE(this, data, event, "executeEventGated - PD_E_DATA_INTEGRITY");
            if ((data < PD_RS232_PARITY_NONE) || (data > PD_RS232_PARITY_SPACE))
            {
                ret = kIOReturnBadArgument;
            } else {
                fPort.TX_Parity = data;
                fPort.RX_Parity = PD_RS232_PARITY_DEFAULT;
			
                setLineCoding();			
            }
            break;
	case PD_E_DATA_RATE:
            XTRACE(this, data, event, "executeEventGated - PD_E_DATA_RATE");
            
                // For API compatiblilty with Intel.
                
            data >>= 1;
            XTRACE(this, data, 0, "executeEventGated - actual data rate");
            if ((data < MIN_BAUD) || (data > kMaxBaudRate))
            {
                ret = kIOReturnBadArgument;
            } else {
                fPort.BaudRate = data;
			
                setLineCoding();			
            }		
            break;
	case PD_E_DATA_SIZE:
            XTRACE(this, data, event, "executeEventGated - PD_E_DATA_SIZE");
            
                // For API compatiblilty with Intel.
                
            data >>= 1;
            XTRACE(this, data, 0, "executeEventGated - actual data size");
            if ((data < 5) || (data > 8))
            {
                ret = kIOReturnBadArgument;
            } else {
                fPort.CharLength = data;
			
                setLineCoding();			
            }
            break;
	case PD_RS232_E_STOP_BITS:
            XTRACE(this, data, event, "executeEventGated - PD_RS232_E_STOP_BITS");
            if ((data < 0) || (data > 20))
            {
                ret = kIOReturnBadArgument;
            } else {
                fPort.StopBits = data;
			
                setLineCoding();
            }
            break;
	case PD_E_RXQ_FLUSH:
            XTRACE(this, data, event, "executeEventGated - PD_E_RXQ_FLUSH");
            break;
	case PD_E_RX_DATA_INTEGRITY:
            XTRACE(this, data, event, "executeEventGated - PD_E_RX_DATA_INTEGRITY");
            if ((data != PD_RS232_PARITY_DEFAULT) &&  (data != PD_RS232_PARITY_ANY))
            {
                ret = kIOReturnBadArgument;
            } else {
                fPort.RX_Parity = data;
            }
            break;
	case PD_E_RX_DATA_RATE:
            XTRACE(this, data, event, "executeEventGated - PD_E_RX_DATA_RATE");
            if (data)
            {
                ret = kIOReturnBadArgument;
            }
            break;
	case PD_E_RX_DATA_SIZE:
            XTRACE(this, data, event, "executeEventGated - PD_E_RX_DATA_SIZE");
            if (data)
            {
                ret = kIOReturnBadArgument;
            }
            break;
	case PD_RS232_E_RX_STOP_BITS:
            XTRACE(this, data, event, "executeEventGated - PD_RS232_E_RX_STOP_BITS");
            if (data)
            {
                ret = kIOReturnBadArgument;
            }
            break;
	case PD_E_TXQ_FLUSH:
            XTRACE(this, data, event, "executeEventGated - PD_E_TXQ_FLUSH");
            break;
	case PD_RS232_E_LINE_BREAK:
            XTRACE(this, data, event, "executeEventGated - PD_RS232_E_LINE_BREAK");
            state &= ~PD_RS232_S_BRK;
            delta |= PD_RS232_S_BRK;
            setStateGated(&state, &delta);
			if (!fTerminate)
			{
				sendBreak((bool)data);
			}
            break;
	case PD_E_DELAY:
            XTRACE(this, data, event, "executeEventGated - PD_E_DELAY");
            fPort.CharLatInterval = long2tval(data * 1000);
            break;
	case PD_E_RXQ_SIZE:
            XTRACE(this, data, event, "executeEventGated - PD_E_RXQ_SIZE");
            break;
	case PD_E_TXQ_SIZE:
            XTRACE(this, data, event, "executeEventGated - PD_E_TXQ_SIZE");
            break;
	case PD_E_RXQ_HIGH_WATER:
            XTRACE(this, data, event, "executeEventGated - PD_E_RXQ_HIGH_WATER");
            break;
	case PD_E_RXQ_LOW_WATER:
            XTRACE(this, data, event, "executeEventGated - PD_E_RXQ_LOW_WATER");
            break;
	case PD_E_TXQ_HIGH_WATER:
            XTRACE(this, data, event, "executeEventGated - PD_E_TXQ_HIGH_WATER");
            break;
	case PD_E_TXQ_LOW_WATER:
            XTRACE(this, data, event, "executeEventGated - PD_E_TXQ_LOW_WATER");
            break;
	default:
            XTRACE(this, data, event, "executeEventGated - unrecognized event");
            ret = kIOReturnBadArgument;
            break;
    }
	
    return ret;
	
}/* end executeEventGated */

/****************************************************************************************************/
//
//		Method:		MSP430LPCDC::requestEvent
//
//		Inputs:		event - The event
//				refCon - unused
//
//		Outputs:	Return Code - kIOReturnSuccess, kIOReturnBadArgument
//				data - any data associated with the event
//
//		Desc:		requestEvent processes the specified event as an immediate request and
//				returns the results in data.  This is primarily used for getting link
//				status information and verifying baud rate etc.
//				For the most part this can be done immediately without being gated.
//
/****************************************************************************************************/

IOReturn MSP430LPCDC::requestEvent(UInt32 event, UInt32 *data, void *refCon)
{
    IOReturn	returnValue = kIOReturnSuccess;

    XTRACE(this, 0, event, "requestEvent");
    
    if (fTerminate || fStopping)
    {
        XTRACE(this, 0, kIOReturnOffline, "requestEvent - Offline");
        return kIOReturnOffline;
    }

    if (data == NULL) 
    {
        XTRACE(this, 0, event, "requestEvent - data is null");
        returnValue = kIOReturnBadArgument;
    } else {
        switch (event)
        {
            case PD_E_ACTIVE:
                XTRACE(this, 0, event, "requestEvent - PD_E_ACTIVE");
                *data = bool(getState(&fPort) & PD_S_ACTIVE);			// Just to be safe put this through the gate
                break;
            case PD_E_FLOW_CONTROL:
                XTRACE(this, fPort.FlowControl, event, "requestEvent - PD_E_FLOW_CONTROL");
                *data = fPort.FlowControl;							
                break;
            case PD_E_DELAY:
                XTRACE(this, 0, event, "requestEvent - PD_E_DELAY");
                *data = tval2long(fPort.CharLatInterval)/ 1000;	
                break;
            case PD_E_DATA_LATENCY:
                XTRACE(this, 0, event, "requestEvent - PD_E_DATA_LATENCY");
                *data = tval2long(fPort.DataLatInterval)/ 1000;	
                break;
            case PD_E_TXQ_SIZE:
                XTRACE(this, 0, event, "requestEvent - PD_E_TXQ_SIZE");
                *data = GetQueueSize(&fPort.TX);	
                break;
            case PD_E_RXQ_SIZE:
                XTRACE(this, 0, event, "requestEvent - PD_E_RXQ_SIZE");
                *data = GetQueueSize(&fPort.RX);	
                break;
            case PD_E_TXQ_LOW_WATER:
                XTRACE(this, 0, event, "requestEvent - PD_E_TXQ_LOW_WATER");
                *data = 0; 
                returnValue = kIOReturnBadArgument; 
                break;
            case PD_E_RXQ_LOW_WATER:
                XTRACE(this, 0, event, "requestEvent - PD_E_RXQ_LOW_WATER");
                *data = 0; 
                returnValue = kIOReturnBadArgument; 
                break;
            case PD_E_TXQ_HIGH_WATER:
                XTRACE(this, 0, event, "requestEvent - PD_E_TXQ_HIGH_WATER");
                *data = 0; 
                returnValue = kIOReturnBadArgument; 
                break;
            case PD_E_RXQ_HIGH_WATER:
                XTRACE(this, 0, event, "requestEvent - PD_E_RXQ_HIGH_WATER");
                *data = 0; 
                returnValue = kIOReturnBadArgument; 
                break;
            case PD_E_TXQ_AVAILABLE:
                XTRACE(this, 0, event, "requestEvent - PD_E_TXQ_AVAILABLE");
                *data = FreeSpaceinQueue(&fPort.TX);	 
                break;
            case PD_E_RXQ_AVAILABLE:
                XTRACE(this, 0, event, "requestEvent - PD_E_RXQ_AVAILABLE");
                *data = UsedSpaceinQueue(&fPort.RX); 	
                break;
            case PD_E_DATA_RATE:
                XTRACE(this, 0, event, "requestEvent - PD_E_DATA_RATE");
                *data = fPort.BaudRate << 1;		
                break;
            case PD_E_RX_DATA_RATE:
                XTRACE(this, 0, event, "requestEvent - PD_E_RX_DATA_RATE");
                *data = 0x00;					
                break;
            case PD_E_DATA_SIZE:
                XTRACE(this, 0, event, "requestEvent - PD_E_DATA_SIZE");
                *data = fPort.CharLength << 1;	
                break;
            case PD_E_RX_DATA_SIZE:
                XTRACE(this, 0, event, "requestEvent - PD_E_RX_DATA_SIZE");
                *data = 0x00;					
                break;
            case PD_E_DATA_INTEGRITY:
                XTRACE(this, 0, event, "requestEvent - PD_E_DATA_INTEGRITY");
                *data = fPort.TX_Parity;			
                break;
            case PD_E_RX_DATA_INTEGRITY:
                XTRACE(this, 0, event, "requestEvent - PD_E_RX_DATA_INTEGRITY");
                *data = fPort.RX_Parity;			
                break;
            case PD_RS232_E_STOP_BITS:
                XTRACE(this, 0, event, "requestEvent - PD_RS232_E_STOP_BITS");
                *data = fPort.StopBits << 1;		
                break;
            case PD_RS232_E_RX_STOP_BITS:
                XTRACE(this, 0, event, "requestEvent - PD_RS232_E_RX_STOP_BITS");
                *data = 0x00;					
                break;
            case PD_RS232_E_XON_BYTE:
                XTRACE(this, 0, event, "requestEvent - PD_RS232_E_XON_BYTE");
                *data = fPort.XONchar;			
                break;
            case PD_RS232_E_XOFF_BYTE:
                XTRACE(this, 0, event, "requestEvent - PD_RS232_E_XOFF_BYTE");
                *data = fPort.XOFFchar;			
                break;
            case PD_RS232_E_LINE_BREAK:
                XTRACE(this, 0, event, "requestEvent - PD_RS232_E_LINE_BREAK");
                *data = bool(getState(&fPort) & PD_RS232_S_BRK);			// This should be gated too
                break;
            case PD_RS232_E_MIN_LATENCY:
                XTRACE(this, 0, event, "requestEvent - PD_RS232_E_MIN_LATENCY");
                *data = bool(fPort.MinLatency);		
                break;
            default:
                XTRACE(this, 0, event, "requestEvent - unrecognized event");
                returnValue = kIOReturnBadArgument; 			
                break;
        }
    }

    return kIOReturnSuccess;
	
}/* end requestEvent */

/****************************************************************************************************/
//
//		Method:		MSP430LPCDC::enqueueEvent
//
//		Inputs:		event - The event
//				data - any data associated with the event, 
//				sleep - true (wait for it), false (don't)
//				refCon - unused
//
//		Outputs:	Return Code - kIOReturnSuccess or kIOReturnNotOpen
//
//		Desc:		Not used by this driver.
//				Events are passed on to executeEvent for immediate action.	
//
/****************************************************************************************************/

IOReturn MSP430LPCDC::enqueueEvent(UInt32 event, UInt32 data, bool sleep, void *refCon)
{
    IOReturn 	ret;
    
    XTRACE(this, data, event, "enqueueEvent");
    
    if (fTerminate || fStopping)
    {
        XTRACE(this, 0, kIOReturnOffline, "enqueueEvent - Offline");
        return kIOReturnOffline;
    }
    
    retain();
    ret = fCommandGate->runAction(executeEventAction, (void *)&event, (void *)&data);
    release();

    return ret;
	
}/* end enqueueEvent */

/****************************************************************************************************/
//
//		Method:		MSP430LPCDC::dequeueEvent
//
//		Inputs:		sleep - true (wait for it), false (don't)
//				refCon - unused
//
//		Outputs:	Return Code - kIOReturnSuccess, kIOReturnNotOpen
//
//		Desc:		Not used by this driver.		
//
/****************************************************************************************************/

IOReturn MSP430LPCDC::dequeueEvent(UInt32 *event, UInt32 *data, bool sleep, void *refCon)
{
	
    XTRACE(this, 0, 0, "dequeueEvent");
    
    if (fTerminate || fStopping)
    {
        XTRACE(this, 0, kIOReturnOffline, "dequeueEvent - Offline");
        return kIOReturnOffline;
    }

    if ((event == NULL) || (data == NULL))
        return kIOReturnBadArgument;

    if (getState(&fPort) & PD_S_ACTIVE)
    {
        return kIOReturnSuccess;
    }

    return kIOReturnNotOpen;
	
}/* end dequeueEvent */

/****************************************************************************************************/
//
//		Method:		MSP430LPCDC::enqueueData
//
//		Inputs:		buffer - the data
//				size - number of bytes
//				sleep - true (wait for it), false (don't)
//				refCon - unused
//
//		Outputs:	Return Code - kIOReturnSuccess, kIOReturnBadArgument or value returned from watchState
//				count - bytes transferred  
//
//		Desc:		set up for enqueueDataGated call.	
//
/****************************************************************************************************/

IOReturn MSP430LPCDC::enqueueData(UInt8 *buffer, UInt32 size, UInt32 *count, bool sleep, void *refCon)
{
    IOReturn 	ret;
    
    XTRACE(this, size, sleep, "enqueueData");
    
    if (fTerminate || fStopping)
    {
        XTRACE(this, 0, kIOReturnOffline, "enqueueData - Offline");
        return kIOReturnOffline;
    }
    
    if (count == NULL || buffer == NULL)
        return kIOReturnBadArgument;
        
    retain();
    ret = fCommandGate->runAction(enqueueDataAction, (void *)buffer, (void *)&size, (void *)count, (void *)&sleep);
    release();

    return ret;
        
}/* end enqueueData */

/****************************************************************************************************/
//
//		Method:		MSP430LPCDC::enqueueDatatAction
//
//		Desc:		Dummy pass through for enqueueDataGated.
//
/****************************************************************************************************/

IOReturn MSP430LPCDC::enqueueDataAction(OSObject *owner, void *arg0, void *arg1, void *arg2, void *arg3)
{

    return ((MSP430LPCDC *)owner)->enqueueDataGated((UInt8 *)arg0, (UInt32 *)arg1, (UInt32 *)arg2, (bool *)arg3);
    
}/* end enqueueDataAction */

/****************************************************************************************************/
//
//		Method:		MSP430LPCDC::enqueueDataGated
//
//		Inputs:		buffer - the data
//				size - number of bytes
//				sleep - true (wait for it), false (don't)
//
//		Outputs:	Return Code - kIOReturnSuccess or value returned from watchState
//				count - bytes transferred,  
//
//		Desc:		enqueueData will attempt to copy data from the specified buffer to
//				the TX queue as a sequence of VALID_DATA events.  The argument
//				bufferSize specifies the number of bytes to be sent.  The actual
//				number of bytes transferred is returned in count.
//				If sleep is true, then this method will sleep until all bytes can be
//				transferred.  If sleep is false, then as many bytes as possible
//				will be copied to the TX queue.
//				Note that the caller should ALWAYS check the transferCount unless
//				the return value was kIOReturnBadArgument, indicating one or more
//				arguments were not valid.  Other possible return values are
//				kIOReturnSuccess if all requirements were met or kIOReturnOffline
//				if the device was unplugged.		
//
/****************************************************************************************************/

IOReturn MSP430LPCDC::enqueueDataGated(UInt8 *buffer, UInt32 *pSize, UInt32 *count, bool *pSleep)
{
	UInt32		size = *pSize;
	bool		sleep = *pSleep;
    UInt32		state = PD_S_TXQ_LOW_WATER;
	UInt32		mask;
    IOReturn 	rtn = kIOReturnSuccess;

    XTRACE(this, size, sleep, "enqueueDataGated");

    if (fTerminate || fStopping)
        return kIOReturnOffline;

    *count = 0;

    if (!(fPort.State & PD_S_ACTIVE))
        return kIOReturnNotOpen;

    XTRACE(this, fPort.State, size, "enqueueDataGated - current State");	
//    LogData(kDataOther, size, buffer);

        // Go ahead and try to add something to the buffer
        
    *count = AddtoQueue(&fPort.TX, buffer, size);
    CheckQueues();

        // Let the tranmitter know that we have something ready to go
    
    setUpTransmit();

        // If we could not queue up all of the data on the first pass and
        // the user wants us to sleep until it's all out then sleep

    while ((*count < size) && sleep)
    {
        state = PD_S_TXQ_LOW_WATER;
		mask = PD_S_TXQ_LOW_WATER;
        rtn = watchStateGated(&state, &mask);
        if (rtn != kIOReturnSuccess)
        {
            XTRACE(this, 0, rtn, "enqueueDataGated - interrupted");
            return rtn;
        }

        *count += AddtoQueue(&fPort.TX, buffer + *count, size - *count);
        CheckQueues();

            // Let the tranmitter know that we have something ready to go.

        setUpTransmit();
    }

    XTRACE(this, *count, size, "enqueueDataGated - Exit");

    return kIOReturnSuccess;
	
}/* end enqueueDataGated */

/****************************************************************************************************/
//
//		Method:		MSP430LPCDC::dequeueData
//
//		Inputs:		size - buffer size
//				min - minimum bytes required
//				refCon - the Port
//
//		Outputs:	buffer - data returned
//				min - number of bytes
//				Return Code - kIOReturnSuccess, kIOReturnBadArgument, kIOReturnNotOpen, or value returned from watchState
//
//		Desc:		set up for dequeueDataGated call.
//
/****************************************************************************************************/

IOReturn MSP430LPCDC::dequeueData(UInt8 *buffer, UInt32 size, UInt32 *count, UInt32 min, void *refCon)
{
    IOReturn 	ret;
    
    XTRACE(this, size, min, "dequeueData");
    
    if (fTerminate || fStopping)
    {
        XTRACE(this, 0, kIOReturnOffline, "dequeueData - Offline");
        return kIOReturnOffline;
    }
    
    if ((count == NULL) || (buffer == NULL) || (min > size))
        return kIOReturnBadArgument;

    retain();
    ret = fCommandGate->runAction(dequeueDataAction, (void *)buffer, (void *)&size, (void *)count, (void *)&min);
    release();

    return ret;

}/* end dequeueData */

/****************************************************************************************************/
//
//		Method:		MSP430LPCDC::dequeueDatatAction
//
//		Desc:		Dummy pass through for dequeueDataGated.
//
/****************************************************************************************************/

IOReturn MSP430LPCDC::dequeueDataAction(OSObject *owner, void *arg0, void *arg1, void *arg2, void *arg3)
{

    return ((MSP430LPCDC *)owner)->dequeueDataGated((UInt8 *)arg0, (UInt32 *)arg1, (UInt32 *)arg2, (UInt32 *)arg3);
    
}/* end dequeueDataAction */

/****************************************************************************************************/
//
//		Method:		MSP430LPCDC::dequeueDataGated
//
//		Inputs:		size - buffer size
//				min - minimum bytes required
//
//		Outputs:	buffer - data returned
//				min - number of bytes
//				Return Code - kIOReturnSuccess, kIOReturnBadArgument, kIOReturnNotOpen, or value returned from watchState
//
//		Desc:		dequeueData will attempt to copy data from the RX queue to the
//				specified buffer.  No more than bufferSize VALID_DATA events
//				will be transferred. In other words, copying will continue until
//				either a non-data event is encountered or the transfer buffer
//				is full.  The actual number of bytes transferred is returned
//				in count.
//				The sleep semantics of this method are slightly more complicated
//				than other methods in this API. Basically, this method will
//				continue to sleep until either min characters have been
//				received or a non data event is next in the RX queue.  If
//				min is zero, then this method never sleeps and will return
//				immediately if the queue is empty.
//				Note that the caller should ALWAYS check the transferCount
//				unless the return value was kIOReturnBadArgument, indicating one or
//				more arguments were not valid.
//
/****************************************************************************************************/

IOReturn MSP430LPCDC::dequeueDataGated(UInt8 *buffer, UInt32 *pSize, UInt32 *count, UInt32 *pMin)
{
	UInt32		size = *pSize;
	UInt32		min = *pMin;
    IOReturn 	rtn = kIOReturnSuccess;
    UInt32		state = 0;
	UInt32		mask;
    bool		goXOIdle;
	UInt32		savCount;
	uintptr_t	addr;

    XTRACE(this, size, min, "dequeueDataGated");
    
    if (fTerminate || fStopping)
        return kIOReturnOffline;
	
        // If the port is not active then there should not be any chars.
        
    *count = 0;
    if (!(fPort.State & PD_S_ACTIVE))
        return kIOReturnNotOpen;

        // Get any data living in the queue.
        
    *count = RemovefromQueue(&fPort.RX, buffer, size);
	if (*count > 0)
	{
		addr = (uintptr_t)buffer;
		XTRACE(this, size, addr, "dequeueDataGated - Removed from Queue (first)");
		LogData(kDataOther, *count, buffer);
		CheckHold();
	}
    CheckQueues();

    while ((min > 0) && (*count < min))
    {
            // Figure out how many bytes we have left to queue up
            
        state = 0;
		mask = PD_S_RXQ_EMPTY;
        rtn = watchStateGated(&state, &mask);

        if (rtn != kIOReturnSuccess)
        {
            XTRACE(this, 0, rtn, "dequeueDataGated - Interrupted!");
            return rtn;
        }
        
            // Try and get more data starting from where we left off
		
//		*count += RemovefromQueue(&fPort.RX, buffer + *count, (size - *count));
		
		savCount = *count;
		*count += RemovefromQueue(&fPort.RX, &buffer[*count], (size - *count));
		addr = (uintptr_t)buffer;
		XTRACE(this, *count, addr, "dequeueDataGated - Removed from Queue (next)");
		LogData(kDataOther, *count, &buffer[savCount]);
		if (*count > 0)
		{
			CheckHold();
		}
        CheckQueues();
    }

        // Now let's check our receive buffer to see if we need to stop
        
    goXOIdle = (UsedSpaceinQueue(&fPort.RX) < fPort.RXStats.LowWater) && (fPort.RXOstate == SENT_XOFF);

    if (goXOIdle)
    {
        fPort.RXOstate = IDLE_XO;
        AddBytetoQueue(&fPort.TX, fPort.XOFFchar);
        setUpTransmit();
    }

    XTRACE(this, *count, size, "dequeueData - Exit");

    return rtn;
	
}/* end dequeueDataGated */

/****************************************************************************************************/
//
//		Method:		MSP430LPCDC::setUpTransmit
//
//		Inputs:		
//
//		Outputs:	return code - true (transmit started), false (transmission already in progress)
//
//		Desc:		Setup and then start transmisson
//
/****************************************************************************************************/

bool MSP430LPCDC::setUpTransmit()
{

    XTRACE(this, 0, 0, "setUpTransmit");
    
        // As a precaution just check we've not been terminated (maybe a woken thread)
    
    if (fTerminate || fStopping)
    {
        XTRACE(this, 0, 0, "setUpTransmit - terminated");
        return false;
    }

    if (UsedSpaceinQueue(&fPort.TX) > 0)
    {
        startTransmission();
    }

    return TRUE;
	
}/* end setUpTransmit */

/****************************************************************************************************/
//
//		Method:		MSP430LPCDC::startTransmission
//
//		Inputs:		
//
//		Outputs:	
//
//		Desc:		Start the transmisson
//				Must be called from a gated method
//
/****************************************************************************************************/

void MSP430LPCDC::startTransmission()
{
    size_t		count;
    IOReturn	ior;
    UInt16		indx;
	bool		gotBuffer = false;
	UInt32		state;
	UInt32		mask;
    
    XTRACE(this, 0, 0, "startTransmission");

        // Get an output buffer
	
	indx = fPort.outPoolIndex;
	if (!fPort.outPool[indx].avail)
	{
		for (indx=0; indx<fPort.outPoolIndex; indx++)
		{
			if (fPort.outPool[indx].avail)
			{
				fPort.outPoolIndex = indx;
				gotBuffer = true;
				break;
			}
		}
	} else {
		gotBuffer = true;
	}
	if (gotBuffer)
	{
		fPort.outPool[indx].avail = false;
		fPort.outPoolIndex++;
		if (fPort.outPoolIndex >= fOutBufPool)
		{
			fPort.outPoolIndex = 0;
		}
	} else {
		XTRACE(this, fOutBufPool, indx, "startTransmission - Output buffer unavailable");
        return;
	}

        // Fill up the buffer with characters from the queue
		
    count = RemovefromQueue(&fPort.TX, fPort.outPool[indx].pipeBuffer, MAX_BLOCK_SIZE);

        // If there are no bytes to send just exit:
		
    if (count <= 0)
    {
            // Updates all the status flags:
			
        CheckQueues();
		fPort.outPool[indx].avail = true;
        return;
    }
    
	state = PD_S_TX_BUSY;
	mask = PD_S_TX_BUSY;
    setStateGated(&state, &mask);
    
    XTRACE(this, fPort.State, count, "startTransmission - Bytes to write");
    LogData(kDataOut, count, fPort.outPool[indx].pipeBuffer);
    	
    fPort.outPool[indx].count = count;
    fPort.outPool[indx].completionInfo.parameter = (void *)&fPort.outPool[indx];
    fPort.outPool[indx].pipeMDP->setLength(count);
    
    ior = fPort.OutPipe->Write(fPort.outPool[indx].pipeMDP, &fPort.outPool[indx].completionInfo);
    if (ior != kIOReturnSuccess)
    {
        XTRACE(this, 0, ior, "startTransmission - Write failed");
    } 

        // We just removed a bunch of stuff from the
        // queue, so see if we can free some thread(s)
        // to enqueue more stuff.
		
    CheckQueues();
	
}/* end startTransmission */

/****************************************************************************************************/
//
//		Method:		MSP430LPCDC::setLineCoding
//
//		Inputs:		
//
//		Outputs:	
//
//		Desc:		Set up and send SetLineCoding Management Element Request(MER) for all settings.
//
/****************************************************************************************************/

void MSP430LPCDC::setLineCoding()
{

    XTRACE(this, 0, 0, "setLineCoding");
    
    	// Check for changes and only do it if something's changed
	
    if ((fPort.BaudRate == fPort.LastBaudRate) && (fPort.StopBits == fPort.LastStopBits) && 
        (fPort.TX_Parity == fPort.LastTX_Parity) && (fPort.CharLength == fPort.LastCharLength))
    {
        return;
    }

        // Now send it to the control driver
		
	if (fControlDriver)
	{
		fControlDriver->USBSendSetLineCoding(fPort.BaudRate, fPort.StopBits, fPort.TX_Parity, fPort.CharLength);
	}
	
	fPort.LastBaudRate = fPort.BaudRate;
	fPort.LastStopBits = fPort.StopBits;
	fPort.LastTX_Parity = fPort.TX_Parity;
	fPort.LastCharLength = fPort.CharLength;
	
}/* end setLineCoding */

/****************************************************************************************************/
//
//		Method:		MSP430LPCDC::setControlLineState
//
//		Inputs:		RTS - true(set RTS), false(clear RTS)
//				DTR - true(set DTR), false(clear DTR)
//
//		Outputs:	
//
//		Desc:		Set up and send SetControlLineState Management Element Request(MER).
//
/****************************************************************************************************/

void MSP430LPCDC::setControlLineState(bool RTS, bool DTR)
{
	
    XTRACE(this, 0, 0, "setControlLineState");

    if (fControlDriver)
    {
        fControlDriver->USBSendSetControlLineState(RTS, DTR);
    }	
		
}/* end setControlLineState */

/****************************************************************************************************/
//
//		Method:		MSP430LPCDC::sendBreak
//
//		Inputs:		sBreak - true(set Break), false(clear Break)
//
//		Outputs:	
//
//		Desc:		Set up and send SendBreak Management Element Request(MER).
//
/****************************************************************************************************/

void MSP430LPCDC::sendBreak(bool sBreak)
{
	
    XTRACE(this, 0, 0, "sendBreak");

    if (fControlDriver)
    {
        fControlDriver->USBSendBreak(sBreak);
    }
			
}/* end sendBreak */

/****************************************************************************************************/
//
//		Method:		MSP430LPCDC::checkPipe
//
//		Inputs:		thePipe - the pipe
//				devReq - true(send CLEAR_FEATURE), false(only if status returns stalled)
//
//		Outputs:	
//
//		Desc:		Clear a stall on the specified pipe. If ClearPipeStall is issued
//				all outstanding I/O is returned with kIOUSBTransactionReturned and
//				a CLEAR_FEATURE Endpoint stall is sent.
//
/****************************************************************************************************/

IOReturn MSP430LPCDC::checkPipe(IOUSBPipe *thePipe, bool devReq)
{
    


    IOReturn 	rtn = kIOReturnSuccess;
    
    XTRACEP(this, 0, thePipe, "checkPipe");
    
    if (!devReq)
    {
        rtn = thePipe->GetPipeStatus();
        if (rtn != kIOUSBPipeStalled)
        {
            XTRACE(this, 0, rtn, "checkPipe - Pipe not stalled");
            return rtn;
        }
    }
    
    rtn = thePipe->ClearPipeStall(true);
    if (rtn == kIOReturnSuccess)
    {
        XTRACE(this, 0, 0, "checkPipe - ClearPipeStall Successful");
    } else {
        XTRACE(this, 0, rtn, "checkPipe - ClearPipeStall Failed");
    }
    
    return rtn;

}/* end checkPipe */

/****************************************************************************************************/
//
//		Method:		MSP430LPCDC::initStructure
//
//		Inputs:		
//
//		Outputs:	
//
//		Desc:		Initialize the port structure
//
/****************************************************************************************************/

void MSP430LPCDC::initStructure()
{
    UInt16	i;
	
    XTRACE(this, 0, 0, "initStructure");

        // These are set up at start and should not be reset during execution.
        
    fPort.FCRimage = 0x00;
    fPort.IERmask = 0x00;

    fPort.State = (PD_S_TXQ_EMPTY | PD_S_TXQ_LOW_WATER | PD_S_RXQ_EMPTY | PD_S_RXQ_LOW_WATER);
    fPort.WatchStateMask = 0x00000000;
    fPort.InPipe = NULL;
    fPort.OutPipe = NULL;
    for (i=0; i<kMaxInBufPool; i++)
    {
        fPort.inPool[i].pipeMDP = NULL;
        fPort.inPool[i].pipeBuffer = NULL;
        fPort.inPool[i].dead = false;
		fPort.inPool[i].count = -1;
		fPort.inPool[i].held = false;
		
		fPort.holdQueue[i] = 0;
    }
	fPort.holdQueueIndxIn = 0;
	fPort.holdQueueIndxOut = 0;
	
    for (i=0; i<kMaxOutBufPool; i++)
    {
        fPort.outPool[i].pipeMDP = NULL;
        fPort.outPool[i].pipeBuffer = NULL;
        fPort.outPool[i].count = -1;
        fPort.outPool[i].avail = false;
    }
    fPort.outPoolIndex = 0;

}/* end initStructure */

/****************************************************************************************************/
//
//		Method:		MSP430LPCDC::setStructureDefaults
//
//		Inputs:		
//
//		Outputs:	
//
//		Desc:		Sets the defaults for the specified port structure
//
/****************************************************************************************************/

void MSP430LPCDC::setStructureDefaults()
{
    UInt32	tmp;
	
    XTRACE(this, 0, 0, "setStructureDefaults");

    fPort.BaudRate = kDefaultBaudRate;			// 9600 bps
    fPort.LastBaudRate = 0;
    fPort.CharLength = 8;				// 8 Data bits
    fPort.LastCharLength = 0;
    fPort.StopBits = 2;					// 1 Stop bit
    fPort.LastStopBits = 0;
    fPort.TX_Parity = 1;				// No Parity
    fPort.LastTX_Parity	= 0;
    fPort.RX_Parity = 1;				// --ditto--
    fPort.MinLatency = false;
    fPort.XONchar = '\x11';
    fPort.XOFFchar = '\x13';
    fPort.FlowControl = 0x00000000;
    fPort.RXOstate = IDLE_XO;
    fPort.TXOstate = IDLE_XO;
    fPort.FrameTOEntry = NULL;

    fPort.RXStats.BufferSize = kMaxCirBufferSize;
    fPort.RXStats.HighWater = (fPort.RXStats.BufferSize << 1) / 3;
    fPort.RXStats.LowWater = fPort.RXStats.HighWater >> 1;
    fPort.TXStats.BufferSize = kMaxCirBufferSize;
    fPort.TXStats.HighWater = (fPort.RXStats.BufferSize << 1) / 3;
    fPort.TXStats.LowWater = fPort.RXStats.HighWater >> 1;

    fPort.FlowControl = (DEFAULT_AUTO | DEFAULT_NOTIFY);

    for (tmp=0; tmp < (256 >> SPECIAL_SHIFT); tmp++)
        fPort.SWspecial[ tmp ] = 0;
	
}/* end setStructureDefaults */


/****************************************************************************************************/
//
//		Method:		AppleUSBCDCACMData::allocateDataResources
//
//		Inputs:		
//
//		Outputs:	return code - true (allocate was successful), false (it failed)
//
//		Desc:		Finishes up the rest of the configuration and gets all the endpoints open etc.
//
/****************************************************************************************************/

bool MSP430LPCDC::allocateDataResources()
{
    IOUSBFindEndpointRequest	epReq;
    UInt16			i;
    
    XTRACE(this, 0, 0, "allocateDataResources.");
    
    // Open all the end points and get the buffers
    
    if (!fMSP430Interface->open(this))
    {
        XTRACE(this, 0, 0, "allocateDataResources - open data interface failed.");
        return false;
    }
    
    // Bulk In pipe
    
    epReq.type = kUSBBulk;
    epReq.direction = kUSBIn;
    epReq.maxPacketSize	= 0;
    epReq.interval = 0;
    fPort.InPipe = fMSP430Interface->FindNextPipe(0, &epReq);
    if (!fPort.InPipe)
    {
        XTRACE(this, 0, 0, "allocateDataResources - no bulk input pipe.");
        return false;
    }
    fPort.InPacketSize = epReq.maxPacketSize;
    XTRACE(this, epReq.maxPacketSize << 16 |epReq.interval, 0, "allocateDataResources - bulk input pipe.");
    
    // Allocate Memory Descriptor Pointer with memory for the bulk in pipe
    
    for (i=0; i<fInBufPool; i++)
    {
        //        fPort.inPool[i].pipeMDP = IOBufferMemoryDescriptor::withCapacity(DATA_BUFF_SIZE, kIODirectionIn);
        fPort.inPool[i].pipeMDP = IOBufferMemoryDescriptor::withOptions(kIODirectionIn | kIOMemoryPhysicallyContiguous, DATA_BUFF_SIZE, PAGE_SIZE);
        if (!fPort.inPool[i].pipeMDP)
        {
            XTRACE(this, 0, i, "allocateDataResources - Allocate input MDP failed");
            return false;
        }
        fPort.inPool[i].pipeBuffer = (UInt8*)fPort.inPool[i].pipeMDP->getBytesNoCopy();
        XTRACEP(this, fPort.inPool[i].pipeMDP, fPort.inPool[i].pipeBuffer, "allocateDataResources - input buffer");
        fPort.inPool[i].dead = false;
    }
    
    // Bulk Out pipe
    
    epReq.direction = kUSBOut;
    fPort.OutPipe = fMSP430Interface->FindNextPipe(0, &epReq);
    if (!fPort.OutPipe)
    {
        XTRACE(this, 0, 0, "allocateDataResources - no bulk output pipe.");
        return false;
    }
    fPort.OutPacketSize = epReq.maxPacketSize;
    XTRACE(this, epReq.maxPacketSize << 16 |epReq.interval, 0, "allocateDataResources - bulk output pipe.");
    
    // Allocate Memory Descriptor Pointer with memory for the bulk out pipe
    
    for (i=0; i<fOutBufPool; i++)
    {
        //        fPort.outPool[i].pipeMDP = IOBufferMemoryDescriptor::withCapacity(MAX_BLOCK_SIZE, kIODirectionOut);
        fPort.outPool[i].pipeMDP = IOBufferMemoryDescriptor::withOptions(kIODirectionOut | kIOMemoryPhysicallyContiguous, MAX_BLOCK_SIZE, PAGE_SIZE);
        if (!fPort.outPool[i].pipeMDP)
        {
            XTRACE(this, 0, i, "allocateDataResources - Allocate output MDP failed");
            return false;
        }
        fPort.outPool[i].pipeBuffer = (UInt8*)fPort.outPool[i].pipeMDP->getBytesNoCopy();
        XTRACEP(this, fPort.outPool[i].pipeMDP, fPort.outPool[i].pipeBuffer, "allocateDataResources - output buffer");
        fPort.outPool[i].avail = true;
    }
    
    // Now the ring buffers
    
    if (!allocateRingBuffer(&fPort.TX, fPort.TXStats.BufferSize))
    {
        XTRACE(this, 0, 0, "allocateDataResources - Couldn't allocate TX ring buffer");
        return false;
    }
    
    XTRACEP(this, 0, fPort.TX.Start, "allocateDataResources - TX ring buffer");
    
    if (!allocateRingBuffer(&fPort.RX, fPort.RXStats.BufferSize)) 
    {
        XTRACE(this, 0, 0, "allocateDataResources - Couldn't allocate RX ring buffer");
        return false;
    }
    
    XTRACEP(this, 0, fPort.RX.Start, "allocateDataResources - RX ring buffer");
    
    return true;
	
}/* end allocateDataResources */


/****************************************************************************************************/
//
//		Method:		AppleUSBCDCACMControl::allocateCommResources
//
//		Inputs:		
//
//		Outputs:	return code - true (allocate was successful), false (it failed)
//
//		Desc:		Finishes up the rest of the configuration and gets all the endpoints open etc.
//
/****************************************************************************************************/

bool MSP430LPCDC::allocateCommResources()
{
    IOUSBFindEndpointRequest	epReq;
    
    XTRACE(this, 0, 0, "allocateCommResources.");
    
    // Open the end point and get the buffers
    
/*    if (!fMSP430Interface->open(this))
    {
        XTRACE(this, 0, 0, "allocateCommResources - open comm interface failed.");
        return false;
    }

 */
    // Interrupt pipe
    
    epReq.type = kUSBInterrupt;
    epReq.direction = kUSBIn;
    fCommPipe = fMSP430Interface->FindNextPipe(0, &epReq);
    if (!fCommPipe)
    {
        XTRACE(this, 0, 0, "allocateCommResources - no comm pipe.");
        return false;
    }
    XTRACE(this, epReq.maxPacketSize << 16 |epReq.interval, 0, "allocateCommResources - comm pipe.");
    
    // Allocate Memory Descriptor Pointer with memory for the Interrupt pipe:
    
    fCommPipeMDP = IOBufferMemoryDescriptor::withCapacity(COMM_BUFF_SIZE, kIODirectionIn);
    if (!fCommPipeMDP)
    {
        XTRACE(this, 0, 0, "allocateCommResources - Couldn't allocate MDP for interrupt pipe");
        return false;
    }
    
    fCommPipeBuffer = (UInt8*)fCommPipeMDP->getBytesNoCopy();
    XTRACEP(this, 0, fCommPipeBuffer, "allocateCommResources - comm buffer");
    
    return true;
	
}/* end allocateCommResources */



/****************************************************************************************************/
//
//		Method:		MSP430LPCDC::releaseCommResources
//
//		Inputs:		
//
//		Outputs:	
//
//		Desc:		Frees up the resources allocated in allocateCommResources
//
/****************************************************************************************************/

void MSP430LPCDC::releaseResources()
{
    UInt16	i;
    
    XTRACE(this, 0, 0, "releaseResources");
	
	if (fMSP430Interface)	
    { 
        fMSP430Interface->close(this);
        fMSP430Interface->release();
        fMSP430Interface = NULL;
    }
    
    for (i=0; i<fInBufPool; i++)
    {
        if (fPort.inPool[i].pipeMDP)	
        { 
            fPort.inPool[i].pipeMDP->release();	
            fPort.inPool[i].pipeMDP = NULL;
            fPort.inPool[i].dead = false;
        }
    }
	
    for (i=0; i<fOutBufPool; i++)
    {
        if (fPort.outPool[i].pipeMDP)	
        { 
            fPort.outPool[i].pipeMDP->release();	
            fPort.outPool[i].pipeMDP = NULL;
            fPort.outPool[i].count = -1;
            fPort.outPool[i].avail = false;
        }
    }
    fPort.outPoolIndex = 0;
    
    if (fWorkLoop)
    {
        fWorkLoop->release();
        fWorkLoop = NULL;
    }

    freeRingBuffer(&fPort.TX);
    freeRingBuffer(&fPort.RX);
    
    if (fCommPipeMDP)	
    { 
        fCommPipeMDP->release();	
        fCommPipeMDP = 0; 
    }
	
}/* end releaseResources */


/****************************************************************************************************/
//
//		Method:		MSP430LPCDC::freeRingBuffer
//
//		Inputs:		Queue - the specified queue to free
//
//		Outputs:	
//
//		Desc:		Frees all resources assocated with the queue, then sets all queue parameters 
//				to safe values.
//
/****************************************************************************************************/

void MSP430LPCDC::freeRingBuffer(CirQueue *Queue)
{
    XTRACEP(this, 0, Queue, "freeRingBuffer");

    if (Queue)
    {
        if (Queue->Start)
        {
            IOFree(Queue->Start, Queue->Size);
        }
        CloseQueue(Queue);
    }
	
}/* end freeRingBuffer */

/****************************************************************************************************/
//
//		Method:		MSP430LPCDC::allocateRingBuffer
//
//		Inputs:		Queue - the specified queue to allocate
//				BufferSize - size to allocate
//
//		Outputs:	return Code - true (buffer allocated), false (it failed)
//
//		Desc:		Allocates resources needed by the queue, then sets up all queue parameters. 
//
/****************************************************************************************************/

bool MSP430LPCDC::allocateRingBuffer(CirQueue *Queue, size_t BufferSize)
{
    UInt8	*Buffer;

        // Size is ignored and kMaxCirBufferSize, which is 4096, is used.
		
    XTRACE(this, 0, BufferSize, "allocateRingBuffer");
    Buffer = (UInt8*)IOMalloc(kMaxCirBufferSize);

    InitQueue(Queue, Buffer, kMaxCirBufferSize);

    if (Buffer)
        return true;

    return false;
	
}/* end allocateRingBuffer */

/****************************************************************************************************/
//
//		Function:	MSP430LPCDC::handleSettingCallback
//
//		Inputs:		
//
//		Outputs:	none	
//
//		Desc:		Handles the async Wake on Ring setting
//
/****************************************************************************************************/

void MSP430LPCDC::handleSettingCallback(const OSSymbol *arg_type, OSObject *arg_val, uintptr_t refcon)
{
    UInt32				WoR;
	
	XTRACE(this, 0, 0, "handleSettingCallback");
		
    WoR = ((OSNumber *)arg_val)->unsigned32BitValue();
    
	if (arg_type == gPMWakeOnRingSymbol)
	{
		if (WoR != fWoR)
		{
			fWoR = WoR;
			if (fTerminate || fStopping)
			{
				XTRACE(this, 0, 0, "handleSettingCallback - Offline");
				return;
			}
			setWakeFeature();
		} else {
			XTRACE(this, 0, 0, "handleSettingCallback - Wake on Ring unchanged");
		}
    }
	
}/* end handleSettingCallback */

/****************************************************************************************************/
//
//		Function:	MSP430LPCDC::setupWakeOnRingPMCallback
//
//		Inputs:		none
//
//		Outputs:	return code - true( callback enabled), false(disabled)	
//
//		Desc:		Set up the PM callback for Wake on Ring
//
/****************************************************************************************************/

bool MSP430LPCDC::setupWakeOnRingPMCallback()
{
	IOReturn		ior;
	bool			worOK = false;
    const OSSymbol	*settings_arr[] = {gPMWakeOnRingSymbol, (const OSSymbol *)NULL};
	
	XTRACE(this, 0, 0, "setupWakeOnRingPMCallback");
	
	if (fPMRootDomain)
	{
		fPMRootDomain->publishFeature("WakeOnRing");
    
		ior = fPMRootDomain->registerPMSettingController(settings_arr,
														 OSMemberFunctionCast(IOPMSettingControllerCallback,
														 (OSObject*)this,
														 &MSP430LPCDC::handleSettingCallback),
														 (OSObject *)this,
														 (uintptr_t)NULL,
														 (OSObject **)&fWakeSettingControllerHandle);
		if (ior == kIOReturnSuccess)
		{
			XTRACE(this, 0, 0, "setupWakeOnRingPMCallback - Setting PM callback successful");
			worOK = true;
		} else {
			XTRACE(this, 0, 0, "setupWakeOnRingPMCallback - Setting PM callback failed, wake-on-ring set at start only");
		}
	} else {
		XTRACE(this, 0, 0, "setupWakeOnRingPMCallback - PM root domain is invalid, wake-on-ring set at start only");
	}
	
    return worOK;
	
}/* end setupWakeOnRingPMCallback */

/****************************************************************************************************/
//
//		Function:	MSP430LPCDC::WakeonRing
//
//		Inputs:		none
//
//		Outputs:	return code - true(always at the moment...)	
//
//		Desc:		Get the current Wake on Ring setting
//
/****************************************************************************************************/

bool MSP430LPCDC::WakeonRing(void)
{
    OSObject	*initWORValue = NULL;
	UInt32		worVal;
	
    XTRACE(this, 0, 0, "WakeonRing");
        
    fPMRootDomain = getPMRootDomain();
	if (fPMRootDomain)
	{
		fPMRootDomain->registerInterestedDriver(this);
        
		initWORValue = fPMRootDomain->copyPMSetting((OSSymbol *)gPMWakeOnRingSymbol);
		if (initWORValue)
		{
			worVal = ((OSNumber *)initWORValue)->unsigned32BitValue();
			if (worVal)
			{
				XTRACE(this, 0, worVal, "WakeonRing - Wake on Ring Enabled");
				fWoR = true;
			} else {
				XTRACE(this, 0, 0, "WakeonRing - Wake on Ring Disabled");
			}
		} else {
			XTRACE(this, 0, 0, "WakeonRing - Initial Wake on Ring unavailable, now disabled...");
		}
	} else {
		XTRACE(this, 0, 0, "WakeonRing - Remote wake up is disabled");
	}
    
    return true;
    
}/* end WakeonRing */

/****************************************************************************************************/
//
//		Function:	MSP430LPCDC::setWakeFeature
//
//		Inputs:		none
//
//		Outputs:	none
//
//		Desc:		Check the wake-on-ring feature and send the device request
//
/****************************************************************************************************/

void MSP430LPCDC::setWakeFeature(void)
{
	IOUSBDevRequest devreq;
	IOReturn		ior;

    XTRACE(this, 0, 0, "setWakeFeature");
	    
		// Set/Clear the Device Remote Wake feature depending upon wake-on-ring
    
	devreq.bmRequestType = USBmakebmRequestType(kUSBOut, kUSBStandard, kUSBDevice);
	if (!fWoR)				
	{
		devreq.bRequest = kUSBRqClearFeature;
	} else {
		devreq.bRequest = kUSBRqSetFeature;
	}
	devreq.wValue = kUSBFeatureDeviceRemoteWakeup;
	devreq.wIndex = 0;
	devreq.wLength = 0;
	devreq.pData = 0;

	ior = fMSP430Interface->GetDevice()->DeviceRequest(&devreq);
	if (ior == kIOReturnSuccess)
	{
		XTRACE(this, fWoR, ior, "setWakeFeature - Set/Clear remote wake up feature successful");
	} else {
		XTRACE(this, fWoR, ior, "setWakeFeature - Set/Clear remote wake up feature failed");
	}

}/* end setWakeFeature */

/****************************************************************************************************/
//
//		Method:		MSP430LPCDC::clearSleepingThreads
//
//		Inputs:		
//
//		Outputs:	
//
//		Desc:		Try to clear any threads asleep on the command gate
//
/****************************************************************************************************/

void MSP430LPCDC::clearSleepingThreads()
{
	
	XTRACE(this, 0, fThreadSleepCount, "clearSleepingThreads");
    
        // If we still have a command gate clean up anything sleeping on it
    
    if (fCommandGate)
    {
        if (fThreadSleepCount > 0)
        {
            fPort.WatchStateMask = 0;
            fCommandGate->commandWakeup((void *)&fPort.State);
        }
    }
		    
}/* end clearSleepingThreads */

/****************************************************************************************************/
//
//		Method:		MSP430LPCDC::resurrectRead
//
//		Inputs:		
//
//		Outputs:	
//
//		Desc:		Try and resurrect any dead reads 
//
/****************************************************************************************************/

void MSP430LPCDC::resurrectRead()
{
    UInt16		i;
	IOReturn	rtn;
	
	XTRACE(this, 0, 0, "resurrectRead");
	
		// Let's check the pipes first
	
	if (fPort.InPipe)
	{
		checkPipe(fPort.InPipe, false);
	}
    
	if (fPort.OutPipe)
	{
		checkPipe(fPort.OutPipe, false);
	}

	for (i=0; i<fInBufPool; i++)
	{
		if (fPort.inPool[i].pipeMDP)
		{
			if (fPort.inPool[i].dead)
			{
				rtn = fPort.InPipe->Read(fPort.inPool[i].pipeMDP, &fPort.inPool[i].completionInfo, NULL);
				if (rtn != kIOReturnSuccess)
				{
					XTRACE(this, i, rtn, "resurrectRead - Read for bulk-in pipe failed, still dead");
				} else {
					XTRACEP(this, &fPort.inPool[i], fPort.InPipe, "resurrectRead - Read posted");
					fPort.inPool[i].dead = false;
				}
			}
		}
	}
	
}/* end resurrectRead */

/****************************************************************************************************/
//
//		Method:		MSP430LPCDC::didTerminate
//
//		Inputs:		type - provider - my provider
//					options - additional parameters
//					defer - defer flag
//
//		Outputs:	return Code
//
//		Desc:		Handle did termination notification
//
/****************************************************************************************************/

bool MSP430LPCDC::didTerminate(IOService *provider, IOOptionBits options, bool *defer)
{
	bool	result = false;
	
	XTRACE(this, 0, fThreadSleepCount, "didTerminate");
    
    fTerminate = true;                  // Just to make sure
    
    clearSleepingThreads();             // All the threads should be gone by now but make sure
 	
	result = super::didTerminate(provider, options, defer);
	
	return result;
    
}/* end didTerminate */

/****************************************************************************************************/
//
//		Method:		MSP430LPCDC::message
//
//		Inputs:		type - message type
//                  provider - my provider
//                  argument - additional parameters
//
//		Outputs:	return Code - kIOReturnSuccess
//
//		Desc:		Handles IOKit messages. 
//
/****************************************************************************************************/

IOReturn MSP430LPCDC::message(UInt32 type, IOService *provider, void *argument)
{	
    IOReturn	rtn;
    
    XTRACE(this, 0, type, "message");
	
    switch (type)
    {
        case kIOMessageServiceIsTerminated:
            XTRACE(this, fSessions, type, "message - kIOMessageServiceIsTerminated");
			
				// As a precaution abort any I/O
			
			if (fPort.InPipe)
				fPort.InPipe->Abort();
			if (fPort.OutPipe)
				fPort.OutPipe->Abort();
			
			if (!fSuppressWarning)
			{
				if (fSessions)
				{
					if (!fTerminate)		// Check if we're already being terminated
					{ 
#if 0
							// NOTE! This call below depends on the hard coded path of this KEXT. Make sure
							// that if the KEXT moves, this path is changed!
						KUNCUserNotificationDisplayNotice(
						10,		// Timeout in seconds
						0,		// Flags (for later usage)
						"",		// iconPath (not supported yet)
						"",		// soundPath (not supported yet)
						"/System/Library/Extensions/IOUSBFamily.kext/Contents/PlugIns/MSP430LPCDC.kext",		// localizationPath
						"Unplug Header",		// the header
						"Unplug Notice",		// the notice - look in Localizable.strings
						"OK"); 
#endif
					}
				}
			}
            			
            fTerminate = true;		// We're being terminated (unplugged) let's see if we can clean up some threads and release some stuff
    
            releaseResources();
            return kIOReturnSuccess;
			
        case kIOMessageServiceIsSuspended: 	
            XTRACE(this, 0, type, "message - kIOMessageServiceIsSuspended");
            break;			
        case kIOMessageServiceIsResumed: 	
            XTRACE(this, 0, type, "message - kIOMessageServiceIsResumed");
            break;			
        case kIOMessageServiceIsRequestingClose: 
            XTRACE(this, 0, type, "message - kIOMessageServiceIsRequestingClose"); 
            break;
        case kIOMessageServiceWasClosed: 	
            XTRACE(this, 0, type, "message - kIOMessageServiceWasClosed"); 
            break;
        case kIOMessageServiceBusyStateChange: 	
            XTRACE(this, 0, type, "message - kIOMessageServiceBusyStateChange"); 
            break;
        case kIOUSBMessagePortHasBeenResumed: 	
            XTRACE(this, 0, type, "message - kIOUSBMessagePortHasBeenResumed");
			if (fReadDead)
			{
				rtn = fCommPipe->Read(fCommPipeMDP, &fCommCompletionInfo, NULL);
				if (rtn != kIOReturnSuccess)
				{
					XTRACE(this, 0, rtn, "message - Read for interrupt-in pipe failed, still dead");
				} else {
					fReadDead = false;
				}
			}
            resurrectRead();
            return kIOReturnSuccess;
        case kIOUSBMessageHubResumePort:
            XTRACE(this, 0, type, "message - kIOUSBMessageHubResumePort");
            break;
        case kIOUSBMessagePortHasBeenReset:
            XTRACE(this, 0, type, "message - kIOUSBMessagePortHasBeenReset");
			resurrectRead();
			if (fConfigAttributes & kUSBAtrRemoteWakeup)
			{
				setWakeFeature();
			}
			if (fReadDead)
			{
				rtn = fCommPipe->Read(fCommPipeMDP, &fCommCompletionInfo, NULL);
				if (rtn != kIOReturnSuccess)
				{
					XTRACE(this, 0, rtn, "message - Read for interrupt-in pipe failed, still dead");
				} else {
					fReadDead = false;
				}
			}
            return kIOReturnSuccess;

        default:
            XTRACE(this, 0, type, "message - unknown message"); 
            break;
    }
    
    return super::message(type, provider, argument);
    
}/* end message */

/****************************************************************************************************/
//
//		Method:		MSP430LPCDC::getPortNameForInterface
//
//		Inputs:		interfaceNumber - the number of the interface we're interested in
//
//		Outputs:	return Code - NULL (not found) or the name
//
//		Desc:		Gets the name from the mapping 
//
/****************************************************************************************************/

OSString *MSP430LPCDC::getPortNameForInterface(UInt8 interfaceNumber)
{
	OSSymbol *ttyName = NULL;
	char	 endPointAddrStr[16];
	
	XTRACE(this, 0, interfaceNumber, "getPortNameForInterface");

	if (fInterfaceMappings)
	{		 
		snprintf(endPointAddrStr,sizeof(endPointAddrStr),"%d",interfaceNumber);		
		ttyName = (OSSymbol *)fInterfaceMappings->getObject(endPointAddrStr);			
	 }

	return ttyName;
	
}/* end getPortNameForInterface */

#undef  super
#define super IOUserClient

OSDefineMetaClassAndStructors(MSP430LPCDCUserClient, IOUserClient);

/****************************************************************************************************/
//
//		Method:		MSP430LPCDCUserClient::getTargetAndMethodForIndex
//
//		Inputs:		
//
//		Outputs:	return code - method index
//
//		Desc:		Get the method index. 
//
/****************************************************************************************************/

//IOExternalMethod *MSP430LPCDCUserClient::getExternalMethodForIndex(UInt32 index)
IOExternalMethod *MSP430LPCDCUserClient::getTargetAndMethodForIndex(IOService **targetP, UInt32 index)
{
    IOExternalMethod	*result = NULL;

    XTRACE(this, 0, index, "getTargetAndMethodForIndex");
    
    if (index == 0)
    {
        result = &fMethods[0];
		*targetP = this;
    }

    return result;
    
}/* end getTargetAndMethodForIndex */

/****************************************************************************************************/
//
//		Method:		MSP430LPCDCUserClient::initWithTask
//
//		Inputs:		owningTask - the owner
//					security_id - Security ID
//					type - Client code (lucky number)
//
//		Outputs:	true - it worked, false - it didn't
//
//		Desc:		Set up the user client task. 
//
/****************************************************************************************************/

bool MSP430LPCDCUserClient::initWithTask(task_t owningTask, void *security_id , UInt32 type)
{

    XTRACE(this, 0, 0, "initWithTask");
    
    if (!super::initWithTask(owningTask, security_id, type))
    {
        XTRACE(this, 0, 0, "initWithTask - super failed");
        return false;
    }
    
    if (!owningTask)
    {
        XTRACE(this, 0, 0, "initWithTask - No owning task");
		return false;
    }
	
    fTask = owningTask;
    fProvider = NULL;
        
    return true;
    
}/* end initWithTask */



/****************************************************************************************************/
//
//		Method:		MSP430LPCDC::commReadComplete
//
//		Inputs:		obj - me
//				rc - return code
//				remaining - what's left
//
//		Outputs:	None
//
//		Desc:		Interrupt pipe (Comm interface) read completion routine
//
/****************************************************************************************************/

void MSP430LPCDC::commReadComplete(void *obj, void *param, IOReturn rc, UInt32 remaining)
{
    MSP430LPCDC	*me = (MSP430LPCDC*)obj;
    IOReturn			ior;
    UInt32			dLen;
    UInt16			*tState;
    UInt32			tempS, value, mask;
    
    XTRACE(me, rc, 0, "commReadComplete");
    
    if (me->fStopping)
        return;
    
    if (rc == kIOReturnSuccess)					// If operation returned ok
    {
        dLen = COMM_BUFF_SIZE - remaining;
        XTRACE(me, 0, dLen, "commReadComplete - data length");
		
        // Now look at the state stuff
		
        if ((dLen > 7) && (me->fCommPipeBuffer[1] == kUSBSERIAL_STATE))
        {
            tState = (UInt16 *)&me->fCommPipeBuffer[8];
            tempS = USBToHostWord(*tState);
            XTRACE(me, 0, tempS, "commReadComplete - kUSBSERIAL_STATE");
			if (tempS & kUSBbRxCarrier)
			{
				value = PD_RS232_S_DCD;
				mask = PD_RS232_S_DCD;
			}
			if (tempS & kUSBbTxCarrier)
			{
				value |= PD_RS232_S_DSR;
				mask |= PD_RS232_S_DSR;
			}
			if (tempS & kUSBbBreak)
			{
				value |= PD_RS232_S_BRK;
				mask |= PD_RS232_S_BRK;
			}
			if (tempS & kUSBbRingSignal)
			{
				value |= PD_RS232_S_RNG;
				mask |= PD_RS232_S_RNG;
			}
			
            //            mask = sMapModemStates[15];				// All 4 on
            //            value = sMapModemStates[tempS & 15];		// now the status bits
            if (me->fDataDriver)
            {
                me->fDataDriver->setState(value, mask, NULL);
            }
        } else {
			XTRACE(me, 0, me->fCommPipeBuffer[1], "commReadComplete - Unhandled notification");
		}
    } else {
        XTRACE(me, 0, rc, "commReadComplete - error");
        if (rc != kIOReturnAborted)
        {
			if (rc == kIOUSBPipeStalled)
			{
				rc = me->checkPipe(me->fCommPipe, true);
			} else {
				rc = me->checkPipe(me->fCommPipe, false);
			}
            if (rc != kIOReturnSuccess)
            {
                XTRACE(me, 0, rc, "dataReadComplete - clear stall failed (trying to continue)");
            }
        }
    }
    
    // Queue the next read only if not aborted
	
    if (rc != kIOReturnAborted)
    {
        ior = me->fCommPipe->Read(me->fCommPipeMDP, &me->fCommCompletionInfo, NULL);
        if (ior != kIOReturnSuccess)
        {
            XTRACE(me, 0, rc, "commReadComplete - Read io error");
			me->fReadDead = true;
        }
    } else {
		me->fReadDead = true;
	}
	
}/* end commReadComplete */

/****************************************************************************************************/
//
//		Method:		MSP430LPCDC::merWriteComplete
//
//		Inputs:		obj - me
//				param - MER 
//				rc - return code
//				remaining - what's left
//
//		Outputs:	None
//
//		Desc:		Management Element Request write completion routine
//
/****************************************************************************************************/

void MSP430LPCDC::merWriteComplete(void *obj, void *param, IOReturn rc, UInt32 remaining)
{
    MSP430LPCDC	*me = (MSP430LPCDC *)obj;
    IOUSBDevRequest		*MER = (IOUSBDevRequest *)param;
    UInt16			dataLen;
    
    XTRACE(me, 0, remaining, "merWriteComplete");
    
    if (me->fStopping)
    {
        if (MER)
        {
            dataLen = MER->wLength;
            if ((dataLen != 0) && (MER->pData))
            {
                IOFree(MER->pData, dataLen);
            }
            IOFree(MER, sizeof(IOUSBDevRequest));
        }
        return;
    }
    
    if (MER)
    {
        if (rc == kIOReturnSuccess)
        {
            XTRACE(me, MER->bRequest, remaining, "merWriteComplete - request");
        } else {
            XTRACE(me, MER->bRequest, rc, "merWriteComplete - io err");
			if (rc == kIOUSBPipeStalled)
			{
				rc = me->checkPipe(me->fCommPipe, false);
			}
        }
		
        dataLen = MER->wLength;
        XTRACE(me, 0, dataLen, "merWriteComplete - data length");
        if ((dataLen != 0) && (MER->pData))
        {
            IOFree(MER->pData, dataLen);
        }
        IOFree(MER, sizeof(IOUSBDevRequest));
		
    } else {
        if (rc == kIOReturnSuccess)
        {
            XTRACE(me, 0, remaining, "merWriteComplete (request unknown)");
        } else {
            XTRACE(me, 0, rc, "merWriteComplete (request unknown) - io err");
			if (rc == kIOUSBPipeStalled)
			{
				rc = me->checkPipe(me->fCommPipe, false);
			}
        }
    }
	
}/* end merWriteComplete */

/****************************************************************************************************/
//
//		Method:		MSP430LPCDC::getFunctionalDescriptors
//
//		Inputs:		
//
//		Outputs:	return - true (descriptors ok), false (somethings not right or not supported)	
//
//		Desc:		Finds all the functional descriptors for the specific interface
//
/****************************************************************************************************/

bool MSP430LPCDC::getFunctionalDescriptors()
{
    bool				gotDescriptors = false;
    UInt16				vers;
    UInt16				*hdrVers;
    const FunctionalDescriptorHeader 	*funcDesc = NULL;
    HDRFunctionalDescriptor		*HDRFDesc;		// hearder functional descriptor
    CMFunctionalDescriptor		*CMFDesc;		// call management functional descriptor
    ACMFunctionalDescriptor		*ACMFDesc;		// abstract control management functional descriptor

    
    XTRACE(this, 0, 0, "getFunctionalDescriptors");
    
    // Set some defaults just in case and then get the associated functional descriptors
	
    fCMCapabilities = CM_ManagementData + CM_ManagementOnData;
    fACMCapabilities = ACM_DeviceSuppControl + ACM_DeviceSuppBreak;
    fDataInterfaceNumber = 0x00;

    do
    {
        funcDesc = (const FunctionalDescriptorHeader *)fMSP430Interface->FindNextAssociatedDescriptor((void*)funcDesc, CS_INTERFACE);

        if (!funcDesc)
        {  
            gotDescriptors = true;				// We're done
        } else {
                //    IOLog("Type %04X, Subtype %04X\n",funcDesc->bDescriptorType, funcDesc->bDescriptorSubtype);
            switch (funcDesc->bDescriptorSubtype)
            {
                case Header_FunctionalDescriptor:
                    HDRFDesc = (HDRFunctionalDescriptor *)funcDesc;
                    XTRACE(this, funcDesc->bDescriptorType, funcDesc->bDescriptorSubtype, "getFunctionalDescriptors - Header Functional Descriptor");
                   // IOLog("Com Class Header Discriptor\n");
                    hdrVers = (UInt16 *)&HDRFDesc->bcdCDC1;
                    vers = USBToHostWord(*hdrVers);
                    if (vers > kUSBRel11)
                    {
                        XTRACE(this, vers, kUSBRel11, "getFunctionalDescriptors - Header descriptor version number is incorrect");
                    }
                    break;
                case CM_FunctionalDescriptor:
                    CMFDesc = (CMFunctionalDescriptor *)funcDesc;
                    XTRACE(this, funcDesc->bDescriptorType, funcDesc->bDescriptorSubtype, "getFunctionalDescriptors - CM Functional Descriptor");
                   // IOLog("Com Class Call Managment Discriptor\n");
                    if (fDataInterfaceNumber != 0xFF)
                    {
                        if (fDataInterfaceNumber != CMFDesc->bDataInterface)
                        {
                            XTRACE(this, fDataInterfaceNumber, CMFDesc->bDataInterface, "getFunctionalDescriptors - CMF and UNNF disagree");
                        }
                    }
                    if (CMFDesc->bmCapabilities & CM_ManagementData)
                    {
                        fDataInterfaceNumber = CMFDesc->bDataInterface;	// This will be the default if CM on data is set
                    }
                    fCMCapabilities = CMFDesc->bmCapabilities;
                    
                    // Check the configuration supports data management on the data interface (that's all we support)
                    
                    if (!(fCMCapabilities & CM_ManagementData))
                    {
                        XTRACE(this, 0, 0, "getFunctionalDescriptors - Interface doesn't support Call Management");
                        // return false;				// We'll relax this check too
                    }
                    if (!(fCMCapabilities & CM_ManagementOnData))
                    {
                        XTRACE(this, 0, 0, "getFunctionalDescriptors - Interface doesn't support Call Management on Data Interface");
                        //  return false;				// Some devices get this wrong so we'll let it slide
                    }
                    break;
                case ACM_FunctionalDescriptor:
                 //   IOLog("Com Class Abstract Control Managment Discriptor\n");
                    ACMFDesc = (ACMFunctionalDescriptor *)funcDesc;
                    XTRACE(this, funcDesc->bDescriptorType, funcDesc->bDescriptorSubtype, "getFunctionalDescriptors - ACM Functional Descriptor");
                    fACMCapabilities = ACMFDesc->bmCapabilities;
                    break;
                default:
                    XTRACE(this, funcDesc->bDescriptorType, funcDesc->bDescriptorSubtype, "getFunctionalDescriptors - unknown Functional Descriptor");
                    break;
            }
        }
    } while(!gotDescriptors);
     
    return true;
    
}/* end getFunctionalDescriptors */

/****************************************************************************************************/
//
//		Method:		MSP430LPCDC::dataAcquired
//
//		Inputs:		None
//
//		Outputs:	return Code - true (it worked), false (it didn't)
//
//		Desc:		Tells this driver the data driver's port has been acquired
//
/****************************************************************************************************/

bool MSP430LPCDC::dataAcquired()
{
    IOReturn 	rtn = kIOReturnSuccess; 
    
    XTRACE(this, 0, 0, "dataAcquired");
    
    // Read the comm interrupt pipe for status
    
    fCommCompletionInfo.target = this;
    fCommCompletionInfo.action = commReadComplete;
    fCommCompletionInfo.parameter = NULL;
    
    rtn = fCommPipe->Read(fCommPipeMDP, &fCommCompletionInfo, NULL);
    if (rtn == kIOReturnSuccess)
    {
        
        // Set up the management Element Request completion routine
        
        fMERCompletionInfo.target = this;
        fMERCompletionInfo.action = merWriteComplete;
        fMERCompletionInfo.parameter = NULL;
    } else {
        XTRACE(this, 0, rtn, "dataAcquired - Reading the interrupt pipe failed");
        return false;
    }
    
    fdataAcquired = true;
    return true;
    
}/* end dataAcquired */

/****************************************************************************************************/
//
//		Method:		MSP430LPCDC::dataReleased
//
//		Inputs:		None
//
//		Outputs:	None
//
//		Desc:		Tells this driver the data driver's port has been released
//
/****************************************************************************************************/

void MSP430LPCDC::dataReleased()
{
    
    XTRACE(this, 0, 0, "dataReleased");
    
	if (fCommPipe)
	{
		fCommPipe->Abort();
	}
    fdataAcquired = false;
    
}/* end dataReleased */

/****************************************************************************************************/
//
//		Method:		MSP430LPCDC::USBSendSetLineCoding
//
//		Inputs:		The line coding parameters
//
//		Outputs:	
//
//		Desc:		Set up and send SetLineCoding Management Element Request(MER) for all settings.
//
/****************************************************************************************************/

void MSP430LPCDC::USBSendSetLineCoding(UInt32 BaudRate, UInt8 StopBits, UInt8 TX_Parity, UInt8 CharLength)
{
    LineCoding		*lineParms;
    UInt16		lcLen = sizeof(LineCoding)-1;
    IOUSBDevRequest	*MER;
    IOReturn		rc;
	
    XTRACE(this, 0, 0, "USBSendSetLineCoding");
	
	if (!fMSP430Interface)
	{
		XTRACE(this, 0, 0, "USBSendSetLineCoding - Control interface has gone");
        return;
	}	
	
	// First check that Set Line Coding is supported
	
    if (!(fACMCapabilities & ACM_DeviceSuppControl))
    {
        XTRACE(this, 0, 0, "USBSendSetLineCoding - SetLineCoding not supported");
        return;
    }
    
    lineParms = (LineCoding *)IOMalloc(lcLen);
    if (!lineParms)
    {
        XTRACE(this, 0, 0, "USBSendSetLineCoding - allocate lineParms failed");
        return;
    }
    bzero(lineParms, lcLen); 
	
    // Convert BaudRate - intel format doubleword (32 bits) 
    
    OSWriteLittleInt32(lineParms, dwDTERateOffset, BaudRate);
    lineParms->bCharFormat = StopBits - 2;
    lineParms->bParityType = TX_Parity - 1;
    lineParms->bDataBits = CharLength;
    
    MER = (IOUSBDevRequest*)IOMalloc(sizeof(IOUSBDevRequest));
    if (!MER)
    {
        XTRACE(this, 0, 0, "USBSendSetLineCoding - allocate MER failed");
        return;
    }
    bzero(MER, sizeof(IOUSBDevRequest));
	
    // now build the Management Element Request
    
    MER->bmRequestType = USBmakebmRequestType(kUSBOut, kUSBClass, kUSBInterface);
    MER->bRequest = kUSBSET_LINE_CODING;
    MER->wValue = 0;
    MER->wIndex = fCommInterfaceNumber;
    MER->wLength = lcLen;
    MER->pData = lineParms;
    
    fMERCompletionInfo.parameter = MER;
	
    rc = fMSP430Interface->GetDevice()->DeviceRequest(MER, &fMERCompletionInfo);
    if (rc != kIOReturnSuccess)
    {
        XTRACE(this, MER->bRequest, rc, "USBSendSetLineCoding - error issueing DeviceRequest");
        IOFree(MER->pData, lcLen);
        IOFree(MER, sizeof(IOUSBDevRequest));
    }
    
}/* end USBSendSetLineCoding */

/****************************************************************************************************/
//
//		Method:		MSP430LPCDC::USBSendSetControlLineState
//
//		Inputs:		RTS - true(set RTS), false(clear RTS)
//				DTR - true(set DTR), false(clear DTR)
//
//		Outputs:	
//
//		Desc:		Set up and send SetControlLineState Management Element Request(MER).
//
/****************************************************************************************************/

void MSP430LPCDC::USBSendSetControlLineState(bool RTS, bool DTR)
{
    IOReturn		rc;
    IOUSBDevRequest	*MER;
    UInt16		CSBitmap = 0;
	
    XTRACE(this, 0, 0, "USBSendSetControlLineState");
	
	if (!fMSP430Interface)
	{
		XTRACE(this, 0, 0, "USBSendSetControlLineState - Control interface has gone");
        return;
	}	
	
	// First check that Set Control Line State is supported
	
    if (!(fACMCapabilities & ACM_DeviceSuppControl))
    {
        XTRACE(this, 0, 0, "USBSendSetControlLineState - SetControlLineState not supported");
        return;
    }
	
    MER = (IOUSBDevRequest*)IOMalloc(sizeof(IOUSBDevRequest));
    if (!MER)
    {
        XTRACE(this, 0, 0, "USBSendSetControlLineState - allocate MER failed");
        return;
    }
    bzero(MER, sizeof(IOUSBDevRequest));
	
    // now build the Management Element Request
    
    MER->bmRequestType = USBmakebmRequestType(kUSBOut, kUSBClass, kUSBInterface);
    MER->bRequest = kUSBSET_CONTROL_LINE_STATE;
    if (RTS)
        CSBitmap |= kRTSOn;
    if (DTR)
        CSBitmap |= kDTROn;
    MER->wValue = CSBitmap;
    MER->wIndex = fCommInterfaceNumber;
    MER->wLength = 0;
    MER->pData = NULL;
    
    fMERCompletionInfo.parameter = MER;
	
    rc = fMSP430Interface->GetDevice()->DeviceRequest(MER, &fMERCompletionInfo);
    if (rc != kIOReturnSuccess)
    {
        XTRACE(this, MER->bRequest, rc, "USBSendSetControlLineState - error issueing DeviceRequest");
        IOFree(MER, sizeof(IOUSBDevRequest));
    }
    
}/* end USBSendSetControlLineState */

/****************************************************************************************************/
//
//		Method:		MSP430LPCDC::USBSendBreak
//
//		Inputs:		sBreak - true(set Break), false(clear Break)
//
//		Outputs:	
//
//		Desc:		Set up and send SendBreak Management Element Request(MER).
//
/****************************************************************************************************/

void MSP430LPCDC::USBSendBreak(bool sBreak)
{
    IOReturn		rc;
    IOUSBDevRequest	*MER;
	
    XTRACE(this, 0, 0, "USBSendBreak");
	
	if (!fMSP430Interface)
	{
		XTRACE(this, 0, 0, "USBSendBreak - Control interface has gone");
        return;
	}
	
	// First check that Send Break is supported
	
    if (!(fACMCapabilities & ACM_DeviceSuppBreak))
    {
        XTRACE(this, 0, 0, "USBSendBreak - SendBreak not supported");
        return;
    }
	
    MER = (IOUSBDevRequest*)IOMalloc(sizeof(IOUSBDevRequest));
    if (!MER)
    {
        XTRACE(this, 0, 0, "USBSendBreak - allocate MER failed");
        return;
    }
    bzero(MER, sizeof(IOUSBDevRequest));
	
    // now build the Management Element Request
    
    MER->bmRequestType = USBmakebmRequestType(kUSBOut, kUSBClass, kUSBInterface);
    MER->bRequest = kUSBSEND_BREAK;
    if (sBreak)
    {
        MER->wValue = 0xFFFF;
    } else {
        MER->wValue = 0;
    }
    MER->wIndex = fCommInterfaceNumber;
    MER->wLength = 0;
    MER->pData = NULL;
    
    fMERCompletionInfo.parameter = MER;
	
    rc = fMSP430Interface->GetDevice()->DeviceRequest(MER, &fMERCompletionInfo);
    if (rc != kIOReturnSuccess)
    {
        XTRACE(this, MER->bRequest, rc, "USBSendBreak - error issueing DeviceRequest");
        IOFree(MER, sizeof(IOUSBDevRequest));
    }
    
}/* end USBSendBreak */


/****************************************************************************************************/
//
//		Method:		MSP430LPCDC::checkInterfaceNumber
//
//		Inputs:		dataDriver - the data driver enquiring
//
//		Outputs:	
//
//		Desc:		Called by the data driver to ask if this is the correct
//				control interface driver. 
//
/****************************************************************************************************/

bool MSP430LPCDC::checkInterfaceNumber(MSP430LPCDC *dataDriver)
{
    IOUSBInterface	*dataInterface;
    
    XTRACEP(this, 0, dataDriver, "checkInterfaceNumber");
    
    // First check we have the same provider (Device)
    
    dataInterface = OSDynamicCast(IOUSBInterface, dataDriver->getProvider());
    if (dataInterface == NULL)
    {
        XTRACE(this, 0, 0, "checkInterfaceNumber - Error getting Data provider");
        return FALSE;
    }
    
    XTRACEP(this, dataInterface->GetDevice(), fMSP430Interface->GetDevice(), "checkInterfaceNumber - Checking device");
    if (dataInterface->GetDevice() == fMSP430Interface->GetDevice())
    {
        
        // Then check to see if it's the correct data interface number
        
        if (dataDriver->fPort.DataInterfaceNumber == fDataInterfaceNumber)
        {
            this->fDataDriver = dataDriver;
            return true;
        } else {
            XTRACE(this, 0, 0, "checkInterfaceNumber - Not correct interface number");
        }
    } else {
        XTRACE(this, 0, 0, "checkInterfaceNumber - Not correct device");
    }
    
    return false;
    
}/* end checkInterfaceNumber */

/****************************************************************************************************/
//
//		Method:		MSP430LPCDC::resetDevice
//
//		Inputs:		
//
//		Outputs:	
//
//		Desc:		Check to see if we need to reset the device on wakeup. 
//
/****************************************************************************************************/

void MSP430LPCDC::resetDevice(void)
{
    IOReturn 	rtn = kIOReturnSuccess;
    USBStatus	status;
    bool	reset = false;
	bool	reenum = false;
    
    XTRACE(this, 0, 0, "resetDevice");
	
	if ((fStopping) || (fMSP430Interface == NULL) || (fTerminate))
	{
		XTRACE(this, 0, 0, "resetDevice - returning early");
		return;
	}
	
    // Let everything settle down first
	
    //	IOSleep(100);
    
    rtn = fMSP430Interface->GetDevice()->GetDeviceStatus(&status);
    if (rtn != kIOReturnSuccess)
    {
        XTRACE(this, 0, rtn, "resetDevice - Error getting device status");
        reset = true;
    } else {
        status = USBToHostWord(status);
        XTRACE(this, 0, status, "resetDevice - Device status");
        if (status & kDeviceSelfPowered)			// Self powered devices that need "reset on close" will also be reset
        {
			if (fDataDriver)
			{
				if (fDataDriver->fResetOnClose)
				{
					reset = true;
				}
				if (fDataDriver->fEnumOnWake)
				{
					reenum = true;
				}
			}
        }
    }
	
	if ((fMSP430Interface) && (fDataDriver))			// Make sure the interface and data driver are still around
	{
		if (reenum)										// Enumeration takes precedent
		{
			fDataDriver->fTerminate = true;				// Warn the data driver early
			XTRACE(this, 0, 0, "resetDevice - Device is being re-enumerated");
			rtn = fMSP430Interface->GetDevice()->ReEnumerateDevice(0);
			if (rtn != kIOReturnSuccess)
			{
				XTRACE(this, 0, rtn, "resetDevice - ReEnumerateDevice failed");
			}
		} else {
			fDataDriver->fTerminate = false;
			if (reset)
			{
				XTRACE(this, 0, 0, "resetDevice - Device is being reset");
				rtn = fMSP430Interface->GetDevice()->ResetDevice();
				if (rtn != kIOReturnSuccess)
				{
					XTRACE(this, 0, rtn, "resetDevice - ResetDevice failed");
				}
			}
		}
    } else {
		XTRACE(this, 0, 0, "resetDevice - Control interface or Data driver has gone");
	}
	
}/* end resetDevice */



/****************************************************************************************************/
//
//		Method:		MSP430LPCDC::initForPM
//
//		Inputs:		provider - my provider
//
//		Outputs:	return code - true(initialized), false(failed)
//
//		Desc:		Add ourselves to the power management tree so we can do
//				the right thing on sleep/wakeup. 
//
/****************************************************************************************************/

bool MSP430LPCDC::initForPM(IOService *provider)
{
    XTRACE(this, 0, 0, "initForPM");
    
    fPowerState = kCDCPowerOnState;				// init our power state to be 'on'
    PMinit();							// init power manager instance variables
    provider->joinPMtree(this);					// add us to the power management tree
    if (pm_vars != NULL)
    {
        
        // register ourselves with ourself as policy-maker
        
        registerPowerDriver(this, gOurPowerStates, kNumCDCStates);
        return true;
    } else {
        XTRACE(this, 0, 0, "initForPM - Initializing power manager failed");
    }
    
    return false;
    
}/* end initForPM */

/****************************************************************************************************/
//
//		Method:		MSP430LPCDC::initialPowerStateForDomainState
//
//		Inputs:		flags - 
//
//		Outputs:	return code - Current power state
//
//		Desc:		Request for our initial power state. 
//
/****************************************************************************************************/

unsigned long MSP430LPCDC::initialPowerStateForDomainState(IOPMPowerFlags flags)
{
    
    XTRACE(this, 0, flags, "initialPowerStateForDomainState");
    
    return fPowerState;
    
}/* end initialPowerStateForDomainState */

/****************************************************************************************************/
//
//		Method:		MSP430LPCDC::setPowerState
//
//		Inputs:		powerStateOrdinal - on/off
//
//		Outputs:	return code - IOPMNoErr, IOPMAckImplied or IOPMNoSuchState
//
//		Desc:		Request to turn device on or off. 
//
/****************************************************************************************************/

IOReturn MSP430LPCDC::setPowerState(unsigned long powerStateOrdinal, IOService *whatDevice)
{
    
    XTRACE(this, 0, powerStateOrdinal, "setPowerState");
	
	if (powerStateOrdinal != fPowerState)
	{
		fPowerState = powerStateOrdinal;
		switch (fPowerState)
		{
			case kCDCPowerOffState:
				
                // Warn our data driver and clean up any threads, if we can
				
				if (fDataDriver)
				{
					XTRACE(this, 0, fDataDriver->fThreadSleepCount, "setPowerState - Warning data driver");
					
					fDataDriver->fTerminate = true;
                    fDataDriver->clearSleepingThreads();
				}
				
				break;
				
			case kCDCPowerOnState:
				
                // See if we need to reset or reenumerate this device
				
				resetDevice();
				
				break;
                
			default:
				return IOPMNoSuchState;
		}
	}
	
	return IOPMAckImplied;
    
}/* end setPowerState */


/****************************************************************************************************/
//
//		Method:		MSP430LPCDCUserClient::start
//
//		Inputs:		provider - my provider
//
//		Outputs:	return code - true(it worked), false (it didn't)
//
//		Desc:		Start the user client task. 
//
/****************************************************************************************************/

bool MSP430LPCDCUserClient::start(IOService *provider)
{

    XTRACE(this, 0, 0, "start");
    
    if (super::start(provider) == false)
    {
        XTRACE(this, 0, 0, "start - Provider start failed");
        return false;
    }
    
    fProvider = OSDynamicCast(MSP430LPCDC, provider);
    if (!fProvider)
    {
        XTRACE(this, 0, 0, "start - Provider invalid");
		return false;
    }
    
        // Initialize the call structure
    
    fMethods[0].object = this;
    fMethods[0].func   = (IOMethod)&MSP430LPCDCUserClient::doRequest;
    fMethods[0].count0 = 0xFFFFFFFF;			/* One input  as big as I need */
    fMethods[0].count1 = 0xFFFFFFFF;			/* One output as big as I need */
    fMethods[0].flags  = kIOUCStructIStructO;
    
    return true;
    
}/* end start */

/****************************************************************************************************/
//
//		Method:		MSP430LPCDCUserClient::clientClose
//
//		Inputs:		
//
//		Outputs:	return code - kIOReturnSuccess
//
//		Desc:		Close things down. 
//
/****************************************************************************************************/

IOReturn MSP430LPCDCUserClient::clientClose()
{
    
    XTRACE(this, 0, 0, "clientClose");
    
    if (!fProvider)
    {
        XTRACE(this, 0, 0, "clientClose - Not attached");
        return kIOReturnNotAttached;
    }

        // Make sure it's open before we close it.
    
    if (fProvider->isOpen(this))
        fProvider->close(this);

    fTask = NULL;
    fProvider = NULL;
           
    return kIOReturnSuccess;
    
}/* end clientClose */

/****************************************************************************************************/
//
//		Method:		MSP430LPCDCUserClient::clientDied
//
//		Inputs:		
//
//		Outputs:	return code - kIOReturnSuccess
//
//		Desc:		Close it down now. 
//
/****************************************************************************************************/

IOReturn MSP430LPCDCUserClient::clientDied()
{

    XTRACE(this, 0, 0, "clientDied");
    
    return clientClose();
    
}/* end clientDied */

/****************************************************************************************************/
//
//		Method:		MSP430LPCDCUserClient::doRequest
//
//		Inputs:		pIn - the input buffer
//					pOut - the output buffer
//					inputSize - Size of input buffer
//					pOutPutSize - Size of output buffer
//
//		Outputs:	pOutSize - Number of bytes returned
//					return code - kIOReturnSuccess or kIOReturnBadArgument
//
//		Desc:		Execute the client request. 
//
/****************************************************************************************************/

IOReturn MSP430LPCDCUserClient::doRequest(void *pIn, void *pOut, IOByteCount inputSize, IOByteCount *pOutPutSize)
{
    UInt8	*input;
    
    XTRACE(this, 0, 0, "doRequest");
    
        // Make sure we actually have a provider
        
    if (!fProvider)
    {
        XTRACE(this, 0, 0, "doRequest - Not attached");
        return kIOReturnNotAttached;
    }

	// check first byte of input data for a command code
        
    if (pIn && (inputSize > 0))
    {
        input = (UInt8 *)pIn;
 
            // 1st byte of input has request ID
            
        switch (*input) 
        {
            case cmdACMData_Message:
                return ACMDataMessage(pIn, pOut, inputSize, pOutPutSize);
                		    
            default:
               XTRACE(this, 0, *input, "doRequest - Invalid command");
               break;    
        }
    } else {
        XTRACE(this, 0, inputSize, "doRequest - pIn/pOut or size error");
    }

    return kIOReturnBadArgument;
    
}/* end doRequest */

/****************************************************************************************************/
//
//		Method:		MSP430LPCDCUserClient::ACMDataMessage
//
//		Inputs:		pIn - the input structure
//					pOut - the output structure
//					inputSize - Size of the input structure
//					pOutSize - Size of the output structure
//
//		Outputs:	return code - kIOReturnSuccess
//
//		Desc:		Process the message 
//
/****************************************************************************************************/

IOReturn MSP430LPCDCUserClient::ACMDataMessage(void *pIn, void *pOut, IOByteCount inputSize, IOByteCount *pOutPutSize)
{
	dataParms	*input = (dataParms *)pIn;
    statusData	*output = (statusData *)pOut;
    
    XTRACE(this, 0, 0, "ACMDataMessage");
    
	switch (input->message)
    {
		case noWarning:
			XTRACE(this, 0, 0, "ACMDataMessage - noWarning");
			if ((input->vendor == fProvider->fVendorID) && (input->product == fProvider->fProductID))
			{
				XTRACE(this, fProvider->fVendorID, fProvider->fProductID, "ACMDataMessage - Unplug warning dialog is being suppressed");
				fProvider->fSuppressWarning = true;
				output->status = kSuccess;
			} else {
				XTRACE(this, 0, 0, "ACMDataMessage - noWarning, not my device");
				output->status = kError;
			}
			break;
		case warning:
			XTRACE(this, 0, 0, "ACMDataMessage - warning");
			if ((input->vendor == fProvider->fVendorID) && (input->product == fProvider->fProductID))
			{
				XTRACE(this, fProvider->fVendorID, fProvider->fProductID, "ACMDataMessage - Unplug warning dialog is being re-instated");
				fProvider->fSuppressWarning = false;
				output->status = kSuccess;
			} else {
				XTRACE(this, 0, 0, "ACMDataMessage - warning, not my device");
				output->status = kError;
			}
			break;
		default:
			XTRACE(this, 0, 0, "ACMDataMessage - Invalid message");
			output->status = kError;
			break;
	}

    return kIOReturnSuccess;
    
}/* end ACMDataMessage */