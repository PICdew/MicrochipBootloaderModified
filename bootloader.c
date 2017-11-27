/*******************************************************************************
  Company:
    Microchip Technology Inc.

  File Name:
    bootloader.c
    
  Summary:
    Interface for the Bootloader library.

  Description:
    This file contains the interface definition for the Bootloader library.
*******************************************************************************/

//DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2014 released Microchip Technology Inc.  All rights reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED AS IS WITHOUT WARRANTY OF ANY KIND,
EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF
MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE.
IN NO EVENT SHALL MICROCHIP OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER
CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR
OTHER LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE OR
CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT OF
SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
(INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.
*******************************************************************************/
//DOM-IGNORE-END

// *****************************************************************************
// *****************************************************************************
// Section: Macro Definitions
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
// *****************************************************************************
// Section: Include Files
// *****************************************************************************
// *****************************************************************************
#include <sys/attribs.h>

#include "bootloader/src/bootloader.h"
#include "peripheral/nvm/plib_nvm.h"
#include "peripheral/int/plib_int.h"
#include "system/devcon/sys_devcon.h"
#include "system/reset/sys_reset.h"

BOOTLOADER_DATA bootloaderData __attribute__((coherent, aligned(16)));

void Bootloader_BufferEventHandler(DATASTREAM_BUFFER_EVENT buffEvent,
                            DATASTREAM_BUFFER_HANDLE hBufferEvent,
                            uint16_t context );

/********************************************************************
* Function:     ConvertAsciiToHex()
*
* Precondition:
*
* Input:        ASCII buffer and hex buffer.
*
* Output:
*
* Side Effects: No return from here.
*
* Overview:     Converts ASCII to Hex.
*
*
* Note:         None.
********************************************************************/
void ConvertAsciiToHex(uint8_t* asciiRec, uint8_t* hexRec)
{
    uint8_t i = 0;
    uint8_t k = 0;
    uint8_t hex;


    while((asciiRec[i] >= 0x30) && (asciiRec[i] <= 0x66))
    {
            // Check if the ASCII values are in alpha numeric range.

            if(asciiRec[i] < 0x3A)
            {
                    // Numerical representation in ASCII found.
                    hex = asciiRec[i] & 0x0F;
            }
            else
            {
                    // Alphabetical value.
                    hex = 0x09 + (asciiRec[i] & 0x0F);
            }

            // Following logic converts 2 bytes of ASCII to 1 byte of hex.
            k = i%2;

            if(k)
            {
                    hexRec[i>>1] |= hex;

            }
            else
            {
                    hexRec[i>>1] = (hex << 4) & 0xF0;
            }
            i++;
    }

}

/**
 * Static table used for the table_driven implementation.
 *****************************************************************************/
static const uint16_t crc_table[16] =
{
    0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50a5, 0x60c6, 0x70e7,
    0x8108, 0x9129, 0xa14a, 0xb16b, 0xc18c, 0xd1ad, 0xe1ce, 0xf1ef
};

/********************************************************************
* Function:     CalculateCrc()
*
* Precondition:
*
* Input:        Data pointer and data length
*
* Output:       CRC.
*
* Side Effects: None.
*
* Overview:     Calculates CRC for the given data and len
*
*
* Note:         None.
********************************************************************/
uint32_t APP_CalculateCrc(uint8_t *data, uint32_t len)
{
    uint32_t i;
    uint16_t crc = 0;
    
    while(len--)
    {
        i = (crc >> 12) ^ (*data >> 4);
        crc = crc_table[i & 0x0F] ^ (crc << 4);
        i = (crc >> 12) ^ (*data >> 0);
        crc = crc_table[i & 0x0F] ^ (crc << 4);
        data++;
    }

    return (crc & 0xFFFF);
}
/******************************************************************************
  Function:
    SYS_MODULE_OBJ Bootloader_Initialize(const SYS_MODULE_INDEX   moduleIndex,
                              const SYS_MODULE_INIT    * const moduleInit)

  Summary:
    Initializes primitive data structures for the general features
    of the primitive layer.

  Description:
    Initializes external and internal data structure for the general
    features of the primitive layer.

    This function must be called at system initialization.

  Remarks:
    None.
*/
void Bootloader_Initialize ( const BOOTLOADER_INIT *drvBootloaderInit )
{
    /* Place the App state machine in it's initial state. */
    bootloaderData.currentState = BOOTLOADER_CHECK_FOR_TRIGGER;
    bootloaderData.cmdBufferLength = 0;
    bootloaderData.streamHandle = DRV_HANDLE_INVALID;
    bootloaderData.datastreamStatus = DRV_CLIENT_STATUS_ERROR;
    bootloaderData.usrBufferEventComplete = false;

    bootloaderData.type = drvBootloaderInit->drvType;

    bootloaderData.FlashEraseFunc = (BOOTLOADER_CALLBACK)NULL;
    bootloaderData.RestoreUserSettingsFunc = (BOOTLOADER_CALLBACK)NULL;
    bootloaderData.BlankCheckFunc = (BOOTLOADER_CALLBACK)NULL;
    bootloaderData.ProgramCompleteFunc = (BOOTLOADER_CALLBACK)NULL;
    bootloaderData.ForceBootloadFunc = (BOOTLOADER_CALLBACK)NULL;
    bootloaderData.softReset = (SYS_RESET_ReasonGet() & RESET_REASON_SOFTWARE) == RESET_REASON_SOFTWARE;
    SYS_RESET_ReasonClear(RESET_REASON_SOFTWARE);
    /* Delay to allow the internal pullups to stabilize */
    _CP0_SET_COUNT(0);
    while (_CP0_GET_COUNT() < SYS_CLK_FREQ / 5000);
}

void BOOTLOADER_FlashEraseRegister(BOOTLOADER_CALLBACK newFunc)
{
    bootloaderData.FlashEraseFunc = newFunc;
}

void BOOTLOADER_UserSettingsRestoreRegister(BOOTLOADER_CALLBACK newFunc)
{
    bootloaderData.RestoreUserSettingsFunc = newFunc;
}

void BOOTLOADER_BlankCheckRegister(BOOTLOADER_CALLBACK newFunc)
{
    bootloaderData.BlankCheckFunc = newFunc;
}

void BOOTLOADER_ProgramCompleteRegister(BOOTLOADER_CALLBACK newFunc)
{
    bootloaderData.ProgramCompleteFunc = newFunc;
}

void BOOTLOADER_ForceBootloadRegister(BOOTLOADER_CALLBACK newFunc)
{
    bootloaderData.ForceBootloadFunc = newFunc;
}

// *****************************************************************************
/* Function:
    void Bootloader_Tasks (SYS_MODULE_INDEX index);

  Summary:
    Maintains the Bootloader module state machine. It manages the Bootloader Module object list
    items and responds to Bootloader Module primitive events.

*/
void Bootloader_Tasks ()
{
    size_t BuffLen=0;
    uint16_t crc;
    unsigned int i;
    void (*fptr)(void);

    /* Check the application state*/
    switch ( bootloaderData.currentState )
    {
        case BOOTLOADER_CHECK_FOR_TRIGGER:
        {
            bool forceBootloadMode = false;
            if (bootloaderData.ForceBootloadFunc != NULL)
            {
                forceBootloadMode = (1 == bootloaderData.ForceBootloadFunc());
            }
            /* Check if the User reset address is erased. */
            forceBootloadMode |= (0xFFFFFFFF == *(unsigned int *)BOOTLOADER_RESET_ADDRESS);
            
            if (forceBootloadMode)
            {
                /* Override any soft reset from the bootloader, so we will do
                 one when bootloader mode is done. */
                bootloaderData.softReset = false;
                bootloaderData.currentState = BOOTLOADER_OPEN_DATASTREAM;
            }
            else
            {
                /* User reset address is not erased. Start program. */
                bootloaderData.currentState = BOOTLOADER_CLOSE_DATASTREAM;
            }
            break;
        }

        case BOOTLOADER_OPEN_DATASTREAM:
        {
            bootloaderData.streamHandle = DATASTREAM_Open(DRV_IO_INTENT_READWRITE | DRV_IO_INTENT_NONBLOCKING);

            if (bootloaderData.streamHandle != DRV_HANDLE_INVALID )
            {
                DATASTREAM_BufferEventHandlerSet(bootloaderData.streamHandle, Bootloader_BufferEventHandler, APP_USR_CONTEXT);
                bootloaderData.currentState = BOOTLOADER_GET_COMMAND;
            }
            break;
        }

        case BOOTLOADER_PROCESS_COMMAND:
        {
            Bootloader_ProcessBuffer(&bootloaderData);
            break;
        }

        case BOOTLOADER_GET_COMMAND:
        {
            /* Get the datastream driver status */
            bootloaderData.datastreamStatus = DATASTREAM_ClientStatus( bootloaderData.streamHandle );
            /* Check if client is ready or not */
            if ( bootloaderData.datastreamStatus == DRV_CLIENT_STATUS_READY )
            {
                bootloaderData.bufferSize = 512;
                
                 DATASTREAM_Data_Read( &(bootloaderData.datastreamBufferHandle),
                        bootloaderData.data.buffers.buff1, bootloaderData.bufferSize);

                if ( bootloaderData.datastreamBufferHandle == DRV_HANDLE_INVALID )
                {
                    /* Set the app state to invalid */
                    bootloaderData.currentState = BOOTLOADER_ERROR;
                }
                else
                {
                    /* Set the App. state to wait for done */
                    bootloaderData.prevState    = BOOTLOADER_GET_COMMAND;
                    bootloaderData.currentState = BOOTLOADER_WAIT_FOR_DONE;
                }
            }
            break;
        }

        case BOOTLOADER_WAIT_FOR_DONE:
        {
            /* check if the datastream buffer event is complete or not */
            if (bootloaderData.usrBufferEventComplete)
            {
                bootloaderData.usrBufferEventComplete = false;
                
                /* Get the next App. State */
                switch (bootloaderData.prevState)
                {
                    case BOOTLOADER_GET_COMMAND:
                        bootloaderData.currentState = BOOTLOADER_PROCESS_COMMAND;
                        break;
                    case BOOTLOADER_SEND_RESPONSE:
                    default:
                        bootloaderData.currentState = BOOTLOADER_GET_COMMAND;
                        break;
                }
            }
            break;
        }

    case BOOTLOADER_WAIT_FOR_NVM:
       if (PLIB_NVM_FlashWriteCycleHasCompleted(NVM_ID_0))
       {
           if(bootloaderData.RestoreUserSettingsFunc != NULL)
               bootloaderData.RestoreUserSettingsFunc();
         bootloaderData.currentState = BOOTLOADER_SEND_RESPONSE;
         PLIB_NVM_MemoryModifyInhibit(NVM_ID_0);
       }
       break;

    case BOOTLOADER_SEND_RESPONSE:
        {
            if(bootloaderData.bufferSize)
            {
                /* Calculate the CRC of the response*/
                crc = APP_CalculateCrc(bootloaderData.data.buffers.buff1, bootloaderData.bufferSize);
                bootloaderData.data.buffers.buff1[bootloaderData.bufferSize++] = (uint8_t)crc;
                bootloaderData.data.buffers.buff1[bootloaderData.bufferSize++] = (crc>>8);

                bootloaderData.data.buffers.buff2[BuffLen++] = SOH;

                for (i = 0; i < bootloaderData.bufferSize; i++)
                {
                    if ((bootloaderData.data.buffers.buff1[i] == EOT) || (bootloaderData.data.buffers.buff1[i] == SOH)
                        || (bootloaderData.data.buffers.buff1[i] == DLE))
                    {
                        bootloaderData.data.buffers.buff2[BuffLen++] = DLE;
                    }
                    bootloaderData.data.buffers.buff2[BuffLen++] = bootloaderData.data.buffers.buff1[i];
                }

                bootloaderData.data.buffers.buff2[BuffLen++] = EOT;
                bootloaderData.bufferSize = 0;

                DATASTREAM_Data_Write( &(bootloaderData.datastreamBufferHandle),
                        bootloaderData.data.buffers.buff2, BuffLen);

                if ( bootloaderData.datastreamBufferHandle == DRV_HANDLE_INVALID )
                {
                    bootloaderData.currentState = BOOTLOADER_ERROR;
                }
                else
                {
                    bootloaderData.prevState = BOOTLOADER_SEND_RESPONSE;
                    bootloaderData.currentState = BOOTLOADER_WAIT_FOR_DONE;
                }
            }
            break;
        }

        case BOOTLOADER_CLOSE_DATASTREAM:
            DATASTREAM_Close();
        case BOOTLOADER_ENTER_APPLICATION:
            /* Do a soft reset in order to reset peripherals */
            /* Disable Global Interrupts */
            PLIB_INT_Disable(INT_ID_0);
            fptr = (void (*)(void))BOOTLOADER_RESET_ADDRESS;
            fptr();
            break;

        case BOOTLOADER_ERROR:
            /* The application comes here when the demo
             * has failed.*/
            break;

        default:
            bootloaderData.currentState = BOOTLOADER_ERROR;
            break;
    }
    /* Maintain Device Drivers */
    DATASTREAM_Tasks();
}
