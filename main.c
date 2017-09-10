/*
 * Copyright (c) 2015-2016, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/***** Includes *****/
#include <stdlib.h>
#include <xdc/std.h>
#include <xdc/cfg/global.h>
#include <xdc/runtime/System.h>

#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>

#include <ti/mw/lcd/LCDDogm1286.h>

/* Drivers */
#include <ti/drivers/rf/RF.h>
#include <ti/drivers/PIN.h>

/* Board Header files */
#include "Board.h"
#include "smartrf_settings/smartrf_settings.h"

static PIN_State ledPinState;

// application pin table - which pins should we initialize with what settings/flags
PIN_Config pinTable[] =
{
    // all LEDs
    Board_PIN_LED1 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW  | PIN_PUSHPULL | PIN_DRVSTR_MAX,
    Board_PIN_LED2 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW  | PIN_PUSHPULL | PIN_DRVSTR_MAX,
    Board_PIN_LED3 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW  | PIN_PUSHPULL | PIN_DRVSTR_MAX,
    Board_PIN_LED4 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW  | PIN_PUSHPULL | PIN_DRVSTR_MAX,
    // all keys
    Board_KEY_UP   | PIN_INPUT_EN | PIN_PULLUP | PIN_IRQ_NEGEDGE,
    Board_KEY_DOWN | PIN_INPUT_EN | PIN_PULLUP | PIN_IRQ_NEGEDGE,
    Board_KEY_RIGHT| PIN_INPUT_EN | PIN_PULLUP | PIN_IRQ_NEGEDGE,
    Board_KEY_LEFT | PIN_INPUT_EN | PIN_PULLUP | PIN_IRQ_NEGEDGE,
    Board_KEY_SELECT| PIN_INPUT_EN | PIN_PULLUP | PIN_IRQ_NEGEDGE,
    // 3V pin for LCD
    Board_3V3_EN   | PIN_GPIO_OUTPUT_EN | PIN_GPIO_HIGH | PIN_PUSHPULL,
    PIN_TERMINATE
};

static LCD_Handle lcdHandle;
Char lcdBuffer_main[LCD_BYTES] = { 0 };
Char lcdBuffer_rssi[LCD_BYTES] = { 0 };

LCD_Buffer lcdBuffers[] = {
    { lcdBuffer_main, LCD_BYTES, {NULL} },
    { lcdBuffer_rssi, LCD_BYTES, {NULL} }
};

/***** Defines *****/
#define TX_TASK_STACK_SIZE 2048
#define TX_TASK_PRIORITY   2

/* Packet TX Configuration */
#define PAYLOAD_LENGTH      5

/* interval step in RF ticks */
#define PINTERVAL_STEP 2000000              /* 0.5s */
/* minimum interval between packets */
#define PINTERVAL_MIN PINTERVAL_STEP        /* 0.5s */
/* maximum interval between packets */
#define PINTERVAL_MAX PINTERVAL_STEP*4      /* 2s */

/* current packet interval */
uint32_t packet_interval = PINTERVAL_MIN; // 0.5s

/***** Prototypes *****/
static void txTaskFunction(UArg arg0, UArg arg1);

/***** Variable declarations *****/
static Task_Params txTaskParams;
Task_Struct txTask;    /* not static so you can see in ROV */
static uint8_t txTaskStack[TX_TASK_STACK_SIZE];

static RF_Object rfObject;
static RF_Handle rfHandle;

uint32_t curtime;
static uint8_t packet[PAYLOAD_LENGTH];
static PIN_Handle pinHandle;

#include "RFQueue.h"

#define NUM_DATA_ENTRIES 2
#define NUM_APPENDED_BYTES 2

/* Buffer which contains all Data Entries for receiving data.
 * Pragmas are needed to make sure this buffer is 4 byte aligned (requirement from the RF Core) */
#if defined(__TI_COMPILER_VERSION__)
    #pragma DATA_ALIGN (rxDataEntryBuffer, 4);
        static uint8_t rxDataEntryBuffer[RF_QUEUE_DATA_ENTRY_BUFFER_SIZE(NUM_DATA_ENTRIES,
                                                                         PAYLOAD_LENGTH,
                                                                 NUM_APPENDED_BYTES)];
#elif defined(__GNUC__)
        static uint8_t rxDataEntryBuffer [RF_QUEUE_DATA_ENTRY_BUFFER_SIZE(NUM_DATA_ENTRIES,
                                                                          PAYLOAD_LENGTH, NUM_APPENDED_BYTES)] __attribute__ ((aligned (4)));
#else
    #error This compiler is not supported.
#endif

static dataQueue_t dataQueue;
static rfc_dataEntryGeneral_t* currentDataEntry;

/***** Function definitions *****/
void TxTask_init()
{
    Task_Params_init(&txTaskParams);
    txTaskParams.stackSize = TX_TASK_STACK_SIZE;
    txTaskParams.priority = TX_TASK_PRIORITY;
    txTaskParams.stack = &txTaskStack;
    txTaskParams.arg0 = (UInt)1000000;

    Task_construct(&txTask, txTaskFunction, &txTaskParams, NULL);
}

int bufIndex = 0;

uint32_t seq = 0;
int mode = 0; // 0 = beacon, 1 = terrain
int32_t rssi = 0;
uint32_t msDelay = 0;

void updateLCD()
{
    char outbuf[12];

    // primary LCD buffer - main info
    LCD_bufferClear(lcdHandle, 0);
    LCD_bufferPrintString(lcdHandle, 0, "Kennny's Twins", 0, LCD_PAGE0);
    // beacon mode - show TX packet count and current delay
    if (mode == 0)
    {
        LCD_bufferPrintString(lcdHandle, 0, "Mode:  beacon", 0, LCD_PAGE2);
        LCD_bufferPrintString(lcdHandle, 0, "TX:    ", 0, LCD_PAGE3);
        LCD_bufferPrintString(lcdHandle, 0, "Delay: ", 0, LCD_PAGE4);
    }
    else // terrain mote mode - show RX packet count, current delay and RSSI
    {
        LCD_bufferPrintString(lcdHandle, 0, "Mode:  terrain", 0, LCD_PAGE2);
        LCD_bufferPrintString(lcdHandle, 0, "RX:    ", 0, LCD_PAGE3);
        LCD_bufferPrintString(lcdHandle, 0, "Delay: ", 0, LCD_PAGE4);
        LCD_bufferPrintString(lcdHandle, 0, "RSSI:  ", 0, LCD_PAGE5);

        // print RSSI value
        ltoa(rssi, outbuf);
        LCD_bufferPrintString(lcdHandle, 0, outbuf, 7*6 /* 6 = char width */, LCD_PAGE5);
    }

    // print SEQ number (shared)
    ltoa(seq, outbuf);
    LCD_bufferPrintString(lcdHandle, 0, outbuf, 7*6 /* 6 = char width */, LCD_PAGE3);

    // print delay number (shared)
    ltoa(msDelay, outbuf);
    LCD_bufferPrintString(lcdHandle, 0, outbuf, 7*6 /* 6 = char width */, LCD_PAGE4);
    LCD_bufferPrintString(lcdHandle, 0, "ms", (msDelay >= 1000 ? 11 : 10)*6 /* 6 = char width */, LCD_PAGE4);



    // secondary buffer, additional data, if any
    LCD_bufferClear(lcdHandle, 1);
    LCD_bufferPrintString(lcdHandle, 1, "Kennny's Twins", 0, LCD_PAGE0);
    // beacon mode - no data
    if (mode == 0)
    {
        LCD_bufferPrintString(lcdHandle, 1, "No additional data", 0, LCD_PAGE2);
    }
    else // terrain mode - RSSI statistics
    {
        // TODO: RSSI data
    }

    LCD_update(lcdHandle, bufIndex);
}

// flag for indicating RX
int rxFlag = 0;

void callback(RF_Handle h, RF_CmdHandle ch, RF_EventMask e)
{
    if (e & RF_EventRxEntryDone)
    {
        rxFlag = 1;

        currentDataEntry = RFQueue_getDataEntry();

        uint8_t* packetDataPointer = (uint8_t*)(&currentDataEntry->data);

        // we accept only packets containing HELLO
        if (memcmp(packetDataPointer, "HELLO", 5) == 0)
        {
            seq++;
        }

        RFQueue_nextEntry();

        //updateLCD();
    }
}

static RF_CmdHandle rxCmdHndl;
static rfc_propRxOutput_t rxStatistics_prop;

static void txTaskFunction(UArg arg0, UArg arg1)
{
    uint32_t curtime;
    RF_Params rfParams;
    RF_Params_init(&rfParams);

    if (RFQueue_defineQueue(&dataQueue, rxDataEntryBuffer, sizeof(rxDataEntryBuffer), NUM_DATA_ENTRIES, PAYLOAD_LENGTH + NUM_APPENDED_BYTES))
        while(1);

    //// RX settings

    // specify queue
    RF_cmdPropRx.pQueue = &dataQueue;
    // flush ignored and frames with errorneous CRC
    RF_cmdPropRx.rxConf.bAutoFlushIgnored = 1;
    RF_cmdPropRx.rxConf.bAutoFlushCrcErr = 1;
    // fixed payload length
    RF_cmdPropRx.maxPktLen = PAYLOAD_LENGTH;
    // RX command will repeat upon completion (either successfull or errorneous (CRC error, ..))
    RF_cmdPropRx.pktConf.bRepeatOk = 1;
    RF_cmdPropRx.pktConf.bRepeatNok = 1;
    // RX statistics (RSSI, timestamp, ..) will be put here
    RF_cmdPropRx.pOutput = (uint8_t*)&rxStatistics_prop;

    //// TX settings

    // fixed payload length
    RF_cmdPropTx.pktConf.bVarLen = 0;
    RF_cmdPropTx.pktLen = PAYLOAD_LENGTH;
    // output buffer
    RF_cmdPropTx.pPkt = packet;
    // all tramsissions are scheduled to specified point (absolute time)
    RF_cmdPropTx.startTrigger.triggerType = TRIG_ABSTIME;
    // run commands from past
    RF_cmdPropTx.startTrigger.pastTrig = 1;
    // startTime will be set before execution
    RF_cmdPropTx.startTime = 0;

    //
    msDelay = packet_interval / 4000;

    /* Request access to the radio */
    rfHandle = RF_open(&rfObject, &RF_prop, (RF_RadioSetup*)&RF_cmdPropRadioDivSetup, &rfParams);

    /* Set the frequency */
    RF_postCmd(rfHandle, (RF_Op*)&RF_cmdFs, RF_PriorityNormal, NULL, 0);

    LCD_Params lcdParams;

    LCD_Params_init(&lcdParams);
    lcdParams.spiBitRate = 1000000;
    lcdHandle = LCD_open(lcdBuffers, 2, &lcdParams);
    if (!lcdHandle)
        while(1);

    updateLCD();

    /* Get current time */
    curtime = RF_getCurrentTime();
    while(1)
    {
        // beacon
        if (mode == 0)
        {
            memcpy(packet, "HELLO", 5);

            while (mode == 0)
            {
                /* Set absolute TX time to utilize automatic power management */
                curtime += packet_interval;
                RF_cmdPropTx.startTime = curtime;

                /* Send packet */
                seq++;
                RF_EventMask result = RF_runCmd(rfHandle, (RF_Op*)&RF_cmdPropTx, RF_PriorityNormal, NULL, 0);
                if (!(result & RF_EventLastCmdDone))
                {
                    /* Error */
                    while(1);
                }

                updateLCD();

                PIN_setOutputValue(pinHandle, Board_PIN_LED1, !PIN_getOutputValue(Board_PIN_LED1));
            }

            seq = 0;
            curtime = RF_getCurrentTime();
        }
        // terrain
        else
        {
            rxStatistics_prop.lastRssi = 0;
            rxCmdHndl = RF_postCmd(rfHandle, (RF_Op*)&RF_cmdPropRx, RF_PriorityNormal, &callback, RF_EventRxEntryDone);

            // while we are in terrain mote mode
            while (mode == 1)
            {
                // TODO: synchronize using RF_pendCmd or Semaphores, for now, the CPU never go to sleep and just consume
                //       100% time in this loop, which is definitelly not power-saving
                //       when using pendCmd, CPU will wait until external signal is received with "IDLE" power consumption

                if (rxFlag == 1)
                {
                    PIN_setOutputValue(pinHandle, Board_PIN_LED1, !PIN_getOutputValue(Board_PIN_LED1));

                    // the value 12 is fixed, dunno what it means exactly, but it contains RF core startup ticks, delay before
                    // RX command is executed and preamble+sync word RX time, and maybe something else
                    msDelay = 12 + ((RF_getCurrentTime() - curtime) / 4000);
                    // store RSSI to be displayed
                    rssi = rxStatistics_prop.lastRssi;

                    // update LCD output
                    updateLCD();

                    // clear RX flag
                    rxFlag = 0;
                    // refresh time
                    curtime = RF_getCurrentTime();
                }
            }

            // when leaving Terrain mote mode, cancel posted command and wait for its execution
            RF_cancelCmd(rfHandle, rxCmdHndl, 0);
            RF_pendCmd(rfHandle, rxCmdHndl, RF_EventRxEntryDone);

            seq = 0;
            curtime = RF_getCurrentTime();
        }
    }
}

void buttonCallbackFunction(PIN_Handle handle, PIN_Id pinId)
{
    CPUdelay((uint32_t)((48000000/3)*0.050f));
    if (PIN_getInputValue(pinId))
        return;

    switch (pinId)
    {
        // UP key changes LCD buffer
        case Board_KEY_UP:
            bufIndex = (bufIndex + 1) % 2;
            updateLCD();
            break;
        // RIGHT key modifies packet delay
        case Board_KEY_RIGHT:
            packet_interval += PINTERVAL_STEP;
            if (packet_interval > PINTERVAL_MAX)
                packet_interval = PINTERVAL_MIN;

            msDelay = packet_interval / 4000;

            updateLCD();
            break;
        // SELECT key switches between modes (beacon - terrain mote)
        case Board_KEY_SELECT:
            mode = (mode + 1) % 2;
            break;
    }
}

/*
 *  ======== main ========
 */
int main(void)
{
    /* Call driver init functions. */
    Board_initGeneral();

    /* Open LED pins */
    pinHandle = PIN_open(&ledPinState, pinTable);
    if(!pinHandle)
    {
        System_abort("Error initializing board LED pins\n");
    }

    PIN_registerIntCb(pinHandle, &buttonCallbackFunction);

    /* Initialize task */
    TxTask_init();

    /* Start BIOS */
    BIOS_start();

    return (0);
}
