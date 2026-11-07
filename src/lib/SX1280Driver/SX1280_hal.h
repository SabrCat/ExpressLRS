#pragma once

/*
  ______                              _
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
    (C)2015 Semtech

Description: Handling of the node configuration protocol

License: Revised BSD License, see LICENSE.TXT file include in the project

Maintainer: Miguel Luis and Gregory Cristian

Heavily modified/simplified by Alessandro Carcione 2020 for ELRS project
*/

#include "SX1280_Regs.h"
#include "SX1280.h"
#include "SX1280_watchdog_policy.h"

enum SX1280_BusyState_
{
    SX1280_NOT_BUSY = true,
    SX1280_BUSY = false,
};

class SX1280Hal
{
public:
    static SX1280Hal *instance;

    SX1280Hal();

    void init();
    void end();
    void reset();
    // Receiver radio watchdog only (single-radio receivers): short reset pulse, then wait for the chip by its measured
    // start-up behaviour (BUSY low where wired; otherwise a bounded wait of ~2x the measured ready time, no SPI) and
    // require status STDBY_RC, instead of reset()'s ~110 ms of fixed waits. Returns false if the chip did not come up.
    // Radio 1 only. Constants and their provenance: SX1280_watchdog_policy.h.
    bool fastReset();
    // True if a command to radio 1 could be sent now without waiting: BUSY low, or (no BUSY pin) the previous
    // command's busy allowance has elapsed. Does not modify the allowance (unlike WaitOnBusy), so it is safe to call
    // with interrupts enabled while the radio ISR may issue commands.
    bool ReadyForCommand();

    void ICACHE_RAM_ATTR WriteCommand(SX1280_RadioCommands_t command, uint8_t val, SX12XX_Radio_Number_t radioNumber, uint32_t busyDelay = 15);
    void ICACHE_RAM_ATTR WriteCommand(SX1280_RadioCommands_t opcode, uint8_t *buffer, uint8_t size, SX12XX_Radio_Number_t radioNumber, uint32_t busyDelay = 15);
    // Like WriteCommand (same bytes on the wire), but full duplex: returns the status byte the chip sends during the
    // opcode. Kept as a separate copy on purpose: routing WriteCommand through a shared helper would add a call layer
    // to every stock write (hop, Rx) and shift their timing, which matters on boards without a BUSY pin.
    uint8_t ICACHE_RAM_ATTR WriteCommandStatus(SX1280_RadioCommands_t opcode, uint8_t *buffer, uint8_t size, SX12XX_Radio_Number_t radioNumber, uint32_t busyDelay = 15);
    void ICACHE_RAM_ATTR WriteRegister(uint16_t address, uint8_t *buffer, uint8_t size, SX12XX_Radio_Number_t radioNumber);
    void ICACHE_RAM_ATTR WriteRegister(uint16_t address, uint8_t value, SX12XX_Radio_Number_t radioNumber);

    SX1280_RadioStatus_t ICACHE_RAM_ATTR ReadCommand(SX1280_RadioCommands_t opcode, uint8_t *buffer, uint8_t size, SX12XX_Radio_Number_t radioNumber);
    void ICACHE_RAM_ATTR ReadRegister(uint16_t address, uint8_t *buffer, uint8_t size, SX12XX_Radio_Number_t radioNumber);
    uint8_t ICACHE_RAM_ATTR ReadRegister(uint16_t address, SX12XX_Radio_Number_t radioNumber);

    void ICACHE_RAM_ATTR WriteBuffer(uint8_t offset, uint8_t *buffer, uint8_t size, SX12XX_Radio_Number_t radioNumber); // Writes and Reads to FIFO
    void ICACHE_RAM_ATTR ReadBuffer(uint8_t offset, uint8_t *buffer, uint8_t size, SX12XX_Radio_Number_t radioNumber);

    bool ICACHE_RAM_ATTR WaitOnBusy(SX12XX_Radio_Number_t radioNumber);

    static ICACHE_RAM_ATTR void dioISR_1();
    static ICACHE_RAM_ATTR void dioISR_2();
    void (*IsrCallback_1)(); //function pointer for callback
    void (*IsrCallback_2)(); //function pointer for callback

    uint32_t BusyDelayStart;
    uint32_t BusyDelayDuration;
    void BusyDelay(uint32_t duration)
    {
        if (GPIO_PIN_BUSY == UNDEF_PIN)
        {
            BusyDelayStart = micros();
            BusyDelayDuration = duration;
        }
    }

private:

};
