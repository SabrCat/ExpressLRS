/*
  ______                              _
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
    (C)2016 Semtech

Description: Handling of the node configuration protocol

License: Revised BSD License, see LICENSE.TXT file include in the project

Maintainer: Miguel Luis, Gregory Cristian and Matthieu Verdy

Modified and adapted by Alessandro Carcione for ELRS project
*/

#ifndef UNIT_TEST
#include "SX1280_Regs.h"
#include "SX1280_hal.h"
#include <SPIEx.h>
#include "logging.h"

SX1280Hal *SX1280Hal::instance = NULL;

SX1280Hal::SX1280Hal()
{
    instance = this;
}

void SX1280Hal::end()
{
    detachInterrupt(GPIO_PIN_DIO1);
    if (GPIO_PIN_DIO1_2 != UNDEF_PIN)
    {
        detachInterrupt(GPIO_PIN_DIO1_2);
    }
    SPIEx.end();
    IsrCallback_1 = nullptr; // remove callbacks
    IsrCallback_2 = nullptr; // remove callbacks
}

void SX1280Hal::init()
{
    DBGLN("Hal Init");

    if (GPIO_PIN_BUSY != UNDEF_PIN)
    {
        pinMode(GPIO_PIN_BUSY, INPUT);
    }
    if (GPIO_PIN_BUSY_2 != UNDEF_PIN)
    {
        pinMode(GPIO_PIN_BUSY_2, INPUT);
    }

    pinMode(GPIO_PIN_DIO1, INPUT);
    if (GPIO_PIN_DIO1_2 != UNDEF_PIN)
    {
        pinMode(GPIO_PIN_DIO1_2, INPUT);
    }

    pinMode(GPIO_PIN_NSS, OUTPUT);
    digitalWrite(GPIO_PIN_NSS, HIGH);

#ifdef PLATFORM_ESP32
    SPIEx.begin(GPIO_PIN_SCK, GPIO_PIN_MISO, GPIO_PIN_MOSI, GPIO_PIN_NSS); // sck, miso, mosi, ss (ss can be any GPIO)
    gpio_pullup_en((gpio_num_t)GPIO_PIN_MISO);
    SPIEx.setFrequency(17500000);
    SPIEx.setHwCs(true);
    if (GPIO_PIN_NSS_2 != UNDEF_PIN)
    {
        pinMode(GPIO_PIN_NSS_2, OUTPUT);
        digitalWrite(GPIO_PIN_NSS_2, HIGH);
        spiAttachSS(SPIEx.bus(), 1, GPIO_PIN_NSS_2);
    }
    spiEnableSSPins(SPIEx.bus(), SX12XX_Radio_All);
#elif defined(PLATFORM_ESP8266)
    DBGLN("PLATFORM_ESP8266");
    SPIEx.begin();
    SPIEx.setHwCs(true);
    SPIEx.setBitOrder(MSBFIRST);
    SPIEx.setDataMode(SPI_MODE0);
    SPIEx.setFrequency(17500000);
#endif

    //attachInterrupt(digitalPinToInterrupt(GPIO_PIN_BUSY), this->busyISR, CHANGE); //not used atm
    attachInterrupt(digitalPinToInterrupt(GPIO_PIN_DIO1), this->dioISR_1, RISING);
    if (GPIO_PIN_DIO1_2 != UNDEF_PIN)
    {
        attachInterrupt(digitalPinToInterrupt(GPIO_PIN_DIO1_2), this->dioISR_2, RISING);
    }
}

void SX1280Hal::reset(void)
{
    DBGLN("SX1280 Reset");

    if (GPIO_PIN_RST != UNDEF_PIN)
    {
        pinMode(GPIO_PIN_RST, OUTPUT);
        digitalWrite(GPIO_PIN_RST, LOW);
        if (GPIO_PIN_RST_2 != UNDEF_PIN)
        {
            pinMode(GPIO_PIN_RST_2, OUTPUT);
            digitalWrite(GPIO_PIN_RST_2, LOW);
        }
        delay(50);
        digitalWrite(GPIO_PIN_RST, HIGH);
        if (GPIO_PIN_RST_2 != UNDEF_PIN)
        {
            digitalWrite(GPIO_PIN_RST_2, HIGH);
        }
        delay(50); // Safety buffer. Busy takes longer to go low than the 1ms timeout in WaitOnBusy().
    }

    BusyDelay(10000); // 10ms delay if GPIO_PIN_BUSY is undefined
    WaitOnBusy(SX12XX_Radio_All);

    //this->BusyState = SX1280_NOT_BUSY;
    DBGLN("SX1280 Ready!");
}

/**
 * Watchdog reset: short NRESET pulse, then wait for the restarted chip by MEASURED start-up behaviour, never by the
 * status byte alone. Bench (Nano, BUSY pin, 100 us and 1 ms pulses): the status reads STDBY_RC from ~375 us after the
 * release, but BUSY stays high until 1434-1455 us - the chip calibrates in STDBY_RC, so 'standby' is not 'ready'.
 * - BUSY pin: BUSY low is the ready signal; the first sample is taken FAST_RESET_BUSY_FIRST_SAMPLE_US after the release
 *   (the chip may not drive BUSY yet in the first moments), bounded by FAST_RESET_BUSY_TIMEOUT_US.
 * - No BUSY pin: no SPI at all for FAST_RESET_NO_BUSY_WAIT_US (~2x the measured ready time; the chip firmware is the
 *   same A9B7 on both bench boards), then the status must read STDBY_RC. The caller verifies the configuration by
 *   read-back afterwards, which catches a chip that was not ready after all.
 * Interrupts stay enabled throughout (loop context). Returns false if the chip did not come up.
 */
bool SX1280Hal::fastReset()
{
    using namespace SX1280WatchdogPolicy;
    if (GPIO_PIN_RST == UNDEF_PIN)
    {
        // No reset line: nothing restarts and there is nothing to wait for. The watchdog keeps the chip awake (no End())
        // and Begin re-runs its configuration on the running chip; Begin's SetStandby + firmware-version read is the
        // liveness check and the caller's read-back verifies the result.
        return true;
    }
    pinMode(GPIO_PIN_RST, OUTPUT);
    digitalWrite(GPIO_PIN_RST, LOW);
    delayMicroseconds(FAST_RESET_PULSE_US);
    digitalWrite(GPIO_PIN_RST, HIGH);
    BusyDelayDuration = 0; // allowances of commands sent before the reset do not apply to the restarted chip

    const uint32_t release = micros();
    if (GPIO_PIN_BUSY != UNDEF_PIN)
    {
        delayMicroseconds(FAST_RESET_BUSY_FIRST_SAMPLE_US);
        while (digitalRead(GPIO_PIN_BUSY) == HIGH)
        {
            if (micros() - release >= FAST_RESET_BUSY_TIMEOUT_US)
            {
                DBGLN("SX1280 fast reset: BUSY still high");
                return false;
            }
        }
    }
    else
    {
        delayMicroseconds(FAST_RESET_NO_BUSY_WAIT_US);
    }
    uint8_t unused = 0; // no data bytes; a valid pointer for ReadCommand's memcpy of size 0
    const uint8_t status = (uint8_t)ReadCommand(SX1280_RADIO_GET_STATUS, &unused, 0, SX12XX_Radio_1);
    const bool standby = (status & SX1280Driver::STATUS_MODE_MASK) == SX1280_STATUS_CIRCUIT_MODE_STDBY_RC;
    DBGLN("SX1280 fast reset: %u us, status %x", micros() - release, status);
    return standby;
}

bool SX1280Hal::ReadyForCommand()
{
    if (GPIO_PIN_BUSY != UNDEF_PIN)
    {
        return digitalRead(GPIO_PIN_BUSY) == LOW;
    }
    return (micros() - BusyDelayStart) >= BusyDelayDuration;
}

void ICACHE_RAM_ATTR SX1280Hal::WriteCommand(SX1280_RadioCommands_t command, uint8_t val, SX12XX_Radio_Number_t radioNumber, uint32_t busyDelay)
{
    WriteCommand(command, &val, 1, radioNumber, busyDelay);
}

void ICACHE_RAM_ATTR SX1280Hal::WriteCommand(SX1280_RadioCommands_t command, uint8_t *buffer, uint8_t size, SX12XX_Radio_Number_t radioNumber, uint32_t busyDelay)
{
    WORD_ALIGNED_ATTR uint8_t OutBuffer[WORD_PADDED(size + 1)] = {
        command,
    };

    memcpy(OutBuffer + 1, buffer, size);

    WaitOnBusy(radioNumber);
    SPIEx.write(radioNumber, OutBuffer, size + 1);

    BusyDelay(busyDelay);
}

uint8_t ICACHE_RAM_ATTR SX1280Hal::WriteCommandStatus(SX1280_RadioCommands_t command, uint8_t *buffer, uint8_t size, SX12XX_Radio_Number_t radioNumber, uint32_t busyDelay)
{
    WORD_ALIGNED_ATTR uint8_t OutBuffer[WORD_PADDED(size + 1)] = {
        command,
    };

    memcpy(OutBuffer + 1, buffer, size);

    WaitOnBusy(radioNumber);
    SPIEx.read(radioNumber, OutBuffer, size + 1); // full duplex: the bus sees the same bytes as write()

    BusyDelay(busyDelay);
    return OutBuffer[0];
}

SX1280_RadioStatus_t ICACHE_RAM_ATTR SX1280Hal::ReadCommand(SX1280_RadioCommands_t command, uint8_t *buffer, uint8_t size, SX12XX_Radio_Number_t radioNumber)
{
    WORD_ALIGNED_ATTR uint8_t OutBuffer[WORD_PADDED(size + 2)] = {
        (uint8_t)command,
        0x00,
        0x00,
    };

    WaitOnBusy(radioNumber);

    SPIEx.read(radioNumber, OutBuffer, size + 2); // first 2 bytes returned are status!
    memcpy(buffer, OutBuffer + 2, size);
    return (SX1280_RadioStatus_t)(OutBuffer[0] & SX1280_STATUS_MASK); // Discard reserved bits
}

void ICACHE_RAM_ATTR SX1280Hal::WriteRegister(uint16_t address, uint8_t *buffer, uint8_t size, SX12XX_Radio_Number_t radioNumber)
{
    WORD_ALIGNED_ATTR uint8_t OutBuffer[WORD_PADDED(size + 3)] = {
        SX1280_RADIO_WRITE_REGISTER,
        (uint8_t)((address & 0xFF00) >> 8),
        (uint8_t)(address & 0x00FF),
    };

    memcpy(OutBuffer + 3, buffer, size);

    WaitOnBusy(radioNumber);
    SPIEx.write(radioNumber, OutBuffer, size + 3);

    BusyDelay(15);
}

void ICACHE_RAM_ATTR SX1280Hal::WriteRegister(uint16_t address, uint8_t value, SX12XX_Radio_Number_t radioNumber)
{
    WriteRegister(address, &value, 1, radioNumber);
}

void ICACHE_RAM_ATTR SX1280Hal::ReadRegister(uint16_t address, uint8_t *buffer, uint8_t size, SX12XX_Radio_Number_t radioNumber)
{
    WORD_ALIGNED_ATTR uint8_t OutBuffer[WORD_PADDED(size + 4)] = {
        SX1280_RADIO_READ_REGISTER,
        (uint8_t)((address & 0xFF00) >> 8),
        (uint8_t)(address & 0x00FF),
        0x00,
    };

    WaitOnBusy(radioNumber);

    SPIEx.read(radioNumber, OutBuffer, size + 4);
    memcpy(buffer, OutBuffer + 4, size);
}

uint8_t ICACHE_RAM_ATTR SX1280Hal::ReadRegister(uint16_t address, SX12XX_Radio_Number_t radioNumber)
{
    uint8_t data;
    ReadRegister(address, &data, 1, radioNumber);
    return data;
}

void ICACHE_RAM_ATTR SX1280Hal::WriteBuffer(uint8_t offset, uint8_t *buffer, uint8_t size, SX12XX_Radio_Number_t radioNumber)
{
    WORD_ALIGNED_ATTR uint8_t OutBuffer[WORD_PADDED(size + 2)] = {
        SX1280_RADIO_WRITE_BUFFER,
        offset
    };

    memcpy(OutBuffer + 2, buffer, size);

    WaitOnBusy(radioNumber);

    SPIEx.write(radioNumber, OutBuffer, size + 2);

    BusyDelay(15);
}

void ICACHE_RAM_ATTR SX1280Hal::ReadBuffer(uint8_t offset, uint8_t *buffer, uint8_t size, SX12XX_Radio_Number_t radioNumber)
{
    WORD_ALIGNED_ATTR uint8_t OutBuffer[WORD_PADDED(size + 3)] = {
        SX1280_RADIO_READ_BUFFER,
        offset,
        0x00
    };

    WaitOnBusy(radioNumber);

    SPIEx.read(radioNumber, OutBuffer, size + 3);

    memcpy(buffer, OutBuffer + 3, size);
}

bool ICACHE_RAM_ATTR SX1280Hal::WaitOnBusy(SX12XX_Radio_Number_t radioNumber)
{
    if (GPIO_PIN_BUSY != UNDEF_PIN)
    {
        constexpr uint32_t wtimeoutUS = 1000U;
        uint32_t startTime = 0;

        while (true)
        {
            if (radioNumber == SX12XX_Radio_1)
            {
                if (digitalRead(GPIO_PIN_BUSY) == LOW) return true;
            }
            else if (radioNumber == SX12XX_Radio_2)
            {
                if (GPIO_PIN_BUSY_2 == UNDEF_PIN || digitalRead(GPIO_PIN_BUSY_2) == LOW) return true;
            }
            else if (radioNumber == SX12XX_Radio_All)
            {
                if (GPIO_PIN_BUSY_2 != UNDEF_PIN)
                {
                    if (digitalRead(GPIO_PIN_BUSY) == LOW && digitalRead(GPIO_PIN_BUSY_2) == LOW) return true;
                }
                else
                {
                    if (digitalRead(GPIO_PIN_BUSY) == LOW) return true;
                }
            }
            // Use this time to call micros().
            uint32_t now = micros();
            if (startTime == 0) startTime = now;
            if ((now - startTime) > wtimeoutUS) return false;
        }
    }
    else
    {
        uint32_t now = micros();
        while ((now - BusyDelayStart) < BusyDelayDuration)
            now = micros();
        BusyDelayDuration = 0;
    }
    return true;
}

void ICACHE_RAM_ATTR SX1280Hal::dioISR_1()
{
    if (instance->IsrCallback_1)
        instance->IsrCallback_1();
}

void ICACHE_RAM_ATTR SX1280Hal::dioISR_2()
{
    if (instance->IsrCallback_2)
        instance->IsrCallback_2();
}

#endif // UNIT_TEST
