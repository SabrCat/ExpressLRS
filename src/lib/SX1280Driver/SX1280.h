#pragma once

#include "targets.h"
#include "SX1280_Regs.h"
#include "SX1280_hal.h"
#include "SX1280_watchdog_policy.h"
#include "SX12xxDriverCommon.h"

#ifdef PLATFORM_ESP8266
#include <cstdint>
#endif

#define RADIO_SNR_SCALE 4 // Units for LastPacketSNRRaw

class SX1280Driver: public SX12xxDriverCommon
{
public:
    static SX1280Driver *instance;

    ////////////////Configuration Functions/////////////
    SX1280Driver();
    // fastReset (receiver radio watchdog): verified-ready reset instead of the stock fixed waits, and the radio ISR
    // callbacks stay detached until the caller enables them (EnableIsrCallbacks) once the radio is configured.
    bool Begin(uint32_t minimumFrequency, uint32_t maximumFrequency, bool fastReset = false);
    void End();
    void EnableIsrCallbacks(bool enable);
    bool ReadyForCommand();  // see SX1280Hal::ReadyForCommand
    // Receiver radio watchdog: reads back everything Begin + Config wrote that the chip lets us read (0x0891 sensitivity
    // bits, packet type, FLRC CRC seed + sync word, or the LoRa SF register, payload length and header mode) and compares
    // (SX1280WatchdogPolicy::configMatches; list of what cannot be read there). Radio 1 only.
    bool VerifyConfig();
    void SetTxIdleMode() { SetMode(SX1280_MODE_FS, SX12XX_Radio_All); }; // set Idle mode used when switching from RX to TX
    void SetStandbyRc() { SetMode(SX1280_MODE_STDBY_RC, SX12XX_Radio_All); };
    uint8_t GetStatusByte(SX12XX_Radio_Number_t radioNumber);
    uint8_t GetRssiInstRaw(SX12XX_Radio_Number_t radioNumber);  // full resolution (GetRssiInst halves it)
    static constexpr uint8_t STATUS_MODE_MASK = 0xE0;     // status bits 7:5 = circuit mode
    static constexpr uint8_t STATUS_COMMAND_MASK = 0x1C;  // status bits 4:2 = command status
    void Config(uint8_t bw, uint8_t sf, uint8_t cr, uint32_t freq,
                uint8_t PreambleLength, bool InvertIQ, uint8_t PayloadLength,
                uint32_t flrcSyncWord, uint16_t flrcCrcSeed, RadioBandMod::Combined modulation);
    void SetFrequencyReg(uint32_t freq, SX12XX_Radio_Number_t radioNumber, bool doRx = false);
    void SetOutputPower(int8_t power);
    void startCWTest(uint32_t freq, SX12XX_Radio_Number_t radioNumber);


    bool GetFrequencyErrorbool(SX12XX_Radio_Number_t radioNumber);

#if defined(TARGET_RX)
    // Radio watchdog inputs (rx_main.cpp radioWatchdog*), taken from status bytes the receive path already reads.
    // clearSuspect: the RX_DONE handler's ClearIrqStatus returned "Rx, command status 0" - on the bench this preceded
    // an ignored clear (IRQ stays set, DIO1 stays high) in 466 of 523 cases, and appeared in 13 of ~2100 good clears.
    volatile bool clearSuspect = false;
    volatile uint32_t clearSuspectMicros = 0;
    // Consecutive receive-path status reads with circuit mode 0 (undocumented; bench: chip still receives, but the 4.x
    // gate rejects every packet).
    volatile uint8_t modeZeroReads = 0;
    // RX_DONE interrupts handled (any packet, valid or not; wraps): proof of a receiving path while disconnected.
    volatile uint8_t rxDoneCount = 0;
    // Set by rx_main on single-radio receivers only (with two radios both chips drive MISO during a shared opcode).
    volatile bool watchdogEnabled = false;
#endif
    bool FrequencyErrorAvailable() const { return modeSupportsFei && (LastPacketSNRRaw > 0); }

    void TXnb(uint8_t * data, bool sendGeminiBuffer, uint8_t * dataGemini, SX12XX_Radio_Number_t radioNumber);
    void RXnb();

    uint16_t GetIrqStatus(SX12XX_Radio_Number_t radioNumber);
    void ClearIrqStatus(uint16_t irqMask, SX12XX_Radio_Number_t radioNumber);

    bool GetRxBufferAddr(SX12XX_Radio_Number_t radioNumber, uint8_t *rxBufferAddr);
    int8_t GetRssiInst(SX12XX_Radio_Number_t radioNumber);
    void GetLastPacketStats();
    void CheckForSecondPacket();

private:
    // constant used for no power change pending
    // must not be a valid power register value
    static constexpr uint8_t PWRPENDING_NONE = 0x7f;
    static constexpr uint8_t SX1280_TX_BUFFER_BASE = 0x00;
    static constexpr uint8_t SX1280_RX_BUFFER_BASE = 0x80;

    SX1280_RadioOperatingModes_t currOpmode;
    uint8_t packet_mode;
    SX1280WatchdogPolicy::ConfigImage writtenConfig;  // what Config wrote, for VerifyConfig
    bool modeSupportsFei;
    uint8_t pwrCurrent;
    uint8_t pwrPending;
    SX1280_RadioOperatingModes_t fallBackMode;

    void SetMode(SX1280_RadioOperatingModes_t OPmode, SX12XX_Radio_Number_t radioNumber);
    void SetFIFOaddr(uint8_t txBaseAddr, uint8_t rxBaseAddr);

    // LoRa functions
    void ConfigModParamsLoRa(uint8_t bw, uint8_t sf, uint8_t cr);
    void SetPacketParamsLoRa(uint8_t PreambleLength, SX1280_RadioLoRaPacketLengthsModes_t HeaderType,
                             uint8_t InvertIQ);
    // FLRC functions
    void ConfigModParamsFLRC(uint8_t bw, uint8_t cr, uint8_t bt=SX1280_FLRC_BT_0_5);
    void SetPacketParamsFLRC(uint8_t HeaderType,
                             uint8_t PreambleLength,
                             uint32_t syncWord,
                             uint16_t crcSeed,
                             uint8_t cr);

    void SetDioIrqParams(uint16_t irqMask,
                         uint16_t dio1Mask=SX1280_IRQ_RADIO_NONE,
                         uint16_t dio2Mask=SX1280_IRQ_RADIO_NONE,
                         uint16_t dio3Mask=SX1280_IRQ_RADIO_NONE);

    static void IsrCallback_1();
    static void IsrCallback_2();
    static void IsrCallback(SX12XX_Radio_Number_t radioNumber);
    bool RXnbISR(uint16_t irqStatus, SX12XX_Radio_Number_t radioNumber); // ISR for non-blocking RX routine
    void TXnbISR(); // ISR for non-blocking TX routine
    void CommitOutputPower();
};
