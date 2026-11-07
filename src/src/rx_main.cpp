#include "CRSFRouter.h"
#include "LowPassFilter.h"
#include "rxtx_common.h"

#include "crc.h"
#include "stubborn_sender.h"
#include "stubborn_receiver.h"

#include "CRSFParameters.h"
#include "MeanAccumulator.h"
#include "PFD.h"
#include "dynpower.h"
#include "freqTable.h"
#include "msp.h"
#include "msptypes.h"
#include "options.h"

#include "rx-serial/SerialIO.h"
#include "rx-serial/SerialNOOP.h"
#include "rx-serial/SerialCRSF.h"
#include "rx-serial/SerialSBUS.h"
#include "rx-serial/SerialSUMD.h"
#include "rx-serial/SerialAirPort.h"
#include "rx-serial/SerialHoTT_TLM.h"
#include "rx-serial/SerialMavlink.h"
#include "rx-serial/SerialTramp.h"
#include "rx-serial/SerialSmartAudio.h"
#include "rx-serial/SerialDisplayport.h"
#include "rx-serial/SerialGPS.h"

#include "devAnalogVbat.h"
#include "devBaro.h"
#include "devButton.h"
#include "devLED.h"
#include "devRXLUA.h"
#include "devServoOutput.h"
#include "devWIFI.h"
#include "RXEndpoint.h"
#include "RXOTAConnector.h"
#include "rx-serial/devSerialIO.h"

#include <LittleFS.h>
#if defined(PLATFORM_ESP8266)
#include <user_interface.h>
#elif defined(PLATFORM_ESP32)
#include "devSerialUpdate.h"
#include "devVTXSPI.h"
#include "devMSPVTX.h"
#include "devThermal.h"
#include "esp_task_wdt.h"
#endif

//
// Code encapsulated by the ARDUINO_CORE_INVERT_FIX #ifdef temporarily fixes EpressLRS issue #2609 which is caused
// by the Arduino core (see https://github.com/espressif/arduino-esp32/issues/9896) and fixed
// by Espressif with Arduino core release 3.0.3 (see https://github.com/espressif/arduino-esp32/pull/9950)
//
// With availability of Arduino core 3.0.3 and upgrading ExpressLRS to Arduino core 3.0.3 the temporary fix
// should be deleted again
//
// ARDUINO_CORE_INVERT_FIX PT1
#define ARDUINO_CORE_INVERT_FIX

#if defined(PLATFORM_ESP32) && defined(ARDUINO_CORE_INVERT_FIX)
#include "driver/uart.h"
#endif
// ARDUINO_CORE_INVERT_FIX PT1 end

//// CONSTANTS ////
#define SEND_LINK_STATS_TO_FC_INTERVAL 100
#define DIVERSITY_ANTENNA_INTERVAL 5
#define DIVERSITY_ANTENNA_RSSI_TRIGGER 5
#define PACKET_TO_TOCK_SLACK 200 // Desired buffer time between Packet ISR and Tock ISR
///////////////////

device_affinity_t ui_devices[] = {
  {&Serial0_device, 1},
#if defined(PLATFORM_ESP32)
  {&Serial1_device, 1},
  {&SerialUpdate_device, 1},
#endif
  {&LED_device, 0},
  {&RXLUA_device, 0},
  {&RGB_device, 0},
  {&WIFI_device, 0},
  {&Button_device, 0},
  {&AnalogVbat_device, 0},
  {&ServoOut_device, 1},
  {&Baro_device, 0}, // must come after AnalogVbat_device to slow updates
#if defined(PLATFORM_ESP32) && !defined(PLATFORM_ESP32_C3)
  {&VTxSPI_device, 0},
  {&MSPVTx_device, 0}, // dependency on VTxSPI_device
  {&Thermal_device, 0},
#endif
};

uint8_t antenna = 0;    // which antenna is currently in use
uint8_t geminiMode = 0;

PFD PFDloop;
Crc2Byte ota_crc;
ELRS_EEPROM eeprom;
RxConfig config;

CRSFRouter crsfRouter;
RXEndpoint crsfReceiver;
RXOTAConnector otaConnector;

bool crsfBatterySensorDetected = false;
bool crsfBaroSensorDetected = false;

extern bool webserverPreventAutoStart;
bool pwmSerialDefined = false;
uint32_t serialBaud;

/* SERIAL_PROTOCOL_TX is used by CRSF output */
#define SERIAL_PROTOCOL_TX Serial

#if defined(PLATFORM_ESP32)
    #define SERIAL1_PROTOCOL_TX Serial1

    // SBUS driver needs to distinguish stream for SBUS/DJI protocol
    const Stream *serial_protocol_tx = &(SERIAL_PROTOCOL_TX);
    const Stream *serial1_protocol_tx = &(SERIAL1_PROTOCOL_TX);

    SerialIO *serial1IO = nullptr;
#endif

SerialIO *serialIO = nullptr;

#define SERIAL_PROTOCOL_RX Serial
#define SERIAL1_PROTOCOL_RX Serial1

StubbornSender DataDlSender;
uint8_t DataDlBuffer[CRSF_MAX_PACKET_LEN];
static uint8_t telemetryBurstCount;
static uint8_t telemetryBurstMax;

StubbornReceiver DataUlReceiver;
uint8_t DataUlBuffer[ELRS_DATA_UL_BUFFER];

static uint8_t NextTelemetryType = PACKET_TYPE_LINKSTATS;
static bool telemBurstValid;
/// PFD Filters ////////////////
LPF LPF_Offset(2);
LPF LPF_OffsetDx(4);

/// LQ/RSSI/SNR Calculation //////////
LQCALC<100> LQCalc;
LQCALC<100> LQCalcDVDA;
uint8_t uplinkLQ;
LPF LPF_UplinkRSSI0(5);  // track rssi per antenna
LPF LPF_UplinkRSSI1(5);
MeanAccumulator<int32_t, int8_t, -16> SnrMean;

static uint8_t scanIndex;
uint8_t ExpressLRS_nextAirRateIndex;
int8_t SwitchModePending;

int32_t PfdPrevRawOffset;
RXtimerState_e RXtimerState;
uint32_t GotConnectionMillis = 0;
const uint32_t ConsiderConnGoodMillis = 1000; // minimum time before we can consider a connection to be 'good'
bool doStartTimer = false;

///////////////////////////////////////////////

static bool alreadyTLMresp = false;

//////////////////////////////////////////////////////////////

///////Variables for Telemetry and Link Quality///////////////
uint32_t LastValidPacket = 0;           //Time the last valid packet was recv
uint32_t LastSyncPacket = 0;            //Time the last valid packet was recv

static uint32_t SendLinkStatstoFCintervalLastSent;
static uint8_t SendLinkStatstoFCForcedSends;

int16_t RFnoiseFloor; //measurement of the current RF noise floor
#if defined(DEBUG_RX_SCOREBOARD)
static bool lastPacketCrcError;
#endif
///////////////////////////////////////////////////////////////

/// Variables for Sync Behaviour ////
uint32_t cycleInterval; // in ms
uint32_t RFmodeLastCycled = 0;
#define RFmodeCycleMultiplierSlow 10
uint8_t RFmodeCycleMultiplier;
bool LockRFmode = false;
///////////////////////////////////////

#if defined(DEBUG_BF_LINK_STATS)
// Debug vars
uint8_t debug1 = 0;
uint8_t debug2 = 0;
uint8_t debug3 = 0;
int8_t debug4 = 0;
///////////////////////////////////////
#endif

#if defined(DEBUG_RCVR_LINKSTATS)
static bool debugRcvrLinkstatsPending;
static uint8_t debugRcvrLinkstatsFhssIdx;
#endif

bool BindingModeRequest = false;

extern void setWifiUpdateMode();
void reconfigureSerial();

uint8_t getLq()
{
    return LQCalc.getLQ();
}

static inline void checkGeminiMode()
{
    if (isDualRadio())
    {
        geminiMode = config.GetAntennaMode() || FHSSuseDualBand; // Force Gemini when in DualBand mode.  Whats the point of using a single frequency!
    }
}

static uint8_t minLqForChaos()
{
    // Determine the most number of CRC-passing packets we could receive on
    // a single channel out of 100 packets that fill the LQcalc span.
    // The LQ must be GREATER THAN this value, not >=
    // The amount of time we coexist on the same channel is
    // 100 divided by the total number of packets in a FHSS loop (rounded up)
    // and there would be 4x packets received each time it passes by so
    // FHSShopInterval * ceil(100 / FHSShopInterval * numfhss) or
    // FHSShopInterval * trunc((100 + (FHSShopInterval * numfhss) - 1) / (FHSShopInterval * numfhss))
    // With a interval of 4 this works out to: 2.4=4, FCC915=4, AU915=8, EU868=8, EU/AU433=36
    const uint32_t numfhss = FHSSgetChannelCount();
    const uint8_t interval = ExpressLRS_currAirRate_Modparams->FHSShopInterval;
    return interval * ((interval * numfhss + 99) / (interval * numfhss));
}

void ICACHE_RAM_ATTR getRFlinkInfo()
{
    int32_t rssiDBM = Radio.LastPacketRSSI;

    if (GPIO_PIN_NSS_2 != UNDEF_PIN)
    {
        int32_t rssiDBM2 = Radio.LastPacketRSSI2;

        #if !defined(DEBUG_RCVR_LINKSTATS)
        rssiDBM = LPF_UplinkRSSI0.update(rssiDBM);
        rssiDBM2 = LPF_UplinkRSSI1.update(rssiDBM2);
        #endif
        rssiDBM = (rssiDBM > 0) ? 0 : rssiDBM;
        rssiDBM2 = (rssiDBM2 > 0) ? 0 : rssiDBM2;

        // BetaFlight/iNav expect positive values for -dBm (e.g. -80dBm -> sent as 80)
        linkStats.uplink_RSSI_1 = -rssiDBM;
        linkStats.uplink_RSSI_2 = -rssiDBM2;
        antenna = (Radio.GetProcessingPacketRadio() == SX12XX_Radio_1) ? 0 : 1;
    }
    else if (antenna == 0)
    {
        #if !defined(DEBUG_RCVR_LINKSTATS)
        rssiDBM = LPF_UplinkRSSI0.update(rssiDBM);
        #endif
        if (rssiDBM > 0) rssiDBM = 0;
        // BetaFlight/iNav expect positive values for -dBm (e.g. -80dBm -> sent as 80)
        linkStats.uplink_RSSI_1 = -rssiDBM;
    }
    else
    {
        #if !defined(DEBUG_RCVR_LINKSTATS)
        rssiDBM = LPF_UplinkRSSI1.update(rssiDBM);
        #endif
        if (rssiDBM > 0) rssiDBM = 0;
        // BetaFlight/iNav expect positive values for -dBm (e.g. -80dBm -> sent as 80)
        // May be overwritten below if DEBUG_BF_LINK_STATS is set
        linkStats.uplink_RSSI_2 = -rssiDBM;
    }

    SnrMean.add(Radio.LastPacketSNRRaw);

    linkStats.active_antenna = antenna;
    linkStats.uplink_SNR = SNR_DESCALE(Radio.LastPacketSNRRaw); // possibly overriden below
    //linkStats.uplink_Link_quality = uplinkLQ; // handled in Tick
    linkStats.rf_Mode = ExpressLRS_currAirRate_Modparams->enum_rate;
    //DBGLN(linkStats.uplink_RSSI_1);
    #if defined(DEBUG_BF_LINK_STATS)
    linkStats.downlink_RSSI_1 = debug1;
    linkStats.downlink_Link_quality = debug2;
    linkStats.downlink_SNR = debug3;
    linkStats.uplink_RSSI_2 = debug4;
    #endif

    #if defined(DEBUG_RCVR_LINKSTATS)
    // DEBUG_RCVR_LINKSTATS gets full precision SNR, override the value
    linkStats.uplink_SNR = Radio.LastPacketSNRRaw;
    debugRcvrLinkstatsFhssIdx = FHSSusePrimaryFreqBand ? FHSSsequence[FHSSptr] : FHSSsequence_DualBand[FHSSptr];
    #endif
}

// Radio modulation/packet setup for a rate, on the initial frequency. Used by SetRFLinkRate and by the radio
// watchdog's reset (which must not touch the timer or apply a pending rate change).
static void ConfigRadio(expresslrs_mod_settings_s *const ModParams, bool invertIQ)
{
    Radio.Config(ModParams->bw, ModParams->sf, ModParams->cr, FHSSgetInitialFreq(),
                 ModParams->PreambleLen, invertIQ, ModParams->PayloadLength
#if defined(RADIO_SX128X)
                 , OtaGetUidSeed(), OtaCrcInitializer, ModParams->radio_type
#endif
#if defined(RADIO_LR1121)
                 , ModParams->radio_type, (uint8_t)UID[5], (uint8_t)UID[4]
#endif
                 );

#if defined(RADIO_LR1121)
    if (FHSSuseDualBand)
    {
        Radio.Config(ModParams->bw2, ModParams->sf2, ModParams->cr2, FHSSgetInitialGeminiFreq(),
                    ModParams->PreambleLen2, invertIQ, ModParams->PayloadLength,
                    ModParams->radio_type, (uint8_t)UID[5], (uint8_t)UID[4], SX12XX_Radio_2);
    }
#endif
}

void SetRFLinkRate(uint8_t index, bool bindMode) // Set speed of RF link
{
    expresslrs_mod_settings_s *const ModParams = get_elrs_airRateConfig(index);
    expresslrs_rf_pref_params_s *const RFperf = get_elrs_RFperfParams(index);

    // Binding always uses invertIQ
    bool invertIQ = bindMode || (UID[5] & 0x01);

    uint32_t interval = ModParams->interval;
#if defined(DEBUG_FREQ_CORRECTION) && defined(RADIO_SX128X)
    interval = interval * 12 / 10; // increase the packet interval by 20% to allow adding packet header
#endif

    hwTimer::updateInterval(interval);

#if defined(RADIO_LR1121)
    FHSSusePrimaryFreqBand = !RadioBandMod::isB2G4(ModParams->radio_type);
    FHSSuseDualBand = RadioBandMod::isBDUAL(ModParams->radio_type);
#endif

    ConfigRadio(ModParams, invertIQ);

    Radio.FuzzySNRThreshold = (RFperf->DynpowerSnrThreshUp == DYNPOWER_SNR_THRESH_NONE) ? 0 : (RFperf->DynpowerSnrThreshDn - RFperf->DynpowerSnrThreshUp);

    checkGeminiMode();
    if (geminiMode)
    {
        Radio.SetFrequencyReg(FHSSgetInitialGeminiFreq(), SX12XX_Radio_2, false);
    }

    OtaUpdateSerializers(smWideOr8ch, ModParams->PayloadLength);
    DataUlReceiver.setMaxPackageIndex(ELRS_MSP_MAX_PACKAGES);
    DataDlSender.setMaxPackageIndex(OtaIsFullRes ? ELRS8_DATA_DL_MAX_PACKAGES : ELRS4_DATA_DL_MAX_PACKAGES);

    // Wait for (11/10) 110% of time it takes to cycle through all freqs in FHSS table (in ms)
    cycleInterval = ((uint32_t)11U * FHSSgetChannelCount() * ModParams->FHSShopInterval * interval) / (10U * 1000U);

    ExpressLRS_currAirRate_Modparams = ModParams;
    ExpressLRS_currAirRate_RFperfParams = RFperf;
    ExpressLRS_nextAirRateIndex = index; // presumably we just handled this
    telemBurstValid = false;

    LbtEnableIfRequired();
}

#if defined(RADIO_SX128X)
// ---------------------------------------------------------------------------------------------------------------
// Radio watchdog (issue #3835). Bench observations on an SX1280 receiver without a BUSY pin: the chip sometimes
// ignores the RX_DONE handler's ClearIrqStatus (IRQ stays set, DIO1 stays high; the rising-edge interrupt never fires
// again), or enters an undocumented circuit mode 0 that the 4.x status gate rejects. Stock firmware stays deaf until
// power cycle in both cases. The watchdog only uses what the receive path already reads (the status byte returned by
// the handler's clear, the gate's status byte) plus the DIO1 level at the tock, and repairs with verified steps:
// a repeat clear >= 1 ms later (earlier repeats were ignored by the chip), standby + Rx for mode 0, and as the last
// step a radio reset that keeps the hw timer and the FHSS position, so no resync is needed. Loop-side radio commands
// mask interrupts only around each single command (radioWatchdogCommand). The reset never uses the stock reset's
// ~110 ms of fixed waits: a short pulse, a wait set by the measured start-up time (BUSY low where wired), and a
// read-back of the configuration; failed attempts are retried in three tiers (3 within half the failsafe budget of the
// fastest rate, up to 8 within the 500/333 Hz budgets, then once per second for the rest of the flight; timings:
// SX1280_watchdog_policy.h). A dead receive path (connected: no packets AND a pinned RSSI; disconnected: RSSI pinned
// for ~0.5 s AND no RX_DONE) gets the Rx repair, then resets. A verified repair or reset must be followed by a valid
// packet within a few slots while connected, else the loop escalates (not every configuration item can be read back).
// ---------------------------------------------------------------------------------------------------------------
static volatile bool radioWatchdogResetting = false;  // loop owns the radio: no hop write / telemetry from the tock
static volatile bool radioWatchdogResetRequested = false;
// The radio was not ready for the hand-back after an attempt (BUSY high / allowance running): the tock stays locked out
// (radioWatchdogResetting) and the loop retries the hand-back every pass until the radio takes commands again.
static bool radioWatchdogHandbackPending = false;
// After a failed attempt, until a valid packet: the tock skips its radio commands while BUSY reads high, so a radio that
// sticks busy again cannot make each stock command wait 1 ms inside the timer ISR (ESP8266 timer stall).
static volatile bool radioWatchdogUnverified = false;
static volatile bool radioWatchdogTockSkip = false;  // this tock: no hop write / telemetry (radio busy, see above)
static volatile uint8_t radioWatchdogStage = 0;        // 1 / 2 = repeat clear sent, verify DIO1 at the next tock
static volatile uint8_t radioWatchdogDio1HighTocks = 0;
static int8_t radioWatchdogDio1Pin = UNDEF_PIN;
static constexpr uint32_t RADIO_WATCHDOG_MIN_RETRY_US = 1000;       // repeat clears < ~300 us were ignored (bench)
static constexpr uint8_t RADIO_WATCHDOG_STUCK_TOCKS = 2;            // DIO1 high at this many tocks in a row = stuck
static constexpr uint32_t RADIO_WATCHDOG_STUCK_DISCONNECTED_MS = 10;
static constexpr uint8_t RADIO_WATCHDOG_MODE_ZERO_READS = 2;
static constexpr uint32_t RADIO_WATCHDOG_RX_KICK_MS = 1000;         // disconnected: re-enter Rx this often
static constexpr uint8_t RADIO_WATCHDOG_RSSI_SAMPLES = SX1280WatchdogPolicy::RSSI_SAMPLES;
static constexpr uint32_t RADIO_WATCHDOG_DIO1_SAMPLE_GAP_MS = 2;  // disconnected stuck-DIO1 check: longer loop gaps restart it
// The radio frequency no longer matches the FHSS position: a reset configured the initial frequency (release retunes).
static bool radioWatchdogFreqStale = false;
static uint8_t radioWatchdogTakenFhssPtr = 0;  // FHSSptr when the loop took the radio (hops skipped since = it moved)

#if defined(PLATFORM_ESP32)
static portMUX_TYPE radioWatchdogMux = portMUX_INITIALIZER_UNLOCKED;
#define RADIO_WATCHDOG_LOCK() portENTER_CRITICAL(&radioWatchdogMux)
#define RADIO_WATCHDOG_UNLOCK() portEXIT_CRITICAL(&radioWatchdogMux)
#else
#define RADIO_WATCHDOG_LOCK() noInterrupts()
#define RADIO_WATCHDOG_UNLOCK() interrupts()
#endif
// Bound for radioWatchdogCommand's ready wait: the longest stock busy allowance is SetStandby(RC)'s 1500 us; with a
// BUSY pin the chip's own busy time is far shorter (WaitOnBusy gives up after 1 ms).
static constexpr uint32_t RADIO_WATCHDOG_READY_TIMEOUT_US = SX1280WatchdogPolicy::COMMAND_READY_TIMEOUT_US;

static void ICACHE_RAM_ATTR radioWatchdogTock()
{
    radioWatchdogTockSkip = false;
    if (!Radio.watchdogEnabled || radioWatchdogResetting)
    {
        return;
    }
    if (radioWatchdogUnverified && GPIO_PIN_BUSY != UNDEF_PIN && digitalRead(GPIO_PIN_BUSY) == HIGH)
    {
        // The sequence still advances (HandleFHSS); the next hop that finds the radio ready retunes it.
        radioWatchdogTockSkip = true;
        return;
    }
    const bool dio1 = digitalRead((uint8_t)radioWatchdogDio1Pin);
    radioWatchdogDio1HighTocks = dio1 ? (uint8_t)(radioWatchdogDio1HighTocks < 255 ? radioWatchdogDio1HighTocks + 1 : 255) : 0;
    if (radioWatchdogStage != 0)
    {
        if (!dio1)
        {
            radioWatchdogStage = 0;  // verified: the repeat clear took effect
            // While the IRQ was stuck, a telemetry TX_DONE raised no edge, so TXdoneISR never put the radio back
            // into Rx (it would sit in FS). Re-enter Rx; the tock is ~200 us after RX done, before the next preamble.
            Radio.RXnb();
            return;
        }
        if (radioWatchdogStage == 2)
        {
            radioWatchdogStage = 0;
            radioWatchdogResetRequested = true;
            return;
        }
        Radio.ClearIrqStatus(SX1280_IRQ_RADIO_ALL, SX12XX_Radio_All);
        radioWatchdogStage = 2;
        return;
    }
    // DIO1 high at two tocks in a row (one packet interval apart) or a suspect clear at least 1 ms old.
    const bool suspectDue = Radio.clearSuspect && (micros() - Radio.clearSuspectMicros) >= RADIO_WATCHDOG_MIN_RETRY_US;
    if (suspectDue || radioWatchdogDio1HighTocks >= RADIO_WATCHDOG_STUCK_TOCKS)
    {
        Radio.clearSuspect = false;
        if (!dio1)
        {
            return;  // the handler's clear did work
        }
        Radio.ClearIrqStatus(SX1280_IRQ_RADIO_ALL, SX12XX_Radio_All);
        radioWatchdogStage = 1;
    }
}
#endif

static void ICACHE_RAM_ATTR HandleFHSS()
{
    uint8_t modresultFHSS = OtaNonce % ExpressLRS_currAirRate_Modparams->FHSShopInterval;

    if ((ExpressLRS_currAirRate_Modparams->FHSShopInterval == 0) || InBindingMode || (modresultFHSS != 0) || (connectionState == disconnected))
    {
        return;
    }
#if defined(RADIO_SX128X)
    if (radioWatchdogResetting || radioWatchdogTockSkip)
    {
        FHSSgetNextFreq();  // keep the sequence position; the radio is retuned at the release / the next hop
        return;
    }
#endif

    if (geminiMode)
    {
        if (((OtaNonce / ExpressLRS_currAirRate_Modparams->FHSShopInterval) % 2 == 0) || FHSSuseDualBand) // When in DualBand do not switch between radios.  The OTA modulation paramters and HighFreq/LowFreq Tx amps are set during Config.

        {
            Radio.SetFrequencyReg(FHSSgetNextFreq(), SX12XX_Radio_1, false);
            Radio.SetFrequencyReg(FHSSgetGeminiFreq(), SX12XX_Radio_2, false);
        }
        else
        {
            // Write radio1 first. This optimises the SPI traffic order.
            uint32_t freqRadio2 = FHSSgetNextFreq();
            Radio.SetFrequencyReg(FHSSgetGeminiFreq(), SX12XX_Radio_1, false);
            Radio.SetFrequencyReg(freqRadio2, SX12XX_Radio_2, false);
        }
    }
    else
    {
        Radio.SetFrequencyReg(FHSSgetNextFreq(), SX12XX_Radio_All, false);
    }

#if defined(RADIO_SX127X)
    // SX127x radio has to reset receive mode after hopping
    uint8_t modresultTLM = OtaNonce % ExpressLRS_currTlmDenom;
    if (modresultTLM != 0 || ExpressLRS_currTlmDenom == 1) // if we are about to send a tlm response don't bother going back to rx
    {
        Radio.RXnb();
    }
#endif
    LbtCcaTimerStart();
}

void ICACHE_RAM_ATTR LinkStatsToOta(OTA_LinkStats_s * const ls)
{
    // The value in linkstatistics is "positivized" (inverted polarity)
    // and must be inverted on the TX side. Positive values are used
    // so save a bit to encode which antenna is in use
    ls->uplink_RSSI_1 = linkStats.uplink_RSSI_1;
    ls->uplink_RSSI_2 = linkStats.uplink_RSSI_2;
    ls->antenna = antenna;
    ls->modelMatch = connectionHasModelMatch;
    ls->lq = linkStats.uplink_Link_quality;
    ls->trueDiversityAvailable = isDualRadio();
#if defined(DEBUG_FREQ_CORRECTION)
    ls->SNR = FreqCorrection * 127 / FreqCorrectionMax;
#else
    if (SnrMean.getCount())
    {
        ls->SNR = SnrMean.mean();
    }
    else
    {
        ls->SNR = SnrMean.previousMean();
    }
#endif
}

#define GenerateOtaDataDl(ota, member, ls)  {\
   const size_t dataLen = sizeof(otaPkt.ota.member.payload); \
   otaPkt.ota.data_dl.stubbornAck = DataUlReceiver.GetCurrentConfirm(); \
   if (geminiMode) \
    { \
        sendGeminiBuffer = true; \
        WORD_ALIGNED_ATTR uint8_t geminiSpanBuffer[2 * dataLen] = {0}; \
\
        otaPkt.ota.data_dl.packageIndex = DataDlSender.GetCurrentPayload(geminiSpanBuffer, sizeof(geminiSpanBuffer)); \
        memcpy(otaPkt.ota.member.payload, geminiSpanBuffer, dataLen); \
        if (ls) LinkStatsToOta(ls); \
\
        otaPktGemini = otaPkt; \
        memcpy(otaPktGemini.ota.member.payload, &geminiSpanBuffer[dataLen], dataLen); \
    } \
    else \
    { \
        otaPkt.ota.data_dl.packageIndex = DataDlSender.GetCurrentPayload(otaPkt.ota.member.payload, dataLen); \
        if (ls) LinkStatsToOta(ls); \
    } \
}

bool ICACHE_RAM_ATTR HandleSendDataDl()
{
#if defined(RADIO_SX128X)
    if (radioWatchdogResetting || radioWatchdogTockSkip)
    {
        return false;  // the loop owns the radio, or it is busy after a failed attempt
    }
#endif
    uint8_t modresult = OtaNonce % ExpressLRS_currTlmDenom;

    if ((connectionState == disconnected) || (ExpressLRS_currTlmDenom == 1) || (alreadyTLMresp == true) || (modresult != 0) || !teamraceHasModelMatch)
    {
        return false; // don't bother sending tlm if disconnected or TLM is off
    }

    // ESP requires word aligned buffer
    WORD_ALIGNED_ATTR OTA_Packet_s otaPkt = {0};
    WORD_ALIGNED_ATTR OTA_Packet_s otaPktGemini = {0};
    alreadyTLMresp = true;
    bool sendGeminiBuffer = false;

    bool tlmQueued = false;
    if (firmwareOptions.is_airport)
    {
        tlmQueued = ((SerialAirPort *)serialIO)->isTlmQueued();
    }
    else
    {
        tlmQueued = DataDlSender.IsActive();
    }

    if (NextTelemetryType == PACKET_TYPE_LINKSTATS || !tlmQueued)
    {
        otaPkt.std.type = PACKET_TYPE_LINKSTATS;
        if (OtaIsFullRes)
        {
            GenerateOtaDataDl(full, data_dl.ul_link_stats,
                &otaPkt.full.data_dl.ul_link_stats.stats);
        }
        else
        {
            GenerateOtaDataDl(std, data_dl.ul_link_stats,
                &otaPkt.std.data_dl.ul_link_stats.stats);
        }

        NextTelemetryType = PACKET_TYPE_DATA;
        // Start the count at 1 because the next will be DATA and doing +1 before checking
        // against Max below is for some reason 10 bytes more code
        telemetryBurstCount = 1;
    }
    else // if tlmQueued
    {
        if (telemetryBurstCount < telemetryBurstMax)
        {
            telemetryBurstCount++;
        }
        else
        {
            NextTelemetryType = PACKET_TYPE_LINKSTATS;
        }

        otaPkt.std.type = PACKET_TYPE_DATA;
        if (firmwareOptions.is_airport)
        {
            OtaPackAirportData(&otaPkt, &((SerialAirPort *)serialIO)->apInputBuffer);
        }
        else if (OtaIsFullRes)
        {
            GenerateOtaDataDl(full, data_dl, nullptr);
        }
        else
        {
            GenerateOtaDataDl(std, data_dl, nullptr);
        }
    }

    OtaGeneratePacketCrc(&otaPkt);
    if (sendGeminiBuffer)
    {
        OtaGeneratePacketCrc(&otaPktGemini);
    }

    SX12XX_Radio_Number_t transmittingRadio;
    if (config.GetForceTlmOff())
    {
        transmittingRadio = SX12XX_Radio_NONE;
    }
    else
    {
        transmittingRadio = LbtChannelIsClear(SX12XX_Radio_All);   // weed out the radio(s) if channel in use
        if (isDualRadio() && !geminiMode && transmittingRadio == SX12XX_Radio_All) // If the receiver is in diversity mode, only send TLM on a single radio.
        {
            transmittingRadio = Radio.GetStrongestReceivingRadio(); // Pick the radio with best rf connection to the tx.
        }
    }

    // Gemini flips frequencies between radios on the rx side only.  This is to help minimise antenna cross polarization.
    // The payloads need to be switch when this happens.
    // GemX does not switch due to the time required to reconfigure the LR1121 params.
    if ((OtaNonce/ExpressLRS_currAirRate_Modparams->FHSShopInterval) % 2 == 0 || !sendGeminiBuffer || FHSSuseDualBand)
    {
        Radio.TXnb((uint8_t*)&otaPkt, sendGeminiBuffer, (uint8_t*)&otaPktGemini, transmittingRadio);
    }
    else
    {
        Radio.TXnb((uint8_t*)&otaPktGemini, sendGeminiBuffer, (uint8_t*)&otaPkt, transmittingRadio);
    }

    if (transmittingRadio == SX12XX_Radio_NONE)
    {
        // No packet will be sent due to LBT / Telem forced off.
        // Defer TXdoneCallback() to prepare for TLM when the IRQ is normally triggered.
        deferExecutionMicros(ExpressLRS_currAirRate_RFperfParams->TOA, Radio.TXdoneCallback);
    }

    return true;
}

int32_t ICACHE_RAM_ATTR HandleFreqCorr(bool value, SX12XX_Radio_Number_t radio)
{
    int32_t tempFC = FreqCorrection;
    if (radio == SX12XX_Radio_2)
    {
        tempFC = FreqCorrection_2;
    }

    if (value)
    {
        if (tempFC > FreqCorrectionMin)
        {
            tempFC--; // FREQ_STEP units
            if (tempFC == FreqCorrectionMin)
            {
                DBGLN("Max -FreqCorrection reached!");
            }
        }
    }
    else
    {
        if (tempFC < FreqCorrectionMax)
        {
            tempFC++; // FREQ_STEP units
            if (tempFC == FreqCorrectionMax)
            {
                DBGLN("Max +FreqCorrection reached!");
            }
        }
    }

    if (radio == SX12XX_Radio_1)
    {
        FreqCorrection = tempFC;
    }
    else
    {
        FreqCorrection_2 = tempFC;
    }

    return tempFC;
}

void ICACHE_RAM_ATTR updatePhaseLock()
{
    if (connectionState != disconnected && PFDloop.hasResult())
    {
        int32_t RawOffset = PFDloop.calcResult();
        int32_t Offset = LPF_Offset.update(RawOffset);
        int32_t OffsetDx = LPF_OffsetDx.update(RawOffset - PfdPrevRawOffset);
        PfdPrevRawOffset = RawOffset;

        if (RXtimerState == tim_locked)
        {
            // limit rate of freq offset adjustment
            if (OtaNonce % 8 == 0)
            {
                if (Offset > 0)
                {
                    hwTimer::incFreqOffset();
                }
                else if (Offset < 0)
                {
                    hwTimer::decFreqOffset();
                }
            }
        }

        if (connectionState != connected)
        {
            hwTimer::phaseShift(RawOffset >> 1);
        }
        else
        {
            hwTimer::phaseShift(Offset >> 2);
        }

        DBGVLN("%d:%d:%d:%d:%d", Offset, RawOffset, OffsetDx, hwTimer::getFreqOffset(), uplinkLQ);
        UNUSED(OffsetDx); // complier warning if no debug
    }

    PFDloop.reset();
}

void ICACHE_RAM_ATTR HWtimerCallbackTick() // this is 180 out of phase with the other callback, occurs mid-packet reception
{
    if (ExpressLRS_currAirRate_Modparams->numOfSends == 1)
    {
        // Save the LQ value before the inc() reduces it by 1
        uplinkLQ = LQCalc.getLQ();
    } else
    if (!((OtaNonce - 1) % ExpressLRS_currAirRate_Modparams->numOfSends))
    {
        uplinkLQ = LQCalcDVDA.getLQ();
        LQCalcDVDA.inc();
    }

    linkStats.uplink_Link_quality = uplinkLQ;
    // Only advance the LQI period counter if we didn't send Telemetry this period
    if (!alreadyTLMresp)
        LQCalc.inc();

    alreadyTLMresp = false;
}

//////////////////////////////////////////////////////////////
// flip to the other antenna
// no-op if GPIO_PIN_ANT_CTRL not defined
static inline void switchAntenna()
{
    if (GPIO_PIN_ANT_CTRL != UNDEF_PIN && config.GetAntennaMode() == 2)
    {
        // 0 and 1 is use for gpio_antenna_select
        // 2 is diversity
        antenna = !antenna;
        (antenna == 0) ? LPF_UplinkRSSI0.reset() : LPF_UplinkRSSI1.reset(); // discard the outdated value after switching
        digitalWrite(GPIO_PIN_ANT_CTRL, antenna);
    }
}

static void ICACHE_RAM_ATTR updateDiversity()
{

    if (GPIO_PIN_ANT_CTRL != UNDEF_PIN)
    {
        if(config.GetAntennaMode() == 2)
        {
            // 0 and 1 is use for gpio_antenna_select
            // 2 is diversity
            static int32_t prevRSSI;        // saved rssi so that we can compare if switching made things better or worse
            static int32_t antennaLQDropTrigger;
            static int32_t antennaRSSIDropTrigger;
            int32_t rssi = (antenna == 0) ? LPF_UplinkRSSI0.value() : LPF_UplinkRSSI1.value();
            int32_t otherRSSI = (antenna == 0) ? LPF_UplinkRSSI1.value() : LPF_UplinkRSSI0.value();

            //if rssi dropped by the amount of DIVERSITY_ANTENNA_RSSI_TRIGGER
            if ((rssi < (prevRSSI - DIVERSITY_ANTENNA_RSSI_TRIGGER)) && antennaRSSIDropTrigger >= DIVERSITY_ANTENNA_INTERVAL)
            {
                switchAntenna();
                antennaLQDropTrigger = 1;
                antennaRSSIDropTrigger = 0;
            }
            else if (rssi > prevRSSI || antennaRSSIDropTrigger < DIVERSITY_ANTENNA_INTERVAL)
            {
                prevRSSI = rssi;
                antennaRSSIDropTrigger++;
            }

            // if we didn't get a packet switch the antenna
            if (!LQCalc.currentIsSet() && antennaLQDropTrigger == 0)
            {
                switchAntenna();
                antennaLQDropTrigger = 1;
                antennaRSSIDropTrigger = 0;
            }
            else if (antennaLQDropTrigger >= DIVERSITY_ANTENNA_INTERVAL)
            {
                // We switched antenna on the previous packet, so we now have relatively fresh rssi info for both antennas.
                // We can compare the rssi values and see if we made things better or worse when we switched
                if (rssi < otherRSSI)
                {
                    // things got worse when we switched, so change back.
                    switchAntenna();
                    antennaLQDropTrigger = 1;
                    antennaRSSIDropTrigger = 0;
                }
                else
                {
                    // all good, we can stay on the current antenna. Clear the flag.
                    antennaLQDropTrigger = 0;
                }
            }
            else if (antennaLQDropTrigger > 0)
            {
                antennaLQDropTrigger ++;
            }
        }
        else
        {
            digitalWrite(GPIO_PIN_ANT_CTRL, config.GetAntennaMode());
            antenna = config.GetAntennaMode();
        }
    }
    if (GPIO_PIN_ANT_GROUP != UNDEF_PIN)
    {
        digitalWrite(GPIO_PIN_ANT_GROUP, config.GetAntennaGroup());
    }
}

void ICACHE_RAM_ATTR HWtimerCallbackTock()
{
    PFDloop.intEvent(micros()); // our internal osc just fired

    if (ExpressLRS_currAirRate_Modparams->numOfSends > 1 && !(OtaNonce % ExpressLRS_currAirRate_Modparams->numOfSends))
    {
        if (LQCalcDVDA.currentIsSet())
        {
            crsfRCFrameAvailable();
            if (teamraceHasModelMatch)
                servoNewChannelsAvailable();
        }
        else
        {
            crsfRCFrameMissed();
        }
    }
    else if (ExpressLRS_currAirRate_Modparams->numOfSends == 1)
    {
        if (!LQCalc.currentIsSet())
        {
            crsfRCFrameMissed();
        }
    }

    // For any serial drivers that need to send on a regular cadence (i.e. CRSF to betaflight)
    sendImmediateRC();

    OtaNonce++;
#if defined(RADIO_SX128X)
    radioWatchdogTock();  // before the hop: the HAL busy timer spaces the hop after a repair clear
#endif
    HandleFHSS();
    updateDiversity();
    bool tlmSent = HandleSendDataDl();
    updatePhaseLock();

    #if defined(DEBUG_RX_SCOREBOARD)
    static bool lastPacketWasTelemetry = false;
    if (!LQCalc.currentIsSet() && !lastPacketWasTelemetry)
        DBGW(lastPacketCrcError ? '.' : '_');
    lastPacketCrcError = false;
    lastPacketWasTelemetry = tlmSent;
    #else
    UNUSED(tlmSent);
    #endif
}

void LostConnection(bool resumeRx)
{
    DBGLN("lost conn fc=%d fo=%d", FreqCorrection, hwTimer::getFreqOffset());

    setConnectionState(disconnected); //set lost connection
    RXtimerState = tim_disconnected;
    hwTimer::resetFreqOffset();
    PfdPrevRawOffset = 0;
    GotConnectionMillis = 0;
    uplinkLQ = 0;
    LQCalc.reset();
    LQCalcDVDA.reset();
    LPF_Offset.init(0);
    LPF_OffsetDx.init(0);
    alreadyTLMresp = false;
    OtaResetChannelDataComplete();

    if (!InBindingMode)
    {
        if (hwTimer::running)
        {
            while(micros() - PFDloop.getIntEventTime() > 250); // time it just after the tock()
            hwTimer::stop();
        }
        SetRFLinkRate(ExpressLRS_nextAirRateIndex, false); // also sets to initialFreq
        // If not resumRx, Radio will be left in SX127x_OPMODE_STANDBY / SX1280_MODE_STDBY_XOSC
        if (resumeRx)
        {
            Radio.RXnb();
        }
    }
}

void ICACHE_RAM_ATTR TentativeConnection(unsigned long now)
{
    PFDloop.reset();
    setConnectionState(tentative);
    connectionHasModelMatch = false;
    ChannelDataReset();
    OtaResetChannelDataComplete();
    RXtimerState = tim_disconnected;
    DBGLN("tentative conn");
    PfdPrevRawOffset = 0;
    LPF_Offset.init(0);
    SnrMean.reset();
    RFmodeLastCycled = now; // give another 3 sec for lock to occur

    // Use this rate as the initial rate next time if we connected on it
    config.SetRateInitialIdx(ExpressLRS_nextAirRateIndex);
    // And stop counting toward binding mode
    if (config.GetPowerOnCounter() != 0)
    {
        config.SetPowerOnCounter(0);
    }

    // The caller MUST call hwTimer::resume(). It is not done here because
    // the timer ISR will fire immediately and preempt any other code
}

void GotConnection(unsigned long now)
{
    if (connectionState == connected)
    {
        return; // Already connected
    }

    LockRFmode = firmwareOptions.lock_on_first_connection;

    setConnectionState(connected); //we got a packet, therefore no lost connection
    RXtimerState = tim_tentative;
    GotConnectionMillis = now;
    webserverPreventAutoStart = true;

    if (firmwareOptions.is_airport)
    {
        ((SerialAirPort *)serialIO)->apInputBuffer.flush();
        ((SerialAirPort *)serialIO)->apOutputBuffer.flush();
    }

    DBGLN("got conn");
}

static void ICACHE_RAM_ATTR ProcessRfPacket_RC(OTA_Packet_s const * const otaPktPtr)
{
    // Must be fully connected to process RC packets, prevents processing RC
    // during sync, where packets can be received before connection
    if (connectionState != connected || SwitchModePending)
        return;

    bool telemetryConfirmValue = OtaUnpackChannelData(otaPktPtr, ChannelData);
    DataDlSender.ConfirmCurrentPayload(telemetryConfirmValue);

    bool const channelDataComplete = OtaIsChannelDataComplete(ChannelData);

    // No channels packets to the FC or PWM pins if no model match
    if (connectionHasModelMatch && channelDataComplete)
    {
        if (ExpressLRS_currAirRate_Modparams->numOfSends == 1)
        {
            crsfRCFrameAvailable();
            // teamrace is only checked for servos because the teamrace model select logic only runs
            // when new frames are available, and will decide later if the frame will be forwarded
            if (teamraceHasModelMatch)
                servoNewChannelsAvailable();
        }
        else if (!LQCalcDVDA.currentIsSet())
        {
            LQCalcDVDA.add();
        }
        #if defined(DEBUG_RCVR_LINKSTATS)
        debugRcvrLinkstatsPending = true;
        #endif
    }
}

void ICACHE_RAM_ATTR OnELRSBindMSP(uint8_t* newUid4)
{
    // Binding over MSP only contains 4 bytes due to packet size limitations, clear out any leading bytes
    UID[0] = 0;
    UID[1] = 0;
    for (unsigned i = 0; i < 4; i++)
    {
        UID[i + 2] = newUid4[i];
    }

    DBGLN("New UID = %u, %u, %u, %u, %u, %u", UID[0], UID[1], UID[2], UID[3], UID[4], UID[5]);

    // Set new UID in eeprom
    // EEPROM commit will happen on the main thread in ExitBindingMode()
    config.SetUID(UID);
}

static void ICACHE_RAM_ATTR ProcessRfPacket_DataUl(OTA_Packet_s const * const otaPktPtr)
{
    if (firmwareOptions.is_airport)
    {
        OtaUnpackAirportData(otaPktPtr, &((SerialAirPort *)serialIO)->apOutputBuffer);
        return;
    }

    uint8_t packageIndex;
    uint8_t const * payload;
    uint8_t dataLen;
    if (OtaIsFullRes)
    {
        packageIndex = otaPktPtr->full.data_ul.packageIndex;
        payload = otaPktPtr->full.data_ul.payload;
        dataLen = sizeof(otaPktPtr->full.data_ul.payload);
        if (config.GetSerialProtocol() == PROTOCOL_MAVLINK)
        {
            DataDlSender.ConfirmCurrentPayload(otaPktPtr->full.data_ul.stubbornAck);
        }
    }
    else
    {
        packageIndex = otaPktPtr->std.data_ul.packageIndex;
        payload = otaPktPtr->std.data_ul.payload;
        dataLen = sizeof(otaPktPtr->std.data_ul.payload);
        if (config.GetSerialProtocol() == PROTOCOL_MAVLINK)
        {
            DataDlSender.ConfirmCurrentPayload(otaPktPtr->std.data_ul.stubbornAck);
        }
    }

    // Always examine DATA type packets for bind information if in bind mode
    // [1] is the package index, first packet of the MSP
    if (InBindingMode && packageIndex == 1 && payload[0] == MSP_ELRS_BIND)
    {
        OnELRSBindMSP((uint8_t *)&payload[1]);
        return;
    }

    // Must be fully connected to process uplink data, prevents processing data
    // during sync, where packets can be received before connection
    if (connectionState == connected)
    {
        DataUlReceiver.ReceiveData(packageIndex, payload, dataLen);
    }
}

static void ICACHE_RAM_ATTR updateSwitchModePendingFromOta(uint8_t newSwitchMode)
{
    if (OtaSwitchModeCurrent == newSwitchMode)
    {
        // Cancel any switch if pending
        SwitchModePending = 0;
        return;
    }

    // One is added to the mode because SwitchModePending==0 means no switch pending
    // and that's also a valid switch mode. The 1 is removed when this is handled.
    // A negative SwitchModePending means not to switch yet
    int8_t newSwitchModePending = -(int8_t)newSwitchMode - 1;

    // Switch mode can be changed while disconnected
    // OR there are two sync packets with the same new switch mode,
    // as a "confirm". No RC packets are processed until
    if (connectionState == disconnected ||
        SwitchModePending == newSwitchModePending)
    {
        // Add one to the mode because SwitchModePending==0 means no switch pending
        // and that's also a valid switch mode. The 1 is removed when this is handled
        SwitchModePending = newSwitchMode + 1;
    }
    else
    {
        // Save the negative version of the new switch mode to compare
        // against on the next SYNC packet, but do not switch yet
        SwitchModePending = newSwitchModePending;
    }
}

static bool ICACHE_RAM_ATTR ProcessRfPacket_SYNC(uint32_t const now, OTA_Sync_s const * const otaSync)
{
    // Verify the first byte of the binding ID, which should always match
    if (otaSync->UID4 != UID[4])
        return false;

    // The third byte will be XORed with inverse of the ModelId if ModelMatch is on
    // Only require the first 18 bits of the UID to match to establish a connection
    // but the last 6 bits must modelmatch before sending any data to the FC
    if ((otaSync->UID5 & ~MODELMATCH_MASK) != (UID[5] & ~MODELMATCH_MASK))
        return false;

    // Check the any proposed packet rate switch and verify the hardware has a valid config
    // and hardware frontend for it. Ignore the mode switch if not valid.
    uint8_t proposedRateIdx;
    expresslrs_RFrates_e proposedRateEnum = (expresslrs_RFrates_e)otaSync->rfRateEnum;
    if (!enumRatetoIndex(proposedRateEnum, proposedRateIdx)
        || !isSupportedRFRate(proposedRateIdx))
    {
        DBGLN("Unsupported rate %u", proposedRateEnum);
        return false;
    }

    LastSyncPacket = now;
#if defined(DEBUG_RX_SCOREBOARD)
    DBGW('s');
#endif

    if (otaSync->otaProtocol == TX_MAVLINK_MODE)
    {
        config.SetSerialProtocol(PROTOCOL_MAVLINK);
    }
    else if (config.GetSerialProtocol() == PROTOCOL_MAVLINK)
    {
        config.SetSerialProtocol(PROTOCOL_CRSF);
    }

    // Check if otaProtocol has been updated.
    if (config.IsModified())
    {
        deferExecutionMillis(100, [](){
            reconfigureSerial();
        });
    }

    if (isDualRadio())
    {
        config.SetAntennaMode(otaSync->geminiMode);
    }

    // Will change the packet air rate in loop() if this changes
    ExpressLRS_nextAirRateIndex = proposedRateIdx;
    updateSwitchModePendingFromOta(otaSync->switchEncMode);

    // Update TLM ratio, should never be TLM_RATIO_STD/DISARMED, the TX calculates the correct value for the RX
    expresslrs_tlm_ratio_e TLMrateIn = (expresslrs_tlm_ratio_e)(otaSync->newTlmRatio + (uint8_t)TLM_RATIO_NO_TLM);
    uint8_t TlmDenom = TLMratioEnumToValue(TLMrateIn);
    if (ExpressLRS_currTlmDenom != TlmDenom)
    {
        DBGLN("New TLMrate 1:%u", TlmDenom);
        ExpressLRS_currTlmDenom = TlmDenom;
        telemBurstValid = false;
    }

    // modelId = 0xff indicates modelMatch is disabled, the XOR does nothing in that case
    uint8_t modelXor = (~config.GetModelId()) & MODELMATCH_MASK;
    bool modelMatched = otaSync->UID5 == (UID[5] ^ modelXor);
    DBGVLN("MM %u=%u %d", otaSync->UID5, UID[5], modelMatched);

    if (connectionState == disconnected
        || OtaNonce != otaSync->nonce
        || FHSSgetCurrIndex() != otaSync->fhssIndex
        || connectionHasModelMatch != modelMatched)
    {
        //DBGLN("\r\n%ux%ux%u", OtaNonce, otaSync->nonce, otaSync->fhssIndex);
        FHSSsetCurrIndex(otaSync->fhssIndex);
        OtaNonce = otaSync->nonce;
        TentativeConnection(now);
        // connectionHasModelMatch must come after TentativeConnection, which resets it
        connectionHasModelMatch = modelMatched;
        return true;
    }

    return false;
}

bool ICACHE_RAM_ATTR ProcessRFPacket(SX12xxDriverCommon::rx_status const status)
{
    if (status != SX12xxDriverCommon::SX12XX_RX_OK)
    {
        DBGVLN("HW CRC error");
        #if defined(DEBUG_RX_SCOREBOARD)
            lastPacketCrcError = true;
        #endif
        return false;
    }
    uint32_t const beginProcessing = micros();

    OTA_Packet_s * const otaPktPtr = (OTA_Packet_s * const)Radio.RXdataBuffer;
    OTA_Packet_s * const otaPktPtrSecond = (OTA_Packet_s * const)Radio.RXdataBufferSecond;

    if (!OtaValidatePacketCrc(otaPktPtr))
    {
        DBGVLN("CRC error");
        #if defined(DEBUG_RX_SCOREBOARD)
            lastPacketCrcError = true;
        #endif
        return false;
    }

    // The extEvent defines where TOCK timer ISR is to be synced to, i.e. where the packet period begins.
    // For rates where the TOA is longer than half the packet period schedule the TOCK for rougly 1x TOA before
    // the TX's end of the period so telemetry is received by the TX in the correct period. For all others,
    // schedule TOCK to be PACKET_TO_TOCK_SLACK (us) after RX packet reception.
    int32_t slack = std::max(ExpressLRS_currAirRate_Modparams->interval - 2 * ExpressLRS_currAirRate_RFperfParams->TOA, (int32_t)PACKET_TO_TOCK_SLACK);
    PFDloop.extEvent(beginProcessing + slack);

    doStartTimer = false;
    unsigned long now = millis();

    LastValidPacket = now;

    Radio.CheckForSecondPacket();
    if (Radio.hasSecondRadioGotData)
    {
        if (!OtaValidatePacketCrc(otaPktPtrSecond))
        {
            Radio.hasSecondRadioGotData = false;
        }
    }

    switch (otaPktPtr->std.type)
    {
    case PACKET_TYPE_RCDATA: //Standard RC Data Packet
        ProcessRfPacket_RC(otaPktPtr);
        break;
    case PACKET_TYPE_SYNC: //sync packet from master
        doStartTimer = ProcessRfPacket_SYNC(now,
            OtaIsFullRes ? &otaPktPtr->full.sync.sync : &otaPktPtr->std.sync)
            && !InBindingMode;
        break;
    case PACKET_TYPE_DATA:
        ProcessRfPacket_DataUl(otaPktPtr);
        break;
    default:
        break;
    }

    // Store the LQ/RSSI/Antenna
    Radio.GetLastPacketStats();
    getRFlinkInfo();

    // Adjusts FreqCorrection for RX freq offset
    if (Radio.FrequencyErrorAvailable())
    {
    #if defined(RADIO_SX127X)
        int32_t tempFreqCorrection = HandleFreqCorr(Radio.GetFrequencyErrorbool(Radio.GetProcessingPacketRadio()), Radio.GetProcessingPacketRadio());
        // Teamp900 also needs to adjust its demood PPM
        Radio.SetPPMoffsetReg(tempFreqCorrection, Radio.GetProcessingPacketRadio());

        if (Radio.hasSecondRadioGotData)
        {
            SX12XX_Radio_Number_t secondRadio = Radio.GetProcessingPacketRadio() == SX12XX_Radio_1 ? SX12XX_Radio_2 : SX12XX_Radio_1;
            tempFreqCorrection = HandleFreqCorr(Radio.GetFrequencyErrorbool(secondRadio), secondRadio);
            Radio.SetPPMoffsetReg(tempFreqCorrection, secondRadio);
        }
    #else
        HandleFreqCorr(Radio.GetFrequencyErrorbool(Radio.GetProcessingPacketRadio()), Radio.GetProcessingPacketRadio());

        if (Radio.hasSecondRadioGotData)
        {
            SX12XX_Radio_Number_t secondRadio = Radio.GetProcessingPacketRadio() == SX12XX_Radio_1 ? SX12XX_Radio_2 : SX12XX_Radio_1;
            HandleFreqCorr(Radio.GetFrequencyErrorbool(secondRadio), secondRadio);
        }
    #endif
    }

    // Received a packet, that's the definition of LQ
    LQCalc.add();
    // Extend sync duration since we've received a packet at this rate
    // but do not extend it indefinitely
    RFmodeCycleMultiplier = RFmodeCycleMultiplierSlow;

#if defined(DEBUG_RX_SCOREBOARD)
    if (otaPktPtr->std.type != PACKET_TYPE_SYNC) DBGW(connectionHasModelMatch ? 'R' : 'r');
#endif

    return true;
}

bool ICACHE_RAM_ATTR RXdoneISR(SX12xxDriverCommon::rx_status const status)
{
    if (LQCalc.currentIsSet() && connectionState == connected)
    {
        return false; // Already received a packet, do not run ProcessRFPacket() again.
    }

    if (ProcessRFPacket(status))
    {
        if (doStartTimer)
        {
            doStartTimer = false;
            hwTimer::resume(); // will throw an interrupt immediately
        }

        return true;
    }
    return false;
}

void ICACHE_RAM_ATTR TXdoneISR()
{
    Radio.RXnb();
    LbtCcaTimerStart();
#if defined(DEBUG_RX_SCOREBOARD)
    DBGW('T');
#endif
}

/**
 * Process the assembled uplink data packet in DataUlBuffer[]
 **/
void DataUlReceiveComplete()
{
    switch (DataUlBuffer[0])
    {
    case MSP_ELRS_SET_RX_WIFI_MODE: //0x0E
        // The MSP packet needs to be ACKed so the TX doesn't
        // keep sending it, so defer the switch to wifi
        deferExecutionMillis(500, []() {
            setWifiUpdateMode();
        });
        break;
    case MSP_ELRS_MAVLINK_TLM: // 0xFD
        // raw mavlink data
        if (config.GetSerialProtocol() == PROTOCOL_MAVLINK)
        {
            ((SerialMavlink *)serialIO)->forwardMessage(DataUlBuffer);
        }
        break;
    default:
        //handle received CRSF package
        const auto receivedHeader = (crsf_header_t *) DataUlBuffer;
        crsfRouter.processMessage(&otaConnector, receivedHeader);

    }

    DataUlReceiver.Unlock();
}

static void setupSerial()
{
    bool sbusSerialOutput = false;
	bool sumdSerialOutput = false;
    bool mavlinkSerialOutput = false;
    bool hottTlmSerial = false;

    if (OPT_CRSF_RCVR_NO_SERIAL)
    {
        // For PWM receivers with no serial pins defined, only turn on the Serial port if logging is on
        #if defined(DEBUG_LOG) || defined(DEBUG_RCVR_LINKSTATS)
        #if defined(PLATFORM_ESP32_S3) && !defined(ESP32_S3_USB_JTAG_ENABLED)
        // Requires pull-down on GPIO3.  If GPIO3 has a pull-up (for JTAG) this doesn't work.
        USBSerial.begin(serialBaud);
        BackpackOrLogStrm = &USBSerial;
        #else
        Serial.begin(serialBaud);
        BackpackOrLogStrm = &Serial;
        #endif
        #else
        BackpackOrLogStrm = new NullStream();
        #endif
        serialIO = new SerialNOOP();
        return;
    }
    if (config.GetSerialProtocol() == PROTOCOL_CRSF || config.GetSerialProtocol() == PROTOCOL_INVERTED_CRSF || firmwareOptions.is_airport)
    {
        serialBaud = firmwareOptions.uart_baud;
    }
    else if (config.GetSerialProtocol() == PROTOCOL_SBUS || config.GetSerialProtocol() == PROTOCOL_INVERTED_SBUS || config.GetSerialProtocol() == PROTOCOL_DJI_RS_PRO)
    {
        sbusSerialOutput = true;
        serialBaud = 100000;
    }
    else if (config.GetSerialProtocol() == PROTOCOL_SUMD)
    {
        sumdSerialOutput = true;
        serialBaud = 115200;
    }
    else if (config.GetSerialProtocol() == PROTOCOL_MAVLINK)
    {
        mavlinkSerialOutput = true;
        serialBaud = 460800;
    }
    else if (config.GetSerialProtocol() == PROTOCOL_MSP_DISPLAYPORT)
    {
        serialBaud = 115200;
    }
    else if (config.GetSerialProtocol() == PROTOCOL_HOTT_TLM)
    {
        hottTlmSerial = true;
        serialBaud = 19200;
    }
    else if (config.GetSerialProtocol() == PROTOCOL_GPS)
    {
        serialBaud = 115200;
    }
    bool invert = config.GetSerialProtocol() == PROTOCOL_SBUS || config.GetSerialProtocol() == PROTOCOL_INVERTED_CRSF || config.GetSerialProtocol() == PROTOCOL_DJI_RS_PRO;

#if defined(PLATFORM_ESP8266)
    SerialConfig serialConfig = SERIAL_8N1;

    if(sbusSerialOutput)
    {
        serialConfig = SERIAL_8E2;
    }
    else if(hottTlmSerial)
    {
        serialConfig = SERIAL_8N2;
    }

    SerialMode mode = (sbusSerialOutput || sumdSerialOutput)  ? SERIAL_TX_ONLY : SERIAL_FULL;
    Serial.begin(serialBaud, serialConfig, mode, -1, invert);
#elif defined(PLATFORM_ESP32)
    uint32_t serialConfig = SERIAL_8N1;

    if(sbusSerialOutput)
    {
        serialConfig = SERIAL_8E2;
    }
    else if(hottTlmSerial)
    {
        serialConfig = SERIAL_8N2;
    }

    // ARDUINO_CORE_INVERT_FIX PT2
    #if defined(ARDUINO_CORE_INVERT_FIX)
    if(invert == false)
    {
        uart_set_line_inverse(0, UART_SIGNAL_INV_DISABLE);
    }
    #endif
    // ARDUINO_CORE_INVERT_FIX PT2 end

    Serial.begin(serialBaud, serialConfig, GPIO_PIN_RCSIGNAL_RX, GPIO_PIN_RCSIGNAL_TX, invert);
#endif

    if (firmwareOptions.is_airport)
    {
        serialIO = new SerialAirPort(SERIAL_PROTOCOL_TX, SERIAL_PROTOCOL_RX);
    }
    else if (sbusSerialOutput)
    {
        serialIO = new SerialSBUS(SERIAL_PROTOCOL_TX, SERIAL_PROTOCOL_RX);
    }
    else if (sumdSerialOutput)
    {
        serialIO = new SerialSUMD(SERIAL_PROTOCOL_TX, SERIAL_PROTOCOL_RX);
    }
    else if (mavlinkSerialOutput)
    {
        serialIO = new SerialMavlink(SERIAL_PROTOCOL_TX, SERIAL_PROTOCOL_RX);
    }
    else if (config.GetSerialProtocol() == PROTOCOL_MSP_DISPLAYPORT)
    {
        serialIO = new SerialDisplayport(SERIAL_PROTOCOL_TX, SERIAL_PROTOCOL_RX);
    }
    else if (config.GetSerialProtocol() == PROTOCOL_GPS)
    {
        serialIO = new SerialGPS(SERIAL_PROTOCOL_TX, SERIAL_PROTOCOL_RX);
    }
    else if (hottTlmSerial)
    {
        serialIO = new SerialHoTT_TLM(SERIAL_PROTOCOL_TX, SERIAL_PROTOCOL_RX);
    }
    else
    {
        serialIO = new SerialCRSF(SERIAL_PROTOCOL_TX, SERIAL_PROTOCOL_RX);
    }

#if defined(DEBUG_ENABLED)
#if defined(PLATFORM_ESP32_S3) || defined(PLATFORM_ESP32_C3)
    USBSerial.begin(460800);
    BackpackOrLogStrm = &USBSerial;
#else
    BackpackOrLogStrm = &Serial;
#endif
#else
    BackpackOrLogStrm = new NullStream();
#endif
}

#if defined(PLATFORM_ESP32)
static void serial1Shutdown()
{
    if(serial1IO != nullptr)
    {
        Serial1.end();
        delete serial1IO;
        serial1IO = nullptr;
    }
}

static void setupSerial1()
{
    //
    // init secondary serial and protocol
    //
    int8_t serial1RXpin = GPIO_PIN_SERIAL1_RX;

    if (serial1RXpin == UNDEF_PIN)
    {
        for (uint8_t ch = 0; ch < GPIO_PIN_PWM_OUTPUTS_COUNT; ch++)
        {
            if (config.GetPwmChannel(ch)->val.mode == somSerial1RX)
                serial1RXpin = GPIO_PIN_PWM_OUTPUTS[ch];
        }
    }

    int8_t serial1TXpin = GPIO_PIN_SERIAL1_TX;

    if (serial1TXpin == UNDEF_PIN)
    {
        for (uint8_t ch = 0; ch < GPIO_PIN_PWM_OUTPUTS_COUNT; ch++)
        {
            if (config.GetPwmChannel(ch)->val.mode == somSerial1TX)
                serial1TXpin = GPIO_PIN_PWM_OUTPUTS[ch];
        }
    }

    switch(config.GetSerial1Protocol())
    {
        case PROTOCOL_SERIAL1_OFF:
            break;
        case PROTOCOL_SERIAL1_CRSF:
            Serial1.begin(firmwareOptions.uart_baud, SERIAL_8N1, serial1RXpin, serial1TXpin, false);
            serial1IO = new SerialCRSF(SERIAL1_PROTOCOL_TX, SERIAL1_PROTOCOL_RX);
            break;
        case PROTOCOL_SERIAL1_INVERTED_CRSF:
            Serial1.begin(firmwareOptions.uart_baud, SERIAL_8N1, serial1RXpin, serial1TXpin, true);
            serial1IO = new SerialCRSF(SERIAL1_PROTOCOL_TX, SERIAL1_PROTOCOL_RX);
            break;
        case PROTOCOL_SERIAL1_SBUS:
        case PROTOCOL_SERIAL1_DJI_RS_PRO:
            Serial1.begin(100000, SERIAL_8E2, UNDEF_PIN, serial1TXpin, true);
            serial1IO = new SerialSBUS(SERIAL1_PROTOCOL_TX, SERIAL1_PROTOCOL_RX);
            break;
        case PROTOCOL_SERIAL1_INVERTED_SBUS:
            Serial1.begin(100000, SERIAL_8E2, UNDEF_PIN, serial1TXpin, false);
            serial1IO = new SerialSBUS(SERIAL1_PROTOCOL_TX, SERIAL1_PROTOCOL_RX);
            break;
        case PROTOCOL_SERIAL1_SUMD:
            Serial1.begin(115200, SERIAL_8N1, UNDEF_PIN, serial1TXpin, false);
            serial1IO = new SerialSUMD(SERIAL1_PROTOCOL_TX, SERIAL1_PROTOCOL_RX);
            break;
        case PROTOCOL_SERIAL1_HOTT_TLM:
            Serial1.begin(19200, SERIAL_8N2, serial1RXpin, serial1TXpin, false);
            serial1IO = new SerialHoTT_TLM(SERIAL1_PROTOCOL_TX, SERIAL1_PROTOCOL_RX, serial1TXpin);
            break;
        case PROTOCOL_SERIAL1_TRAMP:
            Serial1.begin(9600, SERIAL_8N1, UNDEF_PIN, serial1TXpin, false);
            serial1IO = new SerialTramp(SERIAL1_PROTOCOL_TX, SERIAL1_PROTOCOL_RX, serial1TXpin);
            break;
        case PROTOCOL_SERIAL1_SMARTAUDIO:
            Serial1.begin(4800, SERIAL_8N2, UNDEF_PIN, serial1TXpin, false);
            serial1IO = new SerialSmartAudio(SERIAL1_PROTOCOL_TX, SERIAL1_PROTOCOL_RX, serial1TXpin);
            break;
        case PROTOCOL_SERIAL1_MSP_DISPLAYPORT:
            Serial1.begin(115200, SERIAL_8N1, UNDEF_PIN, serial1TXpin, false);
            serial1IO = new SerialDisplayport(SERIAL1_PROTOCOL_TX, SERIAL1_PROTOCOL_RX);
            break;
        case PROTOCOL_SERIAL1_GPS:
            Serial1.begin(115200, SERIAL_8N1, serial1RXpin, serial1TXpin, false);
            serial1IO = new SerialGPS(SERIAL1_PROTOCOL_TX, SERIAL1_PROTOCOL_RX);
            break;
    }
}

void reconfigureSerial1()
{
    serial1Shutdown();
    setupSerial1();
}
#else
    void setupSerial1() {};
    void reconfigureSerial1() {};
#endif

static void serialShutdown()
{
    BackpackOrLogStrm = new NullStream();
    if(serialIO != nullptr)
    {
        Serial.end();
        delete serialIO;
        serialIO = nullptr;
    }
}

void reconfigureSerial()
{
    serialShutdown();
    setupSerial();
}

static void setupConfigAndPocCheck()
{
    eeprom.Begin();
    config.SetStorageProvider(&eeprom); // Pass pointer to the Config class for access to storage
    config.Load();

    // If bound, track number of plug/unplug cycles to go to binding mode in eeprom
    if (config.GetIsBound() && config.GetPowerOnCounter() < 3)
    {
        config.SetPowerOnCounter(config.GetPowerOnCounter() + 1);
        config.Commit();
    }

    // Set a deferred function to clear the power on counter if the RX has been running for more than 2s
    deferExecutionMillis(2000, []() {
        if (connectionState != connected && config.GetPowerOnCounter() != 0)
        {
            config.SetPowerOnCounter(0);
            config.Commit();
        }
    });
}

static void setupTarget()
{
    if (GPIO_PIN_ANT_CTRL != UNDEF_PIN)
    {
        pinMode(GPIO_PIN_ANT_CTRL, OUTPUT);
        digitalWrite(GPIO_PIN_ANT_CTRL, LOW);
    }
    if (GPIO_PIN_ANT_GROUP != UNDEF_PIN)
    {
        pinMode(GPIO_PIN_ANT_GROUP, OUTPUT);
        digitalWrite(GPIO_PIN_ANT_GROUP, LOW);
    }

    setupTargetCommon();
}

static void setupBindingFromConfig()
{
    // VolatileBind's only function is to prevent loading the stored UID into RAM
    // which makes the RX boot into bind mode every time
    if (config.GetIsBound())
    {
        memcpy(UID, config.GetUID(), UID_LEN);
    }

    DBGLN("UID=(%u, %u, %u, %u, %u, %u) ModelId=%u",
        UID[0], UID[1], UID[2], UID[3], UID[4], UID[5], config.GetModelId());

    OtaUpdateCrcInitFromUid();
}

static void setupRadio()
{
    ChannelDataReset();

    Radio.currFreq = FHSSgetInitialFreq();
#if defined(RADIO_SX127X)
    //Radio.currSyncWord = UID[3];
#endif
    bool init_success = Radio.Begin(FHSSgetMinimumFreq(), FHSSgetMaximumFreq());
    POWERMGNT::init();
    if (!init_success)
    {
        DBGLN("Failed to detect RF chipset!!!");
        setConnectionState(radioFailed);
        return;
    }

    DynamicPower_UpdateRx(true);  // Call before SetRFLinkRate(). The LR1121 Radio lib can now set the correct output power in Config().

    Radio.RXdoneCallback = &RXdoneISR;
    Radio.TXdoneCallback = &TXdoneISR;

    scanIndex = config.GetRateInitialIdx();
    for (int i=0 ; i<RATE_MAX ; i++)
    {
        if (isSupportedRFRate(scanIndex))
        {
            break;
        }
        scanIndex = (scanIndex + 1) % RATE_MAX;
    }
    SetRFLinkRate(scanIndex, false);
    // Start slow on the selected rate to give it the best chance
    // to connect before beginning rate cycling
    RFmodeCycleMultiplier = RFmodeCycleMultiplierSlow / 2;
}

static void updateTelemetryBurst()
{
    if (telemBurstValid)
        return;
    telemBurstValid = true;

    uint16_t hz = 1000000 / ExpressLRS_currAirRate_Modparams->interval;
    telemetryBurstMax = TLMBurstMaxForRateRatio(hz, ExpressLRS_currTlmDenom);

    // Notify the sender to adjust its expected throughput
    DataDlSender.UpdateTelemetryRate(hz, ExpressLRS_currTlmDenom, telemetryBurstMax);
}

/* If not connected will rotate through the RF modes looking for sync
 * and blink LED
 */
static void cycleRfMode(unsigned long now)
{
    if (connectionState == connected || connectionState == wifiUpdate || InBindingMode)
        return;

    // Actually cycle the RF mode if not LOCK_ON_FIRST_CONNECTION
    if (LockRFmode == false && (now - RFmodeLastCycled) > (cycleInterval * RFmodeCycleMultiplier))
    {
        RFmodeLastCycled = now;
        LastSyncPacket = now;           // reset this variable
        // Display the current air rate to the user as an indicator something is happening
        SendLinkStatstoFCForcedSends = 2;
        SetRFLinkRate(scanIndex % RATE_MAX, false); // switch between rates
        LQCalc.reset100();
        LQCalcDVDA.reset100();
        scanIndex++;
        Radio.RXnb();
        DBGLN("%u", ExpressLRS_currAirRate_Modparams->interval);

        // Skip unsupported modes for hardware with only a single LR1121 or with a single RF path
        while (!isSupportedRFRate(scanIndex % RATE_MAX))
        {
            DBGLN("Skip %u", get_elrs_airRateConfig(scanIndex % RATE_MAX)->interval);
            scanIndex++;
        }

        // Switch to FAST_SYNC if not already in it (won't be if was just connected)
        RFmodeCycleMultiplier = 1;
    } // if time to switch RF mode
}

static void EnterBindingMode()
{
    if (InBindingMode)
    {
        DBGLN("Already in binding mode");
        return;
    }

    // never enter binding mode if binding is supposed to only be administered through the web UI
    if (config.GetBindStorage() == BINDSTORAGE_ADMINISTERED) {
        return;
    }

    // Any method of entering bind resets a loan
    // Model can be reloaned immediately by binding now
    config.ReturnLoan();
    config.Commit();

    // Binding uses 50Hz, and InvertIQ
    OtaCrcInitializer = OTA_VERSION_ID;
    OtaNonce = 0;
    InBindingMode = true;

    // Start attempting to bind
    // Lock the RF rate and freq while binding
    SetRFLinkRate(enumRatetoIndexSafe(RATE_BINDING), true);

    // If the Radio Params (including InvertIQ) parameter changed, need to restart RX to take effect
    Radio.RXnb();

    DBGLN("Entered binding mode at freq = %d", Radio.currFreq);
    devicesTriggerEvent(EVENT_ENTER_BIND_MODE);
}

static void ExitBindingMode()
{
    if (!InBindingMode)
    {
        DBGLN("Not in binding mode");
        return;
    }

    DataUlReceiver.ResetState();

    // Prevent any new packets from coming in
    Radio.SetTxIdleMode();
    // Write the values to eeprom
    config.Commit();

    OtaUpdateCrcInitFromUid();
    FHSSrandomiseFHSSsequence(OtaGetUidSeed());

    webserverPreventAutoStart = true;

    // Force RF cycling to start at the beginning immediately
    scanIndex = RATE_MAX;
    RFmodeLastCycled = 0;
    LockRFmode = false;
    LostConnection(false);

    // Do this last as LostConnection() will wait for a tock that never comes
    // if we're in binding mode
    InBindingMode = false;
    DBGLN("Exiting binding mode");
    devicesTriggerEvent(EVENT_EXIT_BIND_MODE);
}

static void updateBindingMode(unsigned long now)
{
#if defined(RADIO_LR1121)
    static uint32_t BindingRateChangeMs;
    constexpr uint32_t BindingRateChangeCyclePeriodMs = 125U;
#endif

    // Exit binding mode if the config has been modified, indicating UID has been set
    if (InBindingMode && config.IsModified())
    {
        ExitBindingMode();
    }

#if defined(RADIO_LR1121)
    // Change frequency domains every 500ms.  This will allow single LR1121 receivers to receive bind packets from SX12XX Tx modules.
    else if (InBindingMode && (now - BindingRateChangeMs) > BindingRateChangeCyclePeriodMs)
    {
        BindingRateChangeMs = now;
        if (ExpressLRS_currAirRate_Modparams->enum_rate == RATE_DUALBAND_BINDING)
        {
            SetRFLinkRate(enumRatetoIndexSafe(RATE_BINDING), true);
        }
        else
        {
            SetRFLinkRate(enumRatetoIndexSafe(RATE_DUALBAND_BINDING), true);
        }

        Radio.RXnb();
    }
#endif

    // If the power on counter is >=3, enter binding, the counter will be reset after 2s
    else if (!InBindingMode && config.GetPowerOnCounter() >= 3)
    {
        // Never enter wifi if forced to binding mode
        webserverPreventAutoStart = true;
        DBGLN("Power on counter >=3, enter binding mode");
        EnterBindingMode();
    }

    // If the eeprom is indicating that we're not bound, enter binding
    else if (!OtaUidIsBound(UID) && !InBindingMode)
    {
        DBGLN("RX has not been bound, enter binding mode");
        EnterBindingMode();
    }

    else if (BindingModeRequest)
    {
        DBGLN("Connected request to enter binding mode");
        BindingModeRequest = false;
        if (connectionState == connected)
        {
            LostConnection(false);
            // Skip entering bind mode if on loan. This comes from an OTA request
            // and the model is assumed to be inaccessible, do not want the receiver
            // sitting in a field ready to be bound to anyone within 10km
            if (config.IsOnLoan())
            {
                DBGLN("Model was on loan, becoming inert");
                config.ReturnLoan();
                config.Commit(); // prevents CheckConfigChangePending() re-enabling radio
                Radio.End();
                // Enter a completely invalid state for a receiver, to prevent wifi or radio enabling
                setConnectionState(noCrossfire);
                return;
            }
            // if the InitRate config item was changed by LostConnection
            // save the config before entering bind, as the modified config
            // will immediately boot it out of bind mode
            config.Commit();
        }
        EnterBindingMode();
    }
}

void EnterBindingModeSafely()
{
    // Will not enter Binding mode if in the process of a passthrough update
    // or currently binding
    if (connectionState == serialUpdate || InBindingMode)
        return;

    // Never enter wifi mode after requesting to enter binding mode
    webserverPreventAutoStart = true;

    // If the radio and everything is shut down, better to reboot and boot to binding mode
    if (connectionState == wifiUpdate || connectionState == bleJoystick)
    {
        // Force 3-plug binding mode
        config.SetPowerOnCounter(3);
        config.Commit();
        ESP.restart();
        // Unreachable
    }

    // If connected, handle that in updateBindingMode()
    if (connectionState == connected)
    {
        BindingModeRequest = true;
        return;
    }

    EnterBindingMode();
}

static void checkSendLinkStatsToFc(uint32_t now)
{
    if (now - SendLinkStatstoFCintervalLastSent > SEND_LINK_STATS_TO_FC_INTERVAL)
    {
        if (connectionState == disconnected)
        {
            getRFlinkInfo();
        }

        if ((connectionState != disconnected && connectionHasModelMatch && teamraceHasModelMatch) ||
            SendLinkStatstoFCForcedSends)
        {
            CRSF_MK_FRAME_T(crsfLinkStatistics_t) linkStatisticsFrame;
            crsfRouter.makeLinkStatisticsPacket(&linkStatisticsFrame.h);
            // the linkStats 'originates' from the OTA connector so we don't send it back there.
            crsfRouter.deliverMessage(&otaConnector, &linkStatisticsFrame.h);
            SendLinkStatstoFCintervalLastSent = now;
            if (SendLinkStatstoFCForcedSends)
                --SendLinkStatstoFCForcedSends;
        }
    }
}

static void debugRcvrLinkstats()
{
#if defined(DEBUG_RCVR_LINKSTATS)
    if (debugRcvrLinkstatsPending)
    {
        debugRcvrLinkstatsPending = false;

        // Copy the data out of the ISR-updating bits ASAP
        // While YOLOing (const void *) away the volatile
        crsfLinkStatistics_t ls = *(crsfLinkStatistics_t *)((const void *)&linkStats);
        uint32_t packetCounter = debugRcvrLinkstatsPacketId;
        uint8_t fhss = debugRcvrLinkstatsFhssIdx;
        // actually the previous packet's offset since the update happens in tick, and this will
        // fire right after packet reception (a little before tock)
        int32_t pfd = PfdPrevRawOffset;

        // Use serial instead of DBG() because do not necessarily want all the debug in our logs
        char buf[50];
        snprintf(buf, sizeof(buf), "%u,%u,-%u,%u,%d,%u,%u,%d\r\n",
            packetCounter, ls.active_antenna,
            ls.active_antenna ? ls.uplink_RSSI_2 : ls.uplink_RSSI_1,
            ls.uplink_Link_quality, ls.uplink_SNR,
            ls.uplink_TX_Power, fhss, pfd);
        Serial.write(buf);
    }
#endif
}

static void debugRcvrSignalStats(uint32_t now)
{
#if defined(DEBUG_RCVR_SIGNAL_STATS)
    static uint32_t lastReport = 0;

    // log column header:  cnt1, rssi1, snr1, snr1_max, telem1, fail1, cnt2, rssi2, snr2, snr2_max, telem2, fail2, or, both
    if(now - lastReport >= 1000 && connectionState == connected)
    {
        for (int i = 0 ; i < (isDualRadio()?2:1) ; i++)
        {
            DBG("%d\t%f\t%f\t%f\t%d\t%d\t",
                Radio.rxSignalStats[i].irq_count,
                (Radio.rxSignalStats[i].irq_count==0) ? 0 : double(Radio.rxSignalStats[i].rssi_sum)/Radio.rxSignalStats[i].irq_count,
                (Radio.rxSignalStats[i].irq_count==0) ? 0 : double(Radio.rxSignalStats[i].snr_sum)/Radio.rxSignalStats[i].irq_count/RADIO_SNR_SCALE,
                float(Radio.rxSignalStats[i].snr_max)/RADIO_SNR_SCALE,
                Radio.rxSignalStats[i].telem_count,
                Radio.rxSignalStats[i].fail_count);

                Radio.rxSignalStats[i].irq_count = 0;
                Radio.rxSignalStats[i].snr_sum = 0;
                Radio.rxSignalStats[i].rssi_sum = 0;
                Radio.rxSignalStats[i].snr_max = INT8_MIN;
                Radio.rxSignalStats[i].telem_count = 0;
                Radio.rxSignalStats[i].fail_count = 0;
        }
        if (isDualRadio())
        {
            DBGLN("%d\t%d", Radio.irq_count_or, Radio.irq_count_both);
        }
        else
        {
            DBGLN("");
        }
        Radio.irq_count_or = 0;
        Radio.irq_count_both = 0;

        lastReport = now;
    }
#endif
}

static void updateSwitchMode()
{
    // Negative value means waiting for confirm of the new switch mode while connected
    if (SwitchModePending <= 0)
        return;

    OtaUpdateSerializers((OtaSwitchMode_e)(SwitchModePending - 1), ExpressLRS_currAirRate_Modparams->PayloadLength);
    ChannelDataReset();
    OtaResetChannelDataComplete();
    SwitchModePending = 0;
}

static void CheckConfigChangePending()
{
    if (config.IsModified() && !InBindingMode && connectionState < NO_CONFIG_SAVE_STATES)
    {
        LostConnection(false);
        uint32_t changes = config.Commit();
        devicesTriggerEvent(changes);
        LbtEnableIfRequired();
        Radio.RXnb();
    }
}

#if defined(PLATFORM_ESP8266)
// Called from core's user_rf_pre_init() function (which is called by SDK) before setup()
RF_PRE_INIT()
{
    // Set whether the chip will do RF calibration or not when power up.
    // I believe the Arduino core fakes this (byte 114 of phy_init_data.bin)
    // to be 1, but the TX power calibration can pull over 300mA which can
    // lock up receivers built with a underspeced LDO (such as the EP2 "SDG")
    // Option 2 is just VDD33 measurement
    #if defined(RF_CAL_MODE)
    system_phy_set_powerup_option(RF_CAL_MODE);
    #else
    system_phy_set_powerup_option(2);
    #endif
}
#endif

void resetConfigAndReboot()
{
    config.SetDefaults(true);
    // Prevent WDT from rebooting too early if
    // all this flash write is taking too long
    yield();
    // Remove options.json and hardware.json
    LittleFS.format();
    yield();
    LittleFS.begin();
    options_SetTrueDefaults();

    ESP.restart();
}

void setup()
{
    if (!options_init())
    {
        // In the failure case we set the logging to the null logger so nothing crashes
        // if it decides to log something
        BackpackOrLogStrm = new NullStream();

        // Register the WiFi with the framework
        static device_affinity_t wifi_device[] = {
            {&WIFI_device, 1}
        };
        devicesRegister(wifi_device, ARRAY_SIZE(wifi_device));
        devicesInit();

        setConnectionState(hardwareUndefined);
    }
    else
    {
#if defined(PLATFORM_ESP32)
        // arduino-espressif32 HardwareSerial's constructor for UART0 saves and attaches to GPIO 1 and 3, which
        // will reset any other use of them when begin() is actually called for UART0 by CRSFHandset/SerialIO.
        // Calling end() here, will call _uartDetachPins() on the underlying UART implementation so they won't
        // be saved later (fixed upstream, coming someday)
        Serial.end();
#endif

        // default to CRSF protocol and the compiled baud rate
        serialBaud = firmwareOptions.uart_baud;

        // pre-initialise serial must be done before anything as some libs write
        // to the serial port and they'll block if the buffer fills
        #if defined(DEBUG_LOG)
        Serial.begin(serialBaud);
        BackpackOrLogStrm = &Serial;
        #else
        BackpackOrLogStrm = new NullStream();
        #endif

        // Init EEPROM and load config, checking powerup count
        setupConfigAndPocCheck();
        setupTarget();
        // If serial is not already defined, then see if there is serial pin configured in the PWM configuration
        if (OPT_HAS_SERVO_OUTPUT && GPIO_PIN_RCSIGNAL_RX == UNDEF_PIN && GPIO_PIN_RCSIGNAL_TX == UNDEF_PIN)
        {
            for (int i = 0 ; i < GPIO_PIN_PWM_OUTPUTS_COUNT ; i++)
            {
                eServoOutputMode pinMode = (eServoOutputMode)config.GetPwmChannel(i)->val.mode;
                if (pinMode == somSerial)
                {
                    pwmSerialDefined = true;
                    break;
                }
            }
        }
        crsfRouter.addEndpoint(&crsfReceiver);
        crsfRouter.addConnector(&otaConnector);
        setupSerial();
        setupSerial1();

        devicesRegister(ui_devices, ARRAY_SIZE(ui_devices));
        devicesInit();

        setupBindingFromConfig();

        FHSSrandomiseFHSSsequence(OtaGetUidSeed());

        setupRadio();

        if (connectionState != radioFailed)
        {
#if defined(RADIO_SX128X)
            // Single-radio receivers only: with two radios both chips drive MISO during shared opcodes.
            radioWatchdogDio1Pin = GPIO_PIN_DIO1;
            Radio.watchdogEnabled = !isDualRadio() && radioWatchdogDio1Pin != UNDEF_PIN;
#endif
            // RFnoiseFloor = MeasureNoiseFloor(); //TODO move MeasureNoiseFloor to driver libs
            // DBGLN("RF noise floor: %d dBm", RFnoiseFloor);

            DataUlReceiver.SetDataToReceive(DataUlBuffer, ELRS_DATA_UL_BUFFER);
            Radio.RXnb();
            hwTimer::init(HWtimerCallbackTick, HWtimerCallbackTock);
        }
    }

    registerButtonFunction(ACTION_BIND, EnterBindingModeSafely);
    registerButtonFunction(ACTION_RESET_REBOOT, resetConfigAndReboot);

    devicesStart();

    // setup() eats up some of this time, which can cause the first mode connection to fail.
    // Resetting the time here give the first mode a better chance of connection.
    RFmodeLastCycled = millis();
}

#if defined(RADIO_SX128X)
/**
 * Runs one radio command from the loop with interrupts masked only around the command itself (a few us), never
 * across a busy wait: the wait for the previous command's busy time (BUSY pin, or the fixed allowance on boards
 * without BUSY, up to 1500 us after SetStandby) happens before the lock, and readiness is re-checked inside it in
 * case the radio ISR or the tock sent a command meanwhile. Masking longer than half a packet interval would make the
 * ESP8266 hw timer miss its absolute compare target (~27 s without tick/tock). The lock also keeps the radio ISR from
 * running SPI while the loop is mid-transaction. Returns false (command not sent) if the radio stays busy.
 */
template <typename Command>
static bool radioWatchdogCommand(Command command, uint32_t readyTimeoutUs = RADIO_WATCHDOG_READY_TIMEOUT_US)
{
    const uint32_t start = micros();
    do
    {
        if (Radio.ReadyForCommand())
        {
            RADIO_WATCHDOG_LOCK();
            const bool ready = Radio.ReadyForCommand();
            if (ready)
            {
                command();
            }
            RADIO_WATCHDOG_UNLOCK();
            if (ready)
            {
                return true;
            }
        }
    } while (micros() - start < readyTimeoutUs);
    return false;
}

static bool radioWatchdogInRx()
{
    bool inRx = false;
    radioWatchdogCommand([&inRx]() {
        inRx = (Radio.GetStatusByte(SX12XX_Radio_1) & SX1280Driver::STATUS_MODE_MASK) == SX1280_STATUS_CIRCUIT_MODE_RX;
    });
    return inRx;
}

static bool radioWatchdogRssiRaw(uint8_t &raw)
{
    return radioWatchdogCommand([&raw]() { raw = Radio.GetRssiInstRaw(SX12XX_Radio_1); });
}

/**
 * Bench: in the dead receive path the status byte says Rx but the instantaneous RSSI is pinned at one value.
 * Stops at the first moving sample, and at the first command the radio does not take (one ready timeout, not 16).
 */
static bool radioWatchdogRssiMoves()
{
    uint8_t first = 0;
    if (!radioWatchdogRssiRaw(first))
    {
        return false;
    }
    for (uint8_t sample = 1; sample < RADIO_WATCHDOG_RSSI_SAMPLES; sample++)
    {
        delayMicroseconds(100);
        uint8_t raw = 0;
        if (!radioWatchdogRssiRaw(raw))
        {
            return false;
        }
        if (raw != first)
        {
            return true;
        }
    }
    return false;
}

/**
 * Releases the tock: back to the current FHSS channel if the sequence is live. Only if the radio takes the command
 * (BUSY low / allowance elapsed); returns false otherwise and the tock stays locked out (see radioWatchdogHandback).
 */
static bool radioWatchdogRelease(uint32_t readyTimeoutUs = RADIO_WATCHDOG_READY_TIMEOUT_US)
{
    return radioWatchdogCommand([]() {
        radioWatchdogResetting = false;
        const uint16_t count = FHSSgetSequenceCount();
        const bool retune = radioWatchdogFreqStale || FHSSptr != radioWatchdogTakenFhssPtr;
        radioWatchdogFreqStale = false;
        // Only when needed: a hop write at an arbitrary loop time can abort a packet in flight (stock hops at the tock).
        if (retune && hwTimer::running && connectionState != disconnected && count != 0)
        {
            // Hops were skipped (sequence advanced, no radio write) while the loop owned the radio: retune to the
            // current channel and release in the same critical section, so no tock can advance the sequence in between.
            const uint8_t savedPtr = FHSSptr;
            FHSSptr = (uint8_t)((savedPtr + count - 1) % count);
            const uint32_t freq = FHSSgetNextFreq();  // the current channel; FHSSptr is back at savedPtr
            Radio.SetFrequencyReg(freq, SX12XX_Radio_All, false);
        }
        // else: no hop skipped and no reset, or disconnected (the radio stays on the initial frequency, as stock)
    }, readyTimeoutUs);
}

/**
 * Hands the radio back in the stock receive state: ISR callbacks on + Rx entered (one locked command), then the tock
 * released. Each step only when the radio takes commands; otherwise the hand-back stays pending (tock locked out,
 * callbacks off) and the loop retries it every pass, so neither the tock nor the radio ISR meets a busy radio.
 */
static bool radioWatchdogHandback(uint32_t readyTimeoutUs)
{
    radioWatchdogHandbackPending =
        !(radioWatchdogCommand([]() { Radio.EnableIsrCallbacks(true); Radio.RXnb(); }, readyTimeoutUs) &&
          radioWatchdogRelease(readyTimeoutUs));
    return !radioWatchdogHandbackPending;
}

/** The loop takes the radio from the tock: no hop write / telemetry until the release. */
static void radioWatchdogTake()
{
    radioWatchdogTakenFhssPtr = FHSSptr;
    radioWatchdogResetting = true;
}

/**
 * Receive-path repair without a reset (bench, 12 of 12: after circuit mode 0 the first Rx entry leaves a dead receive
 * path, a second one repairs it): Rx, standby, Rx, verified by status Rx and (requireRssiMoving: a transmitter is on
 * the air) a moving RSSI. While disconnected the band may be quiet, so the slow idle detector judges the RSSI instead.
 * The radio ISR callbacks are off from the standby until the second Rx entry, so an RX ISR cannot spend SetStandby's
 * 1500 us busy allowance spinning in interrupt context (a DIO1 edge dropped meanwhile leaves DIO1 high, which the
 * tock's stuck check repairs).
 */
static bool radioWatchdogRxRepair(bool requireRssiMoving)
{
    radioWatchdogTake();
    const bool sent = radioWatchdogCommand([]() { Radio.RXnb(); }) &&
                      radioWatchdogCommand([]() { Radio.EnableIsrCallbacks(false); Radio.SetStandbyRc(); }) &&
                      radioWatchdogCommand([]() { Radio.RXnb(); Radio.EnableIsrCallbacks(true); });
    if (!sent)
    {
        radioWatchdogHandback(SX1280WatchdogPolicy::HANDBACK_SINGLE_TRY_US);
        return false;
    }
    const bool repaired = radioWatchdogInRx() && (!requireRssiMoving || radioWatchdogRssiMoves());
    radioWatchdogHandbackPending = !radioWatchdogRelease();
    return repaired;
}

/**
 * Radio reset + init for the CURRENT rate and power, keeping timer and FHSS. Returns true if the chip is verified in Rx:
 * configuration read back as written, status Rx, RSSI moving (requireRssiMoving). Reception itself is proven only by a
 * valid packet afterwards (SX1280WatchdogPolicy reception proof). Never uses the stock reset (~110 ms of fixed waits).
 * Boards without a reset line: no End() (it would put the chip to sleep, and only an accidental NSS edge would wake it):
 * Begin's configuration runs again on the running chip (SX1280Hal::fastReset returns at once) and is verified the same way.
 * The radio is handed back in the stock receive state as soon as it takes commands (radioWatchdogHandback); the loop
 * retries failed attempts (SX1280WatchdogPolicy).
 */
static bool radioWatchdogReset(bool requireRssiMoving)
{
    radioWatchdogTake();
    radioWatchdogFreqStale = true;  // Config below tunes the initial frequency
    Radio.EnableIsrCallbacks(false);  // no radio ISR SPI until the radio is configured and back in Rx
    if (GPIO_PIN_RST != UNDEF_PIN)
    {
        Radio.End();
    }
    Radio.currFreq = FHSSgetInitialFreq();
    bool configured = Radio.Begin(FHSSgetMinimumFreq(), FHSSgetMaximumFreq(), true);
    Radio.RXdoneCallback = &RXdoneISR;  // End() replaced them with the null callbacks; restored on every outcome
    Radio.TXdoneCallback = &TXdoneISR;
    if (configured)
    {
        ConfigRadio(ExpressLRS_currAirRate_Modparams, UID[5] & 0x01);
        POWERMGNT::reapplyPower();
        configured = Radio.VerifyConfig();
    }
    // After a failed attempt the hand-back tries each command once: a chip that did not come up gains nothing from the
    // ready wait; if it is not ready, the hand-back stays pending and the loop retries it every pass.
    const uint32_t readyTimeoutUs = configured ? RADIO_WATCHDOG_READY_TIMEOUT_US : SX1280WatchdogPolicy::HANDBACK_SINGLE_TRY_US;
    if (!radioWatchdogCommand([]() { Radio.EnableIsrCallbacks(true); Radio.RXnb(); }, readyTimeoutUs))
    {
        radioWatchdogHandbackPending = true;
        return false;
    }
    // Checked before the release (a telemetry slot could start a TX after it). A moving RSSI as well while a transmitter
    // is on the air: a chip derailed during its start can report Rx without receiving (bench: 'status-only'). While
    // disconnected the band may be quiet (no proof either way); the idle detector watches the RSSI over ~0.5 s instead.
    const bool inRx = configured && radioWatchdogInRx() && (!requireRssiMoving || radioWatchdogRssiMoves());
    radioWatchdogHandbackPending = !radioWatchdogRelease(readyTimeoutUs);
    return inRx;
}

static void radioWatchdogLoop(unsigned long now)
{
    static unsigned long lastActionMs = 0;
    static unsigned long lastRxKickMs = 0;
    static unsigned long dio1HighSinceMs = 0;
    static unsigned long lastDio1SampleMs = 0;
    static unsigned long lastDeadPathCheckMs = 0;
    static bool dio1HighSeen = false;
    static SX1280WatchdogPolicy::RetryState retry;
    static SX1280WatchdogPolicy::ReceptionProof repairProof;  // after a verified Rx repair while connected
    static SX1280WatchdogPolicy::IdleDeadPath idle;
    if (!Radio.watchdogEnabled)
    {
        return;
    }
    if (InBindingMode || connectionState >= MODE_STATES)
    {
        Radio.clearSuspect = false;
        Radio.modeZeroReads = 0;
        radioWatchdogResetRequested = false;
        retry.retryPending = false;
        retry.proof.pending = false;
        repairProof.pending = false;
        SX1280WatchdogPolicy::idleRestart(idle);
        radioWatchdogStage = 0;
        dio1HighSeen = false;
        if (radioWatchdogHandbackPending)
        {
            // Binding / the other modes drive the radio themselves (stock): hand it back now. The tock still skips
            // its commands while BUSY is high (radioWatchdogUnverified stays set until a valid packet).
            radioWatchdogHandbackPending = false;
            Radio.EnableIsrCallbacks(true);
            radioWatchdogResetting = false;
        }
        return;
    }
    // Read once: the radio ISR writes it (a value newer than `now` is handled by the signed compares in the policy).
    const uint32_t lastValidPacket = LastValidPacket;
    // Packets are expected (a transmitter is on the air): the RSSI must move and a verified repair must be followed by
    // a valid packet. Disconnected: the band may be quiet, the idle detector watches the radio instead.
    const bool expectPackets = connectionState == connected && hwTimer::running;
    const uint32_t intervalUs = (uint32_t)ExpressLRS_currAirRate_Modparams->interval;
    SX1280WatchdogPolicy::onPacketEvidence(retry, lastValidPacket);
    SX1280WatchdogPolicy::checkResetProof(retry, expectPackets, lastValidPacket, now, intervalUs);
    if (!expectPackets)
    {
        repairProof.pending = false;
    }
    else if (SX1280WatchdogPolicy::proofFailed(repairProof, lastValidPacket, now, intervalUs))
    {
        radioWatchdogResetRequested = true;  // the repair verified, but nothing is received: escalate to a reset
        DBGLN("radio watchdog: no packet after the Rx repair -> reset");
    }
    radioWatchdogUnverified = SX1280WatchdogPolicy::radioUnverified(retry);
    if (radioWatchdogHandbackPending)
    {
        radioWatchdogHandback(SX1280WatchdogPolicy::HANDBACK_SINGLE_TRY_US);  // one try per loop pass, no waiting
    }
    if (!hwTimer::running)
    {
        // No tocks while disconnected: a flag left set here would keep the receiver deaf, so watch DIO1 from the loop.
        // "Continuously high" needs dense samples: a loop pass gap longer than RADIO_WATCHDOG_DIO1_SAMPLE_GAP_MS (another
        // task blocked the loop) restarts the measurement, so two packet edges far apart cannot look like a stuck line.
        const bool denseSamples = now - lastDio1SampleMs <= RADIO_WATCHDOG_DIO1_SAMPLE_GAP_MS;
        lastDio1SampleMs = now;
        if (digitalRead((uint8_t)radioWatchdogDio1Pin) && denseSamples)
        {
            if (!dio1HighSeen)
            {
                dio1HighSeen = true;
                dio1HighSinceMs = now;
            }
            if (now - dio1HighSinceMs >= RADIO_WATCHDOG_STUCK_DISCONNECTED_MS)
            {
                radioWatchdogResetRequested = true;
            }
        }
        else
        {
            dio1HighSeen = false;
        }
    }
    const bool repairAllowed = !radioWatchdogResetRequested && !radioWatchdogHandbackPending &&
                               now - lastActionMs >= SX1280WatchdogPolicy::ACTION_SPACING_MS;
    const bool idleListening = connectionState == disconnected && !hwTimer::running;
    if (idleListening && repairAllowed && now - lastRxKickMs >= RADIO_WATCHDOG_RX_KICK_MS)
    {
        // Bench: after the stock reconfigure of a chip in circuit mode 0 the receive path is dead (status Rx, RSSI
        // pinned, no packets) and one more Rx entry revives it; with lock-on-first-connection nothing else would ever
        // re-enter Rx while disconnected. Costs at most one missed sync packet per second.
        lastRxKickMs = now;
        radioWatchdogCommand([]() {
            if (!hwTimer::running)  // re-checked inside the lock: a SYNC packet may have restarted the timer meanwhile
            {
                Radio.RXnb();
            }
        });
    }
    if (!idleListening)
    {
        SX1280WatchdogPolicy::idleRestart(idle);
    }
    else if (repairAllowed && !retry.retryPending && SX1280WatchdogPolicy::idleSampleDue(idle, now))
    {
        // Dead receive path while disconnected:the same repair ladder as while connected (double Rx
        // entry, then resets with the normal tiers), driven by the slow flat-RSSI + no-RX_DONE detector (policy).
        uint8_t raw = 0;
        if (radioWatchdogRssiRaw(raw))
        {
            const SX1280WatchdogPolicy::IdleAction action = SX1280WatchdogPolicy::idleOnSample(idle, raw, Radio.rxDoneCount, now);
            if (action == SX1280WatchdogPolicy::IDLE_RX_REPAIR)
            {
                lastActionMs = now;
                const bool repaired = radioWatchdogRxRepair(false);
                radioWatchdogResetRequested = !repaired;
                DBGLN("radio watchdog: idle RSSI pinned -> Rx twice, status Rx %d", repaired);
                return;
            }
            if (action == SX1280WatchdogPolicy::IDLE_RESET)
            {
                radioWatchdogResetRequested = true;
                DBGLN("radio watchdog: idle RSSI still pinned -> reset");
            }
        }
    }
    if (Radio.modeZeroReads >= RADIO_WATCHDOG_MODE_ZERO_READS && repairAllowed)
    {
        lastActionMs = now;
        Radio.modeZeroReads = 0;
        const bool repaired = radioWatchdogRxRepair(expectPackets);
        // A failed repair goes straight to a reset; the reset spacing (not the action spacing) applies to it. A verified
        // one must be followed by a valid packet (repairProof).
        radioWatchdogResetRequested = !repaired;
        if (repaired && expectPackets)
        {
            SX1280WatchdogPolicy::startProof(repairProof, now, millis());
        }
        DBGLN("radio watchdog: mode 0 -> Rx twice, repaired %d", repaired);
        return;
    }
    if (expectPackets && repairAllowed && !retry.retryPending && !retry.proof.pending && !repairProof.pending &&
        SX1280WatchdogPolicy::deadPathCheckDue(now, lastValidPacket, lastDeadPathCheckMs, intervalUs))
    {
        // Fault (c) while connected: no packet for DEAD_PATH_SLOTS intervals AND the RSSI pinned. A fade (RSSI moving)
        // costs this read-only check once per window and nothing else.
        lastDeadPathCheckMs = now;
        if (!radioWatchdogRssiMoves())
        {
            lastActionMs = now;
            const bool repaired = radioWatchdogRxRepair(true);
            radioWatchdogResetRequested = !repaired;
            if (repaired)
            {
                SX1280WatchdogPolicy::startProof(repairProof, now, millis());
            }
            DBGLN("radio watchdog: dead path -> Rx twice, repaired %d", repaired);
            return;
        }
    }
    if (!SX1280WatchdogPolicy::attemptDue(retry, radioWatchdogResetRequested, now))
    {
        return;
    }
    lastActionMs = now;
    radioWatchdogResetRequested = false;
    repairProof.pending = false;
    // Set before the attempt: its release hands the radio to the tock, which must already see "unverified".
    // Stays set until a valid packet (radioUnverified: any attempt counts until then).
    radioWatchdogUnverified = true;
    const bool inRx = radioWatchdogReset(expectPackets);
    Radio.clearSuspect = false;
    Radio.modeZeroReads = 0;
    radioWatchdogStage = 0;
    radioWatchdogDio1HighTocks = 0;
    dio1HighSeen = false;
    SX1280WatchdogPolicy::afterAttempt(retry, inRx, expectPackets, now, millis());
    radioWatchdogUnverified = SX1280WatchdogPolicy::radioUnverified(retry);
    DBGLN("radio watchdog: reset, in Rx %d, attempts %u", inRx, retry.failedResets);
}
#endif

#if defined(PLATFORM_ESP32_C3)
void main_loop()
#else
void loop()
#endif
{
    unsigned long now = millis();
#if defined(RADIO_SX128X)
    radioWatchdogLoop(now);
#endif

    if (DataUlReceiver.HasFinishedData())
    {
        DataUlReceiveComplete();
    }

    devicesUpdate(now);

    // read and process any data from serial ports, send any queued non-RC data
    handleSerialIO();

    checkRebootTime(now);

    CheckConfigChangePending();
    executeDeferredFunction(micros());

    if (connectionState > MODE_STATES)
    {
        return;
    }

    if ((connectionState != disconnected) && (ExpressLRS_currAirRate_Modparams->index != ExpressLRS_nextAirRateIndex)) // forced change
    {
        DBGLN("Req air rate change %u->%u", ExpressLRS_currAirRate_Modparams->index, ExpressLRS_nextAirRateIndex);
        LostConnection(true);
        LastSyncPacket = now;           // reset this variable to stop rf mode switching and add extra time
        RFmodeLastCycled = now;         // reset this variable to stop rf mode switching and add extra time
        SendLinkStatstoFCintervalLastSent = 0;
        SendLinkStatstoFCForcedSends = 2;
    }

    if (connectionState == tentative && (now - LastSyncPacket > ExpressLRS_currAirRate_RFperfParams->RxLockTimeoutMs))
    {
        DBGLN("Bad sync, aborting");
        LostConnection(true);
        RFmodeLastCycled = now;
        LastSyncPacket = now;
    }

    cycleRfMode(now);

    uint32_t localLastValidPacket = LastValidPacket; // Required to prevent race condition due to LastValidPacket getting updated from ISR
    if ((connectionState == connected) && ((int32_t)ExpressLRS_currAirRate_RFperfParams->DisconnectTimeoutMs < (int32_t)(now - localLastValidPacket))) // check if we lost conn.
    {
        LostConnection(true);
    }

    if ((connectionState == tentative) && (abs(LPF_OffsetDx.value()) <= 10) && (LPF_Offset.value() < 100) && (LQCalc.getLQRaw() > minLqForChaos())) //detects when we are connected
    {
        GotConnection(now);
    }

    checkSendLinkStatsToFc(now);

    if ((RXtimerState == tim_tentative) && ((now - GotConnectionMillis) > ConsiderConnGoodMillis) && (abs(LPF_OffsetDx.value()) <= 5))
    {
        RXtimerState = tim_locked;
        DBGLN("Timer locked");
    }

    uint8_t nextPlayloadSize = 0;
    if (!DataDlSender.IsActive() && otaConnector.GetNextPayload(&nextPlayloadSize, DataDlBuffer))
    {
        DataDlSender.SetDataToTransmit(DataDlBuffer, nextPlayloadSize);
    }

    if (config.GetSerialProtocol() == PROTOCOL_MAVLINK && !DataDlSender.IsActive() && ((SerialMavlink *)serialIO)->GetNextPayload(&nextPlayloadSize, DataDlBuffer))
    {
        DataDlSender.SetDataToTransmit(DataDlBuffer, nextPlayloadSize);
    }

    updateTelemetryBurst();
    updateBindingMode(now);
    updateSwitchMode();
    checkGeminiMode();
    DynamicPower_UpdateRx(false);
    debugRcvrLinkstats();
    debugRcvrSignalStats(now);
}

#if defined(PLATFORM_ESP32_C3)
[[noreturn]] void loop()
{
    // We have to do this dodgy hack because on the C3 the Arduino main loop calls
    // a yield function (vTaskDelay) every 2 seconds, which causes us to lose connection!
    extern bool loopTaskWDTEnabled;
    for (;;) {
        if (loopTaskWDTEnabled) {
            esp_task_wdt_reset();
        }
        main_loop();
    }
}
#endif

struct bootloader {
    uint32_t key;
    uint32_t reset_type;
};

void reset_into_bootloader(void)
{
    SERIAL_PROTOCOL_TX.println((const char *)&target_name[4]);
    SERIAL_PROTOCOL_TX.flush();
#if defined(PLATFORM_ESP8266)
    delay(100);
    ESP.rebootIntoUartDownloadMode();
#elif defined(PLATFORM_ESP32)
    delay(100);
    setConnectionState(serialUpdate);
#endif
}
