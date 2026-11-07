#pragma once

// Receiver radio watchdog (SX1280, single radio): timing constants and the pure decisions of the repair loop.
// No hardware access here, so the decisions and the worst-case outage budget can be checked on a host
// (static_asserts below + a host test). Numbers marked (measured) come from bench measurements on an SX1280 with chip
// firmware 0xA9B7 (BetaFPV Nano RX with BUSY, Happymodel EPW6 without BUSY); (assumed) ones are design choices.

#include <stdint.h>

namespace SX1280WatchdogPolicy
{
// --- fast reset (SX1280Hal::fastReset) ---
// NRESET low time. 100 us and 1 ms pulses gave identical start-up timelines (measured); 1 ms keeps margin for a slow
// reset line and costs less than one packet.
constexpr uint32_t FAST_RESET_PULSE_US = 1000;
// After release the status reads STDBY_RC from ~375 us, but BUSY stays high until 1434-1455 us (measured): the chip
// calibrates in STDBY_RC, so the status byte is not a readiness signal.
constexpr uint32_t MEASURED_READY_US = 1455;
// BUSY boards: first BUSY sample this long after release (the chip may not drive BUSY in the first moments) (assumed).
constexpr uint32_t FAST_RESET_BUSY_FIRST_SAMPLE_US = 20;
// BUSY boards: bound of the wait for BUSY low, ~3.4x the measured ready time (assumed margin).
constexpr uint32_t FAST_RESET_BUSY_TIMEOUT_US = 5000;
// Boards without BUSY: wait without any SPI access, ~2x the measured ready time (assumed margin; the ready time of a
// BUSY-less board cannot be measured directly and is taken from the same chip on the BUSY board). The configuration
// is verified by read-back afterwards.
constexpr uint32_t FAST_RESET_NO_BUSY_WAIT_US = 3000;
static_assert(FAST_RESET_PULSE_US >= 100, "NRESET pulse below the measured working minimum");
static_assert(FAST_RESET_NO_BUSY_WAIT_US >= 2 * MEASURED_READY_US, "no-BUSY wait needs 2x margin over the measured ready time");
static_assert(FAST_RESET_BUSY_TIMEOUT_US >= 3 * MEASURED_READY_US, "BUSY timeout needs margin over the measured ready time");

// --- configuration read-back after a fast reset ---
// Everything Begin + Config write that the SX1280 lets us read back (datasheet DS_SX1280-1 rev 3.2: GetPacketType,
// Table 13-1 "List of Registers" and sec. 11.7/14.4), compared under the documented bit masks:
// - 0x0891 bits 7:6 ("Manual Gain Setting"/RxGain; Begin ORs in 0xC0 = high sensitivity, stock comment);
// - packet type (GetPacketType, opcode 0x03);
// - FLRC: CRC seed 0x09C8-0x09C9 ("CRC MSB/LSB Initial Value", written by SetPacketParamsFLRC), sync word 1
//   0x09CF-0x09D2 (SX1280_REG_FLRC_SYNC_WORD, written byte for byte, incl. the datasheet 16.4 byte swap);
// - LoRa: 0x0925 ("SF Additional Configuration", value per SF class from the datasheet), 0x0901 ("Payload Length":
//   datasheet: in implicit-header mode "reading register 0x901 will return the payload size"; compared in the
//   fixed-length mode ELRS uses only), 0x0903 bit 7 ("LoRa Header Mode": 1 = no header = fixed length).
// NOT readable (no Get command, no documented register): modulation params (LoRa BW/CR, FLRC bitrate/CR/BT), the rest
// of the packet params (preamble, CRC on/off, IQ, FLRC sync length/match/whitening), RF frequency, DIO/IRQ mapping and
// masks, buffer base addresses, AutoFS, TX params, regulator mode. The reception proof (PROOF_SLOTS) covers those.
// A moving RSSI only proves a running front end; a chip configured too early or a dropped register write shows up here.
constexpr uint16_t REG_SENSITIVITY = 0x0891;
constexpr uint8_t HIGH_SENSITIVITY_BITS = 0xC0;  // Begin ORs these in (stock: "high sensitivity instead of low power")
constexpr uint16_t REG_LORA_PAYLOAD_LENGTH = 0x0901;
constexpr uint16_t REG_LORA_HEADER_MODE = 0x0903;
constexpr uint8_t LORA_HEADER_MODE_IMPLICIT_BIT = 0x80;
constexpr uint8_t MODEM_REGS_MAX = 6;
struct ConfigImage
{
    uint8_t packetType = 0xFF;                 // packet type written by Config
    uint16_t modemAddress[MODEM_REGS_MAX] = {};  // register of each byte
    uint8_t modemRegs[MODEM_REGS_MAX] = {};      // value written
    uint8_t modemMasks[MODEM_REGS_MAX] = {};     // documented bits of that value
    uint8_t modemRegCount = 0;                 // 0 = nothing written yet -> never matches
};

/** Records one written register byte at index (Config's write order); the count grows to cover it. */
inline void recordModemReg(ConfigImage &image, uint8_t index, uint16_t address, uint8_t value, uint8_t mask)
{
    if (index >= MODEM_REGS_MAX)
    {
        return;
    }
    image.modemAddress[index] = address;
    image.modemRegs[index] = value;
    image.modemMasks[index] = mask;
    image.modemRegCount = (uint8_t)(index + 1);
}

/** True if the values read back from the chip equal what was written (documented bits only). */
inline bool configMatches(const ConfigImage &written, uint8_t sensitivityReg, uint8_t packetType, const uint8_t *modemRegs)
{
    if ((sensitivityReg & HIGH_SENSITIVITY_BITS) != HIGH_SENSITIVITY_BITS || packetType != written.packetType ||
        written.modemRegCount == 0 || written.modemRegCount > MODEM_REGS_MAX)
    {
        return false;
    }
    for (uint8_t index = 0; index < written.modemRegCount; index++)
    {
        const uint8_t mask = written.modemMasks[index];
        if ((modemRegs[index] & mask) != (written.modemRegs[index] & mask))
        {
            return false;
        }
    }
    return true;
}

// --- RSSI-moving check (rx_main radioWatchdogRssiMoves) ---
// Bench: in the dead receive path the status byte says Rx but the instantaneous RSSI is pinned at ONE raw value. The
// check stops at the first sample that differs, so it costs RSSI_CHECK_MAX_US only when the RSSI is flat.
constexpr uint8_t RSSI_SAMPLES = 16;
constexpr uint32_t RSSI_SAMPLE_US = 110;  // 100 us delay + one locked GetRssiInst (~10 us)
constexpr uint32_t RSSI_CHECK_MAX_US = RSSI_SAMPLES * RSSI_SAMPLE_US;

/** Packet slots of intervalUs, rounded UP to whole ms (the loop's clock is millis()). */
constexpr uint32_t slotsToMs(uint32_t slots, uint32_t intervalUs)
{
    return (slots * intervalUs + 999) / 1000;
}

// --- repair loop ---
// Spacing between reset attempts while they keep failing, measured from the END of the previous attempt (an attempt
// lasts up to ~11 ms, so spacing from its start would run attempts back to back). Three tiers, no give-up and no
// permanent failed state:
// - attempts 2..MAX_FAST_RETRIES: 4 ms apart (4-5 ms with millis granularity);
// - attempts up to MAX_MID_RETRIES: 25 ms apart, so a chip that comes back on attempt 4 (1 ms rates), 6 (500 Hz) or
//   8 (333 Hz) still avoids the failsafe on every detection path (static_asserts below). The loop is blocked for one
//   attempt per (attempt + 25 ms): <= ~31 % without BUSY, <= ~25 % with BUSY;
// - after that one attempt per second for the rest of the flight (the radio did not come back after 8 resets).
// Fitting attempts 5-6 before the 1 ms rates' failsafe as well would need ~4 ms spacing for all six, i.e. the loop
// blocked ~70 % for ~85 ms - rejected (V6-REPORT).
constexpr uint32_t RESET_RETRY_SPACING_MS = 4;
constexpr uint8_t MAX_FAST_RETRIES = 3;
constexpr uint32_t RESET_MID_SPACING_MS = 25;
constexpr uint8_t MAX_MID_RETRIES = 8;
constexpr uint32_t RESET_BACKOFF_MS = 1000;
// Spacing after a receive-path repair (unchanged from v5): repeated detections cannot keep the loop busy.
constexpr uint32_t ACTION_SPACING_MS = 20;

/** Minimum time from the END of the last reset attempt until the next one, after failedResets attempts. */
constexpr uint32_t resetSpacingMs(uint8_t failedResets)
{
    return failedResets < MAX_FAST_RETRIES ? RESET_RETRY_SPACING_MS
         : failedResets < MAX_MID_RETRIES  ? RESET_MID_SPACING_MS
                                           : RESET_BACKOFF_MS;
}

/** Attempt count after one more attempt; saturates (never wraps back to the fast spacing). */
inline uint8_t countAfterAttempt(uint8_t failedResets)
{
    return failedResets < 255 ? (uint8_t)(failedResets + 1) : failedResets;
}

/** Times in ms, wrap-safe: a valid packet strictly after the attempt started. */
inline bool packetSinceAttempt(uint32_t lastValidPacketMs, uint32_t lastAttemptMs)
{
    return (int32_t)(lastValidPacketMs - lastAttemptMs) > 0;
}

// --- reception proof (connected, hw timer running) ---
// "Verified in Rx" (read-back + status Rx + moving RSSI) does not prove that the radio receives: modulation and packet
// params (except the LoRa payload length / header mode), frequency, IRQ/DIO mapping, buffer base, AutoFS, TX params
// and the regulator cannot be read back from an SX1280 (V6-REPORT). So after every reset attempt and every
// receive-path repair that verified while connected, a valid packet must follow within PROOF_SLOTS packet intervals;
// if none does, the loop escalates (next reset attempt / a reset after a repair). Normal reception gives a packet in
// the first or second slot after the release (one telemetry slot at most in between); 3 slots also keeps the
// dead-path budget below (static_asserts). Cost of a false escalation (weak link right after a repair): one more
// attempt, paced by the tiers, ended by the first valid packet.
constexpr uint32_t PROOF_SLOTS = 3;

struct ReceptionProof
{
    bool pending = false;
    uint32_t startMs = 0;  // a packet strictly after this counts (the radio was not receiving before)
    uint32_t endMs = 0;    // the window runs from here (end of the attempt / repair)
};

inline void startProof(ReceptionProof &proof, uint32_t startMs, uint32_t endMs)
{
    proof.pending = true;
    proof.startMs = startMs;
    proof.endMs = endMs;
}

/** True once when the window passed without a valid packet; a packet or the expiry ends the proof. */
inline bool proofFailed(ReceptionProof &proof, uint32_t lastValidPacketMs, uint32_t nowMs, uint32_t intervalUs)
{
    if (!proof.pending)
    {
        return false;
    }
    if (packetSinceAttempt(lastValidPacketMs, proof.startMs))
    {
        proof.pending = false;
        return false;
    }
    if (nowMs - proof.endMs >= slotsToMs(PROOF_SLOTS, intervalUs))
    {
        proof.pending = false;
        return true;
    }
    return false;
}

/**
 * Retry bookkeeping of the reset path exactly as the firmware (rx_main radioWatchdogLoop) runs it, kept here so that
 * the host test exercises the code the firmware calls. failedResets counts the attempts since the last valid packet:
 * a verified attempt counts as well until a packet proves reception (a "verified" chip may still not receive).
 */
struct RetryState
{
    uint8_t failedResets = 0;
    bool retryPending = false;  // a failed attempt asks for the next one (kept apart from new detections)
    uint32_t attemptStartMs = 0;
    uint32_t attemptEndMs = 0;
    ReceptionProof proof;       // after a verified attempt while connected
};

/**
 * A valid packet received after the last attempt started: the radio receives again, whatever repaired it (a later
 * attempt, the stock link-loss reconfigure, the Rx kick) - or the attempt's verification failed on a healthy chip.
 * Either way the count goes back to 0 (else a stale count sends a later, unrelated fault straight into a slow tier)
 * and a pending retry is dropped (else a healthy, receiving chip would be reset again and again).
 */
inline void onPacketEvidence(RetryState &state, uint32_t lastValidPacketMs)
{
    if (packetSinceAttempt(lastValidPacketMs, state.attemptStartMs))
    {
        state.failedResets = 0;
        state.retryPending = false;
        state.proof.pending = false;
    }
}

/** Whether a reset attempt runs now: a new request or a pending retry, and the spacing since the last attempt's END. */
inline bool attemptDue(const RetryState &state, bool requested, uint32_t nowMs)
{
    return (requested || state.retryPending) && nowMs - state.attemptEndMs >= resetSpacingMs(state.failedResets);
}

/**
 * Bookkeeping after an attempt. Not verified: the next attempt is requested at once (tier spacing). Verified: while
 * packets are expected (connected, hw timer running) a valid packet must follow (checkResetProof); otherwise
 * (disconnected) the detectors that requested the attempt watch the radio again.
 */
inline void afterAttempt(RetryState &state, bool verified, bool expectPackets, uint32_t startMs, uint32_t endMs)
{
    state.attemptStartMs = startMs;
    state.attemptEndMs = endMs;
    state.failedResets = countAfterAttempt(state.failedResets);
    state.retryPending = !verified;
    state.proof.pending = false;
    if (verified && expectPackets)
    {
        startProof(state.proof, startMs, endMs);
    }
}

/** Verified attempt but no valid packet within PROOF_SLOTS: request the next attempt (tier spacing applies). */
inline void checkResetProof(RetryState &state, bool expectPackets, uint32_t lastValidPacketMs, uint32_t nowMs, uint32_t intervalUs)
{
    if (!expectPackets)
    {
        state.proof.pending = false;  // link lost meanwhile: the disconnected detectors take over
        return;
    }
    if (proofFailed(state.proof, lastValidPacketMs, nowMs, intervalUs))
    {
        state.retryPending = true;
    }
}

/**
 * After an attempt until packet evidence: the tock skips its radio commands while BUSY is high (rx_main). The count
 * alone decides (it is >= 1 after any attempt until a valid packet arrives).
 */
inline bool radioUnverified(const RetryState &state)
{
    return state.failedResets != 0;
}

// --- dead receive path detector, connected (hw timer running) ---
// Fault (c): status Rx, instantaneous RSSI pinned at one value, nothing received (bench). The loop runs the RSSI check
// when no valid packet arrived for DEAD_PATH_SLOTS packet intervals, and again at most once per DEAD_PATH_SLOTS
// intervals while that lasts; only a flat RSSI (16 samples) leads to a repair, so an ordinary fade (RSSI moving: the TX
// is on the air) costs one read-only check of ~0.1-0.2 ms per window and nothing else. 5 slots: the longest normal gap
// is one telemetry slot (at most every 2nd slot), and detection + check + repair + proof must leave room for 4 reset
// attempts before the 100-packet failsafe at the 1 ms rates (static_asserts).
constexpr uint32_t DEAD_PATH_SLOTS = 5;

/**
 * Whether the loop runs the RSSI check now. intervalUs = packet interval of the current rate. The ISR writes
 * lastValidPacketMs and may do so after the loop sampled nowMs: a packet "in the future" counts as just received
 * (signed difference, as the stock link-loss check next to it), never as ~49 days ago.
 */
inline bool deadPathCheckDue(uint32_t nowMs, uint32_t lastValidPacketMs, uint32_t lastCheckMs, uint32_t intervalUs)
{
    const int32_t windowMs = (int32_t)slotsToMs(DEAD_PATH_SLOTS, intervalUs);
    return (int32_t)(nowMs - lastValidPacketMs) >= windowMs && (int32_t)(nowMs - lastCheckMs) >= windowMs;
}

// --- dead receive path detector, disconnected (hw timer stopped: no tock, no packets expected) ---
// Without a transmitter the band can be quiet, so "no packets" proves nothing and the 16-sample flat check is not safe
// on a healthy chip. The loop samples the raw RSSI once per IDLE_SAMPLE_MS and counts the radio's RX_DONE interrupts
// (any packet, valid or not, proves a receiving path). The path counts as dead only if IDLE_FLAT_SAMPLES consecutive
// samples (~0.5 s) are bit-identical AND no RX_DONE happened: the bench dead path reads ONE raw value exactly
// (202 or 242, regardless of TX), while a running front end reports a fresh noise-power estimate at each sample, 20 ms
// apart (far longer than the chip's RSSI averaging), in 0.5 dB steps. Whether a healthy chip in a very quiet band can
// read 25 identical values in a row is UNMEASURED; if it does, the cost is bounded: one double Rx entry
// (~2 ms), then at most one reset per window (~11 ms) while no transmitter is on the channel, never while connected.
// First detection -> the receive-path repair (double Rx entry, the measured cure); still flat a window later -> reset
// requests with the normal tiers. Any moving sample or RX_DONE returns to the first step.
constexpr uint32_t IDLE_SAMPLE_MS = 20;
constexpr uint8_t IDLE_FLAT_SAMPLES = 25;

enum IdleAction : uint8_t
{
    IDLE_NONE = 0,
    IDLE_RX_REPAIR,
    IDLE_RESET,
};

struct IdleDeadPath
{
    bool haveSample = false;
    uint8_t lastRaw = 0;
    uint8_t lastRxDoneCount = 0;
    uint8_t identicalRun = 0;   // consecutive identical samples, incl. the first one
    bool repairTried = false;   // the double Rx entry ran and the path stayed flat since
    uint32_t lastSampleMs = 0;
};

/** Forget the run and the escalation (connected, hw timer running, binding): the next sample starts over. */
inline void idleRestart(IdleDeadPath &state)
{
    state.haveSample = false;
    state.identicalRun = 0;
    state.repairTried = false;
}

inline bool idleSampleDue(const IdleDeadPath &state, uint32_t nowMs)
{
    return !state.haveSample || nowMs - state.lastSampleMs >= IDLE_SAMPLE_MS;
}

/** One RSSI sample (raw) with the RX_DONE count at that moment; returns what the loop does now. */
inline IdleAction idleOnSample(IdleDeadPath &state, uint8_t raw, uint8_t rxDoneCount, uint32_t nowMs)
{
    state.lastSampleMs = nowMs;
    if (!state.haveSample)
    {
        state.haveSample = true;
        state.lastRaw = raw;
        state.lastRxDoneCount = rxDoneCount;
        state.identicalRun = 1;
        return IDLE_NONE;
    }
    if (raw != state.lastRaw || rxDoneCount != state.lastRxDoneCount)
    {
        // The receive path is alive: start over, and the next detection begins with the repair again.
        state.lastRaw = raw;
        state.lastRxDoneCount = rxDoneCount;
        state.identicalRun = 1;
        state.repairTried = false;
        return IDLE_NONE;
    }
    state.identicalRun++;
    if (state.identicalRun < IDLE_FLAT_SAMPLES)
    {
        return IDLE_NONE;
    }
    // A whole new window before the next action; the last value stays for comparison, so a sample that moves right
    // after the action still counts as "alive".
    state.identicalRun = 0;
    if (!state.repairTried)
    {
        state.repairTried = true;
        return IDLE_RX_REPAIR;
    }
    return IDLE_RESET;
}

// --- worst-case outage budget (failsafe = 100 missed packets) ---
// Per-attempt bound, boards WITHOUT BUSY, every term from the code: End (SetSleep) 100, pulse, no-BUSY wait, Begin
// (SetStandby allowance 1500 + firmware/0x0891 reads and writes ~400), Config (SetStandby 1500 + packet type 20 +
// modulation 25..110 + packet params 30 + registers ~200 + frequency/IRQ/FIFO ~150), read-back (<= 8 reads) ~350, Rx entry ~200,
// status ~100, RSSI check, release ~200.
constexpr uint32_t ATTEMPT_MAX_US_NO_BUSY = 100 + FAST_RESET_PULSE_US + FAST_RESET_NO_BUSY_WAIT_US + 1900 + 2000 + 350 +
                                            200 + 100 + RSSI_CHECK_MAX_US + 200;
// Ready wait of a normal watchdog command (v5 RADIO_WATCHDOG_READY_TIMEOUT_US). After a FAILED attempt the hand-back
// (Rx entry, release) tries once without this wait; if the radio is not ready the hand-back stays pending and the loop
// retries it every pass (rx_main), so the tock is never handed a radio that is still busy.
constexpr uint32_t COMMAND_READY_TIMEOUT_US = 3000;
constexpr uint32_t HANDBACK_SINGLE_TRY_US = 100;
// Boards WITH BUSY, worst case = BUSY never falls: End (SetSleep, stock 1 ms WaitOnBusy) + pulse + first sample + BUSY
// timeout + one single-try hand-back command; read-back/status/RSSI are skipped because Begin failed.
// Not covered: BUSY falling and then sticking during Begin/Config (each stock command waits up to 1 ms; V6-REPORT).
constexpr uint32_t ATTEMPT_MAX_US_BUSY = 100 + 1000 + FAST_RESET_PULSE_US + FAST_RESET_BUSY_FIRST_SAMPLE_US +
                                         FAST_RESET_BUSY_TIMEOUT_US + HANDBACK_SINGLE_TRY_US;
// Loop pick-up of a request: the loop is not bounded by this code; assumed <= 1 ms per pass, one pass per action.
constexpr uint32_t LOOP_PICKUP_US = 1000;
// millis() granularity: a spacing of S ms can last up to S + 1 ms.
constexpr uint32_t MILLIS_GRANULARITY_US = 1000;
constexpr uint32_t ATTEMPT_MAX_US = ATTEMPT_MAX_US_NO_BUSY > ATTEMPT_MAX_US_BUSY ? ATTEMPT_MAX_US_NO_BUSY : ATTEMPT_MAX_US_BUSY;
// Receive-path repair (rx_main radioWatchdogRxRepair), boards without BUSY: Rx (allowance 100), standby (allowance
// 1500), Rx (100), command overheads ~150, status read, RSSI check, release ~200. Failed = flat RSSI (full check);
// verified = RSSI moves at the 2nd sample, then the reception proof window.
constexpr uint32_t RX_REPAIR_FAILED_MAX_US = 100 + 1500 + 100 + 150 + RSSI_CHECK_MAX_US + 200;
constexpr uint32_t RX_REPAIR_VERIFIED_MAX_US = 100 + 1500 + 100 + 150 + 2 * RSSI_SAMPLE_US + 200;

constexpr uint32_t maxUs(uint32_t first, uint32_t second)
{
    return first > second ? first : second;
}

/** Reception proof window incl. millis granularity. */
constexpr uint32_t proofWindowUs(uint32_t intervalUs)
{
    return slotsToMs(PROOF_SLOTS, intervalUs) * 1000 + MILLIS_GRANULARITY_US;
}

/** Repair until the reset request: failed at once, or verified and then no packet within the proof window. */
constexpr uint32_t repairToResetUs(uint32_t intervalUs)
{
    return maxUs(RX_REPAIR_FAILED_MAX_US, RX_REPAIR_VERIFIED_MAX_US + proofWindowUs(intervalUs));
}

// Detection, from the fault to the reset request, per path (fault onset with no watchdog action in the preceding
// ACTION_SPACING_MS; a repair right after another action waits up to that spacing - not in the budget).
/** (a) ignored clear: suspect due 1 ms after the clear (tock ~0.2 ms after RX done) + repeat clear + verify + request. */
constexpr uint32_t detectionClearUs(uint32_t intervalUs)
{
    return 4 * intervalUs;
}
/** (b) circuit mode 0: two rejected RX_DONEs, loop pick-up, Rx repair that fails or does not prove reception. */
constexpr uint32_t detectionModeZeroUs(uint32_t intervalUs)
{
    return 2 * intervalUs + LOOP_PICKUP_US + repairToResetUs(intervalUs);
}
/** (c) dead receive path while connected: window without a valid packet, loop pick-up, flat RSSI check, repair. */
constexpr uint32_t detectionDeadPathUs(uint32_t intervalUs)
{
    return slotsToMs(DEAD_PATH_SLOTS, intervalUs) * 1000 + MILLIS_GRANULARITY_US + LOOP_PICKUP_US + RSSI_CHECK_MAX_US +
           repairToResetUs(intervalUs);
}

/**
 * Sum of the gaps before attempts 2..attempts: the tier spacing after k-1 attempts, or the reception proof window if
 * attempt k-1 verified without receiving (whichever is longer), + millis granularity.
 */
constexpr uint32_t spacingBeforeUs(uint8_t attempts, uint32_t intervalUs)
{
    return attempts <= 1 ? 0
                         : spacingBeforeUs((uint8_t)(attempts - 1), intervalUs) +
                               maxUs(resetSpacingMs((uint8_t)(attempts - 1)) * 1000,
                                     slotsToMs(PROOF_SLOTS, intervalUs) * 1000) +
                               MILLIS_GRANULARITY_US;
}

/** Worst-case time from the fault until the first packet after the n-th attempt (one interval after its end). */
constexpr uint32_t outageUs(uint32_t detectionUs, uint8_t attempts, uint32_t intervalUs)
{
    return detectionUs + attempts * (LOOP_PICKUP_US + ATTEMPT_MAX_US) + spacingBeforeUs(attempts, intervalUs) + intervalUs;
}

constexpr uint32_t failsafeUs(uint32_t intervalUs)
{
    return 100 * intervalUs;
}

/** A chip that comes back on attempt `attempts` receives again before the failsafe, on every detection path. */
constexpr bool allPathsRecover(uint8_t attempts, uint32_t intervalUs)
{
    return outageUs(detectionClearUs(intervalUs), attempts, intervalUs) < failsafeUs(intervalUs) &&
           outageUs(detectionModeZeroUs(intervalUs), attempts, intervalUs) < failsafeUs(intervalUs) &&
           outageUs(detectionDeadPathUs(intervalUs), attempts, intervalUs) < failsafeUs(intervalUs);
}

// Packet intervals of the fastest SX1280 rates (F1000, D500, D250), 500 Hz and 333 Hz.
constexpr uint32_t INTERVAL_US_1MS_RATES = 1000;
constexpr uint32_t INTERVAL_US_500HZ = 2000;
constexpr uint32_t INTERVAL_US_333HZ = 3000;
// Attempts that must fit before the failsafe on EVERY path (ignored clear, mode 0, dead receive path).
constexpr uint8_t RECOVERY_ATTEMPTS_1MS_RATES = 4;
constexpr uint8_t RECOVERY_ATTEMPTS_500HZ = 6;
constexpr uint8_t RECOVERY_ATTEMPTS_333HZ = 8;
static_assert(allPathsRecover(RECOVERY_ATTEMPTS_1MS_RATES, INTERVAL_US_1MS_RATES), "attempt 4 must receive before the 1 ms rates' failsafe on every path");
static_assert(allPathsRecover(RECOVERY_ATTEMPTS_500HZ, INTERVAL_US_500HZ), "attempt 6 must receive before the 500 Hz failsafe on every path");
static_assert(allPathsRecover(RECOVERY_ATTEMPTS_333HZ, INTERVAL_US_333HZ), "attempt 8 must receive before the 333 Hz failsafe on every path");
static_assert(MAX_MID_RETRIES >= RECOVERY_ATTEMPTS_333HZ, "the asserted attempts must not fall into the 1 s back-off");
// Design target at the 1 ms rates: half the failsafe budget. It holds for the ignored-clear path: all fast attempts done
// within 50 packets. Mode 0 and the dead path spend their repair (+ proof) first and do not meet it.
constexpr uint32_t OUTAGE_LIMIT_US_1MS_RATES = 50000;
static_assert(outageUs(detectionClearUs(INTERVAL_US_1MS_RATES), MAX_FAST_RETRIES, INTERVAL_US_1MS_RATES) < OUTAGE_LIMIT_US_1MS_RATES,
              "ignored clear: all fast attempts must end inside half the failsafe budget at the 1 ms rates");
} // namespace SX1280WatchdogPolicy
