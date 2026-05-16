# Non-Blocking TX + IRQ-Robustness Watchdog — DRAFT

**Date:** 2026-05-12
**Status:** DRAFT — pending field validation (multi-day soak with 10-device broadcast scenario).
**Scope:** Single file changed in the WLED node firmware: [usermods/racelink_wled/racelink_transport_core.h](../racelink_transport_core.h) plus a one-line rename in [usermods/racelink_wled/racelink_wled.cpp](../racelink_wled.cpp).
**Wire protocol:** unchanged — `racelink_proto.h` was NOT touched. Node ↔ Gateway over-the-air format is identical to the previous release.

> If, after the soak period, no regressions appear, promote this file from DRAFT to a permanent CHANGELOG / release-note. If issues arise, this file is the recovery anchor: it captures *what* was changed and *why*, so a future revert or re-design has full context.

---

## TL;DR

The TX path was changed from RadioLib's blocking `transmit()` to non-blocking `setTx()` + DIO1 IRQ. Three classes of work landed in one wave:

1. **CAD → on-air latency reduction** (the original ask): P1 (Idle→Tx fall-through), P2 (skip redundant standby), P3 (pre-load TX FIFO before CAD + non-blocking setTx). Expected savings: roughly **~300 µs** in the CAD-Done → first-bit-on-air window, plus the main loop is no longer blocked for the full Time-on-Air during TX.
2. **Deadlock fix** for an issue exposed by P3: 2 of 10 devices in a broadcast burst test would stop responding until power-cycle. Root cause was IRQ-vs-state desync (the handler trusted `rfMode` instead of reading actual IRQ flags), with no recovery if an expected IRQ was missed. Fix: read SX126x IRQ flags explicitly, dispatch on bit values, and add a TX watchdog with explicit recovery to `Idle`.
3. **Post-review hardening pass (I1–I6)**: covers a too-tight watchdog timeout, a second watchdog for "stuck in Tx without ever firing setTx", error-path tightening in `fireArmedTx`, less destructive handling of stale TX_DONE/RX_DONE, stream abort on recovery, and a defensive `txArmed` reset in `scheduleSend`.

Build verified on all three RaceLink node profiles:
- `RaceLink_Node_v4_s3_llcc68` — ESP32-S3 + LLCC68
- `RaceLink_Node_v3_s2_llcc68` — ESP32-S2 + LLCC68
- `RaceLink_Node_v1_c3_ct62` — ESP32-C3 + SX1262

Field observation after the patch: a 10-device broadcast-burst run, which previously deadlocked 2/10 units, completed cleanly with zero deadlocks.

---

## Change Inventory

### Layer 1 — CAD → TX latency reduction (P1 / P2 / P3)

| Tag | What | Where |
|---|---|---|
| **P1** | When the Idle block detects `txPending`, fall through to the Tx block in the SAME `service()` call instead of returning. Saves up to one service tick (~13 ms at 75 FPS) when jitter is short. | Idle block in `service()` |
| **P2** | Track `Core::radioStandby` so the explicit `radio->standby()` before each TX is skipped when the chip is already there. Set/cleared at every standby/setTx/startReceive/scanChannel touchpoint. Saves one SPI round-trip per Tx mode entry. | New `Core` field + 5 touch sites |
| **P3** | Pre-load the chip's TX FIFO with `setPacketParams` + `setBufferBaseAddress` + `writeBuffer` BEFORE the CAD scan (during the LBT jitter wait). After CAD-free, only `clearIrqStatus` + `setDioIrqParams` + `setTx` are needed (~30–60 µs SPI). Plus: `setTx` is non-blocking, so the WLED main loop is no longer stalled for the ~51 ms (or more for larger packets) of on-air time. | New helpers `prepareTxBuffer()` and `fireArmedTx()`, plus a TX-Done handler in the IRQ block of `service()` |

`RADIOLIB_LOW_LEVEL=1` is defined right before `#include <RadioLib.h>` in `racelink_transport_core.h` so that the SX126x low-level command set (`setBufferBaseAddress`, `writeBuffer`, `setTx`, `setDioIrqParams`, `clearIrqStatus`, `setPacketParams`) is exposed as public. Since this header is the only place RadioLib is included in the WLED node, the define is contained to this translation unit chain.

### Layer 2 — IRQ-robustness fix (deadlock root cause)

After P3 shipped, 2 of 10 devices in a broadcast-burst test deadlocked. Root cause: the DIO1 IRQ handler assumed it knew what the IRQ meant based on `rfMode` (`if (rfMode==Tx) treat as TX_DONE`). Under non-blocking TX, that assumption can fail when:

- An expected `TX_DONE` IRQ is lost → state stays at `rfMode=Tx`, `radioStandby=false`, and `service()` returns immediately every tick → no RX possible → silent deadlock.
- An unexpected IRQ (residual CAD_DETECTED, spurious TIMEOUT) arrives → was being silently treated as `TX_DONE` → state-machine drift.

Reference for the fix shape: ExpressLRS PR #2055 (LBT-mode hang) and the Lora-net SX126x driver — both read flag bits and run a timeout watchdog.

What landed:

| Item | What | Where |
|---|---|---|
| Flag-based IRQ dispatch | Read `radio->getIrqFlags()`, immediately `clearIrqStatus(ALL)`, then branch on `TX_DONE` / `RX_DONE` / `TIMEOUT` / `HEADER_ERR|CRC_ERR`. Plausibility checks against `rfMode`. | `service()` IRQ block |
| TX in-flight watchdog | `txStartedAtMs` set on successful `setTx`. If `now - txStartedAtMs > ToA(maxPkt) + 100 ms`, force `recoverToIdle`. Catches lost TX_DONE IRQs. | Tx block, new `Core::txStartedAtMs` |
| `recoverToIdle()` helper | Standby + `clearIrqStatus(ALL)` + reset of Tx/Rx state + `rfMode=Idle`. `reqRxKind` is left intact, so the next `service()` iteration auto-restarts RX on Slaves. | New inline function |
| Debug-code sentinel | On recovery the trigger reason is stored in `Core::debug` (0xA1 = stale TX_DONE in non-Tx mode; 0xA2 = stale RX_DONE in non-Rx mode; 0xA3 = TIMEOUT; 0xB0 = in-flight watchdog; 0xB1 = submission watchdog). Visible via WebUI/Info. | `recoverToIdle()` and call sites |
| Telemetry counters | `Core::txWatchdogCount` and `Core::rxErrCount` for monitoring. | New `Core` fields |

### Layer 3 — Post-review hardening (I1–I6)

Re-review of the IRQ-fix surface found six issues, each fixed:

| # | Issue | Fix |
|---|---|---|
| **I1** | Watchdog was computed from ToA of a 16-byte packet (~151 ms total). For a max-size 64-byte packet, ToA is ~150 ms — watchdog could fire during a legitimate large TX. | `Core::toaUsMaxPkt` (renamed from `toaUsMax17`) is now `radio.getTimeOnAir(sizeof(rl.txBuf))`. Watchdog ≈ 250 ms. |
| **I2** | If `prepareTxBuffer` keeps failing (persistent SPI fault, chip wedged in a bad state), the Tx block loops forever because the in-flight watchdog only triggers when `radioStandby==false`. This is a new silent-deadlock path. | New "submission watchdog" anchored at `scheduleSend` (`Core::txSubmittedAtMs`). After 5 s stuck in Tx without firing setTx, `recoverToIdle(0xB1)`. 5 s tolerates heavily congested channels (CAD-busy backoffs of 100–200 ms allow >25 retries) but catches genuine wedges. |
| **I3** | `fireArmedTx` flipped `radioStandby=false` and armed the watchdog even when `setTx()` returned an error. Costs ~250 ms of unnecessary watchdog wait before recovery. | Only flip in-flight state on `setTx` success. On error, the radio is presumed to still be in standby; caller retries via CAD. |
| **I4** | When the watchdog had already recovered a stuck TX and the actual TX_DONE IRQ finally arrived (late), the handler called `recoverToIdle(0xA1)` again — which would cancel any freshly-scheduled TX in the meantime. | On stale TX_DONE / RX_DONE: just `standby()` + clear flags + log debug code. Do NOT touch `rfMode` or `txPending` — the new operation is left alone. |
| **I5** | `recoverToIdle` cleared `txPending` but left `streamMode==Tx` / `streamActive==true`. Next service iteration would `queueNextStreamPacket()` and skip a fragment, causing the receiver's `handleStreamPacket()` to reject the entire stream with `packets_left` mismatch. Sender side wouldn't notice. | `recoverToIdle` now also clears all stream-TX state. |
| **I6** | `scheduleSend` did not defensively reset `txArmed`. It can't actually be `true` here (TX-Done resets it, recovery resets it, init starts `false`, and a re-entry is rejected via `txPending`). Defense-in-depth only. | One-line `rl.txArmed = false;` next to the `memcpy`. |

### Layer 4 — Defensive-code cleanup (2026-05-13)

Self-review identified two defensive constructs that protect against scenarios which cannot occur in the current state machine and which therefore only added SPI cost without measurable benefit. Both removed:

| Removed | Where | Why it was redundant |
|---|---|---|
| `radio->setBufferBaseAddress(0x00, 0x80)` (TX@0, RX@0x80 split) | `prepareTxBuffer` | Between `prepareTxBuffer` and `fireArmedTx` the chip is always in Standby or CAD — never RX. So a "stray RX overwriting the pre-loaded FIFO" cannot occur. Additionally, RadioLib's `startReceive()` resets both base addresses back to `(0, 0)` on every RX entry ([SX126x.cpp:566](.pio/libdeps/RaceLink_Node_v3_s2_llcc68/RadioLib/src/modules/SX126x/SX126x.cpp#L566)), so the split was undone between every TX cycle anyway — it never actually persisted as a defensive layer. Saves one SPI round-trip (~10–20 µs) per TX. ExpressLRS uses the split, but only because they bypass RadioLib entirely and their RX path does not reset the base; with RadioLib in the loop, the split is unmaintainable as a persistent invariant. |
| `radio->standby()` after a confirmed `TX_DONE` | TX_DONE branch in the IRQ handler | SX1262 datasheet §13.1.4 guarantees STDBY_RC immediately after TX_DONE. The explicit call was self-described "defense-in-depth for the rare case the chip is somewhere else" — but if the datasheet contract were ever violated, the next Tx-block entry's `if (!rl.radioStandby)` guard would no longer matter and the in-flight watchdog would catch the resulting stall anyway. Saves one SPI round-trip (~50–100 µs) in the post-TX cleanup path. |
| Unconditional `radio->standby()` in the Idle-confirm branch (Idle block) | Same P2 pattern as the Tx-block standby guard | This branch fires on every `changeMode` transition into Idle — including the very common post-TX-Done path where `radioStandby` is already `true`. Now wrapped in `if (!rl.radioStandby)` for consistency with the Tx-block. Saves one SPI round-trip (~50–100 µs) per Idle re-confirmation, most notably after every successful TX cycle. |

Builds remained green on all three node profiles after the cleanup.

### Layer 5 — Documentation language

All comments in `racelink_transport_core.h` are now in English — both the ones I added during P1/P2/P3/IRQ-fix and any older German comments that pre-existed in the file. This aligns with project policy (English-only docs/comments).

---

## Gateway Compatibility Assessment

`RaceLink_Gateway` is a separate firmware repository that reuses `racelink_transport_core.h` and `racelink_proto.h`. I cannot directly verify the Gateway codebase from this workspace, so the analysis below is *what the Gateway author needs to check*, not "verified working".

### Wire protocol: SAFE

I did not touch [racelink_proto.h](../racelink_proto.h) in this session. The pre-existing diff against HEAD on that file is unrelated to today's work (older session). Header struct layout, packet types, sync word, frequency, SF/BW/CR are all identical. **Node and Gateway will continue to interoperate at the over-the-air level.**

### Public function signatures: UNCHANGED

These remain identical in signature and high-level contract:

- `beginCommon(radio, rl, cfg)`
- `attachDio1(radio, rl)`
- `service(rl, cb)`
- `scheduleSend(rl, buf, len, jitterMaxMs)`, `scheduleSendThenRxWindow(...)`, `buildAndSchedule<T>(...)`, `buildEmptyAndSchedule(...)`
- `scheduleStreamSend(...)`, `queueNextStreamPacket(...)`, `handleStreamPacket(...)`, `streamBuffer(...)`, `clearStreamReady(...)`
- `setDefaultIdle(...)`, `setDefaultRxContinuous(...)`, `requestRxTimed(...)`, `requestRxContinuous(...)`, `cancelRxRequest(...)`
- `Callbacks` struct shape
- All address helpers (`readEfuseMac6`, `last3FromMac6`, `mac6ToStr`, `same3`, `isBroadcast3`, `receiverMatches`)
- `lbtBackoffMaxMs(rl)` (still public, computes from the renamed field but the semantics are stronger now: returns ms based on max-packet ToA, not 16-byte ToA — a slightly larger value)

### `Core` struct changes — what to check

**Renamed field (POTENTIALLY BREAKING):**

- `Core::toaUsMax17` → `Core::toaUsMaxPkt`. The value also grew: was `getTimeOnAir(16)` (~51 ms), now `getTimeOnAir(sizeof(txBuf))` = `getTimeOnAir(64)` (~150 ms).

  **Action for Gateway author:** grep for `toaUsMax17` in the Gateway. If found, rename to `toaUsMaxPkt`. If the Gateway has any logic that assumes this is a *small* number (e.g., uses it as a tight timeout), revisit — the new value is ~3× larger.

**Added fields (ADDITIVE, non-breaking for compile):**

```cpp
bool      radioStandby;     // P2 tracking
bool      txArmed;          // P3 FIFO armed flag
uint16_t  txPreamble;       // cached from PhyCfg
uint8_t   txCrcType;        // cached from PhyCfg
uint32_t  txStartedAtMs;    // IRQ-robustness: in-flight watchdog anchor
uint32_t  txSubmittedAtMs;  // I2: submission watchdog anchor
uint16_t  txWatchdogCount;  // recovery telemetry
uint16_t  rxErrCount;       // HEADER_ERR/CRC_ERR telemetry
```

Gateway compiles without any change, *unless* it serializes or sizes-asserts `Core`. Struct size grew by ~20 bytes total.

**Removed fields:** none.

### New inline functions — ADDITIVE only

- `recoverToIdle(rl, debugCode)`
- `prepareTxBuffer(rl)`
- `fireArmedTx(rl)`

Gateway sees them but does not have to use them.

### Behavioral changes Gateway must be aware of

1. **`service()` no longer blocks for the duration of a TX.** Previously, a `transmit()` call inside `service()` would block the main loop for ~51 ms (SF7/17B) or more. Now `service()` returns immediately after `setTx`; the TX completes asynchronously via DIO1.

   - *Impact if Gateway loop has timing assumptions:* the main loop now finishes each iteration much faster during heavy TX traffic. WLED rendering / LED updates / OPC handling on the Node side get more loop time — generally a *win*. On the Gateway side, any logic that relied on `service()` being a natural pacing mechanism may need a sleep/yield elsewhere.
   - *Impact on `cb.onTxDone` timing:* still fires once per successful TX. The wall-clock latency between `scheduleSend` returning and `onTxDone` firing is roughly the same as before (jitter + CAD + ToA), but it's now spread across multiple `service()` calls. App code that called `scheduleSend` and then *immediately* did `while(rl.txPending);` busy-waiting still works (txPending stays true through TX-Done IRQ), but it now actually blocks the main loop, whereas before that loop would terminate after the single blocking `transmit()` returned. Recommend: don't busy-wait on `txPending`; let the IRQ-driven flow run naturally.

2. **The TX watchdog can spontaneously clear `txPending` to `false` if recovery fires.** Gateway logic that scheduled a TX and assumed it will eventually be delivered (no `false` short-circuit) must now handle the possibility that `txPending` went from `true` → `false` *without* `cb.onTxDone` firing. `Core::txWatchdogCount` increments in that case; `Core::debug` will hold 0xB0 (in-flight) or 0xB1 (submission). If the Gateway has an upper-layer retry mechanism, this becomes the trigger point.

3. **Stream-TX is aborted on recovery.** If a watchdog fires during a multi-fragment stream-TX, the entire stream is dropped (streamMode→None, streamActive→false). The receiver will *not* finish reassembling the partial stream — the partial fragments it already got would have triggered a `packets_left` mismatch and been rejected anyway. Gateway-side application code that does multi-fragment streams should observe `streamActive` going back to `false` *without* the final `streamReady=true`, and treat that as a failed stream that needs a full resend.

4. **`RADIOLIB_LOW_LEVEL=1` is now defined in `racelink_transport_core.h` before `#include <RadioLib.h>`.** This exposes SX126x low-level commands as public.

   - *If the Gateway's translation units all include `racelink_transport_core.h` BEFORE any other path to RadioLib*: no problem, the define propagates naturally.
   - *If the Gateway has additional .cpp files that include RadioLib via a different route (e.g. directly include `<RadioLib.h>` without going through transport_core.h)*: those TUs see `RADIOLIB_LOW_LEVEL=0` and the SX126x class will have low-level methods as `protected`. Layout is identical so the binary is fine, but it's technically a per-TU access-specifier divergence.

   **Recommendation:** add `-D RADIOLIB_LOW_LEVEL=1` to the Gateway's PlatformIO `build_flags` (one line in the .ini). That makes the define global and removes any per-TU divergence. Same change would also be a tidier alternative on the Node side, but for the Node it's not strictly needed because all Node TUs reach RadioLib only through transport_core.h.

5. **CAD parameters and timings are unchanged.** `RADIOLIB_SX126X_CAD_ON_4_SYMB`, `detPeak=18`, `detMin=10`, `exitMode=CAD_GOTO_STDBY` — same as before. CSMA/CA backoff timing (50–300 ms jitter, 100–200 ms CAD-busy backoff) is unchanged. No change in channel-access behaviour relative to other devices on the air.

### Recommended Gateway verification steps

1. **Compile** the Gateway against the updated headers. The `toaUsMax17 → toaUsMaxPkt` rename will surface immediately if Gateway code references the old name.
2. **Sanity test 1:** boot Gateway + 1 Node, send a simple command, observe response. No deadlock, RSSI/SNR reported correctly.
3. **Sanity test 2:** the same 10-Node broadcast-burst that exposed the original deadlock. Confirm none deadlock from the Gateway side either (i.e. the Gateway itself remains responsive).
4. **Stream test:** if the Gateway uses `scheduleStreamSend`, run a multi-fragment stream both Gateway→Node and Node→Gateway. Verify clean completion via `streamReady`.
5. **Long soak:** 24 h with normal traffic. `txWatchdogCount` and `rxErrCount` on both Node and Gateway should remain at or near zero. Monotonic `txCount` / `rxCount`.

---

## Verification Status

### Node-side (this repo)

- [x] Build success: `RaceLink_Node_v4_s3_llcc68` (ESP32-S3 + LLCC68)
- [x] Build success: `RaceLink_Node_v3_s2_llcc68` (ESP32-S2 + LLCC68)
- [x] Build success: `RaceLink_Node_v1_c3_ct62` (ESP32-C3 + SX1262)
- [x] Initial smoke test: 10-Node broadcast burst — no deadlocks observed after the IRQ-fix layer landed.
- [ ] Multi-day soak (pending — this is why the doc is DRAFT)
- [ ] Targeted I2 test: inject a `writeBuffer` failure in a debug build, expect `recoverToIdle(0xB1)` within ~5 s
- [ ] Targeted I4 test: induce a delayed TX_DONE, expect no cancellation of a freshly-scheduled TX
- [ ] Logic-analyzer measurement of CAD-Done → first-bit-on-air delta (P3 effect size)

### Gateway-side

- [ ] Compile against updated headers
- [ ] Sanity round-trip
- [ ] 10-Node broadcast verification from Gateway side
- [ ] Stream round-trip
- [ ] 24h soak

---

## Rollback Notes (if the field test reveals issues)

If the soak surfaces a regression, the safest rollback granularity is by layer:

- **Revert just the IRQ-watchdog layers** (Layer 2 + Layer 3): keep P1/P2/P3 (latency gains), restore the simple "blocking transmit inside service" behaviour. This removes the new code paths but loses the non-blocking benefit.
- **Revert P3 only**: keeps P1/P2 (small wins), reverts to blocking `transmit()`. Removes the deadlock-risk surface entirely. Use this if any unresolved IRQ-related issue appears.
- **Full revert**: drop everything in this PR. Use only as a last resort — the deadlock the IRQ-fix solved would come back.

The whole change is concentrated in one file ([racelink_transport_core.h](../racelink_transport_core.h)) plus a one-line rename in [racelink_wled.cpp](../racelink_wled.cpp), so each rollback is a small, surgical edit, not a sweeping diff.

---

## Telemetry to watch during the test

Display via the WebUI Debug field (`Core::debug`) or any custom JSON exporter:

| Field | Healthy expectation | Pathology meaning |
|---|---|---|
| `txCount`, `rxCount{Total,Filtered}` | Monotonic increase | Stalled = TX/RX broken |
| `txWatchdogCount` | 0 or very small | Persistent watchdog firing = real issue |
| `rxErrCount` | Small (RF noise floor) | Spiking = SNR / antenna problem, not a code regression |
| `Core::debug` codes | Normally 0 or CAD-busy counter | 0xA1/0xA2/0xA3/0xB0/0xB1 = recovery fired |
| `txPending` stalled at `true` for > 5 s | Should never happen anymore | Submission watchdog should clear it |
