# Headless Master — Persistent Slave Registry, Group-ID Layout Fix, TX Indicator

**Date:** 2026-05-17
**Status:** Built (V3 S2 + V4 S3), pending field validation.
**Scope:** Four files in the WLED node firmware:
- [usermods/racelink_wled/racelink_headless.h](../racelink_headless.h) — constants, new struct
- [usermods/racelink_wled/racelink_indicators.h](../racelink_indicators.h) — new catalog row
- [usermods/racelink_wled/racelink_wled.h](../racelink_wled.h) — new members and method declarations
- [usermods/racelink_wled/racelink_wled.cpp](../racelink_wled.cpp) — all logic

**Wire protocol:** unchanged. Group IDs are still 8-bit, `OPC_SET_GROUP` / `OPC_DEVICES` / `OPC_INDICATE` packet shapes identical. The new `IND_PAIRING_TX` indicator type is append-only (ID = 5); older receivers silently drop unknown indicator types.
**Config schema:** one new key under `RaceLink.overrides` in `cfg.json` — additive, backward compatible.

---

## TL;DR

Three independent gaps in the Headless Master mode are closed in a single wave:

1. **Slaves stayed lost after a master reboot.** The master never persisted "which 3-byte address belongs to which group ID." Slaves rebooting alongside the master could re-send `IDENTIFY_REPLY` and trigger the idempotent path, but slaves that kept running through a master swap (typical: battery-powered race lights, master taken out for a battery change) silently lost their pairing. The master now keeps a **40-entry persistent registry** of `(addr3, groupId)` in `cfg.json`, replays it as proactive `SET_GROUP` packets after promotion, and recycles a stored ID when an unconfigured slave (groupId=0) with a known MAC re-appears instead of burning a fresh counter slot.

2. **Group-ID semantics were inconsistent.** `HEADLESS_FIRST_GROUP_ID = 1` meant the first slave assigned was group 1 — colliding with the conceptual "Group 1 belongs to the master, Group 0 is the unconfigured pool." Constants renamed and split: `HEADLESS_MASTER_GROUP_ID = 1` (the master self-assigns on `enterHeadlessMode`), `HEADLESS_FIRST_GROUP_ID = 2` (slaves start here). `exitHeadlessMode` now resets counter + registry + own `groupId` so the next promotion starts from a clean slate.

3. **Pairing activity was invisible.** Operators could not tell when the master was actually configuring a slave (vs. running routine scene/sync/brightness broadcasts that look identical from outside). A new `IND_PAIRING_TX` indicator (green-cyan strobe, ID 5) fires **only on SET_GROUP sends** — both new-device pairings AND the post-reboot re-bind sweep over the persistent registry — through a single chokepoint wrapper `headlessSendTx()`, throttled to 200 ms minimum interval and 1500 ms display time. All non-pairing TX paths (probe, scene, brightness, SYNC keepalive) explicitly pass `armBlip=false`.

Build verified on `RaceLink_Node_v3_s2_llcc68` (ESP32-S2 + LLCC68) and `RaceLink_Node_v4_s3_llcc68` (ESP32-S3 + LLCC68).

---

## Change Inventory

### Layer 1 — Persistent slave registry

#### Storage schema (additive)

| Key (under `RaceLink.overrides`) | Type | Default | Meaning |
|---|---|---|---|
| `Headless Slaves` | JSON array, max 40 entries, each `{ "a": "AABBCC", "g": 2..254 }` | `[]` | Master's record of which slave 3-byte addr is on which group. |

A full 40-entry table renders to ~340 bytes of JSON, well within the existing `cfg.json` budget. Empty arrays are written explicitly so the round-trip stays consistent.

#### Runtime state ([racelink_wled.h](../racelink_wled.h))

New private members on `UsermodRaceLink`:

```cpp
RaceLinkHeadless::HeadlessSlaveRec headlessSlaves[RaceLinkHeadless::HEADLESS_MAX_SLAVES];
uint8_t  headlessSlavesCount   = 0;
bool     headlessPersistDirty  = false;
uint32_t lastSlaveTableMutMs   = 0;
uint8_t  reassignCursor        = 0xFF;     // 0xFF = idle
uint32_t reassignNextSendAtMs  = 0;
uint32_t lastPairingBlipAtMs   = 0;
```

The record struct lives in [racelink_headless.h](../racelink_headless.h) so the catalog header remains the single source of truth for headless wire and state structures:

```cpp
struct HeadlessSlaveRec {
  uint8_t addr3[3];   // {0,0,0} = empty slot
  uint8_t groupId;    // 0       = empty slot
};
```

#### Helper API

| Method | Purpose |
|---|---|
| `findSlaveIdx(a3)` | Linear scan, returns `-1` if not present. 40-entry max, hot path is negligible. |
| `upsertSlave(a3, groupId)` | Update existing OR append. Returns `false` only when the table is full AND the address is new — callers refuse the pairing in that case. |
| `clearSlaveTable()` | Wipes all entries. Called from `exitHeadlessMode()`. |
| `markHeadlessPersistDirty()` | Sets `headlessPersistDirty=true` + `lastSlaveTableMutMs=millis()`. Cheap, called from inside TX paths. |
| `serviceHeadlessPersist(now)` | Loop pump: flips `configNeedsWrite=true` once `HEADLESS_PERSIST_DEBOUNCE_MS` (5 s) of quiet have elapsed. |

### Layer 2 — Group-ID layout

Constants in [racelink_headless.h](../racelink_headless.h):

| Old | New | Meaning |
|---|---|---|
| `HEADLESS_FIRST_GROUP_ID = 1` | `HEADLESS_FIRST_GROUP_ID = 2` | first ID handed out to a slave |
| _(none)_ | `HEADLESS_MASTER_GROUP_ID = 1` | the master's own groupId during headless operation |
| `HEADLESS_MAX_GROUP_ID = 254` | unchanged | last valid slave ID (255 reserved for broadcast pseudo-group) |

Lifecycle:

| Function | Group 0 | Group 1 | Groups 2..254 |
|---|---|---|---|
| `enterHeadlessMode()` | unchanged (unconfigured pool) | `current.groupId = HEADLESS_MASTER_GROUP_ID` | counter starts at `HEADLESS_FIRST_GROUP_ID` (auto-bumped on first `headlessAssignGroupTo`) |
| `headlessAssignGroupTo()` | input case — assign next free ID | n/a (never assigned to slaves) | output values |
| `exitHeadlessMode()` | `current.groupId = 0` (rejoins pool) | freed | counter reset to `0`, registry cleared |

### Layer 3 — `headlessAssignGroupTo` rewrite

The previous function had one early-return for `inGroupId != 0` and one assign-and-increment branch. It now splits cleanly into two cases:

```cpp
// Case A: slave already advertises a non-zero groupId.
//   Mirror it into our registry so a later master reboot can re-bind
//   it proactively, but send NO packet — overwriting a working pairing
//   risks group collisions in the wider fleet.
if (inGroupId != 0) {
  if (upsertSlave(senderLast3, inGroupId)) markHeadlessPersistDirty();
  return;
}

// Case B: slave reports groupId=0 (fresh, factory-reset, or pool member).
//   B.1 If we know its MAC -> recycle its previous ID. Counter untouched.
//   B.2 If we don't know it -> pull the next free counter slot.
int8_t existing = findSlaveIdx(senderLast3);
uint8_t assigned;
bool fromCounter = false;
if (existing >= 0) {
  assigned = headlessSlaves[existing].groupId;
} else {
  // clamp + exhaustion check on overrides.headlessGroupCounter
  assigned    = overrides.headlessGroupCounter;
  fromCounter = true;
  if (!upsertSlave(senderLast3, assigned)) {
    DEBUG_PRINTLN(F("[RaceLink] Headless: slave table full"));
    return;     // counter NOT bumped — no orphan ID
  }
  overrides.headlessGroupCounter = (uint8_t)(assigned + 1);
}

// Build + send + persist
uint8_t out[32];
uint8_t n = RaceLinkHeadless::buildSetGroupPacket(out, rl.myLast3, senderLast3, assigned);
if (!n) {
  if (fromCounter) overrides.headlessGroupCounter = assigned;  // roll back
  return;
}
headlessSendTx(out, n, /*armBlip=*/true);
markHeadlessPersistDirty();
headless.lastBroadcastAtMs = millis();
```

The previous direct `configNeedsWrite = true` was replaced by `markHeadlessPersistDirty()` so a 40-slave pairing burst collapses to **one** `cfg.json` write instead of 40 (see Layer 5 — flash wear).

### Layer 4 — Proactive re-bind after master reboot

The master can come back online after a power cycle while slaves keep running on stored group IDs and never re-emit `IDENTIFY_REPLY`. To re-bind them, the master walks the persisted registry and sends one `SET_GROUP` per `HEADLESS_REASSIGN_INTERVAL_MS` (50 ms) right after promotion.

| Method | Purpose |
|---|---|
| `startHeadlessReassign()` | Initialises the cursor and the first-send timestamp. Called from the end of `enterHeadlessMode()`. No-op when the registry is empty. |
| `serviceHeadlessReassign(now)` | Loop pump: sends one `SET_GROUP` per `HEADLESS_REASSIGN_INTERVAL_MS`, advances the cursor, terminates by setting cursor = `0xFF`. Aborts cleanly if `headless.active` flips off mid-sweep (e.g. a real Gateway took over). |

For a 40-slave registry the sweep takes ~2000 ms — just within the `IND_PAIRING_TX` 1500 ms display window, so the operator sees a continuous green-cyan strobe for the whole re-bind period (each send extends the deadline before it can lapse, with the 200 ms throttle preventing the wrapper from re-firing the indicator within a burst).

Slaves accept the `SET_GROUP` idempotently (existing handler in [racelink_wled.cpp:1410–1438](../racelink_wled.cpp#L1410-L1438) — `SET_GROUP` simply overwrites `current.groupId` and ACKs). A slave that was already on the correct group sees a brief Pair-Confirmed blink and no functional disruption — visually a "fleet roll-call" cue rather than a re-pairing event.

### Layer 5 — Persistence debounce (flash wear)

`cfg.json` saves are full-file rewrites preceded by a full-file backup copy — roughly two LittleFS sector erases per save. On a 960 KB partition with wear-leveling, the headroom is ~120 000 saves. The previous code path fired `configNeedsWrite = true` synchronously on every `headlessAssignGroupTo()` call, meaning a 40-slave pairing burst could produce 40+ saves in ~30 s.

The new pump-based pattern:

| Step | Trigger | Action |
|---|---|---|
| 1 | Any registry mutation in `headlessAssignGroupTo` or `upsertSlave` from elsewhere | `markHeadlessPersistDirty()` sets the flag + the timestamp. No I/O. |
| 2 | `loop()` calls `serviceHeadlessPersist(nowMs)` every tick | If dirty AND `(now - lastSlaveTableMutMs) > HEADLESS_PERSIST_DEBOUNCE_MS` (5 s): flip `configNeedsWrite = true`, clear dirty. |
| 3 | Existing WLED loop machinery | `configNeedsWrite` triggers `serializeConfigToFS()` next tick. |

Result: a continuous pairing burst (one new slave every 1–2 s) collapses to a single save 5 s after the burst ends. The debounce is intentionally scoped to the slave-table path — `OPC_CONFIG`, WebUI Save, and `exitHeadlessMode` continue to write synchronously (rare operator actions where "save now" is the correct UX).

### Layer 6 — Pairing-TX indicator

#### Indicator catalog ([racelink_indicators.h](../racelink_indicators.h))

New enum value (append-only — older firmware silently drops unknown types):

```cpp
IND_PAIRING_TX = 5,   // green-cyan strobe — Headless master sent a SET_GROUP packet
```

New catalog row:

```
type             label         fxMode      spd  int  color1     bri
IND_PAIRING_TX   "Pairing TX"  23 STROBE   248   96  0x00FF40   200
```

Speed 248 sits between the existing "informational" (245) and "alert" (250) tiers — fast enough to read as activity, slow enough to distinguish from `IND_PROBE_REJECTED` (250, red). Intensity 96 shortens the on-pulse so a single fired blip looks like a quick flash rather than a sustained overlay. Brightness 200 (vs. the catalog's normal 230) trades a bit of visibility for less eye fatigue during a 40-slave re-bind burst.

#### Sub-second indicator API

The existing `applyLocalIndicator(type, durationSec)` rounds to whole seconds. The pairing blip needs sub-second precision, so a millisecond variant was added:

```cpp
void applyLocalIndicatorMs(uint8_t type, uint32_t durationMs);
```

Identical semantics otherwise — same `indicator.active = true` + `expiresAtMs = millis() + durationMs` + catalog lookup. The five existing call sites still use the seconds variant; only the pairing path needs the ms form.

#### Wrapper at the TX chokepoint

`RaceLinkTransport::scheduleSend()` is the underlying transport call. Every Headless-master TX site now goes through:

```cpp
bool headlessSendTx(const uint8_t* pkt, uint8_t n, bool armBlip);
```

which:

1. Calls `RaceLinkTransport::scheduleSend(rl, pkt, n)` and captures the success boolean.
2. If `armBlip == true` AND `headless.active` AND we are not in the throttle window:
   - Update `lastPairingBlipAtMs = now`
   - `applyLocalIndicatorMs(IND_PAIRING_TX, HEADLESS_PAIRING_INDICATOR_DURATION_MS)` (1500 ms)
3. Returns the transport's success boolean.

The 200 ms throttle (`HEADLESS_PAIRING_INDICATOR_THROTTLE_MS`) prevents the wrapper from re-extending the indicator deadline within a burst, so 40 back-to-back SET_GROUPs during a re-bind sweep produce **one** continuous blink rather than a flickering retrigger storm.

#### Sites converted

`armBlip = true` is reserved for SET_GROUP sends only (new-device pairing AND the post-reboot re-bind sweep). All other master TX paths pass `false` — the operator-visible signal is "the master is configuring a slave right now", not "the master is transmitting."

| Site | `armBlip` | Notes |
|---|---|---|
| Brightness broadcast on button release | `false` | Routine traffic, not pairing. |
| `serviceHeadless` first probe send | `false` | Master not yet promoted; blinking here would mislead. |
| `serviceHeadless` second probe send | `false` | Same as above. |
| `headlessBroadcastCurrentScene` | `false` | Routine traffic, not pairing. |
| `headlessBroadcastSync` (30 s keepalive) | `false` | Routine traffic, not pairing. |
| `headlessAssignGroupTo` | **`true`** | Every accepted pairing flashes — new device joining the fleet. |
| `serviceHeadlessReassign` | **`true`** | Drives the continuous-blink re-bind sweep over the persistent registry. |

---

## Behavioural Reference

### Pairing a new slave

1. Slave boots with `groupId = 0`.
2. Slave's startup `IDENTIFY_REPLY` reaches the master.
3. Master enters `headlessAssignGroupTo` Case B.2 — first free ID is 2 (or current counter).
4. `upsertSlave({addr3, 2})` succeeds, counter bumps to 3.
5. `SET_GROUP` packet to the slave, `IND_PAIRING_TX` fires (1.5 s green-cyan).
6. Slave applies the group, ACKs, shows `IND_PAIR_CONFIRMED`.
7. 5 s of pairing-silence later → `serviceHeadlessPersist` writes `cfg.json`.

### Re-pairing a known slave (idempotent)

1. Slave already on group 5 reboots OR powers up with master gone and back.
2. Slave sends `IDENTIFY_REPLY` with `groupId = 5`.
3. Master enters Case A — `upsertSlave({addr3, 5})` updates (or no-op if already there), marks dirty, returns. **No SET_GROUP TX.**
4. 5 s later → registry persisted if anything actually changed.

### Recycling after factory reset

1. Slave previously on group 5 → factory reset → `groupId = 0` → `IDENTIFY_REPLY`.
2. Master enters Case B.1 — `findSlaveIdx(addr3)` returns ≥ 0, recycles `assigned = 5`.
3. Counter unchanged. `SET_GROUP` packet with group 5. Registry untouched, no save needed.

### Master reboot (slaves stayed up)

1. Master power-cycles, `headlessPersistedActive = true` in cfg.json triggers `tryStartHeadless()` from `setup()`.
2. Probe window expires without contradiction → `enterHeadlessMode()` runs.
3. `current.groupId = 1`, optional scene restore, optional initial SYNC, then `startHeadlessReassign()`.
4. 40-slave registry sweeps over ~2000 ms, each send arms a `IND_PAIRING_TX` that bridges into the next via the 200 ms throttle.
5. Every slave receives its previous `groupId` again, ACKs with Pair-Confirmed.

### Operator deactivation

1. 5-click → `exitHeadlessMode()`.
2. `reassignCursor = 0xFF`, `overrides.headlessGroupCounter = 0`, `current.groupId = 0`, `clearSlaveTable()`.
3. `headlessPersistDirty = false`, `configNeedsWrite = true` — **synchronous** write, no debounce. The operator expectation "off means off" must survive an immediate battery pull.
4. `IND_HEADLESS_EXIT` indicator fires, slaves on the previous network gradually lose contact and fall back to their boot color.

---

## Backward Compatibility

- Existing `cfg.json` files without `Headless Slaves` read as an empty array; `headlessSlavesCount = 0`. The master operates as before but with empty registry → no proactive re-bind on the next promotion until at least one slave has been (re-)paired.
- Existing `cfg.json` files with `Headless Group Counter > 0` and `Headless Active = true` keep their counter on next boot. The first slave that pairs after the upgrade still uses the old counter value (≥ 2 in normal operation; legacy value = 1 if the master had been activated under the previous firmware will assign 1 to its next slave and only then move to 2 — visible quirk lasting one pairing).
- Old slave firmware: indifferent to `IND_PAIRING_TX`. `OPC_INDICATE` packets aren't sent for the pairing blip anyway (it's a local overlay only), so there's no wire-level forward-compat concern.
- A WLED build with this code talking to an external Gateway+Host pair is unchanged — none of these helpers run when `headless.active == false`.

---

## Potential Issues (acknowledged)

| Risk | Mitigation in code | Operator-visible symptom |
|---|---|---|
| Registry full (>40 slaves) | `upsertSlave` returns false, `headlessAssignGroupTo` refuses without bumping counter. | New slave gets no Pair-Confirmed blink; serial logs "slave table full." Raise `HEADLESS_MAX_SLAVES` if it ever happens in production. |
| Slave belongs to a different master that died | Case A still mirrors the foreign ID into our registry. | Same as today (idempotent path always trusted the slave). Doc'd as known limitation; a future "drop after N unanswered SYNCs" cleanup is the natural follow-up. |
| Mid-event master swap | Re-bind sweep sends `SET_GROUP` to slaves that already have the right group. | Visual: a brief Pair-Confirmed cascade across the fleet. Functional: idempotent — no disruption. |
| OPC_CONFIG write during a pairing burst | `OPC_CONFIG` sets `configNeedsWrite = true` directly; the burst's pending registry change is rolled into the same save. | None — debounce just becomes a re-save no-op. |
| Counter starvation in legacy upgrades | If the counter sat at 254 before the upgrade, the next assignment exhausts it. | Same as before the change; `exitHeadlessMode()` now resets it, so the path out is "deactivate and re-enter headless." |

---

## Flash Wear Headroom (post-change)

| Scenario | Saves per event | Notes |
|---|---|---|
| 40-slave pairing burst (cold start) | **1** | 5 s after the last `markHeadlessPersistDirty`. |
| Re-bind sweep after master reboot | 0 | All TX uses existing registry; nothing mutates `cfg.json`. |
| Operator brightness fade | 1 | Existing behaviour preserved. |
| Headless scene change (1-click) | 1 | Existing behaviour preserved. |
| `exitHeadlessMode` | 1 | Synchronous, immediate. |

A typical event ("start", "run", "stop") now costs 2–3 `cfg.json` saves instead of the previous ~80, comfortably leaving the 120 000-save partition headroom untouched even with weekly events for decades.

---

## Verification Plan

| # | Scenario | Expected |
|---|---|---|
| 1 | Build V3 (S2) + V4 (S3) | Both green, no warnings introduced. |
| 2 | Fresh master, first slave pairs | `cfg.json::RaceLink.overrides["Headless Slaves"]` has one entry with `g: 2`; master's `RaceLink::groupId` is 1. |
| 3 | Re-pair a known slave (idempotent) | No `SET_GROUP` sent, registry unchanged, no second blink. |
| 4 | Slave factory-reset then re-pair | Original groupId recycled; counter unchanged. |
| 5 | Master power-cycle without deactivation | Auto-resume + probe + promotion + re-bind sweep visible as continuous green-cyan strobe for ~2 s; slaves keep their IDs. |
| 6 | 5-click exit | `Headless Group Counter = 0`, `Headless Slaves = []`, `groupId = 0`, `Headless Active = false`. Synchronous save. |
| 7 | Pairing-burst flash wear | 10 slaves paired in 20 s → serial log shows 10× "assigned group" but only **one** "Writing settings to /cfg.json..." 5 s after the last pairing. |
| 8 | Pairing blip visibility | New slave joins → one 1.5 s green-cyan blink on the master synchronised with the `OPC_SET_GROUP` send. 1-click scene advance / SYNC keepalive / brightness broadcast → **no blink** (routine traffic). |
| 9 | Throttle correctness | Force two SET_GROUP sends within 200 ms (e.g. two slaves powering up simultaneously) → indicator fires once, deadline not re-extended; visible as one continuous flash through the overlap. |
| 10 | Probe sends do NOT blip | While `tryStartHeadless()` probes, no green-cyan flash. Only after `enterHeadlessMode()` runs AND a SET_GROUP actually goes out does the indicator fire. |
| 11 | Registry full | Pair the 41st slave → no SET_GROUP, no registry change, serial log "slave table full." First 40 slaves still operate. |

---

## Build Footprint

| Build | RAM delta | Flash delta |
|---|---|---|
| V3 (S2) | +176 B | ~+1.5 KB |
| V4 (S3) | +176 B | ~+1.5 KB |

RAM cost breakdown: 40 × 4 bytes = 160 bytes for `headlessSlaves[]`, plus ~16 bytes of cursors/timestamps/flags.

---

## Reference: Constants Summary

```cpp
// racelink_headless.h
static const uint8_t  HEADLESS_MASTER_GROUP_ID        = 1;
static const uint8_t  HEADLESS_FIRST_GROUP_ID         = 2;
static const uint8_t  HEADLESS_MAX_GROUP_ID           = 254;
static const uint8_t  HEADLESS_MAX_SLAVES             = 40;
static const uint32_t HEADLESS_REASSIGN_INTERVAL_MS   = 50;
static const uint32_t HEADLESS_PERSIST_DEBOUNCE_MS    = 5000;
static const uint32_t HEADLESS_PAIRING_INDICATOR_THROTTLE_MS = 200;
static const uint32_t HEADLESS_PAIRING_INDICATOR_DURATION_MS = 1500;

// racelink_indicators.h
enum IndicatorType : uint8_t {
  ...
  IND_PAIRING_TX = 5,
};
```
