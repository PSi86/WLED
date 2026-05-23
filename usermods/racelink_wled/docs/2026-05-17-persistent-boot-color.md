# Persistent Per-Device Boot Color

**Date:** 2026-05-17
**Status:** Built (V3 S2 + V4 S3), pending field validation.
**Scope:** Two files in the WLED node firmware: [usermods/racelink_wled/racelink_wled.h](../racelink_wled.h) and [usermods/racelink_wled/racelink_wled.cpp](../racelink_wled.cpp).
**Wire protocol:** unchanged. Existing `SCENE_RESTORE_BOOT_COLOR` semantics preserved.
**Config schema:** four new keys under `RaceLink.overrides` in `cfg.json` — additive, backward compatible.

---

## TL;DR

Each RaceLink device used to roll a fresh `esp_random() % 3` boot color on every power-up (red, green or blue), discarding the value on each reboot. Operators could not visually identify a device across power cycles, and the color picked by the 1-click button cycle never survived a battery swap.

After this change:

1. The boot color is rolled **once** on the very first boot, immediately persisted to `cfg.json`, and re-used on every subsequent boot.
2. The existing 1-click color cycle still operates as before (R → G → B → random RGB), but **10 seconds** after the last click the currently-displayed color is written back as the new boot color.
3. Random cycle positions (any RGB outside the three primaries) are persisted **verbatim** as a 3-byte triple — re-applied exactly at the next boot, not re-rolled.
4. `SCENE_RESTORE_BOOT_COLOR` (headless scene 4) now replays the persisted color via the same code path used at boot, dropping the previous "re-roll if uninitialized" fallback because the persisted value is guaranteed to be valid after `setup()`.

Group fairness side-effect: a fleet of 10 freshly-flashed devices, where 9 had shown red/green and only 1 blue on first power-up, now distributes its colors permanently after the first roll. Even if `esp_random()` early in `setup()` returns biased pseudo-random values (ESP-IDF documents that hardware RNG quality is undefined without an active RF subsystem), the value is locked in and editable via the button afterwards — the operator can always re-balance the fleet manually.

---

## Change Inventory

### Storage schema (additive)

| Key (under `RaceLink.overrides`) | Type | Default | Meaning |
|---|---|---|---|
| `Boot Color Mode` | `uint8_t` | `0xFF` (uninit) | 0/1/2 = R/G/B primary, 3 = use stored RGB triple |
| `Boot Color R` | `uint8_t` | `0` | only meaningful when `Mode == 3` |
| `Boot Color G` | `uint8_t` | `0` | only meaningful when `Mode == 3` |
| `Boot Color B` | `uint8_t` | `0` | only meaningful when `Mode == 3` |

Default `0xFF` on the mode is the "first boot ever" signal — `setup()` then rolls `esp_random() % 3`, writes the value back, and flips `configNeedsWrite=true` so the roll is durable before the next power loss.

### Runtime state ([racelink_wled.h](../racelink_wled.h))

`BtnState` (existing struct) gained three fields, renamed one:

| Change | Field | Purpose |
|---|---|---|
| **Renamed** | `bootColorIdx` → `bootColorMode` | Now 0..3 instead of 0..2; "3 = random" is the new mode. |
| **New** | `uint8_t bootColorRgb[3]` | The exact RGB currently shown. Updated by every `applyCycleColor()` call so the deferred save can persist whatever the operator is looking at. |
| **New** | `bool bootColorSavePending` | Edge-trigger flag. Armed by an operator click in `applyColorCycleStep()`, cleared by `serviceBootColorSave()` after the post-click idle window elapses. Prevents the save from re-firing on every loop tick while the click is older than 10 s. |

`OverrideValues` (existing struct) gained the four persistent fields listed in the schema table above.

### Save-on-idle pump

| Function | Trigger | Effect |
|---|---|---|
| `applyCycleColor(idx)` | called by `applyColorCycleStep` and `applyBootColor` | Mirrors the just-painted RGB into `btn.bootColorMode` + `btn.bootColorRgb[]`. **Does not** arm the save. |
| `applyColorCycleStep(now)` | 1-click while the operator is driving the cycle | Sets `btn.bootColorSavePending = true; btn.lastColorClickMs = now;`. Multiple clicks in a burst all reset the timestamp. |
| `serviceBootColorSave(now)` | called from `UsermodRaceLink::loop()` every tick | If pending AND `(now - lastColorClickMs) > RACELINK_BTN_COLOR_RESET_MS` (10 s): copies the runtime values into `overrides.bootColor*`, sets `configNeedsWrite=true`, clears the pending flag. |

A 5-click headless toggle, a 3-click hotspot recovery, or any other gesture leaves `bootColorSavePending` unchanged, so they cannot accidentally persist a partial cycle.

### `applyBootColor()` replaces `showBootRandomColor()`

`showBootRandomColor()` was a "roll + paint + remember for this session" routine. The replacement [`applyBootColor()`](../racelink_wled.cpp) is purely "paint the persisted color":

- For modes 0/1/2 it delegates to `applyCycleColor(mode)`, then seeds the cycle ring buffer so the first post-boot click advances to `(mode+1) % 3` — preserves the "all three primaries reachable in three clicks" UX.
- For mode 3 it writes the stored RGB triple directly into `colPri[]`, sets `FX_MODE_STATIC`, and arms the cycle counter as if all primaries had already been shown (next click jumps straight back to random via the existing idle-reset path after 10 s).
- It explicitly clears `bootColorSavePending` — boot-time display is not an operator change.

### `SCENE_RESTORE_BOOT_COLOR` handler

The wire-side handler in the OPC_HEADLESS dispatcher used to defensively re-roll if `bootColorIdx > 2`:

```cpp
if (btn.bootColorIdx > 2) btn.bootColorIdx = (uint8_t)(esp_random() % 3);
applyCycleColor(btn.bootColorIdx);
```

That fallback is gone. `setup()` guarantees the mode is in 0..3 before the radio is even active, so a remote `SCENE_RESTORE_BOOT_COLOR` packet now collapses to a single line:

```cpp
applyBootColor();   // uses persisted mode + RGB
```

### `setup()` initialisation order

```cpp
if (overrides.bootColorMode > 3) {
  // First boot ever — roll once, persist immediately so a power loss
  // between now and the next save event does not lose the value.
  overrides.bootColorMode = (uint8_t)(esp_random() % 3);
  overrides.bootColorR = overrides.bootColorG = overrides.bootColorB = 0;
  configNeedsWrite = true;
}
// Mirror persisted -> runtime
btn.bootColorMode   = overrides.bootColorMode;
btn.bootColorRgb[0] = overrides.bootColorR;
btn.bootColorRgb[1] = overrides.bootColorG;
btn.bootColorRgb[2] = overrides.bootColorB;

if (bootPreset == 0) {
  applyBootColor();   // (operator did NOT configure a boot preset)
}
```

The mirror also runs on devices where `bootPreset != 0` (operator did set a boot preset). In that case the local display is suppressed, but the runtime mirror still feeds `SCENE_RESTORE_BOOT_COLOR` correctly.

---

## Why ten seconds, and why no MAC mix-in

**10-second debounce** matches the existing `RACELINK_BTN_COLOR_RESET_MS` window. That's the same idle gap that resets the color-cycle ring buffer, so the two ideas are tied to one mental model: "stop clicking for 10 s and the cycle resets AND the current color is locked in."

**No MAC-based entropy mixing on the first roll.** `esp_random()` alone is used. Rationale: the value is only ever rolled once per device. Even if the early-boot RNG returns the same pseudo-random number on two adjacent devices, the operator can always change the colors manually via the button, and the change is durable. Mitigating a one-shot statistical anomaly with permanent code complexity is a poor trade. If field experience shows persistent first-roll skew across a large fleet, this is the natural place to add a `^ macLast3[0..2]` mix.

---

## Backward Compatibility

- Existing `cfg.json` files without the four new keys read defaults (`Mode = 0xFF`, RGB = 0). `setup()` treats that as the first-boot case and rolls + persists.
- The boot color stored for headless slaves prior to this change (which only happened to be implicitly remembered through other paths) is **lost** on first boot under the new firmware. The next 10 s of inactivity locks in a fresh random color; the operator can override via the button.
- Wire format unchanged — no slave/master version dependency.

---

## Verification Plan

| # | Scenario | Expected |
|---|---|---|
| 1 | Build V3 (S2) + V4 (S3) | Both green, no warnings introduced by this change. |
| 2 | Fresh device first boot | One of R/G/B shown; `cfg.json::RaceLink.overrides["Boot Color Mode"]` is 0/1/2, RGB fields = 0. |
| 3 | Power cycle | Same color as before. |
| 4 | 1-click → 1-click → power-cycle within 10 s | Old persisted color (save timer not yet expired). |
| 5 | 1-click → wait 10 s → power-cycle | New color visible after reboot; `cfg.json` reflects the change. |
| 6 | Click through to random RGB → wait 10 s | `Mode = 3`, RGB fields contain the exact shown color. Power cycle reproduces the same RGB. |
| 7 | Headless master broadcasts `SCENE_RESTORE_BOOT_COLOR` | All slaves return to their individual persisted boot colors (mixes of primaries + random RGBs depending on operator history). |
| 8 | 10 devices flashed clean, all powered on simultaneously, observed across several reflash rounds | If a persistent first-roll skew appears (e.g. consistently fewer blues), revisit the MAC-mix decision. |

---

## Build Footprint

V3 (S2): no measurable size change vs. baseline (boot color logic replaced inline).
V4 (S3): no measurable size change vs. baseline.

RAM cost: 5 additional bytes per device (`bootColorRgb[3]` + `bootColorSavePending` + a small alignment slack).
