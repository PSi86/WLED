#pragma once
#include <Arduino.h>

/*
  RaceLink ePaper helper (2.9\" BW, GxEPD2)

  Public API:
    - void epaperInit();
        Initializes SPI + display and shows a boot screen:
          "RaceLink Startblock" + WLED_RELEASE_NAME

    - void setDisplayLayout(uint8_t numPilots);   // 1..4
        1: single row (nickname large, right full-height inverted bar)
        2..4: multi row (one row per slot, right inverted bar per row)

    - bool setPilotSlotData(const char* nickname, const char* raceLabel, uint8_t slot);
        Updates the stored data for the given slot (1..4) and starts / restarts
        a deferred refresh timer. The display is NOT updated immediately.
        Returns false if slot is out of range or exceeds the configured layout.

    - void service_epaper();
        Call this from loop(). It triggers a full refresh 1.5s after the last
        received update command.

  Notes:
    - SPI MOSI/SCK are configurable via build flags:
        -D RACELINK_EPAPER_SPI_MOSI=11
        -D RACELINK_EPAPER_SPI_SCK=12
      (MISO is typically unused; default -1)

    - Display control pins are configurable too:
        -D RACELINK_EPAPER_CS=...
        -D RACELINK_EPAPER_DC=...
        -D RACELINK_EPAPER_RST=...
        -D RACELINK_EPAPER_BUSY=...

    - WLED_RELEASE_NAME is printed on the start screen (fallback \"WLED\").
*/

void epaperInit();
void setDisplayLayout(uint8_t numPilots);
bool setPilotSlotData(const char* nickname, const char* raceLabel, uint8_t slot);
void service_epaper();

// Backwards-compatible name (kept to avoid breaking older sketches)
inline bool renderPilotScreen(const char* nickname, const char* raceLabel, uint8_t slot)
{
  return setPilotSlotData(nickname, raceLabel, slot);
}
