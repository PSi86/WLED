#ifdef RACELINK_EPAPER

#include "racelink_epaper.h"
#include <SPI.h>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>

// base class GxEPD2_GFX can be used to pass references or pointers to the display instance as parameter, uses ~1.2k more code
#define ENABLE_GxEPD2_GFX 0
#include <GxEPD2_BW.h>

// Fonts (Adafruit GFX fonts)
#include <Fonts/FreeSansBold24pt7b.h>
#include <Fonts/FreeSansBold18pt7b.h>
#include <Fonts/FreeSansBold12pt7b.h>
#include <Fonts/FreeMonoBold9pt7b.h>

// Pin assignments are now runtime parameters supplied via epaperInit().
// The old compile-time defines (RACELINK_EPAPER_MOSI etc.) live in
// racelink_wled.h as defaults that seed the UsermodRaceLink::epd* members.
// At static-init time we instantiate the GxEPD2 display with -1 placeholder
// pins (safe — GxEPD2_EPD's constructor only stores pin numbers, no GPIO
// activation happens until display.init() runs). The worker task writes
// the real pin numbers via display.epd2.setPins() before calling init().

// -----------------------------
// Deferred refresh behavior
// -----------------------------
#ifndef RACELINK_EPAPER_MIN_DEFER_MS
  #define RACELINK_EPAPER_MIN_DEFER_MS 1500 // Defer refresh for at least this time to allow multiple updates to come in and be coalesced into a single refresh. Adjust based on your expected update frequency and latency requirements
#endif

#ifndef RACELINK_EPAPER_MAX_DEFER_MS
  #define RACELINK_EPAPER_MAX_DEFER_MS 4000 // If updates keep coming, still perform a refresh after this max deferral
#endif

#ifndef RACELINK_EPAPER_MIN_REFRESH_INTERVAL_MS
  #define RACELINK_EPAPER_MIN_REFRESH_INTERVAL_MS 10000 // Safety: don't full-refresh too frequently.
#endif

// Number of full-screen partial refreshes before a full refresh is enforced.
#ifndef RACELINK_EPAPER_PARTIAL_REFRESH_LIMIT
  #define RACELINK_EPAPER_PARTIAL_REFRESH_LIMIT 5
#endif

// Force a full refresh if no full refresh happened within this interval (6 min).
#ifndef RACELINK_EPAPER_PERIODIC_FULL_REFRESH_MS
  #define RACELINK_EPAPER_PERIODIC_FULL_REFRESH_MS 360000
#endif

// Optional periodic maintenance refresh (disabled by default).
// Set to e.g. 600000 (10min) if you notice ghosting over long runtimes.
#ifndef RACELINK_EPAPER_MAINTENANCE_REFRESH_MS
  #define RACELINK_EPAPER_MAINTENANCE_REFRESH_MS 0
#endif

// Hibernate between refreshes (saves power). If you run into wake issues, set to 0.
#ifndef RACELINK_EPAPER_USE_HIBERNATE
  #define RACELINK_EPAPER_USE_HIBERNATE 1
#endif

// -----------------------------
// Async worker tuning
// -----------------------------
// Stack must accommodate Adafruit GFX + GxEPD2 paging buffer access + four embedded fonts.
#ifndef RACELINK_EPAPER_TASK_STACK
  #define RACELINK_EPAPER_TASK_STACK 8192
#endif

// Match Arduino loop priority (1) so the single-core S2 time-slices fairly.
// On dual-core S3 the task is pinned to core 0 and runs in parallel anyway.
#ifndef RACELINK_EPAPER_TASK_PRIORITY
  #define RACELINK_EPAPER_TASK_PRIORITY 1
#endif

// Dedicated SPI bus for ePaper (keep RaceLink on default SPI)
#if defined(HSPI)
static SPIClass epdSPI(HSPI);
#else
static SPIClass epdSPI;
#endif

/* 2.9'' EPD Module (B/W), DEPG0290BS 128x296, SSD1680
GxEPD2_290_BS.h, GxEPD2_290_BS.cpp: no changes */
// (legacy alternative panel reference — left here as a guide if the
// hardware ever switches back; keep in sync with the configurable wrapper
// pattern below if uncommented.)


/* 3.7'' EPD Module, GDEY037T03 240x416, UC8253
Issue: partial update (fast / normal) creates garbage display.
GxEPD2_370_GDEY037T03.cpp: no changes
GxEPD2_370_GDEY037T03.h:
    static const bool hasFastPartialUpdate = false; // set this false to force full refresh always
    static const bool useFastFullUpdate = false; // set false for extended (low) temperature range, 1005000us vs 2950000us
*/

// GxEPD2_EPD's _cs/_dc/_rst/_busy are protected: derive a thin wrapper that
// inherits the constructor and exposes a setPins() method so we can apply
// runtime-configured pin numbers before display.init() is called. This avoids
// reaching into protected members via static_cast tricks (which would be UB).
class GxEPD2_370_GDEY037T03_Configurable : public GxEPD2_370_GDEY037T03
{
public:
  using GxEPD2_370_GDEY037T03::GxEPD2_370_GDEY037T03;   // inherit constructors
  void setPins(int16_t cs, int16_t dc, int16_t rst, int16_t busy)
  {
    _cs   = cs;
    _dc   = dc;
    _rst  = rst;
    _busy = busy;
  }
};

// Instantiate with -1 placeholder pins. The real pin numbers are written by
// epaperInit() into display.epd2.setPins() *before* the worker task calls
// display.init(); this is safe because GxEPD2_EPD's constructor only stores
// the pin numbers — no pinMode / digitalWrite happens until init() runs.
static GxEPD2_BW<GxEPD2_370_GDEY037T03_Configurable, GxEPD2_370_GDEY037T03_Configurable::HEIGHT> display(
 GxEPD2_370_GDEY037T03_Configurable(/*CS=*/ -1, /*DC=*/ -1, /*RES=*/ -1, /*BUSY=*/ -1)
);

// -----------------------------
// State
// -----------------------------
// Pilot data: written by main (under mutex), snapshotted by worker (under mutex).
static uint8_t g_numPilots = 1;
static char g_nick[8][21];   // max 20 chars + NUL
static char g_label[8][3];   // max 2 chars + NUL

// Cross-thread status flags. `volatile` because the writer and reader live on
// different FreeRTOS tasks (and on S3, different cores).
static volatile bool g_initialized = false; // worker sets after first init+render; main reads in service_epaper
static bool g_hibernated = false;            // worker-only state
static bool g_hasPilotData = false;          // main-only state

// Deferred-refresh bookkeeping (main-thread-only).
static bool g_refreshPending = false;
static uint32_t g_lastNewDataMs = 0;
static uint32_t g_firstNewDataMs = 0;

// Refresh timestamps/counters: written by worker after each refresh, read by main in service_epaper.
static volatile uint32_t g_lastRefreshMs = 0;
static volatile uint32_t g_lastFullRefreshMs = 0;
static volatile uint8_t g_partialRefreshCount = 0;

// -----------------------------
// Async worker plumbing
// -----------------------------
static TaskHandle_t      g_epdTaskHandle = nullptr;
static SemaphoreHandle_t g_epdWake       = nullptr; // binary, given by main, taken by worker
static SemaphoreHandle_t g_epdDataMutex  = nullptr; // protects g_nick / g_label / g_numPilots

static volatile bool g_epdReqInit    = false; // request: run SPI begin + display.init + start screen
static volatile bool g_epdReqRefresh = false; // request: re-render with current snapshot
static volatile bool g_epdReqFull    = false; // refresh kind selected by main at dispatch time
static volatile bool g_epdBusy       = false; // worker is currently mid-init or mid-render

// -----------------------------
// Pin assignments (set by epaperInit, read by the worker task)
// -----------------------------
static int8_t g_epdSck  = -1;
static int8_t g_epdMiso = -1;
static int8_t g_epdMosi = -1;
static int8_t g_epdCs   = -1;
static int8_t g_epdDc   = -1;
static int8_t g_epdRst  = -1;
static int8_t g_epdBusy_pin = -1;   // suffix avoids clash with g_epdBusy (busy-flag) above

// -----------------------------
// Helpers
// -----------------------------
static void toUpperAscii(const char* in, char* out, size_t outSize)
{
  if (!out || outSize == 0) return;
  size_t i = 0;
  for (; in && in[i] && (i + 1) < outSize; i++)
  {
    char c = in[i];
    if (c >= 'a' && c <= 'z') c = char(c - 'a' + 'A');
    out[i] = c;
  }
  out[i] = '\0';
}

static void safeCopyTrunc(const char* in, char* out, size_t outSize)
{
  if (!out || outSize == 0) return;
  if (!in) { out[0] = '\0'; return; }
  size_t i = 0;
  for (; in[i] && (i + 1) < outSize; i++) out[i] = in[i];
  out[i] = '\0';
}

static const GFXfont* pickFontFit(const char* txt, uint16_t maxWidth, uint16_t maxHeight)
{
  const GFXfont* fonts[] = {
    &FreeSansBold24pt7b,
    &FreeSansBold18pt7b,
    &FreeSansBold12pt7b,
    &FreeMonoBold9pt7b,
  };

  int16_t tbx, tby;
  uint16_t tbw, tbh;

  for (auto f : fonts)
  {
    display.setFont(f);
    display.getTextBounds(txt, 0, 0, &tbx, &tby, &tbw, &tbh);
    if (tbw <= maxWidth && tbh <= maxHeight) return f;
  }
  return &FreeMonoBold9pt7b;
}

static const GFXfont* pickLabelFont(uint16_t rowHeight)
{
  const GFXfont* fonts[] = {
    &FreeSansBold24pt7b,
    &FreeSansBold18pt7b,
    &FreeSansBold12pt7b,
    &FreeMonoBold9pt7b,
  };

  const char* sample = "WW";
  int16_t tbx, tby;
  uint16_t tbw, tbh;

  for (auto f : fonts)
  {
    display.setFont(f);
    display.getTextBounds(sample, 0, 0, &tbx, &tby, &tbw, &tbh);
    if (tbh + 6 <= rowHeight) return f;
  }
  return &FreeMonoBold9pt7b;
}

static uint16_t computeBarWidth(const GFXfont* labelFont, uint16_t displayW)
{
  display.setFont(labelFont);
  int16_t tbx, tby;
  uint16_t tbw, tbh;
  display.getTextBounds("WW", 0, 0, &tbx, &tby, &tbw, &tbh);

  const uint16_t padX = 6;
  uint16_t barW = tbw + 2 * padX;

  if (barW < 56) barW = 56;
  if (barW > (displayW * 3) / 5) barW = (displayW * 3) / 5;
  return barW;
}

static void drawCenteredText(const char* txt, const GFXfont* font, int16_t areaX, int16_t areaY, uint16_t areaW, uint16_t areaH, uint16_t color)
{
  display.setFont(font);
  display.setTextColor(color);

  int16_t tbx, tby;
  uint16_t tbw, tbh;
  display.getTextBounds(txt, 0, 0, &tbx, &tby, &tbw, &tbh);

  int16_t x = areaX + int16_t((areaW - tbw) / 2) - tbx;
  int16_t y = areaY + int16_t(areaH / 2) - int16_t(tbh / 2) - tby;

  display.setCursor(x, y);
  display.print(txt);
}

// -----------------------------
// Rendering (worker-context only)
// -----------------------------
// Render functions read from caller-supplied snapshot buffers, never the live
// g_nick / g_label / g_numPilots — so updates from the main thread cannot tear
// the picture mid-render.
static void renderStartScreen()
{
  display.setRotation(1); // landscape
  const uint16_t W = display.width();
  const uint16_t H = display.height();

  display.setFullWindow();
  display.firstPage();
  do
  {
    display.fillScreen(GxEPD_WHITE);

    drawCenteredText("RaceLink Startblock", pickFontFit("RaceLink Startblock", W - 8, 30),
                     0, 0, W, H / 2, GxEPD_BLACK);

    drawCenteredText(WLED_RELEASE_NAME, pickFontFit(WLED_RELEASE_NAME, W - 8, (H / 3) - 4),
                     0, H / 2, W, H / 2, GxEPD_BLACK);
  }
  while (display.nextPage());
}

static void renderLayout1(bool fullRefresh, const char* nickUpper, const char* labelUpper)
{
  display.setRotation(1);
  const uint16_t W = display.width();
  const uint16_t H = display.height();

  const GFXfont* labelFont = &FreeSansBold24pt7b;
  {
    int16_t tbx, tby; uint16_t tbw, tbh;
    display.setFont(labelFont);
    display.getTextBounds("WW", 0, 0, &tbx, &tby, &tbw, &tbh);
    if (tbh + 10 > H) labelFont = &FreeSansBold18pt7b;
  }

  const uint16_t barW = computeBarWidth(labelFont, W);
  const uint16_t barX = W - barW;
  const uint16_t leftW = W - barW;

  const uint16_t leftPadX = 5;
  const uint16_t nickMaxW = (leftW > (2 * leftPadX)) ? (leftW - 2 * leftPadX) : leftW;
  const uint16_t nickMaxH = H - 8;

  const GFXfont* nickFont = pickFontFit(nickUpper, nickMaxW, nickMaxH);

  if (fullRefresh) display.setFullWindow();
  else display.setPartialWindow(0, 0, W, H);
  display.firstPage();
  do
  {
    display.fillScreen(GxEPD_WHITE);
    display.fillRect(barX, 0, barW, H, GxEPD_BLACK);

    // nickname
    display.setFont(nickFont);
    display.setTextColor(GxEPD_BLACK);

    int16_t tbx, tby; uint16_t tbw, tbh;
    display.getTextBounds(nickUpper, 0, 0, &tbx, &tby, &tbw, &tbh);

    int16_t nickX = int16_t(leftPadX) - tbx;
    int16_t nickY = int16_t(H / 2) - int16_t(tbh / 2) - tby;

    display.setCursor(nickX, nickY);
    display.print(nickUpper);

    // label
    display.setFont(labelFont);
    display.setTextColor(GxEPD_WHITE);

    display.getTextBounds(labelUpper, 0, 0, &tbx, &tby, &tbw, &tbh);
    int16_t rX = int16_t(barX) + int16_t((barW - tbw) / 2) - tbx;
    int16_t rY = int16_t(H / 2) - int16_t(tbh / 2) - tby;

    display.setCursor(rX, rY);
    display.print(labelUpper);
  }
  while (display.nextPage());
}

static void renderLayoutMulti(bool fullRefresh, uint8_t n, const char nick[8][21], const char lbl[8][3])
{
  display.setRotation(1);
  const uint16_t W = display.width();
  const uint16_t H = display.height();

  if (n < 1) n = 1;
  if (n > 8) n = 8;
  const uint16_t rowH = H / n;

  const GFXfont* labelFont = pickLabelFont(rowH);
  const uint16_t barW = computeBarWidth(labelFont, W);
  const uint16_t barX = W - barW;
  const uint16_t leftW = W - barW;

  const uint16_t leftPadX = 6;
  const uint16_t nickMaxW = (leftW > (2 * leftPadX)) ? (leftW - 2 * leftPadX) : leftW;
  const uint16_t nickMaxH = rowH - 6;

  if (fullRefresh) display.setFullWindow();
  else display.setPartialWindow(0, 0, W, H);
  display.firstPage();
  do
  {
    display.fillScreen(GxEPD_WHITE);

    for (uint8_t i = 0; i < n; i++)
    {
      const uint16_t y0 = uint16_t(i) * rowH;
      if (i > 0) display.drawLine(0, y0, W - 1, y0, GxEPD_BLACK);

      display.fillRect(barX, y0, barW, rowH, GxEPD_BLACK);

      // nickname
      const char* nickUpper = nick[i];
      const GFXfont* nickFont = pickFontFit(nickUpper, nickMaxW, nickMaxH);

      display.setFont(nickFont);
      display.setTextColor(GxEPD_BLACK);

      int16_t tbx, tby; uint16_t tbw, tbh;
      display.getTextBounds(nickUpper, 0, 0, &tbx, &tby, &tbw, &tbh);

      int16_t nickX = int16_t(leftPadX) - tbx;
      int16_t nickY = int16_t(y0 + rowH / 2) - int16_t(tbh / 2) - tby;

      display.setCursor(nickX, nickY);
      display.print(nickUpper);

      // label
      display.setFont(labelFont);
      display.setTextColor(GxEPD_WHITE);

      display.getTextBounds(lbl[i], 0, 0, &tbx, &tby, &tbw, &tbh);
      int16_t rX = int16_t(barX) + int16_t((barW - tbw) / 2) - tbx;
      int16_t rY = int16_t(y0 + rowH / 2) - int16_t(tbh / 2) - tby;

      display.setCursor(rX, rY);
      display.print(lbl[i]);
    }
  }
  while (display.nextPage());
}

static void renderAll(bool fullRefresh, uint8_t n, const char nick[8][21], const char lbl[8][3])
{
  if (n <= 1) renderLayout1(fullRefresh, nick[0], lbl[0]);
  else        renderLayoutMulti(fullRefresh, n, nick, lbl);
}

static void wakeIfNeeded()
{
  if (!g_initialized) return;
  if (!g_hibernated) return;

  // Wake the panel without a full reset sequence
  display.init(115200, false, 50, false);
  g_hibernated = false;
}

static void maybeHibernate()
{
#if RACELINK_EPAPER_USE_HIBERNATE
  display.hibernate();
  g_hibernated = true;
#else
  g_hibernated = false;
#endif
}

static void scheduleDeferredRefresh()
{
  const uint32_t now = millis();
  if (!g_refreshPending) g_firstNewDataMs = now; // first new-data event of the current defer window
  g_lastNewDataMs = now;
  g_refreshPending = true;
}

// -----------------------------
// Worker task
// -----------------------------
// Owns ALL access to the GxEPD2 `display` object. Wakes on g_epdWake, processes
// any pending init or refresh request, then blocks again. The wake-semaphore is
// binary, so multiple gives during a single render coalesce into one extra cycle.
static void epaperTask(void* /*arg*/)
{
  for (;;)
  {
    if (xSemaphoreTake(g_epdWake, portMAX_DELAY) != pdTRUE) continue;

    if (g_epdReqInit)
    {
      g_epdReqInit = false;
      g_epdBusy = true;

      // Apply runtime-configured pins to the GxEPD2 display before init().
      // Constructor pins were -1 sentinels; setPins() updates the protected
      // _cs/_dc/_rst/_busy members so display.init() will pinMode the right
      // GPIOs.
      display.epd2.setPins(g_epdCs, g_epdDc, g_epdRst, g_epdBusy_pin);

      epdSPI.begin(g_epdSck, g_epdMiso, g_epdMosi, g_epdCs);
      display.epd2.selectSPI(epdSPI, SPISettings(4000000, MSBFIRST, SPI_MODE0));
      display.init(115200, false, 50, false); // reset duration was 50
      g_hibernated = false;

      renderStartScreen();
      const uint32_t now = millis();
      g_lastRefreshMs = now;
      g_lastFullRefreshMs = now;
      g_partialRefreshCount = 0;
      maybeHibernate();

      g_initialized = true; // gate must close only after the panel has shown the boot screen
      g_epdBusy = false;
    }

    if (g_epdReqRefresh)
    {
      g_epdReqRefresh = false;
      g_epdBusy = true;
      const bool full = g_epdReqFull;

      // Snapshot pilot data under the mutex so the main thread can keep updating g_nick/g_label freely.
      uint8_t n = 1;
      char nick[8][21];
      char lbl[8][3];
      if (g_epdDataMutex && xSemaphoreTake(g_epdDataMutex, portMAX_DELAY) == pdTRUE)
      {
        n = g_numPilots;
        memcpy(nick, g_nick, sizeof(nick));
        memcpy(lbl,  g_label, sizeof(lbl));
        xSemaphoreGive(g_epdDataMutex);
      }
      else
      {
        // Mutex unavailable (shouldn't happen post-init) — fall back to direct read.
        n = g_numPilots;
        memcpy(nick, g_nick, sizeof(nick));
        memcpy(lbl,  g_label, sizeof(lbl));
      }

      wakeIfNeeded();
      renderAll(full, n, nick, lbl);
      maybeHibernate();

      const uint32_t now = millis();
      g_lastRefreshMs = now;
      if (full)
      {
        g_lastFullRefreshMs = now;
        g_partialRefreshCount = 0;
      }
      else if (g_partialRefreshCount < 255)
      {
        g_partialRefreshCount++;
      }

      g_epdBusy = false;
    }
  }
}

// -----------------------------
// Public API (non-blocking)
// -----------------------------
void epaperInit(int8_t mosi, int8_t sck, int8_t miso,
                int8_t cs,   int8_t dc,  int8_t rst, int8_t busy)
{
  // Stash the pin assignments for the worker task. setPins() runs on the
  // worker before display.init(), so reading these from the main thread is
  // safe — they're written here once and never change for the lifetime of
  // the device (UI changes trigger a reboot).
  g_epdSck      = sck;
  g_epdMiso     = miso;
  g_epdMosi     = mosi;
  g_epdCs       = cs;
  g_epdDc       = dc;
  g_epdRst      = rst;
  g_epdBusy_pin = busy;

  // Initialize main-thread state before the worker can observe it.
  for (uint8_t i = 0; i < 8; i++)
  {
    g_nick[i][0]  = '\0';
    g_label[i][0] = '\0';
  }
  g_numPilots = 1;
  g_hasPilotData = false;
  g_refreshPending = false;
  g_lastNewDataMs = 0;
  g_firstNewDataMs = 0;
  g_lastRefreshMs = 0;
  g_lastFullRefreshMs = 0;
  g_partialRefreshCount = 0;
  g_hibernated = false;
  g_initialized = false;
  g_epdBusy = false;

  if (!g_epdDataMutex) g_epdDataMutex = xSemaphoreCreateMutex();
  if (!g_epdWake)      g_epdWake      = xSemaphoreCreateBinary();

  if (g_epdTaskHandle == nullptr)
  {
  #if defined(CONFIG_IDF_TARGET_ESP32S3)
    const BaseType_t coreId = 0;          // S3: pin worker to core 0; Arduino loop owns core 1.
  #else
    const BaseType_t coreId = tskNO_AFFINITY; // S2 (single core) and others.
  #endif
    xTaskCreatePinnedToCore(epaperTask,
                            "epaper",
                            RACELINK_EPAPER_TASK_STACK,
                            nullptr,
                            RACELINK_EPAPER_TASK_PRIORITY,
                            &g_epdTaskHandle,
                            coreId);
  }

  // Tell the worker to do the heavy SPI/init/start-screen work, then return immediately.
  g_epdReqInit = true;
  if (g_epdWake) xSemaphoreGive(g_epdWake);
}

void setDisplayLayout(uint8_t numPilots)
{
  if (numPilots < 1) numPilots = 1;
  if (numPilots > 8) numPilots = 8;

  if (g_epdDataMutex && xSemaphoreTake(g_epdDataMutex, portMAX_DELAY) == pdTRUE)
  {
    g_numPilots = numPilots;
    for (uint8_t i = numPilots; i < 8; i++)
    {
      g_nick[i][0]  = '\0';
      g_label[i][0] = '\0';
    }
    xSemaphoreGive(g_epdDataMutex);
  }
  else
  {
    g_numPilots = numPilots;
    for (uint8_t i = numPilots; i < 8; i++)
    {
      g_nick[i][0]  = '\0';
      g_label[i][0] = '\0';
    }
  }

  // If we already have pilot data, schedule a refresh so the new layout becomes visible
  if (g_hasPilotData) scheduleDeferredRefresh();
}

bool setPilotSlotData(const char* nickname, const char* raceLabel, uint8_t slot)
{
  if (slot < 1 || slot > 8) return false;

  bool accepted = false;
  if (g_epdDataMutex && xSemaphoreTake(g_epdDataMutex, portMAX_DELAY) == pdTRUE)
  {
    if (slot <= g_numPilots)
    {
      const uint8_t idx = slot - 1;

      // nickname: max 20 chars, stored uppercase
      char tmpNick[21];
      safeCopyTrunc(nickname, tmpNick, sizeof(tmpNick));
      toUpperAscii(tmpNick, g_nick[idx], sizeof(g_nick[idx]));

      // label: max 2 chars, stored uppercase
      char tmpLbl[3];
      safeCopyTrunc(raceLabel, tmpLbl, sizeof(tmpLbl));
      toUpperAscii(tmpLbl, g_label[idx], sizeof(g_label[idx]));

      accepted = true;
    }
    xSemaphoreGive(g_epdDataMutex);
  }

  if (!accepted) return false;

  g_hasPilotData = true;
  scheduleDeferredRefresh();
  return true;
}

void service_epaper()
{
  if (!g_initialized) return;
  if (g_epdBusy) return; // worker is still rendering; try again next tick

  const uint32_t now = millis();

  const bool minOk = (uint32_t)(now - g_lastRefreshMs) >= (uint32_t)RACELINK_EPAPER_MIN_REFRESH_INTERVAL_MS;
  if (!minOk) return; // enforce minimum interval between refreshes to prevent issues on some panels when refreshing too frequently

  if (!g_hasPilotData) return; // no data yet, nothing to do

  // Deferred refresh after updates
  if (g_refreshPending)
  {
    // Due to the complexity of ePaper refresh timing and the wide variety of panels out there, we use a simple time-based heuristic to decide when to refresh after receiving updates:
    // dueByDelay: at least RACELINK_EPAPER_MIN_DEFER_MS has passed since the last received update command
    // dueByMax: at least RACELINK_EPAPER_MAX_DEFER_MS has passed since the first received update command (prevents starvation if updates keep coming in)
    // minOk: at least RACELINK_EPAPER_MIN_REFRESH_INTERVAL_MS has passed since the last refresh (full or partial)
    // periodicFullDue: if RACELINK_EPAPER_PERIODIC_FULL_REFRESH_MS is set, a full refresh is due if at least that time has passed since the last full refresh (enforces periodic full refreshes to reduce ghosting, even if updates are infrequent)
    // fullRefreshDue: if either periodicFullDue is true or the number of partial refreshes since the last full refresh has reached RACELINK_EPAPER_PARTIAL_REFRESH_LIMIT, a full refresh is due; otherwise, a partial refresh is due

    const bool dueByDelay = (uint32_t)(now - g_lastNewDataMs) >= (uint32_t)RACELINK_EPAPER_MIN_DEFER_MS;
    const bool dueByMax   = (uint32_t)(now - g_firstNewDataMs) >= (uint32_t)RACELINK_EPAPER_MAX_DEFER_MS;

    const bool periodicFullDue = (RACELINK_EPAPER_PERIODIC_FULL_REFRESH_MS > 0) &&
      ((uint32_t)(now - g_lastFullRefreshMs) >= (uint32_t)RACELINK_EPAPER_PERIODIC_FULL_REFRESH_MS);
    const bool fullRefreshDue = periodicFullDue || (g_partialRefreshCount >= RACELINK_EPAPER_PARTIAL_REFRESH_LIMIT);

    if (dueByDelay || dueByMax)
    {
      g_epdReqFull = fullRefreshDue;
      g_epdReqRefresh = true;
      g_refreshPending = false;
      if (g_epdWake) xSemaphoreGive(g_epdWake);
    }
  }
  else
  {
    // Optional periodic maintenance refresh to reduce ghosting over very long runtimes
    if (RACELINK_EPAPER_PERIODIC_FULL_REFRESH_MS > 0)
    {
      if ((uint32_t)(now - g_lastFullRefreshMs) >= (uint32_t)RACELINK_EPAPER_PERIODIC_FULL_REFRESH_MS)
      {
        g_epdReqFull = true;
        g_epdReqRefresh = true;
        if (g_epdWake) xSemaphoreGive(g_epdWake);
      }
    }
    else if (RACELINK_EPAPER_MAINTENANCE_REFRESH_MS > 0)
    {
      if ((uint32_t)(now - g_lastRefreshMs) >= (uint32_t)RACELINK_EPAPER_MAINTENANCE_REFRESH_MS)
      {
        g_epdReqFull = true;
        g_epdReqRefresh = true;
        if (g_epdWake) xSemaphoreGive(g_epdWake);
      }
    }
  }
}

#endif
