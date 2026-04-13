#ifdef RACELINK_EPAPER

#include "racelink_epaper.h"
#include <SPI.h>

// base class GxEPD2_GFX can be used to pass references or pointers to the display instance as parameter, uses ~1.2k more code
#define ENABLE_GxEPD2_GFX 0
#include <GxEPD2_BW.h>

// Fonts (Adafruit GFX fonts)
#include <Fonts/FreeSansBold24pt7b.h>
#include <Fonts/FreeSansBold18pt7b.h>
#include <Fonts/FreeSansBold12pt7b.h>
#include <Fonts/FreeMonoBold9pt7b.h>

// -----------------------------
// Pin defaults (override via PlatformIO build_flags: -D ...)
// -----------------------------
#ifndef RACELINK_EPAPER_CS
  #define RACELINK_EPAPER_CS   10
#endif
#ifndef RACELINK_EPAPER_BUSY
  #define RACELINK_EPAPER_BUSY 3
#endif
#ifndef RACELINK_EPAPER_RST
  #define RACELINK_EPAPER_RST  46
#endif
#ifndef RACELINK_EPAPER_DC
  #define RACELINK_EPAPER_DC   9
#endif

// Configurable SPI pins (MISO is typically not used by ePaper modules)
#ifndef RACELINK_EPAPER_MOSI
  #define RACELINK_EPAPER_MOSI 11
#endif
#ifndef RACELINK_EPAPER_SCK
  #define RACELINK_EPAPER_SCK  12
#endif
#ifndef RACELINK_EPAPER_MISO
  #define RACELINK_EPAPER_MISO -1
#endif

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

// Dedicated SPI bus for ePaper (keep RaceLink on default SPI)
#if defined(HSPI)
static SPIClass epdSPI(HSPI);
#else
static SPIClass epdSPI;
#endif

/* 2.9'' EPD Module (B/W), DEPG0290BS 128x296, SSD1680
GxEPD2_290_BS.h, GxEPD2_290_BS.cpp: no changes */
// static GxEPD2_BW<GxEPD2_290_BS, GxEPD2_290_BS::HEIGHT> display(
//   GxEPD2_290_BS(/*CS=*/ RACELINK_EPAPER_CS, /*DC=*/ RACELINK_EPAPER_DC, /*RES=*/ RACELINK_EPAPER_RST, /*BUSY=*/ RACELINK_EPAPER_BUSY)
// );


/* 3.7'' EPD Module, GDEY037T03 240x416, UC8253
Issue: partial update (fast / normal) creates garbage display.
GxEPD2_370_GDEY037T03.cpp: no changes
GxEPD2_370_GDEY037T03.h:
    static const bool hasFastPartialUpdate = false; // set this false to force full refresh always
    static const bool useFastFullUpdate = false; // set false for extended (low) temperature range, 1005000us vs 2950000us
*/
static GxEPD2_BW<GxEPD2_370_GDEY037T03, GxEPD2_370_GDEY037T03::HEIGHT> display(
 GxEPD2_370_GDEY037T03(/*CS=5*/ RACELINK_EPAPER_CS, /*DC=*/ RACELINK_EPAPER_DC, /*RES=*/ RACELINK_EPAPER_RST, /*BUSY=*/ RACELINK_EPAPER_BUSY)
);

// -----------------------------
// State
// -----------------------------
static uint8_t g_numPilots = 1;

static char g_nick[8][21];   // max 20 chars + NUL
static char g_label[8][3];   // max 2 chars + NUL

static bool g_initialized = false; // true once epaperInit() was called
static bool g_hibernated = false; // true if display is currently hibernated
static bool g_hasPilotData = false; // true once at least one pilot slot has received data

static bool g_refreshPending = false; // set to true to request a deferred refresh executed by service_epaper()
static uint32_t g_lastNewDataMs = 0;  // last received update command
static uint32_t g_firstNewDataMs = 0; // first update command since last refresh (used to enforce max defer time)
static uint32_t g_lastRefreshMs = 0; // last refresh (full or partial)
static uint32_t g_lastFullRefreshMs = 0; // last full refresh (used to enforce periodic full refreshes)
static uint8_t g_partialRefreshCount = 0; // number of partial refreshes since last full refresh (used to enforce full refresh after too many partial refreshes)

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
// Rendering
// -----------------------------
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

    drawCenteredText("GateControl Startblock", pickFontFit("GateControl Startblock", W - 8, 30),
                     0, 0, W, H / 2, GxEPD_BLACK);

    drawCenteredText(WLED_RELEASE_NAME, pickFontFit(WLED_RELEASE_NAME, W - 8, (H / 3) - 4),
                     0, H / 2, W, H / 2, GxEPD_BLACK);
  }
  while (display.nextPage());
}

static void renderLayout1(bool fullRefresh)
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

  const char* nickUpper = g_nick[0];
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

    display.getTextBounds(g_label[0], 0, 0, &tbx, &tby, &tbw, &tbh);
    int16_t rX = int16_t(barX) + int16_t((barW - tbw) / 2) - tbx;
    int16_t rY = int16_t(H / 2) - int16_t(tbh / 2) - tby;

    display.setCursor(rX, rY);
    display.print(g_label[0]);
  }
  while (display.nextPage());
}

static void renderLayoutMulti(bool fullRefresh)
{
  display.setRotation(1);
  const uint16_t W = display.width();
  const uint16_t H = display.height();

  const uint8_t n = g_numPilots;
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
      const char* nickUpper = g_nick[i];
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

      display.getTextBounds(g_label[i], 0, 0, &tbx, &tby, &tbw, &tbh);
      int16_t rX = int16_t(barX) + int16_t((barW - tbw) / 2) - tbx;
      int16_t rY = int16_t(y0 + rowH / 2) - int16_t(tbh / 2) - tby;

      display.setCursor(rX, rY);
      display.print(g_label[i]);
    }
  }
  while (display.nextPage());
}

static void renderAll(bool fullRefresh)
{
  if (g_numPilots == 1) renderLayout1(fullRefresh);
  else renderLayoutMulti(fullRefresh);
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
  if (!g_refreshPending) g_firstNewDataMs = now; // if no refresh pending, this is the first new data; otherwise, keep the original firstNewDataMs to enforce max defer time correctly
  g_lastNewDataMs = now; // always update lastNewDataMs to enable dueByDelay calculation
  g_refreshPending = true;
}

static void performRefresh(bool fullRefresh)
{
  wakeIfNeeded();
  renderAll(fullRefresh);
  const uint32_t now = millis();
  g_lastRefreshMs = now;
  if (fullRefresh)
  {
    g_lastFullRefreshMs = now;
    g_partialRefreshCount = 0;
  }
  else if (g_partialRefreshCount < 255)
  {
    g_partialRefreshCount++;
  }
  maybeHibernate();
}

// -----------------------------
// Public API
// -----------------------------
void epaperInit()
{
  epdSPI.begin(RACELINK_EPAPER_SCK, RACELINK_EPAPER_MISO, RACELINK_EPAPER_MOSI, RACELINK_EPAPER_CS);
  display.epd2.selectSPI(epdSPI, SPISettings(4000000, MSBFIRST, SPI_MODE0));

  display.init(115200, false, 50, false); // reset duration was 50
  g_initialized = true;
  g_hibernated = false;

  for (uint8_t i = 0; i < 8; i++)
  {
    g_nick[i][0]  = '\0';
    g_label[i][0] = '\0';
  }
  g_numPilots = 1;

  renderStartScreen();
  g_lastRefreshMs = millis();
  g_lastFullRefreshMs = g_lastRefreshMs;
  g_partialRefreshCount = 0;
  maybeHibernate();
}

void setDisplayLayout(uint8_t numPilots)
{
  if (numPilots < 1) numPilots = 1;
  if (numPilots > 8) numPilots = 8;

  g_numPilots = numPilots;

  for (uint8_t i = numPilots; i < 8; i++)
  {
    g_nick[i][0]  = '\0';
    g_label[i][0] = '\0';
  }

  // If we already have pilot data, schedule a refresh so the new layout becomes visible
  if (g_hasPilotData) scheduleDeferredRefresh();
}

bool setPilotSlotData(const char* nickname, const char* raceLabel, uint8_t slot)
{
  if (slot < 1 || slot > 8) return false;
  if (slot > g_numPilots) return false;

  const uint8_t idx = slot - 1;

  // nickname: max 20 chars, stored uppercase
  char tmpNick[21];
  safeCopyTrunc(nickname, tmpNick, sizeof(tmpNick));
  toUpperAscii(tmpNick, g_nick[idx], sizeof(g_nick[idx]));

  // label: max 2 chars, stored uppercase
  char tmpLbl[3];
  safeCopyTrunc(raceLabel, tmpLbl, sizeof(tmpLbl));
  toUpperAscii(tmpLbl, g_label[idx], sizeof(g_label[idx]));

  g_hasPilotData = true;
  scheduleDeferredRefresh();
  return true;
}

void service_epaper()
{
  if (!g_initialized) return;

  const uint32_t now = millis();

  const bool minOk = (uint32_t)(now - g_lastRefreshMs) >= (uint32_t)RACELINK_EPAPER_MIN_REFRESH_INTERVAL_MS; // checked: good
  if (!minOk) return; // enforce minimum interval between refreshes to prevent issues on some panels when refreshing too frequently

  if(!g_hasPilotData) return; // no data yet, nothing to do

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
      performRefresh(fullRefreshDue);
      g_refreshPending = false;
    }
  }
  else
  {
    // Optional periodic maintenance refresh to reduce ghosting over very long runtimes
    if (RACELINK_EPAPER_PERIODIC_FULL_REFRESH_MS > 0)
    {
      if ((uint32_t)(now - g_lastFullRefreshMs) >= (uint32_t)RACELINK_EPAPER_PERIODIC_FULL_REFRESH_MS)
      {
        performRefresh(true);
      }
    }
    else if (RACELINK_EPAPER_MAINTENANCE_REFRESH_MS > 0)
    {
      if ((uint32_t)(now - g_lastRefreshMs) >= (uint32_t)RACELINK_EPAPER_MAINTENANCE_REFRESH_MS)
      {
        performRefresh(true);
      }
    }
  }
}

#endif