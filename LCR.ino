// LCR.ino
//
// LCR Instrument
//
// This module implements the LCR / Impedance Analyzer instrument.
//
// Responsibilities:
//
//   Instrument initialization
//   Measurement backend selection
//   Measurement acquisition
//   Display updates
//   Instrument state management

#include "AD9833_Driver.h"

// LCR excitation-generator hardware.
// The AD9833 uses SPI0 independently from the TFT display bus.
static const uint8_t AD9833_FSYNC_PIN = 3;  // GP3, physical pin 5
static const uint8_t AD9833_SCK_PIN   = 6;  // GP6, physical pin 9
static const uint8_t AD9833_DATA_PIN  = 7;  // GP7, physical pin 10

// AD9833 excitation source for the LCR measurement hardware.
AD9833_Driver lcrDDS(AD9833_FSYNC_PIN);

// screen locations for various output displays
constexpr int LABEL_X = 20;
constexpr int VALUE_X = 120;
constexpr int FREQ_Y  = 40;
constexpr int Z_Y     = 60;
constexpr int PHASE_Y = 80;
constexpr int R_Y     = 100;
constexpr int X_Y     = 120;


//
// LCR UI layout tuning
//
// These values control proportional spacing and sizing throughout the
// responsive LCR interface. Values are percentages of the applicable
// screen or layout region so the UI can adapt to different resolutions.
//

constexpr int16_t LCR_HEADER_HEIGHT_PERCENT = 10;
constexpr int16_t LCR_BACK_BUTTON_WIDTH_PERCENT = 22;
constexpr int16_t LCR_TAB_HEIGHT_PERCENT = 10;
constexpr int16_t LCR_FOOTER_HEIGHT_PERCENT = 12;

constexpr int16_t LCR_PRIMARY_HEIGHT_PERCENT = 42;
constexpr int16_t LCR_SECONDARY_HEIGHT_PERCENT = 25;

constexpr int16_t LCR_SELECTOR_MARGIN_PERCENT = 6;
constexpr int16_t LCR_SELECTOR_GAP_X_PERCENT = 4;
constexpr int16_t LCR_SELECTOR_GAP_Y_PERCENT = 6;

constexpr int16_t LCR_SELECTOR_TOP_PERCENT = 22;
constexpr int16_t LCR_SELECTOR_BOTTOM_GAP_PERCENT = 6;

constexpr int16_t LCR_SELECTOR_BUTTON_HEIGHT_PERCENT = 13;

constexpr int16_t LCR_SELECTOR_CANCEL_WIDTH_PERCENT = 35;
constexpr int16_t LCR_SELECTOR_CANCEL_HEIGHT_PERCENT = 16;
constexpr int16_t LCR_SELECTOR_CANCEL_BOTTOM_PERCENT = 5;

// Height of the Sweep Results plot-control strip as a percentage of the
// content region. The strip remains large enough to be an obvious touch target.
constexpr int16_t LCR_SWEEP_PLOT_CONTROL_PERCENT = 15;
// Sweep Results plot margins within the common content region.
// Space outside the plot rectangle is reserved for axis labels.
constexpr int16_t LCR_SWEEP_PLOT_LEFT_PERCENT   = 16;
constexpr int16_t LCR_SWEEP_PLOT_RIGHT_PERCENT  = 5;
constexpr int16_t LCR_SWEEP_PLOT_TOP_PERCENT    = 8;
constexpr int16_t LCR_SWEEP_PLOT_BOTTOM_PERCENT = 12;

// Artificial interval between simulated Sweep measurements.
// This makes progress and Cancel behavior observable during development.
// Real hardware acquisition will determine its own measurement timing.
constexpr uint32_t LCR_SIM_SWEEP_POINT_INTERVAL_MS = 25;

// Controls the maximum refresh rate of dynamic LCR display values.
// Measurement acquisition may occur faster, but TFT updates are limited
// to reduce flicker and unnecessary SPI traffic.
constexpr uint32_t LCR_DISPLAY_INTERVAL_MS = 150;

// Stores the currently selected LCR analyzer tab
// Measure is always the default tab when entering the instrument
LCRTab lcrTab = LCR_TAB_MEASURE;

LCRBackend lcrBackend = LCR_BACKEND_SIMULATION;

// Indicates that the dynamic LCR display must be redrawn immediately.
// This is set whenever the analyzer is entered or its screen changes.
bool lcrDisplayDirty = true;

// Stores the calculated screen regions used by the LCR interface
LCRLayout lcrLayout;

// Stores the current operating state of the Measure tab.
// New LCR analyzer sessions begin in continuous LIVE measurement mode.
LCRMeasureState lcrMeasureState = LCR_MEASURE_LIVE;

// Stores the current Sweep-tab operating state.
// Entering the analyzer initializes Sweep in its configuration state.
LCRSweepState lcrSweepState = LCR_SWEEP_SETUP;

// Stores the quantity currently displayed on the Sweep Results graph.
// Impedance magnitude is the default Results view after startup.
LCRSweepPlotType lcrSweepPlotType = LCR_SWEEP_PLOT_IMPEDANCE;

// Stores the current Sweep Results inspection cursor.
// No point is selected when a new Results view is first displayed.
bool lcrSweepCursorActive = false;
uint16_t lcrSweepCursorIndex = 0;

// Identifies which setting will receive the next frequency selection.
// Measure frequency is the default target when entering the analyzer.
LCRFrequencyTarget lcrFrequencyTarget = LCR_FREQ_MEASURE;

// Stores the current LCR user-interface state.
// The analyzer normally displays the active tab unless a selector is open.
LCRUIState lcrUIState = LCR_UI_NORMAL;

// Stores the active measurement configuration used by the LCR analyzer.
// UI controls modify these values and the measurement backend reads them
// whenever a new impedance measurement is requested.
MeasurementSettings lcrSettings = {
  1000,      // Frequency: 1 kHz
  1000.0f    // Reference resistor: 1 kOhm
};

// Stores the most recently acquired LCR measurement.
// LIVE updates this value continuously while HOLD preserves it for display.
MeasurementPoint lcrMeasurement = {};

// Stores the active LCR sweep configuration.
// Defaults provide a useful full-range linear sweep while allowing the
// Sweep Setup screen to modify these values later.
SweepSettings lcrSweepSettings = {
  100,                 // Start frequency: 100 Hz
  110000,              // Stop frequency: 110 kHz
  LCR_SWEEP_LINEAR,    // Sweep mode
  1000,                // Linear step: 1 kHz
  10                   // Logarithmic density: 10 points/decade
};

// Stores the results of the most recently completed or active sweep.
// The point count identifies how many entries currently contain valid data.
SweepPoint lcrSweepPoints[LCR_MAX_SWEEP_POINTS];

uint16_t lcrSweepPointCount = 0;


// Tracks progress through the currently active frequency sweep.
// Timing fields support non-blocking acquisition and progress estimation.
SweepExecution lcrSweepExecution = {
  0,      // Total points
  0,      // Current point
  0,      // Current frequency
  0,      // Sweep start time
  0,      // Last point acquisition time
  false   // Active
};


// Off-screen drawing buffer used for dynamic LCR measurement fields.
// Rendering into RAM first allows the completed field to be transferred
// to the TFT at once, reducing visible erase/redraw flicker.
TFT_eSprite lcrValueSprite = TFT_eSprite(&display);

// Dynamic Sweep progress information is rendered through a sprite because
// direct TFT erase/redraw produces visible flicker during an active sweep.
// The completed progress area is composed in RAM and transferred at once.
TFT_eSprite lcrSweepSprite = TFT_eSprite(&display);

// Defines one selectable frequency preset.
// The label is used by the UI and the value is stored internally in Hz.
struct FrequencyPreset
{
  const char *label;
  uint32_t value;
};

// Defines all frequency presets available from the Measure screen.
// Adding or removing an entry automatically changes the selector button
// count, layout, drawing, and touch handling.
const FrequencyPreset frequencyPresets[] = {
  { "100 Hz", 100 },
  { "1 kHz", 1000 },
  { "10 kHz", 10000 },
  { "100 kHz", 100000 },
  { "110 kHz", 110000 }
};

constexpr uint8_t FREQUENCY_PRESET_COUNT =
  sizeof(frequencyPresets) / sizeof(frequencyPresets[0]);


// Defines one selectable reference-resistor preset.
// The label is used by the UI and the value is stored internally in Ohms.
struct ReferencePreset
{
  const char *label;
  float value;
};

// Defines all reference-resistor presets available from the Measure screen.
// Adding or removing an entry automatically changes the selector button
// count, layout, drawing, and touch handling.
const ReferencePreset referencePresets[] = {
  { "100 Ohm", 100.0f },
  { "1 kOhm", 1000.0f },
  { "10 kOhm", 10000.0f }
};

constexpr uint8_t REFERENCE_PRESET_COUNT =
  sizeof(referencePresets) / sizeof(referencePresets[0]);


// Defines one selectable linear Sweep step.
// The label is displayed by the UI while the value is stored in Hz.
struct SweepStepPreset
{
  const char *label;
  uint32_t value;
};

// Defines all available linear Sweep step sizes.
// Adding or removing entries automatically changes selector layout,
// drawing, and touch handling.
const SweepStepPreset sweepStepPresets[] = {
  { "100 Hz", 100 },
  { "1 kHz", 1000 },
  { "10 kHz", 10000 }
};

constexpr uint8_t SWEEP_STEP_PRESET_COUNT =
  sizeof(sweepStepPresets) / sizeof(sweepStepPresets[0]);


// Defines one selectable logarithmic Sweep measurement density.
struct SweepDensityPreset
{
  const char *label;
  uint16_t value;
};

// Defines all available logarithmic Sweep densities in points per decade.
const SweepDensityPreset sweepDensityPresets[] = {
  { "10", 10 },
  { "20", 20 },
  { "50", 50 }
};

constexpr uint8_t SWEEP_DENSITY_PRESET_COUNT =
  sizeof(sweepDensityPresets) / sizeof(sweepDensityPresets[0]);

// Defines one selectable Sweep Results plot quantity.
struct SweepPlotPreset
{
  const char *label;
  LCRSweepPlotType type;
};

// Available quantities that can be displayed from retained Sweep results.
// Adding another stored quantity later only requires extending this table
// and teaching the plot-value helpers how to retrieve and scale it.
const SweepPlotPreset sweepPlotPresets[] = {
  { "|Z|",   LCR_SWEEP_PLOT_IMPEDANCE  },
  { "Phase", LCR_SWEEP_PLOT_PHASE      },
  { "R",     LCR_SWEEP_PLOT_RESISTANCE },
  { "X",     LCR_SWEEP_PLOT_REACTANCE },
  { "ESR",   LCR_SWEEP_PLOT_ESR        },
  { "Q",     LCR_SWEEP_PLOT_Q          }
};

constexpr uint8_t SWEEP_PLOT_PRESET_COUNT =
  sizeof(sweepPlotPresets) /
  sizeof(sweepPlotPresets[0]);

// Convert a complete impedance measurement into the compact representation
// retained by the Sweep engine. Acquisition-specific values that are not
// required for Sweep plots are intentionally discarded.
SweepPoint makeSweepPoint(const MeasurementPoint &measurement)
{
  SweepPoint point;

  point.frequency = measurement.frequency;
  point.impedance = measurement.impedance;
  point.phaseDeg = measurement.phaseDeg;
  point.resistance = measurement.resistance;
  point.reactance = measurement.reactance;
  point.esr = measurement.esr;
  point.q = measurement.q;

  return point;
}


// Return the value from one SweepPoint that corresponds to the currently
// selected Results plot type.
float getLCRSweepPlotValue(const SweepPoint &point)
{
  switch (lcrSweepPlotType) {
    case LCR_SWEEP_PLOT_PHASE:
      return point.phaseDeg;

    case LCR_SWEEP_PLOT_RESISTANCE:
      return point.resistance;

    case LCR_SWEEP_PLOT_REACTANCE:
      return point.reactance;

    case LCR_SWEEP_PLOT_ESR:
      return point.esr;

    case LCR_SWEEP_PLOT_Q:
      return point.q;

    case LCR_SWEEP_PLOT_IMPEDANCE:
    default:
      return point.impedance;
  }
}


// Find the minimum and maximum values for the currently selected Sweep plot.
// A zero-height range is expanded so constant measurements still produce
// a usable vertical scale.
SweepPlotRange findLCRSweepPlotRange()
{
  SweepPlotRange range = { 0.0f, 1.0f };

  if (lcrSweepPointCount == 0)
    return range;

  range.minimum = getLCRSweepPlotValue(lcrSweepPoints[0]);
  range.maximum = range.minimum;

  for (uint16_t i = 1; i < lcrSweepPointCount; i++) {
    float value = getLCRSweepPlotValue(lcrSweepPoints[i]);

    if (value < range.minimum)
      range.minimum = value;

    if (value > range.maximum)
      range.maximum = value;
  }

  float span = range.maximum - range.minimum;

  if (span <= 0.0f) {
    float padding = fabsf(range.maximum) * 0.05f;

    if (padding < 0.01f)
      padding = 0.01f;

    range.minimum -= padding;
    range.maximum += padding;
  }

  return range;
}


// Expand a Sweep plot range slightly beyond the measured values.
// Padding prevents the highest and lowest samples from being drawn directly
// against the graph border while preserving signed quantities such as X.
SweepPlotRange padLCRSweepPlotRange(SweepPlotRange range)
{
  float span = range.maximum - range.minimum;

  if (span <= 0.0f)
    return range;

  float padding = span * 0.08f;

  range.minimum -= padding;
  range.maximum += padding;

  // Magnitude-only quantities cannot be negative.
  if ((lcrSweepPlotType == LCR_SWEEP_PLOT_IMPEDANCE ||
       lcrSweepPlotType == LCR_SWEEP_PLOT_ESR ||
       lcrSweepPlotType == LCR_SWEEP_PLOT_Q) &&
      range.minimum < 0.0f) {

    range.minimum = 0.0f;
  }

  return range;
}


// Select the engineering scale and unit appropriate for the active Sweep plot.
// Resistance-like quantities use Ohm engineering prefixes, while Phase and Q
// use their natural units without additional scaling.
SweepAxisScale getLCRSweepAxisScale(const SweepPlotRange &range)
{
  switch (lcrSweepPlotType) {
    case LCR_SWEEP_PLOT_PHASE:
      return { 1.0f, "deg" };

    case LCR_SWEEP_PLOT_Q:
      return { 1.0f, "" };

    case LCR_SWEEP_PLOT_IMPEDANCE:
    case LCR_SWEEP_PLOT_RESISTANCE:
    case LCR_SWEEP_PLOT_REACTANCE:
    case LCR_SWEEP_PLOT_ESR:
    default:
      break;
  }

  float largest = max(fabsf(range.minimum), fabsf(range.maximum));

  if (largest >= 1000000.0f)
    return { 1000000.0f, "MOhm" };

  if (largest >= 1000.0f)
    return { 1000.0f, "kOhm" };

  return { 1.0f, "Ohm" };
}


// Format one Sweep-axis numeric value compactly.
// Precision decreases as the displayed number becomes larger so labels
// remain short enough for the limited graph margin.
void formatLCRSweepAxisNumber(char *buffer,
                              size_t bufferSize,
                              float value)
{
  float magnitude = fabsf(value);

  if (magnitude >= 100.0f) {
    snprintf(
      buffer,
      bufferSize,
      "%.0f",
      value
    );
  } else if (magnitude >= 10.0f) {
    snprintf(
      buffer,
      bufferSize,
      "%.1f",
      value
    );
  } else {
    snprintf(
      buffer,
      bufferSize,
      "%.2f",
      value
    );
  }
}


// Format the selected Sweep cursor value using the engineering scale currently
// displayed on the Y axis.
void formatLCRSweepCursorValue(char *buffer, size_t bufferSize,
                               float value, const SweepAxisScale &scale)
{
  char number[16];

  formatLCRSweepAxisNumber(number, sizeof(number), value / scale.divisor);

  if (strlen(scale.unit) > 0)
    snprintf(buffer, bufferSize, "%s%s", number, scale.unit);
  else
    snprintf(buffer, bufferSize, "%s", number);
}


// Convert a Sweep frequency into an X coordinate within the Results plot.
// Linear sweeps use linear frequency spacing, while logarithmic sweeps use
// logarithmic spacing so the displayed axis matches the acquisition mode.
int16_t calculateLCRSweepPlotX(uint32_t frequency)
{
  const LCRRect &plot = lcrLayout.sweepPlot;
  uint32_t start = lcrSweepSettings.startFrequency;
  uint32_t stop = lcrSweepSettings.stopFrequency;

  if (stop <= start)
    return plot.x;

  float position;

  if (lcrSweepSettings.mode == LCR_SWEEP_LOG) {

    if (frequency == 0 || start == 0)
      return plot.x;

    float logStart = log10f(static_cast<float>(start));
    float logStop = log10f(static_cast<float>(stop));
    float logFrequency = log10f(static_cast<float>(frequency));

    position = (logFrequency - logStart) / (logStop - logStart);
  } else {
    position =
      static_cast<float>(frequency - start) /
      static_cast<float>(stop - start);
  }

  position = constrain(position, 0.0f, 1.0f);

  return
    plot.x +
    static_cast<int16_t>(
      position * (plot.w - 1)
    );
}


// Find the retained SweepPoint whose plotted X coordinate is closest to the
// requested screen position. This automatically follows Linear or Log spacing
// because it uses the same X-coordinate calculation as the displayed trace.
uint16_t findNearestLCRSweepPoint(int16_t touchX)
{
  if (lcrSweepPointCount == 0)
    return 0;

  uint16_t nearestIndex = 0;
  int32_t nearestDistance = INT32_MAX;

  for (uint16_t i = 0; i < lcrSweepPointCount; i++) {
    int16_t pointX = calculateLCRSweepPlotX(lcrSweepPoints[i].frequency);
    int32_t distance = abs(static_cast<int32_t>(touchX) - pointX);

    if (distance < nearestDistance) {
      nearestDistance = distance;
      nearestIndex = i;
    }
  }

  return nearestIndex;
}


// Convert the selected Sweep value into a Y coordinate within the Results
// plot. Higher values appear toward the top of the TFT coordinate system.
int16_t calculateLCRSweepPlotY(float value, const SweepPlotRange &range)
{
  const LCRRect &plot = lcrLayout.sweepPlot;
  float span = range.maximum - range.minimum;

  if (span <= 0.0f)
    return plot.y + plot.h / 2;

  float position = (value - range.minimum) / span;
  position = constrain(position, 0.0f, 1.0f);

  return plot.y + plot.h - 1 -
         static_cast<int16_t>(position * (plot.h - 1));
}


// Return the short display label associated with the currently selected
// Sweep Results quantity.
const char *getLCRSweepPlotLabel()
{
  for (uint8_t i = 0;
       i < SWEEP_PLOT_PRESET_COUNT;
       i++) {
    if (sweepPlotPresets[i].type == lcrSweepPlotType) {
      return sweepPlotPresets[i].label;
    }
  }

  return "|Z|";
}


// Draw the Sweep Results plot-control/status strip.
// Without a cursor it identifies the selected plot quantity. When a point is
// selected it also displays that point's frequency and measured plot value.
void drawLCRSweepPlotControl()
{
  const LCRRect &control = lcrLayout.sweepPlotControl;

  display.fillRect(control.x, control.y, control.w, control.h, BGCOLOR);
  display.drawRect(control.x, control.y, control.w, control.h, GRIDCOLOR);

  display.setTextSize(1);
  display.setTextColor(TXTCOLOR, BGCOLOR);

  char label[64];

  if (lcrSweepCursorActive &&
      lcrSweepCursorIndex < lcrSweepPointCount) {

    const SweepPoint &point = lcrSweepPoints[lcrSweepCursorIndex];

    SweepPlotRange range = findLCRSweepPlotRange();
    range = padLCRSweepPlotRange(range);

    SweepAxisScale scale = getLCRSweepAxisScale(range);

    LCRFormattedValue frequency = formatFrequency(point.frequency);

    char value[20];
    formatLCRSweepCursorValue(
      value,
      sizeof(value),
      getLCRSweepPlotValue(point),
      scale
    );

    snprintf(
      label,
      sizeof(label),
      "%s   %s%s   %s",
      getLCRSweepPlotLabel(),
      frequency.value,
      frequency.unit,
      value
    );
  } else {
    snprintf(label, sizeof(label), "Plot: %s", getLCRSweepPlotLabel());
  }

  display.setCursor( control.x + 8, control.y + (control.h - 8) / 2);

  display.print(label);

  // Down-arrow indicator shows that this strip remains selectable even when
  // cursor information is being displayed.
  const int16_t arrowX = control.x + control.w - 12;
  const int16_t arrowY = control.y + control.h / 2 - 1;

  display.drawLine( arrowX - 3, arrowY - 2, arrowX, arrowY + 1, TXTCOLOR);
  display.drawLine( arrowX, arrowY + 1, arrowX + 3, arrowY - 2, TXTCOLOR);
}


// Draw compact labels around the Sweep impedance plot.
// The Y axis uses one common engineering scale so numeric labels remain
// short while the axis title identifies their shared impedance unit.
void drawLCRSweepPlotLabels(const SweepPlotRange &range)
{
  const LCRRect &plot = lcrLayout.sweepPlot;
  char label[24];

  display.setTextSize(1);
  display.setTextColor(TXTCOLOR, BGCOLOR);

  //
  // Determine the common Y-axis engineering scale.
  //
  SweepAxisScale scale = getLCRSweepAxisScale(range);

  //
  // Y-axis unit.
  //
  if (strlen(scale.unit) > 0) {
    snprintf(label, sizeof(label), "(%s)", scale.unit);
    display.setCursor(plot.x, plot.y - 10);
    display.print(label);
  }

  //
  // Maximum Y value.
  //
  formatLCRSweepAxisNumber(
    label,
    sizeof(label),
    range.maximum / scale.divisor
  );

  int16_t labelWidth = strlen(label) * 6;

  display.setCursor( plot.x - labelWidth - 3, plot.y);

  display.print(label);

  //
  // Midpoint Y value.
  //

  float midpoint = (range.minimum + range.maximum) / 2.0f;

  formatLCRSweepAxisNumber(
    label,
    sizeof(label),
    midpoint / scale.divisor
  );

  labelWidth = strlen(label) * 6;

  display.setCursor( plot.x - labelWidth - 3, plot.y + plot.h / 2 - 4);

  display.print(label);

  //
  // Minimum Y value.
  //

  formatLCRSweepAxisNumber(
    label,
    sizeof(label),
    range.minimum / scale.divisor
  );

  labelWidth = strlen(label) * 6;

  display.setCursor(
    plot.x - labelWidth - 3,
    plot.y + plot.h - 8
  );

  display.print(label);

  //
  // Start frequency.
  //

  LCRFormattedValue startFrequency =
    formatFrequency( lcrSweepSettings.startFrequency);

  snprintf(
    label,
    sizeof(label),
    "%s%s",
    startFrequency.value,
    startFrequency.unit
  );

  display.setCursor(
    plot.x,
    plot.y + plot.h + 4
  );

  display.print(label);

  //
  // Midpoint frequency.
  //
  // The center label represents the frequency at the visual midpoint of the
  // active X axis. Linear axes use the arithmetic midpoint while Log axes use
  // the geometric midpoint.
  //

  uint32_t midpointFrequency;

  if (lcrSweepSettings.mode == LCR_SWEEP_LOG) {
    midpointFrequency =
      static_cast<uint32_t>(
        roundf(
          sqrtf(
            static_cast<float>(
              lcrSweepSettings.startFrequency
            ) *
            static_cast<float>(
              lcrSweepSettings.stopFrequency
            )
          )
        )
      );
  } else {
    midpointFrequency =
      lcrSweepSettings.startFrequency +
      (lcrSweepSettings.stopFrequency -
       lcrSweepSettings.startFrequency) / 2;
  }

  LCRFormattedValue middleFrequency =
    formatFrequency(midpointFrequency);

  snprintf(
    label,
    sizeof(label),
    "%s%s",
    middleFrequency.value,
    middleFrequency.unit
  );

  labelWidth = strlen(label) * 6;

  display.setCursor(
    plot.x +
      (plot.w - labelWidth) / 2,
    plot.y + plot.h + 4
  );

  display.print(label);

  //
  // Stop frequency.
  //

  LCRFormattedValue stopFrequency =
    formatFrequency(
      lcrSweepSettings.stopFrequency
    );

  snprintf(
    label,
    sizeof(label),
    "%s%s",
    stopFrequency.value,
    stopFrequency.unit
  );

  labelWidth = strlen(label) * 6;

  display.setCursor(
    plot.x + plot.w - labelWidth,
    plot.y + plot.h + 4
  );

  display.print(label);
}


// Draw the retained Sweep impedance-magnitude results as a connected trace.
// Plot scaling is derived automatically from the acquired data, while the
// X-axis spacing follows the Linear or Log mode used for the Sweep.
void drawLCRSweepPlot()
{
  const LCRRect &plot = lcrLayout.sweepPlot;

  //
  // Plot boundary
  //

  display.drawRect(
    plot.x,
    plot.y,
    plot.w,
    plot.h,
    GRIDCOLOR
  );

  // Draw a single horizontal midpoint reference through the plot.
  // Keeping the grid minimal provides a useful visual scale without
  // overwhelming the trace on the small display.
  const int16_t midpointY = plot.y + plot.h / 2;

  display.drawFastHLine(
    plot.x + 1,
    midpointY,
    plot.w - 2,
    GRIDCOLOR
  );

  // Draw a single vertical midpoint reference through the plot.
  // Together with the horizontal midpoint this creates a simple 2x2
  // reference grid without overcrowding the Results display.
  const int16_t midpointX = plot.x + plot.w / 2;

  display.drawFastVLine(
    midpointX,
    plot.y + 1,
    plot.h - 2,
    GRIDCOLOR
  );

  if (lcrSweepPointCount == 0)
    return;

  //
  // Determine the displayed impedance range.
  //

  SweepPlotRange range = findLCRSweepPlotRange();
  range = padLCRSweepPlotRange(range);

  // Draw axis and quantity labels using the same range as the trace.
  drawLCRSweepPlotLabels(range);

  //
  // Draw the trace.
  //

  int16_t previousX = calculateLCRSweepPlotX( lcrSweepPoints[0].frequency);
  int16_t previousY =
    calculateLCRSweepPlotY(getLCRSweepPlotValue(lcrSweepPoints[0]), range);

  // A single-point result still gets a visible marker.
  display.drawPixel(
    previousX,
    previousY,
    HIGHCOLOR
  );

  // Trace drawing loop
  for (uint16_t i = 1;
       i < lcrSweepPointCount;
       i++) {

    int16_t x = calculateLCRSweepPlotX( lcrSweepPoints[i].frequency);
    int16_t y =
      calculateLCRSweepPlotY(getLCRSweepPlotValue(lcrSweepPoints[i]), range);

    display.drawLine( previousX, previousY, x, y, HIGHCOLOR);

    previousX = x;
    previousY = y;
  }

  //
  // Results inspection cursor
  //

  if (lcrSweepCursorActive &&
      lcrSweepCursorIndex < lcrSweepPointCount) {

    int16_t cursorX = calculateLCRSweepPlotX(
      lcrSweepPoints[lcrSweepCursorIndex].frequency
    );

    display.drawFastVLine(
      cursorX,
      plot.y + 1,
      plot.h - 2,
      TXTCOLOR
    );
  }
}


// Acquire and store one measurement at the requested Sweep frequency.
// The normal measurement backend is used so this works with the simulation
// backend now and the hardware backend later without changing Sweep logic.
bool acquireLCRSweepPoint(uint32_t frequency)
{
  if (lcrSweepPointCount >= LCR_MAX_SWEEP_POINTS)
    return false;

  MeasurementSettings settings = lcrSettings;
  settings.frequency = frequency;
  MeasurementPoint measurement = measureImpedance(settings);
  lcrSweepPoints[lcrSweepPointCount] = makeSweepPoint(measurement);

  lcrSweepPointCount++;

  return true;
}


// Initialize a new frequency sweep from the current Sweep configuration.
// Existing results are discarded only after validation succeeds, and timing
// state is initialized for non-blocking acquisition and progress reporting.
bool startLCRSweep()
{
  if (!isLCRSweepConfigurationValid(
        lcrSweepSettings)) {

    return false;
  }

  uint32_t totalPoints = calculateSweepPointCount( lcrSweepSettings);

  if (totalPoints == 0 ||
      totalPoints > LCR_MAX_SWEEP_POINTS) {

    return false;
  }

  lcrSweepPointCount = 0;
  lcrSweepCursorActive = false;
  lcrSweepCursorIndex = 0;
  lcrSweepExecution.totalPoints = totalPoints;
  lcrSweepExecution.currentPoint = 0;
  lcrSweepExecution.currentFrequency = lcrSweepSettings.startFrequency;
  lcrSweepExecution.startTime = millis();

  // Allow the first point to be acquired immediately.
  lcrSweepExecution.lastPointTime = millis() - LCR_SIM_SWEEP_POINT_INTERVAL_MS;
  lcrSweepExecution.active = true;
  lcrSweepState = LCR_SWEEP_RUNNING;
  lcrUIState = LCR_UI_NORMAL;

  drawLCRScreen();

  return true;
}


// Complete the active frequency sweep and transition to the Results state.
// All successfully acquired SweepPoint entries remain available for
// plotting and cursor inspection.
void finishLCRSweep()
{
  lcrSweepExecution.active = false;
  lcrSweepState = LCR_SWEEP_RESULTS;
  lcrUIState = LCR_UI_NORMAL;

  drawLCRScreen();
}


// Cancel an active Sweep and return to the Setup state.
// Partial Sweep results are discarded so they cannot be mistaken for a
// completed result set.
void cancelLCRSweep()
{
  lcrSweepExecution.active = false;
  lcrSweepExecution.totalPoints = 0;
  lcrSweepExecution.currentPoint = 0;
  lcrSweepExecution.currentFrequency = 0;
  lcrSweepPointCount = 0;
  lcrSweepState = LCR_SWEEP_SETUP;
  lcrUIState = LCR_UI_NORMAL;

  drawLCRScreen();
}


// Advance an active Sweep by at most one measurement point per call.
// Simulation is intentionally rate-limited so progress and Cancel behavior
// can be tested. The Running display is refreshed after every acquired point.
void updateLCRSweep()
{
  if (!lcrSweepExecution.active)
    return;

  if (lcrSweepExecution.currentPoint >= lcrSweepExecution.totalPoints) {
    finishLCRSweep();
    return;
  }

  uint32_t now = millis();

  // Slow simulated acquisition enough to make progress visible.
  if (lcrBackend == LCR_BACKEND_SIMULATION) {
    if (now - lcrSweepExecution.lastPointTime <
        LCR_SIM_SWEEP_POINT_INTERVAL_MS) {

      return;
    }
  }

  lcrSweepExecution.lastPointTime = now;

  // Calculate the frequency for the point currently being acquired.
  uint32_t frequency =
    calculateSweepFrequency(
      lcrSweepSettings,
      lcrSweepExecution.currentPoint
    );

  lcrSweepExecution.currentFrequency = frequency;

  // Acquire and store one Sweep point.
  if (!acquireLCRSweepPoint(frequency)) {
    finishLCRSweep();
    return;
  }

  // Mark this point complete.
  lcrSweepExecution.currentPoint++;

  // Update the Running display while the Sweep is still active.
  updateLCRSweepRunningDisplay();

  // Transition to Results after the final point has been displayed.
  if (lcrSweepExecution.currentPoint >= lcrSweepExecution.totalPoints) {
    finishLCRSweep();
    return;
  }
}



// measurement objects
MeasurementPoint simulatedMeasurement(const MeasurementSettings &settings);
MeasurementPoint hardwareMeasurement(const MeasurementSettings &settings);



// Generate simulated measurement data for GUI and workflow development
// when measurement hardware is not available.
MeasurementPoint simulatedMeasurement(const MeasurementSettings &settings)
{
  MeasurementPoint m;
  static float phase = -45.0f;
  m.frequency = settings.frequency;
  m.impedance = 1000.0f + 200.0f * sin(millis() / 800.0f);
  m.phaseDeg = phase;
  m.resistance = m.impedance * cos(radians(phase));
  m.reactance = m.impedance * sin(radians(phase));

  // Sweep the simulated capacitance across the nF/uF boundary so automatic
  // engineering-unit selection can be verified without measurement hardware.
  float simulationPosition = (sin(millis() / 2500.0f) + 1.0f) / 2.0f;
  m.capacitance = (500.0e-9f + simulationPosition * 1.0e-6f);

  m.inductance = 0.0f;
  m.esr = 2.5f;
  m.q = fabs(m.reactance) / m.resistance;
  m.dissipation = 0.0012f;

  return m;
}


// Acquire a single measurement from the hardware measurement engine.
// This function will eventually control the AD9833, ADC/DMA, and
// impedance calculations.
MeasurementPoint hardwareMeasurement(const MeasurementSettings &settings)
{
  MeasurementPoint m;

  //
  // Placeholder until hardware exists.
  //

  return m;
}



// Measurement engine interface.
//
// Dispatches a single measurement request to either the simulation
// backend or the hardware backend. The GUI calls only this function
// and does not need to know where the data originated.
MeasurementPoint measureImpedance(const MeasurementSettings &settings)
{
  switch (lcrBackend) {

    case LCR_BACKEND_SIMULATION:
      return simulatedMeasurement(settings);

    case LCR_BACKEND_HARDWARE:
      return hardwareMeasurement(settings);

    default:
      return simulatedMeasurement(settings);
  }
}



// Enter the LCR analyzer and display the default Measure tab.
// Each new analyzer session begins in LIVE mode with no selector open.
void enterLCRMode()
{
  instrumentMode = MODE_LCR;
  lcrTab = LCR_TAB_MEASURE;
  lcrMeasureState = LCR_MEASURE_LIVE;
  lcrSweepState = LCR_SWEEP_SETUP;
  lcrFrequencyTarget = LCR_FREQ_MEASURE;
  lcrUIState = LCR_UI_NORMAL;
  lcrDisplayDirty = true;

  initializeLCR();
  drawLCRScreen();
}



// Exit the LCR instrument and return to oscilloscope mode.
void exitLCRMode()
{
  instrumentMode = MODE_SCOPE;
  display.fillScreen(BGCOLOR);
  DrawText();
}



// Main LCR instrument task.
// Handles Measure acquisition/display updates and routes touchscreen input
// according to the active tab and UI state. Modal selectors receive touch
// input before the controls on the underlying Measure or Sweep screens.
void updateLCR()
{
  static uint32_t lastDisplayUpdate = 0;
  static bool lastPressed = false;

  uint32_t now = millis();

  //
  // Active Sweep execution
  //
  if (lcrTab == LCR_TAB_SWEEP &&
      lcrSweepState == LCR_SWEEP_RUNNING) {

    updateLCRSweep();
  }

  //
  // Measure acquisition and display update
  //

  if (lcrTab == LCR_TAB_MEASURE &&
      lcrMeasureState == LCR_MEASURE_LIVE &&
      lcrUIState == LCR_UI_NORMAL) {

    lcrMeasurement = measureImpedance(lcrSettings);

    if (lcrDisplayDirty ||
        now - lastDisplayUpdate >= LCR_DISPLAY_INTERVAL_MS) {

      lastDisplayUpdate = now;
      lcrDisplayDirty = false;

      updateLCRDisplay(
        lcrMeasurement,
        lcrSettings
      );
    }
  }

  //
  // Touch detection
  //

  uint16_t x = 0;
  uint16_t y = 0;

  bool pressed = readTouch(x, y);

  // Require the screen to be released before accepting another touch.
  if (!pressed) {
    lastPressed = false;
    return;
  }

  if (lastPressed)
    return;

  lastPressed = true;

  //
  // Back
  //

  if (pointInLCRRect(
        x,
        y,
        lcrLayout.backButton)) {

    exitLCRMode();
    return;
  }

  //
  // Top-level analyzer tabs
  //

  if (pointInLCRRect(
        x,
        y,
        lcrLayout.tabs)) {

    handleLCRTabTouch(x, y);
    return;
  }

  //
  // Modal selectors
  //
  // These must be processed before controls on the underlying tab.
  //

  if (lcrUIState == LCR_UI_FREQ_SELECT) {
    handleLCRFrequencySelectorTouch(x, y);
    return;
  }

  if (lcrUIState == LCR_UI_REF_SELECT) {
    handleLCRReferenceSelectorTouch(x, y);
    return;
  }

  if (lcrUIState == LCR_UI_SWEEP_MODE_SELECT) {
    handleLCRSweepModeSelectorTouch(x, y);
    return;
  }

  if (lcrUIState == LCR_UI_SWEEP_STEP_SELECT) {
    handleLCRSweepStepSelectorTouch(x, y);
    return;
  }

  if (lcrUIState == LCR_UI_SWEEP_DENSITY_SELECT) {
    handleLCRSweepDensitySelectorTouch(x, y);
    return;
  }

  if (lcrUIState == LCR_UI_SWEEP_PLOT_SELECT) {
    handleLCRSweepPlotSelectorTouch(x, y);
    return;
  }

  //
  // Sweep Running controls
  //

  if (lcrTab == LCR_TAB_SWEEP &&
      lcrSweepState == LCR_SWEEP_RUNNING &&
      lcrUIState == LCR_UI_NORMAL) {

    handleLCRSweepRunningTouch(x, y);
    return;
  }

  //
  // Sweep Results controls
  //

  if (lcrTab == LCR_TAB_SWEEP &&
      lcrSweepState == LCR_SWEEP_RESULTS &&
      lcrUIState == LCR_UI_NORMAL) {

    handleLCRSweepResultsTouch(x, y);
    return;
  }

  //
  // Sweep Setup controls
  //

  if (lcrTab == LCR_TAB_SWEEP &&
      lcrSweepState == LCR_SWEEP_SETUP &&
      lcrUIState == LCR_UI_NORMAL &&
      pointInLCRRect(
        x,
        y,
        lcrLayout.content)) {

    handleLCRSweepSetupTouch(x, y);
    return;
  }

  //
  // Sweep Setup action
  //

  if (lcrTab == LCR_TAB_SWEEP &&
      lcrSweepState == LCR_SWEEP_SETUP &&
      lcrUIState == LCR_UI_NORMAL &&
      pointInLCRRect(
        x,
        y,
        lcrLayout.footer)) {

    handleLCRSweepFooterTouch(x, y);
    return;
  }

  //
  // Measure soft keys
  //

  if (lcrTab == LCR_TAB_MEASURE &&
      lcrUIState == LCR_UI_NORMAL &&
      pointInLCRRect(
        x,
        y,
        lcrLayout.footer)) {

    handleMeasureSoftKeyTouch(x, y);
    return;
  }
}


// Clear a measurement value before drawing a new one.
//
// This prevents remnants of previous values from remaining on the
// display when the number of digits changes.
void clearValueField(int x, int y, int width = 120)
{
  display.fillRect(x, y, width, 10, BGCOLOR);
}


// Initialize the AD9833 excitation source on the dedicated SPI0 bus.
// The generator starts at 1 kHz so hardware operation can be verified
// independently before integrating the LCR measurement backend.
void initializeLCRGenerator()
{
  SPI.setSCK(AD9833_SCK_PIN);
  SPI.setTX(AD9833_DATA_PIN);
  SPI.setRX(NOPIN);

  lcrDDS.begin(1000);

  Serial.println("LCR AD9833 initialized at 1 kHz");
}


// Set the AD9833 frequency directly for hardware bring-up testing.
// This diagnostic helper will be removed once the generator is controlled
// exclusively through the LCR hardware measurement backend.
void testLCRGeneratorFrequency(uint32_t frequency)
{
  lcrDDS.setFrequencyHz(frequency);

  Serial.print("AD9833 frequency set to ");
  Serial.print(frequency);
  Serial.println(" Hz");
}


// Configure the reusable sprite used for dynamic LCR measurement fields.
// The sprite is created at the largest dynamic-region size and reused for
// both primary and secondary measurements to minimize RAM consumption.
void initializeLCRValueSprite()
{
  // Remove an existing buffer before recreating it.
  lcrValueSprite.deleteSprite();

  int16_t spriteW = max(
    lcrLayout.primary.w,
    lcrLayout.secondary.w
  );

  int16_t spriteH = max(
    lcrLayout.primary.h,
    lcrLayout.secondary.h
  );

  lcrValueSprite.setColorDepth(16);
  lcrValueSprite.createSprite(spriteW, spriteH);

  lcrValueSprite.fillSprite(BGCOLOR);
  lcrValueSprite.setTextColor(TXTCOLOR, BGCOLOR);
}


// Configure the off-screen sprite used for dynamic Sweep progress.
// An 8-bit sprite is sufficient for the progress UI and uses half the RAM
// of a 16-bit sprite, leaving memory available for stored Sweep results.
void initializeLCRSweepSprite()
{
  lcrSweepSprite.deleteSprite();

  lcrSweepSprite.setColorDepth(8);

  void *spriteBuffer =
    lcrSweepSprite.createSprite(
      lcrLayout.content.w,
      lcrLayout.content.h
    );

  if (spriteBuffer == nullptr) {
    Serial.println(
      "ERROR: Unable to allocate LCR Sweep sprite"
    );

    return;
  }

  lcrSweepSprite.fillSprite(BGCOLOR);
  lcrSweepSprite.setTextColor(TXTCOLOR, BGCOLOR);
}



// Calculate only the major screen regions shared by every LCR analyzer tab.
// Tab-specific and selector-specific geometry is calculated separately so
// this function remains independent of individual instrument screens.
void calculateLCRLayout()
{
  const int16_t screenW = display.width();
  const int16_t screenH = display.height();

  const int16_t headerH = screenH * LCR_HEADER_HEIGHT_PERCENT / 100;
  const int16_t tabsH = screenH * LCR_TAB_HEIGHT_PERCENT / 100;
  const int16_t footerH = screenH * LCR_FOOTER_HEIGHT_PERCENT / 100;

  const int16_t contentY = headerH + tabsH;
  const int16_t contentH = screenH - headerH - tabsH - footerH;

  const int16_t footerY = screenH - footerH;

  lcrLayout.header = { 0, 0, screenW, headerH };

  // Define the Back-button touch region within the left side of the header.
  // Keeping this separate from the complete header prevents accidental exits
  // when the instrument title or unused header space is touched.
  const int16_t backButtonW = screenW * LCR_BACK_BUTTON_WIDTH_PERCENT / 100;

  lcrLayout.backButton = {
    lcrLayout.header.x,
    lcrLayout.header.y,
    backButtonW,
    lcrLayout.header.h
  };

  lcrLayout.tabs = { 0, headerH, screenW, tabsH };

  // Divide the analyzer tab bar into four equal touch regions.
  // These rectangles are shared by tab drawing and touch detection so the
  // visible navigation controls always match their active touch areas.
  const int16_t tabCount = 4;
  const int16_t tabW = lcrLayout.tabs.w / tabCount;

  for (int16_t i = 0; i < tabCount; i++) {
    int16_t x = lcrLayout.tabs.x + i * tabW;

    int16_t w = (i == tabCount - 1)
      ? lcrLayout.tabs.w - tabW * i
      : tabW;

    lcrLayout.tabButtons[i] = { x, lcrLayout.tabs.y, w, lcrLayout.tabs.h };
  }

  lcrLayout.content = { 0, contentY, screenW, contentH };
  lcrLayout.footer = { 0, footerY, screenW, footerH };
}



// Divide the common content and footer regions into areas used by the
// Measure tab, including primary/secondary measurements, context fields,
// and the four Measure soft-key touch regions.
void calculateMeasureLayout()
{
  const LCRRect &content = lcrLayout.content;
  const LCRRect &footer = lcrLayout.footer;

  const int16_t primaryH = content.h * LCR_PRIMARY_HEIGHT_PERCENT / 100;
  const int16_t secondaryH = content.h * LCR_SECONDARY_HEIGHT_PERCENT / 100;
  const int16_t secondaryY = content.y + primaryH;
  const int16_t contextY = secondaryY + secondaryH;
  const int16_t contextH = content.h - primaryH - secondaryH;

  lcrLayout.primary = { content.x, content.y, content.w, primaryH };
  lcrLayout.secondary = { content.x, secondaryY, content.w, secondaryH };
  lcrLayout.context = { content.x, contextY, content.w, contextH };

  const int16_t softKeyCount = 4;
  const int16_t softKeyW = footer.w / softKeyCount;

  for (int16_t i = 0; i < softKeyCount; i++) {
    int16_t x = footer.x + i * softKeyW;

    int16_t w = (i == softKeyCount - 1)
      ? footer.w - softKeyW * i
      : softKeyW;

    lcrLayout.measureSoftKeys[i] = { x, footer.y, w, footer.h };
  }
}


// Divide the Sweep Setup content area into five interactive rows.
// The same row rectangles are used for drawing and touch detection so
// Sweep controls remain aligned on different display resolutions.
void calculateSweepLayout()
{
  const LCRRect &content = lcrLayout.content;

  const int16_t rowCount = 5;
  const int16_t rowH = content.h / rowCount;

  for (int16_t i = 0; i < rowCount; i++) {
    int16_t y = content.y + i * rowH;

    int16_t h = (i == rowCount - 1)
      ? content.h - rowH * i
      : rowH;

    lcrLayout.sweepSetupRows[i] = { content.x, y, content.w, h };
  }

  // Use the complete Sweep footer as the Sweep action touch region.
  // Drawing and touch detection therefore share the same geometry.
  lcrLayout.sweepButton = lcrLayout.footer;

  // Define the progress-bar region used while a Sweep is running.
  lcrLayout.sweepProgress = {
    static_cast<int16_t>( content.x + content.w * 10 / 100),
    static_cast<int16_t>( content.y + content.h * 25 / 100),
    static_cast<int16_t>( content.w * 80 / 100),
    static_cast<int16_t>( display.height() * 8 / 100)
  };

  // The Running-state Cancel action occupies the complete footer.
  lcrLayout.sweepCancelButton = lcrLayout.footer;

  // Divide the Sweep Results footer into Setup and Sweep actions.
  // These regions are shared by drawing and touch handling.
  const int16_t resultsButtonW = lcrLayout.footer.w / 2;

  lcrLayout.sweepResultsSetupButton = {
    lcrLayout.footer.x,
    lcrLayout.footer.y,
    resultsButtonW,
    lcrLayout.footer.h
  };

  lcrLayout.sweepResultsSweepButton = {
    static_cast<int16_t>(
      lcrLayout.footer.x + resultsButtonW
    ),
    lcrLayout.footer.y,
    static_cast<int16_t>(
      lcrLayout.footer.w - resultsButtonW
    ),
    lcrLayout.footer.h
  };

  // Calculate the Sweep Results control strip.
  // The complete strip is touchable, providing a large target for changing
  // the plotted quantity without interfering with future graph cursors.
  const int16_t plotControlH = content.h * LCR_SWEEP_PLOT_CONTROL_PERCENT / 100;

  lcrLayout.sweepPlotControl = {
    content.x,
    content.y,
    content.w,
    plotControlH
  };

  // Calculate the Sweep Results graph below the control strip.
  // Margins around the graph provide room for axis values and frequency labels.
  const int16_t plotLeft = content.w * LCR_SWEEP_PLOT_LEFT_PERCENT / 100;
  const int16_t plotRight = content.w * LCR_SWEEP_PLOT_RIGHT_PERCENT / 100;
  const int16_t plotTop = content.h * LCR_SWEEP_PLOT_TOP_PERCENT / 100;
  const int16_t plotBottom = content.h * LCR_SWEEP_PLOT_BOTTOM_PERCENT / 100;
  const int16_t plotAreaY = content.y + plotControlH;
  const int16_t plotAreaH = content.h - plotControlH;

  lcrLayout.sweepPlot = {
    static_cast<int16_t>(
      content.x + plotLeft
    ),
    static_cast<int16_t>(
      plotAreaY + plotTop
    ),
    static_cast<int16_t>(
      content.w - plotLeft - plotRight
    ),
    static_cast<int16_t>(
      plotAreaH - plotTop - plotBottom
    )
  };
}


// Calculate the number of measurements produced by a linear sweep.
// Invalid settings return zero rather than allowing divide-by-zero or an
// inverted frequency range to propagate into the sweep engine.
uint32_t calculateLinearSweepPointCount(const SweepSettings &settings)
{
  if (settings.stepFrequency == 0)
    return 0;

  if (settings.stopFrequency < settings.startFrequency)
    return 0;

  return
    (settings.stopFrequency - settings.startFrequency) /
    settings.stepFrequency + 1;
}


// Calculate the number of measurements produced by a logarithmic sweep.
// Measurement density is specified in points per decade, with one
// additional point included for the initial Start frequency.
uint32_t calculateLogSweepPointCount(const SweepSettings &settings)
{
  if (settings.startFrequency == 0)
    return 0;

  if (settings.stopFrequency <= settings.startFrequency)
    return 0;

  if (settings.pointsPerDecade == 0)
    return 0;

  float decades =
    log10f(
      static_cast<float>(settings.stopFrequency) /
      static_cast<float>(settings.startFrequency)
    );

  uint32_t intervals =
    static_cast<uint32_t>(
      ceilf(decades * settings.pointsPerDecade)
    );

  return intervals + 1;
}


// Return the number of measurement points required by the active Sweep
// configuration. The appropriate calculation is selected by Sweep mode.
uint32_t calculateSweepPointCount(const SweepSettings &settings)
{
  if (settings.mode == LCR_SWEEP_LINEAR)
    return calculateLinearSweepPointCount(settings);

  return calculateLogSweepPointCount(settings);
}


// Calculate the frequency for one point in a linear Sweep.
// Frequencies advance from Start using the configured fixed step size.
// The final generated frequency is limited to the configured Stop value.
uint32_t calculateLinearSweepFrequency(const SweepSettings &settings,
                                       uint32_t pointIndex)
{
  uint64_t frequency =
    static_cast<uint64_t>(settings.startFrequency) +
    static_cast<uint64_t>(pointIndex) *
    static_cast<uint64_t>(settings.stepFrequency);

  if (frequency > settings.stopFrequency)
    frequency = settings.stopFrequency;

  return static_cast<uint32_t>(frequency);
}


// Calculate the frequency for one point in a logarithmic Sweep.
// Each point advances by the configured number of points per decade.
// The final generated frequency is limited to the configured Stop value.
uint32_t calculateLogSweepFrequency(const SweepSettings &settings,
                                    uint32_t pointIndex)
{
  if (pointIndex == 0)
    return settings.startFrequency;

  float exponent =
    static_cast<float>(pointIndex) /
    static_cast<float>(settings.pointsPerDecade);

  float frequency =
    static_cast<float>(settings.startFrequency) *
    powf(10.0f, exponent);

  if (frequency > settings.stopFrequency)
    frequency = settings.stopFrequency;

  return static_cast<uint32_t>(
    roundf(frequency)
  );
}


// Calculate the requested frequency for any Sweep point.
// The Sweep engine uses this function without needing to know whether the
// active configuration is Linear or Logarithmic.
uint32_t calculateSweepFrequency(const SweepSettings &settings,
                                 uint32_t pointIndex)
{
  if (settings.mode == LCR_SWEEP_LINEAR) {
    return calculateLinearSweepFrequency(
      settings,
      pointIndex
    );
  }

  return calculateLogSweepFrequency(
    settings,
    pointIndex
  );
}



// Validate the complete Sweep configuration before acquisition begins.
// In addition to checking the frequency parameters, this prevents a Sweep
// configuration from exceeding the allocated result-storage capacity.
bool isLCRSweepConfigurationValid(const SweepSettings &settings)
{
  if (settings.startFrequency == 0)
    return false;

  if (settings.stopFrequency <= settings.startFrequency)
    return false;

  if (settings.mode == LCR_SWEEP_LINEAR) {
    if (settings.stepFrequency == 0)
      return false;
  }

  if (settings.mode == LCR_SWEEP_LOG) {
    if (settings.pointsPerDecade == 0)
      return false;
  }

  uint32_t pointCount = calculateSweepPointCount(settings);

  if (pointCount == 0)
    return false;

  if (pointCount > LCR_MAX_SWEEP_POINTS)
    return false;

  return true;
}




// Calculate geometry shared by modal selectors.
// Selector dimensions are controlled by the common LCR UI tuning constants
// so visual adjustments do not require changes to the layout algorithm.
void calculateSelectorLayout()
{
  const LCRRect &content = lcrLayout.content;
  const LCRRect &footer = lcrLayout.footer;
  const int16_t selectorH = content.h + footer.h;

  lcrLayout.selector = {
    content.x,
    content.y,
    content.w,
    selectorH
  };

  const int16_t cancelW =
    lcrLayout.selector.w *
    LCR_SELECTOR_CANCEL_WIDTH_PERCENT / 100;

  const int16_t cancelH =
    lcrLayout.selector.h *
    LCR_SELECTOR_CANCEL_HEIGHT_PERCENT / 100;

  const int16_t cancelX =
    lcrLayout.selector.x +
    (lcrLayout.selector.w - cancelW) / 2;

  const int16_t cancelY =
    lcrLayout.selector.y +
    lcrLayout.selector.h -
    cancelH -
    lcrLayout.selector.h *
      LCR_SELECTOR_CANCEL_BOTTOM_PERCENT / 100;

  lcrLayout.selectorCancel = {
    cancelX,
    cancelY,
    cancelW,
    cancelH
  };
}



// Arrange a variable number of selector buttons into a centered grid.
// Button count determines the grid arrangement while common UI tuning
// constants control spacing and maximum button size.
void calculateSelectorButtonGrid(LCRRect *buttons, uint8_t count)
{
  if (count == 0)
    return;

  count = min(count, LCR_MAX_SELECTOR_BUTTONS);

  const LCRRect &selector = lcrLayout.selector;

  uint8_t columns;

  if (count <= 3)
    columns = count;
  else if (count == 4)
    columns = 2;
  else
    columns = 3;

  uint8_t rows = (count + columns - 1) / columns;

  const int16_t marginX = selector.w * LCR_SELECTOR_MARGIN_PERCENT / 100;
  const int16_t gapX = selector.w * LCR_SELECTOR_GAP_X_PERCENT / 100;
  const int16_t gapY = selector.h * LCR_SELECTOR_GAP_Y_PERCENT / 100;
  const int16_t gridTop = selector.y + selector.h * 
                            LCR_SELECTOR_TOP_PERCENT / 100;

  const int16_t gridBottom =
    lcrLayout.selectorCancel.y -
    selector.h * LCR_SELECTOR_BOTTOM_GAP_PERCENT / 100;

  const int16_t availableGridH = gridBottom - gridTop;

  const int16_t buttonW =
    (selector.w -
     marginX * 2 -
     gapX * (columns - 1)) / columns;

  const int16_t maxButtonH = display.height() * 
                              LCR_SELECTOR_BUTTON_HEIGHT_PERCENT / 100;

  int16_t buttonH = (availableGridH - gapY * (rows - 1)) / rows;

  buttonH = min(buttonH, maxButtonH);

  const int16_t actualGridH = rows * buttonH + (rows - 1) * gapY;
  const int16_t centeredGridTop = gridTop + (availableGridH - actualGridH) / 2;

  for (uint8_t i = 0; i < count; i++) {
    uint8_t row = i / columns;
    uint8_t column = i % columns;

    uint8_t itemsInRow =
      min(
        static_cast<uint8_t>(columns),
        static_cast<uint8_t>(count - row * columns)
      );

    int16_t rowWidth = itemsInRow * buttonW + (itemsInRow - 1) * gapX;
    int16_t rowX = selector.x + (selector.w - rowWidth) / 2;
    int16_t x = rowX + column * (buttonW + gapX);
    int16_t y = centeredGridTop + row * (buttonH + gapY);

    buttons[i] = { x, y, buttonW, buttonH };
  }
}



// Calculate frequency-selector geometry from the frequency preset table.
// No button count or arrangement is duplicated outside the preset data.
void calculateFrequencySelectorLayout()
{
  calculateSelectorButtonGrid(
    lcrLayout.frequencyPresets,
    FREQUENCY_PRESET_COUNT
  );
}


// Calculate reference-selector geometry from the reference preset table.
// No button count or arrangement is duplicated outside the preset data.
void calculateReferenceSelectorLayout()
{
  calculateSelectorButtonGrid(
    lcrLayout.referencePresets,
    REFERENCE_PRESET_COUNT
  );
}


// Calculate Sweep-mode selector geometry using the shared selector grid.
// The two available modes are automatically positioned within the common
// modal selector region.
void calculateSweepModeSelectorLayout()
{
  calculateSelectorButtonGrid(
    lcrLayout.sweepModePresets,
    2
  );
}


// Calculate linear Sweep-step selector geometry using the shared dynamic
// selector grid and the number of entries in the Sweep-step preset table.
void calculateSweepStepSelectorLayout()
{
  calculateSelectorButtonGrid(
    lcrLayout.sweepStepPresets,
    SWEEP_STEP_PRESET_COUNT
  );
}


// Calculate logarithmic Sweep-density selector geometry using the shared
// dynamic selector grid and the number of available density presets.
void calculateSweepDensitySelectorLayout()
{
  calculateSelectorButtonGrid(
    lcrLayout.sweepDensityPresets,
    SWEEP_DENSITY_PRESET_COUNT
  );
}


// Calculate Sweep Results plot-selector geometry using the common dynamic
// selector grid and the number of available plot quantities.
void calculateSweepPlotSelectorLayout()
{
  calculateSelectorButtonGrid(
    lcrLayout.sweepPlotPresets,
    SWEEP_PLOT_PRESET_COUNT
  );
}


// Initialize the LCR analyzer and calculate all currently supported UI
// geometry before drawing the instrument screen.
void initializeLCR()
{
  calculateLCRLayout();
  calculateMeasureLayout();
  calculateSweepLayout();
  calculateSelectorLayout();
  calculateFrequencySelectorLayout();
  calculateReferenceSelectorLayout();
  calculateSweepModeSelectorLayout();
  calculateSweepStepSelectorLayout();
  calculateSweepDensitySelectorLayout();
  calculateSweepPlotSelectorLayout();

  initializeLCRValueSprite();
  initializeLCRSweepSprite();

  display.fillScreen(BGCOLOR);

  //
  // Future initialization:
  //
  // - Initialize AD9833
  // - Configure ADC/DMA
  // - Reset measurement state
  // - Load calibration data
  //
}



// Return true when a screen coordinate lies inside a rectangular UI region.
// This allows the same calculated geometry to be shared by drawing and
// touchscreen hit detection.
bool pointInLCRRect(uint16_t x, uint16_t y, const LCRRect &rect)
{
  return x >= rect.x &&
         x < rect.x + rect.w &&
         y >= rect.y &&
         y < rect.y + rect.h;
}


// value formatters
//

// Format a raw measurement using the supplied engineering scale and unit.
// This helper keeps engineering-prefix selection separate from the display
// code so the same formatting can be reused throughout the analyzer.
LCRFormattedValue formatEngineeringValue(float value,
                                          float scale,
                                          const char *unit,
                                          uint8_t decimals)
{
  LCRFormattedValue result;
  float scaledValue = value * scale;
  snprintf( result.value, sizeof(result.value), "%.*f", decimals, scaledValue);
  snprintf( result.unit, sizeof(result.unit), "%s", unit);

  return result;
}


// Format capacitance using an appropriate engineering unit.
// The selected unit keeps the displayed numeric value within a practical
// range while the underlying measurement remains stored in farads.
LCRFormattedValue formatCapacitance(float farads)
{
  float magnitude = fabs(farads);

  if (magnitude < 1.0e-9f) {
    return formatEngineeringValue( farads, 1.0e12f, "pF", 2);
  }

  if (magnitude < 1.0e-6f) {
    return formatEngineeringValue( farads, 1.0e9f, "nF", 2);
  }

  if (magnitude < 1.0e-3f) {
    return formatEngineeringValue( farads, 1.0e6f, "uF", 2);
  }

  return formatEngineeringValue( farads, 1.0f, "F", 3);
}


// Format inductance using an appropriate engineering unit.
// Raw inductance remains stored in henries while the displayed value is
// scaled to uH, mH, or H as appropriate.
LCRFormattedValue formatInductance(float henries)
{
  float magnitude = fabs(henries);

  if (magnitude < 1.0e-3f) {
    return formatEngineeringValue( henries, 1.0e6f, "uH", 2);
  }

  if (magnitude < 1.0f) {
    return formatEngineeringValue( henries, 1.0e3f, "mH", 2);
  }

  return formatEngineeringValue( henries, 1.0f, "H", 3);
}


// Format impedance or resistance using Ohm, kOhm, or MOhm.
// This formatter can be reused for impedance magnitude, resistance,
// reactance, ESR, and reference-resistor values.
LCRFormattedValue formatImpedance(float ohms)
{
  float magnitude = fabs(ohms);

  if (magnitude >= 1.0e6f) {
    return formatEngineeringValue( ohms, 1.0e-6f, "MOhm", 2);
  }

  if (magnitude >= 1.0e3f) {
    return formatEngineeringValue( ohms, 1.0e-3f, "kOhm", 2);
  }

  return formatEngineeringValue( ohms, 1.0f, "Ohm", 2);
}


// Format frequency using compact engineering notation.
// Decimal precision decreases as the displayed magnitude increases,
// providing roughly four significant digits without unnecessary zeros.
LCRFormattedValue formatFrequency(uint32_t frequencyHz)
{
  if (frequencyHz >= 1000000UL) {
    float frequencyMHz = frequencyHz * 1.0e-6f;

    uint8_t decimals =
      frequencyMHz < 10.0f ? 3 :
      frequencyMHz < 100.0f ? 2 : 1;

    return formatEngineeringValue( frequencyHz, 1.0e-6f, "MHz", decimals);
  }

  if (frequencyHz >= 1000UL) {
    float frequencyKHz = frequencyHz * 1.0e-3f;

    uint8_t decimals =
      frequencyKHz < 10.0f ? 3 :
      frequencyKHz < 100.0f ? 2 : 1;

    return formatEngineeringValue( frequencyHz, 1.0e-3f, "kHz", decimals);
  }

  return formatEngineeringValue( frequencyHz, 1.0f, "Hz", 0);
}


// Print a frequency at the current display cursor using compact engineering
// units. Frequency remains stored internally as integer Hz.
void printLCRFrequency(uint32_t frequencyHz)
{
  LCRFormattedValue formatted = formatFrequency(frequencyHz);

  display.print(formatted.value);
  display.print(" ");
  display.print(formatted.unit);
}


// Format an LCR value into a compact single-string axis label.
// The existing engineering formatter supplies the numeric value and unit;
// this helper combines them for use around Sweep plots.
void makeLCRAxisLabel(char *buffer,
                      size_t bufferSize,
                      const LCRFormattedValue &formatted)
{
  snprintf(
    buffer,
    bufferSize,
    "%s %s",
    formatted.value,
    formatted.unit
  );
}


//
// Touch handlers
//

// Handle touches on the four top-level analyzer tabs.
// Leaving the Measure tab resets measurement state to LIVE so returning
// to Measure always resumes acquisition rather than restoring stale HOLD data.
void handleLCRTabTouch(uint16_t x, uint16_t y)
{
  for (uint8_t i = 0; i < 4; i++) {
    if (!pointInLCRRect(
          x,
          y,
          lcrLayout.tabButtons[i])) {

      continue;
    }

    LCRTab newTab = static_cast<LCRTab>(i);

    // Ignore touches on the already active tab.
    if (newTab == lcrTab)
      return;

    // HOLD data is only meaningful while remaining on the Measure tab.
    // Leaving Measure resets it so the next visit begins with live data.
    if (lcrTab == LCR_TAB_MEASURE) {
      lcrMeasureState = LCR_MEASURE_LIVE;
    }

    lcrTab = newTab;
    lcrUIState = LCR_UI_NORMAL;
    lcrDisplayDirty = true;

    drawLCRScreen();
    return;
  }
}


// Handle touches within the Measure-tab soft-key footer.
// Frequency, reference-resistor, and LIVE/HOLD controls are active;
// More remains reserved for a future milestone.
void handleMeasureSoftKeyTouch(uint16_t x, uint16_t y)
{
  // Open the frequency preset selector.
  if (pointInLCRRect(x, y, lcrLayout.measureSoftKeys[0])) {
    lcrFrequencyTarget = LCR_FREQ_MEASURE;
    lcrUIState = LCR_UI_FREQ_SELECT;
    drawLCRFrequencySelector();
    return;
  }

  // Open the reference-resistor preset selector.
  if (pointInLCRRect(x, y, lcrLayout.measureSoftKeys[1])) {
    lcrUIState = LCR_UI_REF_SELECT;
    drawLCRReferenceSelector();
    return;
  }

  // LIVE / HOLD
  if (pointInLCRRect(x, y, lcrLayout.measureSoftKeys[2])) {
    if (lcrMeasureState == LCR_MEASURE_LIVE)
      lcrMeasureState = LCR_MEASURE_HOLD;
    else
      lcrMeasureState = LCR_MEASURE_LIVE;

    drawMeasureSoftKeys();

    // Force an immediate measurement when returning to LIVE.
    if (lcrMeasureState == LCR_MEASURE_LIVE)
      lcrDisplayDirty = true;

    return;
  }

  // More - implemented in a future milestone.
  if (pointInLCRRect(x, y, lcrLayout.measureSoftKeys[3])) {
    return;
  }
}


// Handle touch input while the shared frequency selector is displayed.
// The selected value is applied to whichever frequency setting opened the
// selector: Measure frequency, Sweep Start, or Sweep Stop. Cancel returns
// to the originating screen without modifying the current setting.
void handleLCRFrequencySelectorTouch(uint16_t x, uint16_t y)
{
  // Handle selectable frequency presets. Disabled Sweep limits ignore touch
  // input so an invalid Start/Stop combination cannot be created through
  // the normal user interface.
  for (uint8_t i = 0; i < FREQUENCY_PRESET_COUNT; i++) {
    if (!pointInLCRRect(
          x,
          y,
          lcrLayout.frequencyPresets[i])) {

      continue;
    }

    uint32_t frequency = frequencyPresets[i].value;

    if (!isLCRFrequencyPresetEnabled(frequency))
      return;

    setLCRSelectedFrequency(frequency);

    if (lcrFrequencyTarget == LCR_FREQ_MEASURE) {
      returnToLCRMeasureScreen();
    } else {
      lcrUIState = LCR_UI_NORMAL;
      lcrDisplayDirty = true;

      drawLCRScreen();
    }

    return;
  }

  // Cancel closes the selector without changing the current frequency.
  if (pointInLCRRect(
        x,
        y,
        lcrLayout.selectorCancel)) {

    if (lcrFrequencyTarget == LCR_FREQ_MEASURE) {
      returnToLCRMeasureScreen();
    } else {
      lcrUIState = LCR_UI_NORMAL;

      drawLCRScreen();
    }

    return;
  }
}


// Handle touch input while the reference-resistor selector is displayed.
// Selecting a reference updates the shared measurement settings and resumes
// LIVE acquisition; Cancel closes the selector without changing anything.
void handleLCRReferenceSelectorTouch(uint16_t x, uint16_t y)
{
  // Match touches against the reference preset table so button labels,
  // values, layout, and touch behavior all originate from one definition.
  for (uint8_t i = 0; i < REFERENCE_PRESET_COUNT; i++) {
    if (pointInLCRRect(
          x,
          y,
          lcrLayout.referencePresets[i])) {

      lcrSettings.referenceResistance = referencePresets[i].value;
      lcrMeasureState = LCR_MEASURE_LIVE;

      returnToLCRMeasureScreen();
      return;
    }
  }

  if (pointInLCRRect( x, y, lcrLayout.selectorCancel)) {
    returnToLCRMeasureScreen();
    return;
  }
}


// Handle interactive controls on the Sweep Setup screen.
// Start and Stop currently reuse the common frequency preset selector;
// Mode and Step are added in subsequent Sweep configuration milestones.
void handleLCRSweepSetupTouch(uint16_t x, uint16_t y)
{
  // Open the Sweep-Start selector.
  if (pointInLCRRect( x, y, lcrLayout.sweepSetupRows[0])) {
    lcrFrequencyTarget = LCR_FREQ_SWEEP_START;
    lcrUIState = LCR_UI_FREQ_SELECT;

    drawLCRFrequencySelector();
    return;
  }

  // Open the Sweep-Stop selector.
  if (pointInLCRRect( x, y, lcrLayout.sweepSetupRows[1])) {
    lcrFrequencyTarget = LCR_FREQ_SWEEP_STOP;
    lcrUIState = LCR_UI_FREQ_SELECT;

    drawLCRFrequencySelector();
    return;
  }

  // Open the Sweep-mode selector.
  if (pointInLCRRect( x, y, lcrLayout.sweepSetupRows[2])) {
    lcrUIState = LCR_UI_SWEEP_MODE_SELECT;

    drawLCRSweepModeSelector();
    return;
  }

  // Open the selector appropriate for the current Sweep mode.
  // Linear mode selects a frequency step; Log mode selects points per decade.
  if (pointInLCRRect( x, y, lcrLayout.sweepSetupRows[3])) {
    if (lcrSweepSettings.mode == LCR_SWEEP_LINEAR) {
      lcrUIState = LCR_UI_SWEEP_STEP_SELECT;

      drawLCRSweepStepSelector();
    } else {
      lcrUIState = LCR_UI_SWEEP_DENSITY_SELECT;

      drawLCRSweepDensitySelector();
    }

    return;
  }
}


// Handle the Sweep action shown in the Setup footer.
// Sweep execution begins only when the current configuration passes the
// same validation used by the acquisition engine.
void handleLCRSweepFooterTouch(uint16_t x, uint16_t y)
{
  if (!pointInLCRRect(
        x,
        y,
        lcrLayout.sweepButton)) {

    return;
  }

  startLCRSweep();
}


// Handle touch input while a Sweep is running.
// The only active Sweep control during acquisition is Cancel.
void handleLCRSweepRunningTouch(uint16_t x, uint16_t y)
{
  if (pointInLCRRect(
        x,
        y,
        lcrLayout.sweepCancelButton)) {

    cancelLCRSweep();
    return;
  }
}


// Handle touch input while Sweep Results are displayed.
// Setup returns to the existing Sweep configuration, while Sweep starts
// another acquisition immediately using the current settings.
void handleLCRSweepResultsTouch(uint16_t x, uint16_t y)
{
  // Open the plot selector when the Results control strip is touched.
  if (pointInLCRRect( x, y, lcrLayout.sweepPlotControl)) {
    lcrUIState = LCR_UI_SWEEP_PLOT_SELECT;
    drawLCRSweepPlotSelector();
    return;
  }

  // Select the Sweep result nearest the touched horizontal graph position.
  // Touching another location simply moves the existing inspection cursor.
  if (pointInLCRRect(x, y, lcrLayout.sweepPlot)) {
    if (lcrSweepPointCount == 0)
      return;

    lcrSweepCursorIndex = findNearestLCRSweepPoint(x);
    lcrSweepCursorActive = true;
    drawLCRSweepResults();
    return;
  }

  // Return to Sweep Setup without changing the current configuration.
  if (pointInLCRRect( x, y, lcrLayout.sweepResultsSetupButton)) {
    lcrSweepState = LCR_SWEEP_SETUP;
    lcrUIState = LCR_UI_NORMAL;
    drawLCRScreen();
    return;
  }

  // Run another Sweep using the current configuration.
  if (pointInLCRRect( x, y, lcrLayout.sweepResultsSweepButton)) {
    startLCRSweep();
    return;
  }
}


// Handle touch input while the Sweep-mode selector is displayed.
// Selecting a mode updates the active Sweep configuration and returns to
// Sweep Setup; Cancel returns without modifying the current mode.
void handleLCRSweepModeSelectorTouch(uint16_t x, uint16_t y)
{
  // Linear
  if (pointInLCRRect( x, y, lcrLayout.sweepModePresets[0])) {

    lcrSweepSettings.mode = LCR_SWEEP_LINEAR;

    lcrUIState = LCR_UI_NORMAL;

    drawLCRScreen();
    return;
  }

  // Logarithmic
  if (pointInLCRRect( x, y, lcrLayout.sweepModePresets[1])) {

    lcrSweepSettings.mode = LCR_SWEEP_LOG;

    lcrUIState = LCR_UI_NORMAL;

    drawLCRScreen();
    return;
  }

  // Cancel
  if (pointInLCRRect( x, y, lcrLayout.selectorCancel)) {
    lcrUIState = LCR_UI_NORMAL;

    drawLCRScreen();
    return;
  }
}



// Handle touch input while the linear Sweep-step selector is displayed.
// Selecting a step updates Sweep configuration and returns to Setup;
// Cancel returns without changing the current step.
void handleLCRSweepStepSelectorTouch(uint16_t x, uint16_t y)
{
  for (uint8_t i = 0; i < SWEEP_STEP_PRESET_COUNT; i++) {
    if (pointInLCRRect( x, y, lcrLayout.sweepStepPresets[i])) {
      lcrSweepSettings.stepFrequency = sweepStepPresets[i].value;
      lcrUIState = LCR_UI_NORMAL;

      drawLCRScreen();
      return;
    }
  }

  if (pointInLCRRect( x, y, lcrLayout.selectorCancel)) {
    lcrUIState = LCR_UI_NORMAL;

    drawLCRScreen();
    return;
  }
}


// Handle touch input while the logarithmic Sweep-density selector is shown.
// Selecting a value updates points-per-decade and returns to Sweep Setup;
// Cancel returns without modifying the current density.
void handleLCRSweepDensitySelectorTouch(uint16_t x, uint16_t y)
{
  for (uint8_t i = 0; i < SWEEP_DENSITY_PRESET_COUNT; i++) {
    if (pointInLCRRect( x, y, lcrLayout.sweepDensityPresets[i])) {
      lcrSweepSettings.pointsPerDecade = sweepDensityPresets[i].value;
      lcrUIState = LCR_UI_NORMAL;

      drawLCRScreen();
      return;
    }
  }

  if (pointInLCRRect( x, y, lcrLayout.selectorCancel)) {
    lcrUIState = LCR_UI_NORMAL;

    drawLCRScreen();
    return;
  }
}


// Handle touch input while the Sweep Results plot selector is displayed.
// Selecting a quantity changes only the Results presentation; retained Sweep
// measurements remain unchanged and are immediately redrawn using that data.
void handleLCRSweepPlotSelectorTouch(uint16_t x, uint16_t y)
{
  for (uint8_t i = 0; i < SWEEP_PLOT_PRESET_COUNT; i++) {
    if (pointInLCRRect( x, y, lcrLayout.sweepPlotPresets[i])) {
      lcrSweepPlotType = sweepPlotPresets[i].type;
      lcrUIState = LCR_UI_NORMAL;

      drawLCRScreen();
      return;
    }
  }

  // Cancel returns to Results without changing the displayed quantity.
  if (pointInLCRRect( x, y, lcrLayout.selectorCancel)) {
    lcrUIState = LCR_UI_NORMAL;

    drawLCRScreen();
    return;
  }
}


//
// drawLCRScreen helpers
//


// text scaler for different sized fonts
uint8_t lcrTextScale(uint8_t baseSize)
{
  int16_t scaleX = display.width() / 320;
  int16_t scaleY = display.height() / 240;

  int16_t scale = min(scaleX, scaleY);

  if (scale < 1)
    scale = 1;

  return baseSize * scale;
}


// Draw text horizontally centered within an LCR layout region.
// The caller supplies the vertical position because different measurement
// fields may use different font sizes and vertical arrangements.
void drawLCRCenteredText(const LCRRect &rect, int16_t y,
                         const char *text, uint8_t textSize,
                         uint16_t color)
{
  display.setTextSize(textSize);
  display.setTextColor(color, BGCOLOR);

  int16_t textWidth = strlen(text) * 6 * textSize;
  int16_t textX = rect.x + (rect.w - textWidth) / 2;

  display.setCursor(textX, y);
  display.print(text);
}


//
// Draw Measurement screen routines
//


// Draw a measurement value and its unit as one horizontally centered group.
// The value may use a larger font than the unit while the combined pair
// remains visually centered within the supplied layout region.
void drawLCRCenteredMeasurement(const LCRRect &rect, int16_t y,
                                const char *value, const char *unit,
                                uint8_t valueSize, uint8_t unitSize,
                                uint16_t color)
{
  int16_t valueWidth = strlen(value) * 6 * valueSize;
  int16_t unitWidth = strlen(unit) * 6 * unitSize;

  // Add a small gap between the numeric value and engineering unit.
  int16_t gap = 4;

  int16_t totalWidth = valueWidth + gap + unitWidth;
  int16_t startX = rect.x + (rect.w - totalWidth) / 2;

  display.setTextColor(color, BGCOLOR);

  // Draw the numeric value.
  display.setTextSize(valueSize);
  display.setCursor(startX, y);
  display.print(value);

  // Align the smaller unit near the baseline of the numeric value.
  int16_t unitY = y + (8 * valueSize) - (8 * unitSize);

  display.setTextSize(unitSize);
  display.setCursor(startX + valueWidth + gap, unitY);
  display.print(unit);
}


// Draw a value and engineering unit as one centered group inside the
// reusable sprite. The numeric value and unit may use different text sizes
// while remaining visually centered as a single measurement.
void drawLCRSpriteMeasurement(int16_t width, int16_t height,
                              const char *value, const char *unit,
                              uint8_t valueSize, uint8_t unitSize,
                              uint16_t color)
{
  int16_t valueWidth = strlen(value) * 6 * valueSize;
  int16_t unitWidth = strlen(unit) * 6 * unitSize;
  int16_t gap = 4;

  int16_t totalWidth = valueWidth + gap + unitWidth;
  int16_t startX = (width - totalWidth) / 2;

  int16_t valueHeight = 8 * valueSize;
  int16_t unitHeight = 8 * unitSize;

  int16_t valueY = (height - valueHeight) / 2;
  int16_t unitY = valueY + valueHeight - unitHeight;

  lcrValueSprite.setTextColor(color, BGCOLOR);

  lcrValueSprite.setTextSize(valueSize);
  lcrValueSprite.setCursor(startX, valueY);
  lcrValueSprite.print(value);

  lcrValueSprite.setTextSize(unitSize);
  lcrValueSprite.setCursor(
    startX + valueWidth + gap,
    unitY
  );
  lcrValueSprite.print(unit);
}



// Render the primary measurement using automatically selected engineering
// units. The completed value/unit pair is composed in the sprite and pushed
// to the TFT in one operation to prevent visible refresh flicker.
void drawLCRPrimaryMeasurement(const MeasurementPoint &m)
{
  const LCRRect &r = lcrLayout.primary;

  LCRFormattedValue formatted = formatCapacitance(m.capacitance);

  lcrValueSprite.fillSprite(BGCOLOR);

  drawLCRSpriteMeasurement( 
      r.w,
      r.h,
      formatted.value,
      formatted.unit,
      4,
      2,
      TXTCOLOR
    );

  lcrValueSprite.pushSprite( r.x, r.y, 0, 0, r.w, r.h);
}



// Render the secondary measurement as a horizontal label/value pair.
// The complete field is composed in RAM and pushed to the TFT at once to
// avoid visible clearing between successive display updates.
void drawLCRSecondaryMeasurement(const MeasurementPoint &m)
{
  const LCRRect &r = lcrLayout.secondary;

  char value[16];

  snprintf(value, sizeof(value), "%.4f", m.dissipation);

  const char *label = "D";

  uint8_t labelSize = 1;
  uint8_t valueSize = 2;

  int16_t labelWidth = strlen(label) * 6 * labelSize;
  int16_t valueWidth = strlen(value) * 6 * valueSize;
  int16_t gap = 12;

  int16_t totalWidth = labelWidth + gap + valueWidth;
  int16_t startX = (r.w - totalWidth) / 2;
  int16_t valueHeight = 8 * valueSize;
  int16_t valueY = (r.h - valueHeight) / 2;
  int16_t labelY = valueY + valueHeight - (8 * labelSize);

  lcrValueSprite.fillSprite(BGCOLOR);
  lcrValueSprite.setTextColor(TXTCOLOR, BGCOLOR);

  lcrValueSprite.setTextSize(labelSize);
  lcrValueSprite.setCursor(startX, labelY);
  lcrValueSprite.print(label);

  lcrValueSprite.setTextSize(valueSize);
  lcrValueSprite.setCursor(
    startX + labelWidth + gap,
    valueY
  );
  lcrValueSprite.print(value);

  lcrValueSprite.pushSprite( r.x, r.y, 0, 0, r.w, r.h);
}



// Update the dynamic measurement context fields shown beneath the main
// measurement. Positions are derived from the context region so they remain
// aligned with the static labels across different display resolutions.
void drawLCRMeasurementContext(const MeasurementPoint &m,
                               const MeasurementSettings &settings)
{
  const LCRRect &r = lcrLayout.context;

  int16_t leftValueX = r.x + r.w * 22 / 100;
  int16_t rightValueX = r.x + r.w * 72 / 100;

  int16_t row1Y = r.y + r.h * 20 / 100;
  int16_t row2Y = r.y + r.h * 60 / 100;

  display.setTextSize(1);
  display.setTextColor(TXTCOLOR, BGCOLOR);

  // Frequency
  // Format the active measurement frequency using engineering units
  // so the context display remains compact and easy to read.
  LCRFormattedValue frequency = formatFrequency(m.frequency);
  display.setCursor(leftValueX, row1Y);
  display.print(frequency.value);
  display.print(" ");
  display.print(frequency.unit);

  // Format the selected reference resistor using engineering units so values
  // such as 1000 Ohm and 10000 Ohm display more readably as kOhm.
  LCRFormattedValue reference = formatImpedance(settings.referenceResistance);
  display.setCursor(rightValueX, row1Y);
  display.print(reference.value);
  display.print(" ");
  display.print(reference.unit);

  // Temporary excitation level.
  display.setCursor(leftValueX, row2Y);
  display.print("1.0 V");

  // Temporary averaging setting.
  display.setCursor(rightValueX, row2Y);
  display.print("32");
}


//
// Sweep screen draw routines
//


// Draw the Sweep Setup view using shared row geometry and the active sweep
// configuration. Row rectangles are also used for touch detection so the
// displayed controls and interactive areas remain synchronized.
void drawLCRSweepSetup()
{
  const LCRRect &r = lcrLayout.content;

  display.fillRect(
    r.x,
    r.y,
    r.w,
    r.h,
    BGCOLOR
  );

  display.setTextSize(1);
  display.setTextColor(TXTCOLOR, BGCOLOR);

  const int16_t leftX = r.x + r.w * 10 / 100;
  const int16_t valueX = r.x + r.w * 55 / 100;

  // Start frequency.
  const LCRRect &startRow = lcrLayout.sweepSetupRows[0];
  int16_t y = startRow.y + (startRow.h - 8) / 2;

  display.setCursor(leftX, y);
  display.print("Start");

  display.setCursor(valueX, y);
  printLCRFrequency( lcrSweepSettings.startFrequency);

  // Stop frequency.
  const LCRRect &stopRow = lcrLayout.sweepSetupRows[1];

  y = stopRow.y + (stopRow.h - 8) / 2;

  display.setCursor(leftX, y);
  display.print("Stop");

  display.setCursor(valueX, y);
  printLCRFrequency( lcrSweepSettings.stopFrequency);

  // Sweep mode.
  const LCRRect &modeRow = lcrLayout.sweepSetupRows[2];

  y = modeRow.y + (modeRow.h - 8) / 2;

  display.setCursor(leftX, y);
  display.print("Mode");

  display.setCursor(valueX, y);

  if (lcrSweepSettings.mode == LCR_SWEEP_LINEAR)
    display.print("Linear");
  else
    display.print("Log");

  // Linear step or logarithmic density.
  const LCRRect &stepRow = lcrLayout.sweepSetupRows[3];

  y = stepRow.y + (stepRow.h - 8) / 2;

  display.setCursor(leftX, y);

  if (lcrSweepSettings.mode == LCR_SWEEP_LINEAR)
    display.print("Step");
  else
    display.print("Points/Dec");

  display.setCursor(valueX, y);

  if (lcrSweepSettings.mode == LCR_SWEEP_LINEAR) {
    printLCRFrequency(
      lcrSweepSettings.stepFrequency
    );
  } else {
    display.print(
      lcrSweepSettings.pointsPerDecade
    );
  }

  // Derived measurement count.
  const LCRRect &countRow = lcrLayout.sweepSetupRows[4];

  y = countRow.y + (countRow.h - 8) / 2;

  display.setCursor(leftX, y);
  display.print("Measurements");

  display.setCursor(valueX, y);

  // Display the number of measurements derived from the active Sweep mode.
  if (lcrSweepSettings.mode == LCR_SWEEP_LINEAR) {
    display.print( calculateLinearSweepPointCount( lcrSweepSettings));
  } else {
    display.print( calculateLogSweepPointCount( lcrSweepSettings));
  }
}


// Draw a rectangular selector button with a centered text label.
// Selected controls use the highlight color, while disabled controls are
// visually muted and cannot be selected by their touch handlers.
void drawLCRSelectorButton(const LCRRect &rect,
                           const char *label,
                           bool selected,
                           bool enabled = true)
{
  uint16_t color;

  if (!enabled)
    color = TFT_DARKGREY;
  else if (selected)
    color = HIGHCOLOR;
  else
    color = TXTCOLOR;

  display.drawRect( rect.x, rect.y, rect.w, rect.h, color);

  display.setTextSize(1);
  display.setTextColor(color, BGCOLOR);

  int16_t textWidth = strlen(label) * 6;
  int16_t textX = rect.x + (rect.w - textWidth) / 2;
  int16_t textY = rect.y + (rect.h - 8) / 2;

  display.setCursor(textX, textY);
  display.print(label);
}


// Draw the Sweep Results quantity selector.
// The currently displayed quantity is highlighted and all available plot
// choices are generated from the shared Sweep plot preset table.
void drawLCRSweepPlotSelector()
{
  const LCRRect &r = lcrLayout.selector;

  display.fillRect( r.x, r.y, r.w, r.h, BGCOLOR);

  display.setTextSize(1);
  display.setTextColor(TXTCOLOR, BGCOLOR);

  const char *title = "Select Plot";
  int16_t titleWidth = strlen(title) * 6;

  display.setCursor( r.x + (r.w - titleWidth) / 2, r.y + r.h * 7 / 100);

  display.print(title);

  for (uint8_t i = 0;
       i < SWEEP_PLOT_PRESET_COUNT;
       i++) {

    bool selected = lcrSweepPlotType == sweepPlotPresets[i].type;

    drawLCRSelectorButton(
      lcrLayout.sweepPlotPresets[i],
      sweepPlotPresets[i].label,
      selected
    );
  }

  drawLCRSelectorButton( lcrLayout.selectorCancel, "Cancel", false);
}



// Draw the Sweep-mode selector over the Sweep Setup screen.
// The currently active mode is highlighted so the existing configuration
// remains visible before the user makes a selection.
void drawLCRSweepModeSelector()
{
  const LCRRect &r = lcrLayout.selector;

  display.fillRect(
    r.x,
    r.y,
    r.w,
    r.h,
    BGCOLOR
  );

  display.setTextSize(1);
  display.setTextColor(TXTCOLOR, BGCOLOR);

  const char *title = "Select Sweep Mode";

  int16_t titleWidth = strlen(title) * 6;

  display.setCursor(
    r.x + (r.w - titleWidth) / 2,
    r.y + r.h * 7 / 100
  );

  display.print(title);

  drawLCRSelectorButton(
    lcrLayout.sweepModePresets[0],
    "Linear",
    lcrSweepSettings.mode == LCR_SWEEP_LINEAR
  );

  drawLCRSelectorButton(
    lcrLayout.sweepModePresets[1],
    "Log",
    lcrSweepSettings.mode == LCR_SWEEP_LOG
  );

  drawLCRSelectorButton(
    lcrLayout.selectorCancel,
    "Cancel",
    false
  );
}


// Draw the linear Sweep-step selector.
// The current step is highlighted and all button labels and values come
// directly from the Sweep-step preset table.
void drawLCRSweepStepSelector()
{
  const LCRRect &r = lcrLayout.selector;

  display.fillRect(
    r.x,
    r.y,
    r.w,
    r.h,
    BGCOLOR
  );

  display.setTextSize(1);
  display.setTextColor(TXTCOLOR, BGCOLOR);

  const char *title = "Select Sweep Step";

  int16_t titleWidth = strlen(title) * 6;

  display.setCursor(
    r.x + (r.w - titleWidth) / 2,
    r.y + r.h * 7 / 100
  );

  display.print(title);

  for (uint8_t i = 0;
       i < SWEEP_STEP_PRESET_COUNT;
       i++) {

    bool selected = lcrSweepSettings.stepFrequency == sweepStepPresets[i].value;

    drawLCRSelectorButton(
      lcrLayout.sweepStepPresets[i],
      sweepStepPresets[i].label,
      selected
    );
  }

  drawLCRSelectorButton( lcrLayout.selectorCancel, "Cancel", false);
}



// Draw the logarithmic Sweep-density selector.
// The selected points-per-decade value is highlighted and all available
// choices come directly from the Sweep-density preset table.
void drawLCRSweepDensitySelector()
{
  const LCRRect &r = lcrLayout.selector;

  display.fillRect( r.x, r.y, r.w, r.h, BGCOLOR);

  display.setTextSize(1);
  display.setTextColor(TXTCOLOR, BGCOLOR);

  const char *title = "Select Points/Decade";

  int16_t titleWidth = strlen(title) * 6;

  display.setCursor(
    r.x + (r.w - titleWidth) / 2,
    r.y + r.h * 7 / 100
  );

  display.print(title);

  for (uint8_t i = 0;
       i < SWEEP_DENSITY_PRESET_COUNT;
       i++) {

    bool selected =
      lcrSweepSettings.pointsPerDecade ==
      sweepDensityPresets[i].value;

    drawLCRSelectorButton(
      lcrLayout.sweepDensityPresets[i],
      sweepDensityPresets[i].label,
      selected
    );
  }

  drawLCRSelectorButton(
    lcrLayout.selectorCancel,
    "Cancel",
    false
  );
}



// Draw the Sweep Setup footer.
// The Sweep action will become interactive once sweep configuration and
// acquisition state are implemented.
void drawLCRSweepFooter()
{
  const LCRRect &r = lcrLayout.footer;

  display.fillRect( r.x, r.y, r.w, r.h, BGCOLOR);
  display.drawFastHLine( r.x, r.y, r.w, GRIDCOLOR);

  const char *label = "Sweep";

  display.setTextSize(1);
  display.setTextColor(TXTCOLOR, BGCOLOR);

  int16_t textWidth = strlen(label) * 6;

  display.setCursor(
    r.x + (r.w - textWidth) / 2,
    r.y + (r.h - 8) / 2
  );

  display.print(label);
}


// Draw the Sweep Running interface.
// Dynamic Sweep content is rendered through the Sweep sprite, while the
// Cancel footer remains static for the duration of acquisition.
void drawLCRSweepRunning()
{
  const LCRRect &content = lcrLayout.content;
  const LCRRect &footer = lcrLayout.footer;

  display.fillRect( content.x, content.y, content.w, content.h, BGCOLOR);
  display.fillRect( footer.x, footer.y, footer.w, footer.h, BGCOLOR);
  display.drawFastHLine( footer.x, footer.y, footer.w, GRIDCOLOR);

  display.setTextSize(1);
  display.setTextColor(TXTCOLOR, BGCOLOR);

  const char *cancelLabel = "Cancel";

  int16_t cancelWidth = strlen(cancelLabel) * 6;

  display.setCursor(
    footer.x +
      (footer.w - cancelWidth) / 2,
    footer.y + (footer.h - 8) / 2
  );

  display.print(cancelLabel);

  updateLCRSweepRunningDisplay();
}



// Render all dynamic Sweep progress information into an off-screen sprite.
// The completed content region is transferred to the TFT in one operation,
// eliminating visible erase/redraw flicker during Sweep execution.
void updateLCRSweepRunningDisplay()
{
  if (lcrSweepExecution.totalPoints == 0)
    return;

  const LCRRect &content = lcrLayout.content;
  const LCRRect &progress = lcrLayout.sweepProgress;
  uint32_t completed = lcrSweepExecution.currentPoint;
  uint32_t total = lcrSweepExecution.totalPoints;

  lcrSweepSprite.fillSprite(BGCOLOR);

  lcrSweepSprite.setTextSize(1);
  lcrSweepSprite.setTextColor(TXTCOLOR, BGCOLOR);

  //
  // Title
  //

  const char *title = "Running Sweep";
  int16_t titleWidth = strlen(title) * 6;
  lcrSweepSprite.setCursor( (content.w - titleWidth) / 2, content.h * 8 / 100);
  lcrSweepSprite.print(title);

  //
  // Progress bar
  //

  int16_t progressX = progress.x - content.x;
  int16_t progressY = progress.y - content.y;
  lcrSweepSprite.drawRect(
    progressX,
    progressY,
    progress.w,
    progress.h,
    TXTCOLOR
  );

  int16_t innerW = progress.w - 2;

  int16_t fillW =
    static_cast<int16_t>(
      static_cast<uint64_t>(innerW) *
      completed / total
    );

  if (fillW > 0) {
    lcrSweepSprite.fillRect(
      progressX + 1,
      progressY + 1,
      fillW,
      progress.h - 2,
      HIGHCOLOR
    );
  }

  //
  // Point count
  //

  char buffer[40];

  snprintf(
    buffer,
    sizeof(buffer),
    "%lu / %lu",
    static_cast<unsigned long>(completed),
    static_cast<unsigned long>(total)
  );

  int16_t textTop = progressY + progress.h + 12;
  int16_t width = strlen(buffer) * 6;

  lcrSweepSprite.setCursor( (content.w - width) / 2, textTop);
  lcrSweepSprite.print(buffer);

  //
  // Current frequency
  //

  LCRFormattedValue frequency =
    formatFrequency(
      lcrSweepExecution.currentFrequency
    );

  snprintf(
    buffer,
    sizeof(buffer),
    "Current: %s %s",
    frequency.value,
    frequency.unit
  );

  width = strlen(buffer) * 6;

  lcrSweepSprite.setCursor( (content.w - width) / 2, textTop + 20);
  lcrSweepSprite.print(buffer);

  //
  // Estimated remaining time
  //

  uint32_t elapsed = millis() - lcrSweepExecution.startTime;
  uint32_t remainingMs = 0;

  if (completed > 0 && completed < total) {
    uint32_t averagePointMs = elapsed / completed;
    remainingMs = averagePointMs * (total - completed);
  }

  if (completed == 0) {
    snprintf( buffer, sizeof(buffer), "Remaining: --");
  } else {
    snprintf(
      buffer,
      sizeof(buffer),
      "Remaining: %.1f s",
      remainingMs / 1000.0f
    );
  }

  width = strlen(buffer) * 6;

  lcrSweepSprite.setCursor( (content.w - width) / 2, textTop + 40);
  lcrSweepSprite.print(buffer);

  //
  // Push the completed Running display to the TFT.
  //

  lcrSweepSprite.pushSprite( content.x, content.y);
}


// Draw the temporary Sweep Results screen after acquisition completes.
// Setup returns to Sweep configuration, while Sweep immediately repeats
// the acquisition using the current configuration.
void drawLCRSweepResults()
{
  const LCRRect &content = lcrLayout.content;
  const LCRRect &footer = lcrLayout.footer;

  display.fillRect(
    content.x,
    content.y,
    content.w,
    content.h,
    BGCOLOR
  );

  display.fillRect(
    footer.x,
    footer.y,
    footer.w,
    footer.h,
    BGCOLOR
  );

  display.setTextSize(1);
  display.setTextColor(TXTCOLOR, BGCOLOR);

  // Draw the interactive plot-selection control above the Results graph.
  drawLCRSweepPlotControl();

  // Draw the completed Sweep impedance-magnitude trace.
  drawLCRSweepPlot();

  // Footer separator.
  display.drawFastHLine(
    footer.x,
    footer.y,
    footer.w,
    GRIDCOLOR
  );

  // Separator between Setup and Sweep.
  display.drawFastVLine(
    lcrLayout.sweepResultsSweepButton.x,
    footer.y + 3,
    footer.h - 6,
    GRIDCOLOR
  );

  // Setup action.
  const char *setupLabel = "Setup";

  int16_t setupWidth = strlen(setupLabel) * 6;

  display.setCursor(
    lcrLayout.sweepResultsSetupButton.x +
      (lcrLayout.sweepResultsSetupButton.w - setupWidth) / 2,
    footer.y + (footer.h - 8) / 2
  );

  display.print(setupLabel);

  // Sweep action.
  const char *sweepLabel = "Sweep";

  int16_t sweepWidth = strlen(sweepLabel) * 6;

  display.setCursor(
    lcrLayout.sweepResultsSweepButton.x +
      (lcrLayout.sweepResultsSweepButton.w - sweepWidth) / 2,
    footer.y + (footer.h - 8) / 2
  );

  display.print(sweepLabel);
}



// Draw the common LCR analyzer header.
// The header provides a Back control and identifies the active instrument.
// Its dimensions are derived entirely from the calculated screen layout.
void drawLCRHeader()
{
  const LCRRect &r = lcrLayout.header;

  display.fillRect(r.x, r.y, r.w, r.h, BGCOLOR);

  // Bottom separator.
  display.drawFastHLine( r.x, r.y + r.h - 1, r.w, GRIDCOLOR);

  display.setTextSize(1);
  display.setTextColor(TXTCOLOR, BGCOLOR);

  // Draw the Back label centered inside its actual touch region so the
  // visible control accurately represents the area that responds to touch.
  const LCRRect &back = lcrLayout.backButton;
  const char *backLabel = "< Back";

  int16_t backWidth = strlen(backLabel) * 6;
  int16_t backX = back.x + (back.w - backWidth) / 2;
  int16_t backY = back.y + (back.h - 8) / 2;

  display.setCursor(backX, backY);
  display.print(backLabel);

  // Instrument title.
  const char *title = "LCR ANALYZER";
  int16_t titleWidth = strlen(title) * 6;
  int16_t titleX = r.x + (r.w - titleWidth) / 2;
  int16_t titleY = r.y + (r.h - 8) / 2;

  display.setCursor(titleX, titleY);
  display.print(title);
}



// Draw the four top-level LCR analyzer tabs.
// Each tab receives an equal share of the available width, and the active
// tab is highlighted using the existing GOscillo highlight color.
void drawLCRTabs()
{
  const LCRRect &r = lcrLayout.tabs;

  const char *tabLabels[] = {
    "Measure",
    "Sweep",
    "Cal",
    "Settings"
  };

  const int16_t tabCount = 4;
  const int16_t tabW = r.w / tabCount;

  display.fillRect(r.x, r.y, r.w, r.h, BGCOLOR);

  for (int16_t i = 0; i < tabCount; i++) {
    int16_t x = r.x + i * tabW;

    // Let the last tab absorb any pixels left over by integer division.
    int16_t w = (i == tabCount - 1)
      ? r.w - (tabW * i)
      : tabW;

    bool selected = (i == static_cast<int16_t>(lcrTab));

    uint16_t textColor = selected ? HIGHCOLOR : TXTCOLOR;

    display.setTextSize(1);
    display.setTextColor(textColor, BGCOLOR);

    int16_t textWidth = strlen(tabLabels[i]) * 6;
    int16_t textX = x + (w - textWidth) / 2;
    int16_t textY = r.y + (r.h - 8) / 2;

    display.setCursor(textX, textY);
    display.print(tabLabels[i]);

    // Vertical separator between adjacent tabs.
    if (i < tabCount - 1) {
      display.drawFastVLine( x + w - 1, r.y + 3, r.h - 6, GRIDCOLOR);
    }
  }

  // Bottom separator.
  display.drawFastHLine( r.x, r.y + r.h - 1, r.w, GRIDCOLOR);
}


// Draw the static portions of the Measure tab.
// Measurement values themselves are not drawn here; they are refreshed
// separately by updateLCRDisplay().
void drawMeasureScreen()
{
  const LCRRect &context = lcrLayout.context;

  display.setTextSize(1);
  display.setTextColor(TXTCOLOR, BGCOLOR);

  int16_t leftX = context.x + context.w * 5 / 100;
  int16_t rightX = context.x + context.w * 55 / 100;

  int16_t row1Y = context.y + context.h * 20 / 100;
  int16_t row2Y = context.y + context.h * 60 / 100;

  display.setCursor(leftX, row1Y);
  display.print("FREQ:");

  display.setCursor(rightX, row1Y);
  display.print("REF:");

  display.setCursor(leftX, row2Y);
  display.print("LEVEL:");

  display.setCursor(rightX, row2Y);
  display.print("AVG:");
}


// Draw the Measure-tab soft keys using the same calculated rectangles
// used for touch detection. The third key reflects the current LIVE/HOLD
// measurement state.
void drawMeasureSoftKeys()
{
  const char *labels[] = {
    "Freq",
    "Ref",
    lcrMeasureState == LCR_MEASURE_LIVE ? "LIVE" : "HOLD",
    "More"
  };

  display.fillRect(
    lcrLayout.footer.x,
    lcrLayout.footer.y,
    lcrLayout.footer.w,
    lcrLayout.footer.h,
    BGCOLOR
  );

  display.drawFastHLine(
    lcrLayout.footer.x,
    lcrLayout.footer.y,
    lcrLayout.footer.w,
    GRIDCOLOR
  );

  display.setTextSize(1);

  for (int16_t i = 0; i < 4; i++) {
    const LCRRect &r = lcrLayout.measureSoftKeys[i];

    int16_t textWidth = strlen(labels[i]) * 6;
    int16_t textX = r.x + (r.w - textWidth) / 2;
    int16_t textY = r.y + (r.h - 8) / 2;

    display.setTextColor(TXTCOLOR, BGCOLOR);
    display.setCursor(textX, textY);
    display.print(labels[i]);

    if (i < 3) {
      display.drawFastVLine( r.x + r.w - 1, r.y + 3, r.h - 6, GRIDCOLOR);
    }
  }
}


// Return the frequency associated with the currently active selector target.
// This lets the shared selector highlight the correct value for Measure,
// Sweep Start, or Sweep Stop without duplicating selector code.
uint32_t getLCRSelectedFrequency()
{
  switch (lcrFrequencyTarget) {
    case LCR_FREQ_SWEEP_START:
      return lcrSweepSettings.startFrequency;

    case LCR_FREQ_SWEEP_STOP:
      return lcrSweepSettings.stopFrequency;

    case LCR_FREQ_MEASURE:
    default:
      return lcrSettings.frequency;
  }
}



// Determine whether a frequency preset is valid for the setting currently
// being edited. Sweep Start must remain below Stop, and Sweep Stop must
// remain above Start. Measure frequency has no Sweep-range restriction.
bool isLCRFrequencyPresetEnabled(uint32_t frequency)
{
  switch (lcrFrequencyTarget) {
    case LCR_FREQ_SWEEP_START:
      return frequency <
        lcrSweepSettings.stopFrequency;

    case LCR_FREQ_SWEEP_STOP:
      return frequency >
        lcrSweepSettings.startFrequency;

    case LCR_FREQ_MEASURE:
    default:
      return true;
  }
}


// Store a selected frequency in the setting currently being edited.
// Measure-frequency changes resume LIVE acquisition, while Sweep settings
// return to the Sweep Setup screen for further configuration.
void setLCRSelectedFrequency(uint32_t frequency)
{
  switch (lcrFrequencyTarget) {
    case LCR_FREQ_SWEEP_START:
      lcrSweepSettings.startFrequency = frequency;
      break;

    case LCR_FREQ_SWEEP_STOP:
      lcrSweepSettings.stopFrequency = frequency;
      break;

    case LCR_FREQ_MEASURE:
    default:
      lcrSettings.frequency = frequency;
      lcrMeasureState = LCR_MEASURE_LIVE;
      break;
  }
}

// Draw the frequency preset selector over the active screen.
// The current value for the setting being edited is highlighted.
void drawLCRFrequencySelector()
{
  const LCRRect &r = lcrLayout.selector;

  display.fillRect(
    r.x,
    r.y,
    r.w,
    r.h,
    BGCOLOR
  );

  display.setTextSize(1);
  display.setTextColor(TXTCOLOR, BGCOLOR);

  // Select a title that identifies which frequency setting is being edited.
  // The shared frequency selector is used by Measure, Sweep Start, and
  // Sweep Stop, so its title should provide the appropriate context.
  const char *title;

  switch (lcrFrequencyTarget) {
    case LCR_FREQ_SWEEP_START:
      title = "Select Sweep Start";
      break;

    case LCR_FREQ_SWEEP_STOP:
      title = "Select Sweep Stop";
      break;

    case LCR_FREQ_MEASURE:
    default:
      title = "Select Frequency";
      break;
  }

  int16_t titleWidth = strlen(title) * 6;

  display.setCursor(
    r.x + (r.w - titleWidth) / 2,
    r.y + r.h * 7 / 100
  );

  display.print(title);

  // Draw frequency presets using the validity rules for the setting currently
  // being edited. Invalid Sweep limits remain visible but are disabled.
  for (uint8_t i = 0; i < FREQUENCY_PRESET_COUNT; i++) {
    bool selected =
      getLCRSelectedFrequency() ==
      frequencyPresets[i].value;

    bool enabled =
      isLCRFrequencyPresetEnabled(
        frequencyPresets[i].value
      );

    drawLCRSelectorButton(
      lcrLayout.frequencyPresets[i],
      frequencyPresets[i].label,
      selected,
      enabled
    );
  }

  drawLCRSelectorButton(
    lcrLayout.selectorCancel,
    "Cancel",
    false
  );
}


// Draw the reference-resistor preset selector over the Measure screen.
// The currently selected reference is highlighted, while the shared
// selector geometry keeps drawing and touch targets synchronized.
void drawLCRReferenceSelector()
{
  const LCRRect &r = lcrLayout.selector;

  // Clear the Measure content and footer occupied by the selector.
  display.fillRect(
    r.x,
    r.y,
    r.w,
    r.h,
    BGCOLOR
  );

  display.setTextSize(1);
  display.setTextColor(TXTCOLOR, BGCOLOR);

  // Draw selector title.
  const char *title = "Select Reference";
  int16_t titleWidth = strlen(title) * 6;

  display.setCursor(
    r.x + (r.w - titleWidth) / 2,
    r.y + r.h * 7 / 100
  );

  display.print(title);

  // Draw each reference preset directly from the shared preset table.
  for (uint8_t i = 0; i < REFERENCE_PRESET_COUNT; i++) {
    bool selected =
      lcrSettings.referenceResistance ==
      referencePresets[i].value;

    drawLCRSelectorButton(
      lcrLayout.referencePresets[i],
      referencePresets[i].label,
      selected
    );
  }

  // Draw the common selector Cancel button.
  drawLCRSelectorButton(
    lcrLayout.selectorCancel,
    "Cancel",
    false
  );
}



// Close the active Measure selector and restore the Measure screen.
// When HOLD is active, the preserved measurement is immediately redrawn;
// LIVE instead marks the display dirty for the next acquisition cycle.
void returnToLCRMeasureScreen()
{
  lcrUIState = LCR_UI_NORMAL;

  drawLCRScreen();

  if (lcrMeasureState == LCR_MEASURE_HOLD) {
    updateLCRDisplay(
      lcrMeasurement,
      lcrSettings
    );

    lcrDisplayDirty = false;
  } else {
    lcrDisplayDirty = true;
  }
}



// Draw the complete static LCR analyzer interface.
// Common navigation is drawn first, followed by content belonging to the
// currently selected top-level analyzer tab.
void drawLCRScreen()
{
  display.fillScreen(BGCOLOR);

  drawLCRHeader();
  drawLCRTabs();

  switch (lcrTab) {
    case LCR_TAB_MEASURE:
      drawMeasureScreen();
      drawMeasureSoftKeys();
      break;

    case LCR_TAB_SWEEP:
      switch (lcrSweepState) {
        case LCR_SWEEP_SETUP:
          drawLCRSweepSetup();
          drawLCRSweepFooter();
          break;

        case LCR_SWEEP_RUNNING:
          drawLCRSweepRunning();
          break;

        case LCR_SWEEP_RESULTS:
          drawLCRSweepResults();
          break;
      }
      break;

    case LCR_TAB_CALIBRATION:
    case LCR_TAB_SETTINGS:
      break;
  }
}



// Update all dynamic fields on the Measure tab.
// Static labels and navigation remain untouched so continuous measurement
// updates do not require redrawing the complete interface.
void updateLCRDisplay(const MeasurementPoint &m,
                      const MeasurementSettings &settings)
{
  drawLCRPrimaryMeasurement(m);
  drawLCRSecondaryMeasurement(m);
  drawLCRMeasurementContext(m, settings);
}



