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
#include <hardware/adc.h>
#include <hardware/clocks.h>
#include <hardware/dma.h>

// Enable verbose LCR ADC/DMA diagnostics.
// Set to 1 while debugging acquisition problems; 0 for normal operation.
#define LCR_CAPTURE_DEBUG 0

// LCR excitation-generator hardware.
// The AD9833 uses SPI0 independently from the TFT display bus.
static const uint8_t AD9833_FSYNC_PIN = 3;  // GP3, physical pin 5
static const uint8_t AD9833_SCK_PIN   = 6;  // GP6, physical pin 9
static const uint8_t AD9833_DATA_PIN  = 7;  // GP7, physical pin 10

// Raw dual-channel ADC capture used while integrating the LCR hardware.
// Samples alternate ADC0/ADC1 in one DMA-filled buffer.
static const uint8_t LCR_ADC_TOTAL_PIN = 26;  // GP26, physical pin 31
static const uint8_t LCR_ADC_DUT_PIN   = 27;  // GP27, physical pin 32
static const uint8_t LCR_ADC_CH_TOTAL  = 0;
static const uint8_t LCR_ADC_CH_DUT    = 1;

int lcrAdcDmaChannel = -1;
static bool lcrCaptureFailureDumped = false;

// flag for capture isolation
volatile bool lcrCaptureWindowActive = false;

// Initial LCR acquisition parameters derived from the POC Normal profile.
static const uint16_t LCR_TARGET_SAMPLES_PER_CYCLE = 50;
static const uint16_t LCR_TARGET_CYCLES = 16;
static const uint16_t LCR_CAPTURE_MIN_SAMPLES = 512;
static const uint16_t LCR_CAPTURE_MAX_SAMPLES = 4000;
static const uint16_t LCR_MAX_POINT_MS = 20;
static const uint16_t LCR_CAPTURE_BUFFER_SAMPLES =
  LCR_CAPTURE_MAX_SAMPLES * 2;

uint16_t lcrCaptureBuffer[LCR_CAPTURE_BUFFER_SAMPLES];

// Goertzel processing configuration carried over from the current POC.
static const float LCR_PHASOR_CONJ = 1.0f;
static const float LCR_HANN_COHERENT_GAIN = 0.5f;
static const bool LCR_USE_HANN_WINDOW = true;

// Maximum aggregate ADC rate validated by the standalone PicoLCR POC.
static const float LCR_ADC_MAX_AGGREGATE_RATE = 250000.0f;

// RP2040 ADC conversion floor used when calculating the hardware divisor.
static const uint16_t LCR_ADC_MIN_PERIOD_CYCLES = 96;

// Avoid sampling the excitation too close to the per-channel Nyquist
// frequency. Near 2 samples/cycle the round-robin Goertzel measurement
// develops a large deterministic artifact.
static const float LCR_NYQUIST_AVOID_SPC_LOW  = 1.90f;
static const float LCR_NYQUIST_AVOID_SPC_HIGH = 2.10f;

// When the normal plan lands in the avoidance region, deliberately move the
// tone farther above Nyquist by targeting 1.60 samples/cycle/channel.
static const float LCR_NYQUIST_SAFE_SPC = 1.60f;

// AD9833 excitation source for the LCR measurement hardware.
AD9833_Driver lcrDDS(AD9833_FSYNC_PIN);

// Runtime ADC clock used to calculate exact conversion timing and channel skew.
uint32_t lcrAdcClockHz = 0;
float lcrAdcMaxAggregateRate = 0.0f;

// Tracks the frequency currently programmed into the physical generator.
// Repeated measurements at the same frequency do not need to reprogram the DDS.
uint32_t lcrGeneratorFrequency = 0;

// Sense resistor currently installed in the LCR measurement network.
static const float LCR_R_SENSE_OHMS = 2160.0f;

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

// Pick whether we are simulating our using real hardware
//LCRBackend lcrBackend = LCR_BACKEND_SIMULATION;
LCRBackend lcrBackend = LCR_BACKEND_HARDWARE;

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
  1000,             // Frequency: 1 kHz
  LCR_R_SENSE_OHMS  // Reference resistor: 1 kOhm
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
  { "|Z|",   LCR_SWEEP_PLOT_IMPEDANCE   },
  { "Phase", LCR_SWEEP_PLOT_PHASE       },
  { "R",     LCR_SWEEP_PLOT_RESISTANCE  },
  { "X",     LCR_SWEEP_PLOT_REACTANCE   },
  { "ESR",   LCR_SWEEP_PLOT_ESR         },
  { "Q",     LCR_SWEEP_PLOT_Q           },
  { "C",     LCR_SWEEP_PLOT_CAPACITANCE },
  { "L",     LCR_SWEEP_PLOT_INDUCTANCE  }
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
  point.valid = true;
  point.impedance = measurement.impedance;
  point.phaseDeg = measurement.phaseDeg;
  point.resistance = measurement.resistance;
  point.reactance = measurement.reactance;
  point.capacitance = measurement.capacitance;
  point.inductance = measurement.inductance;
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

    case LCR_SWEEP_PLOT_CAPACITANCE:
    return point.capacitance;

    case LCR_SWEEP_PLOT_INDUCTANCE:
      return point.inductance;

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

  bool haveValidPoint = false;

  for (uint16_t i = 0; i < lcrSweepPointCount; i++) {
    if (!lcrSweepPoints[i].valid)
      continue;

    float value = getLCRSweepPlotValue(lcrSweepPoints[i]);

    if (!haveValidPoint) {
      range.minimum = value;
      range.maximum = value;
      haveValidPoint = true;
      continue;
    }

    if (value < range.minimum)
      range.minimum = value;

    if (value > range.maximum)
      range.maximum = value;
  }

  // If the entire sweep failed, leave the safe default range of 0..1.
  if (!haveValidPoint)
    return range;

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
     lcrSweepPlotType == LCR_SWEEP_PLOT_Q ||
     lcrSweepPlotType == LCR_SWEEP_PLOT_CAPACITANCE ||
     lcrSweepPlotType == LCR_SWEEP_PLOT_INDUCTANCE) &&
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
    case LCR_SWEEP_PLOT_CAPACITANCE: {
      float largest =
        max(fabsf(range.minimum), fabsf(range.maximum));

      if (largest < 1.0e-9f)
        return { 1.0e-12f, "pF" };

      if (largest < 1.0e-6f)
        return { 1.0e-9f, "nF" };

      if (largest < 1.0e-3f)
        return { 1.0e-6f, "uF" };

      return { 1.0f, "F" };
    }

    case LCR_SWEEP_PLOT_INDUCTANCE: {
      float largest =
        max(fabsf(range.minimum), fabsf(range.maximum));

      if (largest < 1.0e-6f)
        return { 1.0e-9f, "nH" };

      if (largest < 1.0e-3f)
        return { 1.0e-6f, "uH" };

      if (largest < 1.0f)
        return { 1.0e-3f, "mH" };

      return { 1.0f, "H" };
    }

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

    if (point.valid) {
      formatLCRSweepCursorValue(
        value,
        sizeof(value),
        getLCRSweepPlotValue(point),
        scale
      );
    }
    else {
      snprintf(
        value,
        sizeof(value),
        "NO DATA"
      );
    }

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
  // X-axis frequency labels.
  //
  // Logarithmic sweeps use conventional major-decade labels.
  // Linear sweeps retain the existing start / midpoint / stop labels.
  //
  if (lcrSweepSettings.mode == LCR_SWEEP_LOG) {

    uint32_t start = lcrSweepSettings.startFrequency;
    uint32_t stop  = lcrSweepSettings.stopFrequency;

    if (start == 0 || stop <= start)
      return;

    //
    // Find the first power of ten at or above the Sweep start.
    //
    uint32_t decade = 1;

    while (decade < start &&
           decade <= UINT32_MAX / 10) {
      decade *= 10;
    }

    //
    // Label every major decade contained within the Sweep.
    //
    while (decade <= stop) {

      char frequencyLabel[12];

      if (decade >= 1000000UL) {
        snprintf(
          frequencyLabel,
          sizeof(frequencyLabel),
          "%luM",
          decade / 1000000UL
        );
      }
      else if (decade >= 1000UL) {
        snprintf(
          frequencyLabel,
          sizeof(frequencyLabel),
          "%luk",
          decade / 1000UL
        );
      }
      else {
        snprintf(
          frequencyLabel,
          sizeof(frequencyLabel),
          "%lu",
          decade
        );
      }

      int16_t x =
        calculateLCRSweepPlotX(decade);

      int16_t width =
        strlen(frequencyLabel) * 6;

      //
      // Center each label under its decade grid line, but keep the text
      // inside the available screen area.
      //
      int16_t labelX = x - width / 2;

      labelX = constrain(
        labelX,
        plot.x,
        plot.x + plot.w - width
      );

      display.setCursor(
        labelX,
        plot.y + plot.h + 4
      );

      display.print(frequencyLabel);

      if (decade > UINT32_MAX / 10)
        break;

      decade *= 10;
    }

    return;
  }

  //
  // Linear Sweep labels.
  //

  // Start frequency.
  LCRFormattedValue startFrequency =
    formatFrequency(lcrSweepSettings.startFrequency);

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


  // Midpoint frequency.
  uint32_t midpointFrequency =
    lcrSweepSettings.startFrequency +
    (lcrSweepSettings.stopFrequency -
     lcrSweepSettings.startFrequency) / 2;

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
    plot.x + (plot.w - labelWidth) / 2,
    plot.y + plot.h + 4
  );

  display.print(label);


  // Stop frequency.
  LCRFormattedValue stopFrequency =
    formatFrequency(lcrSweepSettings.stopFrequency);

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


// Draw conventional logarithmic frequency grid lines.
//
// Major decade divisions use the normal grid color. Minor divisions at
// 2..9 times each decade are drawn with a dimmer color so the plot reads
// naturally as a semilog-X graph without overwhelming the measurement trace.
void drawLCRSweepLogGrid()
{
  if (lcrSweepSettings.mode != LCR_SWEEP_LOG)
    return;

  const LCRRect &plot = lcrLayout.sweepPlot;

  uint32_t start = lcrSweepSettings.startFrequency;
  uint32_t stop  = lcrSweepSettings.stopFrequency;

  if (start == 0 || stop <= start)
    return;

  //
  // Dimmer version of GRIDCOLOR for minor logarithmic divisions.
  //
  uint16_t minorGridColor =
    display.color565(40, 40, 40);

  //
  // Find the power of ten at or below the Sweep start.
  //
  uint32_t decade = 1;

  while (decade <= start / 10)
    decade *= 10;

  //
  // Process each decade intersecting the Sweep range.
  //
  while (decade <= stop) {

    //
    // Minor divisions: 2, 3, ... 9 times the current decade.
    //
    for (uint8_t multiplier = 2;
         multiplier <= 9;
         multiplier++) {

      uint32_t frequency =
        decade * multiplier;

      if (frequency <= start ||
          frequency >= stop) {
        continue;
      }

      int16_t x =
        calculateLCRSweepPlotX(frequency);

      display.drawFastVLine(
        x,
        plot.y + 1,
        plot.h - 2,
        minorGridColor
      );
    }

    //
    // Major decade division.
    //
    if (decade > start &&
        decade < stop) {

      int16_t x =
        calculateLCRSweepPlotX(decade);

      display.drawFastVLine(
        x,
        plot.y + 1,
        plot.h - 2,
        GRIDCOLOR
      );
    }

    if (decade > UINT32_MAX / 10)
      break;

    decade *= 10;
  }
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

  //
  // X-axis grid.
  //
  // Linear plots retain the single midpoint reference.
  // Log plots instead show major frequency decades.
  //
  if (lcrSweepSettings.mode == LCR_SWEEP_LOG) {

    drawLCRSweepLogGrid();

  } else {

    const int16_t midpointX =
      plot.x + plot.w / 2;

    display.drawFastVLine(
      midpointX,
      plot.y + 1,
      plot.h - 2,
      GRIDCOLOR
    );
  }

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
  // Invalid measurements are never connected into the normal trace. A red
  // marker identifies the requested frequency where acquisition failed.
  //

  bool havePreviousValidPoint = false;
  int16_t previousX = 0;
  int16_t previousY = 0;

  for (uint16_t i = 0;
       i < lcrSweepPointCount;
       i++) {

    const SweepPoint &point = lcrSweepPoints[i];

    int16_t x = calculateLCRSweepPlotX(point.frequency);

    //
    // Failed acquisition: mark the frequency in red and break the trace.
    //
    if (!point.valid) {
      display.drawFastVLine(
        x,
        plot.y + plot.h - 5,
        4,
        TFT_RED
      );

      havePreviousValidPoint = false;
      continue;
    }

    //
    // Valid measurement.
    //
    int16_t y =
      calculateLCRSweepPlotY(
        getLCRSweepPlotValue(point),
        range
      );

    // Always make an individual valid measurement visible.
    display.drawPixel(
      x,
      y,
      HIGHCOLOR
    );

    // Connect only consecutive valid measurements.
    if (havePreviousValidPoint) {
      display.drawLine(
        previousX,
        previousY,
        x,
        y,
        HIGHCOLOR
      );
    }

    previousX = x;
    previousY = y;
    havePreviousValidPoint = true;
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

  if (measurement.frequency != frequency)
    return false;

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


// Select the most useful initial Results plot from the completed Sweep.
//
// Classification uses the average phase of the retained Sweep points so a
// noisy individual frequency does not determine the DUT type. The same
// +/-10 degree threshold used by the Measure display separates predominantly
// resistive, capacitive, and inductive behavior.
LCRSweepPlotType getLCRPrimarySweepPlotType()
{
  static const float COMPONENT_PHASE_THRESHOLD_DEG = 10.0f;

  if (lcrSweepPointCount == 0)
    return LCR_SWEEP_PLOT_IMPEDANCE;

  float phaseSum = 0.0f;
  uint16_t validCount = 0;

  for (uint16_t i = 0; i < lcrSweepPointCount; i++) {

    //
    // If your current SweepPoint has a validity flag, keep this check.
    //
    if (!lcrSweepPoints[i].valid)
      continue;

    phaseSum += lcrSweepPoints[i].phaseDeg;
    validCount++;
  }

  if (validCount == 0)
    return LCR_SWEEP_PLOT_IMPEDANCE;

  float averagePhase =
    phaseSum / static_cast<float>(validCount);

  if (averagePhase < -COMPONENT_PHASE_THRESHOLD_DEG)
    return LCR_SWEEP_PLOT_CAPACITANCE;

  if (averagePhase > COMPONENT_PHASE_THRESHOLD_DEG)
    return LCR_SWEEP_PLOT_INDUCTANCE;

  return LCR_SWEEP_PLOT_IMPEDANCE;
}


// Complete the active frequency sweep and transition to the Results state.
// All successfully acquired SweepPoint entries remain available for
// plotting and cursor inspection.
void finishLCRSweep()
{
  lcrSweepExecution.active = false;

  //
  // Choose the most useful initial Results quantity for the measured DUT.
  //
  lcrSweepPlotType = getLCRPrimarySweepPlotType();
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

  // Acquire and store one Sweep point. Retry transient capture failures before
  // skipping the frequency and continuing with the remainder of the sweep.
  bool pointAcquired = false;

  for (uint8_t attempt = 1; attempt <= 3; attempt++) {
    if (acquireLCRSweepPoint(frequency)) {
      pointAcquired = true;
      break;
    }

    Serial.print("LCR sweep retry ");
    Serial.print(attempt);
    Serial.print("/3 at ");
    Serial.print(frequency);
    Serial.println(" Hz");

    delay(5);
  }

  if (!pointAcquired) {
    Serial.print("LCR sweep invalid ");
    Serial.print(frequency);
    Serial.println(" Hz after 3 failed attempts");

    if (lcrSweepPointCount < LCR_MAX_SWEEP_POINTS) {
      SweepPoint &point = lcrSweepPoints[lcrSweepPointCount];

      point = {};
      point.frequency = frequency;
      point.valid = false;

      lcrSweepPointCount++;
    }
  }

  // Advance to the next requested frequency whether this point succeeded or not.
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


// Perform one physical LCR measurement using the AD9833 excitation source
// and the dual-channel ADC/DMA/Goertzel impedance backend.
MeasurementPoint hardwareMeasurement(const MeasurementSettings &settings)
{
  MeasurementPoint measurement = {};

  setLCRGeneratorFrequency(settings.frequency);

  // Allow the analog network to settle after an excitation-frequency change.
  delay(5);

  LCRPhasor impedance;

  if (!measureLCRHardwareImpedance(
        settings.frequency,
        settings.referenceResistance,
        impedance)) {
    return measurement;
  }
  measurement.frequency = settings.frequency;
  measurement.resistance = impedance.re;
  measurement.reactance = impedance.im;
  measurement.impedance = magnitudeLCRPhasor(impedance);
  measurement.phaseDeg = phaseLCRPhasor(impedance);

  // Series-equivalent quantities derived from complex impedance.
  measurement.esr = measurement.resistance;

  float omega = 2.0f * PI * settings.frequency;

  if (measurement.reactance < 0.0f) {
    measurement.capacitance = -1.0f / (omega * measurement.reactance);
    measurement.inductance = 0.0f;
  } else if (measurement.reactance > 0.0f) {
    measurement.capacitance = 0.0f;
    measurement.inductance = measurement.reactance / omega;
  } else {
    measurement.capacitance = 0.0f;
    measurement.inductance = 0.0f;
  }

  if (fabsf(measurement.resistance) > 0.000001f)
    measurement.q = fabsf(measurement.reactance / measurement.resistance);
  else
    measurement.q = 0.0f;

  if (fabsf(measurement.reactance) > 0.000001f)
    measurement.dissipation =
      fabsf(measurement.resistance / measurement.reactance);
  else
    measurement.dissipation = 0.0f;

  return measurement;
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



// Enter the LCR analyzer and transfer ADC ownership from Scope to LCR.
void enterLCRMode()
{
  // Stop Scope acquisition before LCR changes shared ADC state.
  stopScopeAdc();
  lcrCaptureFailureDumped = false;

  instrumentMode = MODE_LCR;
  lcrTab = LCR_TAB_MEASURE;

#if LCR_CAPTURE_DEBUG

  // Start in HOLD during ADC/DMA debugging so simply entering LCR mode does
  // not continuously exercise the acquisition path.
  lcrMeasureState = LCR_MEASURE_HOLD;

#else

  // Normal operation begins with continuous measurement enabled.
  lcrMeasureState = LCR_MEASURE_LIVE;

#endif

  lcrSweepState = LCR_SWEEP_SETUP;
  lcrFrequencyTarget = LCR_FREQ_MEASURE;
  lcrUIState = LCR_UI_NORMAL;
  lcrDisplayDirty = true;

  initializeLCR();
  initializeLCRCapture();

  drawLCRScreen();
}


// Exit the LCR analyzer, release the shared ADC, and restore Scope ownership.
void exitLCRMode()
{
  // Stop any LCR conversion that may still be active.
  adc_run(false);
  adc_set_round_robin(0);
  adc_fifo_drain();

  // Restore the ADC configuration expected by the oscilloscope.
  restoreScopeAdc();

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

  // Defaulting to 1kHz out of the generator, and setting the semaphore
  // lcrGeneratorFrequency to match so we aren't constantly resetting ourselves
  // on every pass.
  lcrDDS.begin(1000);
  lcrGeneratorFrequency = 1000;

  Serial.println("LCR AD9833 initialized at 1 kHz");
}


// Configure clk_adc for the higher-rate LCR acquisition path.
// With the current 192 MHz system clock, dividing PLL_SYS by two produces
// the 96 MHz ADC clock validated by the standalone PicoLCR POC.
void configureLCRAdcClock()
{
  uint32_t systemClock = clock_get_hz(clk_sys);

  clock_configure(
    clk_adc,
    0,
    CLOCKS_CLK_ADC_CTRL_AUXSRC_VALUE_CLKSRC_PLL_SYS,
    systemClock,
    systemClock / 2
  );

  lcrAdcClockHz = clock_get_hz(clk_adc);
  lcrAdcMaxAggregateRate = LCR_ADC_MAX_AGGREGATE_RATE;

  Serial.print("LCR ADC clock: ");
  Serial.print(lcrAdcClockHz / 1000000.0f, 2);
  Serial.println(" MHz");

  Serial.print("LCR ADC aggregate ceiling: ");
  Serial.print(lcrAdcMaxAggregateRate / 1000.0f, 1);
  Serial.println(" kS/s");
}


// Initialize the RP2040 ADC and DMA resources used by the LCR capture path.
// ADC0 and ADC1 are sampled round-robin and transferred directly from the
// ADC FIFO into an interleaved RAM buffer.
void initializeLCRCapture()
{
  configureLCRAdcClock(); // Adjust for higher freq measurements

  adc_init();

  // Clock diag output
  lcrAdcClockHz = clock_get_hz(clk_adc);
  lcrAdcMaxAggregateRate = LCR_ADC_MAX_AGGREGATE_RATE;

  adc_gpio_init(LCR_ADC_TOTAL_PIN);
  adc_gpio_init(LCR_ADC_DUT_PIN);

  adc_set_round_robin(0);
  adc_select_input(LCR_ADC_CH_TOTAL);

  adc_fifo_setup(
    true,   // Enable FIFO
    true,   // Enable DMA request
    1,      // DMA request when at least one sample is present
    false,  // Do not set ERR bit in sample data
    false   // Keep full 12-bit ADC samples
  );

  adc_fifo_drain();

  if (lcrAdcDmaChannel < 0)
    lcrAdcDmaChannel = dma_claim_unused_channel(true);

#if LCR_CAPTURE_DEBUG
  Serial.print("LCR ADC DMA channel: ");
  Serial.println(lcrAdcDmaChannel);
#endif
}


// Calculate the actual ADC timing and record length for one measurement point.
// An integer ADC divisor avoids fractional-period dither, and all downstream
// processing uses the achieved sample rate rather than the requested rate.
LCRCapturePlan calculateLCRCapturePlan(uint32_t frequency)
{
  LCRCapturePlan plan = {};
  plan.frequency = frequency;

  if (frequency == 0 || lcrAdcClockHz == 0)
    return plan;

  float targetAggregateRate =
    2.0f * LCR_TARGET_SAMPLES_PER_CYCLE * frequency;

  if (targetAggregateRate > lcrAdcMaxAggregateRate)
    targetAggregateRate = lcrAdcMaxAggregateRate;

  if (targetAggregateRate < 1.0f)
    targetAggregateRate = 1.0f;

  //
  // Nyquist avoidance.
  //
  // targetAggregateRate contains samples from both round-robin channels, so
  // the per-channel rate is half of it.
  //
  float targetChannelRate = targetAggregateRate / 2.0f;
  float targetSamplesPerCycle =
    targetChannelRate / static_cast<float>(frequency);

  if (targetSamplesPerCycle > LCR_NYQUIST_AVOID_SPC_LOW &&
      targetSamplesPerCycle < LCR_NYQUIST_AVOID_SPC_HIGH) {

    // Nyquist spike troubleshooting
#if LCR_CAPTURE_DEBUG
    Serial.print("LCR Nyquist avoid: ");
    Serial.print(frequency);
    Serial.print(" Hz  normal_spc=");
    Serial.print(targetSamplesPerCycle, 3);
    Serial.print(" -> target_spc=");
    Serial.println(LCR_NYQUIST_SAFE_SPC, 3);
#endif

    // Move the excitation away from exactly two samples/cycle. We deliberately
    // move to the aliased side because measurements above Nyquist have
    // otherwise remained stable in the validated POC and integrated resistor
    // tests.
    targetAggregateRate =
      2.0f * LCR_NYQUIST_SAFE_SPC * static_cast<float>(frequency);

    // Never exceed the normal ADC acquisition ceiling.
    if (targetAggregateRate > lcrAdcMaxAggregateRate)
      targetAggregateRate = lcrAdcMaxAggregateRate;
  }

  // One ADC conversion occupies (div + 1) clk_adc cycles. The hardware
  // conversion floor is 96 cycles.
  float divisor = static_cast<float>(lcrAdcClockHz) /
                  targetAggregateRate - 1.0f;

  if (divisor < LCR_ADC_MIN_PERIOD_CYCLES - 1)
    divisor = LCR_ADC_MIN_PERIOD_CYCLES - 1;

  if (divisor > 65535.0f)
    divisor = 65535.0f;

  uint32_t integerDivisor = static_cast<uint32_t>(divisor + 0.5f);
  uint32_t periodCycles = integerDivisor + 1;

  plan.adcClockDiv = static_cast<float>(integerDivisor);
  plan.aggregateSampleRate =
    static_cast<float>(lcrAdcClockHz) / periodCycles;

  plan.channelSampleRate = plan.aggregateSampleRate / 2.0f;
  plan.samplesPerCycle = plan.channelSampleRate / frequency;

  // Choose the record length using the same cycle-based algorithm as the
  // validated standalone POC.
  uint32_t ceiling = LCR_CAPTURE_MAX_SAMPLES;

  if (LCR_MAX_POINT_MS > 0) {
    uint32_t budgetSamples = static_cast<uint32_t>(
      plan.channelSampleRate * static_cast<float>(LCR_MAX_POINT_MS) * 1.0e-3f
    );

    if (budgetSamples < ceiling)
      ceiling = budgetSamples;
  }

  if (ceiling > LCR_CAPTURE_MAX_SAMPLES)
    ceiling = LCR_CAPTURE_MAX_SAMPLES;

  if (ceiling < LCR_CAPTURE_MIN_SAMPLES)
    ceiling = LCR_CAPTURE_MIN_SAMPLES;

  uint32_t cycles = LCR_TARGET_CYCLES;

  uint32_t minimumCycles = static_cast<uint32_t>(
    ceilf(static_cast<float>(LCR_CAPTURE_MIN_SAMPLES) /
          plan.samplesPerCycle)
  );

  if (minimumCycles > cycles)
    cycles = minimumCycles;

  uint32_t maximumCycles = static_cast<uint32_t>(
    floorf(static_cast<float>(ceiling) / plan.samplesPerCycle)
  );

  if (maximumCycles < 1)
    maximumCycles = 1;

  if (cycles > maximumCycles)
    cycles = maximumCycles;

  long samples = lroundf(
    static_cast<float>(cycles) * plan.samplesPerCycle
  );

  if (samples < 3)
    samples = 3;

  if (samples > LCR_CAPTURE_MAX_SAMPLES)
    samples = LCR_CAPTURE_MAX_SAMPLES;

  plan.samplesPerChannel = static_cast<uint16_t>(samples);

  return plan;
}

#if LCR_CAPTURE_DEBUG
// Dump ADC/DMA state once when the capture path first develops a real FIFO
// error. The state is captured before another measurement can disturb it.
void printLCRCaptureFailureState(uint32_t frequency,
                                 const LCRCapturePlan &plan,
                                 uint32_t fifoStatus)
{
  Serial.println();
  Serial.println("=== LCR CAPTURE FAILURE STATE ===");

  Serial.print("frequency: ");
  Serial.print(frequency);
  Serial.println(" Hz");

  Serial.print("DMA channel: ");
  Serial.println(lcrAdcDmaChannel);

  Serial.print("DMA busy: ");
  Serial.println(dma_channel_is_busy(lcrAdcDmaChannel) ? "yes" : "no");

  Serial.print("DMA transfer_count: ");
  Serial.println(dma_hw->ch[lcrAdcDmaChannel].transfer_count);

  Serial.print("DMA CTRL_TRIG: 0x");
  Serial.println(dma_hw->ch[lcrAdcDmaChannel].ctrl_trig, HEX);

  Serial.print("ADC CS: 0x");
  Serial.println(adc_hw->cs, HEX);

  Serial.print("ADC FCS: 0x");
  Serial.println(fifoStatus, HEX);

  Serial.print("ADC FIFO level: ");
  Serial.println(
    (fifoStatus & ADC_FCS_LEVEL_BITS) >> ADC_FCS_LEVEL_LSB
  );

  Serial.print("ADC OVER: ");
  Serial.println(
    (fifoStatus & ADC_FCS_OVER_BITS) ? "yes" : "no"
  );

  Serial.print("ADC UNDER: ");
  Serial.println(
    (fifoStatus & ADC_FCS_UNDER_BITS) ? "yes" : "no"
  );

  Serial.print("samples/channel: ");
  Serial.println(plan.samplesPerChannel);

  Serial.print("capture time: ");
  Serial.print(plan.captureTimeUs);
  Serial.println(" us");

  Serial.print("programmed aggregate rate: ");
  Serial.print(plan.aggregateSampleRate / 1000.0f, 1);
  Serial.println(" kS/s");

  Serial.print("measured aggregate rate: ");
  Serial.print(plan.measuredAggregateRate / 1000.0f, 1);
  Serial.println(" kS/s");

  Serial.print("clk_adc: ");
  Serial.print(clock_get_hz(clk_adc));
  Serial.println(" Hz");

  Serial.print("plan adcClockDiv: ");
  Serial.println(plan.adcClockDiv, 6);

  Serial.print("ADC DIV register: 0x");
  Serial.println(adc_hw->div, HEX);

  Serial.print("ADC DIV integer: ");
  Serial.println(
    (adc_hw->div & ADC_DIV_INT_BITS) >> ADC_DIV_INT_LSB
  );

  Serial.print("ADC DIV frac: ");
  Serial.println(
    (adc_hw->div & ADC_DIV_FRAC_BITS) >> ADC_DIV_FRAC_LSB
  );

  Serial.print("ADC FCS THRESH: ");
  Serial.println(
    (fifoStatus & ADC_FCS_THRESH_BITS) >> ADC_FCS_THRESH_LSB
  );

  Serial.print("ADC FCS DREQ_EN: ");
  Serial.println(
    (fifoStatus & ADC_FCS_DREQ_EN_BITS) ? "yes" : "no"
  );

  Serial.println("DMA channel states:");

  for (uint8_t ch = 0; ch < NUM_DMA_CHANNELS; ch++) {
    uint32_t ctrl = dma_hw->ch[ch].ctrl_trig;

    Serial.print("  ch");
    Serial.print(ch);

    Serial.print(" busy=");
    Serial.print(
      (ctrl & DMA_CH0_CTRL_TRIG_BUSY_BITS) ? 1 : 0
    );

    Serial.print(" count=");
    Serial.print(dma_hw->ch[ch].transfer_count);

    Serial.print(" ctrl=0x");
    Serial.println(ctrl, HEX);
  }

  Serial.print("DMA INTR: 0x");
  Serial.println(dma_hw->intr, HEX);

  Serial.print("DMA INTE0: 0x");
  Serial.println(dma_hw->inte0, HEX);

  Serial.print("DMA INTS0: 0x");
  Serial.println(dma_hw->ints0, HEX);

  Serial.print("DMA INTE1: 0x");
  Serial.println(dma_hw->inte1, HEX);

  Serial.print("DMA INTS1: 0x");
  Serial.println(dma_hw->ints1, HEX);

  Serial.println("=== END CAPTURE FAILURE STATE ===");
  Serial.println();
}
#endif


// Mark the short ADC/DMA acquisition interval as timing-critical.
//
// Do not perform display updates, network transfers, Scope acquisition,
// or other avoidable high-bandwidth work while this flag is set.
//
// We deliberately do not disable interrupts or lock out the other RP2040
// core here. Those are stronger measures that can be added later if the
// lightweight capture window does not eliminate rare FIFO starvation.
void beginLCRCaptureWindow()
{
  lcrCaptureWindowActive = true;
}


// End the timing-critical ADC/DMA acquisition interval.
void endLCRCaptureWindow()
{
  lcrCaptureWindowActive = false;
}


// Capture one interleaved ADC0/ADC1 record using deterministic round-robin
// startup, DMA transfer, measured-rate verification, and integrity checks.
bool captureLCRRawTest(uint32_t frequency, LCRCapturePlan &plan)
{
  if (lcrAdcDmaChannel < 0)
    return false;

  plan = calculateLCRCapturePlan(frequency);

  if (plan.samplesPerChannel == 0 || plan.adcClockDiv <= 0.0f)
    return false;

  uint32_t totalSamples =
    static_cast<uint32_t>(plan.samplesPerChannel) * 2u;

  //
  // Establish deterministic ADC0, ADC1, ADC0, ADC1... ordering.
  //

  adc_run(false);
  adc_set_round_robin(0);
  adc_fifo_drain();

  adc_select_input(LCR_ADC_CH_TOTAL);

  adc_set_round_robin(
    (1u << LCR_ADC_CH_TOTAL) |
    (1u << LCR_ADC_CH_DUT)
  );

  adc_set_clkdiv(plan.adcClockDiv);

  adc_fifo_setup(
    true,   // Enable FIFO
    true,   // Enable DMA request
    1,      // DREQ on every sample
    false,  // Do not include ERR bit in FIFO data
    false   // Keep full 12-bit samples
  );

  uint32_t fcs = adc_hw->fcs;

  adc_hw->fcs =
    (fcs &
     (ADC_FCS_THRESH_BITS |
      ADC_FCS_DREQ_EN_BITS |
      ADC_FCS_ERR_BITS |
      ADC_FCS_SHIFT_BITS |
      ADC_FCS_EN_BITS)) |
    ADC_FCS_OVER_BITS |
    ADC_FCS_UNDER_BITS;

  // Verify that no stale FIFO error state remains before starting DMA.
  if (adc_hw->fcs &
      (ADC_FCS_OVER_BITS | ADC_FCS_UNDER_BITS)) {

    Serial.println(
      "LCR ADC error: FIFO status would not clear"
    );

    return false;
  }

  //
  // Configure and arm DMA before starting ADC conversions.
  //

  dma_channel_config cfg =
    dma_channel_get_default_config(lcrAdcDmaChannel);

  channel_config_set_transfer_data_size(&cfg, DMA_SIZE_16);
  channel_config_set_read_increment(&cfg, false);
  channel_config_set_write_increment(&cfg, true);
  channel_config_set_dreq(&cfg, DREQ_ADC);
  channel_config_set_high_priority(&cfg, true);

  dma_channel_configure(
    lcrAdcDmaChannel,
    &cfg,
    lcrCaptureBuffer,
    &adc_hw->fifo,
    totalSamples,
    true
  );

  //
  // Time only the actual conversion run.
  //

  beginLCRCaptureWindow();

  uint32_t startUs = time_us_32();

  adc_run(true);

#if LCR_CAPTURE_DEBUG

  //
  // Diagnostic FIFO-overflow mitigation:
  //
  // Protect the final few ADC samples and DMA -> ADC shutdown transition from
  // interrupt latency. This did not eliminate the rare FIFO_OVER condition,
  // but is retained for future investigation.
  //
  while (dma_hw->ch[lcrAdcDmaChannel].transfer_count > 4) {
    tight_loop_contents();
  }

  noInterrupts();

  while (dma_channel_is_busy(lcrAdcDmaChannel)) {
    tight_loop_contents();
  }

  adc_run(false);

  interrupts();

#else

  //
  // Normal capture path.
  //
  dma_channel_wait_for_finish_blocking(lcrAdcDmaChannel);
  adc_run(false);

#endif

  uint32_t stopUs = time_us_32();

  endLCRCaptureWindow();

  //
  // Calculate the actual achieved sample rate.
  //

  plan.captureTimeUs = stopUs - startUs;

  if (plan.captureTimeUs > 0) {
    plan.measuredAggregateRate =
      static_cast<float>(totalSamples) /
      (static_cast<float>(plan.captureTimeUs) * 1.0e-6f);

    plan.measuredChannelRate = plan.measuredAggregateRate / 2.0f;

    plan.measuredSamplesPerCycle =
      plan.measuredChannelRate / static_cast<float>(frequency);
  }

  //
  // Save capture integrity state before disturbing the ADC.
  //
  uint32_t fifoStatus = adc_hw->fcs;

  bool fifoError =
    (fifoStatus &
     (ADC_FCS_OVER_BITS | ADC_FCS_UNDER_BITS)) != 0;

  uint32_t finalChannel =
    (adc_hw->cs & ADC_CS_AINSEL_BITS) >> ADC_CS_AINSEL_LSB;

  // Now that status has been preserved, discard conversions that occurred
  // after DMA completed.
  adc_fifo_drain();
  adc_set_round_robin(0);

  bool channelParityValid = (finalChannel == LCR_ADC_CH_TOTAL);

  // AINSEL may advance once more after the final DMA transfer and before
  // adc_run(false) takes effect. Report this for diagnostics, but do not reject
  // an otherwise clean DMA record solely because of final AINSEL state.
  if (!channelParityValid && !fifoError) {
    Serial.print("LCR ADC note: final AINSEL=");
    Serial.print(finalChannel);
    Serial.println(" after clean DMA capture");
  }

  // FIFO overflow/underflow means samples were actually lost, so that record
  // cannot be trusted.
  if (fifoError) {
    Serial.print("LCR ADC capture error:");

    if (fifoStatus & ADC_FCS_OVER_BITS)
      Serial.print(" FIFO_OVER");

    if (fifoStatus & ADC_FCS_UNDER_BITS)
      Serial.print(" FIFO_UNDER");

    if (!channelParityValid) {
      Serial.print(" final=");
      Serial.print(finalChannel);
    }

    Serial.println();

    // Preserve the first real FIFO failure for diagnosis instead of allowing
    // LIVE Measure to immediately hammer the broken capture path again.
    if (!lcrCaptureFailureDumped) {
      lcrCaptureFailureDumped = true;

#if LCR_CAPTURE_DEBUG
      printLCRCaptureFailureState(
        frequency,
        plan,
        fifoStatus
      );
      lcrMeasureState = LCR_MEASURE_HOLD;

      Serial.println(
        "LCR Measure forced to HOLD after ADC capture failure."
      );
#endif

    }

    return false;
  }

  return true;
}


// Report programmed versus measured ADC rate for selected diagnostic points.
void printLCRCaptureRate(const LCRCapturePlan &plan)
{
  float ratio = 0.0f;

  if (plan.aggregateSampleRate > 0.0f)
    ratio = plan.measuredAggregateRate / plan.aggregateSampleRate;

  Serial.print("LCR rate: ");
  Serial.print(plan.frequency);
  Serial.print(" Hz  programmed=");
  Serial.print(plan.aggregateSampleRate / 1000.0f, 1);
  Serial.print(" kS/s  measured=");
  Serial.print(plan.measuredAggregateRate / 1000.0f, 1);
  Serial.print(" kS/s  ratio=");
  Serial.print(ratio, 4);
  Serial.print("  spc=");
  Serial.println(plan.measuredSamplesPerCycle, 2);
}


// Return the magnitude of a complex Goertzel phasor.
float magnitudeLCRPhasor(const LCRPhasor &p)
{
  return sqrtf(p.re * p.re + p.im * p.im);
}


// Rotate a complex phasor by theta radians.
LCRPhasor rotateLCRPhasor(const LCRPhasor &p, float theta)
{
  float c = cosf(theta);
  float s = sinf(theta);

  return {
    p.re * c - p.im * s,
    p.re * s + p.im * c
  };
}


// Subtract one complex phasor from another.
LCRPhasor subtractLCRPhasor(const LCRPhasor &a, const LCRPhasor &b)
{
  return { a.re - b.re, a.im - b.im };
}


// Divide one complex phasor by another.
LCRPhasor divideLCRPhasor(const LCRPhasor &a, const LCRPhasor &b)
{
  float denominator = b.re * b.re + b.im * b.im;

  if (denominator <= 0.0f)
    return { 0.0f, 0.0f };

  return {
    (a.re * b.re + a.im * b.im) / denominator,
    (a.im * b.re - a.re * b.im) / denominator
  };
}


// Scale a complex phasor by a real value.
LCRPhasor scaleLCRPhasor(const LCRPhasor &p, float scale)
{
  return { p.re * scale, p.im * scale };
}


// Return the phase of a complex phasor in degrees.
float phaseLCRPhasor(const LCRPhasor &p)
{
  return atan2f(p.im, p.re) * 180.0f / PI;
}


// Normalize a phase difference into the range -180 to +180 degrees.
float normalizeLCRPhase(float phase)
{
  while (phase > 180.0f)
    phase -= 360.0f;

  while (phase < -180.0f)
    phase += 360.0f;

  return phase;
}


// Calculate the DC mean of one channel in the interleaved ADC capture buffer.
float meanLCRCaptureChannel(uint16_t sampleCount, uint8_t offset)
{
  uint32_t sum = 0;

  for (uint16_t i = 0; i < sampleCount; i++)
    sum += lcrCaptureBuffer[i * 2 + offset] & 0x0FFF;

  return static_cast<float>(sum) / sampleCount;
}


// Extract the complex amplitude at one frequency from a single channel of the
// interleaved ADC buffer using the POC's single-bin Goertzel implementation.
LCRPhasor calculateLCRGoertzel(uint16_t sampleCount, uint8_t offset,
                               float frequency, float sampleRate, float dc)
{
  float w = 2.0f * PI * frequency / sampleRate;
  float cw = cosf(w);
  float sw = sinf(w);
  float coeff = 2.0f * cw;

  float s1 = 0.0f;
  float s2 = 0.0f;

  float d = (sampleCount > 1) ?
            (2.0f * PI / static_cast<float>(sampleCount - 1)) : 0.0f;

  float windowCoeff = 2.0f * cosf(d);
  float windowPrevious = cosf(d);
  float windowCurrent = 1.0f;

  for (uint16_t i = 0; i < sampleCount; i++) {
    float x = static_cast<float>(lcrCaptureBuffer[i * 2 + offset] & 0x0FFF) - dc;

    if (LCR_USE_HANN_WINDOW) {
      x *= 0.5f * (1.0f - windowCurrent);

      float windowNext =
        windowCoeff * windowCurrent - windowPrevious;

      windowPrevious = windowCurrent;
      windowCurrent = windowNext;
    }

    float s0 = x + coeff * s1 - s2;
    s2 = s1;
    s1 = s0;
  }

  LCRPhasor result = {
    cw * s1 - s2,
    sw * s1
  };

  result.im *= LCR_PHASOR_CONJ;

  return result;
}


// Convert a Goertzel phasor into sinusoidal peak amplitude in ADC counts.
float lcrPhasorAmplitude(const LCRPhasor &p, uint16_t sampleCount)
{
  float coherentGain = LCR_USE_HANN_WINDOW ? LCR_HANN_COHERENT_GAIN : 1.0f;

  return 2.0f * magnitudeLCRPhasor(p) /
         (static_cast<float>(sampleCount) * coherentGain);
}


// Calculate statistics for one channel in the interleaved ADC test buffer.
// Offset 0 selects ADC0 and offset 1 selects ADC1.
LCRCaptureStats calculateLCRCaptureStats(uint8_t offset,
                                         uint16_t sampleCount)
{
  LCRCaptureStats stats;
  stats.minimum = 4095;
  stats.maximum = 0;

  uint32_t sum = 0;

  for (uint16_t i = 0; i < sampleCount; i++) {
    uint16_t sample = lcrCaptureBuffer[i * 2 + offset];

    if (sample < stats.minimum)
      stats.minimum = sample;

    if (sample > stats.maximum)
      stats.maximum = sample;

    sum += sample;
  }

  stats.mean = static_cast<float>(sum) / sampleCount;
  stats.peakToPeak = stats.maximum - stats.minimum;

  return stats;
}


// Print the scheduled acquisition parameters, a small portion of the raw
// record, and statistics calculated across the complete DMA capture.
void printLCRRawTest(const LCRCapturePlan &plan)
{
  Serial.println();
  Serial.println("LCR scheduled ADC test");

  Serial.print("Frequency: ");
  Serial.print(plan.frequency);
  Serial.println(" Hz");

  Serial.print("Aggregate sample rate: ");
  Serial.print(plan.aggregateSampleRate, 1);
  Serial.println(" samples/s");

  Serial.print("Channel sample rate: ");
  Serial.print(plan.channelSampleRate, 1);
  Serial.println(" samples/s");

  Serial.print("Samples/cycle/channel: ");
  Serial.println(plan.samplesPerCycle, 2);

  Serial.print("ADC clock divisor: ");
  Serial.println(plan.adcClockDiv, 3);

  Serial.print("Samples/channel: ");
  Serial.println(plan.samplesPerChannel);

  Serial.println();
  Serial.println("pair,ADC0,ADC1");

  uint16_t printCount = min<uint16_t>(16, plan.samplesPerChannel);

  for (uint16_t i = 0; i < printCount; i++) {
    uint16_t adc0 = lcrCaptureBuffer[i * 2];
    uint16_t adc1 = lcrCaptureBuffer[i * 2 + 1];

    Serial.print(i);
    Serial.print(",");
    Serial.print(adc0);
    Serial.print(",");
    Serial.println(adc1);
  }

  LCRCaptureStats adc0Stats =
    calculateLCRCaptureStats(0, plan.samplesPerChannel);

  LCRCaptureStats adc1Stats =
    calculateLCRCaptureStats(1, plan.samplesPerChannel);

  Serial.println();
  Serial.println("Full capture statistics");

  Serial.print("ADC0  min: ");
  Serial.print(adc0Stats.minimum);
  Serial.print("  max: ");
  Serial.print(adc0Stats.maximum);
  Serial.print("  mean: ");
  Serial.print(adc0Stats.mean, 1);
  Serial.print("  p-p: ");
  Serial.println(adc0Stats.peakToPeak);

  Serial.print("ADC1  min: ");
  Serial.print(adc1Stats.minimum);
  Serial.print("  max: ");
  Serial.print(adc1Stats.maximum);
  Serial.print("  mean: ");
  Serial.print(adc1Stats.mean, 1);
  Serial.print("  p-p: ");
  Serial.println(adc1Stats.peakToPeak);

  if (adc0Stats.peakToPeak > 0) {
    float ratio =
      static_cast<float>(adc1Stats.peakToPeak) / adc0Stats.peakToPeak;

    Serial.print("ADC1 / ADC0 p-p ratio: ");
    Serial.println(ratio, 4);
  }

  Serial.println();
}


// Process the most recent ADC capture through Goertzel and correct the known
// ADC0-to-ADC1 round-robin sampling delay before comparing the two channels.
void printLCRGoertzelTest(const LCRCapturePlan &plan)
{
  uint16_t n = plan.samplesPerChannel;

  if (n == 0 || plan.channelSampleRate <= 0.0f)
    return;

  float dcTotal = meanLCRCaptureChannel(n, 0);
  float dcDut = meanLCRCaptureChannel(n, 1);

  LCRPhasor vTotal =
    calculateLCRGoertzel(n, 0, plan.frequency, plan.channelSampleRate, dcTotal);

  LCRPhasor vDut =
    calculateLCRGoertzel(n, 1, plan.frequency, plan.channelSampleRate, dcDut);

  float totalAmplitude = lcrPhasorAmplitude(vTotal, n);
  float dutAmplitude = lcrPhasorAmplitude(vDut, n);

  float totalPhase = phaseLCRPhasor(vTotal);
  float dutPhaseRaw = phaseLCRPhasor(vDut);
  float rawDifference = normalizeLCRPhase(dutPhaseRaw - totalPhase);

  // ADC1 is sampled exactly one aggregate conversion period after ADC0.
  float skewSeconds = 1.0f / plan.aggregateSampleRate;
  float skewRadians = 2.0f * PI * plan.frequency *
                      skewSeconds * LCR_PHASOR_CONJ;

  LCRPhasor vDutCorrected = rotateLCRPhasor(vDut, -skewRadians);

  float dutPhaseCorrected = phaseLCRPhasor(vDutCorrected);
  float correctedDifference =
    normalizeLCRPhase(dutPhaseCorrected - totalPhase);

  Serial.println();
  Serial.println("LCR Goertzel / de-skew test");

  Serial.print("Vtotal amplitude: ");
  Serial.print(totalAmplitude, 3);
  Serial.println(" counts");

  Serial.print("Vdut amplitude:   ");
  Serial.print(dutAmplitude, 3);
  Serial.println(" counts");

  if (totalAmplitude > 0.0f) {
    Serial.print("Vdut / Vtotal:    ");
    Serial.println(dutAmplitude / totalAmplitude, 4);
  }

  Serial.print("Vtotal phase:      ");
  Serial.print(totalPhase, 3);
  Serial.println(" deg");

  Serial.print("Vdut raw phase:    ");
  Serial.print(dutPhaseRaw, 3);
  Serial.println(" deg");

  Serial.print("Raw difference:    ");
  Serial.print(rawDifference, 3);
  Serial.println(" deg");

  Serial.print("Predicted skew:    ");
  Serial.print(360.0f * plan.frequency * skewSeconds *
               LCR_PHASOR_CONJ, 3);
  Serial.println(" deg");

  Serial.print("Corrected phase:   ");
  Serial.print(dutPhaseCorrected, 3);
  Serial.println(" deg");

  Serial.print("Corrected diff:    ");
  Serial.print(correctedDifference, 3);
  Serial.println(" deg");

  Serial.println();
}


// Compare ADC0 and ADC1 when both channels are connected to the same signal.
// This isolates ADC channel gain and timing errors from the LCR divider.
void printLCRChannelMatchTest(uint32_t frequency)
{
  LCRCapturePlan plan;

  if (!captureLCRRawTest(frequency, plan)) {
    Serial.print("MATCH ");
    Serial.print(frequency);
    Serial.println(" Hz  ADC_ERR");
    return;
  }

  if (plan.measuredChannelRate <= 0.0f)
    return;

  uint16_t n = plan.samplesPerChannel;

  float dc0 = meanLCRCaptureChannel(n, 0);
  float dc1 = meanLCRCaptureChannel(n, 1);

  LCRPhasor adc0 =
    calculateLCRGoertzel(n, 0, frequency, plan.channelSampleRate, dc0);

  LCRPhasor adc1 =
    calculateLCRGoertzel(n, 1, frequency, plan.channelSampleRate, dc1);

  float amp0 = lcrPhasorAmplitude(adc0, n);
  float amp1 = lcrPhasorAmplitude(adc1, n);

  float rawPhase =
    normalizeLCRPhase(phaseLCRPhasor(adc1) - phaseLCRPhasor(adc0));

  float skewSeconds = 1.0f / plan.aggregateSampleRate;
  float skewRadians =
    2.0f * PI * frequency * skewSeconds * LCR_PHASOR_CONJ;

  LCRPhasor adc1Corrected = rotateLCRPhasor(adc1, -skewRadians);

  float correctedPhase =
    normalizeLCRPhase(
      phaseLCRPhasor(adc1Corrected) - phaseLCRPhasor(adc0)
    );

  Serial.print("MATCH ");
  Serial.print(frequency);
  Serial.print(" Hz  ratio=");

  if (amp0 > 0.0f)
    Serial.print(amp1 / amp0, 4);
  else
    Serial.print("ERR");

  Serial.print("  raw=");
  Serial.print(rawPhase, 3);
  Serial.print(" deg  predicted=");
  Serial.print(360.0f * frequency * skewSeconds * LCR_PHASOR_CONJ, 3);
  Serial.print(" deg  corrected=");
  Serial.print(correctedPhase, 3);
  Serial.print("  prog=");
  Serial.print(plan.aggregateSampleRate / 1000.0f, 1);
  Serial.print("k  meas=");
  Serial.print(plan.measuredAggregateRate / 1000.0f, 1);
  Serial.print("k");
  Serial.print(" deg  fs=");
  Serial.print(plan.measuredChannelRate / 1000.0f, 1);
  Serial.print(" kS/s  spc=");
  Serial.println(plan.measuredSamplesPerCycle, 2);
}



// Acquire one hardware record and calculate complex DUT impedance from the
// de-skewed Vtotal and Vdut phasors.
bool measureLCRHardwareImpedance(uint32_t frequency, float senseResistance,
                                 LCRPhasor &impedance)
{
  LCRCapturePlan plan;

  if (!captureLCRRawTest(frequency, plan))
    return false;

  if (plan.measuredChannelRate <= 0.0f)
    return false;

  // Temporary high-frequency ADC-rate diagnostic.
  if ((frequency >= 49000 && frequency <= 51000) ||
      (frequency >= 69000 && frequency <= 71000) ||
      (frequency >= 89000 && frequency <= 91000) ||
      frequency >= 109000) {
    printLCRCaptureRate(plan);
  }

  uint16_t n = plan.samplesPerChannel;

  if (n == 0 || plan.channelSampleRate <= 0.0f)
    return false;

  float dcTotal = meanLCRCaptureChannel(n, 0);
  float dcDut = meanLCRCaptureChannel(n, 1);

  LCRPhasor vTotal =
    calculateLCRGoertzel(n, 0, frequency, plan.channelSampleRate, dcTotal);

  LCRPhasor vDut =
    calculateLCRGoertzel(n, 1, frequency, plan.channelSampleRate, dcDut);

  // ADC1 is sampled one aggregate conversion period after ADC0.
  float skewSeconds = 1.0f / plan.aggregateSampleRate;
  float skewRadians =
    2.0f * PI * frequency * skewSeconds * LCR_PHASOR_CONJ;

  vDut = rotateLCRPhasor(vDut, -skewRadians);

  // V_sense is the complex voltage across the known reference resistor.
  LCRPhasor vSense = subtractLCRPhasor(vTotal, vDut);

  // Z_DUT = R_sense * V_DUT / V_sense
  impedance =
    scaleLCRPhasor(divideLCRPhasor(vDut, vSense), senseResistance);

  return true;
}


// Set the physical LCR excitation frequency only when it actually changes.
// Avoiding redundant AD9833 writes prevents unnecessary SPI traffic during
// continuous single-frequency measurements.
void setLCRGeneratorFrequency(uint32_t frequency)
{
  if (frequency == lcrGeneratorFrequency)
    return;

  lcrDDS.setFrequencyHz(frequency);
  lcrGeneratorFrequency = frequency;
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


// Render the primary component value.
//
// The measured impedance phase is used to classify the DUT as predominantly
// resistive, capacitive, or inductive. The large primary field then displays
// the corresponding component value rather than always displaying |Z|.
void drawLCRPrimaryMeasurement(const MeasurementPoint &m)
{
  const LCRRect &r = lcrLayout.primary;

  static const float COMPONENT_PHASE_THRESHOLD_DEG = 10.0f;

  LCRFormattedValue formatted;

  if (m.phaseDeg < -COMPONENT_PHASE_THRESHOLD_DEG &&
      m.capacitance > 0.0f) {

    //
    // Predominantly capacitive DUT.
    //
    formatted = formatCapacitance(m.capacitance);
  }
  else if (m.phaseDeg > COMPONENT_PHASE_THRESHOLD_DEG &&
           m.inductance > 0.0f) {

    //
    // Predominantly inductive DUT.
    //
    formatted = formatInductance(m.inductance);
  }
  else {

    //
    // Predominantly resistive DUT.
    //
    formatted = formatImpedance(m.resistance);
  }

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

  lcrValueSprite.pushSprite(
    r.x,
    r.y,
    0,
    0,
    r.w,
    r.h
  );
}


// Render the detailed single-frequency measurement results.
//
// The backend already produces the complete complex impedance measurement.
// This region exposes the derived C/L, phase, R, X, ESR, and Q values so the
// Measure tab can be used to validate reactive DUTs directly.
void drawLCRSecondaryMeasurement(const MeasurementPoint &m)
{
  const LCRRect &r = lcrLayout.secondary;

  lcrValueSprite.fillSprite(BGCOLOR);
  lcrValueSprite.setTextColor(TXTCOLOR, BGCOLOR);
  lcrValueSprite.setTextSize(1);

  const int16_t leftLabelX  = 8;
  const int16_t leftValueX  = r.w * 18 / 100;

  const int16_t rightLabelX = r.w * 53 / 100;
  const int16_t rightValueX = r.w * 68 / 100;

  const int16_t row1Y = r.h * 8 / 100;
  const int16_t row2Y = r.h * 40 / 100;
  const int16_t row3Y = r.h * 72 / 100;

  char value[24];

  //
  // Row 1 left: capacitance or inductance.
  //
  if (m.reactance < 0.0f) {
    LCRFormattedValue formatted = formatCapacitance(m.capacitance);

    lcrValueSprite.setCursor(leftLabelX, row1Y);
    lcrValueSprite.print("C");

    snprintf(
      value,
      sizeof(value),
      "%s %s",
      formatted.value,
      formatted.unit
    );
  }
  else if (m.reactance > 0.0f) {
    LCRFormattedValue formatted = formatInductance(m.inductance);

    lcrValueSprite.setCursor(leftLabelX, row1Y);
    lcrValueSprite.print("L");

    snprintf(
      value,
      sizeof(value),
      "%s %s",
      formatted.value,
      formatted.unit
    );
  }
  else {
    lcrValueSprite.setCursor(leftLabelX, row1Y);
    lcrValueSprite.print("C/L");

    snprintf(value, sizeof(value), "--");
  }

  lcrValueSprite.setCursor(leftValueX, row1Y);
  lcrValueSprite.print(value);

  //
  // Row 1 right: phase.
  //
  lcrValueSprite.setCursor(rightLabelX, row1Y);
  lcrValueSprite.print("Phase");

  snprintf(
    value,
    sizeof(value),
    "%.2f deg",
    m.phaseDeg
  );

  lcrValueSprite.setCursor(rightValueX, row1Y);
  lcrValueSprite.print(value);

  //
  // Row 2 left: series resistance.
  //
  lcrValueSprite.setCursor(leftLabelX, row2Y);
  lcrValueSprite.print("R");

  LCRFormattedValue resistance =
    formatImpedance(m.resistance);

  snprintf(
    value,
    sizeof(value),
    "%s %s",
    resistance.value,
    resistance.unit
  );

  lcrValueSprite.setCursor(leftValueX, row2Y);
  lcrValueSprite.print(value);

  //
  // Row 2 right: reactance.
  //
  lcrValueSprite.setCursor(rightLabelX, row2Y);
  lcrValueSprite.print("X");

  LCRFormattedValue reactance =
    formatImpedance(m.reactance);

  snprintf(
    value,
    sizeof(value),
    "%s %s",
    reactance.value,
    reactance.unit
  );

  lcrValueSprite.setCursor(rightValueX, row2Y);
  lcrValueSprite.print(value);

  //
  // Row 3 left: ESR.
  //
  lcrValueSprite.setCursor(leftLabelX, row3Y);
  lcrValueSprite.print("ESR");

  LCRFormattedValue esr =
    formatImpedance(m.esr);

  snprintf(
    value,
    sizeof(value),
    "%s %s",
    esr.value,
    esr.unit
  );

  lcrValueSprite.setCursor(leftValueX, row3Y);
  lcrValueSprite.print(value);

  //
  // Row 3 right: Q.
  //
  lcrValueSprite.setCursor(rightLabelX, row3Y);
  lcrValueSprite.print("Q");

  snprintf(
    value,
    sizeof(value),
    "%.3f",
    m.q
  );

  lcrValueSprite.setCursor(rightValueX, row3Y);
  lcrValueSprite.print(value);

  lcrValueSprite.pushSprite(
    r.x,
    r.y,
    0,
    0,
    r.w,
    r.h
  );
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

    if (strcmp(labels[i], "LIVE") == 0)
      display.setTextColor(TFT_GREEN, BGCOLOR);
    else if (strcmp(labels[i], "HOLD") == 0)
      display.setTextColor(TFT_RED, BGCOLOR);
    else
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



