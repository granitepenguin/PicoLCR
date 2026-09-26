// LCR.h
// ====================================================================
// LCR Instrument Interface
//
// This header defines the public interface for the LCR / Impedance
// Analyzer instrument.
//
// It contains:
//
//    Measurement data structures
//    Instrument configuration structures
//    Backend selection
//    Public API
//
// ====================================================================

#pragma once

#include <Arduino.h>


// Results from a single impedance measurement.
//
// This structure represents one measurement at a single frequency.
// It is used by both the single-measurement mode and the future
// sweep mode.
struct MeasurementPoint
{
  uint32_t frequency;      // Measurement frequency (Hz)

  float vrmsRef;           // RMS voltage across reference resistor
  float vrmsDut;           // RMS voltage across DUT

  float phaseDeg;          // DUT phase angle (degrees)

  float impedance;         // |Z| Magnitude (Ohms)

  float resistance;        // Real component (Ohms)
  float reactance;         // Imaginary component (Ohms)

  float capacitance;       // Equivalent capacitance (Farads)
  float inductance;        // Equivalent inductance (Henries)

  float esr;               // Equivalent Series Resistance (Ohms)

  float q;                 // Quality factor
  float dissipation;       // Dissipation factor
};


// Stores the measurement data retained for one frequency-sweep point.
// This is intentionally smaller than MeasurementPoint so large sweeps can
// be retained in RAM without storing intermediate acquisition values.
struct SweepPoint
{
  uint32_t frequency;

  float impedance;
  float phaseDeg;

  float resistance;
  float reactance;

  float esr;
  float q;
};

// Stores the minimum and maximum values used to scale one Sweep plot axis.
struct SweepPlotRange
{
  float minimum;
  float maximum;
};

// Defines the common engineering scale used by a Sweep plot axis.
// All numeric labels on the axis use the same multiplier and unit so
// individual tick labels can remain compact.
struct SweepAxisScale
{
  float divisor;
  const char *unit;
};

// Identifies which retained Sweep measurement is displayed on the Results
// graph. Changing the plot type does not require another acquisition because
// each SweepPoint already stores all supported result quantities.
enum LCRSweepPlotType
{
  LCR_SWEEP_PLOT_IMPEDANCE,
  LCR_SWEEP_PLOT_PHASE,
  LCR_SWEEP_PLOT_RESISTANCE,
  LCR_SWEEP_PLOT_REACTANCE,
  LCR_SWEEP_PLOT_ESR,
  LCR_SWEEP_PLOT_Q
};

extern LCRSweepPlotType lcrSweepPlotType;

// Tracks the currently inspected Sweep result.
// The cursor index identifies one retained SweepPoint; cursorActive controls
// whether the Results graph and status strip display the selected point.
extern bool lcrSweepCursorActive;
extern uint16_t lcrSweepCursorIndex;

// Maximum number of measurement points retained for one frequency sweep.
// This accommodates the current full-range 100 Hz-step linear sweep while
// bounding RAM usage for sweep-result storage.
constexpr uint16_t LCR_MAX_SWEEP_POINTS = 1200;
extern SweepPoint lcrSweepPoints[LCR_MAX_SWEEP_POINTS];
extern uint16_t lcrSweepPointCount;

// Measurement configuration.
//
// Defines the parameters required to perform a single measurement.
// This structure will expand as additional features such as averaging,
// autoranging, and sweep support are added.
struct MeasurementSettings
{
  uint32_t frequency;          // Test frequency (Hz)
  float referenceResistance;   // Selected reference resistor (Ohms)
};

// Stores the active measurement configuration for the LCR analyzer.
// These settings are shared by measurement acquisition and UI controls
// such as the frequency and reference-resistor selectors.
extern MeasurementSettings lcrSettings;

// Stores the most recently acquired LCR measurement.
// Keeping the latest result as instrument state allows HOLD measurements
// to be restored after temporary selector screens are closed.
extern MeasurementPoint lcrMeasurement;


// Identifies how measurement frequencies are distributed during a sweep.
// LINEAR uses a fixed frequency step; LOG uses a logarithmic distribution.
enum LCRSweepMode
{
  LCR_SWEEP_LINEAR,
  LCR_SWEEP_LOG
};

// Stores the configuration used to perform an LCR frequency sweep.
// Linear sweeps use stepFrequency; logarithmic sweeps use pointsPerDecade.
struct SweepSettings
{
  uint32_t startFrequency;
  uint32_t stopFrequency;

  LCRSweepMode mode;

  uint32_t stepFrequency;
  uint16_t pointsPerDecade;
};

// Stores the active sweep configuration used by the LCR analyzer.
extern SweepSettings lcrSweepSettings;


// Stores the runtime state of an active LCR frequency sweep.
// Configuration remains in SweepSettings while this structure tracks
// progress and timing through the currently executing sweep.
struct SweepExecution
{
  uint32_t totalPoints;
  uint32_t currentPoint;
  uint32_t currentFrequency;

  uint32_t startTime;
  uint32_t lastPointTime;

  bool active;
};

extern SweepExecution lcrSweepExecution;


// Holds a numeric measurement formatted for display.
// The value and engineering unit are kept separate so the renderer can
// use different font sizes while treating them as one measurement.
struct LCRFormattedValue
{
  char value[16];
  char unit[8];
};



// LCR tab displays
//
// enumberation of all the major LCR tabs
enum LCRTab
{
  LCR_TAB_MEASURE,
  LCR_TAB_SWEEP,
  LCR_TAB_CALIBRATION,
  LCR_TAB_SETTINGS
};

extern LCRTab lcrTab;


// Identifies the operating state of the LCR Measure tab.
// LIVE continuously acquires measurements while HOLD preserves the
// most recently acquired measurement on the display.
enum LCRMeasureState
{
  LCR_MEASURE_LIVE,
  LCR_MEASURE_HOLD
};

extern LCRMeasureState lcrMeasureState;


// Identifies the current operating state of the Sweep tab.
// A sweep begins in SETUP, transitions through RUNNING, and automatically
// displays RESULTS when acquisition is complete.
enum LCRSweepState
{
  LCR_SWEEP_SETUP,
  LCR_SWEEP_RUNNING,
  LCR_SWEEP_RESULTS
};

extern LCRSweepState lcrSweepState;


// Identifies the current LCR user-interface state.
// NORMAL displays the active analyzer tab, while selector states temporarily
// replace the tab content with a modal control.
enum LCRUIState
{
  LCR_UI_NORMAL,
  LCR_UI_FREQ_SELECT,
  LCR_UI_REF_SELECT,
  LCR_UI_SWEEP_MODE_SELECT,
  LCR_UI_SWEEP_STEP_SELECT,
  LCR_UI_SWEEP_DENSITY_SELECT,
  LCR_UI_SWEEP_PLOT_SELECT
};

extern LCRUIState lcrUIState;

// Identifies which frequency setting is currently being edited.
// The common frequency selector uses this target to update either the
// Measure frequency or one of the Sweep frequency limits.
enum LCRFrequencyTarget
{
  LCR_FREQ_MEASURE,
  LCR_FREQ_SWEEP_START,
  LCR_FREQ_SWEEP_STOP
};

extern LCRFrequencyTarget lcrFrequencyTarget;

// Defines a rectangular region of the LCR user interface
// The same geometry is used for both drawing and touch detection
struct LCRRect
{
  int16_t x;
  int16_t y;
  int16_t w;
  int16_t h;
};

// Maximum number of preset buttons supported by an LCR selector.
// Actual button counts are derived from their preset data tables.
constexpr uint8_t LCR_MAX_SELECTOR_BUTTONS = 8;


// Defines the major screen regions used by the LCR analyzer
// Common regions are shared by every tab, while measurement-specific
// regions subdivide the content area for the Measure display
struct LCRLayout
{
  LCRRect header;

  LCRRect backButton;

  LCRRect tabs;
  LCRRect tabButtons[4];

  // Measurement tab
  LCRRect content;
  // Content subdivisions
  LCRRect primary;
  LCRRect secondary;
  LCRRect context;

  // Sweep setup tab
  LCRRect sweepSetupRows[5];
  LCRRect sweepModePresets[2];
  LCRRect sweepButton;
  LCRRect sweepProgress;
  LCRRect sweepCancelButton;
  LCRRect sweepResultsSetupButton;
  LCRRect sweepResultsSweepButton;

  // Sweep Results plot area.

  // Sweep Results control/status strip.
  // This provides a large touch target for selecting the plotted quantity
  // while keeping the graph itself available for future cursor interaction.
  LCRRect sweepPlotControl;

  // This rectangle contains the graph itself, excluding axis labels and footer.
  LCRRect sweepPlot;


  LCRRect footer;

  // Submenus
  LCRRect measureSoftKeys[4];

  // Softkey screen info
  LCRRect selector;
  LCRRect frequencyPresets[LCR_MAX_SELECTOR_BUTTONS];
  LCRRect referencePresets[LCR_MAX_SELECTOR_BUTTONS];
  LCRRect sweepStepPresets[LCR_MAX_SELECTOR_BUTTONS];
  LCRRect sweepDensityPresets[LCR_MAX_SELECTOR_BUTTONS];
  LCRRect sweepPlotPresets[LCR_MAX_SELECTOR_BUTTONS];
  LCRRect selectorCancel;
};

extern LCRLayout lcrLayout;


//
// screen geometry functions
//

// Calculate the common screen regions shared by every LCR analyzer tab.
void calculateLCRLayout();

// Calculate regions specific to the Measure tab.
void calculateMeasureLayout();

// Calculate interactive row geometry for the Sweep Setup screen.
void calculateSweepLayout();

// Calculate geometry shared by modal selector screens.
void calculateSelectorLayout();

// Calculate button geometry for the frequency selector.
void calculateFrequencySelectorLayout();

// Calculate button geometry for the reference-resistor selector.
void calculateReferenceSelectorLayout();

// Calculate button geometry for the Sweep-mode selector.
void calculateSweepModeSelectorLayout();

// Calculate button geometry for the linear Sweep-step selector.
void calculateSweepStepSelectorLayout();

// Calculate button geometry for the logarithmic Sweep-density selector.
void calculateSweepDensitySelectorLayout();

// Calculate button geometry for the Sweep Results plot selector.
void calculateSweepPlotSelectorLayout();



// Measurement backend.
//
// Simulation mode allows development of the GUI and workflow before
// measurement hardware is available.
enum LCRBackend
{
  LCR_BACKEND_SIMULATION,
  LCR_BACKEND_HARDWARE
};

extern LCRBackend lcrBackend;


//
// Public API
//

// Perform one impedance measurement using the currently selected backend.
MeasurementPoint measureImpedance(const MeasurementSettings &settings);

// Initialize the LCR instrument.
void initializeLCR();

// Enter / leave LCR instrument mode.
void enterLCRMode();
void exitLCRMode();

// Main LCR task.
// Called once each pass through loop() while in LCR mode.
void updateLCR();

// Draw the static LCR instrument user interface.
void drawLCRScreen();

// Update the dynamic Measure-tab fields from the latest measurement
void updateLCRDisplay(const MeasurementPoint &m,
                      const MeasurementSettings &settings);

// Format raw SI measurements into human-readable engineering units.
LCRFormattedValue formatCapacitance(float farads);
LCRFormattedValue formatInductance(float henries);
LCRFormattedValue formatImpedance(float ohms);
LCRFormattedValue formatFrequency(uint32_t frequencyHz);
