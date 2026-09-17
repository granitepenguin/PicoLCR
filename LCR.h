#pragma once

#include <Arduino.h>

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


// Defines a rectangular region of the LCR user interface
// The same geometry is used for both drawing and touch detection
struct LCRRect
{
  int16_t x;
  int16_t y;
  int16_t w;
  int16_t h;
};


// Defines the major screen regions used by the LCR analyzer
// Common regions are shared by every tab, while measurement-specific
// regions subdivide the content area for the Measure display
struct LCRLayout
{
  LCRRect header;
  LCRRect tabs;
  LCRRect content;
  LCRRect footer;

  // Content subdivisions
  LCRRect primary;
  LCRRect secondary;
  LCRRect context;

  LCRRect measureSoftKeys[4];
};

extern LCRLayout lcrLayout;


// Calculate the LCR screen geometry from the active display dimensions
void calculateLCRLayout();


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
