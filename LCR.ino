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

// Off-screen drawing buffer used for dynamic LCR measurement fields.
// Rendering into RAM first allows the completed field to be transferred
// to the TFT at once, reducing visible erase/redraw flicker.
TFT_eSprite lcrValueSprite = TFT_eSprite(&display);


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
// LIVE mode continuously acquires and periodically displays measurements.
// HOLD preserves the last measurement while leaving all UI controls active.
void updateLCR()
{
  static uint32_t lastDisplayUpdate = 0;
  static bool lastPressed = false;

  uint32_t now = millis();

  // Acquire and update live measurements only while the Measure tab is active.
  // This prevents Measure rendering from continuing underneath other LCR tabs.
  if (lcrTab == LCR_TAB_MEASURE &&
      lcrMeasureState == LCR_MEASURE_LIVE &&
      lcrUIState == LCR_UI_NORMAL) {

    lcrMeasurement = measureImpedance(lcrSettings);

    if (lcrDisplayDirty ||
        now - lastDisplayUpdate >= LCR_DISPLAY_INTERVAL_MS) {

      lastDisplayUpdate = now;
      lcrDisplayDirty = false;

      updateLCRDisplay(lcrMeasurement, lcrSettings);
    }
  }

  //
  // Touch handling code 
  //
  uint16_t x, y;
  bool pressed = readTouch(x, y);

  // Require release before accepting another touch.
  if (!pressed) {
    lastPressed = false;
    return;
  }

  if (lastPressed)
    return;

  lastPressed = true;

  // Exit the analyzer only when the dedicated Back control is touched.
  // The remainder of the header is intentionally non-interactive.
  if (pointInLCRRect(x, y, lcrLayout.backButton)) {
    exitLCRMode();
    return;
  }

  // Top-level analyzer tabs remain available whenever no measurement
  // operation explicitly locks navigation.
  if (pointInLCRRect(x, y, lcrLayout.tabs)) {
    handleLCRTabTouch(x, y);
    return;
  }

  // Modal selectors consume touch input before the underlying tab.
  if (lcrUIState == LCR_UI_FREQ_SELECT) {
    handleLCRFrequencySelectorTouch(x, y);
    return;
  }

  // Route modal reference-selector touches before the underlying
  // Measure-screen controls are allowed to process them.
  if (lcrUIState == LCR_UI_REF_SELECT) {
    handleLCRReferenceSelectorTouch(x, y);
    return;
  }

  if (lcrTab == LCR_TAB_MEASURE &&
      pointInLCRRect(x, y, lcrLayout.footer)) {

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
  const int16_t contentH =
    screenH - headerH - tabsH - footerH;

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
    int16_t x =
      lcrLayout.tabs.x + i * tabW;

    int16_t w = (i == tabCount - 1)
      ? lcrLayout.tabs.w - tabW * i
      : tabW;

    lcrLayout.tabButtons[i] = {
      x,
      lcrLayout.tabs.y,
      w,
      lcrLayout.tabs.h
    };
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

  lcrLayout.primary = {
    content.x,
    content.y,
    content.w,
    primaryH
  };

  lcrLayout.secondary = {
    content.x,
    secondaryY,
    content.w,
    secondaryH
  };

  lcrLayout.context = {
    content.x,
    contextY,
    content.w,
    contextH
  };

  const int16_t softKeyCount = 4;
  const int16_t softKeyW =
    footer.w / softKeyCount;

  for (int16_t i = 0; i < softKeyCount; i++) {
    int16_t x =
      footer.x + i * softKeyW;

    int16_t w = (i == softKeyCount - 1)
      ? footer.w - softKeyW * i
      : softKeyW;

    lcrLayout.measureSoftKeys[i] = {
      x,
      footer.y,
      w,
      footer.h
    };
  }
}



// Calculate geometry shared by modal selectors.
// Selector dimensions are controlled by the common LCR UI tuning constants
// so visual adjustments do not require changes to the layout algorithm.
void calculateSelectorLayout()
{
  const LCRRect &content = lcrLayout.content;
  const LCRRect &footer = lcrLayout.footer;

  const int16_t selectorH =
    content.h + footer.h;

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

  const int16_t gridTop = selector.y + selector.h * LCR_SELECTOR_TOP_PERCENT / 100;

  const int16_t gridBottom =
    lcrLayout.selectorCancel.y -
    selector.h * LCR_SELECTOR_BOTTOM_GAP_PERCENT / 100;

  const int16_t availableGridH = gridBottom - gridTop;

  const int16_t buttonW =
    (selector.w -
     marginX * 2 -
     gapX * (columns - 1)) / columns;

  const int16_t maxButtonH =
    display.height() *
    LCR_SELECTOR_BUTTON_HEIGHT_PERCENT / 100;

  int16_t buttonH =
    (availableGridH -
     gapY * (rows - 1)) / rows;

  buttonH = min(buttonH, maxButtonH);

  const int16_t actualGridH =
    rows * buttonH +
    (rows - 1) * gapY;

  const int16_t centeredGridTop =
    gridTop +
    (availableGridH - actualGridH) / 2;

  for (uint8_t i = 0; i < count; i++) {
    uint8_t row = i / columns;
    uint8_t column = i % columns;

    uint8_t itemsInRow =
      min(
        static_cast<uint8_t>(columns),
        static_cast<uint8_t>(count - row * columns)
      );

    int16_t rowWidth =
      itemsInRow * buttonW +
      (itemsInRow - 1) * gapX;

    int16_t rowX =
      selector.x +
      (selector.w - rowWidth) / 2;

    int16_t x =
      rowX +
      column * (buttonW + gapX);

    int16_t y =
      centeredGridTop +
      row * (buttonH + gapY);

    buttons[i] = {
      x,
      y,
      buttonW,
      buttonH
    };
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



// Initialize the LCR analyzer and calculate all currently supported UI
// geometry before drawing the instrument screen.
void initializeLCR()
{
  calculateLCRLayout();
  calculateMeasureLayout();
  calculateSelectorLayout();
  calculateFrequencySelectorLayout();
  calculateReferenceSelectorLayout();

  initializeLCRValueSprite();

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


//
// Touch handlers
//

// Handle touches on the four top-level analyzer tabs.
// Changing tabs closes any modal selector and redraws the newly selected
// analyzer view while preserving instrument-level measurement settings.
void handleLCRTabTouch(uint16_t x, uint16_t y)
{
  for (uint8_t i = 0; i < 4; i++) {
    if (!pointInLCRRect(x, y, lcrLayout.tabButtons[i]))
      continue;

    LCRTab newTab = static_cast<LCRTab>(i);

    if (newTab == lcrTab)
      return;

    lcrTab = newTab;
    lcrUIState = LCR_UI_NORMAL;
    lcrDisplayDirty = true;

    drawLCRScreen();
    return;
  }
}


// Handle touches within the Measure-tab soft-key footer.
// Only LIVE/HOLD is implemented during this milestone; the remaining
// buttons are reserved for their upcoming Measure-control milestones.
void handleMeasureSoftKeyTouch(uint16_t x, uint16_t y)
{
  // Open the frequency preset selector.
  if (pointInLCRRect(x, y, lcrLayout.measureSoftKeys[0])) {
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


// Handle touch input while the frequency selector is displayed.
// Selecting a preset updates the shared measurement settings and returns
// the analyzer to LIVE measurement; Cancel leaves all settings unchanged.
void handleLCRFrequencySelectorTouch(uint16_t x, uint16_t y)
{
  // Match touches against the frequency preset table so the same data
  // controls layout, labels, values, and touch handling.
  for (uint8_t i = 0; i < FREQUENCY_PRESET_COUNT; i++) {
    if (pointInLCRRect(
          x,
          y,
          lcrLayout.frequencyPresets[i])) {

      lcrSettings.frequency =
        frequencyPresets[i].value;

      lcrMeasureState = LCR_MEASURE_LIVE;

      returnToLCRMeasureScreen();
      return;
    }
  }

  if (pointInLCRRect(
        x,
        y,
        lcrLayout.selectorCancel)) {

    returnToLCRMeasureScreen();
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

      lcrSettings.referenceResistance =
        referencePresets[i].value;

      lcrMeasureState = LCR_MEASURE_LIVE;

      returnToLCRMeasureScreen();
      return;
    }
  }

  if (pointInLCRRect(
        x,
        y,
        lcrLayout.selectorCancel)) {

    returnToLCRMeasureScreen();
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

  drawLCRSpriteMeasurement( r.w, r.h, formatted.value, formatted.unit, 4, 2, TXTCOLOR);

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

  int16_t totalWidth =
    labelWidth + gap + valueWidth;

  int16_t startX =
    (r.w - totalWidth) / 2;

  int16_t valueHeight = 8 * valueSize;

  int16_t valueY =
    (r.h - valueHeight) / 2;

  int16_t labelY =
    valueY + valueHeight - (8 * labelSize);

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


// Draw the initial Sweep Setup view.
// This first implementation establishes the approved screen structure;
// interactive sweep controls are added in subsequent milestones.
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

  int16_t leftX =
    r.x + r.w * 10 / 100;

  int16_t valueX =
    r.x + r.w * 55 / 100;

  int16_t rowH =
    r.h / 5;

  display.setCursor(leftX, r.y + rowH * 0 + 8);
  display.print("Start");

  display.setCursor(valueX, r.y + rowH * 0 + 8);
  display.print("100 Hz");

  display.setCursor(leftX, r.y + rowH * 1 + 8);
  display.print("Stop");

  display.setCursor(valueX, r.y + rowH * 1 + 8);
  display.print("110 kHz");

  display.setCursor(leftX, r.y + rowH * 2 + 8);
  display.print("Mode");

  display.setCursor(valueX, r.y + rowH * 2 + 8);
  display.print("Linear");

  display.setCursor(leftX, r.y + rowH * 3 + 8);
  display.print("Step");

  display.setCursor(valueX, r.y + rowH * 3 + 8);
  display.print("1 kHz");

  display.setCursor(leftX, r.y + rowH * 4 + 8);
  display.print("Measurements");

  display.setCursor(valueX, r.y + rowH * 4 + 8);
  display.print("---");
}


// Draw the Sweep Setup footer.
// The Sweep action will become interactive once sweep configuration and
// acquisition state are implemented.
void drawLCRSweepFooter()
{
  const LCRRect &r = lcrLayout.footer;

  display.fillRect(
    r.x,
    r.y,
    r.w,
    r.h,
    BGCOLOR
  );

  display.drawFastHLine(
    r.x,
    r.y,
    r.w,
    GRIDCOLOR
  );

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


// Draw a rectangular selector button with a centered text label.
// The supplied rectangle is also used by touch detection, keeping visual
// controls and touch targets synchronized.
void drawLCRSelectorButton(const LCRRect &rect,
                           const char *label,
                           bool selected)
{
  uint16_t color = selected ? HIGHCOLOR : TXTCOLOR;

  display.drawRect(
    rect.x,
    rect.y,
    rect.w,
    rect.h,
    color
  );

  display.setTextSize(1);
  display.setTextColor(color, BGCOLOR);

  int16_t textWidth = strlen(label) * 6;
  int16_t textX =
    rect.x + (rect.w - textWidth) / 2;

  int16_t textY =
    rect.y + (rect.h - 8) / 2;

  display.setCursor(textX, textY);
  display.print(label);
}


// Draw the frequency preset selector over the normal Measure content.
// The current frequency is highlighted and the underlying Measure controls
// remain hidden until a preset is selected or the operation is cancelled.
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

  const char *title = "Select Frequency";

  int16_t titleWidth = strlen(title) * 6;

  display.setCursor(
    r.x + (r.w - titleWidth) / 2,
    r.y + r.h * 7 / 100
  );

  display.print(title);

  // Draw each frequency preset directly from the shared preset table.
  for (uint8_t i = 0; i < FREQUENCY_PRESET_COUNT; i++) {
    bool selected =
      lcrSettings.frequency == frequencyPresets[i].value;

    drawLCRSelectorButton(
      lcrLayout.frequencyPresets[i],
      frequencyPresets[i].label,
      selected
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
        case LCR_SWEEP_RESULTS:
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



