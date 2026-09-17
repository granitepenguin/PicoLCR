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

// Off-screen drawing buffer used for dynamic LCR measurement fields.
// Rendering into RAM first allows the completed field to be transferred
// to the TFT at once, reducing visible erase/redraw flicker.
TFT_eSprite lcrValueSprite = TFT_eSprite(&display);

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
// The dynamic display is marked dirty so measurement values are drawn
// immediately when the instrument begins running.
void enterLCRMode()
{
  instrumentMode = MODE_LCR;
  lcrTab = LCR_TAB_MEASURE;
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
// Measurements are acquired continuously while TFT updates are rate-limited.
// A dirty flag forces an immediate refresh after entering or redrawing the
// analyzer interface.
void updateLCR()
{
  static MeasurementSettings settings = {
    1000,
    1000.0f
  };

  static MeasurementPoint measurement;
  static uint32_t lastDisplayUpdate = 0;

  measurement = measureImpedance(settings);

  uint32_t now = millis();

  if (lcrDisplayDirty ||
      now - lastDisplayUpdate >= LCR_DISPLAY_INTERVAL_MS) {

    lastDisplayUpdate = now;
    lcrDisplayDirty = false;

    updateLCRDisplay(measurement, settings);
  }

  uint16_t x, y;

  if (readTouch(x, y)) {
    if (y < 20) {
      exitLCRMode();
      return;
    }
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


// Calculate the major LCR screen regions from the current display size.
// All geometry is derived from the active display dimensions so the
// interface can adapt to different display resolutions.
void calculateLCRLayout()
{
  const int16_t screenW = display.width();
  const int16_t screenH = display.height();

  const int16_t headerH = screenH * 10 / 100;
  const int16_t tabsH = screenH * 10 / 100;
  const int16_t footerH = screenH * 12 / 100;

  const int16_t contentY = headerH + tabsH;
  const int16_t contentH = screenH - headerH - tabsH - footerH;
  const int16_t footerY = screenH - footerH;

  lcrLayout.header = { 0, 0, screenW, headerH };
  lcrLayout.tabs = { 0, headerH, screenW, tabsH };
  lcrLayout.content = { 0, contentY, screenW, contentH };
  lcrLayout.footer = { 0, footerY, screenW, footerH };

  // Divide the Measure content area into primary measurement,
  // secondary measurement, and measurement context regions.
  const int16_t primaryH = contentH * 42 / 100;
  const int16_t secondaryH = contentH * 25 / 100;

  const int16_t secondaryY = contentY + primaryH;
  const int16_t contextY = secondaryY + secondaryH;
  const int16_t contextH = contentH - primaryH - secondaryH;

  lcrLayout.primary = { 0, contentY, screenW, primaryH };
  lcrLayout.secondary = { 0, secondaryY, screenW, secondaryH };
  lcrLayout.context = { 0, contextY, screenW, contextH };
}



// Initialize the LCR analyzer when entering the instrument.
// Screen geometry and the dynamic measurement sprite are prepared before
// any analyzer UI elements are drawn.
void initializeLCR()
{
  calculateLCRLayout();
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
  display.setCursor(leftValueX, row1Y);
  display.print(m.frequency);
  display.print(" Hz");

  // Reference resistor
  display.setCursor(rightValueX, row1Y);
  display.print(settings.referenceResistance, 0);
  display.print(" Ohm");

  // Temporary excitation level.
  display.setCursor(leftValueX, row2Y);
  display.print("1.0 V");

  // Temporary averaging setting.
  display.setCursor(rightValueX, row2Y);
  display.print("32");
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

  // Back control.
  const char *backLabel = "< Back";
  int16_t backY = r.y + (r.h - 8) / 2;

  display.setCursor(r.x + 6, backY);
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


// Draw the Measure tab soft keys in the common footer.
// Button geometry is calculated from the footer width so the controls
// remain evenly spaced on different display resolutions.
void drawMeasureSoftKeys()
{
  const LCRRect &r = lcrLayout.footer;

  const char *labels[] = {
    "Freq",
    "Ref",
    "LIVE",
    "More"
  };

  const int16_t buttonCount = 4;
  const int16_t buttonW = r.w / buttonCount;

  display.fillRect(r.x, r.y, r.w, r.h, BGCOLOR);

  display.drawFastHLine( r.x, r.y, r.w, GRIDCOLOR);

  display.setTextSize(1);

  for (int16_t i = 0; i < buttonCount; i++) {
    int16_t x = r.x + i * buttonW;

    int16_t w = (i == buttonCount - 1)
      ? r.w - buttonW * i
      : buttonW;

    int16_t textWidth = strlen(labels[i]) * 6;
    int16_t textX = x + (w - textWidth) / 2;
    int16_t textY = r.y + (r.h - 8) / 2;

    display.setTextColor(TXTCOLOR, BGCOLOR);
    display.setCursor(textX, textY);
    display.print(labels[i]);

    if (i < buttonCount - 1) {
      display.drawFastVLine( x + w - 1, r.y + 3, r.h - 6, GRIDCOLOR);
    }
  }
}



// Draw the complete static LCR analyzer interface.
// Common navigation is drawn first, followed by the static content and
// soft keys belonging to the currently selected analyzer tab.
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



