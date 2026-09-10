//
// LCR Instrument
//


void initializeLCR()
{
  display.fillScreen(BGCOLOR);

  //
  // Future:
  //
  // Initialize AD9833
  // Configure ADC
  // Reset measurements
  // Load calibration
  //
}


//
// Single impedance measurement
//


// Get results from measurements
MeasurementPoint measureImpedance(uint32_t frequency)
{
  MeasurementPoint result;

  result.frequency = frequency;

  result.vrmsRef = 0.0f;
  result.vrmsDut = 0.0f;

  result.phaseDeg = 0.0f;

  // Test static code to show display update
  //
  static float z = 1000.0f;

  z += 0.1f;

  if (z > 1010.0f)
    z = 1000.0f;

  result.impedance = z;

  //result.impedance = 0.0f;

  result.resistance = 0.0f;
  result.reactance = 0.0f;

  result.capacitance = 0.0f;
  result.inductance = 0.0f;

  result.esr = 0.0f;

  result.q = 0.0f;
  result.dissipation = 0.0f;

  return result;
}


void enterLCRMode()
{
  instrumentMode = MODE_LCR;

  initializeLCR();

  drawLCRScreen();
}

void exitLCRMode()
{
  instrumentMode = MODE_SCOPE;

  display.fillScreen(BGCOLOR);

  DrawText();
}

void updateLCR()
{
  static float z = 1000.0f;

  z += 0.05f;

  if (z > 1010.0f)
    z = 1000.0f;

  display.setTextColor(TXTCOLOR, BGCOLOR);
  display.setTextSize(1);

  display.setCursor(120, 60);
  display.print(z, 2);
}

void drawLCRScreen()
{
  display.fillScreen(BGCOLOR);

  display.setTextColor(TXTCOLOR, BGCOLOR);

  display.setTextSize(2);
  display.setCursor(70, 20);
  display.print("LCR METER");

  display.setTextSize(1);

  display.setCursor(30, 70);
  display.print("Milestone 2.1");

  display.setCursor(30, 90);
  display.print("GUI Framework Complete");

  display.setCursor(30, 120);
  display.print("Measurement engine");
  display.setCursor(30, 132);
  display.print("coming next...");
}

void updateLCRDisplay(const MeasurementPoint &m)
{
  display.setTextColor(TXTCOLOR, BGCOLOR);
  display.setTextSize(1);

  display.setCursor(120, 40);
  display.print(m.frequency);

  display.setCursor(120, 60);
  display.print(m.impedance, 3);

  display.setCursor(120, 80);
  display.print(m.phaseDeg, 2);

  display.setCursor(120,100);
  display.print(m.resistance, 3);

  display.setCursor(120,120);
  display.print(m.reactance, 3);
}
