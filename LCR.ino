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
  uint16_t x, y;

  if (readTouch(x, y))
  {
    if (y < 20)
    {
      exitLCRMode();
    }
  }
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