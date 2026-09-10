#pragma once

#include <Arduino.h>

typedef struct 
{
  uint32_t frequency;

  float vrmsRef;
  float vrmsDut;

  float phaseDeg;

  float impedance;

  float resistance;
  float reactance;

  float capacitance;
  float inductance;

  float esr;

  float q;
  float dissipation;
} MeasurementPoint;

struct MeasurementSettings
{
  uint32_t frequency;
  float referenceResistance;
};

MeasurementPoint measureImpedance(uint32_t frequency);

void initializeLCR();
void enterLCRMode();
void exitLCRMode();
void updateLCR();
void drawLCRScreen();
void updateLCRDisplay(const MeasurementPoint &m);

