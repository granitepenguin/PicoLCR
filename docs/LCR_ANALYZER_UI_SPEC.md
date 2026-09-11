# LCR / Impedance Analyzer
## User Interface Specification

**Version:** 1.0 Draft

**Project:** PicoLCR addition to GOscillo

---

# 1. Introduction

## 1.1 Purpose

...

## 1.2 Project Goals

...

## 1.3 Scope

...

---

# 2. Design Philosophy

## 2.1 User Experience Goals

- Fast access
- Consistency
- Primary measurement first
- Minimal navigation
- Extensibility

## 2.2 Operating Philosophy

The analyzer behaves as a dedicated bench instrument rather than an
oscilloscope feature.

Users remain inside the analyzer while navigating between:

- Measure
- Sweep
- Calibration
- Settings

The analyzer manages workflow internally.

## 2.3 Development Strategy

Development is intentionally divided into:

GUI

↓

Measurement Engine

↓

Hardware

Simulation mode allows complete UI development before measurement
hardware is available.

---

# 3. Instrument Architecture

## 3.1 Current Navigation

Oscilloscope
↓

Function Menu
↓

LCR Analyzer

## 3.2 Future Navigation (Conceptual)

Home
├── Oscilloscope
├── LCR Analyzer
├── Function Generator
├── Frequency Counter
└── Settings

This future navigation is conceptual and not part of Version 1.0.

## 3.3 Instrument Hierarchy

LCR Analyzer
├── Measure
├── Sweep
├── Calibration
└── Settings

These are peer-level tabs.

The user never leaves the analyzer while moving between them.

---

# 4. Common User Interface

Common header

Common tabs

Common footer

Back button

Instrument title

Soft key area

Static vs Dynamic display philosophy

---

# 5. User Workflows

## 5.1 Quick Measurement

Enter Analyzer
↓

Connect DUT
↓

Select Frequency
↓

Observe Measurement
↓

LIVE/HOLD (optional)
↓

Return to Scope

---

## 5.2 Frequency Sweep

Enter Analyzer
↓

Sweep
↓

Configure
↓

Run
↓

Progress
↓

Results
↓

Cursor
↓

Re-Sweep

---

# 6. Measure Tab

Purpose: The Measure tab provides continuous impedance measurements and is the
default page displayed whenever the analyzer is entered.

Layout/Wireframe

```
+------------------------------------------------------+
| ← Scope             LCR ANALYZER                     |
+------------------------------------------------------+

| Measure | Sweep | Cal | Settings |

--------------------------------------------------------
Primary measurement

                    100.18

                      nF

--------------------------------------------------------
Secondary measurement

                       D

                    0.0001

--------------------------------------------------------
Measurement context
Freq:   1.000 kHz

Ref:    1.000 kΩ

Level:  1.0 V

Avg:    32

--------------------------------------------------------

Freq      Ref      LIVE      More
```


Frequency selection

Reference selection

LIVE/HOLD

More menu

---

# 7. Sweep Tab

## Sweep States

### Setup

```
+------------------------------------------------------+
| ← Scope             LCR ANALYZER                     |
+------------------------------------------------------+

| Measure | Sweep | Cal | Settings |

--------------------------------------------------------

Start

100 Hz

Stop

110 kHz

Mode

● Linear

○ Log

--------------------------------------------------------

Step

[100] [1k] [10k] [Custom]

--------------------------------------------------------

Measurements

1101

Sweep Time

22.0 s

--------------------------------------------------------

                 Run Sweep
```
### Running

```
+------------------------------------------------------+
| ← Scope             LCR ANALYZER                     |
+------------------------------------------------------+

| Measure | Sweep | Cal | Settings |

--------------------------------------------------------

Running Sweep

██████████████------------

523 / 1101

Remaining

12.4 s

--------------------------------------------------------

              Cancel Sweep
```

### Results
```
+------------------------------------------------------+
| ← Scope             LCR ANALYZER                     |
+------------------------------------------------------+

| Measure | Sweep | Cal | Settings |

--------------------------------------------------------

             |Z| vs Frequency

│
│
│             ●
│         ●
│      ●
│   ●
└────────────────────────────────────────

100 Hz                          110 kHz

--------------------------------------------------------

Cursor      Plot      Re-Sweep
```

### Plot Functions
Cursor: allow for displaying values by tapping on plot

Plot selection: cycle through plots vs Freq

Re-Sweep : Re-run a sweep (preloading previous settings)



---

# 8. Calibration Tab

Purpose

Workflow

```
+------------------------------------------------------+
| ← Scope             LCR ANALYZER                     |
+------------------------------------------------------+

| Measure | Sweep | Cal | Settings |

--------------------------------------------------------

Calibration

Current Step

OPEN

--------------------------------------------------------

Disconnect DUT

Leave fixture open.

Press Continue when ready.

--------------------------------------------------------

Back        Continue
```

Open

Short

(Optional) Load

Complete

---

# 9. Settings Tab


```
+------------------------------------------------------+
| ← Scope             LCR ANALYZER                     |
+------------------------------------------------------+

| Measure | Sweep | Cal | Settings |

--------------------------------------------------------

Measurement

Reference Resistor

100 Ω

1 kΩ

10 kΩ

--------------------------------------------------------

System

Simulation

Hardware

--------------------------------------------------------

Display

Averaging

About
```

---

# 10. Instrument State Model

Measure
├── LIVE
└── HOLD

Sweep
├── SETUP
├── RUNNING
└── RESULTS

Calibration
├── OPEN
├── SHORT
├── LOAD
└── COMPLETE

Settings

(single state)

---

# 11. Navigation Model

                   GOscillo
                      │
            Function Menu (Version 1)
                      │
                 LCR Analyzer
        ┌─────────────┼─────────────┬──────────────┐
        │             │             │              │
        ▼             ▼             ▼              ▼
    Measure        Sweep      Calibration      Settings

This navigation model is expected to change in future versions as the
GOscillo platform evolves into a collection of independent instruments.

---

# 12. Future Enhancements

## Measurement

- Auto-ranging
- Component identification
- Equivalent circuit modeling

## Sweep

- Graph overlays
- Save/load traces
- Marker search
- Multiple cursors

## Display

- Zoom
- Pan
- Multiple graph windows

## Connectivity

- CSV export
- Remote control
- Screen capture

---

# Appendix A – Definitions

DUT

: Device Under Test

ESR

: Equivalent Series Resistance

Q

: Quality Factor

D

: Dissipation Factor

|Z|

: Magnitude of Complex Impedance

---

# Revision History

| Version | Date | Description |
|----------|------|-------------|
| 1.0 Draft | 11 Sept 2026 | Initial user interface specification |
