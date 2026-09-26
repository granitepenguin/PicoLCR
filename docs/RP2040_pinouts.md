# RP2040 Pins
| Pico physical pin | GPIO / supply   | Current function              | Interface / notes                     |
| ----------------: | --------------- | ----------------------------- | ------------------------------------- |
|             **1** | GP0             | PWM / pulse generator output  | Existing GOscillo                     |
|             **2** | GP1             | Unassigned                    | Keep available                        |
|             **3** | GND             | Digital ground                | General ground                        |
|             **4** | GP2             | GOscillo software DDS output  | Existing GOscillo                     |
|             **5** | **GP3**         | **AD9833 FSYNC**              | **LCR — finalized**                   |
|             **6** | GP4             | FT6206 SDA                    | Touchscreen I²C                       |
|             **7** | GP5             | FT6206 SCL                    | Touchscreen I²C                       |
|             **8** | GND             | Digital ground                | General ground                        |
|             **9** | **GP6**         | **AD9833 SCLK / SPI0 SCK**    | **LCR — finalized**                   |
|            **10** | **GP7**         | **AD9833 SDATA / SPI0 TX**    | **LCR — finalized**                   |
|            **11** | GP8             | Unassigned                    | Available                             |
|            **12** | GP9             | Unassigned on Pico W build    | Used conditionally by Waveshare build |
|            **13** | GND             | Digital ground                | General ground                        |
|            **14** | GP10            | TFT SCLK                      | Display SPI1                          |
|            **15** | GP11            | TFT MOSI                      | Display SPI1                          |
|            **16** | GP12            | TFT MISO                      | Display SPI1                          |
|            **17** | GP13            | TFT CS                        | Display                               |
|            **18** | GND             | Digital ground                | General ground                        |
|            **19** | GP14            | TFT RESET                     | Display                               |
|            **20** | GP15            | TFT DC                        | Display                               |
|            **21** | GP16            | CH0 DC/AC switch              | Existing scope                        |
|            **22** | GP17            | CH1 DC/AC switch              | Existing scope                        |
|            **23** | GND             | Digital ground                | General ground                        |
|            **24** | GP18            | LEFT button                   | Existing scope controls               |
|            **25** | GP19            | RIGHT button                  | Existing scope controls               |
|            **26** | GP20            | UP button                     | Existing scope controls               |
|            **27** | GP21            | DOWN button                   | Existing scope controls               |
|            **28** | GND             | Digital ground                | General ground                        |
|            **29** | GP22            | Frequency-counter input       | Existing GOscillo                     |
|            **30** | RUN             | RP2040 RUN/reset              | Not GPIO                              |
|            **31** | **GP26 / ADC0** | **Scope CH0 / LCR V_total**   | **Shared analog input**               |
|            **32** | **GP27 / ADC1** | **Scope CH1 / LCR V_DUT**     | **Shared analog input**               |
|            **33** | **AGND**        | **Analog measurement ground** | LCR analog reference                  |
|            **34** | GP28 / ADC2     | Unassigned                    | Potential future analog input         |
|            **35** | ADC_VREF        | ADC reference                 | Don't use as ordinary power           |
|            **36** | **3V3(OUT)**    | **3.3-V supply**              | AD9833 + other 3.3-V circuitry        |
|            **37** | 3V3_EN          | 3.3-V regulator enable        | Don't use as GPIO                     |
|            **38** | GND             | Ground                        | General ground                        |
|            **39** | VSYS            | System supply                 | Board power                           |
|            **40** | VBUS            | USB 5 V                       | USB supply                            |


# LCR-specific wiring
```
RP2040 / Pico W                       LCR Hardware
────────────────────────────────────────────────────────

Pin 5    GP3   ──────────────────── AD9833 FSYNC

Pin 9    GP6   / SPI0 SCK ───────── AD9833 SCLK
Pin 10   GP7   / SPI0 TX  ───────── AD9833 SDATA

Pin 31   GP26 / ADC0 ─────────────── V_total
                                      │
AD9833 VOUT ──────────────────────────┤
                                      │
                                   R_SENSE
                                      │
Pin 32   GP27 / ADC1 ─────────────── V_DUT
                                      │
                                     DUT
                                      │
Pin 33   AGND ────────────────────────┘

Pin 36   3V3(OUT) ────────────────── AD9833 VCC

Pin 33   AGND ─────┬──────────────── AD9833 AGND
                   └──────────────── AD9833 DGND
```

# Bus organization
```
SPI0 — LCR
────────────────
GP6  SCK  → AD9833
GP7  TX   → AD9833
GP3  GPIO → FSYNC


SPI1 — Display
────────────────
GP10 SCK  → TFT
GP11 MOSI → TFT
GP12 MISO → TFT
GP13 CS   → TFT


I2C — Touch
────────────────
GP4 SDA → FT6206
GP5 SCL → FT6206
```
