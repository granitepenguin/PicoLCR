# PicoLCR
Web Enabled Pico LCR meter

This project is extending the Oscilloscope project found at https://github.com/siliconvalley4066/RaspberryPiPicoWTFTOscilloscope 

The primary additions are adding LCR functionality to the existing project

Adafruit Libraries being used for Capacitive touch and eventually TFT display:

<img width="281" height="620" alt="578710219-245242b9-7820-4f7b-8994-2059b75028af" src="https://github.com/user-attachments/assets/eb3e4286-bc33-46d4-8d43-bdd95f336685" />

# Things to install:
RP2040 board library: add https://github.com/earlephilhower/arduino-pico/releases/download/global/package_rp2040_index.json link under the Board Manager  
TFT_eSPI library: for screen display  
Adafruit FT6206 Library: For capacitive touch display support  


# Updates to make:
The display is using the TFT_eSPI library. To get the correct pinouts, replace the User_setup files found in the TFT_eSPI library directory.  
