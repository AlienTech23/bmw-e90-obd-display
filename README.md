# BMW E90 Round OBD Display

An auxiliary cupholder-mounted round telemetry display for a BMW E90 328
(also tested on a 2014 Toyota Venza). An ESP32 reads live OBD-II data over a
wired UART link and renders an M-inspired gauge UI on a 1.28" GC9A01 round TFT.
A B10K potentiometer "mode dial" cycles through the screens.

## Screens
Coolant · Intake air temp · Voltage · Estimated HP · Engine load
(Speed and RPM are intentionally omitted - the factory cluster already shows them.)

## Hardware
- ESP32 dev board (ELEGOO WROOM-32, USB-C)
- SparkFun OBD-II UART board (STN1110 / ELM327 command set)
- GC9A01 1.28" round TFT (240x240, SPI, 3.3V)
- B10K linear potentiometer (mode dial)
- 1k + 2k resistor divider on the OBD RX line (5V -> 3.3V)
- OBD-II to DB9 cable; 12V->USB-C buck converter for in-car power

## Wiring
Display: VCC->3V3  GND->GND  SCL->GPIO18  SDA->GPIO23  RST->GPIO4  DC->GPIO2  CS->GPIO5
Pot:     pin1->3V3  wiper->GPIO34  pin3->GND
OBD:     TX-O -> 1k -> node -> GPIO16 ; node -> 2k -> GND ; GPIO17 -> RX-I ; GND common
OBD protocol is forced to ISO 15765-4 CAN (protocol '6').

## Build (PlatformIO / VS Code)
1. Install VS Code + the **PlatformIO IDE** extension.
2. Open this folder. PlatformIO installs the toolchain and libraries automatically.
3. Build (checkmark) and Upload (arrow) from the PlatformIO toolbar.

**Flashing note:** unplug the `GPIO17 -> RX-I` wire while uploading, then reconnect it.

## Libraries (auto-installed via platformio.ini)
- Adafruit GFX Library
- Adafruit GC9A01A
- ELMduino (PowerBroker2)
