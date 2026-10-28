# Hardware Target

The project now targets the D52Q M6 development module / breadboard setup only.
The former prototype-PCB profile has been retired.

## Dev-Kit Wiring

- Display UART TX: P0.06
- Display UART RX: P0.08
- Brake servo PWM: P0.04 (D52Q M6 J102.05), PWM0 channel 0
- Cadence input: P0.11
- Green status LED / SPI OK: P0.17
- Red status LED / SPI error: P0.13
- Spare red LED: P0.12
- SPI NOR SCK: P0.19
- SPI NOR MOSI / SI / IO0: P0.20
- SPI NOR MISO / SO / IO1: P0.22
- SPI NOR CS: P0.24

Tie the IS25LP080D `/WP` and `/HOLD` pins high to 3V3.

## Brake Servo

The Ciclotte has no resistance sensor; the brake is a hobby servo whose angle
the firmware derives from the GUI level (1..10). Signal: 20 ms period,
0.5 ms (0 deg) to 2.5 ms (180 deg) pulse, 3.3 V logic on P0.04.

Pin choice: P0.04 is free on the Dev-Kit, sits next to the former resistance
input P0.03 on J102, and is not an NFC, crystal or reset pin. P0.25-P0.30 are
avoided because the M6 development module routes them to its accelerometer.
The board default for PWM0 is P0.17, which is the green status LED here, so
the overlays reassign it.

Power the servo from its own 5 V supply with a common ground; check that the
servo accepts a 3.3 V signal level, otherwise add a level shifter.

## Build

The root `app.overlay` and `prj.conf` describe this Dev-Kit target directly.
A normal Zephyr/NCS application build therefore uses the Dev-Kit + LED + SPI
configuration without selecting an extra hardware profile.

The helper script remains for explicit build directories:

```powershell
.\tools\build-profile.ps1 -Target app -Pristine
.\tools\build-profile.ps1 -Target dfu -Pristine
```
