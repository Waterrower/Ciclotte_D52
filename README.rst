NOHRD Ciclotte Controller
#########################

Zephyr/Nordic nRF52 firmware for the NOHRD Ciclotte display controller.

Overview
********

Derived from the NOHRD bike controller; corresponds to ``BIKE`` + ``CICLOTTE``
in the original AVR firmware. This application drives the serial display UI,
reads the cadence input, sets the brake via a servo from the GUI +/- buttons,
exposes BLE FTMS indoor bike data, and can connect to BLE heart-rate sensors.

Hardware
********

The current overlay targets ``nrf52dk/nrf52832`` with:

- UART0 TX on P0.06 and RX on P0.08 for the display.
- Cadence input on P0.11.
- Brake servo PWM (20 ms period, 0.5-2.5 ms pulse) on P0.04.
- Debug/display-RX LED on P0.17.

Building
********

Build and flash with the nRF Connect SDK tools for ``nrf52dk/nrf52832``.
