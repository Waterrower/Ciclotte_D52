# Dev-Kit SPI-NOR

This is the only active hardware target.

## Signals

- Display UART TX: P0.06
- Display UART RX: P0.08
- Brake servo PWM: P0.04 (PWM0 channel 0)
- Cadence input: P0.11
- Green status LED / SPI OK: P0.17
- Red status LED / SPI error: P0.13
- Spare red LED: P0.12
- SPI NOR SCK: P0.19
- SPI NOR MOSI / SI / IO0: P0.20
- SPI NOR MISO / SO / IO1: P0.22
- SPI NOR CS: P0.24

Tie `/WP` and `/HOLD` to 3V3. The IS25LP080D is configured as a
`jedec,spi-nor` device with minimal fixed geometry and 1 MHz SPI for bring-up.
