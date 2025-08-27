## Bootloader & Firmware Update - STM32F4

- Various firmwares can be bootloaded onto one single STM32F4 board, performing different kinds of task


## Features

- Bootloading new firmwares by shifting vector table
- Bootloading new firmwares via UART and DMA transmissions
- LEDs control via UART commands
- LEDs concurrent flow algorithm
- Room temperature values reading & reporting using ADC and transmitting via UART
- Gyroscope Sensor that reports which side board is tilted to physically
- Quake Sensor that alerts when board detects high vibration signals


## Hardware Requirements

- An STM32F4-family board (e.g. STM32F411VET6)
- 4 LEDs (optional)


## Usage

Perform independently one of the below bootloading methods:
1. Bootloading by pressing button
2. Firmware updating via DMA and UART protocols (using Hercules software)

Note: if using DMA, rxbuff's size must match exactly size of firmware (.BIN) to be updated onto STM32F4 board


## Video Reel

https://drive.google.com/file/d/1zqdLdZCvbugISHrtnXr1EfANJB-B5JfK/view?usp=drive_link


# Author

Khuong Le