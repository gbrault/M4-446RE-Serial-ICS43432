# M4-446RE-Serial-ICS43432
Nucleo F446RE I2S ICS 43432 microphone to UART

The Nucleo Board is linked to an Invensense ICS 43432 I2S microphone (24bit mono)
- Data is acquired through DMA
- Converted into mono PCM 24 bit
- pushed to UART in binary format

#Work in Progress
- adding an ADPCM codec to improve throughput

#Linked Repository
- see the Plotter Repository
- Reads data from serial and plot using qcustomplot (v1.3.2)

#Project build
- use STM32CubeMX (should be upgradeable with it)
- compatible with AC6 Workbench (SW4STM32 Toolchain)

#CubeMX Parameters
- you can use STM32CubeMX to see the configuration
- I2S: I2S2 enabled (Half-Duplex Master)
  - Master Receive, MSB First, 24Bits Data on 32 Bits Frame, 32KHz, Clock Low
- Usart: Usart2 enabled
  - 230400 B/s, 8bits, No Parity, 1 Stop, Direction: receive and Transmit
- DMA: SPI2_RX DMA1 Stream 3, data width Half Word
- Code is formatted to reuse CubeMX to repurpose the design
