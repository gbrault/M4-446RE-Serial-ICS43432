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
- Reads data from serial an plot it with qcustomplot (v1.3.2)
