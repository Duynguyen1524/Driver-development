# STM32 Driver Development

This is my first low-level driver development project for the STM32 Nucleo Board. I've completed custom GPIO and SPI drivers from scratch using direct register access. Currently developing I2C and UART drivers.


## 🔧 Features

- ✅ GPIO driver (input/output mode, pull-up/down config, alternate function)
- ✅ SPI driver (full-duplex master mode)
- ✅ I2C driver (master transmitter/receiver mode with ACK control)
- ✅ UART driver (basic TX/RX functionality with polling)
## 🛠️ Development Progress

| Driver | Status   | Notes                          |
|--------|----------|--------------------------------|
| GPIO   | ✅ Done   | Supports input/output, AF, pull-up/down |
| SPI    | ✅ Done   | Master mode, full-duplex, basic config |
| I2C    | ✅ Done | Master mode, slave mode, interrupt |
| UART   | 🔄 In Progress | TX/RX polling under development |
