# BMP388 STM32 API

## Introduction

This libraries include API for STM32 MCU family to communicate with BMP388 sensors.

Package include bmp388.h, bmp388.c and bmp388_regs.h

## Integration with project

- Copy bmp388.h, bmp388.c and bmp388_regs.h to your project
- Include bmp388.h in your project

```c
#include "bmp388.h"
```

## File information

- bmp388_regs.h - This header file contains registers addresses, structs type declarations and control macros.
- bmp388.h - This header file contains declarations of API's functions.
- bmp388.c - This header file contains definitions of API's functions.

# Supported interfaces

This library supports only I2C communication.

## Usage guide

## Initialization

To initialize sensor use function BMP_init(). Provide proper address of active I2C handler.

## Credits

Special thanks to Mateusz Salamon. This library is based on BMXX80_STM32_HAL shared on MIT License.