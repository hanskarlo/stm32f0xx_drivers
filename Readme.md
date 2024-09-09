# STM32F0xx HAL Library Repository

STM32F0xx peripheral **Hardware Abstraction Layer** (HAL) library used and initialized as a git submodule for other projects.

<br>

## Available Peripherals
- GPIO
- Timers
- I2C
- USART
- SPI
- CRC

## ToDo
- CAN
- DMA support

## Installation
The following provides instructions on how to incorporate the STM32F0xx HAL library into your STM Cube project.

### Standalone
Clone this repository in your STM project directory.
```bash
cd your-project-dir
git clone https://github.com/hanskarlo/stm32f0xx_drivers.git
```

<br>

Add /Inc to the compiler include path:
```
Project Properties
    -> C/C++ General
        -> Paths and Symbols
            -> Includes tab
                -> Add...
                    -> Workspace...
                        -> Select stm32f0xx_drivers/Inc
```

<br>

Add /Src to the Source Location(s):
```
Project Properties
    -> C/C++ General
        -> Paths and Symbols
            -> Source Location
                -> Add Folder...
                    -> Select stm32f0xx_drivers/Src
```


### Git submodule (Recommended)
Incorporating this repository as a git submodule in your STM projects allows you to more easily keep this code up to date and/or maintain stable versions.

<br>

## Usage

Wherever using the HAL in any project source C/C++ file, `#include` the header file:
```c
#include "stm32f0xx.h"
```

<br>

> :exclamation: If using HAL library in C++ project, include the header with `extern "C"`:
>```cpp
>// In main.cpp or an source cpp file
>
>extern "C"{
>    #include "stm32f0xx.h"
>}
>```
>


<br>


## Examples
The `examples/` directory contains usage examples for all peripherals in different configurations and common scenarios (TODO).