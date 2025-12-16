# NXP Application Code Hub
[<img src="https://mcuxpresso.nxp.com/static/icon/nxp-logo-color.svg" width="100"/>](https://www.nxp.com)

## Remote IO Platform: ECAT Sample Application
This application showcases the analog and digital I/O features supported by the Remote IO Platform in combination with the EtherCAT industrial protocol. The platform communicates over ethernet with a TwinCAT-driven soft-PLC application running on a PC.<br />

### The features include:<br />
<ul>
    <li>Signal generator</li>
    <li>MCU Digital Input pin control</li>
    <li>MCU Digital Output pin control</li>
    <li>AFE Digital I/Os control</li>
    <li>External Voltage Signal Measurement (HVSIG)</li>
    <li>Current Measurement</li>
    <li>Temperature Measurement</li>
    <li>Internal Voltage References measurement (LVSIG)</li>
    <li>Voltage Calibration</li>
    <li>Resistance Calibration</li>
</ul><br />

This README contains simplified information. Please, refer to the <a href="https://www.nxp.com/RIOP">User Guide</a> for comprehensive instructions on what tools to download, how to configure the setup and run the application.

#### Boards: Custom Board
#### Categories: Sensor, Industrial, RTOS, Analog Front End
#### Peripherals: CAN, FLASH, PWM, SPI, UART, PINCTRL, TIMER, CLOCKS, DMA, ETHERNET, GPIO, ADC
#### Toolchains: MCUXpresso IDE, VS Code

## Table of Contents
1. [Software](#step1)
2. [Hardware](#step2)
3. [Setup](#step3)
4. [Support](#step4)
5. [Release Notes](#step5)
5. [Known Issues](#step6)

## 1. Software<a name="step1"></a>
The application supports VS Code with the MCUXpresso for VS Code extension version 25.03 or newer.

### IDE Download:
<ul>
    <li><a href="https://code.visualstudio.com/">VS Code IDE</a></li>
    <li><a href="https://marketplace.visualstudio.com/items?itemName=NXPSemiconductors.mcuxpresso">MCUXpresso for VS Code extension 25.03+</a></li>
</ul>

Additional SW tools are needed to build, flash and use the application. These are the SEC Tool version 25.09 or newer and FreeMASTER 3.2 or newer and TwinCAT tool from Beckhoff.

### Tools Download:
<ul>
    <li><a href="https://www.nxp.com/design/design-center/software/development-software/mcuxpresso-software-and-tools-/mcuxpresso-secure-provisioning-tool:MCUXPRESSO-SECURE-PROVISIONING">Secure Provisioning Tool v25.09+</a></li>
    <li><a href="https://www.nxp.com/design/design-center/software/development-software/freemaster-run-time-debugging-tool:FREEMASTER">FreeMASTER 3.2+</a></li>
    <li><a href="https://www.beckhoff.com/en-gb/support/download-finder/search-result/?c-1=26782567">TwinCAT 3</a></li>
</ul>

MCUxpresso SDK MIMXRT1189xxxxx 25.09.00 is required to enable project importing to VS Code.

### SDK Download:
<ul>
    <li>When using VS Code, download the MCUX 25.09.00 repository using Import Repository feature in the MCUXpresso for VS Code extension</li>
</ul>

## 2. Hardware<a name="step2"></a>
### The contents of the Remote IO Platform package include:
<ul>
    <li>Assembled and tested evaluation board in an antistatic bag</li>
    <li>USB-micro cable</li>
    <li>24 V Power adapter</li>
    <li>3 colored wires</li>
</ul>

### The application requires additional hardware components:
<b>Mandatory:</b>
<ul>
    <li>PC with Windows 10 or newer</li>
    <li>Ethernet cable</li>
</ul>

<b>Optional, based on use-cases:</b>
<ul>
    <li>Shunt resistor (100 Ω, 125 Ω, or 250 Ω)</li>
    <li>PT100 temperature sensor</li>
    <li>(Wire-wound) Resistor (220 Ω)</li>
    <li>Current or voltage power supply with current limitation</li>
</ul>

## 3. Setup<a name="step3"></a>
<b>This README contains simplified information. Please, refer to the <a href="">User Guide</a> for comprehensive instructions on what tools to download, how to configure the setup and run the application.</b>

## 4. Support<a name="step4"></a>
#### Project Metadata

<!----- Boards ----->
[![Board badge](https://img.shields.io/badge/Board-REMOTE&ndash;IO&ndash;PLATFORM&ndash;RIOP-blue)](https://www.nxp.com/riop)

<!----- Categories ----->
[![Category badge](https://img.shields.io/badge/Category-SENSOR-yellowgreen)](https://mcuxpresso.nxp.com/appcodehub?category=sensor)
[![Category badge](https://img.shields.io/badge/Category-INDUSTRIAL-yellowgreen)](https://mcuxpresso.nxp.com/appcodehub?category=industrial)
[![Category badge](https://img.shields.io/badge/Category-RTOS-yellowgreen)](https://mcuxpresso.nxp.com/appcodehub?category=rtos)
[![Category badge](https://img.shields.io/badge/Category-ANALOG%20FRONT%20END-yellowgreen)](https://mcuxpresso.nxp.com/appcodehub?category=analog_front_end)

<!----- Peripherals ----->
[![Peripheral badge](https://img.shields.io/badge/Peripheral-CAN-yellow)](https://mcuxpresso.nxp.com/appcodehub?peripheral=can)
[![Peripheral badge](https://img.shields.io/badge/Peripheral-FLASH-yellow)](https://mcuxpresso.nxp.com/appcodehub?peripheral=flash)
[![Peripheral badge](https://img.shields.io/badge/Peripheral-PWM-yellow)](https://mcuxpresso.nxp.com/appcodehub?peripheral=pwm)
[![Peripheral badge](https://img.shields.io/badge/Peripheral-SPI-yellow)](https://mcuxpresso.nxp.com/appcodehub?peripheral=spi)
[![Peripheral badge](https://img.shields.io/badge/Peripheral-UART-yellow)](https://mcuxpresso.nxp.com/appcodehub?peripheral=uart)
[![Peripheral badge](https://img.shields.io/badge/Peripheral-PINCTRL-yellow)](https://mcuxpresso.nxp.com/appcodehub?peripheral=pinctrl)
[![Peripheral badge](https://img.shields.io/badge/Peripheral-TIMER-yellow)](https://mcuxpresso.nxp.com/appcodehub?peripheral=timer)
[![Peripheral badge](https://img.shields.io/badge/Peripheral-CLOCKS-yellow)](https://mcuxpresso.nxp.com/appcodehub?peripheral=clocks)
[![Peripheral badge](https://img.shields.io/badge/Peripheral-DMA-yellow)](https://mcuxpresso.nxp.com/appcodehub?peripheral=dma)
[![Peripheral badge](https://img.shields.io/badge/Peripheral-ETHERNET-yellow)](https://mcuxpresso.nxp.com/appcodehub?peripheral=ethernet)
[![Peripheral badge](https://img.shields.io/badge/Peripheral-GPIO-yellow)](https://mcuxpresso.nxp.com/appcodehub?peripheral=gpio)
[![Peripheral badge](https://img.shields.io/badge/Peripheral-ADC-yellow)](https://mcuxpresso.nxp.com/appcodehub?peripheral=adc)

<!----- Toolchains ----->
[![Toolchain badge](https://img.shields.io/badge/Toolchain-MCUXPRESSO%20IDE-orange)](https://mcuxpresso.nxp.com/appcodehub?toolchain=mcux)
[![Toolchain badge](https://img.shields.io/badge/Toolchain-VS%20CODE-orange)](https://mcuxpresso.nxp.com/appcodehub?toolchain=vscode)

Questions regarding the content/correctness of this example can be entered as Issues within this GitHub repository.

>**Warning**: For more general technical questions regarding NXP Microcontrollers and the difference in expected functionality, enter your questions on the [NXP Community Forum](https://community.nxp.com/)

[![Follow us on Youtube](https://img.shields.io/badge/Youtube-Follow%20us%20on%20Youtube-red.svg)](https://www.youtube.com/NXP_Semiconductors)
[![Follow us on LinkedIn](https://img.shields.io/badge/LinkedIn-Follow%20us%20on%20LinkedIn-blue.svg)](https://www.linkedin.com/company/nxp-semiconductors)
[![Follow us on Facebook](https://img.shields.io/badge/Facebook-Follow%20us%20on%20Facebook-blue.svg)](https://www.facebook.com/nxpsemi/)
[![Follow us on Twitter](https://img.shields.io/badge/X-Follow%20us%20on%20X-black.svg)](https://x.com/NXP)

## 5. Release Notes<a name="step5"></a>
| Version | Description / Update                           | Date                        |
|:-------:|------------------------------------------------|----------------------------:|
| 1.0.0   | Initial release on Application Code Hub        | June 15<sup>th</sup> 2025   |
| 1.5.0   | SDK 25.09.00 based release with GOAL           | Dec 18<sup>th</sup> 2025    |

## 6. Known Issues<a name="step6"></a>

<ul>
    <li>There are known issues with TwinCAT on Windows 11. For more information and support, please reach out to Beckhoff.</li>
    <li>The AFE digital I/O pins are floating and can toggle between TRUE and FALSE if a wire is connected to one of them and left floating on the other end.</li>
    <li>The AFE digital input pins change from TRUE to FALSE with a latency of about 3 seconds.</li>
</ul>

## Licensing

Copyright NXP 2025 <br />
LA_OPT_Online Code Hosting NXP_Software_License - v1.4 May 2025

## Origin
<ul>
    <li>NXP (Proprietary)</li>
    <li>[NAFE13388 Drivers - NXP (BSD-3-Clause) - riop_ECAT/M7FOLLOWER/source/NAFE_hw](BSD 3-Clause.txt)</li>
</ul>
