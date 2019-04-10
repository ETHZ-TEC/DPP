# BOLT

v1.27, compiled intel hex files compatible with the MSP430FR5969 MCU


## Precompiled binaries

There are 6 different configurations:  
BOLT_64b_noreset.hex  
BOLT_64b_reset.hex  
BOLT_128b_noreset.hex  
BOLT_128b_reset.hex  
BOLT_256b_noreset.hex  
BOLT_256b_reset.hex  

The different variants have different queue element sizes and behaviours after a reset.
64, 128 or 256 bytes are the options for the size of one element in the BOLT queue. Since the amount of FRAM on BOLT is fixed, a larger element size automatically yields a smaller queue size.
The "noreset" in the name means the BOLT queue is not cleared after an MCU reset and therefore retains the data also in case of a power loss.

In addition, there is also a BOLT deepsleep image, which basically just configures the GPIO pins and then enters LPM4. This image is intended to be used on DPP2 DevBoards in case the BOLT functionality is not needed.


## Flash the image

To upload the hex image to the MCU, connect an MSP-FET debugger and run "./flash.sh [filename]".
You may also need to add the path of the file libmsp430.so to LD_LIBRARY_PATH. This library file is part of several TI packages (e.g. MSP430 Flasher).


## Source code

In case you would like to compile your own binary, the source code of our BOLT implementation is available on Github:
https://github.com/ETHZ-TEC/BOLT
