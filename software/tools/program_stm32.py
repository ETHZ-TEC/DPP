#!/usr/bin/env python3
'''

Flash a binary or hex file onto the STM32L4xx MCU via the bootloader.

This routing only works in conjunction with the DPP2 DevBoard.
You can either:
- manually force the STM32 into the bootloader by pulling COM_GPIO1 high
  (short pins 2 and 7 on the debug header) and press the reset button
- or close the jumpers J502/J503 on the back of the DevBoard and short pin
  7 on the debug header with pin 7 on the COM_JTAG connector.
The script will automatically pick the first available serial port that is
identified as an FTDI Dual RS232 device.


last update: 2018-07-25
author:      rdaforno

'''

import sys
import os.path
import serial
import serial.tools.list_ports
from intelhex import hex2bin
import stm32loader.stm32loader as stm32bl


verify = False
invertBOOT0 = False


def getFirstPort(printPorts):
  ports = [p for p in serial.tools.list_ports.comports() if "Dual RS232" in p[1]]
  if printPorts:
    for p in sorted(ports):
      print("%s" % p)
  if ports is None or len(ports) == 0:
    return None
  return sorted(ports)[1][0]


def programSTM32(fileName, serialPort):
  global verify, invertBOOT0
  loader = stm32bl.Stm32Loader()
  stm32bl.VERBOSITY = 0
  loader.configuration['data_file'] = fileName
  loader.configuration['port'] = serialPort
  loader.configuration['baud'] = 115200
  loader.configuration['parity'] = serial.PARITY_EVEN
  loader.configuration['boot0_active_high'] = invertBOOT0
  try:
    loader.connect()
    if loader.read_device_details() != 0x435:
      print("invalid device ID")
      return
    loader.configuration['erase'] = True
    print("erasing...")
    loader.perform_commands()
    loader.configuration['write'] = True
    print("programming...")
    loader.perform_commands()
    print("programming OK")
    loader.configuration['go_address'] = 0x08000000
    if verify:
      print("verifying...")
      loader.configuration['verify'] = True
      loader.perform_commands()
    print("reset")
    loader.reset()
  finally:
    pass



if __name__ == "__main__":
  if len(sys.argv) < 2:
    print("no filename provided\r\nusage:  ./" + os.path.basename(__file__) + " [filename] [port (optional)]")
    sys.exit()
  fileName = sys.argv[1]
  if not os.path.isfile(fileName):
    print("file '%s' not found" % fileName)
    sys.exit()
  f, ext = os.path.splitext(fileName)
  if "hex" in ext:
    hex2bin(fileName, fileName + ".binary")
    fileName = fileName + ".binary"

  if len(sys.argv) > 2:
    # 2nd argument is supposed to be the serial port
    serialPort = sys.argv[2]
  else:
      serialPort = getFirstPort(False)
      if serialPort is None:
        print("no DPP2 DevBoard found")
        sys.exit()
  print("connecting to serial port %s" % serialPort)
  programSTM32(fileName, serialPort)

