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
from stm32loader.main import Stm32Loader


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
  loader = Stm32Loader()
  loader.verbosity = 0
  loader.configuration['data_file'] = fileName
  loader.configuration['port'] = serialPort
  loader.configuration['baud'] = 115200
  loader.configuration['parity'] = serial.PARITY_EVEN
  loader.configuration['boot0_active_high'] = invertBOOT0
  loader.configuration['family'] = 'L4'
  loader.configuration['erase'] = True
  loader.configuration['write'] = True
  loader.configuration['verify'] = True
  loader.configuration['go_address'] = 0x08000000
  loader.configuration['hide_progress_bar'] = True
  loader.connect()
  loader.read_device_id()
  if loader.stm32.get_id() != 0x435:
    print("invalid device ID")
  loader.perform_commands()
  loader.reset()


def convertFile(fileName):
  f, ext = os.path.splitext(fileName)
  if ext.lower() in ('.elf', '.exe', '.out'):
    # 3rd argument (if provided) is the node ID -> can only be embedded into elf file
    if len(sys.argv) > 3:
      try:
        nodeID = int(sys.argv[3])
        # use tos-set-symbols to set the node ID and convert to Intel hex
        if os.system("tos-set-symbols --objcopy objcopy --objdump objdump --target ihex %s %s.hex FLOCKLAB_NODE_ID=%d" % (fileName, fileName, nodeID)) != 0:
          print("failed to set node ID (tos-set-symbols missing?)")
        else:
          print("node ID set to %d" % nodeID)
      except:
        print("invalid node ID %s" % sys.argv[3])
        sys.exit(1)
    else:
      # convert to Intel hex
      os.system("objcopy -O ihex %s %s.hex" % (fileName, fileName))
      print("file converted to Intel hex format")
    fileName = fileName + ".hex"
    ext = ".hex"
  if ext.lower() in ('.hex', '.ihex'):
    hex2bin(fileName, fileName + ".binary")
    fileName = fileName + ".binary"
    print("file converted to binary format")
  return fileName


if __name__ == "__main__":

  if len(sys.argv) < 2:
    print("no filename provided\r\nusage:  ./" + os.path.basename(__file__) + " [filename] [port (optional)]")
    sys.exit(1)

  fileName = sys.argv[1]
  if not os.path.isfile(fileName):
    print("file '%s' not found" % fileName)
    sys.exit(1)

  fileName = convertFile(fileName)

  if len(sys.argv) > 2:
    # 2nd argument is supposed to be the serial port
    serialPort = sys.argv[2]
  else:
      serialPort = getFirstPort(False)
      if serialPort is None:
        print("no DPP2 DevBoard found")
        sys.exit(1)

  print("connecting to serial port %s" % serialPort)
  programSTM32(fileName, serialPort)

