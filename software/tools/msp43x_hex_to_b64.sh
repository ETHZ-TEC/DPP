#!/bin/bash
#
# converts an MSP430/MSP432 Intel hex file into a base64 file (required for FlockLab)

if [ $# -lt 1 ];
then
   echo "no filename provided"
   exit 1
fi

HEXFILE=$1
if [ ! ${HEXFILE: -3} == "hex" ] && [ ! ${HEXFILE: -3} == "HEX" ];
then
  echo "not a hex file"
  exit 1
fi

# Create ELF file from CCS Intel-HEX output
msp430-objcopy -F elf32-msp430 $HEXFILE ${HEXFILE}.elf
base64 ${HEXFILE}.elf > ${HEXFILE}.elf.b64

echo "file" ${HEXFILE}.elf.b64 "created"

exit 0
