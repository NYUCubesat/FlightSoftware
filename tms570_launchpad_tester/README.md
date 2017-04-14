# Test firmware for XL2-TMS57012

## Overview

This firmware is a basic "hello world" working on a TI Launchpad eval 
board that contains a TMS5701224PGE microcontroller. This chip is a 
big-endian chip, which necessitates adding the necessary multilib 
support when compiling GCC from source. Two versions of the requisite 
toolchain are included in the upstream toolchain (within Athena).

## Loading code via XDS110 Debug Interface

The Makefile in this project calls TI's "Uniflash" tool, which is a 
configurable and downloadable CLI utility from http://dev.ti.com. This 
directory also contains the necessary XML target descriptor file 
required for Uniflash to connect to the board and TMS570 chip. Once 
downloaded and extracted to some directory on the system, the user must 
set the UNIFLASH_PATH variable in this project's Makefile. Invocation of 
the necessary target is made by calling:

```bash
make xds
```

## JTAG

If preferred, JTAG may be used. The Makefile supports use of a Segger 
J-Link debugger. The user must download and extract the J-Link software 
and driver package from Segger's website, and set the JLINK_PATH 
variable in the Makefile. This target is invoked via:

```bash
make jtag
```
