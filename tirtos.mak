#
#  ======== tirtos.mak ========
#

#
# Set location of various cgtools
# These variables can be set here or on the command line.
#
CCS_BUILD ?= true
CODEGEN_INSTALLATION_DIR      := c:/ti/ccsv5/tools/compiler

#
# Enable TI-RTOS to build for IAR.
# Set IAR_ENABLE to true and modify path to toolchain
#
IAR_BUILD ?= false
IAR_COMPILER_INSTALLATION_DIR ?= C:/Program Files (x86)/IAR Systems/Embedded Workbench 6.5
IAR_MSP430HEADERS ?= ${IAR_COMPILER_INSTALLATION_DIR}/inc

#
# Enable TI-RTOS to build for GCC.
# Set IAR_ENABLE to true and modify path to toolchain
#
GCC_BUILD ?= true
GCC_INSTALLATION_DIR := c:/ti/ccsv5/tools/compiler/gcc-arm-none-eabi-4_7-2012q4

ti.targets.C28_float ?= $(CODEGEN_INSTALLATION_DIR)/c2000_6.2.0
ti.targets.arm.elf.M3 ?= $(CODEGEN_INSTALLATION_DIR)/arm_5.1.1
ti.targets.arm.elf.M4F ?= $(CODEGEN_INSTALLATION_DIR)/arm_5.1.1
ti.targets.msp430.elf.MSP430X ?= $(CODEGEN_INSTALLATION_DIR)/msp430_4.2.1

iar.targets.msp430.MSP430X_small ?= $(IAR_COMPILER_INSTALLATION_DIR)/430
iar.targets.arm.M3 ?= $(IAR_COMPILER_INSTALLATION_DIR)/arm
iar.targets.arm.M4F ?= $(IAR_COMPILER_INSTALLATION_DIR)/arm

gnu.targets.arm.M3 ?= $(GCC_INSTALLATION_DIR)
gnu.targets.arm.M4F ?= $(GCC_INSTALLATION_DIR)

TIRTOS_INSTALLATION_DIR := c:/ti/tirtos_1_21_00_09
XDCTOOLS_INSTALLATION_DIR ?= c:/ti/xdctools_3_25_04_88
BIOS_INSTALLATION_DIR  ?= $(TIRTOS_INSTALLATION_DIR)/products/bios_6_37_00_20
IPC_INSTALLATION_DIR   ?= $(TIRTOS_INSTALLATION_DIR)/products/ipc_3_10_01_11
UIA_INSTALLATION_DIR   ?= $(TIRTOS_INSTALLATION_DIR)/products/uia_1_04_00_06
NDK_INSTALLATION_DIR   ?= $(TIRTOS_INSTALLATION_DIR)/products/ndk_2_23_01_01
MWARE_INSTALLATION_DIR ?= $(TIRTOS_INSTALLATION_DIR)/products/MWare_v201b/MWare
TIVAWARE_INSTALLATION_DIR ?= $(TIRTOS_INSTALLATION_DIR)/products/TivaWare_C_Series-2.0.1.11577a
MSP430WARE_INSTALLATION_DIR ?= $(TIRTOS_INSTALLATION_DIR)/products/MSP430Ware_1_60_02_09a
MSP430HEADERS          ?= $(CODEGEN_INSTALLATION_DIR)/../../ccs_base/msp430/include

#
# To build TI-RTOS driver libraries for other MSP430 devices; simply append the
# device names to MSP430DEVLIST (separated by whitepsaces)
# MSP430DEVLIST := \
#     MSP430F5529 \
#     MSP430F5527 \
#     MSP430F6459 \
#     etc...
#
MSP430DEVLIST := MSP430F5529

# Setting this variable to 1 causes only NDK base stack libraries to be built
BUILDMINSTACK_CONFIG     := BUILDMINSTACK=1

#
# Set XDCARGS to some of the variables above.  XDCARGS are passed
# to the XDC build engine... which will load tirtos.bld... which will
# extract these variables and use them to determine what to build and which
# toolchains to use.
#
# Note that not all of these variables need to be set to something valid.
# Unfortunately, since these vars are unconditionally assigned, your build line
# will be longer and more noisy than necessary (e.g., it will include CC_V5T
# assignment even if you're just building for C64P).
#
# Some background is here:
#     http://rtsc.eclipse.org/docs-tip/Command_-_xdc#Environment_Variables
#
XDCARGS= \
    MWareDir='$(MWARE_INSTALLATION_DIR)' \
    TivaWareDir='$(TIVAWARE_INSTALLATION_DIR)' \
    MSP430WareDir='$(MSP430WARE_INSTALLATION_DIR)' \
    MSP430DEVLIST='$(MSP430DEVLIST)'

ifeq ("$(CCS_BUILD)", "true")
    XDCARGS += \
       ti.targets.C28_float='$(ti.targets.C28_float)' \
       ti.targets.arm.elf.M3='$(ti.targets.arm.elf.M3)' \
       ti.targets.arm.elf.M4F='$(ti.targets.arm.elf.M4F)' \
       ti.targets.msp430.elf.MSP430X='$(ti.targets.msp430.elf.MSP430X)' \
       MSP430HEADERS='$(MSP430HEADERS)'
       allmsp430ware += msp430ware
endif
ifeq ("$(IAR_BUILD)", "true")
	XDCARGS += \
	iar.targets.arm.M3='$(iar.targets.arm.M3)'\
	iar.targets.arm.M4F='$(iar.targets.arm.M4F)'\
	iar.targets.msp430.MSP430X_small='$(iar.targets.msp430.MSP430X_small)'
	allmsp430ware += iar-msp430ware
endif
ifeq ("$(GCC_BUILD)", "true")
	XDCARGS += \
	gnu.targets.arm.M3='$(GCC_INSTALLATION_DIR)'\
	gnu.targets.arm.M4F='$(GCC_INSTALLATION_DIR)'
endif

export XDCARGS
#
# Set XDCPATH to contain necessary repositories.
#
XDCPATH = $(TIRTOS_INSTALLATION_DIR)/packages;$(UIA_INSTALLATION_DIR)/packages;$(NDK_INSTALLATION_DIR)/packages;$(IPC_INSTALLATION_DIR)/packages;$(BIOS_INSTALLATION_DIR)/packages;$(MWARE_INSTALLATION_DIR);$(TIVAWARE_INSTALLATION_DIR)
export XDCPATH

#
# Set XDCOPTIONS.  Use -v for a verbose build.
#
#XDCOPTIONS=v
export XDCOPTIONS

#
# Set XDC executable command
# Note that XDCBUILDCFG points to the tirtos.bld file which uses
# the arguments specified by XDCARGS
#
XDC = $(XDCTOOLS_INSTALLATION_DIR)/xdc XDCARGS="$(XDCARGS)" XDCBUILDCFG=./tirtos.bld
XS =  $(XDCTOOLS_INSTALLATION_DIR)/xs

######################################################
## Shouldnt have to modify anything below this line ##
######################################################

help:
	@ echo Makefile to build components within TI-RTOS
	@ echo   goal              description
	@ echo  -----------------------------------------------------------------------------
	@ echo   all               Builds SYS/BIOS, IPC, NDK, UIA, and TI-RTOS drivers
	@ echo   drivers           Builds TI-RTOS drivers and other components in /packages/ti
	@ echo   examples          Builds TI-RTOS examples
	@ echo   bios              Builds SYS/BIOS
	@ echo   ipc               Builds IPC
	@ echo   ndk               Builds NDK
	@ echo   uia               Builds UIA
	@ echo   msp430ware        Builds MSP430Ware CCS libraries
	@ echo   iar-msp430ware    Builds MSP430Ware IAR libraries
	@ echo   clean             Cleans SYS/BIOS, IPC, NDK, UIA, and TI-RTOS drivers
	@ echo   clean-drivers     Cleans TI-RTOS drivers and other components in /packages
	@ echo   clean-bios        Cleans SYS/BIOS
	@ echo   clean-ipc         Cleans IPC
	@ echo   clean-ndk         Cleans NDK
	@ echo   clean-uia         Cleans UIA
	@ echo   clean-msp430ware  Cleans MSP430Ware libraries
	@ echo   examplesgen       Generates TI-RTOS examples and makefiles for IAR toolchain
	@ echo   help              Displays this description

all: bios ipc ndk uia drivers

clean: clean-bios clean-ipc clean-ndk clean-uia clean-drivers

drivers: $(allmsp430ware)
	@ echo building ti.* packages ...
	@ $(XDC) -PR ./packages/ti

clean-drivers: clean-msp430ware
	@ echo cleaning ti.* packages ...
	@ $(XDC) clean -PR ./packages/ti

examples: $(allmsp430ware)
	@ echo building examples packages ...
	@ $(XDC) -PR ./packages/examples

clean-examples: clean-msp430ware
	@ echo cleaning examples packages ...
	@ $(XDC) clean -PR ./packages/examples

bios:
	@ echo building bios ...
	@ $(XDCTOOLS_INSTALLATION_DIR)/gmake -f $(BIOS_INSTALLATION_DIR)/bios.mak \
	  XDC_INSTALL_DIR=$(XDCTOOLS_INSTALLATION_DIR) \
	  $(XDCARGS) -C $(BIOS_INSTALLATION_DIR)

clean-bios:
	@ echo cleaning bios ...
	@ $(XDCTOOLS_INSTALLATION_DIR)/gmake -f $(BIOS_INSTALLATION_DIR)/bios.mak \
	  XDC_INSTALL_DIR=$(XDCTOOLS_INSTALLATION_DIR) -C $(BIOS_INSTALLATION_DIR) clean

ipc:
	@ echo building ipc ...
	@ $(XDCTOOLS_INSTALLATION_DIR)/gmake -f $(IPC_INSTALLATION_DIR)/ipc-bios.mak \
	  XDC_INSTALL_DIR=$(XDCTOOLS_INSTALLATION_DIR) \
	  BIOS_INSTALL_DIR=$(BIOS_INSTALLATION_DIR) \
	  $(XDCARGS) -C $(IPC_INSTALLATION_DIR)

clean-ipc:
	@ echo cleaning ipc ...
	@ $(XDCTOOLS_INSTALLATION_DIR)/gmake -f $(IPC_INSTALLATION_DIR)/ipc-bios.mak \
	  XDC_INSTALL_DIR=$(XDCTOOLS_INSTALLATION_DIR) -C $(IPC_INSTALLATION_DIR) clean

ndk:
	@ echo building ndk ...
	@ $(XDCTOOLS_INSTALLATION_DIR)/gmake -f $(NDK_INSTALLATION_DIR)/ndk.mak \
	  XDC_INSTALL_DIR=$(XDCTOOLS_INSTALLATION_DIR) \
	  SYSBIOS_INSTALL_DIR=$(BIOS_INSTALLATION_DIR) \
	  $(BUILDMINSTACK_CONFIG) \
	  $(XDCARGS) -C $(NDK_INSTALLATION_DIR)

clean-ndk:
	@ echo cleaning ndk ...
	@ $(XDCTOOLS_INSTALLATION_DIR)/gmake -f $(NDK_INSTALLATION_DIR)/ndk.mak \
	  XDC_INSTALL_DIR=$(XDCTOOLS_INSTALLATION_DIR) -C $(NDK_INSTALLATION_DIR) clean

uia:
	@ echo building uia ...
	@ $(XDCTOOLS_INSTALLATION_DIR)/gmake -f $(UIA_INSTALLATION_DIR)/uia.mak \
	  XDC_INSTALL_DIR=$(XDCTOOLS_INSTALLATION_DIR) \
	  BIOS_INSTALL_DIR=$(BIOS_INSTALLATION_DIR) \
	  IPC_INSTALL_DIR=$(IPC_INSTALLATION_DIR) \
	  NDK_INSTALL_DIR=$(NDK_INSTALLATION_DIR) \
	  $(XDCARGS) -C $(UIA_INSTALLATION_DIR)

clean-uia:
	@ echo cleaning uia ...
	@ $(XDCTOOLS_INSTALLATION_DIR)/gmake -f $(UIA_INSTALLATION_DIR)/uia.mak \
	  XDC_INSTALL_DIR=$(XDCTOOLS_INSTALLATION_DIR) -C $(UIA_INSTALLATION_DIR) clean

msp430ware:
	@ $(MAKE) -C $(MSP430WARE_INSTALLATION_DIR) PARTS='$(MSP430DEVLIST)' IPATH='$(MSP430HEADERS)' COMPILER=ccs TOOLPATH='$(ti.targets.msp430.elf.MSP430X)'

iar-msp430ware:
	@ $(MAKE) -C $(MSP430WARE_INSTALLATION_DIR) PARTS='$(MSP430DEVLIST)' IPATH='$(IAR_MSP430HEADERS)' COMPILER=iar TOOLPATH='$(iar.targets.msp430.MSP430X_small)'

clean-msp430ware:
	@ $(MAKE) -C $(MSP430WARE_INSTALLATION_DIR) clean

examplesgen:
ifneq ("$(DEST)","")
	@ echo generating examples in $(DEST) ...
	@ $(XS) examples.examplesgen --tirtos "$(TIRTOS_INSTALLATION_DIR)" --toolchain IAR --output "$(DEST)" --xdctools="$(XDCTOOLS_INSTALLATION_DIR)" --bios="$(BIOS_INSTALLATION_DIR)" --uia="$(UIA_INSTALLATION_DIR)" --ndk="$(NDK_INSTALLATION_DIR)" --tivaware="$(TIVAWARE_INSTALLATION_DIR)" --mware="$(MWARE_INSTALLATION_DIR)" --msp430ware="$(MSP430WARE_INSTALLATION_DIR)" --codegendir="$(IAR_COMPILER_INSTALLATION_DIR)"
	@ $(XS) examples.examplesgen --tirtos "$(TIRTOS_INSTALLATION_DIR)" --toolchain GNU --output "$(DEST)" --xdctools="$(XDCTOOLS_INSTALLATION_DIR)" --bios="$(BIOS_INSTALLATION_DIR)" --uia="$(UIA_INSTALLATION_DIR)" --ndk="$(NDK_INSTALLATION_DIR)" --tivaware="$(TIVAWARE_INSTALLATION_DIR)" --mware="$(MWARE_INSTALLATION_DIR)" --msp430ware="$(MSP430WARE_INSTALLATION_DIR)" --codegendir="$(GCC_INSTALLATION_DIR)"
	@ $(XS) examples.examplesgen --tirtos "$(TIRTOS_INSTALLATION_DIR)" --toolchain TI --output "$(DEST)" --xdctools="$(XDCTOOLS_INSTALLATION_DIR)" --bios="$(BIOS_INSTALLATION_DIR)" --uia="$(UIA_INSTALLATION_DIR)" --ndk="$(NDK_INSTALLATION_DIR)" --tivaware="$(TIVAWARE_INSTALLATION_DIR)" --mware="$(MWARE_INSTALLATION_DIR)" --msp430ware="$(MSP430WARE_INSTALLATION_DIR)" --codegendir="$(CODEGEN_INSTALLATION_DIR)"
	@ echo ***********************************************************
	@ echo Please refer to "Examples for TI-RTOS" section in the TI-RTOS
	@ echo "Getting Started Guide" for details on how to build and load the examples
	@ echo into IAR WorkBench and CCS.
else
	@ echo Specify destination path like this: DEST="YOURPATH". Use UNIX style path "C:/examples"
endif
