#
_XDCBUILDCOUNT = 0
ifneq (,$(findstring path,$(_USEXDCENV_)))
override XDCPATH = /home/acolin/rtml/src/ti/tirtos/tirtos_1_21_00_09/packages;/home/acolin/rtml/src/ti/tirtos/tirtos_1_21_00_09/products/uia_1_04_00_06/packages;/home/acolin/rtml/src/ti/tirtos/tirtos_1_21_00_09/products/ndk_2_23_01_01/packages;/home/acolin/rtml/src/ti/tirtos/tirtos_1_21_00_09/products/ipc_3_10_01_11/packages;/home/acolin/rtml/src/ti/tirtos/tirtos_1_21_00_09/products/bios_6_37_00_20/packages;/home/acolin/rtml/src/ti/tirtos/tirtos_1_21_00_09/products/MWare_v201b/MWare;/home/acolin/rtml/src/ti/tirtos/tirtos_1_21_00_09/products/TivaWare_C_Series-2.0.1.11577a
override XDCROOT = /home/acolin/rtml/src/ti/xdctools/xdctools_3_25_05_94
override XDCBUILDCFG = /home/acolin/rtml/src/ti/tirtos/tirtos_1_21_00_09/tirtos.bld
endif
ifneq (,$(findstring args,$(_USEXDCENV_)))
override XDCARGS = MWareDir='/home/acolin/rtml/src/ti/tirtos/tirtos_1_21_00_09/products/MWare_v201b/MWare' TivaWareDir='/home/acolin/rtml/src/ti/tirtos/tirtos_1_21_00_09/products/TivaWare_C_Series-2.0.1.11577a' MSP430WareDir='/home/acolin/rtml/src/ti/tirtos/tirtos_1_21_00_09/products/MSP430Ware_1_60_02_09a' MSP430DEVLIST='MSP430F5529' gnu.targets.arm.M3='/usr' gnu.targets.arm.M4F='/usr'
override XDCTARGETS = 
endif
#
ifeq (0,1)
PKGPATH = /home/acolin/rtml/src/ti/tirtos/tirtos_1_21_00_09/packages;/home/acolin/rtml/src/ti/tirtos/tirtos_1_21_00_09/products/uia_1_04_00_06/packages;/home/acolin/rtml/src/ti/tirtos/tirtos_1_21_00_09/products/ndk_2_23_01_01/packages;/home/acolin/rtml/src/ti/tirtos/tirtos_1_21_00_09/products/ipc_3_10_01_11/packages;/home/acolin/rtml/src/ti/tirtos/tirtos_1_21_00_09/products/bios_6_37_00_20/packages;/home/acolin/rtml/src/ti/tirtos/tirtos_1_21_00_09/products/MWare_v201b/MWare;/home/acolin/rtml/src/ti/tirtos/tirtos_1_21_00_09/products/TivaWare_C_Series-2.0.1.11577a;/home/acolin/rtml/src/ti/xdctools/xdctools_3_25_05_94/packages;..
HOSTOS = Linux
endif
