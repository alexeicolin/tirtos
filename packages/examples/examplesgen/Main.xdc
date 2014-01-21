/*
 *  Copyright 2013 by Texas Instruments Incorporated.
 *
 */

/*!
 *  ======== Main ========
 *  This tool generates TIRTOS examples from the installed product.
 *  It creates a directory structure for the examples based on toolchain with
 *  each toolchain containing folders for all boards it supports which in
 *  turn contain folders for every example supported by that board.
 *  Also makefiles to build these examples with that toolchain are provided so
 *  that a user can run a top level make, to build all examples or run make in
 *  a given example to build it specifically.
 *
 *       options:
 *       [--tirtos      tirtos_product_directory]
 *       [--toolchain   toolchain(e.g IAR)]
 *       [--output      output directory]
 *       [--xdctools    XDC tools install path]
 *       [--bios        BIOS install path]
 *       [--uia         UIA install path]
 *       [--ndk         NDK install path]
 *       [--tivaware    TivaWare install path]
 *       [--mware       MWare install path]
 *       [--msp430ware  MSP430Ware install path]
 *       [--codegendir  Codegen install directory]
 *       [--idefiles    Generates IDE files]
 *
 */

metaonly module Main inherits xdc.tools.ICmd
{
    override config String usage[] = [
        '[--tirtos      tirtos_product_directory]',
        '[--toolchain   toolchain(e.g IAR)]',
        '[--output      output directory]',
        '[--xdctools    XDC tools install path]',
        '[--bios        BIOS install path]',
        '[--uia         UIA install path]',
        '[--ndk         NDK install path]',
        '[--tivaware    TivaWare install path]',
        '[--mware       MWare install path]',
        '[--msp430ware  MSP430Ware install path]',
        '[--codegendir  Codegen install directory]',
        '[--idefiles    Generates IDE files]'
    ];

instance:

    /*!
     *  ======== tirtosproductDir ========
     *  TI-RTOS Product root directory
     *
     *  This option names the tirtos root directory that is used by the
     *  tool to generate the examples and makefiles.
     *
     *  This option should be specified
     *
     */
    @CommandOption("tirtos")
    config String tirtosProductDir;

    /*!
     *  ======== toolChain ========
     *  Option specifying the tool chain to be used build examples
     *
     *  This is a required option
     */
    @CommandOption("toolchain")
    config String toolChain;

    /*!
     *  ======== outputDir ========
     *  Output directory in which the examples will be generated.
     *
     *  This option is required and paths should be speciified with UNIX style
     *  i.e no forward slashes.
     */
    @CommandOption("output")
    config String outputDir;

    /*!
     *  ======== xdctools ========
     * Path to installed xdctools.
     *
     *  This option is required and paths should be speciified with UNIX style
     *  i.e no forward slashes.
     */
    @CommandOption("xdctools")
    config String xdctools;

    /*!
     *  ======== bios ========
     * Path to installed bios.
     *
     *  This option is required and paths should be speciified with UNIX style
     *  i.e no forward slashes.
     */
    @CommandOption("bios")
    config String bios;

    /*!
     *  ======== uia ========
     * Path to installed UIA.
     *
     *  This option is required and paths should be speciified with UNIX style
     *  i.e no forward slashes.
     */
    @CommandOption("uia")
    config String uia;

    /*!
     *  ======== ndk ========
     *  Path to installed NDK.
     *
     *  This option is required and paths should be speciified with UNIX style
     *  i.e no forward slashes.
     */
    @CommandOption("ndk")
    config String ndk;

    /*!
     *  ======== tivaware ========
     *  Path to installed TivaWare.
     *
     *  This option is required and paths should be speciified with UNIX style
     *  i.e no forward slashes.
     */
    @CommandOption("tivaware")
    config String tivaware;

    /*!
     *  ======== mware ========
     *  Path to installed MWare.
     *
     *  This option is required and paths should be speciified with UNIX style
     *  i.e no forward slashes.
     */
    @CommandOption("mware")
    config String mware;

    /*!
     *  ======== msp430 ========
     *  Path to installed MSP40Ware.
     *
     *  This option is required and paths should be speciified with UNIX style
     *  i.e no forward slashes.
     */
    @CommandOption("msp430ware")
    config String msp430;

    /*!
     *  ======== codegendir ========
     *  Path to codegen installation directory.
     *
     *  This option is required and paths should be speciified with UNIX style
     *  i.e no forward slashes.
     */
    @CommandOption("codegendir")
    config String codegendir;

    /*!
     *  ======== idefiles ========
     *  Generate IDE files.
     *
     *  This option is optional and when this option is specified, IDE
     *  integration files are generated. This is supported IAR toolchain only.
     */
    @CommandOption("idefiles")
    config Bool idefiles = true;

    /*!
     *  ======== run ========
     */
    override Any run(xdc.tools.Cmdr.Instance cmdr, String args[]);
}
