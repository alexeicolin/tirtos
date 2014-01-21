/*
 *  Copyright 2013 by Texas Instruments Incorporated.
 *
 */

var tool;
var root;
var tirtosRoot;
var tirtosExamplesPath;
var dest;
var xdctools;
var bios;
var uia;
var ndk;
var tivaware;
var mware;
var msp430;
var codegendir;
var ideFileGen;
var ipcf = [];

function run(cmdr, args){

    /* Check for required command line parameters */
    if (this.tirtosProductDir == undefined || this.toolChain == undefined ||
            this.outputDir == undefined || this.bios == undefined ||
            this.xdctools ==  undefined || this.uia == undefined || this.ndk ==
            undefined || this.tivaware == undefined || this.mware == undefined ||
            this.msp430 == undefined) {
        cmdr.usage();
        return (null);
    }

    /* Retrieve args */
    tool = this.toolChain;
    tirtosRoot = this.tirtosProductDir;
    dest = this.outputDir;
    xdctools = this.xdctools;
    bios = this.bios;
    uia = this.uia;
    ndk = this.ndk;
    tivaware = this.tivaware;
    mware = this.mware;
    msp430 = this.msp430;
    codegendir = this.codegendir;
    if (this.toolChain == "IAR") {
        ideFileGen = this.idefiles;
    }
    else {
        ideFileGen = false;
    }

    /* Extract TI-RTOS version */
    var temp = tirtosRoot.split('/');
    var tirtosVersion = temp[temp.length - 1];

    /* create root directory */
    root = new java.io.File(dest + "/" + tirtosVersion + "_examples");
    root.mkdir();

    /* create directory for toolchain */
    var toolRoot = new java.io.File(root.getPath() + "/" + tool);
    toolRoot.mkdir();

    /* Load Boards.xs script for board data */
    var boardCapsule = xdc.loadCapsule("../Boards.xs");

    /* Load ExampleList script for example data */
    var examplesCapsule = xdc.loadCapsule("../ExampleList.xs");

    var Boards = boardCapsule.examplesgenBoards;
    var Example = examplesCapsule.allExamples;

    /* Path to examples in TI-RTOS product */
    tirtosExamplesPath = tirtosRoot +  "/packages/examples/";

    var boardRoot;
    var exampleRoot;
    for (var i = 0; i < Boards.length; i++) {

        /* check if board is available for that toolchain */
        if (boardCapsule.supportsTool(Boards[i], tool)) {

            /* Create directory for board */
            boardRoot =  new java.io.File(toolRoot.getPath() + "/" +
                    Boards[i].name);
            boardRoot.mkdir();
            copyMakefiles(boardRoot.getPath(), Boards[i], "makedefs", "");
            copyMakefiles(boardRoot.getPath(), Boards[i], "topmake", "");

            for (var k = 0; k < Example.length; k++) {
                /*** TO DO ***/
                if (Example[k].type == "Demo") {
                    continue;
                }
                /* if example is available on board and is supported by a tool */
                if (examplesCapsule.supportsBoard(Example[k], Boards[i]) &&
                        examplesCapsule.supportsTool(Example[k], tool)) {

                    /* create directory for example */
                    exampleRoot =  new java.io.File(boardRoot.getPath() + "/" +
                               Example[k].name);
                    exampleRoot.mkdir();
                    /* copy all board files */
                    copyBoardFiles(Boards[i], exampleRoot.getPath());
                    /* copy all example files */
                    copyExampleFiles(Example[k], Boards[i],
                            exampleRoot.getPath());
                    /* copy necessary makefiles */
            if ('compilerBuildOptions' in Example[k]) {
                copyMakefiles(exampleRoot.getPath(), Boards[i], "lowmake",
                            Example[k].compilerBuildOptions[tool + "_makefile"]);
             }
            else {
                copyMakefiles(exampleRoot.getPath(), Boards[i], "lowmake", "");
            }
                    /* generate necessary ipcf files */
                    if (ideFileGen) {
                        genIARFiles(exampleRoot.getPath(), "ipcf", Example[k],
                            Boards[i]);
                    }

                }
            }
        }
    }

    if (ideFileGen) {
        genIARFiles(toolRoot, "html", tirtosVersion);

        var iconDir = new java.io.File(toolRoot + "/icons");
        iconDir.mkdir();
        filecopy(tirtosExamplesPath + "examplesgen/iar/icons/chip_closed.png",
            iconDir.getPath() + "/chip_closed.png");
        filecopy(tirtosExamplesPath + "examplesgen/iar/icons/chip_open.png",
            iconDir.getPath() + "/chip_open.png");
        filecopy(tirtosExamplesPath + "examplesgen/iar/icons/cubeAqua_closed.png",
            iconDir.getPath() + "/cubeAqua_closed.png");
        filecopy(tirtosExamplesPath + "examplesgen/iar/icons/cubeAqua_open.png",
            iconDir.getPath() + "/cubeAqua_open.png");
        filecopy(tirtosExamplesPath + "examplesgen/iar/icons/examplefile_s.png",
            iconDir.getPath() + "/examplefile_s.png");
        filecopy(tirtosExamplesPath + "examplesgen/iar/icons/tilogo.gif",
            iconDir.getPath() + "/tilogo.gif");
        filecopy(tirtosExamplesPath + "examplesgen/iar/icons/titagline.gif",
            iconDir.getPath() + "/titagline.gif");
    }
}


/*
 *  filecopy - used to copy from one file to new destination
 */
function filecopy(source, target)
{
    var length;
    var inputStream = new java.io.FileInputStream(source);
    var outputStream = new java.io.FileOutputStream(target);
    var buffer = java.lang.reflect.Array.newInstance(java.lang.Byte.TYPE, 1024);

    while ((length = inputStream.read(buffer)) > 0) {
        outputStream.write(buffer, 0, length);
    }

    inputStream.close();
    outputStream.close();

}

/*
 *  copyBoardFiles - copy board related files
 */
function copyBoardFiles(Board, dest)
{
    var srcPath = tirtosExamplesPath + Board.root;
    var srcHandle;
    var destHandle;

    for(var i = 0; i < Board.fileList.length; i++){
        srcHandle = new java.io.File(srcPath + Board.fileList[i]);
        destHandle = new java.io.File(dest + "/" + Board.fileList[i]);

        filecopy(srcHandle, destHandle);
    }
    srcHandle = new java.io.File(srcPath + Board.linkercmd[tool]);
    destHandle = new java.io.File(dest + "/" + Board.linkercmd[tool]);

    filecopy(srcHandle, destHandle);

}

/*
 *  copyExampleFiles - copy example files
 */
function copyExampleFiles(Example, Board, dest)
{
    var srcPath;
    var destPath;
    var srcHandle;
    var destHandle;
    var fileName;
    var basePath;

    if ((Example.name == "empty") || (Example.name == "empty_min")) {
        basePath = tirtosExamplesPath + Board.root + "/";
    }
    else {
        basePath = tirtosExamplesPath;
    }

    /* copy .c file */
    srcPath = basePath + Example.cFile;

    if (Example.type == "demo") {
        var nameArray = Example.cFile.split('/');
        fileName = nameArray[nameArray.length - 1];
        destPath = dest + "/" + fileName;
    }
    else {
        destPath = dest + "/" + Example.cFile;
    }

    srcHandle = new java.io.File(srcPath);
    destHandle = new java.io.File(destPath);
    filecopy(srcHandle, destHandle);

    /* copy .cfg file */
    srcPath = basePath + Example.cfgFile;

    if(Example.type == "demo") {
        var nameArray = Example.cfgFile.split('/');
        fileName = nameArray[nameArray.length - 1];
        destPath = dest + "/" + fileName;
    }
    else {
        destPath = dest + "/" + Example.cfgFile;
    }

    srcHandle = new java.io.File(srcPath);
    destHandle = new java.io.File(destPath);
    filecopy(srcHandle, destHandle);

    for (var i = 0; i < Example.fileList.length; i++) {
        var targetDirectory = Example.fileList[i].targetDirectory;
        if (targetDirectory != ".") {
            var dirs = new java.io.File(dest + "/" + targetDirectory);
            dirs.mkdirs();
        }

        srcPath = basePath + Example.fileList[i].path;

        var nameArray = Example.fileList[i].path.split('/');
        var fileName = nameArray[nameArray.length - 1];
        if (targetDirectory != ".") {
            destPath = dest + "/" + targetDirectory + "/" + fileName;
        }
        else {
            destPath = dest + "/" + fileName;
        }

        srcHandle = new java.io.File(srcPath);
        destHandle = new java.io.File(destPath);
        filecopy(srcHandle, destHandle);
    }

}

/*
 *  copyMakefiles - copy makefiles
 */
function copyMakefiles(dest, Board, type, opt)
{
    var srcPath;
    var destPath;
    var srcHandle;
    var destHandle;

    switch (type) {
        case "topmake":
        srcPath = tirtosExamplesPath + "examplesgen/makefiles" + "/makefile";
        destPath = dest + "/" + "makefile";
        srcHandle = new java.io.File(srcPath);
        destHandle = new java.io.File(destPath);
        filecopy(srcHandle, destHandle);
        break;

        case "lowmake":
        srcPath = tirtosExamplesPath + "examplesgen/makefiles/makefile.xdt";
        destPath = dest + "/" + "makefile";
        var template = xdc.loadTemplate(srcPath);
        template.genFile(destPath, this, [tool, opt]);
        break;

        case "makedefs":
        srcPath = tirtosExamplesPath + "examplesgen/makefiles/makedefs.xdt";
        destPath = dest + "/" + "makedefs";
        var template = xdc.loadTemplate(srcPath);
        template.genFile(destPath, this, [Board, tool, tirtosRoot, xdctools,
            bios, uia, ndk, tivaware, mware, msp430, codegendir]);
        break;
     }
}

/*
 *  genIARFiles - generate IAR files
 */
function genIARFiles(dest, type, example, board)
{
    var srcPath;
    var destPath;

    switch (type) {
        case "ipcf":
        srcPath = tirtosExamplesPath + "examplesgen/iar/ipcf.xdt";
        destPath = dest + "/" + example.name + ".ipcf";
        ipcf[ipcf.length++] = {
            path:     destPath,
            example:  example,
            board:    board,
        };
        var template = xdc.loadTemplate(srcPath);
        template.genFile(destPath, this, [example, board, dest]);
        break;

        case "html":
        srcPath = tirtosExamplesPath + "examplesgen/iar/html.xdt";
        destPath = dest + "/" + "Examples.html";
        var template = xdc.loadTemplate(srcPath);
        template.genFile(destPath, this, [ipcf]);

        srcPath = tirtosExamplesPath + "examplesgen/iar/args.xdt";
        destPath = dest + "/" + example + ".custom_argvars";
        var template = xdc.loadTemplate(srcPath);
        template.genFile(destPath, this, [xdctools, dest, example]);

        break;
    }
}
