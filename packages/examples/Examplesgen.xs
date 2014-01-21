/*
 * Copyright (c) 2013, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 *  ======== Examplesgen.xs ========
 *  This script is run to generate tirtos examples from the installed product.
 *  It creates a directory structure for the examples based on toolchain with
 *  each toolchain containing folders for all boards it supports which in
 *  turn contain folders for every example supported by that board.
 *  Also makefiles to build these examples with that toolchain are provided so
 *  that a user can run a top level make to build all examples or run make to
 *  build any example specifically.
 *
 *        arguments:  an array of string arguments passed in the examplesgen
 *                    rule in tirtos.mak which are initialized as follows:
 *
 *		        arguments[0] - toolchain (e.g IAR)
 *		        arguments[1] - TIRTOS installation directory
 *		        arguments[2] - user provided destination path for
 *		        generated examples
 *
 */

var tool = arguments[0];
var tirtosRoot = arguments[1];
var dest = arguments[2];

/* Extract TI-RTOS version */
var temp = tirtosRoot.split('/');
var tirtosVersion = temp[temp.length - 1];

/* create root directory */
var root = new java.io.File(dest + "/" + tirtosVersion + "_examples");
root.mkdir();

/* create directory for toolchain */
var toolroot = new java.io.File(root.getPath() + "/" + tool);
toolroot.mkdir();

/* Load Boards.xs script for board data */
var boardcapsule = xdc.loadCapsule("Boards.xs");

/* Load ExampleList script for example data */
var examplescapsule = xdc.loadCapsule("ExampleList.xs");

var Boards = boardcapsule.allBoards;
var Example = examplescapsule.allExamples;

/* Path to examples in TI-RTOS product */
var tirtosExamplesPath = tirtosRoot +  "/packages/examples/";

var boardroot;
var exampleroot;

for (var i = 0; i < Boards.length; i++) {

    /* check if board is available for that toolchain */
    if (boardcapsule.supportsTool(Boards[i], tool)) {

        /* Create directory for board */
        boardroot =  new java.io.File(toolroot.getPath() + "/" + Boards[i].name)
        boardroot.mkdir();
        copymakefiles(boardroot.getPath(), Boards[i], "makedefs", tool);
        copymakefiles(boardroot.getPath(), Boards[i], "topmake", tool);

        for (var k = 0; k < Example.length; k++) {

            /* if example is available on board and is supported by a tool */
            if (examplescapsule.supportsBoard(Example[k], Boards[i]) &&
                            examplescapsule.supportsTool(Example[k], tool)) {

                /* create directory for example */
		exampleroot =  new java.io.File(boardroot.getPath() + "/" +
		                   Example[k].name);
                exampleroot.mkdir();
                /* copy all board files */
                copyBoardFiles(Boards[i], exampleroot.getPath());
                /* copy all example files */
                copyExampleFiles(Example[k], Boards[i], exampleroot.getPath());
                /* copy necessary makefiles */
                copymakefiles(exampleroot.getPath(), Boards[i], "lowmake", tool);

                if (Example[k].compilerBuildOptions) {
		    var fileWriter = new
		       java.io.FileWriter(exampleroot.getPath() + "/makefile",
		          true);
                    fileWriter.write("CFLAGS += " +
                                    Example[k].compilerBuildOptions[tool]);
                    fileWriter.flush();
                    fileWriter.close();
                }
            }
        }
    }
}

/*
 *  filecopy - used to copy from one file to new destination
 */
function filecopy(source, target)
{
    var length;
    var inputstream = new java.io.FileInputStream(source);
    var outputstream = new java.io.FileOutputStream(target);
    var buffer = java.lang.reflect.Array.newInstance(java.lang.Byte.TYPE, 1024);

    while ((length = inputstream.read(buffer)) > 0) {
        outputstream.write(buffer, 0, length);
    }

    inputstream.close();
    outputstream.close();

}

/*
 *  copyBoardFiles - copy board related files
 */
function copyBoardFiles(Board, dest)
{
    var srcpath = tirtosExamplesPath + Board.root;
    var srchandle;
    var desthandle;

    for(var i = 0; i < Board.fileList.length; i++){
        srchandle = new java.io.File(srcpath + Board.fileList[i]);
        desthandle = new java.io.File(dest + "/" + Board.fileList[i]);

        filecopy(srchandle, desthandle);
    }
    srchandle = new java.io.File(srcpath + Board.linkercmd[tool]);
    desthandle = new java.io.File(dest + "/" + Board.linkercmd[tool]);

    filecopy(srchandle, desthandle);

}

/*
 *  copyExampleFiles - copy example files
 */
function copyExampleFiles(Example, Board, dest)
{
    var srcpath;
    var destpath;
    var srchandle;
    var desthandle;

    /* copy .c file */
    if (Example.name == "empty" || Example.name == "empty_min") {
        srcpath = tirtosExamplesPath + Board.root + "/" + Example.cFile;
    }
    else {
        srcpath = tirtosExamplesPath + Example.cFile;
    }
    if (Example.type == "demo") {
        var nameArray = Example.cFile.split('/');
        filename = nameArray[nameArray.length - 1];
        destpath = dest + "/" + filename;
    }
    else {
        destpath = dest + "/" + Example.cFile;
    }

    srchandle = new java.io.File(srcpath);
    desthandle = new java.io.File(destpath);
    filecopy(srchandle, desthandle);

    /* copy .cfg file */
    if (Example.name == "empty" || Example.name == "empty_min") {
        srcpath = tirtosExamplesPath + Board.root + "/" + Example.cfgFile;
    }
    else {
        srcpath = tirtosExamplesPath + Example.cfgFile;
    }
    if(Example.type == "demo") {
        var nameArray = Example.cfgFile.split('/');
        filename = nameArray[nameArray.length - 1];
        destpath = dest + "/" + filename;
    }
    else {
        destpath = dest + "/" + Example.cfgFile;
    }

    srchandle = new java.io.File(srcpath);
    desthandle = new java.io.File(destpath);
    filecopy(srchandle, desthandle);

    for (var i = 0; i < Example.fileList.length; i++) {
        srcpath = tirtosExamplesPath + Example.fileList[i];

        if (Example.type == "demo") {
            var nameArray = Example.fileList[i].split('/');
            filename = nameArray[nameArray.length - 1];
            destpath = dest + "/" + filename;
        }
        else {
            destpath = dest + "/" + Example.fileList[i];
        }

        srchandle = new java.io.File(srcpath);
        desthandle = new java.io.File(destpath);
        filecopy(srchandle, desthandle);
    }

}

/*
 *  copyBoardFiles - copy makefiles
 */
function copymakefiles(dest, Board, type)
{
    var srcpath;
    var destpath;
    var srchandle;
    var desthandle;

    switch (type) {
        case "topmake":
        srcpath = tirtosExamplesPath + "Examplegenmakefiles" + "/makefile";
	destpath = dest + "/" + "makefile";
        srchandle = new java.io.File(srcpath);
        desthandle = new java.io.File(destpath);
        filecopy(srchandle, desthandle);
        break;

        case "lowmake":
        srcpath = tirtosExamplesPath + "Examplegenmakefiles/makefile.xdt";
        destpath = dest + "/" + "makefile";
        var template = xdc.loadTemplate(srcpath);
        template.genFile(destpath, this, [Board]);
        break;

        case "makedefs":
        srcpath = tirtosExamplesPath + "Examplegenmakefiles/makedefs.xdt";
        destpath = dest + "/" + "makedefs";
        var template = xdc.loadTemplate(srcpath);
        template.genFile(destpath, this, [Board, tool]);
        break;
     }
}
