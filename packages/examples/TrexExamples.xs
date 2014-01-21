var ExamplesList = xdc.loadCapsule("ExampleList.xs");
var BoardList = xdc.loadCapsule("Boards.xs");

/* Get the xdctools version from the passed in environment variable */
var XDCversion = environment["xdcver"];
XDCtoolsVer = "";
if ((XDCversion != "") && typeof XDCversion != 'undefined') {
    XDCtoolsVer =  XDCversion.match(/((\d+)_)+\w*/g)[0].replace(/_(\d+)/g, ".$1");
}
else {
    print("remark: XDCtools version: \'" + XDCversion + "\'");
}

/*
 *  ======== module$meta$init ========
 */
function module$meta$init() {
    var Examples = this;

    Examples.templateGroupArr.$add({
        id          : "examples.root",
        name        : "TI-RTOS",
        description : "TI-RTOS Templates and Examples"
    });

    /* Generate Board Trex Tree */
    for (var i = 0; i < BoardList.allBoards.length; i++) {

        createBoardRoot(Examples, BoardList.allBoards[i]);

        if(BoardList.supportsTool(BoardList.allBoards[i], "TI")) {
            createBoardTree(Examples, BoardList.allBoards[i], "TI");
        }
        if(BoardList.supportsTool(BoardList.allBoards[i], "GNU")) {
            createBoardTree(Examples, BoardList.allBoards[i], "GNU");
        }

    }

    for (var i = 0; i < ExamplesList.allExamples.length; i++) {
        var example = ExamplesList.allExamples[i];
        if(ExamplesList.supportsTool(example, "TI")) {
            for(var j = 0; j < example.boards.length; j++) {
                if (BoardList.supportsTool(example.boards[j], "TI")) {
                    addExamples(Examples, example.boards[j], example, "TI");
                }
            }
        }
        if(ExamplesList.supportsTool(example, "GNU")) {
            for(var j = 0; j < example.boards.length; j++) {
                if (BoardList.supportsTool(example.boards[j], "GNU")) {
                    addExamples(Examples, example.boards[j], example, "GNU");
                }
            }
        }
    }

    /* EK-TM4C123GXL Groups */
    Examples.templateGroupArr.$add({
        id          : "examples.root.EK_TM4C123GXL.example",
        name        : "Example Projects",
        description : "TI-RTOS Example Projects for the EK-TM4C123GXL",
        groups      : ["examples.root.EK_TM4C123GXL"]
    });
    /* EK-LM4F120XL Groups */
    Examples.templateGroupArr.$add({
        id          : "examples.root.EK_LM4F120XL.example",
        name        : "Example Projects",
        description : "TI-RTOS Example Projects for the EK_LM4F120XL",
        groups      : ["examples.root.EK_LM4F120XL"]
    });
    /* EKS-LM4F232 Groups */
    Examples.templateGroupArr.$add({
        id          : "examples.root.EKS_LM4F232.example",
        name        : "Example Projects",
        description : "TI-RTOS Example Projects for the EKS_LM4F232",
        groups      : ["examples.root.EKS_LM4F232"]
    });
    /* DK-TM4C123G Groups */
    Examples.templateGroupArr.$add({
        id          : "examples.root.DK_TM4C123G.example",
        name        : "Example Projects",
        description : "TI-RTOS Example Projects for the DK_TM4C123G",
        groups      : ["examples.root.DK_TM4C123G"]
    });
    /* DK-TM4C129X Groups */
    Examples.templateGroupArr.$add({
        id          : "examples.root.DK_TM4C129X.example",
        name        : "Example Projects",
        description : "TI-RTOS Example Projects for the DK-TM4C129X",
        groups      : ["examples.root.DK_TM4C129X"]
    });
    /* MSP-EXP430F5529LP Groups */
    Examples.templateGroupArr.$add({
        id          : "examples.root.MSP_EXP430F5529LP.example",
        name        : "Example Projects",
        description : "TI-RTOS Example Projects for the MSP-EXP430F5529LP",
        groups      : ["examples.root.MSP_EXP430F5529LP"]
    });
    /* MSP-EXP430F5529 Groups */
    Examples.templateGroupArr.$add({
        id          : "examples.root.MSP_EXP430F5529.example",
        name        : "Example Projects",
        description : "TI-RTOS Example Projects for the MSP_EXP430F5529",
        groups      : ["examples.root.MSP_EXP430F5529"]
    });
    /* TMDXDOCK28M36 Groups */
    Examples.templateGroupArr.$add({
        id          : "examples.root.TMDXDOCK28M36.example",
        name        : "Example Projects",
        description : "TI-RTOS Example Projects for the TMDXDOCK28M36",
        groups      : ["examples.root.TMDXDOCK28M36"]
    });
    /* TMDXDOCKH52C1 Groups */
    Examples.templateGroupArr.$add({
        id          : "examples.root.TMDXDOCKH52C1.example",
        name        : "Example Projects",
        description : "TI-RTOS Example Projects for the TMDXDOCKH52C1",
        groups      : ["examples.root.TMDXDOCKH52C1"]
    });

}

/*
 *  ======== createBoardRoot =======
 */
function createBoardRoot(Examples, Board) {
    Examples.templateGroupArr.$add({
        id          : "examples.root." + Board.name,
        name        : Board.trexName,
        description : "TI-RTOS Projects for " + Board.name,
        groups      : ["examples.root"]
    });
}

/*
 *  ======== createBoardTree =======
 */
function createBoardTree(Examples, Board, tool) {

    Examples.templateGroupArr.$add({
        id          : "examples.root." + Board.name + "." + tool,
        name        : tool + " Target Examples",
        description : tool + " Projects for " + Board.name,
        groups      : ["examples.root." + Board.name]
    });

    for(var i = 0; i < Board.peripherals.length; i++) {
        Examples.templateGroupArr.$add({
            id          : "examples.root." + Board.name + "." + tool + "." + Board.peripherals[i],
            name        : Board.peripherals[i] + " Examples",
            description : Board.peripherals[i] + " Example Projects for the " + Board.name,
            groups      : ["examples.root." + Board.name + "." + tool]
        });
    }
}

/*
 *  ======== addExamples ========
 */
function addExamples(Examples, opts, example, tool) {
    /* For empty examples */
    if((example.name == "empty") || (example.name == "empty_min")) {
	    /* We don't have GNU in NPW */
	    if (tool == "GNU") {
		    return;
		}
        Examples.templateArr.$add({
            buildProfile: "release",
            filterArr: opts.filter,
            target: opts.target,
            platform: opts.platforms["TI"],
            compilerBuildOptions: opts.compilerBuildOptions["TI"],
            linkerCommandFile: "",
            linkerBuildOptions: opts.linkerBuildOptions["TI"],
            title: example.title,
            name: example.name,
            description: example.description,
            fileList: [
                {path: "./" + opts.root + example.cFile},
                {path: "./" + opts.root + example.cfgFile},
                {path: "./" + opts.root + example.readme}
            ],
            isHybrid: true,
            groups: ["examples.root." + opts.name + "." + example.type],
            requiredProducts: example.requiredProducts,
            options: example.options,
            xdcToolsVersion: XDCtoolsVer
        });

        var ex = Examples.templateArr[Examples.templateArr.length - 1];

        /* Add Linker command file */
        if(opts.linkercmd) {
            ex.fileList.$add({path: "./" + opts.root + opts.linkercmd["TI"]});
        }

        /* Add any additional files to the list */
        for (k = 0; k < opts.fileList.length; k++) {
           ex.fileList.$add({path: "./" + opts.root + opts.fileList[k]});
        }
    }
    else {
        var group;
        if('group' in example) {
            group = "examples.root." + opts.name + "." + tool + "." + example.group;
        }
        else {
            group = "examples.root." + opts.name + "." + tool + "." + example.type;
            opts.filter[0].toolChain = tool;
        }

        opts.filter[0].toolChain = tool;
        Examples.templateArr.$add({
            buildProfile: "release",
            filterArr: opts.filter,
            target: opts.targets[tool],
            platform: opts.platforms[tool],
            compilerBuildOptions: opts.compilerBuildOptions[tool],
            linkerCommandFile: "",
            linkerBuildOptions: opts.linkerBuildOptions[tool],
            title: example.title,
            name: example.name,
            description: example.description,
            fileList: [
                {path: "./" + example.cFile},
                {path: "./" + example.cfgFile}
            ],
            isHybrid: true,
            groups: [group],
            requiredProducts: example.requiredProducts,
            options: example.options,
            xdcToolsVersion: XDCtoolsVer
        });

        var ex = Examples.templateArr[Examples.templateArr.length - 1];

        /* Add Linker command file */
        if(opts.linkercmd) {
            ex.fileList.$add({path: "./" + opts.root + opts.linkercmd[tool]});
        }

        /* Add any additional files to the list */
        for (k = 0; k < opts.fileList.length; k++) {
            ex.fileList.$add({path: "./" + opts.root + opts.fileList[k]});
        }

        /* Add any additional files to the list */
        for (k = 0; k < example.fileList.length; k++) {
            ex.fileList.$add({path: "./" + example.fileList[k].path, targetDirectory: "./" + example.fileList[k].targetDirectory});
        }

        /* Add any additional compiler options specific to example */
        if (example.compilerBuildOptions) {
            ex.compilerBuildOptions += example.compilerBuildOptions[tool];
        }

        /* Add any additional linker options specific to example */
        if (example.linkerOptions) {
            ex.linkerBuildOptions += example.linkerOptions;
        }
    }
}
