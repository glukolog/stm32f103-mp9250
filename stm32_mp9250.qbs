import qbs
//import qbs.FileInfo
//import qbs.ModUtils

Product {
    name: "stm32_u8g2"
    type: ["application"]

    Depends { name:"cpp" }

    property string libPath: path+"/../stm32f10x/Libraries/"

    consoleApplication: true

    cpp.positionIndependentCode: false
    cpp.executableSuffix: ".elf"

    cpp.includePaths: [
        path+"/src/",
        path+"/../stmlibs/",
        path+"/u8g2/csrc/",
        libPath+"STM32F10x_StdPeriph_Driver/inc/",
        libPath+"CMSIS/CM3/CoreSupport/",
        libPath+"CMSIS/CM3/DeviceSupport/ST/STM32F10x/"
    ]

    cpp.defines: [
        "STM32F10X_MD",
        "USE_STDPERIPH_DRIVER"
    ]

    Properties {
        condition: cpp.debugInformation
        cpp.defines: outer.concat("DEBUG")
    }

    cpp.commonCompilerFlags: [
        "-mthumb","-mcpu=cortex-m3","-msoft-float","-mfpu=vfp","-Os",
        "-fdata-sections","-ffunction-sections","-fno-inline","-flto",
        "--specs=nosys.specs"
    ]

    cpp.dynamicLibraries: ["m\ "]
    cpp.linkerFlags:[
        "-mthumb","-mcpu=cortex-m3","-msoft-float","-mfpu=vfp","--specs=nosys.specs",
        //"-L"+path+"/out","-lu8g2",
        "-lgcc","-lc","-lm",
        //"-Wl,--start-group","-Wl,--gc-sections",
        "-lnosys",
        "-T",path+"/STM32F103X8.ld"
    ]

    //cpp.cxxFlags: ["-std=c++11"]
    //cpp.cFlags: ["-std=gnu99"]
    cpp.warningLevel: "all"


    Group {
        name: "Sources"
        prefix: path+"/src/"
        files: [
            "*.h",
            "*.c",
        ]
    }

    Group {
        name: "Libraries"
        files: [
            path+"/../stmlibs/systick.c",
            //path+"/../stmlibs/i2c-soft.c",
            path+"/../stmlibs/bmp280.c",
            path+"/u8g2/csrc/*.c",
        ]
    }

    Group {
        name: "StdPeriph_Driver"
        prefix: libPath+"STM32F10x_StdPeriph_Driver/**/"
        files: [
            "*.h",
            "*.c",
        ]
    }

    Group {
        name: "System"
        files: [
            libPath + "CMSIS/CM3/CoreSupport/*",
            libPath + "CMSIS/CM3/DeviceSupport/ST/STM32F10x/system_stm32f10x.*",
            libPath + "CMSIS/CM3/DeviceSupport/ST/STM32F10x/startup/gcc_ride7/startup_stm32f10x_md.s"
        ]
    }

    Group {
        qbs.install: true
        fileTagsFilter: "application"
    }

}

