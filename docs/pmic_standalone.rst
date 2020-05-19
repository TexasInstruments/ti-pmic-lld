PMIC 
====
Overview
--------

Introduction
^^^^^^^^^^^^

The power-management integrated circuit (PMIC) is designed for powering
embedded systems or system on chip(SoC) in Automotive or Industrial
applications. 

The PMIC Driver supports Leo PMIC TPS65941. The Leo PMIC contains eleven GPIOs 
each with multiple functions and configurable features. The Leo PMIC includes
Real Time Clock (RTC) which provides the alarm and time-keeping functions. It 
provides time information and calendar information. RTC can generate timer 
interrupts(periodic interrupts) and alarm interrupts (precise interrupts).
GPIO can generate interrupts which indicates the High/Rising-Edge or the 
Low/Falling-Edge detection at the GPIO1 through GPIO11 pins.

The Leo PMIC contains two I2C interface channels. I2C channel 1 (I2C1)
is the main channel with access to the registers which control the  RTC and GPIO
registers. I2C channel 2 (I2C2), which is available through GPIO1 and GPIO2 pins, 
is dedicated for accessing the Q&A Watchdog communication registers.
  
Supported PMICS
----------------
1. Leo PMIC - TPS65941 (Texas Instruments)
  
Directory Structure
--------------------

The directory structure of PMIC is as follows:

+---------------+------------------------------------------------------------+
| Directory     |  Description                                               |
+===============+============================================================+
| include       | Contains the interface files for PMIC Driver               |
+---------------+------------------------------------------------------------+
| src           | Contains source(.c) files for the driver                   |
+---------------+------------------------------------------------------------+
| docs          | Contains generic change log for non-SDK delivery           |
+---------------+------------------------------------------------------------+

Build and Run
----------------

Build targets
^^^^^^^^^^^^^^^

The driver provides targets for building library.
The library and unit tests can be built from <PMIC root folder>.

Prerequisite:
   1) make or gmake should be installed.
      http://gnuwin32.sourceforge.net/packages/make.htm
      
   2) A cross compiler is required to be installed for compilation.
      If no cross-compiler is installed, default "gcc" will be used.
      cross-compiler e.g  arm-none-eabi-gcc-9.2.1
      https://armkeil.blob.core.windows.net/developer/Files/downloads/
      gnu-rm/9-2019q4/gcc-arm-none-eabi-9-2019-q4-major-win32-sha2.exe

-  Change directory to PMIC Root folder:
        cd <PMIC root folder> 

-  To build release library:
        <MAKE/GMAKE INSTALLATION DIR>make CC=<CROSS_COMPILER INSTALLATION DIR>arm-none-eabi-gcc-9.2.1.exe build=release all

-  To clean release library:
        <MAKE/GMAKE INSTALLATION DIR>make build=release clean

User Interface
--------------

API descriptions
^^^^^^^^^^^^^^^^^
API reference for application:
::

    #include <pmic.h>

.. rubric:: PMIC Initialization for Main I2C Bus 
.. PMIC Init for Step1:      

Steps to be followed for PMIC Initialization of Main I2C Bus 
::

    Pmic_CoreCfg_t        pmicConfigData =
    {
        PMIC_CFG_DEVICE_TYPE_VALID_SHIFT | PMIC_CFG_COMM_MODE_VALID_SHIFT \
        PMIC_CFG_SLAVEADDR_VALID_SHIFT | PMIC_CFG_COMM_HANDLE_VALID_SHIFT \
        PMIC_CFG_COMM_IO_WR_VALID_SHIFT | PMIC_CFG_COMM_IO_RD_VALID_SHIFT \
        PMIC_CFG_CRITSEC_START_VALID_SHIFT | PMIC_CFG_CRITSEC_STOP_VALID_SHIFT ,
        PMIC_MAIN_INST,
        PMIC_DEV_LEO_TPS6594, // Leo PMIC TPS65941 Slave device
        PMIC_INTF_DUAL_I2C,
        LEO_PMICA_SLAVE_ADDR,
        0,
        false,
        Pmic_regRead,
        Pmic_regWrite,
        NULL,
        NULL,
        Pmic_criticalSectionStartFn,
        Pmic_crit   icalSectionStopFn,
    };
    
    // For I2C Interface 
    pmicStatus = Pmic_i2c_lld_intf_setup(pmicConfigData, PMIC_MAIN_INST);
    ...
    or 
    // For SPI Interface 
    pmicStatus = Pmic_spi_lld_intf_setup(pmicConfigData);
    ...
    Pmic_init(pmicConfigData, pPmicCoreHandle);

 
.. rubric:: PMIC Initialization for QA I2C Bus
.. PMIC Init for Step2:

Steps to be followed for PMIC Initialization of QA I2C Bus 
::

    Pmic_CoreCfg_t        pmicCfgData =
    {
        PMIC_CFG_QASLAVEADDR_VALID_SHIFT | PMIC_CFG_QACOMM_HANDLE_VALID_SHIFT  ,
        PMIC_QA_INST,
        0,
        0,
        LEO_PMICA_WDG_SLAVE_ADDR,
        false,
        NULL,
        NULL,
        NULL,
        NULL,
        NULL,
        NULL,
    };

    Pmic_dual_i2c_pin_setup(pmicHandle);
    .....
    pmicStatus = Pmic_i2c_lld_intf_setup(pmicCfgData, PMIC_QA_INST);
    ...
    Pmic_init(pmicCfgData, pPmicCoreHandle);
   

**Pmic_init()** API is called for creating *pPmicCoreHandle* to be able to talk
to driver.

The **Pmic_init()** API takes a pointer to *Pmic_CoreCfg_t* as an input. The 
parameter *Pmic_CoreCfg_t* contains various parameters which are needed to 
prepare PMIC driver handle


Definition:
::

    Defined in pimc.h interface file as **Pmic_init()**

**Pmic_init()** API provides a software initialization for the driver.
Any application using PMIC Driver must call this API.

-  Application need to initialize pmicCorehandle structure in two steps for
   Dual I2C and one step for Single I2C or SPI Interface using validParams
   struture member
-  Application needs to be provide these functions to PMIC driver. After
   PMIC driver initialization these function will called by PMIC driver
   for PMIC register read and write

-  Application need to create pmicCorehandle structure for each PMIC device.
   For example - Application can create a pmicLeoCorehandle for Leo PMIC device
   and pmicHeraCorehandle for Hera PMIC device

::

    Pmic_regRead                - PMIC I2C/SPI read function

    Pmic_regWrite               - PMIC I2C/SPI write function

    Pmic_criticalSectionStartFn - Critical section start function

    Pmic_criticalSectionStopFn  - Critical section stop function

    Pmic_i2c_lld_intf_setup     - Interface setup function for PMIC to create
    instance and initialise the Main/QA I2C Bus for PMIC Communication based on 
    instance type
  
    Pmic_dual_i2c_pin_setup     - Configures GPIO1 and GPIO2 pins as I2C pins
    for QA I2C Bus interface setup

    Pmic_spi_lld_intf_setup     - Interface setup function for PMIC to create
    instance and initialise the SPI Bus for PMIC Communication      

.. rubric:: PMIC I2C interface setup for Main I2C Bus - Pmic_i2c_lld_intf_setup()

Interface setup function for PMIC to create instance and initialise the
Main/QA I2C Bus for PMIC Communication based on instance type

:: 

    setConfigI2C(0, CSL_WKUP_I2C0_CFG_BASE);
    ...
    I2C_init();
    ...
    I2C_Params_init(&i2cParams);
    ...
    i2cHandle = I2C_open(0, &i2cParams);
    ...
    pPmicConfigData->pCommHandle = i2cHandle;

Similarly we need to define PMIC I2C interface setup for QA I2C Bus

.. rubric:: PMIC de-initialization - Pmic_deinit()

-  De-initialization of PMIC - This de-initialization is specific
   to the application. 
-  It only de-initializes the LLD being used for this Instance

Definition:
::

    Defined in pimc.h interface file as **Pmic_deinit()**
