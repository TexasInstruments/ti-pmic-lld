# Texas Instruments PMIC Low-Level Driver (LLD) for Coach / LP8772X-Q1

This is an API guide for PMIC (Power Management Integrated Circuit) Driver. PMIC
Driver is designed to power up different components on the embedded boards or
provide supply to MCU (Micro Controller Unit) or SoC (System on chip) using
APIs provided in the guide.

The PMIC Driver supports below mentioned PMIC devices and their features or
Modules.

Supported PMIC Devices are:

1. LP8772X-Q1: Three Buck Converters, one Linear Regulator and one Load Switch
   for AWR and IWR Radar Sensors

## Driver Usage

This section provides an introduction to how to use this driver within your project.

### Getting Started

Start by downloading this repository and placing it within your project
structure as appropriate. To ensure simplified delivery of future changes and
bugfixes, it is recommended to clone this repository into the desired location.

git repositories can be cloned into a specific folder by specifying the desired
folder name as the last argument to the `git clone` command, as follows:

    git clone git@github.com:TexasInstruments/ti-pmic-lld.git <DESIRED FOLDER NAME>

To clone the **contents** of this repository into your current directory, use
the `.` folder name, such as:

    git clone git@github.com:TexasInstruments/ti-pmic-lld.git .

Once the repository has been cloned, check-out the LP8772X-Q1 support branch
(to track future changes) or check out a relevant release tag (to freeze
changes). The development branch for this device can be checked out with:

    git checkout device/coach

#### Including in a Project

To include the PMIC driver as part of a larger project, do the following:

- Update compiler include path with the `include/` directory
- Update the compiler source files to compile all files in the `src/` directory

To use the PMIC driver throughout the project, the user can then simply include
"pmic.h" and have access to all APIs provided by this driver.

#### Generating API Documentation

For more information on how to use each module, the header files are documented
using [Doxygen](https://www.doxygen.nl) syntax. This is fairly readable while
in source format, however Doxygen compiled documentation offers advantages of
better cross-referencing and some organization by topic.

To generate the compiled Doxygen documentation included with this driver,
install Doxygen so that it is available on your command line and then execute
the following command from the PMIC driver root folder:

    doxygen docs/ti.doxyfile

Alternatively, if you have Make/GMake installed, run the following command:

    make docs

This will generate HTML documentation in the `docs/` folder which can be viewed
in a web browser. To view the documentation, open the file at
`docs/html/index.html`.

#### Driver Initialization

All APIs provided by this driver expect to receive a `Pmic_CoreHandle_t` in
order to handle communication with the device. This handle should be created
through the use of the `Pmic_CoreCfg_t` structure in `pmic.h` and the
`Pmic_init()` API.

In order to successfully create a handle, the user will need to provide an
implementation for 4 APIs detailed below which inform the driver how to
operate on the specific platform.

##### PMIC Handle User Functions: Critical Section Start/Stop

When constructing `Pmic_CoreCfg_t`, two functions need to be provided in order
for the PMIC to obtain a critical section. These functions are called by the
driver before and after I2C/SPI communications. It is up to the user to
determine what is an appropriate implementation of these APIs as considerations
vary based on platform support and application complexity.

For very simple microcontroller applications, blocking other interrupts or
claiming a simple global variable may be enough. For more complex applications
and on platforms which support it, a proper shared mutex should be
claimed/released as appropriate to ensure no other device drivers are
attempting to use the I2C/SPI bus at the same time.

Within the `Pmic_CoreCfg_t` structure, these two functions are:

```c
{
    .pFnPmicCritSecStart = <your CS start function>,
    .pFnPmicCritSecStop = <your CS stop function>,
}
```

##### PMIC Handle User Functions: Communications I/O Read/Write

When constructing `Pmic_CoreCfg_t`, two functions need to be provided in order
for the PMIC to know how to read and write over the desired communications
channel (I2C or SPI, typically). The specific implementation of these functions
is platform dependent, the chosen processor likely has an SDK which provides
functions that match relatively closely.

Within the `Pmic_CoreCfg_t` structure, these two functions are:

```c
{
    .pFnPmicCommIoRead = <your I/O read function>,
    .pFnPmicCommIoWrite = <your I/O write function>,
}
```

##### Finalizing Initialization

Once the `Pmic_CoreCfg_t` structure has been initialized with the necessary
information, the user should call `Pmic_init()` in order to convert the
`Pmic_CoreCfg_t` into a `Pmic_CoreHandle_t` which will be used with the rest of
the driver APIs.

A full example of what this may look like for LP8772X-Q1 is shown below:

```c
int32_t status;

// The handle should either be declared globally, or stored in a structure that
// can manage access throughout the application, it will need to be re-used
// often.
Pmic_CoreHandle_t PmicHandle;

Pmic_CoreCfg_t coreCfg = {
    .validParams = (
        PMIC_CFG_DEVICE_TYPE_VALID_SHIFT    |
        PMIC_CFG_COMM_MODE_VALID_SHIFT      |
        PMIC_CFG_CRC_ENABLE_VALID_SHIFT     |
        PMIC_CFG_CFG_CRC_ENABLE_VALID_SHIFT |
        PMIC_CFG_SLAVEADDR_VALID_SHIFT      |
        PMIC_CFG_COMM_HANDLE_VALID_SHIFT    |
        PMIC_CFG_COMM_IO_RD_VALID_SHIFT     |
        PMIC_CFG_COMM_IO_WR_VALID_SHIFT     |
        PMIC_CFG_CRITSEC_START_VALID_SHIFT  |
        PMIC_CFG_CRITSEC_STOP_VALID_SHIFT
    ),
    .instType = PMIC_MAIN_INST,
    .pmicDeviceType = PMIC_DEV_COACH_LP8772X,
    .commMode = PMIC_INTF_I2C_SINGLE,
    .crcEnable = PMIC_ENABLE,
    .configCrcEnable = PMIC_ENABLE,
    .slaveAddr = <Device I2C Address>,
    .pCommHandle = &commHandle,
    .pFnPmicCommIoRd = PmicCommIoRead,
    .pFnPmicCommIoWr = PmicCommIoWrite,
    .pFnPmicCritSecStart = CritSecStart,
    .pFnPmicCritSecStop = CritSecStop,
};

status = Pmic_init(&PmicHandle, &coreCfg);

// Check the return code of Pmic_init(), if it is PMIC_ST_SUCCESS, the
// PmicHandle is now valid for use throughout the rest of the application
if (status == PMIC_ST_SUCCESS) {
    // other application code...
}
```

### CRC Enabled I/O

This driver provides two APIs (`Pmic_ioRxByte()` and `Pmic_ioTxByte()`)
which are used internally, but may be useful to end-users in cases where driver
feature support does not exist.

These APIs can be used to read from and write to any PMIC register and will
automatically perform the necessary CRC calculation and frame adjustments in
order to ensure successful communication.

See `include/pmic_io.h` for more information on these APIs.

### Watchdog

The watchdog module for the PMIC driver supports configuration and status
reporting for PMIC watchdog features, and supports calculation and response for
Q&A watchdog mode.

See `include/pmic_wdg.h` for more information on these APIs.

### IRQ Mask Control, Status Read, and Clear

The IRQ module for the PMIC driver supports masking (disable) and un-masking
(enable) of individual interrupt sources on the PMIC, supports reading the
status of all interrupts using an optimal algorithm based on the heirarchical
structure of the IRQs, and supports clearing individual IRQs as handled or all
at once.

See `include/pmic_irq.h` for more information on these APIs.

#### IRQ Status Read and Clear Example

A common pattern for end-user is to recieve an nINT interrupt on the MCU, check
IRQ status on the PMIC, handle relevant interrupts, and then clear these IRQ
sources. An example of how this can be done using the pmic-lld APIs is shown
below:

``` c
// Create IRQ status structure
Pmic_IrqStat_t irqStat;

// Reads all IRQ status registers (optimally, only if relevant), and populates 
// `irqStat` with information necessary for further processing
pmicStatus = Pmic_irqGetStat(&pmicHandle, &irqStat);

void HandleIrqNum(uint8 irqNum) {
    // User implemented function to handle IRQs as desired
}

if (pmicStatus == PMIC_ST_SUCCESS) {
    uint8_t irqFlagStat;
    uint8_t irqNum;

    do {
        irqFlagStat = Pmic_irqGetNextFlag(&irqStat, &irqNum);

        if (irqFlagStat == PMIC_ST_SUCCESS) {
            HandleIrqNum(irqNum);
            Pmic_irqClrFlag(&pmicHandle, irqNum);
        }
    } while (irqFlagStat == PMIC_ST_SUCCESS);
}
```
