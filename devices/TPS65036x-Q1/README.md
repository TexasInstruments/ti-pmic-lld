# Texas Instruments PMIC LLD for Derby / TPS65036x

This is a user's guide for the TPS65036x PMIC (Power Management Integrated Circuit)
low level driver (LLD). The PMIC LLD provides application programming interfaces
(APIs) that enable control/interaction with PMIC features, including, but not limited
to, regulators (BUCKs, LDOs), interrupt requests, error signal monitor (ESM), and
watchdog.

This PMIC LLD supports the following device(s) and their features and/or modules.

1. TPS65036x-Q1: Power Management IC for Safety-Relevant Applications

## How to Use PMIC LLD in Your Project

### Getting Started

To clone the repository into a specific folder:

    git clone git@github.com:TexasInstruments/ti-pmic-lld.git <DESIRED FOLDER NAME>

To clone the repository into your current directory:

    git clone git@github.com:TexasInstruments/ti-pmic-lld.git .

Once the repository has been cloned, check-out the TPS65036x-Q1 branch:

    git checkout device/Derby

#### Include PMIC LLD Within Your Project

To include the PMIC driver as part of your project, do the following:

- Set the `include/` directory to be part of the include path
- Set the `src/` directory to be part of the source path

Include "pmic.h" within your .c source file to have access to all PMIC LLD APIs.

#### Generating API Documentation

For more information on how to use each module, the header files are documented
using [Doxygen](https://www.doxygen.nl) syntax.

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

Most APIs provided by this driver expect to receive a `Pmic_CoreHandle_t` in
order to handle communication with the device. This handle should be created
through the use of the `Pmic_CoreCfg_t` structure in `pmic.h` and the
`Pmic_init()` API. Once created, it is recommended that the handle should not
be modified directly; otherwise the end-user will risk potential driver errors.

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

A full example of what this may look like for TPS65036X-Q1 is shown below:

```c
int32_t status;

// The handle should either be declared globally, or stored in a structure that
// can manage access throughout the application, it will need to be re-used
// often.
Pmic_CoreHandle_t pmicHandle;

Pmic_CoreCfg_t coreCfg = {
    .i2cAddr = 0x60U,
    .commHandle = &app_commHandle,
    .ioRead = &app_ioRead,
    .ioWrite = &app_ioWrite,
    .critSecStart = &app_critSecStart,
    .critSecStop = &app_critSecStop
};

status = Pmic_init(&pmicHandle, &coreCfg);

// Check the return code of Pmic_init(), if it is PMIC_ST_SUCCESS, the
// PmicHandle is now valid for use throughout the rest of the application
if (status == PMIC_ST_SUCCESS) {
    // other application code...
}
```

### CRC Enabled I/O

This driver provides two APIs (`Pmic_ioRxByte()` and `Pmic_ioTxByte()`)
which are used internally, but may be useful to end-users in cases where driver
feature support does not exist or direct register access is desired.

These APIs can be used to read from and write to any PMIC register and will
automatically perform the necessary CRC calculation and frame adjustments in
order to ensure successful communication.

See `include/pmic_io.h` for more information on these APIs.

### Watchdog (WDG)

The TPS65036x WDG helps monitor software errors that occur in the MCU.

See `include/pmic_wdg.h` for more information on the WDG module and its APIs.

### Error Signal Monitor (ESM)

The TPS65036x ESM helps monitor hardware errors that occur in the MCU.

See `include/pmic_esm.h` for more information on the ESM module and its APIs.

### Power (Regulator Control)

The TPS65036x PMIC features three buck regulators and one LDO regulator.

See `include/pmic_power.h` for more information on the power module and its APIs.

### Interrupt Request (IRQ)

The TPS65036x PMIC features fault/error detection and notification in the form of
IRQs.

See `include/pmic_irq.h` for more information on the IRQ module and its APIs.

### Other Features

Other PMIC features not pertaining to previously mentioned modules could be found
in the Core module.

These features include, but are not limited to, getting device information, register
lock/unlock, CRC enable/disable, and low power mode configurations.

See `include/pmic_core.h` for more information on the Core module and its APIs.
