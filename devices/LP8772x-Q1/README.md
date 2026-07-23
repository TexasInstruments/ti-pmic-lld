# Texas Instruments PMIC Low-Level Driver (LLD) for LP8772x-Q1

This is an API guide for PMIC (Power Management Integrated Circuit) Driver. PMIC
Driver is designed to power up different components on the embedded boards or
provide supply to MCU (Micro Controller Unit) or SoC (System on chip) using
APIs provided in the guide.

The PMIC Driver supports below mentioned PMIC devices and their features or
Modules.

Supported PMIC Devices are:

1. LP8772x-Q1: Three Buck Converters, one Linear Regulator and one Load Switch
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

Once the repository has been cloned, navigate to the LP8772x-Q1 device directory:

    cd devices/LP8772x-Q1/

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

All APIs provided by this driver expect to receive a `Pmic_Handle_t` in
order to handle communication with the device. This handle should be created
through the use of the `Pmic_HandleCfg_t` structure in `pmic.h` and the
`Pmic_init()` API.

In order to successfully create a handle, the user will need to provide an
implementation for 5 functions detailed below which inform the driver how to
operate on the specific platform.

##### PMIC Handle User Functions: Critical Section Start/Stop

When constructing `Pmic_HandleCfg_t`, two functions need to be provided for
critical section management. These functions are called by the driver before
and after accessing shared resources.

The callbacks accept a `resource` parameter identifying which resource needs protection:

| Value | Macro | Description |
|-------|-------|-------------|
| 0 | `PMIC_COMMUNICATION` | Protects I2C/SPI bus access during register read/write |
| 1 | `PMIC_DIAGNOSTIC` | Protects diagnostic counters and overflow flags |

**Simple Implementation (Single Mutex)**

For most applications, a single mutex is sufficient:

```c
static pthread_mutex_t g_pmicMutex = PTHREAD_MUTEX_INITIALIZER;

void App_CriticalSectionStart(uint8_t resource)
{
    (void)resource;  // Single mutex for all resources
    pthread_mutex_lock(&g_pmicMutex);
}

void App_CriticalSectionStop(uint8_t resource)
{
    (void)resource;
    pthread_mutex_unlock(&g_pmicMutex);
}
```

**Advanced Implementation (Per-Resource Mutexes)**

For better concurrency in multi-threaded applications, use separate mutexes:

```c
static pthread_mutex_t g_commMutex = PTHREAD_MUTEX_INITIALIZER;
static pthread_mutex_t g_diagMutex = PTHREAD_MUTEX_INITIALIZER;

void App_CriticalSectionStart(uint8_t resource)
{
    switch (resource) {
        case PMIC_COMMUNICATION:
            pthread_mutex_lock(&g_commMutex);
            break;
        case PMIC_DIAGNOSTIC:
            pthread_mutex_lock(&g_diagMutex);
            break;
        default:
            // Unknown resource - handle error or use fallback
            break;
    }
}

void App_CriticalSectionStop(uint8_t resource)
{
    switch (resource) {
        case PMIC_COMMUNICATION:
            pthread_mutex_unlock(&g_commMutex);
            break;
        case PMIC_DIAGNOSTIC:
            pthread_mutex_unlock(&g_diagMutex);
            break;
        default:
            break;
    }
}
```

**Bare-Metal Implementation (Interrupt Disable)**

For bare-metal systems without an RTOS:

```c
static uint32_t g_intState;

void App_CriticalSectionStart(uint8_t resource)
{
    (void)resource;
    g_intState = __get_PRIMASK();
    __disable_irq();
}

void App_CriticalSectionStop(uint8_t resource)
{
    (void)resource;
    __set_PRIMASK(g_intState);
}
```

##### PMIC Handle User Functions: Communications I/O Read/Write

When constructing `Pmic_HandleCfg_t`, two functions need to be provided in order
for the PMIC to know how to read and write over the desired communications
channel (I2C or SPI, typically). The specific implementation of these functions
is platform dependent, the chosen processor likely has an SDK which provides
functions that match relatively closely.

Within the `Pmic_HandleCfg_t` structure, these two functions are:

```c
{
    .ioRead = <your I/O read function>,
    .ioWrite = <your I/O write function>,
}
```

##### PMIC Handle User Functions: Timer Wait

A timer wait function is required to support delays between retry attempts when
communication errors occur. The driver calls this function to wait a specified
number of milliseconds before retrying a failed operation.

```c
{
    .timerWaitMs = App_TimerWaitMs,
}
```

**RTOS Implementation**

```c
void App_TimerWaitMs(uint32_t ms)
{
    // FreeRTOS example
    vTaskDelay(pdMS_TO_TICKS(ms));

    // Or POSIX example
    // usleep(ms * 1000U);
}
```

**Bare-Metal Implementation**

```c
void App_TimerWaitMs(uint32_t ms)
{
    // Hardware timer-based delay (recommended)
    Timer_delay_ms(ms);

    // Or busy-wait (not recommended for long delays)
    // volatile uint32_t count = ms * CYCLES_PER_MS;
    // while (count-- > 0U) { }
}
```

**Important:** The `timerWaitMs` function should be non-blocking with respect to
other system tasks when using an RTOS. Avoid busy-wait implementations in
multi-tasking environments.

##### PMIC Handle Configuration: Retry Mechanism

The driver supports automatic retry of failed I2C/SPI communications. When a
communication error occurs (bus error or CRC failure), the driver waits
`retryIntervalMs` milliseconds and retries the operation, up to `retryCnt` times.

| Parameter | Description | Typical Value |
|-----------|-------------|---------------|
| `retryCnt` | Maximum retry attempts (0 = no retries) | 3 |
| `retryIntervalMs` | Delay between attempts in milliseconds | 10 |

```c
{
    .retryCnt = 3U,           // Retry up to 3 times on failure
    .retryIntervalMs = 10U,   // Wait 10ms between attempts
}
```

**When to Use Retries:**
- Enable retries for systems with potentially noisy communication buses
- Set `retryCnt = 0` for deterministic timing requirements
- Increase `retryIntervalMs` if the bus requires recovery time after errors

**Monitoring Retries:**

Use the diagnostic APIs to monitor retry statistics:

```c
uint32_t retryCnt;
bool overflow;

Pmic_getRetryCnt(&pmicHandle, &retryCnt);
Pmic_getRetryCntOverflow(&pmicHandle, &overflow);

if (overflow) {
    // Retry counter reached threshold - possible hardware issue
    printf("Warning: %u retries occurred, threshold reached\n", retryCnt);
    Pmic_clrRetryCntOverflow(&pmicHandle);
}
```

##### Finalizing Initialization

Once the `Pmic_HandleCfg_t` structure has been initialized with the necessary
information, the user should call `Pmic_init()` in order to convert the
`Pmic_HandleCfg_t` into a `Pmic_Handle_t` which will be used with the rest of
the driver APIs.

A full example of what this may look like for LP8772x-Q1 is shown below:

```c
int32_t status;

// The handle should either be declared globally, or stored in a structure that
// can manage access throughout the application, it will need to be re-used often.
Pmic_Handle_t pmicHandle;

Pmic_HandleCfg_t config = {
    .validParams = (
        PMIC_CFG_INIT_COMM_MODE_VALID              |
        PMIC_CFG_INIT_CRC_ENABLE_VALID             |
        PMIC_CFG_INIT_CONFIG_CRC_ENABLE_VALID      |
        PMIC_CFG_INIT_I2C_ADDR0_VALID              |
        PMIC_CFG_INIT_COMM_HANDLE_0_VALID          |
        PMIC_CFG_INIT_IO_READ_VALID                |
        PMIC_CFG_INIT_IO_WRITE_VALID               |
        PMIC_CFG_INIT_CRITICAL_SECTION_START_VALID |
        PMIC_CFG_INIT_CRITICAL_SECTION_STOP_VALID  |
        PMIC_CFG_INIT_TIMER_WAIT_MS_VALID          |
        PMIC_CFG_INIT_RETRY_CNT_VALID              |
        PMIC_CFG_INIT_RETRY_INTERVAL_MS_VALID
    ),
    .commMode = PMIC_INTF_I2C_SINGLE,
    .crcEnable = PMIC_ENABLE,
    .configCrcEnable = PMIC_ENABLE,
    .i2cAddr0 = 0x48U,  // Device I2C address
    .commHandle0 = &i2cHandle,
    .ioRead = App_PmicIoRead,
    .ioWrite = App_PmicIoWrite,
    .criticalSectionStart = App_CriticalSectionStart,
    .criticalSectionStop = App_CriticalSectionStop,
    .timerWaitMs = App_TimerWaitMs,
    .retryCnt = 3U,
    .retryIntervalMs = 10U,
};

status = Pmic_init(&pmicHandle, &config);

if (status == PMIC_ST_SUCCESS) {
    // pmicHandle is now valid for use with all PMIC APIs
}
```

#### Using validParams

The `validParams` field in `Pmic_HandleCfg_t` allows selective initialization of handle configuration parameters. Each bit in this field corresponds to a structure member:

- Set a bit to 1 to indicate the corresponding parameter is valid and should be processed
- Set a bit to 0 to indicate the corresponding parameter is invalid and should be ignored

For LP8772x-Q1, the following parameters are typically required:
- `PMIC_CFG_INIT_COMM_MODE_VALID`
- `PMIC_CFG_INIT_CRC_ENABLE_VALID`
- `PMIC_CFG_INIT_CONFIG_CRC_ENABLE_VALID`
- `PMIC_CFG_INIT_I2C_ADDR0_VALID`
- `PMIC_CFG_INIT_COMM_HANDLE_0_VALID`
- `PMIC_CFG_INIT_IO_READ_VALID`
- `PMIC_CFG_INIT_IO_WRITE_VALID`
- `PMIC_CFG_INIT_CRITICAL_SECTION_START_VALID`
- `PMIC_CFG_INIT_CRITICAL_SECTION_STOP_VALID`
- `PMIC_CFG_INIT_TIMER_WAIT_MS_VALID`
- `PMIC_CFG_INIT_RETRY_CNT_VALID`
- `PMIC_CFG_INIT_RETRY_INTERVAL_MS_VALID`

**CRC Configuration**: LP8772x-Q1 requires explicit CRC configuration via `crcEnable` and `configCrcEnable` fields. Set both to `PMIC_ENABLE` for proper operation. CRC (Cyclic Redundancy Check) ensures communication integrity between the MCU and PMIC.

The `PMIC_CFG_INIT_IRQ_RESPONSE_CALLBACK_VALID` bit should only be set if you are implementing WDG Q&A mode functionality and have not tied a GPIO to the nINT pin of the PMIC. This callback enables IRQ detection without using the nINT signal line.

Alternatively, use the convenience macro `PMIC_ALL_VALID` to enable all parameters.

### CRC Enabled I/O

This driver provides two APIs (`Pmic_ioRxByte()` and `Pmic_ioTxByte()`)
which are used internally, but may be useful to end-users in cases where driver
feature support does not exist.

These APIs can be used to read from and write to any PMIC register and will
automatically perform the necessary CRC calculation and frame adjustments in
order to ensure successful communication.

See `include/pmic_io.h` for more information on these APIs.

### Watchdog (WDG)

The LP8772x-Q1 watchdog module supports configuration and status reporting for PMIC watchdog features, including trigger mode, fail count threshold, and Q&A mode.

See `include/pmic_wdg.h` for more information on the WDG module and its APIs.

### IRQ Mask Control, Status Read, and Clear

The IRQ module for the PMIC driver supports masking (disable) and un-masking
(enable) of individual interrupt sources on the PMIC, supports reading the
status of all interrupts using an optimal algorithm based on the hierarchical
structure of the IRQs, and supports clearing individual IRQs as handled or all
at once.

See `include/pmic_irq.h` for more information on these APIs.

#### IRQ Status Read and Clear Example

A common pattern for end-user is to receive an nINT interrupt on the MCU, check
IRQ status on the PMIC, handle relevant interrupts, and then clear these IRQ
sources. An example of how this can be done using the pmic-lld APIs is shown
below:

``` c
// Create IRQ status structure
Pmic_IrqStat_t irqStat;

// Reads all IRQ status registers (optimally, only if relevant), and populates
// `irqStat` with information necessary for further processing
pmicStatus = Pmic_irqGetStatus(&pmicHandle, &irqStat);

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

### Error Signal Monitor (ESM)

The Error Signal Monitor (ESM) module monitors MCU error signals and provides
configurable responses when errors are detected. The ESM can operate in two modes:

| Mode | Description |
|------|-------------|
| `PMIC_ESM_MODE_LEVEL` | Level-based error detection |
| `PMIC_ESM_MODE_PWM` | PWM signal monitoring with timing constraints |

See `include/pmic_esm.h` for complete API documentation.

#### ESM Configuration Example

```c
// Configure ESM in level mode with error threshold
Pmic_EsmCfg_t esmCfg = {
    .validParams = (PMIC_CFG_ESM_MODE_VALID | PMIC_CFG_ESM_ERR_CNT_THR_VALID |
                    PMIC_CFG_ESM_DELAY1_VALID | PMIC_CFG_ESM_DELAY2_VALID),
    .mode = PMIC_ESM_MODE_LEVEL,
    .errCntThr = 3U,    // Error threshold (0-15)
    .delay1 = 0x10U,    // Delay 1 value
    .delay2 = 0x20U,    // Delay 2 value
};

status = Pmic_esmSetCfg(&pmicHandle, &esmCfg);
```

#### PWM Mode Configuration

For PWM mode, configure timing constraints for high/low signal periods:

```c
Pmic_EsmCfg_t esmCfg = {
    .validParams = (PMIC_CFG_ESM_MODE_VALID | PMIC_CFG_ESM_LMIN_VALID |
                    PMIC_CFG_ESM_LMAX_VALID | PMIC_CFG_ESM_HMIN_VALID | PMIC_CFG_ESM_HMAX_VALID),
    .mode = PMIC_ESM_MODE_PWM,
    .lmin = 0x10U,  // Minimum low time
    .lmax = 0x80U,  // Maximum low time
    .hmin = 0x10U,  // Minimum high time
    .hmax = 0x80U,  // Maximum high time
};

status = Pmic_esmSetCfg(&pmicHandle, &esmCfg);
```

#### Enabling and Starting ESM

```c
// Enable the ESM module
status = Pmic_esmSetEnableState(&pmicHandle, PMIC_ENABLE);

// Start ESM monitoring
if (status == PMIC_ST_SUCCESS) {
    status = Pmic_esmStart(&pmicHandle);
}
```

#### Monitoring ESM Status

```c
Pmic_EsmStatus_t esmStat = {
    .validParams = PMIC_ESM_STATUS_ALL_VALID
};

status = Pmic_esmGetStatus(&pmicHandle, &esmStat);

if (status == PMIC_ST_SUCCESS) {
    if (esmStat.rstInt) {
        printf("ESM reset interrupt detected\n");
    }
    if (esmStat.failInt) {
        printf("ESM fail interrupt detected\n");
    }
    if (esmStat.pinInt) {
        printf("ESM pin interrupt detected\n");
    }

    // Clear handled status flags
    status = Pmic_esmClrStatus(&pmicHandle, &esmStat);
}
```

#### Reading ESM Error Counter

```c
uint8_t errCnt;
status = Pmic_esmGetErrCnt(&pmicHandle, &errCnt);

if (status == PMIC_ST_SUCCESS) {
    printf("ESM error count: %u\n", errCnt);
}
```

### System Diagnostics

The driver maintains diagnostic information to help monitor communication health
and track error occurrences. This is useful for:

- Detecting intermittent communication issues
- Monitoring system reliability over time
- Debugging field failures

See `include/pmic_common.h` for complete API documentation.

#### Diagnostic Structure

Each diagnostic entry tracks:
- `code`: The error/warning code being tracked
- `cnt`: Number of times this error occurred
- `flag`: Whether the counter reached its overflow threshold

#### Querying Diagnostics

```c
// Query diagnostic info for a specific error
Pmic_Diagnostic_t diag = {
    .validParams = PMIC_COMMON_DIAGNOSTIC_VALID_ALL,
    .code = PMIC_ST_ERR_I2C_COMM_FAIL
};

status = Pmic_getDiagnostic(&pmicHandle, &diag);

if (status == PMIC_ST_SUCCESS) {
    printf("I2C comm failures: %u (overflow: %s)\n",
           diag.cnt, diag.flag ? "yes" : "no");
}
```

#### Querying Multiple Diagnostics

```c
Pmic_Diagnostic_t diags[3] = {
    { .validParams = PMIC_COMMON_DIAGNOSTIC_VALID_ALL, .code = PMIC_ST_ERR_I2C_COMM_FAIL },
    { .validParams = PMIC_COMMON_DIAGNOSTIC_VALID_ALL, .code = PMIC_ST_ERR_DATA_IO_CRC },
    { .validParams = PMIC_COMMON_DIAGNOSTIC_VALID_ALL, .code = PMIC_ST_ERR_INV_PARAM }
};

status = Pmic_getDiagnostics(&pmicHandle, diags, 3U);

if (status == PMIC_ST_SUCCESS) {
    for (uint8_t i = 0U; i < 3U; i++) {
        printf("Error 0x%08X: count=%u, overflow=%s\n",
               diags[i].code, diags[i].cnt, diags[i].flag ? "yes" : "no");
    }
}
```

#### Clearing Diagnostics

```c
// Clear a specific diagnostic
Pmic_Diagnostic_t diag = {
    .validParams = PMIC_COMMON_DIAGNOSTIC_VALID_ALL,
    .code = PMIC_ST_ERR_I2C_COMM_FAIL
};
Pmic_clrDiagnostic(&pmicHandle, &diag);

// Or clear all diagnostics
Pmic_clrDiagnosticsAll(&pmicHandle);
```

#### Monitoring Retry Statistics

```c
uint32_t retryCnt;
bool overflow;

// Get total retry count
Pmic_getRetryCnt(&pmicHandle, &retryCnt);
printf("Total communication retries: %u\n", retryCnt);

// Check if retry threshold was reached
Pmic_getRetryCntOverflow(&pmicHandle, &overflow);
if (overflow) {
    printf("Warning: Retry threshold reached - check hardware\n");
    Pmic_clrRetryCntOverflow(&pmicHandle);
}

// Reset retry counter after logging
Pmic_clrRetryCnt(&pmicHandle);
```
