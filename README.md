# Texas Instruments PMIC Low-Level Drivers (pmic-lld)

PMIC LLD is designed to support the control and configuration of various Texas
Instruments power management ICs (PMICs) in a platform, processor, and OS
agnostic manner.

PMICs supported by this driver are intended for automotive and industrial
applications and this driver is designed in compliance with the ISO 26262
functional safety standard.

Architecturally, PMIC LLD provides a well-defined API that abstracts low-level
details of the PMIC, allowing users to configure and control device features
without the need to worry about register and bit-field level details.

## Device Support Branches

Device support is subdivided based on feature set:

| Part Number | Friendly Name | Support Branch           |
| ----------- | ------------- | ------------------------ |
| TPS6594x    | Leo           | `device/leo-hera-burton` |
| LP8764x     | Hera          | `device/leo-hera-burton` |
| TPS6522x    | Burton        | `device/leo-hera-burton` |
| TPS653860xx | Blackbird     | `device/blackbird`       |
| LP8772X     | Coach         | `device/coach`           |
| TPS65036x   | Derby         | `device/derby`           |
| TPS65385xx  | Green Hornet  | `device/green-hornet`    |

## Contributing

Contributions are accepted through GitHub pull requests, Texas Instruments
reserves the right to modify or reject contributions for any reason.

See CODE_OF_CONDUCT.md and CONTRIBUTING.md for more information.
