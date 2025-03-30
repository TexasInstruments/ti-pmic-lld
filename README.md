<div align="center">

<picture>
  <source media="(prefers-color-scheme: dark)" srcset="https://www.ti.com/etc/designs/ti/images/ui/ic-logo.svg" width="300">
  <img alt="Texas Instruments Logo" src="https://www.ti.com/etc/designs/ti/images/ui/ic-logo.svg" width="300">
</picture>

# Texas Instruments PMIC Low-Level Drivers (pmic-lld)

PMIC LLD is designed to support the control and configuration of various Texas
Instruments power management ICs (PMICs) in a platform, processor, and OS
agnostic manner.

[Summary](#summary) | [Features](#features) | [Supported Devices](#supported-devices) | [Setup Instructions](#setup-instructions) | [Licensing](#licensing) | [Contributions](#contributions) | [Developer Resources](developer-resources)

</div>

## Summary

PMICs supported by this driver are intended for automotive and industrial
applications and this driver is designed in compliance with the ISO 26262
functional safety standard.

Architecturally, PMIC LLD provides a well-defined API that abstracts low-level
details of the PMIC, allowing users to configure and control device features
without the need to worry about register and bit-field level details.

## Supported Devices

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

## Setup Instructions

To start using this repository, find the correct branch for the target device
from the list above and clone it to your project location. A single branch can
be cloned using the git-clone `--single-branch` argument.

For example, to clone the support branch for TPS653860xx:

``` shell
git clone -b device/blackbird --single-branch https://github.com/TexasInstruments/ti-pmic-lld.git
```

Alternatively, packaged releases can be found by navigating to
[Releases](https://github.com/TexasInstruments/ti-pmic-lld/releases) and
identifying the latest release for the target device. From here, expand the
"Assets" section of the release and find downloadable source code for each
release.

For further instructions on integrating the driver into a larger project, refer
to the device specific README.md after downloading the source code, or refer to
the User Guide available from TI on request.

## Licensing

See [LICENSE.md](LICENSE.md).

## Contributions

For information on contributing this repo, please refer to
[CONTRIBUTING.md](CONTRIBUTING.md) and [CODE_OF_CONDUCT.md](CODE_OF_CONDUCT.md) .

---

## Developer Resources

[TI E2E design support forums](https://e2e.ti.com) | [Learn about software development at TI](https://www.ti.com/design-development/software-development.html) | [Training Academies](https://www.ti.com/design-development/ti-developer-zone.html#ti-developer-zone-tab-1) | [TI Developer Zone](https://dev.ti.com/)
