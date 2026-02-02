<div align="center">

<picture>
  <source media="(prefers-color-scheme: dark)" srcset="https://www.ti.com/content/dam/ticom/images/identities/ti-brand/ti-logo-hz-1c-white.svg" width="300">
  <img alt="Texas Instruments Logo" src="https://www.ti.com/content/dam/ticom/images/identities/ti-brand/ti-hz-2c-pos-rgb.svg" width="300">
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

This repository supports the following Power Management ICs:

| Part Number    | Description |
| -------------- | ----------- |
| TPS65036x-Q1   | Power Management IC for Safety-Relevant Applications |
| TPS6522x-Q1    | Four Buck Converters, Three LDOs, Three VMONs, Six GPIOs, ADC, Watchdog, and ESM |
| TPS65386x-Q1   | Power Management IC for Safety-Relevant Applications |
| LP8772x-Q1     | Three Buck Converters, one Linear Regulator and one Load Switch for AWR and IWR Radar Sensors |

All device-specific drivers are located in the `devices/` directory, with each device in its own subdirectory.

## Setup Instructions

### Clone the Repository

To start using this repository, clone it to your project location:

```shell
git clone https://github.com/TexasInstruments/ti-pmic-lld.git
```

Or clone into a specific directory:

```shell
git clone https://github.com/TexasInstruments/ti-pmic-lld.git <DESIRED_FOLDER_NAME>
```

### Access Device-Specific Drivers

All device drivers are organized in the `devices/` directory. Navigate to your target device:

```shell
cd devices/TPS65036x-Q1/
cd devices/TPS6522x-Q1/
cd devices/TPS65386x-Q1/
cd devices/LP8772x-Q1/
```

Each device directory contains:
- `README.md` - Device-specific documentation and API guide
- `include/` - Header files
- `src/` - Source implementation files
- `test/` - Comprehensive test suite
- `Makefile` - Build configuration

Refer to the device-specific README.md for detailed integration instructions.

### Alternative: Packaged Releases

Packaged releases can be found by navigating to
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
