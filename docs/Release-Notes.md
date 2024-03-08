# Release Notes - Milestone 4.2

## Overview
This release includes several new features, enhancements, and bug fixes to improve the overall functionality and stability of the application.

- **Release Date:** February 17, 2024

## New Features
- Added support for ESM Control [commit](3d33a0e5ec397b7f1fbdb6a20ebbcbbbcde1bf14)
- Introduced Safe State Control functionality [commit](72aaaf9eeed91dbeb250994ccbcf89620d5ecf66)
- Implemented Regulator Control feature [commit](1f7dfe72cebac098d1bd48e8b4090a8683efef50)
- Integrated Low IQ Timer Control [commit](d10c2ed93b38e033ac619a00ddb3b14e3ea1b536)
- Added Watchdog Timer Service [commit](c73ea3bd9f25c32d4b2b2fbc1f261d45cc9fe20d)

## Enhancements
- Fixed all the MISRA M4.1 review comments
- Added Doxygen comments for the PMIC source driver code
- Organized test-related APIs into a new folder under the "tests" folder named "tps65386x_tests"
- Created separate Doxygen documentation for the source code and tests

## Bug Fixes
- N/A

## Known Issues
- N/A

For more details, on the source code please refer to the [full changelog](efdf28c0eb32e9813c5e9140776b55a2ffa38788).
For more details, on the test code please refer to the [full changelog](d4cdfd8dcbedd9d7837c20e227ad43e10580f17d).



# Release Notes - Milestone 5.0

## Overview
This release includes several new features, enhancements, and bug fixes to improve the overall functionality and stability of the application.

- **Release Date:** March 8, 2024

## New Features
- Introduced Diagnostic Control Functionality [commit](54596a3eb9abd3747f4693f24257c889283028aa)
- Added support for VMON Level Control [commit](ccdb6d383d6de7665d85f8343cfc1a1371cb7b0e)
- Implemented support for ILIM Functionality [commit](cea702fa053d2701cae6c24f45056357a309cba0)

## Enhancements
- Fixed the M4.2 review comments
- Added Doxygen comments for the PMIC source driver code
- Doxygen for PMIC Core and ESM has been segregated into groups for easy user access, rest modules will be finished in upcoming release
- Created separate Doxygen documentation for the source code and tests

## Bug Fixes
- ESM Test cases had a redundant esmType variable which has been removed

## Known Issues
- N/A
