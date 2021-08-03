OTA feature to reconfigure NVM bits
=====================================
1. This feature is validated by PMIC Apps team
2. Point of contact for any queries related to this feature - Sterzik, Chris
3. Please find the steps shared by PMIC Apps team to test the OTA feature to reconfigure NVM bits

The GUI is located at https://dev.ti.com/gallery/view/PMIC/Scalable-PMICs-GUI/ver/2.0.0/ and a description for launching the scripting tool is found at https://www.ti.com/lit/ug/slvubt8a/slvubt8a.pdf#page=58.  PMIC Apps team used the GUI to also determine the CRC value changes in the process below. 

a. Power up PMIC with enable pulled low to prevent device going to active state.  Active state performs several register writes and changes the CRC so pulling enable low keeps the number of changes limited to those performed in the wait for enable sequence.
b. ‘Undo’ all register writes performed in wait4enable sequence – return registers to their default settings.
c. Perform CRC update and record
d. Make user register changes
e. Perform CRC update and record – in this example script(leo2p0OTA_VVCM_debug.txt) only the BUCK1_VOUT2 was changed so only the lower bytes of the crc was impacted (REGMAP_USER_INCLUDE/EXCLUDE_PRESIST_CRC16_LOW).
f. Construct OTA based upon values.
 
