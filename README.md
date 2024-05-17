/**
 *  @mainpage  PMIC LLD
 *
 *  This is an API guide for PMIC (Power Management Integrated Circuit) LLD (Low
 *  Level Driver). The PMIC LLD provides a set of low level APIs that aim to establish 
 *  the foundation of the end-user's software development with PMICs and jumpstart
 *  evaluation. 
 * 
 *  The PMIC LLD is designed to be MCU-agnostic and can be used in baremetal or RTOS 
 *  applications. With PMIC LLD, the end-user can manage the power of MCUs, SoCs, and/or 
 *  different components on embedded systems.
 * 
 *  PMIC LLD also provides a set of APIs for functional safety applications, such as 
 *  watchdog APIs, error signal monitor APIs, and IRQ APIs.  
 *
 *  PMIC LLD supports the following PMIC devices:
 *    1. TPS65036x
 *
 *  PMIC features that are accessible and configurable by PMIC LLD are as follows: 
 *      1. PMIC communication (such as communicating with or without CRC)
 *      3. interrupt requests (IRQ)
 *      5. Watchdog (WDG)
 *      6. Power (including BUCKs, LDOs, VMONs, thermal)
 *      7. Core (such as device state machine, LPM)
 *      7. and miscellaneous functions (such as writing to scratchpad registers)
 *
 *  The PMIC-LLD includes the following sub-modules/drivers:
 *
 *   - <b> CORE </b> (See @ref DRV_PMIC_CORE_MODULE) <br>
 *   - <b> IRQ </b> (See @ref DRV_PMIC_IRQ_MODULE) <br>
 *   - <b> POWER </b> (See @ref DRV_PMIC_POWER_MODULE) <br>
 *   - <b> WDG </b> (See @ref DRV_PMIC_WDG_MODULE) <br>
 *
 *  All above PMIC features/modules can be accessible and configurable through
 *  PMIC LLD. 
 */