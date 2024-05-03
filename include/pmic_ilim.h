/******************************************************************************
 * Copyright (c) 2024 Texas Instruments Incorporated - http://www.ti.com
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *    Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 *    Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the
 *    distribution.
 *
 *    Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *****************************************************************************/

/**
 *   @file    pmic_ilim.h
 *
 *   @brief   This file contains the default MACRO's and function definitions for
 *            PMIC ILIM configuration
 *
 */

#ifndef __PMIC_ILIM_H__
#define __PMIC_ILIM_H__

#include "pmic.h"
#include "pmic_core.h"
#include "pmic_types.h"
#include "pmic_io.h"

/**
 * @defgroup Pmic_ILIM PMIC ILIM Module
 * @{
 * @brief Contains definitions related to PMIC ILIM functionality.
 */
#define PMIC_ILIM_LDO1_VALID    (0U)
#define PMIC_ILIM_LDO2_VALID    (1U)
#define PMIC_ILIM_LDO3_VALID    (2U)
#define PMIC_ILIM_LDO4_VALID    (3U)
#define PMIC_ILIM_PLDO1_VALID   (4U)
#define PMIC_ILIM_PLDO2_VALID   (5U)
#define PMIC_ILIM_BB_AVG_VALID  (6U)

#define PMIC_ILIM_LDO1_VALID_SHIFT    (1U << PMIC_ILIM_LDO1_VALID)
#define PMIC_ILIM_LDO2_VALID_SHIFT    (1U << PMIC_ILIM_LDO2_VALID)
#define PMIC_ILIM_LDO3_VALID_SHIFT    (1U << PMIC_ILIM_LDO3_VALID)
#define PMIC_ILIM_LDO4_VALID_SHIFT    (1U << PMIC_ILIM_LDO4_VALID)
#define PMIC_ILIM_PLDO1_VALID_SHIFT   (1U << PMIC_ILIM_PLDO1_VALID)
#define PMIC_ILIM_PLDO2_VALID_SHIFT   (1U << PMIC_ILIM_PLDO2_VALID)
#define PMIC_ILIM_BB_AVG_VALID_SHIFT  (1U << PMIC_ILIM_BB_AVG_VALID)

/**
 * @defgroup PMIC_ILIMErrorReactConfig
 */
#define PMIC_ILIM_ERR_REACT_INT_ONLY           (0U)
#define PMIC_ILIM_ERR_REACT_INT_AND_DEACTIVATE (1U)

/**
 * @defgroup Pmic_ILIMDeglitchConfig
 */
#define PMIC_ILIM_DEGLITCH_10US (0U)
#define PMIC_ILIM_DEGLITCH_1MS  (1U)

typedef struct Pmic_ILIMLDOConfig {
    uint8_t errReact;
    uint8_t deglitch;
} Pmic_ILIMLDOConfig_t;

/*==========================================================================*/
/*                         Structures and Enums                             */
/*==========================================================================*/

/**
 * @defgroup Pmic_ILIMStructures PMIC ILIM Structures
 * @{
 * @ingroup Pmic_ILIM
 * @brief Contains structures used in the ILIM module of PMIC driver.
 */
typedef struct Pmic_ILIMConfig {
    uint8_t validParams;
    Pmic_ILIMLDOConfig_t ldo1;
    Pmic_ILIMLDOConfig_t ldo2;
    Pmic_ILIMLDOConfig_t ldo3;
    Pmic_ILIMLDOConfig_t ldo4;
    Pmic_ILIMLDOConfig_t pldo1;
    Pmic_ILIMLDOConfig_t pldo2;
} Pmic_ILIMConfig_t;

typedef struct Pmic_ILIMStatus {
    uint8_t validParams;
    uint8_t ldo1;
    uint8_t ldo2;
    uint8_t ldo3;
    uint8_t ldo4;
    uint8_t pldo1;
    uint8_t pldo2;
    uint8_t bbAvg;
} Pmic_ILIMStatus_t;

/**
 * @}
 */
/* End of Pmic_ILIMStructures */

/* ========================================================================== */
/*                          Function Prototypes                               */
/* ========================================================================== */

/**
 * @defgroup Pmic_ILIMFunctions PMIC ILIM Functions
 * @{
 * @ingroup Pmic_ILIM
 * @brief Contains functions used in the ILIM module of PMIC driver.
 */

/**
 * @brief Sets the ILIM configuration register with the provided ILIM configuration settings.
 * This function configures the ILIM configuration register with the settings specified
 * in the provided Pmic_ilimCfgReg_t structure pointed to by 'pPmicILIMConfig'.
 * It reads the current register value, modifies the necessary fields according to the
 * provided configuration settings, and writes back the updated value to the register.
 * The function also ensures that the critical section is entered before accessing
 * the PMIC to prevent potential race conditions.
 *
 * @param pPmicCoreHandle Pointer to the PMIC core handle.
 * @param pPmicILIMConfig Pointer to the ILIM configuration structure.
 *
 * @return Returns PMIC_ST_SUCCESS if the operation is successful; otherwise, an error code.
 *
 * @ingroup Pmic_ILIMFunctions
 */
int32_t Pmic_SetILIMConfig(Pmic_CoreHandle_t * pPmicCoreHandle,
                           Pmic_ILIMConfig_t * pPmicILIMConfig);


/**
 * @brief Clears the ILIM error status bits in the ILIM status register.
 * This function clears the ILIM error status bits in the ILIM status register based on
 * the provided configuration in the Pmic_ilimStatReg_t structure pointed to by 'pPmicILIMStat'.
 * It reads the current register value, modifies the necessary fields according to the
 * provided configuration settings, and writes back the updated value to the register.
 * The function also ensures that the critical section is entered before accessing
 * the PMIC to prevent potential race conditions.
 *
 * @param pPmicCoreHandle Pointer to the PMIC core handle.
 * @param pPmicILIMStat Pointer to the ILIM status register configuration structure.
 *
 * @return Returns PMIC_ST_SUCCESS if the operation is successful; otherwise, an error code.
 *
 * @ingroup Pmic_ILIMFunctions
 */
int32_t Pmic_ClearILIMErrStat(Pmic_CoreHandle_t *pPmicCoreHandle,
                              Pmic_ILIMStatus_t *pPmicILIMStat);

/**
 * @brief Retrieves the ILIM configuration from the ILIM configuration register.
 * This function reads the ILIM configuration register from the PMIC via the communication
 * interface specified in the PMIC core handle 'pPmicCoreHandle'. It then extracts the
 * individual ILIM configuration settings for PLDO2, PLDO1, LDO4, LDO3, LDO2, and LDO1
 * from the register data and stores them in the provided Pmic_ilimCfgReg_t structure
 * pointed to by 'pPmicILIMConfig'. The function ensures the critical section is entered
 * before accessing the PMIC to prevent potential race conditions.
 *
 * @param pPmicCoreHandle Pointer to the PMIC core handle.
 * @param pPmicILIMConfig Pointer to the ILIM configuration structure to store the retrieved values.
 *
 * @return Returns PMIC_ST_SUCCESS if the operation is successful; otherwise, an error code.
 *
 * @ingroup Pmic_ILIMFunctions
 */
int32_t Pmic_GetILIMConfig(Pmic_CoreHandle_t *pPmicCoreHandle,
                           Pmic_ILIMConfig_t *pPmicILIMConfig);

/**
 * @brief Retrieves the ILIM (Current Limit) error status from the ILIM status register.
 * This function reads the ILIM status register from the PMIC via the communication
 * interface specified in the PMIC core handle 'pPmicCoreHandle'. It then extracts the
 * individual ILIM error status for BB Average, PLDO2, PLDO1, LDO4, LDO3, LDO2, and LDO1
 * from the register data and stores them in the provided Pmic_ilimStatReg_t structure
 * pointed to by 'pPmicILIMStat'. The function ensures the critical section is entered
 * before accessing the PMIC to prevent potential race conditions.
 *
 * @param pPmicCoreHandle Pointer to the PMIC core handle.
 * @param pPmicILIMStat Pointer to the ILIM error status structure to store the retrieved values.
 *
 * @return Returns PMIC_ST_SUCCESS if the operation is successful; otherwise, an error code.
 *
 * @ingroup Pmic_ILIMFunctions
 */
int32_t Pmic_GetILIMErrStat(Pmic_CoreHandle_t *pPmicCoreHandle,
                            Pmic_ILIMStatus_t *pPmicILIMStat);


/**
 * @}
 */
/* End of Pmic_ILIMFunctions */

/**
 * @}
 */
/* End of Pmic_ILIM */

#endif /* __PMIC_ILIM_H__ */
