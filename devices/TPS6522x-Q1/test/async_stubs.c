/**
 * @file async_stubs.c
 * @brief Stub implementations of async test helper functions for BUILD=host mode
 *
 * These functions are only fully implemented in BUILD=mock mode.
 * For host builds, they return error codes to indicate unsupported operation.
 */

#include "pmic.h"

#ifndef BUILD_MOCK

/**
 * @brief Stub for asyncRxStart in host mode
 * @return PMIC_ST_ERR_INV_PARAM to indicate unsupported operation
 */
int32_t test_pmic_asyncRxStart(const Pmic_Handle_t *handle, uint8_t page,
                               uint8_t regAddr, uint8_t *buffer, uint8_t bufLen)
{
    (void)handle;
    (void)page;
    (void)regAddr;
    (void)buffer;
    (void)bufLen;
    return PMIC_ST_ERR_INV_PARAM;
}

/**
 * @brief Stub for asyncTxStart in host mode
 * @return PMIC_ST_ERR_INV_PARAM to indicate unsupported operation
 */
int32_t test_pmic_asyncTxStart(const Pmic_Handle_t *handle, uint8_t page,
                               uint8_t regAddr, const uint8_t *buffer, uint8_t bufLen)
{
    (void)handle;
    (void)page;
    (void)regAddr;
    (void)buffer;
    (void)bufLen;
    return PMIC_ST_ERR_INV_PARAM;
}

/**
 * @brief Stub for asyncRxAwait in host mode
 * @return PMIC_ST_ERR_INV_PARAM to indicate unsupported operation
 */
int32_t test_pmic_asyncRxAwait(const Pmic_Handle_t *handle)
{
    (void)handle;
    return PMIC_ST_ERR_INV_PARAM;
}

/**
 * @brief Stub for asyncTxAwait in host mode
 * @return PMIC_ST_ERR_INV_PARAM to indicate unsupported operation
 */
int32_t test_pmic_asyncTxAwait(const Pmic_Handle_t *handle)
{
    (void)handle;
    return PMIC_ST_ERR_INV_PARAM;
}

#endif /* !BUILD_MOCK */
