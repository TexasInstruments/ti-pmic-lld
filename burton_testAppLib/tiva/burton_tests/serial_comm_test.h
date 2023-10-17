/**
 * \file serial_comm_test.h
 * \author John Bui (j-bui1@ti.com)
 * \brief Header file containing definitions and prototypes for serial communication testing
 * \version 1.0
 * \date 2023-10-11
 *
 * \copyright Copyright (c) 2023
 *
 */

#ifndef SERIAL_COMM_TEST_H
#define SERIAL_COMM_TEST_H

#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

#define BURTON_NUM_USER_OTP_REGS (uint8_t)(71)

/**
 * \brief Unity test to check if the driver's Receive Byte API is
 *        able to read correct values from Burton OTP-programmed user registers.
 *
 */
void test_Pmic_commIntf_recvByte_forCorrectReads(void);

/**
 * \brief Unity test to check if the PMIC handle's read function is
 *        able to read correct values from Burton OTP-programmed user registers
 *
 */
void test_pmicCoreHandle_PmicCommIoRead_forCorrectReads(void);

/**
 * \brief Unity test to check if the driver's send byte API is
 *        able to write the correct values to Burton OTP-programmed user registers.
 *
 */
void test_Pmic_commIntf_sendByte_forCorrectWrites(void);

/**
 * \brief Unity test to check if the PMIC handle's write function is
 *        able to write the correct values to Burton OTP-programmed user registers.
 *
 */
void test_pmicCoreHandle_PmicCommIoWrite_forCorrectWrites(void);

#ifdef __cplusplus
}

#endif /* __cplusplus */

#endif /* SERIAL_COMM_TEST_H */
