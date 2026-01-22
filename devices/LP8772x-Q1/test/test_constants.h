#ifndef TEST_CONSTANTS_H
#define TEST_CONSTANTS_H

/* Sentinel/Magic Values */
#define TEST_PMIC_INIT_MAGIC         ((uint32_t)0x504D4943U)  /* "PMIC" ASCII */
#define TEST_INVALID_MAGIC           ((uint32_t)0xDEADBEEFU)
#define TEST_DUMMY_HANDLE            ((uint32_t)0x12345678U)
#define TEST_INVALID_RSRC_SENTINEL   ((uint8_t)0xFFU)

/* Test Data Patterns */
#define TEST_PATTERN_AA              ((uint8_t)0xAAU)
#define TEST_PATTERN_55              ((uint8_t)0x55U)
#define TEST_PATTERN_A5              ((uint8_t)0xA5U)
#define TEST_PATTERN_5A              ((uint8_t)0x5AU)

/* Common Bit Masks */
#define TEST_MASK_LOW_NIBBLE         ((uint8_t)0x0FU)
#define TEST_MASK_HIGH_NIBBLE        ((uint8_t)0xF0U)
#define TEST_MASK_FULL_BYTE          ((uint8_t)0xFFU)

/* Invalid Parameter Values for Negative Tests */
#define TEST_INVALID_PARAM_99        ((uint8_t)99U)
#define TEST_INVALID_PARAM_255       ((uint8_t)255U)

/* Common Test Register Addresses */
#define TEST_REG_INT_TOP             ((uint8_t)0x46U)
#define TEST_REG_INT_FSM_ERR         ((uint8_t)0x50U)
#define TEST_REG_WD_ERR_STATUS       ((uint8_t)0x5EU)
#define TEST_REG_CONFIG_CRC_CONFIG   ((uint8_t)0x60U)
#define TEST_REG_CALCUL_CRC_1        ((uint8_t)0x61U)
#define TEST_REG_CALCUL_CRC_2        ((uint8_t)0x62U)

/* Bit Position Values (for assertions) */
#define TEST_BIT_4_VALUE             ((uint8_t)16U)   /* Bit 4 value: 1 << 4 */
#define TEST_BIT_7_VALUE             ((uint8_t)128U)  /* Bit 7 value: 1 << 7 */

#endif /* TEST_CONSTANTS_H */
