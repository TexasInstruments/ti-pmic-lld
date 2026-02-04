/******************************************************************************
 * Copyright (c) 2026 Texas Instruments Incorporated - http://www.ti.com
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


#ifndef IRQ_TEST_H
#define IRQ_TEST_H

/* ========================================================================== */
/*                             Macros & Typedefs                              */
/* ========================================================================== */

/* ======================================================================== */
/*              Test APIs: irqSetMask, irqSetMasks, irqGetMask              */
/* ======================================================================== */
#define IRQ_TEST_POS_IRQSETGETMASK() \
    PLATFORM_RUN_TEST(test_pos_irq_irqSetGetMask_single); \
    PLATFORM_RUN_TEST(test_pos_irq_irqSetMasks_multiple); \
    PLATFORM_RUN_TEST(test_pos_irq_irqSetMask_allMaskable); \
    PLATFORM_RUN_TEST(test_pos_irq_irqSetGetMask_gpioInt); \
    PLATFORM_RUN_TEST(test_pos_irq_irqSetGetMask_powerUvov); \
    PLATFORM_RUN_TEST(test_pos_irq_irqSetGetMask_esmInt); \
    PLATFORM_RUN_TEST(test_pos_irq_irqSetGetMask_fsmErrorInt); \
    PLATFORM_RUN_TEST(test_pos_irq_irqSetGetMask_miscInt); \
    PLATFORM_RUN_TEST(test_pos_irq_irqSetGetMask_thermalWarning); \
    PLATFORM_RUN_TEST(test_pos_irq_irqSetGetMask_adcConvReady); \
    PLATFORM_RUN_TEST(test_pos_irq_irqSetGetMask_pushButton); \
    PLATFORM_RUN_TEST(test_pos_irq_irqGetMask_nonMaskable); \
    PLATFORM_RUN_TEST(test_pos_irq_irqSetMask_specific); \
    PLATFORM_RUN_TEST(test_pos_irq_irqSetMask_unmaskSpecific)

#define IRQ_TEST_NEG_IRQSETGETMASK() \
    PLATFORM_RUN_TEST(test_neg_irq_irqSetMask_nullHandle); \
    PLATFORM_RUN_TEST(test_neg_irq_irqSetMask_invalidIrqNum); \
    PLATFORM_RUN_TEST(test_neg_irq_irqSetMask_nonMaskable); \
    PLATFORM_RUN_TEST(test_neg_irq_irqSetMask_invalidIrqNumBeyondMax); \
    PLATFORM_RUN_TEST(test_neg_irq_irqSetMasks_nullHandle); \
    PLATFORM_RUN_TEST(test_neg_irq_irqSetMasks_nullIrqMasks); \
    PLATFORM_RUN_TEST(test_neg_irq_irqSetMasks_zeroCount); \
    PLATFORM_RUN_TEST(test_neg_irq_irqGetMask_nullHandle); \
    PLATFORM_RUN_TEST(test_neg_irq_irqGetMask_nullIrqMasks); \
    PLATFORM_RUN_TEST(test_neg_irq_irqGetMask_zeroCount); \
    PLATFORM_RUN_TEST(test_neg_irq_irqGetMask_invalidIrqInArray)

/* Test: TC-IRQ-0010 */
#define IRQ_TEST_IRQSETGETMASK() \
    IRQ_TEST_POS_IRQSETGETMASK(); \
    IRQ_TEST_NEG_IRQSETGETMASK()

/* ======================================================================== */
/*                         Test APIs: irqGetStatus                          */
/* ======================================================================== */
#define IRQ_TEST_POS_IRQGETSTATUS() \
    PLATFORM_RUN_TEST(test_pos_irq_irqGetStatus_read); \
    PLATFORM_RUN_TEST(test_pos_irq_irqGetStatus_withActiveFlags); \
    PLATFORM_RUN_TEST(test_pos_irq_irqGetStatus_withSetFlag)

#define IRQ_TEST_NEG_IRQGETSTATUS() \
    PLATFORM_RUN_TEST(test_neg_irq_irqGetStatus_nullHandle); \
    PLATFORM_RUN_TEST(test_neg_irq_irqGetStatus_nullIrqStat)

/* Test: TC-IRQ-0011 */
#define IRQ_TEST_IRQGETSTATUS() \
    IRQ_TEST_POS_IRQGETSTATUS(); \
    IRQ_TEST_NEG_IRQGETSTATUS()

/* ======================================================================== */
/*                        Test APIs: irqGetNextFlag                         */
/* ======================================================================== */
#define IRQ_TEST_POS_IRQGETNEXTFLAG() \
    PLATFORM_RUN_TEST(test_pos_irq_irqGetNextFlag_iteration); \
    PLATFORM_RUN_TEST(test_pos_irq_irqGetNextFlag_multipleFlags); \
    PLATFORM_RUN_TEST(test_pos_irq_irqGetNextFlag_fromArray); \
    PLATFORM_RUN_TEST(test_pos_irq_irqGetNextFlag_withSetFlag)

#define IRQ_TEST_NEG_IRQGETNEXTFLAG() \
    PLATFORM_RUN_TEST(test_neg_irq_irqGetNextFlag_nullIrqStat); \
    PLATFORM_RUN_TEST(test_neg_irq_irqGetNextFlag_nullIrqNum)

/* Test: TC-IRQ-0012 */
#define IRQ_TEST_IRQGETNEXTFLAG() \
    IRQ_TEST_POS_IRQGETNEXTFLAG(); \
    IRQ_TEST_NEG_IRQGETNEXTFLAG()

/* ======================================================================== */
/*                          Test APIs: irqGetFlag                           */
/* ======================================================================== */
#define IRQ_TEST_POS_IRQGETFLAG() \
    PLATFORM_RUN_TEST(test_pos_irq_irqGetClrFlag_single)

#define IRQ_TEST_NEG_IRQGETFLAG() \
    PLATFORM_RUN_TEST(test_neg_irq_irqGetFlag_nullHandle); \
    PLATFORM_RUN_TEST(test_neg_irq_irqGetFlag_nullFlag); \
    PLATFORM_RUN_TEST(test_neg_irq_irqGetFlag_invalidIrqNum); \
    PLATFORM_RUN_TEST(test_neg_irq_irqGetFlag_invalidIrqNumBeyondMax)

/* Test: TC-IRQ-0013 */
#define IRQ_TEST_IRQGETFLAG() \
    IRQ_TEST_POS_IRQGETFLAG(); \
    IRQ_TEST_NEG_IRQGETFLAG()

/* ======================================================================== */
/*                  Test APIs: irqClrFlag, irqClrAllFlags                   */
/* ======================================================================== */
#define IRQ_TEST_POS_IRQCLRFLAG() \
    PLATFORM_RUN_TEST(test_pos_irq_irqClrAllFlags)

#define IRQ_TEST_NEG_IRQCLRFLAG() \
    PLATFORM_RUN_TEST(test_neg_irq_irqClrFlag_nullHandle); \
    PLATFORM_RUN_TEST(test_neg_irq_irqClrFlag_invalidIrqNum); \
    PLATFORM_RUN_TEST(test_neg_irq_irqClrFlag_invalidIrqNumBeyondMax); \
    PLATFORM_RUN_TEST(test_neg_irq_irqClrAllFlags_nullHandle)

/* Test: TC-IRQ-0014 */
#define IRQ_TEST_IRQCLRFLAG() \
    IRQ_TEST_POS_IRQCLRFLAG(); \
    IRQ_TEST_NEG_IRQCLRFLAG()

/* ========================================================================== */
/*                          Aggregate Test Macros                             */
/* ========================================================================== */
#define IRQ_TEST_RUN_POSITIVE() \
    IRQ_TEST_POS_IRQSETGETMASK(); \
    IRQ_TEST_POS_IRQGETSTATUS(); \
    IRQ_TEST_POS_IRQGETNEXTFLAG(); \
    IRQ_TEST_POS_IRQGETFLAG(); \
    IRQ_TEST_POS_IRQCLRFLAG()

#define IRQ_TEST_RUN_NEGATIVE() \
    IRQ_TEST_NEG_IRQSETGETMASK(); \
    IRQ_TEST_NEG_IRQGETSTATUS(); \
    IRQ_TEST_NEG_IRQGETNEXTFLAG(); \
    IRQ_TEST_NEG_IRQGETFLAG(); \
    IRQ_TEST_NEG_IRQCLRFLAG()

#define IRQ_TEST_RUN_ALL() \
    IRQ_TEST_IRQSETGETMASK(); \
    IRQ_TEST_IRQGETSTATUS(); \
    IRQ_TEST_IRQGETNEXTFLAG(); \
    IRQ_TEST_IRQGETFLAG(); \
    IRQ_TEST_IRQCLRFLAG()

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

void irq_test(void *args);

/* Negative test functions */
void test_neg_irq_irqSetMask_nullHandle(void);
void test_neg_irq_irqSetMask_invalidIrqNum(void);
void test_neg_irq_irqSetMasks_nullHandle(void);
void test_neg_irq_irqSetMasks_nullIrqMasks(void);
void test_neg_irq_irqSetMasks_zeroCount(void);
void test_neg_irq_irqGetMask_nullHandle(void);
void test_neg_irq_irqGetMask_nullIrqMasks(void);
void test_neg_irq_irqGetMask_zeroCount(void);
void test_neg_irq_irqGetStatus_nullHandle(void);
void test_neg_irq_irqGetStatus_nullIrqStat(void);
void test_neg_irq_irqGetNextFlag_nullIrqStat(void);
void test_neg_irq_irqGetNextFlag_nullIrqNum(void);
void test_neg_irq_irqGetFlag_nullHandle(void);
void test_neg_irq_irqGetFlag_nullFlag(void);
void test_neg_irq_irqGetFlag_invalidIrqNum(void);
void test_neg_irq_irqClrFlag_nullHandle(void);
void test_neg_irq_irqClrFlag_invalidIrqNum(void);
void test_neg_irq_irqClrAllFlags_nullHandle(void);
void test_neg_irq_irqSetMask_nonMaskable(void);
void test_neg_irq_irqSetMask_invalidIrqNumBeyondMax(void);
void test_neg_irq_irqGetFlag_invalidIrqNumBeyondMax(void);
void test_neg_irq_irqClrFlag_invalidIrqNumBeyondMax(void);
void test_neg_irq_irqGetMask_invalidIrqInArray(void);

/* Positive test functions */
void test_pos_irq_irqSetGetMask_single(void);
void test_pos_irq_irqSetMasks_multiple(void);
void test_pos_irq_irqGetStatus_read(void);
void test_pos_irq_irqGetClrFlag_single(void);
void test_pos_irq_irqClrAllFlags(void);
void test_pos_irq_irqGetNextFlag_iteration(void);
void test_pos_irq_irqSetMask_allMaskable(void);
void test_pos_irq_irqSetGetMask_gpioInt(void);
void test_pos_irq_irqSetGetMask_powerUvov(void);
void test_pos_irq_irqSetGetMask_esmInt(void);
void test_pos_irq_irqSetGetMask_fsmErrorInt(void);
void test_pos_irq_irqSetGetMask_miscInt(void);
void test_pos_irq_irqSetGetMask_thermalWarning(void);
void test_pos_irq_irqSetGetMask_adcConvReady(void);
void test_pos_irq_irqSetGetMask_pushButton(void);
void test_pos_irq_irqGetStatus_withActiveFlags(void);
void test_pos_irq_irqGetNextFlag_multipleFlags(void);
void test_pos_irq_irqGetMask_nonMaskable(void);
void test_pos_irq_irqGetNextFlag_fromArray(void);
void test_pos_irq_irqSetMask_specific(void);
void test_pos_irq_irqSetMask_unmaskSpecific(void);
void test_pos_irq_irqGetStatus_withSetFlag(void);
void test_pos_irq_irqGetNextFlag_withSetFlag(void);

#endif /* IRQ_TEST_H */
