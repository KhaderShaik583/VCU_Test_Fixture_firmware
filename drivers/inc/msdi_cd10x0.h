/*
 * Copyright 2018 NXP
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*!
 * @file msdi_cd10x0.h
 *
 * This header file contains register map for following MSDI devices: CD1020,
 * CD1030, MC33978 and MC34978.
 *
 * Note: If there is no other specification, *_CD1020 macros are applicable also
 * to MC33978 and MC34978.
 * Note: All *_MC33978 macros are applicable also to MC34978.
 *
 */


#ifndef MSDI_CD10X0_H_
#define MSDI_CD10X0_H_

/*******************************************************************************
 * Generic definitions
 ******************************************************************************/

/* Number of SP and SG pins. */
#define MSDI_SP_CNT_CD1020                   (8U)
#define MSDI_SP_CNT_CD1030                   (12U)
#define MSDI_SP_CNT_MAX                      (12U)
#define MSDI_SG_CNT_CD1020                   (14U)
#define MSDI_SG_CNT_CD1030                   (21U)
#define MSDI_SG_CNT_MAX                      (21U)

/* Register address macros. */
#define MSDI_REG_ADDR_MASK                   (0xFE000000U)
#define MSDI_REG_ADDR_SHIFT                  (25U)
#define MSDI_REG_ADDR_F(x)                   \
    ((uint32_t)((uint32_t)(x) << MSDI_REG_ADDR_SHIFT) & MSDI_REG_ADDR_MASK)

/* Register R/W macros. */
#define MSDI_REG_RW_MASK                     (0x01000000U)
#define MSDI_REG_RW_R                        (0x00000000U)
#define MSDI_REG_RW_W                        (0x01000000U)

/* Register data macros. */
#define MSDI_REG_DATA_MASK                   (0x00FFFFFFU)
#define MSDI_REG_DATA_SHIFT                  (0U)
#define MSDI_REG_DATA_F(x)                   \
    ((uint32_t)((uint32_t)(x) << MSDI_REG_DATA_SHIFT) & MSDI_REG_DATA_MASK)

/* FAULT STATUS */
#define MSDI_FAULT_STATUS_MASK               (0x00800000U)

/* INTflg */
#define MSDI_INT_FLG_MASK                    (0x00400000U)

/* Mask of all bits without FAULT STATUS and INTflg. */
#define MSDI_RED_RW_DATA_MASK                (0xFF3FFFFFU)

/*******************************************************************************
 * $00 - SPI check
 ******************************************************************************/

#define MSDI_SPI_CHECK_ASW                   (0x00123456U)

/*******************************************************************************
 * $01 - Device configuration register
 ******************************************************************************/

/* SP0 - SP7/SP11
 * MSDI_CFG_SPX_GND(n) - Switch SP(n) to Ground.
 * MSDI_CFG_SPX_BAT(n) - Switch SP(n) to Battery. */
#define MSDI_CFG_SPX_DEF_CD1020              (0x000000FFU)
#define MSDI_CFG_SPX_DEF_CD1030              (0x00000FFFU)
#define MSDI_CFG_SPX_MASK_CD1020             (0x000000FFU)
#define MSDI_CFG_SPX_MASK_CD1030             (0x00000FFFU)
#define MSDI_CFG_SPX_SHIFT                   (0U)
#define MSDI_CFG_SPX_GND(n)                  (((uint32_t)0x00000000U) << (n))
#define MSDI_CFG_SPX_BAT(n)                  (((uint32_t)0x00000001U) << (n))
#define MSDI_CFG_SPX_F_CD1020(x)             \
    ((uint32_t)((uint32_t)(x) << MSDI_CFG_SPX_SHIFT) & MSDI_CFG_SPX_MASK_CD1020)
#define MSDI_CFG_SPX_F_CD1030(x)             \
    ((uint32_t)((uint32_t)(x) << MSDI_CFG_SPX_SHIFT) & MSDI_CFG_SPX_MASK_CD1030)
#define MSDI_CFG_SPX_P_CD1020(x)             \
    ((uint16_t)(((x) & MSDI_CFG_SPX_MASK_CD1020) >> MSDI_CFG_SPX_SHIFT))
#define MSDI_CFG_SPX_P_CD1030(x)             \
    ((uint16_t)(((x) & MSDI_CFG_SPX_MASK_CD1030) >> MSDI_CFG_SPX_SHIFT))

/* Aconfig - CD1030 and MC33978 only. */
#define MSDI_CFG_ACONFIG_MASK_CD1030         (0x00003000U)
#define MSDI_CFG_ACONFIG_MASK_MC33978        (0x00000300U)
#define MSDI_CFG_ACONFIG_SHIFT_CD1030        (12U)
#define MSDI_CFG_ACONFIG_SHIFT_MC33978       (8U)
#define MSDI_CFG_ACONFIG_F_CD1030(x)         \
    ((uint32_t)((uint32_t)(x) << MSDI_CFG_ACONFIG_SHIFT_CD1030) & MSDI_CFG_ACONFIG_MASK_CD1030)
#define MSDI_CFG_ACONFIG_F_MC33978(x)        \
    ((uint32_t)((uint32_t)(x) << MSDI_CFG_ACONFIG_SHIFT_MC33978) & MSDI_CFG_ACONFIG_MASK_MC33978)
#define MSDI_CFG_ACONFIG_P_CD1030(x)         \
    ((msdi_amux_ctrl_t)(((x) & MSDI_CFG_ACONFIG_MASK_CD1030) >> MSDI_CFG_ACONFIG_SHIFT_CD1030))
#define MSDI_CFG_ACONFIG_P_MC33978(x)        \
    ((msdi_amux_ctrl_t)(((x) & MSDI_CFG_ACONFIG_MASK_MC33978) >> MSDI_CFG_ACONFIG_SHIFT_MC33978))

/* Int_B_Out */
#define MSDI_CFG_INT_B_OUT_MASK_CD1020       (0x00000400U)
#define MSDI_CFG_INT_B_OUT_MASK_CD1030       (0x00004000U)
#define MSDI_CFG_INT_B_OUT_SHIFT_CD1020      (10U)
#define MSDI_CFG_INT_B_OUT_SHIFT_CD1030      (14U)
#define MSDI_CFG_INT_B_OUT_F_CD1020(x)       \
    ((uint32_t)((uint32_t)(x) << MSDI_CFG_INT_B_OUT_SHIFT_CD1020) & MSDI_CFG_INT_B_OUT_MASK_CD1020)
#define MSDI_CFG_INT_B_OUT_F_CD1030(x)       \
    ((uint32_t)((uint32_t)(x) << MSDI_CFG_INT_B_OUT_SHIFT_CD1030) & MSDI_CFG_INT_B_OUT_MASK_CD1030)
#define MSDI_CFG_INT_B_OUT_P_CD1020(x)       \
    ((msdi_int_behavior_t)(((x) & MSDI_CFG_INT_B_OUT_MASK_CD1020) >> MSDI_CFG_INT_B_OUT_SHIFT_CD1020))
#define MSDI_CFG_INT_B_OUT_P_CD1030(x)       \
    ((msdi_int_behavior_t)(((x) & MSDI_CFG_INT_B_OUT_MASK_CD1030) >> MSDI_CFG_INT_B_OUT_SHIFT_CD1030))

/* WAKE_B VDDQ Check */
#define MSDI_CFG_WAKE_B_MASK_CD1020          (0x00000800U)
#define MSDI_CFG_WAKE_B_MASK_CD1030          (0x00008000U)
#define MSDI_CFG_WAKE_B_SHIFT_CD1020         (11U)
#define MSDI_CFG_WAKE_B_SHIFT_CD1030         (15U)
#define MSDI_CFG_WAKE_B_F_CD1020(x)          \
    ((uint32_t)((uint32_t)(x) << MSDI_CFG_WAKE_B_SHIFT_CD1020) & MSDI_CFG_WAKE_B_MASK_CD1020)
#define MSDI_CFG_WAKE_B_F_CD1030(x)          \
    ((uint32_t)((uint32_t)(x) << MSDI_CFG_WAKE_B_SHIFT_CD1030) & MSDI_CFG_WAKE_B_MASK_CD1030)
#define MSDI_CFG_WAKE_B_P_CD1020(x)          \
    ((msdi_wake_vddq_check_t)(((x) & MSDI_CFG_WAKE_B_MASK_CD1020) >> MSDI_CFG_WAKE_B_SHIFT_CD1020))
#define MSDI_CFG_WAKE_B_P_CD1030(x)          \
    ((msdi_wake_vddq_check_t)(((x) & MSDI_CFG_WAKE_B_MASK_CD1030) >> MSDI_CFG_WAKE_B_SHIFT_CD1030))

/* VBATP OV Disable */
#define MSDI_CFG_VBATP_OV_MASK_CD1020        (0x00001000U)
#define MSDI_CFG_VBATP_OV_MASK_CD1030        (0x00010000U)
#define MSDI_CFG_VBATP_OV_SHIFT_CD1020       (12U)
#define MSDI_CFG_VBATP_OV_SHIFT_CD1030       (16U)
#define MSDI_CFG_VBATP_OV_F_CD1020(x)        \
    ((uint32_t)((uint32_t)(x) << MSDI_CFG_VBATP_OV_SHIFT_CD1020) & MSDI_CFG_VBATP_OV_MASK_CD1020)
#define MSDI_CFG_VBATP_OV_F_CD1030(x)        \
    ((uint32_t)((uint32_t)(x) << MSDI_CFG_VBATP_OV_SHIFT_CD1030) & MSDI_CFG_VBATP_OV_MASK_CD1030)
#define MSDI_CFG_VBATP_OV_P_CD1020(x)        \
    ((msdi_vbat_ov_t)(((x) & MSDI_CFG_VBATP_OV_MASK_CD1020) >> MSDI_CFG_VBATP_OV_SHIFT_CD1020))
#define MSDI_CFG_VBATP_OV_P_CD1030(x)        \
    ((msdi_vbat_ov_t)(((x) & MSDI_CFG_VBATP_OV_MASK_CD1030) >> MSDI_CFG_VBATP_OV_SHIFT_CD1030))

/* SBPOLLTIME */
#define MSDI_CFG_SBPOLLTIME_MASK_CD1020      (0x00002000U)
#define MSDI_CFG_SBPOLLTIME_MASK_CD1030      (0x00020000U)
#define MSDI_CFG_SBPOLLTIME_SHIFT_CD1020     (13U)
#define MSDI_CFG_SBPOLLTIME_SHIFT_CD1030     (17U)
#define MSDI_CFG_SBPOLLTIME_F_CD1020(x)      \
    ((uint32_t)((uint32_t)(x) << MSDI_CFG_SBPOLLTIME_SHIFT_CD1020) & MSDI_CFG_SBPOLLTIME_MASK_CD1020)
#define MSDI_CFG_SBPOLLTIME_F_CD1030(x)      \
    ((uint32_t)((uint32_t)(x) << MSDI_CFG_SBPOLLTIME_SHIFT_CD1030) & MSDI_CFG_SBPOLLTIME_MASK_CD1030)
#define MSDI_CFG_SBPOLLTIME_P_CD1020(x)      \
    ((msdi_sbpolltime_t)(((x) & MSDI_CFG_SBPOLLTIME_MASK_CD1020) >> MSDI_CFG_SBPOLLTIME_SHIFT_CD1020))
#define MSDI_CFG_SBPOLLTIME_P_CD1030(x)      \
    ((msdi_sbpolltime_t)(((x) & MSDI_CFG_SBPOLLTIME_MASK_CD1030) >> MSDI_CFG_SBPOLLTIME_SHIFT_CD1030))

/*******************************************************************************
 * Common for $02 and $03 (Tri-state registers)
 ******************************************************************************/

#define MSDI_TRI_STATE_MASK(n)               (((uint32_t)0x00000001U) << (n))
#define MSDI_TRI_STATE_EN(n)                 (((uint32_t)0x00000001U) << (n))
#define MSDI_TRI_STATE_DIS(n)                (((uint32_t)0x00000000U) << (n))

/*******************************************************************************
 * $02 - Tri-state SP register
 ******************************************************************************/

#define MSDI_TRI_STATE_SP_MASK_CD1020        (0x000000FFU)
#define MSDI_TRI_STATE_SP_MASK_CD1030        (0x00000FFFU)
#define MSDI_TRI_STATE_SP_DEF_CD1020         (0x000000FFU)
#define MSDI_TRI_STATE_SP_DEF_CD1030         (0x00000FFFU)

/*******************************************************************************
 * $03 - Tri-state SG register
 ******************************************************************************/

#define MSDI_TRI_STATE_SG_MASK_CD1020        (0x00003FFFU)
#define MSDI_TRI_STATE_SG_MASK_CD1030        (0x001FFFFFU)
#define MSDI_TRI_STATE_SG_DEF_CD1020         (0x00003FFFU)
#define MSDI_TRI_STATE_SG_DEF_CD1030         (0x001FFFFFU)

/*******************************************************************************
 * $04 - Wetting current level SP 0 register
 ******************************************************************************/

#define MSDI_WET_CUR_SP_0_SP_MIN             0U
#define MSDI_WET_CUR_SP_0_SP_MAX             7U
#define MSDI_WET_CUR_SP_0_MASK_CD1020        (0x00DB6DB6U)
#define MSDI_WET_CUR_SP_0_MASK_CD1030        (0x00FFFFFFU)
#define MSDI_WET_CUR_SP_0_MASK_MC33978       (0x00FFFFFFU)
#define MSDI_WET_CUR_SP_0_DEF                (0x00DB6DB6U)
#define MSDI_WET_CUR_SP_0_F(x, n)            ((((uint32_t)(x)) & 0x07U) << (3 * (n)))
#define MSDI_WET_CUR_SP_0_P(x, n)            ((msdi_wet_cur_t)(((x) >> (3 * (n))) & 0x07U))

/*******************************************************************************
 * $05 - Wetting Current level SG 0 register
 ******************************************************************************/

#define MSDI_WET_CUR_SG_0_SG_MIN             0U
#define MSDI_WET_CUR_SG_0_SG_MAX             7U
#define MSDI_WET_CUR_SG_0_MASK_CD1020        (0x00DB6DB6U)
#define MSDI_WET_CUR_SG_0_MASK_CD1030        (0x00FFFFFFU)
#define MSDI_WET_CUR_SG_0_MASK_MC33978       (0x00FFFFFFU)
#define MSDI_WET_CUR_SG_0_DEF                (0x00DB6DB6U)
#define MSDI_WET_CUR_SG_0_F(x, n)            ((((uint32_t)(x)) & 0x07U) << (3 * (n)))
#define MSDI_WET_CUR_SG_0_P(x, n)            ((msdi_wet_cur_t)(((x) >> (3 * (n))) & 0x07U))

/*******************************************************************************
 * $06 - Wetting current level SG 1 register
 ******************************************************************************/

#define MSDI_WET_CUR_SG_1_SG_MIN             8U
#define MSDI_WET_CUR_SG_1_SG_MAX_CD1020      13U
#define MSDI_WET_CUR_SG_1_SG_MAX_CD1030      15U
#define MSDI_WET_CUR_SG_1_MASK_CD1020        (0x00036DB6U)
#define MSDI_WET_CUR_SG_1_MASK_CD1030        (0x00FFFFFFU)
#define MSDI_WET_CUR_SG_1_MASK_MC33978       (0x0003FFFFU)
#define MSDI_WET_CUR_SG_1_DEF_CD1020         (0x00036DB6U)
#define MSDI_WET_CUR_SG_1_DEF_CD1030         (0x00DB6DB6U)
#define MSDI_WET_CUR_SG_1_F(x, n)            ((((uint32_t)(x)) & 0x07U) << (3 * ((n) - 8)))
#define MSDI_WET_CUR_SG_1_P(x, n)            ((msdi_wet_cur_t)(((x) >> (3 * ((n) - 8))) & 0x07U))

/*******************************************************************************
 * $07 - Wetting current level SG 2 register - CD1030 only
 ******************************************************************************/

#define MSDI_WET_CUR_SG_2_SG_MIN             16U
#define MSDI_WET_CUR_SG_2_SG_MAX             20U
#define MSDI_WET_CUR_SG_2_MASK_CD1030        (0x00007FFFU)
#define MSDI_WET_CUR_SG_2_DEF_CD1030         (0x00006DB6U)
#define MSDI_WET_CUR_SG_2_F_CD1030(x, n)     ((((uint32_t)(x)) & 0x07U) << (3 * ((n) - 16)))
#define MSDI_WET_CUR_SG_2_P_CD1030(x, n)     ((msdi_wet_cur_t)(((x) >> (3 * ((n) - 16))) & 0x07U))

/*******************************************************************************
 * $08 - Wetting current level SP 1 register - CD1030 only
 ******************************************************************************/

#define MSDI_WET_CUR_SP_1_SP_MIN             8U
#define MSDI_WET_CUR_SP_1_SP_MAX             11U
#define MSDI_WET_CUR_SP_1_MASK_CD1030        (0x00000FFFU)
#define MSDI_WET_CUR_SP_1_DEF_CD1030         (0x00000DB6U)
#define MSDI_WET_CUR_SP_1_F_CD1030(x, n)     ((((uint32_t)(x)) & 0x07U) << (3 * ((n) - 8)))
#define MSDI_WET_CUR_SP_1_P_CD1030(x, n)     ((msdi_wet_cur_t)(((x) >> (3 * ((n) - 8))) & 0x07U))

/*******************************************************************************
 * Common for $0B and $0C (Continuous wetting current registers)
 ******************************************************************************/

#define MSDI_CONT_WET_MASK(n)                (((uint32_t)0x00000001U) << (n))
#define MSDI_CONT_WET_EN(n)                  (((uint32_t)0x00000001U) << (n))
#define MSDI_CONT_WET_DIS(n)                 (((uint32_t)0x00000000U) << (n))

/*******************************************************************************
 * $0B - Continuous wetting current SP register
 ******************************************************************************/

#define MSDI_CONT_WET_SP_MASK_CD1020         (0x000000FFU)
#define MSDI_CONT_WET_SP_MASK_CD1030         (0x00000FFFU)
#define MSDI_CONT_WET_SP_DEF                 (0x00000002U)

/*******************************************************************************
 * $0C - Continuous wetting current SG register
 ******************************************************************************/

#define MSDI_CONT_WET_SG_MASK_CD1020         (0x00003FFFU)
#define MSDI_CONT_WET_SG_MASK_CD1030         (0x001FFFFFU)
#define MSDI_CONT_WET_SG_DEF                 (0x00000000U)

/*******************************************************************************
 * Common for $0D and $0E (Interrupt enable registers)
 ******************************************************************************/

#define MSDI_INT_EN_MASK(n)                  (((uint32_t)0x00000001U) << (n))
#define MSDI_INT_EN(n)                       (((uint32_t)0x00000001U) << (n))
#define MSDI_INT_DIS(n)                      (((uint32_t)0x00000000U) << (n))

/*******************************************************************************
 * $0D - Interrupt enable SP register
 ******************************************************************************/

#define MSDI_INT_EN_SP_MASK_CD1020           (0x000000FFU)
#define MSDI_INT_EN_SP_MASK_CD1030           (0x00000FFDU)
#define MSDI_INT_EN_SP_DEF_CD1020            (0x000000FFU)
#define MSDI_INT_EN_SP_DEF_CD1030            (0x00000000U)

/*******************************************************************************
 * $0E - Interrupt enable SG register
 ******************************************************************************/

#define MSDI_INT_EN_SG_MASK_CD1020           (0x00003FFFU)
#define MSDI_INT_EN_SG_MASK_CD1030           (0x000FFFFFU)
#define MSDI_INT_EN_SG_DEF_CD1020            (0x00003FFFU)
#define MSDI_INT_EN_SG_DEF_CD1030            (0x001FFFFFU)

/*******************************************************************************
 * $0F - Low-power mode configuration
 ******************************************************************************/

/* Int - CD1030 and MC33978 only. */
#define MSDI_LPM_CFG_INT_MASK                (0x000000F0U)
#define MSDI_LPM_CFG_INT_SHIFT               (4U)
#define MSDI_LPM_CFG_INT_F(x)                \
    ((uint32_t)((uint32_t)(x) << MSDI_LPM_CFG_INT_SHIFT) & MSDI_LPM_CFG_INT_MASK)
#define MSDI_LPM_CFG_INT_P(x)                \
    ((msdi_int_tmr_val_t)(((x) & MSDI_LPM_CFG_INT_MASK) >> MSDI_LPM_CFG_INT_SHIFT))

/* Poll */
#define MSDI_LPM_CFG_POLL_MASK               (0x0000000FU)
#define MSDI_LPM_CFG_POLL_SHIFT              (0U)
#define MSDI_LPM_CFG_POLL_F(x)               \
    ((uint32_t)((uint32_t)(x) << MSDI_LPM_CFG_POLL_SHIFT) & MSDI_LPM_CFG_POLL_MASK)
#define MSDI_LPM_CFG_POLL_P(x)               \
    ((msdi_poll_rate_t)(((x) & MSDI_LPM_CFG_POLL_MASK) >> MSDI_LPM_CFG_POLL_SHIFT))

/*******************************************************************************
 * Common for $10 and $11 (Wake-up enable registers)
 ******************************************************************************/

#define MSDI_WAKE_EN_MASK(n)                 (((uint32_t)0x00000001U) << (n))
#define MSDI_WAKE_EN(n)                      (((uint32_t)0x00000001U) << (n))
#define MSDI_WAKE_DIS(n)                     (((uint32_t)0x00000000U) << (n))

/*******************************************************************************
 * $10 - Wake-up enable register SP
 ******************************************************************************/

#define MSDI_WAKE_EN_SP_MASK_CD1020          (0x000000FFU)
#define MSDI_WAKE_EN_SP_MASK_CD1030          (0x00000FFFU)
#define MSDI_WAKE_EN_SP_DEF_CD1020           (0x000000FFU)
#define MSDI_WAKE_EN_SP_DEF_CD1030           (0x00000FFFU)

/*******************************************************************************
 * $11 - Wake-up enable register SG
 ******************************************************************************/

#define MSDI_WAKE_EN_SG_MASK_CD1020          (0x00003FFFU)
#define MSDI_WAKE_EN_SG_MASK_CD1030          (0x001FFFFFU)
#define MSDI_WAKE_EN_SG_DEF_CD1020           (0x00003FFFU)
#define MSDI_WAKE_EN_SG_DEF_CD1030           (0x001FFFFFU)

/*******************************************************************************
 * Common for $12 and $13 (Comparator only registers)
 ******************************************************************************/

#define MSDI_COMP_ONLY_MASK(n)               (((uint32_t)0x00000001U) << (n))
#define MSDI_COMP_ONLY_WO_POLLING(n)         (((uint32_t)0x00000001U) << (n))
#define MSDI_COMP_ONLY_W_POOLING(n)          (((uint32_t)0x00000000U) << (n))

/*******************************************************************************
 * $12 - Comparator only SP
 ******************************************************************************/

#define MSDI_COMP_ONLY_SP_MASK_CD1020        (0x000000FFU)
#define MSDI_COMP_ONLY_SP_MASK_CD1030        (0x00000FFFU)
#define MSDI_COMP_ONLY_SP_DEF                (0x00000000U)

/*******************************************************************************
 * $13 - Comparator only SG
 ******************************************************************************/

#define MSDI_COMP_ONLY_SG_MASK_CD1020        (0x00003FFFU)
#define MSDI_COMP_ONLY_SG_MASK_CD1030        (0x001FFFFFU)
#define MSDI_COMP_ONLY_SG_DEF                (0x00000000U)

/*******************************************************************************
 * Common for $14 and $15 (LPM voltage threshold registers)
 ******************************************************************************/

#define MSDI_LPM_THR_MASK(n)                 (((uint32_t)0x00000001U) << (n))
#define MSDI_LPM_THR_NORMAL(n)               (((uint32_t)0x00000001U) << (n))
#define MSDI_LPM_THR_DELTA(n)                (((uint32_t)0x00000000U) << (n))

/*******************************************************************************
 * $14 - LPM voltage threshold SP configuration
 ******************************************************************************/

#define MSDI_LPM_THR_SP_MASK_CD1020          (0x000000FFU)
#define MSDI_LPM_THR_SP_MASK_CD1030          (0x00000FFFU)
#define MSDI_LPM_THR_SP_DEF                  (0x00000000U)

/*******************************************************************************
 * $15 - LPM voltage threshold SG configuration
 ******************************************************************************/

#define MSDI_LPM_THR_SG_MASK_CD1020          (0x00003FFFU)
#define MSDI_LPM_THR_SG_MASK_CD1030          (0x001FFFFFU)
#define MSDI_LPM_THR_SG_DEF                  (0x00000000U)

/*******************************************************************************
 * Common for $16 and $17 (Polling current configuration registers)
 ******************************************************************************/

#define MSDI_POLL_CUR_MASK(n)                (((uint32_t)0x00000001U) << (n))
#define MSDI_POLL_CUR_IWET(n)                (((uint32_t)0x00000001U) << (n))
#define MSDI_POLL_CUR_NORMAL(n)              (((uint32_t)0x00000000U) << (n))

/*******************************************************************************
 * $16 - Polling current SP configuration
 ******************************************************************************/

#define MSDI_POLL_CUR_SP_MASK_CD1020         (0x000000FFU)
#define MSDI_POLL_CUR_SP_MASK_CD1030         (0x00000FFFU)
#define MSDI_POLL_CUR_SP_DEF                 (0x00000000U)

/*******************************************************************************
 * $17 - Polling current SG configuration
 ******************************************************************************/

#define MSDI_POLL_CUR_SG_MASK_CD1020         (0x00003FFFU)
#define MSDI_POLL_CUR_SG_MASK_CD1030         (0x001FFFFFU)
#define MSDI_POLL_CUR_SG_DEF                 (0x00000000U)

/*******************************************************************************
 * Common for $18 and $19 (Slow polling registers)
 ******************************************************************************/

#define MSDI_SLOW_POLL_MASK(n)               (((uint32_t)0x00000001U) << (n))
#define MSDI_SLOW_POLL_4X_SLOWER(n)          (((uint32_t)0x00000001U) << (n))
#define MSDI_SLOW_POLL_NORMAL(n)             (((uint32_t)0x00000000U) << (n))

/*******************************************************************************
 * $18 - Slow polling SP - CD1030 and MC33978 only
 ******************************************************************************/

#define MSDI_SLOW_POLL_SP_MASK_MC33978       (0x000000FFU)
#define MSDI_SLOW_POLL_SP_MASK_CD1030        (0x00000FFFU)
#define MSDI_SLOW_POLL_SP_DEF                (0x00000000U)

/*******************************************************************************
 * $19 - Slow polling SG - CD1030 and MC33978 only
 ******************************************************************************/

#define MSDI_SLOW_POLL_SG_MASK_MC33978       (0x00003FFFU)
#define MSDI_SLOW_POLL_SG_MASK_CD1030        (0x001FFFFFU)
#define MSDI_SLOW_POLL_SG_DEF                (0x00000000U)

/*******************************************************************************
 * Common for $1A and $1B (Wake-up debounce registers)
 ******************************************************************************/

#define MSDI_WAKE_DEB_MASK(n)                (((uint32_t)0x00000001U) << (n))
#define MSDI_WAKE_DEB_EXTENDED(n)            (((uint32_t)0x00000001U) << (n))
#define MSDI_WAKE_DEB_NORMAL(n)              (((uint32_t)0x00000000U) << (n))

/*******************************************************************************
 * $1A - Wake-up debounce SP - CD1030 and MC33978 only
 ******************************************************************************/

#define MSDI_WAKE_DEB_SP_MASK_MC33978        (0x000000FFU)
#define MSDI_WAKE_DEB_SP_MASK_CD1030         (0x00000FFFU)
#define MSDI_WAKE_DEB_SP_DEF                 (0x00000000U)

/*******************************************************************************
 * $1B - Wake-up debounce SG - CD1030 and MC33978 only
 ******************************************************************************/

#define MSDI_WAKE_DEB_SG_MASK_MC33978        (0x00003FFFU)
#define MSDI_WAKE_DEB_SG_MASK_CD1030         (0x001FFFFFU)
#define MSDI_WAKE_DEB_SG_DEF                 (0x00000000U)

/*******************************************************************************
 * $1C - Enter low-power mode
 ******************************************************************************/

/*******************************************************************************
 * $1D - AMUX control register
 ******************************************************************************/

/* AMUX current select */
#define MSDI_AMUX_CTRL_ASETT0_MASK           (0x00000040U)
#define MSDI_AMUX_CTRL_ASETT0_SHIFT          (6U)
#define MSDI_AMUX_CTRL_ASETT0_F(x)           \
    ((uint32_t)((uint32_t)(x) << MSDI_AMUX_CTRL_ASETT0_SHIFT) & MSDI_AMUX_CTRL_ASETT0_MASK)
#define MSDI_AMUX_CTRL_ASETT0_P(x)           \
    ((msdi_amux_current_t)(((x) & MSDI_AMUX_CTRL_ASETT0_MASK) >> MSDI_AMUX_CTRL_ASETT0_SHIFT))

/* AMUX channel select */
#define MSDI_AMUX_CTRL_ASEL_MASK             (0x0000003FU)
#define MSDI_AMUX_CTRL_ASEL_SHIFT            (0U)
#define MSDI_AMUX_CTRL_ASEL_F(x)             \
    ((uint32_t)((uint32_t)(x) << MSDI_AMUX_CTRL_ASEL_SHIFT) & MSDI_AMUX_CTRL_ASEL_MASK)
#define MSDI_AMUX_CTRL_ASEL_P(x)             \
    ((uint8_t)(((x) & MSDI_AMUX_CTRL_ASEL_MASK) >> MSDI_AMUX_CTRL_ASEL_SHIFT))

/* AMUX channel select - CD1020 */
#define MSDI_AMUX_ASEL_CD1020_NO_INPUT       0x00U
#define MSDI_AMUX_ASEL_CD1020_SG0            0x01U
#define MSDI_AMUX_ASEL_CD1020_SG1            0x02U
#define MSDI_AMUX_ASEL_CD1020_SG2            0x03U
#define MSDI_AMUX_ASEL_CD1020_SG3            0x04U
#define MSDI_AMUX_ASEL_CD1020_SG4            0x05U
#define MSDI_AMUX_ASEL_CD1020_SG5            0x06U
#define MSDI_AMUX_ASEL_CD1020_SG6            0x07U
#define MSDI_AMUX_ASEL_CD1020_SG7            0x08U
#define MSDI_AMUX_ASEL_CD1020_SG8            0x09U
#define MSDI_AMUX_ASEL_CD1020_SG9            0x0AU
#define MSDI_AMUX_ASEL_CD1020_SG10           0x0BU
#define MSDI_AMUX_ASEL_CD1020_SG11           0x0CU
#define MSDI_AMUX_ASEL_CD1020_SG12           0x0DU
#define MSDI_AMUX_ASEL_CD1020_SG13           0x0EU
#define MSDI_AMUX_ASEL_CD1020_SP0            0x0FU
#define MSDI_AMUX_ASEL_CD1020_SP1            0x10U
#define MSDI_AMUX_ASEL_CD1020_SP2            0x11U
#define MSDI_AMUX_ASEL_CD1020_SP3            0x12U
#define MSDI_AMUX_ASEL_CD1020_SP4            0x13U
#define MSDI_AMUX_ASEL_CD1020_SP5            0x14U
#define MSDI_AMUX_ASEL_CD1020_SP6            0x15U
#define MSDI_AMUX_ASEL_CD1020_SP7            0x16U

/* AMUX channel select - CD1030 */
#define MSDI_AMUX_ASEL_CD1030_NO_INPUT       0x00U
#define MSDI_AMUX_ASEL_CD1030_SG0            0x01U
#define MSDI_AMUX_ASEL_CD1030_SG1            0x02U
#define MSDI_AMUX_ASEL_CD1030_SG2            0x03U
#define MSDI_AMUX_ASEL_CD1030_SG3            0x04U
#define MSDI_AMUX_ASEL_CD1030_SG4            0x05U
#define MSDI_AMUX_ASEL_CD1030_SG5            0x06U
#define MSDI_AMUX_ASEL_CD1030_SG6            0x07U
#define MSDI_AMUX_ASEL_CD1030_SG7            0x08U
#define MSDI_AMUX_ASEL_CD1030_SG8            0x09U
#define MSDI_AMUX_ASEL_CD1030_SG9            0x0AU
#define MSDI_AMUX_ASEL_CD1030_SG10           0x0BU
#define MSDI_AMUX_ASEL_CD1030_SG11           0x0CU
#define MSDI_AMUX_ASEL_CD1030_SG12           0x0DU
#define MSDI_AMUX_ASEL_CD1030_SG13           0x0EU
#define MSDI_AMUX_ASEL_CD1030_SG14           0x0FU
#define MSDI_AMUX_ASEL_CD1030_SG15           0x10U
#define MSDI_AMUX_ASEL_CD1030_SG16           0x11U
#define MSDI_AMUX_ASEL_CD1030_SG17           0x12U
#define MSDI_AMUX_ASEL_CD1030_SG18           0x13U
#define MSDI_AMUX_ASEL_CD1030_SG19           0x14U
#define MSDI_AMUX_ASEL_CD1030_SG20           0x15U
#define MSDI_AMUX_ASEL_CD1030_SP0            0x16U
#define MSDI_AMUX_ASEL_CD1030_SP1            0x17U
#define MSDI_AMUX_ASEL_CD1030_SP2            0x18U
#define MSDI_AMUX_ASEL_CD1030_SP3            0x19U
#define MSDI_AMUX_ASEL_CD1030_SP4            0x1AU
#define MSDI_AMUX_ASEL_CD1030_SP5            0x1BU
#define MSDI_AMUX_ASEL_CD1030_SP6            0x1CU
#define MSDI_AMUX_ASEL_CD1030_SP7            0x1DU
#define MSDI_AMUX_ASEL_CD1030_SP8            0x1EU
#define MSDI_AMUX_ASEL_CD1030_SP9            0x1FU
#define MSDI_AMUX_ASEL_CD1030_SP10           0x20U
#define MSDI_AMUX_ASEL_CD1030_SP11           0x21U
#define MSDI_AMUX_ASEL_CD1030_TEMP_DIODE     0x22U
#define MSDI_AMUX_ASEL_CD1030_BATTERY_SENSE  0x23U

/* AMUX channel select - MC33978 */
#define MSDI_AMUX_ASEL_MC33978_NO_INPUT       0x00U
#define MSDI_AMUX_ASEL_MC33978_SG0            0x01U
#define MSDI_AMUX_ASEL_MC33978_SG1            0x02U
#define MSDI_AMUX_ASEL_MC33978_SG2            0x03U
#define MSDI_AMUX_ASEL_MC33978_SG3            0x04U
#define MSDI_AMUX_ASEL_MC33978_SG4            0x05U
#define MSDI_AMUX_ASEL_MC33978_SG5            0x06U
#define MSDI_AMUX_ASEL_MC33978_SG6            0x07U
#define MSDI_AMUX_ASEL_MC33978_SG7            0x08U
#define MSDI_AMUX_ASEL_MC33978_SG8            0x09U
#define MSDI_AMUX_ASEL_MC33978_SG9            0x0AU
#define MSDI_AMUX_ASEL_MC33978_SG10           0x0BU
#define MSDI_AMUX_ASEL_MC33978_SG11           0x0CU
#define MSDI_AMUX_ASEL_MC33978_SG12           0x0DU
#define MSDI_AMUX_ASEL_MC33978_SG13           0x0EU
#define MSDI_AMUX_ASEL_MC33978_SP0            0x0FU
#define MSDI_AMUX_ASEL_MC33978_SP1            0x10U
#define MSDI_AMUX_ASEL_MC33978_SP2            0x11U
#define MSDI_AMUX_ASEL_MC33978_SP3            0x12U
#define MSDI_AMUX_ASEL_MC33978_SP4            0x13U
#define MSDI_AMUX_ASEL_MC33978_SP5            0x14U
#define MSDI_AMUX_ASEL_MC33978_SP6            0x15U
#define MSDI_AMUX_ASEL_MC33978_SP7            0x16U
#define MSDI_AMUX_ASEL_MC33978_TEMP_DIODE     0x17U
#define MSDI_AMUX_ASEL_MC33978_BATTERY_SENSE  0x18U

/*******************************************************************************
 * Common for $1E and $1F (Read switch status)
 ******************************************************************************/

#define READ_SW_STAT_MASK(n)                 (((uint32_t)0x00000001U) << (n))
#define READ_SW_STAT_CLOSED(n)               (((uint32_t)0x00000001U) << (n))
#define READ_SW_STAT_OPEN(n)                 (((uint32_t)0x00000000U) << (n))

/*******************************************************************************
 * $1E - Read switch status registers SP - CD1030 only
 ******************************************************************************/

#define READ_SW_STAT_SP_MASK_CD1030           (0x00000FFFU)

/*******************************************************************************
 * $1F - Read switch status (SG registers only in case of CD1030)
 ******************************************************************************/

#define READ_SW_STAT_SG_MASK_CD1030           (0x001FFFFFU)

#define READ_SW_STAT_SG_MASK_CD1020           (0x00003FFFU)
#define READ_SW_STAT_SG_SHIFT_CD1020          (0U)
#define READ_SW_STAT_SG_P_CD1020(x)           \
    ((uint32_t)(((x) & READ_SW_STAT_SG_MASK_CD1020) >> READ_SW_STAT_SG_SHIFT_CD1020))

#define READ_SW_STAT_SP_MASK_CD1020           (0x003FC000U)
#define READ_SW_STAT_SP_SHIFT_CD1020          (14U)
#define READ_SW_STAT_SP_P_CD1020(x)           \
    ((uint32_t)(((x) & READ_SW_STAT_SP_MASK_CD1020) >> READ_SW_STAT_SP_SHIFT_CD1020))

/*******************************************************************************
 * $21 - Fault status register
 ******************************************************************************/

#define MSDI_FLT_STAT_SPI_ERROR_MASK          (0x00000400U)
#define MSDI_FLT_STAT_HASH_FLT_MASK           (0x00000200U)
#define MSDI_FLT_STAT_UV_MASK                 (0x00000080U)
#define MSDI_FLT_STAT_OV_MASK                 (0x00000040U)
#define MSDI_FLT_STAT_TEMP_FLG_MASK           (0x00000020U)
#define MSDI_FLT_STAT_OT_MASK                 (0x00000010U)
#define MSDI_FLT_STAT_INT_WAKE_MASK           (0x00000008U)
#define MSDI_FLT_STAT_WAKE_WAKE_MASK          (0x00000004U)
#define MSDI_FLT_STAT_SPI_WAKE_MASK           (0x00000002U)
#define MSDI_FLT_STAT_POR_MASK                (0x00000001U)

/*******************************************************************************
 * $23 - Interrupt request
 ******************************************************************************/

/*******************************************************************************
 * $24 - Reset register
 ******************************************************************************/

#endif /* MSDI_CD10X0_H_ */
