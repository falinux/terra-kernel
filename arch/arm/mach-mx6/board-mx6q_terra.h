/*
 * Copyright (C) 2014 Falinux ltd.
 * Based on sabresd board from Freescale Semiconductor, Inc. All Rights Reserved
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.

 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.

 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

#ifndef _BOARD_MX6Q_LGEMS_H
#define _BOARD_MX6Q_LGEMS_H

#include <mach/iomux-mx6q.h>

static iomux_v3_cfg_t mx6q_terra_pads[] = {
	/* UART1 for debug */
	MX6Q_PAD_CSI0_DAT10__UART1_TXD,
	MX6Q_PAD_CSI0_DAT11__UART1_RXD,

	/* UART2  */
	MX6Q_PAD_EIM_D26__UART2_TXD,
	MX6Q_PAD_EIM_D27__UART2_RXD,

	/* UART3 */
	MX6Q_PAD_EIM_D24__UART3_TXD,
	MX6Q_PAD_EIM_D25__UART3_RXD,

	/* UART4 */
	MX6Q_PAD_KEY_COL0__UART4_TXD,
	MX6Q_PAD_KEY_ROW0__UART4_RXD,

	/* I2C3 */
	MX6Q_PAD_GPIO_3__I2C3_SCL,	/* I2C3 SCL */
	MX6Q_PAD_GPIO_6__I2C3_SDA,	/* I2C3 SDA */

	/* PWM1 - GPIO1[21] */
	MX6Q_PAD_SD1_DAT3__PWM1_PWMO,

	/* USB OTG ID */
	MX6Q_PAD_ENET_RX_ER__ANATOP_USBOTG_ID,

	/* ENET */
	MX6Q_PAD_ENET_MDIO__ENET_MDIO,
	MX6Q_PAD_ENET_MDC__ENET_MDC,
	MX6Q_PAD_RGMII_TXC__ENET_RGMII_TXC,
	MX6Q_PAD_RGMII_TD0__ENET_RGMII_TD0,
	MX6Q_PAD_RGMII_TD1__ENET_RGMII_TD1,
	MX6Q_PAD_RGMII_TD2__ENET_RGMII_TD2,
	MX6Q_PAD_RGMII_TD3__ENET_RGMII_TD3,
	MX6Q_PAD_RGMII_TX_CTL__ENET_RGMII_TX_CTL,
	MX6Q_PAD_ENET_REF_CLK__ENET_TX_CLK,
	MX6Q_PAD_RGMII_RXC__ENET_RGMII_RXC,
	MX6Q_PAD_RGMII_RD0__ENET_RGMII_RD0,
	MX6Q_PAD_RGMII_RD1__ENET_RGMII_RD1,
	MX6Q_PAD_RGMII_RD2__ENET_RGMII_RD2,
	MX6Q_PAD_RGMII_RD3__ENET_RGMII_RD3,
	MX6Q_PAD_RGMII_RX_CTL__ENET_RGMII_RX_CTL,
	MX6Q_PAD_ENET_RXD1__GPIO_1_26,		            /* RGMII Phy Interrupt */
	MX6Q_PAD_ENET_CRS_DV__GPIO_1_25,		        /* RGMII reset */
#ifdef CONFIG_FEC_1588
	MX6Q_PAD_GPIO_16__ENET_ANATOP_ETHERNET_REF_OUT, /* Internal connect for 1588 TS Clock */
#endif

	/* USDHC3 */
	MX6Q_PAD_SD3_CLK__USDHC3_CLK_50MHZ,
	MX6Q_PAD_SD3_CMD__USDHC3_CMD_50MHZ,
	MX6Q_PAD_SD3_DAT0__USDHC3_DAT0_50MHZ,
	MX6Q_PAD_SD3_DAT1__USDHC3_DAT1_50MHZ,
	MX6Q_PAD_SD3_DAT2__USDHC3_DAT2_50MHZ,
	MX6Q_PAD_SD3_DAT3__USDHC3_DAT3_50MHZ,
	MX6Q_PAD_SD3_DAT5__GPIO_7_0,	// CD
	MX6Q_PAD_SD3_DAT4__GPIO_7_1,	// WP

	/* USDHC4 */
	MX6Q_PAD_SD4_CLK__USDHC4_CLK_50MHZ,
	MX6Q_PAD_SD4_CMD__USDHC4_CMD_50MHZ,
	MX6Q_PAD_SD4_DAT0__USDHC4_DAT0_50MHZ,
	MX6Q_PAD_SD4_DAT1__USDHC4_DAT1_50MHZ,
	MX6Q_PAD_SD4_DAT2__USDHC4_DAT2_50MHZ,
	MX6Q_PAD_SD4_DAT3__USDHC4_DAT3_50MHZ,
	MX6Q_PAD_SD3_DAT7__GPIO_6_17,	// CD
	MX6Q_PAD_SD3_DAT6__GPIO_6_18,	// WP

	/*PMIC_nINT(PMIC interrupt signal)*/
	MX6Q_PAD_GPIO_18__GPIO_7_13,
	
	/*WDOG_B to reset pmic*/
	MX6Q_PAD_GPIO_1__WDOG2_WDOG_B,
	
    /* TOUCH interrupt signal */
	MX6Q_PAD_KEY_COL1__GPIO_4_8,
	
	/* HDMI CEC */
	MX6Q_PAD_KEY_ROW2__HDMI_TX_CEC_LINE,
		                    
    /* EIM */
    MX6Q_PAD_EIM_OE__WEIM_WEIM_OE,
    MX6Q_PAD_EIM_RW__WEIM_WEIM_RW,
    MX6Q_PAD_EIM_WAIT__WEIM_WEIM_WAIT,
    MX6Q_PAD_EIM_LBA__WEIM_WEIM_LBA,
    MX6Q_PAD_EIM_BCLK__WEIM_WEIM_BCLK,
    MX6Q_PAD_EIM_CS0__WEIM_WEIM_CS_0,   // CS0  
    MX6Q_PAD_EIM_CS1__WEIM_WEIM_CS_1,   // CS1    
    MX6Q_PAD_EIM_DA15__WEIM_WEIM_DA_A_15,
    MX6Q_PAD_EIM_DA14__WEIM_WEIM_DA_A_14,
    MX6Q_PAD_EIM_DA13__WEIM_WEIM_DA_A_13,
    MX6Q_PAD_EIM_DA12__WEIM_WEIM_DA_A_12,
    MX6Q_PAD_EIM_DA11__WEIM_WEIM_DA_A_11,
    MX6Q_PAD_EIM_DA10__WEIM_WEIM_DA_A_10,
    MX6Q_PAD_EIM_DA9__WEIM_WEIM_DA_A_9,
    MX6Q_PAD_EIM_DA8__WEIM_WEIM_DA_A_8,
    MX6Q_PAD_EIM_DA7__WEIM_WEIM_DA_A_7,
    MX6Q_PAD_EIM_DA6__WEIM_WEIM_DA_A_6,
    MX6Q_PAD_EIM_DA5__WEIM_WEIM_DA_A_5,
    MX6Q_PAD_EIM_DA4__WEIM_WEIM_DA_A_4,
    MX6Q_PAD_EIM_DA3__WEIM_WEIM_DA_A_3,
    MX6Q_PAD_EIM_DA2__WEIM_WEIM_DA_A_2,
    MX6Q_PAD_EIM_DA1__WEIM_WEIM_DA_A_1,
    MX6Q_PAD_EIM_DA0__WEIM_WEIM_DA_A_0,
};

static iomux_v3_cfg_t mx6q_terra_hdmi_ddc_pads[] = {
	MX6Q_PAD_KEY_COL3__HDMI_TX_DDC_SCL, /* HDMI DDC SCL */
	MX6Q_PAD_KEY_ROW3__HDMI_TX_DDC_SDA, /* HDMI DDC SDA */
};

static iomux_v3_cfg_t mx6q_terra_i2c2_pads[] = {
	MX6Q_PAD_KEY_COL3__I2C2_SCL,	/* I2C2 SCL */
	MX6Q_PAD_KEY_ROW3__I2C2_SDA,	/* I2C2 SDA */
};

static iomux_v3_cfg_t mx6q_terra_can_pads[] = {
	/* CAN1 */
	MX6Q_PAD_GPIO_7__CAN1_TXCAN,
	MX6Q_PAD_GPIO_8__CAN1_RXCAN,

	/* CAN2 */
	MX6Q_PAD_KEY_COL4__CAN2_TXCAN,
	MX6Q_PAD_KEY_ROW4__CAN2_RXCAN,
};


#define MX6Q_USDHC_PAD_SETTING(id, speed)	\
mx6q_sd##id##_##speed##mhz[] = {		\
	MX6Q_PAD_SD##id##_CLK__USDHC##id##_CLK_##speed##MHZ,	\
	MX6Q_PAD_SD##id##_CMD__USDHC##id##_CMD_##speed##MHZ,	\
	MX6Q_PAD_SD##id##_DAT0__USDHC##id##_DAT0_##speed##MHZ,	\
	MX6Q_PAD_SD##id##_DAT1__USDHC##id##_DAT1_##speed##MHZ,	\
	MX6Q_PAD_SD##id##_DAT2__USDHC##id##_DAT2_##speed##MHZ,	\
	MX6Q_PAD_SD##id##_DAT3__USDHC##id##_DAT3_##speed##MHZ,	\
}

static iomux_v3_cfg_t MX6Q_USDHC_PAD_SETTING(3, 50);
static iomux_v3_cfg_t MX6Q_USDHC_PAD_SETTING(3, 100);
static iomux_v3_cfg_t MX6Q_USDHC_PAD_SETTING(3, 200);
static iomux_v3_cfg_t MX6Q_USDHC_PAD_SETTING(4, 50);
static iomux_v3_cfg_t MX6Q_USDHC_PAD_SETTING(4, 100);
static iomux_v3_cfg_t MX6Q_USDHC_PAD_SETTING(4, 200);

#endif
