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

#ifndef _BOARD_MX6DL_LGEMS_H
#define _BOARD_MX6DL_LGEMS_H

#include <mach/iomux-mx6dl.h>

static iomux_v3_cfg_t mx6dl_lgems_pads[] = {
	/* UART1 for debug */
	MX6DL_PAD_CSI0_DAT10__UART1_TXD,
	MX6DL_PAD_CSI0_DAT11__UART1_RXD,

	/* UART4 for RS485 */
	MX6DL_PAD_CSI0_DAT12__UART4_TXD,
	MX6DL_PAD_CSI0_DAT13__UART4_RXD,

	/* CAN2 */
	MX6DL_PAD_KEY_COL4__CAN2_TXCAN,
	MX6DL_PAD_KEY_ROW4__CAN2_RXCAN,

	/* I2C2, PMCI, RTC */
	MX6DL_PAD_KEY_COL3__I2C2_SCL,
	MX6DL_PAD_KEY_ROW3__I2C2_SDA,

	/* I2C3 - TOUCH */
	MX6DL_PAD_GPIO_3__I2C3_SCL,	/* I2C3 SCL */
	MX6DL_PAD_GPIO_6__I2C3_SDA,	/* I2C3 SDA */

	/* PWM1 - GPIO1[21] */
	MX6DL_PAD_SD1_DAT3__PWM1_PWMO,

	/* USB OTG ID */
	MX6DL_PAD_ENET_RX_ER__ANATOP_USBOTG_ID,

	/* ENET */
	MX6DL_PAD_ENET_MDIO__ENET_MDIO,
	MX6DL_PAD_ENET_MDC__ENET_MDC,
	MX6DL_PAD_RGMII_TXC__ENET_RGMII_TXC,
	MX6DL_PAD_RGMII_TD0__ENET_RGMII_TD0,
	MX6DL_PAD_RGMII_TD1__ENET_RGMII_TD1,
	MX6DL_PAD_RGMII_TD2__ENET_RGMII_TD2,
	MX6DL_PAD_RGMII_TD3__ENET_RGMII_TD3,
	MX6DL_PAD_RGMII_TX_CTL__ENET_RGMII_TX_CTL,
	MX6DL_PAD_ENET_REF_CLK__ENET_TX_CLK,
	MX6DL_PAD_RGMII_RXC__ENET_RGMII_RXC,
	MX6DL_PAD_RGMII_RD0__ENET_RGMII_RD0,
	MX6DL_PAD_RGMII_RD1__ENET_RGMII_RD1,
	MX6DL_PAD_RGMII_RD2__ENET_RGMII_RD2,
	MX6DL_PAD_RGMII_RD3__ENET_RGMII_RD3,
	MX6DL_PAD_RGMII_RX_CTL__ENET_RGMII_RX_CTL,
	MX6DL_PAD_ENET_RXD1__GPIO_1_26,		            /* RGMII Phy Interrupt */
	MX6DL_PAD_ENET_CRS_DV__GPIO_1_25,		        /* RGMII reset */
#ifdef CONFIG_FEC_1588
	MX6DL_PAD_GPIO_16__ENET_ANATOP_ETHERNET_REF_OUT, /* Internal connect for 1588 TS Clock */
#endif

	/* USDHC3 (MicroSD SDIO CMD) */
	MX6DL_PAD_SD3_CLK__USDHC3_CLK_50MHZ,
	MX6DL_PAD_SD3_CMD__USDHC3_CMD_50MHZ,
	MX6DL_PAD_SD3_DAT0__USDHC3_DAT0_50MHZ,
	MX6DL_PAD_SD3_DAT1__USDHC3_DAT1_50MHZ,
	MX6DL_PAD_SD3_DAT2__USDHC3_DAT2_50MHZ,
	MX6DL_PAD_SD3_DAT3__USDHC3_DAT3_50MHZ,
	MX6DL_PAD_SD3_DAT5__GPIO_7_0,	// CD
	MX6DL_PAD_SD3_DAT4__GPIO_7_1,	// WP

	/* USDHC4 (eMMC SDIO) */
	MX6DL_PAD_SD4_CLK__USDHC4_CLK_50MHZ,
	MX6DL_PAD_SD4_CMD__USDHC4_CMD_50MHZ,
	MX6DL_PAD_SD4_DAT0__USDHC4_DAT0_50MHZ,
	MX6DL_PAD_SD4_DAT1__USDHC4_DAT1_50MHZ,
	MX6DL_PAD_SD4_DAT2__USDHC4_DAT2_50MHZ,
	MX6DL_PAD_SD4_DAT3__USDHC4_DAT3_50MHZ,
	MX6DL_PAD_SD4_DAT4__USDHC4_DAT4_50MHZ,
	MX6DL_PAD_SD4_DAT5__USDHC4_DAT5_50MHZ,
	MX6DL_PAD_SD4_DAT6__USDHC4_DAT6_50MHZ,
	MX6DL_PAD_SD4_DAT7__USDHC4_DAT7_50MHZ,
	MX6DL_PAD_NANDF_D6__GPIO_2_6,	// CD
	MX6DL_PAD_NANDF_D7__GPIO_2_7,	// WP

	/* PMIC_nINT(PMIC interrupt signal) */
	MX6DL_PAD_GPIO_18__GPIO_7_13,
	
	/* WDOG_B to reset pmic */
	MX6DL_PAD_GPIO_1__WDOG2_WDOG_B,
	
	/* TOUCH interrupt signal */
	MX6DL_PAD_EIM_D26__GPIO_3_26,	
};

#define MX6DL_USDHC_PAD_SETTING(id, speed)	\
mx6dl_sd##id##_##speed##mhz[] = {		\
	MX6DL_PAD_SD##id##_CLK__USDHC##id##_CLK_##speed##MHZ,	\
	MX6DL_PAD_SD##id##_CMD__USDHC##id##_CMD_##speed##MHZ,	\
	MX6DL_PAD_SD##id##_DAT0__USDHC##id##_DAT0_##speed##MHZ,	\
	MX6DL_PAD_SD##id##_DAT1__USDHC##id##_DAT1_##speed##MHZ,	\
	MX6DL_PAD_SD##id##_DAT2__USDHC##id##_DAT2_##speed##MHZ,	\
	MX6DL_PAD_SD##id##_DAT3__USDHC##id##_DAT3_##speed##MHZ,	\
	MX6DL_PAD_SD##id##_DAT4__USDHC##id##_DAT4_##speed##MHZ,	\
	MX6DL_PAD_SD##id##_DAT5__USDHC##id##_DAT5_##speed##MHZ,	\
	MX6DL_PAD_SD##id##_DAT6__USDHC##id##_DAT6_##speed##MHZ,	\
	MX6DL_PAD_SD##id##_DAT7__USDHC##id##_DAT7_##speed##MHZ,	\
}

static iomux_v3_cfg_t MX6DL_USDHC_PAD_SETTING(3, 50);
static iomux_v3_cfg_t MX6DL_USDHC_PAD_SETTING(3, 100);
static iomux_v3_cfg_t MX6DL_USDHC_PAD_SETTING(3, 200);
static iomux_v3_cfg_t MX6DL_USDHC_PAD_SETTING(4, 50);
static iomux_v3_cfg_t MX6DL_USDHC_PAD_SETTING(4, 100);
static iomux_v3_cfg_t MX6DL_USDHC_PAD_SETTING(4, 200);

#endif
