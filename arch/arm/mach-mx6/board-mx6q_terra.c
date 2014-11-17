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

#include <linux/types.h>
#include <linux/sched.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/nodemask.h>
#include <linux/clk.h>
#include <linux/platform_device.h>
#include <linux/fsl_devices.h>
#include <linux/spi/spi.h>
#include <linux/spi/flash.h>
#include <linux/i2c.h>
#include <linux/i2c/pca953x.h>
#include <linux/ata.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/map.h>
#include <linux/mtd/partitions.h>
#include <linux/regulator/consumer.h>
#include <linux/pmic_external.h>
#include <linux/pmic_status.h>
#include <linux/ipu.h>
#include <linux/mxcfb.h>
#include <linux/pwm_backlight.h>
#include <linux/fec.h>
#include <linux/memblock.h>
#include <linux/gpio.h>
#include <linux/etherdevice.h>
#include <linux/power/sabresd_battery.h>     
#include <linux/regulator/anatop-regulator.h>
#include <linux/regulator/consumer.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/fixed.h>
#ifdef CONFIG_IR_GPIO_CIR
#include <media/gpio-ir-recv.h>
#include <media/rc-map.h>
#endif
#include <linux/mfd/mxc-hdmi-core.h>         

#include <mach/common.h>
#include <mach/hardware.h>
#include <mach/mxc_dvfs.h>
#include <mach/memory.h>
#include <mach/iomux-mx6q.h>
#include <mach/imx-uart.h>
#include <mach/viv_gpu.h>
#include <mach/ahci_sata.h>
#include <mach/ipu-v3.h>
#include <mach/mxc_hdmi.h>
#include <mach/mxc_asrc.h>
#include <mach/mipi_dsi.h>
#include <mach/system.h>
#include <mach/imx_rfkill.h>

#include <asm/irq.h>
#include <asm/setup.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/time.h>

#include "usb.h"
#include "devices-imx6q.h"
#include "crm_regs.h"
#include "cpu_op-mx6.h"
#include "board-mx6q_terra.h"
#include "board-mx6dl_terra.h"


#define TERRA_SD3_CD		    IMX_GPIO_NR(7, 0)
#define TERRA_SD3_WP		    IMX_GPIO_NR(7, 1)
#define TERRA_SD4_CD		    IMX_GPIO_NR(6, 17)
#define TERRA_SD4_WP		    IMX_GPIO_NR(6, 18)

#define TERRA_TS_INT		    IMX_GPIO_NR(4, 8)
#define TERRA_PMIC_INT_B	    IMX_GPIO_NR(7, 13)

#define TERRA_SJA1000_IRQ	    IMX_GPIO_NR(1, 18)

#define TERRA_RGMII_RST	        IMX_GPIO_NR(1, 25)
#define TERRA_RGMII_INT	        IMX_GPIO_NR(1, 26)
#define MX6_ENET_IRQ		    IMX_GPIO_NR(1, 6)

#define IOMUX_OBSRV_MUX1_OFFSET	0x3c
#define OBSRV_MUX1_MASK			0x3f
#define OBSRV_MUX1_ENET_IRQ		0x9


static int max11801_mode = 1;
static int caam_enabled;

static struct clk *sata_clk;

extern char *gp_reg_id;
extern char *soc_reg_id;
extern char *pu_reg_id;
extern bool enet_to_gpio_6;


enum sd_pad_mode {
	SD_PAD_MODE_LOW_SPEED,
	SD_PAD_MODE_MED_SPEED,
	SD_PAD_MODE_HIGH_SPEED,
};

static int plt_sd_pad_change(unsigned int index, int clock)
{
	/* LOW speed is the default state of SD pads */
	static enum sd_pad_mode pad_mode = SD_PAD_MODE_LOW_SPEED;

	iomux_v3_cfg_t *sd_pads_200mhz = NULL;
	iomux_v3_cfg_t *sd_pads_100mhz = NULL;
	iomux_v3_cfg_t *sd_pads_50mhz = NULL;

	u32 sd_pads_200mhz_cnt=0;
	u32 sd_pads_100mhz_cnt=0;
	u32 sd_pads_50mhz_cnt=0;

	switch (index) {
	case 2:
		if (cpu_is_mx6q()) {
			sd_pads_200mhz = mx6q_sd3_200mhz;
			sd_pads_100mhz = mx6q_sd3_100mhz;
			sd_pads_50mhz  = mx6q_sd3_50mhz;

			sd_pads_200mhz_cnt = ARRAY_SIZE(mx6q_sd3_200mhz);
			sd_pads_100mhz_cnt = ARRAY_SIZE(mx6q_sd3_100mhz);
			sd_pads_50mhz_cnt = ARRAY_SIZE(mx6q_sd3_50mhz);
		} else if (cpu_is_mx6dl()) {
			sd_pads_200mhz = mx6dl_sd3_200mhz;
			sd_pads_100mhz = mx6dl_sd3_100mhz;
			sd_pads_50mhz = mx6dl_sd3_50mhz;

			sd_pads_200mhz_cnt = ARRAY_SIZE(mx6dl_sd3_200mhz);
			sd_pads_100mhz_cnt = ARRAY_SIZE(mx6dl_sd3_100mhz);
			sd_pads_50mhz_cnt = ARRAY_SIZE(mx6dl_sd3_50mhz);
		}
		break;
	case 3:
		if (cpu_is_mx6q()) {
			sd_pads_200mhz = mx6q_sd4_200mhz;
			sd_pads_100mhz = mx6q_sd4_100mhz;
			sd_pads_50mhz = mx6q_sd4_50mhz;

			sd_pads_200mhz_cnt = ARRAY_SIZE(mx6q_sd4_200mhz);
			sd_pads_100mhz_cnt = ARRAY_SIZE(mx6q_sd4_100mhz);
			sd_pads_50mhz_cnt = ARRAY_SIZE(mx6q_sd4_50mhz);
		} else if (cpu_is_mx6dl()) {
			sd_pads_200mhz = mx6dl_sd4_200mhz;
			sd_pads_100mhz = mx6dl_sd4_100mhz;
			sd_pads_50mhz = mx6dl_sd4_50mhz;

			sd_pads_200mhz_cnt = ARRAY_SIZE(mx6dl_sd4_200mhz);
			sd_pads_100mhz_cnt = ARRAY_SIZE(mx6dl_sd4_100mhz);
			sd_pads_50mhz_cnt = ARRAY_SIZE(mx6dl_sd4_50mhz);
		}
		break;
	default:
		printk(KERN_ERR "no such SD host controller index %d\n", index);
		return -EINVAL;
	}

	if (clock > 100000000) {
		if (pad_mode == SD_PAD_MODE_HIGH_SPEED)
			return 0;
		BUG_ON(!sd_pads_200mhz);
		pad_mode = SD_PAD_MODE_HIGH_SPEED;
		return mxc_iomux_v3_setup_multiple_pads(sd_pads_200mhz,
							sd_pads_200mhz_cnt);
	} else if (clock > 52000000) {
		if (pad_mode == SD_PAD_MODE_MED_SPEED)
			return 0;
		BUG_ON(!sd_pads_100mhz);
		pad_mode = SD_PAD_MODE_MED_SPEED;
		return mxc_iomux_v3_setup_multiple_pads(sd_pads_100mhz,
							sd_pads_100mhz_cnt);
	} else {
		if (pad_mode == SD_PAD_MODE_LOW_SPEED)
			return 0;
		BUG_ON(!sd_pads_50mhz);
		pad_mode = SD_PAD_MODE_LOW_SPEED;
		return mxc_iomux_v3_setup_multiple_pads(sd_pads_50mhz,
							sd_pads_50mhz_cnt);
	}
}

static const struct esdhc_platform_data mx6q_terra_sd3_data __initconst = {
	.cd_gpio = TERRA_SD3_CD,
	.wp_gpio = TERRA_SD3_WP,
	.always_present = 1,
	.keep_power_at_suspend = 1,
	.support_8bit = 0,
	.delay_line = 0,
	.cd_type = ESDHC_CD_CONTROLLER,
	.platform_pad_change	= plt_sd_pad_change,
};

static const struct esdhc_platform_data mx6q_terra_sd4_data __initconst = {
//	.cd_gpio = TERRA_SD4_CD,
//	.wp_gpio = TERRA_SD4_WP,
	.always_present = 1,
	.keep_power_at_suspend = 1,
	.support_8bit = 0,
	.delay_line = 0,
	.cd_type = ESDHC_CD_PERMANENT,
	.platform_pad_change	= plt_sd_pad_change,	
};


static struct imxi2c_platform_data mx6q_terra_i2c_data = {
	.bitrate = 100000,
};

/* I2C2 */
static struct i2c_board_info mxc_i2c1_board_info[] __initdata = {
	{
		I2C_BOARD_INFO("mxc_hdmi_i2c", 0x50),
	},
};

/* I2C3 */
static struct i2c_board_info mxc_i2c2_board_info[] __initdata = {
	{
		I2C_BOARD_INFO("ds1307", 0x68),
	},
	{
		I2C_BOARD_INFO("max11801", 0x48),
		.platform_data = (void *)&max11801_mode,
		.irq = gpio_to_irq(TERRA_TS_INT),
	},
};

static const struct anatop_thermal_platform_data
	mx6q_sabresd_anatop_thermal_data __initconst = {
		.name = "anatop_thermal",
};


static inline void mx6q_terra_init_uart(void)
{
	imx6q_add_imx_uart(0, NULL);    /* UART1 debug console */
	imx6q_add_imx_uart(1, NULL);    /* UART2               */
	imx6q_add_imx_uart(2, NULL);    /* UART3               */
	imx6q_add_imx_uart(3, NULL);    /* UART4               */
}

static int mx6q_sabresd_fec_phy_init(struct phy_device *phydev)
{
	unsigned short val;

	/* Ar8031 phy SmartEEE feature cause link status generates glitch,
	 * which cause ethernet link down/up issue, so disable SmartEEE
	 */
	phy_write(phydev, 0xd, 0x3);
	phy_write(phydev, 0xe, 0x805d);
	phy_write(phydev, 0xd, 0x4003);
	val = phy_read(phydev, 0xe);
	val &= ~(0x1 << 8);
	phy_write(phydev, 0xe, val);

	/* To enable AR8031 ouput a 125MHz clk from CLK_25M */
	phy_write(phydev, 0xd, 0x7);
	phy_write(phydev, 0xe, 0x8016);
	phy_write(phydev, 0xd, 0x4007);
	val = phy_read(phydev, 0xe);

	val &= 0xffe3;
	val |= 0x18;
	phy_write(phydev, 0xe, val);

	/* Introduce tx clock delay */
	phy_write(phydev, 0x1d, 0x5);
	val = phy_read(phydev, 0x1e);
	val |= 0x0100;
	phy_write(phydev, 0x1e, val);

	/*check phy power*/
	val = phy_read(phydev, 0x0);

	if (val & BMCR_PDOWN)
		phy_write(phydev, 0x0, (val & ~BMCR_PDOWN));

	return 0;
}

static struct fec_platform_data fec_data __initdata = {
	.init = mx6q_sabresd_fec_phy_init,
	.phy = PHY_INTERFACE_MODE_RGMII,
	.gpio_irq = MX6_ENET_IRQ,
};

static struct ipuv3_fb_platform_data terra_fb_data[] = {
	{ /*fb0*/
	.disp_dev = "hdmi",
	.interface_pix_fmt = IPU_PIX_FMT_RGB24,
	.mode_str = "1920x1080M@60",
	.default_bpp = 32,
	.int_clk = false,
	}, {
	.disp_dev = "ldb",
	.interface_pix_fmt = IPU_PIX_FMT_LVDS666,
	.mode_str = "LDB-XGA",
	.default_bpp = 16,
	.int_clk = false,
	}, {
	.disp_dev = "lcd",
	.interface_pix_fmt = IPU_PIX_FMT_RGB565,
	.mode_str = "CLAA-WVGA",
	.default_bpp = 16,
	.int_clk = false,
	.late_init = false,
	}, {
	.disp_dev = "ldb",
	.interface_pix_fmt = IPU_PIX_FMT_RGB666,
	.mode_str = "LDB-VGA",
	.default_bpp = 16,
	.int_clk = false,
	.late_init = false,
	},
};

static void hdmi_init(int ipu_id, int disp_id)
{
	int hdmi_mux_setting;

	if ((ipu_id > 1) || (ipu_id < 0)) {
		printk(KERN_ERR"Invalid IPU select for HDMI: %d. Set to 0\n", ipu_id);
		ipu_id = 0;
	}

	if ((disp_id > 1) || (disp_id < 0)) {
		printk(KERN_ERR"Invalid DI select for HDMI: %d. Set to 0\n", disp_id);
		disp_id = 0;
	}

	/* Configure the connection between IPU1/2 and HDMI */
	hdmi_mux_setting = 2*ipu_id + disp_id;

	/* GPR3, bits 2-3 = HDMI_MUX_CTL */
	mxc_iomux_set_gpr_register(3, 2, 2, hdmi_mux_setting);

	/* Set HDMI event as SDMA event2 while Chip version later than TO1.2 */
	if (hdmi_SDMA_check())
		mxc_iomux_set_gpr_register(0, 0, 1, 1);
}

/* On mx6x sabreauto board i2c2 iomux with hdmi ddc,
 * the pins default work at i2c2 function,
 when hdcp enable, the pins should work at ddc function */

static void hdmi_enable_ddc_pin(void)
{
	mxc_iomux_v3_setup_multiple_pads(mx6q_terra_hdmi_ddc_pads,
		ARRAY_SIZE(mx6q_terra_hdmi_ddc_pads));
}

static void hdmi_disable_ddc_pin(void)
{
	mxc_iomux_v3_setup_multiple_pads(mx6q_terra_i2c2_pads,
		ARRAY_SIZE(mx6q_terra_i2c2_pads));			
}

static struct fsl_mxc_hdmi_platform_data hdmi_data = {
	.init = hdmi_init,
	.enable_pins = hdmi_enable_ddc_pin,
	.disable_pins = hdmi_disable_ddc_pin,
//	.phy_reg_vlev = 0x0294,
//	.phy_reg_cksymtx = 0x800d,
};

static struct fsl_mxc_hdmi_core_platform_data hdmi_core_data = {
	.ipu_id		= 0,
	.disp_id	= 0,
};

static struct fsl_mxc_lcd_platform_data lcdif_data = {
	.ipu_id		= 0,
	.disp_id	= 0,
	.default_ifmt	= IPU_PIX_FMT_RGB565,
};

static struct fsl_mxc_ldb_platform_data ldb_data = {
	.ipu_id = 1,
	.disp_id = 0,
	.ext_ref = 1,
	.mode = LDB_SEP0,
	.sec_ipu_id = 1,
	.sec_disp_id = 1,
};

static struct imx_ipuv3_platform_data ipu_data[] = {
	{
	.rev = 4,
	.csi_clk[0] = "clko_clk",
	.bypass_reset = false,
	}, {
	.rev = 4,
	.csi_clk[0] = "clko_clk",
	.bypass_reset = false,
	},
};

static struct mxc_dvfs_platform_data terra_dvfscore_data = {
	.reg_id = "VDDCORE",
	.soc_id	= "VDDSOC",
	.clk1_id = "cpu_clk",
	.clk2_id = "gpc_dvfs_clk",
	.gpc_cntr_offset = MXC_GPC_CNTR_OFFSET,
	.ccm_cdcr_offset = MXC_CCM_CDCR_OFFSET,
	.ccm_cacrr_offset = MXC_CCM_CACRR_OFFSET,
	.ccm_cdhipr_offset = MXC_CCM_CDHIPR_OFFSET,
	.prediv_mask = 0x1F800,
	.prediv_offset = 11,
	.prediv_val = 3,
	.div3ck_mask = 0xE0000000,
	.div3ck_offset = 29,
	.div3ck_val = 2,
	.emac_val = 0x08,
	.upthr_val = 25,
	.dnthr_val = 9,
	.pncthr_val = 33,
	.upcnt_val = 10,
	.dncnt_val = 10,
	.delay_time = 80,
};

static struct viv_gpu_platform_data imx6q_gpu_pdata __initdata = {
	.reserved_mem_size = SZ_128M,
};

static void __init fixup_mxc_board(struct machine_desc *desc, struct tag *tags,
				   char **cmdline, struct meminfo *mi)
{
	char *str;
	struct tag *t;
	int i = 0;
	struct ipuv3_fb_platform_data *pdata_fb = terra_fb_data;

	for_each_tag(t, tags) {
		if (t->hdr.tag == ATAG_CMDLINE) {
			str = t->u.cmdline.cmdline;
			str = strstr(str, "fbmem=");
			if (str != NULL) {
				str += 6;
				pdata_fb[i++].res_size[0] = memparse(str, &str);
				while (*str == ',' &&
					i < ARRAY_SIZE(terra_fb_data)) {
					str++;
					pdata_fb[i++].res_size[0] = memparse(str, &str);
				}
			}
#ifdef CONFIG_ANDROID
			/* ION reserved memory */
			str = t->u.cmdline.cmdline;
			str = strstr(str, "ionmem=");
			if (str != NULL) {
				str += 7;
				imx_ion_data.heaps[0].size = memparse(str, &str);
			}
#endif
			/* Primary framebuffer base address */
			str = t->u.cmdline.cmdline;
			str = strstr(str, "fb0base=");
			if (str != NULL) {
				str += 8;
				pdata_fb[0].res_base[0] =
						simple_strtol(str, &str, 16);
			}
			/* GPU reserved memory */
			str = t->u.cmdline.cmdline;
			str = strstr(str, "gpumem=");
			if (str != NULL) {
				str += 7;
				imx6q_gpu_pdata.reserved_mem_size = memparse(str, &str);
			}
			break;
		}
	}
}

static void __init imx6q_terra_init_usb(void)
{
	imx_otg_base = MX6_IO_ADDRESS(MX6Q_USB_OTG_BASE_ADDR);

	/*
	 * ID pin is sampled from ENET_RX_ER. Notice that this pad is configured
	 * to be pulled-down 100kOhm by default.
	 * mxc_iomux_set_gpr_register(int group, int start_bit, int num_bits, int value)
	 */
	mxc_iomux_set_gpr_register(1, 13, 1, 0);

//	mx6_set_otghost_vbus_func(imx6q_cubox_i_usbotg_vbus);
//	mx6_set_host1_vbus_func(imx6q_cubox_i_host1_vbus);
}

static struct regulator_consumer_supply terra_vmmc_consumers[] = {
	REGULATOR_SUPPLY("vmmc", "sdhci-esdhc-imx.1"),
	REGULATOR_SUPPLY("vmmc", "sdhci-esdhc-imx.2"),
	REGULATOR_SUPPLY("vmmc", "sdhci-esdhc-imx.3"),	
};

static struct regulator_init_data terra_vmmc_init = {
	.num_consumer_supplies = ARRAY_SIZE(terra_vmmc_consumers),
	.consumer_supplies = terra_vmmc_consumers,
};

static struct fixed_voltage_config terra_vmmc_reg_config = {
	.supply_name	= "vmmc",
	.microvolts		= 3300000,
	.gpio			= -1,
	.init_data		= &terra_vmmc_init,
};

static struct platform_device terra_vmmc_reg_devices = {
	.name	= "reg-fixed-voltage",
	.id	= 3,
	.dev	= {
		.platform_data = &terra_vmmc_reg_config,
	},
};

static struct platform_pwm_backlight_data mx6_terra_pwm_backlight_data = {
	.pwm_id = 0,
	.max_brightness = 248,
	.dft_brightness = 128,
	.pwm_period_ns = 50000,
};

static int __init caam_setup(char *__unused)
{
	caam_enabled = 1;
	return 1;
}
early_param("caam", caam_setup);


/* HW Initialization, if return 0, initialization is successful. */
static int mx6q_terra_sata_init(struct device *dev, void __iomem *addr)
{
	u32 tmpdata;
	int ret = 0;
	struct clk *clk;

	sata_clk = clk_get(dev, "imx_sata_clk");
	if (IS_ERR(sata_clk)) {
		dev_err(dev, "no sata clock.\n");
		return PTR_ERR(sata_clk);
	}
	ret = clk_enable(sata_clk);
	if (ret) {
		dev_err(dev, "can't enable sata clock.\n");
		goto put_sata_clk;
	}

	/* Set PHY Paremeters, two steps to configure the GPR13,
	 * one write for rest of parameters, mask of first write is 0x07FFFFFD,
	 * and the other one write for setting the mpll_clk_off_b
	 *.rx_eq_val_0(iomuxc_gpr13[26:24]),
	 *.los_lvl(iomuxc_gpr13[23:19]),
	 *.rx_dpll_mode_0(iomuxc_gpr13[18:16]),
	 *.sata_speed(iomuxc_gpr13[15]),
	 *.mpll_ss_en(iomuxc_gpr13[14]),
	 *.tx_atten_0(iomuxc_gpr13[13:11]),
	 *.tx_boost_0(iomuxc_gpr13[10:7]),
	 *.tx_lvl(iomuxc_gpr13[6:2]),
	 *.mpll_ck_off(iomuxc_gpr13[1]),
	 *.tx_edgerate_0(iomuxc_gpr13[0]),
	 */
	tmpdata = readl(IOMUXC_GPR13);
	writel(((tmpdata & ~0x07FFFFFD) | 0x0593A044), IOMUXC_GPR13);

	/* enable SATA_PHY PLL */
	tmpdata = readl(IOMUXC_GPR13);
	writel(((tmpdata & ~0x2) | 0x2), IOMUXC_GPR13);

	/* Get the AHB clock rate, and configure the TIMER1MS reg later */
	clk = clk_get(NULL, "ahb");
	if (IS_ERR(clk)) {
		dev_err(dev, "no ahb clock.\n");
		ret = PTR_ERR(clk);
		goto release_sata_clk;
	}
	tmpdata = clk_get_rate(clk) / 1000;
	clk_put(clk);

#ifdef CONFIG_SATA_AHCI_PLATFORM
	ret = sata_init(addr, tmpdata);
	if (ret == 0)
		return ret;
#else
	usleep_range(1000, 2000);
	/* AHCI PHY enter into PDDQ mode if the AHCI module is not enabled */
	tmpdata = readl(addr + PORT_PHY_CTL);
	writel(tmpdata | PORT_PHY_CTL_PDDQ_LOC, addr + PORT_PHY_CTL);
	pr_info("No AHCI save PWR: PDDQ %s\n", ((readl(addr + PORT_PHY_CTL)
					>> 20) & 1) ? "enabled" : "disabled");
#endif

release_sata_clk:
	/* disable SATA_PHY PLL */
	writel((readl(IOMUXC_GPR13) & ~0x2), IOMUXC_GPR13);
	clk_disable(sata_clk);
put_sata_clk:
	clk_put(sata_clk);

	return ret;
}

#ifdef CONFIG_SATA_AHCI_PLATFORM
static void mx6q_terra_sata_exit(struct device *dev)
{
	clk_disable(sata_clk);
	clk_put(sata_clk);
}

static struct ahci_platform_data mx6q_terra_sata_data = {
	.init = mx6q_terra_sata_init,
	.exit = mx6q_terra_sata_exit,
};
#endif


static void mx6q_flexcan0_switch(int enable)
{
}
static void mx6q_flexcan1_switch(int enable)
{
}

static const struct flexcan_platform_data
		mx6q_terra_flexcan_pdata[] __initconst = {
	{
		.transceiver_switch = mx6q_flexcan0_switch,
	}, {
		.transceiver_switch = mx6q_flexcan1_switch,
	}
};

#if defined(CONFIG_CAN_SJA1000)	//============================================================
#include <linux/can/platform/sja1000.h>

static struct sja1000_platform_data mx6_terra_sja1000_data = {
	.osc_freq	= 16000000,
	.ocr		= (OCR_TX0_PUSHPULL | OCR_TX1_PUSHPULL) | OCR_MODE_BIPHASE,
	.cdr    	= CDR_CBP | 0x0,
};
#endif

static void mx6q_setup_weimcs(void)
{
	void __iomem *eim_reg = MX6_IO_ADDRESS(WEIM_BASE_ADDR);
	void __iomem *ccm_reg = MX6_IO_ADDRESS(CCM_BASE_ADDR);
	unsigned int reg;
	struct clk *clk;
	u32 rate;
	u32 tmpdata;

	/* CLKCTL_CCGR6: Set emi_slow_clock to be on in all modes */
	reg = readl(ccm_reg + 0x80);
	reg |= 0x00000C00;
	writel(reg, ccm_reg + 0x80);

	/* These setting assume that the EIM_SLOW_CLOCK is set to 132 MHz */
	clk = clk_get(NULL, "emi_slow_clk");
	if (IS_ERR(clk))
		printk(KERN_ERR "emi_slow_clk not found\n");

	rate = clk_get_rate(clk);
	if (rate != 132000000)
		printk(KERN_ERR "Warning: emi_slow_clk not set to 132 MHz!"
		       " WEIM NOR timing may be incorrect!\n");


    /*****************************************************************************/
    /*
     * IOMUXC_GPR1
     *
     *      CS0(128M), CS1 (0M), CS2 (0M), CS3(0M) [default configuration]
     *      CS0(64M), CS1(64M), CS2(0M), CS3(0M)
     *      CS0(64M), CS1(32M), CS2(32M), CS3(0M)
     *      CS0(32M), CS1(32M), CS2(32M), CS3(32M)
     */
	tmpdata  = readl(IOMUXC_GPR1);
	tmpdata &= 0xfffff249;		// CS0, CS1, CS2, CS3 Active
	tmpdata |= 0x249;
	writel(tmpdata, IOMUXC_GPR1);


    /*****************************************************************************/    
	/*
	 * For EIM General Configuration registers. ( EIM_CS0 : 0x08000000 )
	 *
	 * CS0GCR1:
	 *	GBC = 0; CSREC = 6; 
	 *  DSZ = 1;
            001 : 16 bit port resides on DATA[15:0]
            010 : 16 bit port resides on DATA[31:16]
            011 : 32 bit port resides on DATA[31:0]
            100 :  8 bit port resides on DATA[7:0]
            101 :  8 bit port resides on DATA[15:8]
            110 :  8 bit port resides on DATA[23:16]
            111 :  8 bit port resides on DATA[31:24]
	 *  BL = 0;
	 *	CREP = 1; CSEN = 1;
	 *
	 *	EIM Operation Mode: 
	 *      MUM = 1 : multiplexed       0 : none multiplexed 
	 *      SRD = 0 : Async read        1 : Sync read
	 *      SWR = 0 : Async write       1 : Sync write
	 *
	 * CS0GCR2:
	 *	ADH = 1
	 */
	writel(0x00610089, eim_reg);                // EIM_CS0GCR1 : 0x00620081
	writel(0x00000001, eim_reg + 0x00000004);   // EIM_CS0GCR2 : 0x00000001

	/*
	 * For EIM Read Configuration registers.
	 *
	 * CS0RCR1:
	 *	RWSC = 1C;
	 *	RADVA = 0; RADVN = 2;
	 *	OEA = 2; OEN = 0;
	 *	RCSA = 0; RCSN = 0
	 *
	 * CS0RCR2:
	 *	APR = 1 (Async Page Read);
	 *	PAT = 4 (6 EIM clock sycles)
	 */
	writel(0x1C022000, eim_reg + 0x00000008);   // EIM_CS0RCR1 : 0x1C022000
	writel(0x0000C000, eim_reg + 0x0000000C);   // EIM_CS0RCR2 : 0x0000C000

	/*
	 * For EIM Write Configuration registers.
	 *
	 * CS0WCR1:
	 *	WWSC = 20;
	 *	WADVA = 0; WADVN = 1;
	 *	WBEA = 1; WBEN = 2;
	 *	WEA = 1; WEN = 6;
	 *	WCSA = 1; WCSN = 2;
	 *
	 * CS0WCR2:
	 *	WBCDD = 0
	 */
	writel(0x1404a38e, eim_reg + 0x00000010);   // EIM_CS0WCR1 : 0x1404a38e
	writel(0x00000000, eim_reg + 0x00000014);   // EIM_CS0WCR2 : 0x00000000


    /*****************************************************************************/
	/*
	 * For EIM General Configuration registers. ( EIM_CS1 : 0x0A000000 )
	 *
	 * CS1GCR1:
	 *	GBC = 0; CSREC = 6; 
	 *  DSZ = 1;
            001 : 16 bit port resides on DATA[15:0]
            010 : 16 bit port resides on DATA[31:16]
            011 : 32 bit port resides on DATA[31:0]
            100 :  8 bit port resides on DATA[7:0]
            101 :  8 bit port resides on DATA[15:8]
            110 :  8 bit port resides on DATA[23:16]
            111 :  8 bit port resides on DATA[31:24]
	 *  BL = 0;
	 *	CREP = 1; CSEN = 1;
	 *
	 *	EIM Operation Mode: 
	 *      MUM = 1 : multiplexed       0 : none multiplexed 
	 *      SRD = 0 : Async read        1 : Sync read
	 *      SWR = 0 : Async write       1 : Sync write
	 *
	 * CS1GCR2:
	 *	ADH = 2
	 */
	writel(0x00610089, eim_reg + 0x00000018);   // EIM_CS1GCR1 : 0x00620081
	writel(0x00000002, eim_reg + 0x0000001C);   // EIM_CS1GCR2 : 0x00000001

	/*
	 * For EIM Read Configuration registers.
	 *
	 * CS1RCR1:
	 *	RWSC = 1C;
	 *	RADVA = 0; RADVN = 2;
	 *	OEA = 2; OEN = 0;
	 *	RCSA = 0; RCSN = 0
	 *
	 * CS1RCR2:
	 *	APR = 1 (Async Page Read);
	 *	PAT = 4 (6 EIM clock sycles)
	 */
	writel(0x1C022000, eim_reg + 0x00000020);   // EIM_CS1RCR1 : 0x1C022000
	writel(0x0000C000, eim_reg + 0x00000024);   // EIM_CS1RCR2 : 0x0000C000

	/*
	 * For EIM Write Configuration registers.
	 *
	 * CS1WCR1:
	 *	WWSC = 20;
	 *	WADVA = 0; WADVN = 1;
	 *	WBEA = 1; WBEN = 2;
	 *	WEA = 1; WEN = 6;
	 *	WCSA = 1; WCSN = 2;
	 *
	 * CS1WCR2:
	 *	WBCDD = 0
	 */
	writel(0x1404a38e, eim_reg + 0x00000028);   // EIM_CS1WCR1 : 0x1404a38e
	writel(0x00000000, eim_reg + 0x0000002C);   // EIM_CS1WCR2 : 0x00000000
}


static void __init mx6_board_init(void)
{
	int i;
	struct clk *clko, *clko2;
	struct clk *new_parent;
	int rate;
    
	if (cpu_is_mx6q()) {
		mxc_iomux_v3_setup_multiple_pads(mx6q_terra_pads,
			ARRAY_SIZE(mx6q_terra_pads));
        if (enet_to_gpio_6) {
			iomux_v3_cfg_t enet_gpio_pad =
				MX6Q_PAD_GPIO_6__ENET_IRQ_TO_GPIO_6;
			mxc_iomux_v3_setup_pad(enet_gpio_pad);
		} else {
			iomux_v3_cfg_t i2c3_pad =
				MX6Q_PAD_GPIO_6__I2C3_SDA;
			mxc_iomux_v3_setup_pad(i2c3_pad);
		}			
	} else if (cpu_is_mx6dl()) {
		mxc_iomux_v3_setup_multiple_pads(mx6dl_terra_pads,
			ARRAY_SIZE(mx6dl_terra_pads));
		if (enet_to_gpio_6) {
			iomux_v3_cfg_t enet_gpio_pad =
				MX6DL_PAD_GPIO_6__ENET_IRQ_TO_GPIO_6;
			mxc_iomux_v3_setup_pad(enet_gpio_pad);
		} else {
			iomux_v3_cfg_t i2c3_pad =
				MX6DL_PAD_GPIO_6__I2C3_SDA;
			mxc_iomux_v3_setup_pad(i2c3_pad);
		}
	}

#ifdef CONFIG_FEC_1588
	/* Set GPIO_16 input for IEEE-1588 ts_clk and RMII reference clock
	 * For MX6 GPR1 bit21 meaning:
	 * Bit21:       0 - GPIO_16 pad output
	 *              1 - GPIO_16 pad input
	 * mxc_iomux_set_gpr_register(int group, int start_bit, int num_bits, int value)	 
	 */
	 mxc_iomux_set_gpr_register(1, 21, 1, 1);
#endif
	
	gp_reg_id  = terra_dvfscore_data.reg_id;
	soc_reg_id = terra_dvfscore_data.soc_id;
	mx6q_terra_init_uart();

    mx6q_setup_weimcs();

	if (cpu_is_mx6dl()) {
		ldb_data.ipu_id = 0;
		ldb_data.sec_ipu_id = 0;
	}
	imx6q_add_mxc_hdmi_core(&hdmi_core_data);

	imx6q_add_ipuv3(0, &ipu_data[0]);
	if (cpu_is_mx6q()) {
		imx6q_add_ipuv3(1, &ipu_data[1]);
		for (i = 0; i < 4 && i < ARRAY_SIZE(terra_fb_data); i++)
			imx6q_add_ipuv3fb(i, &terra_fb_data[i]);
	} else
		for (i = 0; i < 2 && i < ARRAY_SIZE(terra_fb_data); i++)
			imx6q_add_ipuv3fb(i, &terra_fb_data[i]);

	imx6q_add_vdoa();
	imx6q_add_ldb(&ldb_data);
//	imx6q_add_imx_snvs_rtc();

	if (1 == caam_enabled)
		imx6q_add_imx_caam();

	imx6q_add_imx_i2c(1, &mx6q_terra_i2c_data);
	imx6q_add_imx_i2c(2, &mx6q_terra_i2c_data);
	i2c_register_board_info(1, mxc_i2c1_board_info,
			ARRAY_SIZE(mxc_i2c1_board_info));
	i2c_register_board_info(2, mxc_i2c2_board_info,
			ARRAY_SIZE(mxc_i2c2_board_info));

	imx6q_add_mxc_hdmi(&hdmi_data);

	imx6q_add_anatop_thermal_imx(1, &mx6q_sabresd_anatop_thermal_data);

	if (enet_to_gpio_6)
		/* Make sure the IOMUX_OBSRV_MUX1 is set to ENET_IRQ. */
		mxc_iomux_set_specialbits_register(
			IOMUX_OBSRV_MUX1_OFFSET,
			OBSRV_MUX1_ENET_IRQ,
			OBSRV_MUX1_MASK);
	else
		fec_data.gpio_irq = -1;
	imx6_init_fec(fec_data);

	/* Set IPU AXI transactions with QoS = 0xf as a real time access */
	mxc_iomux_set_gpr_register(6, 0, 32, 0xffffffff);

	imx6q_add_sdhci_usdhc_imx(3, &mx6q_terra_sd4_data);
	imx6q_add_sdhci_usdhc_imx(2, &mx6q_terra_sd3_data);

	imx_add_viv_gpu(&imx6_gpu_data, &imx6q_gpu_pdata);
	imx6q_terra_init_usb();

	if (cpu_is_mx6q()) {
#ifdef CONFIG_SATA_AHCI_PLATFORM
		imx6q_add_ahci(0, &mx6q_terra_sata_data);
#else
		mx6q_terra_sata_init(NULL,
			(void __iomem *)ioremap(MX6Q_SATA_BASE_ADDR, SZ_4K));
#endif
	}

	imx6q_add_vpu();

    platform_device_register(&terra_vmmc_reg_devices);
	
	imx6q_add_mxc_pwm(0);
	imx6q_add_mxc_pwm_backlight(0, &mx6_terra_pwm_backlight_data);

    
    mxc_iomux_v3_setup_multiple_pads(mx6q_terra_can_pads,
			ARRAY_SIZE(mx6q_terra_can_pads));

    imx6q_add_flexcan0(&mx6q_terra_flexcan_pdata[0]);
	imx6q_add_flexcan1(&mx6q_terra_flexcan_pdata[1]);

#ifdef CONFIG_CAN_SJA1000
	/* Add PCIe RC interface support */
	imx6q_add_sja1000(&mx6_terra_sja1000_data);
#endif

	imx6q_add_otp();
	imx6q_add_viim();
	imx6q_add_imx2_wdt(0, NULL);
	imx6q_add_dma();

	imx6q_add_dvfs_core(&terra_dvfscore_data);

	imx6q_add_hdmi_soc();
	imx6q_add_hdmi_soc_dai();

	if (cpu_is_mx6dl()) {
		imx6dl_add_imx_pxp();
		imx6dl_add_imx_pxp_client();
	}

	clko2 = clk_get(NULL, "clko2_clk");
	if (IS_ERR(clko2))
		pr_err("can't get CLKO2 clock.\n");

	new_parent = clk_get(NULL, "osc_clk");
	if (!IS_ERR(new_parent)) {
		clk_set_parent(clko2, new_parent);
		clk_put(new_parent);
	}
	rate = clk_round_rate(clko2, 24000000);
	clk_set_rate(clko2, rate);
	clk_enable(clko2);

	/* Camera and audio use osc clock */
	clko = clk_get(NULL, "clko_clk");
	if (!IS_ERR(clko))
		clk_set_parent(clko, clko2);

	imx6q_add_busfreq();

	imx6_add_armpmu();
	imx6q_add_perfmon(0);
	imx6q_add_perfmon(1);
	imx6q_add_perfmon(2);
}

extern void __iomem *twd_base;
static void __init mx6_timer_init(void)
{
	struct clk *uart_clk;
#ifdef CONFIG_LOCAL_TIMERS
	twd_base = ioremap(LOCAL_TWD_ADDR, SZ_256);
	BUG_ON(!twd_base);
#endif
	mx6_clocks_init(32768, 24000000, 0, 0);

	uart_clk = clk_get_sys("imx-uart.0", NULL);
	early_console_setup(UART1_BASE_ADDR, uart_clk);
}

static struct sys_timer mxc_timer = {
	.init   = mx6_timer_init,
};

void __init mx6q_terra_reserve(void)
{
#if defined(CONFIG_MXC_GPU_VIV) || defined(CONFIG_MXC_GPU_VIV_MODULE)
	phys_addr_t phys;

	if (imx6q_gpu_pdata.reserved_mem_size) {
		phys = memblock_alloc_base(imx6q_gpu_pdata.reserved_mem_size,
					   SZ_4K, SZ_2G);   // DDR3 2Gbbye : SZ_2G,  DDR3_1Gbyte : SZ_1G
		memblock_remove(phys, imx6q_gpu_pdata.reserved_mem_size);
		imx6q_gpu_pdata.reserved_mem_base = phys;
	}
#endif
}

/*
 * initialize __mach_desc_MX6Q_TERRA data structure.
 */
MACHINE_START(MX6Q_TERRA, "Falinux i.MX 6Quad TERRA Board")
	/* Maintainer: Falinux ltd. */
	.boot_params = MX6_PHYS_OFFSET + 0x100,
	.fixup = fixup_mxc_board,
	.map_io = mx6_map_io,
	.init_irq = mx6_init_irq,
	.init_machine = mx6_board_init,
	.timer = &mxc_timer,
	.reserve = mx6q_terra_reserve,
MACHINE_END
