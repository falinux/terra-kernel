/*
 * Copyright (C) 2012-2013 Freescale Semiconductor, Inc. All Rights Reserved.
 */

/*
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

#include <mach/hardware.h>
#include <mach/devices-common.h>
#include <mach/gpio.h>

#define imx_sja1000_data_entry_single(soc, _id, _hwid, size)		\
	{								\
		.id = _id,						\
		.iobase = soc ## _SJA1000 ## _hwid ## _BASE_ADDR,	\
		.iosize = size,						\
		.irq	= gpio_to_irq(soc ## _INT_SJA1000 ## _hwid),			\
	}

#define imx_sja1000_data_entry(soc, _id, _hwid, _size)			\
	[_id] = imx_sja1000_data_entry_single(soc, _id, _hwid, _size)

#ifdef CONFIG_SOC_IMX6Q
#define MX6Q_SJA1000_BASE_ADDR (CS0_BASE_ADDR)      // arch/arm/plat-mxc/include/mach/mx6.h 0x08000000
#define MX6Q_INT_SJA1000    IMX_GPIO_NR(1, 18)
const struct imx_sja1000_data imx6q_sja1000_data __initconst =
			imx_sja1000_data_entry_single(MX6Q, 0, , 0x10000);
#endif


struct platform_device *__init imx_add_sja1000(
		const struct imx_sja1000_data *data,
		const struct sja1000_platform_data *pdata)
{
	struct resource res[] = {
		{
			.start = data->iobase,
			.end   = data->iobase + data->iosize - 1,
			.flags = IORESOURCE_MEM | IORESOURCE_MEM_16BIT,
		}, {
			.start = data->irq,
			.end   = data->irq,
			.flags = IORESOURCE_IRQ,
		},
	};

	return imx_add_platform_device("sja1000_platform", 0,
			res, ARRAY_SIZE(res),
			pdata, sizeof(*pdata));
}
