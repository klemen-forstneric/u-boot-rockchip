/*
 * eFuse driver for Rockchip devices
 *
 * Copyright 2017, Theobroma Systems Design und Consulting GmbH
 * Written by Philipp Tomsich <philipp.tomsich@theobroma-systems.com>
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <common.h>
#include <asm/io.h>
#include <command.h>
#include <display_options.h>
#include <dm.h>
#include <linux/bitops.h>
#include <linux/delay.h>
#include <misc.h>

#define RK3288_A_SHIFT          6
#define RK3288_A_MASK           0x3ff
#define RK3288_NFUSES           32
#define RK3288_BYTES_PER_FUSE   1
#define RK3288_PGENB            BIT(3)
#define RK3288_LOAD             BIT(2)
#define RK3288_STROBE           BIT(1)
#define RK3288_CSB              BIT(0)

#define NUM_BITS_IN_BYTE 8
#define SEVENTH_BIT (1 << 7)

struct rockchip_efuse_regs {
	u32 ctrl;      /* 0x00  efuse control register */
	u32 dout;      /* 0x04  efuse data out register */
	u32 rf;        /* 0x08  efuse redundancy bit used register */
	u32 _rsvd0;
	u32 jtag_pass; /* 0x10  JTAG password */
	u32 strobe_finish_ctrl;
		       /* 0x14	efuse strobe finish control register */
};

struct rockchip_efuse_platdata {
	void __iomem *base;
	struct clk *clk;
};

static int read_efuses(cmd_tbl_t *cmdtp, int flag, int argc, char * const argv[]) {
	struct udevice *dev;
	u8 fuses[RK3288_NFUSES * RK3288_BYTES_PER_FUSE] = {0};
	int ret;

	/* retrieve the device */
	ret = uclass_get_device_by_driver(UCLASS_MISC, DM_GET_DRIVER(rockchip_efuse), &dev);

	if (ret) {
		printf("%s: no misc-device found\n", __func__);
		return 0;
	}

	ret = misc_read(dev, 0, &fuses, sizeof(fuses));
	if (ret) {
		printf("%s: misc_read failed\n", __func__);
		return 0;
	}

	printf("efuse-contents:\n");
	print_buffer(0, fuses, 1, 128, 16);

	return 0;
}

static int write_efuses(cmd_tbl_t *cmdtp, int flag, int argc, char * const argv[]) {
	struct udevice *dev;
	int ret;

	/* retrieve the device */
	ret = uclass_get_device_by_driver(UCLASS_MISC, DM_GET_DRIVER(rockchip_efuse), &dev);
	if (ret) {
		printf("%s: no misc-device found\n", __func__);
		return 0;
	}

	ret = misc_write(dev, 0, argv[1], strlen(argv[1]));
	if (ret) {
		printf("%s: misc_write failed\n", __func__);
		return 0;
	}

	return 0;
}

U_BOOT_CMD(
	rockchip_read_efuses, 1, 0, read_efuses,
	"Read the content of the efuses",
	""
);

U_BOOT_CMD(
	rockchip_write_efuses, 2, 0, write_efuses,
	"Write the content of the efuses",
	""
);

static int rockchip_efuse_read(struct udevice *dev, int offset, void *buf, int size) {
	struct rockchip_efuse_platdata *plat = dev_get_platdata(dev);
	struct rockchip_efuse_regs *efuse = (struct rockchip_efuse_regs *)plat->base;
	u8 *buffer = buf;
	int max_size = RK3288_NFUSES * RK3288_BYTES_PER_FUSE;

	if (size > (max_size - offset))
		size = max_size - offset;

	/* Switch to read mode */
	writel(RK3288_LOAD | RK3288_PGENB, &efuse->ctrl);
	udelay(1);

	while (size--) {
		writel(readl(&efuse->ctrl) &
				(~(RK3288_A_MASK << RK3288_A_SHIFT)),
				&efuse->ctrl);
		/* set addr */
		writel(readl(&efuse->ctrl) |
				((offset++ & RK3288_A_MASK) << RK3288_A_SHIFT),
				&efuse->ctrl);
		udelay(1);
		/* strobe low to high */
		writel(readl(&efuse->ctrl) |
				RK3288_STROBE, &efuse->ctrl);
		ndelay(60);
		/* read data */
		*buffer++ = readl(&efuse->dout);
		/* reset strobe to low */
		writel(readl(&efuse->ctrl) &
				(~RK3288_STROBE), &efuse->ctrl);
		udelay(1);
	}

	/* Switch to standby mode */
	writel(RK3288_PGENB | RK3288_CSB, &efuse->ctrl);

	return 0;
}

static int rockchip_efuse_write(struct udevice *dev, int offset, void *buf, int size) {
	struct rockchip_efuse_platdata *plat = dev_get_platdata(dev);
	struct rockchip_efuse_regs *efuse = (struct rockchip_efuse_regs *)plat->base;
	u8 *buffer = buf;
	int max_size = RK3288_NFUSES * RK3288_BYTES_PER_FUSE;

	if (size > (max_size - offset))
		size = max_size - offset;

  // Switch to pgm mode by setting load and pgenb to low
	writel(readl(&efuse->ctrl) &
				(~(RK3288_LOAD | RK3288_PGENB | RK3288_CSB)), &efuse->ctrl);

  int i;
  for(i = 0; i < size; ++i) {
    u8 current = buffer[i];
    u8 bitmask = SEVENTH_BIT;

    int j;
    for (j = 0; j < NUM_BITS_IN_BYTE; ++j) {
      if (current & bitmask) {
        writel(readl(&efuse->ctrl) &
            (~(RK3288_A_MASK << RK3288_A_SHIFT)),
            &efuse->ctrl);

        // Set address.
        writel(readl(&efuse->ctrl) |
            ((offset & RK3288_A_MASK) << RK3288_A_SHIFT),
            &efuse->ctrl);
        udelay(1);

        // Set strobe low to high.
        writel(readl(&efuse->ctrl) |
            RK3288_STROBE, &efuse->ctrl);

        // Wait for the fuses to blow.
        ndelay(60);

        // Reset strobe to low.
        writel(readl(&efuse->ctrl) &
            (~RK3288_STROBE), &efuse->ctrl);
        udelay(1);
      }

      ++offset;
      bitmask >>= 1;
    }
  }

	// Switch to standby mode
	writel(RK3288_PGENB | RK3288_CSB, &efuse->ctrl);

  return 0;
}

static const struct misc_ops rockchip_efuse_ops = {
	.read = rockchip_efuse_read,
  .write = rockchip_efuse_write,
};

static int rockchip_efuse_ofdata_to_platdata(struct udevice *dev)
{
	struct rockchip_efuse_platdata *plat = dev_get_platdata(dev);

	plat->base = dev_read_addr_ptr(dev);
	return 0;
}

static const struct udevice_id rockchip_efuse_ids[] = {
	{
		.compatible = "rockchip,rk3399-efuse",
	},
	{}
};

U_BOOT_DRIVER(rockchip_efuse) = {
	.name = "rockchip_efuse",
	.id = UCLASS_MISC,
	.of_match = rockchip_efuse_ids,
	.ofdata_to_platdata = rockchip_efuse_ofdata_to_platdata,
	.platdata_auto_alloc_size = sizeof(struct rockchip_efuse_platdata),
	.ops = &rockchip_efuse_ops,
};
