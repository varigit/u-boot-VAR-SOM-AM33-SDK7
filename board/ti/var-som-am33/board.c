/*
 * board.c
 *
 * Board functions for Variscite AM335X based SOMs
 *
 * Copyright (C) 2014, Variscite, LTD - http://www.variscite.com/
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <common.h>
#include <errno.h>
#include <spl.h>
#include <asm/arch/cpu.h>
#include <asm/arch/hardware.h>
#include <asm/arch/omap.h>
#include <asm/arch/ddr_defs.h>
#include <asm/arch/clock.h>
#include <asm/arch/gpio.h>
#include <asm/arch/mmc_host_def.h>
#include <asm/arch/sys_proto.h>
#include <asm/arch/mem.h>
#include <asm/io.h>
#include <asm/emif.h>
#include <asm/gpio.h>
#include <i2c.h>
#include <miiphy.h>
#include <cpsw.h>
#include <power/tps65910.h>
#include "board.h"

DECLARE_GLOBAL_DATA_PTR;

#define GPIO_LCD_BACKLIGHT     2
#define GPIO_BT_UART_SELECT    20
#define GPIO_SOM_REV_BIT0_GPIO 77
#define GPIO_SOM_REV_BIT1_GPIO 86
#define GPIO_SOM_REV_BIT2_GPIO 75

#define GPIO_PHY1_RST          83
#define GPIO_PHY2_RST          106

static struct ctrl_dev *cdev = (struct ctrl_dev *)CTRL_DEVICE_BASE;
static int som_rev = (-1);

#if defined(CONFIG_SPL_BUILD) || defined(CONFIG_NOR_BOOT)
static const struct ddr_data ddr3_var_am33x_data = {
	.datardsratio0 = MT41K256M16HA125E_RD_DQS,
	.datawdsratio0 = MT41K256M16HA125E_WR_DQS,
	.datafwsratio0 = MT41K256M16HA125E_PHY_FIFO_WE,
	.datawrsratio0 = MT41K256M16HA125E_PHY_WR_DATA,
};

static const struct cmd_control ddr3_var_am33x_cmd_ctrl_data = {
	.cmd0csratio = MT41K256M16HA125E_RATIO,
	.cmd0iclkout = MT41K256M16HA125E_INVERT_CLKOUT,

	.cmd1csratio = MT41K256M16HA125E_RATIO,
	.cmd1iclkout = MT41K256M16HA125E_INVERT_CLKOUT,

	.cmd2csratio = MT41K256M16HA125E_RATIO,
	.cmd2iclkout = MT41K256M16HA125E_INVERT_CLKOUT,
};

static struct emif_regs ddr3_var_am33x_emif_reg_data = {
	.sdram_config = MT41K256M16HA125E_EMIF_SDCFG,
	.ref_ctrl = MT41K256M16HA125E_EMIF_SDREF,
	.sdram_tim1 = MT41K256M16HA125E_EMIF_TIM1,
	.sdram_tim2 = MT41K256M16HA125E_EMIF_TIM2,
	.sdram_tim3 = MT41K256M16HA125E_EMIF_TIM3,
	.zq_config = MT41K256M16HA125E_ZQ_CFG,
	.emif_ddr_phy_ctlr_1 = MT41K256M16HA125E_EMIF_READ_LATENCY,
};

#ifdef CONFIG_SPL_OS_BOOT
int spl_start_uboot(void)
{
	/* break into full u-boot on 'c' */
	return (serial_tstc() && serial_getc() == 'c');
}
#endif

#define OSC	(V_OSCK/1000000)
const struct dpll_params dpll_ddr_var_am33x = {
		400, OSC-1, 1, -1, -1, -1, -1};

void am33xx_spl_board_init(void)
{
	int mpu_vdd;
	int sil_rev;

	/* Get the frequency */
	dpll_mpu_opp100.m = am335x_get_efuse_mpu_max_freq(cdev);

	i2c_set_bus_num(1);

	/*
	 * The SOM use a TPS65910 PMIC.  For all
	 * MPU frequencies we support we use a CORE voltage of
	 * 1.1375V.  For MPU voltage we need to switch based on
	 * the frequency we are running at.
	 */
	if (i2c_probe(TPS65910_CTRL_I2C_ADDR))
		return;

	/*
	 * Depending on MPU clock and PG we will need a different
	 * VDD to drive at that speed.
	 */
	sil_rev = readl(&cdev->deviceid) >> 28;
	mpu_vdd = am335x_get_tps65910_mpu_vdd(sil_rev,
			dpll_mpu_opp100.m);

	/* Tell the TPS65910 to use i2c */
	tps65910_set_i2c_control();

	/* First update MPU voltage. */
	if (tps65910_voltage_update(MPU, mpu_vdd))
		return;

	/* Second, update the CORE voltage. */
	if (tps65910_voltage_update(CORE, TPS65910_OP_REG_SEL_1_1_3))
		return;

	/* Set CORE Frequencies to OPP100 */
	do_setup_dpll(&dpll_core_regs, &dpll_core_opp100);

	/* Set MPU Frequency to what we detected now that voltages are set */
	do_setup_dpll(&dpll_mpu_regs, &dpll_mpu_opp100);
}

const struct dpll_params *get_dpll_ddr_params(void)
{
	return &dpll_ddr_var_am33x;
}

void set_uart_mux_conf(void)
{
#ifdef CONFIG_SERIAL1
	enable_uart0_pin_mux();
#endif /* CONFIG_SERIAL1 */
}

void set_mux_conf_regs(void)
{
	enable_board_pin_mux();

	/* Reset the RMII ethernet chip.
	 */
	gpio_request(GPIO_PHY1_RST, "phy1_rst");
	gpio_direction_output(GPIO_PHY1_RST, 1);
	udelay(10000);
	gpio_set_value(GPIO_PHY1_RST, 0);
	udelay(10000);
	gpio_set_value(GPIO_PHY1_RST, 1);

	enable_rmii1_pin_mux();

	/* Reset the RGMII ethernet chip.
	 */
	gpio_request(GPIO_PHY2_RST, "phy2_rst");
	gpio_request(55, "rgmii_phyaddr2");
	gpio_request(56, "rgmii_mode0");
	gpio_request(57, "rgmii_mode1");
	gpio_request(58, "rgmii_mode2");
	gpio_request(59, "rgmii_mode3");
	gpio_request(49, "rgmii_clk125_ena");

	gpio_direction_output(55, 1);
	gpio_direction_output(56, 1);
	gpio_direction_output(57, 1);
	gpio_direction_output(58, 1);
	gpio_direction_output(59, 1);
	gpio_direction_output(49, 1);
	gpio_direction_output(GPIO_PHY2_RST, 1);

	udelay(10000);
	gpio_set_value(GPIO_PHY2_RST, 0);
	udelay(10000);
	gpio_set_value(GPIO_PHY2_RST, 1);
	udelay(10000);

	enable_rgmii2_pin_mux();
}

void sdram_init(void)
{
	config_ddr(400, MT41K256M16HA125E_IOCTRL_VALUE,
			&ddr3_var_am33x_data,
			&ddr3_var_am33x_cmd_ctrl_data,
			&ddr3_var_am33x_emif_reg_data, 0);
}
#endif

/*
 * Basic board specific setup.  Pinmux has been handled already.
 */
int board_init(void)
{
	gd->bd->bi_boot_params = CONFIG_SYS_SDRAM_BASE + 0x100;
#if defined(CONFIG_NOR) || defined(CONFIG_NAND)
	gpmc_init();
#endif
	rtc32k_enable();


	gpio_request(GPIO_SOM_REV_BIT0_GPIO, "som_rev_bit_0");
	gpio_direction_input(GPIO_SOM_REV_BIT0_GPIO);
	gpio_request(GPIO_SOM_REV_BIT1_GPIO, "som_rev_bit_1");
	gpio_direction_input(GPIO_SOM_REV_BIT1_GPIO);
	gpio_request(GPIO_SOM_REV_BIT2_GPIO, "som_rev_bit_2");
	gpio_direction_input(GPIO_SOM_REV_BIT2_GPIO);

	som_rev = (gpio_get_value(GPIO_SOM_REV_BIT0_GPIO) |
			(gpio_get_value(GPIO_SOM_REV_BIT1_GPIO) << 1)) +
		!gpio_get_value(GPIO_SOM_REV_BIT2_GPIO);

	gpio_free(GPIO_SOM_REV_BIT0_GPIO);
	gpio_free(GPIO_SOM_REV_BIT1_GPIO);
	gpio_free(GPIO_SOM_REV_BIT2_GPIO);

#if !defined(CONFIG_SPL_BUILD)
	if (som_rev > 0)
		printf("Variscite AM33 SOM revision 1.%d detected\n",
				som_rev);
	else {
		printf("ERROR: unknown Variscite AM33X SOM revision.\n");
		hang();
	}

	/* Turn off LCD */
	gpio_request(GPIO_LCD_BACKLIGHT, "backlight");
	gpio_direction_output(GPIO_LCD_BACKLIGHT, 0);

	/* mux bluetooth to omap */
	gpio_request(GPIO_BT_UART_SELECT, "bt_uart_select");
	gpio_direction_output(GPIO_BT_UART_SELECT, 1);

#endif

	return 0;
}

#ifdef CONFIG_BOARD_LATE_INIT
int board_late_init(void)
{
	return 0;
}
#endif

#if (defined(CONFIG_DRIVER_TI_CPSW) && !defined(CONFIG_SPL_BUILD)) || \
	(defined(CONFIG_SPL_ETH_SUPPORT) && defined(CONFIG_SPL_BUILD))
static void cpsw_control(int enabled)
{
	return;
}

int board_phy_config(struct phy_device *phydev)
{
	if (phydev->drv->config)
		phydev->drv->config(phydev);

	if (phydev->phy_id == 0x221560 || phydev->phy_id == 0x00221556) {
		/* KS8051/KS8081 fixup */
		/* override strap, set RMII mode */
		printf ("Found Micrel KS8051/KS8081 PHY\n");
		phy_write(phydev, MDIO_DEVAD_NONE, 0x16, 0x2);
	}
	else if (phydev->phy_id == 0x221611) {
		/* KSZ9021 fixup */
		/* Fine-tune RX data pad skew */
		printf ("Found Micrel KSZ9021 PHY\n");
		phy_write(phydev, MDIO_DEVAD_NONE, 0xb, 0x8104);
		phy_write(phydev, MDIO_DEVAD_NONE, 0xc, 0xA097);

		phy_write(phydev, MDIO_DEVAD_NONE, 0xb, 0x8105);
		phy_write(phydev, MDIO_DEVAD_NONE, 0xc, 0);
	}

	return 0;
}

static struct cpsw_slave_data cpsw_slaves[] = {
	{
		.slave_reg_ofs	= 0x208,
		.sliver_reg_ofs	= 0xd80,
		.phy_addr	= 0,
	},
	{
		.slave_reg_ofs	= 0x308,
		.sliver_reg_ofs	= 0xdc0,
		.phy_addr	= 7,
	},
};

static struct cpsw_platform_data cpsw_data = {
	.mdio_base		= CPSW_MDIO_BASE,
	.cpsw_base		= CPSW_BASE,
	.mdio_div		= 0xff,
	.channels		= 8,
	.cpdma_reg_ofs		= 0x800,
	.slaves			= 2,
	.slave_data		= cpsw_slaves,
	.ale_reg_ofs		= 0xd00,
	.ale_entries		= 1024,
	.host_port_reg_ofs	= 0x108,
	.hw_stats_reg_ofs	= 0x900,
	.bd_ram_ofs		= 0x2000,
	.mac_control		= (1 << 5),
	.control		= cpsw_control,
	.host_port_num		= 0,
	.version		= CPSW_CTRL_VERSION_2,
};
#endif

#if defined(CONFIG_DRIVER_TI_CPSW) || \
	(defined(CONFIG_USB_ETHER) && defined(CONFIG_MUSB_GADGET))
int board_eth_init(bd_t *bis)
{
	int rv, n = 0;
	uint8_t mac_addr[6];
	uint32_t mac_hi, mac_lo;

	/* try reading mac address from efuse */
	mac_lo = readl(&cdev->macid0l);
	mac_hi = readl(&cdev->macid0h);
	mac_addr[0] = mac_hi & 0xFF;
	mac_addr[1] = (mac_hi & 0xFF00) >> 8;
	mac_addr[2] = (mac_hi & 0xFF0000) >> 16;
	mac_addr[3] = (mac_hi & 0xFF000000) >> 24;
	mac_addr[4] = mac_lo & 0xFF;
	mac_addr[5] = (mac_lo & 0xFF00) >> 8;

#if (defined(CONFIG_DRIVER_TI_CPSW) && !defined(CONFIG_SPL_BUILD)) || \
	(defined(CONFIG_SPL_ETH_SUPPORT) && defined(CONFIG_SPL_BUILD))
	if (!getenv("ethaddr")) {
		printf("<ethaddr> not set. Validating first E-fuse MAC\n");

		if (is_valid_ether_addr(mac_addr))
			eth_setenv_enetaddr("ethaddr", mac_addr);
	}

#ifdef CONFIG_DRIVER_TI_CPSW

	mac_lo = readl(&cdev->macid1l);
	mac_hi = readl(&cdev->macid1h);
	mac_addr[0] = mac_hi & 0xFF;
	mac_addr[1] = (mac_hi & 0xFF00) >> 8;
	mac_addr[2] = (mac_hi & 0xFF0000) >> 16;
	mac_addr[3] = (mac_hi & 0xFF000000) >> 24;
	mac_addr[4] = mac_lo & 0xFF;
	mac_addr[5] = (mac_lo & 0xFF00) >> 8;

	if (!getenv("eth1addr")) {
		if (is_valid_ether_addr(mac_addr))
			eth_setenv_enetaddr("eth1addr", mac_addr);
	}

	writel(GMII1_SEL_RMII | GMII2_SEL_RGMII | RMII_CHIPCKL_ENABLE, &cdev->miisel);

	cpsw_slaves[0].phy_if = PHY_INTERFACE_MODE_RMII;
	cpsw_slaves[1].phy_if = PHY_INTERFACE_MODE_RGMII;

	rv = cpsw_register(&cpsw_data);
	if (rv < 0)
		printf("Error %d registering CPSW switch\n", rv);
	else
		n += rv;
#endif

#endif
	return n;
}
#endif
