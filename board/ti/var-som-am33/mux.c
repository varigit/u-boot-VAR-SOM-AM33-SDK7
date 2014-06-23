/*
 * mux.c
 *
 * Copyright (C) 2011 Texas Instruments Incorporated - http://www.ti.com/
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation version 2.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

#include <common.h>
#include <asm/arch/sys_proto.h>
#include <asm/arch/hardware.h>
#include <asm/arch/mux.h>
#include <asm/io.h>
#include <i2c.h>
#include "board.h"

static struct module_pin_mux rev_pin_mux[] = {
	{OFFSET(lcd_data7), (MODE(7) | PULLUDEN | RXACTIVE)},
	{OFFSET(lcd_vsync), (MODE(7) | PULLUDEN | RXACTIVE)},
	{OFFSET(lcd_data5), (MODE(7) | PULLUDEN | RXACTIVE)},
	{-1},
};

static struct module_pin_mux uart0_pin_mux[] = {
	{OFFSET(uart0_rxd), (MODE(0) | PULLUP_EN | RXACTIVE)},  /* UART0_RXD */
	{OFFSET(uart0_txd), (MODE(0) | PULLUDEN)},              /* UART0_TXD */
	{-1},
};

static struct module_pin_mux uart3_pin_mux[] = {
	{OFFSET(spi0_cs1), (MODE(1) | PULLUDEN | RXACTIVE)},    /* UART3_RXD */
	{OFFSET(ecap0_in_pwm0_out), (MODE(1) | PULLUDEN)},      /* UART3_TXD */
	{-1},
};

static struct module_pin_mux gpio0_2_pin_mux[] = {
	{OFFSET(spi0_sclk), MODE(7) | PULLUDEN},       /* GPIO0_2, LCD Backlight */
	{-1},
};

static struct module_pin_mux nand_pin_mux[] = {
	{OFFSET(gpmc_ad0), (MODE(0) | PULLUP_EN | RXACTIVE)},   /* NAND AD0 */
	{OFFSET(gpmc_ad1), (MODE(0) | PULLUP_EN | RXACTIVE)},   /* NAND AD1 */
	{OFFSET(gpmc_ad2), (MODE(0) | PULLUP_EN | RXACTIVE)},   /* NAND AD2 */
	{OFFSET(gpmc_ad3), (MODE(0) | PULLUP_EN | RXACTIVE)},   /* NAND AD3 */
	{OFFSET(gpmc_ad4), (MODE(0) | PULLUP_EN | RXACTIVE)},   /* NAND AD4 */
	{OFFSET(gpmc_ad5), (MODE(0) | PULLUP_EN | RXACTIVE)},   /* NAND AD5 */
	{OFFSET(gpmc_ad6), (MODE(0) | PULLUP_EN | RXACTIVE)},   /* NAND AD6 */
	{OFFSET(gpmc_ad7), (MODE(0) | PULLUP_EN | RXACTIVE)},   /* NAND AD7 */
	{OFFSET(gpmc_wait0), (MODE(0) | RXACTIVE | PULLUP_EN)}, /* NAND WAIT */
	{OFFSET(gpmc_wpn), (MODE(7) | PULLUP_EN | RXACTIVE)},   /* NAND_WPN */
	{OFFSET(gpmc_csn0), (MODE(0) | PULLUDEN)},  /* NAND_CS0 */
	{OFFSET(gpmc_advn_ale), (MODE(0) | PULLUDEN)}, /* NAND_ADV_ALE */
	{OFFSET(gpmc_oen_ren), (MODE(0) | PULLUDEN)},   /* NAND_OE */
	{OFFSET(gpmc_wen), (MODE(0) | PULLUDEN)},   /* NAND_WEN */
	{OFFSET(gpmc_be0n_cle), (MODE(0) | PULLUDEN)},  /* NAND_BE_CLE */
	{-1},
};

static struct module_pin_mux i2c1_pin_mux[] = {
	{OFFSET(spi0_d1), (MODE(2) | RXACTIVE | PULLUDEN | SLEWCTRL)},  /* I2C_DATA */
	{OFFSET(spi0_cs0), (MODE(2) | RXACTIVE | PULLUDEN | SLEWCTRL)}, /* I2C_SCLK */
	{-1},
};

static struct module_pin_mux gpio2_19_pin_mux[] = {
	{OFFSET(mii1_rxd2), MODE(7) | PULLUDEN},    /* GPIO2_19, PHY resrt */
	{-1},
};

static struct module_pin_mux clkout_pin_mux[] = {
	{OFFSET(xdma_event_intr0), MODE(3)},    /* clkout1 */
	{-1},
};

static struct module_pin_mux bt_uart_select_pin_mux[] = {
	{OFFSET(xdma_event_intr1), MODE(7)},    /* GPIO0_20 */
	{-1},
};

static struct module_pin_mux rmii1_pin_mux[] = {
	{OFFSET(mii1_crs), MODE(1) | RXACTIVE},     /* RMII1_CRS */
	{OFFSET(mii1_rxerr), MODE(1) | RXACTIVE},   /* RMII1_RXERR */
	{OFFSET(mii1_txen), MODE(1)},               /* RMII1_TXEN */
	{OFFSET(mii1_txd1), MODE(1)},               /* RMII1_TXD1 */
	{OFFSET(mii1_txd0), MODE(1)},               /* RMII1_TXD0 */
	{OFFSET(mii1_rxd1), MODE(1) | RXACTIVE},    /* RMII1_RXD1 */
	{OFFSET(mii1_rxd0), MODE(1) | RXACTIVE},    /* RMII1_RXD0 */
	{OFFSET(mdio_data), MODE(0) | RXACTIVE | PULLUP_EN}, /* MDIO_DATA */
	{OFFSET(mdio_clk), MODE(0) | PULLUP_EN},    /* MDIO_CLK */
	{OFFSET(rmii1_refclk), MODE(0) | RXACTIVE}, /* RMII1_REFCLK */
	{-1},
};

static struct module_pin_mux rgmii2_pin_mux[] = {
	{OFFSET(gpmc_a0), MODE(2)},
	{OFFSET(gpmc_a1), MODE(2) | PULLUDEN | RXACTIVE},
	{OFFSET(gpmc_a2), MODE(2) },
	{OFFSET(gpmc_a3), MODE(2) },
	{OFFSET(gpmc_a4), MODE(2) },
	{OFFSET(gpmc_a5), MODE(2) },
	{OFFSET(gpmc_a6), MODE(2) },
	{OFFSET(gpmc_a7), MODE(2) | PULLUDEN | RXACTIVE},
	{OFFSET(gpmc_a8), MODE(2) | PULLUDEN | RXACTIVE},
	{OFFSET(gpmc_a9), MODE(2) | PULLUDEN | RXACTIVE},
	{OFFSET(gpmc_a10), MODE(2) | PULLUDEN | RXACTIVE},
	{OFFSET(gpmc_a11), MODE(2) | PULLUDEN | RXACTIVE},
	{OFFSET(mii1_col), MODE(1) | PULLUDEN | RXACTIVE},
	{OFFSET(mdio_data), MODE(0) | RXACTIVE | PULLUP_EN},
	{OFFSET(mdio_clk), MODE(0) | PULLUP_EN},
	{-1},
};

static struct module_pin_mux rgmii2_strapping_pin_mux[] = {
	{OFFSET(gpmc_a7), MODE(7) | PULLUP_EN},
	{OFFSET(gpmc_a8), MODE(7) | PULLUP_EN},
	{OFFSET(gpmc_a9), MODE(7) | PULLUP_EN},
	{OFFSET(gpmc_a10), MODE(7) | PULLUP_EN},
	{OFFSET(gpmc_a11), MODE(7) | PULLUP_EN},
	{OFFSET(gpmc_a1), MODE(7) | PULLUP_EN},
	{OFFSET(mii1_rxclk), MODE(7)| PULLUP_EN},
	{-1},
};

static struct module_pin_mux mmc0_pin_mux[] = {
	{OFFSET(mmc0_dat3), (MODE(0) | RXACTIVE | PULLUP_EN)},  /* MMC0_DAT3 */
	{OFFSET(mmc0_dat2), (MODE(0) | RXACTIVE | PULLUP_EN)},  /* MMC0_DAT2 */
	{OFFSET(mmc0_dat1), (MODE(0) | RXACTIVE | PULLUP_EN)},  /* MMC0_DAT1 */
	{OFFSET(mmc0_dat0), (MODE(0) | RXACTIVE | PULLUP_EN)},  /* MMC0_DAT0 */
	{OFFSET(mmc0_clk), (MODE(0) | RXACTIVE | PULLUP_EN)},   /* MMC0_CLK */
	{OFFSET(mmc0_cmd), (MODE(0) | RXACTIVE | PULLUP_EN)},   /* MMC0_CMD */
	{OFFSET(gpmc_be1n), (MODE(5) | RXACTIVE | PULLUP_EN)},  /* MMC0_CD */
	{-1},
};

#if defined(CONFIG_NOR_BOOT)
void enable_norboot_pin_mux(void)
{
	/*
	 * The ROM will only have set up sufficient pinmux to allow for the
	 * first 4KiB NOR to be read, we must finish doing what we know of
	 * the NOR mux in this space in order to continue.  We do this in
	 * assembly to avoid having to play linker games to ensure that all
	 * functions and data sections are in this special area.
	 */
	asm("stmfd      sp!, {r2 - r4}");
	asm("movw       r4, #0x8A4");
	asm("movw       r3, #0x44E1");
	asm("orr        r4, r4, r3, lsl #16");
	asm("mov        r2, #9");
	asm("mov        r3, #8");
	asm("gpmc_mux:  str     r2, [r4], #4");
	asm("subs       r3, r3, #1");
	asm("bne        gpmc_mux");
	asm("ldmfd      sp!, {r2 - r4}");
}
#endif

#define PROFILE_NONE	0x0
#define PROFILE_0	(1 << 0)
#define PROFILE_1	(1 << 1)
#define PROFILE_2	(1 << 2)
#define PROFILE_3	(1 << 3)
#define PROFILE_4	(1 << 4)
#define PROFILE_5	(1 << 5)
#define PROFILE_6	(1 << 6)
#define PROFILE_7	(1 << 7)
#define PROFILE_MASK	0x7
#define PROFILE_ALL	0xFF

void enable_uart0_pin_mux(void)
{
	configure_module_pin_mux(uart0_pin_mux);
}

void enable_uart3_pin_mux(void)
{
	configure_module_pin_mux(uart3_pin_mux);
}

void enable_rmii1_pin_mux(void)
{
	configure_module_pin_mux(rmii1_pin_mux);
}

void enable_rgmii2_pin_mux(void)
{
	configure_module_pin_mux(rgmii2_pin_mux);
}

void enable_board_pin_mux(void)
{
	configure_module_pin_mux(rev_pin_mux);
	configure_module_pin_mux(mmc0_pin_mux);
	configure_module_pin_mux(nand_pin_mux);
	configure_module_pin_mux(i2c1_pin_mux);
	configure_module_pin_mux(gpio0_2_pin_mux);
	configure_module_pin_mux(gpio2_19_pin_mux);
	configure_module_pin_mux(clkout_pin_mux);
	configure_module_pin_mux(bt_uart_select_pin_mux);
	configure_module_pin_mux(rgmii2_strapping_pin_mux);
}

