/*
 * board.h
 *
 * Variscite AM335x SOMs
 *
 * Copyright (C) 2014, Variscite LTD - http://www.variscite.com/
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#ifndef _BOARD_H_
#define _BOARD_H_

/*
 * We have three pin mux functions that must exist.  We must be able to enable
 * uart0, for initial output and i2c1 to setup PMIC.  We then have a
 * main pinmux function that can be overridden to enable all other pinmux that
 * is required on the board.
 */
void enable_uart0_pin_mux(void);
void enable_rmii1_pin_mux(void);
void enable_rgmii2_pin_mux(void);
void enable_board_pin_mux(void);
#endif
