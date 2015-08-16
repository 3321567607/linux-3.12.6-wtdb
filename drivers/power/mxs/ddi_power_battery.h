/*
 * Copyright (C) 2012 Freescale Semiconductor, Inc. All Rights Reserved.
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

#ifndef __DDI_POWER_BATTERY_H
#define __DDI_POWER_BATTERY_H


uint16_t mxspwr_ma2bits(uint16_t u16ma);
uint16_t mxspwr_bits2ma(uint16_t u16bits);
int      mxspwr_init_bat(void);
uint16_t mxspwr_set_charging_cur(uint16_t u16ma);
uint16_t mxspwr_get_charging_cur(void);
uint16_t mxspwr_set_charg_stop_thsh(uint16_t u16Thresh);
uint16_t mxspwr_expressible_cur(uint16_t u16Current);
uint16_t mxspwr_rampup_chrgr_4p2_limit(uint16_t u16ma);
uint16_t mxspwr_enable4p2(uint16_t u16ma);
void     mxspwr_set4p2bo_thsh(uint16_t bo_mv);
void     mxspwr_adjust_cmptrip(void);
uint16_t mxspwr_get_chrgr_4p2_limit(void);
uint16_t mxspwr_set_chrgr_4p2_limit(uint16_t ilimit);
void     mxspwr_shutdown(char *);

#endif
