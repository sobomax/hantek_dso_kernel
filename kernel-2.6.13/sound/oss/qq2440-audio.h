/*
 * (C) Samsung Electronics 2004
 *
 * Philips UDA1341 Audio Device Driver for SMDK board
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * 2004-07: SW.LEE
 *         comment : Originally made by MIZI Research
 */
//#include <linux/pm.h>

void start_elfin_iis_bus_tx(void);
//int uda1341_pm_callback(struct pm_dev *pmdev, pm_request_t req, void *data);

#ifndef SAMSUNG_IIS_CODEC_CLOCK
#define SAMSUNG_IIS_CODEC_CLOCK
#define S_CLOCK_FREQ	384
#endif


