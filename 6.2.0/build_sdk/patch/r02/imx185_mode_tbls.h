/* SPDX-License-Identifier: GPL-2.0 */
/*
 * imx185_mode_tbls.h - imx274 sensor driver
 *
 * Copyright (c) 2016-2023 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
 */

#ifndef __IMX185_I2C_TABLES__
#define __IMX185_I2C_TABLES__

#include <media/camera_common.h>
#include <linux/miscdevice.h>

#define IMX185_TABLE_WAIT_MS	0
#define IMX185_TABLE_END	1
#define IMX185_MAX_RETRIES	3
#define IMX185_WAIT_MS_STOP	1
#define IMX185_WAIT_MS_START	30
#define IMX185_WAIT_MS_STREAM	210
#define IMX185_GAIN_TABLE_SIZE 255

/* #define INIT_ET_INSETTING 1 */

#define imx185_reg struct reg_8

static imx185_reg imx185_start[] = {
	{IMX185_TABLE_WAIT_MS, IMX185_WAIT_MS_STREAM},
	{ IMX185_TABLE_END, 0x00 }
};

static imx185_reg imx185_stop[] = {
	{IMX185_TABLE_WAIT_MS, IMX185_WAIT_MS_STOP},
	{IMX185_TABLE_END, 0x00 }
};

static  imx185_reg imx185_1920x1080_30fps[] = {
 	{IMX185_TABLE_END, 0x00}
};

static  imx185_reg imx185_1280x720_30fps[] = {
	{IMX185_TABLE_END, 0x00}
};

enum {
	IMX185_MODE_1920X1080_30FPS,
	IMX185_MODE_1280X720_30FPS,
	IMX185_MODE_START_STREAM,
	IMX185_MODE_STOP_STREAM
};

static imx185_reg *mode_table[] = {
	[IMX185_MODE_1920X1080_30FPS] = imx185_1920x1080_30fps,
	[IMX185_MODE_1280X720_30FPS] = imx185_1280x720_30fps,
	[IMX185_MODE_START_STREAM] = imx185_start,
	[IMX185_MODE_STOP_STREAM] = imx185_stop,
};

static const int imx185_30fps[] = {
  30, 15,
};

/*
 * WARNING: frmfmt ordering need to match mode definition in
 * device tree!
 */
static const struct camera_common_frmfmt imx185_frmfmt[] = {
	{{1920, 1080}, imx185_30fps, 2, 0, IMX185_MODE_1920X1080_30FPS},
	{{1280, 720}, imx185_30fps, 2, 0, IMX185_MODE_1280X720_30FPS},
	/* Add modes with no device tree support after below */
};
#endif /* __IMX185_I2C_TABLES__ */
