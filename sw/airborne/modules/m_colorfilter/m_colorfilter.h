/*
 * Copyright (C) Wuyang
 *
 * This file is part of paparazzi
 *
 */
/**
 * @file "modules/m_colorfilter/m_colorfilter.h"
 * @author Wuyang
 * Filter multiple color and count the obstacle pixels in each sides.
 */

#ifndef COLORFILTER_CV_PLUGIN_H
#define COLORFILTER_CV_PLUGIN_H

#include <stdint.h>
#include "modules/computer_vision/cv.h"

// Module functions
extern void colorfilter_init(void);

extern struct thres thres_o;
extern struct thres thres_r;
extern struct thres thres_b;

extern struct count cnt;

extern struct video_listener *listener;

#endif /* COLORFILTER_CV_PLUGIN_H */
