/*
 * Copyright (C) Wuyang
 *
 * This file is part of paparazzi
 *
 */
/**
 * @file "modules/m_colorfilter/m_colorfilter.c"
 * @author Wuyang
 * Filter multiple color and count the obstacle pixels in each sides.
 */

#include "modules/m_colorfilter/m_colorfilter.h"
#include <stdio.h>

#include "modules/computer_vision/lib/vision/m_image.h"

struct video_listener *listener = NULL;

// Filter Setting.
struct thres thres_o = {105, 205, 52, 140, 180, 255};
struct thres thres_r = {4, 91, 0, 124, 127, 255};
struct thres thres_b = {0, 18, 120, 130, 120, 130};
struct count cnt;

// Function
struct image_t *colorfilter_func(struct image_t *img);
struct image_t *colorfilter_func(struct image_t *img)
{
  // Filter
  image_yuv422_colorfilt(img, img, thres_o, thres_r, thres_b, &cnt);
  return img; // Colorfilter did not make a new image. Check if the img is changed or not?
}

void colorfilter_init(void)
{
  listener = cv_add_to_device(&COLORFILTER_CAMERA, colorfilter_func);
}
