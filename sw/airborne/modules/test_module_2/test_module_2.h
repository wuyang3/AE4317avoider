/*
 * Copyright (C) Wuyang
 *
 * This file is part of paparazzi
 *
 */
/**
 * @file "modules/test_module_2/test_module_2.h"
 * @author Wuyang
 * 
 */

#ifndef TEST_MODULE_2_H
#define TEST_MODULE_2_H
#include <inttypes.h>
#include "state.h"

extern uint16_t trajectoryConfidence;
extern void test_periodic(void);
extern uint8_t moveWaypointForward(uint8_t, float);
extern uint8_t moveWaypoint(uint8_t, struct EnuCoor_i *);
extern uint8_t increase_nav_heading(int32_t *, float);
extern bool LongTimeInBlock(int);

#endif

