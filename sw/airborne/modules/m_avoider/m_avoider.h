/*
 * Copyright (C) Wuyang
 *
 * This file is part of paparazzi
 *
 */
/**
 * @file "modules/m_avoider/m_avoider.h"
 * @author Wuyang
 * Avoid several colors.
 */

#ifndef M_AVOIDER_H
#define M_AVOIDER_H
#include <inttypes.h>
#include "state.h"

extern uint8_t safeToGoForwards;
extern float incrementForAvoidance;
extern uint16_t trajectoryConfidence;
extern void m_avoider_init(void);
extern void m_avoider_periodic(void);
extern uint8_t moveWaypointForward(uint8_t, float);
extern uint8_t moveWaypoint(uint8_t, struct EnuCoor_i *);
extern uint8_t increase_nav_heading(int32_t *, float);
extern uint8_t chooseRandomIncrementAvoidance(uint8_t, uint8_t, uint8_t);

#endif

