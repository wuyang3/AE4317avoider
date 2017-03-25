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

static inline bool InObstacleZone(float _x, float _y) {
  if (_y <= 1.5) {
    if (_y <= 0.3) {
      if (_y <= -3.1) {
        return FALSE;
      } else {
        float dy = _y - 0.3;
        return (-1.6+dy*-0.658009<= _x && _x <= 5.8+dy*1.500000);
      }
    } else {
      float dy = _y - 1.5;
      return (-2.4+dy*-0.658009<= _x && _x <= 5.0+dy*-0.631783);
    }
  } else {
    if (_y <= 5.5) {
      float dy = _y - 5.5;
      return (2.5+dy*1.241206<= _x && _x <= 2.5+dy*-0.631783);
    } else {
      return FALSE;
    }
  }
}

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

