/*
 * Copyright (C) Wuyang
 *
 * This file is part of paparazzi
 *
 */
/**
 * @file "modules/test_module/test_module.c"
 * @author Wuyang
 * 
 */

#include "firmwares/rotorcraft/navigation.h"
#include "generated/flight_plan.h"
#include "modules/test_module/test_module.h"

#include <stdbool.h>
bool LongTimeInBlock(int timeInBlock){
  return timeInBlock > 5;
}
