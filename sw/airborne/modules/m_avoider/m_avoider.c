/*
 * Copyright (C) Wuyang
 *
 * This file is part of paparazzi
 *
 */
/**
 * @file "modules/m_avoider/m_avoider.c"
 * @author Wuyang
 * Avoid several colors.
 */

#include <time.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include "firmwares/rotorcraft/navigation.h"

#include "generated/flight_plan.h"
#include "modules/m_colorfilter/m_colorfilter.h"//NOTE TO CHANGE
#include "modules/m_avoider/m_avoider.h"

#define M_AVOIDER_VERBOSE TRUE

#define PRINT(string,...) fprintf(stderr, "[m_avoider->%s()] " string,__FUNCTION__ , ##__VA_ARGS__)
#if M_AVOIDER_VERBOSE
#define VERBOSE_PRINT PRINT
#else
#define VERBOSE_PRINT(...)
#endif

uint8_t safeToGoForwards        = false;
uint8_t l =false;
uint8_t m =false;
uint8_t r =false;
int tresholdColorCount          = 0.03 * 124800; // 520 x 240 = 124.800 total pixels. Originally 0.05 x total.
float incrementForAvoidance;
uint16_t trajectoryConfidence   = 1;
float maxDistance               = 2.25;

/*Initialisation function, setting the colour filter, random seed and incrementForAvoidance*/
void m_avoider_init()
{
  // Initialise the variables of the colorfilter to accept orange, red, etc.
  thres_o.y_m = 20;
  thres_o.y_M = 255;
  thres_o.u_m = 75;
  thres_o.u_M = 145;
  thres_o.v_m = 167;
  thres_o.v_M = 255;
  thres_r.y_m = 0;  //26
  thres_r.y_M = 30;  //42
  thres_r.u_m = 110; //122
  thres_r.u_M = 135; //134
  thres_r.v_m = 140; //150
  thres_r.v_M = 200; //202
  thres_b.y_m = 0;
  thres_b.y_M = 18;
  thres_b.u_m = 120;
  thres_b.u_M = 130;
  thres_b.v_m = 120;
  thres_b.v_M = 130;
  // Initialise random values
  srand(time(NULL));
  chooseRandomIncrementAvoidance(l, m, r);
}

/*
 * Function that checks it is safe to move forwards, and then moves a waypoint forward or changes the heading
 */
void m_avoider_periodic()
{
  //check the amount of obstacle colors. O.O5 can be changed to specific waypoint moving rate.
  l = cnt.cnt_l  > tresholdColorCount;
  m = cnt.cnt_m > tresholdColorCount;
  r = cnt.cnt_r > tresholdColorCount;
  safeToGoForwards = (!(l || m || r)) || (l && (!m) && r);
  VERBOSE_PRINT("Color_left: %d Color_middle: %d Color_right: %d \n", cnt.cnt_l, cnt.cnt_m, cnt.cnt_r);
  VERBOSE_PRINT("Unsafe left: %d Unsafe middle: %d Unsafe right: %d Safe: %d \n",l,m,r,safeToGoForwards);
  chooseRandomIncrementAvoidance(l, m, r);
  float moveDistance = fmin(maxDistance, 0.05 * trajectoryConfidence);
  if(safeToGoForwards) {
	moveWaypointForward(WP_GOAL, moveDistance);
	moveWaypointForward(WP_TRAJECTORY, 1.25 * moveDistance);
	nav_set_heading_towards_waypoint(WP_GOAL);
	trajectoryConfidence += 1;
  }
  else if ((!safeToGoForwards) && (InObstacleZone(WaypointX(WP_TRAJECTORY),WaypointY(WP_TRAJECTORY)))) {
	waypoint_set_here_2d(WP_GOAL);
	waypoint_set_here_2d(WP_TRAJECTORY);
	increase_nav_heading(&nav_heading, incrementForAvoidance);
	if(trajectoryConfidence > 5){
	  trajectoryConfidence -= 4;
	}
	else{
	  trajectoryConfidence = 1;
	}
  }
  else {
  }
  return;
}

/*
 * Increases the NAV heading. Assumes heading is an INT32_ANGLE. It is bound in this function.
 */
uint8_t increase_nav_heading(int32_t *heading, float incrementDegrees)
{
  struct Int32Eulers *eulerAngles   = stateGetNedToBodyEulers_i();
  int32_t newHeading = eulerAngles->psi + ANGLE_BFP_OF_REAL( incrementDegrees / 180.0 * M_PI);
  // Check if your turn made it go out of bounds...
  INT32_ANGLE_NORMALIZE(newHeading); // HEADING HAS INT32_ANGLE_FRAC....
  *heading = newHeading;
  //VERBOSE_PRINT("Increasing heading to %f\n", ANGLE_FLOAT_OF_BFP(*heading) * 180 / M_PI);
  return false;
}

/*
 * Calculates coordinates of a distance of 'distanceMeters' forward w.r.t. current position and heading
 */
uint8_t calculateForwards(struct EnuCoor_i *new_coor, float distanceMeters)
{
  struct EnuCoor_i *pos             = stateGetPositionEnu_i(); // Get your current position
  struct Int32Eulers *eulerAngles   = stateGetNedToBodyEulers_i();
  // Calculate the sine and cosine of the heading the drone is keeping
  float sin_heading                 = sinf(ANGLE_FLOAT_OF_BFP(eulerAngles->psi));
  float cos_heading                 = cosf(ANGLE_FLOAT_OF_BFP(eulerAngles->psi));
  // Now determine where to place the waypoint you want to go to
  new_coor->x                       = pos->x + POS_BFP_OF_REAL(sin_heading * (distanceMeters));
  new_coor->y                       = pos->y + POS_BFP_OF_REAL(cos_heading * (distanceMeters));
  //VERBOSE_PRINT("Calculated %f m forward position. x: %f  y: %f based on pos(%f, %f) and heading(%f)\n", distanceMeters, POS_FLOAT_OF_BFP(new_coor->x), POS_FLOAT_OF_BFP(new_coor->y), POS_FLOAT_OF_BFP(pos->x), POS_FLOAT_OF_BFP(pos->y), ANGLE_FLOAT_OF_BFP(eulerAngles->psi)*180/M_PI);
  return false;
}

/*
 * Sets waypoint 'waypoint' to the coordinates of 'new_coor'
 */
uint8_t moveWaypoint(uint8_t waypoint, struct EnuCoor_i *new_coor)
{
  //VERBOSE_PRINT("Moving waypoint %d to x:%f y:%f\n", waypoint, POS_FLOAT_OF_BFP(new_coor->x), POS_FLOAT_OF_BFP(new_coor->y));
  waypoint_set_xy_i(waypoint, new_coor->x, new_coor->y);
  return false;
}

/*
 * Calculates coordinates of distance forward and sets waypoint 'waypoint' to those coordinates
 */
uint8_t moveWaypointForward(uint8_t waypoint, float distanceMeters)
{
  struct EnuCoor_i new_coor;
  calculateForwards(&new_coor, distanceMeters);
  moveWaypoint(waypoint, &new_coor);
  return false;
}

/*
 * Sets the variable 'incrementForAvoidance' randomly positive/negative
 */
uint8_t chooseRandomIncrementAvoidance(uint8_t left,uint8_t middle, uint8_t right)
{
  if (left) {
	if (middle) {
		incrementForAvoidance = 15.0; //110 & 111
		VERBOSE_PRINT("Set avoidance increment to: %f\n\n", incrementForAvoidance);
	}else {
	  if (right) {
	    incrementForAvoidance = 1.0; // 101
	    VERBOSE_PRINT("Set avoidance increment to: %f\n\n", incrementForAvoidance);
	  }else {
	    incrementForAvoidance = 10.0; // 100
	    VERBOSE_PRINT("Set avoidance increment to: %f\n\n", incrementForAvoidance);
	  }
	}
  }else {
    if (middle) {
      if (right){
        incrementForAvoidance = -15.0; //011
    	VERBOSE_PRINT("Set avoidance increment to: %f\n\n", incrementForAvoidance);
      }else {
        incrementForAvoidance = 10.0; //010
    	VERBOSE_PRINT("Set avoidance increment to: %f\n\n", incrementForAvoidance);
      }
    }else {
      if (right) {
        incrementForAvoidance = -10.0; //001
        VERBOSE_PRINT("Set avoidance increment to: %f\n\n", incrementForAvoidance);
      }else {
        incrementForAvoidance = 0.0; //000
    	VERBOSE_PRINT("Set avoidance increment to: %f\n\n", incrementForAvoidance);
      }
    }
  }
  return false;
}

