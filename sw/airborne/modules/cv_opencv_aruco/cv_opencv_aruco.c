/*
 * cv_opencv_aruco.c
 * Use aruco library on bebop.
 *
 *  Created on: Jun 29, 2017
 *      Author: wuyang
 */
//#include <time.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/time.h>
#include "mcu_periph/sys_time.h"
#include "modules/computer_vision/cv.h"
#include "modules/cv_opencv_aruco/cv_opencv_aruco.h"
#include "modules/cv_opencv_aruco/opencv_aruco.h"

#define ARUCO_VERBOSE TRUE

#define PRINT(string,...) fprintf(stderr, "[Aruco detection] " string, ##__VA_ARGS__)
#if ARUCO_VERBOSE
#define VERBOSE_PRINT PRINT
#else
#define VERBOSE_PRINT(...)
#endif

struct image_t* opencv_aruco_func(struct image_t* img);
struct image_t* opencv_aruco_func(struct image_t* img){
	if (img->type == IMAGE_YUV422){
		//clock_t begin = clock();
		uint32_t begin = get_sys_time_msec();
		aruco_func((char*) img->buf, img->w, img->h);
		uint32_t end = get_sys_time_msec();
		//clock_t end = clock();
		//float elapsed = (float)(end - begin)*1000.0/CLOCKS_PER_SEC;
		uint32_t elapsed = end - begin;
		VERBOSE_PRINT("%d ms\n", elapsed);
	}

	return NULL;
}

void opencv_aruco_init(void){
	cv_add_to_device(&ARUCO_CAMERA, opencv_aruco_func);
}
