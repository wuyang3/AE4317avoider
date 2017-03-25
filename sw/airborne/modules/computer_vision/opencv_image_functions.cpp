/*
 * Copyright (C) 2016 Roland Meertens
 *
 * This file is part of Paparazzi.
 *
 * Paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * Paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 *
 */

/**
 * @file modules/computer_vision/opencv_image_functions.cpp
 *
 * A small library with functions to convert between the Paparazzi used YUV422 arrays
 * and the opencv image functions.
 */

#include "opencv_image_functions.h"

using namespace std;
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/video/video.hpp>
using namespace cv;

void coloryuv_opencv_to_yuv422(Mat image, char *img)
{
  CV_Assert(image.depth() == CV_8U);
  CV_Assert(image.channels() == 3);

  int nRows = image.rows;
  int nCols = image.cols;

  // If the image is one block in memory we can iterate over it all at once!
  if (image.isContinuous()) {
    nCols *= nRows;
    nRows = 1;
  }

  // Iterate over the image, setting only the Y value
  // and setting U and V to 127
  int i, j;
  uchar *p;
  int index_img = 0;
  for (i = 0; i < nRows; ++i) {
    p = image.ptr<uchar>(i);
    for (j = 0; j < nCols; j += 6) {
      img[index_img++] = p[j + 1]; //U
      img[index_img++] = p[j];//Y
      img[index_img++] = p[j + 2]; //V
      img[index_img++] = p[j + 3]; //Y


    }
  }
}

void colorrgb_opencv_to_yuv422(Mat image, char *img)
{
  // Convert to YUV color space
  cvtColor(image, image, COLOR_BGR2YUV);
  // then call the to color function
  coloryuv_opencv_to_yuv422(image, img);
}


void grayscale_opencv_to_yuv422(Mat image, char *img)
{
  CV_Assert(image.depth() == CV_8U);
  CV_Assert(image.channels() == 1);

  int n_rows = image.rows;
  int n_cols = image.cols;

  // If the image is one block in memory we can iterate over it all at once!
  if (image.isContinuous()) {
    n_cols *= n_rows;
    n_rows = 1;
  }

  // Iterate over the image, setting only the Y value
  // and setting U and V to 127
  int i, j;
  uchar *p;
  int index_img = 0;
  for (i = 0; i < n_rows; ++i) {
    p = image.ptr<uchar>(i);
    for (j = 0; j < n_cols; j++) {
      img[index_img++] = 127;
      img[index_img++] = p[j];


    }
  }
}

void flow_divigence_cal_lmr(IplImage *flowx, IplImage *flowy, float *flag)
{//input: optical flow x,flow y, return flag(division)
	//Descripction:
	//float flag[3]={0,0,0};
	float ptrx;
	float ptry;
	float x2_y2 = 0;
	float x2_y2_focuspoint = 0;
	float xydivision = 0;
	float division_sum_l = 0;
	float division_num_l = 0;
	float division_sum_m = 0;
	float division_num_m = 0;
	float division_sum_r = 0;
	float division_num_r = 0;

	for (int x = 0; x<flowx->height; x++)
	{
		for (int y = 0; y<flowx->width; y++)
		{
			ptrx = ((float *)(flowx->imageData + flowx->widthStep*x))[y];
			ptry = ((float *)(flowy->imageData + flowy->widthStep*x))[y];
			x2_y2 = ptrx*ptrx + ptry*ptry;
			x2_y2_focuspoint = (x - flowx->height / 2)*(x - flowx->height / 2) + (y - flowx->width / 2)*(y - flowx->width / 2);
			if (x2_y2_focuspoint != 0)
			{
				xydivision = x2_y2 * 100 / x2_y2_focuspoint;
				if (xydivision>0.0001)
				{
					if (y >= (flowx->width / 3 * 2))
					{
						division_sum_r += xydivision;
						division_num_r++;
					}
					if (y>(flowx->width / 3) && y<(flowx->width / 3 * 2))
					{
						division_sum_m += xydivision;
						division_num_m++;
					}
					if (y <= (flowx->width / 3))
					{
						division_sum_l += xydivision;
						division_num_l++;
					}
				}
			}
		}
	}
	if (division_num_l != 0)
	{
		flag[0] = division_sum_l / division_num_l;
	}
	else
	{
		flag[0] = 0;
	}
	if (division_num_m != 0)
	{
		flag[1] = division_sum_m / division_num_m;
	}
	else
	{
		flag[1] = 0;
	}
	if (division_num_r != 0)
	{
		flag[2] = division_sum_r / division_num_r;
	}
	else
	{
		flag[2] = 0;
	}
	//return flag;
}

void optical_flow_avoid(IplImage *frame_curr, IplImage *frame_pre,float *flag)
{
	/*
	INPUT
	frame_curr:current picture  type:IplImage
	frame_pre :previous picture type:IplImage
	threshold :threshold for optical flow type:int
	OUTPUT
	forward_to_go: 1-safe 0-dangerous
	*/
	//pre dealing with the Image
	IplImage *image1_8bit = cvCreateImage(cvGetSize(frame_pre), IPL_DEPTH_8U, 1);
	IplImage *image2_8bit = cvCreateImage(cvGetSize(frame_curr), IPL_DEPTH_8U, 1);
	cvCvtColor(frame_pre, image1_8bit, CV_RGB2GRAY);
	cvCvtColor(frame_curr, image2_8bit, CV_RGB2GRAY);
	//optical flow caculation
	IplImage* flowx = cvCreateImage(cvSize(image1_8bit->width, image1_8bit->height), IPL_DEPTH_32F, 1);
	IplImage* flowy = cvCreateImage(cvSize(image1_8bit->width, image1_8bit->height), IPL_DEPTH_32F, 1);
	//optical flow calculation
	cvCalcOpticalFlowLK(image1_8bit, image2_8bit, cvSize(5, 5), flowx, flowy);
	//LMR flag calculation
	flow_divigence_cal_lmr(flowx, flowy, flag);
	//printf("division l=%f m=%f r=%f\n", flag[0], flag[1], flag[2]);
	cvReleaseImage(&image1_8bit);
	cvReleaseImage(&image2_8bit);
	cvReleaseImage(&flowx);
	cvReleaseImage(&flowy);

}
