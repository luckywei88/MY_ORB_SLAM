/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/

#include "KeyFrameDrawer.h"
#include "Tracking.h"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include<mutex>
#include <math.h>

namespace ORB_SLAM2
{

KeyFrameDrawer::KeyFrameDrawer()
{
    names=NULL;
    alphabet=NULL;
    result = cv::Mat(480,640,CV_8UC3, cv::Scalar(0,0,0)); 
    update=0;

    float cs[6][3] = { {1,0,1}, {0,0,1},{0,1,1},{0,1,0},{1,1,0},{1,0,0} };
    colors=(float**) calloc(6,sizeof(float*));
    for(int i=0;i<6;i++)
    {
	colors[i]=(float*) calloc(3,sizeof(float));
	for(int j=0;j<3;j++)
             colors[i][j]=cs[i][j];
    }
}

float KeyFrameDrawer::get_color(int c, int x, int max)
{
	float ratio = ((float)x/max)*5;
	int i = floor(ratio);
	int j = ceil(ratio);
	ratio -= i;
	float r = (1-ratio) * colors[i][c] + ratio*colors[j][c];
	return r;	
}

cv::Mat KeyFrameDrawer::DrawKeyFrame()
{
	if(update)
	{
		image copy;
		int num=0;
		//Copy variables within scoped mutex
		{
			unique_lock<mutex> lock(mMutex);
			num=types.size();

			for(int i=0;i<num;i++)
			{
				int width=mIm.h*0.006;
				int clazz=types[i];
				
				int offset = clazz*123457 % classes;
				float red = get_color(2,offset,classes);
				float green = get_color(1,offset,classes);
				float blue = get_color(0,offset,classes);
				float rgb[3];

				rgb[0] = red;
				rgb[1] = green;
				rgb[2] = blue;
				
				int left=boxes[4*i];
				int right=boxes[4*i+1];
				int top=boxes[4*i+2];
				int bot=boxes[4*i+3];

				draw_box_width(mIm,left,top,right,bot,width,red,green,blue);
				image label=get_label(alphabet,names[clazz],(mIm.h*0.03)/10);
				draw_label(mIm,top+width,left,label,rgb);
				free_image(label);


			}
			copy = copy_image(mIm);
			/*
			if(mIm.c == 3) 
				rgbgr_image(copy);
			*/
			update=0;

		} // destroy scoped mutex -> release mutex

		//Draw
		int x,y,k;
		IplImage *disp = cvCreateImage(cvSize(copy.w,copy.h), IPL_DEPTH_8U, copy.c);
		int step = disp->widthStep;

		for(y = 0; y < copy.h; ++y){
			for(x = 0; x < copy.w; ++x){
				for(k= 0; k < copy.c; ++k){

					disp->imageData[y*step + x*copy.c + k] = (unsigned char)(get_pixel(copy,x,y,k)*255);
				}
			}
		}

		cv::Mat im=cv::Mat(disp);
		cv::Mat imWithInfo;
		DrawTextInfo(im,num, imWithInfo);
		result=imWithInfo;
		return result;
	}
	else
	{
		return result;	
	}
}


void KeyFrameDrawer::DrawTextInfo(cv::Mat &im, int num, cv::Mat &imText)
{
	stringstream s;
	s << " Detect "<<num<<" Objects";

	int baseline=0;
	cv::Size textSize = cv::getTextSize(s.str(),cv::FONT_HERSHEY_PLAIN,1,1,&baseline);

	imText = cv::Mat(im.rows+textSize.height+10,im.cols,im.type());
	im.copyTo(imText.rowRange(0,im.rows).colRange(0,im.cols));
	imText.rowRange(im.rows,imText.rows) = cv::Mat::zeros(textSize.height+10,im.cols,im.type());
	cv::putText(imText,s.str(),cv::Point(5,imText.rows-5),cv::FONT_HERSHEY_PLAIN,1,cv::Scalar(255,255,255),1,8);

}

void KeyFrameDrawer::Update(IplImage *img,char** tnames,image** talphabet, int tclasses, vector<int> box,vector<int> type)
{
	names=tnames;
	alphabet=talphabet;
	classes=tclasses;
	{
		unique_lock<mutex> lock(mMutex);
		mIm=Yolo::ipl_to_image(img);
		types.clear();
		boxes.clear();
		for(size_t i=0;i<type.size();i++)
		{
			boxes.push_back(box[4*i]);
			boxes.push_back(box[4*i+1]);
			boxes.push_back(box[4*i+2]);
			boxes.push_back(box[4*i+3]);
			types.push_back(type[i]);	
		}
		update=1;
		//cout<<"update finished"<<endl;
	}
}

} //namespace ORB_SLAM
