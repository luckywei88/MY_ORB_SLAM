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

#ifndef KEYFRAMEDRAWER_H
#define KEYFRAMEDRAWER_H

#include "LocalMapping.h"
#include "MapPoint.h"
#include "Map.h"
#include "Yolo.h"

#include<opencv2/core/core.hpp>
#include<opencv2/features2d/features2d.hpp>

#include <mutex>
#include <vector>


namespace ORB_SLAM2
{

class KeyFrameDrawer;
class Viewer;
class Yolo;

class KeyFrameDrawer
{
public:
    KeyFrameDrawer();

    // Update info from the last processed frame.
    void Update(IplImage* src,char** tnames,image** talphabet,int classes, std::vector<int> box,std::vector<int> type);

    // Draw last processed frame.
    cv::Mat DrawKeyFrame();


protected:

    void DrawTextInfo(cv::Mat &im, int nState, cv::Mat &imText);
    float get_color(int c, int x, int max);

    // Info of the frame to be drawn
    std::vector<int> boxes;
    std::vector<int> types;
    char **names;
    image **alphabet;
    image mIm;
    float **colors;
    int classes;
    volatile int update;
    cv::Mat result;

    std::mutex mMutex;
};

} //namespace ORB_SLAM

#endif // FRAMEDRAWER_H
