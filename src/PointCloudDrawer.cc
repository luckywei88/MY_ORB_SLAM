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

#include "PointCloudDrawer.h"
#include "Objects.h"
#include <pangolin/pangolin.h>
#include <mutex>

namespace ORB_SLAM2
{


PointCloudDrawer::PointCloudDrawer(Objects* objs, const string &strSettingPath):mobjs(objs)
{
    cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);

    mPointSize = fSettings["Viewer.PointSize"];

    x3Dc=(cv::Mat_<float>(3,1)<<0,0,0);
}

void PointCloudDrawer::DrawMapPoints()
{

    const list<Object*> &vobjs = mobjs->GetAllObjects();
    
    if(vobjs.empty())
	return;
	
    glPointSize(mPointSize);
    glBegin(GL_POINTS);

    for(auto itobj=vobjs.begin();itobj!=vobjs.end();itobj++)
    {
	Object* obj=*itobj;
        map<KeyFrame*, PointC::Ptr> pcmap=obj->pcmap;
	for(auto it=pcmap.begin();it!=pcmap.end();it++)
	{
		KeyFrame* kf=it->first;
		if(kf->isBad())
			continue;
		PointC::Ptr pc=it->second;
		cv::Mat pose=kf->GetPose();
		cv::Mat rwc=pose.rowRange(0,3).colRange(0,3);
		cv::Mat ow=pose.rowRange(0,3).col(3);
		for(auto itpc=pc->begin();itpc!=pc->end();itpc++)
		{
			unsigned int col=*reinterpret_cast<unsigned int*>(&itpc->rgb);
			int b=col&0xff;
			int g=(col>>8)&0xff;
			int r=(col>>16)&0xff;
			float bf=b/255.0;
			float gf=g/255.0;
			float rf=r/255.0;
			glColor3f(rf,gf,bf);
			x3Dc.at<float>(0)=itpc->x;
			x3Dc.at<float>(1)=itpc->y;
			x3Dc.at<float>(2)=itpc->z;
			x3Dc=rwc*x3Dc+ow;
			glVertex3f(x3Dc.at<float>(0),x3Dc.at<float>(1),x3Dc.at<float>(2));
		}
		break;

	}
    }
    glEnd();

}

} //namespace ORB_SLAM
