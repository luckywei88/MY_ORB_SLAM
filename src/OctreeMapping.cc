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

#include "LocalMapping.h"
#include "LoopClosing.h"
#include "ORBmatcher.h"
#include "Optimizer.h"
#include "Object.h"

#include<mutex>

using namespace std;
using namespace octomap;

namespace ORB_SLAM2
{

OctreeMapping::OctreeMapping(float m_res);
{
	m_octree=new OcTree(m_res);
	m_octree->totalexpand();
}

point3d OcTreeMapping::pointToOctomap(cv::Mat pose)
{
	float x=;
	float y=;
	float z=;
	
	reutrn point3d(x,y,z);
}


void OcTreeMapping::Run()
{

    mbFinished = false;

    while(1)
    {
	// Tracking will see that Local Mapping is busy
	SetAcceptKeyFrames(false);

	// Check if there are keyframes in the queue
	if(CheckNewKeyFrames())
	{
	    // BoW conversion and insertion in Map

	    ProcessNewKeyFrame();
	}
	else if(Stop())
	{
	    // Safe area to stop
	    while(isStopped() && !CheckFinish())
	    {
		usleep(3000);
	    }
	    if(CheckFinish())
		break;
	}

	ResetIfRequested();

	// Tracking will see that Local Mapping is busy
	SetAcceptKeyFrames(true);

	if(CheckFinish())
	    break;
	
	usleep(3000);
    }

    SetFinish();
}

void LocalMapping::InsertKeyFrame(KeyFrame *pKF)
{
    unique_lock<mutex> lock(mMutexNewKFs);
    mlNewKeyFrames.push_back(pKF);
    mbAbortBA=true;
}


bool LocalMapping::CheckNewKeyFrames()
{
    unique_lock<mutex> lock(mMutexNewKFs);
    return(!mlNewKeyFrames.empty());
}

void LocalMapping::ProcessNewKeyFrame()
{
    {
	unique_lock<mutex> lock(mMutexNewKFs);
	mpCurrentKeyFrame = mlNewKeyFrames.front();
	mlNewKeyFrames.pop_front();
    }
 
    cv::Mat pose=mpCurrentKeyFrame->pose();
    sensorOrigin=PointToOctomap(pose);

    octomap::Pointcloud opcl;
    Point::Ptr pc=mpCurrentKeyFrame->pc;
    for(auto it=pc->begin();it!=pc->end();it++)
	opcl.push_back(it_>x,it->y,it->z);
    m_octree->insertPointCloudfor2d(opcl,sensororigin,10.0,maxheight,lowheight,false);
}

void LocalMapping::RequestStop()
{
    unique_lock<mutex> lock(mMutexStop);
    mbStopRequested = true;
    unique_lock<mutex> lock2(mMutexNewKFs);
    mbAbortBA = true;
}

bool LocalMapping::Stop()
{
    unique_lock<mutex> lock(mMutexStop);
    if(mbStopRequested && !mbNotStop)
    {
	mbStopped = true;
	cout << "Local Mapping STOP" << endl;
	return true;
    }

    return false;
}

bool LocalMapping::isStopped()
{
    unique_lock<mutex> lock(mMutexStop);
    return mbStopped;
}

bool LocalMapping::stopRequested()
{
    unique_lock<mutex> lock(mMutexStop);
    return mbStopRequested;
}

void LocalMapping::Release()
{
    unique_lock<mutex> lock(mMutexStop);
    unique_lock<mutex> lock2(mMutexFinish);
    if(mbFinished)
	return;
    mbStopped = false;
    mbStopRequested = false;
    for(list<KeyFrame*>::iterator lit = mlNewKeyFrames.begin(), lend=mlNewKeyFrames.end(); lit!=lend; lit++)
	delete *lit;
    mlNewKeyFrames.clear();

    cout << "Local Mapping RELEASE" << endl;
}

bool LocalMapping::AcceptKeyFrames()
{
    unique_lock<mutex> lock(mMutexAccept);
    return mbAcceptKeyFrames;
}

void LocalMapping::SetAcceptKeyFrames(bool flag)
{
    unique_lock<mutex> lock(mMutexAccept);
    mbAcceptKeyFrames=flag;
}

bool LocalMapping::SetNotStop(bool flag)
{
    unique_lock<mutex> lock(mMutexStop);

    if(flag && mbStopped)
	return false;

    mbNotStop = flag;

    return true;
}

void LocalMapping::InterruptBA()
{
    mbAbortBA = true;
}


void LocalMapping::RequestReset()
{
    {
	unique_lock<mutex> lock(mMutexReset);
	mbResetRequested = true;
    }

    while(1)
    {
	{
	    unique_lock<mutex> lock2(mMutexReset);
	    if(!mbResetRequested)
		break;
	}
	usleep(3000);
    }
}

void LocalMapping::ResetIfRequested()
{
    unique_lock<mutex> lock(mMutexReset);
    if(mbResetRequested)
    {
	objs->Reset();
	mlNewKeyFrames.clear();
	mlpRecentAddedMapPoints.clear();
	mbResetRequested=false;
    }
}

void LocalMapping::RequestFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    mbFinishRequested = true;
}

bool LocalMapping::CheckFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    return mbFinishRequested;
}

void LocalMapping::SetFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    mbFinished = true;    
    unique_lock<mutex> lock2(mMutexStop);
    mbStopped = true;
}

bool LocalMapping::isFinished()
{
    unique_lock<mutex> lock(mMutexFinish);
    return mbFinished;
}

} //namespace ORB_SLAM
