#include "Object.h"

namespace ORB_SLAM2
{

    Object::Object(KeyFrame* kf, PointC::Ptr pc, float* prob,int n)
    {
	nObs=1;
	maxProb=-1;
	pcmap.insert(make_pair(kf,pc));
	for(int i=0;i<n;i++)
	{
	    probs.push_back(prob[i]);
	    if(prob[i]>maxProb)
	    {
		maxProb=prob[i];
		clazz=i;
	    }
	}
    }

    void Object::add(KeyFrame* kf,PointC::Ptr pc, float* prob)
    {
	nObs++;
	pcmap.insert(make_pair(kf,pc));
	for(size_t i=0;i<probs.size();i++)
	{
		probs[i]+=prob[i];
		if(probs[i]>maxProb)
		{
		    maxProb=probs[i];
		    clazz=i;
		}
	}
    }

    void Object::remove(KeyFrame* kf)
    {
	pcmap.erase(kf);
    }

    bool Object::compare(int type, PointC::Ptr pc)
    {
	if(type==clazz)
	    return true;
	return false;
	/*
	kdtree.setInputCloud(AllPC);
	int k=1;
	size_t size=pc->size();
	cout<<"all pc "<<AllPC->size()<<endl;
	cout<<" pc "<<pc->size()<<endl;
	std::vector<int> IdxVector(k);
	std::vector<float> SDisVector(k);
	size_t confirm=0;
	size_t find=0;
	for(size_t i=0;i<size;i++)
	{
	    if(kdtree.nearestKSearch(pc->at(i),k,IdxVector,SDisVector))
	    {
		find++;
		PointT pt=pc->at(i);
		//cout<<"pt "<<pt.x<<" "<<pt.y<<" "<<pt.z<<endl;
		float dis=SDisVector[0];
		//cout<<"dis "<<dis<<endl;
		if(dis<=0.04)
		    confirm++;
	    }
	}
	cout<<"confirm "<<confirm<<endl;
	cout<<"find "<<find<<endl;
	cout<<"score "<<(double)confirm/(double)size<<endl;
	if(confirm*2>=size)
	    return true;
	return false;
	*/
    }

    float Object::getProb()
    {
	return maxProb/(float) clazz;
    }

    int Object::getType()
    {
	return clazz;
    }

    void Object::SetPC(PointC::Ptr pc)
    {
	AllPC=pc;
    }

    Object::PointC::Ptr Object::GetPC()
    {
	return AllPC;
    }
}
