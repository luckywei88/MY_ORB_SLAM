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


#include "Converter.h"

namespace ORB_SLAM2
{

std::vector<cv::Mat> Converter::toDescriptorVector(const cv::Mat &Descriptors)
{
    std::vector<cv::Mat> vDesc;
    vDesc.reserve(Descriptors.rows);
    for (int j=0;j<Descriptors.rows;j++)
        vDesc.push_back(Descriptors.row(j));

    return vDesc;
}

g2o::SE3Quat Converter::toSE3Quat(const cv::Mat &cvT)
{
    Eigen::Matrix<double,3,3> R;
    R << cvT.at<float>(0,0), cvT.at<float>(0,1), cvT.at<float>(0,2),
         cvT.at<float>(1,0), cvT.at<float>(1,1), cvT.at<float>(1,2),
         cvT.at<float>(2,0), cvT.at<float>(2,1), cvT.at<float>(2,2);

    Eigen::Matrix<double,3,1> t(cvT.at<float>(0,3), cvT.at<float>(1,3), cvT.at<float>(2,3));

    return g2o::SE3Quat(R,t);
}

cv::Mat Converter::toCvMat(const g2o::SE3Quat &SE3)
{
    Eigen::Matrix<double,4,4> eigMat = SE3.to_homogeneous_matrix();
    return toCvMat(eigMat);
}

cv::Mat Converter::toCvMat(const g2o::Sim3 &Sim3)
{
    Eigen::Matrix3d eigR = Sim3.rotation().toRotationMatrix();
    Eigen::Vector3d eigt = Sim3.translation();
    double s = Sim3.scale();
    return toCvSE3(s*eigR,eigt);
}

cv::Mat Converter::toCvMat(const Eigen::Matrix<float,4,4> &m)
{
    cv::Mat cvMat(4,4,CV_32F);
    for(int i=0;i<4;i++)
        for(int j=0; j<4; j++)
            cvMat.at<float>(i,j)=m(i,j);

    return cvMat.clone();
}

cv::Mat Converter::toCvMat(const Eigen::Matrix<double,4,4> &m)
{
    cv::Mat cvMat(4,4,CV_32F);
    for(int i=0;i<4;i++)
        for(int j=0; j<4; j++)
            cvMat.at<float>(i,j)=m(i,j);

    return cvMat.clone();
}


cv::Mat Converter::FtoCvMat(const Eigen::Matrix<double,4,4> &m)
{
    cv::Mat cvMat(4,4,CV_32F);
    for(int i=0;i<4;i++)
        for(int j=0; j<4; j++)
            cvMat.at<float>(i,j)=m(i,j);

    return cvMat.clone();
}


cv::Mat Converter::toCvMat(const Eigen::Matrix3d &m)
{
    cv::Mat cvMat(3,3,CV_32F);
    for(int i=0;i<3;i++)
        for(int j=0; j<3; j++)
            cvMat.at<float>(i,j)=m(i,j);

    return cvMat.clone();
}

cv::Mat Converter::toCvMat(const Eigen::Matrix<double,3,1> &m)
{
    cv::Mat cvMat(3,1,CV_32F);
    for(int i=0;i<3;i++)
            cvMat.at<float>(i)=m(i);

    return cvMat.clone();
}

cv::Mat Converter::toCvSE3(const Eigen::Matrix<double,3,3> &R, const Eigen::Matrix<double,3,1> &t)
{
    cv::Mat cvMat = cv::Mat::eye(4,4,CV_32F);
    for(int i=0;i<3;i++)
    {
        for(int j=0;j<3;j++)
        {
            cvMat.at<float>(i,j)=R(i,j);
        }
    }
    for(int i=0;i<3;i++)
    {
        cvMat.at<float>(i,3)=t(i);
    }

    return cvMat.clone();
}

Eigen::Matrix<double,3,1> Converter::toVector3d(const cv::Mat &cvVector)
{
    Eigen::Matrix<double,3,1> v;
    v << cvVector.at<float>(0), cvVector.at<float>(1), cvVector.at<float>(2);

    return v;
}

Eigen::Matrix<double,3,1> Converter::toVector3d(const cv::Point3f &cvPoint)
{
    Eigen::Matrix<double,3,1> v;
    v << cvPoint.x, cvPoint.y, cvPoint.z;

    return v;
}

Eigen::Matrix<double,3,3> Converter::toMatrix3d(const cv::Mat &cvMat3)
{
    Eigen::Matrix<double,3,3> M;

    M << cvMat3.at<float>(0,0), cvMat3.at<float>(0,1), cvMat3.at<float>(0,2),
         cvMat3.at<float>(1,0), cvMat3.at<float>(1,1), cvMat3.at<float>(1,2),
         cvMat3.at<float>(2,0), cvMat3.at<float>(2,1), cvMat3.at<float>(2,2);

    return M;
}

Eigen::Matrix<float,4,4> Converter::toMatrix4d(const cv::Mat &cvMat4)
{
    Eigen::Matrix<float,4,4> M;

    M << cvMat4.at<float>(0,0), cvMat4.at<float>(0,1), cvMat4.at<float>(0,2), cvMat4.at<float>(0,3),
         cvMat4.at<float>(1,0), cvMat4.at<float>(1,1), cvMat4.at<float>(1,2), cvMat4.at<float>(1,3),
         cvMat4.at<float>(2,0), cvMat4.at<float>(2,1), cvMat4.at<float>(2,2), cvMat4.at<float>(2,3),
	 0,0,0,1;

    return M;
}

std::vector<float> Converter::toQuaternion(const cv::Mat &M)
{
    Eigen::Matrix<double,3,3> eigMat = toMatrix3d(M);
    Eigen::Quaterniond q(eigMat);

    std::vector<float> v(4);
    v[0] = q.x();
    v[1] = q.y();
    v[2] = q.z();
    v[3] = q.w();

    return v;
}

cv::Mat Converter::toLeft(cv::Mat &TwcR)
{
    cv::Mat tmpR=TwcR.rowRange(0,3).colRange(0,3).clone();
    std::vector<float> q=Converter::toQuaternion(tmpR);
    std::vector<float> newq(4);
    newq[0]= q[1];
    newq[1]= q[2];
    newq[2]=-q[0];
    newq[3]=-q[3];
    cv::Mat R=toMatrix3(newq);

    float b[]={
	-TwcR.at<float>(1,3),-TwcR.at<float>(2,3),TwcR.at<float>(0,3)};
    cv::Mat T(3,1,CV_32F,b);

    cv::Mat Twc= cv::Mat::eye(4,4,TwcR.type());
    R.copyTo(Twc.rowRange(0,3).colRange(0,3));
    T.copyTo(Twc.rowRange(0,3).col(3));
    return Twc.clone();
}

cv::Mat Converter::toRight(cv::Mat &Twc)
{
    cv::Mat tmpR=Twc.rowRange(0,3).colRange(0,3).clone();
    std::vector<float> q=Converter::toQuaternion(tmpR);
    std::vector<float> newq(4);
    newq[0]=-q[2];
    newq[1]= q[0];
    newq[2]= q[1];
    newq[3]=-q[3];
    cv::Mat R=toMatrix3(newq);

    float b[]={
	Twc.at<float>(2,3),-Twc.at<float>(0,3),-Twc.at<float>(1,3)};
    cv::Mat T(3,1,CV_32F,b);

    cv::Mat TwcR= cv::Mat::eye(4,4,Twc.type());
    R.copyTo(TwcR.rowRange(0,3).colRange(0,3));
    T.copyTo(TwcR.rowRange(0,3).col(3));
    return TwcR.clone();
}

cv::Mat Converter::toMatrix3(std::vector<float>& q)
{
    float x=q[0];
    float y=q[1];
    float z=q[2];
    float w=q[3];

    float s=2.0f/(x*x+y*y+z*z+w*w);

    float xs=x*s;
    float ys=y*s;
    float zs=z*s;

    float wx=w*xs;
    float wy=w*ys;
    float wz=w*zs;

    float xx=x*xs;
    float xy=x*ys;
    float xz=x*zs;

    float yy=y*ys;
    float yz=y*zs;
    float zz=z*zs;
	
    float a[]={
	     (float)1.0 - (yy + zz), xy - wz,         xz + wy,
	     xy + wz,         (float)1.0 - (xx + zz), yz - wx,
	     xz - wy,         yz + wx,        (float)1.0 - (xx + yy)};
    cv::Mat currentm(3,3,CV_32F,a);

    return currentm.clone();
}



} //namespace ORB_SLAM
