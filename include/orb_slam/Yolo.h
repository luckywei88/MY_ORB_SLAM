#ifndef YOLO_H
#define YOLO_H 

#include <opencv/cv.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

extern "C"
{
	#include "darknet/include/darknet.h"
}

namespace ORB_SLAM2
{

    class Yolo
    {
	    public:
		    char** load_name(char *datacfg);
		    network load_network(char *cfgfile, char *weightfile);
		    void detect_img(network net, image im, float thresh, float hier_thresh, box *boxes, float **probs, float **masks, char **names);
		    void yolo_load(char *datacfg, char *cfgfile, char *weightfile,  float thresh, float hier_thresh);
		    void yolo_detect(IplImage* input);
		    void delet();
		    box *boxes;
		    float **probs;
		    char** names;
		    int total;
		    int classes;
		    float thresh;
	    private:
		    image ipl_to_image(IplImage* src);
		    image make_empty_image(int w,int h,int c);
		    image make_image(int w,int h,int c);
		    float hier_thresh;
		    network net;
		    layer l;
    };
}


#endif
