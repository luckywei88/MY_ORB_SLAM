#ifndef YOLO_H
#define YOLO_H 

#include <opencv/cv.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "parser.h"
#include "detector.h"
#include "cuda.h"
#include "connected_layer.h"
#include "network.h"
#include "region_layer.h"
#include "cost_layer.h"
#include "utils.h"
#include "box.h"
#include "demo.h"
#include "option_list.h"
#include "blas.h"
#include "darknet/include/darknet.h"

namespace ORB_SLAM2
{
    class Yolo
    {
	public:
	    void yolo_load(char *datacfg, char *cfgfile, char *weightfile,  float thresh, float hier_thresh);
	    void yolo_detect(IplImage* input);
	    box *boxes;
	    float **probs;
	    char** names;
	    image** alphabet;
	    int total;
	    int numOfC;
	private:
	    void Yolo::detect_img(network net, image im, float thresh, float hier_thresh, box *boxes, float **probs, char **names);
	    char** Yolo::load_name(char *datacfg);
	    network Yolo::load(char *cfgfile, char *weightfile);
	    
	    image ipl_to_image(IplImage* src);
	    image make_empty_image(int w,int h,int c);
	    image make_image(int w,int h,int c);
	    float thresh;
	    float hier_thresh;
	    network net;
	    layer l;
    };
}


#endif
