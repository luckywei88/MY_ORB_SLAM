#ifndef YOLO_H
#define YOLO_H 

#include <opencv/cv.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#ifdef GPU
#include "cuda_runtime.h"
#include "curand.h"
#include "cublas_v2.h"
#endif

extern "C"
{
#include "darknet/include/darknet.h"
#include "darknet/src/network.h"
#include "darknet/src/image.h"
}

namespace ORB_SLAM2
{

	class Yolo
	{
		public:
			char** load_name(char *datacfg);
			network* load_network(char *cfgfile, char *weightfile, int clear);
			image** load_alphabet(char *labelfile);
			void detect_img(network* net, image im, float thresh, float hier_thresh, box *boxes, float **probs, float **masks, char **names);
			void yolo_load(char *datacfg, char *cfgfile, char *weightfile,  char *labelfile, float thresh, float hier_thresh);
			void yolo_detect(IplImage* input);
			void delet();

			static image ipl_to_image(IplImage* src);
			static image make_empty_image(int w,int h,int c);
			static image make_image(int w,int h,int c);

			box *boxes;
			float **probs;
			char** names;
			image **alphabet;
			int total;
			int classes;
			float thresh;
		private:
			float hier_thresh;
			network* net;
			layer l;
	};
}


#endif
