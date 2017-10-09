#include "Yolo.h"

namespace ORB_SLAM2
{

	char** Yolo::load_name(char *datacfg)
	{
		yolo_list *options = read_data_cfg(datacfg);
		char *name_list = option_find_str(options,"names","data/names.list");
		return get_labels(name_list);
	}

	network Yolo::load_network(char *cfgfile,char *weightfile)
	{
		network net=parse_network_cfg(cfgfile);
		if(weightfile)
			load_weights(&net,weightfile);
		set_batch_network(&net,1);
		return net;
	}

	void Yolo::detect_img(network net, image im, float thresh, float hier_thresh, box *boxes, float **probs, float **masks, char **names)
	{
		float *X = im.data;
		double time=what_time_is_it_now();
		network_predict(net, X);
		printf("Predicted in %f seconds.\n", what_time_is_it_now()-time);
		get_region_boxes(l, im.w, im.h, net.w, net.h, thresh, probs, boxes, masks, 0, 0, hier_thresh, 1);
		do_nms_obj(boxes,probs,l.w*l.h*l.n,l.classes,0.3);	
	}


	void Yolo::yolo_load(char *datacfg, char *cfgfile, char *weightfile,  float mthresh, float mhier_thresh)
	{
		names=load_name(datacfg);
		printf("get weight %s\n",weightfile);
		net=load_network(cfgfile,weightfile);
		thresh=mthresh;
		hier_thresh=mhier_thresh;
	}

	image Yolo::make_empty_image(int w, int h, int c)
	{
		image out;
		out.data = 0;
		out.h = h;
		out.w = w;
		out.c = c;
		return out;
	}

	image Yolo::make_image(int w, int h, int c)
	{
		image out = make_empty_image(w,h,c);
		out.data = (float *)calloc(h*w*c, sizeof(float));
		return out;
	}



	image Yolo::ipl_to_image(IplImage* src)
	{
		unsigned char *data = (unsigned char *)src->imageData;
		int h = src->height;
		int w = src->width;
		int c = src->nChannels;
		int step = src->widthStep;
		image out = make_image(w, h, c);
		int i, j, k, count=0;;

		for(k= 0; k < c; ++k){
			for(i = 0; i < h; ++i){
				for(j = 0; j < w; ++j){
					out.data[count++] = data[i*step + j*c + k]/255.;
				}
			}
		}    
		return out; 

	}

	void Yolo::yolo_detect(IplImage* input)
	{
		image out=ipl_to_image(input);
		l=net.layers[net.n-1];
		total=l.w*l.h*l.n;
		classes=l.classes;
		boxes = (box*)calloc(total, sizeof(box));
		probs = (float**)calloc(total, sizeof(float *));
		for(int j = 0; j < total; ++j) 
			probs[j] = (float*)calloc(l.classes + 1, sizeof(float *));

		float **masks = 0;
		if (l.coords > 4){
			masks = (float**)calloc(l.w*l.h*l.n, sizeof(float*));
			for(int j = 0; j < l.w*l.h*l.n; ++j) 
				masks[j] =(float*) calloc(l.coords-4, sizeof(float *));
		}
		detect_img(net,out,thresh,hier_thresh,boxes,probs,masks,names);

	}

	void Yolo::delet()
	{
		free(probs);
		free(boxes);
	}

}

