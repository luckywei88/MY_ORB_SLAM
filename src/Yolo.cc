#include "Yolo.h"

namespace ORB_SLAM2
{

	char** Yolo::load_name(char *datacfg)
	{
		yolo_list *options = read_data_cfg(datacfg);
		char *name_list = option_find_str(options,"names","data/names.list");
		return get_labels(name_list);
	}

	network* Yolo::load_network(char *cfgfile,char *weightfile,int clear)
	{
		
		network* net=parse_network_cfg(cfgfile);
		l=net->layers[15];
		total=l.w*l.h*l.n;
		if(weightfile && weightfile[0]!=0)
		{
			load_weights(net,weightfile);
		}
		if(clear)
		{
			(*net->seen)=0;
		}
		set_batch_network(net,1);
		return net;
	}

	image** Yolo::load_alphabet(char *labelfile)
	{
		int i, j;
		const int nsize = 8;
		image **alphabets =(image**) calloc(nsize, sizeof(image));
		for(j = 0; j < nsize; ++j){
			alphabets[j] = (image*) calloc(128, sizeof(image));
			for(i = 32; i < 127; ++i){
				char buff[256];
				sprintf(buff, "%s/%d_%d.png", labelfile, i, j);
				alphabets[j][i] = load_image_color(buff, 0, 0);
			}
		}
		return alphabets;

	}	

	void Yolo::detect_img(network* net, image im, float thresh, float hier_thresh, box *boxes, float **probs, float **masks, char **names)
	{
		float *X = im.data;
		double time=what_time_is_it_now();
		network_predict(net, X);
		printf("Predicted in %f seconds.\n", what_time_is_it_now()-time);
		get_region_boxes(l, im.w, im.h, net->w, net->h, thresh, probs, boxes, masks, 0, 0, hier_thresh, 1);
		do_nms_sort(boxes,probs,total,l.classes,0.3);	
	}


	void Yolo::yolo_load(char *datacfg, char *cfgfile, char *weightfile, char *labelfile, float mthresh, float mhier_thresh)
	{
		names=load_name(datacfg);
		alphabet=load_alphabet(labelfile);
		net=load_network(cfgfile,weightfile,0);
		l=net->layers[net->n-1];
		total=l.w*l.h*l.n;
		classes=l.classes;
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
		boxes = (box*)calloc(total, sizeof(box));
		probs = (float**)calloc(total, sizeof(float *));
		for(int j = 0; j < total; ++j) 
			probs[j] = (float*)calloc(l.classes + 1, sizeof(float *));

		float **masks = 0;
		if (l.coords > 4){
			masks = (float**)calloc(total, sizeof(float*));
			for(int j = 0; j < total; ++j) 
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

