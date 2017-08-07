#include "Yolo.h"
#include "image.h"

namespace ORB_SLAM2
{

void Yolo::detect_img(network net, image im, float thresh, float hier_thresh, box *boxes, float **probs, char **names)
{
    clock_t time;
    int j;
    float nms=.4;
    image **alphabet = load_alphabet();
    image sized = letterbox_image(im, im.w*1.5, im.h*1.5);
    resize_network(&net, sized.w, sized.h);
    printf("resize network");
    printf("size w %d h %d\n",sized.w,sized.h);
    layer l = net.layers[net.n-1];

    boxes = calloc(l.w*l.h*l.n, sizeof(box));
    probs = calloc(l.w*l.h*l.n, sizeof(float *));
    for(j = 0; j < l.w*l.h*l.n; ++j) probs[j] = calloc(l.classes + 1, sizeof(float *));

    float *X = sized.data;
    time=clock();
    printf("start detect\n");
    network_predict(net, X);
    printf("Predicted in %f seconds.\n", sec(clock()-time));
    get_region_boxes(l, 1, 1, thresh, probs, boxes, 0, 0, hier_thresh);

    if (l.softmax_tree && nms) 
    {
	do_nms_obj(boxes, probs, l.w*l.h*l.n, l.classes, nms);
    }
    else if (nms) 
    {
	do_nms_sort(boxes, probs, l.w*l.h*l.n, l.classes, nms);
    }
    draw_detections(sized, l.w*l.h*l.n, thresh, boxes, probs, names, alphabet, l.classes);
    save_image(sized, "predictions");

}

network Yolo::load(char *cfgfile, char *weightfile)
{

    network net = parse_network_cfg(cfgfile);
    if(weightfile){
	load_weights(&net, weightfile);
    }   
    set_batch_network(&net, 1);
    return net;
}

char** Yolo::load_name(char *datacfg)
{
    yolo_list *options = read_data_cfg(datacfg);
    char *name_yolo_list = option_find_str(options, "names", "data/names.yolo_list");
    char **names = get_labels(name_yolo_list);
    return names;
}

void Yolo::yolo_load(char *datacfg, char *cfgfile, char *weightfile,  float mthresh, float mhier_thresh)
{
    names=load_name(datacfg);
    printf("get weight %s\n",weightfile);
    net=load(cfgfile,weightfile);
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
    layer l=net.layers[net.n-1];
    total=l.w*l.h*l.n;
    classes=l.classes;
    boxes = (box*)calloc(total, sizeof(box));
    probs = (float**)calloc(total, sizeof(float *));
    for(int j = 0; j < total; ++j) 
	probs[j] = (float*)calloc(l.classes + 1, sizeof(float *));
    detect_img(net,out,thresh,hier_thresh,boxes,probs,names);
}

void Yolo::delet()
{
    free(probs);
    free(boxes);
}

}

