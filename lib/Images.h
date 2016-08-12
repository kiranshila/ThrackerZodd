#pragma once

typedef unsigned char uchar;

class image {
public:
	uchar *data, *t; //brightness values
	int *nodes, *q, *q2; //things used in blob detection
	int width, height, numNodes, numQ, numQ2;
	image() {

	}
	image(int w, int h) {
		width = w;
		height = h;
		nodes = (int*)malloc(sizeof(int) * 2 * width * height);
		q = (int*)malloc(sizeof(int) * 2 * width * height);
		q2 = (int*)malloc(sizeof(int) * 2 * width * height);
		t = (uchar*)malloc(sizeof(uchar) * width * height);
	}
};

