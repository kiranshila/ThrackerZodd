#pragma once

#define PI 3.141592653589

struct point {
	float x, y, brightness;
	int num;
};

/*
This function finds blobs in an image.
The blobs must be of brightness above (T2), and contain pixels
above (T1). The blobs are automatically rejected if the are not
round enough. This is controlled with (r), which determines the
maximum ratio between pixels outside a circle of the same
area as the blob, and the area of the whole blob.
*/
void findPoints(image& I, std::vector<point>& P, uchar T1, uchar T2, float r) {
	int width = I.width;
	int height = I.height;
	I.numNodes = 0; //reset node count (only looks at nodes upto this number)
	for (int x = 0; x < width; x++) {
		for (int y = 0; y < height; y++) {
			if (I.data[x + y*width] > T1) { //add nodes in image that are candidates (>T1)
				I.nodes[I.numNodes * 2] = x;
				I.nodes[I.numNodes * 2 + 1] = y;
				I.numNodes++;
			}
		}
	}
	if (I.numNodes > 0) { //if nodes are found, find points
		memset(I.t, 0, sizeof(uchar) * width * height); //reset t for flood fill
		int count = 0;
		while (count < 2048) { //makes sure not to over load with points
			int bx = -1, by = 0;
			uchar br = 0;
			for (int n = 0; n < I.numNodes; n++) { //find brightest node
				int x = I.nodes[2 * n];
				if (x > 0) { //is node active?
					int y = I.nodes[2 * n + 1];
					uchar b = I.data[x + y*width]; //if it is, see if it is the new brightest node
					if (b > br) {
						br = b;
						bx = x;
						by = y;
					}
					else if (b == 0) //if the brightness is 0, then this node is no longer active
					{
						I.nodes[2 * n] = -1; //remove node
					}
				}
			}
			if (bx < 0) //if no nodes are active, we've found all blobs
				break;
			I.numQ = 0;
			I.numQ2 = 0;
			I.q[I.numQ * 2] = bx; //reset the flood fill queue and add the first job to the queue
			I.q[I.numQ * 2 + 1] = by;
			I.numQ++;
			float mx = 0, my = 0, w = 0; //mean position and weight
			float num = 0; //number of pixels in this blob
			uchar weight; //value used thought flood fill
			int Wx, Wy, Ex, Ey; //westward and eastward runners
			for (int n = 0; n < I.numQ; n++) { //iterate though all jobs in queue
				Wx = I.q[2 * n]; //westward runner is initialized at the job point
				Wy = I.q[2 * n + 1];
				Ex = Wx + 1; //eastward runner is initialized to the right of the job point
				Ey = Wy;
				while (Wx >= 0 && I.data[Wx + Wy*width] >= T2) { //westward runner runs until it hits the edge of the blob or the edge of the image
					weight = I.data[Wx + Wy*width];
					I.data[Wx + Wy*width] = 0;
					I.t[Wx + width*Wy] = 1;
					mx += weight*Wx;
					my += weight*Wy;
					w += weight;
					num++;
					I.q2[I.numQ2 * 2] = Wx;
					I.q2[I.numQ2 * 2 + 1] = Wy;
					I.numQ2++;
					if (Wy < height - 1) { //check above and below the runner for possible candidates for queue
						if (I.t[Wx + width*(Wy + 1)] == 0 && I.data[Wx + (Wy + 1)*width] > T2) {
							I.q[I.numQ * 2] = Wx;
							I.q[I.numQ * 2 + 1] = Wy + 1;
							I.numQ++;
							I.t[Wx + width*(Wy + 1)] = 1;
						}
					}
					if (Wy > 0) {
						if (I.t[Wx + width*(Wy - 1)] == 0 && I.data[Wx + (Wy - 1)*width] > T2) {
							I.q[I.numQ * 2] = Wx;
							I.q[I.numQ * 2 + 1] = Wy - 1;
							I.numQ++;
							I.t[Wx + width*(Wy - 1)] = 1;
						}
					}
					Wx--; //move runner to the left
				}
				if (Ex < width) {
					while (Ex < width && I.data[Ex + Ey*width] >= T2) { //eastward runner runs until it hits the edge of the blob or the edge of the image
						weight = I.data[Ex + Ey*width];
						I.data[Ex + Ey*width] = 0;
						I.t[Ex + width*Ey] = 1;
						mx += weight*Ex;
						my += weight*Ey;
						w += weight;
						num++;
						I.q2[I.numQ2 * 2] = Ex;
						I.q2[I.numQ2 * 2 + 1] = Ey;
						I.numQ2++;
						if (Ey < height - 1) { //check above and below the runner for possible candidates for queue
							if (I.t[Ex + width*(Ey + 1)] == 0 && I.data[Ex + (Ey + 1)*width] > T2) {
								I.q[I.numQ * 2] = Ex;
								I.q[I.numQ * 2 + 1] = Ey + 1;
								I.numQ++;
								I.t[Ex + width*(Ey + 1)] = 1;
							}
						}
						if (Ey > 0) {
							if (I.t[Ex + width*(Ey - 1)] == 0 && I.data[Ex + (Ey - 1)*width] > T2) {
								I.q[I.numQ * 2] = Ex;
								I.q[I.numQ * 2 + 1] = Ey - 1;
								I.numQ++;
								I.t[Ex + width*(Ey - 1)] = 1;
							}
						}
						Ex++; //move runner to the right
					}
				}
			}
			mx /= w; //mean position of points
			my /= w;
			w /= num; //mean brightness
			float r2 = (float)num / PI; //radius squared of a circle with area equal to the number of pixels in the blob
			int round = 0; //number of pixels outside of this circle
			for (int n = 0; n < I.numQ2; n++) { //iterate through all pixels in the queue
				float xx = I.q2[2 * n] - mx;
				float yy = I.q2[2 * n + 1] - my;
				if ((xx*xx + yy*yy) > r2) { //if pixel is outside of the circle, round is incremented
					round++;
				}
			}
			if (((float)round / (float)num) < r) {
				P.push_back({ mx, my, w, (int)num });
			}
			count++;
		}
	}
}

