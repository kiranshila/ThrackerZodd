#include <stdio.h>
#include <iostream>

#include "Tracker.h"

using namespace std;

//#define _draw_

Tracker T;

int main() {

	// Verify that piped outputs are line-buffered
	setvbuf(stdout, (char *) NULL, _IOLBF, 0); /* make line buffered stdout */
	T = Tracker("config.tz");

#ifdef _draw_
	namedWindow("window", 1);
#endif

	while (1) {
		T.update();

#ifdef _draw_
		if (T.C.size() > 0) {
			line(img, T.C[0][0], T.C[0][1], Scalar(0, 0, 255));
			line(img, T.C[0][1], T.C[0][2], Scalar(0, 255, 0));
			line(img, T.C[0][2], T.C[0][3], Scalar(255, 0, 0));
			line(img, T.C[0][3], T.C[0][0], Scalar(128, 128, 128));
			circle(img, T.C[0][0], 40, Scalar(0, 0, 255), 2, 8);
			circle(img, T.C[0][1], 40, Scalar(0, 0, 255), 2, 8);
			circle(img, T.C[0][2], 40, Scalar(0, 0, 255), 2, 8);
			circle(img, T.C[0][3], 40, Scalar(0, 0, 255), 2, 8);
		}

		for (int n = 0; n < T.P.size(); n++) {
			circle(img, Point(T.P[n].x, T.P[n].y), sqrt(T.P[n].num / PI), Scalar(0, 255, 0), 4, 8);
		}

		imshow("window", img);
#endif

		std::cout << T.tvecf.print((T.trackerFound)?1.0:0.0) << "\n";
	}
	return 0;
}
