#pragma once

Mat img, gray_image;

struct Homology {
	float a, b, d, e, g, h, x1, y1;
	/*
	Computes a homology of a point onto a quad
	*/
	void computeMatrix(float X1, float Y1, float X2, float Y2, float X3, float Y3, float X4, float Y4) {
		X2 -= X1;
		X3 -= X1;
		X4 -= X1;
		Y2 -= Y1;
		Y3 -= Y1;
		Y4 -= Y1;
		x1 = X1;
		y1 = Y1;
		float c1 = X2*Y3 - X3*Y2;
		float c2 = X2*Y4 - X4*Y2;
		float c3 = X3*Y4 - X4*Y3;
		float tmp = c1 - c2 + c3;
		float tmp1 = tmp / (c1*c2);
		float tmp2 = -tmp / (c2*c3);
		float tmp3 = c1*c2*c3;
		a = -Y2*tmp1;
		b = X2*tmp1;
		d = -Y4*tmp2;
		e = X4*tmp2;
		g = -(-X2*X2*Y3*Y3*Y4 + X2*X2*Y3*Y4*Y4 + 2 * X2*X3*Y2*Y3*Y4 - 2 * X2*X3*Y2*Y4*Y4 - X3*X3*Y2*Y2*Y4 + X3*X3*Y2*Y4*Y4 + 2 * X3*X4*Y2*Y2*Y4 - 2 * X3*X4*Y2*Y3*Y4 - X4*X4*Y2*Y2*Y3 + X4*X4*Y2*Y3*Y3) / tmp3;
		h = (-X2*X2*X3*Y4*Y4 - X2*X2*X4*Y3*Y3 + 2 * X2*X2*X4*Y3*Y4 + X2*X3*X3*Y4*Y4 + 2 * X2*X3*X4*Y2*Y3 - 2 * X2*X3*X4*Y3*Y4 - 2 * X2*X4*X4*Y2*Y3 + X2*X4*X4*Y3*Y3 - X3*X3*X4*Y2*Y2 + X3*X4*X4*Y2*Y2) / tmp3;
	}
	point apply(float x, float y) {
		x -= x1;
		y -= y1;
		float rec = 1 / (g*x + h*y + 1.0);
		return{ (a*x + b*y)*rec, (d*x + e*y)*rec, 0, 0 };
	}
};

int inter(float x1, float y1, float x2, float y2, float x3, float y3, float x4, float y4) {
	float m1, b1;
	float m2, b2;
	if (x1 == x2) {
		if (x1 > MIN(x3, x4) && x1 < MAX(x3, x4)) {
			if (x3 == x4) {
				if (x1 != x3) {
					return 0;
				}
				if ((y1 > MIN(y3, y4) && y1 < MAX(y3, y4)) || (y2 > MIN(y3, y4) && y2 < MAX(y3, y4))) {
					return 1;
				}
				return 0;
			}
			else {
				m2 = (y3 - y4) / (x3 - x4);
				b2 = y3 - m2*x3;
				float y = x1*m2 + b2;
				if (y > MIN(y1, y2) && y < MAX(y1, y2)) {
					return 1;
				}
				else {
					return 0;
				}
			}
		}
		else {
			return 0;
		}
	}
	else {
		m1 = (y1 - y2) / (x1 - x2);
	}
	if (x3 == x4) {
		if (x3 > MIN(x1, x2) && x3 < MAX(x1, x2)) {
			if (x1 == x2) {
				if (x1 != x3) {
					return 0;
				}
				if ((y3 > MIN(y1, y2) && y3 < MAX(y1, y2)) || (y4 > MIN(y1, y2) && y4 < MAX(y1, y2))) {
					return 1;
				}
				return 0;
			}
			else {
				m1 = (y1 - y2) / (x1 - x2);
				b1 = y1 - m1*x1;
				float y = x3*m1 + b1;
				if (y > MIN(y3, y4) && y < MAX(y3, y4)) {
					return 1;
				}
				else {
					return 0;
				}
			}
		}
		else {
			return 0;
		}
	}
	else {
		m2 = (y3 - y4) / (x3 - x4);
	}
	if (m1 == m2) {
		return 0;
	}
	b1 = y1 - m1*x1;
	b2 = y3 - m2*x3;
	float x = (b1 - b2) / (m2 - m1);
	if (x > MIN(x1, x2) && x < MAX(x1, x2) && x > MIN(x3, x4) && x < MAX(x3, x4)) {
		return 1;
	}
	return 0;
}

float sq(float in) {
	return in*in;
}

Point2f rotate(float x, float y, int rot) {
	if (rot == 0) {
		return Point2f(x, y);
	}
	else if (rot == 1) {
		return Point2f(y, 1.0 - x);
	}
	else if (rot == 2) {
		return Point2f(1.0 - x, 1.0 - y);
	}
	else if (rot == 3) {
		return Point2f(1.0 - y, x);
	}
}

void findNabors(std::vector<point> P, std::vector<std::vector<int>>& N, int max) {
	N.resize(P.size());
	float* dist = (float*)malloc(sizeof(float)*max);
	int* id = (int*)malloc(sizeof(int)*max);
	int num, tmp2;
	float d, tmp;
	for (int n = 0; n < P.size(); n++) {
		for (int m = 0; m < max; m++) {
			dist[m] = 100000000;
		}
		memset(id, 0, sizeof(int)*max);
		num = 1;
		id[0] = n;
		for (int m = 0; m < P.size(); m++) {
			if (n != m) {
				d = sq(P[n].x - P[m].x) + sq(P[n].y - P[m].y);
				if (d < dist[num]) {
					dist[num] = d;
					id[num] = m;
					for (int o = num; o > 0; o--) {
						if (dist[o] < dist[o - 1]) {
							tmp = dist[o];
							dist[o] = dist[o - 1];
							dist[o - 1] = tmp;
							tmp2 = id[o];
							id[o] = id[o - 1];
							id[o - 1] = tmp2;
						}
						else
						{
							break;
						}
					}
				}
				if (num < max - 1) {
					num++;
				}
			}
		}
		N[n] = std::vector<int>(id, id + num + 1);
	}
	free(dist);
	free(id);
}

void removeDuplicates(std::vector<std::vector<Point2f>>& C, std::vector<float> E) {
	for (int n = 0; n < C.size(); n++) {
		for (int j = 0; j < 4; j++) {
			float x = C[n][j].x;
			float y = C[n][j].y;
			for (int m = n + 1; m < C.size(); m++) {
				for (int i = 0; i < 4; i++) {
					if (x == C[m][i].x && y == C[m][i].y) {
						if (E[n] > E[m]) {
							C.erase(C.begin() + n);
							E.erase(E.begin() + n);
							m = n;
							j = 0;
						}
						else
						{
							C.erase(C.begin() + m);
							E.erase(E.begin() + m);
							m--;
							j = 0;
						}
						break;
					}
				}
			}
		}
	}
}

/*
Finds markers with points inside specified by (in).
The error term specifies how close the marker dimensions
must be in order for it to count properly.
The function places the corners of these markers in
(corners).
*/
void findMarkers(std::vector<point> P, std::vector<std::vector<int>> N, std::vector<point> in, std::vector<std::vector<Point2f>>& corners, float error) {
	int i, n1, n2, n3, n4;
	float x1, x2, x3, x4;
	float y1, y2, y3, y4;
	float dx1, dx2, dx3, dx4;
	float dy1, dy2, dy3, dy4;
	float tmpx, tmpy;
	point p;
	Homology h;
	Point2f rotated;
	float *queue = (float*)malloc(sizeof(float) * 2 * in.size());
	int qnum = 0;
	int nnum;
	int n;
	int count1 = 0, count2 = 0;
	std::vector<float> E;
	float ee, dist2;
	for (i = 0; i < P.size(); i++) {
		nnum = N[i].size();
		for (n1 = 0; n1 < nnum; n1++) {
			x1 = P[N[i][n1]].x;
			y1 = P[N[i][n1]].y;
			for (n2 = n1 + 1; n2 < nnum; n2++) {
				for (n3 = n2 + 1; n3 < nnum; n3++) {
					for (n4 = n3 + 1; n4 < nnum; n4++) {
						x2 = P[N[i][n2]].x;
						y2 = P[N[i][n2]].y;
						x3 = P[N[i][n3]].x;
						y3 = P[N[i][n3]].y;
						x4 = P[N[i][n4]].x;
						y4 = P[N[i][n4]].y;
						if (inter(x1, y1, x3, y3, x4, y4, x2, y2) == 0) {
							if (inter(x1, y1, x4, y4, x3, y3, x2, y2) == 1) {
								tmpx = x4;
								tmpy = y4;
								x4 = x3;
								y4 = y3;
								x3 = tmpx;
								y3 = tmpy;
							}
							else if (inter(x1, y1, x2, y2, x4, y4, x3, y3) == 1) {
								tmpx = x3;
								tmpy = y3;
								x3 = x2;
								y3 = y2;
								x2 = tmpx;
								y2 = tmpy;
							}
							else
							{
								continue;
							}
						}
						dx1 = x2 - x1;
						dy1 = y2 - y1;
						dx2 = x3 - x2;
						dy2 = y3 - y2;
						if (dx1*dy2 - dx2*dy1 < 0) {
							tmpx = x2;
							tmpy = y2;
							x2 = x4;
							y2 = y4;
							x4 = tmpx;
							y4 = tmpy;
						}
						h.computeMatrix(x1, y1, x2, y2, x3, y3, x4, y4);
						qnum = 0;
						for (n = 0; n < P.size(); n++) {
							if (n != N[i][n1] && n != N[i][n2] && n != N[i][n3] && n != N[i][n4]) {
								p = h.apply(P[n].x, P[n].y);
								if (p.x > 0.0 && p.x < 1.0 && p.y > 0.0 && p.y < 1.0) {
									if (qnum >= in.size()) {
										qnum++;
										break;
									}
									queue[2 * qnum] = p.x;
									queue[2 * qnum + 1] = p.y;
									qnum++;
								}
							}
						}
						if (qnum == in.size()) {
							for (int rot = 0; rot < 4; rot++) {
								count1 = 0;
								ee = 0;
								for (n = 0; n < qnum; n++) {
									count2 = 0;
									rotated = rotate(queue[2 * n], queue[2 * n + 1], rot);
									for (int m = 0; m < qnum; m++) {
										dist2 = sq(rotated.x - in[m].x) + sq(rotated.y - in[m].y);
										if (dist2 < error) {
											ee += dist2;
											count2++;
										}
									}
									if (count2 != 1)
										break;
									count1++;
								}
								if (count1 == qnum) {
									std::vector<Point2f> out;
									E.push_back(ee);
									switch (rot) {
									case 0:
										out.push_back(Point2f(x1, y1));
										out.push_back(Point2f(x2, y2));
										out.push_back(Point2f(x3, y3));
										out.push_back(Point2f(x4, y4));
										break;
									case 1:
										out.push_back(Point2f(x4, y4));
										out.push_back(Point2f(x1, y1));
										out.push_back(Point2f(x2, y2));
										out.push_back(Point2f(x3, y3));
										break;
									case 2:
										out.push_back(Point2f(x3, y3));
										out.push_back(Point2f(x4, y4));
										out.push_back(Point2f(x1, y1));
										out.push_back(Point2f(x2, y2));
										break;
									case 3:
										out.push_back(Point2f(x2, y2));
										out.push_back(Point2f(x3, y3));
										out.push_back(Point2f(x4, y4));
										out.push_back(Point2f(x1, y1));
										break;
									}
									corners.push_back(out);
								}
							}
						}
					}
				}
			}
		}
	}
	removeDuplicates(corners, E);
	free(queue);
}