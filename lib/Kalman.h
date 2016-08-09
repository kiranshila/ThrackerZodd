#pragma once

struct kState {
	float q, r, p, k, x;
	//float 
	bool first;
	kState() {
		first = true;
	}
};

void kSet(kState& out, float q, float r, float p) {
	out.q = q;
	out.r = r;
	out.p = p;
}

void kUpdate(kState& state, double measurement)	{
	if (state.first) {
		state.x = measurement;
		state.first = false;
	}
	state.p = state.p + pow(abs(measurement - state.x), state.q);
	state.k = state.p / (state.p + state.r);
	state.x = state.x + state.k * (measurement - state.x);
	state.p = (1.0 - state.k) * state.p;
}

struct vecFilter {
	kState x_, y_, z_;
	float x, y, z;
	vecFilter(float q, float r) {
		x_ = kState();
		kSet(x_, q, r, 0);
		y_ = kState();
		kSet(y_, q, r, 0);
		z_ = kState();
		kSet(z_, q, r, 0);
	}
	void update(float ix, float iy, float iz) {
		kUpdate(x_, ix);
		x = x_.x;
		kUpdate(y_, iy);
		y = y_.x;
		kUpdate(z_, iz);
		z = z_.x;
	}
	std::string print() {
		std::string out = "";
		out += to_string(x);
		out += ", ";
		out += to_string(y);
		out += ", ";
		out += to_string(z);
		return out;
	}
};