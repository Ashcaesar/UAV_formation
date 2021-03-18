#ifndef MODEL_H_INCLUDED
#define MODEL_H_INCLUDED

typedef struct {
	double x;
	double y;
	double z;
}axis;

typedef struct {
	axis position;
	axis velocity;
	axis acceleration;
	double speed;
	double phi;
	double theta;
	int teamID;
	int leader;
}UAV;

double get_dis(axis, axis);
double f_dispersion();
double f_speedmatch();

#endif //
