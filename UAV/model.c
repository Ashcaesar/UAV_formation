#include<stdio.h>
#include<math.h>

#include"model.h"
#include"parameter.h"

UAV uav[SIZE+1];
axis p_origin;
axis p_start;
axis p_final;
axis Xcg;

double get_dis(axis A, axis B) {
	double dis;
	dis = pow(A.x - B.x, 2) + pow(A.y - B.y, 2) + pow(A.z - B.z, 2);
	dis = sqrt(dis);
	return dis;
}

void get_Xcg() {
	int i;
	Xcg.x = 0;
	Xcg.y = 0;
	Xcg.z = 0;
	for (i = 1; i <= SIZE; i++) {
		Xcg.x += uav[i].position.x;
		Xcg.y += uav[i].position.y;
		Xcg.z += uav[i].position.z;
	}
	Xcg.x /= SIZE;
	Xcg.y /= SIZE;
	Xcg.z /= SIZE;
}

double dis_to_target(axis point) {
	double dis_target = get_dis(point, Xcg);
	return dis_target;
}

double angle_to_target(axis point) {
	int i;
	double distance, V = 0, angle = 0;
	axis Vcg = { 0,0,0 };
	axis dis = { 0,0,0 };
	for (i = 1; i <= SIZE; i++) {
		Vcg.x += uav[i].velocity.x;
		Vcg.y += uav[i].velocity.y;
		Vcg.z += uav[i].velocity.z;
	}
	dis.x = point.x - Xcg.x;
	dis.y = point.y - Xcg.y;
	dis.z = point.z - Xcg.z;
	Vcg.x /= SIZE;
	Vcg.y /= SIZE;
	Vcg.z /= SIZE;
	distance = dis_to_target(point);
	V = get_dis(Vcg, p_origin);
	angle = fabs(acos((dis.x*Vcg.x + dis.y*Vcg.y + dis.z*Vcg.z) / (distance * V)));
	angle = angle * 180 / PI;
	return angle;
}

double f_dispersion() {
	int i, j;
	double dis, result;
	result = 0;
	for (i = 1; i <= SIZE; i++) {
		for (j = 1; j <= SIZE; j++) {
			if (i != j) {
				dis = get_dis(uav[i].position, uav[j].position);
				result += dis * dis;
			}
		}
	}
	result = sqrt(result) / SIZE;
	return result;
}

double f_speedmatch() {
	int i, j;
	double dis, result;
	result = 0;
	for (i = 1; i <= SIZE; i++) {
		for (j = 1; j <= SIZE; j++) {
			if (i != j) {
				dis = get_dis(uav[i].velocity, uav[j].velocity);
				result += dis * dis;
			}
		}
	}
	result = sqrt(result) / SIZE;
	return result;
}