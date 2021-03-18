#include<stdio.h>
#include<math.h>

#include"model.h"
#include"parameter.h"

UAV uav[SIZE];

double get_dis(axis A, axis B) {
	double dis;
	dis = pow(A.x - B.x, 2) + pow(A.y - B.y, 2) + pow(A.z - B.z, 2);
	dis = sqrt(dis);
	return dis;
}

double f_dispersion() {
	int i, j;
	double dis, result;
	result = 0;
	for (i = 0; i < SIZE; i++) {
		for (j = 0; j < SIZE; j++) {
			if (i != j) {
				dis = get_dis(uav[i].position, uav[j].position);
				result += dis * dis;
			}
		}
	}
	result = sqrt(result / (SIZE*(SIZE - 1)));
	return result;
}

double f_speedmatch() {
	int i, j;
	double dis, result;
	result = 0;
	for (i = 0; i < SIZE; i++) {
		for (j = 0; j < SIZE; j++) {
			if (i != j) {
				dis = get_dis(uav[i].velocity, uav[j].velocity);
				result += dis * dis;
			}
		}
	}
	result = sqrt(result / (SIZE*(SIZE - 1)));
	return result;
}