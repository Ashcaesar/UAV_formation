#include<stdio.h>
#include<math.h>

#include"model.h"

double get_dis(axis A, axis B) {
	double dis;
	dis = pow(A.x - B.x, 2) + pow(A.y - B.y, 2) + pow(A.z - B.z, 2);
	dis = sqrt(dis);
	return dis;
}