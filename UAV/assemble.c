#include<stdio.h>
#include<math.h>
#include<time.h>
#include<stdlib.h>

#include"assemble.h"
#include"parameter.h"

UAV uav[SIZE+1];
axis sum = { 0,0,0 };
axis p_origin = { 0,0,0 };
axis p_start = { 1000,1000,50 };
axis p_final = { 2000,2000,100 };

void initial_uav() {
	int i;
	srand((unsigned)time(NULL));
	do {
		num_crash = 0;
		for (i = 1; i <= SIZE; i++) {
			uav[i].position.x = rand() % 100;
			uav[i].position.y = rand() % 100;
			uav[i].position.z = rand() % 30 - 15;
		}
		crash();
	} while (num_crash > 0);

	for (i = 1; i <= SIZE; i++) {
		uav[i].velocity.x = rand() % 5;
		uav[i].velocity.y = rand() % 5;
		uav[i].velocity.z = rand() % 3;
		uav[i].acceleration.x = 0;
		uav[i].acceleration.y = 0;
		uav[i].acceleration.z = 0;
		uav[i].phi = atan2(uav[i].velocity.z, sqrt(pow(uav[i].velocity.x, 2) + pow(uav[i].velocity.y, 2)));
		uav[i].theta = atan2(uav[i].velocity.y, uav[i].velocity.x);
		uav[i].teamID = -1;
		uav[i].leader = -2;
	}
}

axis f_match(int i) {
	int j, num;
	double dis, aij;
	sum.x = 0; sum.y = 0; sum.z = 0;
	num = 0;
	for (j = 1; j <= SIZE; j++) {
		if (j != i) {
			dis = get_dis(uav[i].position, uav[j].position);
			if (dis < neighbor) {
				num++;
				aij = H / pow(1 + pow(dis, 2), b);
				sum.x += aij * (uav[j].velocity.x - uav[i].velocity.x);
				sum.y += aij * (uav[j].velocity.y - uav[i].velocity.y);
				sum.z += aij * (uav[j].velocity.z - uav[i].velocity.z);
			}
		}
	}
	sum.x /= num;
	sum.y /= num;
	sum.z /= num;
	axis result = { sum.x,sum.y,sum.z };
	return result;
}

axis f_attract(int i) {
	int j;
	double f = 0, dis;
	sum.x = 0; sum.y = 0; sum.z = 0;
	for (j = 1; j <= SIZE; j++) {	
		if (j != i) {
			dis = get_dis(uav[i].position, uav[j].position);
			if ((dis > d) && (dis <= D)) {
				f = k1 * (pow(dis - d, -theta1) - pow(D - d, -theta1));
			}
			else if ((dis > D) && (dis <= neighbor)) {
				f = k2 * (pow(dis - d, -theta2) - pow(D - d, -theta2));
			}
			else if (dis > neighbor) {
				f = k3 * (pow(dis, -theta3));
			}
			sum.x += f * (uav[i].position.x - uav[j].position.x) / (dis + 1e-6);
			sum.y += f * (uav[i].position.y - uav[j].position.y) / (dis + 1e-6);
			sum.z += f * (uav[i].position.z - uav[j].position.z) / (dis + 1e-6);
		}
	}
	axis result = { sum.x,sum.y,sum.z };
	return result;
}

axis f_target(int i) {
	double dis, g;
	sum.x = 0; sum.y = 0; sum.z = 0;
	dis = get_dis(p_start, uav[i].position);
	g = k4 * pow(dis, alpha);
	sum.x += g * (p_start.x - uav[i].position.x) / (dis + 1e-6);
	sum.y += g * (p_start.y - uav[i].position.y) / (dis + 1e-6);
	sum.z += g * (p_start.z - uav[i].position.z) / (dis + 1e-6);
	axis result = { sum.x,sum.y,sum.z };
	return result;
}

void update_assemble(int i) {
	tMatch = f_match(i);
	tAttract = f_attract(i);
	tTarget = f_target(i);

	uav[i].acceleration.x = tMatch.x + tAttract.x + tTarget.x;
	uav[i].acceleration.y = tMatch.y + tAttract.y + tTarget.y;
	uav[i].acceleration.z = tMatch.z + tAttract.z + tTarget.z;
	/*uav[i].acceleration.x = limit_uav(uav[i].acceleration.x, Min_ver_acceleration, Max_ver_acceleration);
	uav[i].acceleration.y = limit_uav(uav[i].acceleration.y, Min_ver_acceleration, Max_ver_acceleration);
	uav[i].acceleration.z = limit_uav(uav[i].acceleration.z, Min_hor_acceleration, Max_hor_acceleration);*/

	uav[i].velocity.x += uav[i].acceleration.x*delt;
	uav[i].velocity.y += uav[i].acceleration.y*delt;
	uav[i].velocity.z += uav[i].acceleration.z*delt;
	/*uav[i].velocity.x = limit_uav(uav[i].velocity.x, Min_ver_velocity, Max_ver_velocity);
	uav[i].velocity.y = limit_uav(uav[i].velocity.y, Min_ver_velocity, Max_ver_velocity);
	uav[i].velocity.z = limit_uav(uav[i].velocity.z, Min_hor_velocity, Max_hor_velocity);*/

	uav[i].position.x += uav[i].velocity.x*delt;
	uav[i].position.y += uav[i].velocity.y*delt;
	uav[i].position.z += uav[i].velocity.z*delt;

	uav[i].phi = atan2(uav[i].velocity.z, sqrt(pow(uav[i].velocity.x, 2) + pow(uav[i].velocity.y, 2)));
	uav[i].theta = atan2(uav[i].velocity.y, uav[i].velocity.x);
}

double limit_uav(double x, double min, double max) {
	double result = x;
	if (fabs(x) < min) {
		if (x > 0) result = min;
		else result = -min;
	}
	else  if (fabs(x) > max) {
		if (x > 0) result = max;
		else result = -max;
	}
	return result;
}

void crash() {
	int i, j;
	double dis;
	for (i = 1; i <= SIZE; i++) {
		for (j = 1; j <= SIZE; j++) {
			if (i == j) continue;
			dis = get_dis(uav[i].position, uav[j].position);
			if (dis < d) num_crash++;
		}
	}
}