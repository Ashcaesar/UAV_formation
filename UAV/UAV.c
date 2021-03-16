#include<stdio.h>
#include<stdlib.h>
#include<math.h>
#include<time.h>
#include"function.h"
#include"parameter.h"

UAV uav[SIZE];

void main() {
	int i;
	double t;
	FILE *fp;
	FILE *fp2;
	if ((fp = fopen("coordinate.txt", "wb")) == NULL) {
		printf("坐标写入失败!\n");
		return;
	}
	if ((fp2 = fopen("index.txt", "wb")) == NULL) {
		printf("离散度写入失败!\n");
		return;
	}
	f_initial();

	for (t = 0; t < 300; t += delt) {
		if (f_stop()) break;
		fprintf(fp2, "%f %f\n", f_dispersion(), f_speedmatch());
		for (i = 0; i < SIZE; i++) {
			fprintf(fp, "%d %d %d\n", (int)uav[i].position.x, (int)uav[i].position.y, (int)uav[i].position.z);
			f_update(i);
		}
		f_crash();
	}
	fclose(fp);
	fclose(fp2);
	
	printf("碰撞次数为%d,长机编号为%d", num_crash, f_leader());

}

double f_metric(axis A, axis B) {
	double dis;
	dis = pow(A.x - B.x, 2) + pow(A.y - B.y, 2) + pow(A.z - B.z, 2);
	dis = sqrt(dis);
	return dis;
}

void f_initial() {
	int i;
	srand((unsigned)time(NULL));
	do {
		num_crash = 0;
		for (i = 0; i < SIZE; i++) {
			uav[i].position.x = rand() % 150;
			uav[i].position.y = rand() % 150;
			uav[i].position.z = rand() % 10 - 5;
		}
		f_crash();
	} while (num_crash > 0);

	for (i = 0; i < SIZE; i++) {
		uav[i].speed = rand() % 10;
		uav[i].phi = (rand() % 360)*PI / 180;
		/*uav[i].phi = (rand() % 90 - 45)*PI / 180;*/
		uav[i].theta = (rand() % 90 - 45)*PI / 180;
		uav[i].velocity.x = uav[i].speed*cos(uav[i].phi)*cos(uav[i].theta);
		uav[i].velocity.y = uav[i].speed*cos(uav[i].phi)*sin(uav[i].theta);
		uav[i].velocity.z = uav[i].speed*sin(uav[i].phi);
		uav[i].acceleration.x = 0;
		uav[i].acceleration.y = 0;
		uav[i].acceleration.z = 0;
	}
}

axis f_match(int i) {
	int j, num;
	double dis, aij;
	sum.x = 0; sum.y = 0; sum.z = 0;
	num = 0;
	for (j = 0; j < SIZE; j++) {
		if (j != i) {
			dis = f_metric(uav[i].position, uav[j].position);
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
	for (j = 0; j < SIZE; j++) {
		if (j != i) {
			dis = f_metric(uav[i].position, uav[j].position);
			if ((dis > d) && (dis <= D)) {
				f = k1 * (pow(dis - d, -theta1) - pow(D - d, -theta1));
			}
			else if ((dis > D) && (dis <= neighbor)) {
				f = k2 * (pow(dis - d, -theta2) - pow(D - d, -theta2));
			}
			else if (dis > neighbor) {
				f = k3 * (pow(dis, -theta3));
			};
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
	dis = f_metric(p_start, uav[i].position);
	g = k4 * pow(dis, -alpha);
	sum.x += g * (p_start.x - uav[i].position.x) / (dis + 1e-6);
	sum.y += g * (p_start.y - uav[i].position.y) / (dis + 1e-6);
	sum.z += g * (p_start.z - uav[i].position.z) / (dis + 1e-6);
	axis result = { sum.x,sum.y,sum.z };
	return result;
}

void f_update(int i) {
	tMatch = f_match(i);
	tAttract = f_attract(i);
	tTarget = f_target(i);

	uav[i].acceleration.x = tMatch.x + tAttract.x + tTarget.x;
	uav[i].acceleration.y = tMatch.y + tAttract.y + tTarget.y;
	uav[i].acceleration.z = tMatch.z + tAttract.z + tTarget.z;
	f_sat(uav[i].acceleration.x, Min_acceleration, Max_acceleration);
	f_sat(uav[i].acceleration.y, Min_acceleration, Max_acceleration);
	f_sat(uav[i].acceleration.z, Min_acceleration, Max_acceleration);

	uav[i].velocity.x += uav[i].acceleration.x*delt;
	uav[i].velocity.y += uav[i].acceleration.y*delt;
	uav[i].velocity.z += uav[i].acceleration.z*delt;
	f_sat(uav[i].velocity.x, Min_velocity, Max_velocity);
	f_sat(uav[i].velocity.y, Min_velocity, Max_velocity);
	f_sat(uav[i].velocity.z, Min_velocity, Max_velocity);

	uav[i].position.x += uav[i].velocity.x*delt;
	uav[i].position.y += uav[i].velocity.y*delt;
	uav[i].position.z += uav[i].velocity.z*delt;

	uav[i].speed = f_metric(uav[i].velocity, p_origin);
	uav[i].phi = atan2(uav[i].velocity.z, sqrt(pow(uav[i].velocity.x, 2) + pow(uav[i].velocity.y, 2)));
	uav[i].theta = atan2(uav[i].velocity.y, uav[i].velocity.x);
}

int f_stop() {
	int i;
	double range = 0, V = 0, angle_to_target = 0;
	axis Xcg = { 0,0,0 };
	axis Vcg = { 0,0,0 };
	axis dis = { 0,0,0 };
	for (i = 0; i < SIZE; i++) {
		Xcg.x += uav[i].position.x;
		Xcg.y += uav[i].position.y;
		Xcg.z += uav[i].position.z;
		Vcg.x += uav[i].velocity.x;
		Vcg.y += uav[i].velocity.y;
		Vcg.z += uav[i].velocity.z;
	}
	Xcg.x /= SIZE;
	Xcg.y /= SIZE;
	Xcg.z /= SIZE;
	Vcg.x /= SIZE;
	Vcg.y /= SIZE;
	Vcg.z /= SIZE;
	range = f_metric(p_start, Xcg);
	V = f_metric(Vcg, p_origin);
	angle_to_target = fabs(acos((dis.x*Vcg.x + dis.y*Vcg.y + dis.z*Vcg.z) / (range * V)));
	if (range <= 50 && angle_to_target <= 10) return 1;
	else return 0;
}

double f_sat(double x, double min, double max) {
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

void f_crash() {
	int i, j;
	double dis;
	for (i = 0; i < SIZE; i++) {
		for (j = 0; j < SIZE; j++) {
			if (i != j) {
				dis = f_metric(uav[i].position, uav[j].position);
				if (dis < d) num_crash++;
			}
		}
	}
}

double f_dispersion() {
	int i, j;
	double dis, result;
	result = 0;
	for (i = 0; i < SIZE; i++) {
		for (j = 0; j < SIZE; j++) {
			if (i != j) {
				dis = f_metric(uav[i].position, uav[j].position);
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
				dis = f_metric(uav[i].velocity, uav[j].velocity);
				result += dis * dis;
			}
		}
	}
	result = sqrt(result / (SIZE*(SIZE - 1)));
	return result;
}

int f_leader() {
	int i, num = 0;
	double dis, temp;
	dis = f_metric(uav[0].position, p_final);
	for (i = 1; i < SIZE; i++) {
		temp = f_metric(uav[i].position, p_final);
		if (temp < dis) {
			dis = temp;
			num = i;
		}
	}
	return num;
}

f_kmeans() {
	int i;

	for (i = 0; i < SIZE; i++) {

	}
}