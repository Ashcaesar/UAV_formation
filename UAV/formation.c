#include<stdio.h>
#include<math.h>

#include"model.h"
#include"parameter.h"

UAV uav[SIZE + 1];
UAV form[SIZE + 1];
axis p_assemble;
axis p_final;
axis sum;
axis Xcg;
axis t_repel;
int A[SIZE + 1][SIZE + 1] = { 0 };
int matrix = 5;
axis R[SIZE + 1] = { {0,0,0},{-2 * M,2 * M,0},{-M,2 * M,0},{0,2 * M,0},{M,2 * M,0},{2 * M,2 * M,0},\
					 {-2 * M, M,0},{-M,M,0},{0,M,0},{ M,M,0},{ 2 * M,M,0}, \
					 {-2 * M,0,0},{-M,0,0},{0,0,0},{M,0,0},{ 2 * M,0,0}, \
					 {-2 * M,-M,0},{-M,-M,0},{0,-M,0},{M,-M,0},{ 2 * M,-M,0}, \
					 {-2 * M,-2 * M,0},{-M,-2 * M,0},{0,-2 * M,0},{ M,-2 * M,0},{ 2 * M,-2 * M,0} };

void initial_form() {
	int i, j, id = SIZE + 1;
	double dis, min_dis;
	axis pos[SIZE + 1];
	get_Xcg();
	uav[0].position.x = Xcg.x;
	uav[0].position.y = Xcg.y;
	uav[0].position.z = Xcg.z;
	uav[0].phi = atan2(0, 1);
	uav[0].theta = atan2(p_final.y - uav[0].position.y, p_final.x - uav[0].position.x);
	uav[0].velocity.x = Max_ver_velocity * cos(uav[0].theta);
	uav[0].velocity.y = Max_ver_velocity * sin(uav[0].theta);
	uav[0].velocity.z = 0;
	uav[0].acceleration.x = 0;
	uav[0].acceleration.y = 0;
	uav[0].acceleration.z = 0;
	
	for (i = 0; i <= SIZE; i++) {
		if (i == 0) continue;
		A[i][0] = 1;
		if (i > matrix) A[i][i - matrix] = 1;
		if (i%matrix ==1 || i % matrix == 2) A[i][i + 1] = 1;
		else if (i%matrix != 3) A[i][i - 1] = 1;
	}
	for (i = 1; i <= SIZE; i++) {
		pos[i].x = uav[0].position.x + R[i].x*sin(uav[0].theta) + R[i].y*cos(uav[0].theta);
		pos[i].y = uav[0].position.y - R[i].x*cos(uav[0].theta) + R[i].y*sin(uav[0].theta);
		pos[i].z = uav[0].position.z + R[i].z;
	}
	for(i=1;i<=SIZE;i++){
		min_dis = 10000;
		id = SIZE + 1;
		for (j = 1; j <= SIZE; j++) {
			dis = get_dis(pos[i], uav[j].position);
			//dis = sqrt(pow(pos[i].x - uav[j].position.x, 2) + pow(pos[i].y - uav[j].position.y, 2));
			if (dis < min_dis && uav[j].flag == 0) {
				id = j;
				min_dis = dis;
			}

		}
		form[i] = uav[id];
		uav[id].flag = 1;
	}
	for (i = 1; i <= SIZE; i++)	uav[i] = form[i];
}

axis f_repel(int i) {
	int j;
	double f = 0, dis;
	sum.x = 0; sum.y = 0; sum.z = 0;
	for (j = 1; j <= SIZE; j++) {
		if (j != i) {
			dis = get_dis(uav[i].position, uav[j].position);
			if (dis <= 15) {
				f = 200 * (pow(dis - d, -theta1) - pow(25 - d, -theta1));
			}
			else {
				f = 40 * (pow(dis - d, -theta2) - pow(25 - d, -theta2));
			}
			sum.x += f * (uav[i].position.x - uav[j].position.x) / (dis + 1e-6);
			sum.y += f * (uav[i].position.y - uav[j].position.y) / (dis + 1e-6);
			sum.z += f * (uav[i].position.z - uav[j].position.z) / (dis + 1e-6);
		}
	}
	axis result = { sum.x,sum.y,sum.z };
	return result;
}

void update_form() {
	int i, j;
	double angle;
	axis newR[SIZE + 1] = { 0 };
	axis f_temp;
	angle = uav[0].theta;
	for (i = 0; i <= SIZE; i++) {
		newR[i].x =  R[i].x*sin(angle) + R[i].y*cos(angle);
		newR[i].y =  - R[i].x*cos(angle) + R[i].y*sin(angle);
		newR[i].z = uav[0].position.z + R[i].z;
	}
	for (i = 1; i <= SIZE; i++) {
		f_temp.x = 0;
		f_temp.y = 0;
		f_temp.z = 0;
		t_repel = f_repel(i);
		for (j = 0; j <= SIZE; j++) {
			f_temp.x += 0.9*A[i][j] * (((uav[i].position.x - newR[i].x) - (uav[j].position.x - newR[j].x)) + (uav[i].velocity.x - uav[j].velocity.x));
			f_temp.y += 0.9*A[i][j] * (((uav[i].position.y - newR[i].y) - (uav[j].position.y - newR[j].y)) + (uav[i].velocity.y - uav[j].velocity.y));
			f_temp.z += 0.9*A[i][j] * (((uav[i].position.z - newR[i].z) - (uav[j].position.z - newR[j].z)) + (uav[i].velocity.z - uav[j].velocity.z));
		}
		uav[i].acceleration.x = uav[0].acceleration.x - f_temp.x + t_repel.x;
		uav[i].acceleration.y = uav[0].acceleration.y - f_temp.y + t_repel.y;
		uav[i].acceleration.z = uav[0].acceleration.z - f_temp.z + t_repel.z;
	}
	for (i = 0; i <= SIZE; i++) {
		uav[i].velocity.x += uav[i].acceleration.x*delt;
		uav[i].velocity.y += uav[i].acceleration.y*delt;
		uav[i].velocity.z += uav[i].acceleration.z*delt;
		uav[i].position.x += uav[i].velocity.x*delt;
		uav[i].position.y += uav[i].velocity.y*delt;
		uav[i].position.z += uav[i].velocity.z*delt;
	}
}