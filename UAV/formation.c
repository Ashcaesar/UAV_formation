#include<stdio.h>
#include<math.h>

#include"model.h"
#include"parameter.h"

UAV uav[SIZE + 1];
UAV form[SIZE + 1];
axis p_assemble;
axis p_final;
axis p_origin;
axis sum;
axis Xcg;
axis t_repulsion;
int A[SIZE + 1][SIZE + 1] = { 0 };
int matrix = 5;
double Rr[3][3] = { 0 }, temp[3] = { 0 };
double R[SIZE + 1][3] = { {0,0,0},{-2 * M,2 * M,0},{-M,2 * M,0},{0,2 * M,0},{M,2 * M,0},{2 * M,2 * M,0},\
					 {-2 * M, M,0},{-M,M,0},{0,M,0},{ M,M,0},{ 2 * M,M,0}, \
					 {-2 * M,0,0},{-M,0,0},{0,0,0},{M,0,0},{ 2 * M,0,0}, \
					 {-2 * M,-M,0},{-M,-M,0},{0,-M,0},{M,-M,0},{ 2 * M,-M,0}, \
					 {-2 * M, -2 * M, 0}, { -M,-2 * M,0 }, { 0,-2 * M,0 }, { M,-2 * M,0 }, { 2 * M,-2 * M,0 } };
axis newR[SIZE + 1];

void initial_form() {
	int i, j, k, id = SIZE + 1;
	double dis, min_dis;
	double phi, theta, speed = 0;
	axis pos[SIZE + 1];
	get_Xcg();
	/*----------------------------------------------------------------------------------------------------------------------------------*/
	/*初始化虚拟长机参数*/
	uav[0].position.x = Xcg.x;
	uav[0].position.y = Xcg.y;
	uav[0].position.z = Xcg.z;
	uav[0].phi = atan2(p_final.z - uav[0].position.z, sqrt(pow(p_final.x - uav[0].position.z, 2) + pow(p_final.y - uav[0].position.y, 2)));
	uav[0].theta = atan2(p_final.y - uav[0].position.y, p_final.x - uav[0].position.x);
	for (i = 1; i <= SIZE; i++) {
		speed += get_dis(uav[i].velocity, p_origin);
	}
	speed /= SIZE;
	uav[0].velocity.x = speed * cos(uav[0].phi)* cos(uav[0].theta);
	uav[0].velocity.y = speed * cos(uav[0].phi)* sin(uav[0].theta);
	uav[0].velocity.z = speed * sin(uav[0].phi);
	uav[0].acceleration.x = 0;
	uav[0].acceleration.y = 0;
	uav[0].acceleration.z = 0;
	/*----------------------------------------------------------------------------------------------------------------------------------*/
	/*初始化僚机位置*/
	phi = uav[0].phi;
	theta = uav[0].theta - PI / 2;
	double Rx[3][3] = { {1, 0, 0}, {0, cos(phi), -sin(phi)}, {0, sin(phi), cos(phi)} };
	double Rz[3][3] = { {cos(theta), -sin(theta), 0 }, { sin(theta), cos(theta), 0 }, { 0, 0, 1 } };
	for (i = 0; i <= SIZE; i++) {
		if (i == 0) continue;
		A[i][0] = 1;
		if (i%matrix == 1 || i % matrix == 2) A[i][i + 1] = 1;
		else if (i%matrix == 0 || i % matrix == 4) A[i][i - 1] = 1;
		else {
			A[i][i - 1] = 1;
			A[i][i + 1] = 1;
		}
		if (i <= 2 * matrix) A[i][i + matrix] = 1;
		else if (i >= 3 * matrix) A[i][i - matrix] = 1;
		else {
			A[i][i + matrix] = 1;
			A[i][i - matrix] = 1;
		}
	}
	for (i = 0; i < 3; i++) {
		for (j = 0; j < 3; j++) {
			for (k = 0; k < 3; k++) Rr[i][j] += Rz[i][k] * Rx[k][j];
		}
	}
	for (i = 1; i <= SIZE; i++) {
		temp[0] = 0;
		temp[1] = 0;
		temp[2] = 0;
		for (j = 0; j < 3; j++) {
			for (k = 0; k < 3; k++) temp[j] += Rr[j][k] * R[i][k];
		}
		newR[i].x = temp[0];
		newR[i].y = temp[1];
		newR[i].z = temp[2];
		/*printf(" %d %f %f %f\n", i, newR[i].x, newR[i].y, newR[i].z);*/
		pos[i].x = uav[0].position.x - newR[i].x;
		pos[i].y = uav[0].position.y - newR[i].y;
		pos[i].z = uav[0].position.z - newR[i].z;
	}
	/*----------------------------------------------------------------------------------------------------------------------------------*/
	/*分配对应僚机*/
	for(i=1;i<=SIZE;i++){
		min_dis = 10000;
		id = SIZE + 1;
		for (j = 1; j <= SIZE; j++) {
			dis = get_dis(pos[i], uav[j].position);
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

axis f_repulsion(int i) {
	int j;
	double f = 0, dd = 20, dis;
	sum.x = 0; sum.y = 0; sum.z = 0;
	for (j = 1; j <= SIZE; j++) {
		if (j != i) {
			dis = get_dis(uav[i].position, uav[j].position);
			if (dis <= dd) {
				f = 4 * (dd - dis);
			}
			else if(dis<=2*M){
				f = 0.04*(dd - dis);
			}
			else f = 0.04*(dd - 2 * M);
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
	double phi, theta;
	axis f_temp;
	phi = uav[0].phi;
	theta = uav[0].theta;
	
	for (i = 1; i <= SIZE; i++) {
		f_temp.x = 0;
		f_temp.y = 0;
		f_temp.z = 0;
		t_repulsion = f_repulsion(i);
		
		for (j = 0; j <= SIZE; j++) {
			f_temp.x += A[i][j] * (((newR[i].x - uav[i].position.x) - (newR[j].x - uav[j].position.x)) + 2 * (uav[j].velocity.x - uav[i].velocity.x));
			f_temp.y += A[i][j] * (((newR[i].y - uav[i].position.y) - (newR[j].y - uav[j].position.y)) + 2 * (uav[j].velocity.y - uav[i].velocity.y));
			f_temp.z += A[i][j] * (((newR[i].z - uav[i].position.z) - (newR[j].z - uav[j].position.z)) + 2 * (uav[j].velocity.z - uav[i].velocity.z));
		}
		uav[i].acceleration.x = uav[0].acceleration.x + f_temp.x + t_repulsion.x;
		uav[i].acceleration.y = uav[0].acceleration.y + f_temp.y + t_repulsion.y;
		uav[i].acceleration.z = uav[0].acceleration.z + f_temp.z + t_repulsion.z;
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