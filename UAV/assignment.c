#include<stdio.h>
#include<math.h>
#include<time.h>
#include<stdlib.h>
#include<stdbool.h>

#include"parameter.h"
#include"assemble.h"
#include"assignment.h"


UAV uav[SIZE];
int change;
int num_leader;
axis centroid[num_team + 1];

void initial_centroid() {	
	int i, j, k, num;
	double temp_dis;
	double sum = 0;
	double probability;
	double min_dis[SIZE] = { 10*D };
	double P[SIZE];
	
	srand((unsigned)time(NULL));
	num = rand() % SIZE;
	uav[num].teamID = 1;
	centroid[1] = uav[num].position;		
	
	for (i = 2; i <= num_team; i++) {
		for (j = 0; j < SIZE; j++) {
			for (k = 1; k < i; k++) {
				temp_dis = get_dis(uav[j].position, centroid[k]);
				if (temp_dis < min_dis[j]) min_dis[j] = temp_dis;
			}
			min_dis[j] = pow(min_dis[j], 2);
			sum += min_dis[j];
		}

		probability = rand();
		for (j = 0; j < SIZE; j++) {
			if (j == 0) P[j] = min_dis[j] / sum;
			else P[j] = min_dis[j] / sum + P[j - 1];

			if (P[j] >= probability && uav[j].teamID == -1) {
				uav[j].teamID = i;
				centroid[i] = uav[j].position;
				break;
			}
		}
		for (j = 0; j < SIZE; j++) min_dis[j] = 10 * D;
		sum = 0;
	}
}

void update_team() {
	int i, j, tempID;
	double dis, min_dis;
	change = 0;
	for (i = 0; i < SIZE; i++) {
		if (i == num_leader) continue;
		tempID = 1;
		min_dis = get_dis(uav[i].position, centroid[1]);
		for (j = 2; j <= num_team; j++) {
			dis = get_dis(uav[i].position, centroid[j]);
			if (dis < min_dis) {
				tempID = j;
				min_dis = dis;				
			}
		}
		if (uav[i].teamID == tempID) continue;
		else { 
			change = 1;
			uav[i].teamID = tempID;
		}
	}
}

void update_centroid() {
	int i, j;
	int num[num_team + 1];
	for (i = 1; i <= num_team; i++) {
		centroid[i].x = 0;
		centroid[i].y = 0;
		centroid[i].z = 0;
		num[i] = 0;
	}
	for (i = 0; i < SIZE; i++) {
		for (j = 1; j <= num_team; j++) {
			if (uav[i].teamID == j) {
				centroid[j].x += uav[i].position.x;
				centroid[j].y += uav[i].position.y;
				centroid[j].z += uav[i].position.z;
				num[j]++;
			}
		}
	}
	for (i = 1; i <= num_team; i++) {
		centroid[i].x /= num[i];
		centroid[i].y /= num[i];
		centroid[i].z /= num[i];
	}
}


