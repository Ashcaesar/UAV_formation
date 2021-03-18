#include<stdio.h>
#include<math.h>
#include<time.h>
#include<stdlib.h>
#include<stdbool.h>

#include"parameter.h"
#include"assemble.h"
#include"assignment.h"


UAV uav[SIZE];
int team_change;
int leader_main;
axis p_final;
axis centroid[num_team + 1];

double get_min_dis(int id_uav, int num) {
	int i;
	double temp_dis, min_dis;
	for (i = 1; i < num; i++) {
		if (i == 1) min_dis = get_dis(uav[id_uav].position, centroid[i]);
		else {
			temp_dis = get_dis(uav[id_uav].position, centroid[i]);
			if (temp_dis < min_dis) min_dis = temp_dis;
		}
	}
	return min_dis;
}

void initial_centroid() {	
	int i, j, num;
	double sum = 0;
	double dis;
	double P[SIZE];
	double probability;
	double min_dis[SIZE] = { neighbor };
	
	srand((unsigned)time(NULL));
	num = rand() % SIZE;
	uav[num].teamID = 1;
	centroid[1] = uav[num].position;		
	//先随机选择第一个质心位置

	for (i = 2; i <= num_team; i++) {
		for (j = 0; j < SIZE; j++) {
			dis = get_min_dis(j, i);
			min_dis[j] = dis * dis;
			sum += min_dis[j];
		}
		probability = ((double)rand()/RAND_MAX);
		for (j = 0; j < SIZE; j++) {
			if (j == 0) P[j] = min_dis[j] / sum;
			else P[j] = min_dis[j] / sum + P[j - 1];
			if (P[j] >= probability && uav[j].teamID == -1) {
				uav[j].teamID = i;
				centroid[i] = uav[j].position;
				break;
			}
		}	
		for (j = 0; j < SIZE; j++) min_dis[j] = neighbor;
		sum = 0;
	}	
}

void update_team() {
	int i, j, tempID;
	double dis, min_dis;
	team_change = 0;
	for (i = 0; i < SIZE; i++) {
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
			team_change = 1;
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

int get_main_leader() {
	int i, leader;
	double dis, min_dis;
	for (i = 0; i < SIZE; i++) {
		if (i == 0) {
			min_dis = get_dis(uav[0].position, p_final);
			leader = 0;
		}
		else {
			dis = get_dis(uav[i].position, p_final);
			if (dis < min_dis) {
				min_dis = dis;
				leader = i;
			}
		}
	}
	uav[leader].teamID = 0;
	uav[leader].leader = -1;
	return leader;
}

void get_team_leader() {
	int i, j, leader, team_leader[num_team + 1];
	double dis, min_dis[num_team + 1];
	for (i = 1; i <= num_team; i++) {
		min_dis[i] = 1000;
		team_leader[i] = -1;
	}
	for (i = 0; i < SIZE && i != leader_main; i++) {
		for (j = 1; j <= num_team + 1; j++) {
			if (uav[i].teamID == j) {
				dis = get_dis(uav[i].position, uav[leader_main].position);
				if (dis < min_dis[j]) {
					min_dis[j] = dis;
					team_leader[j] = i;
				}
				break;
			}
		}
	}
	for (i = 1; i <= num_team; i++) {
		uav[team_leader[i]].leader = leader_main;
	}
	for (i = 0; i < SIZE; i++) {
		if (uav[i].leader == -1 || uav[i].leader == leader_main) continue;
		uav[i].leader = team_leader[uav[i].teamID];
	}
}