#include<stdio.h>
#include<math.h>
#include<time.h>

#include"kmeans.h"
#include"function.h"
#include"parameter.h"

UAV uav[SIZE];
int num_leader;
axis centroid[num_team + 1];

void kmeans_initial() {
	srand((unsigned)time(NULL));
	int i, num;
	for (i = 1; i <= num_team; i++) {
		num = rand() % SIZE;
		if (uav[num].teamID == -1) {
			uav[num].teamID = i;
			centroid[i] = uav[num].position;
		}
	}
}

double kmeans_dis(axis A, axis B) {
	double dis;
	dis = pow(A.x - B.x, 2) + pow(A.y - B.y, 2) + pow(A.z - B.z, 2);
	dis = sqrt(dis);
	return dis;
}

void update_team() {
	int i, j, tempID;
	double dis, min_dis;
	for (i = 0; i < SIZE; i++) {
		tempID = 1;
		min_dis = kmeans_dis(uav[i].position, centroid[1]);
		for (j = 2; j <= num_team; j++) {
			dis = kmeans_dis(uav[i].position, centroid[j]);
			if (dis < min_dis) {
				tempID = j;
				min_dis = dis;				
			}
		}
		uav[i].teamID = tempID;
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


