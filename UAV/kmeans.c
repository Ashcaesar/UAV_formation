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
		if (uav[num].ID == -1) {
			uav[num].ID = i;
			centroid[i] = uav[num].position;
		}
	}
}

void kmeans_update() {

}
