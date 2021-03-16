#include<stdio.h>
#include<math.h>

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

