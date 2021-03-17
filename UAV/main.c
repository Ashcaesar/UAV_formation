#include<stdio.h>
#include<math.h>

#include"kmeans.h"
#include"function.h"
#include"parameter.h"

extern UAV uav[SIZE];
extern axis sum;            /*临时存储数据*/
extern axis p_origin;       /*原点坐标*/
extern axis p_start;        /*出发点坐标*/
extern axis p_final;        /*终点坐标*/
extern int num_leader;      /*记录长机编号*/
extern int change;          /*判定kmeans分组是否完成*/

void main() {
	int i;
	double t;
	FILE *fp;
	FILE *fp2;
	FILE *fp3;
	if ((fp = fopen("coordinate.txt", "wb")) == NULL) {
		printf("离散度写入失败!\n");
		return;
	}
	if ((fp2 = fopen("index.txt", "wb")) == NULL) {
		printf("坐标写入失败!\n");
		return;
	}
	if ((fp3 = fopen("team.txt", "wb")) == NULL) {
		printf("分队写入失败!\n");
		return;
	}

	f_initial();
	for (t = 0; t < 300; t += delt) {
		if (f_stop()) break;
		fprintf(fp, "%f %f\n", f_dispersion(), f_speedmatch());
		for (i = 0; i < SIZE; i++) {
			fprintf(fp2, "%d %d %d\n", (int)uav[i].position.x, (int)uav[i].position.y, (int)uav[i].position.z);
			f_update(i);
		}
		f_crash();
	}
	fclose(fp);
	fclose(fp2);
	num_leader = f_leader();
	uav[num_leader].teamID = 0;
	printf("碰撞次数为%d,长机编号为%d", num_crash, num_leader);

	kmeans_initial();
	do {
		update_team();
		update_centroid();
	} while (change != 0);
	for (i = 0; i < SIZE; i++) {
		fprintf(fp3, "%d %d %d %d\n", (int)uav[i].position.x, (int)uav[i].position.y, (int)uav[i].position.z, uav[i].teamID);
	}
	fclose(fp3);

}
