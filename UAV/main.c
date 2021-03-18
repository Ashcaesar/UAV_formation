#include<stdio.h>
#include<math.h>

#include"model.h"
#include"assemble.h"
#include"assignment.h"
#include"parameter.h"

extern UAV uav[SIZE];
extern axis sum;            /*临时存储数据*/
extern axis p_origin;       /*原点坐标*/
extern axis p_start;        /*出发点坐标*/
extern axis p_final;        /*终点坐标*/
extern axis Xcg;
extern int leader_main;     /*记录长机编号*/
extern int change;          /*判定kmeans分组是否完成*/

void main() {
	int i;
	double t, dis, angle, dispersion, speedmatch;
	FILE *fp;
	FILE *fp2;
	FILE *fp3;
	if ((fp = fopen("data/index.txt", "wb")) == NULL) {
		printf("离散度写入失败!\n");
		return;
	}
	if ((fp2 = fopen("data/coordinate.txt", "wb")) == NULL) {
		printf("坐标写入失败!\n");
		return;
	}
	if ((fp3 = fopen("data/team.txt", "wb")) == NULL) {
		printf("分队写入失败!\n");
		return;
	}

		initial_uav();
		for (t = 0; t < 100; t += delt) {
			get_Xcg();
			dis = dis_to_target(p_start);
			angle = angle_to_target(p_start);
			dispersion = f_dispersion();
			speedmatch = f_speedmatch();
			fprintf(fp, "%f %f %f %f\n", speedmatch, dispersion, dis, angle);
			if (dis <= 100 && angle <= 30) break;		
			for (i = 0; i < SIZE; i++) {
				fprintf(fp2, "%d %d %d\n", (int)uav[i].position.x, (int)uav[i].position.y, (int)uav[i].position.z);
				update_assemble(i);
			}
			crash();
		}

	fclose(fp);
	fclose(fp2);
	
	initial_centroid();
	do {
		update_team();
		update_centroid();
	} while (change != 0);
	leader_main = get_main_leader();
	get_team_leader();
	for (i = 0; i < SIZE; i++) {
		fprintf(fp3, "%d  %d %d %d\n", (int)uav[i].position.x, (int)uav[i].position.y, (int)uav[i].position.z, uav[i].teamID);
	}
	fclose(fp3);

	printf("%.2fs crash:%d\n", t, num_crash);
}
