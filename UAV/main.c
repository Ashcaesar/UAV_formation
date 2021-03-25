#include<stdio.h>
#include<math.h>

#include"model.h"
#include"assemble.h"
#include"assignment.h"
#include"parameter.h"

extern UAV uav[SIZE+1];
extern axis sum;
extern axis Xcg;
extern axis p_origin;
extern axis p_start;
extern axis p_final;
extern int leader_main;
extern int team_change;

//sum临时存储数据用以函数计算
//Xcg存储集群质心位置
//p_origin为原点坐标
//p_start为出发点坐标
//p_final为终点坐标
//leader_main存储中心长机编号
//team_change判定kmeans迭代是否完成

void main() {
	int i, j;
	double t, dis, temp_dis, min_dis, angle, dispersion, speedmatch;
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
			min_dis = 100;
			for (i = 1; i <= SIZE; i++) {
				for (j = 1; j <= SIZE; j++) {
					if (i == j) continue;
					temp_dis = get_dis(uav[i].position, uav[j].position);
					if (temp_dis < min_dis) min_dis = temp_dis;
				}
			}
			fprintf(fp, "%f %f %f %f %f\n", speedmatch, dispersion, dis, angle, min_dis);
			if (dis <= 100 && angle <= 10) break;		
			for (i = 1; i <= SIZE; i++) {
				fprintf(fp2, "%d %d %d %d %d %d\n", (int)uav[i].position.x, (int)uav[i].position.y, (int)uav[i].position.z, (int)uav[i].velocity.x, (int)uav[i].velocity.y, (int)uav[i].velocity.z);
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
	} while (team_change != 0);
	/*leader_main = get_main_leader();
	get_team_leader();*/
	for (i = 1; i <= SIZE; i++) {
		fprintf(fp3, "%d  %d %d %d\n", (int)uav[i].position.x, (int)uav[i].position.y, (int)uav[i].position.z, uav[i].teamID);
	}
	fclose(fp3);

	printf("%.2fs crash:%d\n", t, num_crash);
}
