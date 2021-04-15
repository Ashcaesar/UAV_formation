#include<stdio.h>
#include<math.h>

#include"model.h"
#include"assemble.h"
#include"parameter.h"
#include"formation.h"

extern UAV uav[SIZE + 1];
extern axis sum;
extern axis Xcg;
extern axis p_origin;
extern axis p_assemble;
extern axis p_final;
extern axis R[SIZE + 1];
extern int A[SIZE + 1][SIZE + 1];

//sum临时存储数据用以函数计算
//Xcg存储集群质心位置
//p_origin为原点坐标
//p_assemble为出发点坐标
//p_final为终点坐标
//R[]存储队形相对位置
//A[]为通信邻接矩阵

void main() {
	int i;
	double t, dis, angle;
	FILE *fp;
	FILE *fp2;
	FILE *fp3;
	if ((fp = fopen("data/index.txt", "wb")) == NULL) {
		printf("指标写入失败!\n");
		return;
	}
	if ((fp2 = fopen("data/coordinate.txt", "wb")) == NULL) {
		printf("坐标写入失败!\n");
		return;
	}
	if ((fp3 = fopen("data/final.txt", "wb")) == NULL) {
		printf("写入失败!\n");
		return;
	}
	/*----------------------------------------------------------------------------------------------------------------------------------*/
	/*集结模块*/
	t = 0;
	initial_uav();
	do {
		if (t > 100) break;
		get_Xcg();
		dis = dis_to_target(p_assemble);
		angle = angle_to_target(p_assemble);
		fprintf(fp, "%f %f %f %f %f\n", f_speedmatch(), f_dispersion(), dis, angle, f_min_dis());
		for (i = 1; i <= SIZE; i++) fprintf(fp2, "%.1f %.1f %.1f\n", uav[i].position.x, uav[i].position.y, uav[i].position.z);
		update_assemble();
		crash();
		t += delt;
	} while (dis > 80 || angle > 30);
	printf("集结完成，用时%.1f秒,发生%d次碰撞\n", t, num_crash);
	/*----------------------------------------------------------------------------------------------------------------------------------*/
	/*编队模块*/
	t = 0;
	num_crash = 0;
	initial_form();
	do {
		if (t > 100) break;
		get_Xcg();
		dis = dis_to_target(p_final);
		angle = angle_to_target(p_final);
		fprintf(fp, "%f %f %f %f %f\n", f_speedmatch(), f_dispersion(), dis, angle, f_min_dis());
		for (i = 1; i <= SIZE; i++) fprintf(fp2, "%.1f %.1f %.1f\n", uav[i].position.x, uav[i].position.y, uav[i].position.z);
		update_form();
		crash();
		t += delt;
	} while (dis > 80 || angle > 20);
	printf("编队完成，用时%.1f秒,发生%d次碰撞\n", t, num_crash);
	for (i = 0; i <= SIZE; i++) fprintf(fp3, "%.1f %.1f %.1f\n", uav[i].position.x, uav[i].position.y, uav[i].position.z);
	fclose(fp);
	fclose(fp2);
	fclose(fp3);
}
