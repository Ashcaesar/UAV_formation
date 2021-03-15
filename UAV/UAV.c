#include<stdio.h>
#include<stdlib.h>
#include<math.h>
#include<time.h>

#define SIZE       50         /*SIZE为群体数量*/
#define H          100        /*H为权重aij参数*/
#define b          0.4        /*b为权重aij参数*/
#define k1         1300       /*k1为个体间作用系数*/
#define k2         80        /*k2为个体间作用系数*/
#define k3         0.5        /*k3为个体间作用系数*/
#define k4         60         /*k4为目标趋向系数*/
#define theta1     0.3        /*theta1为个体间作用系数*/
#define theta2     0.1        /*theta2为个体间作用系数*/
#define theta3     0.3        /*theta3为个体间作用系数*/
#define alpha      0.1        /*alpha为目标趋向系数*/
#define d          3          /*d为个体最小间距*/
#define D          30         /*D为个体最大间距*/
#define neighbor   100        /*neighbor为默认邻域距离*/
#define delt       0.02       /*delt为时间步长*/
#define PI         3.14159265
#define Max_velocity  50
#define Max_acceleration  20

typedef struct {
	double x;
	double y;
	double z;
}axis;

typedef struct {			 /*构建结构体UAV，存储每架无人机坐标、速度、加速度信息*/
	axis position;
	axis velocity;
	axis acceleration;
	double speed;
	double phi;
	double theta;
}UAV;

int num_crash;
UAV uav[SIZE];
axis sum = { 0,0,0 };
axis target = { 1000,1000,1000 };      /*集结点坐标*/
axis f_match(int);                     /*f_match为速度匹配项*/
axis f_attract(int);		           /*f_attract为个体间吸引、排斥项*/
axis f_target(int);                    /*f_target为目标趋向作用项*/
axis tMatch, tAttract, tTarget;        /*用于更新数据*/
void f_initial();                      /*f_initial负责初始化*/
void f_bound(int);                     /*f_bound对个体的加速度和速度进行限制*/
void f_crash();                        /*f_crash统计发生碰撞次数*/
void f_update(int);                    /*f_update更新各项数据*/
int f_stop();                          /*f_stop判断集结是否完成,条件为集群速度方向和距离*/
double f_distance(int, int);           /*f_distance计算个体间距离*/
double f_dispersion();
double f_speedmatch();

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
			fprintf(fp, "%d %d %d %d %d %d %d %d\n", i + 1, \
				(int)uav[i].position.x, (int)uav[i].position.y, (int)uav[i].position.z, \
				(int)uav[i].acceleration.x, (int)uav[i].acceleration.y, (int)uav[i].acceleration.z, \
				(int)uav[i].speed);
			f_update(i);
		}
		f_crash();
	}
	fclose(fp);
	printf("总用时:%fs\n碰撞次数:%d\n离散度:%f", t, num_crash / 2, f_dispersion());
}

double f_distance(int i, int j) {
	double dis;
	dis = pow(uav[i].position.x - uav[j].position.x, 2) + \
		pow(uav[i].position.y - uav[j].position.y, 2) + \
		pow(uav[i].position.z - uav[j].position.z, 2);
	dis = sqrt(dis);
	return dis;
}

axis f_match(int i) {
	int j, num;
	double dis, aij;
	sum.x = 0; sum.y = 0; sum.z = 0;
	num = 0;
	for (j = 0; j < SIZE; j++) {
		if (j != i) {
			dis = f_distance(i, j);
			if (dis < neighbor) {
				num++;
				aij = H / pow(1 + pow(dis, 2), b);
				sum.x += aij * (uav[j].velocity.x - uav[i].velocity.x);
				sum.y += aij * (uav[j].velocity.y - uav[i].velocity.y);
				sum.z += aij * (uav[j].velocity.z - uav[i].velocity.z);
			}
		}
	}
	sum.x /= num;
	sum.y /= num;
	sum.z /= num;
	axis result = { sum.x,sum.y,sum.z };
	return result;
}

axis f_attract(int i) {
	int j;
	double f, dis;
	sum.x = 0; sum.y = 0; sum.z = 0;
	for (j = 0; j < SIZE; j++) {
		if (j != i) {
			dis = f_distance(i, j);
			if ((dis > d) && (dis <= D)) {
				f = k1 * (pow(dis - d, -theta1) - pow(D - d, -theta1));
			}
			else if ((dis > D) && (dis <= neighbor)) {
				f = k2 * (pow(dis - d, -theta2) - pow(D - d, -theta2));
			}
			else if (dis > neighbor) {
				f = k3 * (pow(dis, -theta3));
			};
			sum.x += f * (uav[i].position.x - uav[j].position.x) / (dis + 1e-6);
			sum.y += f * (uav[i].position.y - uav[j].position.y) / (dis + 1e-6);
			sum.z += f * (uav[i].position.z - uav[j].position.z) / (dis + 1e-6);
		}
	}
	axis result = { sum.x,sum.y,sum.z };
	return result;
}

axis f_target(int i) {
	double dis, g;
	sum.x = 0; sum.y = 0; sum.z = 0;
	dis = sqrt(pow(target.x - uav[i].position.x, 2) + \
		pow(target.y - uav[i].position.y, 2) + \
		pow(target.z - uav[i].position.z, 2));
	g = k4 * pow(dis, -alpha);
	sum.x += g * (target.x - uav[i].position.x) / (dis + 1e-6);
	sum.y += g * (target.y - uav[i].position.y) / (dis + 1e-6);
	sum.z += g * (target.z - uav[i].position.z) / (dis + 1e-6);
	axis result = { sum.x,sum.y,sum.z };
	return result;
}

int f_stop() {
	int i;
	double range = 0, V = 0, angle_to_target = 0;
	axis Xcg = { 0,0,0 };
	axis Vcg = { 0,0,0 };
	axis dis = { 0,0,0 };
	for (i = 0; i < SIZE; i++) {
		Xcg.x += uav[i].position.x;
		Xcg.y += uav[i].position.y;
		Xcg.z += uav[i].position.z;
		Vcg.x += uav[i].velocity.x;
		Vcg.y += uav[i].velocity.y;
		Vcg.z += uav[i].velocity.z;
	}
	Xcg.x /= SIZE;
	Xcg.y /= SIZE;
	Xcg.z /= SIZE;
	Vcg.x /= SIZE;
	Vcg.y /= SIZE;
	Vcg.z /= SIZE;
	dis.x = target.x - Xcg.x;
	dis.y = target.y - Xcg.y;
	dis.z = target.z - Xcg.z;
	range = sqrt(pow(dis.x, 2) + pow(dis.y, 2) + pow(dis.z, 2));
	V = sqrt(pow(Vcg.x, 2) + pow(Vcg.y, 2) + pow(Vcg.z, 2));
	angle_to_target = fabs(acos((dis.x*Vcg.x + dis.y*Vcg.y + dis.z*Vcg.z) / (range * V)));
	if (range <= 50 && angle_to_target <= 10) return 1;
	else return 0;
}

void f_initial() {
	int i;
	srand((unsigned)time(NULL));
	do {
		num_crash = 0;
		for (i = 0; i < SIZE; i++) {
			uav[i].position.x = rand() % 150;
			uav[i].position.y = rand() % 150;
			uav[i].position.z = rand() % 150;
		}
		f_crash();
	} while (num_crash > 0);

	for (i = 0; i < SIZE; i++) {
		uav[i].speed = rand() % 10;
		uav[i].phi = (rand() % 360)*PI / 180;
		/*uav[i].phi = (rand() % 90 - 45)*PI / 180;*/
		uav[i].theta = (rand() % 90 - 45)*PI / 180;
		uav[i].velocity.x = uav[i].speed*cos(uav[i].phi)*cos(uav[i].theta);
		uav[i].velocity.y = uav[i].speed*cos(uav[i].phi)*sin(uav[i].theta);
		uav[i].velocity.z = uav[i].speed*sin(uav[i].phi);
		uav[i].acceleration.x = 0;
		uav[i].acceleration.y = 0;
		uav[i].acceleration.z = 0;
	}
}

void f_bound(int i) {
	if (fabs(uav[i].acceleration.x) > Max_acceleration) {
		if (uav[i].acceleration.x > Max_acceleration) uav[i].acceleration.x = Max_acceleration;
		else uav[i].acceleration.x = -Max_acceleration;
	}
	if (fabs(uav[i].acceleration.y) > Max_acceleration) {
		if (uav[i].acceleration.y > Max_acceleration) uav[i].acceleration.y = Max_acceleration;
		else uav[i].acceleration.y = -Max_acceleration;
	}
	if (fabs(uav[i].acceleration.z) > Max_acceleration) {
		if (uav[i].acceleration.z > Max_acceleration) uav[i].acceleration.z = Max_acceleration;
		else uav[i].acceleration.z = -Max_acceleration;
	}

	if (fabs(uav[i].velocity.x) > Max_velocity) {
		if (uav[i].velocity.x > Max_velocity) uav[i].velocity.x = Max_velocity;
		else uav[i].velocity.x = -Max_velocity;
	}
	if (fabs(uav[i].velocity.y) > Max_velocity) {
		if (uav[i].velocity.y > Max_velocity) uav[i].velocity.y = Max_velocity;
		else uav[i].velocity.y = -Max_velocity;
	}
	if (fabs(uav[i].velocity.z) > Max_velocity) {
		if (uav[i].velocity.z > Max_velocity) uav[i].velocity.z = Max_velocity;
		else uav[i].velocity.z = -Max_velocity;
	}
}

void f_update(int i) {
	tMatch = f_match(i);
	tAttract = f_attract(i);
	tTarget = f_target(i);

	uav[i].acceleration.x = tMatch.x + tAttract.x + tTarget.x;
	uav[i].acceleration.y = tMatch.y + tAttract.y + tTarget.y;
	uav[i].acceleration.z = tMatch.z + tAttract.z + tTarget.z;
	f_bound(i);

	uav[i].velocity.x += uav[i].acceleration.x*delt;
	uav[i].velocity.y += uav[i].acceleration.y*delt;
	uav[i].velocity.z += uav[i].acceleration.z*delt;
	f_bound(i);

	uav[i].position.x += uav[i].velocity.x*delt;
	uav[i].position.y += uav[i].velocity.y*delt;
	uav[i].position.z += uav[i].velocity.z*delt;

	uav[i].speed = sqrt(pow(uav[i].velocity.x, 2) + pow(uav[i].velocity.y, 2) + pow(uav[i].velocity.z, 2));
	uav[i].phi = atan2(uav[i].velocity.z, sqrt(pow(uav[i].velocity.x, 2) + pow(uav[i].velocity.y, 2)));
	uav[i].theta = atan2(uav[i].velocity.y, uav[i].velocity.x);
}

void f_crash() {
	int i, j;
	double dis;
	for (i = 0; i < SIZE; i++) {
		for (j = 0; j < SIZE; j++) {
			if (i != j) {
				dis = f_distance(i, j);
				if (dis < d) num_crash++;
			}
		}
	}
}

double f_dispersion() {
	int i, j;
	double dis, result;
	result = 0;
	for (i = 0; i < SIZE; i++) {
		for (j = 0; j < SIZE; j++) {
			if (i != j) {
				dis = pow(uav[i].position.x - uav[j].position.x, 2) + \
					pow(uav[i].position.y - uav[j].position.y, 2) + \
					pow(uav[i].position.z - uav[j].position.z, 2);
				result += dis;
			}
		}
	}
	result = sqrt(result / (SIZE*(SIZE - 1)));
	return result;
}

double f_speedmatch() {
	int i, j;
	double dis, result;
	result = 0;
	for (i = 0; i < SIZE; i++) {
		for (j = 0; j < SIZE; j++) {
			if (i != j) {
				dis = pow(uav[i].velocity.x - uav[j].velocity.x, 2) + \
					pow(uav[i].velocity.y - uav[j].velocity.y, 2) + \
					pow(uav[i].velocity.z - uav[j].velocity.z, 2);
				result += dis;
			}
		}
	}
	result = sqrt(result / (SIZE*(SIZE - 1)));
	return result;
}