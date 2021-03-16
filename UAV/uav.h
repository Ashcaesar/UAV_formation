#ifndef UAV_H_INCLUDED
#define UAV_H_INCLUDED

typedef struct {
	double x;
	double y;
	double z;
}axis;

typedef struct {			 
	axis position;
	axis velocity;
	axis acceleration;
	double speed;
	double phi;
	double theta;
}UAV;

int num_crash;
axis f_match(int);					   /*f_match为速度匹配项*/
axis f_attract(int);		           /*f_attract为个体间吸引、排斥项*/
axis f_target(int);                    /*f_target为目标趋向作用项*/
axis tMatch, tAttract, tTarget;        /*用于更新数据*/
void f_initial();                      /*f_initial负责初始化*/
double f_sat(double, double, double);  /*f_sat对个体的加速度和速度进行限制*/
void f_crash();                        /*f_crash统计发生碰撞次数*/
void f_update(int);                    /*f_update更新各项数据*/
int f_stop();                          /*f_stop判断集结是否完成,条件为集群速度方向和距离*/
int f_leader();						   /*f_leader确定长机编号*/
double f_metric(axis, axis);           /*f_metric实现向量度量*/
double f_dispersion();                 /*f_dispersion计算离散度*/
double f_speedmatch();				   /*f_speedmatch计算速度匹配值*/

#endif // UAV_H_INCLUDED
