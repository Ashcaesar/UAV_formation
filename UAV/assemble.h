#ifndef ASSEMBLE_H_INCLUDED
#define ASSEMBLE_H_INCLUDED

#include"model.h"

int num_crash;
axis f_match(int);					   /*f_match为速度匹配项*/
axis f_attract(int);		           /*f_attract为个体间吸引、排斥项*/
axis f_target(int);                    /*f_target为目标趋向作用项*/
axis tMatch, tAttract, tTarget;        /*用于更新数据*/
void initial_uav();                    /*f_initial负责初始化*/
double limit_uav(double, double, double);  /*f_sat对个体的加速度和速度进行限制*/
void crash();                        /*f_crash统计发生碰撞次数*/
void update_assemble(int);                    /*f_update更新各项数据*/
int finish_assemble();                          /*f_stop判断集结是否完成,条件为集群速度方向和距离*/
int get_leader();						   /*f_leader确定长机编号*/
double f_dispersion();                 /*f_dispersion计算离散度*/
double f_speedmatch();				   /*f_speedmatch计算速度匹配值*/



#endif
