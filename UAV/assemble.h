#ifndef ASSEMBLE_H_INCLUDED
#define ASSEMBLE_H_INCLUDED

#include"model.h"

int num_crash;
axis f_match(int);
axis f_attract(int);
axis f_target(int);
axis tMatch, tAttract, tTarget;
void initial_uav();
double limit_uav(double, double, double);
void crash();
void update_assemble();

//num_crash记录碰撞次数
//f_match为速度匹配项
//f_attract为个体间吸引、排斥项
//f_target为目标趋向作用项
//tMatch, tAttract, tTarget用于更新数据
//initial_uav负责初始化无人机信息
//limit_uav对无人机速度、加速度进行限制
//crash计算碰撞次数
//update_assemble负责集群阶段数据更新


#endif
