#ifndef MODEL_H_INCLUDED
#define MODEL_H_INCLUDED

typedef struct {
	double x;
	double y;
	double z;
}axis;

typedef struct {
	axis position;
	axis velocity;
	axis acceleration;
	double phi;
	double theta;
	int teamID;
	int leader;
}UAV;

void get_Xcg();
double get_dis(axis, axis);
double f_dispersion();
double f_speedmatch();
double dis_to_target(axis);
double angle_to_target(axis);

//axis三维坐标结构体
//UAV无人机结构体，包括位置、速度、加速度、角度、编队等信息
//get_Xcg计算当前集群质心位置
//f_dispersion计算当前集群离散度
//f_speedmatch计算当前速度匹配差
//dis_to_target计算当前集群质心与目标点距离
//angle_to_target计算当前集群速度方向与目标点连线夹角

#endif //
