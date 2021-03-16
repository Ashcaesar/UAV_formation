#ifndef PARAMETER_H_INCLUDED
#define PARAMETER_H_INCLUDED

#include"uav.h"

#define SIZE				50				/*SIZE为群体数量*/
#define num_team			4				/*num_group为编队数量*/
#define H					100				/*H为权重aij参数*/
#define b					0.4				/*b为权重aij参数*/
#define k1					1000			/*k1为个体间作用系数*/
#define k2					100				/*k2为个体间作用系数*/
#define k3					0.5				/*k3为个体间作用系数*/
#define k4					40				/*k4为目标趋向系数*/
#define theta1				0.1				/*theta1为个体间作用系数*/
#define theta2				0.1				/*theta2为个体间作用系数*/
#define theta3				0.3				/*theta3为个体间作用系数*/
#define alpha				0.1				/*alpha为目标趋向系数*/
#define d					3				/*d为个体最小间距*/
#define D					30				/*D为个体最大间距*/
#define neighbor			100				/*neighbor为默认邻域距离*/
#define delt				0.02			/*delt为时间步长*/
#define PI					3.14159265
#define Max_velocity		15
#define Min_velocity        10      
#define Max_acceleration	0.5
#define Min_acceleration    0

axis sum = { 0,0,0 };
axis p_origin = { 0,0,0 };             /*原点坐标*/
axis p_start = { 300,300,300 };        /*出发点坐标*/
axis p_final = { 500,1500,800 };       /*终点坐标*/

#endif // PARAMETER_H_INCLUDED

