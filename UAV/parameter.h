#ifndef PARAMETER_H_INCLUDED
#define PARAMETER_H_INCLUDED

#define SIZE				25				/*SIZE为群体数量*/
#define num_team			2				/*num_group为编队数量*/
#define H					120				/*H为权重aij参数*/
#define b					0.4				/*b为权重aij参数*/
#define k1					300				/*k1为个体间作用系数*/
#define k2					60				/*k2为个体间作用系数*/
#define k3					0.5				/*k3为个体间作用系数*/
#define k4					0.2				/*k4为目标趋向系数*/
#define theta1				0.1
#define theta2				0.1
#define theta3				0.3
#define alpha				0.5				/*alpha为目标趋向系数*/
#define d					3				/*d为个体最小间距*/
#define D					20				/*D为个体最大间距*/
#define M					10
#define neighbor			100				/*neighbor为默认邻域距离*/
#define delt				0.01			/*delt为时间步长*/
#define PI					3.14159265
#define Max_ver_velocity		30
#define Min_ver_velocity		-30
#define Max_hor_velocity		5
#define Min_hor_velocity		-5
#define Max_ver_acceleration	10
#define Min_ver_acceleration	-10
#define Max_hor_acceleration	1
#define Min_hor_acceleration	-1

#endif
