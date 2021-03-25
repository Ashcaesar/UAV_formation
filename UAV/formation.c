#include<stdio.h>
#include<math.h>

#include"model.h"
#include"parameter.h"

UAV uav[SIZE + 1];
axis p_start;
axis p_final;

void initial_virtual() {
	uav[0].position.x = p_start.x;
	uav[0].position.y = p_start.y;
	uav[0].position.z = p_start.z;
}
