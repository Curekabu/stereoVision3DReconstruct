#pragma once
#include <vector>
class PointsArray
{
public:
	unsigned int quantity = 0;
	std::vector<float> data;
};

class Point3D
{
public:
	float x, y, z;
	int R, G, B;
	Point3D() :
		x(0),
		y(0),
		z(0),
		R(0),
		G(0),
		B(0){}
};