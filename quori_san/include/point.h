#ifndef POINT_H_
#define POINT_H_

/*
* 	used for storing coordinates
*/
struct point
{
	float x;
	float y;

	point operator + (const point & other)
	{
		this -> x += other.x;
		this -> y += other.y;

		return * this;
	}
};

#endif
