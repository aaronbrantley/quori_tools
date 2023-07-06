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

	point operator - (const point & other)
	{
		this -> x -= other.x;
		this -> y -= other.y;

		return * this;
	}

	point operator * (const point & other)
	{
		this -> x *= other.x;
		this -> y *= other.y;

		return * this;
	}

	point operator / (const point & other)
	{
		this -> x /= other.x;
		this -> y /= other.y;

		return * this;
	}

	point operator / (const float & other)
	{
		this -> x /= other;
		this -> y /= other;

		return * this;
	}
};

#endif
