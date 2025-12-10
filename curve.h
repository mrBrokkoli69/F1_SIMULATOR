#include <iostream>

using namespace std;

typedef float real;
typedef int number;

struct point{
private:
	real point_x;
	real point_y;
public:
	point(real _x, real _y);
	void operator=(const point& p){
		point_x = p.x();
		point_y = p.y();
	}
	point();
	real x() const;
	real y() const;
	point operator + (const point& p) const;
	point operator - (const point& p) const;
	point operator * (real alpha) const;
	bool operator < (const point& p) const;
	bool operator == (const point& p) const;
	real distance_to(const point& p) const;
};

ostream& operator<<(ostream& _os, const point& _p);

class Spline{
	//Stores spline
	private:
	real* a; //x spline coefs
	real* b; //y spline coefs
	point start;
	point finish;
	real length;
	public:
	point diff(real t) const; //derivative

	point ddiff(real t) const; //second derivative
	
	real calc_length();

	Spline();

	Spline(const point& _start, const point& _finish, real a_x, real b_x, real c_x, real d_x,
		       					  real a_y, real b_y, real c_y, real d_y);
	Spline(const Spline& _cpy);
	void copy_data(point _start, point _finish, real _length, real* _a, real* _b);
	void init_data();
	
	Spline& operator=(const Spline& _cpy);
	Spline(point _start, point _finish, point d, point dd);

	~Spline();

	point calc(real t) const; //calculate point on curve
	
	real get_length() const;
};

class Curve{
	//Creates smooth curve by third order spline
	friend class ClosedCurve;
	protected:
	number number_of_points;
	number number_of_splines;
	real length;
	point* points;
	Spline* splines;
	void write_points(point* _points, number _N);
	public:
	Curve():number_of_points(0), length(0.0), points(nullptr), splines(nullptr){};
	Curve(point* _points, number _N);
	real get_length();
	void split(number N, point* p) const;
};

class ClosedCurve: public Curve{
	public:
	ClosedCurve():Curve(){};
	ClosedCurve(point* _points, number _N);
};


