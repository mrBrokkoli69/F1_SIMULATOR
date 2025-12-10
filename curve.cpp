#include <iostream>
#include <cmath>
#include "curve.h"
using namespace std;

typedef float real; //real number type
typedef int number; //natural number type

point::point(real _x, real _y) : point_x(_x), point_y(_y){}
point::point() : point_x(0.0), point_y(0.0){};
real point::x() const {
	return point_x;	
};
real point::y() const {
	return point_y;
};

point point::operator + (const point& p) const{
	return point(point_x + p.x(), point_y + p.y());
}

point point::operator - (const point& p) const{
	return point(point_x - p.x(), point_y - p.y());
}

point point::operator * (real alpha) const{
	return point(alpha * point_x, alpha * point_y);
}
bool point::operator < (const point& p) const{
	if((point_x * p.y() - point_y * p.x()) > 0){
		return true;
	}
	return false;
}
bool point::operator == (const point& p) const{
	if(p == *this){return true;}
	if((point_x == p.x()) && (point_y == p.y())){
		return true;
	}
	else{
		return false;
	}
}

ostream& operator<<(ostream& _os, const point& _p){
	//_os << "(x: " << _p.x() << ", y: " << _p.y() << ")";
	_os << _p.x() << "," << _p.y();
	return _os;
};

real point::distance_to(const point& p) const{
	return sqrt((point_x - p.x())*(point_x - p.x()) +  (point_y - p.y())*(point_y - p.y()));
}


point Spline::diff(real t) const { //first derivatives
	real x_t = (3.0*a[3]*t + 2.0*a[2])*t + a[1]; 
	real y_t = (3.0*b[3]*t + 2.0*b[2])*t + b[1]; 
	return point(x_t, y_t);
};

point Spline::ddiff(real t) const { //second derivatives (x_tt, y_tt)
	real x_tt = 6.0*a[3]*t + 2.0*a[2];
	real y_tt = 6.0*b[3]*t + 2.0*b[2];
	return point(x_tt, y_tt);
};
real Spline::calc_length(){
	//calculate length;
	number N = 1000;
	real dt = 1.0 / ( (real)N);
	real t = 0.0;
	real l = 0;
	point prev = calc(0.0);
	point next;	
	for(number i = 0; i < N; i++){
		t = t+dt;
		next = calc(t+dt);
		l += prev.distance_to(next);
		prev = next;
	}
	return l;
}
Spline::Spline() : a(nullptr), b(nullptr), start(point()), finish(point()), length(0.0){};
Spline::Spline(const point& _start, const point& _finish, real a_x, real b_x, real c_x, real d_x, 
							  real a_y, real b_y, real c_y, real d_y) : start(_start), 
												    finish(_finish){
	a = new real[4];
	b = new real[4];
	a[0] = d_x; b[0] = d_y;
	a[1] = c_x; b[1] = c_y;
	a[2] = b_x; b[2] = b_y;
	a[3] = a_x; b[3] = a_y;
	length = calc_length();	
	
	//cout << "Старт " << start.x() << ", " << start.y() << endl;
	//cout << "Финиш " << finish.x() << ", " << finish.y() << endl;
	//for(int i = 0; i < 4; i++){
	//	cout << i << " " << a[i] << " " << b[i] << endl;
	//}
}; 

void Spline::copy_data(point _start, point _finish, real _length, real* _a, real* _b){
	start = _start;
	finish = _finish;
	length = _length;
	init_data();
	for(int i = 0; i < 4; i++){
		a[i] = _a[i];
		b[i] = _b[i];
	}
}

void Spline::init_data(){
	a = new real[4];
	b = new real[4];
}

Spline::Spline(const Spline& _cpy):length(_cpy.length){
	copy_data(_cpy.start, _cpy.finish, _cpy.length, _cpy.a, _cpy.b);
}

Spline& Spline::operator=(const Spline& _cpy){	
	copy_data(_cpy.start, _cpy.finish, _cpy.length, _cpy.a, _cpy.b);
	return *this;
}

Spline::Spline(point _start, point _finish, point d, point dd) : start(_start), finish(_finish){
	init_data();
	//cout << "Старт " << start.x() << ", " << start.y() << endl;
	//cout << "Финиш " << finish.x() << ", " << finish.y() << endl;
	a[0] = start.x();
	a[1] = d.x();
	a[2] = 0.5*dd.x();
	a[3] = finish.x() - (a[0] + a[1] + a[2]);
	b[0] = start.y();
	b[1] = d.y();
	b[2] = 0.5*dd.y();
	b[3] = finish.y() - (b[0] + b[1] + b[2]);
	//for(int i = 0; i < 4; i++){
		//cout << i << " " << a[i] << " " << b[i] << endl;
	//}
	length = calc_length();
};

Spline::~Spline(){
	delete [] a;
	delete [] b;
}


point Spline::calc(real t) const{ //calculate point on curve
	real x = ((a[3]*t + a[2])*t + a[1])*t + a[0];
	real y = ((b[3]*t + b[2])*t + b[1])*t + b[0];
	return point(x, y); 
};

real Spline::get_length() const{
	return length;
};




void Curve::write_points(point* _points, number _N){
	for(number i = 0; i < _N; i++){
		points[i] = _points[i];
	}
}

Curve::Curve(point* _points, number _N) : number_of_points(_N), number_of_splines(_N-1), length(0.0){
	if(number_of_points < 4){
		exit(1);
	}
	points = new point[number_of_points];
	write_points(_points, number_of_points);
	//Нормальный метод для не замкнутых кривых
	number N = number_of_points;
	splines = new Spline[N-1];
	//Решаем систему методом прогонки(см. файл MethodProgonky.ipynb)
	real alpha[N-3]; alpha[0] = -0.25;
	real beta_x[N-3]; real beta_y[N-3]; 
	beta_x[0] = 3.0*(points[2].x() - 2.0*points[1].x() + points[0].x()) / 4.0; 
	beta_y[0] = 3.0*(points[2].y() - 2.0*points[1].y() + points[0].y()) / 4.0; 
	real f_x; real f_y;
	real denom;
	for(number i = 1; i < N-3; i++){
		denom = alpha[i-1] + 4.0;
		alpha[i] = -1/denom;
		f_x = 3.0*(points[i].x() - 2.0*points[i+1].x() + points[i+2].x());
		f_y = 3.0*(points[i].y() - 2.0*points[i+1].y() + points[i+2].y());
		beta_x[i] = (f_x - beta_x[i-1])/denom;
		beta_y[i] = (f_y - beta_y[i-1])/denom;
	}
	real b_x[N-1];
	real b_y[N-1];
	real a_x; real c_x; real d_x;
	real a_y; real c_y; real d_y;
	b_x[N-2] = (3.0*(points[N-3].x() - 2.0*points[N-2].x() + points[N-1].x()) - beta_x[N-4])/(alpha[N-4] + 4.0);
	b_y[N-2] = (3.0*(points[N-3].y() - 2.0*points[N-2].y() + points[N-1].y()) - beta_y[N-4])/(alpha[N-4] + 4.0);
	a_x = -b_x[N-2]/3.0; a_y = -b_y[N-2]/3.0;
	c_x = points[N-1].x() - points[N-2].x() + 2.0*a_x;
	c_y = points[N-1].y() - points[N-2].y() + 2.0*a_y;
	d_x = points[N-2].x();	d_y = points[N-2].y();
	splines[N-2] = Spline(points[N-2], points[N-1], a_x, b_x[N-2], c_x, d_x,
						        a_y, b_y[N-2], c_y, d_y);
	for(number i = 1; i < N-2; i++){
		b_x[N-2-i] = alpha[N-3-i]*b_x[N-1-i] + beta_x[N-3-i];
		b_y[N-2-i] = alpha[N-3-i]*b_y[N-1-i] + beta_y[N-3-i];
		a_x = (b_x[N-1-i] - b_x[N-2-i])/3.0;	a_y = (b_y[N-1-i]-b_y[N-2-i])/3.0;
		c_x = points[N-1-i].x() - points[N-2-i].x() - (b_x[N-1-i] + 2*b_x[N-2-i])/3.0;
		c_y = points[N-1-i].y() - points[N-2-i].y() - (b_y[N-1-i] + 2*b_y[N-2-i])/3.0;
		d_x = points[N-2-i].x();	d_y = points[N-2-i].y();
		
		splines[N-2-i] = Spline(points[N-2-i], points[N-1-i], a_x, b_x[N-2-i], c_x, d_x,
								      a_y, b_y[N-2-i], c_y, d_y);
	}
	a_x = b_x[1]/3.0; a_y = b_y[1]/3.0;
	c_x = points[1].x() - points[0].x() - a_x;
	c_y = points[1].y() - points[0].y() - a_y;
	d_x = points[0].x();	d_y = points[0].y();
	splines[0] = Spline(points[0], points[1], a_x, 0.0, c_x, d_x, a_y, 0.0, c_y, d_y);
	length = 0.0;
	for(number i = 0; i < N-1; i++){
		length += splines[i].get_length();
	}
};

//Curve::Curve(point* _points, number _N, real _length):number_of_points(_N), length(_length), splines(nullptr){
//	write_points(_points, number_of_points);
//};

real Curve::get_length(){
	return length;
};

void Curve::split (number N, point* p) const {
	real dl = length / ((real) (N-1));
	real t = 0.0;
	real dt = 0.0;
	p[0] = splines[0].calc(0.0);
	number count = 1;
	point diff;
	real delta = 0.0;
	for(number i = 0; i < number_of_points; i++){
		while(t+dt <= 1.0){
			diff = splines[i].diff(t);
			dt = (dl-delta) / diff.distance_to(point(0.0, 0.0));
			t = t+dt;
			p[count] = splines[i].calc(t);
			count++;
			if(count == N-1){
				p[count] = splines[number_of_splines-1].calc(1.0);
				return;
			}
			delta = 0.0;
		}
		//Обработка случая, когда какой-то сплайно нужно перепрыгнуть
		delta = splines[i].calc(t).distance_to(splines[i].calc(1.0));
		dt = 0.0;
		t = 0.0;
	}
	if(count == N){return;}
	if(count < N){
		for(number i = count; i < N; i++){
			p[i] = splines[number_of_splines-1].calc(1.0);
		}	
	}
};

ClosedCurve::ClosedCurve(point* _points, number _N){
	number_of_points = _N;
	number_of_splines = _N;
	points = new point[_N];
	if(number_of_points == 0){
		exit(1);
	}
	write_points(_points, number_of_points);
	number N = number_of_points;
	splines = new Spline[N];
	//Построение замкутой кривой с помощью решения циркулярной трёхдиагональной системы методом
	//Шермана-Моррисона(см. файл MethodProgonky.ipynb)
	
	//Вычисление правой части
	real f_x[N];
	real f_y[N];

	f_x[0] = 3.0*(points[1].x()-2.0*points[0].x()+points[N-1].x());
	f_y[0] = 3.0*(points[1].y()-2.0*points[0].y()+points[N-1].y());
	f_x[N-1] = 3.0*(points[0].x()-2.0*points[N-1].x()+points[N-2].x());
	f_y[N-1] = 3.0*(points[0].y()-2.0*points[N-1].y()+points[N-2].y());
	for(number i = 1; i < N-1; i++){
		f_x[i] = 3.0*(points[i+1].x()-2.0*points[i].x()+points[i-1].x());
		f_y[i] = 3.0*(points[i+1].y()-2.0*points[i].y()+points[i-1].y());
	}	
	//Прямой ход
	real alpha[N-1]; 
	real beta_x[N-1];
	real beta_y[N-1];
	real beta_z[N-1];
	alpha[0] = -1.0/3.0; 
	beta_x[0] = f_x[0]/3.0;
	beta_y[0] = f_y[0]/3.0;
	beta_z[0] = 1.0/3.0;
	for(number i = 1; i < N-1; i++){
		alpha[i] = - 1.0/(alpha[i-1] + 4.0);
		beta_x[i] = (f_x[i] - beta_x[i-1])/(alpha[i-1] + 4.0);	
		beta_y[i] = (f_y[i] - beta_y[i-1])/(alpha[i-1] + 4.0);	
		beta_z[i] = -beta_z[i-1]/(alpha[i-1] + 4.0);	
	}
	//Обратный ход
	real y_x[N]; real y_y[N]; real z[N];
	y_x[N-1] = (f_x[N-1] - beta_x[N-2])/(alpha[N-2] + 3.0);
	y_y[N-1] = (f_y[N-1] - beta_y[N-2])/(alpha[N-2] + 3.0);
	z[N-1] = (1.0 - beta_z[N-2])/(alpha[N-2] + 3.0);
	for(int i = N-2; i >= 0; i--){
		y_x[i] = alpha[i]*y_x[i+1] + beta_x[i];
		y_y[i] = alpha[i]*y_y[i+1] + beta_y[i];
		z[i] = alpha[i]*z[i+1] + beta_z[i];
	}

	//Вычисляем коэффициенты b
	real b_x[N]; real b_y[N];
	real x_const = (y_x[0] + y_x[N-1])/(1 + z[0] + z[N-1]);
	real y_const = (y_y[0] + y_y[N-1])/(1 + z[0] + z[N-1]);
	for(number i = 0; i < N; i++){
		b_x[i] = y_x[i] - x_const*z[i];
		b_y[i] = y_y[i] - y_const*z[i];
	}

	//Вычисляем остальные коэффициенты и записываем их в сплайны
	real a_x; real c_x; real d_x;
	real a_y; real c_y; real d_y;
	for(number i = 0; i < N-1; i++){
		a_x = (b_x[i+1] - b_x[i])/3.0;
		c_x = points[i+1].x() - points[i].x() - (b_x[i+1] + 2*b_x[i])/3.0;
		d_x = points[i].x();
		a_y = (b_y[i+1] - b_y[i])/3.0;
		c_y = points[i+1].y() - points[i].y() - (b_y[i+1] + 2*b_y[i])/3.0;
		d_y = points[i].y();
		splines[i] = Spline(points[i], points[i+1], a_x, b_x[i], c_x, d_x,
						   a_y, b_y[i], c_y, d_y);
		length += splines[i].get_length();
	}	
	a_x = (b_x[0] - b_x[N-1])/3.0;
	c_x = points[0].x() - points[N-1].x() - (b_x[0] + 2*b_x[N-1])/3.0;
	d_x = points[N-1].x();
	a_y = (b_y[0] - b_y[N-1])/3.0;
	c_y = points[0].y() - points[N-1].y() - (b_y[0] + 2*b_y[N-1])/3.0;
	d_y = points[N-1].y();
	splines[N-1] = Spline(points[N-1], points[0], a_x, b_x[N-1], c_x, d_x,
						    a_y, b_y[N-1], c_y, d_y);
	length += splines[N-1].get_length();


};
