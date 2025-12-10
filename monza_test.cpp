// Creates track

#include <iostream>
#include <fstream>
#include <vector>
#include <cmath>
#include "curve.h"
#include <list>
#include <ncurses.h>
using namespace std;

typedef float real;
typedef int number; 
typedef long integer;

enum CellType{
	Asphalt,
	Curb,
	Grass,
	Gravel
};

struct Cell{
	//stores information about conditions in small area of the track
	private:
		real cell_grip;
		real cell_slope;
		point* vertices;
		CellType cell_type;
	public:
		Cell() : cell_type(Asphalt), cell_grip(0.0), cell_slope(0.0), vertices(nullptr){};
		Cell(const point& p1, const point& p2, const point& p3, const point& p4, CellType c_type = Asphalt,
										real _grip = 0.0, real _slope = 0.0) : 
										cell_type(c_type), 
	       							     		cell_grip(_grip), 
								     		cell_slope(_slope){
		vertices = new point[4];
		real eps = 0;
		eps = 0.01;
		vertices[0] = p1 + ((p1 - p4) * eps) + ((p1-p2)*eps); 
		vertices[1] = p2 + ((p2 - p1) * eps) + ((p2-p3)*eps); 
		vertices[2] = p3 + ((p3 - p2) * eps) + ((p3-p4)*eps); 
		vertices[3] = p4 + ((p4 - p3) * eps) + ((p4-p1)*eps); 
	}
		
		CellType type() const { return cell_type; }
		real grip() const { return cell_grip;}
		real slope() const { return cell_slope;}
		bool in_cell(const point& P){
			int flag = false;
			real x = P.x(); real y = P.y();
			int j = 0;
			for(int i = 0; i<4; i++){
				j = (i+1) % 4;
				if((x > vertices[i].x() && x > vertices[j].x()) || 
				  ((y - vertices[i].y())*(y-vertices[j].y()) > 1e-5)){
					continue;
				}
				else{
					real x_int = (y-vertices[i].y())/(vertices[j].y() - vertices[i].y())*
						     (vertices[j].x() - vertices[i].x()) + vertices[i].x();
					if(x_int >= x){
						flag = !flag;
					}
				}
			}
			return flag;

	}
	point* vert() const { return vertices;} 
};
//list of Cells structures
struct Cell_Node{
	friend class Cell_List;
	private:
	Cell_Node* next_node;
	public:
	Cell& cell;
	real area;

	Cell_Node(Cell& _cell, Cell_Node* _next = nullptr, real _area = 0.0):	cell(_cell), 
										next_node(_next), 
										area(_area){};

	Cell_Node* next() {return next_node;}
	const Cell_Node* next() const {return next_node;}

	bool in_cell(point P){
		point* vert = cell.vert();
		int flag = false;
		real x = P.x(); real y = P.y();
		int j = 0;
		for(int i = 0; i<4; i++){
			j = (i+1) % 4;
			if((x > vert[i].x() && x > vert[j].x()) || 
			  ((y - vert[i].y())*(y-vert[j].y()) > 0)){
				continue;
			}
			else{
				real x_int = (y-vert[i].y())/(vert[j].y() - vert[i].y())*
					     (vert[j].x() - vert[i].x()) + vert[i].x();
				if(x_int >= x){
					flag = !flag;
				}
			}
		}
		return flag;

	}

};
class Cell_List{
	//Добавление узла в конец
	//Добавление узла по площади пересечения
	//Поиск по координате <- проверка на попадение
	//Отработка случая когда точка не попадает ни в одну ячейку
	private:
	number length;
	Cell_Node* list_root;
	public:
	Cell_List() : length(0), list_root(nullptr){}
	Cell_List(Cell& _root_cell) : length(0), list_root(nullptr){
		add_after(nullptr, _root_cell);
	} 
	number get_length() const {return length;}
	const Cell_Node* root() const {return list_root;}
	void add_after(Cell_Node* _node, Cell& new_cell, real area = 0.0){
		if(_node == nullptr){
			Cell_Node* new_node = new Cell_Node(new_cell, list_root, area);
			list_root = new_node;
		}
			else{
			Cell_Node* new_node = new Cell_Node(new_cell, _node->next(), area);
			_node -> next_node = new_node;
		}
		length++;
	}
	void push_back(Cell& new_cell, real area = 0.0){
		Cell_Node* tmp = list_root;
		Cell_Node* tmp1 = tmp;
		while(tmp != nullptr){
			tmp1 = tmp;
			tmp = tmp -> next();
		}
		add_after(tmp1, new_cell, area);
		
	}

	void add_by_area(Cell& new_cell, real area){
		if(list_root == nullptr){
			add_after(nullptr, new_cell, area);
		}
		else{
			Cell_Node* tmp = list_root;
			if(tmp -> area <= area){
				if(&(tmp -> cell) != &(new_cell)){
					add_after(nullptr, new_cell, area);
				}
				return;
			}

			while((tmp->next() != nullptr)){
				if(tmp -> next() -> area <= area){
					if(&(tmp -> next() -> cell) != &(new_cell)){
						add_after(tmp, new_cell, area);
					}
					return;
				}
				tmp = tmp->next();
			}
			add_after(tmp, new_cell, area);
		}
		
	}

	void print_grips(){
		Cell_Node* tmp = list_root;
		while(tmp != nullptr){
			cout << tmp->cell.grip() << endl;
			tmp = tmp -> next();
		}
	}
	
	Cell* find_cell(const point& P) const{
		Cell_Node* tmp = list_root;
		while(tmp != nullptr){
			if(tmp -> in_cell(P)){
				return &(tmp->cell);
			}
			tmp = tmp -> next();
		}
		return nullptr;
	}


};



class Track{
	//massive of cells
	vector<Cell> cells;
	//Contains hash-table of lists of cells
	number size_one;
	vector<Cell_List> table;
	//Regular grid params
	real delta; //step of the grid
	//splitting parameters
	number along;
	number across;	
	public:
	Track(number _along, number _across, real _delta): along(_along), across(_across), delta(_delta){
		//calc table size (power of 2)
		number S = _along * _across;
		number size = 1;
		cells.resize(S);
		while(size < S){
			size *= 2;
		} 
		table.resize(size);
		size_one = size - 1;
	}
	
	number hash(number x, number y){
		return (2654435761U*x + 2246822519U*y) & size_one;
	}

	void create_layer(point ins, point out, vector <point>& layer, number N){
		layer[0] = ins;
		layer[N-1] = out;
		real dt = 1.0/((real)(N-1));
		real t = 0.0;
		real x; real y;
		for(number i = 1; i < N-1; i++){
			t = t+dt;
			x = (out.x()-ins.x())*t + ins.x();
			y = (out.y()-ins.y())*t + ins.y();
			layer[i] = point(x, y);
			//cout << layer[i] << endl;

		}
	
	}

	void fill_between_curves(const ClosedCurve& ins, const ClosedCurve& out, CellType type = Asphalt){
		point p_ins[along];
		point p_out[along];
		ins.split(along, p_ins);
		out.split(along, p_out);
		//Отрезок из двух точек, поделить на across-1 частей
		vector <point> prev_layer(across);
		vector <point> next_layer(across);
		create_layer(p_ins[0], p_out[0], prev_layer, across);
		for(number i = 0; i < along; i++){
			create_layer(p_ins[i], p_out[i], next_layer, across);
			for(number j = 1; j < across; j++){
				//Записываем новую ячейку в vector и сохраняем ссылку на него
				Cell& link = cells.emplace_back(prev_layer[j-1], next_layer[j-1],
							 	next_layer[j],   prev_layer[j], type, 0.5, 0.01);
				//Записываем ячейку в таблицу
				write_cell(link);
			}
			//Перезаписываем слои
			prev_layer = next_layer;
		}
	}
	
	void write_cell(Cell& cell){
		real eps = 1e-4;
		point* vert = cell.vert();
		real x = vert[0].x(); 
		real X = vert[0].x();
       		real y = vert[0].y();
       		real Y = vert[0].y();

		for(int i = 1; i < 4; i++){
			if(vert[i].x() > X){
				X = vert[i].x();
			}
			if(vert[i].x() < x){
				x = vert[i].x();
			}		
			if(vert[i].y() > Y){
				Y = vert[i].y();
			}
			if(vert[i].y() < y){
				y = vert[i].y();
			}
		}

		integer start_N_x = floor(x/delta);
		integer start_N_y = floor(y/delta);
		number nx = floor(X/delta) - start_N_x + 1;
		number ny = floor(Y/delta) - start_N_y + 1;
       	
		int j;
		list <point> pins;
		list <point> intersect;
		real y_ins; real x_ins;
		real coef;
		for(int i = 0; i < 4; i++){
			j = (i+1)%4;
			pins.push_back(vert[i]);
			x = vert[i].x();
			y = vert[i].y();
			X = vert[j].x();
			Y = vert[j].y();
			//Пересечения с вертикалью
			if(abs(X - x) > eps){
				coef = (Y-y)/(X-x);
				for(int k = 0; k <= nx; k++){
					x_ins = (start_N_x + k)*delta;
					if(x_ins <= max(x, X) && x_ins >= min(x,X)){
						y_ins = coef*(x_ins - x) + y;
						if(y_ins <= max(y, Y) && y_ins >= min(y, Y)){
							intersect.emplace_back(x_ins, y_ins);
						}
					}
				}
			}
			//Пересечения с горизонталью
			if(abs(Y - y) > eps){
				coef = (X-x)/(Y-y);
				for(int k = 0; k <= ny; k++){
					y_ins = (start_N_y + k)*delta;
					if(y_ins <= max(y, Y) && y_ins >= min(y,Y)){
						x_ins = coef*(y_ins - y) + x;
						if(x_ins <= max(x, X) && x_ins >= min(x, X)){
							intersect.emplace_back(x_ins, y_ins);
						}
					}
				}
			}
			pins.splice(pins.end(), intersect);

		}
	
		vector <point> p_in_square;
		p_in_square.reserve(10);
		point dl; point dr; point ur; point ul;
		real S = 0.0;
		for(number i = 0; i < nx; i++){
			for(number j = 0; j < ny; j++){
				x = (start_N_x + i)*delta;
				X = (start_N_x + i+1)*delta;
				y = (start_N_y + j)*delta;
				Y = (start_N_y + j+1)*delta;
				for(const auto& el : pins){
					if ((x - el.x() < eps)&&(el.x() - X < eps)&&
				   	    (y - el.y() < eps)&&(el.y() - Y < eps)){
						p_in_square.push_back(el);
					}
				}
				dl = point(x, y); dr = point(X, y);
				ur = point(X, Y); ul = point(x, Y);
				if (cell.in_cell(dl)){p_in_square.push_back(dl);}
				if (cell.in_cell(dr)){p_in_square.push_back(dr);}
				if (cell.in_cell(ur)){p_in_square.push_back(ur);}
				if (cell.in_cell(ul)){p_in_square.push_back(ul);}
				traversal_sort(p_in_square);
				S = intersec_area(p_in_square);
				if(S > 0.0){
					table[hash(start_N_x+i, start_N_y + j)].add_by_area(cell, S);
				}
				p_in_square.clear();
				p_in_square.reserve(10);

			}
		}

	}

	real intersec_area(vector <point> p){
		//Шнуровка Гаусса
		//Приводим многоугольник к началу координат для лучшей устойчивости алгоритма
		if(p.size() < 3){return 0.0;}
		real x0 = p[0].x();
		real y0 = p[0].y();
		real S = 0.0;
		int j = 0;
		int N = p.size();
		for(int i = 1; i < N-1; i++){
			S += (p[i].x()-x0)*(p[i+1].y()-y0) - (p[i+1].x()-x0) * (p[i].y()-y0); 
		}
		return abs(S/2.0);
	}
	void traversal_sort(vector <point>& p){
		if(p.size() < 3){ return; }
		point tmp;
		for(int i = 0; i < p.size()-1; i++){
			for(int j = 1; j < p.size()-i-1; j++){
				if((p[j+1]-p[0]) < (p[j]-p[0])){
					tmp = p[j];
					p[j] = p[j+1];
					p[j+1] = tmp;		
				}
			}
		}
	}


	
	Cell* params(const point& p){
		integer x = floor(p.x()/delta);
		integer y = floor(p.y()/delta);
		return table[hash(x, y)].find_cell(p);	
	}


	
};

void draw_track(Track& track, real scale = 1.0){
	real simbol_number = 18.0/25.0;
	real camera_x = 1869;
	real camera_y = 1139;
	initscr();
	cbreak();
	noecho();
	curs_set(0);
	keypad(stdscr, TRUE);
	int max_x, max_y;
	int ch = 0;
	getmaxyx(stdscr, max_y, max_x);
	real x;	
	real y;
	do{
		clear();
		for(int i = 0; i < max_x; i++){
			for(int j = 0; j < max_y; j++){
				x = (camera_x + (i - max_x/2)*scale)*simbol_number;
				y = camera_y + (j - max_y/2)*scale;
				if(track.params(point(x, y)) == nullptr){
					mvprintw(j, i, ".");
				}
				else{
					mvprintw(j, i, " ");
				}
			}
		}
		switch(ch){
				case KEY_UP:
					camera_y-=scale;
					break;
				case KEY_DOWN:
					camera_y+=scale;
					break;
				case KEY_RIGHT:
					camera_x+=scale;
					break;
				case KEY_LEFT:
					camera_x-=scale;
					break;			
				case 112:
					scale *= 2.0;
					break;
				case 109:
					scale /= 2.0;
					break;
		}
		refresh();
	}while((ch = getch()) != 27);
	endwin();
}

int main(){
	//загружаем данные о треке
	ifstream ins_curve_data("ins_points1.csv");
	number Num = 0;
	char comma;
	real x, y;
	vector <point> ps;
	while(ins_curve_data >> x >> comma >> y){
		Num++;
		ps.emplace_back(x, y);
	}
	ins_curve_data.close();
	//cout << Num << endl;
	point points_ins[Num];
	for(number i = 0; i < Num; i++){
		points_ins[i] = ps[i];
	}
	ClosedCurve ins(points_ins, Num);

	ifstream out_curve_data("out_points1.csv");
	Num = 0;
	ps.clear();
	while(out_curve_data >> x >> comma >> y){
		Num++;
		ps.emplace_back(x, y);
	}
	out_curve_data.close();
	//cout << Num << endl;
	point points_out[Num];
	for(number i = 0; i < Num; i++){
		points_out[i] = ps[i];
	}
	ClosedCurve out(points_out, Num);

	
	Track track(4000, 5, 1.0);
	number N = 10;
	track.fill_between_curves(ins, out);
	draw_track(track, 2);
	return 0;
}
