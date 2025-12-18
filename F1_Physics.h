#ifndef F1_PHYSICS_H
#define F1_PHYSICS_H

#include <vector>
#include <array>
#include <cmath>

class F1PhysicsEngine
{
public:
    struct Point2D
    {
        double x, y;
        Point2D(double x = 0, double y = 0) : x(x), y(y) {}
    };

    struct Vector2D
    {
        double x, y;
        Vector2D(double x = 0, double y = 0) : x(x), y(y) {}
    };

    struct TireParams{ //Параметры Шины
        std::vector<double> tire_friction;
        double A_slip_ratio ; //коэффицтента для вычисления трения и темепарутры шин
        double A_slip_angle ;
        double K_temp ;
        double temp_outside ;
        double B_slip_ratio ;
        double B_slip_angle ;
        double K_wear ;
        double temp_optimal; 
        
        TireParams(std::vector<double> friction,double A_slip_ratio, double A_slip_angle, double K_temp, double temp_outside, double B_slip_ratio, double B_slip_angle, double K_wear, double temp_optimal)
        {
            this->tire_friction = friction;
            this->A_slip_ratio = A_slip_ratio;
            this->A_slip_angle = A_slip_angle;
            this->K_temp = K_temp;
            this->temp_outside = temp_outside;
            this->B_slip_ratio = B_slip_ratio;
            this->B_slip_angle = B_slip_angle;
            this->K_wear = K_wear;
            this->temp_optimal = temp_optimal;
        }
    };

    struct CarState
    {
        // Поступательное движение
        Point2D position = {1869.0, 1160.0};
        Vector2D velocity;
        Vector2D acceleration;
        Vector2D world_velocity;
        double speed;
        double wheel_speed;
        double wheel_acceleration;
        double linear_slip;
        

        // Вращательное движение
        double car_angle = 0.0;
        double angular_velocity = 0.0;
        double steering_wheel = 0.0;
        double slip_angle = 0.0;

        // Трение
        bool is_skidding = false;
        double current_tire_friction = 1.5;
        double current_tire_temperature = 70;
        double slip_ratio = 0.0;
        double current_tire_wear = 0.0;
        double tire_idx = 0;

        // Двигатель и трансмиссия
        double max_rpm = 10000;
        double max_torque = 300;
        double acceleration_rate = 200;
        double rotor_resist = 100;
        double last_engine_rpm = 0.0;
        double engine_rpm = 0.0;
        double engine_torque = 0.0;
        double wheel_rpm = 0.0;
        double last_wheel_rpm = 0.0;
        double wheel_torque = 0.0;
        int current_gear = 1;
        int previous_gear = 1;
        double wheel_torque_final = 0.0;

        // Силы
        double traction_force = 0.0;
        double drag_force = 0.0;
        double brake_force = 0.0;
        double down_force = 0.0;
        double long_force = 0.0;
        double side_force = 0.0;

        // Тормозная система
        double brake_factor = 0.0;
    };

private:
    CarState current_state;

    struct CarParameters
    {
        // Геометрия
        double wheel_radius = 0.33;
        double mass = 540.0;
        double wheelbase = 3.2;  // База автомобиля
        double moment_arm = 1.6; // Плечо момента
        double inertia = 500.0;  // Момент инерции

        // Двигатель и трансмиссия
        double max_rpm = 10000.0;
        double rotor_resist_coef = 10.0;
        double rotor_inertia = 200.0;
        double max_limit_torque = 1000;
        double min_limit_torque = 100;
        double dtorque = 300;
        double peak_rpm = 8000.0;
        double accel_coef = 20.0;
        std::array<double, 4> gear_ratios = {0.0, 2.2, 1.5, 1.1};
        double final_drive = 5.0;

        // Аэродинамика
        double drag_coefficient = 0.5; // коэф лобового сопротивления
        double frontal_area = 1.0;
        double air_density = 0.9;   
        double downforce_coefficient = 5.0;  //коэф прижима

        // Шины и тормоза
        
        double max_brake_force = 35000.0;
        double brake_factor_k = 0.2;
        double brake_max = 1.0;
        double brake_min = 0.0;
        double brake_rate = 1000.0;
        double cornering_stiffness = 3000.0; // Жесткость в повороте
        double steering_speed = 1.5; //скорость изменения угла поворота руля
        double lateral_damping_coef = 10; //демпфирование боковой силы
        double rotational_damping = 5.0; // Демпфирование вращения
        double max_steering_wheel = M_PI;
        double wheel_inertia = 0.45;

        //Шины
        std::vector<TireParams> tires ={ TireParams({2.5,0.01},40,500,0.005,25,3.5,3.7,1,850),TireParams({1.0,0.3},10,100,0.015,25,0.5,1.7,0.2,550) }; // Улучшенное сцепление(max,min)
        // double A_slip_ratio = 10; //коэффицтента для вычисления трения и темепарутры шин
        // double A_slip_angle = 100.0;
        // double K_temp = 0.0015;
        // double temp_outside = 25;
        // double B_slip_ratio = 0.5;
        // double B_slip_angle = 0.70;
        // double K_wear = 0.05;
        // double temp_optimal = 250; 

    };

    CarParameters params;

public:
    F1PhysicsEngine(int idx);
    void reset();
    void update(double dt, bool gas_pedal, bool brake_pedal, double steering_wheel = 0.0);
    void shiftUp();
    void shiftDown();
    void gasUp();
    void gasDown();
    void brakeUp();
    void brakeDown();
    const CarState &getState() const { return current_state; }
    void setTire(int tire_idx);

private:
    void calculateEnginePhysics(bool gas_pedal, double dt);
    void calculateRPM(bool gas_pedal, double dt);
    void calculateTorque();
    void calculateWheelParameters();
    void calculateBrakeFactor(bool brake_pedal, double dt);
    void applyBrakes(double dt);
    void TestMaxRPMForCurrentGear();
    void acceleration_rate(double rpm);

    void calculateLongForces(bool gas_pedal, bool brake_pedal);
    void calculateSideForces();
    void calculateDragForce();
    void calculateDownForce();
    void calculateTractionForce(bool gas_pedal);
    void calculateBrakeForce();

    void isSkidding(bool gas_pedal);
    void integrateMotion(double dt);
    void integrateWheelMotion(double dt);
    void calculateTireParams(double dt);

    void saveToTable(double);
};

#endif // F1_PHYSICS_H
